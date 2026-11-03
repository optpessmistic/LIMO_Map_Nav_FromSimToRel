#!/usr/bin/python3
"""Read-only finite RGB-D calibration capture; no world or target-truth inputs.

This script does not import the online detector node, publish observations or
create motion/action clients. --execute creates only sensor/clock/TF subscribers.
The experiment owner must ensure a stationary setup and enforce an outer hard
process deadline; this script does not control another process or prove braking.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import re
import signal
import time
import uuid

from capture_core import (CaptureWriter, FixedWindow, TOPICS, MAX_MEMORY_BYTES, detector, evidence,
                          evaluate, input_reasons, json_bytes, metadata, provenance)


SELECTION_POLICY = ('First N advancing fresh exact RGB-D pairs in callback-time '
    'pair-availability order, including rejected samples; sort by stamp only '
    'among pairs available in the same callback. Do not wait for older unmatched '
    'stamps; discard later-ready pairs at or below the selected stamp watermark.')


class StopRequest:
    """Signals request a checkpoint; they never invoke native shutdown inline."""
    def __init__(self):
        self.signal = None
        self.old_handlers = {}

    def request(self, signum, _frame):
        if self.signal is None:
            self.signal = signal.Signals(signum).name

    def install(self):
        try:
            for signum in (signal.SIGINT, signal.SIGTERM):
                self.old_handlers[signum] = signal.signal(signum, self.request)
        except BaseException:
            self.restore()
            raise

    def restore(self):
        for signum, handler in self.old_handlers.items():
            signal.signal(signum, handler)
        self.old_handlers.clear()


def checkpoint_then_cleanup(writer, sampling, cleanup_steps, stop, monotonic=time.monotonic):
    """Persist immutable raw evidence before potentially blocking native calls.

    Pure helper: cleanup callbacks are injected. An uncatchable native abort or
    owner hard kill can prevent summary.json, but cannot erase raw-checkpoint.json.
    No hard bound on native cleanup is claimed; the external owner enforces it.
    """
    raw = dict(sampling)
    raw.update(report_phase='before_native_cleanup', partial=True, passed=False,
               cleanup_complete=False, cleanup={'status': 'not_started', 'steps': []},
               stop_signal=stop.signal, interrupted=stop.signal is not None,
               artifacts_saved_before_cleanup=dict(writer.artifacts))
    raw_ref = writer.write_report('raw-checkpoint.json', raw)
    cleanup_started = monotonic()
    results, cleanup_errors = [], []
    for name, callback in cleanup_steps:
        try:
            callback()
            results.append({'stage': name, 'status': 'returned'})
        except (Exception, KeyboardInterrupt) as exc:
            message = name+':'+type(exc).__name__+': '+str(exc)
            cleanup_errors.append(message[:2048])
            results.append({'stage': name, 'status': 'error', 'error': message[:2048]})
    cleanup_complete = not cleanup_errors
    summary = dict(sampling)
    errors = list(sampling['errors'])+cleanup_errors
    if stop.signal is not None:
        errors.append('stop_requested:'+stop.signal)
    summary.update(report_phase='after_native_cleanup', stop_signal=stop.signal,
        interrupted=stop.signal is not None, cleanup_complete=cleanup_complete,
        cleanup={'status': 'complete' if cleanup_complete else 'errors', 'steps': results,
                 'elapsed_wall_s': monotonic()-cleanup_started},
        raw_checkpoint={'artifact': 'raw-checkpoint.json', **raw_ref},
        errors=list(dict.fromkeys(errors)))
    summary['passed'] = bool(summary['sampling_passed'] and cleanup_complete and not summary['errors'])
    summary['partial'] = not (summary['sampling_complete'] and
        summary.get('persistence_complete', True) and cleanup_complete and not summary['interrupted'])
    writer.write_report('summary.json', summary)
    return summary


def end_of_window_integrity(window):
    """Do not revise committed sample files when a conflicting repeat arrives."""
    at_write = bool(window.completed) and all(s['frame_valid'] for s in window.completed)
    selected_conflicts = window.selected_conflicts()
    return {'all_samples_valid_when_saved': at_write,
            'all_samples_valid': at_write and not selected_conflicts,
            'selected_sample_conflicts': selected_conflicts,
            'conflicting_duplicate_count': window.counts['conflicting_duplicate'],
            'pinned_signature_counts': {kind: len(history) for kind,history in window.pinned_signatures.items()},
            'errors': ['conflicting_duplicate_input_observed'] if window.counts['conflicting_duplicate'] else []}


def read_exact_transforms(buffer, stamp, optical, reference, base, time_type, duration_type):
    transforms, errors = {}, {}
    for key, target, source in (('reference_to_base', reference, base),
                                ('base_to_camera', base, optical),
                                ('reference_to_camera', reference, optical)):
        try:
            msg = buffer.lookup_transform(target, source, time_type(nanoseconds=stamp),
                                          timeout=duration_type(nanoseconds=0))
            returned = evidence.source_stamp_ns(msg)
            t, q = msg.transform.translation, msg.transform.rotation
            transforms[key] = {'query_stamp_ns': stamp, 'returned_stamp_ns': returned,
                               'target_frame': msg.header.frame_id, 'source_frame': msg.child_frame_id,
                               'translation_xyz_m': [t.x, t.y, t.z],
                               'rotation_xyzw': [q.x, q.y, q.z, q.w],
                               'lookup_policy': 'requested integer acquisition stamp; never latest Time(0)'}
        except Exception as exc:
            errors[key] = type(exc).__name__+': '+str(exc)
    return transforms, errors


def freeze_ready(window, *, now, clock_wall, started, deadline, registered,
                 lookup, stop_signal=None, force=False):
    """Acquisition only: freeze fixed slots without detector, PNG, writer or fsync.

    Selected message objects have already been detached by FixedWindow.receive.
    A frozen slot never adopts late calibration or a replacement image. Duplicate
    conflicts remain observed until window.closed, including for frozen slots.
    """
    if window.closed:
        return
    for sample in window.pending:
        if 'snapshot' in sample:
            continue
        messages = window.messages(sample)
        meta = metadata(messages)
        invalid = input_reasons(meta, registered)
        complete_info = all(messages[k] is not None for k in ('rgb_info', 'depth_info'))
        clock_ready = window.clock_ns is not None and window.clock_ns >= sample['stamp_ns']
        transforms, tf_errors = {}, {}
        optical = meta['rgb'].get('frame_id')
        query_status = 'not_attempted:no_clock_coverage'
        # CameraInfo is not needed to ask TF about a known image optical frame.
        if clock_ready and isinstance(optical, str) and optical and lookup is not None:
            transforms, tf_errors = lookup(sample['stamp_ns'], optical)
            query_status = 'attempted'
        elif not isinstance(optical, str) or not optical:
            query_status = 'not_attempted:invalid_optical_frame'
        elif lookup is None:
            query_status = 'not_attempted:lookup_unavailable'
        ready = complete_info and clock_ready and (bool(invalid) or len(transforms) == 3)
        if not force and not ready and now < sample['deadline_wall']:
            continue
        extra = []
        if stop_signal is not None:
            extra.append('stop_requested:'+stop_signal)
        if force and now >= deadline:
            extra.append('capture_wall_deadline')
        if not clock_ready:
            extra.append('clock_does_not_cover_sample')
        if window.clock_reset:
            extra.append('clock_reset_during_capture')
        if clock_wall is None or now-clock_wall > 2.:
            extra.append('clock_not_fresh')
        if window.clock_ns is not None and window.clock_ns-sample['stamp_ns'] > 2_000_000_000:
            extra.append('sample_older_than_two_sim_seconds')
        if sample['stamp_ns'] in window.conflict_stamps:
            extra.append('conflicting_duplicate_payload')
        sample['snapshot'] = {'messages': dict(messages), 'observation': {
            'capture_barrier_stamp_ns': window.barrier_ns,
            'acquisition_clock_stamp_ns': window.clock_ns,
            'clock_receipt_age_wall_s': None if clock_wall is None else now-clock_wall,
            'selected_wall_offset_s': sample['selected_wall']-started,
            'frozen_wall_offset_s': now-started,
            'per_pair_deadline_wall_offset_s': sample['deadline_wall']-started,
            'tf_query_status': query_status, 'tf_lookup_errors': tf_errors,
            'transforms': transforms, 'extra_reasons': extra,
            'freeze_reason': 'ready' if ready and not force else
                             ('forced_close' if force else 'pair_wait_elapsed')}}


def persist_frozen(writer, window, manifest, *, registered, reference, base,
                   stop, save_timeout, monotonic=time.monotonic):
    """Offline phase. Durable raw checkpoint precedes detector or PNG work."""
    if not window.closed:
        raise ValueError('collection_must_close_before_any_persistence')
    started = monotonic()
    deadline = started+save_timeout
    raw_files, errors = {}, []
    samples = sorted(window.pending, key=lambda s: s['sequence'])
    raw_started = monotonic()
    for sample in samples:
        if monotonic() >= deadline:
            errors.append('raw_persistence_wall_deadline')
            break
        try:
            snap = sample['snapshot']
            raw_files[sample['sequence']] = writer.save_raw(sample['sequence'],
                snap['messages'], snap['observation'])
        except (OSError, ValueError) as exc:
            errors.append('raw_persistence:'+type(exc).__name__+': '+str(exc))
            break
    raw_elapsed = monotonic()-raw_started
    checkpoint = {'schema_version': 1, 'capture_session': manifest['capture_session'],
        'report_phase': 'raw_saved_before_offline_processing', 'passed': False,
        'processing_complete': False, 'cleanup_complete': False,
        'collection_boundary': dict(window.collection_boundary),
        'raw_saved_sequences': sorted(raw_files), 'selected_sequences': [s['sequence'] for s in samples],
        'not_raw_committed_sequences': [s['sequence'] for s in samples if s['sequence'] not in raw_files],
        'selected_sample_conflicts': window.selected_conflicts(), 'errors': list(errors),
        'stop_signal': stop.signal, 'artifacts': dict(writer.artifacts)}
    writer.write_report('collection-checkpoint.json', checkpoint)
    evaluation_s = derived_save_s = 0.
    for sample in samples:
        if sample['sequence'] not in raw_files:
            continue
        if monotonic() >= deadline:
            errors.append('offline_processing_wall_deadline')
            break
        snap = sample['snapshot']
        observation = snap['observation']
        extra = list(observation['extra_reasons'])
        if stop.signal is not None:
            extra.append('stop_requested:'+stop.signal)
        before = monotonic()
        result, arrays = evaluate(snap['messages'], observation['transforms'], registered=registered,
            reference_frame=reference, base_frame=base,
            extra_reasons=extra, run_detection=stop.signal is None)
        evaluation_s += monotonic()-before
        if monotonic() >= deadline:
            errors.append('offline_processing_wall_deadline')
            break  # Raw checkpoint exists; do not begin PNG/fsync after known expiry.
        if observation['tf_query_status'].startswith('not_attempted:'):
            result['tf_reasons'] = ['tf_not_queried:'+observation['tf_query_status'].split(':', 1)[1]]
            result['reasons'] = result['input_reasons']+result['tf_reasons']+result['extra_reasons']
            result['frame_valid'] = False
        result.update(capture_session=manifest['capture_session'], case_id=manifest['case_id'],
                      **{k: v for k, v in observation.items() if k not in ('transforms', 'extra_reasons')},
                      acquisition_extra_reasons=list(observation['extra_reasons']),
                      processing_phase='after_collection_closed',
                      collection_boundary=dict(window.collection_boundary))
        before = monotonic()
        try:
            path = writer.save_sample(sample['sequence'], snap['messages'], result, arrays,
                                     raw_files=raw_files[sample['sequence']])
        except (OSError, ValueError) as exc:
            errors.append('derived_persistence:'+type(exc).__name__+': '+str(exc))
            break
        derived_save_s += monotonic()-before
        window.completed.append({'sequence': sample['sequence'], 'stamp_ns': sample['stamp_ns'],
            'artifact': path, 'frame_valid': result['frame_valid'], 'reasons': result['reasons'],
            'processing_status': result['processing_status'],
            'detection_count': None if result['detections'] is None else len(result['detections'])})
        window.pending.remove(sample)
    elapsed = monotonic()-started
    if elapsed > save_timeout:
        errors.append('persistence_wall_deadline_exceeded')
    return {'errors': errors, 'raw_saved_pairs': len(raw_files),
            'raw_save_elapsed_wall_s': raw_elapsed, 'evaluation_elapsed_wall_s': evaluation_s,
            'derived_save_elapsed_wall_s': derived_save_s, 'persistence_elapsed_wall_s': elapsed,
            'persistence_complete': len(window.completed) == len(samples) and not errors}


def validate_args(args):
    if not args.execute:
        raise ValueError('--execute is required; no ROS connection was made')
    if not args.registered_depth:
        raise ValueError('--registered-depth must explicitly confirm this simulator stream convention')
    if type(args.pairs) is not int or not 1 <= args.pairs <= 32:
        raise ValueError('--pairs must be 1..32')
    if not math.isfinite(args.timeout) or not 1 <= args.timeout <= 300:
        raise ValueError('--timeout must be 1..300 wall seconds')
    if not math.isfinite(args.pair_wait) or not .01 <= args.pair_wait <= 10:
        raise ValueError('--pair-wait must be .01..10 wall seconds')
    if not 1_048_576 <= args.max_bytes <= 1024**3:
        raise ValueError('--max-bytes must be 1 MiB..1 GiB')
    if not math.isfinite(args.save_timeout) or not 1 <= args.save_timeout <= 120:
        raise ValueError('--save-timeout must be 1..120 wall seconds')
    if type(args.max_memory_bytes) is not int or not 1024**2 <= args.max_memory_bytes <= 1024**3:
        raise ValueError('--max-memory-bytes must be 1 MiB..1 GiB')
    if not re.fullmatch(r'[A-Za-z0-9_-]{1,80}', args.case_id):
        raise ValueError('--case-id is a plain opaque label, not a truth-file path')
    for value in (args.reference_frame, args.base_frame):
        if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]*', value):
            raise ValueError('invalid TF frame')
    if args.reference_frame == args.base_frame:
        raise ValueError('reference and base frames must differ')


def run(args):
    validate_args(args)
    stop = StopRequest()
    stop.install()
    try:
        return run_with_stop(args, stop)
    finally:
        stop.restore()


def run_with_stop(args, stop):
    """Runtime body; StopRequest remains installed through checkpoint and cleanup."""
    started = time.monotonic()
    deadline = started+args.timeout
    writer = CaptureWriter(args.output, args.max_bytes)
    window = FixedWindow(args.pairs, args.pair_wait, max_memory_bytes=args.max_memory_bytes)
    sources = provenance([__file__])
    config = detector.DetectorConfig()
    manifest = {'schema_version': 1, 'capture_session': str(uuid.uuid4()),
                'case_id': args.case_id, 'read_only': True, 'use_sim_time': True,
                'input_topics': TOPICS | {'clock': '/clock', 'tf': '/tf', 'tf_static': '/tf_static'},
                'requested_pairs': args.pairs, 'selection_policy': SELECTION_POLICY,
                'wall_timeout_s': args.timeout, 'per_pair_wait_wall_s': args.pair_wait,
                'save_wall_timeout_s': args.save_timeout,
                'raw_memory_max_bytes': args.max_memory_bytes,
                'phases': 'collect fixed slots and TF; close acquisition; save raw checkpoint; offline detect/PNG; cleanup',
                'output_max_bytes': args.max_bytes, 'detector_config': asdict(config),
                'reference_frame': args.reference_frame, 'base_frame': args.base_frame,
                'registration': 'explicit caller assertion; both runtime CameraInfo must match',
                'source': sources,
                'scope': 'Independent current-core image processing. No world, native model, target count/pose, online detections, map, epoch or mission inputs.',
                'stationarity_independently_verified': False,
                'effective_range_claimed': False, 'coverage_claimed': False}
    writer.write('manifest.json', json_bytes(manifest))
    for path in sources['sources']:
        writer.write('source/'+Path(path).name, Path(path).read_bytes())
    node = listener = None
    initialized = False
    errors, clock_wall = [], None
    acquisition_started = None
    lookup = None
    fatal = []
    try:
        if stop.signal is not None:
            raise RuntimeError('stop_requested_before_runtime_initialization')
        # Imported only on explicit runtime execution; pure tests never enter here.
        import rclpy
        from rclpy.duration import Duration
        from rclpy.time import Time
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from sensor_msgs.msg import CameraInfo, Image
        from rosgraph_msgs.msg import Clock
        from tf2_ros import Buffer, TransformListener
        rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        node = rclpy.create_node('limo_red_calibration_capture', enable_rosout=False,
            start_parameter_services=False,
            parameter_overrides=[Parameter('use_sim_time', value=True)])
        buffer = Buffer(cache_time=Duration(seconds=20.))
        listener = TransformListener(buffer, node, spin_thread=False)
        subscriptions = []
        def on_clock(msg):
            nonlocal clock_wall
            if stop.signal is not None:
                return
            s = msg.clock
            if type(s.sec) is not int or type(s.nanosec) is not int or s.sec < 0 or not 0 <= s.nanosec < 1_000_000_000:
                window.event('invalid_clock')
                return
            window.on_clock(s.sec*1_000_000_000+s.nanosec)
            clock_wall = time.monotonic()
        subscriptions.append(node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data))
        def receive(kind, msg):
            if stop.signal is not None:
                return
            try:
                window.receive(kind, msg, time.monotonic())
            except Exception as exc:
                fatal.append('input_callback:'+type(exc).__name__+': '+str(exc))
        for kind, topic in TOPICS.items():
            subscriptions.append(node.create_subscription(Image if kind in ('rgb','depth') else CameraInfo,
                topic, lambda msg, key=kind: receive(key, msg), qos_profile_sensor_data))

        lookup = lambda stamp, optical: read_exact_transforms(buffer, stamp, optical,
            args.reference_frame, args.base_frame, Time, Duration)
        acquisition_started = time.monotonic()
        while (stop.signal is None and time.monotonic() < deadline and
               sum('snapshot' in s for s in window.pending) < args.pairs):
            rclpy.spin_once(node, timeout_sec=min(.05, max(0, deadline-time.monotonic())))
            if stop.signal is not None or fatal or window.clock_reset:
                break
            freeze_ready(window, now=time.monotonic(), clock_wall=clock_wall, started=started,
                deadline=deadline, registered=args.registered_depth, lookup=lookup)
        if fatal:
            errors += fatal
        if window.clock_reset:
            errors.append('clock_reset_during_capture')
    except (Exception, KeyboardInterrupt) as exc:
        errors.append(type(exc).__name__+': '+str(exc))
    finally:
        # End acquisition before all detector, image encoding and fsync work.
        closed_wall = time.monotonic()
        try:
            freeze_ready(window, now=closed_wall, clock_wall=clock_wall, started=started,
                deadline=deadline, registered=args.registered_depth, lookup=lookup,
                force=True, stop_signal=stop.signal)
        except (Exception, KeyboardInterrupt) as exc:
            errors.append('freeze_at_close:'+type(exc).__name__+': '+str(exc))
        window.closed = True
        acquisition_elapsed = None if acquisition_started is None else closed_wall-acquisition_started
        window.collection_boundary = {
            'closed_wall_offset_s': closed_wall-started,
            'last_callback_clock_stamp_ns': window.clock_ns,
            'acquisition_elapsed_wall_s': acquisition_elapsed,
            'sampling_elapsed_wall_s': closed_wall-started,
            'selected_sequences': list(window.selected_sequences.values()),
            'conflicts_observed_through': 'last executed callback before collection closed; not publisher history',
            'post_close_callbacks_or_queued_messages_included': False}
        complete = (len(window.pending) == args.pairs and
                    all('snapshot' in s for s in window.pending) and closed_wall <= deadline)
        persistence = {'errors': [], 'persistence_complete': False, 'raw_saved_pairs': 0}
        try:
            persistence = persist_frozen(writer, window, manifest,
                registered=args.registered_depth, reference=args.reference_frame,
                base=args.base_frame, stop=stop, save_timeout=args.save_timeout)
        except (Exception, KeyboardInterrupt) as exc:
            errors.append('persistence:'+type(exc).__name__+': '+str(exc))
            persistence['raw_saved_pairs'] = sum(name.endswith('/raw.json') for name in writer.artifacts)
        errors.extend(persistence.pop('errors'))
        if not persistence['persistence_complete']:
            errors.append('selected_sample_persistence_incomplete')
        # Native cleanup follows durable partial raw and final sample checkpoints.
        unchanged = True
        for path, original in sources['sources'].items():
            try:
                unchanged &= hashlib.sha256(Path(path).read_bytes()).hexdigest() == original['sha256']
            except OSError:
                unchanged = False
        if not unchanged:
            errors.append('source_changed_during_capture')
        elapsed = closed_wall-started
        if stop.signal is not None:
            errors.append('stop_requested:'+stop.signal)
        if not complete:
            errors.append('requested_pair_window_incomplete_or_wall_deadline')
        integrity = end_of_window_integrity(window)
        errors.extend(integrity.pop('errors'))
        sampling = {'schema_version': 3, 'capture_session': manifest['capture_session'],
            'case_id': args.case_id, 'read_only': True,
            'sampling_complete': complete, 'collection_complete': complete,
            'collection_boundary': dict(window.collection_boundary), **persistence,
            'acquisition_elapsed_wall_s': acquisition_elapsed,
            'source_unchanged': unchanged, 'source_check_phase': 'before_native_cleanup',
            **integrity,
            'errors': list(dict.fromkeys(errors)), 'samples': list(window.completed),
            'selection_policy': SELECTION_POLICY,
            'requested_pairs': args.pairs, 'saved_pairs': len(window.completed),
            'selected_pairs': len(window.completed)+len(window.pending),
            'pending_pairs_not_saved': len(window.pending),
            'pending_samples_not_committed': [{'sequence': s['sequence'], 'stamp_ns': s['stamp_ns']}
                                             for s in window.pending],
            'capture_barrier_stamp_ns': window.barrier_ns, 'last_clock_stamp_ns': window.clock_ns,
            'sampling_elapsed_wall_s': elapsed, 'bytes_written_before_checkpoint': writer.written,
            'max_observed_raw_image_bytes': window.max_observed_image_bytes,
            'raw_memory_max_bytes': window.max_memory_bytes,
            'received_counts': dict(window.counts), 'diagnostic_events': window.events,
            'limits': ['First bounded received sample window only; no lossless-delivery or whole-scene claim.',
                       'No motion was commanded by this tool; physical stationarity is not established.',
                       'No truth labels were read; empty detections are algorithm output, not target absence.',
                       'Input, per-candidate depth and exact-TF rejections remain separate.',
                       'Committed files are flush/fsync complete; pending samples may have only partial artifact files.',
                       'Late duplicate conflicts invalidate the end-of-window verdict and are linked separately; saved samples are never rewritten.',
                       'Conflict observation ends at collection close; queued or later DDS messages are outside this finite window.',
                       'Input/clock/TF validity is frozen at acquisition; saving time does not re-age the source frames.',
                       'A missing summary means cleanup completion is unknown, even if sampling completed.',
                       'An outer owner must enforce a hard deadline if file I/O or native code blocks.'],
            'effective_range_claimed': False, 'coverage_claimed': False}
        sampling['sampling_passed'] = (complete and unchanged and sampling['all_samples_valid']
            and len(window.completed) == args.pairs and persistence['persistence_complete'] and not errors)
        cleanup_steps = []
        if listener is not None:
            cleanup_steps.append(('listener_unregister', listener.unregister))
        if node is not None:
            cleanup_steps.append(('node_destroy', node.destroy_node))
        if initialized:
            cleanup_steps.append(('context_shutdown', rclpy.shutdown))
        summary = checkpoint_then_cleanup(writer, sampling, cleanup_steps, stop)
        print(json.dumps(summary, ensure_ascii=False, allow_nan=False))
    return 0 if summary['passed'] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case-id', required=True)
    parser.add_argument('--pairs', type=int, default=6)
    parser.add_argument('--timeout', type=float, default=60.)
    parser.add_argument('--pair-wait', type=float, default=2.)
    parser.add_argument('--max-bytes', type=int, default=256*1024**2)
    parser.add_argument('--max-memory-bytes', type=int, default=MAX_MEMORY_BYTES)
    parser.add_argument('--save-timeout', type=float, default=10.)
    parser.add_argument('--reference-frame', default='odom')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--registered-depth', action='store_true')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    try:
        return run(args)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    raise SystemExit(main())
