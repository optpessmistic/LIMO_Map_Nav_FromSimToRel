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

from capture_core import (CaptureWriter, FixedWindow, TOPICS, detector, evidence,
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
    summary['partial'] = not (summary['sampling_complete'] and cleanup_complete and not summary['interrupted'])
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
    window = FixedWindow(args.pairs, args.pair_wait)
    sources = provenance([__file__])
    config = detector.DetectorConfig()
    manifest = {'schema_version': 1, 'capture_session': str(uuid.uuid4()),
                'case_id': args.case_id, 'read_only': True, 'use_sim_time': True,
                'input_topics': TOPICS | {'clock': '/clock', 'tf': '/tf', 'tf_static': '/tf_static'},
                'requested_pairs': args.pairs, 'selection_policy': SELECTION_POLICY,
                'wall_timeout_s': args.timeout, 'per_pair_wait_wall_s': args.pair_wait,
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

        def finish_ready(force=False):
            for sample in list(window.pending):
                now = time.monotonic()
                messages = window.messages(sample)
                meta = metadata(messages)
                invalid = input_reasons(meta, args.registered_depth)
                # Missing calibration may arrive within this sample's bounded wait.
                complete_info = messages['rgb_info'] is not None and messages['depth_info'] is not None
                clock_ready = window.clock_ns is not None and window.clock_ns >= sample['stamp_ns']
                transforms, tf_errors = {}, {}
                if complete_info and not invalid and clock_ready and stop.signal is None:
                    transforms, tf_errors = read_exact_transforms(buffer, sample['stamp_ns'],
                        messages['rgb'].header.frame_id, args.reference_frame, args.base_frame, Time, Duration)
                ready = (complete_info and clock_ready and (bool(invalid) or len(transforms) == 3))
                if not force and not ready and now < sample['deadline_wall']:
                    continue
                extra = []
                if stop.signal is not None:
                    extra.append('stop_requested:'+stop.signal)
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
                result, arrays = evaluate(messages, transforms, registered=args.registered_depth,
                    reference_frame=args.reference_frame, base_frame=args.base_frame,
                    config=config, extra_reasons=extra)
                result.update(capture_session=manifest['capture_session'], case_id=args.case_id,
                              capture_barrier_stamp_ns=window.barrier_ns,
                              evaluation_clock_stamp_ns=window.clock_ns,
                              clock_receipt_age_wall_s=None if clock_wall is None else now-clock_wall,
                              tf_lookup_errors=tf_errors,
                              wall_offset_s=now-started)
                path = writer.save_sample(sample['sequence'], messages, result, arrays)
                window.completed.append({'sequence': sample['sequence'], 'stamp_ns': sample['stamp_ns'],
                    'artifact': path, 'frame_valid': result['frame_valid'], 'reasons': result['reasons'],
                    'processing_status': result['processing_status'],
                    'detection_count': None if result['detections'] is None else len(result['detections'])})
                window.pending.remove(sample)

        while stop.signal is None and time.monotonic() < deadline and len(window.completed) < args.pairs:
            rclpy.spin_once(node, timeout_sec=min(.05, max(0, deadline-time.monotonic())))
            if stop.signal is not None or fatal or window.clock_reset:
                break
            finish_ready()
        if fatal:
            errors += fatal
        if window.clock_reset:
            errors.append('clock_reset_during_capture')
        finish_ready(force=True)
    except (Exception, KeyboardInterrupt) as exc:
        errors.append(type(exc).__name__+': '+str(exc))
    finally:
        # Freeze sampling before native cleanup. The immutable checkpoint records
        # only writes that returned after flush/fsync; pending files are not
        # promoted to complete samples if a save failed part-way through.
        unchanged = True
        for path, original in sources['sources'].items():
            try:
                unchanged &= hashlib.sha256(Path(path).read_bytes()).hexdigest() == original['sha256']
            except OSError:
                unchanged = False
        if not unchanged:
            errors.append('source_changed_during_capture')
        elapsed = time.monotonic()-started
        complete = len(window.completed) == args.pairs and elapsed <= args.timeout
        if stop.signal is not None:
            errors.append('stop_requested:'+stop.signal)
        if not complete:
            errors.append('requested_pair_window_incomplete_or_wall_deadline')
        integrity = end_of_window_integrity(window)
        errors.extend(integrity.pop('errors'))
        sampling = {'schema_version': 2, 'capture_session': manifest['capture_session'],
            'case_id': args.case_id, 'read_only': True,
            'sampling_complete': complete, 'collection_complete': complete,
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
            'received_counts': dict(window.counts), 'diagnostic_events': window.events,
            'limits': ['First bounded received sample window only; no lossless-delivery or whole-scene claim.',
                       'No motion was commanded by this tool; physical stationarity is not established.',
                       'No truth labels were read; empty detections are algorithm output, not target absence.',
                       'Input, per-candidate depth and exact-TF rejections remain separate.',
                       'Committed files are flush/fsync complete; pending samples may have only partial artifact files.',
                       'Late duplicate conflicts invalidate the end-of-window verdict and are linked separately; saved samples are never rewritten.',
                       'A missing summary means cleanup completion is unknown, even if sampling completed.',
                       'An outer owner must enforce a hard deadline if file I/O or native code blocks.'],
            'effective_range_claimed': False, 'coverage_claimed': False}
        sampling['sampling_passed'] = complete and unchanged and sampling['all_samples_valid'] and not errors
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
