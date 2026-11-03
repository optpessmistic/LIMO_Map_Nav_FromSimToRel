"""Pure RGB-D sampling helpers. No ROS, Gazebo, mission or truth-file imports."""
from collections import Counter, OrderedDict
from dataclasses import asdict
import hashlib
import importlib
import json
import math
import os
from pathlib import Path
import struct
import sys
import zlib
from types import SimpleNamespace as NS

import numpy as np


def repository():
    for parent in Path(__file__).resolve().parents:
        if (parent / 'src/limo_maze_perception/limo_maze_perception/core.py').is_file():
            return parent
    raise RuntimeError('Cannot locate current checkout perception source')


REPO = repository()
PACKAGE = REPO / 'src/limo_maze_perception'
sys.path.insert(0, str(PACKAGE))
detector = importlib.import_module('limo_maze_perception.core')
evidence = importlib.import_module('limo_maze_perception.evidence')
for module, name in ((detector, 'core.py'), (evidence, 'evidence.py')):
    if Path(module.__file__).resolve() != (PACKAGE/'limo_maze_perception'/name).resolve():
        raise RuntimeError('Perception import resolved outside the current checkout')

TOPICS = {'rgb': '/camera/color/image_raw', 'depth': '/camera/depth/image_raw',
          'rgb_info': '/camera/color/camera_info', 'depth_info': '/camera/depth/camera_info'}
MAX_PIXELS = 1_048_576
MAX_IMAGE_BYTES = 8*1024**2
MAX_MEMORY_BYTES = 128*1024**2


def freeze_message(kind, message):
    """Detach bounded received content from ROS-owned mutable message objects."""
    header = NS(stamp=NS(sec=message.header.stamp.sec, nanosec=message.header.stamp.nanosec),
                frame_id=message.header.frame_id)
    if len(header.frame_id) > 256:
        raise ValueError('frame_id_length_limit')
    if kind in ('rgb', 'depth'):
        if len(message.encoding) > 32 or len(message.data) > MAX_IMAGE_BYTES:
            raise ValueError('image_storage_limit')
        return NS(header=header, data=bytes(message.data), **{key: getattr(message, key)
                  for key in ('encoding', 'width', 'height', 'step', 'is_bigendian')})
    if any(len(getattr(message, key)) > 64 for key in ('k', 'd', 'r', 'p')):
        raise ValueError('camera_calibration_length_limit')
    if len(message.distortion_model) > 128:
        raise ValueError('distortion_model_length_limit')
    return NS(header=header, **{key: getattr(message, key) for key in
              ('width', 'height', 'distortion_model', 'binning_x', 'binning_y')},
              **{key: tuple(getattr(message, key)) for key in ('k', 'd', 'r', 'p')},
              roi=NS(**{key: getattr(message.roi, key) for key in
                       ('x_offset', 'y_offset', 'height', 'width', 'do_rectify')}))


def json_bytes(value):
    return (json.dumps(evidence.finite_json(value), indent=2, sort_keys=True,
                       ensure_ascii=False, allow_nan=False)+'\n').encode('utf-8')


def png_bytes(image):
    """Lossless PNG encoding only; no resizing, filtering or image editing."""
    if image.dtype != np.uint8 or image.ndim not in (2, 3):
        raise ValueError('PNG requires uint8 gray/RGB pixels')
    if image.ndim == 3 and image.shape[2] != 3:
        raise ValueError('PNG RGB requires three channels')
    height, width = image.shape[:2]
    if not 0 < height*width <= MAX_PIXELS:
        raise ValueError('PNG geometry exceeds capture bounds')
    def chunk(name, content):
        return struct.pack('!I', len(content))+name+content+struct.pack('!I', zlib.crc32(name+content)&0xffffffff)
    rows = b''.join(b'\0'+np.ascontiguousarray(row).tobytes() for row in image)
    header = struct.pack('!IIBBBBB', width, height, 8, 2 if image.ndim == 3 else 0, 0, 0, 0)
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR', header)+chunk(b'IDAT', zlib.compress(rows))+chunk(b'IEND', b'')


def decode_image(message):
    """Interpret ROS rows, padding and endianness without CvBridge or ROS."""
    if message.width*message.height > MAX_PIXELS or len(message.data) > MAX_IMAGE_BYTES:
        raise ValueError('image_size_limit')
    raw = bytes(message.data)
    if message.encoding in ('rgb8', 'bgr8'):
        return np.frombuffer(raw, dtype=np.uint8).reshape(message.height, message.step)[:, :message.width*3].reshape(message.height, message.width, 3).copy()
    dtype, native = ({'32FC1': ('f4', np.float32), '16UC1': ('u2', np.uint16)})[message.encoding]
    dtype = np.dtype(('>' if message.is_bigendian else '<')+dtype)
    return np.ndarray((message.height, message.width), dtype=dtype, buffer=raw,
                      strides=(message.step, dtype.itemsize)).astype(native, copy=True)


def metadata(messages):
    return {name: (evidence.image_metadata(msg) if name in ('rgb', 'depth')
                   else evidence.camera_metadata(msg)) for name, msg in messages.items()}


def input_reasons(meta, registered):
    if not registered:
        return ['depth_registration_unconfirmed']
    rgb, depth = meta.get('rgb'), meta.get('depth')
    reasons = []
    for key in ('rgb_info', 'depth_info'):
        reason = evidence.metadata_reason(rgb, depth, meta.get(key), slop_ns=0, info_max_age_ns=0)
        if reason:
            reasons.append(key+':'+reason)
    if not reasons:
        if meta['rgb_info']['calibration_id'] != meta['depth_info']['calibration_id']:
            reasons.append('registered_camera_calibration_mismatch')
        for image in (rgb, depth):
            if image['width']*image['height'] > MAX_PIXELS or image['data_bytes'] > MAX_IMAGE_BYTES:
                reasons.append('image_size_limit')
    return list(dict.fromkeys(reasons))


def validate_transforms(transforms, stamp, optical_frame, reference_frame, base_frame):
    expected = {'reference_to_base': (reference_frame, base_frame, False),
                'base_to_camera': (base_frame, optical_frame, True),
                'reference_to_camera': (reference_frame, optical_frame, False)}
    reasons = []
    for key, (target, source, allow_static_zero) in expected.items():
        item = transforms.get(key)
        if item is None:
            reasons.append('missing_exact_tf:'+key)
            continue
        if (item.get('target_frame') != target or item.get('source_frame') != source
                or item.get('query_stamp_ns') != stamp):
            reasons.append('tf_frame_or_query_stamp_mismatch:'+key)
        returned = item.get('returned_stamp_ns')
        if type(returned) is not int or (returned != stamp and not (allow_static_zero and returned == 0)):
            reasons.append('tf_return_stamp_mismatch:'+key)
        try:
            t, q = item['translation_xyz_m'], item['rotation_xyzw']
            if len(t) != 3 or len(q) != 4 or not all(math.isfinite(v) for v in (*t, *q)):
                raise ValueError('nonfinite or wrong dimensions')
            if abs(sum(v*v for v in q)-1.) > 1e-3:
                raise ValueError('quaternion not unit length')
        except (KeyError, TypeError, ValueError):
            reasons.append('invalid_tf_values:'+key)
    return reasons


def evaluate(messages, transforms, *, registered, reference_frame='odom', base_frame='base_footprint',
             config=None, extra_reasons=(), run_detection=True):
    """Call the unchanged production detector once for eligible image inputs.

    A missing TF does not hide the independent image result. Input rejections,
    detector depth rejections and exact-TF validity are reported separately.
    """
    config = config or detector.DetectorConfig()
    config.validate()
    meta = metadata(messages)
    reasons = input_reasons(meta, registered)
    report = {'schema_version': 1, 'input_metadata': meta,
              'detector_config': asdict(config), 'detector_config_id': evidence.digest(asdict(config)),
              'input_reasons': list(reasons), 'extra_reasons': list(extra_reasons),
              'processing_status': 'not_run', 'detections': None, 'processing': None,
              'transforms': transforms, 'frame_valid': False,
              'interpretation': 'Current color/depth algorithm output only. No target identity, absence, range, coverage or ground-truth assessment.'}
    arrays = {}
    stamp = meta.get('rgb', {}).get('stamp_ns') if meta.get('rgb') else None
    optical = meta.get('rgb', {}).get('frame_id') if meta.get('rgb') else None
    report['stamp_ns'], report['optical_frame'] = stamp, optical
    report['tf_reasons'] = validate_transforms(transforms, stamp, optical, reference_frame, base_frame)
    if not run_detection:
        report['extra_reasons'].append('offline_processing_skipped')
    if not reasons and run_detection:
        try:
            color, depth = decode_image(messages['rgb']), decode_image(messages['depth'])
            arrays['rgb'] = color if messages['rgb'].encoding == 'rgb8' else color[..., ::-1].copy()
            arrays['depth_raw'] = depth
            arrays['depth_m'] = depth.astype(np.float32)*(0.001 if messages['depth'].encoding == '16UC1' else 1.)
            camera = meta['rgb_info']['calibration']
            intrinsics = detector.Intrinsics(camera['width'], camera['height'], camera['k'][0],
                                           camera['k'][4], camera['k'][2], camera['k'][5])
            detections, stats = detector.process_red_frame(color, depth, intrinsics,
                messages['depth'].encoding, messages['rgb'].encoding, config)
            report.update(processing_status='processed', detections=detections, processing=stats)
            arrays['red_mask'] = detector.red_mask(color, messages['rgb'].encoding, config).astype(np.uint8)*255
            if stats['valid_depth_pixels'] < config.min_depth_pixels:
                report['input_reasons'].append('insufficient_frame_depth')
            report['detection_summary'] = ('detections' if detections else stats['empty_reason'])
        except (ValueError, TypeError, KeyError, IndexError, OverflowError) as exc:
            report['processing_status'] = 'rejected'
            report['input_reasons'].append('decode_or_core_error:'+str(exc))
    report['reasons'] = report['input_reasons']+report['tf_reasons']+report['extra_reasons']
    report['frame_valid'] = not report['reasons'] and report['processing_status'] == 'processed'
    return report, arrays


class FixedWindow:
    """Select N advancing pairs in callback-time availability order.

    No valid-until-N policy and no unbounded image history. CameraInfo may arrive
    later. Duplicate payload conflicts remain sticky collection-level evidence.
    Sort only pairs available together in one callback; do not wait for an older
    unmatched sensor stamp. A late pair at/below the watermark is rejected.
    """
    def __init__(self, count, pair_wait=2., cache_size=16, max_memory_bytes=MAX_MEMORY_BYTES):
        if type(count) is not int or not 1 <= count <= 32:
            raise ValueError('sample count must be 1..32')
        if not math.isfinite(pair_wait) or not .01 <= pair_wait <= 10:
            raise ValueError('pair_wait must be .01..10 wall seconds')
        self.count, self.pair_wait, self.cache_size = count, pair_wait, cache_size
        if type(max_memory_bytes) is not int or max_memory_bytes < 1:
            raise ValueError('max_memory_bytes must be positive')
        self.max_memory_bytes = max_memory_bytes
        self.max_observed_image_bytes = 0
        self.closed = False
        self.cache = {key: OrderedDict() for key in TOPICS}
        self.signatures = {key: OrderedDict() for key in TOPICS}
        # Ordinary signature history is bounded independently of selected pairs.
        # Retain the first signature of each selected stamp/topic until sampling
        # ends, even after its images have been written and removed from pending.
        self.pinned_signatures = {key: {} for key in TOPICS}
        self.selected_sequences = {}
        self.clock_ns = self.barrier_ns = self.watermark = None
        self.clock_reset = False
        self.pending, self.completed, self.events = [], [], []
        self.counts = Counter()
        self.conflict_stamps = set()
        self.conflict_details = {}

    def image_bytes_held(self):
        # Count unique detached image payloads, including selected frozen slots.
        payloads = {}
        for kind in ('rgb', 'depth'):
            for message in self.cache[kind].values():
                payloads[id(message.data)] = len(message.data)
            for sample in self.pending:
                message = sample[kind]
                payloads[id(message.data)] = len(message.data)
        return sum(payloads.values())

    def trim_unselected_conflicts(self):
        retained = set(self.selected_sequences)
        for history in self.signatures.values():
            retained.update(history)
        self.conflict_stamps.intersection_update(retained)
        for stamp in list(self.conflict_details):
            if stamp not in retained:
                del self.conflict_details[stamp]

    def pin_selected(self, stamp, sequence):
        self.selected_sequences[stamp] = sequence
        for kind, history in self.signatures.items():
            if stamp in history:
                self.pinned_signatures[kind][stamp] = history[stamp]

    def selected_conflicts(self):
        """Bounded associations survive event truncation and ordinary LRU eviction."""
        saved = {sample['sequence']: sample for sample in self.completed}
        return [{'sequence': sequence, 'stamp_ns': stamp,
                 'artifact': saved.get(sequence, {}).get('artifact'),
                 'sample_written': sequence in saved,
                 'topics': {kind: dict(details) for kind, details in self.conflict_details[stamp].items()}}
                for stamp, sequence in self.selected_sequences.items()
                if stamp in self.conflict_details]

    def event(self, reason, **details):
        self.counts[reason] += 1
        if len(self.events) < 40:
            self.events.append({'reason': reason, **details})

    def on_clock(self, ns):
        if self.closed:
            return
        if type(ns) is not int or ns < 0:
            self.event('invalid_clock')
            return
        if self.clock_ns is not None and ns < self.clock_ns:
            self.clock_reset = True
            self.event('clock_reset', previous=self.clock_ns, current=ns)
        self.clock_ns = ns
        if self.barrier_ns is None and ns > 0:
            self.barrier_ns = ns

    def receive(self, kind, message, wall):
        if self.closed:
            return  # The sampling/conflict boundary is explicit and immutable.
        self.counts['received_'+kind] += 1
        ns = evidence.source_stamp_ns(message)
        if ns is None:
            self.event('invalid_source_stamp', topic=kind)
            return
        if self.barrier_ns is None or ns <= self.barrier_ns or self.clock_reset:
            self.event('before_capture_barrier', topic=kind, stamp_ns=ns)
            return
        if kind in ('rgb', 'depth') and (len(message.data) > MAX_IMAGE_BYTES or
                type(message.width) is not int or type(message.height) is not int or
                message.width*message.height > MAX_PIXELS):
            self.event('oversized_input_dropped', topic=kind, stamp_ns=ns)
            return
        # Limit metadata before its hashing path copies any variable-sized lists.
        if kind not in ('rgb', 'depth'):
            message = freeze_message(kind, message)
        meta = evidence.image_metadata(message) if kind in ('rgb', 'depth') else evidence.camera_metadata(message)
        signature = evidence.digest(meta)
        old = self.pinned_signatures[kind].get(ns, self.signatures[kind].get(ns))
        if old is not None:
            if old != signature:
                self.conflict_stamps.add(ns)
                detail = self.conflict_details.setdefault(ns, {}).setdefault(kind, {
                    'retained_signature': old, 'first_conflicting_signature': signature,
                    'last_conflicting_signature': signature, 'observed_conflicting_count': 0})
                detail['last_conflicting_signature'] = signature
                detail['observed_conflicting_count'] += 1
                self.event('conflicting_duplicate', topic=kind, stamp_ns=ns,
                           selected_sequence=self.selected_sequences.get(ns))
            else:
                self.event('identical_duplicate', topic=kind, stamp_ns=ns)
            return
        if kind in ('rgb', 'depth'):
            if self.image_bytes_held()+len(message.data) > self.max_memory_bytes:
                self.event('memory_budget_exceeded', topic=kind, stamp_ns=ns)
                raise MemoryError('capture_raw_memory_budget_exceeded')
            message = freeze_message(kind, message)
        self.signatures[kind][ns] = signature
        if ns in self.selected_sequences:
            self.pinned_signatures[kind][ns] = signature
        while len(self.signatures[kind]) > 64:
            self.signatures[kind].popitem(last=False)
        self.trim_unselected_conflicts()
        self.cache[kind][ns] = message
        while len(self.cache[kind]) > self.cache_size:
            dropped, _ = self.cache[kind].popitem(last=False)
            self.event('cache_evicted', topic=kind, stamp_ns=dropped)
        self.max_observed_image_bytes = max(self.max_observed_image_bytes, self.image_bytes_held())
        while len(self.pending)+len(self.completed) < self.count:
            shared = sorted(self.cache['rgb'].keys() & self.cache['depth'].keys())
            if not shared:
                break
            stamp = shared[0]
            rgb, depth = self.cache['rgb'].pop(stamp), self.cache['depth'].pop(stamp)
            if self.watermark is not None and stamp <= self.watermark:
                self.event('nonadvancing_pair', stamp_ns=stamp)
                continue
            self.watermark = stamp
            sequence = len(self.pending)+len(self.completed)+1
            self.pin_selected(stamp, sequence)
            self.pending.append({'sequence': sequence,
                                 'stamp_ns': stamp, 'selected_wall': wall, 'deadline_wall': wall+self.pair_wait,
                                 'rgb': rgb, 'depth': depth})

    def messages(self, sample):
        stamp = sample['stamp_ns']
        for key in ('rgb_info', 'depth_info'):
            if key not in sample and stamp in self.cache[key]:
                sample[key] = self.cache[key][stamp]
        return {'rgb': sample['rgb'], 'depth': sample['depth'],
                'rgb_info': sample.get('rgb_info'), 'depth_info': sample.get('depth_info')}


class CaptureWriter:
    """Exclusive writes and a fixed byte budget; no artifact is overwritten."""
    REPORT_SLOT_BYTES = 128*1024
    REPORT_RESERVE_BYTES = 3*REPORT_SLOT_BYTES

    def __init__(self, output, max_bytes=256*1024**2):
        self.root = Path(output)
        self.root.mkdir(parents=True, exist_ok=False)
        self.max_bytes, self.written = max_bytes, 0
        self.artifacts = {}

    def write(self, name, data, final=False):
        limit = self.max_bytes if final else self.max_bytes-self.REPORT_RESERVE_BYTES
        if self.written+len(data) > limit:
            raise OSError('capture_byte_budget_exceeded')
        path = self.root/name
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('xb') as output:
            output.write(data)
            output.flush()
            os.fsync(output.fileno())
        self.written += len(data)
        result = {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
        self.artifacts[name] = result
        return result

    def write_report(self, name, report):
        payload = json_bytes(report)
        if len(payload) > self.REPORT_SLOT_BYTES:
            raise OSError('capture_report_slot_exceeded')
        return self.write(name, payload, final=True)

    def save_raw(self, sequence, messages, observation):
        prefix = f'sample_{sequence:03d}/'
        files = {}
        for key in ('rgb', 'depth'):
            message = messages.get(key)
            if message is not None and len(message.data) <= MAX_IMAGE_BYTES:
                files[key+'.data'] = self.write(prefix+key+'.data', bytes(message.data))
        self.write(prefix+'raw.json', json_bytes({'sequence': sequence,
            'input_metadata': metadata(messages), 'observation': observation, 'files': files,
            'processing_status': 'not_run_yet'}))
        return files

    def save_sample(self, sequence, messages, report, arrays, *, raw_files=None):
        import io
        prefix = f'sample_{sequence:03d}/'
        files = dict(raw_files or {})
        if raw_files is None:
            for key in ('rgb', 'depth'):
                message = messages.get(key)
                if message is not None and len(message.data) <= MAX_IMAGE_BYTES:
                    files[key+'.data'] = self.write(prefix+key+'.data', bytes(message.data))
        for key, array in arrays.items():
            if key in ('rgb', 'red_mask'):
                name, data = key+'.png', png_bytes(array)
            else:
                buffer = io.BytesIO()
                np.save(buffer, array, allow_pickle=False)
                name, data = key+'.npy', buffer.getvalue()
            files[name] = self.write(prefix+name, data)
        report['files'] = files
        report['sequence'] = sequence
        self.write(prefix+'sample.json', json_bytes(report))
        return prefix+'sample.json'


def provenance(extra_sources=()):
    paths = [PACKAGE/'limo_maze_perception'/name for name in ('core.py', 'evidence.py', 'node.py')]
    paths += [Path(__file__), *map(Path, extra_sources)]
    return {'sources': {str(path.resolve()): {'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                                            'bytes': path.stat().st_size} for path in paths},
            'detector_import_path': str(Path(detector.__file__).resolve()),
            'numpy_version': np.__version__,
            'opencv_version': detector.cv2.__version__ if detector.cv2 is not None else None,
            'backend': 'opencv' if detector.cv2 is not None else 'numpy_fallback'}
