"""Read closed retained RGB-D evidence for offline geometric diagnostics only.

Eligibility is the intersection of the saved audit and a fresh byte audit. A
retired observation may supply geometry, never restored observation credit.
No world, detector targets, native scene, or ground-truth files are read.
Decoded rows are compact arrays in the ORIGINAL units: float metres (32FC1),
or unsigned integer millimetres (16UC1). Invalid float depths remain invalid.
"""
from array import array
import hashlib
import json
import math
import os
from pathlib import Path
import stat
import sys

from tools.audit_frame_retention import FILES, audit
from limo_evidence_protocol.retention import sha, validate_request


MAX_DECODED_BYTES = 256 * 1024 * 1024
MAX_INPUT_BYTES = 512 * 1024 * 1024
MAX_IMAGE_BYTES = 32 * 1024 * 1024
MAX_PIXELS = 4 * 1024 * 1024
MAX_MAP_CELLS = 4 * 1024 * 1024
_JSON_LIMIT = 32 * 1024 * 1024


def _integer(value, name, minimum=0, maximum=2**63-1):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError('invalid_' + name)
    return value


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _object_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate_json_key:' + key)
        result[key] = value
    return result


def _invalid_constant(value):
    raise ValueError('nonfinite_json:' + value)


def _json_float(value):
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError('nonfinite_json:' + value)
    return parsed


def _json(raw):
    value = json.loads(raw, object_pairs_hook=_object_pairs,
                       parse_constant=_invalid_constant, parse_float=_json_float)
    if type(value) is not dict:
        raise ValueError('json_object_required')
    return value


def _signature(value):
    return (value.st_dev, value.st_ino, value.st_mode, value.st_size,
            value.st_mtime_ns, value.st_ctime_ns)


def _link(path, value):
    return (stat.S_ISLNK(value.st_mode) or
            bool(getattr(value, 'st_file_attributes', 0) &
                 getattr(stat, 'FILE_ATTRIBUTE_REPARSE_POINT', 0)))


class _Inputs:
    """Bounded stable reads, no-follow opens, and a final complete rehash.

    This detects ordinary concurrent writers/replacements; like the underlying
    audit it is not an adversarial filesystem snapshot or a filesystem lock.
    """
    def __init__(self, root):
        self.root = Path(root).absolute()
        if '..' in self.root.parts or self.root.resolve() != self.root:
            raise ValueError('canonical_run_root_required')
        for path in (*reversed(self.root.parents), self.root):
            value = path.lstat()
            if _link(path, value) or not stat.S_ISDIR(value.st_mode):
                raise ValueError('unsafe_run_root')
        self.records, self.directories = {}, {}
        self.total_bytes = 0
        self._path(self.root)

    def _path(self, path):
        path = Path(path)
        try:
            relative = path.relative_to(self.root)
        except ValueError as exc:
            raise ValueError('path_escape') from exc
        if any(part in ('..', '.') for part in relative.parts):
            raise ValueError('path_escape')
        current = self.root
        for part in (None, *relative.parts):
            if part is not None:
                current = current / part
            try:
                value = current.lstat()
            except FileNotFoundError:
                return None
            if _link(current, value) or current.resolve() != current:
                raise ValueError('unsafe_path:' + str(current.relative_to(self.root)))
            if current != path or stat.S_ISDIR(value.st_mode):
                if not stat.S_ISDIR(value.st_mode):
                    raise ValueError('non_directory_parent')
                # Directory mtime changes when files are added or replaced.
                signature = _signature(value)
                old = self.directories.setdefault(current, signature)
                if old != signature:
                    raise ValueError('input_changed:' + str(current.relative_to(self.root)))
        return value

    def read(self, relative, limit, *, capture=True, optional=False):
        path = self.root / relative
        before = self._path(path)
        if before is None:
            fingerprint, raw = None, None
            if not optional:
                raise ValueError('missing_input:' + str(relative))
        else:
            if not stat.S_ISREG(before.st_mode) or before.st_size > limit:
                raise ValueError('unsafe_or_oversize_input:' + str(relative))
            flags = os.O_RDONLY | getattr(os, 'O_BINARY', 0) | getattr(os, 'O_NOFOLLOW', 0)
            descriptor = os.open(path, flags)
            digest, count, chunks = hashlib.sha256(), 0, []
            with os.fdopen(descriptor, 'rb') as stream:
                opened = os.fstat(stream.fileno())
                # Some Windows Python runtimes expose creation time via
                # lstat.st_ctime but change time via fstat.st_ctime. Compare
                # ctime only within the same API, retaining identity/size/mtime
                # checks across the path and descriptor.
                if _signature(opened)[:-1] != _signature(before)[:-1]:
                    raise ValueError('input_changed:' + str(relative))
                while chunk := stream.read(min(1024 * 1024, limit + 1 - count)):
                    count += len(chunk)
                    if count > limit:
                        raise ValueError('input_grew:' + str(relative))
                    digest.update(chunk)
                    if capture:
                        chunks.append(chunk)
                if _signature(os.fstat(stream.fileno())) != _signature(opened):
                    raise ValueError('input_changed:' + str(relative))
            after = self._path(path)
            if after is None or _signature(after) != _signature(before) or count != before.st_size:
                raise ValueError('input_changed:' + str(relative))
            fingerprint = (_signature(before), digest.hexdigest(), count)
            raw = b''.join(chunks) if capture else None
        key = Path(relative).as_posix()
        if key in self.records:
            if fingerprint != self.records[key][0]:
                raise ValueError('input_changed:' + key)
        else:
            self.total_bytes += fingerprint[2] if fingerprint else 0
            if self.total_bytes > MAX_INPUT_BYTES:
                raise ValueError('input_byte_budget')
            self.records[key] = (fingerprint, limit)
        return raw

    def json(self, relative, limit=_JSON_LIMIT):
        return _json(self.read(relative, limit))

    def finish(self):
        # Reopen and hash all fixed inputs, including excluded/missing frames.
        for path, (_, limit) in list(self.records.items()):
            self.read(path, limit, capture=False, optional=True)
        for path, signature in self.directories.items():
            value = self._path(path)
            if value is None or _signature(value) != signature:
                raise ValueError('input_changed:' + str(path))
        return {path: {'sha256': value[1] if value else None,
                       'bytes': value[2] if value else None}
                for path, (value, _) in sorted(self.records.items())}


def _audit_rows(value, identities, name):
    if (type(value) is not dict or value.get('schema_version') != 1 or
            value.get('errors') != [] or type(value.get('passed')) is not bool):
        raise ValueError(name + '_global_errors_or_schema')
    frames = value.get('frames')
    if type(frames) is not list or len(frames) != len(identities):
        raise ValueError(name + '_missing_audit_identity')
    rows = {}
    for row in frames:
        if type(row) is not dict or type(row.get('verified')) is not bool:
            raise ValueError(name + '_invalid_frame_row')
        rid = row.get('retention_id')
        if (type(rid) is not str or rid in rows or rid not in identities or
                row.get('request_sha256') != identities[rid]):
            raise ValueError(name + '_audit_identity_mismatch')
        rows[rid] = row
    verified = sum(row['verified'] for row in rows.values())
    if (type(value.get('accepted_count')) is not int or value['accepted_count'] != len(rows) or
            type(value.get('verified_count')) is not int or value['verified_count'] != verified or
            value['passed'] != (bool(rows) and verified == len(rows))):
        raise ValueError(name + '_audit_count_or_status_mismatch')
    return rows


def _map(saved, epoch, frame):
    snapshot = saved.get('snapshot', {})
    if (saved.get('schema_version') != 1 or saved.get('passed') is not True or
            saved.get('errors') != [] or snapshot.get('valid') is not True or
            snapshot.get('errors') != []):
        raise ValueError('invalid_final_map_snapshot')
    if snapshot.get('epoch', {}).get('raw') != epoch:
        raise ValueError('final_map_epoch_mismatch')
    grid = snapshot['map']
    width = _integer(grid['width'], 'map_width', 1, MAX_MAP_CELLS)
    height = _integer(grid['height'], 'map_height', 1, MAX_MAP_CELLS)
    if width * height > MAX_MAP_CELLS:
        raise ValueError('map_cell_budget')
    data = grid['data']
    if (grid['frame_id'] != frame or not _finite(grid['resolution']) or grid['resolution'] <= 0 or
            type(data) is not list or len(data) != width * height or
            any(type(v) is not int or not -1 <= v <= 100 for v in data)):
        raise ValueError('invalid_map_grid')
    _integer(grid['stamp_ns'], 'map_stamp')
    origin = grid['origin']
    xyz, quat = origin['xyz'], origin['quaternion_xyzw']
    if (origin.get('valid_geometry') is not True or type(xyz) is not list or len(xyz) != 3 or
            type(quat) is not list or len(quat) != 4 or not all(_finite(v) for v in xyz + quat) or
            abs(sum(v*v for v in quat)-1.) > 1e-3):
        raise ValueError('invalid_map_origin')
    # Fixed whitelist: never return the final-save environment or other sources.
    result = {key: grid[key] for key in
              ('frame_id', 'stamp_ns', 'width', 'height', 'resolution', 'data')}
    result['origin'] = {key: origin[key] for key in ('xyz', 'quaternion_xyzw', 'valid_geometry')}
    return result


def _decode(producer, camera, request, raw):
    rgb, depth, sample = producer['rgb'], producer['depth'], request['sample']
    width = _integer(depth['width'], 'depth_width', 1, 4096)
    height = _integer(depth['height'], 'depth_height', 1, 4096)
    if width * height > MAX_PIXELS:
        raise ValueError('image_pixel_budget')
    stamp, frame = sample['rgb_stamp_ns'], sample['frame_id']
    calibration = camera['calibration']
    _integer(calibration['width'], 'camera_width', 1, 4096)
    _integer(calibration['height'], 'camera_height', 1, 4096)
    _integer(camera['stamp_ns'], 'camera_stamp')
    if (rgb['stamp_ns'] != stamp or depth['stamp_ns'] != stamp or sample['depth_stamp_ns'] != stamp or
            request['tf']['stamp_ns'] != stamp or camera['stamp_ns'] != stamp or
            rgb['frame_id'] != frame or depth['frame_id'] != frame or camera['frame_id'] != frame or
            rgb['width'] != width or rgb['height'] != height or
            calibration['width'] != width or calibration['height'] != height or
            camera['calibration_id'] != sample['calibration_id']):
        raise ValueError('rgb_depth_camera_tf_binding')
    # Camera calibration interpretation (distortion/ROI/etc.) is the geometry
    # layer's responsibility. Here bind its exact retained identity and shape.
    for meta in (rgb, depth):
        for key in ('width', 'height', 'step', 'data_bytes'):
            _integer(meta[key], 'image_' + key, 1, MAX_IMAGE_BYTES)
        _integer(meta['is_bigendian'], 'image_endian', 0, 1)
        if meta['height'] * meta['step'] != meta['data_bytes']:
            raise ValueError('image_row_layout')
    rgb_channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}
    if rgb.get('encoding') not in rgb_channels or rgb['step'] < width * rgb_channels[rgb['encoding']]:
        raise ValueError('unsupported_rgb_layout')
    encoding = depth['encoding']
    if encoding not in ('32FC1', '16UC1'):
        raise ValueError('unsupported_depth_encoding')
    code, itemsize = ('f', 4) if encoding == '32FC1' else ('H', 2)
    if array(code).itemsize != itemsize:
        raise ValueError('unsupported_native_array_size')
    step = depth['step']
    if step < width * itemsize or len(raw) != height * step:
        raise ValueError('depth_row_layout')
    swap = bool(depth['is_bigendian']) != (sys.byteorder == 'big')
    rows = []
    for row in range(height):
        values = array(code)
        values.frombytes(raw[row * step:row * step + width * itemsize])
        if swap:
            values.byteswap()
        rows.append(values)
    return rows, width * height * itemsize


def _mission_summary(mission):
    keys = ('schema_version', 'frame_id', 'map_epoch', 'status', 'reason', 'mission_success',
            'returned_pose_verified', 'map_complete', 'visual_coverage_complete',
            'mission_timed_out', 'coordinates_valid')
    result = {key: mission[key] for key in keys if key in mission}
    # Return the fixed membership and original state, without arbitrary mission
    # extensions or detector output. No window is promoted to "satisfied".
    consumer = mission['frame_retention']
    windows = []
    for window in consumer['windows']:
        rows = [{'request': validate_request(entry['request'])[0],
                 'request_sha256': entry['request_sha256'], 'state': entry.get('state')}
                for entry in window['accepted']]
        windows.append({**{key: window[key] for key in
                           ('state', 'reason', 'binding', 'ids', 'ledger_accepted_count') if key in window},
                        'accepted': rows})
    result['frame_retention'] = {'enabled': consumer['enabled'], 'fault': consumer['fault'], 'windows': windows}
    return result


def load_run(run_root: Path, mission_name: str, *, max_frames=256) -> dict:
    """Load the named fixed mission and closed retained evidence, without ROS.

    Missing/global audit identity, epoch errors, unsafe paths, changed inputs,
    and resource-budget excess raise ValueError. A failed original/current
    per-frame audit remains an exclusion; a later clean audit cannot revive it.
    source_files point at the original directory; no images are copied.
    """
    try:
        return _load_run(run_root, mission_name, max_frames=max_frames)
    except (KeyError, TypeError, AttributeError, OSError, OverflowError) as exc:
        raise ValueError('invalid_or_unreadable_evidence:' + str(exc)[:256]) from exc


def _load_run(run_root, mission_name, *, max_frames):
    _integer(max_frames, 'max_frames', 1, 4096)
    if (type(mission_name) is not str or not mission_name.endswith('.json') or
            '/' in mission_name or '\\' in mission_name or ':' in mission_name or
            mission_name in ('.', '..') or Path(mission_name).name != mission_name):
        raise ValueError('mission_basename_required')
    inputs = _Inputs(run_root)
    mission = inputs.json(Path('mission') / mission_name)
    original = inputs.json('retention-audit.json', 4 * 1024 * 1024)
    epoch, frame = mission.get('map_epoch'), mission.get('frame_id')
    if (mission.get('schema_version') != 1 or type(epoch) is not str or not epoch.strip() or
            type(frame) is not str or not frame.strip()):
        raise ValueError('mission_identity_required')
    consumer = mission['frame_retention']
    entries, identities = [], {}
    for window in consumer['windows']:
        for entry in window['accepted']:
            request, raw = validate_request(entry['request'])
            rid, fingerprint = request['retention_id'], sha(raw)
            if request['map_epoch'] != epoch or request['sample']['map_epoch'] != epoch:
                raise ValueError('accepted_epoch_mismatch')
            if rid in identities or entry.get('request_sha256') != fingerprint:
                raise ValueError('accepted_identity_mismatch')
            identities[rid] = fingerprint
            entries.append((rid, fingerprint, request))
            if len(entries) > max_frames:
                raise ValueError('frame_count_budget')
    original_rows = _audit_rows(original, identities, 'original')
    checkpoint = inputs.json('retained/checkpoint.json', 4 * 1024 * 1024)
    if checkpoint['core']['map_epoch'] != epoch:
        raise ValueError('checkpoint_epoch_mismatch')
    for _, _, request in entries:
        if request['sample']['detector_session_id'] != checkpoint['core']['detector_session_id']:
            raise ValueError('checkpoint_session_mismatch')
    # Preflight all paths/sizes before the existing auditor opens any artifact.
    # The identities are computed from fixed requests, never untrusted paths.
    for _, fingerprint, _ in entries:
        for name in sorted(FILES):
            inputs.read(Path('retained') / fingerprint / name,
                        MAX_IMAGE_BYTES if name.endswith('.bin') else 65536,
                        capture=False, optional=True)
    current = audit(mission, inputs.root / 'retained')
    current_rows = _audit_rows(current, identities, 'current')
    map_snapshot = _map(inputs.json('final-save.json'), epoch, frame)
    frames, excluded, decoded_bytes = [], [], 0
    for rid, fingerprint, request in entries:
        if not original_rows[rid]['verified'] or not current_rows[rid]['verified']:
            excluded.append({'retention_id': rid, 'request_sha256': fingerprint,
                             'original_verified': original_rows[rid]['verified'],
                             'current_verified': current_rows[rid]['verified'],
                             'reason': 'original_audit_rejected' if not original_rows[rid]['verified'] else 'current_audit_rejected',
                             'original_error': original_rows[rid].get('error'),
                             'current_error': current_rows[rid].get('error')})
            continue
        folder = Path('retained') / fingerprint
        producer = inputs.json(folder / 'producer.json', 65536)
        camera = inputs.json(folder / 'camera_info.json', 65536)
        depth_meta = producer['depth']
        width = _integer(depth_meta['width'], 'depth_width', 1, 4096)
        height = _integer(depth_meta['height'], 'depth_height', 1, 4096)
        if depth_meta['encoding'] not in ('32FC1', '16UC1'):
            raise ValueError('unsupported_depth_encoding')
        decoded_bytes += width * height * (4 if depth_meta['encoding'] == '32FC1' else 2)
        if decoded_bytes > MAX_DECODED_BYTES:
            raise ValueError('decoded_byte_budget')
        depth, _ = _decode(producer, camera, request,
                           inputs.read(folder / 'depth.bin', MAX_IMAGE_BYTES))
        frames.append({'retention_id': rid, 'request_sha256': fingerprint, 'request': request,
                       'producer': producer, 'camera_info': camera, 'depth': depth,
                       'source_files': {name: str(inputs.root / folder / name) for name in sorted(FILES)}})
    hashes = inputs.finish()
    return {'mission': _mission_summary(mission), 'map_snapshot': map_snapshot,
            'frames': frames, 'excluded': excluded, 'audit_original': original,
            'audit_current': current, 'inputs_hashes': hashes}
