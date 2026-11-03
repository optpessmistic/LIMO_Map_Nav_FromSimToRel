"""Bounded source-owned frame retention. Standard library only, no ROS or threads.

The owner serializes Core calls. A separate worker may write the immutable Job;
completion is routed back to the owner. This core never selects mission frames.
Payload accounting is a conservative byte bound, not a Python RSS guarantee.
"""
from collections import OrderedDict, deque
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path


def canonical(value, limit=65536):
    data = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()
    if len(data) > limit:
        raise ValueError('metadata_too_large')
    return data


def sha(data):
    return hashlib.sha256(data).hexdigest()


def text(value):
    if type(value) is not str or not value.strip() or value != value.strip() or len(value) > 256:
        raise ValueError('invalid_text')
    return value


def integer(value, minimum=0, maximum=2**63-1):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError('invalid_integer')
    return value


def digest(value):
    if type(value) is not str or len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
        raise ValueError('invalid_digest')
    return value


def sample_identity(record):
    if record.get('schema_version') != 1 or record.get('status') != 'processed' or record.get('reason') is not None:
        raise ValueError('not_processed_record')
    rgb, depth, camera = record['rgb'], record['depth'], record['camera_info']
    return {'map_epoch': text(record['map_epoch']),
            'detector_session_id': text(record['detector_session_id']),
            'sequence': integer(record['sequence'], 1),
            'source': text(record['source']),
            'detector_config_id': digest(record['detector_config_id']),
            'calibration_id': digest(camera['calibration_id']),
            'frame_id': text(rgb['frame_id']),
            'rgb_stamp_ns': integer(rgb['stamp_ns']), 'depth_stamp_ns': integer(depth['stamp_ns']),
            'rgb_sha256': digest(rgb['data_sha256']), 'depth_sha256': digest(depth['data_sha256'])}


def logical_key(identity):
    return identity['map_epoch'], identity['detector_session_id'], identity['sequence']


def validate_request(value):
    # Exact keys: no filename/path field is accepted from a request transport.
    required = {'schema_version', 'retention_id', 'map_epoch', 'goal_token', 'inspection_request_id',
                'after_stamp_ns', 'accepted_ordinal', 'sample', 'tf'}
    if type(value) is not dict or set(value) != required or value['schema_version'] != 1:
        raise ValueError('request_schema')
    encoded = canonical(value, 16384)
    value = json.loads(encoded)
    for key in ('retention_id', 'map_epoch', 'inspection_request_id'):
        text(value[key])
    integer(value['goal_token'], 1)
    integer(value['after_stamp_ns'])
    integer(value['accepted_ordinal'], 1, 32)
    sample = value['sample']
    if set(sample) != {'map_epoch', 'detector_session_id', 'sequence', 'source', 'detector_config_id',
                       'calibration_id', 'frame_id', 'rgb_stamp_ns', 'depth_stamp_ns', 'rgb_sha256', 'depth_sha256'}:
        raise ValueError('sample_schema')
    for key in ('map_epoch', 'detector_session_id', 'source', 'frame_id'):
        text(sample[key])
    for key in ('detector_config_id', 'calibration_id', 'rgb_sha256', 'depth_sha256'):
        digest(sample[key])
    integer(sample['sequence'], 1)
    for key in ('rgb_stamp_ns', 'depth_stamp_ns'):
        if integer(sample[key]) <= value['after_stamp_ns']:
            raise ValueError('sample_before_window')
    if value['map_epoch'] != sample['map_epoch']:
        raise ValueError('epoch_mismatch')
    tf = value['tf']
    if set(tf) != {'stamp_ns', 'base_pose', 'camera_pose'} or tf['stamp_ns'] != sample['rgb_stamp_ns']:
        raise ValueError('exact_sample_tf_required')
    integer(tf['stamp_ns'])
    for key, size in (('base_pose', 3), ('camera_pose', 7)):
        if (type(tf[key]) is not list or len(tf[key]) != size or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in tf[key])):
            raise ValueError('invalid_tf_pose')
    if abs(sum(v*v for v in tf['camera_pose'][3:])-1.) > 1e-3:
        raise ValueError('invalid_tf_quaternion')
    return value, encoded


@dataclass(frozen=True)
class Policy:
    max_candidates: int = 32
    max_payload_bytes: int = 80*1024*1024
    ttl_s: float = 15.
    max_pending: int = 8
    max_jobs: int = 32
    max_file_bytes: int = 96*1024*1024
    max_requests: int = 128
    request_timeout_s: float = 10.

    def __post_init__(self):
        for key, cap in (('max_candidates', 32), ('max_pending', 32), ('max_jobs', 1024),
                         ('max_requests', 4096), ('max_payload_bytes', 2**31), ('max_file_bytes', 2**40)):
            integer(getattr(self, key), 1, cap)
        for key in ('ttl_s', 'request_timeout_s'):
            value = getattr(self, key)
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 < value <= 3600:
                raise ValueError('invalid_duration')


@dataclass(frozen=True)
class Bundle:
    identity_json: bytes
    fingerprint: str
    files: tuple
    size: int

    @classmethod
    def prepare(cls, record, rgb, depth, max_bytes):
        """Validate views before reserving space; no new raw image allocation."""
        if not isinstance(rgb, (bytes, bytearray, memoryview)) or not isinstance(depth, (bytes, bytearray, memoryview)):
            raise ValueError('bytes_required')
        if memoryview(rgb).nbytes+memoryview(depth).nbytes > max_bytes:
            raise ValueError('bundle_over_budget')
        rgb, depth = memoryview(rgb).cast('B'), memoryview(depth).cast('B')
        producer = canonical(record)
        frozen = json.loads(producer)
        identity = sample_identity(frozen)
        for key, raw in (('rgb', rgb), ('depth', depth)):
            meta = frozen[key]
            if meta['data_bytes'] != len(raw) or meta['data_sha256'] != sha(raw):
                raise ValueError('source_bytes_mismatch')
            if integer(meta['height'], 1)*integer(meta['step'], 1) != len(raw):
                raise ValueError('image_layout_mismatch')
        camera = frozen['camera_info']
        if sha(canonical({'frame_id': camera['frame_id'], **camera['calibration']})) != identity['calibration_id']:
            raise ValueError('calibration_digest_mismatch')
        if sha(canonical(frozen['detector_config'])) != identity['detector_config_id']:
            raise ValueError('config_digest_mismatch')
        files = (('rgb.bin', rgb), ('depth.bin', depth), ('producer.json', producer),
                 ('camera_info.json', canonical(camera)))
        size = sum(len(data) for _, data in files)
        if size > max_bytes:
            raise ValueError('bundle_over_budget')
        return canonical(identity), sha(producer), files, size

    @classmethod
    def freeze(cls, record, rgb, depth, max_bytes):
        return cls.from_prepared(cls.prepare(record, rgb, depth, max_bytes))

    @classmethod
    def from_prepared(cls, prepared):
        identity, fingerprint, files, size = prepared
        immutable = tuple((name, bytes(data)) for name, data in files)
        expected = json.loads(identity)
        if sha(immutable[0][1]) != expected['rgb_sha256'] or sha(immutable[1][1]) != expected['depth_sha256']:
            raise ValueError('source_changed_while_freezing')
        return cls(identity, fingerprint, immutable, size)


@dataclass(frozen=True)
class Job:
    request_id: str
    request_sha256: str
    directory: str
    files: tuple
    manifest: bytes
    receipt: bytes
    file_bytes: int


def make_job(request, request_bytes, bundle, cached_wall, requested_wall):
    files = bundle.files+(('request.json', request_bytes),)
    inventory = {name: {'sha256': sha(data), 'bytes': len(data)} for name, data in files}
    manifest = canonical({'schema_version': 1, 'request_sha256': sha(request_bytes),
        'sample': request['sample'], 'cached_wall': cached_wall, 'requested_wall': requested_wall,
        'files': inventory, 'scope': 'source_callback_bytes_and_supplied_exact_tf_snapshot'})
    inventory['manifest.json'] = {'sha256': sha(manifest), 'bytes': len(manifest)}
    receipt = canonical({'request_sha256': sha(request_bytes), 'directory': sha(request_bytes),
                         'files': inventory, 'file_fsync_completed': True})
    return Job(request['retention_id'], sha(request_bytes), sha(request_bytes), files, manifest,
               receipt, sum(len(data) for _, data in files)+len(manifest))


def write_job(job, root):
    """Run in the future adapter's owned worker, never in a perception callback.

    root must be an exclusive, owner-created directory. All child names are
    digest/constants; partial failures remain for audit. No overwrite or deletion.
    fsync applies to files, not a promise of device power-loss durability.
    """
    root = Path(root)
    if not root.is_absolute() or not root.is_dir() or root.is_symlink() or root.resolve() != root:
        raise ValueError('owned_absolute_output_directory_required')
    destination = root/job.directory
    destination.mkdir(exist_ok=False)
    for name, data in job.files+(('manifest.json', job.manifest),):
        with (destination/name).open('xb') as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
    return job.receipt  # Only after the last file's flush+fsync succeeds.


class Core:
    def __init__(self, epoch, session, policy=Policy()):
        self.epoch, self.session, self.policy = text(epoch), text(session), policy
        self.cache, self.records, self.queue = OrderedDict(), {}, deque()
        self.last_wall = None
        self.last_sequence = 0
        self.closed = False
        # If a failure cannot itself be admitted within the original history /
        # payload bounds, permanently deny all as-yet-unrecorded identities.
        # This fixed-size control state does not retain arbitrary rejected IDs.
        self.admission_fault = None
        self.candidate_bytes = self.request_bytes = self.reserved_file_bytes = self.jobs_reserved = 0
        self.events = deque(maxlen=128)

    def _now(self, now):
        if type(now) not in (int, float) or not math.isfinite(now) or now < 0 or (
                self.last_wall is not None and now < self.last_wall):
            raise ValueError('invalid_monotonic_wall')
        self.last_wall = now

    def _event(self, reason, **fields):
        self.events.append({'wall': self.last_wall, 'reason': reason, **fields})

    def _remove(self, key, reason):
        entry = self.cache.pop(key)
        self.candidate_bytes -= entry['bundle'].size
        self._event(reason, sequence=key[2])

    def _release(self, record):
        entry = self.cache.get(record['key'])
        if entry is not None:
            entry['pins'] -= 1
        record['job'] = None

    def _fault(self, record, reason):
        record['fault'] = record['fault'] or reason
        if record['state'] == 'queued':
            record['state'] = 'failed'
            self._release(record)

    def expire(self, now):
        self._now(now)
        for record in self.records.values():
            if record['state'] in ('queued', 'writing') and now >= record['deadline']:
                self._fault(record, 'request_timeout')
        for key, entry in list(self.cache.items()):
            if not entry['pins'] and now-entry['wall'] >= self.policy.ttl_s:
                self._remove(key, 'ttl_expired')

    def offer(self, record, rgb, depth, now):
        self.expire(now)
        if self.closed:
            return {'available': False, 'reason': 'closed'}
        prepared = Bundle.prepare(record, rgb, depth, self.policy.max_payload_bytes)
        identity_json, fingerprint, _, size = prepared
        identity = json.loads(identity_json)
        key = logical_key(identity)
        if key[:2] != (self.epoch, self.session):
            return {'available': False, 'reason': 'wrong_epoch_or_session'}
        old = self.cache.get(key)
        known = [r for r in self.records.values() if r.get('key') == key and r.get('bundle_fingerprint')]
        conflict = ((old is not None and old['bundle'].fingerprint != fingerprint) or
                    any(r['bundle_fingerprint'] != fingerprint for r in known))
        if conflict:
            if old: old['conflict'] = True
            for r in known: self._fault(r, 'source_identity_conflict')
            self._event('source_identity_conflict', sequence=key[2])
            return {'available': False, 'reason': 'source_identity_conflict'}
        if old:
            return {'available': not old['conflict'], 'reason': 'duplicate' if not old['conflict'] else 'source_identity_conflict'}
        if identity['sequence'] <= self.last_sequence:
            return {'available': False, 'reason': 'nonadvancing_cache_input'}
        self.last_sequence = identity['sequence']
        while (len(self.cache) >= self.policy.max_candidates or
               self.candidate_bytes+self.request_bytes+size > self.policy.max_payload_bytes):
            victim = next((k for k, v in self.cache.items() if not v['pins']), None)
            if victim is None:
                return {'available': False, 'reason': 'cache_budget_pinned'}
            self._remove(victim, 'capacity_evicted')
        bundle = Bundle.from_prepared(prepared)  # Copy raw data only after room is reserved.
        self.cache[key] = {'bundle': bundle, 'wall': now, 'pins': 0, 'conflict': False}
        self.candidate_bytes += bundle.size
        return {'available': True, 'reason': 'cached', 'sample': identity}

    def request(self, payload, now):
        self.expire(now)
        request, raw = validate_request(payload)
        rid, fingerprint = request['retention_id'], sha(raw)
        if rid in self.records:
            r = self.records[rid]
            if r['request_sha256'] != fingerprint:
                self._fault(r, 'request_id_conflict')
            return self.result(rid)
        if self.admission_fault is None and (
                len(self.records) >= self.policy.max_requests or
                self.request_bytes+self.candidate_bytes+len(raw) > self.policy.max_payload_bytes):
            self.admission_fault = {'reason': 'request_history_budget', 'wall': now,
                                    'unrecorded_rejections': 0}
        if self.admission_fault is not None:
            # A TTL/capacity release must never resurrect a returned failure.
            # Known IDs were handled above, including their payload conflicts.
            # Unknown IDs have no reserved history space: fail closed globally,
            # instead of claiming to remember which unknown payload conflicted.
            self.admission_fault['unrecorded_rejections'] = min(
                self.admission_fault['unrecorded_rejections']+1, 2**63-1)
            return {'retention_id': rid, 'request_sha256': fingerprint, 'status': 'failed',
                    'reason': self.admission_fault['reason'], 'recorded': False, 'admission_closed': True}
        key = logical_key(request['sample'])
        r = {'request': raw, 'request_sha256': fingerprint, 'key': key, 'state': 'failed', 'fault': None,
             'deadline': now+self.policy.request_timeout_s, 'job': None, 'receipt': None,
             'bundle_fingerprint': None, 'requested_wall': now, 'completed_wall': None}
        self.records[rid] = r
        self.request_bytes += len(raw)
        slot_fields = ('map_epoch', 'goal_token', 'inspection_request_id', 'after_stamp_ns', 'accepted_ordinal')
        slot = tuple(request[field] for field in slot_fields)
        duplicates = [old for key_id, old in self.records.items() if key_id != rid and
                      tuple(json.loads(old['request'])[field] for field in slot_fields) == slot]
        if duplicates:
            r['fault'] = 'accepted_slot_conflict'
            for old in duplicates: self._fault(old, 'accepted_slot_conflict')
            return self.result(rid)
        entry = self.cache.get(key)
        pending = sum(v['state'] in ('queued', 'writing') for v in self.records.values())
        reason = ('closed' if self.closed else
                  'wrong_epoch_or_session' if key[:2] != (self.epoch, self.session) else
                  'cache_miss' if entry is None else
                  'source_identity_conflict' if entry['conflict'] else
                  'sample_identity_mismatch' if canonical(request['sample']) != entry['bundle'].identity_json else
                  'queue_full' if pending >= self.policy.max_pending else
                  'job_count_budget' if self.jobs_reserved >= self.policy.max_jobs else None)
        if reason:
            r['fault'] = reason
            return self.result(rid)
        job = make_job(request, raw, entry['bundle'], entry['wall'], now)
        charge = len(job.manifest)+len(job.receipt)
        if self.candidate_bytes+self.request_bytes+charge > self.policy.max_payload_bytes:
            r['fault'] = 'payload_budget'
        elif self.reserved_file_bytes+job.file_bytes > self.policy.max_file_bytes:
            r['fault'] = 'file_byte_budget'
        else:
            # Conservative reservations never replenish: partial files may exist.
            self.request_bytes += charge
            self.reserved_file_bytes += job.file_bytes
            self.jobs_reserved += 1
            entry['pins'] += 1
            r.update(state='queued', job=job, bundle_fingerprint=entry['bundle'].fingerprint)
            self.queue.append(rid)
        return self.result(rid)

    def take(self, now):
        self.expire(now)
        if any(r['state'] == 'writing' for r in self.records.values()):
            return None  # Exactly one owned worker job.
        while self.queue:
            rid = self.queue.popleft()
            r = self.records[rid]
            if r['state'] == 'queued':
                r['state'] = 'writing'
                return r['job']
        return None

    def finish(self, job, receipt, now):
        self.expire(now)
        r = self.records.get(job.request_id)
        if r is None or r['state'] != 'writing' or r['job'] is not job:
            raise ValueError('unowned_writer_completion')
        receipt_matches = isinstance(receipt, (bytes, bytearray, memoryview)) and receipt == job.receipt
        if not receipt_matches:
            r['fault'] = r['fault'] or 'write_failed_or_receipt_mismatch'
        # Store the owned immutable bytes already charged at admission. Never
        # retain a worker/mailbox buffer, even if its current content matches.
        r['receipt'] = job.receipt if receipt_matches else None
        r['state'] = 'failed' if r['fault'] else 'stored'
        r['completed_wall'] = now
        self._release(r)
        if self.closed and r['key'] in self.cache and not self.cache[r['key']]['pins']:
            self._remove(r['key'], 'closed_writer_released')
        return self.result(job.request_id)

    def result(self, rid):
        r = self.records[rid]
        return {'retention_id': rid, 'request_sha256': r['request_sha256'],
                'status': 'failed' if r['fault'] else r['state'], 'reason': r['fault'],
                'writer_owned': r['state'] == 'writing',
                'requested_wall': r['requested_wall'], 'completed_wall': r['completed_wall'],
                'receipt': json.loads(r['receipt']) if r['receipt'] else None}

    def close(self, now):
        self.expire(now)
        self.closed = True
        for r in self.records.values():
            if r['state'] in ('queued', 'writing'):
                self._fault(r, 'shutdown_aborted')
        for key, entry in list(self.cache.items()):
            if not entry['pins']:
                self._remove(key, 'closed_unpinned')
        return self.checkpoint()

    def checkpoint(self):
        return {'schema_version': 1, 'closed': self.closed, 'map_epoch': self.epoch,
                'detector_session_id': self.session, 'wall': self.last_wall,
                'candidate_count': len(self.cache), 'payload_bytes_reserved': self.candidate_bytes+self.request_bytes,
                'jobs_reserved': self.jobs_reserved, 'file_bytes_reserved': self.reserved_file_bytes,
                'writer_pending': any(r['state'] == 'writing' for r in self.records.values()),
                 'admission': {'closed': self.admission_fault is not None,
                               'fault': dict(self.admission_fault) if self.admission_fault else None,
                               'scope': 'all_unrecorded_ids_denied_after_history_budget; rejected_identities_not_retained'},
                'results': [self.result(rid) for rid in self.records], 'events': list(self.events),
                'scope': 'in_memory_checkpoint_requires_owner_persistence; no process cleanup claim'}


@dataclass(frozen=True)
class AcceptedWindow:
    """An exact frozen selected list, not a 'find any N successful frames' query."""
    requests: tuple

    @classmethod
    def freeze(cls, requests):
        if not 1 <= len(requests) <= 32:
            raise ValueError('accepted_list_limit')
        raw = tuple(validate_request(r)[1] for r in requests)
        decoded = [json.loads(r) for r in raw]
        fields = ('map_epoch', 'goal_token', 'inspection_request_id', 'after_stamp_ns')
        source_fields = ('detector_session_id', 'source', 'detector_config_id', 'calibration_id', 'frame_id')
        if (any(tuple(r[k] for k in fields) != tuple(decoded[0][k] for k in fields) for r in decoded) or
                any(tuple(r['sample'][k] for k in source_fields) != tuple(decoded[0]['sample'][k] for k in source_fields)
                    for r in decoded) or
                [r['accepted_ordinal'] for r in decoded] != list(range(1, len(raw)+1)) or
                len({r['retention_id'] for r in decoded}) != len(raw) or
                len({canonical(r['sample']) for r in decoded}) != len(raw)):
            raise ValueError('accepted_list_binding')
        for left, right in zip(decoded, decoded[1:]):
            if any(left['sample'][k] >= right['sample'][k] for k in ('sequence', 'rgb_stamp_ns', 'depth_stamp_ns')):
                raise ValueError('accepted_list_nonadvancing')
        return cls(raw)

    def evaluate(self, results):
        slots = []
        for raw in self.requests:
            request = json.loads(raw)
            matches = [r for r in results if r.get('retention_id') == request['retention_id']]
            result = matches[0] if len(matches) == 1 else None
            stored = bool(result and result.get('request_sha256') == sha(raw) and
                          result.get('status') == 'stored' and not result.get('writer_owned') and
                          result.get('receipt') and result['receipt'].get('request_sha256') == sha(raw) and
                          result['receipt'].get('file_fsync_completed') is True)
            slots.append({'retention_id': request['retention_id'], 'accepted_ordinal': request['accepted_ordinal'],
                          'stored': stored, 'result': result})
        return {'selected_count': len(slots), 'stored_count': sum(s['stored'] for s in slots),
                'retention_satisfied': all(s['stored'] for s in slots), 'slots': slots,
                'scope': 'receipt_identity_only; offline artifacts must still be rehashed'}
