"""Pure mission-side exact accepted-frame retention; single serialized owner.

Uses the explicitly declared shared limo_evidence_protocol package. No ROS,
threads, image data, filesystem writes, or independent frame selection here.
"""
from dataclasses import dataclass
import json
import math

from limo_evidence_protocol.retention import AcceptedWindow, canonical, digest, integer, sample_identity, sha, text, validate_request
from limo_maze_mission.inspection import EvidenceError, validate_record

REQUEST_LIMIT = RESULT_LIMIT = 16384
# JSON wrapper, IDs, digests and status fields also consume bounded space.
# This accounts serialized metadata; it is not a bound on Python process RSS.
SLOT_RESERVATION = REQUEST_LIMIT + RESULT_LIMIT + 2048
WINDOW_RESERVATION = 4096


@dataclass(frozen=True)
class Policy:
    enabled: bool = False
    max_samples_per_window: int = 32
    max_total_samples: int = 32
    max_windows: int = 64
    max_metadata_bytes: int = 2*1024*1024

    def __post_init__(self):
        if type(self.enabled) is not bool:
            raise ValueError('enabled_requires_bool')
        for key, cap in (('max_samples_per_window', 32), ('max_total_samples', 4096),
                         ('max_windows', 256), ('max_metadata_bytes', 2**28)):
            integer(getattr(self, key), 1, cap)


class Consumer:
    def __init__(self, consumer_session, required_frames, publish, policy=Policy()):
        self.session, self.policy = text(consumer_session), policy
        integer(required_frames, 1, 1000)
        if policy.enabled and required_frames > policy.max_samples_per_window:
            raise ValueError('retained_inspection_required_frames_exceeds_window_limit')
        self.required_frames, self.publish = required_frames, publish
        self.windows, self.slots, self.current = [], {}, None
        self.last_wall, self.fault = None, None
        self.reserved_bytes = 0
        self.ignored_results = 0

    def _now(self, now):
        if type(now) not in (int, float) or not math.isfinite(now) or now < 0 or (
                self.last_wall is not None and now < self.last_wall):
            self.invalidate('invalid_monotonic_wall')
            raise ValueError('invalid_monotonic_wall')
        self.last_wall = now

    def _fail(self, window, reason, fatal=True):
        if window is not None:
            window['state'], window['reason'] = 'failed', window['reason'] or reason
        if fatal:
            self.fault = self.fault or reason

    @staticmethod
    def binding(ledger, goal_token):
        integer(goal_token, 1)
        return {'map_epoch': text(ledger.epoch), 'goal_token': goal_token,
                'inspection_request_id': text(ledger.request_id),
                'after_stamp_ns': integer(ledger.after_stamp_ns)}

    def begin_window(self, ledger, goal_token, now):
        if not self.policy.enabled:
            return False
        self._now(now)
        if self.fault:
            return False
        if self.current is not None and self.current['state'] not in ('retired', 'failed', 'complete'):
            self._fail(self.current, 'previous_window_not_retired')
            return False
        if (not ledger.ready or ledger.status != 'waiting' or ledger.accepted_frames != 0 or
                ledger.policy.required_frames != self.required_frames or now >= ledger.deadline_wall):
            self._fail(None, 'new_window_requires_fresh_ready_ledger')
            return False
        binding = self.binding(ledger, goal_token)
        if any(w['binding'] == binding and w['ids'] for w in self.windows):
            # The producer slot schema has no reset counter. Nonempty reset
            # windows must advance the real barrier or change goal/request.
            self._fail(None, 'nonempty_window_binding_reused')
            return False
        if (len(self.windows) >= self.policy.max_windows or
                self.reserved_bytes+WINDOW_RESERVATION > self.policy.max_metadata_bytes):
            self._fail(None, 'retention_window_budget')
            return False
        window = {'number': len(self.windows)+1, 'binding': binding, 'deadline_wall': ledger.deadline_wall,
                  'state': 'collecting', 'reason': None, 'ids': [], 'processing_satisfied': False,
                  'ledger_accepted_count': 0, 'created_wall': now, 'closed_wall': None,
                  'late_results': 0, 'ledger_reset_count': ledger.reset_count}
        self.windows.append(window)
        self.current = window
        self.reserved_bytes += WINDOW_RESERVATION
        return True

    def observe(self, ledger, record, *, now_ns, wall_now, base_pose, camera_pose, tf_stamp_ns, goal_token):
        """Own the actual Ledger.observe call; freeze ONLY its accepted events.

        Build/validate immutable request metadata and reserve capacity BEFORE
        allowing another Ledger acceptance. Publish immediately after acceptance.
        A failed publisher never erases the accepted sample or decrements Ledger.
        """
        kwargs = dict(now_ns=now_ns, wall_now=wall_now, base_pose=base_pose,
                      camera_pose=camera_pose, tf_stamp_ns=tf_stamp_ns)
        if not self.policy.enabled:
            return ledger.observe(record, **kwargs)
        self.tick(wall_now)
        w = self.current
        if self.fault or w is None or w['state'] != 'collecting':
            return {'accepted': False, 'reason': 'retention_not_collecting', 'status': ledger.status,
                    'consumer_fault': self.fault}
        if (not ledger.ready or self.binding(ledger, goal_token) != w['binding'] or
                ledger.reset_count != w['ledger_reset_count'] or
                ledger.accepted_frames != len(w['ids']) or ledger.deadline_wall != w['deadline_wall']):
            self._fail(w, 'ledger_window_binding_changed')
            return {'accepted': False, 'reason': self.fault, 'status': ledger.status, 'consumer_fault': self.fault}
        try:
            sample = validate_record(record, now_ns, ledger.policy)
        except EvidenceError:
            return self._ledger_observe(ledger, record, kwargs, w, wall_now)
        if (sample.epoch != ledger.epoch or
                min(sample.rgb_stamp_ns, sample.depth_stamp_ns) <= max(ledger.after_stamp_ns, ledger.epoch_barrier_ns)):
            return self._ledger_observe(ledger, record, kwargs, w, wall_now)
        if (len(w['ids']) >= self.policy.max_samples_per_window or
                len(self.slots) >= self.policy.max_total_samples or
                self.reserved_bytes+SLOT_RESERVATION > self.policy.max_metadata_bytes):
            self._fail(w, 'retention_sample_budget')
            return {'accepted': False, 'reason': self.fault, 'status': ledger.status, 'consumer_fault': self.fault}
        try:
            # Detached metadata and exact TF given to Ledger are the same data
            # used by the prepared request; there is no latest-TF lookup here.
            frozen_record = json.loads(canonical(record, 65536))
            sample = sample_identity(frozen_record)
            payload = {'schema_version': 1, 'retention_id': 'pending', **w['binding'],
                       'accepted_ordinal': len(w['ids'])+1, 'sample': sample,
                       'tf': {'stamp_ns': tf_stamp_ns, 'base_pose': list(base_pose),
                              'camera_pose': list(camera_pose)}}
            payload['retention_id'] = 'retain-'+sha(canonical(
                {'consumer_session': self.session, 'window_number': w['number'], 'request': payload}))
            payload, raw = validate_request(payload)
        except (ValueError, TypeError, KeyError, RecursionError):
            self._fail(w, 'retention_metadata_unrepresentable')
            return {'accepted': False, 'reason': self.fault, 'status': ledger.status, 'consumer_fault': self.fault}
        kwargs.update(base_pose=tuple(payload['tf']['base_pose']), camera_pose=tuple(payload['tf']['camera_pose']),
                      tf_stamp_ns=payload['tf']['stamp_ns'])
        decision = self._ledger_observe(ledger, frozen_record, kwargs, w, wall_now)
        if not decision['accepted']:
            return decision
        # No transport call occurs until the exact accepted slot is frozen.
        rid = payload['retention_id']
        self.slots[rid] = {'request': raw, 'sha256': sha(raw), 'window': w['number'], 'state': 'pending',
                           'reason': None, 'result': None, 'stored_receipt_sha256': None}
        w['ids'].append(rid)
        w['ledger_accepted_count'] = ledger.accepted_frames
        self.reserved_bytes += SLOT_RESERVATION
        if ledger.accepted_frames != len(w['ids']):
            self._fail(w, 'accepted_count_binding_failed')
        if not self.fault and ledger.status == 'satisfied':
            w['processing_satisfied'], w['state'] = True, 'awaiting_receipts'
        elif len(w['ids']) >= self.policy.max_samples_per_window:
            # Actual acceptance reached the limit before the original span was
            # satisfied: retain ALL accepted samples and fail, never truncate.
            self._fail(w, 'accepted_window_limit_before_processing_satisfied')
        try:
            self.publish(raw.decode('utf-8'))
        except Exception:
            self._fail(w, 'retention_request_publish_failed')
        self._maybe_complete(w, wall_now)
        return {**decision, 'retention_id': rid, 'consumer_fault': self.fault}

    def _ledger_observe(self, ledger, record, kwargs, window, now):
        decision = ledger.observe(record, **kwargs)
        if not ledger.ready:
            self.invalidate('ledger_invalidated')
        elif ledger.reset_count != window['ledger_reset_count']:
            self.retire('ledger_reset', now)
        return decision

    def _maybe_complete(self, w, now):
        if w['state'] != 'awaiting_receipts' or now >= w['deadline_wall']:
            return
        frozen = AcceptedWindow.freeze([json.loads(self.slots[rid]['request']) for rid in w['ids']])
        results = [json.loads(self.slots[rid]['result']) for rid in w['ids'] if self.slots[rid]['result']]
        if (all(self.slots[rid]['state'] == 'stored' for rid in w['ids']) and
                frozen.evaluate(results)['retention_satisfied']):
            w['state'], w['closed_wall'] = 'complete', now

    def on_result(self, value, now):
        if not self.policy.enabled:
            return {'matched': False, 'reason': 'disabled'}
        self.tick(now)
        try:
            raw = canonical(value, RESULT_LIMIT)
            result = json.loads(raw)
            rid = text(result['retention_id'])
            fingerprint = digest(result['request_sha256'])
        except (ValueError, TypeError, KeyError, RecursionError):
            self.ignored_results = min(self.ignored_results+1, 2**63-1)
            return {'matched': False, 'reason': 'invalid_result'}
        slot = self.slots.get(rid)
        if slot is None:
            self.ignored_results = min(self.ignored_results+1, 2**63-1)
            return {'matched': False, 'reason': 'unknown_retention_id'}
        w = self.windows[slot['window']-1]
        is_current = w is self.current
        if fingerprint != slot['sha256']:
            self._fail(w, 'retention_result_identity_conflict', fatal=is_current or w['state'] == 'complete')
            return {'matched': False, 'reason': 'request_sha256_mismatch', 'window': w['number']}
        if w['state'] in ('retired', 'failed'):
            w['late_results'] = min(w['late_results']+1, 2**63-1)
            return {'matched': True, 'credited': False, 'reason': 'closed_window', 'window': w['number']}
        status = result.get('status')
        if status == 'failed':
            slot.update(state='failed', reason='producer_reported_failed', result=raw)
            self._fail(w, 'producer_reported_failed', fatal=is_current or w['state'] == 'complete')
        elif status == 'stored' and result.get('checkpoint_persisted') is True:
            try:
                receipt = result['receipt']
                receipt_hash = sha(canonical(receipt, RESULT_LIMIT))
                # Keep the ORIGINAL request/ordinal hash; do not manufacture a
                # one-slot request from a later accepted ordinal.
                valid = (result.get('writer_owned') is False and type(receipt) is dict and
                         receipt.get('request_sha256') == slot['sha256'] and
                         receipt.get('directory') == slot['sha256'] and receipt.get('file_fsync_completed') is True)
            except (ValueError, TypeError, KeyError):
                valid, receipt_hash = False, None
            if not valid or (slot['stored_receipt_sha256'] is not None and
                             slot['stored_receipt_sha256'] != receipt_hash):
                self._fail(w, 'invalid_or_conflicting_stored_receipt', fatal=is_current or w['state'] == 'complete')
            else:
                slot.update(state='stored', result=raw, stored_receipt_sha256=receipt_hash)
                self._maybe_complete(w, now)
        elif status not in ('queued', 'writing', 'checkpoint_pending', 'stored'):
            self._fail(w, 'invalid_result_status', fatal=is_current or w['state'] == 'complete')
        # Pending/older queued messages never downgrade a credited stored slot.
        return {'matched': True, 'window': w['number'], 'current': is_current,
                'credited': slot['state'] == 'stored', 'consumer_fault': self.fault}

    def tick(self, now):
        if not self.policy.enabled:
            return
        self._now(now)
        w = self.current
        if w is not None and w['state'] in ('collecting', 'awaiting_receipts') and now >= w['deadline_wall']:
            # Open processing timeout remains the mission's existing decision;
            # sealed-but-unretained timeout is an additional retention failure.
            self._fail(w, 'retention_window_deadline', fatal=w['processing_satisfied'])

    def retire(self, reason, now):
        if not self.policy.enabled:
            return
        self._now(now)
        if self.current is not None and self.current['state'] not in ('failed', 'complete'):
            self.current.update(state='retired', reason=text(reason), closed_wall=now)

    def invalidate(self, reason):
        if self.policy.enabled:
            self._fail(self.current, text(reason))

    def current_satisfied(self, ledger, goal_token, now):
        if not self.policy.enabled:
            return True
        self.tick(now)
        w = self.current
        return bool(not self.fault and w and w['state'] == 'complete' and ledger.ready and
                    now < w['deadline_wall'] and
                    ledger.status == 'satisfied' and self.binding(ledger, goal_token) == w['binding'] and
                    ledger.accepted_frames == len(w['ids']) == w['ledger_accepted_count'])

    def snapshot(self):
        # Detached, bounded audit. This is memory state, not a persisted checkpoint.
        windows = []
        for w in self.windows:
            windows.append({**w, 'accepted': [{
                'request': json.loads(self.slots[rid]['request']), 'request_sha256': self.slots[rid]['sha256'],
                'state': self.slots[rid]['state'], 'reason': self.slots[rid]['reason'],
                'result': json.loads(self.slots[rid]['result']) if self.slots[rid]['result'] else None}
                for rid in w['ids']]})
        return json.loads(canonical({'enabled': self.policy.enabled, 'consumer_session': self.session,
            'current_window': self.current['number'] if self.current else None, 'fault': self.fault,
            'reserved_metadata_bytes': self.reserved_bytes, 'ignored_results': self.ignored_results,
            'windows': windows, 'scope': 'exact_accepted_metadata_and_receipts; offline_artifacts_not_audited'},
            self.policy.max_metadata_bytes+65536))

    def summary(self):
        """Small status only; the full fixed lists belong in the final audit."""
        w = self.current
        return {'enabled': self.policy.enabled, 'fault': self.fault,
                'current_window': w['number'] if w else None,
                'state': w['state'] if w else 'idle',
                'selected_count': len(w['ids']) if w else 0,
                'stored_count': sum(self.slots[rid]['state'] == 'stored' for rid in w['ids']) if w else 0,
                'total_selected_count': len(self.slots), 'window_count': len(self.windows),
                'offline_artifacts_verified': False}
