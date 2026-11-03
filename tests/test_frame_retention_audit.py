"""Offline rehash checks against real temporary artifacts; no ROS claims."""
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from audit_frame_retention import audit
from test_frame_retention_core import sample, request
from limo_evidence_protocol.retention import Core, canonical, sha, write_job


class AuditTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name).resolve()
        core = Core('epoch-a', 'session-a')
        self.accepted = []
        for i in range(1, 4):
            record, rgb, depth = sample(i)
            core.offer(record, rgb, depth, float(i))
            req = request(record)
            core.request(req, float(i))
            job = core.take(float(i))
            result = core.finish(job, write_job(job, self.root), float(i))
            self.accepted.append(dict(request=req, request_sha256=sha(canonical(req)), state='stored',
                                      result={**result, 'checkpoint_persisted': True}))
        core.close(4.)
        self.checkpoint = dict(closed=True, fault=None, checkpoint_failed=False, writer=None, core=core.checkpoint())
        self.save_checkpoint()
        binding = {k: self.accepted[0]['request'][k] for k in
                   ('map_epoch', 'goal_token', 'inspection_request_id', 'after_stamp_ns')}
        self.consumer = dict(enabled=True, fault=None, windows=[dict(accepted=self.accepted, ledger_accepted_count=3,
            ids=[e['request']['retention_id'] for e in self.accepted], binding=binding)])

    def save_checkpoint(self):
        (self.root / 'checkpoint.json').write_bytes(canonical(self.checkpoint))

    def test_three_exact_files_pass(self):
        result = audit(self.consumer, self.root)
        self.assertTrue(result['passed'], result)
        self.assertEqual(result['verified_count'], 3)

    def test_missing_middle_is_not_substituted(self):
        (self.root / self.accepted[1]['request_sha256'] / 'rgb.bin').unlink()
        result = audit(self.consumer, self.root)
        self.assertFalse(result['passed'])
        self.assertEqual(result['verified_count'], 2)
        self.assertFalse(result['frames'][1]['verified'])

    def test_padding_corruption_detected(self):
        path = self.root / self.accepted[0]['request_sha256'] / 'rgb.bin'
        raw = bytearray(path.read_bytes())
        raw[-1] ^= 1
        path.write_bytes(raw)
        self.assertIn('file_hash:rgb.bin', audit(self.consumer, self.root)['frames'][0]['error'])

    def test_late_source_failure_revokes_earlier_receipt(self):
        self.checkpoint['core']['results'][1].update(status='failed', reason='source_identity_conflict')
        self.save_checkpoint()
        self.assertFalse(audit(self.consumer, self.root)['passed'])

    def test_running_checkpoint_rejected(self):
        self.checkpoint['closed'] = False
        self.save_checkpoint()
        self.assertFalse(audit(self.consumer, self.root)['passed'])

    def test_modified_supplied_tf_rejected(self):
        self.accepted[1]['request']['tf']['base_pose'][0] += .01
        self.assertFalse(audit(self.consumer, self.root)['passed'])

    def test_dropped_accepted_entry_cannot_silently_pass(self):
        self.consumer['windows'][0]['accepted'] = self.accepted[:2]
        self.consumer['windows'][0]['ledger_accepted_count'] = 2
        self.consumer['windows'][0]['ids'] = self.consumer['windows'][0]['ids'][:2]
        self.assertFalse(audit(self.consumer, self.root)['passed'])

    def test_no_frames_does_not_pass_vacuously(self):
        self.consumer['windows'] = []
        self.assertFalse(audit(self.consumer, self.root)['passed'])

    def test_unconfirmed_consumer_receipt_rejected(self):
        self.accepted[1]['result']['checkpoint_persisted'] = False
        self.assertFalse(audit(self.consumer, self.root)['passed'])

    def test_symlink_to_other_frame_is_rejected(self):
        path = self.root / self.accepted[1]['request_sha256'] / 'rgb.bin'
        path.unlink()
        path.symlink_to(self.root / self.accepted[0]['request_sha256'] / 'rgb.bin')
        self.assertFalse(audit(self.consumer, self.root)['passed'])

    def test_wrong_consumer_receipt_identity_or_owned_writer_rejected(self):
        for key, value in (('retention_id', 'other'), ('request_sha256', 'b'*64), ('writer_owned', True)):
            with self.subTest(key=key):
                consumer = copy.deepcopy(self.consumer)
                consumer['windows'][0]['accepted'][1]['result'][key] = value
                self.assertFalse(audit(consumer, self.root)['passed'])

    def test_empty_window_binding_rejected(self):
        self.consumer['windows'][0]['binding'] = {}
        self.assertFalse(audit(self.consumer, self.root)['passed'])

    def test_nonobject_or_disabled_result_returns_failure(self):
        for value in (None, [], {'frame_retention': None}):
            with self.subTest(value=value):
                self.assertFalse(audit(value, self.root)['passed'])


if __name__ == '__main__':
    unittest.main()
