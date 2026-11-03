"""Regression tests for the independent review; pure core and temporary files."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/limo_evidence_protocol'))
from limo_evidence_protocol.retention import Bundle, Core, Policy, canonical, sha, write_job
from test_frame_retention_core import sample, request


class FinalRepairTests(unittest.TestCase):
    def failed_admission(self):
        old, rgb_old, depth_old = sample(1)
        rgb_old = bytearray(40000)
        old['rgb'].update(step=len(rgb_old), data_bytes=len(rgb_old), data_sha256=sha(rgb_old))
        fresh, rgb_fresh, depth_fresh = sample(2)
        req = request(fresh, ordinal=1)
        old_size = Bundle.freeze(old, rgb_old, depth_old, 100000).size
        fresh_size = Bundle.freeze(fresh, rgb_fresh, depth_fresh, 100000).size
        budget = old_size + fresh_size + len(canonical(req)) - 1
        core = Core('epoch-a', 'session-a', Policy(ttl_s=1., max_payload_bytes=budget))
        self.assertTrue(core.offer(old, rgb_old, depth_old, 0.)['available'])
        self.assertTrue(core.offer(fresh, rgb_fresh, depth_fresh, .9)['available'])
        first = core.request(req, .9)
        self.assertEqual(first['reason'], 'request_history_budget')
        self.assertFalse(first['recorded'])
        self.assertTrue(first['admission_closed'])
        self.assertEqual(core.records, {})
        return core, req, first

    def test_same_payload_budget_failure_never_resurrects_after_unrelated_ttl_release(self):
        core, req, first = self.failed_admission()
        second = core.request(req, 1.01)
        self.assertEqual(first, second)
        self.assertEqual(len(core.cache), 1)
        self.assertEqual(core.records, {})
        self.assertEqual(core.jobs_reserved, 0)
        self.assertIsNone(core.take(1.01))
        snapshot = core.checkpoint()
        self.assertTrue(snapshot['admission']['closed'])
        self.assertEqual(snapshot['admission']['fault']['unrecorded_rejections'], 2)
        self.assertLessEqual(snapshot['payload_bytes_reserved'], core.policy.max_payload_bytes)

    def test_changed_payload_under_unrecorded_id_cannot_queue_after_budget_release(self):
        core, req, first = self.failed_admission()
        changed = copy.deepcopy(req)
        changed['tf']['base_pose'][0] += .1
        second = core.request(changed, 1.01)
        self.assertEqual(second['status'], 'failed')
        self.assertNotEqual(first['request_sha256'], second['request_sha256'])
        self.assertFalse(second['recorded'])
        self.assertEqual(second['reason'], 'request_history_budget')
        # No invented per-ID conflict classification: this identity was never
        # admitted, and every unadmitted identity is denied after the budget fault.
        new_id = copy.deepcopy(req)
        new_id['retention_id'] = 'new-id-same-accepted-slot'
        self.assertEqual(core.request(new_id, 1.02)['status'], 'failed')
        self.assertEqual(core.records, {})
        self.assertIsNone(core.take(1.02))
        with self.assertRaises(KeyError):
            core.result(req['retention_id'])

    def test_finish_owns_immutable_receipt_when_caller_mutates_or_expands_its_buffer(self):
        for as_view in (False, True):
            with self.subTest(memoryview=as_view), tempfile.TemporaryDirectory() as root:
                core = Core('epoch-a', 'session-a', Policy(max_payload_bytes=10000))
                record, rgb, depth = sample()
                self.assertTrue(core.offer(record, rgb, depth, 0.)['available'])
                req = request(record)
                self.assertEqual(core.request(req, 0.)['status'], 'queued')
                job = core.take(0.)
                buffer = bytearray(write_job(job, Path(root).resolve()))
                receipt = memoryview(buffer) if as_view else buffer
                before = core.finish(job, receipt, .1)
                self.assertEqual(before['status'], 'stored')
                reserved = core.checkpoint()['payload_bytes_reserved']
                changed = json.loads(buffer)
                changed['files']['rgb.bin']['sha256'] = '0' * 64
                if as_view:
                    encoded = canonical(changed)
                    self.assertEqual(len(encoded), len(buffer))
                    buffer[:] = encoded
                else:
                    changed['external_mutation'] = 'x' * 20000
                    buffer[:] = canonical(changed)
                    self.assertGreater(len(buffer), core.policy.max_payload_bytes)
                self.assertEqual(core.result(req['retention_id']), before)
                self.assertIs(core.records[req['retention_id']]['receipt'], job.receipt)
                self.assertIsInstance(core.records[req['retention_id']]['receipt'], bytes)
                self.assertEqual(core.checkpoint()['payload_bytes_reserved'], reserved)
                self.assertLessEqual(reserved, core.policy.max_payload_bytes)
                self.assertEqual(sha((Path(root)/job.directory/'rgb.bin').read_bytes()),
                                 before['receipt']['files']['rgb.bin']['sha256'])

    def test_admission_fault_preserves_existing_writer_and_known_id_conflict_rules(self):
        core = Core('epoch-a', 'session-a', Policy(max_requests=1))
        record, rgb, depth = sample()
        self.assertTrue(core.offer(record, rgb, depth, 0.)['available'])
        req = request(record)
        core.request(req, 0.)
        job = core.take(0.)
        unknown = request(sample(2)[0])
        self.assertEqual(core.request(unknown, .1)['reason'], 'request_history_budget')
        self.assertTrue(core.checkpoint()['writer_pending'])
        self.assertEqual(core.request(req, .1)['status'], 'writing')
        with tempfile.TemporaryDirectory() as root:
            result = core.finish(job, write_job(job, Path(root).resolve()), .2)
        self.assertEqual(result['status'], 'stored')
        self.assertEqual(core.request(req, .3), result)
        changed = copy.deepcopy(req)
        changed['tf']['base_pose'][0] += .1
        self.assertEqual(core.request(changed, .3)['reason'], 'request_id_conflict')
        self.assertEqual(core.request(req, .3)['status'], 'failed')
        self.assertEqual(core.request(unknown, .3)['status'], 'failed')
        self.assertEqual(len(core.records), 1)
        self.assertEqual(core.jobs_reserved, 1)

    def test_unrecorded_rejection_flood_uses_fixed_control_state_without_history_or_jobs(self):
        core, req, _ = self.failed_admission()
        state_keys = set(core.admission_fault)
        for index in range(1000):
            changed = copy.deepcopy(req)
            changed['retention_id'] = 'rejected-' + str(index)
            result = core.request(changed, 2.)
            self.assertEqual(result['status'], 'failed')
            self.assertFalse(result['recorded'])
        snapshot = core.checkpoint()
        self.assertEqual(set(core.admission_fault), state_keys)
        self.assertEqual(len(core.admission_fault), 3)
        self.assertEqual(core.admission_fault['unrecorded_rejections'], 1001)
        self.assertEqual(core.records, {})
        self.assertEqual(snapshot['results'], [])
        self.assertEqual(core.jobs_reserved, 0)
        self.assertEqual(core.reserved_file_bytes, 0)
        self.assertEqual(snapshot['payload_bytes_reserved'], 0)
        snapshot['admission']['fault']['reason'] = 'external-change'
        self.assertEqual(core.admission_fault['reason'], 'request_history_budget')


if __name__ == '__main__':
    unittest.main(verbosity=2)
