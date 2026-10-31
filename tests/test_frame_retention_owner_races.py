"""Deterministic completion/commit race regressions, no ROS or child needed."""
from collections import deque
import json
import os
import signal
import sys
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

REPO = Path(__file__).resolve().parents[1]
SOURCE_ROOTS = [str(REPO / 'src/limo_evidence_protocol'), str(REPO / 'src/limo_maze_perception'), str(Path(__file__).resolve().parent)]
for root in SOURCE_ROOTS:
    if root not in sys.path:
        sys.path.insert(0, root)

from limo_evidence_protocol.retention import canonical, sha, write_job
from limo_maze_perception.retention_adapter import OwnedWriter, ProducerRetention, RuntimePolicy
from test_frame_retention_core import request, sample


class OwnerRaceTests(unittest.TestCase):
    def setUp(self):
        env = patch.dict(os.environ, {'PYTHONPATH': os.pathsep.join(SOURCE_ROOTS + [os.environ.get('PYTHONPATH', '')])})
        env.start()
        self.addCleanup(env.stop)

    def stub_writer(self, raw, *, arrives_at_poll=False):
        writer = OwnedWriter.__new__(OwnedWriter)
        writer.segments = deque()
        writer.stop_reason = writer.stop_started = writer.killed_at = writer.fault = None
        writer.setup_error = None
        writer.output = bytearray()
        writer.errors = bytearray()
        writer.policy = RuntimePolicy()
        writer.started = 0.
        writer.operation_id = 'operation'
        writer.reply = None
        writer.returncode = None
        buffered = bytearray() if arrives_at_poll else bytearray(raw)
        stdout, stderr = object(), object()
        def drain(stream, target, limit):
            if stream is stdout:
                target.extend(buffered)
                buffered.clear()
        def poll():
            if arrives_at_poll:
                buffered.extend(raw)
            return 0
        writer._drain = drain
        writer.process = S(stdin=S(closed=True), stdout=stdout, stderr=stderr, poll=poll)
        return writer

    def test_reply_written_between_first_drain_and_exit_poll_is_collected(self):
        raw = canonical(dict(ok=True, operation_id='operation', receipt_b64=None))
        writer = self.stub_writer(raw, arrives_at_poll=True)
        self.assertTrue(writer.poll(.1))
        self.assertIsNone(writer.fault)
        self.assertTrue(writer.reply['ok'])

    def test_valid_json_nonobject_completion_is_rejected_without_unwinding_owner(self):
        writer = self.stub_writer(b'[]')
        self.assertTrue(writer.poll(.1))
        self.assertEqual(writer.fault, 'writer_reply_missing_or_invalid')

    def test_global_fault_cannot_borrow_old_stored_checkpoint_flag(self):
        with tempfile.TemporaryDirectory() as tmp:
            owner = ProducerRetention(tmp+'/new', 'session-a')
            owner.on_epoch('epoch-a', True, time.monotonic())
            record, rgb, dep = sample()
            owner.offer(record, rgb, dep, time.monotonic())
            req = request(record)
            owner.request(req, time.monotonic())
            job = owner.core.take(time.monotonic())
            result = owner.core.finish(job, write_job(job, owner.output), time.monotonic())
            owner.durable_results[req['retention_id']] = sha(canonical(result))
            self.assertTrue(owner.transport_result(result)['checkpoint_persisted'])
            owner.close('epoch_invalid_or_changed', time.monotonic())
            final = owner.transport_result(result)
            self.assertEqual(final['status'], 'failed')
            self.assertFalse(final['checkpoint_persisted'])

    @unittest.skipUnless(hasattr(os, 'pidfd_open') and hasattr(signal, 'pidfd_send_signal'), 'Linux pidfd required')
    def test_closed_owner_new_fault_advances_checkpoint_revision(self):
        with tempfile.TemporaryDirectory() as tmp:
            owner = ProducerRetention(tmp+'/new', 'session-a')
            owner.on_epoch('epoch-a', True, time.monotonic())
            first = owner.shutdown()
            self.assertTrue(first['shutdown']['final_checkpoint_persisted'])
            self.assertIsNone(json.loads((owner.output/'checkpoint.json').read_bytes())['fault'])
            owner.close('ros_clock_reset', time.monotonic())
            self.assertGreater(owner.revision, owner.persisted_revision)
            final = owner.shutdown()
            self.assertTrue(final['shutdown']['final_checkpoint_persisted'])
            self.assertEqual(json.loads((owner.output/'checkpoint.json').read_bytes())['fault'], 'ros_clock_reset')


if __name__ == '__main__':
    unittest.main()
