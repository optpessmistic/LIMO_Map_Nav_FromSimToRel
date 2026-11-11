"""Bounded writer stage diagnostics; real owned children, no ROS nodes."""
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import time
import unittest

REPO = Path(__file__).resolve().parents[1]
for root in (REPO/'src/limo_evidence_protocol', REPO/'src/limo_maze_perception', REPO/'tests'):
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

from limo_evidence_protocol.retention import Policy, canonical, sha
from limo_maze_perception.retention_adapter import (
    OwnedWriter, ProducerRetention, RuntimePolicy, STAGE_PREFIX, last_writer_stage,
)
from test_frame_retention_core import request, sample


def stage_line(stage, stamp):
    return STAGE_PREFIX + json.dumps({'stage': stage, 'monotonic_s': stamp}).encode() + b'\n'


class StageParserTests(unittest.TestCase):
    def test_only_complete_prefixed_known_records_are_selected(self):
        valid = stage_line('input_reading', 2.0)
        raw = (b'{"stage":"completed","monotonic_s":10}\n' +
               stage_line('script_entered', 1.0) + valid +
               stage_line('not_a_worker_stage', 3.0) +
               STAGE_PREFIX + b'not-json\n' +
               stage_line('completed', 4.0)[:-1])
        self.assertEqual(last_writer_stage(raw), {'stage': 'input_reading', 'monotonic_s': 2.0})

    def test_invalid_timestamps_and_shapes_cannot_replace_last_stage(self):
        raw = stage_line('protocol_imported', 2.0)
        for value in (True, -1, float('inf'), float('nan'), 1.0, '3.0'):
            raw += stage_line('completed', value)
        raw += STAGE_PREFIX + b'[]\n'
        raw += STAGE_PREFIX + b'{"stage":"completed","monotonic_s":3,"extra":1}\n'
        self.assertEqual(last_writer_stage(raw), {'stage': 'protocol_imported', 'monotonic_s': 2.0})

    def test_diagnostic_buffer_and_original_worker_deadline_remain_bounded(self):
        self.assertIsNone(last_writer_stage(b'x'*4096 + stage_line('completed', 1.0)))
        self.assertIsNone(last_writer_stage(b''))
        self.assertEqual(RuntimePolicy().worker_timeout_s, 5.0)


@unittest.skipUnless(hasattr(os, 'pidfd_open') and hasattr(signal, 'pidfd_send_signal'), 'Linux pidfd required')
class RealWorkerStageTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.owners = []
        self.writers = []
        self.addCleanup(self.cleanup)

    def cleanup(self):
        # Only exact children created by this fixture are touched.
        writers = self.writers + [owner.worker for owner in self.owners if owner.worker is not None]
        for writer in writers:
            if writer.process.poll() is None:
                writer.signal(signal.SIGKILL)
                writer.process.wait(timeout=2.0)
            writer.dispose()
        self.tmp.cleanup()

    def owner(self):
        owner = ProducerRetention(self.root/'run', 'session-a',
            policy=Policy(request_timeout_s=60.0), runtime=RuntimePolicy(worker_timeout_s=15.0))
        self.owners.append(owner)
        owner.on_epoch('epoch-a', True, time.monotonic())
        record, rgb, depth = sample()
        owner.offer(record, rgb, depth, time.monotonic())
        req = request(record)
        self.assertEqual(owner.request(req, time.monotonic())['status'], 'queued')
        return owner, req, bytes(rgb)

    def spin(self, owner, predicate):
        deadline = time.monotonic()+35.0
        while time.monotonic() < deadline:
            owner.tick(time.monotonic())
            if predicate():
                return
            time.sleep(.01)
        self.fail(owner.snapshot())

    def test_real_success_keeps_stdout_receipt_and_reports_all_stages(self):
        owner, req, rgb = self.owner()
        self.spin(owner, lambda: owner.worker is None and owner.revision == owner.persisted_revision)
        result = owner.results()[0]
        self.assertEqual(result['status'], 'stored')
        self.assertTrue(result['checkpoint_persisted'])
        self.assertIsNone(owner.fault)
        self.assertEqual((owner.output/result['receipt']['directory']/'rgb.bin').read_bytes(), rgb)
        writer = owner.writer_history[0]
        self.assertEqual(writer['last_stage']['stage'], 'completed')
        stages = [json.loads(line[len(STAGE_PREFIX):])['stage']
                  for line in writer['stderr'].encode().splitlines() if line.startswith(STAGE_PREFIX)]
        self.assertEqual(stages, ['script_entered', 'protocol_imported', 'root_validating',
            'root_validated', 'input_reading', 'input_read', 'checkpoint_writing',
            'checkpoint_written', 'artifact_writing', 'artifact_written', 'completed'])
        self.assertLessEqual(len(writer['stderr'].encode()), 4096)

    def test_real_artifact_write_failure_reports_write_stage_without_success(self):
        owner, req, _ = self.owner()
        (owner.output/sha(canonical(req))).mkdir()
        self.spin(owner, lambda: owner.worker is None and owner.closed and
                  owner.revision == owner.persisted_revision)
        self.assertEqual(owner.results()[0]['status'], 'failed')
        self.assertIn('FileExistsError', owner.fault)
        writer = owner.writer_history[0]
        self.assertEqual(writer['last_stage']['stage'], 'artifact_writing')
        self.assertEqual(writer['returncode'], 1)
        self.assertNotIn('"stage":"completed"', writer['stderr'])

    def test_stalled_child_stage_is_visible_and_timeout_still_closes_exact_child(self):
        fake = self.root/'stalled-stage.py'
        fake.write_text('import sys,time\n'
            'sys.stderr.write("LIMO_RETENTION_STAGE {\\"stage\\":\\"input_reading\\",'
            '\\"monotonic_s\\":%s}\\n" % time.monotonic())\n'
            'sys.stderr.flush()\ntime.sleep(30)\n')
        writer = OwnedWriter(self.root, canonical({}), None,
            RuntimePolicy(worker_timeout_s=1.5), time.monotonic(), worker_path=fake)
        self.writers.append(writer)
        deadline = time.monotonic()+5.0
        finished = False
        while time.monotonic() < deadline:
            finished = writer.poll(time.monotonic())
            if finished:
                break
            time.sleep(.01)
        self.assertTrue(finished, writer.snapshot())
        snapshot = writer.snapshot()
        self.assertEqual(snapshot['last_stage']['stage'], 'input_reading')
        self.assertEqual(snapshot['stop_reason'], 'writer_wall_timeout')
        self.assertEqual(snapshot['returncode'], -signal.SIGTERM)
        self.assertIsNone(writer.reply)
        self.assertFalse(snapshot['termination_unconfirmed'])


if __name__ == '__main__':
    unittest.main()
