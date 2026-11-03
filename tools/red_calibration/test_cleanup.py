"""Offline lifecycle tests; child processes load no ROS or Gazebo runtime."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import capture_core as c
import capture_red_samples as runtime


def sample_report(complete=True):
    return {'schema_version': 2, 'sampling_complete': complete,
            'collection_complete': complete, 'sampling_passed': complete,
            'sampling_elapsed_wall_s': .1, 'errors': [],
            'samples': [{'sequence': 1, 'artifact': 'sample_001/sample.json'}],
            'saved_pairs': 1, 'requested_pairs': 1 if complete else 2}


class CleanupTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='cleanup-test-', dir=Path(__file__).parent)
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)

    def test_every_saved_artifact_and_raw_checkpoint_fsync_precedes_cleanup(self):
        writer = c.CaptureWriter(self.root/'capture')
        synced = []
        actual_fsync = os.fsync
        def fsync(fd):
            actual_fsync(fd)
            synced.append(True)
        def cleanup():
            raw = json.loads((writer.root/'raw-checkpoint.json').read_text())
            self.assertEqual(len(synced), 2)
            self.assertFalse(raw['cleanup_complete'])
            self.assertFalse(raw['passed'])
            self.assertTrue(raw['partial'])
            self.assertTrue(raw['sampling_complete'])
            self.assertEqual(raw['artifacts_saved_before_cleanup']['sample_001/sample.json']['sha256'],
                             hashlib.sha256(b'{}').hexdigest())
            self.assertFalse((writer.root/'summary.json').exists())
        with patch.object(c.os, 'fsync', side_effect=fsync):
            writer.write('sample_001/sample.json', b'{}')
            result = runtime.checkpoint_then_cleanup(writer, sample_report(), [('test', cleanup)], runtime.StopRequest())
        self.assertEqual(len(synced), 3)
        self.assertTrue(result['passed'])
        self.assertTrue(result['cleanup_complete'])
        self.assertFalse(result['partial'])
        self.assertEqual(result['raw_checkpoint']['sha256'],
                         hashlib.sha256((writer.root/'raw-checkpoint.json').read_bytes()).hexdigest())

    def test_python_cleanup_error_retains_sampling_outcome_and_continues(self):
        writer = c.CaptureWriter(self.root/'capture')
        called = []
        def fail(): raise RuntimeError('native wrapper failed')
        result = runtime.checkpoint_then_cleanup(writer, sample_report(),
            [('listener', fail), ('node', lambda: called.append(True))], runtime.StopRequest())
        self.assertTrue(called)
        self.assertTrue(result['sampling_complete'])
        self.assertFalse(result['cleanup_complete'])
        self.assertFalse(result['passed'])
        self.assertTrue(result['partial'])
        self.assertIn('listener:RuntimeError: native wrapper failed', result['errors'])

    def test_cleanup_duration_does_not_rewrite_sampling_completion(self):
        writer = c.CaptureWriter(self.root/'capture')
        times = iter([10., 150.])
        result = runtime.checkpoint_then_cleanup(writer, sample_report(), [('cleanup', lambda: None)],
            runtime.StopRequest(), monotonic=lambda: next(times))
        self.assertEqual(result['cleanup']['elapsed_wall_s'], 140.)
        self.assertEqual(result['sampling_elapsed_wall_s'], .1)
        self.assertTrue(result['sampling_complete'])

    def test_report_slots_and_immutable_checkpoint(self):
        writer = c.CaptureWriter(self.root/'capture')
        writer.write_report('raw-checkpoint.json', {'partial': True})
        with self.assertRaises(FileExistsError):
            writer.write_report('raw-checkpoint.json', {'partial': False})
        with self.assertRaisesRegex(OSError, 'slot_exceeded'):
            writer.write_report('too-big.json', {'value': 'x'*writer.REPORT_SLOT_BYTES})

    def child(self, code, output):
        result = subprocess.run([sys.executable, '-B', '-c', code, str(output)],
                                cwd=Path(__file__).parent, capture_output=True, text=True, timeout=10)
        return result

    def test_uncatchable_cleanup_exit_leaves_raw_checkpoint_without_false_success(self):
        output = self.root/'aborted'
        result = self.child('''
import os, sys
from pathlib import Path
import capture_core as c
import capture_red_samples as runtime
from test_cleanup import sample_report
assert 'rclpy' not in sys.modules
w = c.CaptureWriter(Path(sys.argv[1]))
w.write('sample_001/sample.json', b'{}')
runtime.checkpoint_then_cleanup(w, sample_report(),
    [('simulated_native_abort', lambda: os._exit(86))], runtime.StopRequest())
raise AssertionError('Unreachable after native-exit simulation')
''', output)
        self.assertEqual(result.returncode, 86, result.stderr)
        raw = json.loads((output/'raw-checkpoint.json').read_text())
        self.assertTrue(raw['sampling_complete'])
        self.assertFalse(raw['cleanup_complete'])
        self.assertFalse(raw['passed'])
        self.assertTrue(raw['partial'])
        self.assertEqual(raw['samples'][0]['artifact'], 'sample_001/sample.json')
        self.assertEqual((output/'sample_001/sample.json').read_bytes(), b'{}')
        self.assertFalse((output/'summary.json').exists())

    def test_real_sigint_and_sigterm_preserve_partial_evidence_and_restore_handlers(self):
        for signum in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signal=signum):
                output = self.root/f'signal-{signum}'
                result = self.child('''
import os, signal, sys
from pathlib import Path
import capture_core as c
import capture_red_samples as runtime
from test_cleanup import sample_report
assert 'rclpy' not in sys.modules
before = {n: signal.getsignal(n) for n in (signal.SIGINT, signal.SIGTERM)}
stop = runtime.StopRequest()
stop.install()
try:
    w = c.CaptureWriter(Path(sys.argv[1]))
    w.write('sample_001/sample.json', b'{}')
    os.kill(os.getpid(), SIGNAL_NUMBER)
    os.kill(os.getpid(), SIGNAL_NUMBER)
    report = runtime.checkpoint_then_cleanup(w, sample_report(False), [('fake_cleanup', lambda: None)], stop)
    assert report['interrupted'] and not report['passed']
finally:
    stop.restore()
assert all(signal.getsignal(n) == handler for n, handler in before.items())
assert 'rclpy' not in sys.modules
'''.replace('SIGNAL_NUMBER', str(int(signum))), output)
                self.assertEqual(result.returncode, 0, result.stderr)
                raw = json.loads((output/'raw-checkpoint.json').read_text())
                final = json.loads((output/'summary.json').read_text())
                self.assertEqual(raw['stop_signal'], signal.Signals(signum).name)
                self.assertEqual(final['saved_pairs'], 1)
                self.assertFalse(final['sampling_complete'])
                self.assertTrue(final['cleanup_complete'])
                self.assertTrue(final['partial'])
                self.assertFalse(final['passed'])

    def test_signal_during_cleanup_does_not_turn_raw_checkpoint_into_success(self):
        writer = c.CaptureWriter(self.root/'capture')
        stop = runtime.StopRequest()
        result = runtime.checkpoint_then_cleanup(writer, sample_report(),
            [('cleanup', lambda: stop.request(signal.SIGTERM, None))], stop)
        self.assertTrue(result['sampling_complete'])
        self.assertTrue(result['cleanup_complete'])
        self.assertFalse(result['passed'])
        self.assertTrue(result['partial'])
        self.assertEqual(result['stop_signal'], 'SIGTERM')
        raw = json.loads((writer.root/'raw-checkpoint.json').read_text())
        self.assertIsNone(raw['stop_signal'])
        self.assertFalse(raw['passed'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
