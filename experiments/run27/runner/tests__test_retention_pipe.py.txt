"""Exercise the real adapter with mocked OS pipes/child; no processes or ROS."""
import errno
from pathlib import Path
import signal
import sys
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, call, patch

REPO = Path(__file__).resolve().parents[1]
for root in (REPO/'src/limo_evidence_protocol', REPO/'src/limo_maze_perception'):
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

from limo_evidence_protocol.retention import canonical
from limo_maze_perception import retention_adapter as adapter


class Stream:
    def __init__(self, fd):
        self.fd, self.closed = fd, False

    def fileno(self):
        return self.fd

    def close(self):
        self.closed = True


class RetentionPipeTests(unittest.TestCase):
    def setUp(self):
        # No child is launched: package-path canonicalization is outside this
        # transport test and can be denied by a Windows filesystem sandbox.
        self.mock(adapter.Path, 'resolve', autospec=True,
                  side_effect=lambda path, strict=False: path)
        self.process = S(pid=123, stdin=Stream(10), stdout=Stream(11), stderr=Stream(12),
                         poll=Mock(return_value=None))
        self.popen = self.mock(adapter.subprocess, 'Popen', return_value=self.process)
        self.pidfd_open = self.mock(adapter.os, 'pidfd_open', return_value=77, create=True)
        self.nonblocking = self.mock(adapter.os, 'set_blocking')
        self.send_signal = self.mock(adapter.signal, 'pidfd_send_signal', create=True)
        self.mock(adapter.signal, 'SIGKILL', 9, create=True)
        self.read = self.mock(adapter.os, 'read', side_effect=BlockingIOError(errno.EAGAIN, 'empty'))
        self.write = self.mock(adapter.os, 'write', side_effect=lambda fd, raw: len(raw))
        self.api = S(F_GETPIPE_SZ=1032, F_SETPIPE_SZ=1031,
                     fcntl=Mock(side_effect=[65536, 1048576, 1048576]))
        self.mock(adapter, 'fcntl', self.api)

    def mock(self, target, name, *args, **kwargs):
        replacement = patch.object(target, name, *args, **kwargs)
        value = replacement.start()
        self.addCleanup(replacement.stop)
        return value

    def writer(self, payload=b'{}'):
        return adapter.OwnedWriter(REPO/'unused-mocked-root', payload, None, adapter.RuntimePolicy(), 0.)

    def healthy_fallback(self, writer):
        self.assertIs(writer.process, self.process)
        self.assertEqual(writer.pidfd, 77)
        self.assertIsNone(writer.setup_error)
        self.assertIsNone(writer.fault)
        self.assertIsNone(writer.stop_reason)
        self.assertFalse(writer.process.stdin.closed)
        self.popen.assert_called_once()
        self.send_signal.assert_not_called()

    def test_tuning_occurs_after_pidfd_and_nonblocking_and_reads_actual_capacity(self):
        replies = iter([65536, 1048576, 524288])
        def control(fd, operation, *args):
            self.pidfd_open.assert_called_once_with(123)
            self.assertEqual(self.nonblocking.call_args_list,
                             [call(10, False), call(11, False), call(12, False)])
            self.assertEqual(fd, 10)
            return next(replies)
        self.api.fcntl.side_effect = control
        writer = self.writer()
        self.healthy_fallback(writer)
        self.assertEqual(self.api.fcntl.call_args_list,
            [call(10, 1032), call(10, 1031, 1048576), call(10, 1032)])
        self.assertEqual(writer.snapshot()['stdin_pipe'], dict(initial_bytes=65536,
            requested_bytes=1048576, actual_bytes=524288, tuning_error=None, readback_error=None))

    def test_permission_denied_keeps_original_pipe_and_owned_child(self):
        self.api.fcntl.side_effect = [65536, PermissionError(errno.EPERM, 'denied'), 65536]
        writer = self.writer()
        self.healthy_fallback(writer)
        self.assertEqual(writer.stdin_pipe['actual_bytes'], 65536)
        self.assertEqual(writer.stdin_pipe['tuning_error'], 'PermissionError:errno=1')
        self.assertFalse(writer.poll(.05))
        self.assertIsNone(writer.fault)
        self.assertTrue(writer.process.stdin.closed)

    def test_missing_fcntl_module_or_linux_constants_is_safe(self):
        for api in (None, S(fcntl=Mock()), S(F_GETPIPE_SZ=1032, fcntl=Mock(return_value=65536))):
            with self.subTest(api=api), patch.object(adapter, 'fcntl', api):
                self.popen.reset_mock()
                writer = self.writer()
                self.healthy_fallback(writer)
                self.assertEqual(writer.stdin_pipe['tuning_error'], 'unsupported')
                self.assertIsNone(writer.stdin_pipe['requested_bytes'])
                if api is not None and hasattr(api, 'F_GETPIPE_SZ'):
                    self.assertEqual(writer.stdin_pipe['actual_bytes'], 65536)
                else:
                    self.assertIsNone(writer.stdin_pipe['actual_bytes'])

    def test_get_capacity_failure_preserves_ownership_and_reports_unknown_actual(self):
        self.api.fcntl.side_effect = OSError(errno.EINVAL, 'unsupported ioctl')
        writer = self.writer()
        self.healthy_fallback(writer)
        self.assertIsNone(writer.stdin_pipe['initial_bytes'])
        self.assertIsNone(writer.stdin_pipe['actual_bytes'])
        self.assertIsNone(writer.stdin_pipe['requested_bytes'])
        self.assertIn('errno=22', writer.stdin_pipe['tuning_error'])
        self.assertIn('errno=22', writer.stdin_pipe['readback_error'])

    def test_failed_readback_does_not_claim_requested_capacity(self):
        self.api.fcntl.side_effect = [65536, 1048576, OSError(errno.EBADF, 'readback failed')]
        writer = self.writer()
        self.healthy_fallback(writer)
        self.assertEqual(writer.stdin_pipe['initial_bytes'], 65536)
        self.assertIsNone(writer.stdin_pipe['actual_bytes'])
        self.assertIsNone(writer.stdin_pipe['tuning_error'])
        self.assertIn('errno=9', writer.stdin_pipe['readback_error'])

    def test_existing_larger_pipe_is_not_shrunk_or_reported_as_a_new_request(self):
        self.api.fcntl.side_effect = [2 * 1048576]
        writer = self.writer()
        self.healthy_fallback(writer)
        self.api.fcntl.assert_called_once_with(10, 1032)
        self.assertEqual(writer.stdin_pipe['actual_bytes'], 2 * 1048576)
        self.assertIsNone(writer.stdin_pipe['requested_bytes'])

    def test_required_setup_failure_is_still_fatal_and_skips_optional_tuning(self):
        self.pidfd_open.side_effect = OSError(errno.EMFILE, 'no pidfd')
        writer = self.writer()
        self.api.fcntl.assert_not_called()
        self.assertEqual(writer.setup_error, 'writer_setup_failed:OSError')
        self.assertEqual(writer.fault, writer.setup_error)
        self.assertIsNone(writer.pidfd)
        self.assertTrue(all(stream.closed for stream in
                            (self.process.stdin, self.process.stdout, self.process.stderr)))

    def test_large_payload_keeps_one_mib_tick_budget_and_exact_sent_bytes(self):
        writer = self.writer(b'x' * (2 * 1048576 + 13))
        expected = b''.join(writer.segments)
        transmitted = bytearray()
        def send(fd, raw):
            self.assertEqual(fd, 10)
            self.assertLessEqual(len(raw), 65536)
            transmitted.extend(raw)
            return len(raw)
        self.write.side_effect = send
        for now, expected_sent in ((.05, 1048576), (.10, 2 * 1048576), (.15, len(expected))):
            self.assertFalse(writer.poll(now))
            self.assertEqual(len(transmitted), expected_sent)
            progress = writer.snapshot()['input_transfer']
            self.assertEqual(progress['sent_bytes'], expected_sent)
            self.assertEqual(progress['remaining_bytes'], len(expected) - expected_sent)
        self.assertEqual(bytes(transmitted), expected)
        self.assertTrue(self.process.stdin.closed)
        self.assertEqual(progress['total_bytes'], len(expected))
        self.assertEqual(progress['first_progress_wall'], .05)
        self.assertEqual(progress['last_progress_wall'], .15)
        self.assertEqual(progress['stdin_closed_wall'], .15)
        self.assertEqual(progress['eagain_count'], 0)
        self.assertAlmostEqual(progress['max_poll_gap_s'], .05)
        # Sending all bytes is not completion: only the original valid reply + exit completes it.
        self.assertIsNone(writer.returncode)
        self.process.poll.return_value = 0
        writer.output.extend(canonical(dict(ok=True, operation_id=writer.operation_id)))
        self.assertTrue(writer.poll(.20))
        self.assertTrue(writer.reply['ok'])
        self.assertIsNone(writer.fault)
        self.assertEqual(writer.snapshot()['input_transfer']['stdin_closed_wall'], .15)

    def test_backpressure_partial_send_and_original_five_second_timeout_evidence(self):
        writer = self.writer(b'x' * 100000)
        self.write.side_effect = [7, BlockingIOError(errno.EAGAIN, 'full')]
        self.assertFalse(writer.poll(.2))
        self.write.side_effect = BlockingIOError(errno.EAGAIN, 'still full')
        self.assertFalse(writer.poll(.35))
        self.assertFalse(writer.poll(5.))
        snapshot = writer.snapshot()
        progress = snapshot['input_transfer']
        self.assertEqual(progress['sent_bytes'], 7)
        self.assertEqual(progress['remaining_bytes'], progress['total_bytes'] - 7)
        self.assertEqual(progress['eagain_count'], 3)
        self.assertEqual(progress['first_progress_wall'], .2)
        self.assertEqual(progress['last_progress_wall'], .2)
        self.assertIsNone(progress['stdin_closed_wall'])
        self.assertAlmostEqual(progress['max_poll_gap_s'], 4.65)
        self.assertEqual(writer.policy.worker_timeout_s, 5.)
        self.assertEqual(writer.policy.shutdown_timeout_s, 8.)
        self.assertEqual(snapshot['stop_reason'], 'writer_wall_timeout')
        self.assertIsNone(snapshot['returncode'])
        self.assertIsNone(snapshot['fault'])
        self.assertFalse(snapshot['termination_unconfirmed'])
        self.send_signal.assert_called_once_with(77, signal.SIGTERM)
        self.assertFalse(writer.poll(5.5))
        self.assertEqual(self.send_signal.call_args_list,
                         [call(77, signal.SIGTERM), call(77, signal.SIGKILL)])
        self.assertEqual(writer.input_sent_bytes, 7)
        self.assertEqual(writer.input_eagain_count, 3)
        self.assertIs(writer.process, self.process)
        self.popen.assert_called_once()

    def test_snapshots_do_not_expose_mutable_diagnostic_state(self):
        writer = self.writer()
        snapshot = writer.snapshot()
        snapshot['stdin_pipe']['actual_bytes'] = 1
        snapshot['input_transfer']['sent_bytes'] = 99
        self.assertEqual(writer.snapshot()['stdin_pipe']['actual_bytes'], 1048576)
        self.assertEqual(writer.snapshot()['input_transfer']['sent_bytes'], 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
