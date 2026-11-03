"""No ROS needed: exercise the supervisor with real, isolated Linux processes."""
from dataclasses import replace
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock


SCRIPT = Path(__file__).resolve().parents[1]/'scripts'/'record_bounded.py'
SPEC = importlib.util.spec_from_file_location('bounded_recording_under_test', SCRIPT)
recorder = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = recorder
SPEC.loader.exec_module(recorder)
LINUX = (sys.platform.startswith('linux') and hasattr(os, 'pidfd_open') and
         hasattr(signal, 'pidfd_send_signal'))


DUMMY = r'''
import json, os, pathlib, signal, subprocess, sys, time
mode, bag_arg, fixture_arg = sys.argv[1:]
bag, fixture = pathlib.Path(bag_arg), pathlib.Path(fixture_arg)
bag.mkdir(parents=True, exist_ok=True)
def ready():
    fields = pathlib.Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()
    (fixture/('pid_'+str(os.getpid())+'.json')).write_text(json.dumps({
        'pid': os.getpid(), 'start_ticks': int(fields[19]), 'mode': mode}))
def flush(signum, frame):
    (bag/'metadata.yaml').write_text('dummy_fixture_only: true\n')
    sys.exit(0)
signal.signal(signal.SIGINT, flush)
if mode == 'ignore':
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
if mode == 'default_int':
    signal.signal(signal.SIGINT, signal.SIG_DFL)
if mode in ('leader_exit', 'foreign_child'):
    environment = os.environ.copy()
    if mode == 'foreign_child':
        environment.pop('LIMO_BOUNDED_RECORDER_TOKEN')
    child = subprocess.Popen([sys.executable, __file__, 'child', bag_arg, fixture_arg], env=environment)
    deadline = time.monotonic()+3
    while not (fixture/('pid_'+str(child.pid)+'.json')).exists():
        if time.monotonic() > deadline: sys.exit(2)
        time.sleep(.01)
ready()
if mode == 'leader_exit':
    time.sleep(.15)
    sys.exit(0)
if mode == 'payload':
    (bag/'payload.mcap').write_bytes(b'x'*200000)
while True:
    time.sleep(.05)
'''


class ConfigTests(unittest.TestCase):
    def test_limits_and_topics_cannot_supply_commands(self):
        base = recorder.Config(Path('out'), Path('mission'), 'epoch', 1., ('/clock',))
        for value in (0., -1., float('nan'), float('inf'), True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                replace(base, max_wall_seconds=value).validate()
        for topics in ((), ('--output',), ('/clock;touch /tmp/no',), ('relative',)):
            with self.subTest(topics=topics), self.assertRaises(ValueError):
                replace(base, topics=topics).validate()
        with mock.patch('sys.stderr'), self.assertRaises(SystemExit):
            recorder.parser().parse_args(['--output', 'out', '--mission-dir', 'm',
                '--map-epoch', 'e', '--max-wall-seconds', '1', '--topics', '/clock',
                '--node-name', 'injected'])

    def test_terminal_requires_complete_current_schema_and_epoch(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'mission_one.json'
            good = {'schema_version': 1, 'status': 'blocked', 'map_epoch': 'current'}
            for data in ('{"schema_version":', json.dumps({**good, 'map_epoch': 'old'}),
                         json.dumps({**good, 'schema_version': True}),
                         json.dumps({**good, 'status': 'running'}), '[]'):
                path.write_text(data)
                self.assertIsNone(recorder.terminal_result(root, 'current'))
            path.write_text(json.dumps({**good, 'mission_success': float('nan')}))
            result = recorder.terminal_result(root, 'current')
            self.assertEqual(result['status'], 'blocked')
            self.assertIsNone(result['reported_mission_success'])

    @unittest.skipUnless(LINUX, 'Linux ownership inspection')
    def test_exit_race_is_dead_but_live_missing_token_remains_rejected(self):
        live = {'pid': 123, 'pgid': 123, 'sid': 123, 'start_ticks': 456, 'state': 'S'}
        owner = {**live, 'token': 'expected'}
        dead = {**live, 'state': 'Z'}
        for read_error in (None, PermissionError('process exited')):
            with self.subTest(read_error=read_error), \
                    mock.patch.object(recorder, 'proc_stat', side_effect=[live, dead]), \
                    mock.patch.object(Path, 'stat', return_value=mock.Mock(st_uid=os.geteuid())), \
                    mock.patch.object(Path, 'read_bytes', return_value=b'', side_effect=read_error):
                self.assertEqual(recorder.owned_member(owner, 123)['state'], 'Z')
        with mock.patch.object(recorder, 'proc_stat', side_effect=[live, live]), \
                mock.patch.object(Path, 'stat', return_value=mock.Mock(st_uid=os.geteuid())), \
                mock.patch.object(Path, 'read_bytes', return_value=b''), \
                self.assertRaises(recorder.OwnershipError):
            recorder.owned_member(owner, 123)


@unittest.skipUnless(LINUX, 'Real ownership/signal tests require Linux /proc and pidfds')
class BoundedRecordingTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='limo-bounded-test-')
        self.root = Path(self.temporary.name)
        self.mission = self.root/'mission'
        self.mission.mkdir()
        self.fixture = self.root/'fixtures'
        self.fixture.mkdir()
        self.dummy = self.root/'dummy.py'
        self.dummy.write_text(DUMMY)
        self.processes = []
        self.threads = []
        self.thread_errors = []
        self.finished = threading.Event()
        self.base = recorder.Config(self.root/'output', self.mission, 'test-epoch', .7, ('/clock', '/tf'))
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.finished.set()
        for thread in self.threads:
            thread.join(timeout=4)
        # Only fixtures that registered their exact PID/start ticks can be killed.
        identities = []
        for path in self.fixture.glob('pid_*.json'):
            try:
                identities.append(json.loads(path.read_text()))
            except (OSError, ValueError):
                pass
        for process, identity in self.processes:
            identities.append(identity)
        for identity in identities:
            try:
                fd = os.pidfd_open(identity['pid'])
            except ProcessLookupError:
                continue
            try:
                current = recorder.proc_stat(identity['pid'])
                if current['start_ticks'] == identity['start_ticks'] and current['state'] not in ('Z', 'X'):
                    signal.pidfd_send_signal(fd, signal.SIGKILL)
            except (FileNotFoundError, ProcessLookupError):
                pass
            finally:
                os.close(fd)
        for process, _ in self.processes:
            process.wait(timeout=3)
        self.temporary.cleanup()
        self.assertFalse(any(thread.is_alive() for thread in self.threads), 'fixture thread leaked')
        self.assertEqual(self.thread_errors, [])

    def factory(self, mode='normal'):
        def spawn(command, **kwargs):
            self.assertEqual(command[:3], ['ros2', 'bag', 'record'])
            self.assertIn('--use-sim-time', command)
            self.assertIn('--disable-keyboard-controls', command)
            self.assertEqual(command[command.index('--storage-preset-profile')+1], 'zstd_fast')
            self.assertTrue(kwargs['start_new_session'])
            self.assertIn(recorder.TOKEN_KEY, kwargs['env'])
            bag = command[command.index('--output')+1]
            process = subprocess.Popen([sys.executable, str(self.dummy), mode, bag, str(self.fixture)], **kwargs)
            self.processes.append((process, recorder.proc_stat(process.pid)))
            return process
        return spawn

    def ready(self, mode=None):
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            for path in self.fixture.glob('pid_*.json'):
                try:
                    data = json.loads(path.read_text())
                    if mode is None or data['mode'] == mode:
                        return data
                except (OSError, ValueError):
                    pass
            time.sleep(.01)
        raise AssertionError('dummy did not become ready')

    def thread(self, callback):
        def run():
            try:
                callback()
            except BaseException as exc:
                self.thread_errors.append(repr(exc))
        thread = threading.Thread(target=run, daemon=True)
        self.threads.append(thread)
        thread.start()

    def run_supervisor(self, mode='normal', **changes):
        report = recorder.supervise(replace(self.base, **changes), popen=self.factory(mode),
                                    poll_seconds=.02, cleanup_graces=(.25, .15, .3))
        saved = json.loads((self.root/'output'/'supervisor.json').read_text())
        self.assertEqual(saved, report)
        self.assertFalse(report['mission_success_inferred'])
        self.assertFalse(report['robot_stop_verified'])
        self.assertFalse(report['metadata']['integrity_verified'])
        return report

    def test_wall_deadline_flushes_without_any_ros_clock_or_terminal(self):
        report = self.run_supervisor()
        self.assertEqual(report['reason'], 'wall_budget')
        self.assertIsNone(report['terminal'])
        self.assertTrue(report['graceful_recorder_shutdown'])
        self.assertTrue(report['metadata']['exists'])
        self.assertGreaterEqual(report['stop_requested_elapsed_wall_seconds'], .7)
        self.assertLess(report['stop_requested_elapsed_wall_seconds'], 1.5)
        self.assertLess(report['elapsed_wall_seconds'], 2.)
        self.assertEqual(report['cleanup']['remaining'], [])

    def test_partial_wrong_epoch_then_terminal_has_fixed_first_tail(self):
        def writer():
            self.ready()
            path = self.mission/'mission_one.json'
            path.write_text('{"schema_version":')
            time.sleep(.12)
            path.write_text(json.dumps({'schema_version': 1, 'status': 'done', 'map_epoch': 'old'}))
            time.sleep(.12)
            data = {'schema_version': 1, 'status': 'blocked', 'map_epoch': 'test-epoch', 'mission_success': False}
            path.write_text(json.dumps(data))
            while not self.finished.wait(.04):
                # Subsequent valid files and rewrites must not postpone this first tail.
                path.write_text(json.dumps({**data, 'status': 'done'}))
                (self.mission/'mission_two.json').write_text(json.dumps(data))
        self.thread(writer)
        report = self.run_supervisor(max_wall_seconds=2., post_terminal_wall_seconds=.35)
        self.finished.set()
        self.assertEqual(report['reason'], 'terminal_tail')
        terminal = report['terminal']
        self.assertEqual(terminal['status'], 'blocked')
        self.assertFalse(terminal['reported_mission_success'])
        self.assertGreaterEqual(terminal['observed_elapsed_wall_seconds'], .24)
        self.assertAlmostEqual(terminal['tail_deadline_elapsed_wall_seconds']-
                               terminal['observed_elapsed_wall_seconds'], .35)
        self.assertLess(report['stop_requested_elapsed_wall_seconds']-
                        terminal['tail_deadline_elapsed_wall_seconds'], .15)
        self.assertTrue(report['graceful_recorder_shutdown'])

    def test_partial_file_never_extends_total_wall_deadline(self):
        def writer():
            self.ready()
            while not self.finished.wait(.02):
                (self.mission/'mission_partial.json').write_text('{"schema_version":1,')
        self.thread(writer)
        report = self.run_supervisor(post_terminal_wall_seconds=90.)
        self.assertEqual(report['reason'], 'wall_budget')
        self.assertIsNone(report['terminal'])
        self.assertLess(report['elapsed_wall_seconds'], 2.)

    def test_terminal_tail_cannot_extend_total_wall_budget(self):
        def writer():
            self.ready()
            (self.mission/'mission_done.json').write_text(json.dumps({
                'schema_version': 1, 'status': 'done', 'map_epoch': 'test-epoch', 'mission_success': True}))
        self.thread(writer)
        report = self.run_supervisor(post_terminal_wall_seconds=90.)
        self.assertEqual(report['reason'], 'wall_budget')
        self.assertEqual(report['terminal']['status'], 'done')
        self.assertTrue(report['terminal']['reported_mission_success'])
        self.assertGreater(report['terminal']['tail_deadline_elapsed_wall_seconds'], 90.)
        self.assertLess(report['elapsed_wall_seconds'], 2.)

    def test_supervisor_sigint_flushes_and_restores_handlers(self):
        before = signal.getsignal(signal.SIGINT)
        def interrupt():
            self.ready()
            os.kill(os.getpid(), signal.SIGINT)
        self.thread(interrupt)
        report = self.run_supervisor(max_wall_seconds=3.)
        self.assertEqual(report['reason'], 'supervisor_SIGINT')
        self.assertTrue(report['graceful_recorder_shutdown'])
        self.assertTrue(report['metadata']['exists'])
        self.assertEqual(signal.getsignal(signal.SIGINT), before)

    def test_ignoring_int_and_term_is_killed_and_not_graceful(self):
        report = self.run_supervisor('ignore')
        self.assertEqual(report['reason'], 'wall_budget')
        self.assertEqual(report['status'], 'failed')
        self.assertTrue(report['cleanup']['forced_kill'])
        self.assertTrue(report['cleanup']['terminate_used'])
        self.assertEqual(report['recorder_exit_code'], -signal.SIGKILL)
        self.assertEqual(report['cleanup']['remaining'], [])
        self.assertFalse(report['metadata']['exists'])
        self.assertLess(report['elapsed_wall_seconds'], 2.)

    def test_default_sigint_exit_is_not_confirmed_flush(self):
        report = self.run_supervisor('default_int')
        self.assertEqual(report['recorder_exit_code'], -signal.SIGINT)
        self.assertFalse(report['graceful_recorder_shutdown'])
        self.assertFalse(report['metadata']['exists'])
        self.assertEqual(report['cleanup']['remaining'], [])

    def test_early_leader_exit_still_cleans_child_without_touching_unrelated(self):
        unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'], start_new_session=True)
        self.processes.append((unrelated, recorder.proc_stat(unrelated.pid)))
        report = self.run_supervisor('leader_exit', max_wall_seconds=3.)
        self.assertEqual(report['reason'], 'recorder_early_exit')
        self.assertFalse(report['graceful_recorder_shutdown'])
        self.assertTrue(report['metadata']['exists'])
        child = self.ready('child')
        self.assertIn(child['pid'], [entry['pid'] for entry in report['cleanup']['signals']])
        self.assertEqual(report['cleanup']['remaining'], [])
        self.assertIsNone(unrelated.poll())

    def test_live_unknown_member_refuses_all_group_signals(self):
        report = self.run_supervisor('foreign_child')
        self.assertEqual(report['status'], 'failed')
        self.assertTrue(report['cleanup']['ownership_errors'])
        self.assertFalse(report['cleanup']['remaining_check_complete'])
        self.assertEqual(report['cleanup']['signals'], [])
        self.assertIsNone(self.processes[0][0].poll())
        self.assertNotIn(recorder.proc_stat(self.ready('child')['pid'])['state'], ('Z', 'X'))

    def test_permission_error_is_not_empty_group_and_cannot_signal(self):
        original = Path.read_bytes
        def denied(path):
            if path.name == 'environ' and str(path).startswith('/proc/'):
                raise PermissionError('fixture denied')
            return original(path)
        with mock.patch.object(Path, 'read_bytes', denied):
            report = self.run_supervisor()
        self.assertTrue(report['cleanup']['ownership_errors'])
        self.assertFalse(report['cleanup']['remaining_check_complete'])
        self.assertEqual(report['cleanup']['signals'], [])
        self.assertIsNone(self.processes[0][0].poll())

    def test_cleanup_exception_persists_failure_and_restores_handler(self):
        before = signal.getsignal(signal.SIGINT)
        with mock.patch.object(recorder, 'stop_owned', side_effect=RuntimeError('fixture inspection failure')):
            report = self.run_supervisor()
        self.assertEqual(report['status'], 'failed')
        self.assertFalse(report['cleanup']['remaining_check_complete'])
        self.assertTrue(report['errors'])
        self.assertEqual(signal.getsignal(signal.SIGINT), before)
        self.assertIsNone(self.processes[0][0].poll())

    def test_soft_total_bytes_counts_bag_payload(self):
        report = self.run_supervisor('payload', max_wall_seconds=3., max_bytes=100000)
        self.assertEqual(report['reason'], 'max_bytes')
        self.assertGreaterEqual(report['observed_bytes'], 200000)
        self.assertTrue(report['graceful_recorder_shutdown'])

    def test_free_space_threshold_stops_owned_recorder(self):
        actual = recorder.shutil.disk_usage(self.root)
        def disk_usage(path):
            if list(self.fixture.glob('pid_*.json')):
                return actual._replace(free=0)
            return actual
        with mock.patch.object(recorder.shutil, 'disk_usage', disk_usage):
            report = self.run_supervisor(max_wall_seconds=3., min_free_bytes=1)
        self.assertEqual(report['reason'], 'min_free_bytes')
        self.assertTrue(report['graceful_recorder_shutdown'])

    def test_existing_output_or_mission_is_rejected_before_spawn(self):
        spawn = mock.Mock()
        self.base.output.mkdir()
        with self.assertRaises(FileExistsError):
            recorder.supervise(self.base, popen=spawn)
        (self.mission/'mission_old.json').write_text('{}')
        with self.assertRaisesRegex(ValueError, 'already contains'):
            recorder.supervise(replace(self.base, output=self.root/'new-output'), popen=spawn)
        spawn.assert_not_called()
        self.assertFalse((self.root/'new-output').exists())

    def test_byte_accounting_rejects_symlink_directory_and_scan_errors(self):
        output = self.root/'accounting'
        output.mkdir()
        (output/'linked').symlink_to(self.mission, target_is_directory=True)
        with self.assertRaisesRegex(OSError, 'Symlink directory'):
            recorder.tree_bytes(output)
        def failed_walk(path, **kwargs):
            kwargs['onerror'](PermissionError('fixture unreadable directory'))
            return iter(())
        with mock.patch.object(recorder.os, 'walk', failed_walk), self.assertRaises(PermissionError):
            recorder.tree_bytes(output)


if __name__ == '__main__':
    unittest.main()
