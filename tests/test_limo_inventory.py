import contextlib
import io
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import collect_limo_inventory as inventory


class FakeRunner:
    def __init__(self, response=None):
        self.calls = []
        self.response = response

    def run(self, argv):
        self.calls.append(argv)
        if self.response:
            return self.response(argv)
        return {'status': 'unknown', 'reason': 'command_not_found'}


class InventoryTests(unittest.TestCase):
    def test_default_never_calls_ros_cli(self):
        runner = FakeRunner()
        with tempfile.TemporaryDirectory() as directory, patch.object(inventory.platform, 'system', return_value='Linux'):
            report = inventory.collect_inventory(runner=runner, root=Path(directory), environ={})
        self.assertEqual(report['ros_graph'], {'status': 'not_requested', 'queried': False})
        self.assertEqual([cmd[0] for cmd in runner.calls], ['dpkg-query', 'lsusb'])
        self.assertFalse(report['physical_limo_verified'])

    def test_environment_has_exactly_one_allowed_read(self):
        class WhitelistEnvironment:
            def __init__(self):
                self.reads = []
            def get(self, key):
                self.reads.append(key)
                if key != 'ROS_DISTRO':
                    raise AssertionError('Unexpected environment access')
                return 'humble'
        env = WhitelistEnvironment()
        self.assertEqual(inventory.ros_environment(env), {'ROS_DISTRO': {'status': 'ok', 'value': 'humble'}})
        self.assertEqual(env.reads, ['ROS_DISTRO'])
        invalid = inventory.ros_environment({'ROS_DISTRO': 'secret data', 'TOKEN': 'other-secret'})
        self.assertNotIn('secret', json.dumps(invalid))

    def test_missing_command_is_unknown_without_execution(self):
        with patch.object(inventory.shutil, 'which', return_value=None), patch.object(inventory.subprocess, 'run') as run:
            result = inventory.Runner().run(['lsusb'])
        run.assert_not_called()
        self.assertEqual(result, {'status': 'unknown', 'reason': 'command_not_found'})

    def test_timeout_does_not_report_partial_sensitive_output(self):
        exception = subprocess.TimeoutExpired(['lsusb'], 4, output='secret-host', stderr='192.0.2.1')
        with patch.object(inventory.shutil, 'which', return_value='/bin/lsusb'), \
                patch.object(inventory.subprocess, 'run', side_effect=exception) as run:
            result = inventory.Runner().run(['lsusb'])
        self.assertEqual(result['reason'], 'command_timeout')
        self.assertNotIn('secret', json.dumps(result))
        self.assertNotIn('192.0.2.1', json.dumps(result))
        self.assertLessEqual(run.call_args[1]['timeout'], inventory.COMMAND_TIMEOUT_S)
        self.assertFalse(run.call_args[1]['shell'])

    def test_total_budget_prevents_more_commands(self):
        now = [0.0]
        runner = inventory.Runner(clock=lambda: now[0])
        now[0] = inventory.TOTAL_COMMAND_BUDGET_S + 1
        with patch.object(inventory.subprocess, 'run') as run:
            result = runner.run(['ros2', 'node', 'list'])
        run.assert_not_called()
        self.assertEqual(result['reason'], 'total_command_budget_exhausted')

    def test_remaining_budget_reduces_command_timeout(self):
        now = [0.0]
        runner = inventory.Runner(clock=lambda: now[0])
        now[0] = inventory.TOTAL_COMMAND_BUDGET_S - .25
        with patch.object(inventory.shutil, 'which', return_value='/bin/lsusb'), \
                patch.object(inventory.subprocess, 'run', return_value=SimpleNamespace(returncode=0, stdout='')) as run:
            runner.run(['lsusb'])
        self.assertEqual(run.call_args[1]['timeout'], .25)

    def test_output_refused_before_collection_and_exclusive_write(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'inventory.json'
            path.write_text('existing', encoding='utf-8')
            with patch.object(inventory, 'collect_inventory') as collect, contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit):
                    inventory.main(['--output', str(path)])
            collect.assert_not_called()
            with self.assertRaises(FileExistsError):
                inventory.write_report({'new': True}, path)
            self.assertEqual(path.read_text(encoding='utf-8'), 'existing')

    def test_stdout_default_is_json_without_raw_environment(self):
        report = {'read_only': True, 'physical_limo_verified': False}
        output = io.StringIO()
        with patch.object(inventory, 'collect_inventory', return_value=report) as collect, contextlib.redirect_stdout(output):
            self.assertEqual(inventory.main([]), 0)
        collect.assert_called_once_with(False)
        self.assertEqual(json.loads(output.getvalue()), report)

    def test_graph_requires_verified_flags_for_each_command(self):
        runner = FakeRunner(lambda argv: {'status': 'ok', 'returncode': 0, 'stdout': '--spin-time only'})
        result = inventory.ros_graph(runner, True)
        self.assertFalse(result['queried'])
        self.assertEqual(len(runner.calls), 2)
        self.assertTrue(all(cmd[-1] == '--help' for cmd in runner.calls))
        self.assertEqual(result['status'], 'incomplete')

    def test_explicit_graph_never_uses_daemon_or_other_ros_commands(self):
        def response(argv):
            return {'status': 'ok', 'returncode': 0,
                    'stdout': '--no-daemon --spin-time' if argv[-1] == '--help' else '/camera\n192.0.2.1\nwarning: host\n'}
        runner = FakeRunner(response)
        result = inventory.ros_graph(runner, True)
        self.assertEqual(runner.calls, [
            ['ros2', 'node', 'list', '--help'], ['ros2', 'node', 'list', '--no-daemon', '--spin-time', '1'],
            ['ros2', 'topic', 'list', '--help'], ['ros2', 'topic', 'list', '--no-daemon', '--spin-time', '1'],
        ])
        self.assertEqual(result['nodes']['names'], ['/camera'])
        self.assertNotIn('192.0.2.1', json.dumps(result))

    def test_package_status_and_usb_parse_do_not_include_raw_diagnostics(self):
        package_runner = FakeRunner(lambda argv: {'status': 'error', 'returncode': 1,
            'stdout': 'ros-jazzy-rclpy\t7.1.1-1\tinstalled\nros-foxy-removed\t1.0\tconfig-files\n'})
        packages = inventory.installed_packages(package_runner)
        self.assertEqual(packages['status'], 'partial')
        self.assertEqual(packages['packages'], [{'name': 'ros-jazzy-rclpy', 'version': '7.1.1-1'}])
        usb_runner = FakeRunner(lambda argv: {'status': 'ok', 'returncode': 0,
            'stdout': 'Bus 001 Device 002: ID 2bc5:0403 Orbbec 3D Camera\niSerial 123456789\n'})
        usb = inventory.usb_devices(usb_runner)
        self.assertEqual(usb_runner.calls, [['lsusb']])
        self.assertEqual(usb['devices'][0]['vendor_id'], '2bc5')
        self.assertNotIn('123456789', json.dumps(usb))
        self.assertNotIn('Bus 001', json.dumps(usb))

    def test_only_allowed_os_fields_and_wsl_is_not_robot(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(inventory.platform, 'system', return_value='Linux'):
            root = Path(directory)
            (root / 'etc').mkdir()
            (root / 'etc/os-release').write_text('ID=ubuntu\nVERSION_ID="22.04"\nPRETTY_NAME="Ubuntu 22.04"\nSECRET=ignored\n', encoding='utf-8')
            (root / 'proc/sys/kernel').mkdir(parents=True)
            (root / 'proc/sys/kernel/osrelease').write_text('6.6-microsoft-standard-WSL2', encoding='utf-8')
            report = inventory.collect_inventory(runner=FakeRunner(), root=root, environ={'ROS_DISTRO': 'humble', 'TOKEN': 'hidden'})
        self.assertEqual(report['collection_machine_kind'], 'wsl_development_environment')
        self.assertFalse(report['physical_limo_verified'])
        self.assertEqual(set(report['os_release']['fields']), {'ID', 'VERSION_ID', 'PRETTY_NAME'})
        self.assertNotIn('hidden', json.dumps(report))
        self.assertNotIn('SECRET', json.dumps(report))
        self.assertEqual(report['device_tree_model']['status'], 'unknown')


if __name__ == '__main__':
    unittest.main()
