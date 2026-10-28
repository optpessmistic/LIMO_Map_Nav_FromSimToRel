#!/usr/bin/env python3
"""Minimal read-only Linux inventory (Python 3.6+, standard library only).

No drivers, serial ports, ROS daemon, motion commands, or parameter writes.
ROS graph discovery is disabled unless --ros-graph is explicitly supplied.
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import platform
import re
import shutil
import subprocess
import sys
import time


COMMAND_TIMEOUT_S = 4.0
TOTAL_COMMAND_BUDGET_S = 25.0
TEXT_LIMIT = 1024 * 1024
PACKAGE_PATTERNS = (
    'ros-*', 'limo*', 'librealsense*', 'realsense*', 'libuvc*',
    'astra*', 'orbbec*', 'rplidar*', 'ydlidar*', 'sllidar*', 'urg-*', 'hokuyo*',
)
PACKAGE_NAME = re.compile(r'^[a-z0-9][a-z0-9+.-]*(?::[a-z0-9_-]+)?$')
ROS_NAME = re.compile(r'^/[A-Za-z0-9_/]+$')


class Runner:
    """Bounded subprocess calls, with no shell and no raw diagnostic output."""
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.deadline = clock() + TOTAL_COMMAND_BUDGET_S

    def run(self, argv):
        remaining = self.deadline - self.clock()
        if remaining <= 0:
            return {'status': 'unknown', 'reason': 'total_command_budget_exhausted'}
        if shutil.which(argv[0]) is None:
            return {'status': 'unknown', 'reason': 'command_not_found'}
        timeout = min(COMMAND_TIMEOUT_S, remaining)
        try:
            result = subprocess.run(
                argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL, universal_newlines=True,
                timeout=timeout, check=False, shell=False,
            )
        except subprocess.TimeoutExpired:
            return {'status': 'error', 'reason': 'command_timeout', 'timeout_s': timeout}
        except (OSError, UnicodeError):
            return {'status': 'error', 'reason': 'command_execution_error'}
        if len(result.stdout) > TEXT_LIMIT:
            return {'status': 'error', 'reason': 'command_output_too_large'}
        # stdout is temporary parser input, never included wholesale in a report.
        return {'status': 'ok' if result.returncode == 0 else 'error',
                'returncode': result.returncode, 'stdout': result.stdout}


def result_status(result):
    return {key: value for key, value in result.items() if key != 'stdout'}


def read_small(path, limit=4096):
    try:
        with path.open('rb') as handle:
            data = handle.read(limit + 1)
        if len(data) > limit:
            return {'status': 'error', 'reason': 'file_too_large'}
        return {'status': 'ok', 'value': data.decode('utf-8').strip('\x00\r\n ')}
    except FileNotFoundError:
        return {'status': 'unknown', 'reason': 'not_present'}
    except PermissionError:
        return {'status': 'error', 'reason': 'permission_denied'}
    except (OSError, UnicodeError):
        return {'status': 'error', 'reason': 'read_error'}


def os_release(path):
    result = read_small(path, 16384)
    if result['status'] != 'ok':
        return result
    fields = {}
    for line in result['value'].splitlines():
        key, separator, value = line.partition('=')
        if separator and key in ('ID', 'VERSION_ID', 'PRETTY_NAME'):
            fields[key] = value.strip().strip('"\'')[:200]
    return {'status': 'ok' if fields else 'unknown', 'fields': fields}


def ros_environment(environ):
    # Do not iterate the environment, or read HOME, USER, hostname/network vars.
    value = environ.get('ROS_DISTRO')
    if value is None or value == '':
        return {'ROS_DISTRO': {'status': 'unknown', 'reason': 'unset'}}
    if not re.fullmatch(r'[a-z][a-z0-9_-]{0,31}', value):
        return {'ROS_DISTRO': {'status': 'error', 'reason': 'invalid_distro_label'}}
    return {'ROS_DISTRO': {'status': 'ok', 'value': value}}


def ros_installations(path):
    try:
        names = sorted(child.name for child in path.iterdir()
                       if child.is_dir() and re.fullmatch(r'[a-z][a-z0-9_-]{0,31}', child.name))
        return {'status': 'ok' if names else 'unknown', 'directory_names': names,
                'note': 'Directory names only; not proof of an active ROS distribution.'}
    except FileNotFoundError:
        return {'status': 'unknown', 'reason': 'opt_ros_not_present'}
    except PermissionError:
        return {'status': 'error', 'reason': 'permission_denied'}
    except OSError:
        return {'status': 'error', 'reason': 'directory_read_error'}


def installed_packages(runner):
    argv = ['dpkg-query', '-W', '-f=${binary:Package}\t${Version}\t${db:Status-Status}\n']
    result = runner.run(argv + list(PACKAGE_PATTERNS))
    packages = []
    for line in result.get('stdout', '').splitlines():
        parts = line.split('\t')
        if len(parts) != 3 or parts[2] != 'installed':
            continue
        name, version = parts[:2]
        if PACKAGE_NAME.fullmatch(name) and re.fullmatch(r'[A-Za-z0-9.+:~_-]{1,200}', version):
            packages.append({'name': name, 'version': version})
    status = result_status(result)
    if packages and result.get('returncode') == 1:
        status = {'status': 'partial', 'returncode': 1,
                  'reason': 'some_wildcards_may_have_no_matches'}
    elif not packages and result.get('returncode') in (0, 1):
        status = {'status': 'unknown', 'reason': 'no_installed_matching_packages_reported',
                  'returncode': result['returncode']}
    status.update({'packages': sorted(packages, key=lambda p: p['name']),
                   'query_patterns': list(PACKAGE_PATTERNS),
                   'scope': 'Debian package database only; source workspaces and pip installations are not inventoried.'})
    return status


def usb_devices(runner):
    result = runner.run(['lsusb'])  # Never -v/-D: no serial-number query.
    devices = []
    for line in result.get('stdout', '').splitlines():
        match = re.fullmatch(r'Bus \d+ Device \d+: ID ([0-9a-fA-F]{4}):([0-9a-fA-F]{4})(?: (.*))?', line)
        if match:
            devices.append({'vendor_id': match.group(1).lower(),
                            'product_id': match.group(2).lower(),
                            'vendor_product_label': ''.join(c for c in (match.group(3) or '') if c.isprintable())[:240]})
    status = result_status(result)
    status['devices'] = devices
    status['note'] = 'Basic lsusb vendor/product identities only; absence does not establish that the robot has no camera or lidar.'
    return status


def ros_graph(runner, enabled):
    if not enabled:
        return {'status': 'not_requested', 'queried': False}
    graph = {'status': 'requested', 'queried': False,
             'scope': 'ROS 2 discovery in this shell context only; no ROS 1 master query.'}
    for kind in ('node', 'topic'):
        help_result = runner.run(['ros2', kind, 'list', '--help'])
        help_text = help_result.get('stdout', '')
        if (help_result.get('status') != 'ok' or
                '--no-daemon' not in help_text or '--spin-time' not in help_text):
            graph[kind + 's'] = {'status': 'unknown', 'reason': 'safe_cli_flags_not_verified',
                                'help_check': result_status(help_result)}
            continue
        result = runner.run(['ros2', kind, 'list', '--no-daemon', '--spin-time', '1'])
        graph['queried'] = True
        names = [line.strip() for line in result.get('stdout', '').splitlines()
                 if ROS_NAME.fullmatch(line.strip())]
        entry = result_status(result)
        entry['names'] = sorted(set(names))
        entry['empty_is_not_proof_of_absence'] = True
        graph[kind + 's'] = entry
    graph['status'] = 'ok' if all(graph[k]['status'] == 'ok' for k in ('nodes', 'topics')) else 'incomplete'
    return graph


def collect_inventory(include_ros_graph=False, runner=None, root=Path('/'), environ=None):
    runner = runner or Runner()
    environ = os.environ if environ is None else environ
    system = platform.system()
    report = {
        'schema_version': 1,
        'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'read_only': True,
        'subject': 'The machine executing this script; not automatically the physical LIMO.',
        'physical_limo_verified': False,
        'system': system,
        'architecture': platform.machine(),
        'environment': ros_environment(environ),
        'command_limits': {'per_command_wall_s': COMMAND_TIMEOUT_S,
                           'total_subprocess_budget_wall_s': TOTAL_COMMAND_BUDGET_S},
        'excluded': ['username', 'hostname', 'IP addresses', 'MAC addresses', 'serial numbers',
                     'complete environment', 'credentials', 'serial-port access', 'driver startup',
                     'motion commands', 'parameter writes', 'ROS daemon startup'],
    }
    if system != 'Linux':
        report.update({'collection_machine_kind': 'unsupported_non_linux',
                       'inventory_status': 'unknown', 'ros_graph': {'status': 'not_queried', 'queried': False}})
        return report
    kernel = read_small(root / 'proc/sys/kernel/osrelease')
    wsl = kernel.get('status') == 'ok' and 'microsoft' in kernel['value'].lower()
    report.update({
        'collection_machine_kind': 'wsl_development_environment' if wsl else 'linux_machine_unverified',
        'os_release': os_release(root / 'etc/os-release'),
        'device_tree_model': read_small(root / 'proc/device-tree/model'),
        'ros_installations': ros_installations(root / 'opt/ros'),
        'installed_packages': installed_packages(runner),
        'usb': usb_devices(runner),
        'ros_graph': ros_graph(runner, include_ros_graph),
    })
    return report


def write_report(report, path):
    # Exclusive creation also closes the exists-check race; no parent mkdir.
    with path.open('x', encoding='utf-8') as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2, allow_nan=False)
        handle.write('\n')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros-graph', action='store_true', help='Explicit bounded ROS 2 node/topic discovery without a daemon.')
    parser.add_argument('--output', type=Path, help='New JSON file; default prints JSON to stdout. Never overwrites.')
    args = parser.parse_args(argv)
    if args.output is not None and args.output.exists():
        parser.error('Output already exists; no inventory commands were run.')
    report = collect_inventory(args.ros_graph)
    if args.output is None:
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    else:
        try:
            write_report(report, args.output)
        except (OSError, ValueError):
            parser.error('Could not create a new output file; existing files were not overwritten.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
