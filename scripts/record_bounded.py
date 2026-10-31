#!/usr/bin/env python3
"""Bound one simulation rosbag recorder; never control the robot or other jobs.

Linux /proc and pidfds are required. Wall/size/free-space limits start bounded
recorder shutdown, not an instantaneous exit guarantee. Cleanup adds at most
10 s SIGINT + 3 s TERM + 2 s KILL polling budgets, excluding scheduling/I/O
delays. A metadata file's existence does not establish a complete/lossless bag.
"""
import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import time
import uuid


TOKEN_KEY = 'LIMO_BOUNDED_RECORDER_TOKEN'
TOPIC = re.compile(r'/[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*\Z')


@dataclass(frozen=True)
class Config:
    output: Path
    mission_dir: Path
    map_epoch: str
    max_wall_seconds: float
    topics: tuple
    post_terminal_wall_seconds: float = 90.
    max_bytes: int | None = None
    min_free_bytes: int = 0
    qos_profile: Path | None = None
    max_bag_size: int = 256*1024*1024

    def validate(self):
        for name, zero in (('max_wall_seconds', False), ('post_terminal_wall_seconds', True)):
            value = getattr(self, name)
            if type(value) not in (float, int) or not math.isfinite(value) or value < 0 or (not zero and value == 0):
                raise ValueError(name+' must be finite and '+('nonnegative' if zero else 'positive'))
        for name, minimum in (('max_bytes', 1), ('min_free_bytes', 0), ('max_bag_size', 1)):
            value = getattr(self, name)
            if name == 'max_bytes' and value is None:
                continue
            if type(value) is not int or value < minimum:
                raise ValueError(name+' is invalid')
        if not isinstance(self.map_epoch, str) or not self.map_epoch or self.map_epoch.strip() != self.map_epoch or len(self.map_epoch) > 256:
            raise ValueError('An explicit nonempty map epoch is required')
        if not self.topics or any(not isinstance(t, str) or not TOPIC.fullmatch(t) for t in self.topics):
            raise ValueError('topics must be explicit absolute ROS topic names')
        if self.qos_profile is not None and not Path(self.qos_profile).is_file():
            raise ValueError('QoS profile does not exist')


def proc_stat(pid):
    """Read Linux stat without splitting a command name containing spaces/)."""
    raw = Path('/proc', str(pid), 'stat').read_text()
    fields = raw[raw.rfind(')')+2:].split()
    return {'pid': pid, 'state': fields[0], 'pgid': int(fields[2]),
            'sid': int(fields[3]), 'start_ticks': int(fields[19])}


class OwnershipError(RuntimeError):
    pass


def owned_member(owner, pid):
    before = proc_stat(pid)
    if before['pgid'] != owner['pgid'] or before['sid'] != owner['sid']:
        raise OwnershipError('process group/session changed')
    if before['state'] in ('Z', 'X'):
        return before
    try:
        uid = Path('/proc', str(pid)).stat().st_uid
        environment = Path('/proc', str(pid), 'environ').read_bytes().split(b'\0')
    except OSError:
        after = proc_stat(pid)
        if after['start_ticks'] == before['start_ticks'] and after['state'] in ('Z', 'X'):
            return after  # Exit can empty/restrict environ before /proc disappears.
        raise
    after = proc_stat(pid)
    if any(before[k] != after[k] for k in ('start_ticks', 'pgid', 'sid')):
        raise OwnershipError('process identity changed during inspection')
    if after['state'] in ('Z', 'X'):
        return after
    if uid != os.geteuid():
        raise OwnershipError('process UID differs')
    if (TOKEN_KEY+'='+owner['token']).encode() not in environment:
        raise OwnershipError('process token missing or different')
    return after


def scan_owned_group(owner):
    """Unknown ownership/read permission is an error, never an empty group."""
    members, errors = [], []
    try:
        leader = proc_stat(owner['pid'])
        if leader['start_ticks'] != owner['start_ticks']:
            errors.append('leader PID was reused')
    except (FileNotFoundError, ProcessLookupError):
        pass
    except (OSError, ValueError, IndexError):
        errors.append('leader identity could not be read')
    try:
        entries = list(Path('/proc').iterdir())
    except OSError:
        return [], ['proc enumeration failed']
    for entry in entries:
        if not entry.name.isdecimal():
            continue
        pid = int(entry.name)
        try:
            info = proc_stat(pid)
            if info['pgid'] != owner['pgid']:
                continue
            if info['state'] in ('Z', 'X'):
                continue  # Dead processes cannot record and must not be signaled.
            info = owned_member(owner, pid)
            if info['state'] not in ('Z', 'X'):
                members.append(info)
        except (FileNotFoundError, ProcessLookupError):
            continue
        except (OSError, ValueError, IndexError, OwnershipError):
            if len(errors) < 8:
                errors.append('ownership/read uncertain for pid '+str(pid))
    return members, errors


def signal_owned(owner, member, signum):
    """Pidfd pins identity; fresh /proc checks precede every signal."""
    fd = os.pidfd_open(member['pid'])
    try:
        fresh = owned_member(owner, member['pid'])
        if fresh['start_ticks'] != member['start_ticks']:
            raise OwnershipError('PID start ticks changed before signal')
        if fresh['state'] not in ('Z', 'X'):
            signal.pidfd_send_signal(fd, signum)
    finally:
        os.close(fd)


def stop_owned(owner, process, *, graces=(10., 3., 2.), poll_seconds=.2):
    started = time.monotonic()
    result = {'signals': [], 'ownership_errors': [], 'remaining': [], 'remaining_check_complete': False, 'forced_kill': False,
              'terminate_used': False, 'cleanup_budget_seconds': sum(graces)}
    for signum, grace in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), graces):
        deadline, sent = time.monotonic()+grace, set()
        while True:
            process.poll()  # Reap only our leader, not unrelated/orphan processes.
            members, errors = scan_owned_group(owner)
            result['remaining'] = members
            result['remaining_check_complete'] = not errors
            if errors:
                result['ownership_errors'] = errors
                result['elapsed_wall_seconds'] = time.monotonic()-started
                return result  # Fail closed: never killpg based on incomplete inspection.
            if not members:
                result['elapsed_wall_seconds'] = time.monotonic()-started
                return result
            for member in members:
                identity = member['pid'], member['start_ticks']
                if identity in sent:
                    continue
                try:
                    signal_owned(owner, member, signum)
                    sent.add(identity)
                    result['signals'].append({'pid': member['pid'], 'start_ticks': member['start_ticks'],
                                              'signal': signal.Signals(signum).name})
                    result['forced_kill'] |= signum == signal.SIGKILL
                    result['terminate_used'] |= signum == signal.SIGTERM
                except (FileNotFoundError, ProcessLookupError):
                    continue
                except (OSError, ValueError, OwnershipError):
                    result['ownership_errors'] = ['signal ownership/access uncertain for pid '+str(member['pid'])]
                    result['remaining_check_complete'] = False
                    result['elapsed_wall_seconds'] = time.monotonic()-started
                    return result
            if time.monotonic() >= deadline:
                break
            time.sleep(min(poll_seconds, max(0., deadline-time.monotonic())))
    process.poll()
    result['remaining'], result['ownership_errors'] = scan_owned_group(owner)
    result['remaining_check_complete'] = not result['ownership_errors']
    result['elapsed_wall_seconds'] = time.monotonic()-started
    return result


def terminal_result(directory, epoch):
    """Incomplete, unrelated, or oversized files cannot authorize a tail timer."""
    for path in sorted(Path(directory).glob('mission_*.json')):
        try:
            before = path.stat()
            if before.st_size > 1024*1024:
                continue
            raw = path.read_bytes()
            after = path.stat()
            if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
                continue
            data = json.loads(raw)
            if (type(data) is dict and type(data.get('schema_version')) is int and
                    data['schema_version'] == 1 and data.get('status') in ('done', 'blocked') and
                    data.get('map_epoch') == epoch):
                return {'path': str(path), 'sha256': hashlib.sha256(raw).hexdigest(),
                        'status': data['status'], 'map_epoch': epoch,
                        'reported_mission_success': (data.get('mission_success')
                            if type(data.get('mission_success')) is bool else None)}
        except (OSError, ValueError, RecursionError):
            continue
    return None


def tree_bytes(directory):
    total = 0
    def raise_error(error):
        raise error
    for root, directories, files in os.walk(directory, followlinks=False, onerror=raise_error):
        for name in directories:
            if Path(root, name).is_symlink():
                raise OSError('Symlink directory in recording output; byte accounting uncertain')
        for name in files:
            path = Path(root, name)
            if path.is_symlink():
                raise OSError('Symlink in recording output; byte accounting uncertain')
            total += path.stat().st_size
    return total


def write_report(output, report):
    temporary = output/'supervisor.json.tmp'
    temporary.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    temporary.replace(output/'supervisor.json')


def supervise(config, *, popen=subprocess.Popen, poll_seconds=.2, cleanup_graces=(10., 3., 2.)):
    """The injectable Popen factory is for tests; CLI has no arbitrary command option."""
    config.validate()
    if os.name != 'posix' or not Path('/proc').is_dir() or not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
        raise ValueError('Linux /proc and Python pidfd support are required')
    if not 0 < poll_seconds <= 1 or len(cleanup_graces) != 3 or any(not math.isfinite(x) or x <= 0 for x in cleanup_graces):
        raise ValueError('Invalid supervisor polling/cleanup budgets')
    output, mission_dir = Path(config.output).resolve(), Path(config.mission_dir).resolve()
    if not mission_dir.is_dir():
        raise ValueError('Mission directory must already exist')
    if next(mission_dir.glob('mission_*.json'), None) is not None:
        raise ValueError('Mission directory already contains mission_*.json; use a fresh run directory')
    output.mkdir(parents=True, exist_ok=False)
    command = ['ros2', 'bag', 'record', '--storage', 'mcap', '--storage-preset-profile', 'zstd_fast',
               '--use-sim-time', '--disable-keyboard-controls', '--output', str(output/'bag'),
               '--max-bag-size', str(config.max_bag_size)]
    if config.qos_profile is not None:
        command += ['--qos-profile-overrides-path', str(Path(config.qos_profile).resolve())]
    command += ['--topics', *config.topics]
    environment, token = os.environ.copy(), uuid.uuid4().hex
    environment[TOKEN_KEY] = token
    started, process, owner = time.monotonic(), None, None
    requested = []
    old_handlers = {}
    report = {'schema_version': 1, 'status': 'starting', 'command': command,
              'mission_dir': str(mission_dir), 'map_epoch': config.map_epoch,
              'limits': {'max_wall_seconds': config.max_wall_seconds,
                         'post_terminal_wall_seconds': config.post_terminal_wall_seconds,
                         'max_bytes': config.max_bytes, 'min_free_bytes': config.min_free_bytes},
              'terminal': None, 'reason': None, 'errors': [], 'owner': None,
              'mission_success_inferred': False, 'robot_stop_verified': False,
              'limitations': ['Only the original owned recorder process group is supervised.',
                              'Limits initiate shutdown; cleanup budgets and scheduling/I/O delays add time.',
                              'Byte/free-space limits are polled soft thresholds.',
                              'A terminal file or tail period does not prove robot stop or mission success.']}
    def request_stop(signum, frame):
        if not requested:
            requested.append(signal.Signals(signum).name)
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            old_handlers[signum] = signal.signal(signum, request_stop)
        with (output/'recorder.log').open('xb') as log:
            process = popen(command, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                            start_new_session=True, env=environment)
            leader = proc_stat(process.pid)
            if leader['pgid'] != process.pid or leader['sid'] != process.pid:
                raise OwnershipError('Recorder did not create its own session/group')
            owner = {**{k: leader[k] for k in ('pid', 'pgid', 'sid', 'start_ticks')}, 'token': token}
            report.update(status='recording', owner=owner)
            write_report(output, report)
            while True:
                now = time.monotonic()
                if requested:
                    report['reason'] = 'supervisor_'+requested[0]
                    break
                if process.poll() is not None:
                    report['reason'] = 'recorder_early_exit'
                    break
                if now-started >= config.max_wall_seconds:
                    report['reason'] = 'wall_budget'
                    break
                report['observed_bytes'] = tree_bytes(output)
                report['observed_free_bytes'] = shutil.disk_usage(output).free
                if config.max_bytes is not None and report['observed_bytes'] >= config.max_bytes:
                    report['reason'] = 'max_bytes'
                    break
                if report['observed_free_bytes'] < config.min_free_bytes:
                    report['reason'] = 'min_free_bytes'
                    break
                if report['terminal'] is None:
                    terminal = terminal_result(mission_dir, config.map_epoch)
                    if terminal is not None:
                        terminal['observed_elapsed_wall_seconds'] = now-started
                        terminal['tail_deadline_elapsed_wall_seconds'] = now-started+config.post_terminal_wall_seconds
                        report['terminal'] = terminal
                        write_report(output, report)
                terminal = report['terminal']
                if terminal is not None and now-started >= terminal['tail_deadline_elapsed_wall_seconds']:
                    report['reason'] = 'terminal_tail'
                    break
                time.sleep(min(poll_seconds, max(0., config.max_wall_seconds-(time.monotonic()-started))))
    except Exception as exc:
        report['reason'] = 'supervisor_error'
        report['errors'].append(type(exc).__name__+': '+str(exc))
    finally:
        report['stop_requested_elapsed_wall_seconds'] = time.monotonic()-started
        report['cleanup'] = {'ownership_errors': ['No verified recorder identity'], 'remaining': [],
                             'remaining_check_complete': False, 'forced_kill': False,
                             'terminate_used': False, 'signals': [],
                             'cleanup_budget_seconds': sum(cleanup_graces)}
        try:
            if owner is not None:
                report['cleanup'] = stop_owned(owner, process, graces=cleanup_graces, poll_seconds=poll_seconds)
        except Exception as exc:
            report['errors'].append('Cleanup failed: '+type(exc).__name__+': '+str(exc))
            report['cleanup']['ownership_errors'] = ['Cleanup inspection failed; remaining processes unknown']
        try:
            report['recorder_exit_code'] = None if process is None else process.poll()
            cleanup = report['cleanup']
            report['graceful_recorder_shutdown'] = bool(owner is not None and
                report['reason'] not in ('recorder_early_exit', 'supervisor_error') and
                not report['errors'] and cleanup['remaining_check_complete'] and
                not cleanup['ownership_errors'] and not cleanup['remaining'] and
                not cleanup['forced_kill'] and not cleanup['terminate_used'] and
                report['recorder_exit_code'] == 0)
            report['status'] = 'stopped' if report['graceful_recorder_shutdown'] else 'failed'
            report['elapsed_wall_seconds'] = time.monotonic()-started
            metadata = output/'bag'/'metadata.yaml'
            report['metadata'] = {'exists': metadata.is_file(), 'integrity_verified': False}
            write_report(output, report)
        finally:
            for signum, handler in old_handlers.items():
                signal.signal(signum, handler)
    return report


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--output', required=True, type=Path)
    result.add_argument('--mission-dir', required=True, type=Path)
    result.add_argument('--map-epoch', required=True)
    result.add_argument('--max-wall-seconds', required=True, type=float)
    result.add_argument('--post-terminal-wall-seconds', default=90., type=float)
    result.add_argument('--max-bytes', type=int)
    result.add_argument('--min-free-bytes', default=0, type=int)
    result.add_argument('--qos-profile-overrides-path', type=Path)
    result.add_argument('--max-bag-size', default=256*1024*1024, type=int)
    result.add_argument('--topics', nargs='+', required=True)
    return result


def main(argv=None):
    args = parser().parse_args(argv)
    config = Config(args.output, args.mission_dir, args.map_epoch, args.max_wall_seconds,
                    tuple(args.topics), args.post_terminal_wall_seconds, args.max_bytes,
                    args.min_free_bytes, args.qos_profile_overrides_path, args.max_bag_size)
    try:
        report = supervise(config)
    except (ValueError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(json.dumps({'status': report['status'], 'reason': report['reason'],
                      'report': str(Path(args.output).resolve()/'supervisor.json')}))
    return 0 if report['graceful_recorder_shutdown'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
