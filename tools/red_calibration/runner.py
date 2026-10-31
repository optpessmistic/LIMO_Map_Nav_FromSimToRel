#!/usr/bin/python3
"""Finite sensor-only calibration batch. Default is plan-only, not execution."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
CAPTURE = HERE / 'capture_red_samples.py'
MAX_JOBS = 6  # isolation, sim, probe, native-before, capture, native-after
GRACES = (20., 5., 3.)
CLEANUP_RESERVE = MAX_JOBS * sum(GRACES)
LABEL = re.compile(r'[A-Za-z0-9_-]{1,80}\Z')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    data = json.dumps(value, indent=2, allow_nan=False) + '\n'
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


def commands(case, directory, egl=False):
    """Truth is absent from every capture argument. Native snapshots are separate."""
    native = ['/usr/bin/python3', str(ROOT / 'src/limo_sim_bringup/scripts/inspect_scene_once.py'),
              '--execute', '--world', case['world_name'], '--name', 'limo']
    return {
        'isolation': ['/usr/bin/python3', str(HERE / 'isolation_probe.py'), '--execute',
                      '--output', str(directory / 'isolation.json')],
        'sim': ['ros2', 'launch', 'limo_sim_bringup', 'sim.launch.py',
                'world:=' + case['world'], 'headless:=true',
                'headless_rendering:=' + str(egl).lower(),
                'spawn_artifacts:=' + str(directory / 'spawn')],
        'probe': ['/usr/bin/python3', str(ROOT / 'scripts/check_sim_runtime.py'),
                  '--timeout', '120', '--max-initial-sim-time', '30',
                  '--output', str(directory / 'sim-runtime.json')],
        'native-before': native + ['--output', str(directory / 'native-before.json')],
        'capture': ['/usr/bin/python3', str(CAPTURE), '--execute', '--registered-depth',
                    '--case-id', case['id'], '--pairs', '6', '--timeout', '60',
                    '--pair-wait', '2', '--max-bytes', str(256*1024**2),
                    '--reference-frame', 'odom', '--base-frame', 'base_footprint',
                    '--output', str(directory / 'capture')],
        'native-after': native + ['--output', str(directory / 'native-after.json')],
    }


def build_plan(manifest, selected, all_cases, output, case_wall=300., batch_wall=7200., domain=77, egl=False):
    manifest, output = Path(manifest).resolve(strict=True), Path(output).resolve()
    if output.exists():
        raise ValueError('Output must not exist; prior evidence is never overwritten')
    if (not math.isfinite(case_wall) or not 30 <= case_wall <= 600 or
            not math.isfinite(batch_wall) or not CLEANUP_RESERVE+30 <= batch_wall <= 14400):
        raise ValueError('case wall must be 30..600; batch wall must be 198..14400 seconds')
    if type(domain) is not int or not 0 <= domain <= 232:
        raise ValueError('Invalid ROS domain')
    data = json.loads(manifest.read_text())
    if data.get('schema_version') != 1 or not isinstance(data.get('cases'), list):
        raise ValueError('Expected calibration manifest schema 1')
    index = {}
    for entry in data['cases']:
        identity = entry.get('id')
        if not isinstance(identity, str) or not LABEL.fullmatch(identity) or identity in index:
            raise ValueError('Invalid or duplicate fixture ID')
        index[identity] = entry
    if bool(selected) == bool(all_cases):
        raise ValueError('Choose --case ID (repeatable) or --all')
    names = list(index) if all_cases else selected
    if not names or len(names) > 12 or len(set(names)) != len(names) or any(name not in index for name in names):
        raise ValueError('Choose 1..12 distinct known fixture IDs')
    cases = []
    for identity in names:
        entry = index[identity]
        world = (manifest.parent / entry['world']).resolve(strict=True)
        if not world.is_relative_to(manifest.parent) or not world.is_file():
            raise ValueError('World must remain within the manifest directory')
        sha = digest(world)
        if sha != entry.get('world_sha256'):
            raise ValueError('World source hash mismatch: ' + identity)
        worlds = ET.parse(world).getroot().findall('world')
        if len(worlds) != 1 or not LABEL.fullmatch(worlds[0].get('name', '')):
            raise ValueError('Expected exactly one plainly named world')
        case = {'id': identity, 'world': str(world), 'world_name': worlds[0].get('name'),
                'world_sha256': sha}
        case['commands'] = commands(case, output / identity, egl)
        cases.append(case)
    # Only declared source provenance, never offline_truth files or geometry labels.
    reference = data.get('input_reference', {})
    return {'schema_version': 1, 'status': 'planned_not_run', 'output': str(output),
            'manifest': str(manifest), 'manifest_sha256': digest(manifest), 'cases': cases,
            'reference_source_paths': reference.get('source_paths', {}),
            'reference_source_sha256': reference.get('source_sha256', {}),
            'case_active_wall_seconds': case_wall, 'batch_wall_seconds': batch_wall,
            'cleanup_reserve_per_case_seconds': CLEANUP_RESERVE,
            'deadline_scope': 'Case deadline begins cleanup. Batch deadline includes reserved cleanup; OS scheduling or blocked OS I/O can overrun it. No hard OS-level lifetime guarantee.',
            'ros_domain': domain, 'headless_rendering': egl, 'pairs_per_case': 6,
            'truth_scoring': 'not_run; separate offline work only', 'effective_range_claimed': False,
            'stationarity_verified': False,
            'geometry_limit': 'Scene snapshots contain native IDs, not actual poses. Nominal camera world z is not a measured settled height; map/odom TF heights use different origins and are not world-truth heights.',
            'selection_policy': 'first six distinct fresh synchronized pairs, including detector/input rejections; no retry based on detections',
            'motion_authorized': False}


def case_deadline(now, case_wall, batch_deadline):
    deadline = min(now + case_wall, batch_deadline - CLEANUP_RESERVE)
    if deadline <= now:
        raise TimeoutError('Insufficient remaining batch budget including cleanup reserve')
    return deadline


def cleanup_confirmed(stops, launched):
    return bool(launched) and set(stops) == set(launched) and all(
        item.get('remaining_check_complete') is True and not item.get('remaining')
        and not item.get('ownership_errors') and not item.get('error') for item in stops.values())


class OwnedJobs:
    """Own only gate-created sessions; borrow audited pidfd signaling, never killpg."""
    def __init__(self, directory, environment, token):
        sys.path.insert(0, str(ROOT / 'scripts'))
        from record_bounded import TOKEN_KEY, proc_stat, stop_owned
        self.token_key, self.proc_stat, self.stop_owned = TOKEN_KEY, proc_stat, stop_owned
        self.directory, self.environment, self.token, self.jobs = directory, environment, token, {}

    def start(self, name, command):
        environment = dict(self.environment, **{self.token_key: self.token,
                           'ROS_LOG_DIR': str(self.directory / 'ros_logs' / name)})
        read_fd, write_fd = os.pipe()
        bootstrap = ('import os,sys; fd=int(sys.argv[1]); ready=os.read(fd,1); os.close(fd); '
                     'os.execvpe(sys.argv[2],sys.argv[2:],os.environ) if ready==b"1" else sys.exit(125)')
        try:
            with (self.directory / (name + '.log')).open('xb') as log:
                process = subprocess.Popen(['/usr/bin/python3', '-c', bootstrap, str(read_fd), *command],
                    stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True, pass_fds=(read_fd,), cwd=ROOT, env=environment)
            self.jobs[name] = (process, None)
            row = self.proc_stat(process.pid)
            if row['pgid'] != process.pid or row['sid'] != process.pid:
                raise RuntimeError('Child session ownership not established')
            owner = {key: row[key] for key in ('pid', 'pgid', 'sid', 'start_ticks')}
            owner['token'] = self.token
            self.jobs[name] = (process, owner)
            write(self.directory / ('process-' + name + '.json'), {'owner': owner, 'command': command})
            os.write(write_fd, b'1')
        finally:
            os.close(read_fd)
            os.close(write_fd)
        return process

    def wait(self, name, deadline):
        process = self.jobs[name][0]
        while process.poll() is None and time.monotonic() < deadline:
            if name != 'sim' and 'sim' in self.jobs and self.jobs['sim'][0].poll() is not None:
                raise RuntimeError('Simulator exited during ' + name)
            time.sleep(.1)
        if process.poll() is None:
            raise TimeoutError(name + ' exceeded wall deadline')
        return process.returncode

    def close(self):
        stops = {}
        for name, (process, owner) in reversed(list(self.jobs.items())):
            try:
                if owner is None:
                    # EOF on the closed gate prevents exec, but uncertain ownership
                    # cannot authorize signals or another case.
                    process.wait(timeout=3)
                    stops[name] = {'launch_gate_not_released': True, 'exit_code': process.returncode,
                                   'remaining_check_complete': False}
                else:
                    result = self.stop_owned(owner, process, graces=GRACES)
                    result['exit_code'] = process.poll()
                    stops[name] = result
            except Exception as exc:
                stops[name] = {'error': type(exc).__name__ + ': ' + str(exc),
                               'remaining_check_complete': False}
        return stops


def run_case(case, plan, batch_deadline):
    directory = Path(plan['output']) / case['id']
    directory.mkdir()
    start = time.monotonic()
    deadline = case_deadline(start, plan['case_active_wall_seconds'], batch_deadline)
    token = uuid.uuid4().hex
    partition = 'limo_red_cal_' + token
    environment = dict(os.environ, ROS_DOMAIN_ID=str(plan['ros_domain']),
                       ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', GZ_PARTITION=partition)
    owned = OwnedJobs(directory, environment, token)
    report = {'case_id': case['id'], 'errors': [], 'stops': {}, 'exit_codes': {},
              'commands': case['commands'], 'world_sha256': case['world_sha256'],
              'environment': {'ROS_DOMAIN_ID': plan['ros_domain'], 'GZ_PARTITION': partition,
                              'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'},
              'capture_passed': False, 'truth_scoring_performed': False,
              'stationarity_verified': False,
              'motion_started': False, 'active_budget_wall_s': deadline-start,
              'cleanup_reserve_wall_s': CLEANUP_RESERVE}

    def execute(name, seconds, require_zero=True):
        if time.monotonic() >= deadline:
            raise TimeoutError('Case/batch active deadline before ' + name)
        owned.start(name, case['commands'][name])
        code = owned.wait(name, min(deadline, time.monotonic()+seconds))
        report['exit_codes'][name] = code
        if require_zero and code != 0:
            raise RuntimeError(name + ' exited ' + str(code))
        return code

    def read(name):
        return json.loads((directory / name).read_text())

    try:
        write(directory / 'result.json', dict(report, status='running'))
        if digest(case['world']) != case['world_sha256']:
            raise RuntimeError('World changed since planning')
        for path, sha in plan.get('source_sha256', {}).items():
            if digest(path) != sha:
                raise RuntimeError('Runner/runtime source changed since planning: ' + path)
        execute('isolation', 10)
        if not read('isolation.json').get('passed'):
            raise RuntimeError('ROS domain is not isolated')
        owned.start('sim', case['commands']['sim'])
        execute('probe', 145)
        if not read('sim-runtime.json').get('passed'):
            raise RuntimeError('Sensor integrity probe failed')
        report['sensor_passed'] = True
        spawn = read('spawn/result/report.json')
        if not spawn.get('success') or spawn.get('request_attempts') != 1:
            raise RuntimeError('Single spawn acknowledgement missing')
        report['spawn_acknowledged_once'] = True
        execute('native-before', 45)
        first = read('native-before.json')
        if not first.get('passed') or not first.get('unique_instance_verified_in_snapshot'):
            raise RuntimeError('Native model uniqueness not verified')
        capture_code = execute('capture', 75, require_zero=False)
        capture = read('capture/summary.json')
        report['capture_passed'] = (capture_code == 0 and capture.get('passed') is True
                                    and capture.get('saved_pairs') == 6)
        report['capture_summary'] = {key: capture.get(key) for key in
            ('passed', 'collection_complete', 'all_samples_valid', 'source_unchanged',
             'errors', 'requested_pairs', 'saved_pairs', 'selected_pairs')}
        execute('native-after', 45)
        last = read('native-after.json')
        report['same_native_model_in_two_snapshots'] = bool(last.get('passed')
            and last.get('unique_instance_verified_in_snapshot')
            and first['target_models'] == last['target_models'])
        report['native_snapshot_scope'] = 'Model ID/count only; neither physical stationarity nor camera world pose is verified.'
        if not report['same_native_model_in_two_snapshots']:
            raise RuntimeError('Native identity changed across capture')
        if not report['capture_passed']:
            report['errors'].append('Capture validity failed; samples retained, no resampling by detector outcome')
    except (Exception, KeyboardInterrupt) as exc:
        report['errors'].append(type(exc).__name__ + ': ' + str(exc))
        report['interrupted'] = isinstance(exc, KeyboardInterrupt)
    finally:
        old = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
        for sig in old:
            signal.signal(sig, signal.SIG_IGN)
        try:
            report['stops'] = owned.close()
            report['all_owned_groups_empty'] = cleanup_confirmed(report['stops'], owned.jobs)
            report['forced_kill_used'] = any(s.get('forced_kill') for s in report['stops'].values())
            report['elapsed_wall_s'] = time.monotonic()-start
            try:
                report['source_unchanged'] = (digest(case['world']) == case['world_sha256'] and
                    all(digest(path) == sha for path, sha in plan.get('source_sha256', {}).items()))
            except OSError as exc:
                report['source_unchanged'] = False
                report['errors'].append('Source read failed after cleanup: ' + str(exc))
            report['status'] = 'captured' if (report['capture_passed'] and not report['errors']
                and report['all_owned_groups_empty'] and not report['forced_kill_used']
                and report['source_unchanged']) else 'failed_or_incomplete'
            write(directory / 'result.json', report)
        finally:
            for sig, handler in old.items():
                signal.signal(sig, handler)
    return report


def run_batch(plan, *, case_runner=run_case, clock=time.monotonic):
    output = Path(plan['output'])
    output.mkdir(parents=True, exist_ok=False)
    start = clock()
    deadline = start + plan['batch_wall_seconds']
    report = {'schema_version': 1, 'status': 'running', 'cases': [], 'errors': [],
              'truth_scoring_performed': False, 'motion_authorized': False}
    write(output / 'plan.json', plan)
    try:
        for case in plan['cases']:
            case_deadline(clock(), plan['case_active_wall_seconds'], deadline)
            result = case_runner(case, plan, deadline)
            report['cases'].append(result)
            write(output / 'batch.json', report)
            if not result.get('all_owned_groups_empty'):
                raise RuntimeError('Cleanup unconfirmed: refusing subsequent cases')
            if result.get('status') != 'captured':
                raise RuntimeError('Case failed: batch stops without retrying/replacing selected samples')
    except (Exception, KeyboardInterrupt) as exc:
        report['errors'].append(type(exc).__name__ + ': ' + str(exc))
    finally:
        report['elapsed_wall_s'] = clock()-start
        if clock() > deadline:
            report['errors'].append('Batch wall deadline exceeded during cleanup or scheduling/I/O')
        report['completed_all_cases'] = len(report['cases']) == len(plan['cases']) and not report['errors']
        report['status'] = 'capture_batch_complete' if report['completed_all_cases'] else 'stopped_incomplete'
        write(output / 'batch.json', report)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    choice = parser.add_mutually_exclusive_group(required=True)
    choice.add_argument('--case', action='append', default=[])
    choice.add_argument('--all', action='store_true')
    parser.add_argument('--case-wall', type=float, default=300.)
    parser.add_argument('--batch-wall', type=float, default=7200.)
    parser.add_argument('--domain', type=int, default=77)
    parser.add_argument('--egl', action='store_true')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    plan = build_plan(args.manifest, args.case, args.all, args.output, args.case_wall, args.batch_wall, args.domain, args.egl)
    source_paths = [Path(__file__), HERE/'env.sh', HERE/'isolation_probe.py', CAPTURE,
        CAPTURE.with_name('capture_core.py'), ROOT/'scripts/record_bounded.py',
        ROOT/'scripts/check_sim_runtime.py', ROOT/'scripts/odom_integrity.py',
        ROOT/'src/limo_sim_bringup/launch/sim.launch.py',
        ROOT/'src/limo_sim_bringup/scripts/spawn_once.py',
        ROOT/'src/limo_sim_bringup/scripts/inspect_scene_once.py',
        ROOT/'src/limo_maze_perception/limo_maze_perception/core.py',
        ROOT/'src/limo_maze_perception/limo_maze_perception/evidence.py']
    plan['source_sha256'] = {str(path): digest(path) for path in source_paths}
    if not args.execute:
        print(json.dumps(plan, indent=2, allow_nan=False))
        return 0
    if sys.platform != 'linux' or not hasattr(os, 'pidfd_open'):
        raise RuntimeError('Execution requires Linux /proc and pidfd support')
    from ament_index_python.packages import get_package_prefix
    sim_prefix = Path(get_package_prefix('limo_sim_bringup')).resolve()
    if sim_prefix != (ROOT/'install/limo_sim_bringup').resolve():
        raise RuntimeError('Current checkout sim bringup overlay is not selected; use env.sh')
    for key, path in plan['reference_source_paths'].items():
        if digest(path) != plan['reference_source_sha256'].get(key):
            raise RuntimeError('Fixture reference source changed: ' + key)
    def interrupt(signum, _frame):
        raise KeyboardInterrupt(signal.Signals(signum).name)
    old = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        for sig in old:
            signal.signal(sig, interrupt)
        report = run_batch(plan)
    finally:
        for sig, handler in old.items():
            signal.signal(sig, handler)
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report['completed_all_cases'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
