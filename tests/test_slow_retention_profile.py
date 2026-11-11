"""Linux-only real-writer regression for the complex-maze wall-time profile.

Three VGA frame requests are queued before polling. A fixture delays one job's
input by 5.5 seconds, then delegates all validation, writes, receipts and
checkpoints to the production worker. This is controlled offline IPC/filesystem
evidence, not a ROS, simulator, sensor-coverage or machine-throughput test.
"""
from dataclasses import fields, replace
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / 'src/limo_evidence_protocol', ROOT / 'src/limo_maze_perception', ROOT / 'tests'):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from limo_evidence_protocol.retention import AcceptedWindow, Policy, canonical, sha
from limo_maze_perception.retention_adapter import ProducerRetention, RuntimePolicy
from test_frame_retention_core import request, sample

WORKER_DIRECTORY = ROOT / 'src/limo_maze_perception/limo_maze_perception'


def vga_sample(sequence):
    record, _, _ = sample(sequence)
    width, height = 640, 480
    rgb = bytes((255, sequence, 0)) * (width * height)
    depth = struct.pack('<f', float(sequence)) * (width * height)
    calibration = record['camera_info']['calibration']
    calibration.update(width=width, height=height,
                       k=[500., 0., 320., 0., 500., 240., 0., 0., 1.],
                       p=[500., 0., 320., 0., 0., 500., 240., 0., 0., 0., 1., 0.])
    record['camera_info']['calibration_id'] = sha(canonical({
        'frame_id': record['camera_info']['frame_id'], **calibration}))
    for name, raw, bytes_per_pixel in (('rgb', rgb, 3), ('depth', depth, 4)):
        record[name].update(width=width, height=height, step=width * bytes_per_pixel,
                            data_bytes=len(raw), data_sha256=sha(raw))
    return record, rgb, depth


def spin(owner, predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        owner.tick(time.monotonic())
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError(owner.snapshot())


@unittest.skipUnless(sys.platform.startswith('linux') and hasattr(os, 'pidfd_open')
                     and hasattr(signal, 'pidfd_send_signal'), 'Linux pidfd required')
class SlowRetentionProfileTests(unittest.TestCase):
    def setUp(self):
        import yaml
        profile = yaml.safe_load((ROOT / 'config/complex_maze.yaml').read_text(encoding='utf-8'))
        self.mission = profile['limo_maze_mission']['ros__parameters']
        detector = profile['red_rgbd_sim_baseline']['ros__parameters']
        self.policy = Policy(**{field.name: detector['retention_' + field.name]
                                for field in fields(Policy) if 'retention_' + field.name in detector})
        self.runtime = RuntimePolicy(**{field.name: detector['retention_' + field.name]
                                       for field in fields(RuntimePolicy)
                                       if 'retention_' + field.name in detector})
        self.tmp = tempfile.TemporaryDirectory(prefix='limo-slow-retention-')
        self.root = Path(self.tmp.name).resolve()
        self.owners, self.unrelated = [], []
        self.addCleanup(self.cleanup)

    def cleanup(self):
        try:
            for owner in self.owners:
                worker = owner.worker
                if worker is not None:
                    if worker.process.poll() is None:
                        worker.signal(signal.SIGKILL)  # Its owned pidfd, never a PID/group search.
                        worker.process.wait(timeout=2.)
                    worker.dispose()
                    owner.worker = None
        finally:
            try:
                for process in self.unrelated:
                    if process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=2.)
                        except subprocess.TimeoutExpired:
                            process.kill()  # Exact unreaped fixture Popen child only.
                            process.wait(timeout=2.)
            finally:
                self.tmp.cleanup()

    def wrapper(self, mode):
        path = self.root / (mode + '-worker.py')
        preamble = ('import json, os, signal, sys, time\nfrom pathlib import Path\n'
                    'root = Path(sys.argv[sys.argv.index("--root") + 1])\n'
                    'sys.path.insert(0, ' + repr(str(WORKER_DIRECTORY)) + ')\n')
        if mode == 'delay':
            body = '''import retention_worker as real_worker
original_read = real_worker.read_exact
reads = 0
header = None
marker = root / '.fixture-delay.json'
def delayed_read(stream, size):
    global reads, header
    if reads == 2 and header['job'] is not None and not marker.exists():
        evidence = {'pid': os.getpid(), 'job_id': header['job']['request_id'],
                    'started_wall': time.monotonic(), 'finished_wall': None}
        marker.write_text(json.dumps(evidence))
        print('fixture_delay_start', file=sys.stderr, flush=True)
        time.sleep(5.5)
        evidence['finished_wall'] = time.monotonic()
        marker.write_text(json.dumps(evidence))
        print('fixture_delay_end', file=sys.stderr, flush=True)
    raw = original_read(stream, size)
    if reads == 1:
        header = json.loads(raw)
    reads += 1
    return raw
real_worker.read_exact = delayed_read
raise SystemExit(real_worker.main())
'''
        elif mode == 'stalled':
            # The test queues before its first tick, so the first child is a
            # job writer. Later checkpoint children must run without a stall.
            body = '''marker = root / '.fixture-stalled.json'
if not marker.exists():
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    marker.write_text(json.dumps({'pid': os.getpid(), 'started_wall': time.monotonic()}))
    print('fixture_stalled_without_reading_stdin', file=sys.stderr, flush=True)
    time.sleep(30)
    raise SystemExit(99)
import retention_worker as real_worker
raise SystemExit(real_worker.main())
'''
        else:
            raise ValueError(mode)
        path.write_text(preamble + body, encoding='utf-8')
        return path

    def owner(self, mode, runtime=None):
        owner = ProducerRetention(self.root / ('run-' + mode), 'session-a',
                                  policy=self.policy, runtime=runtime or self.runtime,
                                  worker_path=self.wrapper(mode))
        self.owners.append(owner)
        owner.on_epoch('epoch-a', True, time.monotonic())
        return owner

    def queue_three(self, owner):
        requests, expected = [], {}
        for sequence in range(1, 4):
            record, rgb, depth = vga_sample(sequence)
            self.assertTrue(owner.offer(record, rgb, depth, time.monotonic())['available'])
            payload = request(record)
            self.assertEqual(owner.request(payload, time.monotonic())['status'], 'queued')
            requests.append(payload)
            expected[payload['retention_id']] = {'rgb.bin': rgb, 'depth.bin': depth}
        self.assertIsNone(owner.worker)
        self.assertEqual([result['status'] for result in owner.results()], ['queued'] * 3)
        return requests, expected

    def assert_closed_checkpoint(self, owner, report):
        self.assertTrue(report['shutdown']['owned_writer_exit_confirmed'], report)
        self.assertTrue(report['shutdown']['final_checkpoint_persisted'], report)
        self.assertIsNone(owner.worker)
        checkpoint = json.loads((owner.output / 'checkpoint.json').read_bytes())
        self.assertTrue(checkpoint['closed'])
        self.assertFalse(checkpoint['core']['writer_pending'])
        return checkpoint

    def test_three_queued_vga_frames_survive_one_input_delay_beyond_five_seconds(self):
        self.assertEqual(self.mission['inspection_required_frames'], 3)
        self.assertEqual(self.runtime.worker_timeout_s, 15.)
        self.assertEqual(self.policy.request_timeout_s, 60.)
        owner = self.owner('delay')
        requests, expected = self.queue_three(owner)
        started = time.monotonic()
        spin(owner, lambda: owner.fault is not None or (len(owner.results()) == 3
             and all(result['status'] == 'stored' for result in owner.results())),
             timeout=self.policy.request_timeout_s)
        elapsed = time.monotonic() - started
        self.assertLess(elapsed, self.policy.request_timeout_s)
        self.assertLess(elapsed, self.mission['inspection_timeout_s'])
        self.assertIsNone(owner.fault, owner.snapshot())
        evidence = json.loads((owner.output / '.fixture-delay.json').read_bytes())
        self.assertGreaterEqual(evidence['finished_wall'] - evidence['started_wall'], 5.5)
        self.assertEqual(evidence['job_id'], requests[0]['retention_id'])
        delayed = [entry for entry in owner.writer_history if 'fixture_delay_start' in entry['stderr']]
        self.assertEqual(len(delayed), 1)
        self.assertIn('fixture_delay_end', delayed[0]['stderr'])
        self.assertEqual(delayed[0]['pid'], evidence['pid'])
        self.assertIsNone(delayed[0]['stop_reason'])
        self.assertGreaterEqual(delayed[0]['input_transfer']['stdin_closed_wall']
                                - delayed[0]['started_wall'], 5.5)
        results = owner.results()
        self.assertTrue(AcceptedWindow.freeze(requests).evaluate(results)['retention_satisfied'])
        for result in results:
            self.assertTrue(result['checkpoint_persisted'])
            self.assertFalse(result['writer_owned'])
            self.assertLess(result['completed_wall'] - result['requested_wall'], self.policy.request_timeout_s)
            directory = owner.output / result['receipt']['directory']
            for name, metadata in result['receipt']['files'].items():
                raw = (directory / name).read_bytes()
                self.assertEqual((len(raw), sha(raw)), (metadata['bytes'], metadata['sha256']))
            for name, raw in expected[result['retention_id']].items():
                self.assertEqual((directory / name).read_bytes(), raw)
        report = owner.shutdown()
        checkpoint = self.assert_closed_checkpoint(owner, report)
        self.assertEqual([result['status'] for result in checkpoint['core']['results']], ['stored'] * 3)
        self.assertIsNone(report['fault'])
        print('SLOW_PROFILE_STORED ' + json.dumps({'elapsed_s': elapsed, 'stored': 3,
                                                'delay_s': evidence['finished_wall'] - evidence['started_wall'],
                                                'shutdown': report['shutdown']}))

    def test_nonreading_worker_still_times_out_and_only_its_owned_child_is_reaped(self):
        injected = replace(self.runtime, worker_timeout_s=2., term_grace_s=.05,
                           kill_grace_s=.05, shutdown_timeout_s=6.)
        owner = self.owner('stalled', runtime=injected)
        requests, _ = self.queue_three(owner)
        unrelated = subprocess.Popen([sys.executable, '-S', '-B', '-c', 'import time; time.sleep(30)'],
                                     stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
        self.unrelated.append(unrelated)
        owner.tick(time.monotonic())
        self.assertIsNotNone(owner.worker)
        child = owner.worker.process
        identity = owner.worker.snapshot()
        self.assertEqual(identity['job_id'], requests[0]['retention_id'])
        spin(owner, lambda: owner.worker is not None
             and b'fixture_stalled_without_reading_stdin' in owner.worker.errors, timeout=2.)
        self.assertGreater(owner.worker.snapshot()['input_transfer']['remaining_bytes'], 0)
        spin(owner, lambda: owner.closed and owner.worker is None
             and owner.revision == owner.persisted_revision, timeout=8.)
        self.assertEqual(owner.fault, 'writer_wall_timeout')
        self.assertEqual(child.poll(), -signal.SIGKILL)
        self.assertIsNone(unrelated.poll())
        jobs = [entry for entry in owner.writer_history if entry['job_id'] is not None]
        self.assertEqual(len(jobs), 1)
        for key in ('pid', 'operation_id', 'owner_token'):
            self.assertEqual(jobs[0][key], identity[key])
        self.assertTrue(jobs[0]['forced_kill'])
        self.assertFalse(jobs[0]['termination_unconfirmed'])
        self.assertEqual(jobs[0]['stop_reason'], 'writer_wall_timeout')
        self.assertEqual([result['status'] for result in owner.results()], ['failed'] * 3)
        self.assertTrue(all(result['receipt'] is None for result in owner.results()))
        self.assertFalse(AcceptedWindow.freeze(requests).evaluate(owner.results())['retention_satisfied'])
        self.assertEqual(list(owner.output.rglob('rgb.bin')), [])
        report = owner.shutdown()
        checkpoint = self.assert_closed_checkpoint(owner, report)
        self.assertEqual(checkpoint['fault'], 'writer_wall_timeout')
        self.assertTrue(all(result['status'] == 'failed' for result in checkpoint['core']['results']))
        self.assertIsNone(unrelated.poll())
        print('SLOW_PROFILE_TIMEOUT ' + json.dumps({'injected_worker_timeout_s': injected.worker_timeout_s,
                                                 'owned_pid': child.pid, 'returncode': child.poll(),
                                                 'stored': 0, 'shutdown': report['shutdown']}))


if __name__ == '__main__':
    unittest.main()
