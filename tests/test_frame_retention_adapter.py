"""Real owned stdlib child tests and extracted source-hook checks; no ROS."""
import ast
import copy
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

REPO = Path(__file__).resolve().parents[1]
TESTS = Path(__file__).resolve().parent
SOURCE_ROOTS = [str(REPO / 'src/limo_evidence_protocol'), str(REPO / 'src/limo_maze_perception'), str(TESTS)]
for root in SOURCE_ROOTS:
    if root not in sys.path:
        sys.path.insert(0, root)

from limo_evidence_protocol.retention import Policy, canonical, sha
from limo_maze_perception.retention_adapter import ProducerRetention, RuntimePolicy
from test_frame_retention_core import request, sample

HERE = REPO / 'src/limo_maze_perception/limo_maze_perception'


def spin(owner, predicate, timeout=12.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        owner.tick(time.monotonic())
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError(owner.snapshot())


def cleanup_signal_harness(process, identity_path, worker_path):
    """Bounded fallback for this exact harness and its recorded test worker."""
    identities = {}

    def remember():
        if identity_path.exists():
            entry = json.loads(identity_path.read_text())
            identities[(entry['pid'], entry['start_ticks'])] = entry
        if process.poll() is not None:
            return
        # Covers a parent interrupted between Popen and its identity write.
        children = Path('/proc', str(process.pid), 'task', str(process.pid), 'children')
        try:
            pids = [int(value) for value in children.read_text().split()]
        except FileNotFoundError:
            pids = []
        for pid in pids:
            directory = Path('/proc', str(pid))
            try:
                fields = (directory/'stat').read_text().rsplit(')', 1)[1].split()
                if fields[0] in ('Z', 'X'):
                    continue
                argv = (directory/'cmdline').read_bytes().split(b'\0')
                env = (directory/'environ').read_bytes().split(b'\0')
                token = next((value.split(b'=', 1)[1].decode() for value in env
                              if value.startswith(b'LIMO_RETENTION_WRITER_TOKEN=')), None)
                if int(fields[1]) != process.pid or str(worker_path).encode() not in argv or not token:
                    raise AssertionError('Refusing unrecognized harness child')
                entry = dict(pid=pid, start_ticks=int(fields[19]), token=token)
                identities[(pid, entry['start_ticks'])] = entry
            except FileNotFoundError:
                pass

    try:
        remember()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        try:
            output = process.communicate(timeout=10.)
        except subprocess.TimeoutExpired:
            remember()
            process.kill()  # Exact unreaped Popen child, never a process group.
            output = process.communicate(timeout=2.)
        remember()
        return output
    finally:
        try:
            for entry in identities.values():
                directory = Path('/proc', str(entry['pid']))
                try:
                    fd = os.pidfd_open(entry['pid'])
                except ProcessLookupError:
                    continue
                try:
                    fields = (directory/'stat').read_text().rsplit(')', 1)[1].split()
                    if int(fields[19]) != entry['start_ticks'] or fields[0] in ('Z', 'X'):
                        continue
                    argv = (directory/'cmdline').read_bytes().split(b'\0')
                    env = (directory/'environ').read_bytes().split(b'\0')
                    if (str(worker_path).encode() not in argv or
                            ('LIMO_RETENTION_WRITER_TOKEN='+entry['token']).encode() not in env or
                            directory.stat().st_uid != os.geteuid()):
                        raise AssertionError('Refusing changed worker identity')
                    signal.pidfd_send_signal(fd, signal.SIGKILL)
                    if not select.select([fd], [], [], 2.)[0]:
                        raise AssertionError('Known fixture worker exit unconfirmed')
                except (FileNotFoundError, ProcessLookupError):
                    pass
                finally:
                    os.close(fd)
        finally:
            for stream in (process.stdin, process.stdout, process.stderr):
                if stream is not None:
                    stream.close()


@unittest.skipUnless(hasattr(os, 'pidfd_open') and hasattr(signal, 'pidfd_send_signal'), 'Linux pidfd required')
class AdapterTests(unittest.TestCase):
    def setUp(self):
        env = patch.dict(os.environ, {'PYTHONPATH': os.pathsep.join(SOURCE_ROOTS + [os.environ.get('PYTHONPATH', '')])})
        env.start()
        self.addCleanup(env.stop)
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.owners = []
        self.addCleanup(self.cleanup)

    def cleanup(self):
        for owner in self.owners:
            if owner.worker and owner.worker.process.poll() is None:
                owner.worker.signal(signal.SIGKILL)
                owner.worker.process.wait(timeout=2.)
            if owner.worker:
                owner.worker.dispose()
                owner.worker = None
        self.tmp.cleanup()

    def owner(self, **kwargs):
        owner = ProducerRetention(self.root / ('run-' + str(len(self.owners))), 'session-a',
            runtime=kwargs.pop('runtime', RuntimePolicy()), **kwargs)
        self.owners.append(owner)
        owner.on_epoch('epoch-a', True, time.monotonic())
        return owner

    def queue(self, owner, seq=1):
        record, rgb, depth = sample(seq)
        self.assertTrue(owner.offer(record, rgb, depth, time.monotonic())['available'])
        req = request(record)
        self.assertEqual(owner.request(req, time.monotonic())['status'], 'queued')
        return req, rgb, depth

    def test_roundtrip_durable_gate_and_source_padding_no_idle_checkpoint_loop(self):
        owner = self.owner()
        req, rgb, depth = self.queue(owner)
        original = bytes(rgb)
        rgb[0] = 0
        owner.tick(time.monotonic())
        self.assertTrue(owner.core.result(req['retention_id'])['writer_owned'])
        spin(owner, lambda: owner.results()[0]['status'] == 'stored')
        result = owner.results()[0]
        self.assertTrue(result['checkpoint_persisted'])
        directory = owner.output / result['receipt']['directory']
        self.assertEqual((directory/'rgb.bin').read_bytes(), original)
        for name, entry in result['receipt']['files'].items():
            raw = (directory/name).read_bytes()
            self.assertEqual((len(raw), sha(raw)), (entry['bytes'], entry['sha256']))
        checkpoint = json.loads((owner.output/'checkpoint.json').read_bytes())
        self.assertEqual(checkpoint['core']['results'][0]['status'], 'stored')
        self.assertFalse(checkpoint['core']['writer_pending'])
        revision = owner.revision
        for _ in range(10):
            owner.tick(time.monotonic())
        self.assertIsNone(owner.worker)
        self.assertEqual(owner.revision, revision)
        report = owner.shutdown()
        self.assertTrue(report['shutdown']['final_checkpoint_persisted'])
        self.assertTrue(json.loads((owner.output/'checkpoint.json').read_bytes())['closed'])

    def test_ttl_failure_persists_and_same_id_never_turns_green(self):
        owner = self.owner(policy=Policy(ttl_s=.02))
        record, rgb, depth = sample()
        owner.offer(record, rgb, depth, time.monotonic())
        time.sleep(.03)
        req = request(record)
        self.assertEqual(owner.request(req, time.monotonic())['reason'], 'cache_miss')
        owner.offer(*sample(2), time.monotonic())
        self.assertEqual(owner.request(req, time.monotonic())['reason'], 'cache_miss')
        report = owner.shutdown()
        self.assertTrue(report['shutdown']['final_checkpoint_persisted'])
        saved = json.loads((owner.output/'checkpoint.json').read_bytes())
        self.assertEqual(saved['core']['results'][0]['reason'], 'cache_miss')

    def test_real_worker_ignores_ambient_site_packages_and_python_startup_overrides(self):
        poison = self.root/'ambient-python'
        poison.mkdir()
        marker = self.root/'ambient-python-executed'
        poisoned_import = ('from pathlib import Path\nPath('+repr(str(marker))+').write_text("executed")\n'
                           'raise RuntimeError("ambient Python imports must not run")\n')
        (poison/'sitecustomize.py').write_text(poisoned_import)
        (poison/'usercustomize.py').write_text(poisoned_import)
        (poison/'limo_evidence_protocol').mkdir()
        (poison/'limo_evidence_protocol/__init__.py').write_text(poisoned_import)
        owner = self.owner()
        req, rgb, _ = self.queue(owner)
        expected_rgb = bytes(rgb)
        with patch.dict(os.environ, {'PYTHONPATH': str(poison), 'PYTHONHOME': str(poison/'not-a-python-home'),
                'PYTHONINSPECT': '1', 'PYTHONMALLOC': 'invalid-allocator',
                'PYTHONUSERBASE': str(poison), 'PYTHONSTARTUP': str(poison/'sitecustomize.py')}):
            spin(owner, lambda: owner.results()[0]['status'] == 'stored')
            result = owner.results()[0]
            self.assertTrue(result['checkpoint_persisted'])
            self.assertEqual((owner.output/result['receipt']['directory']/'rgb.bin').read_bytes(), expected_rgb)
            report = owner.shutdown()
        self.assertIsNone(report['fault'])
        self.assertTrue(report['shutdown']['owned_writer_exit_confirmed'])
        self.assertTrue(report['shutdown']['final_checkpoint_persisted'])
        self.assertFalse(marker.exists())

    def test_global_admission_fault_is_durable_not_reset_by_epoch(self):
        owner = self.owner(policy=Policy(max_requests=1))
        req, _, _ = self.queue(owner)
        other = copy.deepcopy(req); other['retention_id'] = 'over-budget'
        result = owner.request(other, time.monotonic())
        self.assertFalse(result['recorded'])
        self.assertTrue(result['admission_closed'])
        core = owner.core
        owner.on_epoch('epoch-b', True, time.monotonic())
        self.assertIs(owner.core, core)
        owner.shutdown()
        saved = json.loads((owner.output/'checkpoint.json').read_bytes())
        self.assertTrue(saved['core']['admission']['closed'])
        self.assertEqual(saved['core']['map_epoch'], 'epoch-a')

    def test_late_source_conflict_overrides_durable_stored(self):
        owner = self.owner()
        req, _, _ = self.queue(owner)
        spin(owner, lambda: owner.results()[0]['status'] == 'stored')
        record, rgb, depth = sample()
        rgb[0] = 0; record['rgb']['data_sha256'] = sha(rgb)
        self.assertEqual(owner.offer(record, rgb, depth, time.monotonic())['reason'], 'source_identity_conflict')
        self.assertEqual(owner.results()[0]['status'], 'failed')
        self.assertFalse(owner.results()[0]['checkpoint_persisted'])
        owner.shutdown()
        saved = json.loads((owner.output/'checkpoint.json').read_bytes())
        self.assertEqual(saved['core']['results'][0]['reason'], 'source_identity_conflict')

    def test_epoch_change_keeps_owned_job_and_cannot_receive_late_success(self):
        owner = self.owner()
        req, _, _ = self.queue(owner)
        owner.tick(time.monotonic())
        original = owner.core
        owner.on_epoch('epoch-b', True, time.monotonic())
        self.assertIs(owner.core, original)
        self.assertTrue(owner.core.result(req['retention_id'])['writer_owned'])
        report = owner.shutdown()
        self.assertTrue(report['shutdown']['owned_writer_exit_confirmed'])
        self.assertEqual(owner.results()[0]['status'], 'failed')
        self.assertEqual(owner.core.epoch, 'epoch-a')

    def fake_worker(self):
        path = self.root/'ignore-worker.py'
        path.write_text('''import sys, signal, time, json\nfrom pathlib import Path\n'''+
            'sys.path.insert(0, '+repr(str(HERE))+')\n'+
            '''from retention_worker import read_exact, save_checkpoint\nsize=int.from_bytes(read_exact(sys.stdin.buffer,8),'big')\nh=json.loads(read_exact(sys.stdin.buffer,size))\ncp=read_exact(sys.stdin.buffer,h['checkpoint_bytes'])\nrest=sys.stdin.buffer.read()\nif h['job']:\n signal.signal(signal.SIGTERM, signal.SIG_IGN)\n print('ready',file=sys.stderr,flush=True)\n time.sleep(30)\nelse:\n root=Path(sys.argv[sys.argv.index('--root')+1])\n save_checkpoint(root,cp)\n print(json.dumps({'ok':True,'operation_id':h['operation_id'],'receipt_b64':None}),flush=True)\n''')
        return path

    def test_ignored_term_escalates_only_owned_child_and_final_failure_checkpoint(self):
        owner = self.owner(worker_path=self.fake_worker(), runtime=RuntimePolicy(
            worker_timeout_s=5., term_grace_s=.05, kill_grace_s=.05, shutdown_timeout_s=2.))
        self.queue(owner)
        unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(30)'])
        try:
            # Establish that this child really installed its TERM ignore
            # handler; cold package imports must not count as that scenario.
            spin(owner, lambda: owner.worker and b'ready' in owner.worker.errors, timeout=4.)
            spin(owner, lambda: owner.worker is None and owner.closed and
                 owner.revision == owner.persisted_revision, timeout=10.)
            self.assertIsNone(unrelated.poll())
            self.assertTrue(any(x['forced_kill'] for x in owner.writer_history))
            self.assertFalse(owner.core.checkpoint()['writer_pending'])
            self.assertEqual(owner.results()[0]['status'], 'failed')
            self.assertEqual(owner.fault, 'writer_wall_timeout')
            saved = json.loads((owner.output/'checkpoint.json').read_bytes())
            self.assertEqual(saved['fault'], 'writer_wall_timeout')
        finally:
            unrelated.terminate(); unrelated.wait(timeout=2.)

    def test_signal_style_shutdown_is_bounded_flush_not_success(self):
        owner = self.owner(worker_path=self.fake_worker(), runtime=RuntimePolicy(
            worker_timeout_s=5., term_grace_s=.05, kill_grace_s=.05, shutdown_timeout_s=8.))
        self.queue(owner)
        spin(owner, lambda: owner.worker and b'ready' in owner.worker.errors)
        started = time.monotonic()
        report = owner.shutdown('SIGTERM')
        self.assertLess(time.monotonic()-started, 8.5)
        self.assertTrue(report['shutdown']['owned_writer_exit_confirmed'], report)
        self.assertTrue(report['shutdown']['final_checkpoint_persisted'], report)
        self.assertEqual(owner.results()[0]['status'], 'failed')
        self.assertFalse(owner.core.checkpoint()['writer_pending'])

    def test_short_shutdown_reports_unconfirmed_writer_not_success(self):
        stalled = self.root/'stalled-checkpoint.py'
        stalled.write_text('import time\ntime.sleep(30)\n')
        owner = self.owner(worker_path=stalled, runtime=RuntimePolicy(shutdown_timeout_s=.05))
        started = time.monotonic()
        report = owner.shutdown('SIGTERM')
        self.assertLess(time.monotonic()-started, .5)
        self.assertFalse(report['shutdown']['owned_writer_exit_confirmed'], report)
        self.assertFalse(report['shutdown']['final_checkpoint_persisted'], report)
        self.assertIsNotNone(report['writer'])

    def test_write_error_never_publishes_stored(self):
        owner = self.owner()
        req, _, _ = self.queue(owner)
        # Exact destination obstruction is an actual exclusive-create failure.
        (owner.output/sha(canonical(req))).mkdir()
        spin(owner, lambda: owner.closed and owner.worker is None and
             owner.revision == owner.persisted_revision)
        self.assertEqual(owner.results()[0]['status'], 'failed')
        self.assertIn('FileExistsError', owner.fault)

    def test_checkpoint_failure_is_explicit_and_no_infinite_retry(self):
        owner = self.owner()
        (owner.output/'.checkpoint.pending').mkdir()
        owner.close('shutdown', time.monotonic())
        spin(owner, lambda: owner.checkpoint_failed and owner.worker is None)
        self.assertFalse(owner.shutdown()['shutdown']['final_checkpoint_persisted'])
        for _ in range(5):
            owner.tick(time.monotonic())
        self.assertIsNone(owner.worker)

    def test_existing_output_and_off_owner_calls_rejected(self):
        owner = self.owner()
        with self.assertRaises(FileExistsError):
            ProducerRetention(owner.output, 'session-a')
        errors = []
        def wrong_thread():
            try:
                owner.on_epoch('epoch-b', True, time.monotonic())
            except RuntimeError as exc:
                errors.append(str(exc))
        thread = threading.Thread(target=wrong_thread)
        thread.start(); thread.join(timeout=1.)
        self.assertEqual(len(errors), 1)
        self.assertFalse(owner.closed)
        owner.shutdown()

    def test_valid_request_before_epoch_closes_run_instead_of_later_resurrection(self):
        owner = ProducerRetention(self.root/'pre-epoch', 'session-a')
        self.owners.append(owner)
        result = owner.request(request(sample()[0]), time.monotonic())
        self.assertEqual(result['status'], 'failed')
        owner.on_epoch('epoch-a', True, time.monotonic())
        self.assertIsNone(owner.core)
        self.assertTrue(owner.closed)
        owner.shutdown()

    def test_spawn_failure_does_not_leave_fictitious_writer_or_retry_budget(self):
        owner = self.owner()
        self.queue(owner)
        with patch('limo_maze_perception.retention_adapter.subprocess.Popen', side_effect=OSError('spawn unavailable')):
            owner.tick(time.monotonic())
        self.assertIsNone(owner.worker)
        self.assertTrue(owner.checkpoint_failed)
        self.assertFalse(owner.core.checkpoint()['writer_pending'])
        self.assertEqual(owner.results()[0]['status'], 'failed')
        self.assertEqual(owner.core.jobs_reserved, 1)

    def test_pidfd_setup_failure_retains_child_until_confirmed_exit(self):
        owner = self.owner()
        self.queue(owner)
        with patch('limo_maze_perception.retention_adapter.os.pidfd_open', side_effect=PermissionError('denied')):
            owner.tick(time.monotonic())
        child = owner.worker.process
        self.assertIsNotNone(owner.worker.setup_error)
        self.assertTrue(owner.core.checkpoint()['writer_pending'])
        spin(owner, lambda: owner.worker is None and owner.revision == owner.persisted_revision)
        self.assertIsNotNone(child.poll())
        self.assertFalse(owner.core.checkpoint()['writer_pending'])
        self.assertEqual(owner.results()[0]['status'], 'failed')

    def make_signal_harness(self):
        fake = self.fake_worker()
        script = self.root/'signal-harness.py'
        prefix = ('import ast,json,signal,threading,time,sys\nfrom pathlib import Path\n' +
            'sys.path.insert(0,'+repr(str(HERE))+')\n' +
            'import retention_adapter as adapter\n' +
            'from retention_adapter import ProducerRetention,RuntimePolicy\n' +
            'from test_frame_retention_core import sample,request\n' +
            'source=Path('+repr(str(HERE/'node.py'))+').read_text()\n' +
            'fake='+repr(str(fake))+'\n')
        script.write_text(prefix + '''
class RecordedWriter(adapter.OwnedWriter):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs)
  pid=self.process.pid
  fields=Path('/proc',str(pid),'stat').read_text().rsplit(')',1)[1].split()
  identity=dict(pid=pid,start_ticks=int(fields[19]),token=self.token)
  destination=Path(sys.argv[1]+'.child.json')
  temporary=destination.with_suffix('.pending')
  temporary.write_text(json.dumps(identity))
  temporary.replace(destination)
adapter.OwnedWriter=RecordedWriter
class Node:
 def __init__(self):
  self.retention=ProducerRetention(sys.argv[1],'session-a',runtime=RuntimePolicy(worker_timeout_s=10.,term_grace_s=.05,kill_grace_s=.05,shutdown_timeout_s=8.),worker_path=fake)
  self.retention.on_epoch('epoch-a',True,time.monotonic())
  record,rgb,dep=sample()
  self.retention.offer(record,rgb,dep,time.monotonic())
  self.retention.request(request(record),time.monotonic())
  self.retention_shutdown=None
  self.retention_exit_reason='shutdown'
  self.ready=False
 def destroy_node(self):
  self.retention_shutdown=self.retention.shutdown(self.retention_exit_reason)
  print('REPORT '+json.dumps(self.retention_shutdown),flush=True)
class ROS:
 def init(self,**kw): pass
 def ok(self): return True
 def try_shutdown(self): pass
 def spin_once(self,node,**kw):
  node.retention.tick(time.monotonic())
  worker=node.retention.worker
  if worker and b'ready' in worker.errors and not node.ready:
   node.ready=True
   Path(sys.argv[1]+'.worker-ready').touch()
   if '--no-ready' not in sys.argv:
    print('READY',flush=True)
  time.sleep(.005)
method=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='main')
ns=dict(rclpy=ROS(),RedRGBDBaseline=Node,threading=threading,signal=signal,ExternalShutdownException=type('ExternalShutdownException',(Exception,),{}))
exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),'<main>', 'exec'),ns)
raise SystemExit(ns['main']())
''')
        return fake, script

    def test_real_signals_use_extracted_main_and_leave_no_owned_writer(self):
        fake, script = self.make_signal_harness()
        for sig in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(signal=sig):
                output = self.root/('signal-'+sig.name)
                process = subprocess.Popen([sys.executable, '-B', str(script), str(output)],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                try:
                    os.set_blocking(process.stdout.fileno(), False)
                    captured = bytearray(); deadline=time.monotonic()+4.
                    while b'READY\n' not in captured and time.monotonic()<deadline:
                        try: captured.extend(os.read(process.stdout.fileno(), 65536))
                        except BlockingIOError: pass
                        time.sleep(.01)
                    self.assertIn(b'READY\n', captured)
                    process.send_signal(sig)
                    os.set_blocking(process.stdout.fileno(), True)
                    out, err = process.communicate(timeout=10.)
                    captured.extend(out)
                    self.assertEqual(process.returncode, 2, (captured, err))
                    report=json.loads(bytes(captured).split(b'REPORT ',1)[1])
                    self.assertTrue(report['shutdown']['owned_writer_exit_confirmed'], report)
                    self.assertTrue(report['shutdown']['final_checkpoint_persisted'], report)
                    self.assertEqual(report['core']['results'][0]['status'], 'failed')
                    for child in report['writer_history']:
                        self.assertIsNotNone(child['returncode'])
                finally:
                    cleanup_signal_harness(process, Path(str(output)+'.child.json'), fake)

    def test_missing_ready_assertion_still_flushes_worker_and_closes_pipes(self):
        fake, script = self.make_signal_harness()
        output = self.root/'missing-ready'
        process = subprocess.Popen([sys.executable, '-B', str(script), str(output), '--no-ready'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        captured = bytearray()
        try:
            os.set_blocking(process.stdout.fileno(), False)
            deadline = time.monotonic()+4.
            while not Path(str(output)+'.worker-ready').exists() and time.monotonic()<deadline:
                try: captured.extend(os.read(process.stdout.fileno(), 65536))
                except BlockingIOError: pass
                time.sleep(.01)
            self.assertTrue(Path(str(output)+'.worker-ready').exists(), 'Controlled worker must be running')
            # Exercise the same finally path as the observed readiness failure.
            with self.assertRaises(AssertionError):
                self.assertIn(b'READY\n', captured)
        finally:
            out, err = cleanup_signal_harness(process, Path(str(output)+'.child.json'), fake)
        self.assertNotIn(b'READY\n', captured+out)
        self.assertEqual(process.returncode, 2, (captured+out, err))
        report = json.loads(bytes(captured+out).split(b'REPORT ', 1)[1])
        self.assertTrue(report['shutdown']['owned_writer_exit_confirmed'], report)
        self.assertTrue(report['shutdown']['final_checkpoint_persisted'], report)
        self.assertTrue(process.stdout.closed)
        self.assertTrue(process.stderr.closed)
    def test_default_off_main_callback_exception_is_not_swallowed(self):
        tree = ast.parse((HERE/'node.py').read_text())
        method = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
        calls=[]
        class ROS:
            def init(self, **kw): pass
            def spin(self, node): raise ValueError('ordinary_callback_failure')
            def try_shutdown(self): calls.append('shutdown')
        node=S(retention=None, destroy_node=lambda:calls.append('destroy'))
        ns=dict(rclpy=ROS(), RedRGBDBaseline=lambda:node, threading=threading, signal=signal,
                ExternalShutdownException=type('ExternalShutdownException',(Exception,),{}))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])), '<main>', 'exec'),ns)
        with self.assertRaisesRegex(ValueError, 'ordinary_callback_failure'):
            ns['main']()
        self.assertEqual(calls, ['destroy','shutdown'])


class SourceHookTests(unittest.TestCase):
    def method(self, name):
        tree = ast.parse((HERE/'node.py').read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RedRGBDBaseline')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
        namespace = dict(json=json, time=time, String=lambda **kw: S(**kw),
                         super=lambda:S(destroy_node=lambda:True))
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), '<source-hook>', 'exec'), namespace)
        return namespace[name]

    def test_finish_offer_same_callback_raw_then_publish_unchanged_metadata(self):
        calls=[]; payload=sample()[0]
        color, depth = S(data=bytearray(b'raw-rgb')), S(data=bytearray(b'raw-depth'))
        evidence = S(epoch_ready=True, clock_reset=False,
            finish=lambda *args: calls.append('finish') or payload)
        retention = S(core=object(), offer=lambda rec, rgb, dep, now:
            calls.append(('offer', rec is payload, bytes(rgb), bytes(dep))))
        node = S(frame_evidence=evidence, retention=retention, retention_clock_guard=lambda: None,
                 get_clock=lambda: S(now=lambda: S(nanoseconds=1)),
                 frame_publisher=S(publish=lambda msg: calls.append(('publish', json.loads(msg.data)))))
        self.method('publish_frame_record')(node, {}, {}, None, color, depth)
        self.assertEqual(calls[:2], ['finish', ('offer', True, b'raw-rgb', b'raw-depth')])
        self.assertEqual(calls[2], ('publish', payload))

    def test_default_off_hook_keeps_original_record_publication(self):
        calls=[]; payload=sample()[0]
        node=S(frame_evidence=S(finish=lambda *args:payload), retention=None,
               get_clock=lambda:S(now=lambda:S(nanoseconds=1)),
               frame_publisher=S(publish=lambda msg:calls.append(json.loads(msg.data))))
        self.method('publish_frame_record')(node, {}, {}, None)
        self.assertEqual(calls, [payload])

    def test_pre_epoch_rejected_pair_does_not_prematurely_close_retention(self):
        calls=[]; payload=sample()[0]; payload['status']='rejected'
        node=S(frame_evidence=S(finish=lambda *args:payload, epoch_ready=False, clock_reset=False),
               retention=S(core=None, close=lambda *args:calls.append('closed')),
               retention_clock_guard=lambda:None, get_clock=lambda:S(now=lambda:S(nanoseconds=1)),
               frame_publisher=S(publish=lambda msg:calls.append('published')))
        self.method('publish_frame_record')(node, {}, {}, None)
        self.assertEqual(calls, ['published'])

    def test_unconfirmed_writer_identity_is_in_exit_log(self):
        logs=[]
        report=dict(shutdown={'owned_writer_exit_confirmed':False,'final_checkpoint_persisted':False},
                    writer={'pid':123,'operation_id':'owned-op','owner_token':'token'},
                    fault='shutdown_wall_timeout',reason='SIGTERM')
        node=S(retention=S(shutdown=lambda reason:report), retention_timer=S(cancel=lambda:None),
               retention_shutdown=None, retention_exit_reason='SIGTERM', evidence_clock_jump=None,
               get_logger=lambda:S(info=lambda msg:logs.append(msg)))
        self.assertTrue(self.method('destroy_node')(node))
        logged=json.loads(logs[0].split(' ',1)[1])
        self.assertEqual(logged['writer'],report['writer'])
        self.assertEqual(logged['fault'],'shutdown_wall_timeout')
        self.assertFalse(logged['shutdown']['owned_writer_exit_confirmed'])


if __name__ == '__main__':
    unittest.main()
