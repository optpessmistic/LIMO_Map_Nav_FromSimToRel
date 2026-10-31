"""One baseline and one -S candidate: real run21 bytes, no ROS nodes.

This is an isolated replay measurement, not a repair of run21 mission evidence.
The owner polls at 50 ms with the original production timing policies.
"""
import hashlib
import json
import os
from pathlib import Path
import signal
import sys
import time

import limo_evidence_protocol.retention as protocol
import limo_maze_perception.retention_adapter as adapter

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
SOURCE = ROOT/'outputs/ros_smoke/run21-observation-retention'
MISSION = SOURCE/'mission/mission_20261031_053024.json'
COMMON_PARENT = str(Path(protocol.__file__).resolve().parents[1])
ORIGINAL_WRITER = adapter.OwnedWriter
ORIGINAL_POPEN = adapter.subprocess.Popen


def save(path, value):
    with path.open('x') as handle:
        json.dump(value, handle, indent=2, allow_nan=False)
        handle.write('\n')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def source_rows():
    mission = json.loads(MISSION.read_text())
    checkpoint = json.loads((SOURCE/'retained/checkpoint.json').read_text())
    results = {r['retention_id']: r for r in checkpoint['core']['results']}
    accepted = mission['frame_retention']['windows'][0]['accepted']
    rows = []
    for selected in accepted:
        request = selected['request']
        original = results[request['retention_id']]
        directory = SOURCE/'retained'/original['receipt']['directory']
        names = ('rgb.bin', 'depth.bin', 'producer.json', 'camera_info.json', 'request.json')
        files = {name: (directory/name).read_bytes() for name in names}
        for name, raw in files.items():
            assert sha(raw) == original['receipt']['files'][name]['sha256']
        assert json.loads(files['request.json']) == request
        rows.append(dict(request=request, producer=json.loads(files['producer.json']), files=files,
                         requested_wall=original['requested_wall'], directory=directory))
    assert len(rows) == 3
    first = rows[0]['requested_wall']
    for row in rows:
        row['offset'] = row['requested_wall']-first
    return rows, mission['frame_retention']['windows'][0]['deadline_wall']-first


def verify_files(rows, result_root, checkpoint):
    result_rows = {r['retention_id']: r for r in checkpoint['core']['results']}
    verified = []
    for source in rows:
        rid = source['request']['retention_id']
        row = dict(retention_id=rid, errors=[], files={})
        try:
            result = result_rows[rid]
            assert result['status'] == 'stored' and not result['writer_owned'] and result['reason'] is None
            receipt = result['receipt']
            assert receipt['request_sha256'] == sha(source['files']['request.json'])
            assert receipt['directory'] == receipt['request_sha256']
            directory = result_root/receipt['directory']
            expected_names = set(source['files']) | {'manifest.json'}
            assert set(receipt['files']) == expected_names
            for name in sorted(expected_names):
                raw = (directory/name).read_bytes()
                actual = dict(bytes=len(raw), sha256=sha(raw))
                assert actual == receipt['files'][name]
                if name in source['files']:
                    assert raw == source['files'][name]
                row['files'][name] = actual
            manifest = json.loads((directory/'manifest.json').read_text())
            assert manifest['request_sha256'] == receipt['request_sha256']
            assert manifest['sample'] == source['request']['sample']
            assert manifest['files'] == {k:v for k,v in receipt['files'].items() if k!='manifest.json'}
        except (AssertionError, KeyError, ValueError, OSError) as exc:
            row['errors'].append(type(exc).__name__+': '+str(exc))
        row['verified'] = not row['errors']
        verified.append(row)
    return dict(passed=all(r['verified'] for r in verified), frames=verified,
                scope='reopened benchmark files; source bytes and request identities only; not run21 mission success')


def run_attempt(name, rows, window_remaining):
    directory = OUT/name
    directory.mkdir(exist_ok=False)
    instances, trace = [], []
    started = time.monotonic()
    producer = None
    first_request = None
    report = dict(variant=name, errors=[], source_request_offsets_s=[r['offset'] for r in rows],
                  original_consumer_remaining_from_first_request_s=window_remaining,
                  polling_interval_s=.05, max_tick_transfer_bytes=1024*1024,
                  worker_timeout_s=5., request_timeout_s=10., shutdown_timeout_s=8.,
                  candidate_change=('Python -S; PYTHONPATH only '+COMMON_PARENT) if name=='candidate-no-site' else None,
                  limitations=['No Gazebo, ROS executor, image processing, or concurrent camera load.',
                               'Constructor duration is Popen cost, not full interpreter startup.',
                               'Input drain includes scheduling/backpressure; no child phase or fsync instrumentation.',
                               'One attempt per variant in baseline-then-candidate order; cache/order effects are not controlled.'])

    class MeasuredWriter(ORIGINAL_WRITER):
        def __init__(self, *args, **kwargs):
            begin = time.monotonic()
            if name == 'candidate-no-site':
                def candidate_popen(command, *pos, **kw):
                    assert command[0] == sys.executable and command[1] == '-B'
                    assert Path(command[2]).name == 'retention_worker.py'
                    command = [command[0], '-S', *command[1:]]
                    kw['env'] = dict(kw['env'], PYTHONPATH=COMMON_PARENT)
                    return ORIGINAL_POPEN(command, *pos, **kw)
                adapter.subprocess.Popen = candidate_popen
            try:
                super().__init__(*args, **kwargs)
            finally:
                adapter.subprocess.Popen = ORIGINAL_POPEN
            self.measurement = dict(pid=self.process.pid, job_id=self.job.request_id if self.job else None,
                operation_id=self.operation_id, started_wall=self.started,
                constructor_start_wall=begin, constructor_end_wall=time.monotonic(),
                initial_wire_bytes=sum(map(len,self.segments)), input_drained_wall=None,
                terminal_observed_wall=None, polls=[])
            instances.append(self)

        def poll(self, now):
            before = sum(map(len,self.segments))
            entered = time.monotonic()
            done = super().poll(now)
            after = sum(map(len,self.segments))
            ended = time.monotonic()
            self.measurement['polls'].append(dict(wall=now, entered=entered, exited=ended,
                                                  bytes_sent=before-after, remaining=after, done=done))
            if not after and self.measurement['input_drained_wall'] is None:
                self.measurement['input_drained_wall'] = ended
            if done and self.measurement['terminal_observed_wall'] is None:
                self.measurement['terminal_observed_wall'] = ended
            return done

    adapter.OwnedWriter = MeasuredWriter
    try:
        producer = adapter.ProducerRetention(directory/'retained', rows[0]['producer']['detector_session_id'],
            protocol.Policy(max_jobs=96, max_file_bytes=268435456), adapter.RuntimePolicy())
        producer.on_epoch(rows[0]['producer']['map_epoch'], True, time.monotonic())
        prewarm_until = time.monotonic()+7.
        while time.monotonic() < prewarm_until:
            producer.tick(time.monotonic())
            if producer.fault:
                raise RuntimeError('Initial checkpoint failed: '+producer.fault)
            if producer.worker is None and producer.revision == producer.persisted_revision:
                break
            time.sleep(.05)
        else:
            raise TimeoutError('Initial checkpoint not ready')
        report['initial_checkpoint_ready_wall'] = time.monotonic()
        first_request = time.monotonic()
        pending = list(rows)
        last_result = None
        while time.monotonic()-first_request < 12.:
            now = time.monotonic()
            while pending and now-first_request >= pending[0]['offset']:
                row = pending.pop(0)
                offered = producer.offer(row['producer'],row['files']['rgb.bin'],row['files']['depth.bin'],time.monotonic())
                requested = producer.request(row['request'],time.monotonic())
                trace.append(dict(event='offer_request', elapsed=time.monotonic()-first_request,
                                  retention_id=row['request']['retention_id'],offer=offered,result=requested))
            producer.tick(time.monotonic())
            results = producer.results()
            compact = [(r['retention_id'],r['status'],r['checkpoint_persisted'],r['reason']) for r in results]
            if compact != last_result:
                trace.append(dict(event='results',elapsed=time.monotonic()-first_request,results=results))
                last_result=compact
            if producer.fault:
                raise RuntimeError('Producer fault: '+producer.fault)
            if len(results)==3 and all(r['status']=='stored' and r['checkpoint_persisted'] for r in results):
                report['all_durable_elapsed_s']=time.monotonic()-first_request
                report['would_meet_original_consumer_deadline']=report['all_durable_elapsed_s'] < window_remaining
                report['durable_results']=results
                break
            if any(r['status']=='failed' for r in results):
                raise RuntimeError('Request failed')
            time.sleep(.05)
        else:
            raise TimeoutError('Durable workload not complete in bounded collection window')
    except BaseException as exc:
        report['errors'].append(type(exc).__name__+': '+str(exc))
    finally:
        if producer is not None:
            report['before_shutdown']=producer.snapshot()
            try:
                report['shutdown']=producer.shutdown()
            except BaseException as exc:
                report['errors'].append('Shutdown '+type(exc).__name__+': '+str(exc))
        emergency=[]
        for writer in instances:
            if writer.process.poll() is None:
                for sig in (signal.SIGTERM,signal.SIGKILL):
                    writer.signal(sig)
                    until=time.monotonic()+.5
                    while writer.process.poll() is None and time.monotonic()<until:
                        time.sleep(.01)
                    emergency.append(dict(pid=writer.process.pid,signal=sig.name,returncode=writer.process.poll()))
                    if writer.process.poll() is not None:
                        break
                if writer.process.poll() is not None and writer.pidfd is not None:
                    writer.dispose()
            writer.measurement['final_returncode']=writer.process.poll()
        report['emergency_cleanup']=emergency
        report['all_owned_helpers_closed']=all(w.process.poll() is not None for w in instances)
        report['elapsed_wall_s']=time.monotonic()-started
        report['workers']=[w.measurement for w in instances]
        report['source_bytes_verification']=None
        if producer is not None and (directory/'retained/checkpoint.json').is_file():
            checkpoint=json.loads((directory/'retained/checkpoint.json').read_text())
            report['source_bytes_verification']=verify_files(rows,directory/'retained',checkpoint)
        save(directory/'trace.json',trace)
        save(directory/'report.json',report)
        adapter.OwnedWriter=ORIGINAL_WRITER
        adapter.subprocess.Popen=ORIGINAL_POPEN
    print(json.dumps({k:report.get(k) for k in ('variant','errors','all_durable_elapsed_s','would_meet_original_consumer_deadline','all_owned_helpers_closed','elapsed_wall_s')}),flush=True)
    return report


def main():
    if (OUT/'baseline').exists() or (OUT/'candidate-no-site').exists() or (OUT/'comparison.json').exists():
        raise SystemExit('Refuse repeated or overwritten benchmark attempts')
    rows, remaining=source_rows()
    module_paths=[Path(protocol.__file__),Path(adapter.__file__),Path(adapter.__file__).with_name('retention_worker.py')]
    before={str(p):sha(p.read_bytes()) for p in module_paths}
    save(OUT/'inputs.json',dict(mission=str(MISSION),mission_sha256=sha(MISSION.read_bytes()),
        modules=before, python=sys.executable, common_parent=COMMON_PARENT,
        environment={k:os.environ.get(k) for k in ('PYTHONPATH','AMENT_PREFIX_PATH','LD_LIBRARY_PATH','ROS_DISTRO')},
        source_pairs=[dict(directory=str(r['directory']),offset_s=r['offset'],
                           files={name:dict(bytes=len(raw),sha256=sha(raw)) for name,raw in r['files'].items()}) for r in rows]))
    reports=[run_attempt('baseline',rows,remaining),run_attempt('candidate-no-site',rows,remaining)]
    after={str(p):sha(p.read_bytes()) for p in module_paths}
    save(OUT/'comparison.json',dict(source_modules_unchanged=before==after,modules_after=after,
        variants=[{k:r.get(k) for k in ('variant','errors','all_durable_elapsed_s',
                    'would_meet_original_consumer_deadline','all_owned_helpers_closed','elapsed_wall_s')} for r in reports],
        conclusion_scope='one isolated replay per variant; no ROS or Gazebo load; do not extrapolate directly to mission success'))
    return 0 if before==after and all(not r['errors'] and r['all_owned_helpers_closed'] for r in reports) else 1


if __name__=='__main__':
    raise SystemExit(main())
