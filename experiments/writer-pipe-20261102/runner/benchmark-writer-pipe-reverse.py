"""Bounded filesystem/pipe comparison using frozen run26 source bytes; no ROS."""
import fcntl
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/'outputs/setup/writer-pipe-reverse-20261102'
assert not OUT.exists(), 'Refuse previous benchmark output'
OUT.mkdir()
path = ROOT/'outputs/setup/run21-writer-benchmark/benchmark.py'
spec = importlib.util.spec_from_file_location('previous_benchmark', path)
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)
SOURCE = ROOT/'outputs/ros_smoke/run26-tf-tolerance'
MISSION = SOURCE/'mission/mission_20261101_231156.json'
closed = json.loads((SOURCE/'orchestration.json').read_text())
assert closed['all_owned_groups_empty'] and closed['elapsed_wall_s'] > 0
mission = json.loads(MISSION.read_text())
checkpoint = json.loads((SOURCE/'retained/checkpoint.json').read_text())
results = {r['retention_id']: r for r in checkpoint['core']['results']}
window = mission['frame_retention']['windows'][4]
rows = []
for selected in window['accepted']:
    request = selected['request']
    result = results[request['retention_id']]
    assert result['status'] == 'stored'
    directory = SOURCE/'retained'/result['receipt']['directory']
    files = {name:(directory/name).read_bytes() for name in ('rgb.bin','depth.bin','producer.json','camera_info.json','request.json')}
    for name,raw in files.items():
        assert bench.sha(raw) == result['receipt']['files'][name]['sha256']
    assert json.loads(files['request.json']) == request
    rows.append(dict(request=request, producer=json.loads(files['producer.json']), files=files,
                     requested_wall=result['requested_wall'], directory=directory))
assert len(rows) == 3
first = rows[0]['requested_wall']
for row in rows: row['offset'] = row['requested_wall']-first
remaining = window['deadline_wall']-first
native = Path(tempfile.mkdtemp(prefix='limo-writer-fs-pipe-20261102-'))
assert native.is_absolute() and not native.is_symlink() and native.resolve() == native
paths = [Path(bench.protocol.__file__),Path(bench.adapter.__file__),Path(bench.adapter.__file__).with_name('retention_worker.py')]
hashes = {str(p):bench.sha(p.read_bytes()) for p in paths}
bench.save(OUT/'inputs.json',dict(mission=str(MISSION),mission_sha256=bench.sha(MISSION.read_bytes()),
    source_window=5, source_modules=hashes, native_root=str(native),
    output_filesystems={str(p):subprocess.check_output(['stat','-f','-c','%T',str(p)],text=True).strip() for p in (OUT,native)},
    source_pairs=[dict(directory=str(r['directory']),offset_s=r['offset'],
        files={name:dict(bytes=len(raw),sha256=bench.sha(raw)) for name,raw in r['files'].items()}) for r in rows],
    scope='Same last fully stored run26 window, not missing window6 pixels; no ROS/Gazebo workload.'))
Original = bench.ORIGINAL_WRITER
reports = []
for name,root,pipe_bytes in (('drvfs-large-pipe',OUT,1048576),('drvfs-default',OUT,None)):
    writers = []
    class ProbeWriter(Original):
        def __init__(self,*args,**kwargs):
            super().__init__(*args,**kwargs)
            self.pipe_probe = {'pid':self.process.pid,'operation_id':self.operation_id,
                'requested_bytes':pipe_bytes,'resize_error':None,
                'initial_bytes':fcntl.fcntl(self.process.stdin.fileno(),fcntl.F_GETPIPE_SZ)}
            if pipe_bytes is not None:
                try: fcntl.fcntl(self.process.stdin.fileno(),fcntl.F_SETPIPE_SZ,pipe_bytes)
                except OSError as exc: self.pipe_probe['resize_error'] = repr(exc)
            self.pipe_probe['actual_bytes'] = fcntl.fcntl(self.process.stdin.fileno(),fcntl.F_GETPIPE_SZ)
            writers.append(self)
    bench.ORIGINAL_WRITER = ProbeWriter
    bench.OUT = root
    report = bench.run_attempt(name,rows,remaining)
    transfer = []
    for w in report['workers']:
        transfer.append(dict(operation_id=w['operation_id'],job_id=w['job_id'],
            wire_bytes=w['initial_wire_bytes'],
            input_drain_s=None if w['input_drained_wall'] is None else w['input_drained_wall']-w['started_wall'],
            total_s=None if w['terminal_observed_wall'] is None else w['terminal_observed_wall']-w['started_wall'],
            max_tick_gap_s=max((b['wall']-a['wall'] for a,b in zip(w['polls'],w['polls'][1:])),default=0)))
    summary = dict(variant=name,output=str(root/name),pipe_probes=[w.pipe_probe for w in writers],transfer=transfer,
        **{k:report.get(k) for k in ('errors','all_durable_elapsed_s','would_meet_original_consumer_deadline','all_owned_helpers_closed','elapsed_wall_s','source_bytes_verification')})
    reports.append(summary)
    bench.save(OUT/(name+'-summary.json'),summary)
    # Capture reports from native disk only after helpers have closed; do not delete raw data.
    if root != OUT:
        assert report['all_owned_helpers_closed']
        for filename in ('report.json','trace.json'):
            (OUT/(name+'-'+filename)).write_bytes((root/name/filename).read_bytes())
    print(json.dumps({'variant':name,'transfer':transfer,'pipes':[w.pipe_probe for w in writers]}),flush=True)
    if not report['all_owned_helpers_closed']: break
bench.save(OUT/'comparison.json',dict(variants=reports,
    modules_unchanged=hashes=={str(p):bench.sha(p.read_bytes()) for p in paths},
    limitations=['One fixed-order attempt per cell; cache/order effects not separated.',
                 'No concurrent simulator/ROS/image-processing load.',
                 'Failed original window pixels do not exist, so these are preceding stored bytes.',
                 'This benchmark does not repair or reclassify run26.']))
assert all(r['all_owned_helpers_closed'] for r in reports)
