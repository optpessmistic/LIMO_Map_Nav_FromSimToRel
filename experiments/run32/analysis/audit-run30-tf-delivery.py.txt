"""Offline run29/run30 raw TF comparison; only after root confirms run30 closed.

No ROS initialization, playback, node, motion or process-control operations.
"""
import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/'outputs/setup/run30-tf-delivery-audit.json'
RUNS = ('run29-navigation-recovery', 'run30-steady-tf')


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        while chunk := stream.read(8*1024*1024):
            h.update(chunk)
    return h.hexdigest()


def read(path):
    return json.loads(path.read_bytes())


def row_json(row):
    return None if row is None else {'bag_ros_s': row[0]/1e9, 'header_s': row[1]/1e9,
                                    'bag_minus_header_s': (row[0]-row[1])/1e9}


def largest_gap(rows, column):
    pair = max(zip(rows, rows[1:]), key=lambda pair: pair[1][column]-pair[0][column], default=None)
    return None if pair is None else {'gap_s': (pair[1][column]-pair[0][column])/1e9,
                                     'before': row_json(pair[0]), 'after': row_json(pair[1])}


def summary(rows):
    advances = []
    high = None
    for row in rows:
        if high is None or row[1] > high:
            advances.append(row)
            high = row[1]
    arrival = largest_gap(rows, 0)
    progression = largest_gap(advances, 0)
    step = largest_gap(rows, 1)
    ages = [(row[0]-row[1])/1e9 for row in rows]
    return {'count': len(rows), 'first': row_json(rows[0]) if rows else None,
        'last': row_json(rows[-1]) if rows else None,
        'header_backwards_count': sum(b[1] < a[1] for a,b in zip(rows, rows[1:])),
        'bag_receipt_backwards_count': sum(b[0] < a[0] for a,b in zip(rows, rows[1:])),
        'equal_adjacent_header_count': sum(b[1] == a[1] for a,b in zip(rows, rows[1:])),
        'max_adjacent_bag_gap_s': arrival['gap_s'] if arrival else None,
        'max_adjacent_bag_gap': arrival,
        'bag_gaps_above_0_15s_count': sum(b[0]-a[0] > 150_000_000 for a,b in zip(rows, rows[1:])),
        'strictly_advancing_header_receipts_count': len(advances),
        'max_receipt_gap_between_header_advances_s': progression['gap_s'] if progression else None,
        'max_receipt_gap_between_header_advances': progression,
        'max_adjacent_header_step_s': step['gap_s'] if step else None,
        'max_adjacent_header_step': step,
        'observed_tail_since_last_header_advance_s': (rows[-1][0]-advances[-1][0])/1e9 if rows else None,
        'max_bag_minus_header_s': max(ages) if ages else None,
        'min_bag_minus_header_s': min(ages) if ages else None}


def closed_receipt(run):
    value = read(run/'orchestration.json')
    elapsed = value.get('elapsed_wall_s')
    if not (value.get('all_owned_groups_empty') is True and type(elapsed) in (float,int)
            and math.isfinite(elapsed) and elapsed > 0):
        raise RuntimeError('Run lacks final owned-process closure: '+run.name)
    stops, nested = value.get('stops') or {}, value.get('recorder_inner_recheck') or {}
    if not (stops and all(r.get('remaining') == [] and r.get('ownership_errors') == []
                         and r.get('remaining_check_complete') is True for r in stops.values())
            and nested.get('remaining') == [] and nested.get('ownership_errors') == []
            and nested.get('remaining_check_complete') is True):
        raise RuntimeError('Owned group/nested recorder closure incomplete: '+run.name)
    writer = read(run/'retention-shutdown.json')
    if not (writer.get('writer') is None and writer.get('shutdown',{}).get('owned_writer_exit_confirmed') is True):
        raise RuntimeError('Writer exit not confirmed: '+run.name)
    return value


def decode(run):
    # Imports provide offline CDR decoding only. Never call rclpy.init().
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    closed_receipt(run)
    bag_dir = run/'recording/bag'
    bags = sorted(bag_dir.glob('*.mcap'))
    if not bags:
        raise RuntimeError('No recorded MCAP files: '+run.name)
    inputs = [*bags, run/'orchestration.json', run/'navigation.log',
              run/'preparation.json', run/'retention-shutdown.json']
    hashes = {str(p.relative_to(run)): digest(p) for p in inputs}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {x.name: get_message(x.type) for x in reader.get_all_topics_and_types()
             if x.name in ('/clock','/tf','/odom','/scan','/map')}
    if not {'/tf','/clock','/odom','/scan'}.issubset(types):
        raise RuntimeError('Required reference topics missing: '+run.name)
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    series = defaultdict(list)
    def ns(stamp):
        return stamp.sec*10**9+stamp.nanosec
    while reader.has_next():
        topic, raw, receipt = reader.read_next()
        msg = deserialize_message(raw, types[topic])
        if topic == '/tf':
            for tf in msg.transforms:
                series[tf.header.frame_id+'->'+tf.child_frame_id].append((receipt, ns(tf.header.stamp)))
        else:
            series[topic].append((receipt, ns(msg.clock if topic == '/clock' else msg.header.stamp)))
    del reader
    map_odom = series['map->odom']
    if not map_odom:
        raise RuntimeError('No map->odom messages; cannot claim zero gaps: '+run.name)
    events = []
    navigation = (run/'navigation.log').read_text()
    for line_no, line in enumerate(navigation.splitlines(),1):
        match = re.search(r'Requested time ([0-9.]+) but the latest data is at time ([0-9.]+)', line)
        if not match:
            continue
        requested, latest = map(float, match.groups())
        indices = [i for i,row in enumerate(map_odom) if row[1] == round(latest*1e9)]
        lo, hi = requested-1, requested+1.5
        event = {'navigation_line':line_no, 'text':line, 'requested_s':requested,
            'latest_s':latest, 'residual_s':requested-latest, 'window_ros_s':[lo,hi],
            'window':{name:summary([row for row in rows if lo <= row[0]/1e9 <= hi])
                      for name,rows in series.items()}, 'matching_stale_header_block':None}
        if indices:
            first,last = indices[0],indices[-1]
            event['matching_stale_header_block'] = {
                'message_count':len(indices), 'contiguous':len(indices)==last-first+1,
                'first':row_json(map_odom[first]), 'last':row_json(map_odom[last]),
                'preceding':row_json(map_odom[first-1]) if first else None,
                'following':row_json(map_odom[last+1]) if last+1<len(map_odom) else None,
                'span_of_repeated_receipts_s':(map_odom[last][0]-map_odom[first][0])/1e9,
                'gap_until_following_receipt_s':(map_odom[last+1][0]-map_odom[last][0])/1e9
                    if last+1<len(map_odom) else None}
        events.append(event)
    if hashes != {str(p.relative_to(run)):digest(p) for p in inputs}:
        raise RuntimeError('Input changed during offline decoding: '+run.name)
    return {'run':run.name, 'epoch':read(run/'preparation.json')['epoch'],
        'input_sha256':hashes, 'input_unchanged_after_decode':True,
        'topics':{name:summary(rows) for name,rows in series.items()},
        'future_extrapolation_event_count':len(events), 'events':events,
        'fault_window_status':'matched_log_event_windows' if events else
            'zero_matching_future_extrapolation_events; events=[] is observed zero, not missing data',
        'navigation_log_extrapolation_line_count':sum('extrapolat' in line.lower() for line in navigation.splitlines())}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--confirmed-run30-root-session',type=int,choices=[96593],required=True)
    parser.add_argument('--confirmed-run30-root-exit-code',type=int,required=True)
    args = parser.parse_args()
    if OUT.exists() or OUT.is_symlink():
        raise RuntimeError('Refuse overwriting audit')
    for name in RUNS:
        closed_receipt(ROOT/'outputs/ros_smoke'/name)
    reports = {name:decode(ROOT/'outputs/ros_smoke'/name) for name in RUNS}
    probe_path = ROOT/'outputs/setup/run30-wsl-system-steady-clock-probe-receipt.json'
    probe = read(probe_path)
    if probe['epoch'] != reports[RUNS[1]]['epoch']:
        raise RuntimeError('Clock probe receipt belongs to another epoch')
    fields = ('count','max_adjacent_bag_gap_s','max_receipt_gap_between_header_advances_s',
              'max_adjacent_header_step_s','bag_gaps_above_0_15s_count',
              'equal_adjacent_header_count','header_backwards_count','max_bag_minus_header_s')
    comparison = {name:{field:reports[name]['topics']['map->odom'][field] for field in fields}
                  for name in RUNS}
    result = {'schema_version':2, 'read_only':True, 'script_sha256':digest(Path(__file__)),
        'root_exec_completion':{'session_id':args.confirmed_run30_root_session,
            'exit_code':args.confirmed_run30_root_exit_code,'source':'Explicit root confirmation'},
        'runs':reports,'map_odom_comparison':comparison,
        'concurrent_system_steady_probe':{'receipt':probe,'receipt_sha256':digest(probe_path)},
        'definitions':{
            'max_adjacent_bag_gap_s':'Maximum adjacent receive-stamp difference in bag reader order, over the entire recorded series.',
            'max_receipt_gap_between_header_advances_s':'Maximum receive-stamp gap between receipts establishing a strictly higher header high-water mark; repeated/backward stamps do not reset advancement.',
            'max_adjacent_header_step_s':'Maximum adjacent source header-stamp difference, distinct from receive interval.',
            'zero_errors':'An empty events array explicitly means no matching future-extrapolation log events; full-series statistics are still computed.'},
        'limitations':[
            'Recorder used --use-sim-time: receipt stamps are recorder ROS time, not independent wall arrival. System clock probe remains on its own monotonic time axis.',
            'Full series means the observed bag interval; gaps before first/after last receipt are censored. The last partial header-advance wait is reported separately.',
            'No ROS node/playback or Nav2 private TF-cache reconstruction. A bag gap alone cannot isolate publishing, transport, or recorder scheduling.',
            'Each run has a different path and duration; full-run maxima are descriptive, not matched-load causal proof.',
            'Zero matching future-extrapolation events does not prove zero other TF/navigation errors; extrapolation log-line count is also retained.']}
    with OUT.open('x',encoding='utf-8',newline='\n') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
        stream.write('\n')
    print(json.dumps({'output':str(OUT),'map_odom_comparison':comparison,
        'future_extrapolation_events':{name:r['future_extrapolation_event_count'] for name,r in reports.items()},
        'ros_nodes_started':False},indent=2))


if __name__ == '__main__':
    main()
