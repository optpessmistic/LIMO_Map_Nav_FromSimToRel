"""Decode the CLOSED run29 bag offline; no node, playback, or runtime changes."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import re
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = Path(__file__).resolve().parents[2]
RUN = ROOT/'outputs/ros_smoke/run29-navigation-recovery'
OUT = ROOT/'outputs/setup/run29-tf-delivery-audit.json'
if OUT.exists():
    raise RuntimeError('Refuse overwriting audit')
closed = json.loads((RUN/'orchestration.json').read_bytes())
assert closed['all_owned_groups_empty'] and closed['elapsed_wall_s'] > 0
bag_dir = RUN/'recording/bag'
bags = sorted(bag_dir.glob('*.mcap'))
assert bags

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        while chunk := stream.read(8*1024*1024):
            h.update(chunk)
    return h.hexdigest()

hashes = {p.name: digest(p) for p in bags}
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
            rosbag2_py.ConverterOptions('', ''))
topics = {x.name: get_message(x.type) for x in reader.get_all_topics_and_types()
          if x.name in ('/clock', '/tf', '/odom', '/scan', '/map')}
reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
series = defaultdict(list)

def ns(stamp):
    return stamp.sec*10**9 + stamp.nanosec

while reader.has_next():
    name, raw, receipt = reader.read_next()
    msg = deserialize_message(raw, topics[name])
    if name == '/tf':
        for tf in msg.transforms:
            series[tf.header.frame_id+'->'+tf.child_frame_id].append((receipt, ns(tf.header.stamp)))
    else:
        series[name].append((receipt, ns(msg.clock if name == '/clock' else msg.header.stamp)))

def row_json(row):
    if row is None:
        return None
    return {'bag_ros_s': row[0]/1e9, 'header_s': row[1]/1e9,
            'bag_minus_header_s': (row[0]-row[1])/1e9}

def summary(rows):
    gaps = [(b[0]-a[0])/1e9 for a, b in zip(rows, rows[1:])]
    return {'count': len(rows), 'first': row_json(rows[0]) if rows else None,
            'last': row_json(rows[-1]) if rows else None,
            'header_backwards_count': sum(b[1] < a[1] for a, b in zip(rows, rows[1:])),
            'equal_adjacent_header_count': sum(b[1] == a[1] for a, b in zip(rows, rows[1:])),
            'max_adjacent_bag_gap_s': max(gaps) if gaps else None,
            'bag_gaps_above_0_15s': [dict(before=row_json(a), after=row_json(b), gap_s=(b[0]-a[0])/1e9)
                for a, b in zip(rows, rows[1:]) if b[0]-a[0] > 150_000_000]}

navigation = (RUN/'navigation.log').read_text()
events = []
map_odom = series['map->odom']
for line_no, line in enumerate(navigation.splitlines(), 1):
    m = re.search(r'Requested time ([0-9.]+) but the latest data is at time ([0-9.]+)', line)
    if not m:
        continue
    requested, latest = map(float, m.groups())
    latest_ns = round(latest*1e9)
    indices = [i for i, row in enumerate(map_odom) if row[1] == latest_ns]
    lo, hi = requested-1, requested+1.5
    event = {'navigation_line': line_no, 'text': line, 'requested_s': requested,
             'latest_s': latest, 'residual_s': requested-latest,
             'window_ros_s': [lo, hi],
             'window': {name: summary([row for row in rows if lo <= row[0]/1e9 <= hi])
                        for name, rows in series.items()}}
    if indices:
        first, last = indices[0], indices[-1]
        event['matching_stale_header_block'] = {
            'message_count': len(indices), 'contiguous': len(indices) == last-first+1,
            'first': row_json(map_odom[first]), 'last': row_json(map_odom[last]),
            'preceding': row_json(map_odom[first-1]) if first else None,
            'following': row_json(map_odom[last+1]) if last+1 < len(map_odom) else None,
            'span_of_repeated_receipts_s': (map_odom[last][0]-map_odom[first][0])/1e9,
            'gap_until_following_receipt_s': ((map_odom[last+1][0]-map_odom[last][0])/1e9
                if last+1 < len(map_odom) else None)}
    else:
        event['matching_stale_header_block'] = None
    events.append(event)

assert hashes == {p.name: digest(p) for p in bags}
result = {'schema_version': 1, 'read_only': True, 'run': RUN.name,
          'input_bag_sha256': hashes, 'script_sha256': digest(Path(__file__)),
          'closed_orchestration_sha256': digest(RUN/'orchestration.json'),
          'topics': {name: summary(rows) for name, rows in series.items()}, 'events': events,
          'limitations': ['Recorder used --use-sim-time, so receipt stamps are recorder ROS time, not independent wall arrival.',
              'No ROS node or playback was started. No private Nav2 TF cache is reconstructed.',
              'A bag receipt gap can reflect publishing, transport, or recorder scheduling; it does not by itself prove a publisher pause.',
              'The exception latest stamp is selected from the Nav2 log; exact repeated TF-header blocks are reported without fitting clocks.']}
with OUT.open('x') as stream:
    json.dump(result, stream, indent=2, allow_nan=False)
    stream.write('\n')
print(json.dumps({'output': str(OUT), 'events': [
    {'line': e['navigation_line'], 'requested_s': e['requested_s'],
     'stale_block': e['matching_stale_header_block'],
     'window_counts_and_gaps': {name: {'count': s['count'], 'max_gap_s': s['max_adjacent_bag_gap_s']}
                              for name, s in e['window'].items()}}
    for e in events]}, indent=2))
