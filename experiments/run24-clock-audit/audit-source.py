"""Decode closed run24 stamps only; no rclpy.init, replay, publishers or motion."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT=Path(__file__).resolve().parents[2]
RUN=ROOT/'outputs/ros_smoke/run24-startup-gate'
OUT=ROOT/'outputs/setup/run24-clock-audit.json'
if OUT.exists(): raise RuntimeError('Refuse overwriting audit')
closed=json.loads((RUN/'orchestration.json').read_text())
assert closed['all_owned_groups_empty'] and closed['elapsed_wall_s']>0
bag=RUN/'recording/bag/bag_0.mcap'
sha=hashlib.sha256(bag.read_bytes()).hexdigest()
assert sha=='a4afa08357a7cfd8b027ee5b7dae08f4afb0cce2afc8cc9a9f1543a6985d531c'
reader=rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(bag.parent),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
topics={x.name:get_message(x.type) for x in reader.get_all_topics_and_types() if x.name in ('/clock','/tf','/odom','/scan')}
reader.set_filter(rosbag2_py.StorageFilter(topics=list(topics)))
series=defaultdict(list)
def ns(stamp): return stamp.sec*10**9+stamp.nanosec
while reader.has_next():
    name,raw,stamp=reader.read_next()
    msg=deserialize_message(raw,topics[name])
    if name=='/tf':
        for tf in msg.transforms:
            series[tf.header.frame_id+'->'+tf.child_frame_id].append((stamp,ns(tf.header.stamp)))
    else:
        series[name].append((stamp,ns(msg.clock if name=='/clock' else msg.header.stamp)))
result={'run':RUN.name,'bag_sha256':sha,'read_only':True,'topics':{},'event_window':{},
 'limitations':['Bag recorded with use-sim-time: bag timestamp is recorder ROS clock, NOT wall receipt time.',
 'Independent subscribers may receive different subsets/order; this cannot reconstruct mission/Nav2 private TF buffers.',
 'Observed monotonicity cannot prove unique /clock publisher; next runtime should enumerate publisher endpoints.',
 'TF map->odom can be future-dated by SLAM transform_timeout; that is distinct from a wall/sim clock mismatch.']}
for name,rows in series.items():
    nonzero=[r for r in rows if r[1]>0]
    ages=[(b-s)/1e9 for b,s in nonzero]
    distinct=[]
    for row in rows:
        if not distinct or row[1]!=distinct[-1][1]: distinct.append(row)
    gaps=[(y[0]-x[0])/1e9 for x,y in zip(distinct,distinct[1:])]
    result['topics'][name]={'count':len(rows),'header_first_s':rows[0][1]/1e9,'header_last_s':rows[-1][1]/1e9,
      'min_header_s':min(r[1] for r in rows)/1e9,'max_header_s':max(r[1] for r in rows)/1e9,
      'header_backwards_count':sum(y[1]<x[1] for x,y in zip(rows,rows[1:])),
      'bag_minus_header_min_s':min(ages) if ages else None,'bag_minus_header_max_s':max(ages) if ages else None,
      'max_bag_time_between_header_changes_s':max(gaps,default=0)}
    if name!='/clock':
        result['event_window'][name]=[{'bag_ros_s':b/1e9,'header_s':s/1e9,'bag_minus_header_s':(b-s)/1e9}
          for b,s in distinct if 113.2e9<=b<=115.2e9]
capture=json.loads((RUN/'parameters-early.json').read_text())
result['parameter_receipts']={node:{'status':row['status'],'use_sim_time':row['values'].get('use_sim_time')}
  for node,row in capture['parameters'].items()}
with OUT.open('x') as stream:json.dump(result,stream,indent=2)
print(json.dumps({'output':str(OUT),'topics':result['topics'],'parameter_receipts':result['parameter_receipts']},indent=2))
