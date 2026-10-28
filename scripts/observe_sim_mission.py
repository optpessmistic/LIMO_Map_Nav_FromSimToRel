#!/usr/bin/env python3
"""Read-only Gazebo mission diagnostics. SIGINT saves; it never controls motion."""
import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import signal
import threading
import time


class SampleGate:
    """Monotonic wall-time sampling: no catch-up burst after a delayed callback."""
    def __init__(self, hz):
        self.period=1./hz;self.last=None
    def due(self, elapsed):
        if not math.isfinite(elapsed):raise ValueError('Nonfinite sampling time')
        if self.last is not None and elapsed-self.last < self.period-1e-12:return False
        self.last=elapsed;return True


def sample_receipt(latest, elapsed, previous_sequence):
    """Keep receipt age/sequence explicit; a stale cached value is not a new command."""
    if latest is None:return None
    count=latest['sequence']-previous_sequence
    return {**latest,'receipt_age_wall_s':elapsed-latest['receipt_wall_s'],
        'new_message_since_previous_sample':count>0,'messages_since_previous_sample':count}


def twist_values(twist):
    values=[twist.linear.x,twist.linear.y,twist.linear.z,twist.angular.x,twist.angular.y,twist.angular.z]
    if not all(math.isfinite(v) for v in values):raise ValueError('Nonfinite velocity sample')
    return values


def decode_parameter(value):
    fields={1:'bool_value',2:'integer_value',3:'double_value',4:'string_value',
            5:'byte_array_value',6:'bool_array_value',7:'integer_array_value',8:'double_array_value',9:'string_array_value'}
    if value.type==0:return None
    data=getattr(value,fields[value.type])
    return list(data) if value.type>=5 else data


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def path_record(msg):
    """Preserve stamped path geometry; an unset orientation is not a measured yaw."""
    def ns(header):return header.stamp.sec*1_000_000_000+header.stamp.nanosec
    poses=[]
    for item in msg.poses:
        p,q=item.pose.position,item.pose.orientation
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w]
        if not all(math.isfinite(v) for v in values):raise ValueError('Nonfinite path pose')
        norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        poses.append({'xyz':[p.x,p.y,p.z],'yaw':yaw(q) if abs(norm-1.)<1e-3 else None,
            'quaternion_xyzw':[q.x,q.y,q.z,q.w],'orientation_valid':abs(norm-1.)<1e-3,
            'frame_id':item.header.frame_id or msg.header.frame_id,'stamp_ns':ns(item.header)})
    return {'stamp_ns':ns(msg.header),'frame_id':msg.header.frame_id,'poses':poses,
            'pose_count':len(poses),'all_pose_frames_match_path':all(p['frame_id']==msg.header.frame_id for p in poses)}


def path_geometry_key(record):
    # Re-stamping an otherwise identical plan does not constitute a geometry change.
    return (record['frame_id'],tuple((p['frame_id'],*p['xyz'],*p['quaternion_xyzw']) for p in record['poses']))


def native_tail(samples, now_wall, terminal_ros=None):
    if len(samples)<2:
        return {'evidence_sufficient':False,'reason':'insufficient_native_samples'}
    end=samples[-1]['sim_s']; rows=[p for p in samples if p['sim_s']>=end-30.]
    if len(rows)<2:
        return {'evidence_sufficient':False,'reason':'insufficient_tail_samples'}
    gaps=[b['sim_s']-a['sim_s'] for a,b in zip(rows,rows[1:])]
    angles=[math.remainder(p['yaw']-rows[0]['yaw'],2*math.pi) for p in rows]
    span=rows[-1]['sim_s']-rows[0]['sim_s']
    xspan=max(p['x'] for p in rows)-min(p['x'] for p in rows)
    yspan=max(p['y'] for p in rows)-min(p['y'] for p in rows)
    aspan=max(angles)-min(angles)
    fresh=now_wall-samples[-1]['receipt_wall_elapsed_s']<=1.
    sufficient=span>=29.9 and max(gaps)<=.1 and min(gaps)>0 and fresh
    stationary=sufficient and math.hypot(xspan,yspan)<=.001 and aspan<=.002
    after_terminal=terminal_ros is not None and rows[0]['sim_s']>=terminal_ros
    return {'sample_count':len(rows),'first_sim_s':rows[0]['sim_s'],'last_sim_s':end,'span_s':span,
        'max_gap_s':max(gaps),'x_span_m':xspan,'y_span_m':yspan,'yaw_span_rad':aspan,
        'latest_native_receipt_fresh':fresh,'evidence_sufficient':sufficient,
        'stationary_within_1mm_002rad':stationary,'entire_window_after_terminal':after_terminal,
        'ready_for_operator_to_end_observer':bool(stationary and after_terminal)}


def disk_conflicts(record, position, radius=.25):
    """Exact pose disk versus raw occupancy rectangles; no prior/free-space edits."""
    m=record['map'];res=m['resolution'];ox,oy,angle=m['origin']
    dx,dy=position[0]-ox,position[1]-oy;c,s=math.cos(angle),math.sin(angle)
    gx,gy=c*dx+s*dy,-s*dx+c*dy;ix,iy=math.floor(gx/res),math.floor(gy/res)
    counts={'unknown':0,'occupied_or_intermediate':0,'outside':0};reach=math.ceil(radius/res)+1
    for y in range(iy-reach,iy+reach+1):
        for x in range(ix-reach,ix+reach+1):
            distance=math.hypot(max(x*res-gx,0,gx-(x+1)*res),max(y*res-gy,0,gy-(y+1)*res))
            if distance>radius:continue
            if not (0<=x<m['width'] and 0<=y<m['height']):counts['outside']+=1
            else:
                v=record['map_data'][y*m['width']+x]
                if v<0:counts['unknown']+=1
                elif v!=0:counts['occupied_or_intermediate']+=1
    return {'radius_m':radius,'raw_exact_pose_disk_clear':not any(counts.values()),'conflict_counts':counts}


def parse_arguments(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('outputs/ros_smoke/mission-observer.json'))
    parser.add_argument('--seconds',type=float,default=1000.,help='Wall-time window, 10..1500 seconds; SIGINT saves earlier')
    args=parser.parse_args(argv)
    if not math.isfinite(args.seconds) or not 10<=args.seconds<=1500:
        parser.error('--seconds must be 10..1500 wall seconds; a short window cannot establish a 30sim-second stop')
    snapshots=args.output.with_suffix('').with_name(args.output.stem+'-snapshots')
    if args.output.exists() or snapshots.exists():parser.error('Refusing existing output or snapshot directory')
    return args,snapshots


def main():
    args,snapshots=parse_arguments()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    stream=args.output.open('x',encoding='utf-8');snapshots.mkdir()
    began=time.monotonic();node=ros=gz=listener=None;lock=threading.Lock();world=[];counts=Counter()
    report={'schema_version':1,'read_only':True,'application_publishers':[],'status':'error',
        'requested_wall_seconds':args.seconds,'world_topic':'/world/limo_maze/pose/info','world_entity':'limo',
        'truth_used_only_for_offline_diagnosis':True,'status_changes':[],'target_changes':[],
        'target_observations':[],'slam_poses':[],'mission_events':[],'snapshot_files':[],'errors':[],
        'continuous_tf_samples':[],'command_samples':[],'plan_samples':[],
        'sampling':{'tf_wall_hz':5.,'command_wall_hz':10.,'tf_reference':'latest map-to-base_link stamp; other transforms queried at exactly this stamp',
            'command_topics':['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel'],
            'command_time_basis':'Twist has no header. ROS clock and monotonic wall values are receipt times, not acquisition timestamps.',
            'command_sampling':'Latest received message per topic at each wall-time tick; receipt age and sequence expose stale/reused values and skipped messages.',
            'plan_topic':'/plan','plan_wall_hz_max':1.,
            'plan_sampling':'At most 1Hz wall-time, save latest newly received Path in full. Source/pose header stamps, frames and receipt times retained. Unchanged source is not duplicated; intermediate plans may be skipped.'},
        'limitations':['No motion/goal actions or parameter writes; mission and controller each receive one read-only GetParameters request. SIGINT stops only this observer.',
            'Native world trajectory is never transformed into or published to the controller.',
            'Goal snapshots are near log receipt, not an atomic mission/map transaction; all acquisition timestamps are retained.',
            'Snapshot footprint test uses only raw occupancy and radius .25m, not the externally guaranteed startup prior.',
            'Physical stationarity requires advancing native samples; wheel odometry is not substituted.',
            'Velocity-topic samples are asynchronous receipt records, not matched pipeline messages. Short transients may occur between 10Hz samples.',
            'Latest /odom velocity is not interpolated to the TF time; its own header stamp, child frame, receipt time and age are retained.',
            '/plan is the published global path, not MPPI local trajectories or the controller-internal transformed/pruned plan. Geometry changes ignore timestamp-only differences. Invalid/unset pose orientation has yaw=null.']}
    command_topics=report['sampling']['command_topics']
    state={'status':None,'map':None,'epoch':None,'terminal_ros':None,'goals':{},'pending':[],'snap_index':0,
           'active_goal_token':None,'odom':None,'plan':None,'commands':{topic:None for topic in command_topics}}
    previous_commands={topic:0 for topic in command_topics};previous_odom=0;previous_tf_stamp=None
    command_gate=SampleGate(10.);tf_gate=SampleGate(5.);plan_gate=SampleGate(1.)
    previous_plan=0;previous_path_geometry=None
    stop={'requested':False}
    previous_sigint=signal.getsignal(signal.SIGINT)
    def request_stop(_signum,_frame):stop['requested']=True
    signal.signal(signal.SIGINT,request_stop)
    def fault(exc):
        counts['callback_errors']+=1
        if len(report['errors'])<100:report['errors'].append(str(exc))
    try:
        if os.environ.get('ROS_DOMAIN_ID')!='73' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
            raise ValueError('Require scripts/env.sh domain73 LOCALHOST environment')
        import rclpy
        from rclpy.parameter import Parameter
        from rclpy.signals import SignalHandlerOptions
        from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
        from rclpy.time import Time
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
        from rcl_interfaces.msg import Log
        from rcl_interfaces.srv import GetParameters
        from std_msgs.msg import String
        from visualization_msgs.msg import MarkerArray
        from tf2_ros import Buffer,TransformListener
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V
        ros=rclpy;ros.init(signal_handler_options=SignalHandlerOptions.NO)
        node=ros.create_node('limo_sim_mission_observer',parameter_overrides=[Parameter('use_sim_time',value=True)],
                             enable_rosout=False,start_parameter_services=False)
        buffer=Buffer();listener=TransformListener(buffer,node)
        parameter_names=['goal_arrival_margin_m','robot_radius_m','mission_timeout_s','return_timeout_s',
            'initial_clearance_radius_m','initial_clearance_margin_m','initial_clearance_source',
            'home_position_tolerance_m','home_yaw_tolerance_rad','goal_timeout_s','autonomous','use_sim_time']
        parameter_client=node.create_client(GetParameters,'/limo_maze_mission/get_parameters')
        parameter_future=None;parameter_done=False
        controller_names=['controller_plugins','FollowPath.GoalAngleCritic.threshold_to_consider',
                          'FollowPath.GoalAngleCritic.enabled','FollowPath.GoalAngleCritic.cost_weight']
        controller_client=node.create_client(GetParameters,'/controller_server/get_parameters')
        controller_future=None;controller_done=False
        def entry():return {'wall_s':time.monotonic()-began,'ros_s':node.get_clock().now().nanoseconds/1e9}
        def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec/1e9
        def stamp_ns(m):return m.header.stamp.sec*1_000_000_000+m.header.stamp.nanosec
        def transform_record(tf):
            p,q=tf.transform.translation,tf.transform.rotation
            row={'parent':tf.header.frame_id,'child':tf.child_frame_id,'stamp_ns':stamp_ns(tf),
                 'xyz':[p.x,p.y,p.z],'quaternion_xyzw':[q.x,q.y,q.z,q.w],'yaw':yaw(q)}
            json.dumps(row,allow_nan=False);return row
        def continuous_tf(elapsed):
            nonlocal previous_odom,previous_tf_stamp
            row={**entry(),'map_epoch':state['epoch'],'active_goal_token':state['active_goal_token'],
                 'transforms':{},'tf_errors':{}}
            try:
                base=buffer.lookup_transform('map','base_link',Time())
                row['transforms']['map_to_base_link']=transform_record(base)
                ns=stamp_ns(base);row['reference_stamp_ns']=ns
                if ns<=0:raise ValueError('Nonpositive dynamic TF reference stamp')
                row['reference_stamp_advanced']=previous_tf_stamp is None or ns>previous_tf_stamp
                row['tf_age_ros_s']=row['ros_s']-ns*1e-9;previous_tf_stamp=ns
                for key,parent,child in (('map_to_odom','map','odom'),('odom_to_base_link','odom','base_link')):
                    try:
                        tf=buffer.lookup_transform(parent,child,Time(nanoseconds=ns))
                        row['transforms'][key]=transform_record(tf)
                        if stamp_ns(tf)!=ns:raise ValueError('TF response stamp does not match requested reference')
                    except Exception as exc:row['tf_errors'][key]=str(exc)
                goal=state['goals'].get(state['active_goal_token'])
                if goal:
                    p=base.transform.translation;row['active_goal']=goal
                    row['goal_xy_error_m']=math.hypot(p.x-goal['x'],p.y-goal['y'])
                    row['goal_yaw_error_rad']=math.remainder(yaw(base.transform.rotation)-goal['yaw'],2*math.pi)
            except Exception as exc:row['tf_errors']['map_to_base_link']=str(exc)
            row['same_stamp_chain_complete']=not row['tf_errors'] and len(row['transforms'])==3
            row['latest_odom']=sample_receipt(state['odom'],row['wall_s'],previous_odom)
            if state['odom'] is not None:
                previous_odom=state['odom']['sequence'];row['latest_odom']['header_age_ros_s']=row['ros_s']-state['odom']['stamp_ns']*1e-9
                if 'reference_stamp_ns' in row:
                    row['latest_odom']['stamp_minus_tf_reference_s']=(state['odom']['stamp_ns']-row['reference_stamp_ns'])*1e-9
            json.dumps(row,allow_nan=False);report['continuous_tf_samples'].append(row)
        def sample_commands(elapsed):
            row={**entry(),'active_goal_token':state['active_goal_token'],'topics':{}}
            for topic in command_topics:
                latest=state['commands'][topic]
                row['topics'][topic]=sample_receipt(latest,row['wall_s'],previous_commands[topic])
                if latest is not None:previous_commands[topic]=latest['sequence']
            report['command_samples'].append(row)
        def sample_plan(elapsed):
            nonlocal previous_plan,previous_path_geometry
            latest=state['plan']
            if latest is None or latest['sequence']==previous_plan:return
            # Convert only the selected latest Path, keeping callback cost independent of path length.
            record=path_record(latest['message']);geometry=path_geometry_key(record)
            row={**entry(),**record,'map_epoch':state['epoch'],'active_goal_token':state['active_goal_token'],
                'receipt_wall_s':latest['receipt_wall_s'],'receipt_ros_s':latest['receipt_ros_s'],
                'active_goal_token_at_receipt':latest['active_goal_token_at_receipt'],
                'sequence':latest['sequence'],'messages_since_previous_sample':latest['sequence']-previous_plan,
                'receipt_age_wall_s':elapsed-latest['receipt_wall_s'],
                'geometry_changed_since_previous_sample':previous_path_geometry is None or geometry!=previous_path_geometry}
            row['header_age_ros_s']=row['ros_s']-record['stamp_ns']*1e-9
            report['plan_samples'].append(row);previous_plan=latest['sequence'];previous_path_geometry=geometry
        def snapshot(reason,token=None):
            item={**entry(),'reason':reason,'token':token,'map_epoch':state['epoch'],'mission_status':state['status']}
            item['goal']=state['goals'].get(token)
            if state['map'] is not None:
                m=state['map'];p=m.info.origin.position;q=m.info.origin.orientation
                item.update(map={'stamp':stamp(m),'frame':m.header.frame_id,'width':m.info.width,'height':m.info.height,
                    'resolution':m.info.resolution,'origin':[p.x,p.y,yaw(q)]},map_data=list(m.data))
                item['map_age_s']=item['ros_s']-stamp(m)
            try:
                tf=buffer.lookup_transform('map','base_link',Time());p=tf.transform.translation;q=tf.transform.rotation
                item['pose']={'stamp':stamp(tf),'xyz':[p.x,p.y,p.z],'yaw':yaw(q)}
                item['pose_age_s']=item['ros_s']-stamp(tf)
                if item.get('goal'):
                    goal=item['goal'];item['goal_xy_error_m']=math.hypot(p.x-goal['x'],p.y-goal['y'])
                    item['goal_yaw_error_rad']=math.remainder(yaw(q)-goal['yaw'],2*math.pi)
                if 'map' in item:
                    radius=report.get('mission_parameters',{}).get('values',{}).get('robot_radius_m',.25)
                    item['raw_footprint']=disk_conflicts(item,[p.x,p.y],radius)
            except Exception as exc:item['pose_error']=str(exc)
            state['snap_index']+=1;path=snapshots/f"{state['snap_index']:04d}-{reason}.json"
            with path.open('x',encoding='utf-8') as handle:json.dump(item,handle,allow_nan=False)
            report['snapshot_files'].append(str(path))
        def native(msg):
            try:
                for p in msg.pose:
                    if p.name!='limo':continue
                    t=msg.header.stamp.sec+msg.header.stamp.nsec/1e9
                    row={'sim_s':t,'x':p.position.x,'y':p.position.y,'z':p.position.z,'yaw':yaw(p.orientation),
                         'id':p.id,'receipt_wall_elapsed_s':time.monotonic()-began}
                    if not all(math.isfinite(row[k]) for k in ('sim_s','x','y','z','yaw')):raise ValueError('Nonfinite native pose')
                    with lock:
                        if world and t<world[-1]['sim_s']:raise ValueError('Native simulation clock moved backwards')
                        if not world or t-world[-1]['sim_s']>=.019:world.append(row)
                    return
            except Exception as exc:fault(exc)
        def status(msg):
            value=json.loads(msg.data);json.dumps(value,allow_nan=False);counts['status']+=1
            if value!=state['status']:
                report['status_changes'].append({**entry(),'value':value})
                previous=state['status'];state['status']=value
                if previous is None or previous.get('phase')!=value.get('phase'):
                    print('PHASE '+json.dumps({**entry(),'value':value}),flush=True);snapshot('phase-'+value.get('phase','unknown'))
            if value.get('phase') in ('blocked','done') and state['terminal_ros'] is None:state['terminal_ros']=entry()['ros_s']
        def log(msg):
            if msg.name!='limo_maze_mission':return
            name,_,payload=msg.msg.partition(' ')
            if name not in ('goal_sent','goal_terminal','cancel_requested','home_recorded'):return
            fields=json.loads(payload);json.dumps(fields,allow_nan=False)
            row={**entry(),'log_stamp':msg.stamp.sec+msg.stamp.nanosec/1e9,'event':name,**fields}
            report['mission_events'].append(row)
            if name=='goal_sent':
                state['goals'][fields['token']]=fields['goal'];state['active_goal_token']=fields['token']
            if name=='goal_terminal':
                snapshot('goal-terminal-receipt',fields['token']);state['pending'].append((time.monotonic()+1.,fields['token']))
                if state['active_goal_token']==fields['token']:state['active_goal_token']=None
        def targets(msg):
            row={**entry(),'markers':[{'id':m.id,'ns':m.ns,'action':m.action,
                'frame':m.header.frame_id,'position':[m.pose.position.x,m.pose.position.y,m.pose.position.z],'text':m.text} for m in msg.markers]}
            json.dumps(row,allow_nan=False);report['target_changes'].append(row)
        def observations(msg):
            value=json.loads(msg.data);json.dumps(value,allow_nan=False);report['target_observations'].append({**entry(),'value':value})
        def pose(msg):
            p,q=msg.pose.pose.position,msg.pose.pose.orientation
            row={'stamp':stamp(msg),'frame':msg.header.frame_id,'position':[p.x,p.y,p.z],'quaternion':[q.x,q.y,q.z,q.w]}
            json.dumps(row,allow_nan=False);report['slam_poses'].append(row)
        def command(topic,msg):
            values=twist_values(msg);counts[topic]+=1;received=entry()
            state['commands'][topic]={'sequence':counts[topic],'receipt_wall_s':received['wall_s'],
                'receipt_ros_s':received['ros_s'],'has_header_stamp':False,'twist_linear_xyz_angular_xyz':values}
        def odom(msg):
            p,q=msg.pose.pose.position,msg.pose.pose.orientation;counts['odom']+=1;received=entry()
            row={'sequence':counts['odom'],'receipt_wall_s':received['wall_s'],'receipt_ros_s':received['ros_s'],
                'stamp_ns':stamp_ns(msg),'parent':msg.header.frame_id,'child':msg.child_frame_id,
                'xyz':[p.x,p.y,p.z],'quaternion_xyzw':[q.x,q.y,q.z,q.w],'yaw':yaw(q),
                'twist_linear_xyz_angular_xyz':twist_values(msg.twist.twist)}
            json.dumps(row,allow_nan=False);state['odom']=row
        def plan(msg):
            counts['plan']+=1;received=entry()
            state['plan']={'message':msg,'sequence':counts['plan'],
                'receipt_wall_s':received['wall_s'],'receipt_ros_s':received['ros_s'],
                'active_goal_token_at_receipt':state['active_goal_token']}
        def guarded(fn):
            def cb(msg):
                try:fn(msg)
                except Exception as exc:fault(exc)
            return cb
        latched=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for kind,topic,callback,qos in ((String,'/mission/status',status,latched),
            (String,'/mission/map_epoch',lambda m:state.update(epoch=m.data),latched),
            (MarkerArray,'/mission/targets',targets,latched),(String,'/mission/target_observations',observations,qos_profile_sensor_data),
            (OccupancyGrid,'/map',lambda m:state.update(map=m),latched),
            (PoseWithCovarianceStamped,'/pose',pose,1000),(Log,'/rosout',log,1000),
            (Odometry,'/odom',odom,qos_profile_sensor_data),(NavPath,'/plan',plan,qos_profile_sensor_data)):
            node.create_subscription(kind,topic,guarded(callback),qos)
        for topic in command_topics:
            node.create_subscription(Twist,topic,guarded(lambda msg,topic=topic:command(topic,msg)),qos_profile_sensor_data)
        gz=GzNode()
        if not gz.subscribe(Pose_V,report['world_topic'],native):raise RuntimeError('Native subscription failed')
        print('READY: simulation observer subscriptions created; waiting for native data. No motion/action commands.',flush=True)
        next_progress=0.
        while time.monotonic()-began<args.seconds and not stop['requested']:
            ros.spin_once(node,timeout_sec=.05);elapsed=time.monotonic()-began
            if not parameter_done:
                if parameter_future is None and parameter_client.service_is_ready():
                    request=GetParameters.Request();request.names=parameter_names
                    parameter_future=parameter_client.call_async(request)
                if parameter_future is not None and parameter_future.done():
                    parameter_done=True
                    try:
                        values=parameter_future.result().values
                        decoded={name:decode_parameter(value) for name,value in zip(parameter_names,values)}
                        report['mission_parameters']={**entry(),'service':'/limo_maze_mission/get_parameters','values':decoded,
                                                       'types':{name:int(v.type) for name,v in zip(parameter_names,values)}}
                        print('MISSION_PARAMETERS '+json.dumps(report['mission_parameters']),flush=True)
                    except Exception as exc:fault('GetParameters: '+str(exc))
            if not controller_done:
                if controller_future is None and controller_client.service_is_ready():
                    request=GetParameters.Request();request.names=controller_names
                    controller_future=controller_client.call_async(request)
                if controller_future is not None and controller_future.done():
                    controller_done=True
                    try:
                        values=controller_future.result().values
                        decoded={name:decode_parameter(value) for name,value in zip(controller_names,values)}
                        report['controller_parameters']={**entry(),'service':'/controller_server/get_parameters','values':decoded,
                            'types':{name:int(v.type) for name,v in zip(controller_names,values)}}
                        print('CONTROLLER_PARAMETERS '+json.dumps(report['controller_parameters']),flush=True)
                        if decoded.get('FollowPath.GoalAngleCritic.threshold_to_consider') is None:
                            fault('FollowPath.GoalAngleCritic.threshold_to_consider is unavailable; no assumed value substituted')
                    except Exception as exc:fault('Controller GetParameters: '+str(exc))
            if tf_gate.due(elapsed):continuous_tf(elapsed)
            if command_gate.due(elapsed):sample_commands(elapsed)
            if plan_gate.due(elapsed):
                try:sample_plan(elapsed)
                except Exception as exc:fault('Path sample: '+str(exc))
            for due,token in list(state['pending']):
                if time.monotonic()>=due:snapshot('goal-terminal-plus-1wall',token);state['pending'].remove((due,token))
            if elapsed>=next_progress:
                with lock:tail=native_tail(world,elapsed,state['terminal_ros'])
                print('NATIVE_TAIL '+json.dumps({'wall_s':elapsed,'count':len(world),'tail':tail}),flush=True)
                next_progress=elapsed+10.
        report['status']='operator_interrupt_saved' if stop['requested'] else 'window_complete'
    except KeyboardInterrupt:
        report['status']='operator_interrupt_saved'
    except Exception as exc:
        report['error']=f'{type(exc).__name__}: {exc}'
    finally:
        if gz is not None:
            try:gz.unsubscribe(report['world_topic'])
            except Exception as exc:fault(exc)
        with lock:report['world_samples']=list(world)
        if node is not None:
            try:snapshot('observer-final')
            except Exception as exc:fault(exc)
        report['wall_seconds']=time.monotonic()-began;report['final_status']=state['status']
        report['terminal_ros_s']=state['terminal_ros'];report['message_counts']=dict(counts)
        report['final_native_tail_30sim']=native_tail(report['world_samples'],report['wall_seconds'],state['terminal_ros'])
        report['goal_terminal_log_count']=sum(e['event']=='goal_terminal' for e in report['mission_events'])
        if 'mission_parameters' not in report:report['errors'].append('Mission GetParameters response not captured')
        if 'controller_parameters' not in report:report['errors'].append('Controller GetParameters response not captured')
        report['continuous_summary']={'tf_sample_count':len(report['continuous_tf_samples']),
            'same_stamp_chain_complete_count':sum(row['same_stamp_chain_complete'] for row in report['continuous_tf_samples']),
            'command_sample_count':len(report['command_samples']),
            'raw_received_command_counts':{topic:counts[topic] for topic in command_topics},'raw_received_odom_count':counts['odom'],
            'plan_sample_count':len(report['plan_samples']),'raw_received_plan_count':counts['plan']}
        if not report['plan_samples']:report['errors'].append('No /plan samples captured; global path analysis unavailable')
        if not world or state['status'] is None:
            report['status']='missing_required_evidence';report['errors'].append('Missing native world poses or mission status')
        try:
            if listener is not None:listener.unregister()
            if node is not None:node.destroy_node()
            if ros is not None and ros.ok():ros.shutdown()
        except Exception as exc:fault(exc)
        with stream:json.dump(report,stream,indent=2,allow_nan=False);stream.write('\n')
        signal.signal(signal.SIGINT,previous_sigint)
        print('SAVED '+json.dumps({'path':str(args.output),'status':report['status'],'native_count':len(world),
            'goal_terminal_logs':report['goal_terminal_log_count'],'tail':report['final_native_tail_30sim']}),flush=True)
    return 0 if report['status'] in ('window_complete','operator_interrupt_saved') and not report['errors'] else 2


if __name__=='__main__':raise SystemExit(main())
