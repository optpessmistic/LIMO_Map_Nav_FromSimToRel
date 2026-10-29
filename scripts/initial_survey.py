#!/usr/bin/env python3
"""Optional initial rotation in isolated simulation; defaults to read-only.

Motion requires --execute --verified-clear-radius .30 (or greater), ROS domain
73, simulation time, an idle Nav2 chain and no mission/exploration node. The
radius is an INDEPENDENT setup guarantee about unseen space, never inferred
from a partial scan or world metadata. The swept robot radius is .25 m plus
.03 m clearance. No self-return filtering is performed: use the verified
240-degree scanner configuration. Motion uses Nav2 Spin actions with owned UUIDs, never a custom nonzero Twist
controller. Do not start another operator/controller during the survey.
"""
import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import time
import uuid


class SurveyAbort(ValueError):
    pass


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def authorization(execute, radius, domain, sim_time):
    if execute and (domain != '73' or sim_time is not True or radius is None
                    or not math.isfinite(radius) or radius < .30):
        raise SurveyAbort('Motion requires domain 73, use_sim_time=true and an explicit verified radius >=.30m')


def stop_burst(publish_zero, spin_once, is_ok, duration=2.0, clock=time.monotonic):
    """Continue zero attempts despite callback/publish errors; report failures."""
    started, last_zero, count, errors = clock(), -math.inf, 0, []
    def error(exc):
        message = type(exc).__name__+': '+str(exc)
        if message not in errors:
            errors.append(message)
    while clock()-started < duration:
        try:
            if not is_ok():
                raise RuntimeError('ROS context unavailable during stop burst')
            if clock()-last_zero >= .05:
                last_zero = clock()
                publish_zero()
                count += 1
        except Exception as exc:
            error(exc)
        try:
            spin_once()
        except Exception as exc:
            error(exc)  # A broken callback must never interrupt remaining zero commands.
            time.sleep(.005)
    return count, errors


def scan_clearance(ranges, angle_min, increment, range_min, range_max, translation, quaternion):
    """Minimum base-frame XY distance; transform *each* laser endpoint.

    Positive infinity denotes no hit. NaN is an unknown ray; neither is free
    space evidence. Negative infinity/below-min returns may hide close objects
    or self hits and therefore reject this baseline rather than filtering them.
    """
    values = (angle_min, increment, range_min, range_max, *translation, *quaternion)
    if (not all(math.isfinite(v) for v in values) or increment <= 0
            or not 0 <= range_min < range_max or len(ranges) < 10
            or abs(sum(v*v for v in quaternion)-1) > .01):
        raise SurveyAbort('Invalid scan geometry or laser-to-base transform')
    qx, qy, qz, qw = quaternion
    closest, valid = math.inf, 0
    for i, r in enumerate(ranges):
        if r == -math.inf or (math.isfinite(r) and r < range_min):
            raise SurveyAbort('Below-minimum scan return; no self-echo filtering is allowed')
        if not math.isfinite(r) or r > range_max:
            continue
        x, y = r*math.cos(angle_min+i*increment), r*math.sin(angle_min+i*increment)
        tx, ty, tz = -2*qz*y, 2*qz*x, 2*(qx*y-qy*x)
        bx = x+qw*tx+qy*tz-qz*ty+translation[0]
        by = y+qw*ty+qz*tx-qx*tz+translation[1]
        closest, valid = min(closest, math.hypot(bx, by)), valid+1
    if valid < 10:
        raise SurveyAbort('Insufficient finite scan returns to monitor rotation')
    return closest, valid


@dataclass
class Survey:
    clock: tuple | None = None
    odom: tuple | None = None
    scan: tuple | None = None
    home: tuple | None = None
    phase: str = 'preflight'
    yaw_total: float = 0.0
    max_yaw: float = 0.0
    max_displacement: float = 0.0
    settle_since: float | None = None

    def on_clock(self, stamp, wall):
        if not math.isfinite(stamp) or stamp < 0:
            raise SurveyAbort('Invalid simulation clock')
        if self.clock and (stamp < self.clock[0] or stamp-self.clock[0] > .5):
            raise SurveyAbort('Simulation clock reset/jump')
        if self.clock is None or stamp > self.clock[0]:
            self.clock = (stamp, wall)

    def on_odom(self, stamp, wall, x, y, yaw, linear, angular):
        if abs(angular) > .45:
            raise SurveyAbort('Measured angular speed exceeded .45rad/s guard')
        if not all(math.isfinite(v) for v in (stamp, wall, x, y, yaw, linear, angular)):
            raise SurveyAbort('Nonfinite odometry')
        if self.odom:
            dt = stamp-self.odom[0]
            distance = math.hypot(x-self.odom[2], y-self.odom[3])
            turn = angle_delta(yaw, self.odom[4])
            if dt < 0 or dt > .5 or abs(turn) > .45*dt+.015 or distance > .05*max(dt, 0)+.005:
                raise SurveyAbort('Odometry time/pose jump')
            if dt == 0:
                if distance > 1e-8 or abs(turn) > 1e-8:
                    raise SurveyAbort('Pose changed at duplicate odometry timestamp')
                return False
            if self.home:
                self.yaw_total += turn
                self.max_yaw = max(self.max_yaw, self.yaw_total)
        self.odom = (stamp, wall, x, y, yaw, linear, angular)
        if self.home:
            self.max_displacement = max(self.max_displacement, math.hypot(x-self.home[2], y-self.home[3]))
            if self.max_displacement > .03:
                raise SurveyAbort('Translation exceeded .03m')
        return True

    def check(self, wall):
        if self.clock is None or self.odom is None or self.scan is None:
            raise SurveyAbort('Waiting for clock, odometry and transformed scan')
        if wall-self.clock[1] > .75:
            raise SurveyAbort('Simulation clock stopped/stale')
        for name, sample in (('odom', self.odom), ('scan', self.scan)):
            if wall-sample[1] > 1 or not -.10 <= self.clock[0]-sample[0] <= .5:
                raise SurveyAbort(name+' stale or outside simulation clock interval')
        if self.scan[2] <= .28:
            raise SurveyAbort('Observed obstacle inside .25m rotation envelope + .03m margin')

    def start(self, wall):
        self.check(wall)
        if self.odom[5] > .01 or abs(self.odom[6]) > .02:
            raise SurveyAbort('Robot is moving during preflight')
        self.home, self.phase = self.odom, 'rotate'


def foreign_active(status_entries, own_ids, topic):
    return any(status in (1, 2, 3) and not (topic == '/spin/_action/status' and goal_id in own_ids)
               for goal_id, status in status_entries)


def stopped_after(odom, clock, wall, stage_stamp, accepted_stamps):
    """Require accepted, advancing observations acquired after this stop stage."""
    return bool(odom and clock and stage_stamp is not None
                and sum(stamp > stage_stamp for stamp in accepted_stamps) >= 3
                and odom[0] > stage_stamp and wall-odom[1] < 1 and wall-clock[1] < .75
                and -.1 <= clock[0]-odom[0] <= .5
                and odom[5] <= .005 and abs(odom[6]) <= .02)


class OwnedSpin:
    # Keeps cancellation intent while send_goal is pending; a late acceptance is canceled.
    def __init__(self, goal_id, send_future, target):
        self.goal_id, self.send_future, self.target = goal_id, send_future, target
        self.handle = self.result_future = self.cancel_future = None
        self.cancel_requested = self.cancel_ack = False
        self.terminal = self.result_code = None
        self.errors = []

    def poll(self):
        errors = []
        def failed(operation, exc):
            message = operation+': '+type(exc).__name__+': '+str(exc)
            errors.append(message)
            if message not in self.errors:
                self.errors.append(message)
        if self.handle is None and self.terminal is None and self.send_future.done():
            try:
                self.handle = self.send_future.result()
                if not self.handle.accepted:
                    self.terminal = 'rejected'
            except Exception as exc:
                failed('send result', exc)
        if self.handle is not None and self.handle.accepted and self.terminal is None and self.result_future is None:
            try:
                self.result_future = self.handle.get_result_async()
            except Exception as exc:
                failed('request result', exc)
        if self.result_future is not None and self.result_future.done():
            try:
                result = self.result_future.result()
                self.terminal, self.result_code = result.status, result.result.error_code
            except Exception as exc:
                failed('read result', exc)
                self.result_future = None  # Allow a fresh result request during cleanup.
        # Cancellation and its ACK must run even when either result operation failed.
        if self.cancel_requested and self.handle is not None and self.handle.accepted and self.terminal is None:
            if self.cancel_future is None:
                try:
                    self.cancel_future = self.handle.cancel_goal_async()
                except Exception as exc:
                    failed('request cancel', exc)
        if self.cancel_future is not None and self.cancel_future.done():
            try:
                response = self.cancel_future.result()
                self.cancel_ack = response.return_code == 0 and any(
                    bytes(g.goal_id.uuid) == self.goal_id for g in response.goals_canceling)
            except Exception as exc:
                failed('read cancel', exc)
        if errors:
            raise SurveyAbort('; '.join(errors))

    def record(self):
        return {'uuid': self.goal_id.hex(), 'target_yaw_rad': self.target,
                'accepted': self.handle.accepted if self.handle else None,
                'cancel_requested': self.cancel_requested, 'cancel_ack': self.cancel_ack,
                'terminal_status': self.terminal, 'result_error_code': self.result_code,
                'errors': list(self.errors)}


def run(args):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from rclpy.signals import SignalHandlerOptions
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import Twist
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import Spin
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from unique_identifier_msgs.msg import UUID
    from tf2_ros import Buffer, TransformListener, TransformException

    authorization(args.execute, args.verified_clear_radius, os.environ.get('ROS_DOMAIN_ID'), True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupt)
    node = rclpy.create_node('limo_initial_survey_'+str(os.getpid()),
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    authorization(args.execute, args.verified_clear_radius, os.environ.get('ROS_DOMAIN_ID'),
                  node.get_parameter('use_sim_time').value)
    state, failures, samples, statuses = Survey(), [], [], {}
    epoch, started, own_ids, requests = [None], time.monotonic(), set(), []
    counts = {'scan': 0, 'odom': 0, 'clock': 0}
    odom_stamps = []
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    action = ActionClient(node, Spin, '/spin')
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def guarded(callback):
        def receive(msg):
            try:
                callback(msg)
            except Exception as exc:
                if state.home or not isinstance(exc, TransformException):
                    failures.append(type(exc).__name__+': '+str(exc))
        return receive

    stamp = lambda t: t.sec+t.nanosec*1e-9
    def clock(msg):
        counts['clock'] += 1
        state.on_clock(stamp(msg.clock), time.monotonic())

    def odom(msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id not in ('base_footprint', 'base_link'):
            raise SurveyAbort('Unexpected odometry frames')
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1) > .01:
            raise SurveyAbort('Invalid odometry quaternion')
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        if state.on_odom(stamp(msg.header.stamp), time.monotonic(), p.x, p.y, yaw,
                         math.hypot(v.linear.x, v.linear.y), v.angular.z):
            counts['odom'] += 1
            odom_stamps.append(state.odom[0])

    def scan(msg):
        if abs(msg.angle_increment*(len(msg.ranges)-1)-math.radians(240)) > .03:
            raise SurveyAbort('This baseline requires the verified 240-degree scanner configuration')
        if state.scan and stamp(msg.header.stamp) < state.scan[0]:
            raise SurveyAbort('Scan timestamp moved backwards')
        tf = buffer.lookup_transform('base_link', msg.header.frame_id, Time.from_msg(msg.header.stamp)).transform
        t, q = tf.translation, tf.rotation
        distance, valid = scan_clearance(msg.ranges, msg.angle_min, msg.angle_increment,
                                        msg.range_min, msg.range_max, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        state.scan = (stamp(msg.header.stamp), time.monotonic(), distance, valid)
        counts['scan'] += 1
        if distance <= .28:
            raise SurveyAbort('Observed obstacle inside rotation envelope plus margin')

    def map_epoch(msg):
        if not msg.data.strip() or (epoch[0] is not None and epoch[0] != msg.data):
            raise SurveyAbort('Missing/changed map_epoch')
        epoch[0] = msg.data

    def command_seen(msg):
        values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            raise SurveyAbort('Nonfinite command observed')
        if any(abs(v) > 1e-5 for v in values[:5]) or abs(values[5]) > .45:
            raise SurveyAbort('Observed command incompatible with guarded Spin')
        if not requests and any(abs(v) > 1e-5 for v in values):
            raise SurveyAbort('Nonzero command during preflight')

    subscriptions = [node.create_subscription(typ, topic, guarded(callback), qos_profile_sensor_data)
                     for typ, topic, callback in ((Clock, '/clock', clock), (Odometry, '/odom', odom),
                                                   (LaserScan, '/scan', scan))]
    subscriptions += [node.create_subscription(String, '/mission/map_epoch', guarded(map_epoch), latched),
                      node.create_subscription(Twist, '/cmd_vel_nav', guarded(command_seen), 10)]
    clients = {name: node.create_client(GetState, '/'+name+'/get_state')
               for name in ('behavior_server', 'velocity_smoother', 'collision_monitor')}
    lifecycle, pending, status_subs = {}, {}, {}

    def action_status(topic, msg):
        entries = [(bytes(entry.goal_info.goal_id.uuid), entry.status) for entry in msg.status_list]
        statuses[topic] = [{'uuid': gid.hex(), 'status': status} for gid, status in entries]
        if foreign_active(entries, own_ids, topic):
            failures.append('Other active/accepted/canceling action: '+topic)

    def graph_check(now):
        if any('mission' in name or 'explor' in name for name, ns in node.get_node_names_and_namespaces()):
            raise SurveyAbort('Mission/exploration node present')
        allowed = {'/cmd_vel_nav': {'controller_server', 'behavior_server', 'docking_server'},
                   '/cmd_vel_smoothed': {'velocity_smoother'}, '/cmd_vel': {'collision_monitor'}}
        for topic, names in allowed.items():
            endpoints = node.get_publishers_info_by_topic(topic)
            if not endpoints or any(e.node_name not in names for e in endpoints):
                raise SurveyAbort('Missing or unfamiliar publisher on '+topic)
        for topic, types in node.get_topic_names_and_types():
            if topic.endswith('/_action/status') and 'action_msgs/msg/GoalStatusArray' in types and topic not in status_subs:
                status_subs[topic] = node.create_subscription(GoalStatusArray, topic,
                    guarded(lambda msg, name=topic: action_status(name, msg)), latched)
        for name, client in clients.items():
            if name in pending and pending[name].done():
                lifecycle[name] = (pending.pop(name).result().current_state.id, now)
            if name not in pending and client.service_is_ready():
                pending[name] = client.call_async(GetState.Request())
        if any(name not in lifecycle or lifecycle[name][0] != 3 or now-lifecycle[name][1] > 2 for name in clients):
            raise SurveyAbort('Waiting for active behavior/smoother/collision-monitor chain')
        if not action.server_is_ready() or any(node.count_publishers(topic) == 0 for topic in
                ('/follow_path/_action/status', '/spin/_action/status')):
            raise SurveyAbort('Spin action or controller/behavior status unavailable')

    def send_spin(target):
        goal_id = uuid.uuid4().bytes
        own_ids.add(goal_id)  # Must be registered before any ACCEPTED status can arrive.
        goal = Spin.Goal()
        goal.target_yaw = float(target)
        goal.time_allowance.sec = 60 if abs(target) > 1 else 15
        request = OwnedSpin(goal_id, action.send_goal_async(goal, goal_uuid=UUID(uuid=list(goal_id))), target)
        requests.append(request)
        return request

    reason, success, ready_since, last_graph, last_sample = 'preflight_timeout', False, None, 0, 0
    current, stopped_since, settle_anchor = None, None, None
    try:
        # Reserve 10 seconds for own-goal cancellation/stop within the total wall budget.
        while time.monotonic()-started < args.timeout-10:
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic()
            if failures:
                raise SurveyAbort(failures[0])
            if now-last_graph >= .25:
                last_graph = now
                try:
                    graph_check(now)
                    state.check(now)
                    if not epoch[0] or min(counts.values()) < 3:
                        raise SurveyAbort('Waiting for map_epoch and repeated sensor samples')
                except SurveyAbort as exc:
                    if state.home:
                        raise
                    ready_since, reason = None, str(exc)
                else:
                    ready_since = now if ready_since is None else ready_since
            if state.home is None:
                if now-started > 15:
                    break
                if ready_since is None or now-ready_since < 2:
                    continue
                if not args.execute:
                    success, reason = True, 'read_only_preflight_passed_no_motion'
                    break
                state.start(now)
                current = send_spin(2*math.pi+.1)
            state.check(now)
            current.poll()
            if current.terminal is not None:
                if current.terminal != 4 or current.result_code != 0:
                    raise SurveyAbort('Own Spin did not succeed: '+str(current.record()))
                if settle_anchor is None:
                    settle_anchor = (state.clock[0], counts['odom'])
                stopped = stopped_after(state.odom, state.clock, now, settle_anchor[0],
                                        odom_stamps[settle_anchor[1]:])
                stopped_since = (stopped_since if stopped_since is not None else now) if stopped else None
                if stopped_since is not None and now-stopped_since >= 1:
                    if state.phase == 'rotate':
                        if state.max_yaw < 2*math.pi+.07:
                            raise SurveyAbort('Spin result not corroborated by full odometry rotation')
                        error = angle_delta(state.home[4], state.odom[4])
                        if abs(error) > .03:
                            if abs(error) > .5:
                                raise SurveyAbort('Unexpected heading error after full rotation')
                            current = send_spin(error)
                            state.phase, stopped_since, settle_anchor = 'align', None, None
                        else:
                            state.phase = 'done'
                    elif state.phase == 'align':
                        if abs(angle_delta(state.odom[4], state.home[4])) > .05:
                            raise SurveyAbort('Alignment Spin result not corroborated by heading')
                        state.phase = 'done'
                    if state.phase == 'done':
                        success, reason = True, 'spin_and_odometry_rotation_heading_stop_verified'
                        break
            if now-last_sample >= .1:
                last_sample = now
                samples.append({'wall_s': now-started, 'sim_s': state.clock[0],
                                'yaw_accumulated_rad': state.yaw_total, 'angular_velocity_rad_s': state.odom[6],
                                'x': state.odom[2], 'y': state.odom[3],
                                'minimum_observed_base_distance_m': state.scan[2]})
        else:
            reason = 'wall_timeout'
    except (Exception, KeyboardInterrupt) as exc:
        success, reason = False, type(exc).__name__+': '+str(exc)
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        cleanup_errors, zero_count, stop_verified, cancel_required = [], 0, None, False
        stop_count, stop_started = counts['odom'], time.monotonic()
        stop_stage_stamp = state.clock[0] if state.clock else None
        if current is not None:
            cancel_required = current.terminal is None
            current.cancel_requested |= cancel_required
            terminal_seen, still_since = current.terminal is not None, None
            while time.monotonic()-stop_started < 7 and time.monotonic()-started < args.timeout-2.1:
                try:
                    rclpy.spin_once(node, timeout_sec=.02)
                    current.poll()
                    terminal_seen = current.terminal is not None
                    canceled = (not cancel_required or current.cancel_ack or current.terminal == 'rejected'
                                or (terminal_seen and current.cancel_future is None))
                    if terminal_seen and canceled and stopped_after(
                            state.odom, state.clock, time.monotonic(), stop_stage_stamp, odom_stamps[stop_count:]):
                        still_since = time.monotonic() if still_since is None else still_since
                        if time.monotonic()-still_since >= .5:
                            stop_verified = True
                            break
                    else:
                        still_since = None
                except Exception as exc:
                    message = type(exc).__name__+': '+str(exc)
                    if message not in cleanup_errors:
                        cleanup_errors.append(message)
            if stop_verified is not True:
                stop_verified = False
                success = False
                reason += '; own-goal cancellation/terminal/stop not fully verified'
                # Emergency fallback ONLY after our goal actually caused observed motion.
                moved = state.max_yaw > .001 or state.max_displacement > .001
                if cancel_required and not (terminal_seen and current.cancel_ack) and moved:
                    fallback = node.create_publisher(Twist, '/cmd_vel_nav', 10)
                    zero_count, errors = stop_burst(lambda: fallback.publish(Twist()),
                        lambda: rclpy.spin_once(node, timeout_sec=.01), rclpy.ok,
                        duration=max(0, min(2, args.timeout-(time.monotonic()-started))))
                    cleanup_errors.extend(errors)
                    node.destroy_publisher(fallback)
        if failures or cleanup_errors:
            success = False
            reason = reason+'; '+('; '.join(failures[:2]+cleanup_errors[:2]))
        report = {'schema_version': 2, 'success': success, 'reason': reason, 'executed': args.execute,
                  'motion_started': bool(requests), 'map_epoch': epoch[0],
                  'assumption': {'independently_verified_clear_radius_m': args.verified_clear_radius,
                    'source': 'operator_setup_guarantee_not_scan_or_world_metadata',
                    'rotation_envelope_m': .25, 'clearance_margin_m': .03},
                  'control': 'Nav2 Spin action; no custom nonzero Twist publisher',
                  'behavior_max_angular_rad_s_at_verified_startup': .4, 'odom_angular_abort_rad_s': .45,
                  'wall_seconds': time.monotonic()-started, 'phase': state.phase, 'counts': counts,
                  'yaw_accumulated_rad': state.yaw_total, 'maximum_yaw_rad': state.max_yaw,
                  'max_displacement_m': state.max_displacement,
                  'final_heading_error_rad': angle_delta(state.odom[4], state.home[4]) if state.home else None,
                  'spin_goals': [request.record() for request in requests],
                  'action_statuses_received': statuses, 'action_status_topics_monitored': sorted(status_subs),
                  'cancel_required': cancel_required, 'stop_verified': stop_verified,
                  'stop_stage_stamp': stop_stage_stamp,
                  'stop_new_odom_samples': sum(s > stop_stage_stamp for s in odom_stamps[stop_count:])
                      if stop_stage_stamp is not None else 0,
                  'fallback_zero_messages': zero_count, 'cleanup_errors': cleanup_errors, 'samples': samples,
                  'stop_verification_source': 'wheel_odometry',
                  'physical_motion_independently_verified': False,
                  'limitations': ['Wheel odometry alone cannot prove physical rotation or stopping; compare independent motion evidence.',
                    'Partial-FOV scan cannot establish unseen starting clearance.',
                    'An offset partial-FOV lidar may leave a central blind disk after rotation.',
                    'Action UUID/status and graph monitoring are cooperative, not an atomic ownership lock.',
                    'Rotation does not establish map completion, safe map cells, or search coverage.']}
        try:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
            print(json.dumps({k: report[k] for k in ('success', 'reason', 'wall_seconds', 'stop_verified')}))
        finally:
            action.destroy()
            listener.unregister()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    return 0 if success else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--verified-clear-radius', type=float)
    parser.add_argument('--timeout', type=float, default=150)
    parser.add_argument('--output', type=Path, default=Path('outputs/ros_smoke/initial_survey.json'))
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or not 15 <= args.timeout <= 150:
        parser.error('--timeout must be between 15 and 150 wall seconds')
    if args.verified_clear_radius is not None and (not math.isfinite(args.verified_clear_radius) or args.verified_clear_radius < .30):
        parser.error('--verified-clear-radius must be finite and at least .30m')
    if args.output.exists():
        parser.error('Output exists; choose a new evidence filename')
    try:
        return run(args)
    except SurveyAbort as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    raise SystemExit(main())
