"""ROS 2 Jazzy adapter. No simulator or world-state interfaces are used.

Only this node may submit navigation goals during the mission. Start Nav2 and
SLAM separately; do not also run explore_lite or an RViz goal sender.
"""
from dataclasses import asdict
import json
from math import atan2, cos, hypot, isfinite, sin
from pathlib import Path
import signal
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from visualization_msgs.msg import Marker, MarkerArray

from .core import Coverage, FailureBook, Goal, Grid, Planner, TargetStore, angle_delta
from .inspection import EvidenceError, InspectionLedger, InspectionPolicy, validate_record


def yaw_of(q):
    return atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))


def rotate(q, p):
    # Quaternion-vector rotation; TF quaternions are expected to be normalized.
    x, y, z = p
    tx, ty, tz = 2*(q.y*z-q.z*y), 2*(q.z*x-q.x*z), 2*(q.x*y-q.y*x)
    return (x+q.w*tx+q.y*tz-q.z*ty,
            y+q.w*ty+q.z*tx-q.x*tz,
            z+q.w*tz+q.x*ty-q.y*tx)


class MissionNode(Node):
    def __init__(self, **kwargs):
        super().__init__('limo_maze_mission', **kwargs)
        defaults = {
            'autonomous': False, 'map_topic': '/map', 'map_frame': 'map',
            'base_frame': 'base_link', 'odom_frame': 'odom',
            'navigation_action': 'navigate_to_pose', 'observation_topic': '/mission/target_observations',
            # Covers the padded Nav2 rectangle: hypot(.18+.02, .12+.02) < .25.
            'output_dir': 'mission_results', 'robot_radius_m': .25,
            'initial_clearance_radius_m': 0.0, 'initial_clearance_margin_m': .05,
            'initial_clearance_source': '',
            'frontier_standoff_m': .55, 'frontier_min_translation_m': .20,
            'goal_arrival_margin_m': .05,
            'goal_timeout_s': 90.0,
            'mission_timeout_s': 900.0, 'return_timeout_s': 180.0,
            'cancel_timeout_s': 10.0, 'tf_max_age_s': 1.0, 'map_max_age_s': 5.0,
            'observation_max_age_s': 1.5, 'detector_timeout_s': 5.0,
            'observation_dwell_s': 3.0, 'startup_wait_s': 60.0,
            'home_position_tolerance_m': .15, 'home_yaw_tolerance_rad': .26,
            'home_stable_s': 2.0, 'localization_jump_m': .75,
            'localization_jump_rad': .7, 'max_goal_attempts': 2,
            'target_min_hits': 3, 'target_merge_radius_m': .30,
            'target_min_confidence': .70, 'target_confirmation_window_s': 3.0,
            'target_definition_verified': False, 'camera_heading_offset_rad': 0.0,
            'coverage_max_range_m': 3.0,
            'require_processed_inspection': False,
            'frame_processing_topic': '/mission/frame_processing',
            'frame_processing_source': 'red_rgbd_sim_baseline',
            'inspection_timeout_s': 12.0, 'inspection_settle_s': .5,
            'inspection_yaw_tolerance_rad': .26,
            'inspection_required_frames': 3, 'inspection_min_span_s': .2,
            'inspection_odom_topic': '/odom', 'inspection_odom_max_age_s': .3,
            'inspection_linear_stop_m_s': .005, 'inspection_angular_stop_rad_s': .02,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        for key, value in self.cfg.items():
            if key == 'initial_clearance_radius_m':
                if not isfinite(float(value)) or value < 0:
                    raise ValueError(key + ' must be finite and nonnegative (zero disables the prior)')
                continue
            if key.endswith(('_s', '_m')) and (not isfinite(float(value)) or value <= 0):
                raise ValueError(f'{key} must be finite and positive')
        if self.cfg['initial_clearance_radius_m'] > 0 and (
                not self.cfg['initial_clearance_source'].strip() or
                self.cfg['initial_clearance_radius_m'] <= self.cfg['initial_clearance_margin_m']):
            raise ValueError('An initial clearance prior requires an independent source and radius greater than its margin')
        if self.cfg['require_processed_inspection'] and self.cfg['inspection_timeout_s'] > 300:
            raise ValueError('inspection_timeout_s must be at most 300 seconds')
        self.prior_filled_cells = 0
        self.grid = None
        self.map_received = None
        self.map_signature = None
        self.map_epoch = None
        self.map_epoch_ready = False
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = ActionClient(self, NavigateToPose, self.cfg['navigation_action'])
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, self.cfg['map_topic'], self.on_map, latched)
        self.create_subscription(String, self.cfg['observation_topic'], self.on_observations, 10)
        self.create_subscription(String, '/mission/map_epoch', self.on_epoch, latched)
        self.status_pub = self.create_publisher(String, '/mission/status', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/mission/targets', latched)
        self.planner = Planner(self.cfg['robot_radius_m'], self.cfg['frontier_standoff_m'],
                               self.cfg['frontier_min_translation_m'], self.cfg['goal_arrival_margin_m'])
        self.failures = FailureBook(max_attempts=self.cfg['max_goal_attempts'])
        self.coverage = Coverage()
        self.targets = TargetStore(self.cfg['target_min_hits'], self.cfg['target_merge_radius_m'],
                                   self.cfg['target_confirmation_window_s'], self.cfg['target_min_confidence'])
        self.sources = set()
        self.phase, self.reason = 'waiting', ''
        self.home = None
        self.started = None
        self.created = time.monotonic()
        self.last_clock = None
        self.last_map_odom = None
        self.detector_received = None
        self.selection = None
        self.goal = None
        self.goal_handle = None
        self.send_pending = False
        self.token = 0
        self.goal_started = None
        self.goal_result = None
        self.canceling = False
        self.cancel_response = False
        self.cancel_terminal = False
        self.cancel_started = None
        self.cancel_next = None
        self.dwell_until = None
        self.return_started = None
        self.return_attempts = 0
        self.home_stable_since = None
        self.last_home_pose = None
        self.result_written = False
        self.coordinates_valid = True
        self.map_complete = False
        self.visual_complete = False
        self.mission_timed_out = False
        self.events = []
        self.inspection = None
        self.inspection_clock_jump = None
        self.inspection_clock_invalid = False
        self.inspection_deadline = None
        self.inspection_terminal_received = None
        self.inspection_pose_since = None
        self.inspection_goal_token = None
        self.inspection_sampling_after_ns = None
        self.inspection_conditions = None
        self.last_inspection_failure = None
        self.inspection_odom = dict(since=None, count=0, received=None, stamp_ns=None,
                                    first_stamp_ns=None, stationary=False,
                                    sample_fresh=False, sample_stationary=False)
        if self.cfg['require_processed_inspection']:
            self.inspection = InspectionLedger(InspectionPolicy(
                required_frames=self.cfg['inspection_required_frames'],
                min_span_ns=round(self.cfg['inspection_min_span_s']*1e9),
                max_sample_age_ns=round(self.cfg['observation_max_age_s']*1e9),
                max_processing_age_ns=round(self.cfg['observation_max_age_s']*1e9),
                xy_tolerance=self.cfg['goal_arrival_margin_m'],
                yaw_tolerance=self.cfg['inspection_yaw_tolerance_rad'],
                allowed_source=self.cfg['frame_processing_source']))
            self.create_subscription(String, self.cfg['frame_processing_topic'], self.on_frame_processing, 10)
            self.create_subscription(Odometry, self.cfg['inspection_odom_topic'],
                                     self.on_inspection_odom, qos_profile_sensor_data)
            self.inspection_clock_jump = self.get_clock().create_jump_callback(
                JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True),
                pre_callback=self.on_inspection_clock_jump)
        # ROS time still timestamps measurements; wall deadlines must run while /clock is paused.
        timer_options = {'clock': Clock(clock_type=ClockType.STEADY_TIME)} if self.inspection else {}
        self._mission_timer = self.create_timer(.1 if self.inspection else .5, self.tick, **timer_options)
        self.get_logger().info('Mission loaded; autonomous=' + str(self.cfg['autonomous']))

    def seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def event(self, name, **fields):
        self.events.append({'stamp': self.seconds(), 'event': name, **fields})
        self.get_logger().info(name + (' ' + json.dumps(fields) if fields else ''))

    def on_map(self, msg):
        if msg.header.frame_id != self.cfg['map_frame']:
            self.block('map_frame_mismatch')
            return
        signature = (msg.header.frame_id, msg.info.resolution, yaw_of(msg.info.origin.orientation))
        # Width/height and x/y origin may legitimately expand during mapping.
        if self.map_signature is not None and signature != self.map_signature:
            self.block('map_geometry_changed_requires_reanchor')
            return
        try:
            self.grid = Grid(msg.info.width, msg.info.height, msg.info.resolution,
                             msg.info.origin.position.x, msg.info.origin.position.y,
                             signature[2], tuple(msg.data))
        except ValueError as exc:
            self.block('invalid_map: ' + str(exc))
            return
        self.map_signature = signature
        self.map_received = time.monotonic()

    def on_epoch(self, msg):
        epoch = msg.data.strip()
        if getattr(self, 'inspection', None) is not None:
            if self.inspection_clock_invalid and self.home is None:
                self.inspection.invalidate('clock_reset_requires_new_epoch')
                self.inspection_clock_invalid = False
            self.inspection.set_epoch(epoch, self.get_clock().now().nanoseconds)
        if not epoch:
            # Retain the last legitimate ID for audit, but a blank current
            # session announcement cannot authorize startup or continued use.
            self.map_epoch_ready = False
            if self.home is not None:
                self.block('map_epoch_missing_requires_reanchor')
            return
        if self.home is not None and epoch != self.map_epoch:
            self.map_epoch_ready = False
            self.block('map_reset_requires_new_mission')
            return
        self.map_epoch, self.map_epoch_ready = epoch, True

    def get_pose(self):
        transform = self.tf_buffer.lookup_transform(self.cfg['map_frame'], self.cfg['base_frame'], Time())
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec/1e9
        age = self.seconds() - stamp
        if age < -.1 or age > self.cfg['tf_max_age_s']:
            raise ValueError('stale_base_tf')
        p, q = transform.transform.translation, transform.transform.rotation
        if (not all(isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
                abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > 1e-3):
            raise ValueError('invalid_base_tf')
        map_odom = self.tf_buffer.lookup_transform(self.cfg['map_frame'], self.cfg['odom_frame'], Time())
        p2, q2 = map_odom.transform.translation, map_odom.transform.rotation
        if (not all(isfinite(v) for v in (p2.x, p2.y, p2.z, q2.x, q2.y, q2.z, q2.w)) or
                abs(q2.x*q2.x+q2.y*q2.y+q2.z*q2.z+q2.w*q2.w-1.) > 1e-3):
            raise ValueError('invalid_localization_tf_requires_reanchor')
        correction = p2.x, p2.y, yaw_of(q2)
        if self.last_map_odom is not None:
            old = self.last_map_odom
            if (hypot(correction[0]-old[0], correction[1]-old[1]) > self.cfg['localization_jump_m'] or
                    abs(angle_delta(correction[2], old[2])) > self.cfg['localization_jump_rad']):
                raise ValueError('localization_jump_requires_reanchor')
        self.last_map_odom = correction
        return p.x, p.y, yaw_of(q)

    def planning_grid(self):
        """Fixed, explicit starting-area prior; never modify the SLAM map.

        This does not grow with the robot, clear measured obstacles, or count
        as sensor coverage. The external setup guarantee is bound to home and
        the current map epoch; normal reset/jump guards invalidate the mission.
        """
        self.prior_filled_cells = 0
        if self.home is None or self.cfg['initial_clearance_radius_m'] == 0:
            return self.grid
        grid, self.prior_filled_cells = self.grid.with_free_prior(
            self.home[:2], self.cfg['initial_clearance_radius_m']-self.cfg['initial_clearance_margin_m'])
        return grid

    def on_observations(self, msg):
        if self.grid is None or self.phase in ('blocked', 'done') or self.cancel_next == 'blocked':
            return
        try:
            data = json.loads(msg.data)
            if data.get('schema_version') != 1:
                raise ValueError('schema_version must be 1')
            stamp = data['stamp']
            if isinstance(stamp, dict):
                stamp = float(stamp['sec']) + float(stamp['nanosec'])/1e9
            stamp = float(stamp)
            if not isfinite(stamp) or not -.1 <= self.seconds()-stamp <= self.cfg['observation_max_age_s']:
                raise ValueError('observation timestamp stale or in future')
            frame, source = str(data['frame_id']), str(data['source'])
            if not frame or not source:
                raise ValueError('source/frame_id required')
            transform = self.tf_buffer.lookup_transform(self.cfg['map_frame'], frame,
                                                       Time(nanoseconds=int(stamp*1e9)))
            t, q = transform.transform.translation, transform.transform.rotation
            detections = data.get('detections', [])
            if not isinstance(detections, list) or len(detections) > 100:
                raise ValueError('detections must be a bounded array')
            for item in detections:
                point = tuple(float(v) for v in item['position'])
                if len(point) != 3:
                    raise ValueError('position must contain x,y,z')
                rotated = rotate(q, point)
                xyz = rotated[0]+t.x, rotated[1]+t.y, rotated[2]+t.z
                self.targets.add(str(item['class_id']), xyz, stamp, float(item['confidence']),
                                 str(item.get('instance_id', '')))
            coverage = data.get('coverage')
            if coverage and coverage.get('valid') is True:
                # Camera optical +Z is its viewing axis; transform to the map XY plane.
                axis = rotate(q, (0.0, 0.0, 1.0))
                if hypot(axis[0], axis[1]) < .5:
                    raise ValueError('camera axis unsuitable for 2D coverage approximation')
                far = float(coverage['far_m'])
                if far > self.cfg['coverage_max_range_m']:
                    raise ValueError('coverage exceeds configured validated range')
                self.coverage.observe(self.grid, (t.x, t.y), atan2(axis[1], axis[0]),
                                      float(coverage['near_m']), far, float(coverage['horizontal_fov_rad']))
            self.sources.add(source)
            self.detector_received = time.monotonic()
            self.publish_targets()
        except (KeyError, ValueError, TypeError, TransformException) as exc:
            self.get_logger().warning('Rejected target observation: ' + str(exc))

    def on_inspection_clock_jump(self):
        # Jump callbacks run under the clock lock. Defer ROS calls to the steady timer.
        self.inspection_clock_invalid = True

    def reset_inspection_stop(self):
        self.inspection_odom.update(since=None, count=0, first_stamp_ns=None, stationary=False)
        self.inspection_pose_since = None

    def on_inspection_odom(self, msg):
        if self.phase not in ('settling', 'inspecting', 'home_verification'):
            return
        now, now_ns = time.monotonic(), self.get_clock().now().nanoseconds
        state = self.inspection_odom
        stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        twist = msg.twist.twist
        values = (twist.linear.x, twist.linear.y, twist.linear.z,
                  twist.angular.x, twist.angular.y, twist.angular.z)
        maximum_age = self.cfg['inspection_odom_max_age_s']
        fresh = (msg.header.stamp.sec >= 0 and 0 <= msg.header.stamp.nanosec < 1_000_000_000 and
                 0 <= now_ns-stamp <= maximum_age*1e9 and
                 (state['stamp_ns'] is None or stamp > state['stamp_ns']) and
                 stamp > self.inspection_terminal_received[1])
        continuous = state['received'] is None or now-state['received'] <= maximum_age
        stationary = (msg.header.frame_id == self.cfg['odom_frame'] and
                      msg.child_frame_id in (self.cfg['base_frame'], 'base_footprint') and
                      all(isfinite(v) for v in values) and
                      hypot(values[0], values[1]) <= self.cfg['inspection_linear_stop_m_s'] and
                      abs(values[2]) <= self.cfg['inspection_linear_stop_m_s'] and
                      max(abs(v) for v in values[3:]) <= self.cfg['inspection_angular_stop_rad_s'])
        if not fresh or not stationary or not continuous:
            self.restart_inspection_window('odometry_not_continuously_stationary')
        else:
            state['since'] = now if state['since'] is None else state['since']
            state['first_stamp_ns'] = stamp if state['first_stamp_ns'] is None else state['first_stamp_ns']
            state['count'] += 1
            state['stationary'] = True
        state['received'], state['stamp_ns'] = now, stamp
        # Preserve the latest sample facts independently of the settling window:
        # a pose-window reset is not evidence that a stationary sample moved.
        state['sample_fresh'], state['sample_stationary'] = fresh, stationary

    def inspection_stopped(self, now):
        state = self.inspection_odom
        return (state['stationary'] and state['count'] >= 3 and state['since'] is not None and
                now-state['since'] >= self.cfg['inspection_settle_s'] and
                now-state['received'] <= self.cfg['inspection_odom_max_age_s'] and
                0 <= self.get_clock().now().nanoseconds-state['stamp_ns'] <=
                self.cfg['inspection_odom_max_age_s']*1e9 and
                state['stamp_ns']-state['first_stamp_ns'] >= self.cfg['inspection_min_span_s']*1e9)

    def restart_inspection_window(self, reason):
        self.reset_inspection_stop()
        if self.phase == 'inspecting':
            self.inspection.reset_window(self.get_clock().now().nanoseconds, reason)
            self.inspection_sampling_after_ns = None
            self.phase = 'settling'
            self.event('inspection_window_reset', reason=reason)

    def inspection_pose_matches(self, pose):
        return (hypot(pose[0]-self.goal.x, pose[1]-self.goal.y) <= self.cfg['goal_arrival_margin_m'] and
                abs(angle_delta(pose[2], self.goal.yaw)) <= self.cfg['inspection_yaw_tolerance_rad'])

    def begin_inspection(self, now):
        terminal_wall, terminal_ns = self.inspection_terminal_received
        previous = self.inspection
        # Never turn a new request into authority to recover an invalid epoch.
        if (not previous.ready or self.inspection_clock_invalid or
                previous.epoch != self.map_epoch or terminal_ns < previous.epoch_barrier_ns):
            self.block('inspection_epoch_invalid_requires_reanchor')
            return
        current = InspectionLedger(previous.policy)
        current.set_epoch(previous.epoch, previous.epoch_barrier_ns)
        # This initial lower bound belongs to the terminal result. Acquisition
        # is still disabled until settling establishes the sampling barrier.
        current.begin(after_stamp_ns=terminal_ns,
                      expected_pose=(self.goal.x, self.goal.y, self.goal.yaw),
                      wall_now=terminal_wall, timeout_seconds=self.cfg['inspection_timeout_s'],
                      request_id='goal-'+str(self.token))
        self.inspection = current
        self.inspection_goal_token = self.token
        self.inspection_sampling_after_ns = None
        self.inspection_conditions = None
        self.phase = 'settling'
        self.inspection_deadline = current.deadline_wall
        self.inspection_odom.update(received=None, stamp_ns=None,
                                    sample_fresh=False, sample_stationary=False)
        self.reset_inspection_stop()
        self.event('inspection_settling', token=self.token, timeout_s=self.cfg['inspection_timeout_s'],
                   evidence=self.inspection_snapshot())

    def inspection_snapshot(self):
        if getattr(self, 'inspection', None) is None:
            return None
        result = self.inspection.snapshot()
        result.update(goal_token=getattr(self, 'inspection_goal_token', None),
                      sampling_after_stamp_ns=getattr(self, 'inspection_sampling_after_ns', None),
                      conditions=getattr(self, 'inspection_conditions', None))
        return result

    def inspection_wait_conditions(self, now, pose):
        """Bounded diagnostic facts; these do not alter any acceptance check."""
        state = self.inspection_odom
        now_ns = self.get_clock().now().nanoseconds
        wall_age = None if state['received'] is None else now-state['received']
        ros_age = None if state['stamp_ns'] is None else (now_ns-state['stamp_ns'])/1e9
        fresh = (state.get('sample_fresh', False) and wall_age is not None and ros_age is not None and
                 0 <= wall_age <= self.cfg['inspection_odom_max_age_s'] and
                 0 <= ros_age <= self.cfg['inspection_odom_max_age_s'])
        distance = hypot(pose[0]-self.goal.x, pose[1]-self.goal.y)
        yaw_error = abs(angle_delta(pose[2], self.goal.yaw))
        pose_matches = self.inspection_pose_matches(pose)
        stopped = self.inspection_stopped(now)
        pose_stable = 0. if self.inspection_pose_since is None else now-self.inspection_pose_since
        if not pose_matches:
            reason = 'current_pose_outside_goal_tolerance'
        elif not stopped:
            reason = 'stationary_odometry_unavailable'
        elif pose_stable < self.cfg['inspection_settle_s']:
            reason = 'pose_settle_time_not_elapsed'
        elif self.inspection.status != 'satisfied':
            reason = 'processed_frames_pending'
        else:
            reason = None
        return {'stamp_ns': now_ns, 'phase': self.phase, 'blocking_condition': reason,
                'current_pose': list(pose), 'distance_m': distance, 'yaw_error_rad': yaw_error,
                'position_tolerance_m': self.cfg['goal_arrival_margin_m'],
                'yaw_tolerance_rad': self.cfg['inspection_yaw_tolerance_rad'],
                'pose_within_tolerance': pose_matches, 'pose_stable_wall_s': pose_stable,
                'odom_stamp_ns': state['stamp_ns'], 'odom_wall_age_s': wall_age,
                'odom_ros_age_s': ros_age, 'odom_fresh': bool(fresh),
                'odom_sample_stationary': state.get('sample_stationary', False),
                'odom_stationary_count': state['count'], 'stop_confirmed': bool(stopped)}

    def record_inspection_failure(self, reason):
        self.last_inspection_failure = {'reason': reason, 'stamp': self.seconds(),
                                        'goal': asdict(self.goal), 'evidence': self.inspection_snapshot()}
        return self.last_inspection_failure

    def update_inspection(self, now, pose):
        # Refresh before checking the deadline so a timeout describes this
        # goal's actual pose/odometry guard, even if it never reached inspecting.
        self.inspection_conditions = self.inspection_wait_conditions(now, pose)
        if now >= self.inspection_deadline:
            failure = self.record_inspection_failure('inspection_wall_timeout')
            self.event('inspection_failed', **failure)
            self.block('inspection_wall_timeout_incomplete')
            return False
        if not self.inspection_pose_matches(pose):
            self.restart_inspection_window('current_pose_outside_goal_tolerance')
            return False
        if self.inspection_pose_since is None:
            self.inspection_pose_since = now
        if not self.inspection_stopped(now):
            if self.phase == 'inspecting':
                self.restart_inspection_window('stationary_odometry_unavailable')
            return False
        if now-self.inspection_pose_since < self.cfg['inspection_settle_s']:
            return False
        if self.phase == 'settling':
            self.inspection_sampling_after_ns = self.get_clock().now().nanoseconds
            self.inspection.reset_window(self.inspection_sampling_after_ns, 'stationarity_established')
            self.phase = 'inspecting'
            self.event('inspection_started', token=self.token)
        self.inspection.tick(self.get_clock().now().nanoseconds, now)
        if self.inspection.snapshot()['status'] == 'satisfied':
            self.event('inspection_complete', evidence=self.inspection_snapshot())
            self.phase, self.dwell_until = 'observing', now
            self.inspection_deadline = None
            return True
        return False

    def inspection_sample_transform(self, frame, stamp_ns):
        # Exact acquisition-time lookup; missing TF is rejected, never replaced by latest TF.
        transform = self.tf_buffer.lookup_transform(self.cfg['map_frame'], frame,
                                                   Time(nanoseconds=stamp_ns))
        p, q = transform.transform.translation, transform.transform.rotation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if (not all(isfinite(v) for v in values) or
                abs(sum(v*v for v in values[3:])-1.) > 1e-3):
            raise ValueError('invalid_sample_transform')
        return values

    def on_frame_processing(self, msg):
        if self.inspection is None or self.phase != 'inspecting' or self.canceling:
            return
        now, now_ns = time.monotonic(), self.get_clock().now().nanoseconds
        if now >= self.inspection_deadline or self.inspection_clock_invalid:
            return  # The steady timer owns bounded failure transitions.
        if not self.inspection_stopped(now):
            self.restart_inspection_window('frame_received_without_stationary_odometry')
            return
        try:
            if len(msg.data) > 65536:
                raise ValueError('evidence_payload_too_large')
            record = json.loads(msg.data)
            validate_record(record, now_ns, self.inspection.policy)
            stamp_ns = record['rgb']['stamp_ns']
            # Require an odometry sample at or after both image samples.
            if self.inspection_odom['stamp_ns'] < max(stamp_ns, record['depth']['stamp_ns']):
                raise ValueError('odometry_does_not_cover_sample_time')
            camera = self.inspection_sample_transform(record['rgb']['frame_id'], stamp_ns)
            base = self.inspection_sample_transform(self.cfg['base_frame'], stamp_ns)
            qx, qy, qz, qw = base[3:]
            pose = (base[0], base[1], atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz)))
            decision = self.inspection.observe(record, now_ns=now_ns, wall_now=now, base_pose=pose,
                                               camera_pose=camera, tf_stamp_ns=stamp_ns)
            if decision['reason'] == 'sample_pose_outside_tolerance':
                self.restart_inspection_window('sample_pose_outside_tolerance')
        except TransformException:
            self.inspection.reject('sample_tf_unavailable')
        except EvidenceError as exc:
            self.inspection.reject(str(exc))
        except (ValueError, TypeError, KeyError, RecursionError) as exc:
            reason = str(exc) if str(exc) in ('evidence_payload_too_large', 'invalid_sample_transform',
                                              'odometry_does_not_cover_sample_time') else 'invalid_json'
            self.inspection.reject(reason)

    def destroy_node(self):
        try:
            handle = getattr(self, 'inspection_clock_jump', None)
            if handle is not None:
                handle.unregister()
                self.inspection_clock_jump = None
            # ActionClient is a waitable; Node.destroy_node does not destroy it.
            # Its retained handle otherwise keeps the ROS node alive.
            client = getattr(self, 'nav', None)
            if client is not None:
                client.destroy()
                self.nav = None
            listener = getattr(self, 'tf_listener', None)
            if listener is not None:
                listener.unregister()
                self.tf_listener = None
        finally:
            result = super().destroy_node()
        return result

    def publish_targets(self):
        array = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        for target in self.targets.confirmed():
            for text in (False, True):
                marker = Marker()
                marker.header.frame_id = self.cfg['map_frame']
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = 'target_labels' if text else 'confirmed_targets'
                marker.id, marker.action = target.uid, Marker.ADD
                marker.type = Marker.TEXT_VIEW_FACING if text else Marker.SPHERE
                marker.pose.position.x, marker.pose.position.y = target.x, target.y
                marker.pose.position.z = target.z + (.25 if text else 0)
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = marker.scale.z = .16
                marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, .5, .05, 1.0
                marker.text = f'{target.class_id} #{target.uid}'
                array.markers.append(marker)
        self.marker_pub.publish(array)

    def send_goal(self, goal):
        if self.goal_handle is not None or self.send_pending or self.canceling:
            raise RuntimeError('Attempt to overlap owned navigation goals')
        if not self.cfg['autonomous']:
            return
        self.token += 1
        token = self.token
        self.goal, self.goal_started, self.goal_result = goal, time.monotonic(), None
        request = NavigateToPose.Goal()
        request.pose = PoseStamped()
        request.pose.header.frame_id = self.cfg['map_frame']
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x, request.pose.pose.position.y = goal.x, goal.y
        request.pose.pose.orientation.z, request.pose.pose.orientation.w = sin(goal.yaw/2), cos(goal.yaw/2)
        self.send_pending = True
        future = self.nav.send_goal_async(request)
        future.add_done_callback(lambda f: self.on_goal_response(f, token))
        self.event('goal_sent', token=token, goal=asdict(goal))

    def on_goal_response(self, future, token):
        try:
            handle = future.result()
        except Exception as exc:
            if token == self.token:
                # Transport failure is neither rejection nor proof that the
                # server did not accept. Retain unknown ownership/pending so
                # bounded shutdown cannot certify terminal or stop from this.
                self.block('goal_send_exception_stop_not_verified:' + str(exc))
            return
        if token != self.token:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        self.send_pending = False
        if not handle.accepted:
            self.goal_result = (False, 'goal_rejected')
            if self.canceling:
                self.cancel_response = self.cancel_terminal = True
            return
        self.goal_handle = handle
        if self.canceling or self.phase in ('blocked', 'done'):
            # A timed-out pending send can be accepted after we entered blocked.
            # Blocking the scheduler must not erase the intent to cancel it.
            self.issue_cancel(token)
        try:
            handle.get_result_async().add_done_callback(lambda f: self.on_goal_result(f, token))
        except Exception as exc:
            # A failed result channel must not prevent canceling an accepted
            # goal. Keep its handle: cancellation alone cannot prove terminal.
            self.event('result_request_error', error=str(exc))
            self.block('navigation_result_request_unavailable:' + str(exc))

    def on_goal_result(self, future, token):
        if token != self.token:
            return
        try:
            result = future.result()
            if result.status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED,
                                     GoalStatus.STATUS_ABORTED):
                # rclpy can return UNKNOWN for an unrecognized goal UUID. That
                # response (or an active status) does not establish termination.
                self.event('nonterminal_result_status', token=token, status=result.status)
                self.block('navigation_terminal_status_unverified:' + str(result.status))
                return
            success = result.status == GoalStatus.STATUS_SUCCEEDED
            detail = str(result.status) + ':' + str(getattr(result.result, 'error_msg', ''))
        except Exception as exc:
            # An exception is not proof of a terminal action; preserve the handle.
            self.block('navigation_result_unavailable:' + str(exc))
            return
        self.goal_handle = None
        self.goal_result = success, detail
        if success and getattr(self, 'inspection', None) is not None:
            self.inspection_terminal_received = time.monotonic(), self.get_clock().now().nanoseconds
        if self.canceling:
            self.cancel_terminal = True
        self.event('goal_terminal', token=token, success=success, detail=detail)

    def issue_cancel(self, token):
        if self.goal_handle is None:
            return
        try:
            self.goal_handle.cancel_goal_async().add_done_callback(lambda f: self.on_cancel_response(f, token))
        except Exception as exc:
            # Preserve the handle and cancel intent; this is not a terminal ACK.
            self.event('cancel_error', error=str(exc))

    def on_cancel_response(self, future, token):
        if token != self.token or not self.canceling:
            return
        try:
            response = future.result()
            self.cancel_response = bool(response.goals_canceling) or self.cancel_terminal
        except Exception as exc:
            self.event('cancel_error', error=str(exc))

    def cancel_to(self, next_phase, reason):
        if getattr(self, 'inspection', None) is not None and self.phase in ('settling', 'inspecting'):
            previous_failure = getattr(self, 'last_inspection_failure', None)
            if previous_failure is None or previous_failure['evidence']['request_id'] != self.inspection.request_id:
                self.record_inspection_failure(reason)
            self.event('inspection_interrupted', reason=reason, evidence=self.inspection_snapshot())
            self.inspection.invalidate(reason)
            self.inspection_deadline = None
        self.reason = reason
        if self.canceling:
            if next_phase == 'blocked':
                self.cancel_next = next_phase
            return
        self.canceling, self.cancel_next = True, next_phase
        self.cancel_started = time.monotonic()
        self.cancel_response = self.cancel_terminal = self.goal_handle is None and not self.send_pending
        self.phase = 'canceling'
        if self.goal_handle is not None:
            self.issue_cancel(self.token)
        self.event('cancel_requested', next_phase=next_phase, reason=reason)

    def block(self, reason):
        if self.phase in ('blocked', 'done'):
            return
        self.map_complete = self.visual_complete = False
        if any(word in reason for word in ('reset', 'reanchor', 'geometry_changed', 'frame_mismatch')):
            self.coordinates_valid = False
            clear = Marker()
            clear.action = Marker.DELETEALL
            self.marker_pub.publish(MarkerArray(markers=[clear]))
        self.cancel_to('blocked', reason)

    def start_return(self, reason):
        self.return_attempts = 0
        self.cancel_to('returning', reason)

    def tick(self):
        now = time.monotonic()
        ros_now = self.seconds()
        if getattr(self, 'inspection_clock_invalid', False):
            if self.home is None:
                self.inspection.invalidate('clock_reset_requires_new_epoch')
                self.inspection_clock_invalid = False
            else:
                self.block('ros_clock_reset_requires_new_mission')
        if self.last_clock is not None and ros_now < self.last_clock - .1 and self.home is not None:
            self.block('ros_clock_reset_requires_new_mission')
        self.last_clock = ros_now
        if self.canceling:
            if self.cancel_terminal and (self.cancel_response or self.goal_handle is None):
                # A terminal action is stronger evidence than a cancel response.
                self.canceling = False
                self.phase = self.cancel_next
                self.goal_result = None
                self.dwell_until = None
                if self.phase == 'returning':
                    self.return_started = now
            elif now-self.cancel_started > self.cfg['cancel_timeout_s']:
                self.phase, self.reason = 'blocked', 'cancel_timeout_stop_not_confirmed'
                self.canceling = False
            self.publish_status()
            if self.phase == 'blocked':
                self.save_result(False)
            return
        if self.phase in ('done', 'blocked'):
            self.publish_status()
            return
        if self.home is None and (not self.map_epoch_ready or
                getattr(self, 'inspection', None) is not None and not self.inspection.ready):
            if now-self.created >= self.cfg['startup_wait_s']:
                self.block('startup_no_map_epoch')
            self.publish_status()
            return
        if self.grid is None or self.map_received is None or now-self.map_received > self.cfg['map_max_age_s']:
            if self.home is not None:
                self.block('map_stale')
            elif now-self.created > self.cfg['startup_wait_s']:
                self.block('startup_no_map')
            self.publish_status()
            return
        try:
            pose = self.get_pose()
        except (TransformException, ValueError) as exc:
            if self.home is not None:
                self.block(str(exc))
            elif now-self.created > self.cfg['startup_wait_s']:
                self.block('startup_tf_unavailable:' + str(exc))
            self.publish_status()
            return
        if self.home is None:
            if not self.nav.server_is_ready():
                if now-self.created > self.cfg['startup_wait_s']:
                    self.block('startup_nav2_unavailable')
                self.publish_status()
                return
            self.home, self.started = pose, now
            self.phase = 'exploring' if self.cfg['autonomous'] else 'dry_run'
            self.event('home_recorded', pose=list(pose), map_epoch=self.map_epoch)
        if self.phase == 'dry_run':
            self.publish_status()
            return
        if self.phase in ('exploring', 'observing', 'settling', 'inspecting') and now-self.started >= self.cfg['mission_timeout_s']:
            self.mission_timed_out = True
            self.start_return('mission_timeout_incomplete')
            self.publish_status()
            return
        if self.phase in ('returning', 'home_verification') and now-self.return_started > self.cfg['return_timeout_s']:
            self.block('return_total_timeout')
            self.publish_status()
            return
        if self.goal_handle is not None or self.send_pending:
            timeout = self.cfg['return_timeout_s'] if self.phase == 'returning' else self.cfg['goal_timeout_s']
            if now-self.goal_started > timeout:
                if self.phase == 'returning':
                    self.block('return_navigation_timeout')
                else:
                    self.failures.fail(self.goal, now, 'navigation_timeout')
                    self.cancel_to('exploring', 'goal_timeout')
            self.publish_status()
            return
        if self.goal_result is not None:
            success, detail = self.goal_result
            self.goal_result = None
            if self.phase == 'returning':
                if success:
                    self.phase = 'home_verification'
                    self.home_stable_since = None
                    if getattr(self, 'inspection', None) is not None:
                        self.reset_inspection_stop()
                elif self.return_attempts < self.cfg['max_goal_attempts']:
                    self.event('return_retry', detail=detail)
                else:
                    self.block('return_navigation_failed')
            elif success:
                if self.cfg.get('require_processed_inspection', False):
                    self.begin_inspection(now)
                else:
                    self.phase, self.dwell_until = 'observing', now+self.cfg['observation_dwell_s']
            else:
                self.failures.fail(self.goal, now, detail)
                self.phase = 'exploring'
        if self.phase in ('settling', 'inspecting'):
            if not self.update_inspection(now, pose):
                self.publish_status()
                return
        if self.phase == 'observing':
            if now < self.dwell_until:
                self.publish_status()
                return
            anchor = self.grid.to_cell(*self.goal.anchor)
            observed = (not self.grid.frontier(anchor) if self.goal.kind == 'frontier' else
                        self.coverage.contains(self.grid.to_world(anchor), self.grid.resolution))
            if not observed:
                self.failures.fail(self.goal, now, 'arrived_without_new_observation')
            self.phase = 'exploring'
        if self.phase == 'exploring':
            ready = (self.detector_received is not None and now-self.detector_received <= self.cfg['detector_timeout_s']
                     and self.coverage.frames > 0)
            self.selection = self.planner.select(self.planning_grid(), pose, self.coverage, self.failures,
                                                 now, ready, observed_grid=self.grid)
            state = self.selection.state
            self.map_complete = (self.selection.frontier_count == 0 and self.selection.unobserved_count == 0 and
                                 state != 'unsafe_or_unlocalized' and
                                 not any(f.kind == 'frontier' for f in self.failures.entries))
            self.visual_complete = ready and self.selection.uncovered_count == 0
            if state == 'goal':
                g = self.selection.goal
                self.send_goal(Goal(g.x, g.y, g.yaw-self.cfg['camera_heading_offset_rad'], g.kind, g.anchor))
            elif state == 'retry_wait':
                pass
            elif state == 'complete':
                self.start_return('mapping_and_observation_coverage_complete')
            elif state == 'unsafe_or_unlocalized':
                self.block('robot_footprint_not_in_known_safe_component')
            else:
                self.start_return(state + '_incomplete')
        elif self.phase == 'returning':
            if now-self.return_started > self.cfg['return_timeout_s']:
                self.block('return_total_timeout')
            else:
                self.return_attempts += 1
                self.send_goal(Goal(*self.home, 'home', self.home[:2]))
        elif self.phase == 'home_verification':
            distance = hypot(pose[0]-self.home[0], pose[1]-self.home[1])
            aligned = abs(angle_delta(pose[2], self.home[2])) <= self.cfg['home_yaw_tolerance_rad']
            stable = self.last_home_pose is None or (hypot(pose[0]-self.last_home_pose[0], pose[1]-self.last_home_pose[1]) < .03 and
                                                     abs(angle_delta(pose[2], self.last_home_pose[2])) < .04)
            if getattr(self, 'inspection', None) is not None:
                stable = stable and self.inspection_stopped(now)
            self.last_home_pose = pose
            if distance <= self.cfg['home_position_tolerance_m'] and aligned and stable:
                self.home_stable_since = self.home_stable_since or now
                if now-self.home_stable_since >= self.cfg['home_stable_s']:
                    # SLAM may reveal new frontiers on the return trip; recheck.
                    ready = (self.detector_received is not None and
                             now-self.detector_received <= self.cfg['detector_timeout_s'] and self.coverage.frames > 0)
                    final = self.planner.select(self.planning_grid(), pose, self.coverage, self.failures,
                                                now, ready, observed_grid=self.grid)
                    self.map_complete = (final.frontier_count == 0 and final.unobserved_count == 0 and
                                         final.state != 'unsafe_or_unlocalized' and
                                         not any(f.kind == 'frontier' for f in self.failures.entries))
                    self.visual_complete = ready and final.uncovered_count == 0
                    if final.state == 'goal' and now-self.started < self.cfg['mission_timeout_s']:
                        self.event('completion_changed_during_return', selection=asdict(final))
                        self.phase, self.home_stable_since = 'exploring', None
                    else:
                        self.phase = 'done'
                        if not self.map_complete or not self.visual_complete:
                            self.reason += ';final_observation_checks_incomplete'
                        self.save_result(True, pose)
            else:
                self.home_stable_since = None
            if self.phase != 'done' and now-self.return_started > self.cfg['return_timeout_s']:
                self.block('home_pose_verification_failed')
        self.publish_status()

    def publish_status(self):
        value = {'phase': self.phase, 'reason': self.reason, 'autonomous': self.cfg['autonomous'],
                 'confirmed_target_count': len(self.targets.confirmed()),
                 'map_complete': self.map_complete, 'visual_coverage_complete': self.visual_complete,
                 'unresolved_failures': len(self.failures.entries),
                 'inspection': self.inspection_snapshot() if getattr(self, 'inspection', None) else None,
                 'last_inspection_failure': getattr(self, 'last_inspection_failure', None),
                 'selection': asdict(self.selection) if self.selection else None}
        self.status_pub.publish(String(data=json.dumps(value)))

    def save_result(self, returned, final_pose=None):
        if self.result_written:
            return
        mission_success = (returned and self.map_complete and self.visual_complete and
                           not self.mission_timed_out and not self.failures.entries and
                           self.cfg['target_definition_verified'])
        if self.mission_timed_out and 'mission_timeout_incomplete' not in self.reason:
            self.reason += (';' if self.reason else '') + 'mission_timeout_incomplete'
        result = {'schema_version': 1, 'frame_id': self.cfg['map_frame'], 'map_epoch': self.map_epoch,
                  'status': self.phase, 'reason': self.reason, 'mission_success': bool(mission_success),
                  'returned_pose_verified': returned, 'map_complete': self.map_complete,
                  'visual_coverage_complete': self.visual_complete,
                  'mission_timed_out': self.mission_timed_out,
                  'target_definition_verified': self.cfg['target_definition_verified'],
                  'coordinates_valid': self.coordinates_valid,
                  'inspection': self.inspection_snapshot() if getattr(self, 'inspection', None) else None,
                  'last_inspection_failure': getattr(self, 'last_inspection_failure', None),
                  'initial_clearance_prior': {
                      'enabled': self.cfg['initial_clearance_radius_m'] > 0,
                      'coordinates_valid': self.coordinates_valid and self.home is not None,
                      'frame_id': self.cfg['map_frame'], 'map_epoch': self.map_epoch,
                      'center': self.home[:2] if self.home else None,
                      'verified_radius_m': self.cfg['initial_clearance_radius_m'],
                      'margin_m': self.cfg['initial_clearance_margin_m'],
                      'source': self.cfg['initial_clearance_source'],
                      'currently_filled_unknown_cells': self.prior_filled_cells,
                      'sensor_observation': False, 'raw_slam_map_modified': False},
                  'target_count_prior': None, 'targets': [asdict(t) for t in self.targets.confirmed()],
                  'unconfirmed_target_candidates': [asdict(t) for t in self.targets.targets if not t.confirmed],
                  'observation_sources': sorted(self.sources), 'home': self.home, 'final_pose': final_pose,
                  'failures': [asdict(f) for f in self.failures.entries], 'events': self.events,
                  'limitations': ['Map-frame observations are not pose-graph landmarks.',
                                  '2D coverage needs a validated detector and target-height assumption.',
                                  'Navigation success is independently checked against TF at home.',
                                  'A red simulation surrogate does not establish real target recognition.']}
        try:
            output = Path(self.cfg['output_dir']).expanduser()
            output.mkdir(parents=True, exist_ok=True)
            destination = output / ('mission_' + time.strftime('%Y%m%d_%H%M%S') + '.json')
            destination.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding='utf-8')
            self.result_written = True
            self.get_logger().info('Saved mission evidence: ' + str(destination))
        except OSError as exc:
            self.get_logger().error('Failed to save mission evidence: ' + str(exc))


def shutdown_mission(node, reason):
    """Cancel owned work with the context alive; a cancel ACK alone is insufficient.

    Stop evidence is fresh wheel odometry, not proof of physical braking. The
    deadline limits callback polling; this is not a replacement for an E-stop.
    """
    started = time.monotonic()
    deadline = started + node.cfg['cancel_timeout_s']
    evidence = {'reason': reason, 'goal_terminal_verified': False, 'stop_verified': False,
                'stop_verification_source': 'wheel_odometry', 'errors': [],
                'linear_stop_threshold_m_s': .005, 'angular_stop_threshold_rad_s': .02,
                'limitations': ['Wheel odometry is not independent physical braking evidence.',
                    'After bounded cleanup destroys this client, an unresolved send may still be accepted by Nav2.',
                    'No controller lease or independent velocity watchdog is implemented here.']}
    stationary = {'since': None, 'count': 0, 'received': None, 'stamp': None}
    terminal_since = None
    subscription = None

    def odometry(msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        twist = msg.twist.twist
        values = (twist.linear.x, twist.linear.y, twist.linear.z,
                  twist.angular.x, twist.angular.y, twist.angular.z)
        fresh = (isfinite(stamp) and -.1 <= node.seconds()-stamp <= .3 and
                 (stationary['stamp'] is None or stamp > stationary['stamp']))
        continuous = stationary['received'] is None or now-stationary['received'] <= .3
        still = (all(isfinite(v) for v in values) and
                 max(abs(v) for v in values[:3]) <= .005 and
                 max(abs(v) for v in values[3:]) <= .02)
        if terminal_since is None or not fresh or not still or not continuous:
            stationary['since'], stationary['count'] = None, 0
        else:
            stationary['since'] = stationary['since'] if stationary['since'] is not None else now
            stationary['count'] += 1
        stationary['received'], stationary['stamp'] = now, stamp

    try:
        node._mission_timer.cancel()  # No new goals while callbacks drain.
    except Exception as exc:
        evidence['errors'].append('stop_mission_timer: ' + str(exc))
    try:
        subscription = node.create_subscription(Odometry, '/odom', odometry, qos_profile_sensor_data)
    except Exception as exc:
        evidence['errors'].append('subscribe_odometry: ' + str(exc))
    try:
        # This also preserves cancel intent for a send still awaiting acceptance.
        node.cancel_to('blocked', reason + '_stop_unverified')
    except Exception as exc:
        evidence['errors'].append('request_cancel: ' + str(exc))
    while time.monotonic() < deadline and rclpy.ok():
        terminal = (node.cancel_terminal and not node.send_pending and node.goal_handle is None)
        if terminal and terminal_since is None:
            terminal_since = time.monotonic()
            stationary['since'], stationary['count'] = None, 0
        evidence['goal_terminal_verified'] = terminal
        now = time.monotonic()
        if (terminal and stationary['count'] >= 3 and stationary['since'] is not None and
                now-stationary['since'] >= .5 and now-stationary['received'] <= .3):
            evidence['stop_verified'] = True
            break
        try:
            rclpy.spin_once(node, timeout_sec=min(.1, max(0., deadline-now)))
        except Exception as exc:
            evidence['errors'].append('drain_callbacks: ' + str(exc))
            # Keep the cancel intent and try remaining callbacks until deadline.
    if subscription is not None:
        try:
            node.destroy_subscription(subscription)
        except Exception as exc:
            evidence['errors'].append('destroy_subscription: ' + str(exc))
    evidence.update(elapsed_s=time.monotonic()-started, stationary_samples=stationary['count'],
                    cancel_ack_received=node.cancel_response,
                    send_pending=node.send_pending, goal_handle_present=node.goal_handle is not None,
                    late_acceptance_after_client_destroy_possible=node.send_pending)
    node.reason = reason + ('_stop_verified' if evidence['stop_verified'] else '_stop_unverified')
    node.phase = 'blocked'
    # Existing completed mission files remain immutable; always create a separate
    # shutdown audit so a later process exit cannot replace the mission outcome.
    try:
        node.save_result(False)
        output = Path(node.cfg['output_dir']).expanduser()
        output.mkdir(parents=True, exist_ok=True)
        path = output / ('shutdown_' + time.strftime('%Y%m%d_%H%M%S') + '.json')
        path.write_text(json.dumps(evidence, indent=2), encoding='utf-8')
        node.get_logger().info('Shutdown ' + node.reason + '; evidence: ' + str(path))
    except Exception as exc:
        evidence['errors'].append('save_shutdown_evidence: ' + str(exc))
    return evidence


def main(args=None):
    # rclpy's automatic SIGINT handler would invalidate the context before the
    # late-accept/cancel/result callbacks can run. Own signals until cleanup ends.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    requested, previous = {'reason': None}, {}
    def request_shutdown(signum, _frame):
        requested['reason'] = 'operator_' + signal.Signals(signum).name.lower()
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.signal(signum, request_shutdown)
    node = None
    try:
        node = MissionNode()
        while rclpy.ok() and requested['reason'] is None:
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        requested['reason'] = 'operator_sigint'
    except Exception:
        requested['reason'] = 'executor_exception'
        raise
    finally:
        try:
            if node is not None:
                shutdown_mission(node, requested['reason'] or 'context_shutdown')
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                try:
                    if rclpy.ok():
                        rclpy.shutdown()
                finally:
                    for signum, handler in previous.items():
                        signal.signal(signum, handler)
