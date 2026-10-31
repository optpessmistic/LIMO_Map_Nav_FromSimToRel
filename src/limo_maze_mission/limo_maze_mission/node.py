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
import uuid

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
from .observation import ObservationQueue, Policy as ObservationPolicy
from .retention import Consumer as RetentionConsumer, Policy as RetentionPolicy


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
            'require_retained_inspection': False,
            'frame_retention_request_topic': '/mission/frame_retention/request',
            'frame_retention_result_topic': '/mission/frame_retention/result',
            'retention_max_samples_per_window': 32, 'retention_max_total_samples': 32,
            'retention_max_windows': 64, 'retention_max_metadata_bytes': 2097152,
            'frame_processing_topic': '/mission/frame_processing',
            'frame_processing_source': 'red_rgbd_sim_baseline',
            'inspection_timeout_s': 12.0, 'inspection_settle_s': .5,
            'inspection_yaw_tolerance_rad': .26,
            'inspection_required_frames': 3, 'inspection_min_span_s': .2,
            'inspection_odom_topic': '/odom', 'inspection_odom_max_age_s': .3,
            'inspection_linear_stop_m_s': .005, 'inspection_angular_stop_rad_s': .02,
            'post_mapping_observation_mode': 'off',
            'observation_station_spacing_m': 1.0, 'observation_max_stations': 3,
            'observation_headings_per_station': 4, 'observation_max_dispatches': 12,
            'observation_stage_timeout_s': 180.0, 'observation_max_grid_cells': 250000,
            'home_frontier_resume_limit': 1, 'home_frontier_return_reserve_s': 15.0,
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
        if type(self.cfg['require_retained_inspection']) is not bool:
            raise ValueError('require_retained_inspection must be bool')
        if self.cfg['require_retained_inspection'] and not self.cfg['require_processed_inspection']:
            raise ValueError('retained inspection requires processed inspection')
        if self.cfg['post_mapping_observation_mode'] not in ('off', 'bounded_processed_frames'):
            raise ValueError('Unknown post_mapping_observation_mode')
        if self.bounded_observation_enabled() and not self.cfg['require_processed_inspection']:
            raise ValueError('bounded_processed_frames requires processed inspection; dwell fallback forbidden')
        if (type(self.cfg['home_frontier_resume_limit']) is not int or
                not 0 <= self.cfg['home_frontier_resume_limit'] <= 8):
            raise ValueError('home_frontier_resume_limit must be an integer from 0 to 8')
        self.observation_policy = ObservationPolicy(
            self.cfg['robot_radius_m'], self.cfg['goal_arrival_margin_m'],
            self.cfg['observation_station_spacing_m'], self.cfg['observation_max_stations'],
            self.cfg['observation_headings_per_station'], self.cfg['observation_max_dispatches'],
            max_attempts_per_view=1, max_grid_cells=self.cfg['observation_max_grid_cells'])
        self.observation_queue = None
        self.observation_closed = False
        self.observation_close_reason = None
        self.observation_deadline_wall = None
        self.observation_started_wall = None
        self.observation_active = None
        self.observation_attempt_records = []
        self.observation_cancel_outcome = None
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
        self.return_deadline_wall = None
        self.home_frontier_resumes = 0
        self.home_frontier_resume_active = False
        self.home_frontier_decisions = []
        self.home_stable_since = None
        self.last_home_pose = None
        self.result_written = False
        self.coordinates_valid = True
        self.map_complete = False
        self.visual_complete = False
        self.mission_timed_out = False
        self.events = []
        self.inspection = None
        self.frame_retention = None
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
        if self.cfg['require_retained_inspection']:
            retention_qos = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE)
            self.retention_pub = self.create_publisher(String, self.cfg['frame_retention_request_topic'], retention_qos)
            self.frame_retention = RetentionConsumer(
                str(uuid.uuid4()), self.cfg['inspection_required_frames'],
                lambda raw: self.retention_pub.publish(String(data=raw)),
                RetentionPolicy(enabled=True,
                    max_samples_per_window=self.cfg['retention_max_samples_per_window'],
                    max_total_samples=self.cfg['retention_max_total_samples'],
                    max_windows=self.cfg['retention_max_windows'],
                    max_metadata_bytes=self.cfg['retention_max_metadata_bytes']))
            self.create_subscription(String, self.cfg['frame_retention_result_topic'],
                                     self.on_frame_retention_result, retention_qos)
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
        retention = getattr(self, 'frame_retention', None)
        if retention is not None and (not epoch or self.map_epoch not in (None, epoch)):
            retention.invalidate('map_epoch_changed')
        if (self.bounded_observation_enabled() and self.home is not None and
                epoch != self.map_epoch):
            # Preserve this attempt's original epoch and partial frame evidence
            # before set_epoch invalidates/clears the live Ledger. Navigation
            # ownership is still retained until block/cancel verifies terminal.
            reason = 'map_reset_requires_new_mission' if epoch else 'map_epoch_missing_requires_reanchor'
            if self.phase in ('settling', 'inspecting'):
                self.record_inspection_failure(reason)
            self.close_observation('blocked:'+reason, invalid=True)
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
        if getattr(getattr(self, 'targets', None), 'frame_epoch', None) not in (None, epoch):
            self.map_epoch_ready = False
            self.block('target_map_epoch_changed_requires_new_mission')
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
        if (self.grid is None or not self.map_epoch_ready or not self.coordinates_valid or
                self.phase in ('blocked', 'done') or self.cancel_next == 'blocked'):
            return
        try:
            data = json.loads(msg.data)
            if data.get('schema_version') != 1:
                raise ValueError('schema_version must be 1')
            raw_stamp = data['stamp']
            if isinstance(raw_stamp, dict):
                sec, nanosec = raw_stamp['sec'], raw_stamp['nanosec']
                if type(sec) is not int or type(nanosec) is not int or sec < 0 or not 0 <= nanosec < 1_000_000_000:
                    raise ValueError('invalid integer acquisition stamp')
                stamp_ns = sec*1_000_000_000+nanosec
            else:
                if (type(raw_stamp) not in (int, float) or not isfinite(raw_stamp) or
                        not 0 <= raw_stamp <= (2**63-1)/1e9):
                    raise ValueError('invalid acquisition stamp')
                stamp_ns = round(raw_stamp*1e9)
            if not 0 <= stamp_ns <= 2**63-1:
                raise ValueError('acquisition stamp out of bounds')
            stamp = stamp_ns/1e9
            if not isfinite(stamp) or not -.1 <= self.seconds()-stamp <= self.cfg['observation_max_age_s']:
                raise ValueError('observation timestamp stale or in future')
            if data.get('map_epoch', self.map_epoch) != self.map_epoch:
                raise ValueError('observation map_epoch mismatch')
            frame, source = str(data['frame_id']), str(data['source'])
            if not frame or not source:
                raise ValueError('source/frame_id required')
            transform = self.tf_buffer.lookup_transform(self.cfg['map_frame'], frame,
                                                       Time(nanoseconds=stamp_ns))
            t, q = transform.transform.translation, transform.transform.rotation
            detections = data.get('detections', [])
            if not isinstance(detections, list) or len(detections) > 100:
                raise ValueError('detections must be a bounded array')
            transformed = []
            for item in detections:
                raw_point = item['position']
                if not isinstance(raw_point, (list, tuple)) or len(raw_point) != 3:
                    raise ValueError('position must contain x,y,z')
                if any(type(value) not in (int, float) or not isfinite(value) for value in raw_point):
                    raise ValueError('position must be finite XYZ numbers')
                point = tuple(float(v) for v in raw_point)
                rotated = rotate(q, point)
                xyz = rotated[0]+t.x, rotated[1]+t.y, rotated[2]+t.z
                transformed.append({'class_id': item['class_id'], 'position': xyz,
                                    'confidence': item['confidence'],
                                    'instance_id': item.get('instance_id', '')})
            association = self.targets.add_frame(transformed, stamp_ns, source=source,
                                                  sensor_frame=frame, epoch=self.map_epoch)
            if not association['accepted']:
                raise ValueError('target frame association: '+association['reason'])
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
            retention = getattr(self, 'frame_retention', None)
            if retention is not None:
                retention.retire('inspection_window_reset', time.monotonic())
            self.inspection.reset_window(self.get_clock().now().nanoseconds, reason)
            self.inspection_sampling_after_ns = None
            self.phase = 'settling'
            self.event('inspection_window_reset', reason=reason)

    def inspection_pose_matches(self, pose):
        return (hypot(pose[0]-self.goal.x, pose[1]-self.goal.y) <= self.cfg['goal_arrival_margin_m'] and
                abs(angle_delta(pose[2], self.goal.yaw)) <= self.cfg['inspection_yaw_tolerance_rad'])

    def begin_inspection(self, now):
        if self.goal.kind == 'observation_queue' and not self.observation_binding_valid():
            self.block('observation_binding_invalid')
            return
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
        retention = getattr(self, 'frame_retention', None)
        if retention is not None:
            result['frame_retention'] = retention.summary()
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
            continue_view = self.observation_timeout_can_continue(now, pose)
            if continue_view:
                self.inspection.tick(self.get_clock().now().nanoseconds, now)
            failure = self.record_inspection_failure('inspection_wall_timeout')
            self.event('inspection_failed', **failure)
            if continue_view and self.inspection.ready and self.inspection.status == 'timed_out':
                # Freeze the failed window before moving phase. A timed-out
                # Ledger stays ready for a NEW request, never for late samples.
                retention = getattr(self, 'frame_retention', None)
                if retention is not None:
                    retention.retire('processed_frames_timeout_while_stationary', now)
                self.finish_observation('inspection_failed', 'processed_frames_timeout_while_stationary')
                if self.observation_active is None and not self.canceling:
                    self.inspection_deadline = self.inspection_sampling_after_ns = None
                    self.inspection_goal_token = None
                    self.phase = 'observation_select'
                    return False  # A separate steady tick owns the next dispatch.
                self.block('observation_timeout_retirement_binding_invalid')
                return False
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
            retention = getattr(self, 'frame_retention', None)
            if retention is not None and not retention.begin_window(
                    self.inspection, self.inspection_goal_token, time.monotonic()):
                self.block('frame_retention_window_failed')
                return False
            self.phase = 'inspecting'
            self.event('inspection_started', token=self.token)
        self.inspection.tick(self.get_clock().now().nanoseconds, now)
        if self.inspection.snapshot()['status'] == 'satisfied':
            retention = getattr(self, 'frame_retention', None)
            if retention is not None and not retention.current_satisfied(
                    self.inspection, self.inspection_goal_token, time.monotonic()):
                if retention.fault:
                    self.block('frame_retention_failed')
                return False
            self.event('inspection_complete', evidence=self.inspection_snapshot())
            self.phase, self.dwell_until = 'observing', now
            self.inspection_deadline = None
            return True
        return False

    def observation_timeout_can_continue(self, now, pose):
        """Only image insufficiency at a currently verified stopped queue view.

        tick has just checked map freshness and obtained a valid current TF pose.
        Recheck the binding and odometry here; a stale 'inspecting' phase alone
        is not evidence of stationarity. No success condition is weakened.
        """
        if (not self.bounded_observation_enabled() or self.phase != 'inspecting' or
                self.canceling or self.send_pending or self.goal_handle is not None or
                not self.observation_binding_valid()):
            return False
        active, ledger, conditions = self.observation_active, self.inspection, self.inspection_conditions
        return bool(ledger and ledger.ready and ledger.status == 'waiting' and
                    self.inspection_goal_token == active['nav_token'] and
                    ledger.epoch == active['epoch'] and
                    ledger.request_id == active['inspection_request_id'] and
                    ledger.expected_pose == (self.goal.x, self.goal.y, self.goal.yaw) and
                    self.inspection_sampling_after_ns is not None and
                    ledger.after_stamp_ns == self.inspection_sampling_after_ns and
                    active['nav_terminal_status'] == GoalStatus.STATUS_SUCCEEDED and
                    active['nav_goal_uuid'] is not None and
                    self.inspection_pose_matches(pose) and self.inspection_stopped(now) and
                    conditions['pose_stable_wall_s'] >= self.cfg['inspection_settle_s'] and
                    conditions['odom_fresh'] and conditions['odom_sample_stationary'] and
                    conditions['blocking_condition'] == 'processed_frames_pending')

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
            retention = getattr(self, 'frame_retention', None)
            if retention is None:
                decision = self.inspection.observe(record, now_ns=now_ns, wall_now=now, base_pose=pose,
                                                   camera_pose=camera, tf_stamp_ns=stamp_ns)
            else:
                decision = retention.observe(self.inspection, record, now_ns=now_ns, wall_now=now,
                    base_pose=pose, camera_pose=camera, tf_stamp_ns=stamp_ns, goal_token=self.inspection_goal_token)
                if retention.fault:
                    self.block('frame_retention_failed')
                    return
                if decision['reason'] == 'producer_identity_changed':
                    self.restart_inspection_window('producer_identity_changed')
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

    def on_frame_retention_result(self, msg):
        retention = getattr(self, 'frame_retention', None)
        if retention is None:
            return
        try:
            if len(msg.data.encode('utf-8')) > 16384:
                raise ValueError('retention_result_too_large')
            value = json.loads(msg.data)
        except (ValueError, TypeError, RecursionError):
            value = None
        try:
            retention.on_result(value, time.monotonic())
        except ValueError:
            retention.invalidate('invalid_retention_result_time')
        # A terminal result is never rewritten or used to restart navigation.
        # Its later validity must be checked against the final producer checkpoint.
        if retention.fault and self.phase not in ('done', 'blocked'):
            self.block('frame_retention_failed')

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

    def dispatch_budget_allows(self, kind):
        # Planning may have crossed a wall deadline since tick's initial check.
        # Run before consuming a Nav token/home send or creating new ownership.
        if not self.bounded_observation_enabled():
            return True
        now = time.monotonic()
        if self.return_budget_expired(now):
            self.block('return_total_timeout')
        elif kind != 'home' and now-self.started >= self.cfg['mission_timeout_s']:
            self.mission_timed_out = True
            self.start_return('mission_timeout_incomplete')
        elif (kind != 'home' and self.home_frontier_resume_active and
                now >= self.return_deadline_wall-self.cfg['home_frontier_return_reserve_s']):
            self.start_return('home_frontier_return_reserve_reached')
        elif (kind != 'home' and self.observation_queue is not None and not self.observation_closed and
                now >= self.observation_deadline_wall):
            self.start_return('bounded_observation_wall_timeout')
        else:
            return True
        return False

    def send_goal(self, goal):
        if self.goal_handle is not None or self.send_pending or self.canceling:
            raise RuntimeError('Attempt to overlap owned navigation goals')
        if not self.cfg['autonomous']:
            return False
        if not self.dispatch_budget_allows(goal.kind):
            return False
        self.token += 1
        token = self.token
        self.goal, self.goal_started, self.goal_result = goal, time.monotonic(), None
        if goal.kind == 'home':
            self.return_attempts += 1
        if goal.kind == 'observation_queue':
            active = self.observation_active
            if active is None or active['nav_token'] is not None or active['goal'] != asdict(goal):
                raise RuntimeError('Observation dispatch must be bound before sending navigation')
            active['nav_token'] = token
            active['inspection_request_id'] = 'goal-'+str(token)
            active['dispatched_wall'] = self.goal_started
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
        return True

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
        if self.observation_active is not None and self.observation_active['nav_token'] == token:
            # ROS sequence elements may be numpy.uint8; keep the audit JSON
            # composed of built-in scalars while preserving every UUID byte.
            self.observation_active['nav_goal_uuid'] = [int(value) for value in handle.goal_id.uuid]
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
        if self.observation_active is not None and self.observation_active['nav_token'] == token:
            self.observation_active['nav_terminal_status'] = result.status
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
        retention = getattr(self, 'frame_retention', None)
        if retention is not None:
            if next_phase == 'returning' and not retention.fault:
                # Closing selection cannot re-authorize Ledger/epoch. Preserve
                # the existing mission-budget return behavior and its deadline.
                retention.retire('mission_request_canceled_for_return', time.monotonic())
            else:
                retention.invalidate('mission_request_invalidated')
        if getattr(self, 'inspection', None) is not None and self.phase in ('settling', 'inspecting'):
            previous_failure = getattr(self, 'last_inspection_failure', None)
            if previous_failure is None or previous_failure['evidence']['request_id'] != self.inspection.request_id:
                self.record_inspection_failure(reason)
            self.event('inspection_interrupted', reason=reason, evidence=self.inspection_snapshot())
            if (self.bounded_observation_enabled() and next_phase == 'returning' and
                    reason == 'bounded_observation_wall_timeout' and
                    self.goal.kind == 'observation_queue' and
                    self.goal_handle is None and not self.send_pending and not self.canceling and
                    self.coordinates_valid and self.map_epoch_ready and not self.inspection_clock_invalid and
                    self.inspection.ready and self.inspection.epoch == self.map_epoch):
                # A scheduled phase boundary retires the request, not a still
                # valid epoch. Fault-driven cancellation remains invalidating.
                self.inspection.retire_request(reason)
            else:
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
        retention = getattr(self, 'frame_retention', None)
        if retention is not None:
            retention.invalidate('mission_blocked')
        coordinate_fault = (reason == 'target_map_epoch_changed_requires_new_mission' or
                            any(word in reason for word in ('reset', 'reanchor', 'geometry_changed', 'frame_mismatch')))
        if coordinate_fault and self.coordinates_valid:
            # Old target coordinates remain only for audit. Clear their display
            # even after an unrelated fault has already stopped the scheduler.
            self.coordinates_valid = False
            clear = Marker()
            clear.action = Marker.DELETEALL
            self.marker_pub.publish(MarkerArray(markers=[clear]))
        if self.phase in ('blocked', 'done'):
            return
        if self.bounded_observation_enabled():
            self.close_observation('blocked:'+reason, invalid=True)
        self.map_complete = self.visual_complete = False
        self.cancel_to('blocked', reason)

    def start_return(self, reason):
        if self.bounded_observation_enabled():
            self.close_observation(reason)
            self.home_frontier_resume_active = False
        else:
            self.return_attempts = 0
        self.cancel_to('returning', reason)

    def return_attempt_limit(self):
        # Bounded mode counts all home sends across resumptions, not only errors.
        multiplier = self.cfg['home_frontier_resume_limit']+1 if self.bounded_observation_enabled() else 1
        return self.cfg['max_goal_attempts'] * multiplier

    def return_budget_expired(self, now):
        if self.bounded_observation_enabled():
            return self.return_deadline_wall is not None and now >= self.return_deadline_wall
        return self.return_started is not None and now-self.return_started > self.cfg['return_timeout_s']

    def resume_home_frontier(self, final, now):
        """Called only after successful home Nav AND current stable home pose.

        The planner's frontier goal is reachable through known safe cells. No
        simulator truth or coverage-only change can authorize another outing.
        A new map will be planned again on the next tick before dispatch.
        """
        if (not self.bounded_observation_enabled() or final.state != 'goal' or
                final.goal is None or final.goal.kind != 'frontier'):
            return False
        reason = None
        if (self.phase != 'home_verification' or self.canceling or self.send_pending or
                self.goal_handle is not None or self.observation_active is not None or
                not self.observation_closed or not self.coordinates_valid or not self.map_epoch_ready or
                self.inspection_clock_invalid or not self.inspection.ready or self.inspection.epoch != self.map_epoch):
            reason = 'home_frontier_resume_state_invalid'
        elif self.home_frontier_resumes >= self.cfg['home_frontier_resume_limit']:
            reason = 'home_frontier_resume_limit'
        elif self.mission_timed_out or now >= self.started+self.cfg['mission_timeout_s']:
            self.mission_timed_out = True
            reason = 'home_frontier_mission_budget_exhausted'
        elif (self.return_deadline_wall is None or
                now >= self.return_deadline_wall-self.cfg['home_frontier_return_reserve_s']):
            reason = 'home_frontier_return_reserve_reached'
        elif self.return_attempts >= self.return_attempt_limit():
            reason = 'home_frontier_return_attempts_exhausted'
        decision = {'wall': now, 'resumed': reason is None, 'reason': reason,
                    'selection': asdict(final), 'resume_count_before': self.home_frontier_resumes,
                    'return_deadline_wall': self.return_deadline_wall,
                    'return_attempts': self.return_attempts}
        self.home_frontier_decisions.append(decision)
        self.event('home_frontier_decision', **decision)
        if reason is not None:
            self.reason += ';'+reason
            return False
        self.home_frontier_resumes += 1
        self.home_frontier_resume_active = True
        self.phase, self.home_stable_since, self.last_home_pose = 'exploring', None, None
        self.map_complete = self.visual_complete = False
        return True

    def bounded_observation_enabled(self):
        return self.cfg.get('post_mapping_observation_mode', 'off') == 'bounded_processed_frames'

    def observation_binding_valid(self):
        active, queue = self.observation_active, self.observation_queue
        return bool(active and queue and not self.observation_closed and not queue.invalid_reason and
                    self.coordinates_valid and self.map_epoch_ready and not self.inspection_clock_invalid and
                    active['epoch'] == self.map_epoch == queue.epoch and
                    active['action_id'] == queue.active_id and active['dispatch_id'] == queue.active_dispatch_id and
                    active['nav_token'] == self.token and active['goal'] == asdict(self.goal))

    def finish_observation(self, outcome, reason):
        active = self.observation_active
        if active is None:
            return
        if self.goal_handle is not None or self.send_pending:
            raise RuntimeError('Cannot retire an observation with unknown navigation ownership')
        ledger = (self.inspection_snapshot() if active['nav_token'] is not None and
                  self.inspection_goal_token == active['nav_token'] else None)
        if ledger and (ledger['map_epoch'] != active['epoch'] or
                       ledger['request_id'] != active['inspection_request_id']):
            # A token match alone cannot attach another epoch/request's data.
            ledger = None
        if outcome == 'inspection_satisfied' and not (
                self.observation_binding_valid() and not self.canceling and ledger and
                active['nav_terminal_status'] == GoalStatus.STATUS_SUCCEEDED and
                active['nav_goal_uuid'] is not None and
                ledger['status'] == 'satisfied' and ledger['ready'] and
                ledger['map_epoch'] == active['epoch'] and
                ledger['request_id'] == active['inspection_request_id'] and
                ledger['expected_pose'] == [self.goal.x, self.goal.y, self.goal.yaw]):
            self.block('observation_completion_binding_invalid')
            return
        queue = self.observation_queue
        if (active['epoch'] != queue.epoch or active['action_id'] != queue.active_id or
                active['dispatch_id'] != queue.active_dispatch_id):
            # Failure cleanup must not invent a corrected identity either. Keep
            # the unresolved active record for the blocked result; never dispatch.
            active['retirement_error'] = 'queue_binding_mismatch'
            return
        self.observation_queue.finish(active['action_id'], active['epoch'], outcome, active['dispatch_id'])
        record = {**active, 'outcome': outcome, 'reason': reason,
                  'finished_wall': time.monotonic(), 'inspection': ledger}
        self.observation_attempt_records.append(record)
        self.observation_active = None
        self.observation_cancel_outcome = None
        self.event('observation_action_finished', record=record)

    def close_observation(self, reason, invalid=False):
        if not self.observation_closed:
            self.observation_closed, self.observation_close_reason = True, reason
        if self.observation_queue is not None and invalid:
            self.observation_queue.invalidate(reason)
        if self.observation_active is not None:
            self.observation_cancel_outcome = (
                'inspection_failed' if self.phase in ('settling', 'inspecting') else 'canceled', reason)
            if self.goal_handle is None and not self.send_pending:
                self.finish_observation(*self.observation_cancel_outcome)

    def observation_snapshot(self):
        if not self.bounded_observation_enabled():
            return None
        return {'mode': self.cfg['post_mapping_observation_mode'],
                'closed': self.observation_closed, 'close_reason': self.observation_close_reason,
                'started_wall': self.observation_started_wall, 'deadline_wall': self.observation_deadline_wall,
                'queue': self.observation_queue.snapshot() if self.observation_queue else None,
                'active': dict(self.observation_active) if self.observation_active else None,
                'attempts': list(self.observation_attempt_records),
                'continuation': {'home_frontier_resumes': self.home_frontier_resumes,
                    'resume_limit': self.cfg['home_frontier_resume_limit'],
                    'resume_active': self.home_frontier_resume_active,
                    'return_started_wall': self.return_started,
                    'return_deadline_wall': self.return_deadline_wall,
                    'return_reserve_s': self.cfg['home_frontier_return_reserve_s'],
                    'return_attempts': self.return_attempts, 'return_attempt_limit': self.return_attempt_limit(),
                    'decisions': list(self.home_frontier_decisions)},
                'visual_coverage_claimed': False, 'validation_scope': 'processed_frame_windows_only'}

    def begin_observation_stage(self, now, pose):
        if self.observation_closed:
            self.start_return('observation_stage_already_closed')
            return
        if self.observation_queue is None:
            if (self.goal_handle is not None or self.send_pending or self.canceling or
                    not self.map_epoch_ready or not self.inspection.ready or
                    self.inspection.epoch != self.map_epoch or self.inspection_clock_invalid):
                self.block('observation_entry_not_ready')
                return
            self.observation_started_wall = now
            self.observation_deadline_wall = min(now+self.cfg['observation_stage_timeout_s'],
                                                 self.started+self.cfg['mission_timeout_s'])
            try:
                # No initial-clearance prior and no simulator/target truth.
                self.observation_queue = ObservationQueue(self.grid, pose, self.map_epoch, self.observation_policy)
            except ValueError:
                self.block('observation_planning_invalid_or_over_budget')
                return
            self.event('observation_queue_frozen', queue=self.observation_queue.snapshot())
        self.phase = 'observation_select'

    def step_observation(self, now, pose):
        if self.observation_closed:
            self.start_return('observation_stage_already_closed')
            return
        try:
            decision = self.observation_queue.take_next(self.grid, pose, self.map_epoch)
        except ValueError:
            # A valid SLAM map can expand beyond the fixed work budget after
            # queue creation. Reject it without unwinding the executor, making
            # a new queue, or changing the original deadline/dispatch budget.
            self.block('observation_planning_invalid_or_over_budget')
            return
        if decision.state == 'dispatch':
            view = decision.view
            self.observation_active = {
                'epoch': view.epoch, 'action_id': view.action_id, 'dispatch_id': decision.dispatch_id,
                'station_id': view.station_id, 'heading_index': view.heading_index,
                'goal': asdict(view.goal), 'nav_token': None, 'nav_goal_uuid': None,
                'inspection_request_id': None, 'nav_terminal_status': None,
                'selected_wall': now, 'dispatched_wall': None}
            self.phase = 'observation_navigating'
            # Queue yaw is already BASE yaw. Do not apply camera_heading_offset.
            self.send_goal(view.goal)
        elif decision.state in ('empty', 'exhausted', 'budget_exhausted'):
            self.start_return('bounded_observation_'+decision.state)
        elif decision.state != 'active':
            self.block('observation_'+decision.state)

    def tick(self):
        now = time.monotonic()
        ros_now = self.seconds()
        retention = getattr(self, 'frame_retention', None)
        if retention is not None:
            if getattr(self, 'inspection_clock_invalid', False):
                retention.invalidate('ros_clock_reset')
            try:
                retention.tick(now)
            except ValueError:
                retention.invalidate('invalid_monotonic_wall')
            if retention.fault and self.phase not in ('done', 'blocked'):
                self.block('frame_retention_failed')
        if getattr(self, 'inspection_clock_invalid', False):
            if self.home is None:
                self.inspection.invalidate('clock_reset_requires_new_epoch')
                self.inspection_clock_invalid = False
            else:
                self.block('ros_clock_reset_requires_new_mission')
        if self.last_clock is not None and ros_now < self.last_clock - .1 and self.home is not None:
            self.block('ros_clock_reset_requires_new_mission')
        self.last_clock = ros_now
        if (self.bounded_observation_enabled() and self.phase not in ('done', 'blocked') and
                self.return_budget_expired(now)):
            self.block('return_total_timeout')
        if self.canceling:
            if self.cancel_terminal and (self.cancel_response or self.goal_handle is None):
                # A terminal action is stronger evidence than a cancel response.
                self.canceling = False
                self.phase = self.cancel_next
                self.goal_result = None
                self.dwell_until = None
                if self.observation_active is not None and self.observation_cancel_outcome is not None:
                    self.finish_observation(*self.observation_cancel_outcome)
                if self.phase == 'returning':
                    if not self.bounded_observation_enabled() or self.return_started is None:
                        self.return_started = now
                    if self.bounded_observation_enabled() and self.return_deadline_wall is None:
                        self.return_deadline_wall = now+self.cfg['return_timeout_s']
            elif now-self.cancel_started > self.cfg['cancel_timeout_s']:
                self.phase, self.reason = 'blocked', 'cancel_timeout_stop_not_confirmed'
                self.canceling = False
                if self.bounded_observation_enabled():
                    self.close_observation('blocked:cancel_timeout_stop_not_confirmed', invalid=True)
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
        if self.phase in ('exploring', 'observing', 'settling', 'inspecting', 'observation_select', 'observation_navigating') and now-self.started >= self.cfg['mission_timeout_s']:
            self.mission_timed_out = True
            self.start_return('mission_timeout_incomplete')
            self.publish_status()
            return
        if (self.bounded_observation_enabled() and self.home_frontier_resume_active and
                now >= self.return_deadline_wall-self.cfg['home_frontier_return_reserve_s']):
            self.start_return('home_frontier_return_reserve_reached')
            self.publish_status()
            return
        if (self.bounded_observation_enabled() and self.observation_queue is not None and
                not self.observation_closed and now >= self.observation_deadline_wall):
            self.start_return('bounded_observation_wall_timeout')
            self.publish_status()
            return
        if self.phase in ('returning', 'home_verification') and self.return_budget_expired(now):
            self.block('return_total_timeout')
            self.publish_status()
            return
        if self.goal_handle is not None or self.send_pending:
            timeout = self.cfg['return_timeout_s'] if self.phase == 'returning' else self.cfg['goal_timeout_s']
            if now-self.goal_started > timeout:
                if self.phase == 'returning':
                    self.block('return_navigation_timeout')
                elif self.goal.kind == 'observation_queue':
                    self.observation_cancel_outcome = ('navigation_failed', 'navigation_timeout')
                    self.cancel_to('observation_select', 'observation_navigation_timeout')
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
                elif self.return_attempts < self.return_attempt_limit():
                    self.event('return_retry', detail=detail)
                else:
                    self.block('return_navigation_failed')
            elif success:
                if self.cfg.get('require_processed_inspection', False):
                    self.begin_inspection(now)
                else:
                    self.phase, self.dwell_until = 'observing', now+self.cfg['observation_dwell_s']
            else:
                if self.goal.kind == 'observation_queue':
                    self.finish_observation('navigation_failed', detail)
                    self.phase = 'observation_select'
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
            if self.goal.kind == 'observation_queue':
                self.finish_observation('inspection_satisfied', 'processed_frame_window')
                if self.canceling:
                    self.publish_status()
                    return
                self.phase = 'observation_select'
            else:
                anchor = self.grid.to_cell(*self.goal.anchor)
                observed = (not self.grid.frontier(anchor) if self.goal.kind == 'frontier' else
                            self.coverage.contains(self.grid.to_world(anchor), self.grid.resolution))
                if not observed:
                    self.failures.fail(self.goal, now, 'arrived_without_new_observation')
                self.phase = 'exploring'
        if self.phase == 'observation_select':
            self.step_observation(now, pose)
        if self.phase == 'exploring':
            ready = (self.detector_received is not None and now-self.detector_received <= self.cfg['detector_timeout_s']
                     and self.coverage.frames > 0)
            self.selection = self.planner.select(self.planning_grid(), pose, self.coverage, self.failures,
                                                 now, ready, observed_grid=self.grid)
            state = self.selection.state
            self.map_complete = (self.selection.frontier_count == 0 and self.selection.unobserved_count == 0 and
                                 state != 'unsafe_or_unlocalized' and
                                 not any(f.kind == 'frontier' for f in self.failures.entries))
            self.visual_complete = (not self.bounded_observation_enabled() and ready and self.selection.uncovered_count == 0)
            if self.bounded_observation_enabled() and self.map_complete:
                self.begin_observation_stage(now, pose)
            elif state == 'goal':
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
            if self.return_budget_expired(now):
                self.block('return_total_timeout')
            elif self.bounded_observation_enabled() and self.return_attempts >= self.return_attempt_limit():
                self.block('return_navigation_attempt_budget_exhausted')
            else:
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
                    # Final planning also consumes the same return budget. Do
                    # not certify return or resume using the pre-planning time.
                    now = time.monotonic()
                    if self.bounded_observation_enabled() and self.return_budget_expired(now):
                        self.block('return_total_timeout')
                        self.publish_status()
                        return
                    self.map_complete = (final.frontier_count == 0 and final.unobserved_count == 0 and
                                         final.state != 'unsafe_or_unlocalized' and
                                         not any(f.kind == 'frontier' for f in self.failures.entries))
                    self.visual_complete = (not self.bounded_observation_enabled() and ready and final.uncovered_count == 0)
                    if self.resume_home_frontier(final, now):
                        pass
                    elif (not self.bounded_observation_enabled() and final.state == 'goal'
                            and now-self.started < self.cfg['mission_timeout_s']):
                        self.event('completion_changed_during_return', selection=asdict(final))
                        self.phase, self.home_stable_since = 'exploring', None
                    else:
                        self.phase = 'done'
                        if not self.map_complete or not self.visual_complete:
                            self.reason += ';final_observation_checks_incomplete'
                        self.save_result(True, pose)
            else:
                self.home_stable_since = None
            if self.phase != 'done' and self.return_budget_expired(now):
                self.block('home_pose_verification_failed')
        self.publish_status()

    def publish_status(self):
        value = {'phase': self.phase, 'reason': self.reason, 'autonomous': self.cfg['autonomous'],
                 'confirmed_target_count': len(self.targets.confirmed()),
                 'map_complete': self.map_complete, 'visual_coverage_complete': self.visual_complete,
                 'unresolved_failures': len(self.failures.entries),
                 'inspection': self.inspection_snapshot() if getattr(self, 'inspection', None) else None,
                 'last_inspection_failure': getattr(self, 'last_inspection_failure', None),
                 'observation_actions': self.observation_snapshot(),
                 'selection': asdict(self.selection) if self.selection else None}
        self.status_pub.publish(String(data=json.dumps(value)))

    def save_result(self, returned, final_pose=None):
        if self.result_written:
            return
        if self.bounded_observation_enabled():
            self.visual_complete = False
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
                  'observation_actions': self.observation_snapshot(),
                  'frame_retention': (self.frame_retention.snapshot()
                      if getattr(self, 'frame_retention', None) is not None else None),
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
        if getattr(node, 'bounded_observation_enabled', lambda: False)():
            # Snapshot a terminal attempt's partial Ledger before cancel_to
            # invalidates it. Pending/accepted Nav remains active and owned.
            node.close_observation('shutdown:'+reason, invalid=True)
    except Exception as exc:
        evidence['errors'].append('close_observation: ' + str(exc))
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
    if (getattr(node, 'observation_active', None) is not None and node.cancel_terminal and
            not node.send_pending and node.goal_handle is None):
        try:
            # The timer is stopped, so its ordinary cancellation retirement
            # branch cannot run. An ACK alone never reaches this branch.
            node.finish_observation(*node.observation_cancel_outcome)
        except Exception as exc:
            evidence['errors'].append('retire_observation: ' + str(exc))
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
