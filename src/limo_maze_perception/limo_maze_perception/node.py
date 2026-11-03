"""Explicitly enabled simulation surrogate; never emits search-coverage claims."""
import json
from math import isfinite
import signal
import threading
import time

import message_filters
import rclpy
from rclpy.clock import Clock, ClockType, JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

from .core import DetectorConfig, Intrinsics, process_red_frame, observation_payload, validate_frame_metadata
from .evidence import FrameEvidence
from limo_evidence_protocol.retention import Policy as RetentionPolicy
from .retention_adapter import ProducerRetention, RuntimePolicy


def stamp_seconds(stamp):
    if not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("Invalid ROS timestamp nanoseconds")
    return stamp.sec + stamp.nanosec * 1e-9


class RedRGBDBaseline(Node):
    def __init__(self, **kwargs):
        super().__init__("red_rgbd_sim_baseline", **kwargs)
        defaults = {"enabled": False, "depth_registered_to_color": False,
                    "color_topic": "/camera/color/image_raw", "depth_topic": "/camera/depth/image_raw",
                    "camera_info_topic": "/camera/color/camera_info", "output_topic": "/mission/target_observations",
                    "publish_frame_processing": False, "frame_processing_topic": "/mission/frame_processing",
                    "retention_enabled": False, "retention_output_dir": "",
                    "retention_request_topic": "/mission/frame_retention/request",
                    "retention_result_topic": "/mission/frame_retention/result",
                    "retention_max_payload_bytes": 80*1024*1024,
                    "retention_max_file_bytes": 96*1024*1024, "retention_max_jobs": 32,
                    "retention_ttl_s": 15.0, "retention_request_timeout_s": 10.0,
                    "retention_worker_timeout_s": 5.0, "retention_shutdown_timeout_s": 8.0,
                    "sync_slop_seconds": 0.05, "camera_info_max_age_seconds": 2.0,
                    "min_area_pixels": 30, "min_depth_pixels": 8,
                    "min_depth_m": 0.15, "max_depth_m": 6.0}
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self.enabled = self.get_parameter("enabled").value
        self.registered = self.get_parameter("depth_registered_to_color").value
        self.slop = float(self.get_parameter("sync_slop_seconds").value)
        self.info_max_age = float(self.get_parameter("camera_info_max_age_seconds").value)
        if not isfinite(self.slop) or self.slop <= 0 or not isfinite(self.info_max_age) or self.info_max_age <= 0:
            raise ValueError("Synchronization tolerances must be finite and positive")
        self.config = DetectorConfig(min_area_pixels=self.get_parameter("min_area_pixels").value,
                                     min_depth_pixels=self.get_parameter("min_depth_pixels").value,
                                     min_depth_m=self.get_parameter("min_depth_m").value,
                                     max_depth_m=self.get_parameter("max_depth_m").value)
        self.config.validate()
        self.bridge, self.info, self.last_stamp = CvBridge(), None, None
        self.publisher = self.create_publisher(String, self.get_parameter("output_topic").value, 10)
        self.frame_evidence = self.frame_publisher = self.evidence_clock_jump = None
        self.retention = self.retention_timer = None
        self.retention_shutdown = None
        self.retention_versions = {}
        self.retention_clock_invalid = threading.Event()
        self.retention_exit_reason = 'shutdown'
        if self.get_parameter("retention_enabled").value and not self.get_parameter("publish_frame_processing").value:
            raise ValueError('retention_enabled requires publish_frame_processing')
        if self.get_parameter("publish_frame_processing").value:
            self.frame_evidence = FrameEvidence(self.config, self.slop, self.info_max_age)
            self.frame_publisher = self.create_publisher(String, self.get_parameter("frame_processing_topic").value, 10)
            epoch_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                   reliability=ReliabilityPolicy.RELIABLE)
            self.epoch_sub = self.create_subscription(String, "/mission/map_epoch", self.on_epoch, epoch_qos)
            self.evidence_clock_jump = self.get_clock().create_jump_callback(
                JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1), on_clock_change=True),
                pre_callback=self.on_evidence_clock_jump)
        if self.get_parameter("retention_enabled").value:
            output = self.get_parameter("retention_output_dir").value
            if not output.strip():
                raise ValueError('retention_output_dir must explicitly name a new directory')
            self.retention = ProducerRetention(output, self.frame_evidence.session_id,
                RetentionPolicy(max_payload_bytes=self.get_parameter('retention_max_payload_bytes').value,
                    max_file_bytes=self.get_parameter('retention_max_file_bytes').value,
                    max_jobs=self.get_parameter('retention_max_jobs').value,
                    ttl_s=self.get_parameter('retention_ttl_s').value,
                    request_timeout_s=self.get_parameter('retention_request_timeout_s').value),
                RuntimePolicy(worker_timeout_s=self.get_parameter('retention_worker_timeout_s').value,
                    shutdown_timeout_s=self.get_parameter('retention_shutdown_timeout_s').value))
            retention_qos = QoSProfile(depth=128, reliability=ReliabilityPolicy.RELIABLE,
                                      durability=DurabilityPolicy.VOLATILE)
            self.retention_pub = self.create_publisher(String, self.get_parameter('retention_result_topic').value,
                                                       retention_qos)
            self.retention_sub = self.create_subscription(String, self.get_parameter('retention_request_topic').value,
                                                          self.on_retention_request, retention_qos)
            self.retention_timer = self.create_timer(.05, self.on_retention_tick,
                clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.info_sub = self.create_subscription(CameraInfo, self.get_parameter("camera_info_topic").value,
                                                 self.on_info, qos_profile_sensor_data)
        self.rgb_sub = message_filters.Subscriber(self, Image, self.get_parameter("color_topic").value,
                                                  qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, self.get_parameter("depth_topic").value,
                                                    qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, self.slop,
                                                                 allow_headerless=False)
        self.sync.registerCallback(self.on_pair)
        self.get_logger().warn("Red RGB-D detector is a SIMULATION SURROGATE, not final object identity recognition; no coverage is published.")
        if not self.enabled or not self.registered:
            self.get_logger().warn("Detector inactive: both enabled:=true and depth_registered_to_color:=true are required at launch.")

    def on_info(self, msg):
        self.info = msg

    def destroy_node(self):
        try:
            if self.retention is not None and self.retention_shutdown is None:
                self.retention_timer.cancel()
                self.retention_shutdown = self.retention.shutdown(self.retention_exit_reason)
                report = self.retention_shutdown
                self.get_logger().info('retention_shutdown ' + json.dumps({
                    'shutdown': report['shutdown'], 'writer': report['writer'],
                    'fault': report['fault'], 'reason': report['reason']}))
        finally:
            if getattr(self, "evidence_clock_jump", None) is not None:
                self.evidence_clock_jump.unregister()
                self.evidence_clock_jump = None
            result = super().destroy_node()
        return result

    def on_evidence_clock_jump(self):
        self.frame_evidence.invalidate_clock()
        # Clock hooks may run outside the executor. Never call Core here.
        self.retention_clock_invalid.set()

    def retention_clock_guard(self):
        if self.retention is not None and self.retention_clock_invalid.is_set():
            self.retention.close('ros_clock_reset', time.monotonic())

    def on_retention_request(self, msg):
        self.retention_clock_guard()
        result = self.retention.request(msg.data, time.monotonic())
        self.retention_pub.publish(String(data=json.dumps(result, allow_nan=False)))

    def on_retention_tick(self):
        self.retention_clock_guard()
        self.retention.tick(time.monotonic())
        for result in self.retention.results():
            raw = json.dumps(result, sort_keys=True, allow_nan=False)
            rid = result['retention_id']
            if self.retention_versions.get(rid) != raw:
                self.retention_pub.publish(String(data=raw))
                self.retention_versions[rid] = raw

    def on_epoch(self, msg):
        self.retention_clock_guard()
        self.frame_evidence.on_epoch(msg.data, self.get_clock().now().nanoseconds)
        if self.retention is not None:
            self.retention.on_epoch(self.frame_evidence.epoch, self.frame_evidence.epoch_ready, time.monotonic())

    def on_pair(self, color, depth):
        # Epoch and sample watermarks qualify only the optional evidence stream;
        # the existing target-observation contract and defaults stay unchanged.
        record = (self.frame_evidence.begin(color, depth, self.info, self.get_clock().now().nanoseconds,
                                            enabled=self.enabled, registered=self.registered)
                  if self.frame_evidence is not None else None)
        stats, evidence_error = None, "algorithm_not_completed"
        if not self.enabled or not self.registered or self.info is None:
            self.publish_frame_record(record, stats, evidence_error, color, depth)
            return
        try:
            info = self.info
            stamp = stamp_seconds(color.header.stamp)
            validate_frame_metadata(color_stamp=stamp, depth_stamp=stamp_seconds(depth.header.stamp),
                                    info_stamp=stamp_seconds(info.header.stamp), color_frame=color.header.frame_id,
                                    depth_frame=depth.header.frame_id, info_frame=info.header.frame_id,
                                    registered=self.registered, slop=self.slop, max_info_age=self.info_max_age)
            if self.last_stamp == stamp:
                self.publish_frame_record(record, stats, "duplicate_target_sample", color, depth)
                return
            if color.encoding not in ("rgb8", "bgr8"):
                raise ValueError("This baseline requires RGB8 or BGR8 color images")
            if any(not isfinite(d) or abs(d) > 1e-8 for d in info.d):
                raise ValueError("Baseline requires zero-distortion simulation imagery; distorted real images need rectification")
            if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
                raise ValueError("Binned/cropped CameraInfo requires calibrated intrinsic adjustment")
            intrinsics = Intrinsics(info.width, info.height, info.k[0], info.k[4], info.k[2], info.k[5])
            color_array = self.bridge.imgmsg_to_cv2(color, desired_encoding="passthrough")
            depth_array = self.bridge.imgmsg_to_cv2(depth, desired_encoding="passthrough")
            detections, stats = process_red_frame(color_array, depth_array, intrinsics, depth.encoding, color.encoding, self.config)
            payload = observation_payload(stamp, color.header.frame_id, detections)
            self.publisher.publish(String(data=json.dumps(payload, allow_nan=False)))
            self.last_stamp = stamp
            evidence_error = None
        except (ValueError, TypeError, IndexError, CvBridgeError) as exc:
            evidence_error = "input_or_processing_error"
            self.get_logger().warn(f"Dropped RGB-D pair: {exc}", throttle_duration_sec=5.0)
        self.publish_frame_record(record, stats, evidence_error, color, depth)

    def publish_frame_record(self, record, stats, error, color=None, depth=None):
        if record is not None:
            payload = self.frame_evidence.finish(record, stats, self.get_clock().now().nanoseconds, error)
            if self.retention is not None:
                self.retention_clock_guard()
                if (self.frame_evidence.clock_reset or
                        self.retention.core is not None and not self.frame_evidence.epoch_ready):
                    self.retention.close('processing_epoch_or_clock_invalid', time.monotonic())
                elif payload['status'] == 'processed':
                    # Exact same callback bytes, including padding. Offer before
                    # metadata publication; never reconstruct from subscriptions.
                    self.retention.offer(payload, memoryview(color.data), memoryview(depth.data), time.monotonic())
            self.frame_publisher.publish(String(data=json.dumps(payload, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    old_signals = {}
    try:
        node = RedRGBDBaseline()
        if getattr(node, 'retention', None) is None:
            rclpy.spin(node)
        else:
            stop = threading.Event()
            def stop_requested(signum, frame):
                node.retention_exit_reason = signal.Signals(signum).name
                stop.set()
            for signum in (signal.SIGINT, signal.SIGTERM):
                old_signals[signum] = signal.signal(signum, stop_requested)
            while rclpy.ok() and not stop.is_set():
                rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        # Jazzy's signal handler may stop the context before spin returns.
        # Do not hide an unrelated exception raised while it is still active.
        if rclpy.ok():
            raise
    except Exception:
        if node is not None:
            node.retention_exit_reason = 'callback_exception'
        raise
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            for signum, previous in old_signals.items():
                signal.signal(signum, previous)
            rclpy.try_shutdown()
    if node is not None and getattr(node, 'retention_shutdown', None) is not None:
        report = node.retention_shutdown
        if report['fault'] or not report['shutdown']['final_checkpoint_persisted']:
            return 2
    return 0
