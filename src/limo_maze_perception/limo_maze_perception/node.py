"""Explicitly enabled simulation surrogate; never emits search-coverage claims."""
import json
from math import isfinite

import message_filters
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

from .core import DetectorConfig, Intrinsics, detect_red_objects, observation_payload, validate_frame_metadata


def stamp_seconds(stamp):
    if not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("Invalid ROS timestamp nanoseconds")
    return stamp.sec + stamp.nanosec * 1e-9


class RedRGBDBaseline(Node):
    def __init__(self):
        super().__init__("red_rgbd_sim_baseline")
        defaults = {"enabled": False, "depth_registered_to_color": False,
                    "color_topic": "/camera/color/image_raw", "depth_topic": "/camera/depth/image_raw",
                    "camera_info_topic": "/camera/color/camera_info", "output_topic": "/mission/target_observations",
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

    def on_pair(self, color, depth):
        if not self.enabled or not self.registered or self.info is None:
            return
        try:
            info = self.info
            stamp = stamp_seconds(color.header.stamp)
            validate_frame_metadata(color_stamp=stamp, depth_stamp=stamp_seconds(depth.header.stamp),
                                    info_stamp=stamp_seconds(info.header.stamp), color_frame=color.header.frame_id,
                                    depth_frame=depth.header.frame_id, info_frame=info.header.frame_id,
                                    registered=self.registered, slop=self.slop, max_info_age=self.info_max_age)
            if self.last_stamp == stamp:
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
            detections = detect_red_objects(color_array, depth_array, intrinsics, depth.encoding, color.encoding, self.config)
            payload = observation_payload(stamp, color.header.frame_id, detections)
            self.publisher.publish(String(data=json.dumps(payload, allow_nan=False)))
            self.last_stamp = stamp
        except (ValueError, TypeError, IndexError, CvBridgeError) as exc:
            self.get_logger().warn(f"Dropped RGB-D pair: {exc}", throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = RedRGBDBaseline()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
