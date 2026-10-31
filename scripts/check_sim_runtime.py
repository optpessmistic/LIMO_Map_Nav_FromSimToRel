#!/usr/bin/env python3
"""Read-only ROS integration probe. Run while sim.launch.py is active."""

import argparse
from array import array
from collections import Counter, defaultdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import sys
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from odom_integrity import MIN_OBSERVATION_NS, OdomIntegrity, stamp_to_nanoseconds


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="Maximum wall-clock wait for sensors (default: %(default)s s).")
    parser.add_argument("--expected-lidar-fov-deg", type=float, choices=(240.0, 360.0),
                        default=240.0)
    parser.add_argument("--max-initial-sim-time", type=float, default=None,
                        help="For a fresh launch, reject clocks older than this many simulation seconds.")
    parser.add_argument("--output", type=Path,
                        default=Path("outputs/ros_smoke/sim_runtime.json"))
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be a finite positive number of seconds")
    if args.max_initial_sim_time is not None and (
        not math.isfinite(args.max_initial_sim_time) or args.max_initial_sim_time < 0
    ):
        parser.error("--max-initial-sim-time must be finite and nonnegative")
    rclpy.init()
    node = rclpy.create_node(
        "limo_sim_interface_probe",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    counts, latest, first_stamps = Counter(), {}, {}
    observed_stamps = defaultdict(set)
    odom_integrity = OdomIntegrity()
    subscriptions = []
    topics = {
        "/clock": Clock,
        "/odom": Odometry,
        "/scan": LaserScan,
        "/imu": Imu,
        "/joint_states": JointState,
        "/camera/color/image_raw": Image,
        "/camera/depth/image_raw": Image,
        "/camera/color/camera_info": CameraInfo,
        "/camera/depth/camera_info": CameraInfo,
    }

    def receive(topic, msg):
        counts[topic] += 1
        latest[topic] = msg
        stamp = msg.clock if topic == "/clock" else msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        first_stamps.setdefault(topic, stamp_ns)
        observed_stamps[topic].add(stamp_ns)
        if topic == "/odom":
            try:
                odom_stamp_ns = stamp_to_nanoseconds(stamp.sec, stamp.nanosec)
            except ValueError:
                odom_stamp_ns = None
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            linear, angular = msg.twist.twist.linear, msg.twist.twist.angular
            odom_integrity.observe(
                odom_stamp_ns, (p.x, p.y, p.z, q.x, q.y, q.z, q.w),
                (linear.x, linear.y, linear.z, angular.x, angular.y, angular.z),
            )

    for topic, msg_type in topics.items():
        subscriptions.append(node.create_subscription(
            msg_type, topic, lambda msg, name=topic: receive(name, msg),
            qos_profile_sensor_data,
        ))
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    start = time.monotonic()
    transform = None
    transforms = {}
    while time.monotonic() - start < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        clock_span_ns = (latest["/clock"].clock.sec * 1_000_000_000
                         + latest["/clock"].clock.nanosec - first_stamps["/clock"]
                         if "/clock" in latest else 0)
        if (all(counts[topic] >= 3 for topic in topics)
                and odom_integrity.observation_sufficient
                and clock_span_ns >= MIN_OBSERVATION_NS):
            try:
                image_transform = buffer.lookup_transform(
                    "odom", "camera_optical_frame",
                    Time.from_msg(latest["/camera/color/image_raw"].header.stamp),
                )
                links = (("odom", "base_footprint"), ("base_footprint", "base_link"),
                         ("base_link", "laser_link"), ("base_link", "camera_optical_frame"))
                transforms = {
                    f"{parent}->{child}": buffer.lookup_transform(parent, child, Time())
                    for parent, child in links
                }
                transform = image_transform
                break
            except TransformException:
                pass

    odom_report = odom_integrity.report()
    errors = list(odom_report["errors"])
    clock_span_ns = (latest["/clock"].clock.sec * 1_000_000_000
                     + latest["/clock"].clock.nanosec - first_stamps["/clock"]
                     if "/clock" in latest else 0)
    if clock_span_ns < MIN_OBSERVATION_NS:
        errors.append(f"/clock: observed span {clock_span_ns} ns is shorter than {MIN_OBSERVATION_NS} ns")
    publisher_counts = {topic: node.count_publishers(topic) for topic in topics}
    for topic, count in publisher_counts.items():
        if count != 1:
            errors.append(f"{topic}: expected exactly one ROS publisher, found {count}")
    if args.max_initial_sim_time is not None and "/clock" in first_stamps:
        if first_stamps["/clock"] * 1e-9 > args.max_initial_sim_time:
            errors.append("Clock is older than the allowed fresh-launch limit; check for an orphan server")
    for topic in topics:
        if counts[topic] < 3:
            errors.append(f"{topic}: expected >=3 messages, received {counts[topic]}")
    expected_frames = {
        "/odom": "odom", "/scan": "laser_link", "/imu": "imu_link",
        "/camera/color/image_raw": "camera_optical_frame",
        "/camera/depth/image_raw": "camera_optical_frame",
        "/camera/color/camera_info": "camera_optical_frame",
        "/camera/depth/camera_info": "camera_optical_frame",
    }
    for topic, expected in expected_frames.items():
        if topic in latest and latest[topic].header.frame_id != expected:
            errors.append(f"{topic}: frame {latest[topic].header.frame_id!r} != {expected!r}")
    for topic, msg in latest.items():
        stamp = msg.clock if topic == "/clock" else msg.header.stamp
        if stamp.sec * 1_000_000_000 + stamp.nanosec <= first_stamps[topic]:
            errors.append(f"{topic}: simulation timestamp did not advance")
    if "/odom" in latest and latest["/odom"].child_frame_id != "base_footprint":
        errors.append("odom child_frame_id must be base_footprint")
    if transform is None:
        errors.append("Required TF chain unavailable, including odom->camera at RGB observation stamp")
    for topic, encoding in {
        "/camera/color/image_raw": "rgb8", "/camera/depth/image_raw": "32FC1",
    }.items():
        if topic in latest and latest[topic].encoding != encoding:
            errors.append(f"{topic}: encoding {latest[topic].encoding!r} != {encoding!r}")
    for topic in ("/camera/color/camera_info", "/camera/depth/camera_info"):
        if topic in latest and (not all(math.isfinite(value) for value in latest[topic].k)
                                or latest[topic].k[0] <= 0 or latest[topic].k[4] <= 0):
            errors.append(f"{topic}: invalid focal lengths")
        if topic in latest and any(abs(value) > 1e-9 for value in latest[topic].d):
            errors.append(f"{topic}: expected a rectified camera with zero distortion")
    camera_topics = ("/camera/color/image_raw", "/camera/depth/image_raw",
                     "/camera/color/camera_info", "/camera/depth/camera_info")
    shapes = {(latest[topic].width, latest[topic].height)
              for topic in camera_topics if topic in latest}
    if len(shapes) > 1:
        errors.append("RGB, depth and CameraInfo dimensions differ")
    shared_image_stamps = observed_stamps["/camera/color/image_raw"] & observed_stamps["/camera/depth/image_raw"]
    if not shared_image_stamps:
        errors.append("No RGB/depth pair with the same simulation timestamp was received")
    if "/scan" in latest and len(latest["/scan"].ranges) != 720:
        errors.append("scan must contain 720 beams")
    if "/scan" in latest:
        observed_fov = math.degrees(latest["/scan"].angle_max - latest["/scan"].angle_min)
        if abs(observed_fov - args.expected_lidar_fov_deg) > 0.01:
            errors.append(f"scan FOV {observed_fov:.5f} != expected {args.expected_lidar_fov_deg}")

    report = {"passed": not errors, "errors": errors,
              "checked_at_utc": datetime.now(timezone.utc).isoformat(),
              "environment": {name: os.environ.get(name, "") for name in (
                  "ROS_DISTRO", "ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE", "GZ_PARTITION")},
              "wall_seconds": round(time.monotonic() - start, 2), "counts": dict(counts),
              "shared_rgb_depth_stamps": len(shared_image_stamps),
              "odom_integrity": odom_report,
              "observed_clock_span_ns": clock_span_ns,
              "ros_publisher_counts": publisher_counts,
              "first_clock_seconds": first_stamps.get("/clock", 0) * 1e-9,
              "max_initial_sim_time": args.max_initial_sim_time}
    report["frames"] = {topic: latest[topic].header.frame_id
                        for topic in expected_frames if topic in latest}
    report["images"] = {
        topic: {"encoding": latest[topic].encoding,
                "width": latest[topic].width, "height": latest[topic].height}
        for topic in ("/camera/color/image_raw", "/camera/depth/image_raw") if topic in latest
    }
    if "/camera/depth/image_raw" in latest:
        depth = latest["/camera/depth/image_raw"]
        if depth.encoding == "32FC1":
            values = array("f", bytes(depth.data))
            if bool(depth.is_bigendian) != (sys.byteorder == "big"):
                values.byteswap()
            finite_depth = [value for value in values if math.isfinite(value) and value > 0]
            report["depth_values_m"] = {
                "finite_positive_count": len(finite_depth),
                "minimum": min(finite_depth) if finite_depth else None,
                "maximum": max(finite_depth) if finite_depth else None,
            }
    if "/camera/color/image_raw" in latest:
        rgb_data = latest["/camera/color/image_raw"].data
        report["rgb_byte_range"] = [min(rgb_data), max(rgb_data)] if rgb_data else []
    report["timestamps"] = {
        topic: {"sec": latest[topic].header.stamp.sec,
                "nanosec": latest[topic].header.stamp.nanosec}
        for topic in expected_frames if topic in latest
    }
    report["camera_info"] = {
        topic: {"width": latest[topic].width, "height": latest[topic].height,
                "k": list(latest[topic].k), "p": list(latest[topic].p),
                "d": list(latest[topic].d), "distortion_model": latest[topic].distortion_model}
        for topic in ("/camera/color/camera_info", "/camera/depth/camera_info") if topic in latest
    }
    report["tf_chain"] = {}
    for name, tf in transforms.items():
        pose = tf.transform
        report["tf_chain"][name] = {
            "xyz": [pose.translation.x, pose.translation.y, pose.translation.z],
            "quaternion_xyzw": [pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w],
        }
    if transform is not None:
        pose = transform.transform
        report["odom_to_camera_at_rgb_stamp"] = {
            "stamp": {"sec": transform.header.stamp.sec, "nanosec": transform.header.stamp.nanosec},
            "xyz": [pose.translation.x, pose.translation.y, pose.translation.z],
            "quaternion_xyzw": [pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w],
        }
    if "/scan" in latest:
        scan = latest["/scan"]
        ranges = scan.ranges
        finite = [value for value in ranges if math.isfinite(value)]
        rear = [(index, value) for index, value in enumerate(ranges)
                if abs(scan.angle_min + index * scan.angle_increment) > math.radians(120) + 1e-6]
        rear_finite = [value for _, value in rear if math.isfinite(value)]
        sample_beams = []
        for degrees in (-180, -170, -150, -130, -120, -90, 0, 90, 120, 130, 150, 170, 180):
            requested_angle = math.radians(degrees)
            if scan.angle_min - 1e-6 <= requested_angle <= scan.angle_max + 1e-6:
                index = max(0, min(len(ranges) - 1, round(
                    (requested_angle - scan.angle_min) / scan.angle_increment)))
                value = ranges[index]
                sample_beams.append({"index": index,
                                     "angle_deg": math.degrees(scan.angle_min + index * scan.angle_increment),
                                     "range_m": value if math.isfinite(value) else None,
                                     "nonfinite": None if math.isfinite(value) else str(value)})
        report["scan"] = {"beams": len(ranges), "finite": len(finite),
                          "minimum_m": min(finite) if finite else None,
                          "angle_min": scan.angle_min, "angle_max": scan.angle_max,
                          "angle_increment": scan.angle_increment,
                          "range_min": scan.range_min, "range_max": scan.range_max,
                          "fov_deg": math.degrees(scan.angle_max - scan.angle_min),
                          "expected_fov_deg": args.expected_lidar_fov_deg,
                          "rear_abs_angle_over_120_deg": {
                              "beams": len(rear), "finite": len(rear_finite),
                              "negative_infinity": sum(value == -math.inf for _, value in rear),
                              "positive_infinity": sum(value == math.inf for _, value in rear),
                              "nan": sum(math.isnan(value) for _, value in rear),
                              "minimum_m": min(rear_finite) if rear_finite else None,
                              "finite_at_min_range": sum(value <= scan.range_min + 1e-6 for value in rear_finite),
                              "finite_within_0_12_m": sum(value <= 0.12 for value in rear_finite)},
                          "selected_beams": sample_beams}
    serialized = json.dumps(report, indent=2) + "\n"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    print(f"Saved: {args.output.resolve()}")
    listener.unregister()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
