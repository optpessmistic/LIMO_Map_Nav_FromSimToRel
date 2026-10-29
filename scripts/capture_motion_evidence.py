#!/usr/bin/env python3
"""Read-only Harmonic/Jazzy motion evidence; never publishes commands or truth.

Start before an independently controlled test. A completed Action or wheel
odometry alone does not prove physical motion. This diagnostic compares native
Gazebo model pose, odometry and timestamped SLAM TF offline, without changing
the controller's inputs. It produces evidence, not an automatic pass verdict.
"""
import argparse
import bisect
import json
import math
from pathlib import Path
import threading
import time

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformException, TransformListener


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def summarize(samples):
    if not samples:
        return {"count": 0}
    total = 0.0
    accumulated = [0.0]
    for previous, current in zip(samples, samples[1:]):
        total += angle_delta(current["yaw"], previous["yaw"])
        accumulated.append(total)
    return {"count": len(samples), "first": samples[0], "last": samples[-1],
            "yaw_accumulated_rad": total, "yaw_accumulated_min_rad": min(accumulated),
            "yaw_accumulated_max_rad": max(accumulated),
            "max_displacement_m": max(math.hypot(s["x"]-samples[0]["x"], s["y"]-samples[0]["y"]) for s in samples)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60)
    ap.add_argument("--motion-wait", type=float, default=90)
    ap.add_argument("--world", default="limo_maze")
    ap.add_argument("--model", default="limo")
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    for value in (args.duration, args.motion_wait):
        if not math.isfinite(value) or not 0 < value <= 300:
            ap.error("duration and motion-wait must be finite and within (0, 300] seconds")
    if args.output.exists():
        ap.error("output already exists; choose a new evidence filename")
    topic = "/world/" + args.world + "/pose/info"
    rclpy.init()
    node = rclpy.create_node("offline_world_odom_diagnostic", parameter_overrides=[Parameter("use_sim_time", value=True)])
    buffer = Buffer(cache_time=Duration(seconds=args.duration+args.motion_wait+10), node=node)
    listener = TransformListener(buffer, node)
    gz_samples, odom_samples = [], []
    lock = threading.Lock()

    def gz_cb(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
        for p in msg.pose:
            if p.name == args.model:
                sample = {"sim_s": stamp, "yaw": yaw(p.orientation), "x": p.position.x,
                          "y": p.position.y, "z": p.position.z, "id": p.id}
                with lock:
                    if not gz_samples or stamp - gz_samples[-1]["sim_s"] >= .019:
                        gz_samples.append(sample)
                return

    def odom_cb(msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose
        if not odom_samples or stamp - odom_samples[-1]["sim_s"] >= .019:
            odom_samples.append({"sim_s": stamp, "yaw": yaw(p.orientation), "x": p.position.x,
                                 "y": p.position.y, "angular_velocity": msg.twist.twist.angular.z,
                                 "linear_velocity": math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)})

    sub = node.create_subscription(Odometry, "/odom", odom_cb, qos_profile_sensor_data)
    gz = GzNode()
    if not gz.subscribe(Pose_V, topic, gz_cb):
        raise RuntimeError("Could not subscribe to native Gazebo model poses")
    start, motion_start, ready = time.monotonic(), None, False
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=.05)
        elapsed = time.monotonic() - start
        if gz_samples and odom_samples and not ready:
            print("READY: native Gazebo limo world pose and ROS odom received; waiting for controlled motion; no commands published.", flush=True)
            ready = True
        if motion_start is None and odom_samples and (abs(odom_samples[-1]["angular_velocity"]) > .05
                                                       or odom_samples[-1]["linear_velocity"] > .01):
            motion_start = time.monotonic()
            print(f"MOTION DETECTED: collecting {args.duration:g} wall seconds from now.", flush=True)
        if motion_start is not None and time.monotonic() - motion_start >= args.duration:
            break
        if motion_start is None and elapsed >= args.motion_wait:
            break
    gz.unsubscribe(topic)
    with lock:
        world = list(gz_samples)
    odom_times = [sample["sim_s"] for sample in odom_samples]
    aligned = []
    for sample in world:
        index = bisect.bisect_left(odom_times, sample["sim_s"])
        choices = [i for i in (index-1, index) if 0 <= i < len(odom_times)]
        if not choices:
            continue
        closest = min(choices, key=lambda i: abs(odom_times[i]-sample["sim_s"]))
        if abs(odom_times[closest]-sample["sim_s"]) > .04:
            continue
        record = {"sim_s": sample["sim_s"], "world_yaw": sample["yaw"],
                  "odom_yaw": odom_samples[closest]["yaw"],
                  "odom_dt_s": odom_times[closest]-sample["sim_s"]}
        for target, source, field in (("map", "base_link", "map_base_yaw"), ("map", "odom", "map_odom_yaw")):
            try:
                tf = buffer.lookup_transform(target, source, Time(nanoseconds=round(sample["sim_s"]*1e9)))
                record[field] = yaw(tf.transform.rotation)
            except TransformException:
                record[field] = None
        aligned.append(record)
    report = {"read_only": True, "truth_used_only_for_offline_diagnosis": True,
              "world_topic": topic, "world_entity": args.model,
              "motion_detected": motion_start is not None, "wall_seconds": time.monotonic()-start,
              "requested_seconds_after_motion": args.duration, "world_summary": summarize(world),
              "odom_summary": summarize(odom_samples), "aligned_count": len(aligned),
              "world_samples": world, "odom_samples": odom_samples, "aligned_samples": aligned}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2)
        stream.write("\n")
    print(json.dumps({k:v for k,v in report.items() if not k.endswith("samples")}, indent=2), flush=True)
    listener.unregister()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
