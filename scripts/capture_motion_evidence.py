#!/usr/bin/env python3
"""Read-only Harmonic/Jazzy motion evidence; never publishes commands or truth.

Start before an independently controlled test. A completed Action or wheel
odometry alone does not prove physical motion. This diagnostic compares native
Gazebo model pose, odometry and timestamped SLAM TF offline, without changing
the controller's inputs. It produces evidence, not an automatic pass verdict.
"""
import argparse
import bisect
import gc
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
from types import SimpleNamespace


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


def load_runtime():
    """Importing this diagnostic for offline tests never initializes ROS/Gazebo."""
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V
    import rclpy
    from rclpy.duration import Duration
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from tf2_ros import Buffer, TransformException, TransformListener
    return SimpleNamespace(**locals())


def persist(stream, report):
    # Serialize first: invalid data must not truncate the previous checkpoint.
    encoded = json.dumps(report, indent=2, allow_nan=False) + "\n"
    stream.seek(0)
    stream.write(encoded)
    stream.truncate()
    stream.flush()
    os.fsync(stream.fileno())


def main(argv=None, *, runtime_factory=load_runtime, monotonic=time.monotonic):
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=60)
    ap.add_argument("--motion-wait", type=float, default=90)
    ap.add_argument("--world", default="limo_maze")
    ap.add_argument("--model", default="limo")
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args(argv)
    for value in (args.duration, args.motion_wait):
        if not math.isfinite(value) or not 0 < value <= 300:
            ap.error("duration and motion-wait must be finite and within (0, 300] seconds")
    if args.output.exists():
        ap.error("output already exists; choose a new evidence filename")
    topic = "/world/" + args.world + "/pose/info"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve before imports/init. Repeated SIGINT/TERM requests only set a flag.
    stream = args.output.open("x+", encoding="utf-8")
    stop = {"signal": None}
    previous_handlers = {}

    def request_stop(signum, _frame):
        if stop["signal"] is None:
            stop["signal"] = signal.Signals(signum).name

    start, motion_start, ready = monotonic(), None, False
    rt = ros = node = buffer = listener = gz = None
    initialized = subscribed = False
    gz_samples, odom_samples = [], []
    lock = threading.Lock()
    state = {"accepting": True, "callback_error": False}
    report = {"schema_version": 1, "read_only": True,
              "truth_used_only_for_offline_diagnosis": True,
              "world_topic": topic, "world_entity": args.model,
              "motion_detected": False, "wall_seconds": 0.0,
              "requested_seconds_after_motion": args.duration,
              "partial": True, "reason": "initializing", "error": None, "errors": [],
              "requested_signal": None, "automatic_pass_verdict": False,
              "world_samples": [], "odom_samples": [], "aligned_samples": [],
              "world_summary": {"count": 0}, "odom_summary": {"count": 0}, "aligned_count": 0,
              "cleanup": {"status": "pending", "native_unsubscribed": False,
                          "native_release_attempted": False, "ros_shutdown_called": False}}

    def fault(stage, exc):
        item = {"stage": stage, "error": f"{type(exc).__name__}: {exc}"}
        report["errors"].append(item)
        if report["error"] is None:
            report["error"] = item["error"]
        report["partial"] = True

    def callback_fault(stage, exc):
        with lock:
            if state["accepting"]:
                state["callback_error"] = True
                fault(stage, exc)

    def gz_cb(msg):
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
            for p in msg.pose:
                if p.name != args.model:
                    continue
                sample = {"sim_s": stamp, "yaw": yaw(p.orientation), "x": p.position.x,
                          "y": p.position.y, "z": p.position.z, "id": p.id}
                if not all(math.isfinite(value) for value in sample.values()):
                    raise ValueError("Nonfinite native model pose")
                with lock:
                    if not state["accepting"]:
                        return
                    if not gz_samples or stamp - gz_samples[-1]["sim_s"] >= .019:
                        gz_samples.append(sample)
                return
        except Exception as exc:
            callback_fault("native_callback", exc)

    def odom_cb(msg):
        try:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p = msg.pose.pose
            sample = {"sim_s": stamp, "yaw": yaw(p.orientation), "x": p.position.x,
                      "y": p.position.y, "angular_velocity": msg.twist.twist.angular.z,
                      "linear_velocity": math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)}
            if not all(math.isfinite(value) for value in sample.values()):
                raise ValueError("Nonfinite odometry sample")
            with lock:
                if not state["accepting"]:
                    return
                if not odom_samples or stamp - odom_samples[-1]["sim_s"] >= .019:
                    odom_samples.append(sample)
        except Exception as exc:
            callback_fault("odom_callback", exc)

    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.getsignal(signum)
            signal.signal(signum, request_stop)
        persist(stream, report)
        rt = runtime_factory()
        ros = rt.rclpy
        ros.init(signal_handler_options=rt.SignalHandlerOptions.NO)
        initialized = True
        node = ros.create_node("offline_world_odom_diagnostic",
                               parameter_overrides=[rt.Parameter("use_sim_time", value=True)])
        buffer = rt.Buffer(cache_time=rt.Duration(seconds=args.duration+args.motion_wait+10), node=node)
        listener = rt.TransformListener(buffer, node)
        node.create_subscription(rt.Odometry, "/odom", odom_cb, rt.qos_profile_sensor_data)
        gz = rt.GzNode()
        if not gz.subscribe(rt.Pose_V, topic, gz_cb):
            raise RuntimeError("Could not subscribe to native Gazebo model poses")
        subscribed = True
        # Keep the original waiting/measurement window origin after setup.
        start = monotonic()
        report["reason"] = "ros_context_stopped"
        while ros.ok() and stop["signal"] is None:
            ros.spin_once(node, timeout_sec=.05)
            elapsed = monotonic() - start
            with lock:
                callback_error = state["callback_error"]
                have_both = bool(gz_samples and odom_samples)
                latest_odom = odom_samples[-1] if odom_samples else None
            if callback_error:
                report["reason"] = "sampling_error"
                break
            if stop["signal"] is not None:
                break
            if have_both and not ready:
                print("READY: native Gazebo limo world pose and ROS odom received; waiting for controlled motion; no commands published.", flush=True)
                ready = True
            if motion_start is None and latest_odom and (abs(latest_odom["angular_velocity"]) > .05
                                                        or latest_odom["linear_velocity"] > .01):
                motion_start = monotonic()
                print(f"MOTION DETECTED: collecting {args.duration:g} wall seconds from now.", flush=True)
            if motion_start is not None and monotonic() - motion_start >= args.duration:
                report["reason"] = "duration_complete"
                break
            if motion_start is None and elapsed >= args.motion_wait:
                report["reason"] = "motion_wait_timeout"
                break
        if stop["signal"] is not None and report["reason"] != "sampling_error":
            report["reason"] = "signal_interrupt"
    except KeyboardInterrupt as exc:
        report["reason"] = "keyboard_interrupt"
        fault("sampling", exc)
    except Exception as exc:
        report["reason"] = "sampling_error"
        fault("sampling", exc)
    finally:
        # Freeze and persist BEFORE any native unsubscribe/destructor. A native
        # abort is not a catchable Python exception; leave usable raw evidence.
        with lock:
            state["accepting"] = False
            world, odom = list(gz_samples), list(odom_samples)
        report.update(world_samples=world, odom_samples=odom,
                      world_summary=summarize(world), odom_summary=summarize(odom),
                      motion_detected=motion_start is not None,
                      wall_seconds=monotonic()-start, requested_signal=stop["signal"])
        # Collection can finish before cleanup. Checkpoints remain partial until
        # both have finished, so an abrupt native exit cannot leave a full report.
        report["collection_complete"] = (report["reason"] == "duration_complete"
                                         and not report["errors"] and bool(world) and bool(odom))
        report["partial"] = True
        report["cleanup"]["status"] = "raw_snapshot_saved_before_native_cleanup"
        try:
            persist(stream, report)
            if gz is not None and subscribed:
                try:
                    if gz.unsubscribe(topic) is False:
                        raise RuntimeError("Native unsubscribe returned false")
                    report["cleanup"]["native_unsubscribed"] = True
                except Exception as exc:
                    fault("native_unsubscribe", exc)
            if buffer is not None:
                odom_times = [sample["sim_s"] for sample in odom]
                try:
                    for sample in world:
                        index = bisect.bisect_left(odom_times, sample["sim_s"])
                        choices = [i for i in (index-1, index) if 0 <= i < len(odom_times)]
                        if not choices:
                            continue
                        closest = min(choices, key=lambda i: abs(odom_times[i]-sample["sim_s"]))
                        if abs(odom_times[closest]-sample["sim_s"]) > .04:
                            continue
                        record = {"sim_s": sample["sim_s"], "world_yaw": sample["yaw"],
                                  "odom_yaw": odom[closest]["yaw"], "odom_dt_s": odom_times[closest]-sample["sim_s"]}
                        for target, source, field in (("map", "base_link", "map_base_yaw"), ("map", "odom", "map_odom_yaw")):
                            try:
                                tf = buffer.lookup_transform(target, source, rt.Time(nanoseconds=round(sample["sim_s"]*1e9)))
                                record[field] = yaw(tf.transform.rotation)
                                if not math.isfinite(record[field]):
                                    raise ValueError("Nonfinite aligned TF yaw")
                            except rt.TransformException:
                                record[field] = None
                        report["aligned_samples"].append(record)
                except Exception as exc:
                    fault("alignment", exc)
            report["aligned_count"] = len(report["aligned_samples"])
            report["cleanup"]["status"] = "aligned_snapshot_saved_before_native_release"
            persist(stream, report)
        finally:
            # Release native callbacks while Python and ROS are still alive,
            # not later during interpreter finalization.
            if gz is not None:
                report["cleanup"]["native_release_attempted"] = True
                try:
                    gz = None
                    gc.collect()
                except Exception as exc:
                    fault("native_release", exc)
            for stage, cleanup in (
                    ("tf_listener", lambda: listener.unregister() if listener is not None else None),
                    ("ros_node", lambda: node.destroy_node() if node is not None else None),
                    ("ros_context", lambda: ros.shutdown() if initialized and ros.ok() else None)):
                try:
                    if stage == "ros_context":
                        report["cleanup"]["ros_shutdown_called"] = bool(initialized and ros.ok())
                    cleanup()
                except Exception as exc:
                    fault(stage, exc)
            report["cleanup"]["status"] = "errors" if any(e["stage"] in (
                "native_unsubscribe", "native_release", "tf_listener", "ros_node", "ros_context") for e in report["errors"]) else "complete"
            report["partial"] = not report["collection_complete"] or bool(report["errors"])
            try:
                persist(stream, report)
            finally:
                stream.close()
                for signum, previous in previous_handlers.items():
                    signal.signal(signum, previous)
    print(json.dumps({k:v for k,v in report.items() if not k.endswith("samples")}, indent=2), flush=True)
    return 2 if report["partial"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
