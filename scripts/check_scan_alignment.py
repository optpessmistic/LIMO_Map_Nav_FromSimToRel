#!/usr/bin/env python3
"""Read-only ROS/Gazebo scan alignment diagnostic; never publishes commands.

World truth is used only in this offline report, never relayed to ROS.
Replay requires Python + NumPy only. Live capture additionally uses Jazzy/gz.
"""
import argparse
import bisect
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np


class Unsupported(ValueError):
    pass


def rotation(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise Unsupported("Invalid quaternion")
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def matrix(element):
    if element is not None and element.attrib:
        raise Unsupported("Pose attributes / relative_to / quaternion or degree syntax are unsupported")
    values = list(map(float, (element.text if element is not None else "0 0 0 0 0 0").split()))
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise Unsupported("Expected finite xyz/rpy pose")
    x, y, z, r, p, h = values
    cr, sr, cp, sp, ch, sh = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(h), math.sin(h)
    m = np.eye(4)
    m[:3, :3] = [[ch*cp, ch*sp*sr-sh*cr, ch*sp*cr+sh*sr],
                 [sh*cp, sh*sp*sr+ch*cr, sh*sp*cr-ch*sr], [-sp, cp*sr, cp*cr]]
    m[:3, 3] = [x, y, z]
    return m


def fingerprint(path):
    data = path.read_bytes()
    return {"path": str(path.resolve()), "sha256": hashlib.sha256(data).hexdigest()}


def load_geometry(world_path, model_path, model_name, sensor_name):
    document = ET.parse(world_path).getroot()
    worlds = document.findall("world")
    if len(worlds) != 1:
        raise Unsupported("Require one world in the SDF")
    world = worlds[0]
    if world.findall(".//include") or world.findall(".//frame") or world.findall(".//model/model"):
        raise Unsupported("Includes, explicit frames and nested models are unsupported")
    boxes = []
    for model in world.findall("model"):
        if model.findtext("static", "false").lower() not in ("true", "1"):
            raise Unsupported(f"World contains non-static model {model.get('name')}")
        if model.findall(".//plugin") or model.findall("joint"):
            raise Unsupported("Animated/plugin-controlled or articulated world geometry is unsupported")
        parent = matrix(model.find("pose"))
        for link in model.findall("link"):
            base = parent @ matrix(link.find("pose"))
            shapes = {}
            for kind in ("collision", "visual"):
                shapes[kind] = []
                for item in link.findall(kind):
                    geometry = item.find("geometry")
                    if geometry is None or len(geometry) != 1 or geometry[0].tag != "box":
                        raise Unsupported("Only static box visual/collision geometry is supported (including floor)")
                    if kind == "visual" and float(item.findtext("transparency", "0")) != 0:
                        raise Unsupported("Transparent box visuals are unsupported")
                    if kind == "visual" and item.find("visibility_flags") is not None:
                        raise Unsupported("Explicit visual visibility_flags are unsupported; require default visibility")
                    size = np.asarray(list(map(float, geometry.findtext("box/size", "").split())))
                    if size.shape != (3,) or not np.all(np.isfinite(size)) or np.any(size <= 0):
                        raise Unsupported("Invalid box size")
                    shapes[kind].append((base @ matrix(item.find("pose")), size))
            unmatched = list(shapes["visual"])
            for transform, size in shapes["collision"]:
                match = next((i for i, (vt, vs) in enumerate(unmatched)
                              if np.allclose(vt, transform, atol=1e-9, rtol=0)
                              and np.allclose(vs, size, atol=1e-9, rtol=0)), None)
                if match is None:
                    raise Unsupported("Visual and collision box poses/sizes differ")
                unmatched.pop(match)
                boxes.append((np.linalg.inv(transform), size / 2))
            if unmatched or not shapes["collision"]:
                raise Unsupported("Every world link needs matched visual/collision boxes")
    if not boxes:
        raise Unsupported("No supported world boxes")
    model = ET.parse(model_path).getroot().find("model")
    if model is None or model.get("name") != model_name:
        raise Unsupported("Require expanded stand-alone model SDF with matching model name")
    sensors = [(link, sensor) for link in model.findall("link") for sensor in link.findall("sensor")
               if sensor.get("name") == sensor_name]
    if len(sensors) != 1 or sensors[0][1].get("type") != "gpu_lidar":
        raise Unsupported("Require one named gpu_lidar")
    link, sensor = sensors[0]
    # Do not attempt a general SDFormat semantic-frame resolver. The project
    # expands fixed sensors into the model's canonical link before this step.
    canonical = model.get("canonical_link", model.find("link").get("name"))
    if link.get("name") != canonical:
        raise Unsupported("Sensor must be on the expanded model's canonical link")
    local = matrix(link.find("pose")) @ matrix(sensor.find("pose"))
    vertical = sensor.find("lidar/scan/vertical")
    if vertical is not None and (int(vertical.findtext("samples", "1")) != 1
                                 or float(vertical.findtext("min_angle", "0")) != 0
                                 or float(vertical.findtext("max_angle", "0")) != 0):
        raise Unsupported("Only one-plane zero-elevation scans are supported")
    return boxes, local, world.get("name")


def pose_message(p):
    return {"position": [p.position.x, p.position.y, p.position.z],
            "quaternion": [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]}


def capture(args, world_name):
    import rclpy
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from tf2_ros import Buffer, TransformListener
    from gz.transport13 import Node
    from gz.msgs10.laserscan_pb2 import LaserScan as NativeScan
    from gz.msgs10.pose_v_pb2 import Pose_V
    native, models, scans, transforms = {}, {}, {}, {}
    lock = threading.Lock()
    rclpy.init()
    ros = rclpy.create_node("scan_alignment_readonly")
    buffer = Buffer()
    listener = TransformListener(buffer, ros)
    gz = Node()
    motion_start = None
    model_topic = f"/world/{world_name}/pose/info"

    def native_cb(m):
        stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nsec
        with lock:
            native[stamp] = {"pose_field": pose_message(m.world_pose), "ranges": list(m.ranges),
                "angle_min": m.angle_min, "angle_step": m.angle_step,
                "range_max": m.range_max, "range_min": m.range_min,
                "count": m.count, "vertical_count": m.vertical_count, "frame": m.frame}

    def model_cb(m):
        stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nsec
        with lock:
            for p in m.pose:
                if p.name == args.model:
                    models[stamp] = pose_message(p)

    def scan_cb(m):
        stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        scans[stamp] = {"ranges": list(m.ranges), "angle_min": m.angle_min,
            "angle_max": m.angle_max, "angle_step": m.angle_increment,
            "range_min": m.range_min, "range_max": m.range_max,
            "intensities": list(m.intensities),
            "frame": m.header.frame_id, "scan_time": m.scan_time,
            "time_increment": m.time_increment, "receipt_wall": time.monotonic()}

    def odom_cb(m):
        nonlocal motion_start
        if motion_start is None and (abs(m.twist.twist.angular.z) > .05 or abs(m.twist.twist.linear.x) > .02):
            motion_start = time.monotonic()
            print("MOTION DETECTED; observing only", flush=True)

    subscriptions = []
    start, ready, error = time.monotonic(), False, None
    try:
        if not gz.subscribe(NativeScan, args.native_scan_topic, native_cb) or not gz.subscribe(Pose_V, model_topic, model_cb):
            raise RuntimeError("Native subscription failed")
        subscriptions.extend((ros.create_subscription(LaserScan, args.ros_scan_topic, scan_cb, qos_profile_sensor_data),
                              ros.create_subscription(Odometry, "/odom", odom_cb, qos_profile_sensor_data)))
        while True:
            rclpy.spin_once(ros, timeout_sec=.03)
            for stamp, scan in list(scans.items()):
                if stamp in transforms or time.monotonic() - scan["receipt_wall"] > 3:
                    continue
                record = {}
                # map TF is optional: SLAM absence/failure must not discard
                # otherwise valid odom-to-sensor input evidence.
                for target in (args.odom_frame, args.map_frame):
                    if not target:
                        continue
                    try:
                        m = buffer.lookup_transform(target, scan["frame"], Time(nanoseconds=stamp))
                        t, q = m.transform.translation, m.transform.rotation
                        record[target] = {"position": [t.x, t.y, t.z], "quaternion": [q.x, q.y, q.z, q.w],
                                          "stamp_ns": m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec}
                    except Exception:
                        pass
                if args.odom_frame in record:
                    transforms[stamp] = record
            if not ready and native and models and transforms:
                ready = True
                print("READY: same-stamp native/ROS scan and odom TF; no commands published", flush=True)
            if args.motion_wait == 0 and time.monotonic() - start >= args.duration:
                break
            if args.motion_wait > 0:
                if motion_start is not None and time.monotonic() - motion_start >= args.duration:
                    break
                if motion_start is None and time.monotonic() - start >= args.motion_wait:
                    break
    except (Exception, KeyboardInterrupt) as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        gz.unsubscribe(args.native_scan_topic)
        gz.unsubscribe(model_topic)
        listener.unregister()
        ros.destroy_node()
        rclpy.shutdown()
    return {"native_scans": native, "ros_scans": scans, "native_model_poses": models,
            "transforms": transforms, "capture": {"error": error, "ready": ready,
                "motion_detected": motion_start is not None, "wall_seconds": time.monotonic() - start}}


def interpolate(models, times, stamp):
    i = bisect.bisect_left(times, stamp)
    if i == 0 or i == len(times):
        return None
    a, b = times[i-1], times[i]
    if b-a > 60_000_000:
        return None
    fraction = (stamp-a)/(b-a)
    pa, pb = models[a], models[b]
    qa, qb = np.asarray(pa["quaternion"]), np.asarray(pb["quaternion"])
    qa, qb = qa/np.linalg.norm(qa), qb/np.linalg.norm(qb)
    dot = float(np.dot(qa, qb))
    if dot < 0:
        qb, dot = -qb, -dot
    if dot > .9995:
        q = qa + fraction * (qb-qa)
        q /= np.linalg.norm(q)
    else:
        angle = math.acos(min(1, dot))
        q = (math.sin((1-fraction)*angle)*qa + math.sin(fraction*angle)*qb)/math.sin(angle)
    return (np.asarray(pa["position"]) + fraction*(np.asarray(pb["position"])-pa["position"]),
            rotation(q), b-a, a if stamp-a <= b-stamp else b)


def expected(boxes, origin, directions, maximum):
    result = np.full(directions.shape[1], maximum)
    for inverse, half in boxes:
        o = inverse[:3, :3] @ origin + inverse[:3, 3]
        d = inverse[:3, :3] @ directions
        near, far = np.zeros(len(result)), result.copy()
        for j in range(3):
            active = abs(d[j]) >= 1e-10
            if abs(o[j]) > half[j]:
                far[~active] = -1
            q1 = np.divide(-half[j]-o[j], d[j], out=np.full(len(result), -np.inf), where=active)
            q2 = np.divide(half[j]-o[j], d[j], out=np.full(len(result), np.inf), where=active)
            near, far = np.maximum(near, np.minimum(q1, q2)), np.minimum(far, np.maximum(q1, q2))
        hit = (near <= far) & (far >= 0)
        result[hit] = near[hit]
    return result


def stats(values):
    a = np.asarray(values, dtype=float)
    return None if not len(a) else {"count": len(a), "mean": float(np.mean(a)),
        "rms": float(np.sqrt(np.mean(a*a))), "p95_abs": float(np.percentile(abs(a), 95)),
        "max_abs": float(np.max(abs(a)))}


def heading(r):
    return math.atan2(r[1, 0], r[0, 0])


def wrapped(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def analyze(raw, boxes, local, args):
    native, scans, models = ({int(k): v for k, v in raw[name].items()}
                             for name in ("native_scans", "ros_scans", "native_model_poses"))
    if "transforms" in raw:
        transforms = {int(k): v for k, v in raw["transforms"].items()}
    else:  # Original run5b evidence, preserved without rewriting it.
        transforms = {r["stamp_ns"]: {args.odom_frame: r["tf"]["odom_to_laser_link"]}
                      for r in raw["aligned"] if "odom_to_laser_link" in r["tf"]}
    times = sorted(models)
    rows, skipped, lag_errors, non_edge = [], [], {lag: [] for lag in args.lags_ms}, []
    for stamp in sorted(set(native) & set(scans) & set(transforms)):
        interpolated = interpolate(models, times, stamp)
        if interpolated is None or args.odom_frame not in transforms[stamp]:
            skipped.append({"stamp_ns": stamp, "reason": "missing pose bracket or odom TF"})
            continue
        n, s = native[stamp], scans[stamp]
        if int(n.get("vertical_count", 1)) != 1:
            raise Unsupported("Native scan has more than one vertical layer")
        observed, ros_ranges = np.asarray(n["ranges"], dtype=float), np.asarray(s["ranges"], dtype=float)
        if observed.shape != ros_ranges.shape or observed.ndim != 1 or not len(observed):
            skipped.append({"stamp_ns": stamp, "reason": "range count mismatch"})
            continue
        if s["frame"] != args.laser_frame or float(s.get("time_increment", 0)) != 0:
            raise Unsupported("Expected named laser frame and instantaneous time_increment=0 scans")
        p, r, gap, nearest = interpolated
        actual_rotation, actual_position = r @ local[:3, :3], p + r @ local[:3, 3]
        angles = n["angle_min"] + np.arange(len(observed))*n["angle_step"]
        directions = np.vstack((np.cos(angles), np.sin(angles), np.zeros(len(angles))))
        prediction = expected(boxes, actual_position, actual_rotation @ directions, n["range_max"])
        valid = np.isfinite(observed) & (observed >= n.get("range_min", 0)) & (observed < n["range_max"])
        finite_pair = np.isfinite(observed) & np.isfinite(ros_ranges)
        special_match = np.array_equal(np.isnan(observed), np.isnan(ros_ranges)) and np.array_equal(
            np.isposinf(observed), np.isposinf(ros_ranges)) and np.array_equal(np.isneginf(observed), np.isneginf(ros_ranges))
        tf = transforms[stamp][args.odom_frame]
        nearest_pose = models[nearest]
        nearest_position = np.asarray(nearest_pose["position"]) + rotation(nearest_pose["quaternion"]) @ local[:3, 3]
        row = {"stamp_ns": stamp, "native_ros_finite_ranges_max_difference": float(np.max(
                   abs(observed[finite_pair]-ros_ranges[finite_pair]))) if np.any(finite_pair) else None,
               "native_ros_special_ranges_match": special_match,
               "native_ros_angle_min_difference": n["angle_min"]-s["angle_min"],
               "native_ros_angle_step_difference": n["angle_step"]-s["angle_step"],
               "model_bracket_gap_s": gap*1e-9, "nearest_model_dt_s": (nearest-stamp)*1e-9,
               "interpolation_vs_nearest_sensor_position_m": float(np.linalg.norm(actual_position-nearest_position)),
               "world_minus_odom_sensor_xy_m": (actual_position[:2]-np.asarray(tf["position"])[:2]).tolist(),
               "world_minus_odom_sensor_yaw_rad": wrapped(heading(actual_rotation)-heading(rotation(tf["quaternion"]))),
               "ray_error_m": stats(observed[valid]-prediction[valid]), "tf": transforms[stamp]}
        before, after = interpolate(models, times, stamp-20_000_000), interpolate(models, times, stamp+20_000_000)
        rate = wrapped(heading(after[1])-heading(before[1]))/.04 if before and after else 0
        row["native_yaw_rate_radps"] = rate
        if abs(rate) > .05:
            # Preserve all-beam metrics. Edge exclusion is an additional named
            # diagnostic, never a replacement for inconvenient residuals.
            edge = np.zeros(len(observed), dtype=bool)
            jumps = abs(np.diff(prediction)) > .1
            edge[:-1] |= jumps
            edge[1:] |= jumps
            non_edge.extend((observed[valid & ~edge]-prediction[valid & ~edge]).tolist())
            row["ray_error_by_pose_offset_ms"] = {}
            for lag in args.lags_ms:
                alternative = interpolate(models, times, stamp+round(lag*1_000_000))
                if alternative is None:
                    continue
                ap, ar = alternative[:2]
                predicted = expected(boxes, ap+ar@local[:3, 3], ar@local[:3, :3]@directions, n["range_max"])
                errors = observed[valid]-predicted[valid]
                row["ray_error_by_pose_offset_ms"][lag] = stats(errors)
                lag_errors[lag].extend(errors.tolist())
        rows.append(row)
    summary = {"aligned_scan_count": len(rows), "moving_scan_count": sum(abs(x["native_yaw_rate_radps"]) > .05 for x in rows),
        "world_box_count": len(boxes), "ray_error_by_pose_offset_ms": {k: stats(v) for k, v in lag_errors.items()},
        "moving_non_edge_error_m": stats(non_edge), "edge_definition": "adjacent predicted ranges differ by >0.1m; mark both beams",
        "same_stamp_world_odom_yaw_difference_rad": stats([x["world_minus_odom_sensor_yaw_rad"] for x in rows]),
        "nearest_model_dt_s": stats([x["nearest_model_dt_s"] for x in rows]),
        "interpolated_vs_nearest_sensor_position_m": stats([x["interpolation_vs_nearest_sensor_position_m"] for x in rows])}
    return summary, rows, skipped


def json_safe(value):
    # Preserve non-finite raw ranges without emitting non-standard JSON numbers.
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--model-sdf", type=Path, required=True, help="Expanded SDF of the actual spawned model")
    parser.add_argument("--model", default="limo")
    parser.add_argument("--sensor-name", default="laser_sensor")
    parser.add_argument("--output", type=Path, required=True, help="New JSON file; refuses overwrite")
    parser.add_argument("--replay", type=Path, help="Replay a previous report; opens no ROS/Gazebo connections")
    parser.add_argument("--duration", type=float, default=15, help="Wall seconds of live observation")
    parser.add_argument("--motion-wait", type=float, default=0, help="Optional wait for externally commanded motion; 0 starts immediately")
    parser.add_argument("--lags-ms", default="-100,-50,-20,-5,-1,0,1,5,20,50,100")
    parser.add_argument("--native-scan-topic", default="/limo/scan")
    parser.add_argument("--ros-scan-topic", default="/scan")
    parser.add_argument("--laser-frame", default="laser_link")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--map-frame", default="map", help="Optional auxiliary TF; empty disables it")
    args = parser.parse_args()
    args.lags_ms = [float(v) for v in args.lags_ms.split(",")]
    if not (1 <= args.duration <= 180 and 0 <= args.motion_wait <= 180 and 0 in args.lags_ms
            and 1 <= len(args.lags_ms) <= 21 and all(math.isfinite(v) and abs(v) <= 500 for v in args.lags_ms)):
        parser.error("Duration 1..180s, motion wait 0..180s, and <=21 finite lags within ±500ms including 0 required")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        report = {"schema_version": 1, "read_only": True, "truth_only_for_offline_diagnosis": True,
            "status": "error", "limitations": ["Input-file hashes are not an attestation of live spawned geometry.",
                "Assumes the supplied world contains all external static obstacles; robot self-returns are not modeled.",
                "World/odom pose differences assume their origins coincide; no fitted alignment is applied.",
                "Metrics diagnose alignment, not SLAM or navigation success."]}
        try:
            report["inputs"] = {"world": fingerprint(args.world), "model_sdf": fingerprint(args.model_sdf)}
            boxes, local, world_name = load_geometry(args.world, args.model_sdf, args.model, args.sensor_name)
            report["geometry"] = {"contract": "static matched visual/collision boxes", "world": world_name,
                                   "model": args.model, "sensor": args.sensor_name, "sensor_model_matrix": local.tolist()}
            report["configuration"] = {"lags_ms": args.lags_ms, "laser_frame": args.laser_frame,
                "odom_frame": args.odom_frame, "gz_partition": os.environ.get("GZ_PARTITION"),
                "ros_domain_id": os.environ.get("ROS_DOMAIN_ID")}
            if args.replay:
                report["inputs"]["replay"] = fingerprint(args.replay)
                previous = json.loads(args.replay.read_text(encoding="utf-8"))
                provenance = previous.get("inputs")
                if provenance:
                    for field in ("world", "model_sdf"):
                        if provenance[field]["sha256"] != report["inputs"][field]["sha256"]:
                            raise Unsupported("Replay input geometry hash mismatch")
                report["geometry_provenance"] = previous.get("geometry_provenance", "legacy_unverified")
                if report["geometry_provenance"] != "input_hashes_recorded_at_capture":
                    report["limitations"].append("Legacy replay lacks capture-time world/model hashes; supplied geometry provenance is unverified.")
                raw = previous.get("raw", previous)
                report["mode"] = "offline_replay"
            else:
                raw = capture(args, world_name)
                report["mode"] = "live_readonly"
                report["geometry_provenance"] = "input_hashes_recorded_at_capture"
            report["raw"] = raw
            report["summary"], report["aligned"], report["skipped"] = analyze(raw, boxes, local, args)
            report["status"] = "complete" if report["summary"]["aligned_scan_count"] else "insufficient_data"
            if raw.get("capture", raw.get("summary", {})).get("error"):
                report["status"] = "capture_error"
        except Unsupported as exc:
            report["status"], report["error"] = "unsupported", str(exc)
        except (Exception, KeyboardInterrupt) as exc:
            report["error"] = f"{type(exc).__name__}: {exc}"
        json.dump(json_safe(report), stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps({k: report[k] for k in ("status", "error", "summary") if k in report}, indent=2))
    return 0 if report["status"] == "complete" else 2


if __name__ == "__main__":
    raise SystemExit(main())
