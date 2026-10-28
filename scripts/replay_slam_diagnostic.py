#!/usr/bin/env python3
"""Replay recorded ROS scan/odom evidence into an isolated, owned SLAM process.

No Gazebo, Nav2, velocity or action publishers are started. Pure input helpers
can be tested without ROS; runtime imports are intentionally deferred.
"""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def finite_vector(value, length, name):
    result = [float(x) for x in value]
    if len(result) != length or not all(math.isfinite(x) for x in result):
        raise ValueError(f"{name}: expected {length} finite values")
    return result


def quaternion(value):
    q = finite_vector(value, 4, "quaternion")
    norm = math.sqrt(sum(x*x for x in q))
    if abs(norm-1) > 1e-3:
        raise ValueError("Quaternion is not unit length")
    return [x/norm for x in q]


def yaw(q):
    x, y, z, w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def base_from_laser(source, offset):
    """T_odom_base = T_odom_laser inverse(T_base_laser); extrinsic R=I."""
    q = quaternion(source["quaternion"])
    p = finite_vector(source["position"], 3, "sensor position")
    x, y, z, w = q
    a, b, c = offset
    rotated = [(1-2*(y*y+z*z))*a+2*(x*y-z*w)*b+2*(x*z+y*w)*c,
               2*(x*y+z*w)*a+(1-2*(x*x+z*z))*b+2*(y*z-x*w)*c,
               2*(x*z-y*w)*a+2*(y*z+x*w)*b+(1-2*(x*x+y*y))*c]
    return {"position": [u-v for u, v in zip(p, rotated)], "quaternion": q}


def prepare_input(document, runtime, segment="full"):
    """Only ROS measurements are selected; native/world fields are never used."""
    if segment not in ("full", "first_half", "second_half"):
        raise ValueError("Unknown segment")
    if runtime.get("passed") is not True:
        raise ValueError("Runtime metadata must be from a passed interface check")
    limits = runtime["scan"]
    minimum, maximum = float(limits["range_min"]), float(limits["range_max"])
    if not (math.isfinite(minimum) and math.isfinite(maximum) and 0 <= minimum < maximum):
        raise ValueError("Invalid measured LaserScan range limits")
    extrinsic = copy.deepcopy(runtime["tf_chain"]["base_link->laser_link"])
    offset = finite_vector(extrinsic["xyz"], 3, "laser extrinsic")
    q = quaternion(extrinsic["quaternion_xyzw"])
    if max(abs(v) for v in q[:3]) > 1e-9 or abs(abs(q[3])-1) > 1e-9:
        raise ValueError("Only measured identity laser extrinsic rotation is supported")
    extrinsic["xyz"], extrinsic["quaternion_xyzw"] = offset, q
    raw = document.get("raw", document)
    scans = {int(k): v for k, v in raw["ros_scans"].items()}
    if len(scans) != len(raw["ros_scans"]):
        raise ValueError("Duplicate normalized scan timestamps")
    if "transforms" in raw:
        transforms = {int(k): v["odom"] for k, v in raw["transforms"].items() if "odom" in v}
        tf_field = "raw.transforms.odom"
    else:
        transforms = {row["stamp_ns"]: row["tf"]["odom_to_laser_link"]
                      for row in raw["aligned"] if "odom_to_laser_link" in row["tf"]}
        tf_field = "aligned.tf.odom_to_laser_link"
    records, reconstructed, missing_intensities = [], {"range_min": 0, "range_max": 0}, 0
    for ns in sorted(set(scans) & set(transforms)):
        if ns <= 0 or transforms[ns].get("stamp_ns") != ns:
            raise ValueError("Require positive, exact same-stamp odom-to-laser TF")
        scan = copy.deepcopy(scans[ns])
        if scan["frame"] != "laser_link":
            raise ValueError("Require laser_link scans")
        values = finite_vector([scan[k] for k in ("angle_min", "angle_max", "angle_step",
                                                  "time_increment", "scan_time")], 5, "scan metadata")
        if values[2] <= 0 or values[3] != 0 or values[4] < 0:
            raise ValueError("Require positive angle increment and instantaneous time_increment=0")
        scan.update(zip(("angle_min","angle_max","angle_step","time_increment","scan_time"), values))
        ranges = [float(v) for v in scan["ranges"]]
        if not 2 <= len(ranges) <= 10000:
            raise ValueError("Unsupported scan size")
        if abs(values[0]+(len(ranges)-1)*values[2]-values[1]) > 1e-5:
            raise ValueError("Scan angles do not match the range count")
        for name, expected in (("range_min", minimum), ("range_max", maximum)):
            if name not in scan:
                scan[name] = expected
                reconstructed[name] += 1
            if not math.isclose(float(scan[name]), expected, rel_tol=0, abs_tol=1e-6):
                raise ValueError("Recorded and runtime range limits disagree")
            scan[name] = float(scan[name])
        if "intensities" not in scan:
            missing_intensities += 1
        scan["intensities"] = [float(v) for v in scan.get("intensities", [])]
        if len(scan["intensities"]) not in (0, len(ranges)):
            raise ValueError("Intensity array must be empty or match the range count")
        scan["ranges"] = ranges  # Preserve original finite / NaN / infinity measurements.
        base = base_from_laser(transforms[ns], offset)
        records.append({"stamp_ns": ns, "scan": scan, "base": base})
    if len(records) < 3:
        raise ValueError("Insufficient same-stamp ROS scan/TF pairs")
    moving = [b["stamp_ns"] for a, b in zip(records, records[1:])
              if abs(math.remainder(yaw(b["base"]["quaternion"])-yaw(a["base"]["quaternion"]),
                                    2*math.pi))/((b["stamp_ns"]-a["stamp_ns"])*1e-9) > .05]
    if not moving:
        raise ValueError("No recorded angular motion; this diagnostic expects a rotation segment")
    midpoint = (moving[0]+moving[-1])//2
    selected = [r for r in records if moving[0]-1_000_000_000 <= r["stamp_ns"] <= moving[-1]+1_000_000_000
                and (segment != "first_half" or r["stamp_ns"] <= midpoint)
                and (segment != "second_half" or r["stamp_ns"] > midpoint)]
    if len(selected) < 2 or len(selected) > 600 or (selected[-1]["stamp_ns"]-selected[0]["stamp_ns"])*1e-9 > 40:
        raise ValueError("Require a segment of 2..600 scans spanning at most 40 seconds")
    if any(b["stamp_ns"]-a["stamp_ns"] > 500_000_000 for a, b in zip(selected, selected[1:])):
        raise ValueError("Selected evidence has a scan/TF gap larger than 0.5 seconds")
    return selected, extrinsic, {"input_fields": ["ros_scans", tf_field],
        "unpaired_ros_scan_count": len(set(scans)-set(transforms)), "same_stamp_pair_count": len(records),
        "reconstructed_range_fields": reconstructed,
        "missing_intensities_count": missing_intensities,
        "missing_intensities_policy": "Empty optional array when absent; Karto consumes ranges, not intensities",
        "range_metadata_source": "runtime-evidence.scan; missing original fields are explicitly reconstructed",
        "range_min": minimum, "range_max": maximum, "base_link_to_laser_link": extrinsic}


def configure(document, overrides):
    if len(overrides) > 1:
        raise ValueError("At most one scalar --override is allowed; no parameter sweeps")
    settings = copy.deepcopy(document)
    params = settings["slam_toolbox"]["ros__parameters"]
    fixed = {"use_sim_time": True, "base_frame": "base_link", "odom_frame": "odom",
             "map_frame": "map", "scan_topic": "/scan"}
    if any(params.get(k) != v for k, v in fixed.items()):
        raise ValueError("YAML must use sim time and the fixed base_link/odom/map/scan replay frames")
    for item in overrides:
        key, text = item.split("=", 1)
        if key not in params or key in fixed:
            raise ValueError("Unknown or fixed replay parameter: " + key)
        value, old = json.loads(text), params[key]
        if isinstance(old, float) and isinstance(value, (int, float)) and not isinstance(value, bool):
            value = float(value)
        if type(value) is not type(old) or not isinstance(value, (bool, int, float, str)):
            raise ValueError("Override must retain the existing scalar parameter type")
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError("Override must be finite")
        params[key] = value
    json.dumps(settings, allow_nan=False)
    return settings


def check_environment(environment):
    if environment.get("ROS_DOMAIN_ID") != "75" or environment.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST":
        raise ValueError("Requires ROS_DOMAIN_ID=75 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST")


def group_members(pgid):
    result = subprocess.run(["ps", "-eo", "pid=,pgid=,stat="], capture_output=True, text=True,
                            timeout=2, check=True)
    return [{"pid": int(p), "state": state} for p, group, state in
            (line.split() for line in result.stdout.splitlines()) if int(group) == pgid]


def stop_owned_group(process, waits=(4, 2, 1)):
    """Do not trust launcher exit alone: its owned process group must be empty."""
    evidence = {"pgid": process.pid, "signals": [], "errors": []}
    def inspect():
        try:
            return group_members(process.pid)
        except Exception as exc:
            evidence["errors"].append(str(exc))
            return None  # Inspection failure is not proof that the group exited.
    for sig, wait in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), waits):
        try:
            if inspect() == []:
                break
            os.killpg(process.pid, sig)
            evidence["signals"].append(sig.name)
        except ProcessLookupError:
            break
        except Exception as exc:
            evidence["errors"].append(str(exc))
        deadline = time.monotonic()+wait
        while time.monotonic() < deadline:
            process.poll()  # Reap our launcher, including when it has already exited.
            if inspect() == []:
                break
            time.sleep(.05)
    evidence["launch_exit_code"] = process.poll()
    evidence["remaining_group_members"] = inspect()
    evidence["confirmed_empty"] = evidence["remaining_group_members"] == []
    return evidence


def file_identity(path):
    content = path.read_bytes()
    return {"path": str(path.resolve()), "sha256": hashlib.sha256(content).hexdigest(), "bytes": len(content)}


def selected_installation():
    evidence = {"command": ["ros2", "pkg", "prefix", "slam_toolbox"], "status": "unavailable"}
    try:
        result = subprocess.run(evidence["command"], capture_output=True, text=True, timeout=10, check=True)
        prefix = Path(result.stdout.strip())
        evidence.update(prefix=str(prefix), executable=file_identity(prefix/"lib/slam_toolbox/async_slam_toolbox_node"),
                        status="selected_path_recorded")
    except Exception as exc:
        evidence["error"] = str(exc)
    return evidence


def loaded_libraries(pgid):
    """Sample owned process /proc mappings, not merely the current overlay path."""
    evidence = {"status": "unavailable", "processes": [], "errors": [],
        "limitation": "Library hashes sample filesystem bytes after checking mapped inode/device; not an in-memory code snapshot."}
    try:
        for member in group_members(pgid):
            directory = Path("/proc")/str(member["pid"])
            try:
                executable = directory/"exe"
                if executable.resolve().name != "async_slam_toolbox_node":
                    continue
                process = {"pid": member["pid"], "executable": file_identity(executable), "libraries": []}
                seen = set()
                for line in (directory/"maps").read_text().splitlines():
                    fields = line.split(maxsplit=5)
                    if len(fields) < 6 or not any(name in fields[5] for name in
                                                  ("libtoolbox_common", "libkarto", "libasync_slam_toolbox")):
                        continue
                    mapped = fields[5]
                    if mapped in seen: continue
                    seen.add(mapped)
                    item = {"mapped_path": mapped, "mapping_evidence": line, "hash_status": "unavailable"}
                    try:
                        path = Path(mapped)
                        stat = path.stat()
                        major, minor = [int(v, 16) for v in fields[3].split(":")]
                        if stat.st_ino != int(fields[4]) or (os.major(stat.st_dev), os.minor(stat.st_dev)) != (major, minor):
                            raise RuntimeError("Mapped inode/device differs from current path")
                        identity = file_identity(path)
                        after = path.stat()
                        if (stat.st_ino, stat.st_size, stat.st_mtime_ns) != (after.st_ino, after.st_size, after.st_mtime_ns):
                            raise RuntimeError("Library changed while hashing")
                        item.update(identity, hash_status="mapped_inode_matches_sampled_file")
                    except Exception as exc:
                        item["error"] = str(exc)
                    process["libraries"].append(item)
                evidence["processes"].append(process)
            except Exception as exc:
                evidence["errors"].append(f"pid {member['pid']}: {exc}")
        required = ("libtoolbox_common", "libkarto")
        libraries = evidence["processes"][0]["libraries"] if len(evidence["processes"]) == 1 else []
        evidence["status"] = "mapped_files_recorded" if all(any(name in x["mapped_path"] and
            x["hash_status"] == "mapped_inode_matches_sampled_file" for x in libraries) for name in required) else "partial"
    except Exception as exc:
        evidence["errors"].append(str(exc))
    return evidence


def run_replay(samples, extrinsic, config, destination, report):
    import rclpy
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
    from lifecycle_msgs.srv import GetState
    process, node, log = None, None, None
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    began = time.monotonic()
    try:
        node = rclpy.create_node("offline_slam_diagnostic")
        discovery_end = time.monotonic()+2
        while time.monotonic() < discovery_end:
            rclpy.spin_once(node, timeout_sec=.05)
        found = node.get_node_names()
        report["preflight_nodes"] = found
        if found != [node.get_name()]:
            raise RuntimeError("Domain75 has an existing or conflicting node")
        report["selected_installation"] = selected_installation()
        clock_pub = node.create_publisher(Clock, "/clock", 10)
        scan_pub = node.create_publisher(LaserScan, "/scan", 10)
        dynamic, static = TransformBroadcaster(node), StaticTransformBroadcaster(node)
        def stamp(ns):
            return Time(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)
        def tick(ns):
            message = Clock(); message.clock = stamp(ns); clock_pub.publish(message)
        def spin(timeout=.02):
            if time.monotonic()-began > 90:
                raise TimeoutError("Replay exceeded its 90-second wall limit")
            if process is not None and process.poll() is not None:
                raise RuntimeError("Owned SLAM launcher exited during replay")
            rclpy.spin_once(node, timeout_sec=max(0., timeout))
        def on_tf(message):
            for t in message.transforms:
                if t.header.frame_id == "map" and t.child_frame_id == "odom":
                    p, q = t.transform.translation, t.transform.rotation
                    row = {"stamp_ns": t.header.stamp.sec*1_000_000_000+t.header.stamp.nanosec,
                           "position": [p.x,p.y,p.z], "quaternion": [q.x,q.y,q.z,q.w]}
                    if not report["map_odom"] or row != report["map_odom"][-1]:
                        report["map_odom"].append(row)
        def on_pose(message):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            ns = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            if message.header.frame_id != "map" or ns not in report["selected_timestamps_ns"]:
                raise RuntimeError("Unexpected SLAM pose frame or source timestamp")
            report["poses"].append({"stamp_ns": ns, "position": finite_vector([p.x,p.y,p.z],3,"SLAM pose"),
                                    "quaternion": quaternion([q.x,q.y,q.z,q.w])})
        def on_map(message):
            p, q = message.info.origin.position, message.info.origin.orientation
            report["map"] = {"width": message.info.width, "height": message.info.height,
                "resolution": message.info.resolution, "origin": [p.x,p.y,p.z,q.x,q.y,q.z,q.w],
                "data": list(message.data)}
        subscriptions = [node.create_subscription(TFMessage, "/tf", on_tf, 100),
            node.create_subscription(PoseWithCovarianceStamped, "/pose", on_pose, 100),
            node.create_subscription(OccupancyGrid, "/map", on_map,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                           reliability=ReliabilityPolicy.RELIABLE))]
        ext = TransformStamped(); ext.header.frame_id="base_link"; ext.child_frame_id="laser_link"
        ext.header.stamp=stamp(samples[0]["stamp_ns"])
        ext.transform.translation.x,ext.transform.translation.y,ext.transform.translation.z=extrinsic["xyz"]
        ext.transform.rotation.w=1.
        log=(destination/"slam.log").open("w")
        command=["ros2","launch","slam_toolbox","online_async_launch.py","use_sim_time:=true",
                 "autostart:=true","use_lifecycle_manager:=false","slam_params_file:="+str(config.resolve())]
        process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        report["owned_process"]={"pid":process.pid,"pgid":process.pid,"command":command}
        client=node.create_client(GetState,"/slam_toolbox/get_state")
        deadline=time.monotonic()+20; future=None
        while True:
            tick(samples[0]["stamp_ns"]);static.sendTransform(ext);spin(.05)
            if future is None and client.service_is_ready():
                future=client.call_async(GetState.Request())
            if future is not None and future.done():
                if future.result().current_state.id==3 and scan_pub.get_subscription_count()>0:
                    break
                future=None
            if time.monotonic()>deadline:
                raise TimeoutError("Owned SLAM did not activate within 20 seconds")
        report["loaded_library_evidence"] = loaded_libraries(process.pid)
        warmup=time.monotonic()+.5
        while time.monotonic()<warmup:
            tick(samples[0]["stamp_ns"]);static.sendTransform(ext);spin(.02)
        started=time.monotonic()
        for sample in samples:
            ns=sample["stamp_ns"];due=started+(ns-samples[0]["stamp_ns"])*1e-9
            while time.monotonic()<due:
                spin(min(.02,due-time.monotonic()))
            tick(ns)
            t=TransformStamped();t.header.stamp=stamp(ns);t.header.frame_id="odom";t.child_frame_id="base_link"
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z=sample["base"]["position"]
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w=sample["base"]["quaternion"]
            dynamic.sendTransform(t);spin(.005)
            original=sample["scan"];scan=LaserScan();scan.header.stamp=stamp(ns);scan.header.frame_id="laser_link"
            for source,target in (("angle_min","angle_min"),("angle_max","angle_max"),("angle_step","angle_increment"),
                                  ("time_increment","time_increment"),("scan_time","scan_time"),
                                  ("range_min","range_min"),("range_max","range_max"),("ranges","ranges"),
                                  ("intensities","intensities")):
                setattr(scan,target,original[source])
            scan_pub.publish(scan);report["sent_count"]+=1;spin(.005)
        drain=time.monotonic()
        while time.monotonic()-drain<3:
            tick(samples[-1]["stamp_ns"]+int((time.monotonic()-drain)*1e9));spin(.02)
        report["accepted_timestamps_ns"]=[p["stamp_ns"] for p in report["poses"]]
        report["accepted_pose_count"]=len(report["poses"])
        report["final_map_odom"]=report["map_odom"][-1] if report["map_odom"] else None
        report["status"]="complete" if report["poses"] and report.get("map") and report["map_odom"] else "insufficient_output"
    finally:
        # A second Ctrl-C must not interrupt cleanup of our owned subprocess.
        previous=signal.signal(signal.SIGINT,signal.SIG_IGN)
        try:
            if process is not None:
                try:
                    report["process_cleanup"]=stop_owned_group(process)
                except Exception as exc:
                    report["process_cleanup"]={"confirmed_empty":False,"error":str(exc)}
                if not report["process_cleanup"]["confirmed_empty"]:
                    report["status"]="cleanup_failed"
            if log is not None:log.close()
            if node is not None:node.destroy_node()
            if rclpy.ok():rclpy.shutdown()
        finally:
            signal.signal(signal.SIGINT,previous)
            report["wall_seconds"]=time.monotonic()-began


def read_json(path, inputs, key):
    if path.stat().st_size>128*1024*1024:
        raise ValueError("Input exceeds the 128 MiB diagnostic limit")
    content=path.read_bytes()
    inputs[key]={"path":str(path.resolve()),"sha256":hashlib.sha256(content).hexdigest()}
    return json.loads(content)


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input",type=Path,required=True)
    parser.add_argument("--runtime-evidence",type=Path,required=True)
    parser.add_argument("--params",type=Path,required=True)
    parser.add_argument("--output-dir",type=Path,required=True)
    parser.add_argument("--override",action="append",default=[],metavar="NAME=JSON_SCALAR")
    parser.add_argument("--segment",choices=("full","first_half","second_half"),default="full")
    args=parser.parse_args(argv)
    try:
        args.output_dir.mkdir(parents=True,exist_ok=False)
    except FileExistsError:
        print("Refusing to overwrite an existing output directory",file=sys.stderr);return 2
    report={"schema_version":1,"status":"error","inputs":{},"domain":75,
        "world_truth_used_as_input":False,"publishers":["/scan","/tf","/tf_static","/clock"],
        "segment":args.segment,"sent_count":0,"poses":[],"map_odom":[],
        "limitations":["Input hashes identify files; they do not attest capture provenance or new measurements.",
                       "A new SLAM graph is used; original graph history and callback scheduling are not reproduced.",
                       "Completion means replay/output collection completed, not SLAM or navigation success."]}
    try:
        check_environment(os.environ)
        source=read_json(args.input,report["inputs"],"recording")
        runtime=read_json(args.runtime_evidence,report["inputs"],"runtime_evidence")
        samples,extrinsic,metadata=prepare_input(source,runtime,args.segment)
        report["input_contract"]=metadata
        report["selected_timestamps_ns"]=[s["stamp_ns"] for s in samples]
        report["selected_count"]=len(samples)
        import yaml
        text=args.params.read_bytes()
        report["inputs"]["params"]={"path":str(args.params.resolve()),"sha256":hashlib.sha256(text).hexdigest()}
        settings=configure(yaml.safe_load(text),args.override)
        report["params"]=settings["slam_toolbox"]["ros__parameters"]
        report["override"]=args.override
        config=args.output_dir/"params.yaml";config.write_text(yaml.safe_dump(settings),encoding="utf-8")
        run_replay(samples,extrinsic,config,args.output_dir,report)
    except (Exception,KeyboardInterrupt) as exc:
        if report["status"]!="cleanup_failed":report["status"]="error"
        report["error"]=f"{type(exc).__name__}: {exc}"
    finally:
        report["accepted_timestamps_ns"]=[p["stamp_ns"] for p in report["poses"]]
        report["accepted_pose_count"]=len(report["poses"])
        (args.output_dir/"result.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps({k:report.get(k) for k in ("status","error","selected_count","sent_count","accepted_pose_count","process_cleanup")},indent=2))
    return 0 if report["status"]=="complete" else 2


if __name__=="__main__":
    raise SystemExit(main())
