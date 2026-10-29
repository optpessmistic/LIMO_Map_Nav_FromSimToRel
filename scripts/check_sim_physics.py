#!/usr/bin/env python3
"""Audit the installed model; --execute adds one isolated native-Gazebo test.

No ROS node, bridge, world-pose topic relay, or navigation controller is used.
Truth is recorded only for offline validation. Each invocation owns its server.
"""
import argparse
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid
import xml.etree.ElementTree as ET


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def summarize(samples):
    if not samples:
        return {"count": 0}
    return {
        "count": len(samples), "first": samples[0], "last": samples[-1],
        "yaw_delta_rad": sum(wrap(b["yaw"] - a["yaw"])
                             for a, b in zip(samples, samples[1:])),
        "max_displacement_m": max(math.hypot(s["x"] - samples[0]["x"],
                                             s["y"] - samples[0]["y"])
                                   for s in samples),
    }


def save(path, value):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def prepare(out):
    from ament_index_python.packages import get_package_share_directory
    share = Path(get_package_share_directory("limo_harmonic_description"))
    source = share / "urdf/limo_gazebo.xacro"
    urdf = subprocess.check_output(["xacro", str(source)], text=True, timeout=20)
    (out / "model.urdf").write_text(urdf, encoding="utf-8")
    sdf = subprocess.check_output(["gz", "sdf", "-p", str(out / "model.urdf")],
                                  text=True, timeout=20)
    (out / "model.sdf").write_text(sdf, encoding="utf-8")
    model = ET.fromstring(sdf).find("model")
    assert model is not None and model.get("name") == "limo"
    wheels = [link for link in model.findall("link")
              if link.get("name", "").endswith("wheel_link")]
    assert len(wheels) == 4, "Expected four wheel links"
    for wheel in wheels:
        collision = wheel.find("collision")
        assert collision is not None
        radius = collision.findtext("geometry/sphere/radius")
        assert radius is not None and math.isclose(float(radius), .045)
        friction = collision.find("surface/friction/ode")
        assert friction is not None
        assert [float(x) for x in friction.findtext("fdir1").split()] == [0, 0, 1]
        assert math.isclose(float(friction.findtext("slip1")), .035)
        assert float(friction.findtext("slip2")) == 0
        assert float(friction.findtext("mu")) == float(friction.findtext("mu2")) == 10
    drive = model.find("plugin[@name='gz::sim::systems::DiffDrive']")
    assert drive is not None and math.isclose(float(drive.findtext("wheel_separation")), .175)
    urdf_model = ET.fromstring(urdf)
    centers = []
    for wheel in wheels:
        name = wheel.get("name")
        origin = urdf_model.find(f"joint[@name='{name.removesuffix('_link')}']/origin")
        collision = urdf_model.find(f"link[@name='{name}']/collision/origin")
        x, y, _ = map(float, origin.get("xyz").split())
        roll, pitch, heading = map(float, origin.get("rpy").split())
        dx, dy, dz = map(float, collision.get("xyz").split())
        assert pitch == heading == 0, "Envelope derivation expects a roll-only wheel joint"
        centers.append((x + dx, y + math.cos(roll) * dy - math.sin(roll) * dz))
        assert math.isclose(float(collision.get("rpy").split()[0]), math.pi / 2, abs_tol=1e-12)
    half_x = max(abs(x) for x, _ in centers) + .045
    half_y = max(abs(y) for _, y in centers) + .045
    separation = max(y for _, y in centers) - min(y for _, y in centers)
    assert math.isclose(float(drive.findtext("wheel_separation")), separation)
    for name in ("front_right_wheel", "rear_right_wheel"):
        roll = float(urdf_model.find(f"joint[@name='{name}']/origin").get("rpy").split()[0])
        assert math.isclose(roll, math.pi, abs_tol=1e-12)
    project = Path(__file__).resolve().parents[1]
    import yaml
    nav = yaml.safe_load((project / "src/limo_maze_bringup/config/nav2.yaml").read_text())
    padded = []
    for costmap in ("local_costmap", "global_costmap"):
        config = nav[costmap][costmap]["ros__parameters"]
        footprint = ast.literal_eval(config["footprint"])
        pad = float(config["footprint_padding"])
        limits = [max(abs(p[0]) for p in footprint) + pad,
                  max(abs(p[1]) for p in footprint) + pad]
        assert limits[0] >= half_x and limits[1] >= half_y, "Padded footprint does not cover wheel collision envelope"
        assert math.hypot(*limits) <= .25, "Current mission safety-radius assumption no longer covers Nav2 footprint"
        padded.append(limits)
    audit = {
        "installed_xacro": str(source),
        "urdf_sha256": hashlib.sha256(urdf.encode()).hexdigest(),
        "sdf_sha256": hashlib.sha256(sdf.encode()).hexdigest(),
        "geometry_checks_passed": True,
        "wheel_contact": "sphere radius 0.045 m, original centers and inertia",
        "wheel_separation_m": separation, "contact_width_m": 2 * half_y,
        "contact_aabb_half_extents_m": [half_x, half_y],
        "contact_aabb_circumradius_m": math.hypot(half_x, half_y),
        "nav2_padded_half_extents_m": padded,
        "mission_safety_radius_assumption_m": .25,
        "calibration": "simulation approximation; not real-LIMO tire calibration",
    }
    # Isolate contact dynamics from CPU rendering. IMU, masses, geometry,
    # friction and joints are preserved. No model pose is injected into ROS.
    for link in model.findall("link"):
        for sensor in list(link.findall("sensor")):
            if sensor.get("type") != "imu":
                link.remove(sensor)
    model.set("canonical_link", "base_footprint")
    world_doc = ET.parse(project / "src/limo_sim_bringup/worlds/empty.sdf")
    world = world_doc.getroot().find("world")
    world.set("name", "physics_diag")
    for plugin in list(world.findall("plugin")):
        if "Sensors" in plugin.get("name", ""):
            world.remove(plugin)
    world.append(model)
    ET.indent(world_doc)
    world_doc.write(out / "world.sdf", encoding="utf-8", xml_declaration=True)
    save(out / "geometry.json", audit)
    return audit


def measure(case, out):
    # Set by main before importing native transport. Every run has a UUID suffix.
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.model_pb2 import Model
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.twist_pb2 import Twist
    node = Node()
    data = {key: [] for key in ("world", "odom", "joints", "imu", "command_echo")}
    fresh = {}
    stage = "waiting"

    def pose_sample(kind, pose, stamp):
        fresh[kind] = time.monotonic()
        if data[kind] and stamp - data[kind][-1]["sim_s"] < .019:
            return
        data[kind].append({"sim_s": stamp, "stage": stage,
                           "x": pose.position.x, "y": pose.position.y,
                           "z": pose.position.z, "yaw": yaw(pose.orientation)})

    def stamp(msg):
        return msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9

    def world_cb(msg):
        for pose in msg.pose:
            if pose.name == "limo":
                pose_sample("world", pose, stamp(msg))

    def odom_cb(msg):
        pose_sample("odom", msg.pose, stamp(msg))

    def joint_cb(msg):
        fresh["joints"] = time.monotonic()
        data["joints"].append({"sim_s": stamp(msg), "stage": stage,
                               "positions": {j.name: j.axis1.position for j in msg.joint}})

    def imu_cb(msg):
        fresh["imu"] = time.monotonic()
        data["imu"].append({"sim_s": stamp(msg), "stage": stage,
                            "yaw": yaw(msg.orientation), "wz": msg.angular_velocity.z})

    def echo_cb(msg):
        data["command_echo"].append({"wall_s": time.monotonic(),
                                     "linear_x": msg.linear.x, "angular_z": msg.angular.z})

    node.subscribe(Pose_V, "/world/physics_diag/pose/info", world_cb)
    node.subscribe(Odometry, "/limo/odometry", odom_cb)
    node.subscribe(Model, "/limo/joint_states", joint_cb)
    node.subscribe(IMU, "/limo/imu", imu_cb)
    node.subscribe(Twist, "/cmd_vel", echo_cb)
    publisher = node.advertise("/cmd_vel", Twist)
    zero, command = Twist(), Twist()
    if case == "forward":
        command.linear.x = .05
    else:
        command.angular.z = -.15 if case == "reverse_spin" else .15
    duration = 60 if case == "full_turn" else 8
    error = None
    began = time.monotonic()
    try:
        while time.monotonic() - began < 15 and not all(data[k] for k in fresh_keys):
            publisher.publish(zero)
            time.sleep(.05)
        assert all(data[k] for k in fresh_keys), "Required native observations missing"
        stage = "settle"
        began = time.monotonic()
        while time.monotonic() - began < 2:
            publisher.publish(zero)
            time.sleep(.05)
        stage = "command"
        began = time.monotonic()
        turn_start = len(data["odom"]) - 1
        while time.monotonic() - began < duration:
            assert all(time.monotonic() - fresh[k] < 1 for k in fresh_keys), "Observation freshness lost"
            if case == "full_turn" and summarize(data["odom"][turn_start:])["yaw_delta_rad"] >= 2 * math.pi + .1:
                break
            publisher.publish(command)
            time.sleep(.05)
    except Exception as exc:
        error = str(exc)
    finally:
        stage = "stop"
        began = time.monotonic()
        while time.monotonic() - began < 2:
            publisher.publish(zero)
            time.sleep(.05)
    world, odom = summarize(data["world"]), summarize(data["odom"])
    imu = data["imu"]
    integral = sum(.5 * (a["wz"] + b["wz"]) * (b["sim_s"] - a["sim_s"])
                   for a, b in zip(imu, imu[1:]))
    stop = data["world"]
    tail = [p for p in stop if p["stage"] == "stop" and p["sim_s"] >= stop[-1]["sim_s"] - .5] if stop else []
    stopped = summarize(tail)
    dt = tail[-1]["sim_s"] - tail[0]["sim_s"] if len(tail) > 1 else 0
    stop_linear = stopped.get("max_displacement_m", math.inf) / dt if dt else None
    stop_angular = abs(stopped.get("yaw_delta_rad", math.inf)) / dt if dt else None
    checks = {"observations_fresh": error is None and all(
                  k in fresh and time.monotonic() - fresh[k] < 1 for k in fresh_keys),
              "enough_native_samples": all(len(data[k]) >= 20 for k in fresh_keys),
              "zero_echo_at_end": len(data["command_echo"]) >= 10 and all(
                  c["linear_x"] == 0 and c["angular_z"] == 0 for c in data["command_echo"][-10:]),
              "world_stop_observed": dt >= .3 and stop_linear < .001 and stop_angular < .005}
    if world["count"] and odom["count"]:
        checks["imu_agrees_with_native_yaw"] = abs(integral - world["yaw_delta_rad"]) < .03
        if case == "forward":
            def forward_projection(value):
                a, b = value["first"], value["last"]
                return ((b["x"] - a["x"]) * math.cos(a["yaw"])
                        + (b["y"] - a["y"]) * math.sin(a["yaw"]))
            world_forward, odom_forward = forward_projection(world), forward_projection(odom)
            checks["forward_motion"] = world_forward > .2 and odom_forward > .2
            checks["translation_agreement"] = abs(world_forward - odom_forward) < .01
            checks["no_uncommanded_turn"] = abs(world["yaw_delta_rad"]) < .01
        else:
            direction = -1 if case == "reverse_spin" else 1
            checks["native_rotation"] = direction * world["yaw_delta_rad"] > (2 * math.pi if case == "full_turn" else .5)
            checks["yaw_agreement"] = abs(world["yaw_delta_rad"] - odom["yaw_delta_rad"]) < .02
            checks["rotation_center_stable"] = world["max_displacement_m"] < .005
    result = {"case": case, "partition": os.environ["GZ_PARTITION"],
              "motion_wall_limit_s": duration, "command": {"linear_x": command.linear.x, "angular_z": command.angular.z},
              "error": error, "checks": checks, "passed": all(checks.values()),
              "world_summary": world, "odom_summary": odom,
              "imu_integrated_yaw_rad": integral, "stop_tail_linear_mps": stop_linear,
              "stop_tail_angular_radps": stop_angular, "zero_published_wall_s": 2}
    save(out / "samples.json", data)
    return result


fresh_keys = ("world", "odom", "joints", "imu")


def stop_owned_group(server):
    """Reap descendants even if the original launcher has already exited."""
    for sig, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 8), (signal.SIGKILL, 5)):
        try:
            os.killpg(server.pid, sig)
        except ProcessLookupError:
            server.poll()
            break
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            server.poll()  # Reap the direct child before checking its group.
            try:
                os.killpg(server.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New directory; refuses to overwrite")
    parser.add_argument("--execute", action="store_true", help="Explicitly enable bounded native commands in a new isolated empty world")
    parser.add_argument("--case", choices=("forward", "spin", "reverse_spin", "full_turn"), default="spin")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    out = args.output.resolve()
    prepare(out)
    if not args.execute:
        print(json.dumps({"geometry_passed": True, "motion_executed": False, "output": str(out)}))
        return 0
    os.environ["GZ_PARTITION"] = "limo_physics_check_" + uuid.uuid4().hex
    assert os.environ["GZ_PARTITION"] != "limo_maze_jazzy"
    command = ["gz", "sim", "-r", "-s", "-v", "2", str(out / "world.sdf"), "--force-version", "8"]
    result, process = {}, {}
    with (out / "server.log").open("x") as logfile:
        server = subprocess.Popen(command, stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
        process.update(pid=server.pid, pgid=server.pid, command=command, partition=os.environ["GZ_PARTITION"])
        try:
            time.sleep(.5)
            assert server.poll() is None, "Gazebo exited before measurement"
            result = measure(args.case, out)
        except (Exception, KeyboardInterrupt) as exc:
            result = {"passed": False, "error": f"{type(exc).__name__}: {exc}"}
        finally:
            stop_owned_group(server)
            process["exit_code"] = server.returncode
            remaining = subprocess.run(["ps", "--no-headers", "-g", str(server.pid), "-o", "pid,ppid,args"],
                                       capture_output=True, text=True, timeout=3).stdout.strip()
            process["remaining_owned_group"] = remaining
            save(out / "process.json", process)
    result["server_reaped"] = not process["remaining_owned_group"]
    result["passed"] = result.get("passed", False) and result["server_reaped"]
    save(out / "result.json", result)
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
