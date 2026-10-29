#!/usr/bin/env python3
"""Read-only inventory; never starts a driver, sends velocity, or changes config.

Python 3.6+ for old Jetson images. Run after sourcing the intended ROS workspace.
"""
import argparse
import datetime
import json
import os
import platform
import shutil
import subprocess
from pathlib import Path


def command(argv, timeout=8):
    if not shutil.which(argv[0]):
        return {"status": "missing", "argv": argv}
    try:
        result = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                universal_newlines=True, timeout=timeout)
        return {"status": "ok" if result.returncode == 0 else "error",
                "argv": argv, "returncode": result.returncode,
                "stdout": result.stdout[:16000], "stderr": result.stderr[:3000]}
    except subprocess.TimeoutExpired:
        return {"status": "timeout", "argv": argv, "seconds": timeout}
    except (OSError, UnicodeError) as exc:
        return {"status": "error", "argv": argv, "error": str(exc)}


def inventory():
    data = {"schema": 1, "captured_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "system": platform.system(), "architecture": platform.machine(),
            "python": platform.python_version(),
            "ros_version_env": os.environ.get("ROS_VERSION"),
            "ros_distro_env": os.environ.get("ROS_DISTRO"),
            "note": "Read-only inventory; not a readiness or safety certificate.",
            "checks": {}}
    os_release = Path("/etc/os-release")
    if platform.system() == "Linux" and os_release.exists():
        data["os_release"] = os_release.read_text()
    commands = {"ros1_distro": ["rosversion", "-d"],
                "ros1_topics": ["rostopic", "list"],
                "ros1_nodes": ["rosnode", "list"],
                "ros2_packages": ["ros2", "pkg", "list"],
                "ros2_topics": ["ros2", "topic", "list", "-t"],
                "ros2_actions": ["ros2", "action", "list", "-t"]}
    for name, argv in commands.items():
        data["checks"][name] = command(argv)
    packages = ["limo_base", "limo_bringup", "limo_gazebo_sim", "gmapping",
                "robot_pose_ekf", "move_base", "explore_lite", "rrt_exploration",
                "astra_camera", "realsense2_camera", "apriltag_ros"]
    if shutil.which("rospack"):
        for name in packages:
            check = command(["rospack", "find", name])
            if check.get("status") == "ok":
                location = Path(check["stdout"].strip())
                launch = location / "launch"
                if launch.is_dir():
                    check["launch_files"] = sorted(str(p.relative_to(location)) for p in launch.rglob("*") if p.is_file())
            data["checks"]["ros1_package_" + name] = check
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="environment.json")
    args = parser.parse_args()
    output = Path(args.output)
    if output.exists():
        parser.error("Output already exists; choose a new filename to preserve evidence.")
    output.parent.mkdir(parents=True, exist_ok=True)
    data = inventory()
    output.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    print(str(output.resolve()))


if __name__ == "__main__":
    main()
