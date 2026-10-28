#!/usr/bin/env python3
"""Derive checked-in robot parameters from the installed Jazzy Nav2 defaults.

Run with /usr/bin/python3 in WSL after setup_jazzy.sh. Requires python3-yaml.
"""
import argparse
from copy import deepcopy
from pathlib import Path


def configure(data):
    if not isinstance(data, dict):
        raise ValueError("Nav2 defaults must be a YAML mapping")
    data = deepcopy(data)
    def walk(value):
        if isinstance(value, list):
            for item in value:
                walk(item)
            return
        if not isinstance(value, dict):
            return
        for key, item in value.items():
            if key in ("robot_base_frame", "base_frame_id", "base_frame"):
                value[key] = "base_link"
            elif key == "odom_topic":
                value[key] = "/odom"
            elif key == "use_sim_time":
                value[key] = True
            elif key == "enable_stamped_cmd_vel":
                value[key] = False
            elif key == "ros__parameters" and isinstance(item, dict):
                item["use_sim_time"] = True
                walk(item)
            else:
                walk(item)
    walk(data)
    if "bt_navigator" in data:
        # Simulation scheduling profile: allow action request/ACK round trips
        # more than 20 ms. This does not change loop/service-wait durations or
        # establish the cause of a separate controller/mission goal timeout.
        data["bt_navigator"]["ros__parameters"]["default_server_timeout"] = 200
    # The native LIMO bridge consumes Twist; do not mix TwistStamped.
    for node in ("controller_server", "velocity_smoother", "behavior_server", "collision_monitor"):
        if node in data:
            data[node]["ros__parameters"]["enable_stamped_cmd_vel"] = False
    controller = data["controller_server"]["ros__parameters"]
    controller["controller_frequency"] = 10.0
    controller["min_x_velocity_threshold"] = 0.001
    controller["min_y_velocity_threshold"] = 0.001
    controller["min_theta_velocity_threshold"] = 0.001
    follow = controller["FollowPath"]
    plugin = follow.get("plugin")
    if plugin == "dwb_core::DWBLocalPlanner":
        follow.update(max_vel_x=0.15, min_vel_x=0.0, max_vel_y=0.0,
                      max_vel_theta=0.4, max_speed_xy=0.15, min_speed_xy=0.0,
                      min_speed_theta=0.0, acc_lim_x=0.3, acc_lim_y=0.0,
                      acc_lim_theta=0.6, decel_lim_x=-0.3, decel_lim_y=0.0,
                      decel_lim_theta=-0.6, vy_samples=1,
                      trans_stopped_velocity=0.03, xy_goal_tolerance=0.05)
    elif plugin == "nav2_mppi_controller::MPPIController":
        # Newer Jazzy defaults use MPPI. Its optimizer rejects a controller
        # period greater than model_dt, so changing frequency requires both.
        follow.update(motion_model="DiffDrive", model_dt=1.0/controller["controller_frequency"],
                      vx_max=0.15, vx_min=-0.10, vy_max=0.0, wz_max=0.4,
                      ax_max=0.3, ax_min=-0.3, ay_max=0.0, ay_min=0.0, az_max=0.6,
                      vx_std=0.08, wz_std=0.15, visualize=False)
        if "CostCritic" in follow:
            follow["CostCritic"]["consider_footprint"] = True
    else:
        raise ValueError(f"Unreviewed controller plugin {plugin!r}; expected Jazzy DWB or MPPI defaults")
    for checker in controller.get("goal_checker_plugins", []):
        # SimpleGoalChecker must recheck XY after its final heading adjustment;
        # stateful=true latches the first XY pass and breaks the arrival bound.
        controller[checker].update(xy_goal_tolerance=0.05, yaw_goal_tolerance=0.26, stateful=False)
    for checker in controller.get("progress_checker_plugins", []):
        # At 0.4 rad/s an about-turn alone takes nearly 8 s. The generic
        # 0.5 m / 10 s default can abort a valid low-speed turn and departure.
        controller[checker].update(required_movement_radius=0.15, movement_time_allowance=30.0)
    for name in ("local_costmap", "global_costmap"):
        cost = data[name][name]["ros__parameters"]
        cost.pop("robot_radius", None)
        cost["footprint"] = "[[0.18, 0.12], [0.18, -0.12], [-0.18, -0.12], [-0.18, 0.12]]"
        cost["footprint_padding"] = 0.02
        cost["resolution"] = 0.05
        cost["inflation_layer"]["inflation_radius"] = 0.30
        cost["inflation_layer"]["cost_scaling_factor"] = 4.0
        for layer_name in ("obstacle_layer", "voxel_layer"):
            layer = cost.get(layer_name, {})
            if "scan" in layer:
                layer["scan"]["topic"] = "/scan"
                layer["scan"]["obstacle_max_range"] = 7.0
                layer["scan"]["raytrace_max_range"] = 8.0
    global_cost = data["global_costmap"]["global_costmap"]["ros__parameters"]
    global_cost["track_unknown_space"] = True
    global_cost["static_layer"]["subscribe_to_updates"] = True
    # Goals and paths remain in observed free space.
    data["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"] = False
    smoother = data["velocity_smoother"]["ros__parameters"]
    smoother.update(max_velocity=[0.15, 0.0, 0.4], min_velocity=[-0.10, 0.0, -0.4],
                    max_accel=[0.3, 0.0, 0.6], max_decel=[-0.3, 0.0, -0.6])
    if "behavior_server" in data:
        data["behavior_server"]["ros__parameters"].update(
            max_rotational_vel=0.4, min_rotational_vel=0.1, rotational_acc_lim=0.6)
    if "collision_monitor" in data:
        monitor = data["collision_monitor"]["ros__parameters"]
        # Replace static TurtleBot-sized stop/slowdown zones with predictive
        # collision checking against THIS robot's padded costmap footprint.
        # The monitor remains active in the final Twist command chain.
        monitor.update(cmd_vel_in_topic="cmd_vel_smoothed", cmd_vel_out_topic="cmd_vel",
                       polygons=["FootprintApproach"])
        monitor["FootprintApproach"] = {
            "type": "polygon", "action_type": "approach",
            "footprint_topic": "/local_costmap/published_footprint",
            "time_before_collision": 1.2, "simulation_time_step": 0.1,
            "min_points": 6, "visualize": False, "enabled": True,
        }
        for source_name in monitor.get("observation_sources", []):
            source = monitor.get(source_name, {})
            if source.get("type") == "scan":
                source["topic"] = "/scan"
    # Avoid default AMCL/map server parameters suggesting localization startup.
    data.pop("amcl", None)
    data.pop("map_server", None)
    return data


if __name__ == "__main__":
    import yaml
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path("/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml"))
    parser.add_argument("--output", type=Path, default=Path(__file__).resolve().parents[1] / "src/limo_maze_bringup/config/nav2.yaml")
    args = parser.parse_args()
    try:
        document = configure(yaml.safe_load(args.source.read_text(encoding="utf-8")))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text("# Derived from installed ROS2 Jazzy Nav2 defaults. See tools/configure_nav2.py.\n" +
                               yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
        print(args.output)
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        parser.error(str(exc))
