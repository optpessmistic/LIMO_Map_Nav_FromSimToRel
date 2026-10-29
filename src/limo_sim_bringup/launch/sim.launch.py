"""Start native Harmonic simulation; never start a physical LIMO driver."""

import os
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def _bool_arg(context, name):
    value = LaunchConfiguration(name).perform(context).lower()
    if value not in {"true", "false"}:
        raise ValueError(f"{name} must be true or false, got {value!r}")
    return value == "true"


def _start(context):
    share = Path(get_package_share_directory("limo_sim_bringup"))
    model_share = Path(get_package_share_directory("limo_harmonic_description"))
    world = Path(LaunchConfiguration("world").perform(context)).expanduser().resolve()
    if not world.is_file():
        raise FileNotFoundError(f"Gazebo world does not exist: {world}")
    world_element = ET.parse(world).getroot().find("world")
    if world_element is None or not world_element.get("name"):
        raise ValueError(f"Expected an SDF <world name=...> in {world}")
    world_name = world_element.get("name")
    robot_xml = xacro.process_file(
        str(model_share / "urdf/limo_gazebo.xacro"),
        mappings={"lidar_fov_deg": LaunchConfiguration("lidar_fov_deg").perform(context)},
    ).toxml()

    gazebo_args = ["-r", "-v", "3"]
    if _bool_arg(context, "headless"):
        gazebo_args.append("-s")
    if _bool_arg(context, "headless_rendering"):
        gazebo_args.append("--headless-rendering")
    gazebo_args.append(str(world))

    # Launch without a shell: ros_gz_sim's shell wrapper can orphan the Ruby
    # Gazebo process when only the wrapper receives SIGINT during shutdown.
    simulator = ExecuteProcess(
        cmd=[FindExecutable(name="gz"), "sim", *gazebo_args, "--force-version", "8"],
        name="gazebo",
        output="screen",
        on_exit=[EmitEvent(event=Shutdown(reason="Gazebo server exited"))],
    )
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="limo_sim_state_publisher",
        parameters=[{"robot_description": ParameterValue(robot_xml, value_type=str),
                     "use_sim_time": True}],
        output="screen",
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="limo_sim_bridge",
        parameters=[{
            "config_file": str(share / "config/bridge.yaml"),
            "use_sim_time": True,
        }],
        output="screen",
    )
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_limo",
        arguments=[
            "-world", world_name,
            "-name", "limo",
            "-allow_renaming", "false",
            "-string", robot_xml,
            "-x", LaunchConfiguration("start_x"),
            "-y", LaunchConfiguration("start_y"),
            "-z", LaunchConfiguration("start_z"),
            "-Y", LaunchConfiguration("start_yaw"),
        ],
        output="screen",
    )

    def spawn_finished(event, _context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason="LIMO model spawn failed"))]
        return []

    paths = [str(model_share), str(model_share.parent)]
    existing_paths = os.environ.get("GZ_SIM_RESOURCE_PATH")
    if existing_paths:
        paths.append(existing_paths)
    return [
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.pathsep.join(paths)),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=spawn_finished)),
        simulator,
        state_publisher,
        bridge,
        spawn,
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("limo_sim_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=str(share / "worlds/empty.sdf")),
        DeclareLaunchArgument("headless", default_value="true",
                              description="Run the server without the Gazebo GUI."),
        DeclareLaunchArgument("headless_rendering", default_value="false",
                              description="Use EGL rendering when supported by the host GPU."),
        DeclareLaunchArgument("lidar_fov_deg", default_value="240", choices=["240", "360"],
                              description="240 uses the clear front sector; 360 exposes self-occlusion for study."),
        DeclareLaunchArgument("start_x", default_value="0.0"),
        DeclareLaunchArgument("start_y", default_value="0.0"),
        DeclareLaunchArgument("start_z", default_value="0.0"),
        DeclareLaunchArgument("start_yaw", default_value="0.0"),
        OpaqueFunction(function=_start),
    ])
