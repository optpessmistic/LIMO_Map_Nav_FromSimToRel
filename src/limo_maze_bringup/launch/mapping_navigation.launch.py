"""Online SLAM + Nav2 for an already running robot/simulator (no AMCL)."""
import os
import uuid
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_stack(context):
    nav = get_package_share_directory("limo_maze_bringup")
    slam = get_package_share_directory("slam_toolbox")
    use_sim_time = LaunchConfiguration("use_sim_time")
    for name in ("nav_params", "slam_params"):
        path = LaunchConfiguration(name).perform(context)
        if not os.path.isfile(path):
            raise FileNotFoundError(f"Missing {name}: {path}. Generate config/nav2.yaml with tools/configure_nav2.py before building, or pass an existing parameter file.")
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(slam, "launch", "online_async_launch.py")),
            launch_arguments={"use_sim_time": use_sim_time, "slam_params_file": LaunchConfiguration("slam_params"),
                              "autostart": "true", "use_lifecycle_manager": "false"}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(nav, "launch", "navigation.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time, "params_file": LaunchConfiguration("nav_params")}.items()),
    ]


def generate_launch_description():
    project = get_package_share_directory("limo_maze_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("map_epoch", default_value=str(uuid.uuid4())),
        DeclareLaunchArgument("nav_params", default_value=os.path.join(project, "config", "nav2.yaml")),
        DeclareLaunchArgument("slam_params", default_value=os.path.join(project, "config", "slam.yaml")),
        Node(package="limo_maze_bringup", executable="map_session.py", output="screen",
             parameters=[{"epoch": LaunchConfiguration("map_epoch")}]),
        OpaqueFunction(function=_launch_stack),
    ])
