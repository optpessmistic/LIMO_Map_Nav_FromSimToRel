"""One launch for simulation, online SLAM, Nav2, and optional autonomous mission."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('limo_maze_bringup')
    sim_share = get_package_share_directory('limo_sim_bringup')
    mission_share = get_package_share_directory('limo_maze_mission')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(share, 'worlds', 'example_maze.world')),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('headless_rendering', default_value='false'),
        DeclareLaunchArgument('autonomous', default_value='false'),
        DeclareLaunchArgument('red_surrogate', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value='outputs/mission'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(sim_share, 'launch', 'sim.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'), 'headless': LaunchConfiguration('headless'),
                              'headless_rendering': LaunchConfiguration('headless_rendering')}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'mapping_navigation.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(mission_share, 'launch', 'mission.launch.py')),
            launch_arguments={'autonomous': LaunchConfiguration('autonomous'), 'use_sim_time': 'true',
                              'output_dir': LaunchConfiguration('output_dir')}.items()),
        Node(package='limo_maze_perception', executable='red_rgbd_detector', output='screen',
             parameters=[{'use_sim_time': True,
                          'enabled': ParameterValue(LaunchConfiguration('red_surrogate'), value_type=bool),
                          'depth_registered_to_color': True}]),
    ])
