from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('autonomous', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('output_dir', default_value='mission_results'),
        Node(package='limo_maze_mission', executable='mission', output='screen',
             parameters=[{'autonomous': ParameterValue(LaunchConfiguration('autonomous'), value_type=bool),
                          'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                          'output_dir': LaunchConfiguration('output_dir')}]),
    ])
