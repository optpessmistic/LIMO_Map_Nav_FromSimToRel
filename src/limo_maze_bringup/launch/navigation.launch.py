"""Nav2 nodes used by this project, with one final velocity output.

The distro's generic navigation launch also starts docking, which publishes
directly to cmd_vel. This task does not use docking or route following. Keep
the project command path controller/behaviors -> smoother -> monitor explicit.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _nodes(context):
    params = ParameterFile(RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={'use_sim_time': LaunchConfiguration('use_sim_time')},
        convert_types=True), allow_substs=True)
    specifications = [
        ('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_smoother', 'smoother_server', []),
        ('nav2_planner', 'planner_server', []),
        ('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_bt_navigator', 'bt_navigator', []),
        ('nav2_velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_collision_monitor', 'collision_monitor', []),
    ]
    nodes = [Node(package=package, executable=name, name=name, output='screen',
                  parameters=[params], remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]+remappings)
             for package, name, remappings in specifications]
    nodes.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                      name='lifecycle_manager_navigation', output='screen',
                      parameters=[{'autostart': True, 'node_names': [name for _, name, _ in specifications]}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=_nodes),
    ])
