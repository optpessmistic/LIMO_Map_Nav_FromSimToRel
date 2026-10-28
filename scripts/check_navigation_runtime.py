#!/usr/bin/env python3
"""Read-only readiness check for online SLAM and Nav2; sends no motion goal."""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformException, TransformListener


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=90)
    parser.add_argument('--output', type=Path, default=Path('outputs/ros_smoke/navigation_runtime.json'))
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('limo_navigation_probe',
                            parameter_overrides=[Parameter('use_sim_time', value=True)])
    maps = []
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(OccupancyGrid, '/map',
                                            lambda msg: maps.append((time.monotonic(), msg)), qos)
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node)
    action = ActionClient(node, NavigateToPose, 'navigate_to_pose')
    names = ('planner_server', 'controller_server', 'bt_navigator',
             'behavior_server', 'velocity_smoother', 'collision_monitor')
    clients = {name: node.create_client(GetState, '/' + name + '/get_state') for name in names}
    pending, states, last_request = {}, {}, {}
    transform, errors = None, []
    started = time.monotonic()
    while time.monotonic() - started < args.timeout:
        rclpy.spin_once(node, timeout_sec=.1)
        now = time.monotonic()
        for name, client in clients.items():
            future = pending.get(name)
            if future is not None and future.done():
                try:
                    state = future.result().current_state
                    states[name] = {'id': state.id, 'label': state.label}
                except Exception as exc:
                    states[name] = {'error': str(exc)}
                pending.pop(name)
            if name not in pending and now-last_request.get(name, 0) > 1 and client.service_is_ready():
                pending[name] = client.call_async(GetState.Request())
                last_request[name] = now
        try:
            transform = buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            transform = None
        if (len(maps) >= 2 and transform is not None and action.server_is_ready()
                and all(states.get(name, {}).get('id') == 3 for name in names)):
            break

    if not action.server_is_ready():
        errors.append('NavigateToPose action unavailable')
    for name in names:
        if states.get(name, {}).get('id') != 3:
            errors.append(name + ' lifecycle state is not active')
    map_info = None
    if not maps:
        errors.append('No OccupancyGrid received on /map')
    else:
        received, msg = maps[-1]
        map_info = {'frame_id': msg.header.frame_id, 'width': msg.info.width,
                    'height': msg.info.height, 'resolution': msg.info.resolution,
                    'messages': len(maps), 'free_cells': sum(v == 0 for v in msg.data),
                    'occupied_cells': sum(v >= 65 for v in msg.data),
                    'unknown_cells': sum(v < 0 for v in msg.data)}
        if (msg.header.frame_id != 'map' or msg.info.resolution <= 0
                or len(msg.data) != msg.info.width*msg.info.height
                or not map_info['free_cells'] or not map_info['occupied_cells']):
            errors.append('Map geometry/frame or free/occupied observations invalid')
        if len(maps) < 2 or time.monotonic()-received > 5:
            errors.append('Map updates missing or stale')
    pose = None
    if transform is None:
        errors.append('map->base_link TF unavailable')
    else:
        tf_ns = Time.from_msg(transform.header.stamp).nanoseconds
        age = (node.get_clock().now().nanoseconds-tf_ns)/1e9
        position, orientation = transform.transform.translation, transform.transform.rotation
        pose = {'x': position.x, 'y': position.y, 'z': position.z,
                'quaternion': [orientation.x, orientation.y, orientation.z, orientation.w],
                'tf_age_s': age}
        if age > 1 or age < -.5:
            errors.append('map->base_link TF timestamp is stale or too far in future')
    report = {'schema_version': 1, 'utc': datetime.now(timezone.utc).isoformat(),
              'passed': not errors, 'read_only': True,
              'ros_distro': os.environ.get('ROS_DISTRO'),
              'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
              'map': map_info, 'pose': pose, 'lifecycle': states, 'errors': errors}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2))
    action.destroy()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
