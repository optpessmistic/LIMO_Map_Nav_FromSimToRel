#!/usr/bin/env python3
"""Publish identity of a fresh SLAM session, not an automatic reset detector."""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = Node('limo_map_session')
    node.declare_parameter('epoch', '')
    epoch = node.get_parameter('epoch').value
    if not isinstance(epoch, str) or not epoch.strip():
        raise ValueError('A nonempty, unique map session epoch is required')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(String, '/mission/map_epoch', qos)
    publisher.publish(String(data=epoch))
    node.get_logger().info('Fresh SLAM map epoch: ' + epoch)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
