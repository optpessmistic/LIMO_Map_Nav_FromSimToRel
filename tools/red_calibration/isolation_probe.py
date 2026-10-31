#!/usr/bin/python3
"""Explicit read-only ROS-domain probe, bounded by its owning subprocess runner."""
import argparse
import json
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if not args.execute:
        parser.error('--execute required; no ROS imports or nodes created')
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    report = {'passed': False, 'read_only': True, 'nodes': [], 'errors': []}
    try:
        node = rclpy.create_node('red_calibration_isolation_probe', enable_rosout=False,
                                 start_parameter_services=False)
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        report['nodes'] = node.get_node_names()
        report['passed'] = report['nodes'] == [node.get_name()]
    except (Exception, KeyboardInterrupt) as exc:
        report['errors'].append(type(exc).__name__+': '+str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        with args.output.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, indent=2)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
