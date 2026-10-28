#!/usr/bin/env python3
"""Capture one exact-stamp simulator RGB-D pair; subscribe only, never move.

Source scripts/env.sh first. Requires registered, rectified RGB-D imagery.
An absent matching detection message is unknown, not a negative detection.
"""
import argparse
from collections import OrderedDict
import json
import math
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def stamp_ns(msg):
    stamp = msg.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('Invalid ROS acquisition timestamp')
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def save_capture(output, pair, observation):
    color, depth, info = pair
    ns = stamp_ns(color)
    if ns != stamp_ns(depth) or ns != stamp_ns(info):
        raise ValueError('RGB, depth and CameraInfo must have identical timestamps')
    if not color.header.frame_id or len({m.header.frame_id for m in pair}) != 1:
        raise ValueError('RGB, registered depth and calibration frames must match')
    if len({(m.width, m.height) for m in pair}) != 1 or min(info.width, info.height) <= 0:
        raise ValueError('RGB, depth and calibration dimensions must match and be positive')
    if color.encoding not in ('rgb8', 'bgr8') or depth.encoding not in ('32FC1', '16UC1'):
        raise ValueError('Expected RGB8/BGR8 and 32FC1 metres or 16UC1 millimetres')
    if (not all(math.isfinite(v) for v in info.k) or min(info.k[0], info.k[4]) <= 0
            or any(not math.isfinite(v) or abs(v) > 1e-8 for v in info.d)
            or info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset):
        raise ValueError('Expected valid calibrated, rectified, unbinned simulator CameraInfo')
    if observation is not None and (observation['frame_id'] != color.header.frame_id
                                    or abs(observation['stamp'] - ns / 1e9) > 1e-9):
        raise ValueError('Observation does not match RGB acquisition time/frame')
    bridge = CvBridge()
    bgr = bridge.imgmsg_to_cv2(color, desired_encoding='bgr8')
    raw = bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
    expected_type = np.float32 if depth.encoding == '32FC1' else np.uint16
    if raw.dtype != expected_type or raw.shape != (info.height, info.width):
        raise ValueError('Decoded depth shape/type disagrees with Image encoding')
    metres = raw.astype(np.float32) * (0.001 if depth.encoding == '16UC1' else 1.0)
    valid = np.isfinite(metres) & (metres > 0)
    if not valid.any():
        raise ValueError('Depth contains no finite positive measurements')
    output.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output / 'rgb.png'), bgr):
        raise OSError('PNG encoder failed')
    np.save(output / 'depth_m.npy', metres, allow_pickle=False)  # Keep invalid samples unchanged.
    report = {'schema_version': 1, 'read_only': True, 'stamp_ns': ns,
              'frame_id': color.header.frame_id, 'synchronization': 'exact_RGB_depth_CameraInfo',
              'source_encodings': {'rgb': color.encoding, 'depth': depth.encoding},
              'depth_unit': 'metre', 'depth_valid_count': int(valid.sum()),
              'camera_info': {'width': info.width, 'height': info.height,
                              'distortion_model': info.distortion_model,
                              **{k: list(getattr(info, k)) for k in ('k', 'd', 'r', 'p')}},
              'target_observation': observation,
              'observation_status': 'matched' if observation is not None else 'missing_at_timeout',
              'detected_red_surrogate': (any(d['class_id'] == 'red_object' for d in observation['detections'])
                                         if observation is not None else None),
              'note': 'Simulation red surrogate only; no target identity or search-coverage claim.'}
    (output / 'capture.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps({'output': str(output.resolve()), 'stamp_ns': ns,
                      'observation_status': report['observation_status'],
                      'detected_red_surrogate': report['detected_red_surrogate']}))
    return 0 if observation is not None else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--output', type=Path, default=Path('outputs/ros_smoke/evidence'))
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    if any((args.output / name).exists() for name in ('rgb.png', 'depth_m.npy', 'capture.json')):
        parser.error('Evidence files already exist; choose another output directory')
    rclpy.init()
    node = rclpy.create_node('limo_rgbd_evidence_capture')
    frames, observations = OrderedDict(), OrderedDict()

    def retain(cache, key, value):
        cache[key] = value
        while len(cache) > 10:
            cache.popitem(last=False)

    def on_observation(msg):
        try:
            payload = json.loads(msg.data)
            if (payload['schema_version'] != 1 or not math.isfinite(payload['stamp'])
                    or payload['stamp'] < 0 or not isinstance(payload['detections'], list)):
                raise ValueError('Invalid observation schema/stamp/detections')
            for detection in payload['detections']:
                if (not isinstance(detection['class_id'], str) or len(detection['position']) != 3
                        or not all(math.isfinite(v) for v in detection['position'])):
                    raise ValueError('Invalid observed class or position')
            retain(observations, round(payload['stamp'] * 1e9), payload)
        except (ValueError, TypeError, KeyError) as exc:
            node.get_logger().warning('Ignoring malformed observation: ' + str(exc))

    topics = ('/camera/color/image_raw', '/camera/depth/image_raw', '/camera/color/camera_info')
    subscribers = [message_filters.Subscriber(node, typ, topic, qos_profile=qos_profile_sensor_data)
                   for typ, topic in zip((Image, Image, CameraInfo), topics)]
    sync = message_filters.TimeSynchronizer(subscribers, 10)
    sync.registerCallback(lambda *pair: retain(frames, stamp_ns(pair[0]), pair))
    subscription = node.create_subscription(String, '/mission/target_observations', on_observation, 10)
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            for ns in frames.keys() & observations.keys():
                return save_capture(args.output, frames[ns], observations[ns])
        if frames:
            return save_capture(args.output, next(reversed(frames.values())), None)
        raise ValueError('Timeout: no exactly synchronized RGB/depth/CameraInfo triple received')
    except (ValueError, OSError, CvBridgeError) as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
