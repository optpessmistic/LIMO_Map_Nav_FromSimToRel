"""Run actual get_pose with synthetic TF/clock; no ROS imports or processes.

AST extracts only get_pose, event and yaw_of, plus the production defaults.
This checks adapter age semantics and diagnostics, not real TF delivery.
"""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock


SOURCE = Path(__file__).resolve().parents[1] / 'src/limo_maze_mission/limo_maze_mission/node.py'
TREE = ast.parse(SOURCE.read_text(encoding='utf-8'), filename=str(SOURCE))
CLASS = next(item for item in TREE.body if isinstance(item, ast.ClassDef) and item.name == 'MissionNode')
INIT = next(item for item in CLASS.body if isinstance(item, ast.FunctionDef) and item.name == '__init__')
DEFAULTS = ast.literal_eval(next(item.value for item in INIT.body if isinstance(item, ast.Assign)
    and any(isinstance(target, ast.Name) and target.id == 'defaults' for target in item.targets)))
METHODS = [item for item in CLASS.body if isinstance(item, ast.FunctionDef) and item.name in ('get_pose', 'event')]
YAW = next(item for item in TREE.body if isinstance(item, ast.FunctionDef) and item.name == 'yaw_of')
ENV = {'Time': object, 'json': json,
       'angle_delta': lambda a, b: math.atan2(math.sin(a-b), math.cos(a-b))}
ENV.update({name: getattr(math, name) for name in ('atan2', 'hypot', 'isfinite')})
exec(compile(ast.Module(body=[YAW, *METHODS], type_ignores=[]), str(SOURCE), 'exec'), ENV)


def transform(stamp_ns, parent='map', child='base_link'):
    return S(header=S(frame_id=parent, stamp=S(sec=stamp_ns // 10**9, nanosec=stamp_ns % 10**9)),
             child_frame_id=child,
             transform=S(translation=S(x=1.25, y=-.75, z=0.), rotation=S(x=0., y=0., z=0., w=1.)))


class BaseTfAgeTests(unittest.TestCase):
    def node(self, now_s, stamp_ns, **overrides):
        cfg = {**DEFAULTS, **overrides}
        base = transform(stamp_ns, cfg['map_frame'], cfg['base_frame'])
        odom = transform(stamp_ns, cfg['map_frame'], cfg['odom_frame'])
        lookup = Mock(side_effect=lambda parent, child, when: base if child == cfg['base_frame'] else odom)
        node = S(cfg=cfg, tf_buffer=S(lookup_transform=lookup), last_map_odom=None,
                 seconds=Mock(return_value=now_s), event=Mock())
        node.get_pose = ENV['get_pose'].__get__(node)
        return node, base

    def rejected(self, node):
        with self.assertRaisesRegex(ValueError, '^stale_base_tf$'):
            node.get_pose()

    def test_default_one_second_bound_remains_inclusive(self):
        self.assertEqual(DEFAULTS['tf_max_age_s'], 1.0)
        accepted, _ = self.node(2., 1_000_000_000)
        self.assertEqual(accepted.get_pose(), (1.25, -.75, 0.))
        accepted.event.assert_not_called()
        for stamp in (999_999_999, 800_000_000):
            with self.subTest(stamp_ns=stamp):
                rejected, _ = self.node(2., stamp)
                self.rejected(rejected)
                self.assertEqual(rejected.event.call_args.kwargs['max_age_s'], 1.)

    def test_one_point_five_override_accepts_one_point_two_and_boundary(self):
        for stamp in (800_000_000, 500_000_000):
            with self.subTest(stamp_ns=stamp):
                node, _ = self.node(2., stamp, tf_max_age_s=1.5)
                self.assertEqual(node.get_pose(), (1.25, -.75, 0.))
                self.assertEqual(node.seconds.call_count, 1)
                self.assertEqual(node.tf_buffer.lookup_transform.call_count, 2)
                node.event.assert_not_called()

    def test_override_still_rejects_older_than_one_point_five(self):
        node, _ = self.node(2., 499_999_999, tf_max_age_s=1.5)
        self.rejected(node)
        self.assertGreater(node.event.call_args.kwargs['age_s'], 1.5)
        self.assertEqual(node.event.call_args.kwargs['max_age_s'], 1.5)
        self.assertEqual(node.tf_buffer.lookup_transform.call_count, 1)

    def test_future_limit_unchanged_for_default_and_override(self):
        for limit in (1., 1.5):
            with self.subTest(limit=limit):
                boundary, _ = self.node(0., 100_000_000, tf_max_age_s=limit)
                self.assertEqual(boundary.get_pose(), (1.25, -.75, 0.))
                future, _ = self.node(0., 100_000_001, tf_max_age_s=limit)
                self.rejected(future)
                fields = future.event.call_args.kwargs
                self.assertLess(fields['age_s'], -.1)
                self.assertEqual(fields['min_age_s'], -.1)

    def test_first_rejection_records_exact_decision_sample_and_frames_once(self):
        node, base = self.node(100.05, 98_200_000_123, tf_max_age_s=1.5,
                               map_frame='lab_map', base_frame='robot_base')
        self.rejected(node)
        node.seconds.assert_called_once_with()
        node.event.assert_called_once_with('base_tf_age_rejected', now_ros_s=100.05,
            tf_stamp_ros_s=98.200000123, tf_stamp_sec=98, tf_stamp_nanosec=200_000_123,
            age_s=100.05-98.200000123, min_age_s=-.1, max_age_s=1.5,
            map_frame='lab_map', base_frame='robot_base', tf_parent_frame='lab_map', tf_child_frame='robot_base')
        for _ in range(20):
            self.rejected(node)
        # Even recovery and a later rejection cannot flood the lifetime log.
        node.seconds.return_value = 98.3
        self.assertEqual(node.get_pose(), (1.25, -.75, 0.))
        node.seconds.return_value = 105.
        self.rejected(node)
        self.assertEqual(node.event.call_count, 1)
        self.assertEqual(base.header.stamp.nanosec, 200_000_123)

    def test_existing_event_logging_clock_does_not_change_recorded_age(self):
        node, _ = self.node(2., 499_000_000, tf_max_age_s=1.5)
        node.seconds.side_effect = [2., 20.]
        node.events = []
        logger = Mock()
        node.get_logger = lambda: logger
        node.event = ENV['event'].__get__(node)
        self.rejected(node)
        entry = node.events[0]
        self.assertEqual(entry['stamp'], 20.)  # Existing event emission time.
        self.assertEqual(entry['now_ros_s'], 2.)  # Single rejection decision time.
        self.assertEqual(entry['tf_stamp_ros_s'], .499)
        self.assertEqual(entry['age_s'], 2.-.499)
        self.assertEqual(node.seconds.call_count, 2)
        self.assertEqual(len(node.events), 1)
        logger.info.assert_called_once()

    def test_diagnostic_event_failure_preserves_stale_rejection_without_retry(self):
        node, _ = self.node(2., 499_000_000, tf_max_age_s=1.5)
        node.event.side_effect = RuntimeError('diagnostic logger unavailable')
        self.rejected(node)
        self.rejected(node)
        self.assertTrue(node._base_tf_age_rejection_logged)
        node.event.assert_called_once()
        self.assertIsNone(node.last_map_odom)
        self.assertEqual(node.tf_buffer.lookup_transform.call_count, 2)


if __name__ == '__main__':
    unittest.main()
