"""Execute the actual perception constructor with real retention policy validation.

ROS, image conversion and worker ownership are stubbed; no ROS node, subprocess,
camera or filesystem output is created. These tests verify parameter wiring,
not DDS delivery or the independent retention worker's behavior.
"""
import ast
from math import isfinite
from pathlib import Path
import sys
import threading
from types import SimpleNamespace as S
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/limo_evidence_protocol'), str(ROOT/'src/limo_maze_perception')]
from limo_evidence_protocol.retention import Policy
from limo_maze_perception.retention_adapter import RuntimePolicy


class FakeNode:
    def __init__(self, name, **kwargs):
        self.params = dict(kwargs.get('parameter_overrides', {}))

    def declare_parameter(self, name, value):
        self.params.setdefault(name, value)

    def get_parameter(self, name):
        return S(value=self.params[name])

    def create_publisher(self, *args, **kwargs):
        return S()

    def create_subscription(self, *args, **kwargs):
        return S()

    def create_timer(self, *args, **kwargs):
        return S()

    def get_clock(self):
        return S(create_jump_callback=lambda *args, **kwargs: S())

    def get_logger(self):
        return S(warn=lambda *args: None)


class RetentionRequestParameterTests(unittest.TestCase):
    def setUp(self):
        self.calls = []

        def producer(output, session_id, policy, runtime):
            owner = S(output=output, session_id=session_id, policy=policy, runtime=runtime)
            self.calls.append(owner)
            return owner

        namespace = dict(Node=FakeNode, isfinite=isfinite, threading=threading,
            RetentionPolicy=Policy, RuntimePolicy=RuntimePolicy, ProducerRetention=producer,
            CvBridge=lambda: S(), DetectorConfig=lambda **kwargs: S(validate=lambda: None),
            FrameEvidence=lambda *args: S(session_id='constructor-fixture'),
            String=S, Image=S, CameraInfo=S, qos_profile_sensor_data=None,
            QoSProfile=lambda **kwargs: S(**kwargs),
            DurabilityPolicy=S(TRANSIENT_LOCAL=1, VOLATILE=0), ReliabilityPolicy=S(RELIABLE=1),
            JumpThreshold=lambda **kwargs: S(**kwargs), Duration=lambda **kwargs: S(**kwargs),
            Clock=lambda **kwargs: S(**kwargs), ClockType=S(STEADY_TIME=1),
            message_filters=S(Subscriber=lambda *args, **kwargs: S(),
                ApproximateTimeSynchronizer=lambda *args, **kwargs:
                    S(registerCallback=lambda callback: None)))
        source = ROOT/'src/limo_maze_perception/limo_maze_perception/node.py'
        tree = ast.parse(source.read_text(encoding='utf-8'), filename=str(source))
        cls = next(item for item in tree.body if isinstance(item, ast.ClassDef)
                   and item.name == 'RedRGBDBaseline')
        exec(compile(ast.Module(body=[cls], type_ignores=[]), str(source), 'exec'), namespace)
        self.node_type = namespace['RedRGBDBaseline']

    def make_node(self, **overrides):
        params = {'publish_frame_processing': True, 'retention_enabled': True,
                  'retention_output_dir': '/unused-constructor-fixture'}
        params.update(overrides)
        return self.node_type(parameter_overrides=params)

    def test_default_request_history_matches_existing_policy_default(self):
        node = self.make_node()
        self.assertEqual(node.get_parameter('retention_max_requests').value, 128)
        self.assertEqual(node.retention.policy, Policy())
        self.assertEqual(len(self.calls), 1)

    def test_configured_request_history_reaches_real_policy_independently_of_job_budget(self):
        node = self.make_node(retention_max_requests=256, retention_max_jobs=256,
                              retention_max_file_bytes=805306368)
        self.assertEqual(node.retention.policy.max_requests, 256)
        self.assertEqual(node.retention.policy.max_jobs, 256)
        self.assertEqual(node.retention.policy.max_file_bytes, 805306368)
        self.assertEqual(node.retention.policy.max_payload_bytes, Policy().max_payload_bytes)
        # Raising jobs alone must not silently alter request-history semantics.
        unchanged = self.make_node(retention_max_jobs=256)
        self.assertEqual(unchanged.retention.policy.max_requests, 128)

    def test_invalid_request_history_uses_existing_policy_bounds_before_owner_creation(self):
        for value in (0, -1, 4097, 1.5, True, '256'):
            with self.subTest(value=value), self.assertRaisesRegex(ValueError, 'invalid_integer'):
                self.make_node(retention_max_requests=value)
        self.assertEqual(self.calls, [])
        for value in (1, 4096):
            self.assertEqual(self.make_node(retention_max_requests=value).retention.policy.max_requests, value)

    def test_default_disabled_retention_does_not_create_owner(self):
        node = self.node_type()
        self.assertEqual(node.get_parameter('retention_max_requests').value, 128)
        self.assertIsNone(node.retention)
        self.assertEqual(self.calls, [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
