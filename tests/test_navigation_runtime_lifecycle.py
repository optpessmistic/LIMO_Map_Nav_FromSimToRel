"""Run the real navigation probe with deterministic, in-process ROS substitutes.

These tests exercise main(), its lifecycle gate and the written report. They
create no ROS nodes, DDS traffic, motion commands, or simulation processes.
Transport discovery/delivery is simulated, not validated by these unit tests.
"""
from concurrent.futures import Future
from contextlib import redirect_stdout
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace as S
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))


def module(name, **attributes):
    result = ModuleType(name)
    result.__dict__.update(attributes)
    return result


class FakeTime:
    def __init__(self, nanoseconds=0):
        self.nanoseconds = nanoseconds

    @classmethod
    def from_msg(cls, stamp):
        return cls(stamp.nanoseconds)


class TransformError(Exception):
    pass


class Client:
    def __init__(self, behavior='active', clock=lambda: 0):
        self.behavior, self.calls, self.removed = behavior, [], []
        self.clock, self.request_times = clock, []

    def service_is_ready(self):
        return self.behavior != 'unavailable'

    def call_async(self, request):
        future = Future()
        self.calls.append(future)
        self.request_times.append(self.clock())
        missing = (self.behavior == 'missing'
                   or self.behavior == 'missing_first' and len(self.calls) == 1
                   or self.behavior == 'active_then_missing' and len(self.calls) > 1)
        if not missing:
            state_id = 2 if self.behavior == 'inactive' else 3
            future.set_result(S(current_state=S(id=state_id,
                                               label='active' if state_id == 3 else 'inactive')))
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


class Runtime:
    def __init__(self, planner='active', *, action_ready_after=0, action_available=True,
                 map_available=True, tf_age=0, spin_step_s=.1, clock_running=True):
        self.tick, self.callbacks, self.clients = 0, [], {}
        self.spin_step_s = spin_step_s
        self.clock_running = clock_running
        self.planner = planner
        self.action_ready_after, self.action_available = action_ready_after, action_available
        self.map_available, self.tf_age = map_available, tf_age
        self.destroyed, self.shutdown = False, False
        action = S(server_is_ready=lambda: self.action_available
                   and self.monotonic() >= self.action_ready_after, destroy=lambda: None)
        self.modules = {
            'rclpy': module('rclpy', init=lambda: None,
                           create_node=lambda *args, **kwargs: self,
                           spin_once=self.spin_once, shutdown=self.close),
            'rclpy.action': module('rclpy.action', ActionClient=lambda *args: action),
            'rclpy.parameter': module('rclpy.parameter', Parameter=lambda *args, **kwargs: S()),
            'rclpy.qos': module('rclpy.qos', DurabilityPolicy=S(TRANSIENT_LOCAL=1),
                               QoSProfile=lambda **kwargs: S(**kwargs), ReliabilityPolicy=S(RELIABLE=1)),
            'rclpy.time': module('rclpy.time', Time=FakeTime),
            'lifecycle_msgs': module('lifecycle_msgs'),
            'lifecycle_msgs.srv': module('lifecycle_msgs.srv', GetState=S(Request=object)),
            'nav_msgs': module('nav_msgs'),
            'nav_msgs.msg': module('nav_msgs.msg', OccupancyGrid=object),
            'nav2_msgs': module('nav2_msgs'),
            'nav2_msgs.action': module('nav2_msgs.action', NavigateToPose=object),
            'tf2_ros': module('tf2_ros', Buffer=lambda **kwargs: self,
                              TransformException=TransformError, TransformListener=lambda *args: S()),
        }

    def monotonic(self):
        return self.tick * self.spin_step_s

    def ros_now(self):
        elapsed_ns = round(self.monotonic() * 1e9) if self.clock_running else 0
        return FakeTime(10_000_000_000 + elapsed_ns)

    def get_clock(self):
        return S(now=self.ros_now)

    def create_subscription(self, message_type, topic, callback, qos):
        assert topic == '/map'
        self.callbacks.append(callback)
        return S()

    def create_client(self, service_type, name):
        node = name.split('/')[1]
        client = Client(self.planner if node == 'planner_server' else 'active', self.monotonic)
        self.clients[node] = client
        return client

    def spin_once(self, node, timeout_sec):
        self.tick += 1
        if self.map_available:
            for callback in self.callbacks:
                callback(S(header=S(frame_id='map'), info=S(width=2, height=2, resolution=.05),
                           data=[0, 100, -1, 0]))

    def lookup_transform(self, target, source, when):
        assert (target, source) == ('map', 'base_link')
        stamp = FakeTime(self.ros_now().nanoseconds - int(self.tf_age * 1e9))
        return S(header=S(stamp=stamp), transform=S(translation=S(x=0, y=0, z=0),
                                                   rotation=S(x=0, y=0, z=0, w=1)))

    def destroy_node(self):
        self.destroyed = True

    def close(self):
        self.shutdown = True


class NavigationLifecycleIntegrationTests(unittest.TestCase):
    def run_probe(self, runtime):
        with tempfile.TemporaryDirectory(prefix='navigation-probe-', dir=ROOT) as temporary, \
                patch.dict(sys.modules, runtime.modules):
            self.assertTrue(Path(temporary).resolve().is_relative_to(ROOT.resolve()))
            spec = importlib.util.spec_from_file_location('navigation_runtime_under_test',
                                                          ROOT / 'scripts/check_navigation_runtime.py')
            probe = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(probe)
            output = Path(temporary) / 'navigation.json'
            with patch.object(probe.time, 'monotonic', runtime.monotonic), \
                    patch.object(sys, 'argv', ['probe', '--timeout', '4', '--output', str(output)]), \
                    redirect_stdout(io.StringIO()):
                exit_code = probe.main()
            report = json.loads(output.read_text(encoding='utf-8'))
        self.assertTrue(runtime.destroyed)
        self.assertTrue(runtime.shutdown)
        self.assertTrue(report['read_only'])
        self.assertEqual(exit_code, 0 if report['passed'] else 1)
        return report

    def test_missing_first_planner_reply_retries_then_passes(self):
        runtime = Runtime('missing_first')
        report = self.run_probe(runtime)
        self.assertTrue(report['passed'], report['errors'])
        client = runtime.clients['planner_server']
        self.assertGreaterEqual(len(client.calls), 2)
        self.assertTrue(client.calls[0].cancelled())
        self.assertEqual(client.removed, client.calls[:1])
        self.assertEqual(report['lifecycle']['planner_server'], {'id': 3, 'label': 'active'})
        detail = report['lifecycle_probe']['nodes']['planner_server']
        self.assertEqual(detail['timeouts'], 1)
        self.assertEqual(detail['responses'], 1)
        self.assertLess(runtime.monotonic(), 4)

    def test_permanent_missing_reply_is_not_reported_as_observed_inactive(self):
        runtime = Runtime('missing')
        report = self.run_probe(runtime)
        self.assertFalse(report['passed'])
        self.assertEqual(len(report['errors']), 1)
        self.assertIn('planner_server: no valid response', report['errors'][0])
        self.assertNotIn('planner_server', report['lifecycle'])
        self.assertEqual(set(report['lifecycle']), set(runtime.clients) - {'planner_server'})
        detail = report['lifecycle_probe']['nodes']['planner_server']
        self.assertGreaterEqual(detail['requests'], 2)
        self.assertGreaterEqual(detail['timeouts'], 1)
        self.assertEqual(detail['responses'], 0)
        self.assertIsNone(detail['state_id'])

    def test_real_inactive_response_remains_a_failure(self):
        report = self.run_probe(Runtime('inactive'))
        self.assertFalse(report['passed'])
        self.assertIn('planner_server: state=2', report['errors'][0])
        self.assertEqual(report['lifecycle']['planner_server'], {'id': 2, 'label': 'inactive'})
        self.assertGreater(report['lifecycle_probe']['nodes']['planner_server']['responses'], 0)

    def test_unavailable_service_has_distinct_reason(self):
        report = self.run_probe(Runtime('unavailable'))
        self.assertFalse(report['passed'])
        self.assertIn('planner_server: service unavailable', report['errors'][0])
        detail = report['lifecycle_probe']['nodes']['planner_server']
        self.assertFalse(detail['service_ready'])
        self.assertEqual(detail['requests'], 0)

    def test_cached_active_response_cannot_pass_after_other_gates_become_ready(self):
        report = self.run_probe(Runtime('active_then_missing', action_ready_after=3))
        self.assertFalse(report['passed'])
        self.assertEqual(len(report['errors']), 1)
        self.assertIn('planner_server: state evidence stale', report['errors'][0])
        self.assertEqual(report['lifecycle']['planner_server']['id'], 3)
        detail = report['lifecycle_probe']['nodes']['planner_server']
        self.assertGreater(detail['state_request_age_s'], report['lifecycle_probe']['max_state_age_s'])

    def assert_bounded_requests(self, runtime, report):
        interval = report['lifecycle_probe']['poll_interval_s']
        self.assertEqual(interval, .1)
        for client in runtime.clients.values():
            self.assertLessEqual(len(client.calls), int(runtime.monotonic() / interval) + 1)
            for earlier, later in zip(client.request_times, client.request_times[1:]):
                self.assertGreaterEqual(later - earlier, interval - 1e-9)

    def test_fast_spinning_does_not_flood_get_state_services(self):
        runtime = Runtime(action_ready_after=3, spin_step_s=.001)
        report = self.run_probe(runtime)
        self.assertTrue(report['passed'], report['errors'])
        self.assertGreaterEqual(runtime.tick, 3000)
        self.assert_bounded_requests(runtime, report)
        self.assertTrue(all(state['id'] == 3 for state in report['lifecycle'].values()))

    def test_fast_spinning_still_retries_a_missing_first_reply(self):
        runtime = Runtime('missing_first', spin_step_s=.001)
        report = self.run_probe(runtime)
        self.assertTrue(report['passed'], report['errors'])
        self.assertTrue(runtime.clients['planner_server'].calls[0].cancelled())
        self.assertEqual(report['lifecycle_probe']['nodes']['planner_server']['timeouts'], 1)
        self.assert_bounded_requests(runtime, report)

    def test_freshness_expiring_between_polls_cannot_pass(self):
        # First request at .001 expires after 2.001. The action becomes ready
        # between lifecycle polls: cached readiness would incorrectly pass.
        runtime = Runtime('active_then_missing', action_ready_after=2.002, spin_step_s=.001)
        report = self.run_probe(runtime)
        self.assertFalse(report['passed'])
        self.assertIn('planner_server: state evidence stale', report['errors'][0])
        self.assert_bounded_requests(runtime, report)

    def test_other_readiness_gates_still_block_with_active_lifecycle(self):
        cases = [({'action_available': False}, 'NavigateToPose action unavailable'),
                 ({'map_available': False}, 'No OccupancyGrid received on /map'),
                 ({'clock_running': False}, 'Simulation clock has not advanced or is stale'),
                 ({'tf_age': 2}, 'map->base_link TF timestamp is stale or too far in future')]
        for configuration, expected in cases:
            with self.subTest(configuration=configuration):
                runtime = Runtime(spin_step_s=.001, **configuration)
                report = self.run_probe(runtime)
                self.assertFalse(report['passed'])
                self.assertIn(expected, report['errors'])
                self.assertTrue(all(state['id'] == 3 for state in report['lifecycle'].values()))
                self.assert_bounded_requests(runtime, report)


if __name__ == '__main__':
    unittest.main()
