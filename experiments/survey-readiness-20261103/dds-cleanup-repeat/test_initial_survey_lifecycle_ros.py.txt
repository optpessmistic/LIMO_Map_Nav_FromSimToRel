"""Opt-in GetState retry/late-response DDS regression, without motion.

After sourcing Jazzy, run in an otherwise unused test domain:
  RUN_ROS_SURVEY_TEST=1 ROS_DOMAIN_ID=74 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST /usr/bin/python3 -B -m unittest \
    discover -s tests -p test_initial_survey_lifecycle_ros.py -v

Only lifecycle GetState clients/servers are created. No velocity publisher,
Nav2, Gazebo, simulation clock, or initial-survey run() is started. A real
service callback gate delays the first reply; production polling and rclpy
futures are used unchanged. These tests do not prove whole-preflight readiness.
"""
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
import unittest
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from initial_survey import LifecycleReadiness, SurveyAbort


class RecordingClient:
    """Observe calls to a real rclpy Client; never synthesize a result/future."""
    def __init__(self, client):
        self.client, self.futures, self.removed = client, [], []

    def service_is_ready(self):
        return self.client.service_is_ready()

    def call_async(self, request):
        future = self.client.call_async(request)
        self.futures.append(future)
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)
        return self.client.remove_pending_request(future)


@unittest.skipUnless(importlib.util.find_spec('rclpy') is not None, 'Source ROS Jazzy for rclpy')
@unittest.skipUnless(importlib.util.find_spec('lifecycle_msgs') is not None, 'Source lifecycle_msgs')
@unittest.skipUnless(os.environ.get('RUN_ROS_SURVEY_TEST') == '1', 'Opt-in: RUN_ROS_SURVEY_TEST=1, domain 74')
class LifecycleReadinessDDSTests(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '74')
        from rclpy.context import Context
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        import rclpy
        self.rclpy = rclpy
        self.context = self.executor = self.client_node = self.server_node = None
        self.first_gate, self.later_gate = threading.Event(), threading.Event()
        self.first_started, self.first_returned = threading.Event(), threading.Event()
        self.lock, self.server_trace = threading.Lock(), []
        self.readiness = self.client = None
        self.addCleanup(self.cleanup_ros)
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=74, signal_handler_options=SignalHandlerOptions.NO)
        self.executor = MultiThreadedExecutor(num_threads=4, context=self.context)
        suffix = uuid.uuid4().hex
        self.client_node = rclpy.create_node('survey_lifecycle_client_' + suffix, context=self.context)
        self.server_node = rclpy.create_node('survey_lifecycle_server_' + suffix, context=self.context)
        self.executor.add_node(self.client_node)
        self.executor.add_node(self.server_node)
        self.service_name = '/survey_lifecycle_test_' + suffix + '/get_state'

    def cleanup_ros(self):
        self.first_gate.set()
        self.later_gate.set()
        closed = True
        drain = {'timeout_s': 2.0, 'pending_futures': 0, 'server_callbacks_pending': 0,
                 'unexpected_future_errors': [], 'new_requests_sent': False}
        try:
            if self.executor is not None:
                # No readiness.poll here: it would create another request.
                # Finish only the already issued DDS calls before destroying
                # handles. A stable drain interval also processes done callbacks.
                started, stable_since = time.monotonic(), None
                futures = list(self.client.futures) if self.client is not None else []
                while time.monotonic() - started < drain['timeout_s']:
                    self.executor.spin_once(timeout_sec=.01)
                    drain['pending_futures'] = sum(not (f.done() or f.cancelled()) for f in futures)
                    with self.lock:
                        drain['server_callbacks_pending'] = sum('returned_state' not in row for row in self.server_trace)
                    if not drain['pending_futures'] and not drain['server_callbacks_pending']:
                        stable_since = stable_since or time.monotonic()
                        if time.monotonic() - stable_since >= .05:
                            break
                    else:
                        stable_since = None
                for index, future in enumerate(futures):
                    if future.done() and not future.cancelled():
                        try:
                            future.result()
                        except Exception as exc:
                            drain['unexpected_future_errors'].append({'future_index': index,
                                'error': type(exc).__name__ + ': ' + str(exc)})
                drain['elapsed_s'] = time.monotonic() - started
                closed = self.executor.shutdown(timeout_sec=5.0)
        finally:
            for node in (self.client_node, self.server_node):
                if node is not None:
                    node.destroy_node()
            if self.context is not None and self.context.ok():
                self.rclpy.shutdown(context=self.context)
        print('LIFECYCLE_DDS_CLEANUP ' + json.dumps(drain))
        self.assertTrue(closed, 'Controlled service callbacks must finish')
        self.assertEqual(drain['pending_futures'], 0, 'Existing client futures must drain within 2 seconds')
        self.assertEqual(drain['server_callbacks_pending'], 0, 'Existing server callbacks must drain within 2 seconds')
        self.assertEqual(drain['unexpected_future_errors'], [], 'Completed future exceptions must not be silently ignored')

    def create_service(self, delay_first):
        from lifecycle_msgs.srv import GetState
        from rclpy.callback_groups import ReentrantCallbackGroup

        def callback(request, response):
            with self.lock:
                number = len(self.server_trace) + 1
                row = {'request_number': number, 'started_monotonic_s': time.monotonic()}
                self.server_trace.append(row)
            if delay_first and number == 1:
                self.first_started.set()
                row['gate_released'] = self.first_gate.wait(timeout=6.0)
                state = 2  # Late original INACTIVE reply must not replace retry ACTIVE.
            elif delay_first and number == 2:
                state = 3
            elif delay_first:
                # Keep later ACTIVE replies from masking a stale overwrite.
                row['gate_released'] = self.later_gate.wait(timeout=6.0)
                state = 2
            else:
                state = 2
            response.current_state.id = state
            response.current_state.label = 'active' if state == 3 else 'inactive'
            with self.lock:
                row.update(returned_state=state, returned_monotonic_s=time.monotonic())
            if delay_first and number == 1:
                self.first_returned.set()
            return response

        self.server_node.create_service(GetState, self.service_name, callback,
                                        callback_group=ReentrantCallbackGroup())
        real_client = self.client_node.create_client(GetState, self.service_name)
        self.assertTrue(real_client.wait_for_service(timeout_sec=5.0), 'Real GetState service discovery')
        self.client = RecordingClient(real_client)
        self.readiness = LifecycleReadiness({'fixture': self.client}, GetState.Request,
                                           request_timeout_s=1.0, max_age_s=2.0)

    def spin_until(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.readiness.poll(time.monotonic())
            self.executor.spin_once(timeout_sec=.01)
            if predicate():
                return
        self.fail('DDS condition not reached: ' + json.dumps(self.snapshot()))

    def snapshot(self):
        with self.lock:
            trace = [dict(row) for row in self.server_trace]
        return {'readiness': self.readiness.report(time.monotonic()), 'server_trace': trace,
                'real_client_requests': len(self.client.futures), 'pending_requests_removed': len(self.client.removed)}

    def test_retry_accepts_fresh_active_and_discards_delayed_original_inactive(self):
        self.create_service(delay_first=True)
        self.spin_until(self.first_started.is_set)
        with self.assertRaises(SurveyAbort):
            self.readiness.require_active(time.monotonic())
        self.spin_until(lambda: self.readiness.report(time.monotonic())['nodes']['fixture']['state_id'] == 3)
        self.readiness.require_active(time.monotonic())
        fresh = self.readiness.report(time.monotonic())['nodes']['fixture']
        self.assertEqual(fresh['timeouts'], 1)
        self.assertEqual(fresh['responses'], 1)
        self.assertGreaterEqual(fresh['requests'], 2)
        self.assertIn(self.client.futures[0], self.client.removed)
        self.assertTrue(self.client.futures[0].cancelled(), 'Real original future must be retired')
        self.assertTrue(self.client.futures[1].done())
        self.assertEqual(self.client.futures[1].result().current_state.id, 3)
        self.assertLessEqual(fresh['state_request_age_s'], 2.0)

        with self.lock:
            first_delay_deadline = self.server_trace[0]['started_monotonic_s'] + 1.1
        while time.monotonic() < first_delay_deadline:
            self.executor.spin_once(timeout_sec=.01)
            self.readiness.poll(time.monotonic())
            self.readiness.require_active(time.monotonic())
        self.first_gate.set()
        self.spin_until(self.first_returned.is_set, timeout=.75)
        # Continue real DDS dispatch and production polling after the original
        # server callback returned. No later successful reply can mask it.
        deadline = time.monotonic() + .25
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=.01)
            self.readiness.poll(time.monotonic())
            self.readiness.require_active(time.monotonic())
        late = self.readiness.report(time.monotonic())['nodes']['fixture']
        self.assertEqual(late['state_id'], 3)
        self.assertEqual(late['responses'], 1, 'Only the second real response may be accepted')
        self.assertEqual(late['timeouts'], 1)
        with self.lock:
            first, second = dict(self.server_trace[0]), dict(self.server_trace[1])
        self.assertEqual(first['returned_state'], 2)
        self.assertEqual(second['returned_state'], 3)
        self.assertGreater(first['returned_monotonic_s'] - first['started_monotonic_s'], 1.0)
        self.assertGreater(first['returned_monotonic_s'], second['returned_monotonic_s'])
        print('LIFECYCLE_DDS_RETRY ' + json.dumps(self.snapshot()))

    def test_fresh_repeated_inactive_responses_never_authorize_preflight(self):
        self.create_service(delay_first=False)
        self.spin_until(lambda: self.readiness.report(time.monotonic())['nodes']['fixture']['responses'] >= 2)
        detail = self.readiness.report(time.monotonic())['nodes']['fixture']
        self.assertTrue(detail['service_ready'])
        self.assertEqual(detail['state_id'], 2)
        self.assertLessEqual(detail['state_request_age_s'], 2.0)
        with self.assertRaises(SurveyAbort):
            self.readiness.require_active(time.monotonic())
        with self.lock:
            returned = [row['returned_state'] for row in self.server_trace if 'returned_state' in row]
        self.assertGreaterEqual(len(returned), 2)
        self.assertTrue(all(state == 2 for state in returned))
        print('LIFECYCLE_DDS_INACTIVE ' + json.dumps(self.snapshot()))


if __name__ == '__main__':
    unittest.main()
