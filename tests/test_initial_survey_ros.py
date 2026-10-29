"""Opt-in real ROS Action callbacks on domain 74; no simulator or velocity topics.

Source /opt/ros/jazzy/setup.bash first. Run from the repository root:
ROS_DOMAIN_ID=74 RUN_ROS_SURVEY_TEST=1 python3 -m unittest discover -s tests -p test_initial_survey_ros.py -v
Each run saves a new uniquely named JSON report under ignored outputs/ros_smoke/.
"""
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time
import unittest
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from initial_survey import OwnedSpin


@unittest.skipUnless(importlib.util.find_spec('rclpy') is not None, 'rclpy unavailable; source the ROS Jazzy environment')
@unittest.skipUnless(importlib.util.find_spec('nav2_msgs') is not None, 'nav2_msgs unavailable; install/source Nav2')
@unittest.skipUnless(os.environ.get('RUN_ROS_SURVEY_TEST') == '1', 'Opt-in: set RUN_ROS_SURVEY_TEST=1 and ROS_DOMAIN_ID=74')
class ActualSpinActionTests(unittest.TestCase):
    def test_real_goal_result_and_uuid_specific_cancel(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '74')
        import rclpy
        from rclpy.action import ActionClient, ActionServer, CancelResponse
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from nav2_msgs.action import Spin
        from unique_identifier_msgs.msg import UUID

        rclpy.init(domain_id=74, signal_handler_options=SignalHandlerOptions.NO)
        client_node = rclpy.create_node('survey_action_test_client')
        server_node = rclpy.create_node('survey_action_test_server')
        seen = []
        def execute(handle):
            seen.append(handle.request.target_yaw)
            if handle.request.target_yaw < 0:
                deadline = time.monotonic()+5
                while not handle.is_cancel_requested and time.monotonic() < deadline:
                    time.sleep(.01)
                if handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.abort()
            else:
                handle.succeed()
            return Spin.Result()
        name = '/initial_survey_test/spin'
        server = ActionServer(server_node, Spin, name, execute,
                              cancel_callback=lambda request: CancelResponse.ACCEPT,
                              callback_group=ReentrantCallbackGroup())
        client = ActionClient(client_node, Spin, name)
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(client_node)
        executor.add_node(server_node)
        try:
            self.assertTrue(client.wait_for_server(timeout_sec=5))
            records = []
            for target in (2*math.pi+.1, -.1):
                goal = Spin.Goal()
                goal.target_yaw = target
                goal.time_allowance.sec = 10
                goal_id = uuid.uuid4().bytes
                owned = OwnedSpin(goal_id, client.send_goal_async(goal, goal_uuid=UUID(uuid=list(goal_id))), target)
                owned.cancel_requested = target < 0  # Exercise cancellation before acceptance arrives.
                deadline = time.monotonic()+5
                while time.monotonic() < deadline and (owned.terminal is None or (target < 0 and not owned.cancel_ack)):
                    executor.spin_once(timeout_sec=.02)
                    owned.poll()
                self.assertEqual(owned.terminal, 4 if target > 0 else 5)
                if target < 0:
                    self.assertTrue(owned.cancel_ack)
                records.append(owned.record())
            self.assertEqual(len(seen), 2, 'Actual server callbacks must run')
            self.assertAlmostEqual(seen[0], 2*math.pi+.1, places=5)
            report = {'passed': True, 'ros_domain_id': 74, 'action': name,
                      'actual_server_callbacks': len(seen), 'goals': records,
                      'velocity_topics_used': False, 'physical_robot_or_simulator_used': False}
            path = Path('outputs/ros_smoke') / ('initial_survey_spin_action_regression_'+uuid.uuid4().hex+'.json')
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open('x', encoding='utf-8') as stream:
                stream.write(json.dumps(report, indent=2)+'\n')
            print(json.dumps(report))
            print('Report:', path)
        finally:
            client.destroy()
            server.destroy()
            executor.shutdown()
            client_node.destroy_node()
            server_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
