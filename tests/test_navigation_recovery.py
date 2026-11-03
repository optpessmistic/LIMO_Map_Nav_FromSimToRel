"""Actual adapter with existing synthetic ROS/action fixture; no ROS processes.

Controlled wall/ROS time and TF stamps test dispatch and budget contracts, not
TF transport latency, physical stopping, or successful navigation.
"""
import ast
import copy
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch

import test_observation_adapter as t


class NavigationRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.f = t.AdapterTests()
        self.f.setUp()

    def mission(self, **overrides):
        m = self.f.mission(**{'navigation_failure_cooldown_s': .5, **overrides})
        m.fixture_map_odom_ns = t.TIME.ns
        def lookup(parent, child, when):
            self.assertEqual((parent, child), ('map', 'odom'))
            return S(header=S(stamp=S(sec=m.fixture_map_odom_ns//10**9,
                                     nanosec=m.fixture_map_odom_ns % 10**9)))
        m.tf_buffer.lookup_transform = Mock(side_effect=lookup)
        return m

    def send(self, m, kind='frontier'):
        goal = t.Goal(1.8, 1.55, .2, kind, (1.8, 1.55))
        self.assertTrue(m.send_goal(goal))
        handle = t.Handle(m.token)
        m.nav.sent[-1][1].resolve(handle)
        return handle

    def result(self, handle, status=6, payload=None):
        handle.result_future.resolve(S(status=status, result=payload if payload is not None else S()))

    def test_disabled_default_retains_immediate_next_observation_dispatch(self):
        m = self.mission(navigation_failure_cooldown_s=0.)
        self.f.start(m)
        self.f.terminal(m, status=6)
        self.assertIsNone(m.navigation_recovery)
        m.tf_buffer.lookup_transform.assert_not_called()
        self.assertEqual(len(m.nav.sent), 2)
        self.assertEqual(m.observation_queue.dispatches, 2)

    def test_finite_parameter_bounds_and_enabled_wall_timer_without_inspection(self):
        for overrides in ({'navigation_failure_cooldown_s': value} for value in (-1., float('nan'), float('inf'), True, 3.)):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                self.mission(**overrides)
        for value in (0., -1., float('nan'), float('inf'), True, 30.01):
            with self.subTest(timeout=value), self.assertRaises(ValueError):
                self.mission(navigation_recovery_timeout_s=value)
        with patch.object(t.FakeNode, 'create_timer', return_value=S()) as timer:
            m = t.adapter.MissionNode(parameter_overrides={'navigation_failure_cooldown_s': .5})
            self.assertIsNone(m.inspection)
            self.assertIn('clock', timer.call_args.kwargs)
            self.assertEqual(timer.call_args.args[0], .5)
        with patch.object(t.FakeNode, 'create_timer', return_value=S()) as timer:
            t.adapter.MissionNode()
            self.assertNotIn('clock', timer.call_args.kwargs)

    def test_observation_wait_consumes_neither_queue_attempt_nor_nav_ownership(self):
        m = self.mission()
        self.f.start(m)
        self.f.terminal(m, status=6)
        baseline = copy.deepcopy(m.navigation_recovery)
        original = (m.token, len(m.nav.sent), m.observation_queue.dispatches, m.started,
                    m.observation_deadline_wall, m.return_attempts)
        # Callback and ordinary tick both use the failure-time stamp, not a
        # pre-failure sample. An early advancement alone cannot skip cooldown.
        m.fixture_map_odom_ns += 1
        self.f.tick(m, .25, ros=False)
        self.assertIsNone(m.observation_active)
        self.assertIsNone(m.goal_handle)
        self.assertFalse(m.send_pending)
        self.assertEqual((m.token, len(m.nav.sent), m.observation_queue.dispatches, m.started,
                          m.observation_deadline_wall, m.return_attempts), original)
        self.assertEqual(m.navigation_recovery['deadline_wall'], baseline['deadline_wall'])
        self.f.tick(m, .25, ros=False)
        self.assertIsNone(m.navigation_recovery)
        self.assertEqual(len(m.nav.sent), 2)
        self.assertEqual(m.observation_queue.dispatches, 2)

    def test_cooldown_alone_and_backward_stamp_cannot_dispatch(self):
        m = self.mission()
        self.result(self.send(m))
        original_token = m.token
        t.TIME.advance(.5, ros=False)
        for stamp in (m.fixture_map_odom_ns, m.fixture_map_odom_ns-1):
            m.fixture_map_odom_ns = stamp
            self.assertFalse(m.send_goal(t.Goal(*m.home, 'home', m.home[:2])))
            self.assertEqual(m.token, original_token)
            self.assertEqual(m.return_attempts, 0)
            self.assertEqual(len(m.nav.sent), 1)
        m.fixture_map_odom_ns = m.navigation_recovery['failed_map_odom_stamp_ns']+1
        self.assertTrue(m.send_goal(t.Goal(*m.home, 'home', m.home[:2])))
        self.assertEqual(m.return_attempts, 1)

    def test_same_tick_frontier_failure_cannot_plan_or_redispatch(self):
        m = self.mission()
        self.result(self.send(m))
        with patch.object(m.planner, 'select', side_effect=AssertionError('planning while recovery waits')):
            self.f.tick(m)
        self.assertEqual(m.phase, 'exploring')
        self.assertEqual(len(m.nav.sent), 1)
        self.assertEqual(len(m.failures.entries), 1)
        self.assertIsNotNone(m.navigation_recovery)

    def test_paused_ros_clock_hits_original_three_second_wall_limit(self):
        m = self.mission()
        self.f.start(m)
        self.f.terminal(m, status=6)
        stamp = t.TIME.ns
        deadline = m.navigation_recovery['deadline_wall']
        self.f.tick(m, 2.99, ros=False)
        self.assertEqual(m.phase, 'observation_select')
        # A late ready transform cannot revive an already expired recovery.
        m.fixture_map_odom_ns += 1
        self.f.tick(m, deadline-t.TIME.wall, ros=False)
        self.assertEqual(t.TIME.ns, stamp)
        self.assertEqual(m.reason, 'navigation_recovery_timeout')
        self.assertEqual(m.cancel_next, 'blocked')
        self.assertEqual(len(m.nav.sent), 1)
        self.assertEqual(m.observation_queue.dispatches, 1)
        self.f.tick(m, ros=False)
        self.assertEqual(m.phase, 'blocked')

    def test_mission_and_return_deadlines_win_without_resetting_recovery(self):
        for budget in ('mission', 'return', 'observation'):
            with self.subTest(budget=budget):
                self.setUp()
                m = self.mission()
                self.f.start(m)
                self.f.terminal(m, status=6)
                recovery = copy.deepcopy(m.navigation_recovery)
                if budget == 'mission':
                    m.started = -m.cfg['mission_timeout_s']+.2
                    expected = 'mission_timeout_incomplete'
                elif budget == 'return':
                    m.return_started, m.return_deadline_wall = 0., .2
                    expected = 'return_total_timeout'
                else:
                    m.observation_deadline_wall = .2
                    expected = 'bounded_observation_wall_timeout'
                started, return_started, deadline = m.started, m.return_started, m.return_deadline_wall
                self.f.tick(m, .3, ros=False)
                self.assertEqual(m.reason, expected)
                self.assertEqual(m.navigation_recovery, recovery)
                self.assertEqual((m.started, m.return_started, m.return_deadline_wall),
                                 (started, return_started, deadline))
                self.assertEqual(len(m.nav.sent), 1)

    def test_wait_keeps_existing_tf_and_clock_failures_active(self):
        for cause in ('stale_base_tf', 'clock_reset'):
            with self.subTest(cause=cause):
                self.setUp()
                m = self.mission()
                self.f.start(m)
                self.f.terminal(m, status=6)
                if cause == 'stale_base_tf':
                    m.get_pose = Mock(side_effect=ValueError(cause))
                    expected = cause
                else:
                    t.TIME.ns -= 2_000_000_000
                    expected = 'ros_clock_reset_requires_new_mission'
                self.f.tick(m)
                self.assertEqual(m.reason, expected)
                self.assertEqual(m.cancel_next, 'blocked')
                self.assertEqual(len(m.nav.sent), 1)

    def test_tf_lookup_loss_while_waiting_uses_existing_block_reason(self):
        m = self.mission()
        self.result(self.send(m))
        m.tf_buffer.lookup_transform.side_effect = LookupError('map odom unavailable')
        self.assertFalse(m.navigation_recovery_allows_dispatch())
        self.assertEqual(m.reason, 'map odom unavailable')
        self.assertEqual(m.cancel_next, 'blocked')

    def test_known_rejection_arms_recovery_but_unknown_terminal_keeps_handle(self):
        m = self.mission()
        self.assertTrue(m.send_goal(t.Goal(*m.home, 'home', m.home[:2])))
        m.nav.sent[-1][1].resolve(S(accepted=False))
        self.assertEqual(m.navigation_recovery['detail'], 'goal_rejected')
        self.assertFalse(m.send_pending)
        self.assertIsNone(m.goal_handle)
        self.setUp()
        m = self.mission()
        handle = self.send(m)
        self.result(handle, status=0)
        self.assertIsNone(m.navigation_recovery)
        self.assertIs(m.goal_handle, handle)
        self.assertEqual(m.reason, 'navigation_terminal_status_unverified:0')
        m.tf_buffer.lookup_transform.assert_not_called()

    def test_result_exception_cannot_release_ownership_or_start_recovery(self):
        m = self.mission()
        handle = self.send(m)
        future = S(result=Mock(side_effect=RuntimeError('transport lost')))
        m.on_goal_result(future, m.token)
        self.assertIs(m.goal_handle, handle)
        self.assertIsNone(m.navigation_recovery)
        self.assertEqual(m.reason, 'navigation_result_unavailable:transport lost')

    def test_pending_timeout_then_rejection_also_requires_recovery(self):
        m = self.mission()
        self.assertTrue(m.send_goal(t.Goal(1.8, 1.55, .2, 'frontier', (1.8, 1.55))))
        m.cancel_to('exploring', 'goal_timeout')
        self.assertFalse(m.cancel_terminal)
        m.nav.sent[-1][1].resolve(S(accepted=False))
        self.assertTrue(m.cancel_terminal and m.cancel_response)
        self.assertEqual(m.navigation_recovery['detail'], 'goal_rejected')
        self.f.tick(m)  # Retire cancellation before any new dispatch.
        self.f.tick(m)
        self.assertEqual(m.phase, 'exploring')
        self.assertEqual(len(m.nav.sent), 1)

    def test_tf_lookup_and_ready_log_cannot_cross_recovery_deadline_and_dispatch(self):
        for slow_part in ('lookup', 'log'):
            with self.subTest(slow_part=slow_part):
                self.setUp()
                m = self.mission()
                self.result(self.send(m))
                t.TIME.advance(.5, ros=False)
                m.fixture_map_odom_ns += 1
                if slow_part == 'lookup':
                    original = m.tf_buffer.lookup_transform.side_effect
                    def slow(*args):
                        result = original(*args)
                        t.TIME.advance(2.5, ros=False)
                        return result
                    m.tf_buffer.lookup_transform.side_effect = slow
                else:
                    original = m.event
                    def slow(name, **fields):
                        original(name, **fields)
                        if name == 'navigation_recovery_ready':
                            t.TIME.advance(2.5, ros=False)
                    m.event = slow
                self.assertFalse(m.send_goal(t.Goal(*m.home, 'home', m.home[:2])))
                self.assertEqual(m.reason, 'navigation_recovery_timeout')
                self.assertEqual((len(m.nav.sent), m.token, m.return_attempts), (1, 1, 0))

    def test_original_deadlines_crossed_during_lookup_precede_queue_consumption(self):
        for budget, reason in (('mission', 'mission_timeout_incomplete'),
                               ('return', 'return_total_timeout'),
                               ('observation', 'bounded_observation_wall_timeout')):
            with self.subTest(budget=budget):
                self.setUp()
                m = self.mission()
                self.f.start(m)
                self.f.terminal(m, status=6)
                if budget == 'mission':
                    m.started = .75-m.cfg['mission_timeout_s']
                elif budget == 'return':
                    m.return_started, m.return_deadline_wall = 0., .75
                else:
                    m.observation_deadline_wall = .75
                t.TIME.advance(.5, ros=False)
                m.fixture_map_odom_ns += 1
                original = m.tf_buffer.lookup_transform.side_effect
                def slow(*args):
                    result = original(*args)
                    t.TIME.advance(.4, ros=False)
                    return result
                m.tf_buffer.lookup_transform.side_effect = slow
                m.step_observation(t.TIME.wall, m.synthetic_pose)
                self.assertEqual(m.reason, reason)
                self.assertEqual(m.observation_queue.dispatches, 1)
                self.assertIsNone(m.observation_active)
                self.assertEqual(len(m.nav.sent), 1)

    def test_final_budget_check_after_gate_precedes_send_or_queue_consumption(self):
        for entry in ('send', 'queue'):
            with self.subTest(entry=entry):
                self.setUp()
                m = self.mission()
                self.f.start(m)
                self.f.terminal(m, status=6)
                t.TIME.advance(.5, ros=False)
                m.fixture_map_odom_ns += 1
                if entry == 'send':
                    m.phase, m.return_started, m.return_deadline_wall = 'returning', 0., .75
                    expected = 'return_total_timeout'
                else:
                    m.observation_deadline_wall = .75
                    expected = 'bounded_observation_wall_timeout'
                original = m.navigation_recovery_allows_dispatch
                def scheduled_after_gate(*args):
                    result = original(*args)
                    t.TIME.advance(.3, ros=False)
                    return result
                m.navigation_recovery_allows_dispatch = scheduled_after_gate
                if entry == 'send':
                    self.assertFalse(m.send_goal(t.Goal(*m.home, 'home', m.home[:2])))
                else:
                    m.step_observation(t.TIME.wall, m.synthetic_pose)
                self.assertEqual(m.reason, expected)
                self.assertEqual((len(m.nav.sent), m.token, m.return_attempts), (1, 1, 0))
                self.assertEqual(m.observation_queue.dispatches, 1)
                self.assertIsNone(m.observation_active)

    def test_success_and_deliberate_cancel_do_not_arm_but_timeout_cancel_does(self):
        for status, reason, recover in ((4, None, False), (5, 'mission_timeout_incomplete', False),
                                         (5, 'goal_timeout', True), (5, None, True)):
            with self.subTest(status=status, reason=reason):
                self.setUp()
                m = self.mission()
                handle = self.send(m)
                if reason is not None:
                    m.cancel_to('exploring' if reason == 'goal_timeout' else 'returning', reason)
                self.result(handle, status=status)
                self.assertEqual(m.navigation_recovery is not None, recover)
                self.assertIsNone(m.goal_handle)
                if reason is not None:
                    self.assertTrue(m.cancel_terminal)
                    self.assertEqual(handle.cancel_calls, 1)

    def test_structured_errors_and_duplicate_callback_do_not_extend_budget(self):
        m = self.mission()
        handle = self.send(m)
        self.result(handle, payload=S(error_code=309, error_msg='fixture controller error'))
        event = next(e for e in m.events if e['event'] == 'goal_terminal')
        self.assertEqual((event['status'], event['error_code'], event['error_msg']),
                         (6, 309, 'fixture controller error'))
        original = copy.deepcopy(m.navigation_recovery)
        t.TIME.advance(.2)
        m.fixture_map_odom_ns += 100
        self.result(handle, payload=S())
        self.assertEqual(m.navigation_recovery, original)
        event = [e for e in m.events if e['event'] == 'goal_terminal'][-1]
        self.assertIsNone(event['error_code'])
        self.assertIsNone(event['error_msg'])

    def test_launch_declares_and_forwards_typed_opt_in_arguments(self):
        source = Path(__file__).resolve().parents[1]/'src/limo_maze_mission/launch/mission.launch.py'
        tree = ast.parse(source.read_text(encoding='utf-8'))
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef))
        env = {'LaunchDescription': list, 'DeclareLaunchArgument': lambda *a, **kw: ('argument', a, kw),
               'LaunchConfiguration': lambda name: ('configuration', name),
               'ParameterValue': lambda value, **kw: ('parameter', value, kw),
               'Node': lambda **kw: ('node', kw)}
        exec(compile(ast.Module(body=[function], type_ignores=[]), str(source), 'exec'), env)
        description = env['generate_launch_description']()
        arguments = {item[1][0]: item[2]['default_value'] for item in description if item[0] == 'argument'}
        params = next(item[1]['parameters'][0] for item in description if item[0] == 'node')
        for name, default in (('navigation_failure_cooldown_s', '0.0'), ('navigation_recovery_timeout_s', '3.0')):
            self.assertEqual(arguments[name], default)
            self.assertEqual(params[name], ('parameter', ('configuration', name), {'value_type': float}))


if __name__ == '__main__':
    unittest.main()
