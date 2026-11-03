"""Pure regressions for independent-review defects; no ROS imports or processes."""
import json
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

import test_observation_adapter as t


class ReviewRepairTests(unittest.TestCase):
    def setUp(self):
        self.f = t.AdapterTests()
        self.f.setUp()

    def partial_window(self, count=1):
        m = self.f.mission()
        self.f.start(m)
        self.f.terminal(m)
        self.f.settle(m)
        for seq in range(1, count+1):
            t.TIME.advance(.12)
            self.f.odom(m)
            m.on_frame_processing(S(data=json.dumps(t.frame(t.TIME.ns-20_000_000, seq=seq))))
        self.assertEqual(m.inspection.accepted_frames, count)
        return m

    def shutdown(self, m, terminal=True):
        callbacks, handles = {}, []
        if m.goal_handle is not None:
            handles.append(m.goal_handle)
        def subscription(_type, _topic, callback, _qos):
            callbacks['odom'] = callback
            return S()
        def spin_once(_node, timeout_sec):
            t.TIME.advance(max(timeout_sec, .01))
            if m.send_pending:
                handle = t.Handle(m.token)
                handles.append(handle)
                m.nav.sent[-1][1].resolve(handle)
            if handles and not handles[-1].cancel_future.done:
                handles[-1].cancel_future.resolve(S(goals_canceling=[S(goal_id=handles[-1].goal_id)]))
            if terminal and handles and not handles[-1].result_future.done:
                handles[-1].result_future.resolve(S(status=5, result=S(error_msg='owned canceled')))
            stamp = S(sec=t.TIME.ns//10**9, nanosec=t.TIME.ns%10**9)
            zero = S(x=0., y=0., z=0.)
            callbacks['odom'](S(header=S(stamp=stamp), twist=S(twist=S(linear=zero, angular=zero))))
        m.create_subscription = subscription
        m.destroy_subscription = lambda _: None
        with tempfile.TemporaryDirectory() as directory:
            m.cfg['output_dir'] = directory
            with patch.object(t.adapter, 'rclpy', S(ok=lambda: True, spin_once=spin_once), create=True), \
                 patch.object(t.adapter.time, 'strftime', lambda _: 'repair_fixture', create=True):
                evidence = t.adapter.shutdown_mission(m, 'operator_sigint')
        return evidence, handles

    def test_map_growth_exceeding_budget_closes_without_executor_error_or_refill(self):
        m = self.f.mission(observation_max_grid_cells=900)
        self.f.start(m)
        queue, deadline = m.observation_queue, m.observation_deadline_wall
        m.grid = t.Grid(31, 30, .1, 0., 0., 0., tuple(
            100 if x in (0, 30) or y in (0, 29) else 0 for y in range(30) for x in range(31)))
        self.f.terminal(m, status=6)
        self.assertTrue(m.observation_closed)
        self.assertEqual(m.cancel_next, 'blocked')
        self.assertEqual(m.reason, 'observation_planning_invalid_or_over_budget')
        self.assertIs(m.observation_queue, queue)
        self.assertEqual(m.observation_deadline_wall, deadline)
        self.assertEqual(queue.dispatches, 1)
        self.assertEqual(len(m.nav.sent), 1)
        self.assertEqual(m.observation_attempt_records[0]['outcome'], 'navigation_failed')
        self.f.tick(m)
        self.assertEqual(m.phase, 'blocked')

    def test_changed_or_missing_epoch_preserves_partial_and_satisfied_old_evidence(self):
        for epoch in ('map-b', ''):
            for count in (1, 3):
                with self.subTest(epoch=epoch, accepted=count):
                    self.setUp()
                    m = self.partial_window(count)
                    before = m.inspection_snapshot()
                    m.on_epoch(S(data=epoch))
                    record = m.observation_attempt_records[0]
                    self.assertEqual(record['epoch'], 'map-a')
                    self.assertEqual(record['inspection'], before)
                    self.assertEqual(m.last_inspection_failure['evidence'], before)
                    self.assertEqual(record['outcome'], 'inspection_failed')
                    self.assertEqual(m.cancel_next, 'blocked')
                    self.assertFalse(m.observation_queue.snapshot()['all_planned_actions_completed'])
                    self.assertEqual(len(m.nav.sent), 1)
                    self.f.tick(m)
                    self.assertEqual(m.phase, 'blocked')

    def test_failure_does_not_attach_ledger_from_other_epoch_or_request(self):
        for field in ('epoch', 'request_id'):
            with self.subTest(field=field):
                self.setUp()
                m = self.partial_window()
                setattr(m.inspection, field, 'other-identity')
                m.block('synthetic_identity_failure')
                self.assertIsNone(m.observation_attempt_records[0]['inspection'])
                self.assertEqual(m.observation_attempt_records[0]['outcome'], 'inspection_failed')
                self.assertEqual(m.cancel_next, 'blocked')

    def test_shutdown_terminal_window_preserves_frames_and_retires_without_new_goal(self):
        m = self.partial_window()
        before = m.inspection_snapshot()
        evidence, _ = self.shutdown(m)
        self.assertTrue(evidence['goal_terminal_verified'])
        self.assertTrue(evidence['stop_verified'])
        self.assertEqual(evidence['errors'], [])
        snapshot = m.saved_results[-1][2]
        self.assertTrue(snapshot['closed'])
        self.assertIsNone(snapshot['active'])
        self.assertEqual(len(snapshot['attempts']), 1)
        self.assertEqual(snapshot['attempts'][0]['inspection'], before)
        self.assertEqual(snapshot['attempts'][0]['outcome'], 'inspection_failed')
        self.assertFalse(snapshot['queue']['all_planned_actions_completed'])
        self.assertEqual(len(m.nav.sent), 1)

    def test_shutdown_accepted_and_late_accepted_goals_retire_only_after_terminal(self):
        for pending in (False, True):
            with self.subTest(pending=pending):
                self.setUp()
                m = self.f.mission()
                self.f.start(m)
                if not pending:
                    m.nav.sent[-1][1].resolve(t.Handle(m.token))
                evidence, handles = self.shutdown(m)
                self.assertTrue(evidence['goal_terminal_verified'])
                self.assertTrue(evidence['stop_verified'])
                self.assertEqual(evidence['errors'], [])
                self.assertEqual(handles[0].cancel_calls, 1)
                snapshot = m.saved_results[-1][2]
                self.assertTrue(snapshot['closed'])
                self.assertIsNone(snapshot['active'])
                self.assertEqual(snapshot['attempts'][0]['nav_terminal_status'], 5)
                self.assertEqual(snapshot['attempts'][0]['outcome'], 'canceled')
                self.assertIsNone(snapshot['attempts'][0]['inspection'])
                self.assertEqual(len(m.nav.sent), 1)

    def test_shutdown_ack_only_keeps_unresolved_active_and_never_dispatches(self):
        m = self.f.mission(cancel_timeout_s=.8)
        self.f.start(m)
        m.nav.sent[-1][1].resolve(t.Handle(m.token))
        evidence, handles = self.shutdown(m, terminal=False)
        self.assertFalse(evidence['goal_terminal_verified'])
        self.assertFalse(evidence['stop_verified'])
        self.assertTrue(evidence['cancel_ack_received'])
        self.assertEqual(handles[0].cancel_calls, 1)
        snapshot = m.saved_results[-1][2]
        self.assertTrue(snapshot['closed'])
        self.assertTrue(snapshot['queue']['cancel_required'])
        self.assertIsNotNone(snapshot['active'])
        self.assertEqual(snapshot['attempts'], [])
        self.assertIsNone(snapshot['active']['nav_terminal_status'])
        self.assertEqual(len(m.nav.sent), 1)

    def test_shutdown_audit_exception_cannot_prevent_owned_cancel(self):
        m = self.f.mission()
        self.f.start(m)
        handle = t.Handle(m.token)
        m.nav.sent[-1][1].resolve(handle)
        with patch.object(m, 'close_observation', side_effect=ValueError('synthetic audit failure')):
            evidence, _ = self.shutdown(m)
        self.assertEqual(handle.cancel_calls, 1)
        self.assertTrue(evidence['goal_terminal_verified'])
        self.assertTrue(evidence['stop_verified'])
        self.assertTrue(any(e.startswith('close_observation:') for e in evidence['errors']))
        self.assertIsNotNone(m.observation_active)
        self.assertEqual(m.observation_attempt_records, [])
        self.assertEqual(len(m.nav.sent), 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
