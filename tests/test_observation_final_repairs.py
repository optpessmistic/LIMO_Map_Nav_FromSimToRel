"""Actual pure adapter/Planner/Ledger regressions for both independent findings."""
import copy
import json
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

import test_observation_adapter as t
import test_observation_continuation as c


class FinalRepairTests(unittest.TestCase):
    def setUp(self):
        self.c = c.ContinuationTests()
        self.c.setUp()

    def stage_budget_retirement(self):
        m = self.c.timed_window(frames=1, observation_stage_timeout_s=1., home_stable_s=.2)
        before = copy.deepcopy(m.inspection_snapshot())
        old_frame = t.frame(t.TIME.ns-20_000_000, seq=2)
        while t.TIME.wall < m.observation_deadline_wall:
            t.TIME.advance(.1)
            self.c.f.odom(m)
            self.c.f.tick(m)
        self.assertEqual(m.cancel_next, 'returning')
        self.assertEqual(m.inspection.status, 'retired')
        self.assertTrue(m.inspection.ready)
        self.assertEqual(m.inspection.reason, 'bounded_observation_wall_timeout')
        self.assertEqual(m.observation_attempt_records[0]['inspection']['accepted_frames'], 1)
        self.assertEqual(m.observation_attempt_records[0]['inspection']['request_id'], before['request_id'])
        return m, before, old_frame

    def test_counterexample_stage_budget_retirement_allows_fresh_request_after_verified_home(self):
        m, before, old_frame = self.stage_budget_retirement()
        epoch, epoch_barrier, started = m.map_epoch, m.inspection.epoch_barrier_ns, m.started
        queue, stage_deadline = m.observation_queue, m.observation_deadline_wall
        attempts = copy.deepcopy(m.observation_attempt_records)
        m.on_frame_processing(S(data=json.dumps(old_frame)))
        self.assertEqual(m.inspection.accepted_frames, 0)
        self.c.f.tick(m)
        return_deadline = m.return_deadline_wall
        self.c.f.tick(m)
        self.assertEqual(m.goal.kind, 'home')
        self.c.f.terminal(m)
        self.c.frontier_map(m)
        self.c.verify_home(m)
        self.assertEqual(m.phase, 'exploring')
        self.assertEqual(m.home_frontier_resumes, 1)
        self.assertTrue(m.home_frontier_decisions[-1]['resumed'])
        self.assertEqual(m.return_attempts, 1)
        self.assertEqual(m.started, started)
        self.assertEqual(m.map_epoch, epoch)
        self.assertEqual(m.inspection.epoch_barrier_ns, epoch_barrier)
        self.assertEqual(m.return_deadline_wall, return_deadline)
        self.assertIs(m.observation_queue, queue)
        self.assertEqual(queue.dispatches, 1)
        self.assertEqual(m.observation_deadline_wall, stage_deadline)
        self.assertTrue(m.observation_closed)
        self.c.f.tick(m)
        self.assertEqual(m.goal.kind, 'frontier')
        self.c.f.terminal(m)
        self.assertNotEqual(m.inspection.request_id, before['request_id'])
        self.assertEqual(m.inspection.accepted_frames, 0)
        self.c.f.settle(m)
        m.on_frame_processing(S(data=json.dumps(old_frame)))
        self.assertEqual(m.inspection.accepted_frames, 0)
        self.c.f.processed_window(m)
        self.assertEqual(m.inspection.accepted_frames, 3)
        self.assertEqual(m.observation_attempt_records, attempts)
        self.assertEqual(queue.dispatches, 1)
        self.assertFalse(m.visual_complete)

    def test_counterexample_planning_crosses_absolute_deadline_without_new_frontier_send(self):
        m = self.c.close_views_and_arrive(home_frontier_return_reserve_s=.2, return_timeout_s=20.)
        self.c.frontier_map(m)
        self.c.verify_home(m)
        deadline, count, token = m.return_deadline_wall, len(m.nav.sent), m.token
        attempts = m.return_attempts
        t.TIME.advance(deadline-t.TIME.wall-.3, ros=False)
        actual = m.planner.select
        def slow_select(*args, **kwargs):
            selected = actual(*args, **kwargs)
            self.assertEqual(selected.goal.kind, 'frontier')
            t.TIME.advance(.5, ros=False)
            return selected
        with patch.object(m.planner, 'select', side_effect=slow_select):
            self.c.f.tick(m)
        self.assertGreater(t.TIME.wall, deadline)
        self.assertEqual(len(m.nav.sent), count)
        self.assertEqual(m.token, token)
        self.assertEqual(m.return_attempts, attempts)
        self.assertFalse(m.send_pending)
        self.assertTrue(m.canceling)
        self.assertEqual(m.cancel_next, 'blocked')
        self.assertEqual(m.reason, 'return_total_timeout')
        self.assertEqual(m.return_deadline_wall, deadline)
        self.c.f.tick(m)
        self.assertEqual(m.phase, 'blocked')

    def test_planning_crosses_reserve_returns_without_new_frontier_or_resetting_deadline(self):
        m = self.c.close_views_and_arrive(home_frontier_return_reserve_s=2., return_timeout_s=20.)
        self.c.frontier_map(m)
        self.c.verify_home(m)
        deadline, count, token = m.return_deadline_wall, len(m.nav.sent), m.token
        t.TIME.advance(deadline-m.cfg['home_frontier_return_reserve_s']-t.TIME.wall-.1, ros=False)
        actual = m.planner.select
        def slow_select(*args, **kwargs):
            selected = actual(*args, **kwargs)
            t.TIME.advance(.2, ros=False)
            return selected
        with patch.object(m.planner, 'select', side_effect=slow_select):
            self.c.f.tick(m)
        self.assertEqual(m.cancel_next, 'returning')
        self.assertEqual(m.reason, 'home_frontier_return_reserve_reached')
        self.assertEqual(len(m.nav.sent), count)
        self.assertEqual(m.token, token)
        self.assertFalse(m.home_frontier_resume_active)
        self.c.f.tick(m)
        self.c.f.tick(m)
        self.assertEqual(m.goal.kind, 'home')
        self.assertEqual(m.return_attempts, 2)
        self.assertEqual(m.return_deadline_wall, deadline)

    def test_planning_crosses_mission_budget_returns_without_exploration_send(self):
        m = self.c.f.mission()
        self.c.frontier_map(m)
        started = m.started
        t.TIME.advance(m.cfg['mission_timeout_s']-.1, ros=False)
        actual = m.planner.select
        def slow_select(*args, **kwargs):
            selected = actual(*args, **kwargs)
            t.TIME.advance(.2, ros=False)
            return selected
        with patch.object(m.planner, 'select', side_effect=slow_select):
            self.c.f.tick(m)
        self.assertTrue(m.mission_timed_out)
        self.assertEqual(m.cancel_next, 'returning')
        self.assertEqual(m.started, started)
        self.assertEqual(m.token, 0)
        self.assertEqual(m.nav.sent, [])

    def test_queue_planning_crosses_stage_budget_records_unexecuted_reservation_once(self):
        m = self.c.f.mission(observation_stage_timeout_s=1.)
        self.c.f.tick(m)
        queue, deadline = m.observation_queue, m.observation_deadline_wall
        t.TIME.advance(.7)
        actual = queue.take_next
        def slow_take_next(*args, **kwargs):
            selected = actual(*args, **kwargs)
            self.assertEqual(selected.state, 'dispatch')
            t.TIME.advance(.5, ros=False)
            return selected
        with patch.object(queue, 'take_next', side_effect=slow_take_next):
            self.c.f.tick(m)
        self.assertEqual(m.nav.sent, [])
        self.assertEqual(m.token, 0)
        self.assertEqual(queue.dispatches, 1)  # Reserved finite budget is not refilled.
        self.assertEqual(m.observation_deadline_wall, deadline)
        self.assertTrue(m.observation_closed)
        self.assertIsNone(m.observation_active)
        self.assertEqual(m.cancel_next, 'returning')
        self.assertEqual(len(m.observation_attempt_records), 1)
        record = copy.deepcopy(m.observation_attempt_records[0])
        self.assertEqual(record['outcome'], 'canceled')
        self.assertIsNone(record['nav_token'])
        self.assertIsNone(record['nav_goal_uuid'])
        self.assertIsNone(record['nav_terminal_status'])
        self.assertIsNone(record['dispatched_wall'])
        self.assertIsNone(record['inspection'])
        self.c.f.tick(m)
        self.c.f.tick(m)
        self.assertEqual(m.goal.kind, 'home')
        self.assertEqual(m.return_attempts, 1)
        self.assertEqual(m.observation_attempt_records, [record])

    def test_refused_expired_home_send_does_not_consume_token_or_attempt(self):
        m = self.c.f.mission()
        m.start_return('synthetic_completed_stage')
        self.c.f.tick(m)
        t.TIME.advance(m.return_deadline_wall-t.TIME.wall+.01, ros=False)
        sent = m.send_goal(t.Goal(*m.home, 'home', m.home[:2]))
        self.assertFalse(sent)
        self.assertEqual(m.return_attempts, 0)
        self.assertEqual(m.token, 0)
        self.assertEqual(m.nav.sent, [])
        self.assertEqual(m.cancel_next, 'blocked')

    def test_final_home_planning_crossing_absolute_deadline_cannot_save_return_success(self):
        m = self.c.close_views_and_arrive(return_timeout_s=2.)
        deadline = m.return_deadline_wall
        actual = m.planner.select
        def slow_final_select(*args, **kwargs):
            selected = actual(*args, **kwargs)
            self.assertEqual(selected.frontier_count, 0)
            t.TIME.advance(deadline-t.TIME.wall+.1, ros=False)
            return selected
        with patch.object(m.planner, 'select', side_effect=slow_final_select):
            self.c.verify_home(m)
        self.assertGreater(t.TIME.wall, deadline)
        self.assertEqual(m.cancel_next, 'blocked')
        self.assertEqual(m.reason, 'return_total_timeout')
        self.assertFalse(any(result[0] for result in m.saved_results))
        self.assertEqual(m.return_deadline_wall, deadline)
        self.assertEqual(m.home_frontier_resumes, 0)
        self.c.f.tick(m)
        self.assertEqual(m.phase, 'blocked')

    def test_retirement_never_reauthorizes_invalid_epoch_or_accepts_old_request(self):
        m, _, _ = self.stage_budget_retirement()
        ledger, epoch = m.inspection, m.map_epoch
        before = ledger.snapshot()
        decision = ledger.observe(t.frame(t.TIME.ns-20_000_000), now_ns=t.TIME.ns,
            wall_now=t.TIME.wall, base_pose=m.synthetic_pose, camera_pose=(0.,0.,0.,0.,0.,0.,1.),
            tf_stamp_ns=t.TIME.ns-20_000_000)
        self.assertFalse(decision['accepted'])
        self.assertEqual(decision['reason'], 'no_active_inspection')
        self.assertEqual(ledger.status, 'retired')
        self.assertEqual(ledger.epoch, before['map_epoch'])
        ledger.invalidate('clock_reset_requires_new_epoch')
        invalid = ledger.snapshot()
        ledger.retire_request('bounded_observation_wall_timeout')
        self.assertEqual(ledger.snapshot(), invalid)
        ledger.set_epoch(epoch, t.TIME.ns)
        self.assertFalse(ledger.ready)
        with self.assertRaises(ValueError):
            ledger.begin(t.TIME.ns, m.synthetic_pose, t.TIME.wall, request_id='new')

    def test_epoch_or_clock_fault_after_benign_retirement_still_blocks(self):
        for fault in ('epoch', 'clock'):
            with self.subTest(fault=fault):
                self.setUp()
                m, _, _ = self.stage_budget_retirement()
                attempts = copy.deepcopy(m.observation_attempt_records)
                if fault == 'epoch':
                    m.on_epoch(S(data='map-b'))
                else:
                    m.on_inspection_clock_jump()
                self.c.f.tick(m)
                self.assertEqual(m.phase, 'blocked')
                self.assertEqual(m.home_frontier_resumes, 0)
                self.assertEqual(m.observation_attempt_records, attempts)
                self.assertEqual(len(m.nav.sent), 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
