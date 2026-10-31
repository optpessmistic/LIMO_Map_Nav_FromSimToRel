"""Pure continuation regressions: actual adapter/Planner/Ledger, synthetic IO."""
import copy
import json
from types import SimpleNamespace as S
import unittest

import test_observation_adapter as t


class ContinuationTests(unittest.TestCase):
    def setUp(self):
        self.f = t.AdapterTests()
        self.f.setUp()

    def frontier_map(self, m):
        # An actual newly exposed frontier, with safe connected standoff cells.
        m.grid = t.Grid(30, 30, .1, 0., 0., 0., tuple(
            100 if x in (0, 29) or y in (0, 29) else (-1 if x >= 24 else 0)
            for y in range(30) for x in range(30)))

    def close_views_and_arrive(self, **overrides):
        m = self.f.mission(home_stable_s=.2, **overrides)
        self.f.start(m)
        for _ in range(2):
            self.f.terminal(m, status=6)
        self.f.tick(m)
        self.f.tick(m)
        self.assertEqual(m.goal.kind, 'home')
        self.f.terminal(m)
        self.assertEqual(m.phase, 'home_verification')
        return m

    def verify_home(self, m):
        for _ in range(20):
            t.TIME.advance(.1)
            self.f.odom(m)
            self.f.tick(m)
            if m.phase != 'home_verification':
                return
        self.fail('home verification did not finish')

    def timed_window(self, frames=0, **overrides):
        m = self.f.mission(inspection_timeout_s=2., **overrides)
        self.f.start(m)
        self.f.terminal(m)
        self.f.settle(m)
        for seq in range(1, frames+1):
            t.TIME.advance(.12)
            self.f.odom(m)
            m.on_frame_processing(S(data=json.dumps(t.frame(t.TIME.ns-20_000_000, seq=seq))))
        return m

    def approach_timeout(self, m):
        while t.TIME.wall+.100001 < m.inspection_deadline:
            t.TIME.advance(.1)
            self.f.odom(m)
            self.f.tick(m)
        t.TIME.advance(m.inspection_deadline-t.TIME.wall+.001)
        self.f.odom(m)
        m.map_received = t.TIME.wall

    def test_new_reachable_frontier_resumes_without_reopening_queue_or_resetting_budgets(self):
        m = self.close_views_and_arrive()
        queue = m.observation_queue
        deadline, return_deadline, started = m.observation_deadline_wall, m.return_deadline_wall, m.started
        records = copy.deepcopy(m.observation_attempt_records)
        # A still unresolved failure elsewhere must not be cleared by resumption.
        m.failures.fail(t.Goal(10., 10., 0., 'coverage', (10., 10.)), t.TIME.wall, 'prior')
        failures = copy.deepcopy(m.failures.entries)
        self.frontier_map(m)
        self.verify_home(m)
        self.assertEqual(m.phase, 'exploring')
        self.assertEqual(m.home_frontier_resumes, 1)
        self.assertTrue(m.home_frontier_resume_active)
        self.assertFalse(m.saved_results)
        self.assertEqual(m.return_attempts, 1)
        self.assertIs(m.observation_queue, queue)
        self.assertTrue(m.observation_closed)
        self.assertEqual(queue.dispatches, 2)
        self.assertEqual(m.observation_deadline_wall, deadline)
        self.assertEqual(m.return_deadline_wall, return_deadline)
        self.assertEqual(m.started, started)
        self.assertEqual(m.observation_attempt_records, records)
        self.assertEqual(m.failures.entries, failures)
        self.f.tick(m)
        self.assertEqual(m.goal.kind, 'frontier')
        self.assertTrue(m.send_pending)

    def test_early_mapping_return_then_home_resume_allows_first_observation_stage(self):
        m = self.f.mission(home_stable_s=.2)
        complete = m.grid
        values = list(m.grid.data)
        for x in range(20, 29):
            for y in range(1, 29):
                values[y*30+x] = 0 if y == 15 else 100
        values[15*30+28] = -1
        m.grid = t.Grid(30, 30, .1, 0., 0., 0., tuple(values))
        self.f.tick(m)
        self.assertEqual(m.reason, 'unreachable_frontiers_or_coverage_incomplete')
        self.assertTrue(m.observation_closed)
        self.assertIsNone(m.observation_queue)
        self.f.tick(m); self.f.tick(m); self.f.terminal(m)
        self.assertEqual(m.phase, 'home_verification')
        deadline, started, return_started = m.return_deadline_wall, m.started, m.return_started
        self.frontier_map(m)
        self.verify_home(m)
        self.assertEqual(m.phase, 'exploring')
        self.assertFalse(m.observation_closed)
        self.assertIsNone(m.observation_queue)
        self.f.tick(m)
        self.assertEqual(m.goal.kind, 'frontier')
        self.f.terminal(m); self.f.settle(m)
        # New observed map resolves the anchor before its successful frame
        # window is consumed; no unknown region is erased by production code.
        m.grid = complete
        self.f.processed_window(m)
        self.assertEqual(m.phase, 'observation_select')
        self.assertIsNotNone(m.observation_queue)
        self.assertEqual(m.observation_queue.dispatches, 0)
        self.assertEqual(m.started, started)
        self.assertEqual(m.return_started, return_started)
        self.assertEqual(m.return_deadline_wall, deadline)
        self.assertEqual(m.return_attempts, 1)
        self.assertEqual(m.home_frontier_resumes, 1)
        self.assertLessEqual(m.observation_deadline_wall, deadline-m.cfg['home_frontier_return_reserve_s'])
        self.assertLessEqual(m.observation_deadline_wall, started+m.cfg['mission_timeout_s'])
        self.assertFalse(m.visual_complete)
        self.f.tick(m)
        self.assertEqual(m.goal.kind, 'observation_queue')
        self.assertEqual(m.observation_queue.dispatches, 1)

    def test_fallback_parameter_reaches_planner_and_invalid_bounds_fail_closed(self):
        m = self.f.mission(frontier_fallback_standoff_m=.9)
        self.assertEqual(m.cfg['frontier_fallback_standoff_m'], .9)
        self.assertEqual(m.planner.frontier_fallback_standoff_m, .9)
        for value in (.54, 1.01, float('nan'), float('inf'), True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.f.mission(frontier_fallback_standoff_m=value)

    def test_second_mapping_completion_returns_without_new_views_or_time_reset(self):
        m = self.close_views_and_arrive()
        original_map = m.grid
        self.frontier_map(m)
        self.verify_home(m)
        return_started, deadline = m.return_started, m.return_deadline_wall
        attempts = copy.deepcopy(m.observation_attempt_records)
        # A map update resolves the frontier before another goal is dispatched.
        m.grid = original_map
        self.f.tick(m)
        self.assertEqual(m.cancel_next, 'returning')
        self.f.tick(m)
        self.f.tick(m)
        self.assertEqual(m.goal.kind, 'home')
        self.assertEqual(m.return_attempts, 2)
        self.assertEqual(m.return_started, return_started)
        self.assertEqual(m.return_deadline_wall, deadline)
        self.assertEqual(m.observation_attempt_records, attempts)
        self.assertEqual(m.observation_queue.dispatches, 2)
        self.f.terminal(m)
        self.verify_home(m)
        self.assertEqual(m.phase, 'done')
        self.assertFalse(m.visual_complete)

    def test_resume_denied_for_each_budget_without_fabricating_map_completion(self):
        for budget in ('limit', 'mission', 'reserve', 'return_attempts'):
            with self.subTest(budget=budget):
                self.setUp()
                m = self.close_views_and_arrive()
                self.frontier_map(m)
                if budget == 'limit':
                    m.home_frontier_resumes = m.cfg['home_frontier_resume_limit']
                elif budget == 'mission':
                    m.started = t.TIME.wall-m.cfg['mission_timeout_s']
                elif budget == 'reserve':
                    m.return_deadline_wall = t.TIME.wall+m.cfg['home_frontier_return_reserve_s']
                else:
                    m.return_attempts = m.return_attempt_limit()
                self.verify_home(m)
                self.assertEqual(m.phase, 'done')
                self.assertFalse(m.map_complete)
                self.assertFalse(m.visual_complete)
                self.assertFalse(m.home_frontier_decisions[-1]['resumed'])
                self.assertEqual(len(m.nav.sent), 3)
                self.assertTrue(m.observation_closed)

    def test_actual_second_home_cannot_exceed_resume_count(self):
        m = self.close_views_and_arrive()
        original = m.grid
        self.frontier_map(m); self.verify_home(m)
        deadline = m.return_deadline_wall
        m.grid = original
        self.f.tick(m); self.f.tick(m); self.f.tick(m)
        self.f.terminal(m)
        self.frontier_map(m); self.verify_home(m)
        self.assertEqual(m.phase, 'done')
        self.assertFalse(m.map_complete)
        self.assertEqual(m.home_frontier_resumes, 1)
        self.assertEqual([d['resumed'] for d in m.home_frontier_decisions], [True, False])
        self.assertEqual(m.home_frontier_decisions[-1]['reason'], 'home_frontier_resume_limit')
        self.assertEqual(m.return_deadline_wall, deadline)
        self.assertEqual(m.return_attempts, 2)
        self.assertEqual(m.observation_queue.dispatches, 2)

    def test_second_return_failure_cannot_replenish_total_send_attempts(self):
        m = self.close_views_and_arrive(max_goal_attempts=1)
        original = m.grid
        self.frontier_map(m); self.verify_home(m)
        m.grid = original
        self.f.tick(m); self.f.tick(m); self.f.tick(m)
        self.assertEqual(m.return_attempts, m.return_attempt_limit())
        self.f.terminal(m, status=6)
        self.assertEqual(m.cancel_next, 'blocked')
        self.f.tick(m); self.f.tick(m)
        self.assertEqual(m.phase, 'blocked')
        self.assertEqual(m.return_attempts, 2)
        self.assertEqual(len(m.nav.sent), 4)

    def test_unreachable_or_coverage_only_frontier_change_does_not_resume(self):
        for change in ('coverage', 'unreachable_frontier'):
            with self.subTest(change=change):
                self.setUp()
                m = self.close_views_and_arrive()
                m.detector_received, m.coverage.frames = t.TIME.wall, 1
                if change == 'unreachable_frontier':
                    # Narrow branch reachable as free cells but no R-safe goal.
                    cells = list(m.grid.data)
                    for x in range(20, 29):
                        for y in range(1, 29):
                            cells[y*30+x] = 0 if y == 15 else 100
                    cells[15*30+28] = -1
                    m.grid = t.Grid(30, 30, .1, 0., 0., 0., tuple(cells))
                self.verify_home(m)
                self.assertEqual(m.phase, 'done')
                self.assertEqual(m.home_frontier_resumes, 0)
                self.assertEqual(m.observation_queue.dispatches, 2)

    def test_no_resume_before_verified_home_or_on_invalid_state(self):
        m = self.close_views_and_arrive()
        self.frontier_map(m)
        m.synthetic_pose = (m.home[0]+.3, m.home[1], m.home[2])
        for _ in range(10):
            t.TIME.advance(.1); self.f.odom(m); self.f.tick(m)
        self.assertEqual(m.phase, 'home_verification')
        self.assertEqual(m.home_frontier_resumes, 0)
        self.assertEqual(len(m.nav.sent), 3)

    def test_epoch_or_clock_invalid_at_home_still_blocks(self):
        for invalid in ('epoch', 'clock'):
            with self.subTest(invalid=invalid):
                self.setUp()
                m = self.close_views_and_arrive()
                self.frontier_map(m)
                if invalid == 'epoch': m.on_epoch(S(data='map-b'))
                else: m.on_inspection_clock_jump()
                self.f.tick(m)
                if m.canceling: self.f.tick(m)
                self.assertEqual(m.phase, 'blocked')
                self.assertEqual(m.home_frontier_resumes, 0)

    def test_resumed_pending_goal_reserve_waits_for_owned_terminal_and_keeps_absolute_deadline(self):
        m = self.close_views_and_arrive()
        self.frontier_map(m); self.verify_home(m); self.f.tick(m)
        self.assertEqual(m.goal.kind, 'frontier')
        deadline, start = m.return_deadline_wall, m.return_started
        reserve_time = deadline-m.cfg['home_frontier_return_reserve_s']
        self.f.tick(m, reserve_time-t.TIME.wall+.01, ros=False)
        self.assertTrue(m.canceling)
        self.assertEqual(m.cancel_next, 'returning')
        self.assertEqual(len(m.nav.sent), 4)
        h = t.Handle(m.token); m.nav.sent[-1][1].resolve(h)
        h.cancel_future.resolve(S(goals_canceling=[S(goal_id=h.goal_id)]))
        self.f.tick(m)
        self.assertTrue(m.canceling)
        self.assertEqual(len(m.nav.sent), 4)
        h.result_future.resolve(S(status=5, result=S(error_msg='canceled')))
        self.f.tick(m); self.f.tick(m)
        self.assertEqual(len(m.nav.sent), 5)
        self.assertEqual(m.goal.kind, 'home')
        self.assertEqual(m.return_started, start)
        self.assertEqual(m.return_deadline_wall, deadline)
        self.assertEqual(m.return_attempts, 2)

    def test_absolute_deadline_while_canceling_paused_clock_upgrades_to_block_without_new_send(self):
        m = self.close_views_and_arrive(cancel_timeout_s=30.)
        self.frontier_map(m); self.verify_home(m); self.f.tick(m)
        deadline = m.return_deadline_wall
        self.f.tick(m, deadline-m.cfg['home_frontier_return_reserve_s']-t.TIME.wall+.01, ros=False)
        self.assertEqual(m.cancel_next, 'returning')
        self.f.tick(m, deadline-t.TIME.wall+.01, ros=False)
        self.assertEqual(m.cancel_next, 'blocked')
        self.assertEqual(m.reason, 'return_total_timeout')
        self.assertTrue(m.send_pending)
        self.assertEqual(len(m.nav.sent), 4)

    def test_timeout_zero_and_partial_frames_freeze_once_then_next_view(self):
        for frames in (0, 1, 2):
            with self.subTest(frames=frames):
                self.setUp()
                m = self.timed_window(frames)
                self.approach_timeout(m)
                self.f.tick(m)
                self.assertEqual(m.phase, 'observation_select')
                self.assertFalse(m.canceling)
                self.assertFalse(m.observation_closed)
                self.assertIsNone(m.observation_active)
                self.assertEqual(len(m.nav.sent), 1)
                record = copy.deepcopy(m.observation_attempt_records[0])
                self.assertEqual(record['outcome'], 'inspection_failed')
                self.assertEqual(record['inspection']['accepted_frames'], frames)
                self.assertEqual(record['inspection']['status'], 'timed_out')
                self.assertFalse(m.visual_complete)
                self.f.tick(m)
                self.assertEqual(len(m.nav.sent), 2)
                self.assertEqual(m.observation_attempt_records, [record])

    def test_late_frame_and_result_cannot_credit_next_request(self):
        m = self.timed_window(1)
        old_token = m.token
        late = t.frame(t.TIME.ns-20_000_000, seq=2)
        self.approach_timeout(m); self.f.tick(m)
        record = copy.deepcopy(m.observation_attempt_records[0])
        m.on_frame_processing(S(data=json.dumps(late)))  # Retired phase rejects immediately.
        self.f.tick(m); self.f.terminal(m); self.f.settle(m)
        self.assertNotEqual(m.inspection.request_id, record['inspection']['request_id'])
        m.on_frame_processing(S(data=json.dumps(late)))  # Before next barrier, never credited.
        future = t.Future(); future.resolve(S(status=4, result=S(error_msg='late')))
        m.on_goal_result(future, old_token)
        self.assertEqual(m.inspection.accepted_frames, 0)
        self.assertEqual(m.observation_attempt_records[0], record)
        self.f.processed_window(m)
        self.assertEqual(len(m.observation_attempt_records), 2)
        self.assertEqual(m.observation_attempt_records[1]['outcome'], 'inspection_satisfied')
        self.assertEqual(m.observation_attempt_records[0], record)

    def test_all_view_timeouts_exhaust_queue_and_return_without_certification(self):
        m = self.timed_window()
        for index in range(2):
            if index:
                self.f.tick(m); self.f.terminal(m); self.f.settle(m)
            self.approach_timeout(m); self.f.tick(m)
        self.f.tick(m)
        self.assertTrue(m.observation_closed)
        self.assertEqual(m.cancel_next, 'returning')
        self.assertEqual(m.observation_queue.dispatches, 2)
        self.assertEqual(len(m.observation_attempt_records), 2)
        self.assertTrue(all(r['outcome'] == 'inspection_failed' for r in m.observation_attempt_records))
        self.assertFalse(m.observation_queue.snapshot()['all_planned_actions_completed'])
        self.assertFalse(m.visual_complete)

    def test_safety_and_binding_failures_at_timeout_still_block(self):
        for invalid in ('pose', 'odom_stale', 'moving', 'token', 'request', 'expected_pose',
                        'epoch', 'nav_status', 'nav_uuid', 'coordinates', 'queue_dispatch'):
            with self.subTest(invalid=invalid):
                self.setUp()
                m = self.timed_window(1)
                self.approach_timeout(m)
                if invalid == 'pose': m.synthetic_pose = (m.goal.x+.1, m.goal.y, m.goal.yaw)
                elif invalid == 'odom_stale': m.inspection_odom['received'] -= 1.
                elif invalid == 'moving': self.f.odom(m, moving=True)
                elif invalid == 'token': m.inspection_goal_token += 1
                elif invalid == 'request': m.inspection.request_id = 'other'
                elif invalid == 'expected_pose': m.inspection.expected_pose = (0., 0., 0.)
                elif invalid == 'epoch': m.inspection.epoch = 'other'
                elif invalid == 'nav_status': m.observation_active['nav_terminal_status'] = 6
                elif invalid == 'nav_uuid': m.observation_active['nav_goal_uuid'] = None
                elif invalid == 'coordinates': m.coordinates_valid = False
                else: m.observation_active['dispatch_id'] += 100
                self.f.tick(m)
                self.assertTrue(m.canceling)
                self.assertEqual(m.cancel_next, 'blocked')
                self.assertEqual(len(m.nav.sent), 1)
                self.assertTrue(m.observation_closed)

    def test_unknown_navigation_ownership_cannot_use_safe_timeout_branch(self):
        for pending in (True, False):
            with self.subTest(pending=pending):
                self.setUp()
                m = self.timed_window()
                self.approach_timeout(m)
                if pending: m.send_pending = True
                else: m.goal_handle = t.Handle(m.token)
                m.update_inspection(t.TIME.wall, m.synthetic_pose)
                self.assertEqual(m.cancel_next, 'blocked')
                self.assertIsNotNone(m.observation_active)
                self.assertFalse(m.observation_attempt_records)

    def test_satisfied_ledger_after_deadline_is_not_relabelled_recoverable_timeout_or_success(self):
        m = self.timed_window(frames=3)
        self.assertEqual(m.inspection.status, 'satisfied')
        # Advance current stop evidence without letting a scheduler tick consume
        # satisfaction. Crossing the deadline cannot retroactively accept it.
        while t.TIME.wall+.100001 < m.inspection_deadline:
            t.TIME.advance(.1); self.f.odom(m)
        t.TIME.advance(m.inspection_deadline-t.TIME.wall+.001); self.f.odom(m)
        self.f.tick(m)
        self.assertEqual(m.cancel_next, 'blocked')
        self.assertEqual(m.observation_attempt_records[0]['outcome'], 'inspection_failed')
        self.assertEqual(len(m.nav.sent), 1)

    def test_clock_map_and_tf_failures_do_not_become_recoverable_image_failure(self):
        for invalid in ('clock', 'map', 'tf'):
            with self.subTest(invalid=invalid):
                self.setUp()
                m = self.timed_window()
                self.approach_timeout(m)
                if invalid == 'clock': m.on_inspection_clock_jump()
                elif invalid == 'map': m.map_received = t.TIME.wall-m.cfg['map_max_age_s']-1.
                else: m.get_pose = lambda: (_ for _ in ()).throw(LookupError('synthetic unavailable TF'))
                m.tick()
                self.assertEqual(m.cancel_next, 'blocked')
                self.assertTrue(m.observation_closed)
                self.assertEqual(len(m.nav.sent), 1)

    def test_total_or_stage_budget_wins_over_safe_image_timeout(self):
        for budget in ('mission', 'stage'):
            with self.subTest(budget=budget):
                self.setUp()
                m = self.timed_window()
                self.approach_timeout(m)
                if budget == 'mission': m.started = t.TIME.wall-m.cfg['mission_timeout_s']
                else: m.observation_deadline_wall = t.TIME.wall
                self.f.tick(m)
                self.assertEqual(m.cancel_next, 'returning')
                self.assertTrue(m.observation_closed)
                self.assertEqual(len(m.observation_attempt_records), 1)
                self.assertEqual(len(m.nav.sent), 1)

    def test_default_off_keeps_old_return_reset_and_timeout_block(self):
        m = self.f.mission(post_mapping_observation_mode='off')
        m.return_attempts = 3
        m.start_return('legacy')
        self.assertEqual(m.return_attempts, 0)
        self.f.tick(m)
        self.assertIsNone(m.return_deadline_wall)
        m = self.timed_window()
        m.cfg['post_mapping_observation_mode'] = 'off'
        self.approach_timeout(m); self.f.tick(m)
        self.assertEqual(m.cancel_next, 'blocked')

    def test_resume_limit_parameter_is_bounded_integer_zero_disables(self):
        for invalid in (-1, 9, 1.5, True):
            with self.assertRaises(ValueError):
                self.f.mission(home_frontier_resume_limit=invalid)
        m = self.close_views_and_arrive(home_frontier_resume_limit=0)
        self.frontier_map(m); self.verify_home(m)
        self.assertEqual(m.phase, 'done')
        self.assertEqual(m.home_frontier_resumes, 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
