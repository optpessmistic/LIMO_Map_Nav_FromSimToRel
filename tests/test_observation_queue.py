"""Synthetic observed grids only; no ROS, simulator files, target truth or camera range."""
from dataclasses import replace
from math import cos, hypot, pi, sin
import unittest
from unittest import mock

from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/limo_maze_mission'))
from limo_maze_mission.observation import Grid, ObservationQueue, Policy, core, endpoint_radius, reachable


def room(width=30, height=24, origin=(0., 0.), resolution=.1):
    return Grid(width, height, resolution, *origin, 0.,
                tuple(0 if 0 < x < width-1 and 0 < y < height-1 else 100
                      for y in range(height) for x in range(width)))


def change(grid, cells, value):
    data = list(grid.data)
    for x, y in cells:
        data[y*grid.width+x] = value
    return replace(grid, data=tuple(data))


class ObservationQueueTests(unittest.TestCase):
    def setUp(self):
        self.grid = room()
        self.pose = (*self.grid.to_world((7, 7)), .3)
        self.policy = Policy(.12, .08, .65, 4, 4, 16, 2)

    def queue(self, **policy):
        return ObservationQueue(self.grid, self.pose, 'epoch-a', replace(self.policy, **policy))

    def test_finite_deterministic_spaced_safe_reachable_views(self):
        original = self.grid.data
        queue = self.queue(max_dispatches=11)
        self.assertEqual(queue.views, self.queue(max_dispatches=11).views)
        self.assertLessEqual(len(queue.views), 11)
        self.assertEqual(len(queue.stations), 4)
        for i, a in enumerate(queue.stations):
            for b in queue.stations[i+1:]:
                self.assertGreaterEqual(hypot(a[0]-b[0], a[1]-b[1]), self.policy.station_spacing_m)
        expected = reachable(self.grid, self.pose, self.policy)
        endpoints = self.grid.safe_cells(endpoint_radius(self.grid, self.policy))
        identities = set()
        for view in queue.views.values():
            cell = self.grid.to_cell(view.goal.x, view.goal.y)
            self.assertIn(cell, expected)
            self.assertIn(cell, endpoints)
            self.assertEqual(view.goal.kind, 'observation_queue')
            identity = (view.goal.x, view.goal.y, view.goal.yaw)
            self.assertNotIn(identity, identities)
            identities.add(identity)
        self.assertEqual(self.grid.data, original)

    def test_arrival_margin_preserves_core_safe_cells_at_sampled_offsets(self):
        queue = self.queue()
        safe = self.grid.safe_cells(self.policy.robot_radius_m)
        for view in queue.views.values():
            for step in range(32):
                angle = 2*pi*step/32
                xy = (view.goal.x+self.policy.arrival_margin_m*cos(angle),
                      view.goal.y+self.policy.arrival_margin_m*sin(angle))
                self.assertIn(self.grid.to_cell(*xy), safe)

    def test_unknown_intermediate_barrier_and_disconnected_room_excluded(self):
        for value in (-1, 50, 100):
            grid = change(self.grid, [(15, y) for y in range(self.grid.height)], value)
            queue = ObservationQueue(grid, self.pose, 'epoch-a', self.policy)
            self.assertTrue(queue.views)
            self.assertTrue(all(grid.to_cell(v.goal.x, v.goal.y)[0] < 15 for v in queue.views.values()))

    def test_narrow_gap_is_not_assumed_traversable(self):
        grid = change(self.grid, [(15, y) for y in range(self.grid.height) if y != 10], 100)
        queue = ObservationQueue(grid, self.pose, 'epoch-a', self.policy)
        self.assertTrue(all(grid.to_cell(v.goal.x, v.goal.y)[0] < 15 for v in queue.views.values()))

    def test_actual_pose_offset_cannot_borrow_cell_center_clearance(self):
        # At x=.25 the center disk of radius .12 is clear of the wall at x=.1;
        # x=.201 has the same cell but only .101 actual clearance.
        pose = (.201, .75, 0.)
        self.assertIn(self.grid.to_cell(*pose[:2]), self.grid.safe_cells(.12))
        queue = ObservationQueue(self.grid, pose, 'epoch-a', self.policy)
        self.assertFalse(queue.views)
        self.assertEqual(queue.take_next(self.grid, pose, 'epoch-a').reason, 'unsafe_or_unlocalized')

    def test_processing_completion_terminates_without_any_coverage_contains(self):
        queue = self.queue()
        with mock.patch.object(core.Coverage, 'contains', side_effect=AssertionError('coverage is not an action ledger')):
            for _ in range(16):
                decision = queue.take_next(self.grid, self.pose, 'epoch-a')
                self.assertEqual(decision.state, 'dispatch')
                queue.finish(decision.view.action_id, 'epoch-a', 'inspection_satisfied', decision.dispatch_id)
            self.assertEqual(queue.take_next(self.grid, self.pose, 'epoch-a').state, 'exhausted')
        summary = queue.snapshot()
        self.assertTrue(summary['all_planned_actions_completed'])
        self.assertFalse(summary['visual_coverage_claimed'])
        self.assertFalse(summary['mission_success_claimed'])

    def test_active_action_cannot_double_dispatch_or_accept_other_results(self):
        queue = self.queue()
        dispatch = queue.take_next(self.grid, self.pose, 'epoch-a')
        view = dispatch.view
        for _ in range(10):
            self.assertEqual(queue.take_next(self.grid, self.pose, 'epoch-a').state, 'active')
        self.assertEqual(queue.dispatches, 1)
        for action, epoch in (('wrong-action', 'epoch-a'), (view.action_id, 'old-epoch')):
            with self.assertRaises(ValueError):
                queue.finish(action, epoch, 'inspection_satisfied', dispatch.dispatch_id)
        queue.finish(view.action_id, 'epoch-a', 'inspection_satisfied', dispatch.dispatch_id)
        with self.assertRaises(ValueError):
            queue.finish(view.action_id, 'epoch-a', 'inspection_satisfied', dispatch.dispatch_id)

    def test_all_failures_are_bounded_without_contains_retry_loop(self):
        queue = self.queue(max_stations=1, headings_per_station=1, max_dispatches=5, max_attempts_per_view=2)
        for _ in range(2):
            decision = queue.take_next(self.grid, self.pose, 'epoch-a')
            self.assertEqual(decision.state, 'dispatch')
            queue.finish(decision.view.action_id, 'epoch-a', 'inspection_failed', decision.dispatch_id)
        self.assertEqual(queue.take_next(self.grid, self.pose, 'epoch-a').state, 'exhausted')
        self.assertEqual(queue.dispatches, 2)
        self.assertEqual(queue.snapshot()['state_counts'], {'failed': 1})
        self.assertFalse(queue.snapshot()['all_planned_actions_completed'])

    def test_global_dispatch_budget_includes_retries_and_never_refills(self):
        queue = self.queue(max_dispatches=3, max_attempts_per_view=8)
        for _ in range(3):
            decision = queue.take_next(self.grid, self.pose, 'epoch-a')
            self.assertEqual(decision.state, 'dispatch')
            queue.finish(decision.view.action_id, 'epoch-a', 'navigation_failed', decision.dispatch_id)
        for _ in range(10):
            self.assertEqual(queue.take_next(self.grid, self.pose, 'epoch-a').state, 'budget_exhausted')
        self.assertEqual(queue.dispatches, 3)
        self.assertFalse(queue.snapshot()['all_planned_actions_completed'])

    def test_latest_map_obstacle_skips_view_instead_of_dispatching_old_safe_goal(self):
        queue = self.queue()
        first = next(iter(queue.views.values()))
        blocked = change(self.grid, [self.grid.to_cell(first.goal.x, first.goal.y)], 100)
        # Use a separate valid current pose so the new obstacle blocks only the goal.
        pose = (*self.grid.to_world((20, 10)), 0.)
        decision = queue.take_next(blocked, pose, 'epoch-a')
        self.assertEqual(queue.records[first.action_id]['state'], 'skipped_unsafe_or_unreachable')
        if decision.view:
            self.assertNotEqual(decision.view.action_id, first.action_id)
            self.assertIn(blocked.to_cell(decision.view.goal.x, decision.view.goal.y), reachable(blocked, pose, self.policy))

    def test_new_map_disconnection_skips_other_component(self):
        queue = self.queue()
        grid = change(self.grid, [(15, y) for y in range(self.grid.height)], 100)
        while True:
            decision = queue.take_next(grid, self.pose, 'epoch-a')
            if decision.state != 'dispatch':
                break
            self.assertLess(grid.to_cell(decision.view.goal.x, decision.view.goal.y)[0], 15)
            queue.finish(decision.view.action_id, 'epoch-a', 'inspection_satisfied', decision.dispatch_id)
        self.assertEqual(decision.state, 'exhausted')
        self.assertGreater(queue.snapshot()['state_counts'].get('skipped_unsafe_or_unreachable', 0), 0)

    def test_epoch_change_retains_cancel_obligation_and_rejects_old_success(self):
        queue = self.queue()
        dispatch = queue.take_next(self.grid, self.pose, 'epoch-a')
        view = dispatch.view
        self.assertEqual(queue.take_next(self.grid, self.pose, 'epoch-b').state, 'invalidated')
        self.assertTrue(queue.snapshot()['cancel_required'])
        with self.assertRaises(ValueError):
            queue.finish(view.action_id, 'epoch-a', 'inspection_satisfied', dispatch.dispatch_id)
        queue.finish(view.action_id, 'epoch-a', 'canceled', dispatch.dispatch_id)
        self.assertFalse(queue.snapshot()['cancel_required'])
        self.assertEqual(queue.take_next(self.grid, self.pose, 'epoch-a').state, 'invalidated')
        self.assertEqual(queue.dispatches, 1)

    def test_grid_expansion_and_pose_jitter_do_not_refill_completed_views(self):
        queue = self.queue(max_stations=1, headings_per_station=1)
        dispatch = queue.take_next(self.grid, self.pose, 'epoch-a')
        view = dispatch.view
        queue.finish(view.action_id, 'epoch-a', 'inspection_satisfied', dispatch.dispatch_id)
        expanded = room(50, 44, origin=(-1., -1.))
        for i in range(10):
            self.assertEqual(queue.take_next(expanded, (self.pose[0]+i*1e-4, self.pose[1], 0.), 'epoch-a').state, 'exhausted')
        self.assertEqual(queue.dispatches, 1)
        self.assertEqual(len(queue.views), 1)

    def test_same_epoch_fractional_origin_change_revalidates_frozen_world_goals(self):
        queue = self.queue()
        grid = room(50, 44, origin=(-1.023, -1.017))
        original = [(v.goal.x, v.goal.y, v.goal.yaw) for v in queue.views.values()]
        decision = queue.take_next(grid, self.pose, 'epoch-a')
        self.assertEqual(decision.state, 'dispatch')
        safe = grid.safe_cells(self.policy.robot_radius_m)
        for i in range(16):
            angle = 2*pi*i/16
            cell = grid.to_cell(decision.view.goal.x+.08*cos(angle), decision.view.goal.y+.08*sin(angle))
            self.assertIn(cell, safe)
        self.assertEqual(original, [(v.goal.x, v.goal.y, v.goal.yaw) for v in queue.views.values()])

    def test_unsafe_current_pose_does_not_consume_attempts_or_silently_complete(self):
        queue = self.queue()
        decision = queue.take_next(self.grid, (.01, .01, 0.), 'epoch-a')
        self.assertEqual(decision.state, 'unsafe_or_unlocalized')
        self.assertEqual(queue.dispatches, 0)
        self.assertFalse(queue.snapshot()['all_planned_actions_completed'])

    def test_invalid_policy_pose_and_work_budget_fail_explicitly(self):
        for values in ({'robot_radius_m': float('nan')}, {'station_spacing_m': 0},
                       {'max_dispatches': True}, {'headings_per_station': 17}):
            with self.subTest(values=values), self.assertRaises(ValueError):
                replace(self.policy, **values)
        with self.assertRaises(ValueError):
            ObservationQueue(self.grid, self.pose, '', self.policy)
        with self.assertRaises(ValueError):
            ObservationQueue(self.grid, (0, 0, float('inf')), 'epoch-a', self.policy)
        with self.assertRaises(ValueError):
            ObservationQueue(self.grid, self.pose, 'epoch-a', replace(self.policy, max_grid_cells=5))

    def test_previous_attempt_result_cannot_complete_retry_of_same_view(self):
        queue = self.queue(max_stations=1, headings_per_station=1, max_dispatches=3)
        first = queue.take_next(self.grid, self.pose, 'epoch-a')
        queue.finish(first.view.action_id, 'epoch-a', 'navigation_failed', first.dispatch_id)
        retry = queue.take_next(self.grid, self.pose, 'epoch-a')
        self.assertEqual(first.view.action_id, retry.view.action_id)
        self.assertNotEqual(first.dispatch_id, retry.dispatch_id)
        with self.assertRaises(ValueError):
            queue.finish(first.view.action_id, 'epoch-a', 'inspection_satisfied', first.dispatch_id)
        self.assertEqual(queue.active_dispatch_id, retry.dispatch_id)
        queue.finish(retry.view.action_id, 'epoch-a', 'inspection_satisfied', retry.dispatch_id)


if __name__ == '__main__':
    unittest.main(verbosity=2)
