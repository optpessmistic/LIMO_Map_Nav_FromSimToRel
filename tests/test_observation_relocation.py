"""Bounded observation relocation on synthetic raw grids; no ROS or truth."""
from dataclasses import replace
from math import cos, hypot, pi, sin
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/limo_maze_mission'))
from limo_maze_mission.observation import (
    Grid, ObservationQueue, Policy, endpoint_radius, reachable)


EPOCH = 'relocation-epoch'


def room(resolution=.1, origin=(0., 0.), yaw=0.):
    return Grid(26, 26, resolution, *origin, yaw, tuple(
        0 if 0 < x < 25 and 0 < y < 25 else 100
        for y in range(26) for x in range(26)))


def change(grid, cells, value=100):
    data = list(grid.data)
    for x, y in cells:
        data[y * grid.width + x] = value
    return replace(grid, data=tuple(data))


def pose_at(grid, cell, yaw=0.):
    return (*grid.to_world(cell), yaw)


class ObservationRelocationTests(unittest.TestCase):
    def setUp(self):
        self.grid = room()
        self.anchor_cell = (10, 10)
        self.initial_pose = pose_at(self.grid, self.anchor_cell)
        self.policy = Policy(.04, .08, 1.2, 1, 2, 6, 2,
                             relocation_radius_m=.3)

    def queue(self, grid=None, **policy_changes):
        grid = self.grid if grid is None else grid
        return ObservationQueue(grid, pose_at(grid, self.anchor_cell), EPOCH,
                                replace(self.policy, **policy_changes))

    def finish(self, queue, decision, outcome='inspection_satisfied'):
        queue.finish(decision.view.action_id, EPOCH, outcome, decision.dispatch_id)

    def assert_disk_clear(self, grid, xy, radius):
        """Independent continuous disk/rectangle check, including map edges."""
        dx, dy = xy[0]-grid.origin_x, xy[1]-grid.origin_y
        x = cos(grid.origin_yaw)*dx + sin(grid.origin_yaw)*dy
        y = -sin(grid.origin_yaw)*dx + cos(grid.origin_yaw)*dy
        self.assertGreater(min(x, y, grid.width*grid.resolution-x,
                               grid.height*grid.resolution-y), radius)
        for row in range(grid.height):
            for col in range(grid.width):
                if grid.data[row*grid.width+col] == 0:
                    continue
                left, bottom = col*grid.resolution, row*grid.resolution
                distance = hypot(max(left-x, 0., x-left-grid.resolution),
                                 max(bottom-y, 0., y-bottom-grid.resolution))
                self.assertGreater(distance, radius)

    def assert_identity_preserved(self, original, actual):
        self.assertEqual((actual.action_id, actual.epoch, actual.station_id,
                          actual.heading_index),
                         (original.action_id, original.epoch, original.station_id,
                          original.heading_index))
        self.assertEqual((actual.goal.yaw, actual.goal.kind, actual.goal.anchor),
                         (original.goal.yaw, original.goal.kind, original.goal.anchor))

    def test_radius_is_default_disabled_and_has_finite_geometry_bound(self):
        self.assertEqual(Policy(.04, .08, 1.2, 1, 2, 6).relocation_radius_m, 0.)
        for radius in (0., .3):
            self.assertEqual(replace(self.policy, relocation_radius_m=radius).relocation_radius_m, radius)
        self.assertEqual(replace(self.policy, station_spacing_m=2.,
                                 relocation_radius_m=.5).relocation_radius_m, .5)
        for radius in (-.01, True, float('nan'), float('inf'), .300001):
            with self.subTest(radius=radius), self.assertRaises(ValueError):
                replace(self.policy, relocation_radius_m=radius)
        with self.assertRaises(ValueError):
            replace(self.policy, station_spacing_m=10., relocation_radius_m=.500001)

    def test_default_skip_is_permanent_and_never_becomes_retry(self):
        queue = self.queue(relocation_radius_m=0.)
        blocked = change(self.grid, [(12, 10)])
        decision = queue.take_next(blocked, self.initial_pose, EPOCH, map_stamp_ns=10)
        self.assertEqual(decision.state, 'exhausted')
        self.assertEqual(queue.dispatches, 0)
        self.assertEqual(queue.snapshot()['state_counts'], {'skipped_unsafe_or_unreachable': 2})
        self.assertTrue(all(r['attempts'] == 0 for r in queue.records.values()))
        history = queue.snapshot()['selection_history']
        self.assertEqual(len(history), 2)
        for entry in history:
            self.assertEqual(entry['state'], 'skipped_unsafe_or_unreachable')
            self.assertEqual(entry['map']['stamp_ns'], 10)
            self.assertIsNone(entry['selected_check'])
        for _ in range(3):
            self.assertEqual(queue.take_next(self.grid, self.initial_pose, EPOCH).state, 'exhausted')
        self.assertEqual(queue.snapshot()['selection_history'], history)

    def test_safe_preferred_endpoint_is_not_moved(self):
        queue = self.queue()
        expected = queue.views[queue.pending[0]]
        decision = queue.take_next(self.grid, pose_at(self.grid, (18, 10)), EPOCH)
        self.assertEqual(decision.state, 'dispatch')
        self.assertEqual(decision.view, expected)
        self.assertEqual(decision.validation['selected_shift_from_anchor_m'], 0.)

    def test_local_relocation_preserves_frozen_views_and_arrival_footprint(self):
        for blocked_value in (-1, 50, 100):
            with self.subTest(blocked_value=blocked_value):
                queue = self.queue()
                frozen, stations = dict(queue.views), queue.stations
                grid = change(self.grid, [(12, 10)], blocked_value)
                self.assertIn(self.anchor_cell, grid.safe_cells(self.policy.robot_radius_m))
                self.assertNotIn(self.anchor_cell, grid.safe_cells(endpoint_radius(grid, self.policy)))
                decision = queue.take_next(grid, self.initial_pose, EPOCH, map_stamp_ns=20)
                self.assertEqual(decision.state, 'dispatch')
                self.assertEqual(grid.to_cell(decision.view.goal.x, decision.view.goal.y), (9, 10))
                self.assert_identity_preserved(frozen[decision.view.action_id], decision.view)
                self.assertEqual(queue.views, frozen)
                self.assertEqual(queue.stations, stations)
                self.assertEqual(queue.station_positions[0], grid.to_world((9, 10)))
                self.assertIn((9, 10), reachable(grid, self.initial_pose, self.policy))
                self.assertLessEqual(decision.validation['selected_shift_from_anchor_m'], .3)
                for step in range(32):
                    angle = 2*pi*step/32
                    xy = (decision.view.goal.x+self.policy.arrival_margin_m*cos(angle),
                          decision.view.goal.y+self.policy.arrival_margin_m*sin(angle))
                    self.assert_disk_clear(grid, xy, self.policy.robot_radius_m)
                # The active return must use the actual dispatch, not its frozen XY.
                again = queue.take_next(self.grid, pose_at(self.grid, (18, 10)), EPOCH,
                                        map_stamp_ns=21)
                self.assertEqual(again.state, 'active')
                self.assertEqual(again.view, decision.view)
                self.assertEqual(again.dispatch_id, decision.dispatch_id)
                self.assertEqual(queue.dispatches, 1)
                self.assertEqual(len(queue.snapshot()['selection_history']), 1)

    def test_safe_last_dispatch_xy_persists_for_next_heading_when_anchor_recovers(self):
        queue = self.queue()
        first = queue.take_next(change(self.grid, [(12, 10)]), self.initial_pose, EPOCH)
        self.finish(queue, first)
        second = queue.take_next(self.grid, pose_at(self.grid, (9, 10)), EPOCH)
        self.assertEqual(second.state, 'dispatch')
        self.assertNotEqual(second.view.action_id, first.view.action_id)
        self.assertEqual((second.view.goal.x, second.view.goal.y),
                         (first.view.goal.x, first.view.goal.y))
        self.assert_identity_preserved(queue.views[second.view.action_id], second.view)
        self.assertNotEqual(second.view.goal.yaw, first.view.goal.yaw)

    def test_blocked_original_anchor_cannot_be_replaced_even_if_nearby_endpoint_exists(self):
        policy = replace(self.policy, station_spacing_m=1.4, relocation_radius_m=.35)
        for value in (-1, 50, 100):
            with self.subTest(value=value):
                queue = ObservationQueue(self.grid, self.initial_pose, EPOCH, policy)
                grid = change(self.grid, [self.anchor_cell], value)
                anchor = queue.stations[0]
                self.assertTrue(any(hypot(*(a-b for a, b in zip(grid.to_world(c), anchor))) < .35
                                    for c in grid.safe_cells(endpoint_radius(grid, policy))))
                decision = queue.take_next(grid, pose_at(grid, (18, 10)), EPOCH)
                self.assertEqual(decision.state, 'exhausted')
                self.assertEqual(queue.dispatches, 0)
                self.assertTrue(all(not row['anchor_robot_disk_safe']
                                    for row in queue.snapshot()['selection_history']))

    def test_globally_disconnected_anchor_cannot_jump_to_current_component(self):
        policy = replace(self.policy, station_spacing_m=2., relocation_radius_m=.5)
        queue = ObservationQueue(self.grid, self.initial_pose, EPOCH, policy)
        grid = change(self.grid, [(11, y) for y in range(self.grid.height)])
        pose = pose_at(grid, (18, 10))
        distances = reachable(grid, pose, policy)
        self.assertNotIn(self.anchor_cell, distances)
        self.assertIn((14, 10), distances)
        self.assertIn((14, 10), grid.safe_cells(endpoint_radius(grid, policy)))
        self.assertLess(hypot(*(a-b for a, b in zip(grid.to_world((14, 10)), queue.stations[0]))), .5)
        self.assertEqual(queue.take_next(grid, pose, EPOCH).state, 'exhausted')
        self.assertEqual(queue.dispatches, 0)

    def test_local_component_cannot_cross_wall_even_when_global_detour_exists(self):
        policy = replace(self.policy, robot_radius_m=.02, arrival_margin_m=.02,
                         station_spacing_m=1.6, relocation_radius_m=.4)
        queue = ObservationQueue(self.grid, self.initial_pose, EPOCH, policy)
        # A narrow R-safe corridor has no arrival-safe endpoint. The exit is
        # beyond the relocation disk; a legal endpoint across the wall is near
        # in Euclidean distance and globally reachable via the long detour.
        grid = change(self.grid, [(x, y) for x in (9, 11) for y in range(5, 16)])
        pose = pose_at(grid, (18, 10))
        distances = reachable(grid, pose, policy)
        self.assertIn(self.anchor_cell, distances)
        self.assertIn((13, 10), distances)
        self.assertIn((13, 10), grid.safe_cells(endpoint_radius(grid, policy)))
        self.assertLess(hypot(*(a-b for a, b in zip(grid.to_world((13, 10)), queue.stations[0]))), .4)
        self.assertEqual(queue.take_next(grid, pose, EPOCH).state, 'exhausted')
        self.assertTrue(all(row['anchor_robot_disk_safe'] and row['local_candidate_count'] == 0
                            for row in queue.snapshot()['selection_history']))

    def test_unsafe_actual_pose_does_not_consume_pending_attempt_or_history(self):
        queue = self.queue()
        pending, stations = list(queue.pending), dict(queue.station_positions)
        pose = (.1001, .75, 0.)  # Its cell center is clear, but the actual disk touches the wall.
        self.assertIn(self.grid.to_cell(*pose[:2]), self.grid.safe_cells(self.policy.robot_radius_m))
        decision = queue.take_next(self.grid, pose, EPOCH, map_stamp_ns=30)
        self.assertEqual(decision.state, 'unsafe_or_unlocalized')
        self.assertEqual(list(queue.pending), pending)
        self.assertEqual(queue.station_positions, stations)
        self.assertEqual(queue.dispatches, 0)
        self.assertEqual(queue.snapshot()['selection_history'], [])
        self.assertTrue(all(r['attempts'] == 0 for r in queue.records.values()))
        self.assertEqual(queue.take_next(self.grid, self.initial_pose, EPOCH).state, 'dispatch')

    def test_radius_is_always_measured_from_original_anchor_not_previous_dispatch(self):
        queue = self.queue(relocation_radius_m=.11)
        first = queue.take_next(change(self.grid, [(12, 10)]), self.initial_pose, EPOCH)
        self.assertEqual(self.grid.to_cell(first.view.goal.x, first.view.goal.y), (9, 10))
        self.finish(queue, first)
        newer = change(self.grid, [(11, 10)])
        self.assertIn((8, 10), newer.safe_cells(endpoint_radius(newer, queue.policy)))
        # (8,10) is one more step from the previous choice, but two from anchor.
        self.assertLess(hypot(*(a-b for a, b in zip(newer.to_world((8, 10)),
                                                   (first.view.goal.x, first.view.goal.y)))), .11)
        second = queue.take_next(newer, pose_at(newer, (9, 10)), EPOCH)
        self.assertEqual(second.state, 'exhausted')
        self.assertEqual(queue.dispatches, 1)
        self.assertEqual(queue.station_positions[0], self.grid.to_world((9, 10)))
        self.assertEqual(queue.snapshot()['state_counts'], {'completed': 1, 'skipped_unsafe_or_unreachable': 1})

    def test_fractional_origin_anchor_requires_actual_disk_not_only_cell_center(self):
        queue = self.queue()
        shifted = change(replace(self.grid, origin_x=.04), [(9, 10)])
        anchor = queue.stations[0]
        cell = shifted.to_cell(*anchor)
        center = shifted.to_world(cell)
        offset = hypot(center[0]-anchor[0], center[1]-anchor[1])
        self.assertIn(cell, shifted.safe_cells(self.policy.robot_radius_m))
        self.assertNotIn(cell, shifted.safe_cells(self.policy.robot_radius_m+offset))
        self.assertEqual(queue.take_next(shifted, pose_at(shifted, (18, 10)), EPOCH).state, 'exhausted')
        self.assertTrue(all(not row['anchor_robot_disk_safe']
                            for row in queue.snapshot()['selection_history']))

    def test_rotated_and_shifted_map_relocation_stays_in_world_anchor_disk(self):
        for yaw in (0., .7):
            for shift in (0., .02):
                with self.subTest(yaw=yaw, shift=shift):
                    original = room(origin=(2., -1.), yaw=yaw)
                    queue = self.queue(original)
                    anchor = queue.stations[0]
                    current = replace(original, origin_x=original.origin_x+cos(yaw)*shift,
                                      origin_y=original.origin_y+sin(yaw)*shift)
                    current = change(current, [(12, 10)])
                    decision = queue.take_next(current, pose_at(current, (18, 10)), EPOCH,
                                               map_stamp_ns=40)
                    self.assertEqual(decision.state, 'dispatch')
                    self.assertEqual(current.to_cell(decision.view.goal.x, decision.view.goal.y), (9, 10))
                    self.assertLessEqual(hypot(decision.view.goal.x-anchor[0], decision.view.goal.y-anchor[1]), .3)
                    self.assertEqual(decision.view.goal.anchor, anchor)
                    self.assertEqual(decision.validation['map']['origin'],
                                     [current.origin_x, current.origin_y, yaw])
                    for angle in (0., pi/2, pi, -pi/2):
                        self.assert_disk_clear(current,
                            (decision.view.goal.x+.08*cos(angle), decision.view.goal.y+.08*sin(angle)), .04)

    def test_nearest_candidate_ties_use_path_steps_then_cell(self):
        # Binary-exact grid coordinates give a real tie, not decimal roundoff.
        grid = room(resolution=.125)
        policy = replace(self.policy, arrival_margin_m=.14, relocation_radius_m=.25)
        current = change(grid, [(12, 12)])
        for pose_cell, expected in (((10, 7), (10, 9)), ((8, 8), (9, 10))):
            with self.subTest(pose_cell=pose_cell):
                queue = ObservationQueue(grid, pose_at(grid, self.anchor_cell), EPOCH, policy)
                decision = queue.take_next(current, pose_at(current, pose_cell), EPOCH)
                self.assertEqual(decision.state, 'dispatch')
                self.assertEqual(current.to_cell(decision.view.goal.x, decision.view.goal.y), expected)

    def test_retry_identity_and_dispatch_budget_do_not_expand_after_relocation(self):
        queue = self.queue(headings_per_station=1, max_attempts_per_view=2)
        current = change(self.grid, [(12, 10)])
        first = queue.take_next(current, self.initial_pose, EPOCH)
        self.finish(queue, first, 'navigation_failed')
        retry = queue.take_next(current, self.initial_pose, EPOCH)
        self.assertEqual(retry.view, first.view)
        self.assertNotEqual(retry.dispatch_id, first.dispatch_id)
        with self.assertRaises(ValueError):
            queue.finish(first.view.action_id, EPOCH, 'inspection_satisfied', first.dispatch_id)
        self.assertEqual(queue.active_dispatch_id, retry.dispatch_id)
        self.finish(queue, retry, 'navigation_failed')
        self.assertEqual(queue.take_next(current, self.initial_pose, EPOCH).state, 'exhausted')
        self.assertEqual(queue.dispatches, 2)
        self.assertEqual(queue.snapshot()['planned_views'], 1)

        bounded = self.queue(max_dispatches=2, max_attempts_per_view=8)
        for _ in range(2):
            dispatch = bounded.take_next(current, self.initial_pose, EPOCH)
            self.assertEqual(dispatch.state, 'dispatch')
            self.finish(bounded, dispatch, 'navigation_failed')
        before = bounded.snapshot()['selection_history']
        for _ in range(3):
            self.assertEqual(bounded.take_next(self.grid, self.initial_pose, EPOCH).state, 'budget_exhausted')
        self.assertEqual(bounded.dispatches, 2)
        self.assertEqual(bounded.snapshot()['selection_history'], before)

    def test_epoch_change_keeps_relocated_active_identity_until_cancel_terminal(self):
        queue = self.queue()
        dispatch = queue.take_next(change(self.grid, [(12, 10)]), self.initial_pose, EPOCH)
        self.assertEqual(queue.take_next(self.grid, self.initial_pose, 'different-epoch').state, 'invalidated')
        self.assertTrue(queue.snapshot()['cancel_required'])
        self.assertEqual(queue.active_view, dispatch.view)
        with self.assertRaises(ValueError):
            self.finish(queue, dispatch)
        self.finish(queue, dispatch, 'canceled')
        self.assertFalse(queue.snapshot()['cancel_required'])
        self.assertEqual(queue.take_next(self.grid, self.initial_pose, EPOCH).state, 'invalidated')
        self.assertEqual(queue.dispatches, 1)
        self.assertFalse(queue.snapshot()['visual_coverage_claimed'])
        self.assertFalse(queue.snapshot()['mission_success_claimed'])

    def test_map_stamp_validation_and_history_bind_data_geometry_and_copy(self):
        for stamp in (-1, True, 1.5, '123'):
            with self.subTest(stamp=stamp):
                queue = self.queue()
                pending = list(queue.pending)
                with self.assertRaises(ValueError):
                    queue.take_next(self.grid, self.initial_pose, EPOCH, map_stamp_ns=stamp)
                self.assertEqual(list(queue.pending), pending)
                self.assertEqual(queue.dispatches, 0)
                self.assertEqual(queue.snapshot()['selection_history'], [])
        queue = self.queue()
        current = change(self.grid, [(12, 10)])
        dispatch = queue.take_next(current, self.initial_pose, EPOCH, map_stamp_ns=0)
        history = queue.snapshot()['selection_history']
        self.assertEqual(history, [dispatch.validation])
        fingerprint = history[0]['map']['grid_sha256']
        self.assertRegex(fingerprint, '^[0-9a-f]{64}$')
        self.assertEqual(history[0]['map']['stamp_ns'], 0)
        self.assertEqual(history[0]['pose'], list(self.initial_pose))
        self.assertFalse(history[0]['preferred_check']['endpoint_safe'])
        self.assertTrue(history[0]['selected_check']['endpoint_safe'])
        self.assertTrue(history[0]['selected_check']['reachable'])
        dispatch.validation['map']['grid_sha256'] = 'not-the-record'
        history[0]['selected_check']['xy'][0] = 999.
        self.assertEqual(queue.snapshot()['selection_history'][0]['map']['grid_sha256'], fingerprint)
        self.assertNotEqual(queue.snapshot()['selection_history'][0]['selected_check']['xy'][0], 999.)
        same = self.queue().take_next(current, self.initial_pose, EPOCH, map_stamp_ns=1)
        self.assertEqual(same.validation['map']['grid_sha256'], fingerprint)
        changed = self.queue().take_next(change(current, [(20, 20)], -1), self.initial_pose, EPOCH)
        self.assertNotEqual(changed.validation['map']['grid_sha256'], fingerprint)
        geometry = replace(current, origin_x=.01)
        moved = self.queue().take_next(geometry, pose_at(geometry, (18, 10)), EPOCH)
        self.assertNotEqual(moved.validation['map']['grid_sha256'], fingerprint)


if __name__ == '__main__':
    unittest.main(verbosity=2)
