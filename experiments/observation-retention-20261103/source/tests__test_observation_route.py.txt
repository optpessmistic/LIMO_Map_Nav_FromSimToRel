"""Frozen-view scheduling on synthetic raw maps, without ROS or scene truth."""
from dataclasses import replace
from math import hypot, pi
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/limo_maze_mission'))
from limo_maze_mission.observation import (
    Grid, Goal, ObservationQueue, Policy, View, core, reachable, station_first_order)


def room():
    return Grid(30, 24, .1, 0., 0., 0., tuple(
        0 if 0 < x < 29 and 0 < y < 23 else 100
        for y in range(24) for x in range(30)))


def path_length(grid, pose, views, order, radius):
    """Sum shortest four-neighbor safe-grid legs, not straight-line lengths."""
    safe = grid.safe_cells(radius)
    cursor = grid.to_cell(*pose[:2])
    cache, total = {}, 0
    for action_id in order:
        if cursor not in cache:
            cache[cursor] = grid.component(cursor, safe)
        goal = views[action_id].goal
        destination = grid.to_cell(goal.x, goal.y)
        total += cache[cursor][destination]
        cursor = destination
    return total*grid.resolution


class ObservationRouteTests(unittest.TestCase):
    def setUp(self):
        self.grid = room()
        self.pose = (*self.grid.to_world((7, 7)), .3)
        self.policy = Policy(.12, .08, .65, 4, 4, 16, 2)

    def test_truncated_selection_keeps_original_ids_stations_and_headings(self):
        # Reordering must not replace the old round-robin prefix with a prefix
        # containing more headings at fewer stations when the budget is small.
        for limit in (1, 3, 7, 11, 16):
            with self.subTest(limit=limit):
                policy = replace(self.policy, max_dispatches=limit)
                queue = ObservationQueue(self.grid, self.pose, 'epoch-a', policy)
                expected = []
                for heading in (0, 1, 3, 2):
                    for station, xy in enumerate(queue.stations):
                        if len(expected) < limit:
                            expected.append((station, heading, xy))
                self.assertEqual(len(queue.views), len(expected))
                for number, (station, heading, xy) in enumerate(expected, 1):
                    action_id = 'observation-%04d' % number
                    view = queue.views[action_id]
                    self.assertEqual((view.station_id, view.heading_index), (station, heading))
                    self.assertEqual((view.goal.x, view.goal.y), xy)
                    self.assertEqual(view.goal.yaw, core.angle_delta(2*pi*heading/4, 0.))
                self.assertEqual(set(queue.pending), set(queue.views))
                self.assertEqual(len(queue.pending), len(queue.views))
                self.assertEqual(queue.snapshot()['dispatch_budget'], limit)
                self.assertFalse(queue.snapshot()['all_planned_actions_completed'])

    def test_full_queue_groups_stations_and_reduces_safe_path_length(self):
        queue = ObservationQueue(self.grid, self.pose, 'epoch-a', self.policy)
        stations = [queue.views[a].station_id for a in queue.pending]
        blocks = [station for i, station in enumerate(stations)
                  if i == 0 or station != stations[i-1]]
        self.assertEqual(len(blocks), len(queue.stations))
        self.assertEqual(len(set(blocks)), len(blocks))
        for station in blocks:
            self.assertEqual({queue.views[a].heading_index for a in queue.pending
                              if queue.views[a].station_id == station}, {0, 1, 2, 3})
        old_length = path_length(self.grid, self.pose, queue.views,
                                 list(queue.views), self.policy.robot_radius_m)
        new_length = path_length(self.grid, self.pose, queue.views,
                                 list(queue.pending), self.policy.robot_radius_m)
        self.assertLess(new_length, old_length)
        self.assertEqual(list(queue.pending), list(ObservationQueue(
            self.grid, self.pose, 'epoch-a', self.policy).pending))

    def test_station_order_follows_safe_detour_not_euclidean_nearness(self):
        for obstacle in (-1, 50, 100):
            with self.subTest(obstacle=obstacle):
                data = list(self.grid.data)
                for y in range(1, 19):
                    data[y*self.grid.width+8] = obstacle
                grid = replace(self.grid, data=tuple(data))
                cells = ((6, 5), (10, 5), (6, 12))
                pose = (*grid.to_world(cells[0]), 0.)
                policy = Policy(.02, .01, .1, 3, 1, 3)
                distances = reachable(grid, pose, policy)
                views = {}
                for station, cell in enumerate(cells):
                    xy = grid.to_world(cell)
                    action = 'station-%d' % station
                    views[action] = View(action, 'epoch-a', station, 0,
                                         Goal(*xy, 0., 'observation_queue', xy))
                xy = [grid.to_world(cell) for cell in cells]
                self.assertLess(hypot(xy[1][0]-xy[0][0], xy[1][1]-xy[0][1]),
                                hypot(xy[2][0]-xy[0][0], xy[2][1]-xy[0][1]))
                self.assertGreater(distances[cells[1]], distances[cells[2]])
                self.assertEqual(station_first_order(grid, pose, distances, views),
                                 ['station-0', 'station-2', 'station-1'])

    def test_heading_order_uses_last_planned_yaw_without_adding_views(self):
        cells = ((7, 7), (20, 10))
        # Deliberately omit pi: scheduling must use only the selected headings.
        headings = {0: 0., 1: pi/2, 3: -pi/2}
        pose = (*self.grid.to_world(cells[0]), pi/2)
        views = {}
        for heading, yaw in headings.items():
            for station, cell in enumerate(cells):
                xy = self.grid.to_world(cell)
                action = '%d/%d' % (station, heading)
                views[action] = View(action, 'epoch-a', station, heading,
                                     Goal(*xy, yaw, 'observation_queue', xy))
        order = station_first_order(self.grid, pose, reachable(self.grid, pose, self.policy), views)
        self.assertEqual(order, ['0/1', '0/0', '0/3', '1/3', '1/0', '1/1'])
        self.assertEqual(set(order), set(views))
        self.assertEqual(order, station_first_order(
            self.grid, pose, reachable(self.grid, pose, self.policy), views))

    def test_retries_still_append_and_do_not_credit_or_expand_the_frozen_set(self):
        policy = replace(self.policy, max_stations=2, headings_per_station=2, max_dispatches=6)
        queue = ObservationQueue(self.grid, self.pose, 'epoch-a', policy)
        frozen = dict(queue.views)
        dispatch = queue.take_next(self.grid, self.pose, 'epoch-a')
        pending = list(queue.pending)
        queue.finish(dispatch.view.action_id, 'epoch-a', 'navigation_failed', dispatch.dispatch_id)
        self.assertEqual(list(queue.pending), pending+[dispatch.view.action_id])
        self.assertEqual(queue.views, frozen)
        self.assertEqual(queue.records[dispatch.view.action_id]['state'], 'pending')
        self.assertEqual(queue.dispatches, 1)
        self.assertFalse(queue.snapshot()['all_planned_actions_completed'])
        while queue.pending:
            dispatch = queue.take_next(self.grid, self.pose, 'epoch-a')
            self.assertEqual(dispatch.state, 'dispatch')
            queue.finish(dispatch.view.action_id, 'epoch-a', 'inspection_satisfied', dispatch.dispatch_id)
        self.assertEqual(queue.dispatches, 5)
        self.assertTrue(queue.snapshot()['all_planned_actions_completed'])
        self.assertFalse(queue.snapshot()['visual_coverage_claimed'])
        self.assertFalse(queue.snapshot()['mission_success_claimed'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
