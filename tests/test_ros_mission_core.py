"""Pure core tests plus an isolated, optional real-ROS startup regression."""
import math
import ast
import importlib.util
import json
import os
import random
from pathlib import Path
import subprocess
import sys
import textwrap
import tempfile
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'limo_maze_mission'))
from limo_maze_mission.core import Coverage, FailureBook, Goal, Grid, Planner, TargetStore


def grid(rows, resolution=.1, origin=(0., 0., 0.)):
    values = {'#': 100, '.': 0, '?': -1}
    return Grid(len(rows[0]), len(rows), resolution, *origin,
                tuple(values[c] for row in rows for c in row))


def cover_all(g):
    coverage = Coverage()
    coverage.points = {coverage.key(g.to_world(p)) for p in g.cells() if g.free(p)}
    coverage.frames = 1
    return coverage


class GridTests(unittest.TestCase):
    def test_rotated_origin_roundtrip(self):
        g = grid(['...', '...'], origin=(4., -3., math.pi/2))
        for p in g.cells():
            self.assertEqual(g.to_cell(*g.to_world(p)), p)

    def test_inflation_blocks_walls_unknown_and_outside(self):
        g = grid(['#######', '#.....#', '#..?..#', '#.....#', '#######'])
        safe = g.safe_cells(.06)
        self.assertNotIn((2, 2), safe)  # The footprint reaches the unknown cell.
        self.assertNotIn((1, 1), safe)  # Footprint reaches the outer wall.
        open_grid = grid(['.....']*5)
        self.assertNotIn((0, 2), open_grid.safe_cells(.06))
        self.assertIn((2, 2), open_grid.safe_cells(.06))

    def test_unknown_and_probabilistic_occupied_are_not_paths(self):
        g = Grid(3, 1, .1, 0, 0, 0, (0, 20, 0))
        self.assertEqual(set(g.component((0, 0))), {(0, 0)})
        g = grid(['.?.'])
        self.assertEqual(set(g.component((0, 0))), {(0, 0)})

    def test_supercover_blocks_corner(self):
        g = grid(['.#', '#.'])
        self.assertFalse(g.line_free((0, 0), (1, 1)))

    def test_frontier_includes_unknown_beyond_grid_edge(self):
        g = grid(['...']*3)
        self.assertTrue(g.frontier((0, 1)))
        self.assertFalse(g.frontier((1, 1)))

    def test_optimized_footprint_matches_direct_collision_check(self):
        rng = random.Random(7)
        g = Grid(21, 21, .05, 0, 0, 0, tuple(rng.choices((0, -1, 100), weights=(8, 1, 1), k=441)))
        for radius in (.04, .12, .24):
            extent = math.ceil(radius/g.resolution+.5)
            reference = set()
            for p in g.cells():
                blocked = False
                for dy in range(-extent, extent+1):
                    for dx in range(-extent, extent+1):
                        touches = math.hypot(max(abs(dx)-.5, 0), max(abs(dy)-.5, 0))*g.resolution <= radius
                        if touches and not g.free((p[0]+dx, p[1]+dy)):
                            blocked = True
                if not blocked:
                    reference.add(p)
            self.assertEqual(g.safe_cells(radius), reference)


class FreePriorTests(unittest.TestCase):
    def test_rotated_translated_origin_uses_world_center(self):
        original = grid(['???']*3, resolution=1)
        rotated = grid(['???']*3, resolution=1, origin=(4., -3., math.pi/3))
        expected, count = original.with_free_prior(original.to_world((1, 1)), .8)
        result, rotated_count = rotated.with_free_prior(rotated.to_world((1, 1)), .8)
        self.assertEqual((count, rotated_count), (1, 1))
        self.assertEqual(result.data, expected.data)
        self.assertEqual(result.value((1, 1)), 0)
        self.assertEqual(result.origin_x, rotated.origin_x)
        self.assertEqual(result.origin_y, rotated.origin_y)
        self.assertEqual(result.origin_yaw, rotated.origin_yaw)

    def test_center_inside_is_not_enough_and_corners_on_boundary_are_allowed(self):
        original = grid(['?'], resolution=1)
        partial, count = original.with_free_prior((.5, .5), .6)
        self.assertEqual(count, 0)  # Cell center is inside; all corners are outside.
        self.assertEqual(partial.data, (-1,))
        boundary, count = original.with_free_prior((.5, .5), math.hypot(.5, .5))
        self.assertEqual((boundary.data, count), ((0,), 1))

    def test_measured_values_and_all_other_fields_are_preserved(self):
        original = Grid(5, 1, .1, 2., -1., .3, (-1, 0, 1, 64, 100), occupied_threshold=70)
        result, count = original.with_free_prior(original.to_world((2, 0)), 1)
        self.assertEqual(count, 1)
        self.assertEqual(result.data, (0, 0, 1, 64, 100))
        self.assertEqual(original.data, (-1, 0, 1, 64, 100))
        self.assertIsNot(result, original)
        for name in ('width', 'height', 'resolution', 'origin_x', 'origin_y', 'origin_yaw', 'occupied_threshold'):
            self.assertEqual(getattr(result, name), getattr(original, name))

    def test_prior_outside_map_never_expands_map(self):
        original = grid(['??'], resolution=1)
        result, count = original.with_free_prior((10., 10.), 1)
        self.assertEqual((count, result.data), (0, original.data))
        result, count = original.with_free_prior((-.5, .5), 2)
        self.assertEqual((result.width, result.height, len(result.data)), (2, 1, 2))
        self.assertEqual(count, 1)  # First cell is entirely in disk; second is not.
        self.assertEqual(result.value((-1, 0)), -1)

    def test_invalid_prior_geometry_is_rejected(self):
        original = grid(['?'])
        for center, radius in [((0, 0), 0), ((0, 0), -1), ((0, 0), math.inf),
                               ((0, 0), math.nan), ((math.nan, 0), 1),
                               ((0, math.inf), 1), ((0,), 1), ((0, 0, 0), 1),
                               (None, 1), (('0', 0), 1), ((0, 0), True)]:
            with self.subTest(center=center, radius=radius), self.assertRaises(ValueError):
                original.with_free_prior(center, radius)

    def test_using_copy_does_not_change_original_planning_or_coverage(self):
        original = grid(['?????']*5)
        pose = (*original.to_world((2, 2)), 0)
        planner, coverage = Planner(radius=.04), Coverage()
        before = planner.select(original, pose, coverage, FailureBook(), 0, False)
        result, count = original.with_free_prior(pose[:2], .22)
        self.assertGreater(count, 0)
        self.assertTrue(result.free((2, 2)))
        after = planner.select(original, pose, coverage, FailureBook(), 0, False)
        self.assertEqual(before, after)
        self.assertEqual(after.state, 'unsafe_or_unlocalized')
        self.assertEqual(original.component((2, 2)), {})
        self.assertEqual(original.safe_cells(.04), set())
        self.assertEqual((coverage.frames, coverage.points), (0, set()))


class PlanningTests(unittest.TestCase):
    # These .1 m toy grids use a .04 m robot and explicitly scaled .01 m
    # arrival margin when testing unrelated translation/blacklist semantics.
    # The real profile remains .25 m robot / .05 m arrival margin below.
    def test_frontier_translation_parameter_must_be_finite_and_positive(self):
        for minimum in (0, -.1, math.nan, math.inf):
            with self.subTest(minimum=minimum), self.assertRaisesRegex(ValueError, 'frontier_min_translation_m'):
                Planner(frontier_min_translation_m=minimum)

    def test_frontier_filter_uses_actual_pose_not_cell_center(self):
        g = grid(['#########', '#......?#', '#......?#', '#......?#', '#########'])
        center = g.to_world((2, 2))
        # Both poses occupy the same cell. A target .20 m from its center can
        # lie outside or inside the threshold measured from the real pose.
        for offset, expected_cell in ((-.024, (4, 2)), (.024, (5, 2))):
            with self.subTest(offset=offset):
                pose = (center[0]+offset, center[1], 0.)
                result = Planner(.04, goal_arrival_margin_m=.01).select(g, pose, Coverage(), FailureBook(), 0, False)
                self.assertEqual(result.state, 'goal')
                self.assertEqual(g.to_cell(result.goal.x, result.goal.y), expected_cell)
                self.assertGreater(math.dist((result.goal.x, result.goal.y), pose[:2]), .20)

    def test_frontier_translation_threshold_is_strict(self):
        g = grid(['#########', '#......?#', '#......?#', '#......?#', '#########'])
        pose = (*g.to_world((2, 2)), 0.)
        boundary_xy = g.to_world((4, 2))
        distance = math.dist(boundary_xy, pose[:2])
        at_boundary = Planner(.04, frontier_min_translation_m=distance, goal_arrival_margin_m=.01).select(
            g, pose, Coverage(), FailureBook(), 0, False)
        below_boundary = Planner(.04, frontier_min_translation_m=distance-1e-6, goal_arrival_margin_m=.01).select(
            g, pose, Coverage(), FailureBook(), 0, False)
        self.assertEqual(at_boundary.state, 'goal')
        self.assertGreater(math.dist((at_boundary.goal.x, at_boundary.goal.y), pose[:2]), distance)
        self.assertEqual((below_boundary.goal.x, below_boundary.goal.y), boundary_xy)

    def test_frontier_zero_heading_vector_is_rejected_even_after_translation(self):
        g = grid(['#########', '#......?#', '#......?#', '#......?#', '#########'])
        pose = (*g.to_world((1, 2)), 0.)
        # A standoff below one grid step leaves only the anchor itself as a
        # candidate. It is far from the robot but has no defined look direction.
        result = Planner(.04, .04, frontier_fallback_standoff_m=.04).select(
            g, pose, Coverage(), FailureBook(), 0, False)
        self.assertGreater(result.frontier_count, 0)
        self.assertEqual(result.state, 'unreachable_frontiers_or_coverage')
        self.assertIsNone(result.goal)

    def test_coverage_can_still_choose_current_position(self):
        g = grid(['#########']+['#.......#']*5+['#########'])
        pose = (*g.to_world((2, 3)), 0.)
        coverage = cover_all(g)
        coverage.points.remove(coverage.key(g.to_world((5, 3))))
        default = Planner(.04, goal_arrival_margin_m=.01).select(g, pose, coverage, FailureBook(), 0, True)
        large_minimum = Planner(.04, frontier_min_translation_m=1., goal_arrival_margin_m=.01).select(
            g, pose, coverage, FailureBook(), 0, True)
        self.assertEqual(default, large_minimum)
        self.assertEqual(default.goal.kind, 'coverage')
        self.assertEqual((default.goal.x, default.goal.y), pose[:2])

    def test_frontier_goal_uses_reachable_inflated_free_space(self):
        g = grid(['#########', '#......?#', '#......?#', '#......?#', '#########'])
        planner = Planner(radius=.04, standoff=.25, goal_arrival_margin_m=.01)
        pose = (*g.to_world((2, 2)), 0.)
        result = planner.select(g, pose, Coverage(), FailureBook(), 0, False)
        self.assertEqual(result.state, 'goal')
        self.assertEqual(result.goal.kind, 'frontier')
        self.assertIn(g.to_cell(result.goal.x, result.goal.y), g.component((2, 2), g.safe_cells(.04)))

    def test_full_map_is_not_visual_search_complete(self):
        g = grid(['#######', '#.....#', '#.....#', '#.....#', '#######'])
        result = Planner(.04, .4).select(g, (*g.to_world((3, 2)), 0), Coverage(), FailureBook(), 0, False)
        self.assertEqual(result.state, 'perception_unverified')
        self.assertEqual(result.frontier_count, 0)
        self.assertGreater(result.uncovered_count, 0)

    def test_complete_requires_live_detector_even_with_old_coverage(self):
        g = grid(['#####', '#...#', '#...#', '#...#', '#####'])
        planner, pose = Planner(.04, .4), (*g.to_world((2, 2)), 0)
        coverage = cover_all(g)
        self.assertEqual(planner.select(g, pose, coverage, FailureBook(), 0, False).state, 'perception_unverified')
        self.assertEqual(planner.select(g, pose, coverage, FailureBook(), 0, True).state, 'complete')

    def test_unresolved_blacklist_does_not_mean_complete(self):
        g = grid(['#########', '#......?#', '#......?#', '#......?#', '#########'])
        planner, book, coverage = Planner(.04, .4, goal_arrival_margin_m=.01), FailureBook(radius=2, max_attempts=1), Coverage()
        pose = (*g.to_world((2, 2)), 0)
        first = planner.select(g, pose, coverage, book, 0, True)
        book.fail(first.goal, 0, 'navigation_failed')
        self.assertEqual(planner.select(g, pose, coverage, book, 100, True).state, 'blocked_frontiers_or_coverage')

    def test_failed_goal_has_bounded_retry_and_cooldown(self):
        book = FailureBook(max_attempts=2, cooldown=5)
        goal = Goal(1, 1, 0, 'frontier', (1, 1))
        book.fail(goal, 10, 'failure')
        self.assertFalse(book.allowed(goal.anchor, goal.kind, 11))
        self.assertTrue(book.allowed(goal.anchor, goal.kind, 15))
        book.fail(goal, 15, 'failure')
        self.assertFalse(book.allowed(goal.anchor, goal.kind, 100))

    def test_failure_is_not_cleared_just_because_anchor_is_outside_new_map(self):
        g = grid(['#####', '#...#', '#...#', '#...#', '#####'])
        book = FailureBook()
        book.fail(Goal(10, 10, 0, 'frontier', (10, 10)), 0, 'failed')
        book.resolve_observed(g, cover_all(g))
        self.assertEqual(len(book.entries), 1)
        result = Planner(.04, .4).select(g, (*g.to_world((2, 2)), 0), cover_all(g), book, 100, True)
        self.assertNotEqual(result.state, 'complete')


class FrontierFallbackTests(unittest.TestCase):
    def room(self, other_frontier=False):
        # A short narrow branch is observable from the roomy chamber, but its
        # closest arrival-safe endpoint is farther than the primary radius.
        rows = [['.' if 1 <= x <= 30 and 1 <= y <= 29 else '#'
                 for x in range(41)] for y in range(31)]
        for x in range(31, 34):
            rows[15][x] = '.'
        rows[15][34] = '?'
        if other_frontier:
            rows[29][15] = '?'
        return grid([''.join(row) for row in rows])

    def select(self, g, fallback=.8, *, book=None, now=0., radius=.25, start=(22, 15)):
        return Planner(radius=radius, frontier_fallback_standoff_m=fallback).select(
            g, (*g.to_world(start), 0.), Coverage(), book or FailureBook(), now, False)

    def test_fallback_is_finite_bounded_and_can_be_disabled_at_primary_radius(self):
        for primary, fallback in ((.55, 0), (.55, .54), (.55, 1.00001), (.55, math.inf),
                                  (.55, math.nan), (.55, True), (.55, '0.8'),
                                  (0., .8), (math.nan, .8), (True, .8)):
            with self.subTest(primary=primary, fallback=fallback), self.assertRaises(ValueError):
                Planner(standoff=primary, frontier_fallback_standoff_m=fallback)
        g = self.room()
        for fallback in (.55, .59):
            selected = self.select(g, fallback)
            self.assertEqual(selected.state, 'unreachable_frontiers_or_coverage')
            self.assertEqual(selected.frontier_count, 1)
        selected = self.select(g)
        self.assertEqual(selected.state, 'goal')
        self.assertEqual(g.to_cell(*selected.goal.anchor), (33, 15))
        distance = math.dist((selected.goal.x, selected.goal.y), selected.goal.anchor)
        self.assertGreater(distance, .55)
        self.assertLessEqual(distance, .8)
        end = g.to_cell(selected.goal.x, selected.goal.y)
        self.assertIn(end, g.component((22, 15), g.safe_cells(.25)))
        self.assertIn(end, g.safe_cells(.25+.05+math.hypot(g.resolution, g.resolution)/2))
        self.assertTrue(g.line_free(end, (33, 15)))

    def test_all_primary_anchors_precede_nearer_anchor_fallback(self):
        g = self.room(other_frontier=True)
        primary = self.select(g, .55)
        enabled = self.select(g, .8)
        self.assertEqual(primary.state, 'goal')
        self.assertEqual(enabled, primary)
        self.assertNotEqual(g.to_cell(*enabled.goal.anchor), (33, 15))
        self.assertEqual(self.select(self.room()).goal.anchor, g.to_world((33, 15)))

    def test_fallback_preserves_blacklist_cooldown_and_attempt_limit(self):
        g, book = self.room(), FailureBook(max_attempts=2, cooldown=8.)
        first = self.select(g, book=book)
        book.fail(first.goal, 0., 'navigation_failed')
        self.assertEqual(self.select(g, book=book, now=1.).state, 'retry_wait')
        retried = self.select(g, book=book, now=8.)
        self.assertEqual(retried.state, 'goal')
        book.fail(retried.goal, 8., 'navigation_failed')
        self.assertEqual(self.select(g, book=book, now=100.).state, 'blocked_frontiers_or_coverage')
        self.assertEqual(len(book.entries), 1)

    def test_fallback_cannot_relax_footprint_or_known_safe_connectivity(self):
        g = self.room()
        self.assertEqual(self.select(g, radius=.5).state, 'unreachable_frontiers_or_coverage')
        values = list(g.data)
        for y in range(1, 30):
            if y != 15:
                values[y*g.width+18] = 100
        narrow = Grid(g.width, g.height, g.resolution, 0., 0., 0., tuple(values))
        self.assertIn((33, 15), narrow.component((10, 15)))
        selected = self.select(narrow, start=(10, 15))
        self.assertEqual(selected.state, 'unreachable_frontiers_or_coverage')
        self.assertGreater(selected.frontier_count, 0)

    def test_coverage_does_not_use_frontier_fallback(self):
        g = self.room()
        values = list(g.data)
        values[15*g.width+34] = 100
        g = Grid(g.width, g.height, g.resolution, 0., 0., 0., tuple(values))
        coverage = cover_all(g)
        coverage.points.remove(coverage.key(g.to_world((33, 15))))
        selected = Planner().select(g, (*g.to_world((22, 15)), 0.), coverage, FailureBook(), 0, True)
        self.assertEqual(selected.state, 'unreachable_frontiers_or_coverage')
        self.assertEqual((selected.frontier_count, selected.uncovered_count), (0, 1))

    def test_frozen_run22_final_grid_recovers_view_but_departure_remains_incomplete(self):
        archive = Path(__file__).resolve().parents[1]/'experiments/run22'
        departure = json.loads((archive/'observer-snapshots/0033-phase-canceling.json').read_text())
        final = json.loads((archive/'final-save.json').read_text())['snapshot']
        mission = json.loads(next((archive/'mission').glob('mission_*.json')).read_text())
        for name in ('departure', 'final'):
            with self.subTest(grid=name):
                raw = departure['map'] if name == 'departure' else final['map']
                origin = raw['origin'] if name == 'departure' else [*raw['origin']['xyz'][:2], 0.]
                observed = Grid(raw['width'], raw['height'], raw['resolution'], *origin,
                                tuple(departure['map_data'] if name == 'departure' else raw['data']))
                planning, filled = observed.with_free_prior(mission['home'][:2], .30-.05)
                self.assertEqual(filled, 0)
                tf = departure['pose'] if name == 'departure' else final['transforms']['map->base_link']
                q = tf.get('quaternion_xyzw', (0., 0., 0., 1.))
                yaw = tf.get('yaw', math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2)))
                pose = (*tf['xyz'][:2], yaw)
                primary = Planner(frontier_fallback_standoff_m=.55).select(
                    planning, pose, Coverage(), FailureBook(), 0, False, observed_grid=observed)
                selected = Planner().select(planning, pose, Coverage(), FailureBook(), 0, False,
                                            observed_grid=observed)
                self.assertEqual(primary.state, 'unreachable_frontiers_or_coverage')
                self.assertEqual(selected.frontier_count, 3)
                if name == 'departure':
                    self.assertEqual(selected, primary)
                    continue
                self.assertEqual(selected.state, 'goal')
                self.assertEqual(observed.to_cell(*selected.goal.anchor), (80, 31))
                distance = math.dist((selected.goal.x, selected.goal.y), selected.goal.anchor)
                self.assertGreaterEqual(distance, .5830951981733254-1e-9)
                self.assertLessEqual(distance, .8)
                end = planning.to_cell(selected.goal.x, selected.goal.y)
                self.assertIn(end, planning.component(planning.to_cell(*pose[:2]), planning.safe_cells(.25)))
                self.assertIn(end, planning.safe_cells(.25+.05+math.hypot(planning.resolution, planning.resolution)/2))
                self.assertTrue(planning.line_free(end, (80, 31)))


class ArrivalMarginTests(unittest.TestCase):
    def test_arrival_margin_must_be_finite_positive(self):
        for margin in (0, -.01, math.inf, math.nan, True):
            with self.subTest(margin=margin), self.assertRaisesRegex(ValueError, 'goal_arrival_margin_m'):
                Planner(goal_arrival_margin_m=margin)

    def test_arrival_disk_maps_only_to_radius_safe_cells_in_rotated_grid(self):
        for angle in (0., .37, math.pi/2):
            rows = ['#'*41]+['#'+'.'*32+'?'*7+'#']*29+['#'*41]
            g = grid(rows, resolution=.05, origin=(2.3, -1.7, angle))
            planner = Planner()
            selection = planner.select(g, (*g.to_world((12, 15)), 0.), Coverage(), FailureBook(), 0, False)
            self.assertEqual(selection.state, 'goal')
            safe = g.safe_cells(planner.radius)
            # Exercise actual to_cell rounding at all arrival-disk directions,
            # including the tolerance boundary, rather than duplicating its
            # radius formula as the only assertion.
            for fraction in (0., .25, .5, .75, 1.):
                for i in range(96):
                    offset = planner.goal_arrival_margin_m*fraction
                    a = i*2*math.pi/96
                    arrived = (selection.goal.x+offset*math.cos(a), selection.goal.y+offset*math.sin(a))
                    self.assertIn(g.to_cell(*arrived), safe)

    def test_endpoint_margin_does_not_inflate_connecting_corridor(self):
        rows = [list('#'*41)]+[list('#'+'.'*38+'?#') for _ in range(29)]+[list('#'*41)]
        for y in range(1, 30):
            if not 10 <= y <= 20:
                for x in range(18, 23):
                    rows[y][x] = '#'
        g = grid([''.join(row) for row in rows], resolution=.05)
        start = (10, 15)
        planner = Planner()
        selected = planner.select(g, (*g.to_world(start), 0.), Coverage(), FailureBook(), 0, False)
        self.assertEqual(selected.state, 'goal')
        end = g.to_cell(selected.goal.x, selected.goal.y)
        self.assertGreater(end[0], 23)
        self.assertIn(end, g.component(start, g.safe_cells(.25)))
        # Endpoints cannot occupy the .55 m passage, but the R=.25 route can
        # traverse it to reach a roomy observation position in the next room.
        endpoint_radius = .25+.05+math.hypot(g.resolution, g.resolution)/2
        self.assertNotIn(end, g.component(start, g.safe_cells(endpoint_radius)))

    def test_coverage_endpoints_receive_same_arrival_protection(self):
        g = grid(['#'*31]+['#'+'.'*29+'#']*29+['#'*31], resolution=.05)
        pose = (*g.to_world((24, 15)), 0.)
        coverage = cover_all(g)
        coverage.points.remove(coverage.key(g.to_world((18, 15))))
        selected = Planner().select(g, pose, coverage, FailureBook(), 0, True)
        self.assertEqual(selected.state, 'goal')
        self.assertEqual(selected.goal.kind, 'coverage')
        self.assertNotEqual((selected.goal.x, selected.goal.y), pose[:2])
        safe = g.safe_cells(.25)
        for a in range(0, 360, 5):
            arrived = (selected.goal.x+.05*math.cos(math.radians(a)),
                       selected.goal.y+.05*math.sin(math.radians(a)))
            self.assertIn(g.to_cell(*arrived), safe)

    def test_no_roomy_endpoint_stays_incomplete_for_frontier_or_coverage(self):
        for unknown in (False, True):
            interior = '#'+'.'*38+('?' if unknown else '.')+'#'
            g = grid(['#'*41]+[interior]*11+['#'*41], resolution=.05)
            selected = Planner().select(g, (*g.to_world((10, 6)), 0.), Coverage(), FailureBook(), 0, True)
            self.assertEqual(selected.state, 'unreachable_frontiers_or_coverage')
            self.assertIsNone(selected.goal)
            self.assertGreater(selected.uncovered_count, 0)
            self.assertEqual(selected.frontier_count > 0, unknown)

    def test_run10_style_small_arrival_error_can_leave_original_safe_disk(self):
        rows = [list('.'*41) for _ in range(41)]
        rows[10][10] = '#'
        g = grid([''.join(row) for row in rows], resolution=.05)
        nominal = g.to_world((15, 13))
        arrived = nominal[0]-.041, nominal[1]
        self.assertIn((15, 13), g.safe_cells(.25))
        self.assertNotIn(g.to_cell(*arrived), g.safe_cells(.25))
        self.assertLess(math.dist(nominal, arrived), .05)
        # The new endpoint condition excludes this thin-margin nominal goal.
        endpoint_radius = .25+.05+math.hypot(g.resolution, g.resolution)/2
        self.assertNotIn((15, 13), g.safe_cells(endpoint_radius))


class PerceptionTests(unittest.TestCase):
    def test_coverage_has_range_view_angle_and_wall_occlusion(self):
        g = grid(['#########', '#...#...#', '#...#...#', '#...#...#', '#########'])
        coverage = Coverage()
        camera = g.to_world((2, 2))
        coverage.observe(g, camera, 0, .01, .7, math.pi/2)
        self.assertTrue(coverage.contains(g.to_world((3, 2)), .1))
        self.assertFalse(coverage.contains(g.to_world((1, 2)), .1))
        self.assertFalse(coverage.contains(g.to_world((6, 2)), .1))

    def test_confirm_multiple_targets_without_count_prior(self):
        store = TargetStore(min_hits=3)
        for stamp in (1., 2., 3.):
            store.add('object', (1, 1, .2), stamp, .9)
            store.add('object', (3, 1, .2), stamp, .9)
        self.assertEqual(len(store.confirmed()), 2)
        self.assertEqual({t.uid for t in store.confirmed()}, {1, 2})

    def test_replayed_frames_and_outliers_do_not_confirm(self):
        store = TargetStore()
        for _ in range(5):
            store.add('object', (1, 1, .2), 1, .9)
        self.assertEqual(len(store.confirmed()), 0)
        store.add('object', (9, 9, .2), 2, .9)
        self.assertEqual(len(store.confirmed()), 0)
        store.add('object', (1, 1, .2), 10, .9)
        self.assertEqual(store.targets[0].hits, 1)

    def test_different_instance_ids_do_not_merge(self):
        store = TargetStore(min_hits=1)
        store.add('tag', (1, 1, .2), 1, .9, 'A')
        store.add('tag', (1.1, 1, .2), 1, .9, 'B')
        self.assertEqual(len(store.confirmed()), 2)

    def test_bad_confidence_and_nonfinite_points_rejected(self):
        store = TargetStore(min_hits=1)
        self.assertIsNone(store.add('object', (1, 1, .2), 1, .2))
        self.assertIsNone(store.add('object', (1, 1, .2), 1, 2))
        self.assertIsNone(store.add('object', (math.nan, 1, .2), 1, .9))
        self.assertEqual(store.confirmed(), [])


class PlanningEvidenceTests(unittest.TestCase):
    def test_near_field_unknowns_choose_translated_safe_goal_without_evidence_credit(self):
        # Simplified run6: an observed corridor and three unseen near-field
        # cells. The independently verified start disk only affects collision.
        rows = ['#'*61]+['#'+'.'*59+'#']*15+['#'*61]
        unknown = {(9, 8), (8, 9), (9, 9)}
        for x, y in unknown:
            rows[y] = rows[y][:x]+'?'+rows[y][x+1:]
        raw = grid(rows, resolution=.05)
        cx, cy = raw.to_world((8, 8))
        pose = (cx+.014, cy-.011, -.029)
        planning, filled = raw.with_free_prior(pose[:2], .25)
        self.assertEqual(filled, 3)
        self.assertNotIn(raw.to_cell(*pose[:2]), raw.safe_cells(.25))
        result = Planner().select(planning, pose, cover_all(raw), FailureBook(), 0,
                                  True, observed_grid=raw)
        self.assertEqual(result.state, 'goal')
        self.assertEqual(result.goal.kind, 'frontier')
        xy = (result.goal.x, result.goal.y)
        self.assertGreater(math.dist(xy, pose[:2]), .20)
        self.assertNotEqual(xy, result.goal.anchor)
        self.assertIn(planning.to_cell(*xy), planning.component(planning.to_cell(*pose[:2]), planning.safe_cells(.25)))
        self.assertEqual(result.unobserved_count, 3)
        self.assertGreater(result.frontier_count, 0)
        self.assertTrue(all(raw.value(p) == -1 for p in unknown))

    def test_only_current_safe_position_does_not_complete_prior_frontier(self):
        raw = grid(['#####', '#...#', '#..?#', '#...#', '#####'])
        pose = (*raw.to_world((2, 2)), 0.)
        planning, filled = raw.with_free_prior(pose[:2], .25)
        self.assertEqual(filled, 1)
        self.assertEqual(planning.safe_cells(.10), {(2, 2)})
        result = Planner(.10).select(planning, pose, cover_all(raw), FailureBook(), 0,
                                     True, observed_grid=raw)
        self.assertEqual(result.state, 'unreachable_frontiers_or_coverage')
        self.assertIsNone(result.goal)
        self.assertEqual(result.unobserved_count, 1)
        self.assertGreater(result.frontier_count, 0)
        self.assertEqual(raw.value((3, 2)), -1)

    def test_prior_does_not_remove_raw_frontiers_or_resolve_failures(self):
        rows = ['#'*21] + ['#'+'.'*19+'#']*19 + ['#'*21]
        rows[10] = rows[10][:10]+'?'+rows[10][11:]
        observed = grid(rows)
        pose = (*observed.to_world((10, 10)), 0.)
        planning, count = observed.with_free_prior(pose[:2], .25)
        self.assertEqual(count, 1)
        coverage = cover_all(observed)
        book = FailureBook(radius=.01)
        anchor = observed.to_world((9, 10))
        book.fail(Goal(*anchor, 0, 'frontier', anchor), 0, 'not_observed')
        result = Planner().select(planning, pose, coverage, book, 1, True, observed_grid=observed)
        self.assertEqual(result.frontier_count, 4)
        self.assertEqual(result.unobserved_count, 1)
        self.assertEqual(result.uncovered_count, 0)  # No impossible camera goal on a raw unknown cell.
        self.assertNotEqual(result.state, 'complete')
        self.assertEqual(len(book.entries), 1)
        self.assertEqual(result.goal.kind, 'frontier')
        self.assertTrue(observed.frontier(observed.to_cell(*result.goal.anchor)))
        self.assertEqual(observed.value((10, 10)), -1)

    def test_prior_alone_is_not_a_completed_map_even_without_raw_frontiers(self):
        observed = grid(['?'*21]*21)
        pose = (*observed.to_world((10, 10)), 0.)
        planning, count = observed.with_free_prior(pose[:2], .8)
        result = Planner().select(planning, pose, cover_all(planning), FailureBook(), 0,
                                  True, observed_grid=observed)
        self.assertGreater(count, 0)
        self.assertEqual(result.frontier_count, 0)
        self.assertEqual(result.unobserved_count, count)
        self.assertEqual(result.state, 'mapping_evidence_incomplete')

    def test_observed_and_planning_geometry_must_match(self):
        planning = grid(['.....']*5)
        shifted = grid(['.....']*5, origin=(.1, 0., 0.))
        with self.assertRaisesRegex(ValueError, 'identical geometry'):
            Planner(.04).select(planning, (.25, .25, 0), Coverage(), FailureBook(), 0,
                                False, observed_grid=shifted)


class AdapterCallbackTests(unittest.TestCase):
    def test_timeout_latches_and_disqualifies_success_after_return_checks_pass(self):
        # Execute the actual adapter methods without importing or starting ROS.
        # Only external time/pose/publishing and result destination are fixtures.
        node_path = Path(__file__).resolve().parents[1] / 'src' / 'limo_maze_mission' / 'limo_maze_mission' / 'node.py'
        tree = ast.parse(node_path.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MissionNode')
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in
                   ('tick', 'save_result', 'bounded_observation_enabled', 'observation_snapshot')]
        namespace = {'time': SimpleNamespace(monotonic=lambda: 1000., strftime=lambda _: 'test'),
                     'Path': Path, 'json': json}
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(node_path), 'exec'), namespace)
        # Keep the disposable result under this test's workspace in restricted
        # Windows environments whose system temp directory is not writable.
        with tempfile.TemporaryDirectory(prefix='limo-timeout-regression-',
                                         dir=Path(__file__).resolve().parent) as directory:
            node = SimpleNamespace(
                last_clock=None, home=(1., 2., 0.), canceling=False, phase='exploring',
                grid=object(), map_received=1000., started=0., mission_timed_out=False,
                seconds=lambda: 1000., get_pose=lambda: (1., 2., 0.), publish_status=lambda: None,
                result_written=False, map_complete=True, visual_complete=True,
                failures=SimpleNamespace(entries=[]), targets=SimpleNamespace(confirmed=lambda: [], targets=[]),
                coordinates_valid=True, map_epoch='test-epoch', prior_filled_cells=0,
                sources=set(), events=[], get_logger=lambda: SimpleNamespace(info=lambda _: None, error=self.fail),
                cfg={'mission_timeout_s': 900., 'map_max_age_s': 5., 'target_definition_verified': True,
                     'map_frame': 'map', 'initial_clearance_radius_m': 0., 'initial_clearance_margin_m': .05,
                     'initial_clearance_source': '', 'output_dir': directory})
            def start_return(reason):
                node.reason, node.phase = reason, 'returning'
            node.start_return = start_return
            node.bounded_observation_enabled = lambda: namespace['bounded_observation_enabled'](node)
            node.observation_snapshot = lambda: namespace['observation_snapshot'](node)
            namespace['tick'](node)
            self.assertTrue(node.mission_timed_out)
            self.assertEqual(node.reason, 'mission_timeout_incomplete')
            self.assertEqual(node.phase, 'returning')
            # Model later complete home checks: geometry/coverage are complete,
            # the return pose is verified, and the detector definition is valid.
            node.phase, node.reason = 'done', 'returned_after_deadline'
            namespace['save_result'](node, True, node.home)
            result = json.loads((Path(directory) / 'mission_test.json').read_text(encoding='utf-8'))
            self.assertTrue(result['returned_pose_verified'])
            self.assertTrue(result['map_complete'] and result['visual_coverage_complete'])
            self.assertTrue(result['mission_timed_out'])
            self.assertFalse(result['mission_success'])
            self.assertIn('mission_timeout_incomplete', result['reason'])

    def test_late_goal_acceptance_after_cancel_timeout_is_canceled(self):
        # Load only this callback's AST so the regression also runs without ROS.
        node_path = Path(__file__).resolve().parents[1] / 'src' / 'limo_maze_mission' / 'limo_maze_mission' / 'node.py'
        tree = ast.parse(node_path.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MissionNode')
        callback = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'on_goal_response')
        namespace = {}
        exec(compile(ast.Module(body=[callback], type_ignores=[]), str(node_path), 'exec'), namespace)

        class Future:
            def result(self):
                return handle
            def add_done_callback(self, callback):
                pass

        class Handle:
            accepted = True
            def get_result_async(self):
                return Future()

        class Adapter:
            token, send_pending, canceling, phase = 1, True, False, 'blocked'
            observation_active = None
            def issue_cancel(self, token):
                self.canceled_token = token
            def on_goal_result(self, future, token):
                pass

        handle, adapter = Handle(), Adapter()
        namespace['on_goal_response'](adapter, Future(), 1)
        self.assertIs(adapter.goal_handle, handle)
        self.assertEqual(adapter.canceled_token, 1)


class MapEpochAdapterTests(unittest.TestCase):
    def adapter(self):
        # Run actual callbacks and startup/cancel transitions without ROS. The
        # fixture substitutes only time, TF, Nav2 transport and publishers.
        node_path = Path(__file__).resolve().parents[1] / 'src' / 'limo_maze_mission' / 'limo_maze_mission' / 'node.py'
        tree = ast.parse(node_path.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MissionNode')
        names = ('on_epoch', 'tick', 'block', 'cancel_to', 'bounded_observation_enabled')
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
        marker = type('Marker', (), {'DELETEALL': 3})
        namespace = {'time': SimpleNamespace(monotonic=lambda: 1000.),
                     'Marker': marker, 'MarkerArray': SimpleNamespace}
        exec(compile(ast.Module(body=methods, type_ignores=[]), str(node_path), 'exec'), namespace)
        node = type('Adapter', (), {name: namespace[name] for name in names})()
        node.__dict__.update(
            last_clock=None, seconds=lambda: 1000., created=990., home=None,
            map_epoch=None, map_epoch_ready=False, phase='waiting', reason='',
            grid=object(), map_received=1000., started=None, canceling=False,
            goal_handle=None, send_pending=False, token=12, map_complete=False,
            visual_complete=False, coordinates_valid=True,
            observation_active=None,
            cfg={'startup_wait_s': 60., 'map_max_age_s': 5., 'autonomous': False,
                 'cancel_timeout_s': 10.},
            pose_queries=[], sent_goals=[], cancellations=[], markers=[], events=[], results=[],
            nav=SimpleNamespace(server_is_ready=lambda: True), publish_status=lambda: None)
        def pose():
            node.pose_queries.append(True)
            return (1., 2., .3)
        node.get_pose = pose
        node.send_goal = node.sent_goals.append
        node.issue_cancel = node.cancellations.append
        node.marker_pub = SimpleNamespace(publish=node.markers.append)
        node.event = lambda name, **fields: node.events.append((name, fields))
        node.save_result = node.results.append
        return node

    def test_missing_and_blank_epoch_cannot_record_home_or_send_goals(self):
        for message in (None, '', ' \t\n'):
            with self.subTest(epoch=message):
                node = self.adapter()
                node.cfg['autonomous'] = True
                if message is not None:
                    node.on_epoch(SimpleNamespace(data=message))
                node.tick()
                self.assertIsNone(node.home)
                self.assertEqual(node.phase, 'waiting')
                self.assertEqual(node.pose_queries, [])
                self.assertEqual(node.sent_goals, [])

    def test_missing_epoch_times_out_explicitly_without_recording_home(self):
        node = self.adapter()
        node.created = 940.  # Exactly startup_wait_s elapsed.
        node.tick()
        self.assertEqual(node.reason, 'startup_no_map_epoch')
        self.assertTrue(node.canceling)
        node.tick()
        self.assertEqual(node.phase, 'blocked')
        self.assertEqual(node.results, [False])
        self.assertIsNone(node.home)
        self.assertEqual(node.sent_goals, [])

    def test_valid_epoch_allows_start_only_after_blank_state_is_replaced(self):
        node = self.adapter()
        node.on_epoch(SimpleNamespace(data=' session-a '))
        node.on_epoch(SimpleNamespace(data='   '))
        self.assertEqual(node.map_epoch, 'session-a')  # Audit ID is not erased.
        self.assertFalse(node.map_epoch_ready)
        node.tick()
        self.assertIsNone(node.home)
        node.on_epoch(SimpleNamespace(data=' session-a '))
        node.tick()
        self.assertTrue(node.map_epoch_ready)
        self.assertEqual(node.home, (1., 2., .3))
        self.assertEqual(node.phase, 'dry_run')
        self.assertEqual(node.events[-1][0], 'home_recorded')
        self.assertEqual(node.events[-1][1]['map_epoch'], 'session-a')
        self.assertEqual(node.sent_goals, [])

    def test_running_blank_or_changed_epoch_cancels_and_invalidates_coordinates(self):
        for message, reason in (('  ', 'map_epoch_missing_requires_reanchor'),
                                ('session-b', 'map_reset_requires_new_mission')):
            with self.subTest(epoch=message):
                node = self.adapter()
                node.map_epoch, node.map_epoch_ready = 'session-a', True
                node.home, node.phase, node.goal_handle = (1., 2., .3), 'exploring', object()
                node.on_epoch(SimpleNamespace(data=message))
                self.assertEqual(node.map_epoch, 'session-a')
                self.assertFalse(node.map_epoch_ready)
                self.assertFalse(node.coordinates_valid)
                self.assertEqual(node.reason, reason)
                self.assertEqual(node.phase, 'canceling')
                self.assertEqual(node.cancel_next, 'blocked')
                self.assertEqual(node.cancellations, [12])
                self.assertEqual(len(node.markers), 1)
                self.assertEqual(node.markers[0].markers[0].action, 3)
                self.assertEqual(node.sent_goals, [])


class ShutdownTests(unittest.TestCase):
    def run_cleanup(self, terminal=True, odometry='stopped', pending=True, accept_goal=True,
                    result_request_error=False, result_status=5, send_response_error=False):
        # Execute the real cleanup and action callbacks. Only ROS transport,
        # clock and odometry delivery are substituted; no navigation is started.
        path = Path(__file__).resolve().parents[1] / 'src/limo_maze_mission/limo_maze_mission/node.py'
        tree = ast.parse(path.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MissionNode')
        names = ('cancel_to', 'on_goal_response', 'issue_cancel', 'on_cancel_response',
                 'on_goal_result', 'block', 'bounded_observation_enabled')
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
        cleanup = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'shutdown_mission')
        clock = SimpleNamespace(now=0., steps=0)
        namespace = {'time': SimpleNamespace(monotonic=lambda: clock.now, strftime=lambda _: 'test'),
                     'Path': Path, 'json': json, 'isfinite': math.isfinite,
                     'Odometry': object, 'qos_profile_sensor_data': object(),
                     'GoalStatus': SimpleNamespace(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6)}
        exec(compile(ast.Module(body=methods+[cleanup], type_ignores=[]), str(path), 'exec'), namespace)
        class Future:
            def __init__(self):
                self.callbacks, self.value, self.error = [], None, None
            def result(self):
                if self.error is not None:
                    raise self.error
                return self.value
            def add_done_callback(self, callback):
                self.callbacks.append(callback)
            def finish(self, value):
                self.value = value
                for callback in self.callbacks:
                    callback(self)
        result, cancel, accepted = Future(), Future(), Future()
        def get_result():
            if result_request_error:
                raise RuntimeError('synchronous result request failure')
            return result
        handle = SimpleNamespace(accepted=True, get_result_async=get_result,
                                 cancel_goal_async=lambda: cancel)
        node = type('Adapter', (), {name: namespace[name] for name in names})()
        node.__dict__.update(token=1, goal_handle=None, send_pending=pending, canceling=False,
                             observation_active=None,
                             phase='exploring', cancel_response=False, cancel_terminal=False,
                             seconds=lambda: clock.now, event=lambda *a, **k: None,
                             saved=[], timer_stopped=False)
        node._mission_timer = SimpleNamespace(cancel=lambda: setattr(node, 'timer_stopped', True))
        def subscribe(_type, _topic, callback, _qos):
            node.odom_callback = callback
            return object()
        node.create_subscription = subscribe
        node.destroy_subscription = lambda _: None
        node.save_result = node.saved.append
        node.get_logger = lambda: SimpleNamespace(info=lambda _: None)
        def spin_once(_node, timeout_sec):
            clock.now += timeout_sec
            clock.steps += 1
            if pending and accept_goal and clock.steps == 2:
                accepted.value = handle
                if send_response_error:
                    accepted.error = RuntimeError('send RPC response unavailable')
                node.on_goal_response(accepted, node.token)  # Accepted after cancel intent.
            if pending and accept_goal and clock.steps == 3:
                cancel.finish(SimpleNamespace(goals_canceling=[object()]))
            if pending and accept_goal and terminal and clock.steps == 5:
                result.finish(SimpleNamespace(status=result_status, result=SimpleNamespace(error_msg='fixture')))
            if odometry != 'missing':
                stamp = clock.now-(2. if odometry == 'stale' else 0.)
                seconds = math.floor(stamp)
                velocity = .1 if odometry == 'moving' else (.006 if odometry == 'slow_linear' else 0.)
                angular = .021 if odometry == 'slow_angular' else 0.
                node.odom_callback(SimpleNamespace(
                    header=SimpleNamespace(stamp=SimpleNamespace(sec=seconds, nanosec=int((stamp-seconds)*1e9))),
                    twist=SimpleNamespace(twist=SimpleNamespace(
                        linear=SimpleNamespace(x=velocity, y=0., z=0.),
                        angular=SimpleNamespace(x=0., y=0., z=angular)))))
        namespace['rclpy'] = SimpleNamespace(ok=lambda: True, spin_once=spin_once)
        with tempfile.TemporaryDirectory(prefix='limo-shutdown-', dir=Path(__file__).parent) as directory:
            node.cfg = {'cancel_timeout_s': 2., 'output_dir': directory}
            evidence = namespace['shutdown_mission'](node, 'operator_sigint')
            audit = json.loads((Path(directory)/'shutdown_test.json').read_text())
            self.assertEqual(audit, evidence)
        self.assertTrue(node.timer_stopped)
        self.assertEqual(node.saved, [False])
        self.assertLessEqual(clock.now, 2.001)
        return evidence

    def test_pending_acceptance_is_canceled_and_terminal_precedes_stop_verification(self):
        result = self.run_cleanup()
        self.assertTrue(result['cancel_ack_received'])
        self.assertTrue(result['goal_terminal_verified'])
        self.assertTrue(result['stop_verified'])
        self.assertGreaterEqual(result['elapsed_s'], 1.)
        self.assertFalse(result['send_pending'] or result['goal_handle_present'])

    def test_cancel_ack_and_stationary_odometry_without_terminal_are_not_enough(self):
        result = self.run_cleanup(terminal=False)
        self.assertTrue(result['cancel_ack_received'])
        self.assertFalse(result['goal_terminal_verified'])
        self.assertFalse(result['stop_verified'])
        self.assertTrue(result['goal_handle_present'])

    def test_unanswered_send_remains_pending_at_bounded_exit(self):
        result = self.run_cleanup(accept_goal=False)
        self.assertTrue(result['send_pending'])
        self.assertFalse(result['goal_terminal_verified'])
        self.assertFalse(result['stop_verified'])
        self.assertFalse(result['cancel_ack_received'])
        self.assertTrue(result['late_acceptance_after_client_destroy_possible'])
        self.assertGreaterEqual(result['elapsed_s'], 2.)

    def test_synchronous_result_request_failure_cannot_block_late_acceptance_cancel(self):
        result = self.run_cleanup(result_request_error=True)
        self.assertTrue(result['cancel_ack_received'])
        self.assertTrue(result['goal_handle_present'])
        self.assertFalse(result['goal_terminal_verified'])
        self.assertFalse(result['stop_verified'])

    def test_unknown_or_active_result_status_is_not_terminal_evidence(self):
        for status in (0, 1, 2, 3):
            with self.subTest(status=status):
                result = self.run_cleanup(result_status=status)
                self.assertTrue(result['cancel_ack_received'])
                self.assertTrue(result['goal_handle_present'])
                self.assertFalse(result['goal_terminal_verified'])
                self.assertFalse(result['stop_verified'])

    def test_send_response_exception_preserves_unknown_ownership(self):
        result = self.run_cleanup(send_response_error=True)
        self.assertTrue(result['send_pending'])
        self.assertTrue(result['late_acceptance_after_client_destroy_possible'])
        self.assertFalse(result['goal_terminal_verified'])
        self.assertFalse(result['stop_verified'])

    def test_missing_moving_or_stale_odometry_keeps_stop_unverified(self):
        for kind in ('missing', 'moving', 'stale', 'slow_linear', 'slow_angular'):
            with self.subTest(odometry=kind):
                result = self.run_cleanup(odometry=kind, pending=False)
                self.assertTrue(result['goal_terminal_verified'])
                self.assertFalse(result['stop_verified'])


class ConfigurationConsistencyTests(unittest.TestCase):
    @unittest.skipUnless(importlib.util.find_spec('yaml') is not None,
                         'PyYAML unavailable; run with the ROS setup Python environment to validate generated Nav2 YAML')
    def test_generated_nav2_footprint_and_goal_tolerance_fit_mission_defaults(self):
        import yaml
        repository = Path(__file__).resolve().parents[1]
        configuration = yaml.safe_load((repository / 'src' / 'limo_maze_bringup' / 'config' / 'nav2.yaml').read_text(encoding='utf-8'))
        node_path = repository / 'src' / 'limo_maze_mission' / 'limo_maze_mission' / 'node.py'
        tree = ast.parse(node_path.read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MissionNode')
        init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
        assignment = next(n for n in init.body if isinstance(n, ast.Assign)
                          and any(isinstance(t, ast.Name) and t.id == 'defaults' for t in n.targets))
        defaults = ast.literal_eval(assignment.value)
        controller = configuration['controller_server']['ros__parameters']
        for name in controller['goal_checker_plugins']:
            tolerance = controller[name]['xy_goal_tolerance']
            self.assertGreater(defaults['frontier_min_translation_m'], tolerance,
                               'Mission frontier translation must exceed the configured Nav2 XY goal tolerance')
            self.assertGreater(Planner().frontier_min_translation_m, tolerance)
            self.assertGreaterEqual(defaults['goal_arrival_margin_m'], tolerance)
            self.assertGreaterEqual(Planner().goal_arrival_margin_m, tolerance)
            self.assertFalse(controller[name]['stateful'],
                             'Final goal success must recheck XY after heading adjustment')
        bt = configuration['bt_navigator']['ros__parameters']
        self.assertEqual(bt['default_server_timeout'], 200)
        self.assertEqual(bt['bt_loop_duration'], 10)
        self.assertEqual(bt['wait_for_service_timeout'], 1000)
        # Derive geometry from the generated configuration, not copied dimensions
        # or a duplicated radius constant. Nav2 pads each vertex along x and y.
        for name in ('local_costmap', 'global_costmap'):
            parameters = configuration[name][name]['ros__parameters']
            points = yaml.safe_load(parameters['footprint'])
            padding = parameters['footprint_padding']
            self.assertTrue(math.isfinite(padding) and padding >= 0)
            self.assertGreaterEqual(len(points), 3)
            required = max(math.hypot(abs(x)+padding, abs(y)+padding) for x, y in points)
            with self.subTest(costmap=name, required_radius=required):
                self.assertTrue(math.isfinite(required))
                self.assertGreaterEqual(defaults['robot_radius_m'], required,
                                        'MissionNode default must enclose the padded Nav2 footprint')
                self.assertGreaterEqual(Planner().radius, required,
                                        'Standalone Planner default must enclose the padded Nav2 footprint')


class RosStartupTests(unittest.TestCase):
    @unittest.skipUnless(importlib.util.find_spec('rclpy') is not None,
                         'ROS rclpy is unavailable; source the ROS environment to run startup regression')
    def test_construct_spin_and_destroy_with_autonomy_disabled(self):
        # A child process isolates the ROS context and domain from other tests
        # and any experiment on domain 73. No navigation server is required.
        source = Path(__file__).resolve().parents[1] / 'src' / 'limo_maze_mission'
        protocol_source = source.parent / 'limo_evidence_protocol'
        environment = dict(os.environ, ROS_DOMAIN_ID='74', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        environment['PYTHONPATH'] = os.pathsep.join(
            (str(source), str(protocol_source), environment.get('PYTHONPATH', '')))
        script = textwrap.dedent('''
            import ast
            from contextlib import contextmanager
            import inspect
            import json
            from pathlib import Path
            import tempfile
            import rclpy
            from rclpy.node import Node
            from limo_maze_mission.core import Grid
            from limo_maze_mission.node import MissionNode

            # Check all assigned adapter attributes against the real base API,
            # including read-only properties such as Node.handle.
            tree = ast.parse(inspect.getsource(MissionNode))
            assigned = {n.attr for n in ast.walk(tree)
                        if isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Store)
                        and isinstance(n.value, ast.Name) and n.value.id == 'self'}
            conflicts = assigned.intersection(dir(Node))
            assert not conflicts, 'Mission attributes shadow Node API: '+str(sorted(conflicts))

            @contextmanager
            def mission(parameters=()):
                # Exercise real ROS parameter parsing and declaration. Retain a
                # reference even if validation raises after Node construction.
                arguments = ['--ros-args'] if parameters else []
                for parameter in parameters:
                    arguments.extend(['-p', parameter])
                rclpy.init(args=arguments)
                node = MissionNode.__new__(MissionNode)
                try:
                    node.__init__()
                    assert node.cfg['autonomous'] is False
                    def unexpected_goal(*args, **kwargs):
                        raise AssertionError('Disabled-autonomy adapter sent a navigation goal')
                    node.nav.send_goal_async = unexpected_goal
                    yield node
                finally:
                    node.destroy_node()
                    rclpy.shutdown()

            values = [-1] * 81
            values[4*9+5], values[5*9+4] = 100, 20
            raw = Grid(9, 9, .1, 1.55, -2.45, 0., tuple(values))
            home = (2., -2., 1.2)
            with mission() as node:
                assert node.cfg['autonomous'] is False
                assert node.get_parameter('autonomous').value is False
                assert node.handle is not None  # The native rclpy handle survives.
                assert node.goal_handle is None
                assert node.mission_timed_out is False
                assert node.cfg['frontier_min_translation_m'] == .20
                assert node.planner.frontier_min_translation_m == node.cfg['frontier_min_translation_m']
                assert node.cfg['goal_arrival_margin_m'] == .05
                assert node.planner.goal_arrival_margin_m == .05
                assert node.cfg['frontier_fallback_standoff_m'] == .8
                assert node.planner.frontier_fallback_standoff_m == .8
                rclpy.spin_once(node, timeout_sec=0.6)
                assert node.phase == 'waiting'
                assert node.goal_handle is None and not node.send_pending
                assert node.cfg['initial_clearance_radius_m'] == 0.
                assert node.cfg['initial_clearance_source'] == ''
                assert node.cfg['initial_clearance_margin_m'] == .05
                node.grid, node.home = raw, home
                assert node.planning_grid() is raw
                assert node.prior_filled_cells == 0
                print('ROS_STARTUP_OK domain=74 autonomous=False spin=ok')

            for invalid in (
                    ('initial_clearance_radius_m:=0.3',),
                    ('initial_clearance_radius_m:=0.3', "initial_clearance_source:='   '"),
                    ('initial_clearance_radius_m:=0.05', 'initial_clearance_source:=fixture'),
                    ('initial_clearance_radius_m:=0.04', 'initial_clearance_source:=fixture')):
                try:
                    with mission(invalid):
                        raise AssertionError('Invalid clearance prior was accepted: '+str(invalid))
                except ValueError as error:
                    assert 'independent source and radius greater than its margin' in str(error)

            with tempfile.TemporaryDirectory(prefix='limo-adapter-regression-') as directory:
                with mission(('initial_clearance_radius_m:=0.3',
                              'frontier_min_translation_m:=0.32',
                              'frontier_fallback_standoff_m:=0.9',
                              'goal_arrival_margin_m:=0.08',
                              'initial_clearance_source:=independently_verified_test_fixture',
                              'output_dir:='+directory)) as node:
                    assert node.cfg['initial_clearance_radius_m'] == .3
                    assert node.cfg['frontier_min_translation_m'] == .32
                    assert node.planner.frontier_min_translation_m == .32
                    assert node.get_parameter('frontier_fallback_standoff_m').value == .9
                    assert node.planner.frontier_fallback_standoff_m == .9
                    assert node.cfg['goal_arrival_margin_m'] == .08
                    assert node.planner.goal_arrival_margin_m == .08
                    assert node.cfg['initial_clearance_margin_m'] == .05
                    assert node.get_parameter('initial_clearance_source').value == node.cfg['initial_clearance_source']
                    node.grid, node.map_epoch = raw, 'adapter-regression-epoch'
                    assert node.planning_grid() is raw  # No home means no anchor.
                    assert node.prior_filled_cells == 0
                    node.home = home
                    planning = node.planning_grid()
                    # At .1 m resolution, only this 3x3 square lies wholly in
                    # the effective .25 m disk. Two measured cells must survive.
                    expected_filled = {(x, y) for x in (3, 4, 5) for y in (3, 4, 5)} - {(5, 4), (4, 5)}
                    changed = {(x, y) for x, y in raw.cells()
                               if raw.data[y*9+x] != planning.data[y*9+x]}
                    assert changed == expected_filled, changed
                    assert node.prior_filled_cells == 7
                    assert planning is not raw and node.grid is raw
                    assert raw.data == tuple(values)
                    assert planning.data[4*9+5] == 100 and planning.data[5*9+4] == 20
                    assert planning.data[4*9+6] == -1  # Inside .30, outside .25: margin applied.
                    assert node.coverage.frames == 0 and not node.coverage.points
                    node.last_home_pose = (2.3, -2., 0.)
                    node.get_pose = lambda: (2.3, -2., 0.)
                    assert node.planning_grid() == planning  # The prior cannot follow the robot.
                    assert node.home == home and raw.data == tuple(values)
                    node.phase, node.reason = 'done', 'adapter_regression_only'
                    node.save_result(False)
                    paths = list(Path(directory).glob('mission_*.json'))
                    assert node.result_written and len(paths) == 1
                    result = json.loads(paths[0].read_text(encoding='utf-8'))
                    assert result['initial_clearance_prior'] == {
                        'enabled': True, 'coordinates_valid': True, 'frame_id': 'map',
                        'map_epoch': 'adapter-regression-epoch', 'center': [2., -2.],
                        'verified_radius_m': .3, 'margin_m': .05,
                        'source': 'independently_verified_test_fixture',
                        'currently_filled_unknown_cells': 7,
                        'sensor_observation': False, 'raw_slam_map_modified': False}
                    assert result['visual_coverage_complete'] is False
                    assert result['mission_success'] is False
                    assert result['returned_pose_verified'] is False
                    assert not node.send_pending and node.goal_handle is None
            print('ROS_CLEARANCE_ADAPTER_OK defaults=off validation=ok copy=ok audit=ok')
            # Exercise the actual main SIGINT path with a live context. Domain
            # 74 has only a stationary odometry fixture, no navigation server.
            import os
            import signal
            import subprocess
            import sys
            import time
            from nav_msgs.msg import Odometry
            rclpy.init(args=[])
            fixture = rclpy.create_node('mission_shutdown_odometry_fixture')
            publisher = fixture.create_publisher(Odometry, '/odom', 10)
            try:
                with tempfile.TemporaryDirectory(prefix='limo-signal-regression-') as directory:
                    log_path = Path(directory)/'child.log'
                    with log_path.open('w') as log:
                        child = subprocess.Popen([sys.executable, '-B', '-c',
                            'from limo_maze_mission.node import main; main()', '--ros-args',
                            '-p', 'autonomous:=false', '-p', 'use_sim_time:=false',
                            '-p', 'cancel_timeout_s:=3.0', '-p', 'output_dir:='+directory],
                            stdout=log, stderr=subprocess.STDOUT)
                        sent_signal = False
                        deadline = time.monotonic()+10.
                        try:
                            while child.poll() is None and time.monotonic() < deadline:
                                msg = Odometry()
                                msg.header.stamp = fixture.get_clock().now().to_msg()
                                publisher.publish(msg)
                                rclpy.spin_once(fixture, timeout_sec=.05)
                                if not sent_signal and 'Mission loaded' in log_path.read_text():
                                    child.send_signal(signal.SIGINT)
                                    sent_signal = True
                            assert child.poll() == 0, log_path.read_text()
                            assert sent_signal
                        finally:
                            if child.poll() is None:
                                child.kill()  # Own disabled-autonomy test process only.
                                child.wait(timeout=3.)
                    audit = json.loads(next(Path(directory).glob('shutdown_*.json')).read_text())
                    assert audit['reason'] == 'operator_sigint', audit
                    assert audit['goal_terminal_verified'] and audit['stop_verified'], audit
                    assert not audit['send_pending'] and not audit['goal_handle_present'], audit
                    assert audit['stop_verification_source'] == 'wheel_odometry'
            finally:
                fixture.destroy_node()
                rclpy.shutdown()
            print('ROS_SIGNAL_CLEANUP_OK context=alive terminal=verified odometry=stationary')
            print('ROS_TEARDOWN_OK')
        ''')
        result = subprocess.run([sys.executable, '-B', '-c', script], env=environment,
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout+'\n'+result.stderr)
        self.assertIn('ROS_STARTUP_OK domain=74 autonomous=False spin=ok', result.stdout)
        self.assertIn('ROS_CLEARANCE_ADAPTER_OK defaults=off validation=ok copy=ok audit=ok', result.stdout)
        self.assertIn('ROS_SIGNAL_CLEANUP_OK context=alive terminal=verified odometry=stationary', result.stdout)
        self.assertIn('ROS_TEARDOWN_OK', result.stdout)


if __name__ == '__main__':
    unittest.main()
