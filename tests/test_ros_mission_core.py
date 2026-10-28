"""Independent of ROS: safety, information boundary and multi-target evidence."""
import math
import ast
import random
from pathlib import Path
import sys
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


class PlanningTests(unittest.TestCase):
    def test_frontier_goal_uses_reachable_inflated_free_space(self):
        g = grid(['#########', '#......?#', '#......?#', '#......?#', '#########'])
        planner = Planner(radius=.04, standoff=.25)
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
        planner, book, coverage = Planner(.04, .4), FailureBook(radius=2, max_attempts=1), Coverage()
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


class AdapterCallbackTests(unittest.TestCase):
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
            def issue_cancel(self, token):
                self.canceled_token = token
            def on_goal_result(self, future, token):
                pass

        handle, adapter = Handle(), Adapter()
        namespace['on_goal_response'](adapter, Future(), 1)
        self.assertIs(adapter.handle, handle)
        self.assertEqual(adapter.canceled_token, 1)


if __name__ == '__main__':
    unittest.main()
