"""Safety, information-boundary, coverage, and mission-outcome regression tests."""
import json
from pathlib import Path
import unittest

from sim.core import Config, Explorer, FREE, Observation, SCENARIOS, World, generated_maze, run_mission, shortest_path
from sim.render import write_artifacts


class ObservationTests(unittest.TestCase):
    def test_agent_starts_unknown(self):
        agent = Explorer(7, 7, (1, 1), Config())
        self.assertEqual(agent.known, {})
        self.assertIsNone(agent.target)

    def test_wall_blocks_detection_even_inside_sensor_range(self):
        world = World(["#######", "#S#T..#", "#.....#", "#######"])
        obs = world.observe(world.start, 10, 10)
        self.assertEqual(obs.cells[(2, 1)], "#")
        self.assertNotIn((3, 1), obs.cells)
        self.assertIsNone(obs.target)

    def test_diagonal_corner_does_not_leak(self):
        world = World(["#####", "#S#.#", "##T.#", "#####"])
        self.assertNotIn(world.target, world.observe(world.start, 10, 10).cells)

    def test_range_limits_geometry_and_target_separately(self):
        world = World(["########", "#S...T.#", "########"])
        obs = world.observe(world.start, 5, 2)
        self.assertIn(world.target, obs.cells)
        self.assertNotIn(world.target, obs.searched_cells)
        self.assertIsNone(obs.target)
        self.assertNotIn((7, 1), obs.cells)

    def test_unobserved_target_does_not_change_agent_actions(self):
        hidden = World(SCENARIOS["unreachable"])
        absent = World([r.replace("T", ".") for r in SCENARIOS["unreachable"]])
        a, b = run_mission(hidden, Config()), run_mission(absent, Config())
        self.assertEqual(a["trajectory"], b["trajectory"])
        self.assertEqual(a["frames"], b["frames"])
        self.assertFalse(a["metrics"]["target_detected"])

    def test_multiple_target_ids_are_stable_and_only_observed_ids_are_exposed(self):
        world = World(["##########", "#S.T....T#", "##########"])
        obs = world.observe(world.start, 3, 2)
        self.assertEqual([t.target_id for t in obs.targets], ["item_3_1"])
        self.assertEqual(obs.targets, world.observe(world.start, 3, 2).targets)
        other_world = World(["##########", "#S.T.....#", "##########"])
        self.assertEqual(obs, other_world.observe(other_world.start, 3, 2))

    def test_unknown_free_space_is_not_plannable(self):
        known = {(1, 1): FREE, (3, 1): FREE}
        self.assertIsNone(shortest_path(known, (1, 1), {(3, 1)}, 5, 3))
        known[(2, 1)] = FREE
        self.assertEqual(shortest_path(known, (1, 1), {(3, 1)}, 5, 3), [(1, 1), (2, 1), (3, 1)])


class MissionTests(unittest.TestCase):
    def assert_legal_trajectory(self, result, world):
        trajectory = [tuple(p) for p in result["trajectory"]]
        known = {}
        for frame, old, new in zip(result["frames"], trajectory, trajectory[1:]):
            known.update({(x, y): v for x, y, v in frame["new_cells"]})
            self.assertEqual(known.get(new), FREE, "Action must use already observed free space")
            self.assertEqual(world.move(old, new), new)
        self.assertEqual(trajectory[0], world.start)
        self.assertEqual(trajectory[-1], world.start)
        self.assertLessEqual(result["metrics"]["steps"], result["config"]["max_steps"])

    def test_found_then_return(self):
        world = World(SCENARIOS["demo"])
        result = run_mission(world, Config(policy="find_then_return"))
        self.assert_legal_trajectory(result, world)
        self.assertTrue(result["metrics"]["target_detected"])
        self.assertIn(result["metrics"]["status"], ("found_returned", "found_explored_returned"))
        first = result["metrics"]["first_detection_step"]
        self.assertIn(result["frames"][first]["phase"], ("return", "done"))

    def test_full_exploration_with_multiple_seeds(self):
        for seed in range(10):
            with self.subTest(seed=seed):
                world = generated_maze(seed, 11, 9)
                result = run_mission(world, Config(policy="explore_then_return"))
                self.assert_legal_trajectory(result, world)
                m = result["metrics"]
                self.assertEqual(m["status"], "found_explored_returned")
                self.assertEqual(m["reachable_free_coverage"], 1)
                self.assertEqual(m["searched_reachable_free_coverage"], 1)
                self.assertEqual(m["detected_target_map_grid_xy"], list(world.target))
                self.assertEqual(result["target_annotations"][0]["grid_xy"], list(world.target))

    def test_zero_one_three_unknown_targets_across_seeds(self):
        for seed in range(6):
            trajectories = []
            for count in (0, 1, 3):
                with self.subTest(seed=seed, count=count):
                    world = generated_maze(seed, 11, 9, target_count=count)
                    result = run_mission(world, Config())
                    self.assert_legal_trajectory(result, world)
                    m = result["metrics"]
                    self.assertTrue(m["search_complete_reachable_region"])
                    self.assertEqual(m["detected_target_count"], count)
                    self.assertTrue(m["all_targets_detected_ground_truth"])
                    self.assertEqual({t["id"]: tuple(t["grid_xy"]) for t in result["target_annotations"]}, world.targets)
                    trajectories.append(result["trajectory"])
            # Unknown target count cannot affect the complete-exploration route.
            self.assertEqual(trajectories[0], trajectories[1])
            self.assertEqual(trajectories[1], trajectories[2])

    def test_multiple_occluded_branch_targets_are_all_annotated(self):
        world = World(SCENARIOS["multi_branch"])
        self.assertEqual(world.observe(world.start, 3, 2).targets, ())
        result = run_mission(world, Config())
        self.assertEqual(result["metrics"]["detected_target_count"], 3)
        self.assertEqual(result["metrics"]["status"], "found_explored_returned")
        self.assertTrue(all(t["first_detection_step"] > 0 for t in result["target_annotations"]))

    def test_unreachable_targets_are_counted_only_by_offline_evaluator(self):
        world = World(SCENARIOS["multi_unreachable"])
        result = run_mission(world, Config())
        m = result["metrics"]
        self.assertEqual(m["target_count_ground_truth"], 3)
        self.assertEqual(m["reachable_target_count_ground_truth"], 1)
        self.assertEqual(m["detected_target_count"], 1)
        self.assertEqual(m["missed_target_count_ground_truth"], 2)
        self.assertFalse(m["all_targets_detected_ground_truth"])
        self.assertTrue(m["all_reachable_targets_detected_ground_truth"])
        self.assertEqual([t["id"] for t in result["target_annotations"]], ["item_3_1"])
        self.assertNotIn("target_count", result["config"])

    def test_absent_target_finishes_without_false_positive(self):
        world = World(SCENARIOS["absent"])
        result = run_mission(world, Config())
        self.assert_legal_trajectory(result, world)
        self.assertEqual(result["metrics"]["status"], "not_found_explored_returned")
        self.assertEqual(result["metrics"]["reachable_free_coverage"], 1)
        self.assertEqual(result["metrics"]["searched_reachable_free_coverage"], 1)

    def test_unreachable_target_is_not_mislabeled_found_or_global_complete(self):
        world = World(SCENARIOS["unreachable"])
        result = run_mission(world, Config())
        self.assert_legal_trajectory(result, world)
        m = result["metrics"]
        self.assertEqual(m["status"], "not_found_explored_returned")
        self.assertTrue(m["target_exists_ground_truth"])
        self.assertFalse(m["target_reachable_ground_truth"])
        self.assertEqual(m["reachable_free_coverage"], 1)
        self.assertLess(m["observed_grid_fraction"], 1)

    def test_exploration_budget_returns_but_does_not_claim_completion(self):
        world = generated_maze(5)
        result = run_mission(world, Config(explore_budget=4))
        self.assert_legal_trajectory(result, world)
        m = result["metrics"]
        self.assertEqual(m["status"], "search_incomplete_returned")
        self.assertFalse(m["search_complete_reachable_region"])
        self.assertEqual(m["return_reason"], "exploration_budget")
        self.assertLess(m["reachable_free_coverage"], 1)

    def test_total_budget_always_reserves_return(self):
        world = generated_maze(4)
        for budget in (0, 1, 2, 3, 11, 30):
            with self.subTest(max_steps=budget):
                result = run_mission(world, Config(max_steps=budget))
                self.assert_legal_trajectory(result, world)
                self.assertEqual(result["metrics"]["status"], "search_incomplete_returned")
                self.assertEqual(result["metrics"]["return_reason"], "total_budget_return_reserve")

    def test_geometric_coverage_alone_does_not_finish_search(self):
        world = World(["##########", "#S......T#", "##########"])
        result = run_mission(world, Config(sensor_range=20, target_range=1, policy="explore_then_return"))
        self.assertGreater(result["metrics"]["steps"], 0)
        self.assertEqual(result["metrics"]["status"], "found_explored_returned")
        self.assertEqual(result["metrics"]["searched_reachable_free_coverage"], 1)

    def test_tiny_map_can_complete_without_movement(self):
        result = run_mission(World(["###", "#S#", "###"]), Config())
        self.assertEqual(result["metrics"]["steps"], 0)
        self.assertEqual(result["metrics"]["status"], "not_found_explored_returned")

    def test_loss_of_known_return_path_is_explicit_failure(self):
        agent = Explorer(5, 5, (1, 1), Config())
        agent.ingest(Observation({(3, 3): FREE}, searched_cells=frozenset({(3, 3)})))
        self.assertIsNone(agent.choose((3, 3), 2))
        self.assertEqual(agent.status, "failed_no_known_return_path")

    def test_seed_is_reproducible(self):
        a = run_mission(generated_maze(8, 9, 9), Config())
        b = run_mission(generated_maze(8, 9, 9), Config())
        self.assertEqual(a, b)

    def test_invalid_inputs_are_rejected(self):
        for rows in (["###", "#.#", "###"], ["S", ".."], ["SST"], ["S?"]):
            with self.subTest(rows=rows), self.assertRaises(ValueError):
                World(rows)
        for config in (Config(sensor_range=.5), Config(target_range=4), Config(max_steps=-1)):
            with self.subTest(config=config), self.assertRaises(ValueError):
                config.validate()
        for count in (-1, 10000):
            with self.subTest(target_count=count), self.assertRaises(ValueError):
                generated_maze(0, 5, 5, target_count=count)

    def test_artifacts_contain_replay_and_match_run(self):
        result = run_mission(World(SCENARIOS["demo"]), Config())
        tmp = Path(__file__).resolve().parents[1] / "outputs" / "simulation" / "test_artifacts"
        paths = write_artifacts(result, tmp)
        loaded = json.loads(Path(paths["json"]).read_text(encoding="utf-8"))
        self.assertEqual(loaded["metrics"], result["metrics"])
        self.assertEqual(Path(paths["maze"]).read_text(encoding="utf-8").splitlines(), result["offline_ground_truth"]["rows"])
        self.assertIn('id="slider"', Path(paths["html"]).read_text(encoding="utf-8"))
        self.assertIn("OFFLINE ground truth", Path(paths["svg"]).read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
