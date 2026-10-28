"""Guard Nav2 controller selection and checked-in experiment profiles without ROS."""
from copy import deepcopy
from pathlib import Path
import unittest

import yaml

from configure_nav2 import configure, parse_args


def defaults(plugin):
    # Minimal structural fixtures for the two released controller configurations.
    controller = {"FollowPath": {"plugin": plugin, "critics": ["ObstacleFootprint"],
                                  "CostCritic": {"consider_footprint": False}},
                  "goal_checker_plugins": ["checker"], "checker": {"xy_goal_tolerance": .25, "stateful": True},
                  "progress_checker_plugins": ["progress"], "progress": {"required_movement_radius": .5,
                                                                               "movement_time_allowance": 10.0}}
    cost = {"robot_base_frame": "base_footprint", "robot_radius": .22,
            "inflation_layer": {}, "static_layer": {}, "obstacle_layer": {"scan": {"topic": "scan"}}}
    return {"controller_server": {"ros__parameters": controller},
            "bt_navigator": {"ros__parameters": {"default_server_timeout": 20,
                "bt_loop_duration": 10, "wait_for_service_timeout": 1000}},
            "local_costmap": {"local_costmap": {"ros__parameters": deepcopy(cost)}},
            "global_costmap": {"global_costmap": {"ros__parameters": deepcopy(cost)}},
            "planner_server": {"ros__parameters": {"GridBased": {"allow_unknown": True}}},
            "velocity_smoother": {"ros__parameters": {"enable_stamped_cmd_vel": True}},
            "behavior_server": {"ros__parameters": {}},
            "collision_monitor": {"ros__parameters": {"base_frame_id": "base_footprint",
                "polygons": ["PolygonStop", "PolygonSlow"], "PolygonStop": {"enabled": True},
                "observation_sources": ["scan"], "scan": {"type": "scan", "topic": "scan"}}},
            "amcl": {}, "map_server": {}}


class Nav2ConfigurationTests(unittest.TestCase):
    def test_rpp_replaces_all_previous_controller_parameters_without_mutating_input(self):
        for plugin in ("dwb_core::DWBLocalPlanner", "nav2_mppi_controller::MPPIController"):
            with self.subTest(plugin=plugin):
                original = defaults(plugin)
                original["controller_server"]["ros__parameters"]["FollowPath"].update(
                    visualize=True, model_dt=.1, GoalAngleCritic={"enabled": True})
                before = deepcopy(original)
                result = configure(original, controller_profile="rpp")
                follow = result["controller_server"]["ros__parameters"]["FollowPath"]
                self.assertEqual(follow["plugin"],
                    "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController")
                for old in ("critics", "CostCritic", "GoalAngleCritic", "model_dt", "visualize"):
                    self.assertNotIn(old, follow)
                self.assertEqual(follow["desired_linear_vel"], .15)
                self.assertEqual(follow["rotate_to_heading_min_angle"], .3)
                self.assertTrue(follow["use_collision_detection"])
                self.assertFalse(follow["use_velocity_scaled_lookahead_dist"])
                self.assertFalse(follow["stateful"])
                self.assertEqual(configure(result, controller_profile="rpp"), result)
                self.assertEqual(original, before)
                # The controller selection must not change the common safety profile.
                installed = configure(original)
                del installed["controller_server"]["ros__parameters"]["FollowPath"]
                del result["controller_server"]["ros__parameters"]["FollowPath"]
                self.assertEqual(result, installed)

    def test_controller_default_output_is_separate_and_explicit_output_is_respected(self):
        _, installed = parse_args([])
        _, rpp = parse_args(["--controller", "rpp"])
        _, explicit = parse_args(["--controller", "rpp", "--output", "custom.yaml"])
        self.assertEqual(installed.controller, "installed")
        self.assertEqual(installed.output.name, "nav2.yaml")
        self.assertEqual(rpp.output.name, "nav2_rpp.yaml")
        self.assertNotEqual(installed.output, rpp.output)
        self.assertEqual(explicit.output, Path("custom.yaml"))

    def test_unknown_controller_profile_is_rejected_before_configuration(self):
        with self.assertRaises(ValueError):
            configure(defaults("nav2_mppi_controller::MPPIController"), controller_profile="typo")

    def test_checked_in_rpp_profile_regenerates_and_preserves_default_mppi(self):
        config = Path(__file__).resolve().parents[1] / "src/limo_maze_bringup/config"
        installed = yaml.safe_load((config / "nav2.yaml").read_text(encoding="utf-8"))
        rpp = yaml.safe_load((config / "nav2_rpp.yaml").read_text(encoding="utf-8"))
        self.assertEqual(installed["controller_server"]["ros__parameters"]["FollowPath"]["plugin"],
                         "nav2_mppi_controller::MPPIController")
        self.assertEqual(configure(installed, controller_profile="rpp"), rpp)
        self.assertEqual(configure(rpp, controller_profile="rpp"), rpp)

    def test_smoke_slam_profile_changes_only_documented_small_maze_parameters(self):
        config = Path(__file__).resolve().parents[1] / "src/limo_maze_bringup/config"
        original = yaml.safe_load((config / "slam.yaml").read_text(encoding="utf-8"))
        experimental = yaml.safe_load((config / "slam_smoke_experimental.yaml").read_text(encoding="utf-8"))
        expected = deepcopy(original)
        expected["slam_toolbox"]["ros__parameters"].update(
            loop_search_space_dimension=1.0, distance_variance_penalty=.05)
        self.assertEqual(experimental, expected)
        self.assertTrue(experimental["slam_toolbox"]["ros__parameters"]["do_loop_closing"])
        self.assertTrue(experimental["slam_toolbox"]["ros__parameters"]["use_scan_matching"])

    def test_dwb_low_speed_and_unknown_space_remain_conservative(self):
        result = configure(defaults("dwb_core::DWBLocalPlanner"))
        controller = result["controller_server"]["ros__parameters"]
        self.assertEqual(controller["FollowPath"]["max_vel_x"], .15)
        self.assertEqual(controller["FollowPath"]["max_vel_y"], 0)
        self.assertFalse(result["planner_server"]["ros__parameters"]["GridBased"]["allow_unknown"])
        self.assertNotIn("amcl", result)
        self.assertNotIn("map_server", result)

    def test_mppi_time_step_matches_controller_period(self):
        original = defaults("nav2_mppi_controller::MPPIController")
        original["controller_server"]["ros__parameters"]["FollowPath"]["model_dt"] = .05
        result = configure(original)
        controller = result["controller_server"]["ros__parameters"]
        follow = controller["FollowPath"]
        self.assertEqual(follow["model_dt"], 1/controller["controller_frequency"])
        self.assertEqual(follow["motion_model"], "DiffDrive")
        self.assertEqual(follow["vx_max"], .15)
        self.assertEqual(follow["wz_max"], .4)
        self.assertTrue(follow["CostCritic"]["consider_footprint"])
        self.assertEqual(controller["progress"]["required_movement_radius"], .15)
        self.assertGreater(controller["progress"]["movement_time_allowance"], 3.1416/.4+.15/.15)
        self.assertNotIn("max_vel_x", follow)
        self.assertEqual(original["controller_server"]["ros__parameters"]["FollowPath"]["model_dt"], .05)

    def test_complete_command_chain_uses_twist_and_actual_footprint(self):
        result = configure(defaults("nav2_mppi_controller::MPPIController"))
        for node in ("controller_server", "velocity_smoother", "behavior_server", "collision_monitor"):
            params = result[node]["ros__parameters"]
            self.assertFalse(params["enable_stamped_cmd_vel"])
            self.assertTrue(params["use_sim_time"])
        monitor = result["collision_monitor"]["ros__parameters"]
        self.assertEqual(monitor["cmd_vel_in_topic"], "cmd_vel_smoothed")
        self.assertEqual(monitor["cmd_vel_out_topic"], "cmd_vel")
        self.assertEqual(monitor["polygons"], ["FootprintApproach"])
        approach = monitor["FootprintApproach"]
        self.assertTrue(approach["enabled"])
        self.assertEqual(approach["footprint_topic"], "/local_costmap/published_footprint")
        self.assertEqual(monitor["scan"]["topic"], "/scan")
        self.assertEqual(monitor["base_frame_id"], "base_link")

    def test_unreviewed_plugin_is_rejected(self):
        with self.assertRaises(ValueError):
            configure(defaults("some_vendor::UnreviewedController"))

    def test_arrival_tolerance_and_ack_profile_are_regenerated_for_both_controllers(self):
        for plugin in ("dwb_core::DWBLocalPlanner", "nav2_mppi_controller::MPPIController"):
            with self.subTest(plugin=plugin):
                original = defaults(plugin)
                result = configure(original)
                controller = result["controller_server"]["ros__parameters"]
                self.assertEqual(controller["checker"]["xy_goal_tolerance"], .05)
                self.assertEqual(controller["checker"]["yaw_goal_tolerance"], .26)
                self.assertFalse(controller["checker"]["stateful"])
                if plugin == "dwb_core::DWBLocalPlanner":
                    self.assertEqual(controller["FollowPath"]["xy_goal_tolerance"], .05)
                bt = result["bt_navigator"]["ros__parameters"]
                self.assertEqual(bt["default_server_timeout"], 200)
                self.assertEqual(bt["bt_loop_duration"], 10)
                self.assertEqual(bt["wait_for_service_timeout"], 1000)
                self.assertEqual(configure(result), result)
                self.assertEqual(original["bt_navigator"]["ros__parameters"]["default_server_timeout"], 20)
                self.assertTrue(original["controller_server"]["ros__parameters"]["checker"]["stateful"])


if __name__ == "__main__":
    unittest.main()
