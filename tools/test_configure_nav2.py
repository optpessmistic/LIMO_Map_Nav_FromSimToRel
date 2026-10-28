"""Guard version-dependent Nav2 configuration without importing ROS or YAML."""
from copy import deepcopy
import unittest

from configure_nav2 import configure


def defaults(plugin):
    # Minimal structural fixtures for the two released controller configurations.
    controller = {"FollowPath": {"plugin": plugin, "critics": ["ObstacleFootprint"],
                                  "CostCritic": {"consider_footprint": False}},
                  "goal_checker_plugins": ["checker"], "checker": {"xy_goal_tolerance": .25},
                  "progress_checker_plugins": ["progress"], "progress": {"required_movement_radius": .5,
                                                                               "movement_time_allowance": 10.0}}
    cost = {"robot_base_frame": "base_footprint", "robot_radius": .22,
            "inflation_layer": {}, "static_layer": {}, "obstacle_layer": {"scan": {"topic": "scan"}}}
    return {"controller_server": {"ros__parameters": controller},
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


if __name__ == "__main__":
    unittest.main()
