"""Prevent the TF trial from silently changing run24 navigation parameters."""
from pathlib import Path
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]


class TfExperimentConfigTests(unittest.TestCase):
    def test_only_rpp_wait_differs_from_run24(self):
        old = yaml.safe_load((ROOT / 'experiments/run24/nav2-rpp-03m.yaml').read_text())
        new = yaml.safe_load((ROOT / 'src/limo_maze_bringup/config/nav2_rpp_tf_experimental.yaml').read_text())
        controller = new['controller_server']['ros__parameters']
        self.assertEqual(controller['FollowPath']['transform_tolerance'], .5)
        self.assertEqual(controller['general_goal_checker']['xy_goal_tolerance'], .03)
        controller['FollowPath']['transform_tolerance'] = .1
        self.assertEqual(old, new)
