import math
import unittest
from build_gazebo_world import build, parse_maze


class WorldExportTests(unittest.TestCase):
    def test_spawn_alignment_and_wall_geometry(self):
        root, meta = build('#####\n#S.T#\n#####', cell=0.6)
        models = {m.attrib['name']: m for m in root.findall('./world/model')}
        self.assertEqual(len([n for n in models if n.startswith('wall_')]), 12)
        self.assertEqual(models['wall_0_0'].findtext('pose'), '-0.6 0.6 0.3 0 0 0')
        self.assertEqual(meta['robot_spawn'], {'x': 0, 'y': 0, 'yaw': 0})
        self.assertAlmostEqual(meta['targets_ground_truth'][0]['x'], 1.44)
        self.assertIsNotNone(models['wall_0_0'].find('./link/collision/geometry/box'))
        self.assertEqual(len(root.findall('./world/plugin')), 5)

    def test_multiple_targets_and_no_planner_map(self):
        root, meta = build('#######\n#ST.TT#\n#######')
        self.assertEqual(len(meta['targets_ground_truth']), 3)
        self.assertEqual(meta['role'], 'simulator_and_offline_evaluator_only')
        self.assertEqual(len([m for m in root.findall('./world/model') if m.attrib['name'].startswith('target_surrogate_')]), 3)

    def test_bad_input(self):
        for text in ['###\n#S\n###', '###\n#.#\n###', '###\n#SS\n###', '...\n.S.\n...']:
            with self.assertRaises(ValueError):
                parse_maze(text)
        for cell in [0, -1, math.nan, math.inf]:
            with self.assertRaises(ValueError):
                build('###\n#S#\n###', cell=cell)


if __name__ == '__main__':
    unittest.main()
