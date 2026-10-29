import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
from build_gazebo_world import build, parse_maze


class WorldExportTests(unittest.TestCase):
    def test_spawn_alignment_and_wall_geometry(self):
        root, meta = build('#####\n#S.T#\n#####', cell=0.6)
        models = {m.attrib['name']: m for m in root.findall('./world/model')}
        self.assertEqual(len([n for n in models if n.startswith('wall_')]), 12)
        self.assertEqual(models['wall_0_0'].findtext('pose'), '-0.6 0.6 0.3 0 0 0')
        self.assertEqual(meta['robot_spawn'], {'x': 0, 'y': 0, 'yaw': 0})
        self.assertAlmostEqual(meta['targets_ground_truth'][0]['x'], 1.485)
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

    def test_horizontal_and_vertical_corridors_use_side_walls(self):
        for text, normal in [
            ('#######\n#S.T..#\n#######', [0, 1]),
            ('#####\n#S..#\n###T#\n###.#\n#####', [-1, 0]),
            # A T junction has only the south side wall: check grid/world y.
            ('#######\n#.....#\n#S.T..#\n#######', [0, -1]),
        ]:
            with self.subTest(text=text):
                root, meta = build(text)
                self.assertEqual(meta['targets_ground_truth'][0]['wall_normal_world_xy'], normal)
                self.assert_target_geometry(text, root, meta)

    def test_dead_end_uses_end_wall(self):
        root, meta = build('#####\n#S.T#\n#####')
        self.assertEqual(meta['targets_ground_truth'][0]['wall_normal_world_xy'], [1, 0])
        self.assert_target_geometry('#####\n#S.T#\n#####', root, meta)

    def test_wall_free_target_fails_instead_of_obstructing_open_space(self):
        with self.assertRaisesRegex(ValueError, r'Target at grid \(2, 2\) has no adjacent wall'):
            build('#####\n#...#\n#.T.#\n#S..#\n#####')

    def test_checked_in_worlds_match_sources_and_preserve_target_counts(self):
        worlds = Path(__file__).resolve().parents[1] / 'worlds'
        for name, count, cell in [('example_maze', 3, .6), ('smoke_maze', 2, .8)]:
            with self.subTest(name=name):
                text = (worlds / (name+'.txt')).read_text(encoding='utf-8')
                root, meta = build(text, cell=cell)
                self.assertEqual(len(meta['targets_ground_truth']), count)
                self.assertEqual(ET.tostring(root), ET.tostring(ET.parse(worlds / (name+'.world')).getroot()))
                self.assert_target_geometry(text, root, meta)

    def assert_target_geometry(self, text, root, meta):
        """Independent AABB check against every traversable cell-centre edge."""
        rows, start, targets = parse_maze(text)
        cell = meta['cell_m']
        xy = lambda x, y: ((x-start[0])*cell, (start[1]-y)*cell)
        for i, target in enumerate(meta['targets_ground_truth']):
            model = root.find("./world/model[@name='target_surrogate_{}']".format(i))
            pose = list(map(float, model.findtext('pose').split()))
            size = list(map(float, model.findtext('./link/collision/geometry/box/size').split()))
            self.assertEqual(pose[:3], [target['x'], target['y'], target['z']])
            self.assertEqual(size, target['size_xyz'])
            self.assertEqual(list(targets[i]), target['grid_xy'])
            wx, wy = target['wall_grid_xy']
            self.assertEqual(rows[wy][wx], '#')
            normal = target['wall_normal_world_xy']
            axis = 0 if normal[0] else 1
            sign = normal[axis]
            wall_center = xy(wx, wy)
            # The outer face touches the wall; no gap or embedded box.
            self.assertAlmostEqual(pose[axis]+sign*size[axis]/2,
                                   wall_center[axis]-sign*cell/2)
            center = xy(*targets[i])
            gap = abs(pose[axis]-center[axis])-size[axis]/2
            self.assertAlmostEqual(gap, target['cell_center_clearance_m'])
            self.assertAlmostEqual(gap, cell/2-size[axis])
            low = [pose[a]-size[a]/2 for a in (0, 1)]
            high = [pose[a]+size[a]/2 for a in (0, 1)]
            for y, row in enumerate(rows):
                for x, char in enumerate(row):
                    if char == '#':
                        continue
                    for dx, dy in ((1, 0), (0, 1)):
                        if rows[y+dy][x+dx] == '#':
                            continue
                        p, q = xy(x, y), xy(x+dx, y+dy)
                        # Axis-aligned segment/AABB distance, independent of
                        # mounting metadata. Include the padded rectangular
                        # footprint's circumcircle. Runtime margin is tested
                        # separately against the generated Nav2 configuration.
                        distance = math.hypot(*[
                            max(low[a]-max(p[a], q[a]), min(p[a], q[a])-high[a], 0)
                            for a in (0, 1)])
                        self.assertGreater(distance, math.hypot(.20, .14))


if __name__ == '__main__':
    unittest.main()
