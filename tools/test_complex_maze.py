"""Static geometry checks for the complex maze; no ROS or simulator is started.

This file and its SVG are offline evaluator ground truth. Their coordinates,
topology and target count must never be supplied to the autonomous mission.
Run: python -m unittest discover -s tools -p test_complex_maze.py -v
Print geometry evidence: python tools/test_complex_maze.py --summary
"""
from collections import Counter, deque
import hashlib
import json
import math
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

from build_gazebo_world import build, parse_maze


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'worlds/complex_maze'
CELL = .8
WALL_HEIGHT = .6
ROBOT_RADIUS = .25
ARRIVAL_MARGIN = .05
RASTER_MARGIN = .05
SAFE_RADIUS = ROBOT_RADIUS + ARRIVAL_MARGIN + RASTER_MARGIN


def topology(rows, start):
    free = {(x, y) for y, row in enumerate(rows) for x, c in enumerate(row) if c != '#'}
    adjacent = {p: {q for q in ((p[0]-1, p[1]), (p[0]+1, p[1]),
                              (p[0], p[1]-1), (p[0], p[1]+1)) if q in free} for p in free}
    reached, pending = {start}, deque([start])
    while pending:
        for q in adjacent[pending.popleft()] - reached:
            reached.add(q)
            pending.append(q)
    edges = {tuple(sorted((p, q))) for p in free for q in adjacent[p]}
    degrees = Counter({p: len(neighbors) for p, neighbors in adjacent.items()})
    return free, edges, reached, degrees


def aabbs(root):
    """Read actual SDF collision boxes, independent of generator metadata."""
    boxes = {}
    for model in root.findall('./world/model'):
        name = model.attrib['name']
        if name == 'floor':
            continue
        pose = tuple(map(float, model.findtext('pose').split()))
        size = tuple(map(float, model.findtext('./link/collision/geometry/box/size').split()))
        if pose[3:] != (0., 0., 0.) or any(s <= 0 for s in size):
            raise ValueError('Expected unrotated positive collision box: ' + name)
        if pose[2]+size[2]/2 <= 0:
            continue
        boxes[name] = tuple((pose[a]-size[a]/2, pose[a]+size[a]/2) for a in (0, 1))
    return boxes


def segment_distance(p, q, box):
    """Exact separation between an axis-aligned segment and an AABB."""
    if p[0] != q[0] and p[1] != q[1]:
        raise ValueError('Only axis-aligned centreline edges are supported')
    return math.hypot(*(max(low-max(p[a], q[a]), min(p[a], q[a])-high, 0.)
                        for a, (low, high) in enumerate(box)))


def geometry_evidence():
    source = BASE.with_suffix('.txt').read_text(encoding='utf-8')
    rows, start, target_cells = parse_maze(source)
    root = ET.parse(BASE.with_suffix('.world')).getroot()
    meta = json.loads(BASE.with_suffix('.metadata.json').read_text(encoding='utf-8'))
    free, edges, reached, degrees = topology(rows, start)
    boxes = aabbs(root)
    xy = lambda p: ((p[0]-start[0])*CELL, (start[1]-p[1])*CELL)
    wall_clearance = min(segment_distance((0., 0.), (0., 0.), box)
                         for name, box in boxes.items() if name.startswith('wall_'))
    spawn_clearance = min(segment_distance((0., 0.), (0., 0.), box) for box in boxes.values())
    route_clearance = min(segment_distance(xy(p), xy(q), box)
                         for p, q in edges for box in boxes.values())
    return {
        'role': 'offline_evaluator_ground_truth_only',
        'source_sha256': hashlib.sha256(BASE.with_suffix('.txt').read_bytes()).hexdigest(),
        'world_sha256': hashlib.sha256(BASE.with_suffix('.world').read_bytes()).hexdigest(),
        'dimensions_cells': [len(rows[0]), len(rows)], 'cell_m': CELL,
        'floor_dimensions_m': [len(rows[0])*CELL, len(rows)*CELL],
        'wall_height_m': WALL_HEIGHT, 'wall_boxes': sum(n.startswith('wall_') for n in boxes),
        'traversable_cells': len(free), 'traversable_area_m2': len(free)*CELL**2,
        'all_cells_connected': reached == free, 'adjacent_cell_edges': len(edges),
        'independent_cycles': len(edges)-len(free)+1 if reached == free else None,
        'dead_ends_including_spawn': len([p for p, d in degrees.items() if d == 1]),
        'dead_ends_excluding_spawn': len([p for p, d in degrees.items() if d == 1 and p != start]),
        'junctions': len([p for p, d in degrees.items() if d >= 3]),
        'target_count': len(target_cells), 'robot_spawn': meta['robot_spawn'],
        'spawn_nearest_wall_m': wall_clearance, 'spawn_nearest_collision_m': spawn_clearance,
        'verified_spawn_disk_radius_m': .30,
        'route_check': 'exact AABB distance for every traversable cell-centre edge',
        'robot_radius_m': ROBOT_RADIUS, 'arrival_margin_m': ARRIVAL_MARGIN,
        'costmap_cell_reserve_m': RASTER_MARGIN, 'required_route_clearance_m': SAFE_RADIUS,
        'minimum_route_collision_clearance_m': route_clearance,
        'remaining_route_margin_m': route_clearance-SAFE_RADIUS,
        'limits': ['Static collision geometry only; no Gazebo or ROS execution.',
                   'No guarantee of Nav2 completion, SLAM accuracy, or visual target detection.',
                   'Target positions, layout and count are not mission inputs.'],
    }


class ComplexMazeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = BASE.with_suffix('.txt').read_text(encoding='utf-8')
        cls.rows, cls.start, cls.targets = parse_maze(cls.source)
        cls.sdf = ET.parse(BASE.with_suffix('.world')).getroot()
        cls.meta = json.loads(BASE.with_suffix('.metadata.json').read_text(encoding='utf-8'))
        cls.boxes = aabbs(cls.sdf)
        cls.free, cls.edges, cls.reached, cls.degrees = topology(cls.rows, cls.start)

    def xy(self, p):
        return ((p[0]-self.start[0])*CELL, (self.start[1]-p[1])*CELL)

    def test_connected_topology_has_two_cycles_and_four_non_start_dead_ends(self):
        self.assertEqual((len(self.rows[0]), len(self.rows)), (15, 11))
        self.assertEqual(self.reached, self.free)
        self.assertEqual(len(self.free), 71)
        self.assertEqual(len(self.edges)-len(self.free)+1, 2)
        self.assertEqual(sum(d == 1 and p != self.start for p, d in self.degrees.items()), 4)
        self.assertEqual(sum(d >= 3 for d in self.degrees.values()), 7)
        self.assertEqual(len(self.targets), 5)

    def test_committed_sdf_and_metadata_exactly_match_source_and_parameters(self):
        generated, meta = build(self.source, cell=CELL, wall_height=WALL_HEIGHT)
        self.assertEqual(ET.tostring(self.sdf), ET.tostring(generated))
        self.assertEqual(self.meta, meta)
        self.assertEqual(self.meta['role'], 'simulator_and_offline_evaluator_only')
        self.assertEqual(len(self.boxes), 94+5)
        for name, box in self.boxes.items():
            if name.startswith('wall_'):
                self.assertAlmostEqual(box[0][1]-box[0][0], CELL)
                self.assertAlmostEqual(box[1][1]-box[1][0], CELL)
                size = self.sdf.find(f"./world/model[@name='{name}']/link/collision/geometry/box/size")
                self.assertAlmostEqual(float(size.text.split()[2]), WALL_HEIGHT)
        self.assertFalse(self.sdf.findall('.//uri'), 'World must not include robots or remote models')

    def test_spawn_independent_aabb_check_supports_point_three_metre_disk(self):
        self.assertEqual(self.start, (1, 1))
        self.assertEqual(self.meta['robot_spawn'], {'x': 0, 'y': 0, 'yaw': 0})
        walls = [b for n, b in self.boxes.items() if n.startswith('wall_')]
        targets = [b for n, b in self.boxes.items() if n.startswith('target_')]
        wall_distance = min(segment_distance((0., 0.), (0., 0.), b) for b in walls)
        self.assertAlmostEqual(wall_distance, .4)
        self.assertGreaterEqual(wall_distance+1e-12, .30)
        self.assertGreater(min(segment_distance((0., 0.), (0., 0.), b) for b in targets), 2.)

    def test_every_nominal_route_edge_clears_padded_robot_plus_raster_reserve(self):
        # Checking whole segments (not just sampled points) covers turns by a
        # circular envelope and proves a connected nominal route to every cell.
        # The extra .05 m exceeds sqrt(2)*.05/2 cell-centre quantization error.
        self.assertGreater(RASTER_MARGIN, math.sqrt(2)*.05/2)
        for p, q in self.edges:
            for name, box in self.boxes.items():
                self.assertGreaterEqual(segment_distance(self.xy(p), self.xy(q), box)+1e-12,
                                        SAFE_RADIUS, (p, q, name))
        target_clearance = min(segment_distance(self.xy(p), self.xy(q), b)
                               for p, q in self.edges for n, b in self.boxes.items()
                               if n.startswith('target_'))
        self.assertAlmostEqual(target_clearance, .37)
        self.assertAlmostEqual(target_clearance-SAFE_RADIUS, .02)

    def test_targets_are_flush_wall_mounted_and_do_not_cut_off_dead_ends(self):
        for i, item in enumerate(self.meta['targets_ground_truth']):
            self.assertEqual(item['grid_xy'], list(self.targets[i]))
            tx, ty = item['grid_xy']
            wx, wy = item['wall_grid_xy']
            self.assertEqual(self.rows[wy][wx], '#')
            self.assertEqual(abs(wx-tx)+abs(wy-ty), 1)
            box = self.boxes[f'target_surrogate_{i}']
            wall = self.boxes[f'wall_{wx}_{wy}']
            axis = 0 if wx != tx else 1
            direction = (wx-tx) if axis == 0 else (ty-wy)
            self.assertAlmostEqual(box[axis][1 if direction > 0 else 0],
                                   wall[axis][0 if direction > 0 else 1])
            self.assertGreaterEqual(segment_distance(self.xy((tx, ty)), self.xy((tx, ty)), box),
                                    SAFE_RADIUS-1e-12)

    def test_distance_checker_detects_corridor_obstruction_and_diagonal_input(self):
        self.assertEqual(segment_distance((0., 0.), (.8, 0.), ((.35, .45), (-.1, .1))), 0.)
        self.assertAlmostEqual(segment_distance((0., 0.), (.8, 0.), ((.2, .4), (.37, .4))), .37)
        with self.assertRaises(ValueError):
            segment_distance((0., 0.), (.8, .8), ((.2, .4), (.37, .4)))

    def test_reference_svg_is_labelled_offline_ground_truth(self):
        image = ET.parse(ROOT / 'docs/assets/complex-maze-layout.svg').getroot()
        visible_text = ' '.join(image.itertext())
        self.assertIn('OFFLINE EVALUATOR GROUND TRUTH', visible_text)
        self.assertIn('Never provide this layout or target count to the mission', visible_text)
        self.assertEqual(len([e for e in image.iter() if 'data-target-id' in e.attrib]), 5)
        metadata = image.find('{http://www.w3.org/2000/svg}metadata')
        self.assertIsNotNone(metadata)
        world_hash = hashlib.sha256(BASE.with_suffix('.world').read_bytes()).hexdigest()
        self.assertIn(f'world_sha256={world_hash};', metadata.text)


if __name__ == '__main__':
    if sys.argv[1:] == ['--summary']:
        print(json.dumps(geometry_evidence(), indent=2))
    else:
        unittest.main()
