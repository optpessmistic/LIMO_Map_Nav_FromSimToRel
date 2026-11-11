"""Cross-component budget and scene binding checks; no ROS or simulator."""
import ast
import hashlib
from pathlib import Path
import sys
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
for package in ('limo_evidence_protocol', 'limo_maze_mission'):
    sys.path.insert(0, str(ROOT/'src'/package))
from limo_evidence_protocol.retention import Policy as ProducerPolicy
from limo_maze_mission.retention import Policy as ConsumerPolicy, SLOT_RESERVATION, WINDOW_RESERVATION
from limo_maze_mission.observation import Policy as ObservationPolicy


def defaults(package, class_name):
    tree = ast.parse((ROOT/'src'/package/package/'node.py').read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    init = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
    assignment = next(n for n in init.body if isinstance(n, ast.Assign) and
                      any(isinstance(t, ast.Name) and t.id == 'defaults' for t in n.targets))
    # Perception has arithmetic defaults; evaluate only its local constant AST.
    return eval(compile(ast.Expression(assignment.value), '<constant defaults>', 'eval'), {'__builtins__': {}})


class ComplexMazeProfileTests(unittest.TestCase):
    def setUp(self):
        self.profile = yaml.safe_load((ROOT/'config/complex_maze.yaml').read_text(encoding='utf-8'))
        self.mission = self.profile['limo_maze_mission']['ros__parameters']
        self.detector = self.profile['red_rgbd_sim_baseline']['ros__parameters']

    def test_parameters_are_declared_and_private_outputs_are_not_reused(self):
        for package, cls, values in (
                ('limo_maze_mission', 'MissionNode', self.mission),
                ('limo_maze_perception', 'RedRGBDBaseline', self.detector)):
            self.assertFalse(set(values) - set(defaults(package, cls)) - {'use_sim_time'})
            self.assertIs(values['use_sim_time'], True)
        self.assertNotIn('output_dir', self.mission)
        self.assertNotIn('retention_output_dir', self.detector)
        self.assertEqual(set(self.profile), {'limo_maze_mission', 'red_rgbd_sim_baseline'})

    def test_retention_limits_support_the_declared_capacity_on_both_sides(self):
        m, p = self.mission, self.detector
        consumer = ConsumerPolicy(enabled=m['require_retained_inspection'],
            max_total_samples=m['retention_max_total_samples'],
            max_windows=m['retention_max_windows'], max_metadata_bytes=m['retention_max_metadata_bytes'])
        producer = ProducerPolicy(max_jobs=p['retention_max_jobs'],
            max_requests=p['retention_max_requests'], max_file_bytes=p['retention_max_file_bytes'])
        self.assertGreaterEqual(producer.max_jobs, consumer.max_total_samples)
        self.assertGreaterEqual(producer.max_requests, consumer.max_total_samples)
        reservation = consumer.max_total_samples*SLOT_RESERVATION + consumer.max_windows*WINDOW_RESERVATION
        self.assertGreaterEqual(consumer.max_metadata_bytes, reservation)
        observation = ObservationPolicy(robot_radius_m=.25, arrival_margin_m=.05, station_spacing_m=1.,
            max_stations=m['observation_max_stations'], headings_per_station=m['observation_headings_per_station'],
            max_dispatches=m['observation_max_dispatches'], relocation_radius_m=m['observation_relocation_radius_m'])
        views = observation.max_stations*observation.headings_per_station
        self.assertGreaterEqual(observation.max_dispatches, views)
        # Extra capacity must remain for exploration, not just final viewpoints.
        self.assertGreater(consumer.max_total_samples, views*m['inspection_required_frames'])
        self.assertGreater(consumer.max_windows, views)
        self.assertGreater(m['mission_timeout_s'], m['observation_stage_timeout_s'])

    def test_start_clearance_claim_is_bound_to_the_exact_scene(self):
        digest = hashlib.sha256((ROOT/'worlds/complex_maze.world').read_bytes()).hexdigest()
        self.assertEqual(self.mission['initial_clearance_source'], 'complex_world_sha256_'+digest)
        self.assertEqual(self.mission['initial_clearance_radius_m'], .30)
        # No simulator ground-truth map, target location/count or metadata input.
        serialized = repr(self.profile)
        self.assertNotIn('.metadata.json', serialized)
        self.assertNotIn('targets_ground_truth', serialized)

    def test_slow_writer_budget_does_not_expire_requests_or_observation_first(self):
        m, p = self.mission, self.detector
        # Three serial frame jobs plus one checkpoint that may already be active.
        queued_write_budget = (m['inspection_required_frames'] + 1) * p['retention_worker_timeout_s']
        self.assertGreaterEqual(p['retention_request_timeout_s'], queued_write_budget)
        # Keep time for settling/acquisition and the final durable checkpoint.
        self.assertGreater(m['inspection_timeout_s'],
                           p['retention_request_timeout_s'] + p['retention_worker_timeout_s'])
        self.assertGreater(p['retention_shutdown_timeout_s'], p['retention_worker_timeout_s'] + 1.)


if __name__ == '__main__':
    unittest.main()
