"""Independent pure merged-production review. No ROS imports or nodes."""
import hashlib
import importlib
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace as S
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tests'))
import test_observation_adapter as t


class MergedIntersectionTests(unittest.TestCase):
    def setUp(self):
        self.f = t.AdapterTests()
        self.f.setUp()
        t.adapter.Time = lambda **kw: S(**kw)

    def add_two_targets(self, m, frames=3):
        m.tf_buffer = S(lookup_transform=lambda *args: S(transform=S(
            translation=S(x=0., y=0., z=0.), rotation=S(x=0., y=0., z=0., w=1.))))
        m.publish_targets = lambda: None
        for _ in range(frames):
            t.TIME.advance(.1)
            message = {'schema_version': 1, 'stamp': {'sec': t.TIME.ns//10**9, 'nanosec': t.TIME.ns%10**9},
                       'map_epoch': m.map_epoch, 'frame_id': 'camera_optical', 'source': 'red',
                       'detections': [{'class_id': 'red_object', 'position': [1., y, .1], 'confidence': .95}
                                      for y in (-.09, .09)]}
            m.on_observations(S(data=json.dumps(message)))
        self.assertEqual(len(m.targets.confirmed()), 2)

    def test_two_targets_pending_observation_epoch_change_ack_cannot_retire_owned_nav(self):
        m = self.f.mission()
        self.add_two_targets(m)
        self.f.start(m)
        self.assertTrue(m.send_pending)
        original_targets = [t.asdict(item) for item in m.targets.targets]
        m.on_epoch(S(data='map-b'))
        self.assertFalse(m.coordinates_valid)
        self.assertFalse(m.map_epoch_ready)
        self.assertEqual(m.map_epoch, 'map-a')
        self.assertEqual(m.targets.frame_epoch, 'map-a')
        self.assertTrue(m.send_pending)
        self.assertEqual(m.cancel_next, 'blocked')
        self.assertIsNotNone(m.observation_active)
        self.assertEqual(m.observation_attempt_records, [])
        handle = t.Handle(m.token)
        m.nav.sent[-1][1].resolve(handle)
        handle.cancel_future.resolve(S(goals_canceling=[S(goal_id=handle.goal_id)]))
        self.f.tick(m)
        self.assertTrue(m.canceling)
        self.assertFalse(m.cancel_terminal)
        self.assertIs(m.goal_handle, handle)
        self.assertIsNotNone(m.observation_active)
        self.assertEqual(m.observation_attempt_records, [])
        handle.result_future.resolve(S(status=5, result=S(error_msg='owned canceled')))
        self.f.tick(m)
        self.assertEqual(m.phase, 'blocked')
        self.assertIsNone(m.observation_active)
        self.assertEqual(m.observation_attempt_records[0]['outcome'], 'canceled')
        self.assertEqual([t.asdict(item) for item in m.targets.targets], original_targets)
        self.assertEqual(len(m.marker_pub.messages), 1)

    def test_targets_do_not_satisfy_processed_window_and_epoch_preserves_partial_evidence(self):
        m = self.f.mission()
        self.add_two_targets(m)
        self.f.start(m)
        self.f.terminal(m)
        self.f.settle(m)
        self.assertEqual(m.inspection.accepted_frames, 0)
        self.assertNotEqual(m.inspection.status, 'satisfied')
        t.TIME.advance(.12)
        self.f.odom(m)
        m.on_frame_processing(S(data=json.dumps(t.frame(t.TIME.ns-20_000_000, seq=1))))
        self.assertEqual(m.inspection.accepted_frames, 1)
        m.on_epoch(S(data='map-b'))
        attempt = m.observation_attempt_records[0]
        self.assertEqual(attempt['inspection']['accepted_frames'], 1)
        self.assertEqual(attempt['inspection']['map_epoch'], 'map-a')
        self.assertEqual(attempt['outcome'], 'inspection_failed')
        self.assertFalse(m.coordinates_valid)
        self.assertFalse(m.inspection.ready)
        self.assertFalse(m.visual_complete)
        self.assertEqual(len(m.targets.confirmed()), 2)  # Kept for audit, not republished.

    def test_default_off_has_same_target_epoch_guard_without_observation_queue(self):
        m = self.f.mission(post_mapping_observation_mode='off')
        m.home, m.phase = None, 'waiting'
        self.add_two_targets(m)
        self.assertIsNone(m.observation_snapshot())
        self.assertIsNone(m.observation_queue)
        self.assertIsNone(m.return_deadline_wall)
        m.on_epoch(S(data='map-b'))
        self.assertEqual(m.reason, 'target_map_epoch_changed_requires_new_mission')
        self.assertFalse(m.map_epoch_ready)
        self.assertFalse(m.coordinates_valid)
        self.assertIsNone(m.observation_queue)
        self.assertEqual(m.nav.sent, [])


def main():
    relative = ['src/limo_maze_mission/limo_maze_mission/'+name for name in
                ('core.py', 'node.py', 'inspection.py', 'observation.py')]
    relative += ['src/limo_maze_mission/launch/mission.launch.py']
    before = {path: hashlib.sha256((ROOT/path).read_bytes()).hexdigest() for path in relative}
    modules = ['test_observation_adapter', 'test_observation_continuation', 'test_observation_final_repairs',
               'test_observation_queue', 'test_observation_repairs', 'test_mission_inspection',
               'test_target_frame_association', 'test_target_epoch_cleanup']
    loader, suite = unittest.defaultTestLoader, unittest.TestSuite()
    for name in modules:
        suite.addTests(loader.loadTestsFromModule(importlib.import_module(name)))
    existing = importlib.import_module('test_ros_mission_core')
    omitted = []
    for name in dir(existing):
        item = getattr(existing, name)
        if isinstance(item, type) and issubclass(item, unittest.TestCase):
            if name == 'RosStartupTests':
                omitted.append(name)
            else:
                suite.addTests(loader.loadTestsFromTestCase(item))
    suite.addTests(loader.loadTestsFromTestCase(MergedIntersectionTests))
    stream = io.StringIO()
    result = unittest.TextTestRunner(stream=stream, verbosity=2).run(suite)
    after = {path: hashlib.sha256((ROOT/path).read_bytes()).hexdigest() for path in relative}
    stem = ROOT/'outputs/setup/observation-association-merged-pure-tests'
    with stem.with_suffix('.log').open('x', encoding='utf-8') as output:
        output.write(stream.getvalue())
    value = {'tests_run': result.testsRun, 'failures': len(result.failures), 'errors': len(result.errors),
             'skipped': len(result.skipped), 'passed': result.wasSuccessful(), 'production_sha256': before,
             'source_unchanged_during_tests': before == after, 'ros_nodes_started': False,
             'explicitly_excluded': omitted+['test_observation_ros', 'test_mission_inspection_ros'],
             'independent_intersection_tests': 3,
             'scope': 'merged production pure cores and AST-executed adapter with synthetic IO only'}
    with stem.with_suffix('.json').open('x', encoding='utf-8') as output:
        json.dump(value, output, indent=2)
        output.write('\n')
    print(json.dumps(value))
    if not result.wasSuccessful():
        print(stream.getvalue())
        raise SystemExit(1)


if __name__ == '__main__':
    main()
