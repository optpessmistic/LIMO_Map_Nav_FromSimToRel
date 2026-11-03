"""No-return task regressions using the production adapter and real Planner.

ROS transport, odometry and TF are synthetic. These are scheduler/evidence
regressions, not a simulation run or a claim about physical stopping.
"""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

import test_observation_adapter as t
import test_observation_continuation as c
import test_frame_retention_node as r


class MappingCompletionTests(unittest.TestCase):
    def setUp(self):
        self.f = t.AdapterTests()
        self.f.setUp()

    def mission(self, **overrides):
        return self.f.mission(return_home=False, **overrides)

    def finish_stopped(self, m):
        for _ in range(70):
            t.TIME.advance(.1)
            self.f.odom(m)
            self.f.tick(m)
            if m.phase not in ('canceling', 'completion_verification'):
                return
        self.fail('fresh stationary odometry should resolve completion verification')

    def completed_views(self):
        m = self.mission()
        self.f.start(m)
        for _ in range(2):
            self.f.terminal(m)
            self.f.settle(m)
            self.f.processed_window(m)
        self.assertEqual(m.cancel_next, 'completion_verification')
        self.f.tick(m)
        self.assertEqual(m.phase, 'completion_verification')
        return m

    def test_return_is_opt_in_and_boolean(self):
        self.assertFalse(t.adapter.MissionNode().cfg['return_home'])
        for invalid in (0, 1, 'false', None):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                t.adapter.MissionNode(parameter_overrides={'return_home': invalid})

    def test_completed_bounded_views_stop_at_current_pose_without_home_goal(self):
        m = self.completed_views()
        last_pose = m.synthetic_pose
        records = copy.deepcopy(m.observation_attempt_records)
        self.finish_stopped(m)
        self.assertEqual(m.phase, 'done')
        self.assertTrue(m.map_complete)
        self.assertFalse(m.visual_complete)
        self.assertEqual(m.return_attempts, 0)
        self.assertIsNone(m.return_started)
        self.assertIsNone(m.return_deadline_wall)
        self.assertEqual(len(m.nav.sent), 2)
        self.assertEqual(m.saved_results[0][0:2], (False, last_pose))
        self.assertEqual(m.observation_attempt_records, records)

    def test_terminal_action_alone_or_moving_odometry_does_not_certify_completion(self):
        m = self.completed_views()
        for _ in range(5):
            self.f.tick(m, .1)
        self.assertEqual(m.phase, 'completion_verification')
        self.assertEqual(m.saved_results, [])
        for _ in range(8):
            t.TIME.advance(.1)
            self.f.odom(m, moving=True)
            self.f.tick(m)
        self.assertEqual(m.phase, 'completion_verification')
        self.assertEqual(m.saved_results, [])
        self.finish_stopped(m)
        self.assertEqual(m.phase, 'done')

    def test_paused_ros_clock_cannot_reuse_one_stationary_sample_as_stop_proof(self):
        m = self.completed_views()
        t.TIME.advance(.1)
        self.f.odom(m)
        for _ in range(8):
            t.TIME.advance(.1, ros=False)
            self.f.odom(m)
            self.f.tick(m)
        self.assertEqual(m.phase, 'completion_verification')
        self.assertEqual(m.saved_results, [])
        self.finish_stopped(m)
        self.assertEqual(m.phase, 'done')

    def test_stationary_wheels_do_not_replace_stable_map_pose(self):
        m = self.completed_views()
        pose = m.synthetic_pose
        for index in range(10):
            m.synthetic_pose = (pose[0]+(.08 if index % 2 else 0.), pose[1], pose[2])
            t.TIME.advance(.1)
            self.f.odom(m)
            self.f.tick(m)
        self.assertEqual(m.phase, 'completion_verification')
        self.assertEqual(m.saved_results, [])
        self.finish_stopped(m)
        self.assertEqual(m.phase, 'done')

    def test_new_reachable_frontier_at_final_check_resumes_without_creating_return_budget(self):
        m = self.completed_views()
        original_queue = m.observation_queue
        original_deadline = m.observation_deadline_wall
        records = copy.deepcopy(m.observation_attempt_records)
        c.ContinuationTests.frontier_map(self, m)
        self.finish_stopped(m)
        self.assertEqual(m.phase, 'exploring')
        self.assertFalse(m.map_complete)
        self.assertEqual(m.saved_results, [])
        self.assertIsNone(m.return_deadline_wall)
        self.assertIs(m.observation_queue, original_queue)
        self.assertEqual(m.observation_deadline_wall, original_deadline)
        self.assertEqual(m.observation_attempt_records, records)
        self.f.tick(m)
        self.assertEqual(m.goal.kind, 'frontier')
        self.assertEqual(m.return_attempts, 0)

    def test_unreachable_unknown_branch_remains_incomplete_after_stop(self):
        m = self.completed_views()
        values = list(m.grid.data)
        for x in range(20, 29):
            for y in range(1, 29):
                values[y*30+x] = 0 if y == 15 else 100
        values[15*30+28] = -1
        m.grid = t.Grid(30, 30, .1, 0., 0., 0., tuple(values))
        self.finish_stopped(m)
        self.assertEqual(m.phase, 'done')
        self.assertFalse(m.map_complete)
        self.assertFalse(m.visual_complete)
        self.assertEqual(len(m.nav.sent), 2)
        self.assertFalse(m.saved_results[0][0])
        self.assertEqual(m.grid.value((28, 15)), -1)

    def test_completion_waits_for_terminal_after_late_accept_and_cancel_ack(self):
        m = self.mission()
        self.f.start(m)
        m.start_return('mission_timeout_incomplete')
        self.assertTrue(m.send_pending)
        self.assertEqual(m.cancel_next, 'completion_verification')
        handle = t.Handle(m.token)
        m.nav.sent[-1][1].resolve(handle)
        self.assertEqual(handle.cancel_calls, 1)
        handle.cancel_future.resolve(S(goals_canceling=[S(goal_id=handle.goal_id)]))
        self.f.tick(m)
        self.assertTrue(m.canceling)
        self.assertEqual(m.saved_results, [])
        self.assertEqual(len(m.nav.sent), 1)
        handle.result_future.resolve(S(status=5, result=S(error_msg='owned canceled')))
        self.f.tick(m)
        self.assertEqual(m.phase, 'completion_verification')
        self.assertEqual(m.saved_results, [])
        self.assertEqual(len(m.nav.sent), 1)
        self.finish_stopped(m)
        self.assertEqual(m.phase, 'done')

    def test_no_processed_inspection_still_requires_fresh_stop_evidence(self):
        with patch.object(t.FakeNode, 'create_subscription', autospec=True, return_value=S()) as subscribe:
            m = self.mission(post_mapping_observation_mode='off', require_processed_inspection=False)
        self.assertTrue(any(call.args[2] == '/odom' for call in subscribe.call_args_list))
        self.assertIsNone(m.inspection)
        m.start_return('mapping_and_annotation_finished')
        self.f.tick(m)
        for _ in range(8):
            self.f.tick(m, .1)
        self.assertEqual(m.phase, 'completion_verification')
        self.assertEqual(m.saved_results, [])
        self.finish_stopped(m)
        self.assertEqual(m.phase, 'done')
        self.assertTrue(m.map_complete)
        self.assertFalse(m.saved_results[0][0])

    def test_tf_fault_during_final_verification_cannot_save_success(self):
        m = self.completed_views()
        m.get_pose = lambda: (_ for _ in ()).throw(LookupError('synthetic unavailable TF'))
        self.f.tick(m)
        self.assertEqual(m.cancel_next, 'blocked')
        self.f.tick(m)
        self.assertEqual(m.phase, 'blocked')
        self.assertFalse(m.map_complete)
        self.assertEqual(m.return_attempts, 0)

    def test_normal_no_return_completion_preserves_strict_retention_receipts(self):
        retention_fixture = r.RetentionNodeTests()
        retention_fixture.setUp()
        self.addCleanup(retention_fixture.doCleanups)
        m = retention_fixture.mission(return_home=False)
        self.f.start(m)
        for _ in range(2):
            self.f.terminal(m)
            self.f.settle(m)
            retention_fixture.seal(m)
            retention_fixture.receipts(m)
            self.f.tick(m)
        self.assertEqual(m.cancel_next, 'completion_verification')
        self.assertIsNone(m.frame_retention.fault)
        retained_before = copy.deepcopy(m.frame_retention.snapshot()['windows'])
        self.f.tick(m)
        self.finish_stopped(m)
        self.assertEqual(m.phase, 'done')
        self.assertIsNone(m.frame_retention.fault)
        self.assertEqual(m.frame_retention.snapshot()['windows'], retained_before)
        self.assertEqual(len(m.nav.sent), 2)

    def test_saved_mapping_task_succeeds_without_return_or_claiming_all_targets_found(self):
        m = self.completed_views()
        m.detector_received = t.TIME.wall
        m.sources.add('red_rgbd_sim_baseline')
        self.assertEqual(m.coverage.frames, 0)  # A real surrogate need not publish coverage polygons.
        for stamp in (1., 1.2, 1.4):
            m.targets.add('red_object', (1., 1.2, .15), stamp, .99)
        with tempfile.TemporaryDirectory(dir=t.ROOT) as directory:
            m.cfg['output_dir'] = directory
            m.save_result = t.adapter.MissionNode.save_result.__get__(m)
            with patch.object(t.adapter.time, 'strftime', lambda fmt: 'fixture', create=True):
                self.finish_stopped(m)
            path, = Path(directory).glob('mission_fixture*.json')
            result = json.loads(path.read_text())
            bundle, = Path(directory).glob('mission_fixture*_map')
            saved_targets = json.loads((bundle/'targets.json').read_text())
            saved_map = json.loads((bundle/'map.json').read_text())
            self.assertEqual(saved_map['data'], list(m.grid.data))
            self.assertEqual(saved_map['map_epoch'], m.map_epoch)
            self.assertEqual(saved_targets['targets'], result['targets'])
            self.assertIsNone(saved_targets['target_count_prior'])
            self.assertIsNone(saved_targets['all_targets_found'])
            self.assertTrue((bundle/'annotated.svg').is_file())
        self.assertTrue(result['map_complete'])
        self.assertTrue(result['mapping_annotation_complete'])
        self.assertTrue(result['task_success'])
        self.assertFalse(result['returned_pose_verified'])
        self.assertFalse(result['visual_coverage_complete'])
        self.assertFalse(result['mission_success'])
        self.assertFalse(result['target_definition_verified'])
        self.assertEqual(len(result['targets']), 1)

    def test_map_geometry_completion_alone_does_not_certify_annotation_pipeline(self):
        m = self.completed_views()
        self.assertIsNone(m.detector_received)
        self.assertFalse(m.sources)
        with tempfile.TemporaryDirectory(dir=t.ROOT) as directory:
            m.cfg['output_dir'] = directory
            m.save_result = t.adapter.MissionNode.save_result.__get__(m)
            with patch.object(t.adapter.time, 'strftime', lambda fmt: 'fixture', create=True):
                self.finish_stopped(m)
            path, = Path(directory).glob('mission_fixture*.json')
            result = json.loads(path.read_text())
        self.assertTrue(result['map_complete'])
        self.assertFalse(result['mapping_annotation_complete'])
        self.assertFalse(result['task_success'])
        self.assertFalse(result['mission_success'])

    def test_artifact_write_failure_preserves_json_evidence_but_not_completion_claim(self):
        m = self.completed_views()
        m.detector_received = t.TIME.wall
        m.sources.add('red_rgbd_sim_baseline')
        with tempfile.TemporaryDirectory(dir=t.ROOT) as directory:
            m.cfg['output_dir'] = directory
            m.save_result = t.adapter.MissionNode.save_result.__get__(m)
            with patch.object(t.adapter.time, 'strftime', lambda fmt: 'fixture', create=True), \
                    patch.object(t.adapter, 'export_bundle', side_effect=OSError('synthetic disk failure')):
                self.finish_stopped(m)
            path, = Path(directory).glob('mission_fixture*.json')
            result = json.loads(path.read_text())
        self.assertTrue(result['map_complete'])
        self.assertFalse(result['mapping_annotation_complete'])
        self.assertFalse(result['task_success'])
        self.assertIn('synthetic disk failure', json.dumps(result))

    def test_failed_configured_observation_queue_still_prevents_task_success(self):
        m = self.mission()
        self.f.start(m)
        for _ in range(2):
            self.f.terminal(m, status=6)
        self.f.tick(m)
        self.assertEqual(m.phase, 'completion_verification')
        m.detector_received = t.TIME.wall
        m.sources.add('red_rgbd_sim_baseline')
        with tempfile.TemporaryDirectory(dir=t.ROOT) as directory:
            m.cfg['output_dir'] = directory
            m.save_result = t.adapter.MissionNode.save_result.__get__(m)
            with patch.object(t.adapter.time, 'strftime', lambda fmt: 'fixture', create=True):
                self.finish_stopped(m)
            path, = Path(directory).glob('mission_fixture*.json')
            result = json.loads(path.read_text())
        self.assertTrue(result['map_complete'])
        self.assertTrue(result['mapping_annotation_complete'])
        self.assertFalse(result['task_success'])
        self.assertFalse(result['observation_actions']['queue']['all_planned_actions_completed'])
        self.assertEqual(len(result['observation_actions']['attempts']), 2)

    def test_startup_without_map_preserves_json_failure_evidence(self):
        m = self.mission(post_mapping_observation_mode='off', require_processed_inspection=False)
        m.home = m.grid = m.map_received = None
        m.phase = 'waiting'
        with tempfile.TemporaryDirectory(dir=t.ROOT) as directory:
            m.cfg['output_dir'] = directory
            m.save_result = t.adapter.MissionNode.save_result.__get__(m)
            with patch.object(t.adapter.time, 'strftime', lambda fmt: 'fixture', create=True):
                t.TIME.advance(m.cfg['startup_wait_s']+.1)
                m.tick()  # Real startup guard, not a manually fabricated result.
                self.assertEqual(m.cancel_next, 'blocked')
                self.assertEqual(m.reason, 'startup_no_map')
                m.tick()  # Own cancellation reaches terminal and writes evidence.
            path, = Path(directory).glob('mission_fixture*.json')
            result = json.loads(path.read_text())
        self.assertEqual(result['status'], 'blocked')
        self.assertEqual(result['reason'], 'startup_no_map')
        self.assertFalse(result['map_complete'])
        self.assertFalse(result['mapping_annotation_complete'])
        self.assertFalse(result['task_success'])
        self.assertFalse(result['mission_success'])
        self.assertFalse(result['completion_stop_verified'])
        self.assertEqual(result['artifacts']['status'], 'failed')
        self.assertIsNone(result['home'])
        self.assertTrue(m.result_written)


if __name__ == '__main__':
    unittest.main(verbosity=2)
