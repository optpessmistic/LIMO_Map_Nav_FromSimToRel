"""Map timing diagnostics preserve the production adapter's existing guards.

The real callbacks, scheduler and final Planner run with the established fake
ROS/clock fixture. No ROS nodes, simulator or external processes are started.
"""
import copy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

import test_mapping_completion as completion
import test_observation_adapter as t


def map_message(mission, stamp_ns=None):
    grid = mission.grid
    stamp = t.TIME.ns if stamp_ns is None else stamp_ns
    return S(header=S(frame_id='map', stamp=S(sec=stamp // 10**9, nanosec=stamp % 10**9)),
             info=S(width=grid.width, height=grid.height, resolution=grid.resolution,
                    origin=S(position=S(x=grid.origin_x, y=grid.origin_y),
                             orientation=S(x=0., y=0., z=0., w=1.))), data=grid.data)


class MapFreshnessDiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.fixture = completion.MappingCompletionTests()
        self.fixture.setUp()
        self.f = self.fixture.f

    def mission(self):
        return self.fixture.mission()

    def failures(self, mission):
        return [event for event in mission.events if event['event'] == 'map_freshness_failed']

    def test_valid_map_receipts_keep_only_eight_gaps_and_measure_callback_work(self):
        mission = self.mission()
        message = map_message(mission)
        original_grid = t.adapter.Grid

        def delayed_grid(*args):
            t.TIME.advance(.025, ros=False)
            return original_grid(*args)

        with patch.object(t.adapter, 'Grid', side_effect=delayed_grid):
            mission.on_map(message)
        self.assertEqual(mission.map_valid_receive_count, 1)
        self.assertAlmostEqual(mission.map_last_callback_duration_s, .025)
        self.assertEqual(mission.map_recent_receive_gaps_s, [])
        for gap in range(1, 12):
            t.TIME.advance(gap)
            mission.on_map(message)
        self.assertEqual(mission.map_valid_receive_count, 12)
        self.assertEqual(len(mission.map_recent_receive_gaps_s), 8)
        for actual, expected in zip(mission.map_recent_receive_gaps_s, range(4, 12)):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(mission.map_max_receive_gap_s, 11.)
        self.assertEqual(mission.map_last_callback_duration_s, 0.)
        self.assertEqual(self.failures(mission), [])

    def test_rejected_maps_do_not_refresh_or_count_as_valid_receipts(self):
        for invalid in ('frame', 'payload'):
            with self.subTest(invalid=invalid):
                mission = self.mission()
                message = map_message(mission)
                mission.on_map(message)
                previous = (mission.map_received, mission.map_valid_receive_count,
                            mission.map_last_callback_duration_s, mission.map_stamp_ns)
                bad = copy.deepcopy(message)
                if invalid == 'frame':
                    bad.header.frame_id = 'odom'
                else:
                    bad.data = []
                t.TIME.advance(.5)
                mission.on_map(bad)
                self.assertEqual((mission.map_received, mission.map_valid_receive_count,
                                  mission.map_last_callback_duration_s, mission.map_stamp_ns), previous)
                self.assertEqual(mission.cancel_next, 'blocked')
                self.assertEqual(self.failures(mission), [])

    def test_fresh_map_at_original_boundary_does_not_use_old_header_as_heartbeat(self):
        mission = self.mission()
        mission.phase = 'dry_run'
        mission.on_map(map_message(mission, stamp_ns=1_000_000_000))
        t.TIME.advance(mission.cfg['map_max_age_s'])
        mission.tick()
        self.assertEqual(mission.phase, 'dry_run')
        self.assertFalse(mission.canceling)
        self.assertEqual(self.failures(mission), [])
        self.assertIsNone(mission.last_map_freshness_failure)
        self.assertGreater((t.TIME.ns - mission.map_stamp_ns) / 1e9, mission.cfg['map_max_age_s'])

    def test_expired_wall_age_still_blocks_with_paused_ros_clock_and_saves_facts(self):
        mission = self.mission()
        mission.on_map(map_message(mission))
        started, threshold = mission.started, mission.cfg['map_max_age_s']
        t.TIME.advance(threshold + .1, ros=False)
        with tempfile.TemporaryDirectory(prefix='map-freshness-', dir=t.ROOT) as directory:
            self.assertTrue(Path(directory).resolve().is_relative_to(t.ROOT.resolve()))
            mission.cfg['output_dir'] = directory
            mission.save_result = t.adapter.MissionNode.save_result.__get__(mission)
            with patch.object(t.adapter.time, 'strftime', lambda fmt: 'fixture', create=True):
                mission.tick()
                self.assertEqual(mission.reason, 'map_stale')
                self.assertEqual(mission.cancel_next, 'blocked')
                evidence = mission.last_map_freshness_failure
                self.assertEqual(evidence['check'], 'tick')
                self.assertAlmostEqual(evidence['valid_receive_wall_age_s'], threshold + .1)
                self.assertEqual(evidence['header_stamp_age_s'], 0.)
                self.assertEqual(evidence['max_age_s'], threshold)
                self.assertEqual(evidence['valid_receive_count'], 1)
                self.assertIn('not_publication_heartbeat', evidence['header_stamp_semantics'])
                events = [event['event'] for event in mission.events]
                self.assertLess(events.index('map_freshness_failed'), events.index('cancel_requested'))
                mission.tick()
            path, = Path(directory).glob('mission_fixture*.json')
            saved = json.loads(path.read_text())
        self.assertEqual(mission.phase, 'blocked')
        self.assertEqual(mission.started, started)
        self.assertEqual(mission.cfg['map_max_age_s'], threshold)
        self.assertEqual(len(self.failures(mission)), 1)
        self.assertEqual(saved['last_map_freshness_failure'], evidence)
        self.assertFalse(saved['task_success'])
        self.assertEqual(mission.nav.sent, [])

    def test_final_planning_expiring_the_map_still_blocks_before_completion(self):
        mission = self.fixture.completed_views()
        mission.on_map(map_message(mission))
        started = mission.completion_started_wall
        original_select = mission.planner.select

        def delayed_select(*args, **kwargs):
            selection = original_select(*args, **kwargs)
            t.TIME.advance(mission.cfg['map_max_age_s'] + .1)
            return selection

        with patch.object(mission.planner, 'select', side_effect=delayed_select) as select:
            for _ in range(50):
                t.TIME.advance(.1)
                self.f.odom(mission)
                self.f.tick(mission)
                if select.called:
                    break
            self.assertEqual(select.call_count, 1)
        self.assertEqual(mission.reason, 'map_stale')
        self.assertEqual(mission.cancel_next, 'blocked')
        self.assertEqual(mission.last_map_freshness_failure['check'], 'verify_completion_after_planning')
        self.assertAlmostEqual(mission.last_map_freshness_failure['valid_receive_wall_age_s'], 5.1)
        self.assertEqual(mission.completion_started_wall, started)
        self.assertFalse(mission.completion_stop_verified)
        self.assertEqual(mission.saved_results, [])
        self.assertEqual(len(mission.nav.sent), 2)

    def test_diagnostic_log_failure_cannot_prevent_map_stale_blocking(self):
        mission = self.mission()
        mission.on_map(map_message(mission))
        t.TIME.advance(mission.cfg['map_max_age_s'] + .1)
        original_event = mission.event

        def event(name, **fields):
            if name == 'map_freshness_failed':
                raise RuntimeError('fixture logging failure')
            return original_event(name, **fields)

        mission.event = event
        mission.tick()
        self.assertEqual(mission.reason, 'map_stale')
        self.assertEqual(mission.cancel_next, 'blocked')
        self.assertIsNotNone(mission.last_map_freshness_failure)


if __name__ == '__main__':
    unittest.main()
