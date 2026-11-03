"""Final planning must not turn old odometry into fresh stopping evidence.

The production adapter and Planner run with synthetic ROS/TF/clock inputs.
These regressions do not start ROS or verify physical braking.
"""
import unittest
from unittest.mock import patch

import test_mapping_completion as completion
import test_observation_adapter as t


class CompletionFreshnessTests(unittest.TestCase):
    def setUp(self):
        self.fixture = completion.MappingCompletionTests()
        self.fixture.setUp()

    def delayed_final_plan(self):
        mission = self.fixture.completed_views()
        started = mission.completion_started_wall
        actual_select = mission.planner.select

        def delayed_select(*args, **kwargs):
            selected = actual_select(*args, **kwargs)
            # No callbacks are serviced while synchronous planning consumes
            # two seconds: the previously stationary wheel sample gets stale.
            t.TIME.advance(2.0)
            return selected

        with patch.object(mission.planner, 'select', side_effect=delayed_select) as select:
            for _ in range(50):
                t.TIME.advance(.1)
                self.fixture.f.odom(mission)
                self.fixture.f.tick(mission)
                if select.called:
                    break
            self.assertEqual(select.call_count, 1)

        self.assertEqual(mission.phase, 'completion_verification')
        self.assertFalse(mission.completion_stop_verified)
        self.assertFalse(mission.inspection_stopped(t.TIME.wall))
        self.assertAlmostEqual(t.TIME.wall-mission.inspection_odom['received'], 2.0)
        self.assertEqual(mission.saved_results, [])
        self.assertEqual(mission.completion_started_wall, started)
        self.assertIsNone(mission.completion_stable_since)
        self.assertIsNone(mission.last_completion_pose)
        self.assertEqual(len(mission.nav.sent), 2)
        self.assertLess(t.TIME.wall-started, mission.cfg['cancel_timeout_s'])
        return mission, started

    def test_planning_expired_odometry_waits_for_new_continuous_stationary_evidence(self):
        mission, started = self.delayed_final_plan()
        self.fixture.finish_stopped(mission)
        self.assertEqual(mission.phase, 'done')
        self.assertTrue(mission.completion_stop_verified)
        self.assertTrue(mission.inspection_stopped(t.TIME.wall))
        self.assertEqual(len(mission.saved_results), 1)
        self.assertEqual(mission.completion_started_wall, started)
        self.assertEqual(len(mission.nav.sent), 2)

    def test_rejected_old_stop_evidence_does_not_extend_completion_wall_deadline(self):
        mission, started = self.delayed_final_plan()
        deadline = started+mission.cfg['cancel_timeout_s']
        self.fixture.f.tick(mission, deadline-t.TIME.wall+.01)
        self.assertEqual(mission.cancel_next, 'blocked')
        self.assertEqual(mission.reason, 'completion_stop_verification_timeout')
        self.assertFalse(mission.completion_stop_verified)
        self.assertEqual(mission.completion_started_wall, started)
        self.fixture.f.tick(mission)
        self.assertEqual(mission.phase, 'blocked')
        self.assertEqual(len(mission.saved_results), 1)
        self.assertFalse(mission.saved_results[0][0])
        self.assertEqual(len(mission.nav.sent), 2)


if __name__ == '__main__':
    unittest.main(verbosity=2)
