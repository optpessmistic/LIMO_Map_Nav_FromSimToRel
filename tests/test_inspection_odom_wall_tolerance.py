"""Separate delivery jitter from ROS sample age without relaxing stop evidence."""
import unittest

import test_observation_adapter as t


class OdomWallToleranceTests(unittest.TestCase):
    def setUp(self):
        self.f = t.AdapterTests()
        self.f.setUp()

    def inspecting(self, **parameters):
        m = self.f.mission(**parameters)
        self.f.start(m)
        self.f.terminal(m)
        self.f.settle(m)
        self.assertTrue(m.inspection_stopped(t.TIME.wall))
        return m

    def test_default_inherits_existing_ros_age_parameter_and_rejects_bad_overrides(self):
        m = self.f.mission(inspection_odom_max_age_s=.6)
        self.assertEqual(m.inspection_odom_wall_limit(), .6)
        for value in (-1., float('nan'), float('inf'), True, '1.0', 10.1):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.f.mission(inspection_odom_wall_max_age_s=value)

    def test_measured_326_ms_delivery_gap_keeps_stationary_window_with_override(self):
        m = self.inspecting(inspection_odom_wall_max_age_s=1.)
        deadline, resets = m.inspection_deadline, m.inspection.reset_count
        t.TIME.advance(.326, ros=False)
        t.TIME.ns += 10_000_000
        self.assertTrue(m.inspection_stopped(t.TIME.wall))
        self.f.odom(m)
        self.assertEqual(m.phase, 'inspecting')
        self.assertEqual(m.inspection.reset_count, resets)
        self.assertEqual(m.inspection_deadline, deadline)
        facts = m.inspection_wait_conditions(t.TIME.wall, m.synthetic_pose)
        self.assertTrue(facts['odom_fresh'])
        self.assertEqual(facts['odom_ros_max_age_s'], .3)
        self.assertEqual(facts['odom_wall_max_age_s'], 1.)

    def test_default_still_restarts_after_the_same_delivery_gap(self):
        m = self.inspecting()
        t.TIME.advance(.326, ros=False)
        t.TIME.ns += 10_000_000
        self.assertFalse(m.inspection_stopped(t.TIME.wall))
        self.f.odom(m)
        self.assertEqual(m.phase, 'settling')
        self.assertEqual(m.inspection_odom['count'], 0)

    def test_old_ros_sample_is_rejected_even_with_wall_override(self):
        m = self.inspecting(inspection_odom_wall_max_age_s=1.)
        t.TIME.advance(.31)
        self.assertFalse(m.inspection_stopped(t.TIME.wall))
        self.assertFalse(m.inspection_wait_conditions(t.TIME.wall, m.synthetic_pose)['odom_fresh'])

    def test_wall_silence_over_limit_is_rejected_even_when_ros_clock_is_paused(self):
        m = self.inspecting(inspection_odom_wall_max_age_s=1.)
        t.TIME.advance(1.01, ros=False)
        self.assertFalse(m.inspection_stopped(t.TIME.wall))
        self.assertFalse(m.inspection_wait_conditions(t.TIME.wall, m.synthetic_pose)['odom_fresh'])

    def test_duplicate_stamps_and_motion_still_reset_the_window(self):
        for moving, ros in ((False, False), (True, True)):
            with self.subTest(moving=moving):
                self.f.setUp()
                m = self.inspecting(inspection_odom_wall_max_age_s=1.)
                deadline = m.inspection_deadline
                t.TIME.advance(.1, ros=ros)
                self.f.odom(m, moving=moving)
                self.assertEqual(m.phase, 'settling')
                self.assertFalse(m.inspection_stopped(t.TIME.wall))
                self.assertEqual(m.inspection_deadline, deadline)


if __name__ == '__main__':
    unittest.main()
