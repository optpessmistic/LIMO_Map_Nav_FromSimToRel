"""Odometry integrity tests use synthetic received streams; no ROS required."""
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from odom_integrity import MIN_OBSERVATION_NS, MIN_UNIQUE_ODOM, OdomIntegrity, stamp_to_nanoseconds


POSE = (0., 0., 0., 0., 0., 0., 1.)
TWIST = (0.,) * 6


def changed(vector, index, value):
    result = list(vector)
    result[index] = value
    return result


def enough(checker=None, start=0):
    checker = checker or OdomIntegrity()
    for i in range(51):
        checker.observe(start+i*20_000_000, POSE, TWIST)
    return checker


class OdomIntegrityTests(unittest.TestCase):
    def test_clean_one_second_stream_passes_only_after_full_window(self):
        checker = OdomIntegrity()
        for i in range(50):
            self.assertTrue(checker.observe(i*20_000_000, POSE, TWIST))
        self.assertFalse(checker.observation_sufficient)  # 50 stamps, only .98 s.
        self.assertFalse(checker.report()['passed'])
        checker.observe(MIN_OBSERVATION_NS, POSE, TWIST)
        self.assertTrue(checker.report()['passed'])

    def test_many_duplicates_and_three_widely_spaced_samples_are_insufficient(self):
        checker = OdomIntegrity()
        for stamp in (0, 1_000_000_000, 2_000_000_000):
            for _ in range(100):
                checker.observe(stamp, POSE, TWIST)
        report = checker.report()
        self.assertEqual(report['valid_unique_stamps'], 3)
        self.assertEqual(report['counts']['duplicate_messages'], 297)
        self.assertEqual(report['repeated_stamps'], 3)
        self.assertFalse(report['passed'])

    def test_exact_count_and_time_boundaries(self):
        checker = OdomIntegrity()
        for i in range(MIN_UNIQUE_ODOM-1):
            checker.observe(i*30_000_000, POSE, TWIST)
        self.assertGreater(checker.span_ns, MIN_OBSERVATION_NS)
        self.assertFalse(checker.report()['passed'])
        checker.observe(49*30_000_000, POSE, TWIST)
        self.assertTrue(checker.report()['passed'])

    def test_identical_same_stamp_repeats_are_counted_without_advancing(self):
        checker = enough()
        original = checker.report()
        for _ in range(7):
            self.assertFalse(checker.observe(MIN_OBSERVATION_NS, POSE, TWIST))
        report = checker.report()
        self.assertTrue(report['passed'])
        self.assertEqual(report['counts']['identical_duplicate_messages'], 7)
        self.assertEqual(report['valid_unique_stamps'], original['valid_unique_stamps'])
        self.assertEqual(report['valid_span_ns'], original['valid_span_ns'])

    def test_run18_sized_same_stamp_translation_is_not_tolerated(self):
        checker = enough()
        checker.observe(MIN_OBSERVATION_NS, changed(POSE, 0, 1.34e-7), TWIST)
        # A later repeat of the original value must not erase the earlier conflict.
        checker.observe(MIN_OBSERVATION_NS, POSE, TWIST)
        report = checker.report()
        self.assertFalse(report['passed'])
        self.assertEqual(report['counts']['pose_conflict_messages'], 1)
        self.assertEqual(report['counts']['identical_duplicate_messages'], 1)
        self.assertAlmostEqual(report['maximum_same_stamp_deltas']['position_delta_m'], 1.34e-7)

    def test_every_pose_and_twist_component_is_checked(self):
        for vector, size in (('pose', 7), ('twist', 6)):
            for index in range(size):
                with self.subTest(vector=vector, index=index):
                    checker = enough()
                    pose = changed(POSE, index, POSE[index]+1e-12) if vector == 'pose' else POSE
                    twist = changed(TWIST, index, 1e-12) if vector == 'twist' else TWIST
                    checker.observe(MIN_OBSERVATION_NS, pose, twist)
                    report = checker.report()
                    self.assertFalse(report['passed'])
                    self.assertEqual(report['counts'][vector+'_conflict_messages'], 1)

    def test_first_value_is_not_replaced_and_counters_count_messages(self):
        checker = enough()
        for offset in (1e-7, 2e-7):
            checker.observe(MIN_OBSERVATION_NS, changed(POSE, 0, offset), changed(TWIST, 5, .01))
        report = checker.report()
        self.assertEqual(report['counts']['conflicting_duplicate_messages'], 2)
        self.assertEqual(report['counts']['pose_conflict_messages'], 2)
        self.assertEqual(report['counts']['twist_conflict_messages'], 2)
        self.assertEqual(report['conflicting_stamps'], 1)
        self.assertEqual(report['maximum_same_stamp_deltas']['position_delta_m'], 2e-7)

    def test_late_repeat_and_new_out_of_order_stamp_fail_high_watermark(self):
        checker = enough()
        checker.observe(500_000_000, POSE, TWIST)  # A previously seen stamp.
        checker.observe(510_000_000, POSE, TWIST)  # New stamp, still below watermark.
        report = checker.report()
        self.assertFalse(report['passed'])
        self.assertEqual(report['counts']['backward_messages'], 2)
        self.assertEqual(report['counts']['duplicate_messages'], 1)

    def test_nonfinite_any_component_fails_and_is_json_safe(self):
        for value in (math.nan, math.inf, -math.inf):
            for vector, size in (('pose', 7), ('twist', 6)):
                for index in range(size):
                    with self.subTest(value=value, vector=vector, index=index):
                        checker = enough()
                        pose = changed(POSE, index, value) if vector == 'pose' else POSE
                        twist = changed(TWIST, index, value) if vector == 'twist' else TWIST
                        checker.observe(MIN_OBSERVATION_NS+1, pose, twist)
                        report = checker.report()
                        self.assertFalse(report['passed'])
                        self.assertEqual(report['counts']['nonfinite_messages'], 1)
                        self.assertEqual(report['valid_unique_stamps'], 51)
                        json.dumps(report, allow_nan=False)

    def test_invalid_stamps_and_payloads_cannot_make_observation_sufficient(self):
        checker = OdomIntegrity()
        for invalid in (None, True, 1., math.nan, -1):
            checker.observe(invalid, POSE, TWIST)
        for pose, twist in ((POSE[:-1], TWIST), (None, TWIST), (POSE, [True]*6)):
            checker.observe(0, pose, twist)
        report = checker.report()
        self.assertFalse(report['passed'])
        self.assertEqual(report['counts']['invalid_stamp_messages'], 5)
        self.assertEqual(report['counts']['invalid_payload_messages'], 3)
        self.assertEqual(report['valid_unique_stamps'], 0)
        json.dumps(report, allow_nan=False)

    def test_integer_nanoseconds_do_not_merge_at_large_times(self):
        start = stamp_to_nanoseconds(2_000_000_000, 1)
        self.assertEqual(stamp_to_nanoseconds(2_000_000_000, 2)-start, 1)
        checker = OdomIntegrity()
        checker.observe(start, POSE, TWIST)
        checker.observe(start+1, POSE, TWIST)
        self.assertEqual(checker.report()['unique_stamps'], 2)
        self.assertEqual(checker.report()['counts']['duplicate_messages'], 0)
        self.assertEqual(checker.span_ns, 1)
        for sec, nano in ((-1, 0), (0, -1), (0, 1_000_000_000), (0., 0), (True, 0)):
            with self.assertRaises(ValueError):
                stamp_to_nanoseconds(sec, nano)

    def test_nonfinite_first_value_cannot_be_repaired_by_later_duplicate(self):
        checker = enough()
        checker.observe(2_000_000_000, changed(POSE, 0, math.nan), TWIST)
        checker.observe(2_000_000_000, POSE, TWIST)
        report = checker.report()
        self.assertFalse(report['passed'])
        self.assertEqual(report['counts']['nonfinite_messages'], 1)
        self.assertEqual(report['counts']['duplicate_messages'], 1)

    def test_extreme_finite_delta_report_remains_json_safe(self):
        checker = enough()
        checker.observe(2_000_000_000, changed(POSE, 0, 1e308), TWIST)
        checker.observe(2_000_000_000, changed(POSE, 0, -1e308), TWIST)
        report = checker.report()
        self.assertFalse(report['passed'])
        self.assertTrue(report['delta_overflow'])
        json.dumps(report, allow_nan=False)


if __name__ == '__main__':
    unittest.main()
