"""Pure observer regressions. No ROS/Gazebo imports, nodes, or motion."""
from contextlib import redirect_stderr
import copy
import importlib.util
import io
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest


path = Path(__file__).resolve().parents[1] / 'scripts/observe_sim_mission.py'
spec = importlib.util.spec_from_file_location('sim_mission_observer', path)
observer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(observer)


def header(frame='map', sec=2, nanosec=3):
    return NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nanosec))


def pose(x=1., frame='', quaternion=None):
    return NS(header=header(frame), pose=NS(position=NS(x=x, y=2., z=0.),
        orientation=quaternion or NS(x=0., y=0., z=0., w=1.)))


def native_samples(duration=30.):
    return [{'sim_s':i * .03, 'x':0., 'y':0., 'yaw':0.,
             'receipt_wall_elapsed_s':i * .03} for i in range(round(duration / .03) + 1)]


class SamplingTests(unittest.TestCase):
    def test_throttle_does_not_catch_up_after_a_pause(self):
        gate = observer.SampleGate(10.)
        self.assertEqual([gate.due(t) for t in (0., .05, .1, .5, .5, .55, .6)],
                         [True, False, True, True, False, False, True])

    def test_nonfinite_sample_time_is_rejected(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.subTest(value=value), self.assertRaises(ValueError):
                observer.SampleGate(5.).due(value)

    def test_missing_or_stale_commands_do_not_become_new_zero_commands(self):
        self.assertIsNone(observer.sample_receipt(None, 3., 0))
        source = {'sequence':3, 'receipt_wall_s':1., 'has_header_stamp':False, 'velocity':.2}
        new = observer.sample_receipt(source, 2., 1)
        stale = observer.sample_receipt(source, 3., 3)
        self.assertEqual(new['messages_since_previous_sample'], 2)
        self.assertEqual(stale['messages_since_previous_sample'], 0)
        self.assertFalse(stale['new_message_since_previous_sample'])
        self.assertEqual(stale['receipt_age_wall_s'], 2.)
        self.assertEqual(stale['velocity'], .2)
        self.assertNotIn('receipt_age_wall_s', source)

    def test_twist_retains_all_six_components(self):
        twist = NS(linear=NS(x=1., y=2., z=3.), angular=NS(x=4., y=5., z=6.))
        self.assertEqual(observer.twist_values(twist), [1., 2., 3., 4., 5., 6.])
        for attr in ('linear', 'angular'):
            invalid = copy.deepcopy(twist)
            getattr(invalid, attr).y = math.nan
            with self.subTest(attr=attr), self.assertRaises(ValueError):
                observer.twist_values(invalid)

    def test_parameter_types_are_not_guessed(self):
        for value, expected in [(NS(type=0), None), (NS(type=1, bool_value=True), True),
                (NS(type=2, integer_value=4), 4), (NS(type=3, double_value=.5), .5),
                (NS(type=4, string_value='map'), 'map'),
                (NS(type=9, string_array_value=('FollowPath',)), ['FollowPath'])]:
            with self.subTest(type=value.type):
                self.assertEqual(observer.decode_parameter(value), expected)


class NativeStationarityTests(unittest.TestCase):
    def test_fresh_full_window_after_terminal_is_sufficient(self):
        tail = observer.native_tail(native_samples(), 30., -.1)
        self.assertTrue(tail['ready_for_operator_to_end_observer'])
        self.assertEqual(tail['sample_count'], 1001)
        self.assertAlmostEqual(tail['span_s'], 30.)
        self.assertAlmostEqual(tail['max_gap_s'], .03)

    def test_short_window_cannot_prove_thirty_seconds_of_stop(self):
        for duration in (0., 10., 20.):
            with self.subTest(duration=duration):
                self.assertFalse(observer.native_tail(native_samples(duration), duration, -.1)['evidence_sufficient'])

    def test_paused_repeated_or_reversed_native_time_is_insufficient(self):
        for mutation in ('duplicate', 'reverse', 'gap'):
            rows = native_samples()
            if mutation == 'duplicate':
                rows[500]['sim_s'] = rows[499]['sim_s']
            elif mutation == 'reverse':
                rows[500]['sim_s'] = rows[499]['sim_s'] - .01
            else:
                del rows[400:410]
            with self.subTest(mutation=mutation):
                self.assertFalse(observer.native_tail(rows, 30., -.1)['evidence_sufficient'])

    def test_motion_within_window_prevents_stop_claim(self):
        for field, excursion in (('x', .002), ('y', .002), ('yaw', .003)):
            rows = native_samples(); rows[500][field] = excursion
            with self.subTest(field=field):
                result = observer.native_tail(rows, 30., -.1)
                self.assertTrue(result['evidence_sufficient'])
                self.assertFalse(result['stationary_within_1mm_002rad'])

    def test_wrapped_yaw_does_not_invent_a_full_rotation(self):
        rows = native_samples()
        for i, row in enumerate(rows):
            row['yaw'] = math.pi - .00001 if i % 2 else -math.pi + .00001
        result = observer.native_tail(rows, 30., -.1)
        self.assertTrue(result['stationary_within_1mm_002rad'])
        self.assertAlmostEqual(result['yaw_span_rad'], .00002)

    def test_stale_receipt_or_preterminal_window_cannot_end_observer(self):
        self.assertFalse(observer.native_tail(native_samples(), 32., -.1)['evidence_sufficient'])
        for terminal in (None, 10.):
            with self.subTest(terminal=terminal):
                result = observer.native_tail(native_samples(), 30., terminal)
                self.assertTrue(result['stationary_within_1mm_002rad'])
                self.assertFalse(result['ready_for_operator_to_end_observer'])


class PathEvidenceTests(unittest.TestCase):
    def test_retains_full_path_source_and_individual_pose_frames_and_stamps(self):
        msg = NS(header=header('map'), poses=[pose(), pose(3., 'odom')])
        msg.poses[1].header.stamp.nanosec = 7
        record = observer.path_record(msg)
        self.assertEqual(record['stamp_ns'], 2_000_000_003)
        self.assertEqual(record['poses'][1]['stamp_ns'], 2_000_000_007)
        self.assertEqual(record['poses'][1]['xyz'], [3., 2., 0.])
        self.assertEqual(record['poses'][0]['frame_id'], 'map')
        self.assertEqual(record['poses'][1]['frame_id'], 'odom')
        self.assertEqual(record['poses'][0]['yaw'], 0.)
        self.assertFalse(record['all_pose_frames_match_path'])

    def test_restamping_does_not_change_geometry(self):
        msg = NS(header=header(), poses=[pose()])
        original = observer.path_geometry_key(observer.path_record(msg))
        msg.header.stamp.sec = 5; msg.poses[0].header.stamp.sec = 6
        self.assertEqual(original, observer.path_geometry_key(observer.path_record(msg)))

    def test_position_orientation_or_frame_changes_are_preserved(self):
        original = NS(header=header(), poses=[pose()])
        key = observer.path_geometry_key(observer.path_record(original))
        for field in ('position', 'orientation', 'frame'):
            changed = copy.deepcopy(original)
            if field == 'position':changed.poses[0].pose.position.x += .001
            elif field == 'orientation':changed.poses[0].pose.orientation = NS(x=0., y=0., z=1., w=0.)
            else:changed.header.frame_id = 'odom'
            with self.subTest(field=field):
                self.assertNotEqual(key, observer.path_geometry_key(observer.path_record(changed)))

    def test_unset_orientation_has_no_invented_yaw(self):
        msg = NS(header=header(), poses=[pose(quaternion=NS(x=0., y=0., z=0., w=0.))])
        point = observer.path_record(msg)['poses'][0]
        self.assertIsNone(point['yaw'])
        self.assertFalse(point['orientation_valid'])
        self.assertEqual(point['quaternion_xyzw'], [0., 0., 0., 0.])

    def test_nonfinite_position_and_orientation_are_rejected(self):
        for field in ('position', 'orientation'):
            msg = NS(header=header(), poses=[pose()])
            getattr(msg.poses[0].pose, field).x = math.inf
            with self.subTest(field=field), self.assertRaises(ValueError):
                observer.path_record(msg)

    def test_empty_path_is_an_explicit_empty_message(self):
        record = observer.path_record(NS(header=header(), poses=[]))
        self.assertEqual(record['pose_count'], 0)
        self.assertEqual(record['poses'], [])
        self.assertEqual(record['stamp_ns'], 2_000_000_003)


class FileAndWindowTests(unittest.TestCase):
    def test_window_boundaries_and_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'capture.json'
            for seconds in (10., 1000., 1500.):
                args, snapshots = observer.parse_arguments(['--output', str(output), '--seconds', str(seconds)])
                self.assertEqual(args.seconds, seconds)
                self.assertEqual(snapshots, output.with_name('capture-snapshots'))
                self.assertFalse(output.exists())
            args, _ = observer.parse_arguments(['--output', str(output)])
            self.assertEqual(args.seconds, 1000.)

    def test_invalid_windows_are_rejected_before_loading_ros(self):
        for value in ('9.99', '1500.01', 'nan', 'inf', '-inf'):
            with self.subTest(value=value), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                observer.parse_arguments(['--seconds=' + value])
            self.assertEqual(raised.exception.code, 2)

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'capture.json'; output.write_text('existing evidence')
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                observer.parse_arguments(['--output', str(output)])
            self.assertEqual(output.read_text(), 'existing evidence')

    def test_existing_snapshot_directory_is_never_reused(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'capture.json'
            snapshots = Path(directory) / 'capture-snapshots'; snapshots.mkdir()
            sentinel = snapshots / 'original.json'; sentinel.write_text('original')
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                observer.parse_arguments(['--output', str(output)])
            self.assertFalse(output.exists())
            self.assertEqual(sentinel.read_text(), 'original')


class FootprintSnapshotTests(unittest.TestCase):
    def test_raw_unknown_and_occupied_cells_are_never_accepted_as_free(self):
        record = {'map':{'resolution':1., 'origin':[0., 0., 0.], 'width':3, 'height':3}, 'map_data':[0] * 9}
        self.assertTrue(observer.disk_conflicts(record, [1.5, 1.5], .25)['raw_exact_pose_disk_clear'])
        for value, key in ((-1, 'unknown'), (50, 'occupied_or_intermediate'), (100, 'occupied_or_intermediate')):
            record['map_data'][4] = value
            with self.subTest(value=value):
                result = observer.disk_conflicts(record, [1.5, 1.5], .25)
                self.assertFalse(result['raw_exact_pose_disk_clear'])
                self.assertEqual(result['conflict_counts'][key], 1)

    def test_rotated_origin_and_map_edge_are_respected(self):
        record = {'map':{'resolution':1., 'origin':[10., 20., math.pi / 2], 'width':3, 'height':3}, 'map_data':[0] * 9}
        self.assertTrue(observer.disk_conflicts(record, [8.5, 21.5], .25)['raw_exact_pose_disk_clear'])
        outside = observer.disk_conflicts(record, [9.9, 20.1], .25)
        self.assertGreater(outside['conflict_counts']['outside'], 0)


if __name__ == '__main__':
    unittest.main()
