"""Pure observer regressions. No ROS/Gazebo imports, nodes, or motion."""
from contextlib import redirect_stderr
from array import array
import copy
from concurrent.futures import Future
import importlib.util
import io
import json
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


def costmap_message():
    return NS(header=header('odom', 12, 345), metadata=NS(
        map_load_time=NS(sec=0, nanosec=0), update_time=NS(sec=11, nanosec=123),
        layer='master', resolution=.05, size_x=3, size_y=2,
        origin=NS(position=NS(x=-1., y=2., z=.1),
            orientation=NS(x=0., y=0., z=math.sqrt(.5), w=math.sqrt(.5)))),
        data=array('B', [0, 1, 252, 253, 254, 255]))


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


class ParameterQueryTests(unittest.TestCase):
    def setUp(self):
        self.calls=[];self.removed=[];self.ready=True
        def send(request):
            future=Future();self.calls.append((request,future));return future
        client=NS(service_is_ready=lambda:self.ready,call_async=send,
                  remove_pending_request=self.removed.append)
        self.query=observer.ParameterQuery(client,['radius'],NS)

    def complete(self, value=.25):
        self.calls[-1][1].set_result(NS(values=[NS(type=3,double_value=value)]))

    def test_absent_service_does_not_consume_attempts_or_block(self):
        self.ready=False
        for elapsed in (0.,5.,50.):self.assertIsNone(self.query.tick(elapsed))
        self.assertEqual(self.calls,[])
        self.ready=True;self.query.tick(51.)
        self.complete();self.assertEqual(self.query.tick(51.1)['values'],{'radius':.25})
        self.query.tick(100.);self.assertEqual(len(self.calls),1)

    def test_timeout_detaches_old_request_and_retries_after_delay(self):
        self.query.tick(0.);old=self.calls[0][1]
        self.query.tick(4.9);self.assertFalse(old.cancelled())
        self.query.tick(5.)
        self.assertTrue(old.cancelled());self.assertEqual(self.removed,[old])
        self.query.tick(5.9);self.assertEqual(len(self.calls),1)
        self.query.tick(6.);self.complete(.3)
        self.assertEqual(self.query.tick(6.1)['values'],{'radius':.3})
        self.assertEqual([a['status'] for a in self.query.evidence['attempts']],['timed_out','succeeded'])

    def test_completion_observed_at_timeout_wins(self):
        self.query.tick(0.);self.complete()
        self.assertIsNotNone(self.query.tick(5.))
        self.assertEqual(self.query.evidence['attempts'][0]['status'],'succeeded')

    def test_permanently_lost_replies_stop_after_three_attempts(self):
        for elapsed in (0.,5.,6.,11.,12.,17.,100.):self.query.tick(elapsed)
        self.assertTrue(self.query.done);self.assertEqual(len(self.calls),3)
        self.assertTrue(all(f.cancelled() for _,f in self.calls))
        self.assertEqual(len(self.removed),3)

    def test_failed_or_malformed_response_is_recorded_and_retryable(self):
        for failure in ('exception','short','nonfinite'):
            self.setUp();self.query.tick(0.);future=self.calls[0][1]
            if failure=='exception':future.set_exception(RuntimeError('server unavailable'))
            elif failure=='short':future.set_result(NS(values=[]))
            else:self.complete(math.nan)
            self.assertIsNone(self.query.tick(.1))
            self.assertEqual(self.query.evidence['attempts'][0]['status'],'failed')
            self.query.tick(1.2);self.complete();self.assertIsNotNone(self.query.tick(1.3))

    def test_send_exception_is_bounded_and_does_not_create_pending_future(self):
        def fail(_request):raise RuntimeError('send failed')
        self.query.client.call_async=fail
        for elapsed in (0.,1.,2.,3.,50.):self.query.tick(elapsed)
        self.assertEqual(len(self.query.evidence['attempts']),3)
        self.assertTrue(self.query.done);self.assertIsNone(self.query.future)

    def test_shutdown_detaches_pending_request_without_inventing_result(self):
        self.query.tick(0.);future=self.calls[0][1];self.query.close(.2)
        self.assertTrue(future.cancelled());self.assertEqual(self.removed,[future])
        self.assertEqual(self.query.evidence['attempts'][0]['status'],'observer_stopped')
        self.assertIsNone(self.query.future)
        self.query.tick(10.);self.assertEqual(len(self.calls),1)


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
            self.assertFalse(args.costmaps)

    def test_costmaps_are_explicitly_opted_in_before_loading_ros(self):
        with tempfile.TemporaryDirectory() as directory:
            args, _ = observer.parse_arguments(['--output', str(Path(directory) / 'capture.json'), '--costmaps'])
            self.assertTrue(args.costmaps)

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


class CostmapEvidenceTests(unittest.TestCase):
    def test_raw_bytes_metadata_and_rotated_origin_are_preserved(self):
        msg = costmap_message()
        record = observer.costmap_record(msg)
        self.assertEqual(record['data'], [0, 1, 252, 253, 254, 255])
        self.assertEqual(record['frame_id'], 'odom')
        self.assertEqual(record['stamp_ns'], 12_000_000_345)
        meta = record['metadata']
        self.assertEqual((meta['map_load_time_ns'], meta['update_time_ns']), (0, 11_000_000_123))
        self.assertEqual((meta['size_x'], meta['size_y'], meta['resolution'], meta['layer']), (3, 2, .05, 'master'))
        self.assertEqual(meta['origin']['xyz'], [-1., 2., .1])
        self.assertEqual(meta['origin']['quaternion_xyzw'], [0., 0., math.sqrt(.5), math.sqrt(.5)])
        self.assertAlmostEqual(meta['origin']['yaw'], math.pi / 2)
        record['data'][0] = 255
        self.assertEqual(list(msg.data), [0, 1, 252, 253, 254, 255])

    def test_unset_origin_orientation_does_not_invent_a_yaw(self):
        msg = costmap_message(); msg.metadata.origin.orientation = NS(x=0., y=0., z=0., w=0.)
        origin = observer.costmap_record(msg)['metadata']['origin']
        self.assertFalse(origin['orientation_valid'])
        self.assertIsNone(origin['yaw'])

    def test_invalid_geometry_dimensions_and_data_are_rejected(self):
        mutations = [lambda m: setattr(m.metadata, 'size_x', 0),
            lambda m: setattr(m.metadata, 'size_y', 1.5),
            lambda m: setattr(m.metadata, 'resolution', 0.),
            lambda m: setattr(m.metadata, 'resolution', math.inf),
            lambda m: setattr(m.metadata.origin.position, 'x', math.nan),
            lambda m: setattr(m.metadata.origin.orientation, 'w', math.nan),
            lambda m: setattr(m, 'data', [0] * 5)]
        mutations += [lambda m, value=value: setattr(m, 'data', [0, 1, 2, 3, 4, value])
                      for value in (-1, 256, 1.5, True)]
        for index, mutate in enumerate(mutations):
            msg = costmap_message(); mutate(msg)
            with self.subTest(index=index), self.assertRaises(ValueError):
                observer.costmap_record(msg)

    def test_invalid_source_or_metadata_stamp_is_rejected(self):
        for field in ('header', 'map_load_time', 'update_time'):
            msg = costmap_message()
            stamp = msg.header.stamp if field == 'header' else getattr(msg.metadata, field)
            stamp.nanosec = 1_000_000_000
            with self.subTest(field=field), self.assertRaises(ValueError):
                observer.costmap_record(msg)

    def test_only_new_receipts_are_saved_at_most_once_per_wall_second(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = observer.CostmapRecorder(True, Path(directory))
            msg = costmap_message()
            def receive(t):recorder.receive('global', msg, {'wall_s':t, 'ros_s':12.}, 2, 'epoch')
            def sample(t):return recorder.sample('global', {'wall_s':t, 'ros_s':12.}, 2, 'epoch')
            receive(0.); first = sample(0.)
            receive(.1); receive(.2)
            self.assertIsNone(sample(.999))
            second = sample(1.)
            self.assertEqual(second['sequence'], 3)
            self.assertEqual(second['messages_since_previous_sample'], 2)
            self.assertIsNone(sample(2.))  # Cached message is not a new sample.
            receive(2.1); third = sample(2.1)  # New receipt of identical bytes/stamp is still a message.
            self.assertEqual(third['sequence'], 4)
            self.assertEqual(len(recorder.files), 3)
            self.assertNotIn('data', first)
            self.assertNotIn('metadata', first)
            self.assertEqual(json.loads(Path(first['file']).read_text())['data'], list(msg.data))

    def test_source_receipt_sample_times_and_goal_epoch_changes_are_distinct(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = observer.CostmapRecorder(True, Path(directory))
            msg = costmap_message()
            recorder.receive('local', msg, {'wall_s':2., 'ros_s':13.}, 2, 'old')
            manifest = recorder.sample('local', {'wall_s':2.25, 'ros_s':14.}, 3, 'new')
            row = json.loads(Path(manifest['file']).read_text())
            self.assertEqual(row['receipt_age_wall_s'], .25)
            self.assertAlmostEqual(row['header_age_ros_s'], 1.999999655)
            self.assertAlmostEqual(row['header_age_at_receipt_ros_s'], .999999655)
            self.assertEqual((row['active_goal_token_at_receipt'], row['active_goal_token']), (2, 3))
            self.assertEqual((row['map_epoch_at_receipt'], row['map_epoch']), ('old', 'new'))
            self.assertIn('not synchronized', row['synchronization'])
            msg = costmap_message(); msg.header.stamp.sec = 20
            recorder.receive('local', msg, {'wall_s':4., 'ros_s':15.}, None, 'new')
            row = recorder.sample('local', {'wall_s':4., 'ros_s':15.}, None, 'new')
            self.assertLess(row['header_age_ros_s'], 0.)  # Do not hide a future source stamp by clamping age.

    def test_both_maps_sample_independently_and_unsampled_tail_is_counted(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = observer.CostmapRecorder(True, Path(directory))
            at = {'wall_s':0., 'ros_s':12.}
            for name in observer.COSTMAP_TOPICS:
                recorder.receive(name, costmap_message(), at, None, None)
                self.assertIsNotNone(recorder.sample(name, at, None, None))
            recorder.receive('local', costmap_message(), {'wall_s':.5, 'ros_s':12.1}, None, None)
            self.assertIsNone(recorder.sample('local', {'wall_s':.5, 'ros_s':12.1}, None, None))
            summary = recorder.summary()
            self.assertEqual(summary['missing_maps'], [])
            self.assertEqual(summary['topics']['local']['received_message_count'], 2)
            self.assertEqual(summary['topics']['local']['saved_sample_count'], 1)
            self.assertEqual(summary['topics']['local']['messages_after_last_saved_sample'], 1)

    def test_missing_maps_are_explicit_only_when_enabled(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = observer.CostmapRecorder(False, Path(directory))
            self.assertIsNone(recorder.sample('global', {'wall_s':0., 'ros_s':0.}, None, None))
            self.assertEqual(recorder.summary()['missing_maps'], [])
            self.assertEqual(list(Path(directory).iterdir()), [])
            recorder = observer.CostmapRecorder(True, Path(directory))
            self.assertEqual(recorder.summary()['missing_maps'], ['global', 'local'])
            at = {'wall_s':0., 'ros_s':12.}
            recorder.receive('global', costmap_message(), at, None, None)
            recorder.sample('global', at, None, None)
            self.assertEqual(recorder.summary()['missing_maps'], ['local'])

    def test_invalid_receipt_is_not_retried_in_a_busy_loop(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = observer.CostmapRecorder(True, Path(directory))
            msg = costmap_message(); msg.data = [0]
            at = {'wall_s':0., 'ros_s':12.}
            recorder.receive('global', msg, at, None, None)
            with self.assertRaises(ValueError):recorder.sample('global', at, None, None)
            self.assertIsNone(recorder.sample('global', {'wall_s':2., 'ros_s':13.}, None, None))
            recorder.receive('global', costmap_message(), {'wall_s':2., 'ros_s':13.}, None, None)
            self.assertIsNotNone(recorder.sample('global', {'wall_s':2., 'ros_s':13.}, None, None))
            summary = recorder.summary()['topics']['global']
            self.assertEqual((summary['attempted_sample_count'], summary['failed_sample_count'], summary['saved_sample_count']), (2, 1, 1))

    def test_snapshot_file_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'costmap-global-00000001.json'; path.write_text('existing evidence')
            recorder = observer.CostmapRecorder(True, Path(directory)); at = {'wall_s':0., 'ros_s':12.}
            recorder.receive('global', costmap_message(), at, None, None)
            with self.assertRaises(FileExistsError):recorder.sample('global', at, None, None)
            self.assertEqual(path.read_text(), 'existing evidence')
            self.assertEqual(recorder.summary()['topics']['global']['saved_sample_count'], 0)


if __name__ == '__main__':
    unittest.main()
