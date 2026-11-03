"""Pure tests: production image core + synthetic arrays, never ROS or Gazebo."""
import copy
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
import zlib

import numpy as np

import capture_core as c
import capture_red_samples as runtime


def header(ns=10_000_000_000, frame='camera_optical_frame'):
    return NS(stamp=NS(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000), frame_id=frame)


def image(array, encoding, ns=10_000_000_000, padding=0, bigendian=False):
    h, w = array.shape[:2]
    if encoding in ('32FC1', '16UC1'):
        array = array.astype(('>' if bigendian else '<')+('f4' if encoding == '32FC1' else 'u2'))
    rows = [row.tobytes()+b'\xa5'*padding for row in array]
    return NS(header=header(ns), width=w, height=h, encoding=encoding,
              step=len(rows[0]), is_bigendian=int(bigendian), data=b''.join(rows))


def info(ns=10_000_000_000):
    return NS(header=header(ns), width=80, height=60, distortion_model='plumb_bob',
              k=[100.,0.,39.5,0.,100.,29.5,0.,0.,1.], d=[0.]*5,
              r=[1.,0.,0.,0.,1.,0.,0.,0.,1.],
              p=[100.,0.,39.5,0.,0.,100.,29.5,0.,0.,0.,1.,0.],
              binning_x=0, binning_y=0,
              roi=NS(x_offset=0,y_offset=0,height=0,width=0,do_rectify=False))


def fixture(ns=10_000_000_000, red=True, depth_encoding='32FC1', padding=0, bigendian=False):
    rgb = np.zeros((60,80,3), np.uint8)
    if red: rgb[20:40,30:50] = [255,0,0]
    depth = np.full((60,80), 2 if depth_encoding == '32FC1' else 2000,
                    np.float32 if depth_encoding == '32FC1' else np.uint16)
    return {'rgb': image(rgb, 'rgb8', ns, padding),
            'depth': image(depth, depth_encoding, ns, padding, bigendian),
            'rgb_info': info(ns), 'depth_info': info(ns)}


def transforms(ns=10_000_000_000):
    result = {}
    for key, target, source, stamp in (
        ('reference_to_base','odom','base_footprint',ns),
        ('base_to_camera','base_footprint','camera_optical_frame',0),
        ('reference_to_camera','odom','camera_optical_frame',ns)):
        result[key] = {'query_stamp_ns': ns, 'returned_stamp_ns': stamp,
                       'target_frame': target, 'source_frame': source,
                       'translation_xyz_m': [0.,0.,0.], 'rotation_xyzw': [0.,0.,0.,1.]}
    return result


class EvaluationTests(unittest.TestCase):
    def test_current_core_runs_and_preserves_optical_projection(self):
        report, arrays = c.evaluate(fixture(), transforms(), registered=True)
        self.assertTrue(report['frame_valid'])
        self.assertEqual(report['processing_status'], 'processed')
        self.assertEqual(len(report['detections']), 1)
        np.testing.assert_allclose(report['detections'][0]['position'], [0,0,2])
        self.assertEqual(report['stamp_ns'], 10_000_000_000)
        self.assertEqual(report['processing']['detection_count'], 1)
        self.assertEqual(arrays['red_mask'].sum(), 400*255)
    def test_blank_is_processed_empty_not_truth_negative(self):
        report, _ = c.evaluate(fixture(red=False), transforms(), registered=True)
        self.assertTrue(report['frame_valid'])
        self.assertEqual(report['detections'], [])
        self.assertEqual(report['detection_summary'], 'no_red_candidates')
        self.assertNotIn('target_absent', report)
    def test_missing_tf_keeps_independent_detection_but_rejects_frame(self):
        report, _ = c.evaluate(fixture(), {}, registered=True)
        self.assertFalse(report['frame_valid'])
        self.assertEqual(len(report['detections']), 1)
        self.assertEqual(len(report['tf_reasons']), 3)
    def test_registration_must_be_explicit(self):
        report, _ = c.evaluate(fixture(), transforms(), registered=False)
        self.assertEqual(report['input_reasons'], ['depth_registration_unconfirmed'])
        self.assertIsNone(report['detections'])
    def test_different_integer_stamps_rejected_even_one_nanosecond(self):
        data = fixture()
        data['depth'].header = header(10_000_000_001)
        report, _ = c.evaluate(data, transforms(), registered=True)
        self.assertIn('rgb_info:rgb_depth_unsynchronized', report['input_reasons'])
        self.assertIsNone(report['detections'])
    def test_nonidentical_camera_stamps_and_calibration_rejected(self):
        data = fixture()
        data['depth_info'].header = header(9_999_999_999)
        report, _ = c.evaluate(data, transforms(), registered=True)
        self.assertIn('depth_info:stale_camera_info', report['input_reasons'])
        data = fixture()
        data['depth_info'].k[0] += 1.
        report, _ = c.evaluate(data, transforms(), registered=True)
        self.assertIn('registered_camera_calibration_mismatch', report['input_reasons'])
    def test_malformed_buffer_and_nonfinite_calibration_rejected(self):
        data = fixture()
        data['depth'].data = data['depth'].data[:-1]
        report, _ = c.evaluate(data, transforms(), registered=True)
        self.assertIn('rgb_info:invalid_image_buffer', report['input_reasons'])
        data = fixture()
        data['rgb_info'].k[0] = float('nan')
        report, _ = c.evaluate(data, transforms(), registered=True)
        self.assertIn('rgb_info:invalid_calibration', report['input_reasons'])
        self.assertNotIn('NaN', c.json_bytes(report).decode())
    def test_bad_frames_and_nonrectified_roi_rejected(self):
        data = fixture()
        data['depth'].header.frame_id = 'unregistered_depth_frame'
        report, _ = c.evaluate(data, transforms(), registered=True)
        self.assertIn('rgb_info:frame_mismatch', report['input_reasons'])
        data = fixture()
        data['rgb_info'].roi.do_rectify = True
        report, _ = c.evaluate(data, transforms(), registered=True)
        self.assertIn('rgb_info:unsupported_roi_or_binning', report['input_reasons'])
    def test_padding_and_big_endian_mm_match_float_metres(self):
        expected, _ = c.evaluate(fixture(), transforms(), registered=True)
        report, arrays = c.evaluate(fixture(depth_encoding='16UC1', padding=6, bigendian=True),
                                    transforms(), registered=True)
        self.assertTrue(report['frame_valid'])
        self.assertEqual(report['detections'], expected['detections'])
        np.testing.assert_allclose(arrays['depth_m'], 2.)
        self.assertEqual(arrays['depth_raw'].dtype, np.uint16)
    def test_invalid_whole_frame_depth_is_not_negative_detection(self):
        data = fixture()
        depth = np.full((60,80), np.nan, np.float32)
        data['depth'] = image(depth, '32FC1')
        report, arrays = c.evaluate(data, transforms(), registered=True)
        self.assertFalse(report['frame_valid'])
        self.assertEqual(report['detections'], [])
        self.assertIn('insufficient_frame_depth', report['input_reasons'])
        self.assertEqual(report['processing']['rejection_counts']['insufficient_valid_depth'], 1)
        self.assertTrue(np.isnan(arrays['depth_raw']).all())
    def test_unlocalized_red_candidate_is_explicit_algorithm_result(self):
        data = fixture()
        depth = np.full((60,80), 2, np.float32)
        depth[20:40,30:50] = np.inf
        data['depth'] = image(depth, '32FC1')
        report, _ = c.evaluate(data, transforms(), registered=True)
        self.assertTrue(report['frame_valid'])
        self.assertEqual(report['detection_summary'], 'all_candidates_rejected')
        self.assertEqual(report['processing']['unlocalized_candidate_count'], 1)
    def test_bad_dynamic_tf_stamp_and_nonfinite_quaternion_rejected(self):
        tf = transforms()
        tf['reference_to_camera']['returned_stamp_ns'] = 0
        tf['base_to_camera']['rotation_xyzw'][0] = float('nan')
        report, _ = c.evaluate(fixture(), tf, registered=True)
        self.assertIn('tf_return_stamp_mismatch:reference_to_camera', report['tf_reasons'])
        self.assertIn('invalid_tf_values:base_to_camera', report['tf_reasons'])
    def test_latest_time_query_and_wrong_frame_rejected(self):
        tf = transforms()
        tf['reference_to_base']['query_stamp_ns'] = 0
        tf['base_to_camera']['source_frame'] = 'wrong'
        report, _ = c.evaluate(fixture(), tf, registered=True)
        self.assertEqual(len([r for r in report['tf_reasons'] if 'query_stamp' in r]), 2)


class WindowTests(unittest.TestCase):
    def setUp(self):
        self.window = c.FixedWindow(2)
        self.window.on_clock(9_000_000_000)
    def add(self, ns, **overrides):
        data = fixture(ns)
        data.update(overrides)
        for key, msg in data.items(): self.window.receive(key, msg, 1.)
        return data
    def test_fixed_first_two_pairs_even_when_first_rejected(self):
        bad = fixture()['depth']
        bad.data = bad.data[:-1]
        self.add(10_000_000_000, depth=bad)
        self.add(10_100_000_000)
        self.add(10_200_000_000)
        self.assertEqual([p['stamp_ns'] for p in self.window.pending], [10_000_000_000,10_100_000_000])
        result, _ = c.evaluate(self.window.messages(self.window.pending[0]), transforms(), registered=True)
        self.assertFalse(result['frame_valid'])
    def test_duplicate_does_not_count_or_replace_first_payload(self):
        data = self.add(10_000_000_000)
        self.window.receive('rgb', data['rgb'], 1.1)
        self.assertEqual(len(self.window.pending), 1)
        altered = copy.deepcopy(data['rgb'])
        altered.data = bytes([1])+altered.data[1:]
        self.window.receive('rgb', altered, 1.2)
        self.assertIn(10_000_000_000, self.window.conflict_stamps)
        self.assertEqual(self.window.pending[0]['rgb'].data, data['rgb'].data)
    def test_before_barrier_and_clock_reset_do_not_select_frames(self):
        self.add(8_000_000_000)
        self.assertEqual(self.window.pending, [])
        self.window.on_clock(8_500_000_000)
        self.add(10_000_000_000)
        self.assertTrue(self.window.clock_reset)
        self.assertEqual(self.window.pending, [])
    def test_info_arriving_later_is_retained_for_tf_wait(self):
        data = fixture()
        for key in ('rgb', 'depth'): self.window.receive(key, data[key], 1.)
        sample = self.window.pending[0]
        self.assertIsNone(self.window.messages(sample)['rgb_info'])
        for key in ('rgb_info', 'depth_info'): self.window.receive(key, data[key], 1.2)
        self.assertIsNotNone(self.window.messages(sample)['rgb_info'])
        self.window.cache['rgb_info'].clear()
        self.assertIsNotNone(self.window.messages(sample)['rgb_info'])
    def test_unmatched_caches_are_bounded(self):
        for n in range(100):
            self.window.receive('rgb', fixture(10_000_000_000+n)['rgb'], float(n))
        self.assertLessEqual(len(self.window.cache['rgb']), 16)
        self.assertLessEqual(len(self.window.signatures['rgb']), 64)
        self.assertEqual(self.window.pending, [])
    def test_pair_availability_order_is_not_global_sensor_stamp_order(self):
        older, newer = fixture(10_000_000_000), fixture(10_100_000_000)
        self.window.receive('rgb', older['rgb'], 1.)
        for key in ('rgb', 'depth'):
            self.window.receive(key, newer[key], 1.1)
        self.window.receive('depth', older['depth'], 1.2)
        self.assertEqual([p['stamp_ns'] for p in self.window.pending], [10_100_000_000])
        self.assertEqual(self.window.counts['nonadvancing_pair'], 1)
        self.assertIn('callback-time', runtime.SELECTION_POLICY)
    def test_selected_four_topic_signatures_survive_65_newer_messages(self):
        for kind in c.TOPICS:
            with self.subTest(kind=kind):
                window = c.FixedWindow(2)
                window.on_clock(9_000_000_000)
                original = fixture()
                for key,msg in original.items(): window.receive(key,msg,1.)
                self.assertIn(10_000_000_000, window.pinned_signatures[kind])
                for n in range(65):
                    window.receive(kind, fixture(10_100_000_000+n)[kind], 2.+n)
                self.assertNotIn(10_000_000_000, window.signatures[kind])
                altered = copy.deepcopy(original[kind])
                if kind in ('rgb','depth'):
                    altered.data = bytes([1])+altered.data[1:]
                else:
                    altered.k[0] += 1.
                window.receive(kind,altered,70.)
                self.assertEqual(window.counts['conflicting_duplicate'],1)
                self.assertIn(10_000_000_000,window.conflict_stamps)
                conflict = window.selected_conflicts()[0]
                self.assertEqual(conflict['sequence'],1)
                self.assertIn(kind,conflict['topics'])
                self.assertNotEqual(conflict['topics'][kind]['retained_signature'],
                                    conflict['topics'][kind]['first_conflicting_signature'])
                self.assertLessEqual(len(window.signatures[kind]),64)
                self.assertEqual(len(window.pinned_signatures[kind]),1)
    def test_maximum_32_pins_per_topic_remain_bounded_under_more_traffic(self):
        window = c.FixedWindow(32)
        window.on_clock(9_000_000_000)
        for n in range(100):
            for kind,msg in fixture(10_000_000_000+n).items():
                window.receive(kind,msg,float(n))
        self.assertEqual(len(window.selected_sequences),32)
        self.assertTrue(all(len(pins)==32 for pins in window.pinned_signatures.values()))
        self.assertTrue(all(len(history)<=64 for history in window.signatures.values()))
        self.assertTrue(all(len(cache)<=16 for cache in window.cache.values()))
    def test_unselected_conflicts_are_bounded_but_collection_failure_is_sticky(self):
        for n in range(100):
            message=fixture(10_000_000_000+n)['rgb']
            self.window.receive('rgb',message,float(n))
            altered=copy.deepcopy(message)
            altered.data=bytes([1])+altered.data[1:]
            self.window.receive('rgb',altered,float(n))
        self.assertLessEqual(len(self.window.conflict_stamps),64)
        self.assertLessEqual(len(self.window.conflict_details),64)
        for n in range(65):
            self.window.receive('rgb',fixture(11_000_000_000+n)['rgb'],101.+n)
        self.assertFalse(self.window.conflict_stamps)
        self.assertEqual(self.window.counts['conflicting_duplicate'],100)
        self.assertEqual(runtime.end_of_window_integrity(self.window)['errors'],
                         ['conflicting_duplicate_input_observed'])


class StorageAndRuntimeBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='test-', dir=Path(__file__).parent)
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
    def test_exclusive_output_and_file_writes(self):
        w = c.CaptureWriter(self.root/'capture')
        w.write('one.json', b'{}')
        with self.assertRaises(FileExistsError): w.write('one.json', b'changed')
        with self.assertRaises(FileExistsError): c.CaptureWriter(self.root/'capture')
        self.assertEqual((w.root/'one.json').read_bytes(), b'{}')
    def test_byte_budget_keeps_reserved_final_report_space(self):
        w = c.CaptureWriter(self.root/'capture', max_bytes=c.CaptureWriter.REPORT_RESERVE_BYTES+4)
        w.write('first', b'1234')
        with self.assertRaisesRegex(OSError, 'budget'): w.write('second', b'x')
        w.write_report('raw-checkpoint.json', {})
        w.write_report('summary.json', {})
        self.assertGreater(w.written, 4)
        self.assertEqual(len(w.artifacts), 3)
    def test_saved_source_bytes_depth_arrays_png_and_hashes_are_reproducible(self):
        messages = fixture()
        report, arrays = c.evaluate(messages, transforms(), registered=True)
        w = c.CaptureWriter(self.root/'capture')
        path = w.save_sample(1, messages, report, arrays)
        saved = json.loads((w.root/path).read_text())
        self.assertEqual((w.root/'sample_001/rgb.data').read_bytes(), messages['rgb'].data)
        np.testing.assert_array_equal(np.load(w.root/'sample_001/depth_raw.npy', allow_pickle=False), arrays['depth_raw'])
        png = (w.root/'sample_001/rgb.png').read_bytes()
        self.assertEqual(png[:8], b'\x89PNG\r\n\x1a\n')
        offset, compressed = 8, b''
        while offset < len(png):
            length = struct.unpack('!I', png[offset:offset+4])[0]
            name = png[offset+4:offset+8]
            payload = png[offset+8:offset+8+length]
            if name == b'IDAT': compressed += payload
            offset += 12+length
        pixels = zlib.decompress(compressed)
        self.assertEqual(pixels, b''.join(b'\0'+row.tobytes() for row in arrays['rgb']))
        self.assertEqual(saved['files']['rgb.data']['sha256'], saved['input_metadata']['rgb']['data_sha256'])
    def test_real_lookup_adapter_always_requests_exact_integer_stamp(self):
        calls = []
        class Buffer:
            def lookup_transform(self, target, source, stamp, timeout):
                calls.append((target,source,stamp.nanoseconds,timeout.nanoseconds))
                return NS(header=header(stamp.nanoseconds,target), child_frame_id=source,
                          transform=NS(translation=NS(x=0.,y=0.,z=0.), rotation=NS(x=0.,y=0.,z=0.,w=1.)))
        constructor = lambda nanoseconds: NS(nanoseconds=nanoseconds)
        values, errors = runtime.read_exact_transforms(Buffer(), 12345678901,
            'camera_optical_frame', 'odom', 'base_footprint', constructor, constructor)
        self.assertFalse(errors)
        self.assertEqual(len(values), 3)
        self.assertTrue(all(call[2:] == (12345678901,0) for call in calls))
    def test_import_and_disabled_cli_path_do_not_import_ros(self):
        self.assertNotIn('rclpy', sys.modules)
        with self.assertRaisesRegex(ValueError, 'no ROS connection'):
            runtime.validate_args(NS(execute=False))
        self.assertNotIn('rclpy', sys.modules)
    def test_source_provenance_is_current_checkout_not_installed_overlay(self):
        provenance = c.provenance([runtime.__file__])
        self.assertTrue(provenance['detector_import_path'].endswith('src/limo_maze_perception/limo_maze_perception/core.py'))
        self.assertEqual(len(provenance['sources']), 5)
        self.assertFalse(any('worlds' in p or 'truth' in p for p in provenance['sources']))
    def test_conflict_after_sample_write_fails_summary_without_rewriting_sample(self):
        window = c.FixedWindow(2)
        window.on_clock(9_000_000_000)
        writer = c.CaptureWriter(self.root/'late-conflict')
        def collect_save(ns):
            for key,msg in fixture(ns).items(): window.receive(key,msg,1.)
            sample=window.pending[-1]
            messages=window.messages(sample)
            report,arrays=c.evaluate(messages,transforms(ns),registered=True)
            path=writer.save_sample(sample['sequence'],messages,report,arrays)
            window.completed.append({'sequence':sample['sequence'],'stamp_ns':ns,
                                     'artifact':path,'frame_valid':report['frame_valid']})
            window.pending.remove(sample)
            return path
        path=collect_save(10_000_000_000)
        raw_before=(writer.root/path).read_bytes()
        for n in range(65):
            window.receive('rgb',fixture(10_100_000_000+n)['rgb'],2.+n)
        self.assertEqual(len(window.events),40)  # association must survive event truncation
        altered=fixture()['rgb']
        altered.data=bytes([1])+altered.data[1:]
        window.receive('rgb',altered,70.)
        collect_save(11_000_000_000)
        integrity=runtime.end_of_window_integrity(window)
        sampling={'sampling_complete':True,'collection_complete':True,
                  'samples':list(window.completed),**integrity}
        sampling['sampling_passed']=integrity['all_samples_valid'] and not integrity['errors']
        result=runtime.checkpoint_then_cleanup(writer,sampling,[],runtime.StopRequest())
        self.assertTrue(result['sampling_complete'])
        self.assertTrue(result['cleanup_complete'])
        self.assertTrue(result['all_samples_valid_when_saved'])
        self.assertFalse(result['all_samples_valid'])
        self.assertFalse(result['passed'])
        self.assertEqual(result['selected_sample_conflicts'][0]['artifact'],path)
        self.assertTrue(result['selected_sample_conflicts'][0]['sample_written'])
        self.assertEqual(result['selected_sample_conflicts'][0]['stamp_ns'],10_000_000_000)
        self.assertEqual((writer.root/path).read_bytes(),raw_before)
        self.assertTrue(json.loads(raw_before)['frame_valid'])
        raw_checkpoint=json.loads((writer.root/'raw-checkpoint.json').read_text())
        self.assertEqual(raw_checkpoint['selected_sample_conflicts'],result['selected_sample_conflicts'])
        self.assertFalse(raw_checkpoint['sampling_passed'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
