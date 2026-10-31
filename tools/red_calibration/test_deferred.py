"""Offline fixed-window phase tests; no ROS imports or simulated runtime."""
import copy
import json
from pathlib import Path
import signal
import tempfile
import time
import unittest
from unittest.mock import patch

import capture_core as c
import capture_red_samples as runtime
from test_capture import fixture, transforms


class DeferredTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='deferred-test-', dir=Path(__file__).parent)
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.stop = runtime.StopRequest()
        self.manifest = {'capture_session': 'test', 'case_id': 'test'}

    def window(self, count=1):
        window = c.FixedWindow(count)
        window.on_clock(9_000_000_000)
        return window

    def add(self, window, ns=10_000_000_000, kinds=tuple(c.TOPICS)):
        data = fixture(ns)
        for kind in kinds:
            window.receive(kind, data[kind], 1.)
        window.on_clock(ns+100_000_000)
        return data

    def freeze(self, window, now=1.1, force=False, lookup=None):
        runtime.freeze_ready(window, now=now, clock_wall=now, started=0., deadline=10.,
            registered=True, lookup=lookup or (lambda stamp, optical: (transforms(stamp), {})),
            force=force, stop_signal=self.stop.signal)

    def close(self, window):
        self.freeze(window, force=True)
        window.closed = True
        window.collection_boundary = {'closed_wall_offset_s': 1.1,
            'last_callback_clock_stamp_ns': window.clock_ns,
            'post_close_callbacks_or_queued_messages_included': False}

    def persist(self, window, writer, **kwargs):
        return runtime.persist_frozen(writer, window, self.manifest, registered=True,
            reference='odom', base='base_footprint', stop=self.stop, save_timeout=10., **kwargs)

    def test_collect_six_never_runs_processing_png_fsync_or_slow_writer(self):
        window = self.window(6)
        with patch.object(runtime, 'evaluate', side_effect=AssertionError('detector in acquisition')), \
             patch.object(c, 'png_bytes', side_effect=AssertionError('PNG in acquisition')), \
             patch.object(c.os, 'fsync', side_effect=AssertionError('fsync in acquisition')), \
             patch.object(c.CaptureWriter, 'save_sample', side_effect=AssertionError('writer in acquisition')):
            for n in range(6):
                self.add(window, 10_000_000_000+n*100_000_000)
                self.freeze(window)
            self.assertEqual(len(window.pending), 6)
            self.assertTrue(all('snapshot' in s for s in window.pending))
        self.close(window)
        writer = c.CaptureWriter(self.root/'slow')
        actual = writer.save_sample
        def slow(*args, **kwargs):
            time.sleep(.02)
            return actual(*args, **kwargs)
        with patch.object(writer, 'save_sample', side_effect=slow):
            report = self.persist(window, writer)
        self.assertTrue(report['persistence_complete'])
        self.assertGreaterEqual(report['derived_save_elapsed_wall_s'], .12)
        self.assertEqual(window.collection_boundary['closed_wall_offset_s'], 1.1)
        self.assertEqual([s['sequence'] for s in window.completed], list(range(1, 7)))

    def test_persist_rejects_open_acquisition_before_writer_called(self):
        window = self.window()
        with self.assertRaisesRegex(ValueError, 'must_close'):
            self.persist(window, None)

    def test_missing_calibration_still_queries_exact_tf_and_is_not_valid(self):
        window = self.window()
        self.add(window, kinds=('rgb', 'depth'))
        calls = []
        def lookup(stamp, optical):
            calls.append((stamp, optical))
            return transforms(stamp), {}
        self.freeze(window, now=3.01, lookup=lookup)
        self.assertEqual(calls, [(10_000_000_000, 'camera_optical_frame')])
        self.close(window)
        writer = c.CaptureWriter(self.root/'missing-info')
        self.persist(window, writer)
        saved = json.loads((writer.root/'sample_001/sample.json').read_text())
        self.assertEqual(saved['tf_query_status'], 'attempted')
        self.assertEqual(saved['tf_reasons'], [])
        self.assertFalse(saved['frame_valid'])
        self.assertEqual(saved['processing_status'], 'not_run')

    def test_no_clock_coverage_reports_not_queried_not_transport_loss(self):
        window = self.window()
        self.add(window)
        window.clock_ns = 9_900_000_000
        self.close(window)
        writer = c.CaptureWriter(self.root/'no-clock')
        self.persist(window, writer)
        saved = json.loads((writer.root/'sample_001/sample.json').read_text())
        self.assertEqual(saved['tf_reasons'], ['tf_not_queried:no_clock_coverage'])
        self.assertFalse(saved['frame_valid'])

    def test_original_messages_are_detached_and_selected_slots_are_not_replaced(self):
        window = self.window(2)
        original = self.add(window)
        old = window.pending[0]['rgb'].data
        original['rgb'].data = b'altered'
        original['rgb_info'].k[0] = 999.
        self.freeze(window)
        self.add(window, 10_100_000_000, kinds=('rgb', 'depth'))
        self.freeze(window, now=3.01)
        self.add(window, 10_200_000_000)
        self.assertEqual([s['stamp_ns'] for s in window.pending], [10_000_000_000, 10_100_000_000])
        self.assertEqual(window.pending[0]['snapshot']['messages']['rgb'].data, old)
        self.assertEqual(window.pending[0]['snapshot']['messages']['rgb_info'].k[0], 100.)
        self.assertIsNone(window.pending[1]['snapshot']['messages']['rgb_info'])

    def test_late_conflict_until_close_fails_then_later_callbacks_are_excluded(self):
        window = self.window(2)
        original = self.add(window)
        self.freeze(window)
        bad = copy.deepcopy(original['rgb'])
        bad.data = bytes([7])+bad.data[1:]
        window.receive('rgb', bad, 1.5)
        self.add(window, 10_100_000_000)
        self.close(window)
        before = dict(window.counts)
        window.receive('rgb', bad, 4.)
        self.assertEqual(window.counts, before)
        writer = c.CaptureWriter(self.root/'conflict')
        self.persist(window, writer)
        integrity = runtime.end_of_window_integrity(window)
        self.assertTrue(integrity['all_samples_valid_when_saved'])
        self.assertFalse(integrity['all_samples_valid'])
        self.assertEqual(integrity['selected_sample_conflicts'][0]['sequence'], 1)

    def test_memory_limit_raises_without_replacing_selected_image(self):
        data = fixture()
        pair_bytes = len(data['rgb'].data)+len(data['depth'].data)
        window = c.FixedWindow(2, max_memory_bytes=pair_bytes)
        window.on_clock(9_000_000_000)
        self.add(window)
        # Duplicate checks still work at the budget, including conflicting repeats.
        window.receive('rgb', data['rgb'], 1.1)
        with self.assertRaisesRegex(MemoryError, 'memory_budget'):
            window.receive('rgb', fixture(10_100_000_000)['rgb'], 1.2)
        self.assertEqual(window.pending[0]['stamp_ns'], 10_000_000_000)
        self.assertEqual(window.image_bytes_held(), pair_bytes)

    def test_interrupt_preserves_partial_raw_and_skips_detector(self):
        window = self.window(6)
        self.add(window, kinds=('rgb', 'depth'))
        self.stop.request(signal.SIGTERM, None)
        self.close(window)
        writer = c.CaptureWriter(self.root/'interrupted')
        with patch.object(c.detector, 'process_red_frame', side_effect=AssertionError('must not process')):
            report = self.persist(window, writer)
        self.assertEqual(report['raw_saved_pairs'], 1)
        checkpoint = json.loads((writer.root/'collection-checkpoint.json').read_text())
        self.assertEqual(checkpoint['raw_saved_sequences'], [1])
        self.assertFalse(checkpoint['passed'])
        self.assertTrue((writer.root/'sample_001/rgb.data').is_file())
        self.assertFalse(window.completed[0]['frame_valid'])
        self.assertEqual(window.completed[0]['processing_status'], 'not_run')
        saved = json.loads((writer.root/'sample_001/sample.json').read_text())
        self.assertIn('offline_processing_skipped', saved['extra_reasons'])
        self.assertIn('stop_requested:SIGTERM', saved['extra_reasons'])

    def test_save_deadline_keeps_raw_checkpoint_without_processing_or_replacement(self):
        window = self.window(2)
        self.add(window)
        self.add(window, 10_100_000_000)
        self.close(window)
        writer = c.CaptureWriter(self.root/'save-deadline')
        clock = [0.]
        actual = writer.save_raw
        def slow_raw(*args):
            result = actual(*args)
            clock[0] = 2.
            return result
        with patch.object(writer, 'save_raw', side_effect=slow_raw), \
             patch.object(runtime, 'evaluate', side_effect=AssertionError('deadline exceeded')):
            report = runtime.persist_frozen(writer, window, self.manifest, registered=True,
                reference='odom', base='base_footprint', stop=self.stop, save_timeout=1.,
                monotonic=lambda: clock[0])
        self.assertFalse(report['persistence_complete'])
        self.assertEqual(report['raw_saved_pairs'], 1)
        checkpoint = json.loads((writer.root/'collection-checkpoint.json').read_text())
        self.assertEqual(checkpoint['not_raw_committed_sequences'], [2])
        self.assertFalse(window.completed)
        self.assertEqual([s['sequence'] for s in window.pending], [1, 2])

    def test_processing_phase_failure_is_partial_even_if_collection_complete(self):
        from test_cleanup import sample_report
        writer = c.CaptureWriter(self.root/'partial-offline')
        sampling = sample_report(True)
        sampling.update(persistence_complete=False, sampling_passed=False)
        result = runtime.checkpoint_then_cleanup(writer, sampling, [], self.stop)
        self.assertTrue(result['collection_complete'])
        self.assertTrue(result['partial'])
        self.assertFalse(result['passed'])

    def test_evaluate_exhausting_budget_does_not_start_png_or_derived_save(self):
        window = self.window()
        self.add(window)
        self.close(window)
        writer = c.CaptureWriter(self.root/'evaluate-deadline')
        clock = [0.]
        actual = runtime.evaluate
        def slow_evaluate(*args, **kwargs):
            result = actual(*args, **kwargs)
            clock[0] = 2.
            return result
        with patch.object(runtime, 'evaluate', side_effect=slow_evaluate), \
             patch.object(writer, 'save_sample', side_effect=AssertionError('known deadline passed')):
            report = runtime.persist_frozen(writer, window, self.manifest, registered=True,
                reference='odom', base='base_footprint', stop=self.stop, save_timeout=1.,
                monotonic=lambda: clock[0])
        self.assertFalse(report['persistence_complete'])
        self.assertEqual(report['raw_saved_pairs'], 1)
        self.assertIn('offline_processing_wall_deadline', report['errors'])
        self.assertFalse(window.completed)
        self.assertTrue((writer.root/'collection-checkpoint.json').is_file())
        self.assertFalse((writer.root/'sample_001/sample.json').exists())

    def test_raw_checkpoint_exists_before_detector_exception(self):
        window = self.window()
        self.add(window)
        self.close(window)
        writer = c.CaptureWriter(self.root/'detector-abort')
        def fail(*args, **kwargs):
            checkpoint = json.loads((writer.root/'collection-checkpoint.json').read_text())
            self.assertEqual(checkpoint['raw_saved_sequences'], [1])
            raise RuntimeError('offline detector failure')
        with patch.object(runtime, 'evaluate', side_effect=fail):
            with self.assertRaisesRegex(RuntimeError, 'detector failure'):
                self.persist(window, writer)
        self.assertTrue((writer.root/'sample_001/depth.data').is_file())
        self.assertFalse((writer.root/'sample_001/sample.json').exists())


if __name__ == '__main__':
    unittest.main(verbosity=2)
