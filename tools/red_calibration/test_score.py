"""Synthetic on-disk captures; no ROS/Gazebo, subprocesses or live experiment reads."""
from contextlib import redirect_stdout, redirect_stderr
from dataclasses import asdict
import io
import json
import shutil
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import score

# Reuse this checkout's capture encoder/schema to make realistic test artifacts.
CAPTURE = Path(__file__).resolve().parent
sys.path.insert(0, str(CAPTURE))
import capture_core as capture
import capture_red_samples as runtime
from test_capture import fixture as images, transforms


def put(path, value):
    path.write_text(json.dumps(value, indent=2)+'\n')


class ScoreTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.case = self.root/'closed-case'
        self.case.mkdir()
        self.fixture = self.root/'fixture'
        self.fixture.mkdir()
        self.manifest = self.fixture/'manifest.json'

    def build(self, kind='blank', target_count=0, red=False, count=6, bad_tf=None, late_conflict=False):
        case_id, session = 'test_control', 'synthetic-capture-session'
        world = b'<sdf version="1.9"><world name="test_control"/></sdf>'
        (self.fixture/'world.sdf').write_bytes(world)
        world_hash = score.digest(world)
        truth = {'schema_version': 1, 'role': 'simulator_and_offline_evaluator_only',
                 'case': {'id': case_id, 'kind': kind}, 'world_sha256': world_hash,
                 'target_count_offline_only': target_count,
                 'targets_ground_truth': [{'name': 'box_'+str(i)} for i in range(target_count)],
                 'label_scope': 'Synthetic control labels only'}
        put(self.fixture/'truth.json', truth)
        put(self.manifest, {'schema_version': 1, 'cases': [
            {'id': case_id, 'world': 'world.sdf', 'world_sha256': world_hash,
             'offline_truth': 'truth.json', 'truth_sha256': score.digest((self.fixture/'truth.json').read_bytes())}]})
        writer = capture.CaptureWriter(self.case/'capture')
        config = asdict(capture.detector.DetectorConfig())
        sources = capture.provenance([runtime.__file__])
        manifest = {'schema_version': 1, 'case_id': case_id, 'capture_session': session,
                    'requested_pairs': 6, 'reference_frame': 'odom', 'base_frame': 'base_footprint',
                    'detector_config': config, 'source': sources}
        writer.write('manifest.json', capture.json_bytes(manifest))
        for path in sources['sources']:
            writer.write('source/'+Path(path).name, Path(path).read_bytes())
        samples = []
        for sequence in range(1, count+1):
            stamp = 10_000_000_000+sequence*100_000_000
            messages, tf = images(stamp, red=red), transforms(stamp)
            if sequence == bad_tf:
                tf['reference_to_camera']['returned_stamp_ns'] = 0
            report, arrays = capture.evaluate(messages, tf, registered=True)
            report.update(case_id=case_id, capture_session=session)
            artifact = writer.save_sample(sequence, messages, report, arrays)
            samples.append({'sequence': sequence, 'stamp_ns': stamp, 'artifact': artifact,
                            'frame_valid': report['frame_valid'], 'reasons': report['reasons'],
                            'processing_status': report['processing_status'],
                            'detection_count': len(report['detections']) if report['detections'] is not None else None})
        valid = all(item['frame_valid'] for item in samples) and not late_conflict
        errors = ['conflicting_duplicate_input_observed'] if late_conflict else []
        sampling = {'schema_version': 2, 'capture_session': session, 'case_id': case_id,
                    'samples': samples, 'sampling_complete': count == 6, 'collection_complete': count == 6,
                    'requested_pairs': 6, 'saved_pairs': count, 'selected_pairs': count,
                    'source_unchanged': True, 'all_samples_valid': valid,
                    'all_samples_valid_when_saved': all(item['frame_valid'] for item in samples),
                    'errors': errors, 'sampling_passed': count == 6 and valid,
                    'conflicting_duplicate_count': int(late_conflict),
                    'selected_sample_conflicts': ([{'sequence': 1, 'stamp_ns': samples[0]['stamp_ns'],
                                                   'artifact': samples[0]['artifact'], 'topics': {'rgb': {}}}]
                                                  if late_conflict else [])}
        summary = runtime.checkpoint_then_cleanup(writer, sampling, [], runtime.StopRequest())
        result = {'status': 'captured' if summary['passed'] else 'failed_or_incomplete',
                  'case_id': case_id, 'world_sha256': world_hash, 'all_owned_groups_empty': True,
                  'commands': {'capture': ['synthetic-no-process']},
                  'stops': {'capture': {'remaining_check_complete': True, 'remaining': [], 'ownership_errors': []}},
                  'capture_passed': summary['passed'], 'exit_codes': {'capture': 0 if summary['passed'] else 1},
                  'source_unchanged': True, 'forced_kill_used': False, 'errors': []}
        put(self.case/'result.json', result)
        return summary

    def audit(self):
        return score.score_case(self.case, self.manifest)

    def build_deferred(self, *, reject_sequence=None, early_only=False, count=6):
        """Real schema3 pure writer, including its pre-processing durability boundary."""
        self.build(count=0)
        root = self.case/'capture'
        manifest = json.loads((root/'manifest.json').read_text())
        root.rename(self.case/'legacy_fixture_not_audit_input')
        writer = capture.CaptureWriter(root)
        writer.write('manifest.json', capture.json_bytes(manifest))
        for path in manifest['source']['sources']:
            writer.write('source/'+Path(path).name, Path(path).read_bytes())
        window = capture.FixedWindow(6)
        window.on_clock(9_000_000_000)
        for sequence in range(1, count+1):
            stamp = 10_000_000_000+sequence*100_000_000
            for kind, msg in images(stamp, red=False).items():
                if sequence == reject_sequence and kind.endswith('_info'):
                    continue
                window.receive(kind, msg, 1.+sequence*.1)
            window.on_clock(stamp+50_000_000)
            runtime.freeze_ready(window, now=1.+sequence*.1, clock_wall=1.+sequence*.1,
                started=0., deadline=10., registered=True,
                lookup=lambda stamp, optical: (transforms(stamp), {}))
        runtime.freeze_ready(window, now=1.7, clock_wall=1.7, started=0., deadline=10.,
            registered=True, lookup=lambda stamp, optical: (transforms(stamp), {}), force=True)
        window.closed = True
        window.collection_boundary = {'closed_wall_offset_s': 1.7,
            'selected_sequences': list(range(1, count+1)), 'last_callback_clock_stamp_ns': window.clock_ns,
            'post_close_callbacks_or_queued_messages_included': False}
        if early_only:
            with patch.object(runtime, 'evaluate', side_effect=RuntimeError('offline interruption')):
                with self.assertRaisesRegex(RuntimeError, 'offline interruption'):
                    runtime.persist_frozen(writer, window, manifest, registered=True,
                        reference='odom', base='base_footprint', stop=runtime.StopRequest(), save_timeout=10.)
            passed = False
        else:
            persistence = runtime.persist_frozen(writer, window, manifest, registered=True,
                reference='odom', base='base_footprint', stop=runtime.StopRequest(), save_timeout=10.)
            integrity = runtime.end_of_window_integrity(window)
            errors = persistence.pop('errors')+integrity.pop('errors')
            sampling = {'schema_version': 3, 'capture_session': manifest['capture_session'],
                'case_id': manifest['case_id'], 'samples': list(window.completed),
                'sampling_complete': count == 6, 'collection_complete': count == 6, 'requested_pairs': 6,
                'saved_pairs': len(window.completed), 'selected_pairs': count,
                'source_unchanged': True, 'errors': errors, 'sampling_passed':
                len(window.completed) == 6 and integrity['all_samples_valid'] and not errors,
                'collection_boundary': dict(window.collection_boundary), **persistence, **integrity}
            summary = runtime.checkpoint_then_cleanup(writer, sampling, [], runtime.StopRequest())
            passed = summary['passed']
        result = json.loads((self.case/'result.json').read_text())
        result.update(status='captured' if passed else 'failed_or_incomplete', capture_passed=passed,
                      exit_codes={'capture': 0 if passed else 1}, errors=[] if passed else ['capture incomplete'])
        put(self.case/'result.json', result)

    def batch_of_twelve(self):
        directory = self.root/'batch'
        directory.mkdir()
        shutil.copytree(self.case, directory/'test_control')
        manifest = json.loads(self.manifest.read_text())
        original = manifest['cases'][0]
        for number in range(2,13):
            case_id = 'unrun_%02d' % number
            truth = json.loads((self.fixture/'truth.json').read_text())
            truth['case']['id'] = case_id
            path = self.fixture/(case_id+'.json')
            put(path, truth)
            manifest['cases'].append({**original, 'id': case_id, 'offline_truth': path.name,
                                      'truth_sha256': score.digest(path.read_bytes())})
        put(self.manifest, manifest)
        put(directory/'plan.json', {'cases': [{'id': item['id'], 'world_sha256': item['world_sha256']}
                                             for item in manifest['cases']],
                                     'manifest_sha256': score.digest(self.manifest.read_bytes())})
        put(directory/'batch.json', {'status': 'stopped_incomplete', 'completed_all_cases': False,
            'errors': ['Synthetic batch stopped after first case'],
            'cases': [json.loads((self.case/'result.json').read_text())]})
        return directory

    def test_blank_keeps_six_zero_observations_without_claiming_recognition(self):
        self.build()
        report = self.audit()
        self.assertTrue(report['eligible_for_control_count_comparison'])
        self.assertEqual(report['summary']['detection_count_by_sequence'], [0]*6)
        self.assertEqual(report['summary']['mask_red_pixels_by_sequence'], [0]*6)
        self.assertIsNone(report['recognition_success'])
        self.assertFalse(report['world_position_error_computed'])
        self.assertFalse(report['effective_range_claimed'])

    def test_positive_count_and_observed_mask_are_not_identity_or_accuracy(self):
        self.build(kind='single', target_count=1, red=True)
        report = self.audit()
        self.assertTrue(report['evidence_integrity_passed'])
        self.assertEqual(report['summary']['detection_count_by_sequence'], [1]*6)
        self.assertEqual(report['summary']['mask_red_pixels_by_sequence'], [400]*6)
        self.assertEqual(report['frames'][0]['mask']['bbox_xyxy_inclusive'], [30,20,49,39])
        self.assertEqual(report['frames'][0]['count_relation_to_declared_instances'], 'equal')
        self.assertFalse(report['recognition_success_evaluated'])

    def test_red_negative_control_detection_does_not_fail_data_integrity(self):
        self.build(kind='red_distractor', target_count=0, red=True)
        report = self.audit()
        self.assertTrue(report['eligible_for_control_count_comparison'])
        self.assertEqual(report['frames'][0]['count_relation_to_declared_instances'], 'more')
        self.assertEqual(report['control']['case']['kind'], 'red_distractor')
        self.assertIsNone(report['recognition_success'])

    def test_two_declared_instances_with_one_detection_is_count_relation_only(self):
        self.build(kind='pair', target_count=2, red=True)
        report = self.audit()
        self.assertEqual(report['frames'][0]['count_relation_to_declared_instances'], 'fewer')
        self.assertTrue(report['eligible_for_control_count_comparison'])

    def test_missing_sixth_frame_is_retained_not_replaced(self):
        self.build(count=5)
        report = self.audit()
        self.assertEqual(len(report['frames']), 6)
        self.assertEqual(report['summary']['detection_count_by_sequence'], [0,0,0,0,0,None])
        self.assertFalse(report['fixed_collection_complete'])
        self.assertFalse(report['eligible_for_control_count_comparison'])

    def test_invalid_tf_frame_still_has_original_observed_count(self):
        self.build(red=True, target_count=1, kind='single', bad_tf=3)
        report = self.audit()
        self.assertTrue(report['fixed_collection_complete'])
        self.assertEqual(report['summary']['detection_count_by_sequence'], [1]*6)
        self.assertEqual(report['summary']['input_valid_frames'], 5)
        self.assertFalse(report['frames'][2]['input_valid_at_collection_end'])
        self.assertIsNone(report['frames'][2]['count_relation_to_declared_instances'])

    def test_late_conflict_preserves_sample_and_vetoes_collection(self):
        self.build(late_conflict=True)
        path = self.case/'capture/sample_001/sample.json'
        before = path.read_bytes()
        report = self.audit()
        self.assertTrue(report['frames'][0]['input_valid_when_saved'])
        self.assertFalse(report['frames'][0]['input_valid_at_collection_end'])
        self.assertFalse(report['eligible_for_control_count_comparison'])
        self.assertEqual(path.read_bytes(), before)

    def test_raw_tampering_is_detected(self):
        self.build()
        path = self.case/'capture/sample_002/rgb.data'
        path.write_bytes(b'1'+path.read_bytes()[1:])
        report = self.audit()
        self.assertFalse(report['evidence_integrity_passed'])
        self.assertFalse(report['eligible_for_control_count_comparison'])
        self.assertTrue(any('Hash/size mismatch' in issue for issue in report['audit_issues']))

    def test_summary_cannot_erase_checkpoint_conflict(self):
        self.build(late_conflict=True)
        path = self.case/'capture/summary.json'
        summary = json.loads(path.read_text())
        summary.update(selected_sample_conflicts=[], conflicting_duplicate_count=0,
                       all_samples_valid=True, sampling_passed=True, passed=True, errors=[])
        put(path, summary)
        report = self.audit()
        self.assertFalse(report['evidence_integrity_passed'])
        self.assertIn('summary_changed_frozen_sampling_field:selected_sample_conflicts', report['audit_issues'])
        self.assertIn('summary_removed_checkpoint_error', report['audit_issues'])

    def test_duplicate_sequence_does_not_select_an_alternative_frame(self):
        self.build()
        path = self.case/'capture/summary.json'
        summary = json.loads(path.read_text())
        summary['samples'][2]['sequence'] = 1
        put(path, summary)
        report = self.audit()
        self.assertEqual(len(report['frames']), 6)
        self.assertEqual(report['frames'][0]['audit_issues'], ['duplicate_sequence'])
        self.assertEqual(report['frames'][2]['audit_issues'], ['missing_sequence'])
        self.assertIsNone(report['frames'][0]['detection_count'])
        self.assertFalse(report['eligible_for_control_count_comparison'])

    def test_case_must_be_closed_before_capture_is_read(self):
        self.build()
        result = json.loads((self.case/'result.json').read_text())
        result['status'] = 'running'
        put(self.case/'result.json', result)
        with self.assertRaisesRegex(ValueError, 'not recorded closed'):
            self.audit()

    def test_wrong_truth_world_binding_is_refused(self):
        self.build()
        result = json.loads((self.case/'result.json').read_text())
        result['world_sha256'] = '0'*64
        put(self.case/'result.json', result)
        with self.assertRaisesRegex(ValueError, 'identity mismatch'):
            self.audit()

    def test_truth_path_cannot_escape_fixture_directory(self):
        self.build()
        manifest = json.loads(self.manifest.read_text())
        manifest['cases'][0]['offline_truth'] = '../elsewhere.json'
        put(self.manifest, manifest)
        with self.assertRaisesRegex(ValueError, 'escapes root'):
            self.audit()

    def test_missing_summary_uses_checkpoint_as_ineligible_partial_evidence(self):
        self.build()
        (self.case/'capture/summary.json').unlink()
        report = self.audit()
        self.assertEqual(report['summary']['detection_count_by_sequence'], [0]*6)
        self.assertFalse(report['eligible_for_control_count_comparison'])
        self.assertIn('final_summary_missing_cleanup_unknown', report['audit_issues'])

    def test_cli_writes_only_new_output_outside_inputs_and_never_imports_ros(self):
        self.build()
        before = {str(path): score.digest(path.read_bytes()) for path in self.case.rglob('*') if path.is_file()}
        output = self.root/'scores/result.json'
        with redirect_stdout(io.StringIO()):
            code = score.main(['--case-dir',str(self.case),'--manifest',str(self.manifest),'--output',str(output)])
        self.assertEqual(code, 0)
        after = {str(path): score.digest(path.read_bytes()) for path in self.case.rglob('*') if path.is_file()}
        self.assertEqual(before, after)
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            score.main(['--case-dir',str(self.case),'--manifest',str(self.manifest),'--output',str(output)])
        self.assertNotIn('rclpy', sys.modules)

    def test_png_crc_or_nonbinary_mask_is_refused(self):
        import numpy as np
        data = capture.png_bytes(np.array([[0,255]], dtype=np.uint8))
        self.assertEqual(score.mask_metrics(data)['red_pixels'], 1)
        with self.assertRaisesRegex(ValueError, 'CRC'):
            score.mask_metrics(data[:-5]+bytes([data[-5]^1])+data[-4:])
        with self.assertRaisesRegex(ValueError, 'Nonbinary'):
            score.mask_metrics(capture.png_bytes(np.array([[1]], dtype=np.uint8)))

    def test_batch_preserves_twelve_case_denominator_and_null_unrun_slots(self):
        self.build()
        report = score.score_batch(self.batch_of_twelve(), self.manifest)
        self.assertEqual(report['planned_case_count'], 12)
        self.assertEqual(report['planned_frame_slots'], 72)
        self.assertEqual(report['ran_case_count'], 1)
        self.assertEqual(report['not_run_case_count'], 11)
        self.assertEqual(report['eligible_case_count'], 1)
        self.assertFalse(report['all_planned_cases_have_eligible_capture'])
        for case in report['cases'][1:]:
            self.assertEqual(case['execution_state'], 'not_run')
            self.assertEqual(case['summary']['detection_count_by_sequence'], [None]*6)
            self.assertIsNone(case['summary']['input_valid_frames'])

    def test_batch_invalid_run_is_neither_omitted_nor_called_unrun(self):
        self.build(bad_tf=2, red=True, target_count=1, kind='single')
        report = score.score_batch(self.batch_of_twelve(), self.manifest)
        self.assertEqual(report['ran_but_ineligible_case_count'], 1)
        self.assertEqual(report['eligible_case_count'], 0)
        self.assertEqual(report['cases'][0]['summary']['input_valid_frames'], 5)
        self.assertEqual(report['not_run_case_count'], 11)

    def test_batch_cannot_relabel_unlisted_partial_artifacts_as_not_run(self):
        self.build()
        directory = self.batch_of_twelve()
        (directory/'unrun_02').mkdir()
        with self.assertRaisesRegex(ValueError, 'Unlisted case artifacts'):
            score.score_batch(directory, self.manifest)

    def test_batch_running_status_is_refused(self):
        self.build()
        directory = self.batch_of_twelve()
        report = json.loads((directory/'batch.json').read_text())
        report['status'] = 'running'
        put(directory/'batch.json', report)
        with self.assertRaisesRegex(ValueError, 'not a recorded closed'):
            score.score_batch(directory, self.manifest)

    def test_real_schema3_deferred_normal_artifacts_remain_eligible(self):
        self.build_deferred()
        report = self.audit()
        self.assertTrue(report['eligible_for_control_count_comparison'])
        self.assertTrue(report['evidence_integrity_passed'])
        self.assertEqual(report['summary']['detection_count_by_sequence'], [0]*6)
        self.assertTrue(any(path.endswith('/collection-checkpoint.json') for path in report['sources']))
        self.assertTrue(any(path.endswith('/sample_006/raw.json') for path in report['sources']))

    def test_real_schema3_rejected_third_frame_is_null_not_zero_or_miss(self):
        self.build_deferred(reject_sequence=3)
        report = self.audit()
        self.assertEqual(report['summary']['detection_count_by_sequence'], [0,0,None,0,0,0])
        self.assertEqual(report['summary']['input_valid_frames'], 5)
        self.assertIsNone(report['frames'][2]['count_relation_to_declared_instances'])
        self.assertFalse(report['eligible_for_control_count_comparison'])

    def test_early_checkpoint_keeps_nulls_and_ignores_orphan_derived_files(self):
        self.build_deferred(early_only=True)
        for name in ('sample.json', 'red_mask.png'):
            (self.case/'capture/sample_001'/name).write_bytes(b'incomplete derivative')
        report = self.audit()
        self.assertEqual([row['sequence'] for row in report['frames']], list(range(1,7)))
        self.assertEqual(report['summary']['detection_count_by_sequence'], [None]*6)
        self.assertEqual(report['summary']['mask_red_pixels_by_sequence'], [None]*6)
        self.assertEqual(report['summary']['verified_raw_sequences'], list(range(1,7)))
        self.assertIsNone(report['summary']['input_valid_frames'])
        self.assertIsNone(report['summary']['frames_with_nonzero_detections'])
        self.assertIsNone(report['capture_cleanup_complete'])
        self.assertFalse(report['fixed_collection_complete'])
        self.assertFalse(report['evidence_integrity_passed'])
        self.assertFalse(report['eligible_for_control_count_comparison'])
        self.assertFalse(any(path.endswith('/sample.json') or path.endswith('/red_mask.png') for path in report['sources']))
        for row in report['frames']:
            self.assertIsNone(row['processing'])
            self.assertIsNone(row['input_valid_at_collection_end'])
            self.assertIsNone(row['count_relation_to_declared_instances'])

    def test_early_checkpoint_batch_keeps_ran_ineligible_and_planned_denominator(self):
        self.build_deferred(early_only=True)
        report = score.score_batch(self.batch_of_twelve(), self.manifest)
        self.assertEqual(report['planned_case_count'], 12)
        self.assertEqual(report['planned_frame_slots'], 72)
        self.assertEqual(report['ran_case_count'], 1)
        self.assertEqual(report['ran_but_ineligible_case_count'], 1)
        self.assertEqual(report['not_run_case_count'], 11)
        self.assertEqual(report['eligible_case_count'], 0)
        self.assertEqual(report['cases'][0]['execution_state'], 'ran')
        self.assertTrue(all(case['summary']['detection_count_by_sequence'] == [None]*6 for case in report['cases']))

    def test_early_partial_collection_never_fills_missing_sequence(self):
        self.build_deferred(early_only=True, count=5)
        report = self.audit()
        self.assertEqual(report['summary']['verified_raw_sequences'], list(range(1,6)))
        self.assertEqual(report['frames'][5]['status'], 'missing_raw')
        self.assertEqual(report['summary']['detection_count_by_sequence'], [None]*6)

    def test_early_tampered_raw_payload_is_not_verified(self):
        self.build_deferred(early_only=True)
        path = self.case/'capture/sample_002/rgb.data'
        path.write_bytes(b'1'+path.read_bytes()[1:])
        report = self.audit()
        self.assertEqual(report['summary']['verified_raw_sequences'], [1,3,4,5,6])
        self.assertTrue(any('Hash/size mismatch' in issue for issue in report['audit_issues']))
        self.assertEqual(report['summary']['detection_count_by_sequence'], [None]*6)

    def test_early_unreferenced_raw_json_is_never_read(self):
        self.build_deferred(early_only=True)
        path = self.case/'capture/collection-checkpoint.json'
        checkpoint = json.loads(path.read_text())
        del checkpoint['artifacts']['sample_003/raw.json']
        put(path, checkpoint)
        (self.case/'capture/sample_003/raw.json').write_bytes(b'invalid unreferenced JSON')
        report = self.audit()
        self.assertEqual(report['summary']['verified_raw_sequences'], [1,2,4,5,6])
        self.assertFalse(any(path.endswith('/sample_003/raw.json') for path in report['sources']))

    def test_early_does_not_bypass_outer_owned_cleanup(self):
        self.build_deferred(early_only=True)
        path = self.case/'result.json'
        result = json.loads(path.read_text())
        result['all_owned_groups_empty'] = False
        put(path, result)
        with self.assertRaisesRegex(ValueError, 'not recorded closed'):
            self.audit()

    def test_early_checkpoint_wrong_session_is_refused(self):
        self.build_deferred(early_only=True)
        path = self.case/'capture/collection-checkpoint.json'
        checkpoint = json.loads(path.read_text())
        checkpoint['capture_session'] = 'different-run'
        put(path, checkpoint)
        with self.assertRaisesRegex(ValueError, 'checkpoint identity'):
            self.audit()

    def test_existing_malformed_later_report_never_falls_back_to_early(self):
        self.build_deferred(early_only=True)
        (self.case/'capture/raw-checkpoint.json').write_bytes(b'{partial')
        with self.assertRaises(json.JSONDecodeError):
            self.audit()

    def test_early_checkpoint_cannot_remap_to_different_sequence_window(self):
        self.build_deferred(early_only=True)
        path = self.case/'capture/collection-checkpoint.json'
        checkpoint = json.loads(path.read_text())
        checkpoint['selected_sequences'] = [2,3,4,5,6]
        put(path, checkpoint)
        with self.assertRaisesRegex(ValueError, 'fixed original sequences'):
            self.audit()


if __name__ == '__main__':
    unittest.main(verbosity=2)
