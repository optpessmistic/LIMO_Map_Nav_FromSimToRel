"""Small real writer/checkpoint artifacts; no ROS, scene truth or simulation."""
from array import array
import copy
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

from tools.visibility import retained
from tools.audit_frame_retention import audit
from limo_evidence_protocol.retention import Core, canonical, sha, sample_identity, write_job


def _write_json(path, value):
    path.write_text(json.dumps(value, allow_nan=False), encoding='utf-8')


def _sample(sequence, encoding, bigendian):
    stamp, frame = 10_000_000_000 + sequence * 1_000_000, 'camera_optical'
    calibration = {'width': 2, 'height': 2, 'distortion_model': 'plumb_bob',
                   'k': [2., 0., 1., 0., 2., 1., 0., 0., 1.], 'd': [],
                   'r': [1., 0., 0., 0., 1., 0., 0., 0., 1.],
                   'p': [2., 0., 1., 0., 0., 2., 1., 0., 0., 0., 1., 0.],
                   'binning_x': 0, 'binning_y': 0,
                   'roi': {'x_offset': 0, 'y_offset': 0, 'height': 0, 'width': 0, 'do_rectify': False}}
    camera = {'stamp_ns': stamp, 'frame_id': frame, 'calibration': calibration,
              'calibration_id': sha(canonical({'frame_id': frame, **calibration}))}
    config = {'minimum_depth_m': .1, 'maximum_depth_m': 4.0}
    record = {'schema_version': 1, 'source': 'red_rgbd_sim_baseline', 'map_epoch': 'epoch-a',
              'detector_session_id': 'session-a', 'sequence': sequence, 'status': 'processed', 'reason': None,
              'detector_config': config, 'detector_config_id': sha(canonical(config)), 'camera_info': camera,
              'callback_stamp_ns': stamp + 10, 'processed_stamp_ns': stamp + 100,
              'processing': {'detection_count': 0}}
    values = ((1.25, math.nan), (math.inf, 2.75)) if encoding == '32FC1' else ((1250, 0), (65535, 2750))
    code, size = ('f', 4) if encoding == '32FC1' else ('H', 2)
    depth = b''.join(struct.pack(('>' if bigendian else '<') + '2' + code, *row) + b'PAD' for row in values)
    rgb = b'\xff\x00\x00\x00\x00\x00XY' * 2
    for name, raw, image_encoding, step in (('rgb', rgb, 'rgb8', 8), ('depth', depth, encoding, 2 * size + 3)):
        record[name] = {'stamp_ns': stamp, 'frame_id': frame, 'width': 2, 'height': 2, 'step': step,
                        'encoding': image_encoding, 'is_bigendian': int(bigendian),
                        'data_bytes': len(raw), 'data_sha256': sha(raw)}
    return record, rgb, depth


class RetainedTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()
        (self.root / 'mission').mkdir()
        (self.root / 'retained').mkdir()
        self.mission_name = 'mission_fixed.json'

    def fixture(self, layouts=(('32FC1', False), ('32FC1', True), ('16UC1', True)), mutate=None):
        core = Core('epoch-a', 'session-a')
        accepted = []
        for i, (encoding, endian) in enumerate(layouts, 1):
            record, rgb, depth = _sample(i, encoding, endian)
            if mutate:
                mutate(record)
            core.offer(record, rgb, depth, float(i))
            identity = sample_identity(record)
            request = {'schema_version': 1, 'retention_id': 'view-' + str(i), 'map_epoch': 'epoch-a',
                       'goal_token': 7, 'inspection_request_id': 'goal-7', 'after_stamp_ns': 10_000_000_000,
                       'accepted_ordinal': i, 'sample': identity,
                       'tf': {'stamp_ns': identity['rgb_stamp_ns'], 'base_pose': [1., 2., 0.],
                              'camera_pose': [1., 2., .18, 0., 0., 0., 1.]}}
            core.request(request, float(i))
            job = core.take(float(i))
            result = core.finish(job, write_job(job, self.root / 'retained'), float(i))
            accepted.append({'request': request, 'request_sha256': sha(canonical(request)), 'state': 'stored',
                             'result': {**result, 'checkpoint_persisted': True}})
        core.close(float(len(layouts) + 1))
        self.checkpoint = {'closed': True, 'fault': None, 'checkpoint_failed': False, 'writer': None,
                           'core': core.checkpoint()}
        _write_json(self.root / 'retained/checkpoint.json', self.checkpoint)
        binding = {k: accepted[0]['request'][k] for k in
                   ('map_epoch', 'goal_token', 'inspection_request_id', 'after_stamp_ns')}
        self.mission = {'schema_version': 1, 'map_epoch': 'epoch-a', 'frame_id': 'map', 'status': 'done',
                        'mission_success': False, 'map_complete': True, 'visual_coverage_complete': False,
                        'targets': [{'must_not_escape': True}], 'target_count_prior': 999,
                        'unconfirmed_target_candidates': ['do not expose'],
                        'frame_retention': {'enabled': True, 'fault': None, 'windows': [
                            {'accepted': accepted, 'ledger_accepted_count': len(accepted), 'binding': binding,
                             'ids': [e['request']['retention_id'] for e in accepted], 'state': 'retired',
                             'reason': 'inspection_window_reset'}]}}
        self.save_mission()
        self.original = audit(self.mission, self.root / 'retained')
        self.assertTrue(self.original['passed'], self.original)
        self.save_original()
        self.final = {'schema_version': 1, 'passed': True, 'errors': [], 'snapshot': {
            'valid': True, 'errors': [], 'epoch': {'raw': 'epoch-a'}, 'map': {
                'frame_id': 'map', 'stamp_ns': 20_000_000_000, 'width': 2, 'height': 2, 'resolution': .05,
                'origin': {'xyz': [0., 0., 0.], 'quaternion_xyzw': [0., 0., 0., 1.], 'valid_geometry': True},
                'data': [0, -1, 100, 0]}}}
        _write_json(self.root / 'final-save.json', self.final)
        return accepted

    def save_mission(self):
        _write_json(self.root / 'mission' / self.mission_name, self.mission)

    def save_original(self):
        _write_json(self.root / 'retention-audit.json', self.original)

    def load(self, **kwargs):
        return retained.load_run(self.root, self.mission_name, **kwargs)

    def test_padding_endian_nan_units_retired_and_no_truth_exposure(self):
        self.fixture()
        loaded = self.load()
        self.assertEqual(len(loaded['frames']), 3)
        self.assertEqual(loaded['excluded'], [])
        for frame in loaded['frames'][:2]:
            rows = frame['depth']
            self.assertTrue(all(isinstance(row, array) and row.typecode == 'f' for row in rows))
            self.assertEqual(rows[0][0], 1.25)
            self.assertTrue(math.isnan(rows[0][1]))
            self.assertTrue(math.isinf(rows[1][0]))
            self.assertEqual(rows[1][1], 2.75)
        depth = loaded['frames'][2]['depth']
        self.assertTrue(all(row.typecode == 'H' for row in depth))
        self.assertEqual([list(row) for row in depth], [[1250, 0], [65535, 2750]])
        self.assertEqual(loaded['mission']['frame_retention']['windows'][0]['state'], 'retired')
        self.assertFalse(loaded['mission']['mission_success'])
        self.assertNotIn('targets', loaded['mission'])
        self.assertNotIn('target_count_prior', loaded['mission'])
        self.assertNotIn('unconfirmed_target_candidates', loaded['mission'])
        self.assertEqual(loaded['map_snapshot']['data'], [0, -1, 100, 0])
        first = loaded['frames'][0]
        source = Path(first['source_files']['depth.bin'])
        self.assertEqual(source, self.root / 'retained' / first['request_sha256'] / 'depth.bin')
        self.assertEqual(loaded['inputs_hashes'][source.relative_to(self.root).as_posix()]['sha256'],
                         first['producer']['depth']['data_sha256'])

    def test_original_rejection_cannot_be_revived_by_current_success(self):
        self.fixture()
        self.original['frames'][1].update(verified=False, error='consumer_receipt_not_confirmed')
        self.original.update(passed=False, verified_count=2)
        self.save_original()
        loaded = self.load()
        self.assertTrue(loaded['audit_current']['passed'])
        self.assertEqual([frame['retention_id'] for frame in loaded['frames']], ['view-1', 'view-3'])
        self.assertEqual(loaded['excluded'][0]['reason'], 'original_audit_rejected')

    def test_little_endian_integer_depth_stays_in_millimetres(self):
        self.fixture(layouts=(('16UC1', False),))
        self.assertEqual([list(row) for row in self.load()['frames'][0]['depth']],
                         [[1250, 0], [65535, 2750]])

    def test_current_hash_rejection_excludes_original_verified_frame(self):
        entries = self.fixture()
        path = self.root / 'retained' / entries[0]['request_sha256'] / 'depth.bin'
        raw = bytearray(path.read_bytes())
        raw[-1] ^= 1  # Padding is audited even though it never enters depth rows.
        path.write_bytes(raw)
        loaded = self.load()
        self.assertEqual(len(loaded['frames']), 2)
        self.assertEqual(loaded['excluded'][0]['reason'], 'current_audit_rejected')
        self.assertIn('file_hash:depth.bin', loaded['excluded'][0]['current_error'])

    def test_missing_current_file_is_excluded_and_fingerprinted_missing(self):
        entries = self.fixture()
        relative = Path('retained') / entries[0]['request_sha256'] / 'rgb.bin'
        (self.root / relative).unlink()
        loaded = self.load()
        self.assertEqual(len(loaded['frames']), 2)
        self.assertEqual(loaded['inputs_hashes'][relative.as_posix()], {'sha256': None, 'bytes': None})

    def test_original_global_errors_reject_even_when_frames_pass(self):
        self.fixture()
        self.original['errors'] = ['accepted_and_producer_request_sets_differ']
        self.save_original()
        with self.assertRaisesRegex(ValueError, 'original_global_errors'):
            self.load()

    def test_current_global_error_rejects(self):
        self.fixture()
        self.checkpoint['closed'] = False
        _write_json(self.root / 'retained/checkpoint.json', self.checkpoint)
        with self.assertRaisesRegex(ValueError, 'current_global_errors'):
            self.load()

    def test_missing_or_mismatched_original_identity_rejects(self):
        self.fixture()
        for defect in ('missing', 'digest', 'duplicate'):
            with self.subTest(defect=defect):
                old = copy.deepcopy(self.original)
                if defect == 'missing':
                    self.original['frames'].pop()
                elif defect == 'digest':
                    self.original['frames'][0]['request_sha256'] = 'b' * 64
                else:
                    self.original['frames'][1] = copy.deepcopy(self.original['frames'][0])
                self.save_original()
                with self.assertRaisesRegex(ValueError, 'audit_identity'):
                    self.load()
                self.original = old
        self.save_original()

    def test_epoch_mismatch_rejects(self):
        self.fixture()
        self.final['snapshot']['epoch']['raw'] = 'another-epoch'
        _write_json(self.root / 'final-save.json', self.final)
        with self.assertRaisesRegex(ValueError, 'final_map_epoch_mismatch'):
            self.load()
        self.mission['map_epoch'] = 'another-epoch'
        self.save_mission()
        with self.assertRaisesRegex(ValueError, 'accepted_epoch_mismatch'):
            self.load()

    def test_session_mismatch_rejects_globally(self):
        self.fixture()
        self.checkpoint['core']['detector_session_id'] = 'other-session'
        _write_json(self.root / 'retained/checkpoint.json', self.checkpoint)
        with self.assertRaisesRegex(ValueError, 'checkpoint_session_mismatch'):
            self.load()

    def test_frame_and_decoded_byte_budgets_refuse_not_truncate(self):
        self.fixture()
        with self.assertRaisesRegex(ValueError, 'frame_count_budget'):
            self.load(max_frames=2)
        with patch.object(retained, 'MAX_DECODED_BYTES', 20):
            with self.assertRaisesRegex(ValueError, 'decoded_byte_budget'):
                self.load()

    def test_input_and_individual_byte_budgets(self):
        self.fixture()
        with patch.object(retained, 'MAX_INPUT_BYTES', 1):
            with self.assertRaisesRegex(ValueError, 'input_byte_budget'):
                self.load()
        with patch.object(retained, 'MAX_IMAGE_BYTES', 4):
            with self.assertRaisesRegex(ValueError, 'oversize_input'):
                self.load()

    def test_bound_depth_and_rgb_stamps_even_if_byte_audit_passes(self):
        self.fixture(layouts=(('32FC1', False),), mutate=lambda record: record['depth'].update(stamp_ns=10_002_000_000))
        with self.assertRaisesRegex(ValueError, 'rgb_depth_camera_tf_binding'):
            self.load()

    def test_bound_depth_frame_even_if_byte_audit_passes(self):
        self.fixture(layouts=(('32FC1', False),), mutate=lambda record: record['depth'].update(frame_id='different_optical'))
        with self.assertRaisesRegex(ValueError, 'rgb_depth_camera_tf_binding'):
            self.load()

    def test_bound_camera_stamp_even_if_byte_audit_passes(self):
        self.fixture(layouts=(('32FC1', False),), mutate=lambda record: record['camera_info'].update(stamp_ns=1))
        with self.assertRaisesRegex(ValueError, 'rgb_depth_camera_tf_binding'):
            self.load()

    def test_camera_resolution_binding_even_if_byte_audit_passes(self):
        def mutate(record):
            camera = record['camera_info']
            camera['calibration']['width'] = 3
            camera['calibration_id'] = sha(canonical({'frame_id': camera['frame_id'], **camera['calibration']}))
        self.fixture(layouts=(('32FC1', False),), mutate=mutate)
        with self.assertRaisesRegex(ValueError, 'rgb_depth_camera_tf_binding'):
            self.load()

    def test_tf_must_be_exact_sample_stamp(self):
        self.fixture()
        entry = self.mission['frame_retention']['windows'][0]['accepted'][0]
        entry['request']['tf']['stamp_ns'] += 1
        entry['request_sha256'] = sha(canonical(entry['request']))
        self.save_mission()
        with self.assertRaisesRegex(ValueError, 'exact_sample_tf_required'):
            self.load()

    def test_malformed_mission_and_duplicate_json_are_rejected(self):
        self.fixture()
        self.mission['frame_retention'] = None
        self.save_mission()
        with self.assertRaisesRegex(ValueError, 'invalid_or_unreadable_evidence'):
            self.load()
        (self.root / 'mission' / self.mission_name).write_text('{"schema_version": 1, "schema_version": 1}')
        with self.assertRaisesRegex(ValueError, 'duplicate_json_key'):
            self.load()

    def test_mission_name_cannot_escape(self):
        for name in ('../mission.json', '..\\mission.json', 'C:mission.json', '/tmp/mission.json'):
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, 'mission_basename'):
                retained.load_run(self.root, name)

    def test_symlink_frame_file_rejected(self):
        entries = self.fixture()
        path = self.root / 'retained' / entries[0]['request_sha256'] / 'rgb.bin'
        other = self.root / 'retained' / entries[1]['request_sha256'] / 'rgb.bin'
        path.unlink()
        try:
            path.symlink_to(other)
        except OSError as error:
            self.skipTest('symlink creation unavailable: ' + str(error))
        with self.assertRaisesRegex(ValueError, 'unsafe_path'):
            self.load()

    def test_concurrent_replacement_after_current_audit_rejects(self):
        entries = self.fixture()
        path = self.root / 'retained' / entries[0]['request_sha256'] / 'depth.bin'
        real_audit = retained.audit
        def changed(consumer, root):
            result = real_audit(consumer, root)
            raw = bytearray(path.read_bytes())
            raw[0] ^= 1
            path.write_bytes(raw)
            return result
        with patch.object(retained, 'audit', changed):
            with self.assertRaisesRegex(ValueError, 'input_changed'):
                self.load()

    def test_concurrent_original_audit_mutation_rejected_at_final_rehash(self):
        self.fixture()
        real_decode = retained._decode
        def changed(*args):
            result = real_decode(*args)
            self.original['scope'] = 'changed during decoding'
            self.save_original()
            return result
        with patch.object(retained, '_decode', changed):
            with self.assertRaisesRegex(ValueError, 'input_changed'):
                self.load()


if __name__ == '__main__':
    unittest.main()
