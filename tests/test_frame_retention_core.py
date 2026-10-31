"""Pure state/filesystem tests; no ROS imports, processes, clocks, or simulator."""
import copy
import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/limo_evidence_protocol'))
from limo_evidence_protocol.retention import AcceptedWindow, Bundle, Core, Policy, canonical, sha, sample_identity, validate_request, write_job


def sample(seq=1):
    rgb, depth = bytearray(b'\xff\x00\x00\xa5'), bytearray(struct.pack('<f', 1.))
    calibration = {'width': 1, 'height': 1, 'distortion_model': 'plumb_bob',
                   'k': [1., 0., 0., 0., 1., 0., 0., 0., 1.], 'd': [],
                   'r': [1., 0., 0., 0., 1., 0., 0., 0., 1.], 'p': [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0.],
                   'binning_x': 0, 'binning_y': 0, 'roi': {'x_offset': 0, 'y_offset': 0, 'height': 0, 'width': 0, 'do_rectify': False}}
    frame = 'camera_optical'
    camera = {'stamp_ns': 10_000_000_000+seq*1_000_000, 'frame_id': frame,
              'calibration': calibration, 'calibration_id': sha(canonical({'frame_id': frame, **calibration}))}
    config = {'minimum_depth_m': .1, 'maximum_depth_m': 4.0}
    record = {'schema_version': 1, 'source': 'red_rgbd_sim_baseline', 'map_epoch': 'epoch-a',
              'detector_session_id': 'session-a', 'sequence': seq, 'status': 'processed', 'reason': None,
              'detector_config': config, 'detector_config_id': sha(canonical(config)), 'camera_info': camera,
              'callback_stamp_ns': camera['stamp_ns']+10, 'processed_stamp_ns': camera['stamp_ns']+100,
              'processing': {'detection_count': 0}}
    for name, data, encoding in (('rgb', rgb, 'rgb8'), ('depth', depth, '32FC1')):
        record[name] = {'stamp_ns': camera['stamp_ns'], 'frame_id': frame, 'width': 1, 'height': 1,
                        'step': 4, 'encoding': encoding, 'is_bigendian': 0,
                        'data_bytes': len(data), 'data_sha256': sha(data)}
    return record, rgb, depth


def request(record, ordinal=None, rid=None):
    identity = sample_identity(record)
    return {'schema_version': 1, 'retention_id': rid or 'view-a-'+str(record['sequence']),
            'map_epoch': record['map_epoch'], 'goal_token': 7, 'inspection_request_id': 'goal-7',
            'after_stamp_ns': 10_000_000_000, 'accepted_ordinal': ordinal or record['sequence'],
            'sample': identity, 'tf': {'stamp_ns': identity['rgb_stamp_ns'], 'base_pose': [1., 2., 0.],
                                    'camera_pose': [1.084, 2., .18, 0., 0., 0., 1.]}}


class RetentionTests(unittest.TestCase):
    def core(self, **policy):
        return Core('epoch-a', 'session-a', Policy(**policy))

    def offer(self, core, seq=1, now=0.):
        record, rgb, depth = sample(seq)
        self.assertTrue(core.offer(record, rgb, depth, now)['available'])
        return record

    def store(self, core, root, now=0.):
        job = core.take(now)
        self.assertIsNotNone(job)
        receipt = write_job(job, Path(root).resolve())
        return core.finish(job, receipt, now), job

    def test_round_trip_original_padding_calibration_producer_request_and_exact_tf(self):
        core = self.core()
        record = self.offer(core)
        req = request(record)
        self.assertEqual(core.request(req, 0.)['status'], 'queued')
        with tempfile.TemporaryDirectory() as root:
            result, job = self.store(core, root)
            self.assertEqual(result['status'], 'stored')
            folder = Path(root)/job.directory
            self.assertEqual((folder/'rgb.bin').read_bytes(), b'\xff\x00\x00\xa5')
            self.assertEqual(json.loads((folder/'producer.json').read_bytes()), record)
            self.assertEqual(json.loads((folder/'request.json').read_bytes()), req)
            manifest = json.loads((folder/'manifest.json').read_bytes())
            for name, meta in result['receipt']['files'].items():
                data = (folder/name).read_bytes()
                self.assertEqual((len(data), sha(data)), (meta['bytes'], meta['sha256']))
            self.assertEqual(manifest['sample'], req['sample'])
            self.assertEqual(sum(p.stat().st_size for p in folder.iterdir()), core.reserved_file_bytes)
        self.assertTrue(AcceptedWindow.freeze([req]).evaluate([result])['retention_satisfied'])

    def test_source_and_request_mutation_after_queue_cannot_change_job(self):
        core = self.core()
        record, rgb, depth = sample()
        core.offer(record, rgb, depth, 0.)
        req = request(record)
        original = copy.deepcopy(req)
        core.request(req, 0.)
        rgb[0] = 0; depth[0] = 255
        record['camera_info']['calibration']['k'][0] = 99.
        req['tf']['camera_pose'][0] = 999.
        job = core.take(0.)
        files = dict(job.files)
        self.assertEqual(files['rgb.bin'], b'\xff\x00\x00\xa5')
        self.assertEqual(json.loads(files['request.json']), original)
        self.assertEqual(json.loads(files['camera_info.json'])['calibration']['k'][0], 1.)

    def test_identical_request_is_idempotent_before_during_and_after_write(self):
        core = self.core(); req = request(self.offer(core))
        first = core.request(req, 0.)
        self.assertEqual(core.request(req, 0.), first)
        job = core.take(0.)
        self.assertEqual(core.request(req, 0.)['status'], 'writing')
        self.assertIsNone(core.take(0.))
        with tempfile.TemporaryDirectory() as root:
            result = core.finish(job, write_job(job, Path(root).resolve()), 0.)
            self.assertEqual(core.request(req, .1), result)
            self.assertEqual(len(list(Path(root).iterdir())), 1)
        self.assertEqual(core.jobs_reserved, 1)

    def test_request_id_different_payload_is_sticky_failure_and_does_not_replace_original(self):
        core = self.core(); req = request(self.offer(core)); core.request(req, 0.)
        changed = copy.deepcopy(req); changed['tf']['base_pose'][0] += .1
        self.assertEqual(core.request(changed, 0.)['reason'], 'request_id_conflict')
        self.assertEqual(core.request(req, 0.)['status'], 'failed')
        self.assertIsNone(core.take(0.))
        self.assertEqual(json.loads(core.records[req['retention_id']]['request']), req)

    def test_source_same_sequence_changed_bytes_poison_queued_and_stored_without_editing_files(self):
        for stored in (False, True):
            with self.subTest(stored=stored), tempfile.TemporaryDirectory() as root:
                core = self.core(); record = self.offer(core); req = request(record); core.request(req, 0.)
                before = None
                if stored:
                    result, job = self.store(core, root)
                    path = Path(root)/job.directory/'rgb.bin'; before = path.read_bytes()
                _, rgb, depth = sample(); rgb[0] = 7
                record['rgb']['data_sha256'] = sha(rgb)
                self.assertEqual(core.offer(record, rgb, depth, .1)['reason'], 'source_identity_conflict')
                self.assertEqual(core.result(req['retention_id'])['status'], 'failed')
                self.assertFalse(AcceptedWindow.freeze([req]).evaluate([core.result(req['retention_id'])])['retention_satisfied'])
                if stored: self.assertEqual(path.read_bytes(), before)
                else: self.assertIsNone(core.take(.1))

    def test_bad_raw_hash_config_calibration_or_layout_refused_before_cache(self):
        for defect in ('raw', 'config', 'calibration', 'layout'):
            with self.subTest(defect=defect):
                core = self.core(); record, rgb, depth = sample()
                if defect == 'raw': rgb[0] = 0
                elif defect == 'config': record['detector_config']['minimum_depth_m'] = .2
                elif defect == 'calibration': record['camera_info']['calibration']['k'][0] = 2.
                else: record['rgb']['height'] = 2
                with self.assertRaises(ValueError): core.offer(record, rgb, depth, 0.)
                self.assertEqual(core.candidate_bytes, 0)

    def test_ttl_miss_stays_missing_and_same_frame_cannot_refresh_ttl(self):
        core = self.core(ttl_s=1.)
        record = self.offer(core)
        _, rgb, depth = sample()
        self.assertEqual(core.offer(record, rgb, depth, .9)['reason'], 'duplicate')
        result = core.request(request(record), 1.)
        self.assertEqual(result['reason'], 'cache_miss')
        self.assertEqual(core.offer(record, rgb, depth, 1.)['reason'], 'nonadvancing_cache_input')
        self.assertEqual(core.request(request(record), 1.)['reason'], 'cache_miss')

    def test_capacity_eviction_only_unpinned_and_no_hidden_queue_bytes(self):
        core = self.core(max_candidates=1, request_timeout_s=100.)
        record = self.offer(core); core.request(request(record), 0.)
        r2, rgb2, depth2 = sample(2)
        self.assertEqual(core.offer(r2, rgb2, depth2, .1)['reason'], 'cache_budget_pinned')
        snapshot = core.checkpoint()
        self.assertLessEqual(snapshot['payload_bytes_reserved'], core.policy.max_payload_bytes)
        with tempfile.TemporaryDirectory() as root: self.store(core, root, .1)
        self.offer(core, 3, .2)
        self.assertEqual(len(core.cache), 1)
        self.assertEqual(core.request(request(r2), .2)['reason'], 'cache_miss')

    def test_byte_budget_is_actual_bytes_not_frame_count_and_includes_metadata(self):
        record, rgb, depth = sample()
        frozen = Bundle.freeze(record, rgb, depth, 100000)
        core = self.core(max_payload_bytes=frozen.size+10)
        self.assertTrue(core.offer(record, rgb, depth, 0.)['available'])
        self.assertEqual(core.request(request(record), 0.)['reason'], 'request_history_budget')
        self.assertEqual(core.checkpoint()['payload_bytes_reserved'], frozen.size)
        with self.assertRaises(ValueError): Bundle.freeze(record, rgb, depth, frozen.size-1)

    def test_queue_full_and_job_and_disk_caps_are_explicit_sticky_failures(self):
        for cap, reason in (({'max_pending': 1}, 'queue_full'), ({'max_jobs': 1}, 'job_count_budget'),
                            ({'max_file_bytes': 1}, 'file_byte_budget')):
            with self.subTest(reason=reason):
                core = self.core(**cap)
                a, b = self.offer(core), self.offer(core, 2)
                first = core.request(request(a), 0.)
                if reason == 'job_count_budget':
                    job = core.take(0.); core.finish(job, None, 0.)  # Failed writer still consumes reservation.
                result = core.request(request(b), 0.) if reason != 'file_byte_budget' else first
                self.assertEqual(result['reason'], reason)
                self.assertNotEqual(result['status'], 'stored')
                self.assertLessEqual(core.jobs_reserved, core.policy.max_jobs)
                self.assertLessEqual(core.reserved_file_bytes, core.policy.max_file_bytes)

    def test_history_and_candidate_and_accepted_list_bounds(self):
        core = self.core(max_requests=1)
        record = self.offer(core)
        missing, _, _ = sample(2)
        self.assertEqual(core.request(request(missing), 0.)['reason'], 'cache_miss')
        self.assertEqual(core.request(request(record), 0.)['reason'], 'request_history_budget')
        core = self.core()
        for seq in range(1, 40): self.offer(core, seq)
        self.assertEqual(len(core.cache), 32)
        with self.assertRaises(ValueError): AcceptedWindow.freeze([request(sample(i)[0]) for i in range(1, 34)])
        with self.assertRaises(ValueError): Policy(max_candidates=33)

    def test_wrong_epoch_session_identity_and_integer_stamp_rejected(self):
        core = self.core(); record = self.offer(core)
        for field in ('map_epoch', 'detector_session_id', 'rgb_sha256'):
            req = request(record, rid=field)
            req['sample'][field] = 'other' if field != 'rgb_sha256' else '0'*64
            if field == 'map_epoch': req['map_epoch'] = 'other'
            result = core.request(req, 0.)
            self.assertEqual(result['status'], 'failed')
        req = request(record); req['sample']['rgb_stamp_ns'] = float(req['sample']['rgb_stamp_ns'])
        with self.assertRaises(ValueError): validate_request(req)

    def test_tf_stamp_quaternion_source_barrier_and_extra_path_fields_rejected(self):
        for defect in ('stamp', 'quaternion', 'barrier', 'path', 'nan'):
            with self.subTest(defect=defect):
                req = request(sample()[0])
                if defect == 'stamp': req['tf']['stamp_ns'] += 1
                elif defect == 'quaternion': req['tf']['camera_pose'][6] = 0.
                elif defect == 'barrier': req['after_stamp_ns'] = req['sample']['depth_stamp_ns']
                elif defect == 'path': req['path'] = '../escape'
                else: req['tf']['base_pose'][0] = float('nan')
                with self.assertRaises(ValueError): validate_request(req)

    def test_request_timeout_pins_owned_writer_until_completion_and_cannot_be_late_stored(self):
        core = self.core(ttl_s=.1, request_timeout_s=.2)
        req = request(self.offer(core)); core.request(req, 0.); job = core.take(0.)
        core.expire(.3)
        result = core.result(req['retention_id'])
        self.assertEqual(result['status'], 'failed'); self.assertTrue(result['writer_owned'])
        self.assertEqual(len(core.cache), 1)
        with tempfile.TemporaryDirectory() as root:
            result = core.finish(job, write_job(job, Path(root).resolve()), .3)
        self.assertEqual(result['reason'], 'request_timeout'); self.assertFalse(result['writer_owned'])
        core.expire(.3); self.assertFalse(core.cache)

    def test_close_checkpoint_keeps_pending_ownership_and_never_claims_cleanup(self):
        core = self.core()
        a, b = self.offer(core), self.offer(core, 2)
        core.request(request(a), 0.); core.request(request(b), 0.)
        job = core.take(0.)
        checkpoint = core.close(.1)
        self.assertTrue(checkpoint['writer_pending'])
        self.assertTrue(all(r['status'] == 'failed' for r in checkpoint['results']))
        self.assertEqual(len(core.cache), 1)
        json.dumps(checkpoint, allow_nan=False)
        self.assertIsNone(core.take(.1))
        result = core.finish(job, None, .2)
        self.assertEqual(result['reason'], 'shutdown_aborted')
        self.assertFalse(core.checkpoint()['writer_pending']); self.assertFalse(core.cache)
        record, rgb, depth = sample(3)
        self.assertFalse(core.offer(record, rgb, depth, .2)['available'])

    def test_exclusive_writer_failure_or_fsync_error_cannot_report_stored(self):
        for defect in ('exists', 'fsync'):
            with self.subTest(defect=defect), tempfile.TemporaryDirectory() as root:
                core = self.core(); req = request(self.offer(core)); core.request(req, 0.); job = core.take(0.)
                root = Path(root).resolve()
                if defect == 'exists':
                    folder = root/job.directory; folder.mkdir(); (folder/'rgb.bin').write_bytes(b'original')
                    with self.assertRaises(FileExistsError): write_job(job, root)
                    self.assertEqual((folder/'rgb.bin').read_bytes(), b'original')
                else:
                    with patch('limo_evidence_protocol.retention.os.fsync', side_effect=OSError('synthetic disk failure')):
                        with self.assertRaises(OSError): write_job(job, root)
                    self.assertFalse((root/job.directory/'manifest.json').exists())
                    self.assertTrue((root/job.directory/'rgb.bin').exists())
                self.assertEqual(core.finish(job, None, .1)['status'], 'failed')

    def test_request_identifier_cannot_control_output_path_and_receipt_mutation_is_isolated(self):
        core = self.core(); req = request(self.offer(core), rid='../escape')
        core.request(req, 0.)
        with tempfile.TemporaryDirectory() as root:
            result, job = self.store(core, root)
            self.assertEqual(len(list(Path(root).iterdir())), 1)
            self.assertEqual(job.directory, sha(canonical(req)))
            result['receipt']['files']['rgb.bin']['sha256'] = 'forged'
            self.assertNotEqual(core.result('../escape')['receipt']['files']['rgb.bin']['sha256'], 'forged')

    def test_foreign_or_duplicate_writer_completion_refused(self):
        core = self.core(); req = request(self.offer(core)); core.request(req, 0.); job = core.take(0.)
        foreign = copy.copy(job)
        with self.assertRaises(ValueError): core.finish(foreign, job.receipt, 0.)
        core.finish(job, None, 0.)
        with self.assertRaises(ValueError): core.finish(job, job.receipt, 0.)

    def test_fixed_three_selection_middle_missing_is_not_replaced_by_fourth(self):
        core = self.core()
        records = [sample(seq)[0] for seq in range(1, 5)]
        for seq in (1, 3, 4): self.offer(core, seq)
        selected = [request(record) for record in records[:3]]
        window = AcceptedWindow.freeze(selected)
        for req in selected: core.request(req, 0.)
        extra = request(records[3]); core.request(extra, 0.)
        with tempfile.TemporaryDirectory() as root:
            while (job := core.take(0.)) is not None:
                core.finish(job, write_job(job, Path(root).resolve()), 0.)
        result = window.evaluate(core.checkpoint()['results'])
        self.assertEqual(result['selected_count'], 3)
        self.assertEqual(result['stored_count'], 2)
        self.assertFalse(result['retention_satisfied'])
        self.assertEqual(result['slots'][1]['result']['reason'], 'cache_miss')
        self.assertEqual(core.result(extra['retention_id'])['status'], 'stored')

    def test_old_window_result_and_ambiguous_duplicate_receipts_cannot_satisfy_reset_window(self):
        core = self.core(); req = request(self.offer(core)); core.request(req, 0.)
        with tempfile.TemporaryDirectory() as root: result, _ = self.store(core, root)
        new = copy.deepcopy(req); new['after_stamp_ns'] += 1
        self.assertFalse(AcceptedWindow.freeze([new]).evaluate([result])['retention_satisfied'])
        self.assertFalse(AcceptedWindow.freeze([req]).evaluate([result, result])['retention_satisfied'])

    def test_monotonic_time_and_metadata_flood_bounds(self):
        core = self.core(); self.offer(core, now=1.)
        for invalid in (0., float('nan'), True):
            with self.assertRaises(ValueError): core.expire(invalid)
        record, rgb, depth = sample(2); record['extra'] = 'x'*70000
        with self.assertRaises(ValueError): core.offer(record, rgb, depth, 1.)
        self.assertEqual(len(core.cache), 1)

    def test_same_accepted_slot_with_new_request_id_conflicts_instead_of_double_writing(self):
        core = self.core(); first, second = self.offer(core), self.offer(core, 2)
        a, b = request(first), request(second, ordinal=1)
        core.request(a, 0.)
        result = core.request(b, 0.)
        self.assertEqual(result['reason'], 'accepted_slot_conflict')
        self.assertEqual(core.result(a['retention_id'])['reason'], 'accepted_slot_conflict')
        self.assertEqual(core.jobs_reserved, 1)
        self.assertIsNone(core.take(0.))

    def test_partial_freeze_mutation_detected_and_pinned_rejection_does_not_copy_raw(self):
        record, rgb, depth = sample()
        prepared = Bundle.prepare(record, rgb, depth, 100000)
        rgb[0] = 9
        with self.assertRaises(ValueError): Bundle.from_prepared(prepared)
        core = self.core(max_candidates=1)
        req = request(self.offer(core)); core.request(req, 0.)
        record, rgb, depth = sample(2)
        with patch.object(Bundle, 'from_prepared', side_effect=AssertionError('must reserve before raw copy')):
            self.assertEqual(core.offer(record, rgb, depth, 0.)['reason'], 'cache_budget_pinned')

    def test_same_blank_pixels_new_source_stamp_is_distinct_and_not_a_conflict(self):
        core = self.core()
        a, b = self.offer(core), self.offer(core, 2)
        self.assertEqual(a['rgb']['data_sha256'], b['rgb']['data_sha256'])
        self.assertNotEqual(a['rgb']['stamp_ns'], b['rgb']['stamp_ns'])
        for r in (a, b): self.assertEqual(core.request(request(r), 0.)['status'], 'queued')
        with tempfile.TemporaryDirectory() as root:
            results = [self.store(core, root)[0], self.store(core, root)[0]]
        self.assertTrue(AcceptedWindow.freeze([request(a), request(b)]).evaluate(results)['retention_satisfied'])

    def test_real_frame_evidence_schema_and_calibration_digest_round_trip(self):
        # Read-only import of the existing pure producer, never perception node.
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/limo_maze_perception'))
        from limo_maze_perception.core import DetectorConfig
        from limo_maze_perception.evidence import FrameEvidence
        example, rgb, depth = sample()
        def message(meta, data):
            stamp = S(sec=meta['stamp_ns']//10**9, nanosec=meta['stamp_ns']%10**9)
            return S(header=S(stamp=stamp, frame_id=meta['frame_id']), data=data,
                     **{k: meta[k] for k in ('width', 'height', 'step', 'encoding', 'is_bigendian')})
        c = example['camera_info']; calibration = dict(c['calibration'])
        calibration['roi'] = S(**calibration['roi'])
        info = S(header=S(stamp=S(sec=c['stamp_ns']//10**9, nanosec=c['stamp_ns']%10**9),
                          frame_id=c['frame_id']), **calibration)
        producer = FrameEvidence(DetectorConfig(min_depth_pixels=1), session_id='session-a')
        producer.on_epoch('epoch-a', 10_000_000_000)
        record = producer.begin(message(example['rgb'], rgb), message(example['depth'], depth), info,
                                10_001_000_100, enabled=True, registered=True)
        producer.finish(record, {'valid_depth_pixels': 1, 'detection_count': 0}, 10_001_000_200)
        self.assertEqual(record['status'], 'processed')
        core = self.core()
        self.assertTrue(core.offer(record, rgb, depth, 0.)['available'])
        req = request(record); core.request(req, 0.)
        with tempfile.TemporaryDirectory() as root:
            result, job = self.store(core, root)
            self.assertEqual(json.loads((Path(root)/job.directory/'producer.json').read_bytes()), record)
        self.assertTrue(AcceptedWindow.freeze([req]).evaluate([result])['retention_satisfied'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
