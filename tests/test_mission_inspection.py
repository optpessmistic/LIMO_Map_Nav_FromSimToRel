"""Adversarial processing evidence and bounded inspection request tests."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/limo_maze_mission"))
from limo_maze_mission.inspection import EvidenceError, InspectionLedger, InspectionPolicy, validate_record


N = 1_000_000_000
BASE = (1.0, 2.0, .2)
CAMERA = (1.08, 2.0, .25, 0.0, 0.0, 0.0, 1.0)


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


def frame(stamp=10*N+100_000_000, seq=1, session="detector-a", depth_stamp=None):
    latest_sample = max(stamp, stamp if depth_stamp is None else depth_stamp)
    def image(encoding, channels, stamp):
        return {"encoding": encoding, "width": 80, "height": 60, "step": 80*channels,
                "is_bigendian": 0, "stamp_ns": stamp, "frame_id": "camera_optical",
                "data_bytes": 4800*channels, "data_sha256": "a"*64}
    calibration = {"width": 80, "height": 60, "distortion_model": "plumb_bob", "binning_x": 0, "binning_y": 0,
                   "k": [100.,0.,39.5,0.,100.,29.5,0.,0.,1.], "d": [0.]*5,
                   "r": [1.,0.,0.,0.,1.,0.,0.,0.,1.],
                   "p": [100.,0.,39.5,0.,0.,100.,29.5,0.,0.,0.,1.,0.],
                   "roi": {"x_offset": 0, "y_offset": 0, "height": 0, "width": 0, "do_rectify": False}}
    config = {"hue_tolerance_degrees": 15.0, "min_saturation": .55, "min_value": .30, "min_area_pixels": 30,
              "min_depth_pixels": 8, "min_valid_depth_fraction": .5, "min_depth_m": .15,
              "max_depth_m": 6.0, "inner_fraction": .6}
    return {"schema_version": 1, "source": "red_rgbd_sim_baseline", "detector_session_id": session,
            "sequence": seq, "map_epoch": "map-a", "status": "processed", "reason": None,
            "rgb": image("rgb8", 3, stamp),
            "depth": image("32FC1", 4, stamp if depth_stamp is None else depth_stamp),
            "camera_info": {"stamp_ns": stamp, "frame_id": "camera_optical", "calibration": calibration,
                            "calibration_id": digest({"frame_id": "camera_optical", **calibration})},
            "callback_stamp_ns": latest_sample+10_000_000, "processed_stamp_ns": latest_sample+20_000_000,
            "detector_config": config, "detector_config_id": digest(config),
            "processing": {"total_pixels": 4800, "valid_depth_pixels": 4800, "candidate_count": 0,
                           "detection_count": 0, "unlocalized_candidate_count": 0,
                           "rejection_counts": {"too_few_inner_pixels": 0, "insufficient_valid_depth": 0,
                                                "mixed_depth": 0, "insufficient_depth_inliers": 0,
                                                "nonfinite_projection": 0}, "empty_reason": "no_red_candidates"},
            "hash_scope": "Original Image.data bytes, including padding; hashes do not prove source images were saved."}


def nested_set(record, path, value):
    for key in path[:-1]:
        record = record[key]
    record[path[-1]] = value


class ValidationTests(unittest.TestCase):
    def reject(self, record, reason=None, now=11*N, policy=InspectionPolicy()):
        with self.assertRaises(EvidenceError) as caught:
            validate_record(record, now, policy)
        if reason:
            self.assertEqual(str(caught.exception), reason)

    def test_empty_and_unlocalized_candidates_are_both_valid_processing_evidence(self):
        record = frame()
        self.assertEqual(validate_record(record, 11*N).detections, 0)
        record["processing"].update(candidate_count=1, unlocalized_candidate_count=1,
                                     empty_reason="all_candidates_rejected")
        record["processing"]["rejection_counts"]["mixed_depth"] = 1
        sample = validate_record(record, 11*N)
        self.assertEqual((sample.candidates, sample.unlocalized_candidates), (1, 1))

    def test_config_and_calibration_digests_cannot_hide_metadata_changes(self):
        for path, value, reason in ((["detector_config", "min_area_pixels"], 31, "config_digest_mismatch"),
                                    (["camera_info", "calibration", "p", 0], 101., "calibration_digest_mismatch")):
            record = frame(); nested_set(record, path, value)
            self.reject(record, reason)

    def test_boolean_numeric_values_and_nonfinite_numbers_are_rejected(self):
        for path, value in ((["schema_version"], True), (["sequence"], True),
                            (["rgb", "stamp_ns"], True), (["rgb", "width"], True),
                            (["depth", "is_bigendian"], True),
                            (["detector_config", "min_area_pixels"], True),
                            (["camera_info", "calibration", "k", 0], float("nan")),
                            (["camera_info", "calibration", "r", 3], float("inf")),
                            (["processing", "valid_depth_pixels"], True)):
            with self.subTest(path=path):
                record = frame(); nested_set(record, path, value)
                self.reject(record)

    def test_malformed_and_oversized_metadata_fails_without_crashing(self):
        for path, value in ((["rgb"], None), (["rgb", "encoding"], []),
                            (["rgb", "data_sha256"], "x"*10000), (["rgb", "data_bytes"], 8),
                            (["rgb", "width"], 1000000), (["rgb", "step"], 10),
                            (["camera_info", "calibration", "k"], []),
                            (["camera_info", "calibration", "d"], [0.]*10000),
                            (["detector_session_id"], "x"*257), (["sequence"], 2**200),
                            (["detector_config"], []), (["processing", "rejection_counts"], [])):
            with self.subTest(path=path):
                record = frame(); nested_set(record, path, value)
                self.reject(record)
        for value in (None, [], True, 1, "{}", {}, {"bad": 1}):
            self.reject(value)

    def test_status_source_and_schema_are_explicit_not_truthiness_checks(self):
        for path, value in ((["status"], "rejected"), (["reason"], "error"),
                            (["schema_version"], 2), (["source"], "unknown")):
            record = frame(); nested_set(record, path, value)
            self.reject(record)
        record = frame(); record["source"] = "replacement-v1"
        self.assertEqual(validate_record(record, 11*N, InspectionPolicy(allowed_source="replacement-v1")).sequence, 1)

    def test_sync_frames_and_calibration_contracts(self):
        for path, value, reason in ((["depth", "frame_id"], "other", "frame_mismatch"),
                                    (["depth", "stamp_ns"], 10*N, "rgb_depth_unsynchronized"),
                                    (["camera_info", "stamp_ns"], 0, "stale_camera_info"),
                                    (["camera_info", "calibration", "d", 0], .1, "unsupported_distortion"),
                                    (["camera_info", "calibration", "roi", "do_rectify"], True, "unsupported_roi_or_binning"),
                                    (["camera_info", "calibration", "binning_x"], 2, "unsupported_roi_or_binning")):
            with self.subTest(path=path):
                record = frame(); nested_set(record, path, value)
                self.reject(record, reason)

    def test_source_and_processing_age_and_future_are_separate_checks(self):
        self.reject(frame(), "stale_sample", now=12*N)
        record = frame(); record["processed_stamp_ns"] = 12*N
        self.reject(record, "sample_in_future")
        record = frame(11*N)
        self.reject(record, "sample_in_future")
        record = frame(); record["processed_stamp_ns"] = record["callback_stamp_ns"]-1
        self.reject(record, "invalid_processing_time")
        self.reject(frame(), "stale_processing", policy=InspectionPolicy(max_processing_age_ns=10_000_000))

    def test_queued_receipt_cannot_hide_processing_before_source_sample(self):
        record = frame(10*N+50_000_000)
        record["callback_stamp_ns"] = 10*N
        record["processed_stamp_ns"] = 10*N+10_000_000
        self.reject(record, "invalid_processing_time", now=10*N+100_000_000)
        record["processed_stamp_ns"] = 10*N+60_000_000
        self.assertEqual(validate_record(record, 10*N+100_000_000).rgb_stamp_ns,
                         10*N+50_000_000)

    def test_stats_counts_and_empty_reason_must_agree(self):
        for path, value in ((["processing", "total_pixels"], 4799),
                            (["processing", "valid_depth_pixels"], 7),
                            (["processing", "candidate_count"], 1),
                            (["processing", "detection_count"], -1),
                            (["processing", "empty_reason"], None),
                            (["processing", "rejection_counts", "mixed_depth"], 1)):
            record = frame(); nested_set(record, path, value)
            self.reject(record, "invalid_processing_stats")

    def test_actual_producer_round_trip(self):
        # This integration is optional only on hosts without the detector's numpy dependency.
        try:
            import numpy as np
        except ImportError:
            self.skipTest("numpy unavailable")
        from types import SimpleNamespace as S
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/limo_maze_perception"))
        from limo_maze_perception.core import DetectorConfig, Intrinsics, process_red_frame
        from limo_maze_perception.evidence import FrameEvidence
        record = frame()
        def header():
            return S(stamp=S(sec=10, nanosec=100_000_000), frame_id="camera_optical")
        color, depth = np.zeros((60, 80, 3), np.uint8), np.full((60, 80), 2., np.float32)
        rgb_msg = S(header=header(), width=80, height=60, encoding="rgb8", step=240, is_bigendian=0, data=color.tobytes())
        depth_msg = S(header=header(), width=80, height=60, encoding="32FC1", step=320, is_bigendian=0, data=depth.tobytes())
        c = record["camera_info"]["calibration"]
        info = S(header=header(), **{k:v for k,v in c.items() if k != "roi"}, roi=S(**c["roi"]))
        producer = FrameEvidence(DetectorConfig()); producer.on_epoch("map-a", 10*N)
        attempt = producer.begin(rgb_msg, depth_msg, info, 10*N+110_000_000, enabled=True, registered=True)
        stats = process_red_frame(color, depth, Intrinsics(80,60,100,100,39.5,29.5), "32FC1")[1]
        emitted = producer.finish(attempt, stats, 10*N+120_000_000)
        sample = validate_record(json.loads(json.dumps(emitted, allow_nan=False)), 11*N)
        self.assertEqual((sample.rgb_stamp_ns, sample.detections), (10*N+100_000_000, 0))


class InspectionTests(unittest.TestCase):
    def setUp(self):
        self.ledger = InspectionLedger()
        self.ledger.set_epoch("map-a", 10*N)
        self.ledger.begin(10*N, BASE, 100., 8., "frontier-1")

    def observe(self, record=None, **kwargs):
        record = record or frame()
        now_ns = record["processed_stamp_ns"]+10_000_000
        args = dict(now_ns=now_ns, wall_now=100.+(now_ns-10*N)/N,
                    base_pose=BASE, camera_pose=CAMERA, tf_stamp_ns=record["rgb"]["stamp_ns"])
        args.update(kwargs)
        return self.ledger.observe(record, **args)

    def test_three_distinct_frames_and_both_time_spans_are_required(self):
        for seq, offset in enumerate((.1, .15, .2), 1):
            self.assertTrue(self.observe(frame(10*N+round(offset*N), seq))["accepted"])
        self.assertEqual(self.ledger.status, "waiting")
        self.assertEqual(self.observe(frame(10*N+300_000_000, 4))["status"], "satisfied")
        report = self.ledger.snapshot()
        self.assertEqual(report["accepted_frames"], 4)
        self.assertFalse(report["visual_coverage_claimed"])
        self.assertNotIn("targets_verified", report)
        self.assertNotIn("coverage", report)

    def test_depth_span_cannot_be_replaced_by_rgb_span(self):
        for seq, rgb, depth in ((1, .10, .14), (2, .20, .20), (3, .30, .30)):
            result = self.observe(frame(10*N+round(rgb*N), seq, depth_stamp=10*N+round(depth*N)),
                                  now_ns=10*N+round((max(rgb,depth)+.04)*N),
                                  wall_now=100.+max(rgb,depth)+.04)
            self.assertTrue(result["accepted"])
        self.assertEqual(self.ledger.status, "waiting")
        self.assertEqual(self.observe(frame(10*N+340_000_000, 4))["status"], "satisfied")

    def test_replayed_sequence_rgb_or_depth_cannot_count(self):
        self.assertTrue(self.observe()["accepted"])
        for record in (frame(seq=2), frame(10*N+120_000_000, 1),
                       frame(10*N+130_000_000, 3, depth_stamp=10*N+100_000_000)):
            result = self.observe(record, now_ns=10*N+200_000_000, wall_now=100.2)
            self.assertEqual(result["reason"], "nonadvancing_sample")
        self.assertEqual(self.ledger.accepted_frames, 1)

    def test_both_images_must_be_strictly_after_request_barrier(self):
        record = frame(10*N+20_000_000, depth_stamp=10*N)
        self.assertEqual(self.observe(record)["reason"], "sample_before_barrier")
        self.assertEqual(self.ledger.accepted_frames, 0)

    def test_invalid_tf_stamp_or_quaternion_never_count(self):
        result = self.observe(tf_stamp_ns=0)
        self.assertEqual(result["reason"], "sample_time_tf_required")
        self.assertEqual(self.observe()["reason"], "nonadvancing_sample")
        result = self.observe(frame(10*N+200_000_000, 2), camera_pose=(1.,2.,.2,0.,0.,0.,0.))
        self.assertEqual(result["reason"], "invalid_sample_pose")
        result = self.observe(frame(10*N+300_000_000, 3), base_pose=(True,2.,.2))
        self.assertEqual(result["reason"], "invalid_sample_pose")
        self.assertEqual(self.ledger.accepted_frames, 0)

    def test_pose_excursion_discards_prior_frames_and_advances_barrier(self):
        self.observe()
        result = self.observe(frame(10*N+200_000_000, 2), base_pose=(1.1,2.,.2))
        self.assertEqual(result["reason"], "sample_pose_outside_tolerance")
        self.assertEqual(self.ledger.accepted_frames, 0)
        self.assertEqual(self.ledger.deadline_wall, 108.)
        self.assertEqual(self.observe(frame(10*N+220_000_000, 3))["reason"], "sample_before_barrier")
        self.assertTrue(self.observe(frame(10*N+300_000_000, 4))["accepted"])
        self.assertEqual(self.ledger.accepted_frames, 1)

    def test_stationarity_reset_preserves_original_wall_deadline(self):
        self.observe()
        self.ledger.reset_window(10*N+300_000_000, "stationarity_lost")
        self.assertEqual(self.ledger.accepted_frames, 0)
        self.assertEqual(self.ledger.tick(10*N+300_000_000, 108.)["status"], "timed_out")

    def test_motion_before_scheduler_consumes_success_clears_satisfied_window(self):
        for seq, offset in enumerate((.1,.2,.3), 1):
            self.observe(frame(10*N+round(offset*N), seq))
        self.assertEqual(self.ledger.status, "satisfied")
        self.ledger.reset_window(10*N+400_000_000, "stationarity_lost")
        self.assertEqual((self.ledger.status, self.ledger.accepted_frames), ("waiting", 0))
        self.assertEqual(self.ledger.deadline_wall, 108.)
        self.assertEqual(self.ledger.tick(10*N+400_000_000, 108.)["status"], "timed_out")

    def test_detector_restart_requires_new_receipt_barrier_and_new_window(self):
        self.observe()
        changed = frame(10*N+200_000_000, 1, session="detector-b")
        self.assertEqual(self.observe(changed)["reason"], "producer_identity_changed")
        self.assertEqual((self.ledger.accepted_frames, self.ledger.deadline_wall), (0,108.))
        replay = frame(10*N+210_000_000, 20, session="detector-a")
        self.assertEqual(self.observe(replay, now_ns=10*N+250_000_000, wall_now=100.25)["reason"], "sample_before_barrier")
        for seq, offset in enumerate((.3,.4,.5), 2):
            result = self.observe(frame(10*N+round(offset*N), seq, session="detector-b"))
        self.assertEqual(result["status"], "satisfied")

    def test_config_or_calibration_change_resets_window_even_with_same_session(self):
        for change in ("config", "calibration"):
            with self.subTest(change=change):
                self.setUp(); self.observe()
                record = frame(10*N+200_000_000, 2)
                if change == "config":
                    record["detector_config"]["min_area_pixels"] = 31
                    record["detector_config_id"] = digest(record["detector_config"])
                else:
                    c = record["camera_info"]["calibration"]; c["k"][0] = 101.
                    record["camera_info"]["calibration_id"] = digest({"frame_id":"camera_optical", **c})
                self.assertEqual(self.observe(record)["reason"], "producer_identity_changed")
                self.assertEqual(self.ledger.accepted_frames, 0)

    def test_epoch_mismatch_and_duplicate_epoch_do_not_rearm_after_clock_jump(self):
        record = frame(); record["map_epoch"] = "old-map"
        self.assertEqual(self.observe(record)["reason"], "epoch_mismatch")
        self.ledger.tick(5*N, 101.)
        self.assertEqual(self.ledger.status, "invalidated")
        self.ledger.set_epoch("map-a", 11*N)
        self.assertFalse(self.ledger.ready)
        with self.assertRaises(ValueError):
            self.ledger.begin(11*N, BASE, 102.)
        self.ledger.set_epoch("map-b", 11*N)
        self.assertTrue(self.ledger.ready)
        self.ledger.begin(11*N, BASE, 102.)
        record = frame(11*N+100_000_000); record["map_epoch"] = "map-b"
        self.assertTrue(self.observe(record, wall_now=102.5)["accepted"])

    def test_between_frame_clock_jump_invalidation_stays_locked_when_time_catches_up(self):
        self.observe()
        self.ledger.invalidate("clock_reset_requires_new_epoch")
        self.ledger.tick(11*N, 101.)
        self.assertFalse(self.ledger.ready)
        self.assertEqual(self.ledger.accepted_frames, 0)

    def test_paused_ros_clock_times_out_using_monotonic_wall_time(self):
        self.observe()
        self.assertEqual(self.ledger.tick(10*N+130_000_000, 108.)["status"], "timed_out")
        self.assertFalse(self.observe(frame(10*N+200_000_000, 2), wall_now=108.1)["accepted"])

    def test_new_request_cannot_reuse_earlier_samples(self):
        self.observe()
        self.ledger.begin(10*N+200_000_000, BASE, 101., 8., "frontier-2")
        self.assertEqual(self.observe(frame(), wall_now=101.1)["reason"], "sample_before_barrier")
        self.assertEqual(self.ledger.accepted_frames, 0)

    def test_payload_flood_does_not_grow_frame_or_session_history(self):
        for i in range(1000):
            self.ledger.observe({}, now_ns=10*N, wall_now=100., base_pose=BASE,
                                camera_pose=CAMERA, tf_stamp_ns=10*N)
        snapshot = self.ledger.snapshot()
        self.assertEqual(snapshot["rejection_counts"], {"invalid_schema": 1000})
        self.assertLess(len(json.dumps(snapshot)), 1800)
        self.assertFalse(any(key in vars(self.ledger) for key in ("frames", "sessions", "history")))

    def test_adapter_reasons_do_not_allow_unbounded_counter_keys(self):
        for index in range(1000):
            self.ledger.reject(f"exception text {index}")
        self.ledger.reject(["not a string"])
        self.ledger.reject("sample_tf_unavailable")
        self.assertEqual(self.ledger.snapshot()["rejection_counts"],
                         {"adapter_rejected": 1001, "sample_tf_unavailable": 1})

    def test_bad_policy_and_request_inputs_fail_closed(self):
        for kwargs in ({"required_frames": True}, {"min_span_ns": 0}, {"xy_tolerance": float("nan")},
                       {"max_sample_age_ns": True}, {"allowed_source": ""}):
            with self.assertRaises(ValueError):
                InspectionPolicy(**kwargs)
        for pose, deadline in (((True,2.,.2), 8.), (BASE, float("inf")), (BASE, -1.)):
            with self.assertRaises(ValueError):
                self.ledger.begin(10*N, pose, 100., deadline)


if __name__ == "__main__":
    unittest.main()
