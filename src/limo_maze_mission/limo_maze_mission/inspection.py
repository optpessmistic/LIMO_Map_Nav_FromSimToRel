"""Bounded checks for processed RGB-D frames at a settled observation pose.

This module never writes Coverage or interprets an empty detection as absence.
The caller must establish stationarity, obtain both TF poses at the RGB sample
stamp (no latest-TF fallback), and call reset_window whenever stationarity is
lost. A pose check here is a second guard, not proof that the robot is stopped.
"""
from dataclasses import dataclass
import hashlib
import json
from math import hypot, isfinite
import re

from .core import angle_delta


_MAX_NS = 2**63 - 1
_SHA = re.compile(r"[0-9a-f]{64}\Z")
_REJECTIONS = ("too_few_inner_pixels", "insufficient_valid_depth", "mixed_depth",
               "insufficient_depth_inliers", "nonfinite_projection")
_CONFIG = ("hue_tolerance_degrees", "min_saturation", "min_value", "min_area_pixels",
           "min_depth_pixels", "min_valid_depth_fraction", "min_depth_m", "max_depth_m",
           "inner_fraction")
_REASON_CODES = frozenset((
    "invalid_schema", "producer_rejected", "invalid_identity", "invalid_image_metadata",
    "unsupported_encoding", "invalid_source_stamp", "invalid_image_geometry", "invalid_image_buffer",
    "invalid_camera_info", "invalid_calibration", "dimension_mismatch", "unsupported_distortion",
    "unsupported_roi_or_binning", "calibration_digest_mismatch", "invalid_detector_config",
    "config_digest_mismatch", "frame_mismatch", "rgb_depth_unsynchronized", "stale_camera_info",
    "invalid_processing_time", "sample_in_future", "stale_sample", "stale_processing",
    "invalid_processing_stats", "invalid_clock", "no_active_inspection", "epoch_mismatch",
    "sample_before_barrier", "producer_identity_changed", "nonadvancing_sample",
    "sample_time_tf_required", "invalid_sample_pose", "sample_pose_outside_tolerance",
    "evidence_payload_too_large", "invalid_json", "odometry_does_not_cover_sample_time",
    "sample_tf_unavailable", "invalid_sample_transform", "adapter_rejected",
))


class EvidenceError(ValueError):
    """A fixed, bounded reason code; never includes arbitrary payload text."""


def _require(condition, reason):
    if not condition:
        raise EvidenceError(reason)


def _integer(value, minimum=0, maximum=_MAX_NS):
    return type(value) is int and minimum <= value <= maximum


def _number(value, limit=1e9):
    return type(value) in (int, float) and abs(value) <= limit and isfinite(value)


def _text(value):
    return type(value) is str and 0 < len(value) <= 256 and value.strip() == value


def _sha(value):
    return type(value) is str and len(value) == 64 and _SHA.fullmatch(value) is not None


def _mapping(value, keys, reason):
    _require(type(value) is dict and set(value) == set(keys), reason)


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode("utf-8")).hexdigest()


@dataclass(frozen=True)
class InspectionPolicy:
    required_frames: int = 3
    min_span_ns: int = 200_000_000
    max_sample_age_ns: int = 1_500_000_000
    max_processing_age_ns: int = 1_500_000_000
    sync_slop_ns: int = 50_000_000
    info_max_age_ns: int = 2_000_000_000
    xy_tolerance: float = .04
    yaw_tolerance: float = .10
    allowed_source: str = "red_rgbd_sim_baseline"

    def __post_init__(self):
        if (not _integer(self.required_frames, 1, 1000) or
                not all(_integer(v, 1) for v in (self.min_span_ns, self.max_sample_age_ns,
                                                 self.max_processing_age_ns)) or
                not all(_integer(v) for v in (self.sync_slop_ns, self.info_max_age_ns)) or
                not _number(self.xy_tolerance) or not 0 < self.xy_tolerance <= 1 or
                not _number(self.yaw_tolerance) or not 0 < self.yaw_tolerance <= 3.141593 or
                not _text(self.allowed_source)):
            raise ValueError("Invalid inspection policy")


@dataclass(frozen=True)
class FrameSample:
    session_id: str
    sequence: int
    epoch: str
    rgb_stamp_ns: int
    depth_stamp_ns: int
    callback_stamp_ns: int
    processed_stamp_ns: int
    frame_id: str
    config_id: str
    calibration_id: str
    rgb_sha256: str
    depth_sha256: str
    candidates: int
    detections: int
    unlocalized_candidates: int


def _image(item, encodings):
    _mapping(item, ("encoding", "width", "height", "step", "is_bigendian", "stamp_ns",
                    "frame_id", "data_bytes", "data_sha256"), "invalid_image_metadata")
    _require(type(item["encoding"]) is str and item["encoding"] in encodings,
             "unsupported_encoding")
    _require(_integer(item["stamp_ns"]) and _text(item["frame_id"]), "invalid_source_stamp")
    _require(_integer(item["width"], 1, 8192) and _integer(item["height"], 1, 8192) and
             item["width"] * item["height"] <= 16_777_216, "invalid_image_geometry")
    _require(_integer(item["step"], item["width"] * encodings[item["encoding"]], 131072) and
             _integer(item["is_bigendian"], 0, 1) and
             _integer(item["data_bytes"], 1, 134_217_728) and
             item["data_bytes"] == item["step"] * item["height"] and
             _sha(item["data_sha256"]), "invalid_image_buffer")


def _calibration(info, rgb):
    _mapping(info, ("stamp_ns", "frame_id", "calibration", "calibration_id"), "invalid_camera_info")
    _require(_integer(info["stamp_ns"]) and _text(info["frame_id"]), "invalid_camera_info")
    c = info["calibration"]
    _mapping(c, ("width", "height", "distortion_model", "binning_x", "binning_y",
                 "k", "d", "r", "p", "roi"), "invalid_calibration")
    _require(_integer(c["width"], 1, 8192) and _integer(c["height"], 1, 8192) and
             (c["width"], c["height"]) == (rgb["width"], rgb["height"]), "dimension_mismatch")
    _require(type(c["distortion_model"]) is str and len(c["distortion_model"]) <= 64,
             "invalid_calibration")
    for key, length in (("k", 9), ("r", 9), ("p", 12)):
        _require(type(c[key]) is list and len(c[key]) == length and
                 all(_number(x) for x in c[key]), "invalid_calibration")
    _require(type(c["d"]) is list and len(c["d"]) <= 16 and
             all(_number(x) and abs(x) <= 1e-8 for x in c["d"]), "unsupported_distortion")
    _require(c["k"][0] > 0 and c["k"][4] > 0 and 0 <= c["k"][2] < c["width"] and
             0 <= c["k"][5] < c["height"], "invalid_calibration")
    roi = c["roi"]
    _mapping(roi, ("x_offset", "y_offset", "height", "width", "do_rectify"), "invalid_calibration")
    _require(all(_integer(c[key], 0, 1) for key in ("binning_x", "binning_y")) and
             all(_integer(roi[key], 0, 0) for key in ("x_offset", "y_offset")) and
             _integer(roi["width"]) and roi["width"] in (0, c["width"]) and
             _integer(roi["height"]) and roi["height"] in (0, c["height"]) and
             roi["do_rectify"] is False, "unsupported_roi_or_binning")
    _require(_sha(info["calibration_id"]) and info["calibration_id"] ==
             _digest({"frame_id": info["frame_id"], **c}), "calibration_digest_mismatch")


def _config(config, config_id):
    _mapping(config, _CONFIG, "invalid_detector_config")
    _require(all(_number(v) for v in config.values()), "invalid_detector_config")
    _require(0 < config["hue_tolerance_degrees"] < 60 and
             all(0 < config[k] <= 1 for k in ("min_saturation", "min_value",
                                              "min_valid_depth_fraction", "inner_fraction")) and
             all(_integer(config[k], 1, 16_777_216) for k in ("min_area_pixels", "min_depth_pixels")) and
             0 < config["min_depth_m"] < config["max_depth_m"] <= 10000, "invalid_detector_config")
    _require(_sha(config_id) and config_id == _digest(config), "config_digest_mismatch")


def validate_record(record, now_ns, policy=InspectionPolicy()):
    """Validate JSON-decoded evidence without state changes or producer imports.

    Hashes establish consistency of claimed metadata, not authenticity, retained
    source images, calibration quality, or correctness of the detector itself.
    A caller should bound the JSON byte length before decoding it.
    """
    _require(_integer(now_ns), "invalid_clock")
    _require(type(record) is dict and len(record) <= 24, "invalid_schema")
    required = ("schema_version", "source", "detector_session_id", "sequence", "map_epoch", "status",
                "reason", "rgb", "depth", "camera_info", "callback_stamp_ns", "processed_stamp_ns",
                "detector_config", "detector_config_id", "processing", "hash_scope")
    _require(set(record) == set(required) and type(record["schema_version"]) is int and
             record["schema_version"] == 1 and record["source"] == policy.allowed_source,
             "invalid_schema")
    _require(record["status"] == "processed" and record["reason"] is None, "producer_rejected")
    _require(_text(record["detector_session_id"]) and _integer(record["sequence"], 1) and
             _text(record["map_epoch"]), "invalid_identity")
    _require(type(record["hash_scope"]) is str and len(record["hash_scope"]) <= 512, "invalid_schema")
    rgb, depth, info = record["rgb"], record["depth"], record["camera_info"]
    _image(rgb, {"rgb8": 3, "bgr8": 3})
    _image(depth, {"32FC1": 4, "16UC1": 2})
    _calibration(info, rgb)
    _require((rgb["width"], rgb["height"]) == (depth["width"], depth["height"]), "dimension_mismatch")
    _require(rgb["frame_id"] == depth["frame_id"] == info["frame_id"], "frame_mismatch")
    _require(abs(rgb["stamp_ns"] - depth["stamp_ns"]) <= policy.sync_slop_ns, "rgb_depth_unsynchronized")
    _require(abs(rgb["stamp_ns"] - info["stamp_ns"]) <= policy.info_max_age_ns, "stale_camera_info")
    callback, processed = record["callback_stamp_ns"], record["processed_stamp_ns"]
    _require(_integer(callback) and _integer(processed) and callback <= processed and
             max(rgb["stamp_ns"], depth["stamp_ns"]) <= processed,
             "invalid_processing_time")
    _require(max(rgb["stamp_ns"], depth["stamp_ns"], info["stamp_ns"], processed) <= now_ns,
             "sample_in_future")
    _require(max(rgb["stamp_ns"], depth["stamp_ns"]) <= callback + 100_000_000,
             "invalid_processing_time")
    _require(now_ns - min(rgb["stamp_ns"], depth["stamp_ns"]) <= policy.max_sample_age_ns,
             "stale_sample")
    _require(now_ns - processed <= policy.max_processing_age_ns, "stale_processing")
    config = record["detector_config"]
    _config(config, record["detector_config_id"])
    stats = record["processing"]
    _mapping(stats, ("total_pixels", "valid_depth_pixels", "candidate_count", "detection_count",
                     "unlocalized_candidate_count", "rejection_counts", "empty_reason"), "invalid_processing_stats")
    total = rgb["width"] * rgb["height"]
    keys = ("total_pixels", "valid_depth_pixels", "candidate_count", "detection_count", "unlocalized_candidate_count")
    _require(all(_integer(stats[k], 0, total) for k in keys) and stats["total_pixels"] == total and
             stats["valid_depth_pixels"] >= config["min_depth_pixels"], "invalid_processing_stats")
    rejected = stats["rejection_counts"]
    _mapping(rejected, _REJECTIONS, "invalid_processing_stats")
    _require(all(_integer(v, 0, total) for v in rejected.values()) and
             sum(rejected.values()) == stats["unlocalized_candidate_count"] and
             stats["candidate_count"] == stats["detection_count"] + stats["unlocalized_candidate_count"],
             "invalid_processing_stats")
    expected_empty = (None if stats["detection_count"] else
                      "all_candidates_rejected" if stats["candidate_count"] else "no_red_candidates")
    _require(stats["empty_reason"] == expected_empty, "invalid_processing_stats")
    return FrameSample(record["detector_session_id"], record["sequence"], record["map_epoch"],
                       rgb["stamp_ns"], depth["stamp_ns"], callback, processed, rgb["frame_id"],
                       record["detector_config_id"], info["calibration_id"], rgb["data_sha256"],
                       depth["data_sha256"], stats["candidate_count"], stats["detection_count"],
                       stats["unlocalized_candidate_count"])


def _pose(value, count):
    return (type(value) in (tuple, list) and len(value) == count and all(_number(x) for x in value))


class InspectionLedger:
    """One request and one producer identity: memory does not grow with frames.

    Restart/version changes discard the window, then require samples acquired
    after their receipt. An old session replay cannot revive a previous window.
    Resets preserve the original wall deadline, including when ROS time stops.
    """
    def __init__(self, policy=InspectionPolicy()):
        self.policy = policy
        self.epoch = None
        self.epoch_barrier_ns = None
        self.ready = False
        self.clock_ns = None
        self.wall_ns = None
        self.identity = None
        self.sequence = self.rgb_watermark = self.depth_watermark = None
        self.request_id = None
        self.status = "idle"
        self.reason = None
        self.after_stamp_ns = None
        self.expected_pose = None
        self.deadline_wall = None
        self.rejection_counts = {}
        self.reset_count = 0
        self._clear_window()

    def _clear_window(self):
        self.accepted_frames = 0
        self.first_rgb_stamp_ns = self.last_rgb_stamp_ns = None
        self.first_depth_stamp_ns = self.last_depth_stamp_ns = None
        self.last_evidence = None

    def _clock(self, now_ns, wall_now=None):
        if not _integer(now_ns):
            self.invalidate("invalid_clock")
            return False
        if self.clock_ns is not None and now_ns < self.clock_ns:
            self.clock_ns = now_ns
            self.invalidate("clock_reset_requires_new_epoch")
            return False
        self.clock_ns = now_ns
        if wall_now is not None:
            if (not _number(wall_now, 1e15) or wall_now < 0 or
                    self.wall_ns is not None and wall_now < self.wall_ns):
                self.invalidate("invalid_wall_clock")
                return False
            self.wall_ns = wall_now
        return True

    def set_epoch(self, epoch, now_ns):
        self._clock(now_ns)
        if not _text(epoch):
            self.invalidate("invalid_epoch")
            return
        if epoch == self.epoch:
            return  # A repeated announcement cannot unlock invalidated evidence.
        if not _integer(now_ns):
            return
        self.epoch, self.epoch_barrier_ns = epoch, now_ns
        self.ready = True
        self.identity = None
        self.sequence = self.rgb_watermark = self.depth_watermark = None
        self.status = "invalidated" if self.status == "waiting" else "idle"
        self.reason = "epoch_changed" if self.status == "invalidated" else None
        self._clear_window()

    def invalidate(self, reason="invalidated"):
        # Only local reason codes are persisted, never unbounded transport input.
        if not _text(reason):
            reason = "invalidated"
        self.ready = False
        self.status, self.reason = "invalidated", reason
        self._clear_window()

    def retire_request(self, reason="request_retired"):
        """End only this request; never grant or restore epoch/clock authority.

        The caller must preserve its audit snapshot first. Late samples remain
        rejected, and the next request needs a new terminal/settling barrier.
        An invalidated Ledger stays invalidated, including its original reason.
        """
        if not _text(reason):
            raise ValueError("Request retirement requires a bounded reason")
        if self.ready and self.status in ("waiting", "satisfied", "timed_out"):
            self.status, self.reason = "retired", reason
            self._clear_window()
        return self.snapshot()

    def begin(self, after_stamp_ns, expected_pose, wall_now, timeout_seconds=8.0, request_id="inspection"):
        if (not self.ready or not _integer(after_stamp_ns) or after_stamp_ns < self.epoch_barrier_ns or
                not _pose(expected_pose, 3) or not _number(wall_now, 1e15) or wall_now < 0 or
                not _number(timeout_seconds) or not 0 < timeout_seconds <= 300 or not _text(request_id)):
            raise ValueError("Inspection requires a valid epoch, settled pose, barrier and wall deadline")
        if self.wall_ns is not None and wall_now < self.wall_ns:
            raise ValueError("Wall clock cannot move backward")
        self.wall_ns = wall_now
        self.request_id, self.expected_pose = request_id, tuple(expected_pose)
        self.after_stamp_ns, self.deadline_wall = after_stamp_ns, wall_now + timeout_seconds
        self.status, self.reason = "waiting", None
        self.rejection_counts = {}
        self.reset_count = 0
        self._clear_window()
        return self.snapshot()

    def reset_window(self, after_stamp_ns, reason="stationarity_lost"):
        if self.status not in ("waiting", "satisfied"):
            return self.snapshot()
        if not _integer(after_stamp_ns) or not _text(reason):
            self.invalidate("invalid_reset")
            return self.snapshot()
        self.after_stamp_ns = max(self.after_stamp_ns, after_stamp_ns)
        self.status = "waiting"
        self.reset_count = min(self.reset_count + 1, _MAX_NS)
        self.reason = reason
        self._clear_window()
        return self.snapshot()

    def tick(self, now_ns, wall_now):
        if self._clock(now_ns, wall_now) and self.status == "waiting" and wall_now >= self.deadline_wall:
            self.status, self.reason = "timed_out", "inspection_deadline"
        return self.snapshot()

    def _reject(self, reason):
        return self.reject(reason)

    def reject(self, reason):
        """Record bounded adapter failures without retaining exception text."""
        if type(reason) is not str or reason not in _REASON_CODES:
            reason = "adapter_rejected"
        self.rejection_counts[reason] = min(self.rejection_counts.get(reason, 0) + 1, _MAX_NS)
        return {"accepted": False, "reason": reason, "status": self.status}

    def observe(self, record, *, now_ns, wall_now, base_pose, camera_pose, tf_stamp_ns):
        self.tick(now_ns, wall_now)
        if self.status != "waiting" or not self.ready:
            return self._reject("no_active_inspection")
        try:
            sample = validate_record(record, now_ns, self.policy)
        except EvidenceError as exc:
            return self._reject(str(exc))
        if sample.epoch != self.epoch:
            return self._reject("epoch_mismatch")
        if min(sample.rgb_stamp_ns, sample.depth_stamp_ns) <= max(self.after_stamp_ns, self.epoch_barrier_ns):
            return self._reject("sample_before_barrier")
        identity = (sample.session_id, sample.config_id, sample.calibration_id, sample.frame_id)
        if self.identity is None:
            self.identity = identity
        elif self.identity != identity:
            self.identity = identity
            self.sequence = self.rgb_watermark = self.depth_watermark = None
            self.reset_window(now_ns, "producer_identity_changed")
            return self._reject("producer_identity_changed")
        if ((self.sequence is not None and sample.sequence <= self.sequence) or
                (self.rgb_watermark is not None and sample.rgb_stamp_ns <= self.rgb_watermark) or
                (self.depth_watermark is not None and sample.depth_stamp_ns <= self.depth_watermark)):
            return self._reject("nonadvancing_sample")
        # A sample cannot be retried with a different claimed transform or pose.
        self.sequence, self.rgb_watermark, self.depth_watermark = sample.sequence, sample.rgb_stamp_ns, sample.depth_stamp_ns
        if not _integer(tf_stamp_ns) or tf_stamp_ns != sample.rgb_stamp_ns:
            return self._reject("sample_time_tf_required")
        if (not _pose(base_pose, 3) or not _pose(camera_pose, 7) or
                abs(sum(v*v for v in camera_pose[3:]) - 1.0) > 1e-3):
            return self._reject("invalid_sample_pose")
        if (hypot(base_pose[0]-self.expected_pose[0], base_pose[1]-self.expected_pose[1]) > self.policy.xy_tolerance or
                abs(angle_delta(base_pose[2], self.expected_pose[2])) > self.policy.yaw_tolerance):
            self.reset_window(now_ns, "sample_pose_outside_tolerance")
            return self._reject("sample_pose_outside_tolerance")
        self.accepted_frames += 1
        if self.first_rgb_stamp_ns is None:
            self.first_rgb_stamp_ns, self.first_depth_stamp_ns = sample.rgb_stamp_ns, sample.depth_stamp_ns
        self.last_rgb_stamp_ns, self.last_depth_stamp_ns = sample.rgb_stamp_ns, sample.depth_stamp_ns
        self.last_evidence = {"rgb_stamp_ns": sample.rgb_stamp_ns, "depth_stamp_ns": sample.depth_stamp_ns,
                              "processed_stamp_ns": sample.processed_stamp_ns, "sequence": sample.sequence,
                              "rgb_sha256": sample.rgb_sha256, "depth_sha256": sample.depth_sha256,
                              "base_pose": list(base_pose), "camera_pose": list(camera_pose),
                              "candidate_count": sample.candidates, "detection_count": sample.detections,
                              "unlocalized_candidate_count": sample.unlocalized_candidates}
        if (self.accepted_frames >= self.policy.required_frames and
                min(self.last_rgb_stamp_ns-self.first_rgb_stamp_ns,
                    self.last_depth_stamp_ns-self.first_depth_stamp_ns) >= self.policy.min_span_ns):
            self.status, self.reason = "satisfied", "processed_frame_window"
        return {"accepted": True, "reason": None, "status": self.status}

    def snapshot(self):
        return {"request_id": self.request_id, "status": self.status, "reason": self.reason,
                "map_epoch": self.epoch, "ready": self.ready, "after_stamp_ns": self.after_stamp_ns,
                "deadline_wall": self.deadline_wall, "expected_pose": list(self.expected_pose) if self.expected_pose else None,
                "required_frames": self.policy.required_frames, "min_span_ns": self.policy.min_span_ns,
                "accepted_frames": self.accepted_frames, "first_rgb_stamp_ns": self.first_rgb_stamp_ns,
                "last_rgb_stamp_ns": self.last_rgb_stamp_ns, "first_depth_stamp_ns": self.first_depth_stamp_ns,
                "last_depth_stamp_ns": self.last_depth_stamp_ns, "producer_identity": list(self.identity) if self.identity else None,
                "reset_count": self.reset_count, "rejection_counts": dict(self.rejection_counts),
                "last_evidence": dict(self.last_evidence) if self.last_evidence else None,
                "visual_coverage_claimed": False}
