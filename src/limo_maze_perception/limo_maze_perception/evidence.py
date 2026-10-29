"""Bounded processing evidence, independent of ROS, TF, coverage and mission control."""
from dataclasses import asdict
import hashlib
import json
from math import isfinite
import uuid

from .core import Intrinsics


def finite_json(value):
    """Malformed metadata remains explicit null; never emit NaN/Infinity."""
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return float(value) if isfinite(value) else None
    if isinstance(value, dict):
        return {str(k): finite_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite_json(v) for v in value]
    return None


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode("utf-8")).hexdigest()


def source_stamp_ns(message):
    try:
        stamp = message.header.stamp
        if type(stamp.sec) is not int or type(stamp.nanosec) is not int:
            return None
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            return None
        return stamp.sec * 1_000_000_000 + stamp.nanosec
    except AttributeError:
        return None


def image_metadata(message):
    if message is None:
        return None
    result = {name: finite_json(getattr(message, name, None)) for name in
              ("encoding", "width", "height", "step", "is_bigendian")}
    result.update(stamp_ns=source_stamp_ns(message),
                  frame_id=finite_json(getattr(getattr(message, "header", None), "frame_id", None)))
    try:
        raw = bytes(message.data)
        result.update(data_bytes=len(raw), data_sha256=hashlib.sha256(raw).hexdigest())
    except (AttributeError, TypeError, ValueError, OverflowError):
        result.update(data_bytes=None, data_sha256=None)
    return result


def camera_metadata(message):
    if message is None:
        return None
    calibration = {name: finite_json(getattr(message, name, None)) for name in
                   ("width", "height", "distortion_model", "binning_x", "binning_y")}
    for name in ("k", "d", "r", "p"):
        try:
            calibration[name] = finite_json(list(getattr(message, name)))
        except (AttributeError, TypeError):
            calibration[name] = None
    roi = getattr(message, "roi", None)
    calibration["roi"] = {name: finite_json(getattr(roi, name, None)) for name in
                          ("x_offset", "y_offset", "height", "width", "do_rectify")}
    frame = finite_json(getattr(getattr(message, "header", None), "frame_id", None))
    # Acquisition time is intentionally excluded: this is a calibration version.
    return {"stamp_ns": source_stamp_ns(message), "frame_id": frame,
            "calibration": calibration,
            "calibration_id": digest({"frame_id": frame, **calibration})}


def metadata_reason(rgb, depth, info, slop_ns, info_max_age_ns):
    if rgb is None or depth is None:
        return "missing_image"
    if info is None:
        return "missing_camera_info"
    stamps = [item["stamp_ns"] for item in (rgb, depth, info)]
    if any(value is None for value in stamps):
        return "invalid_source_stamp"
    if abs(stamps[0]-stamps[1]) > slop_ns:
        return "rgb_depth_unsynchronized"
    if abs(stamps[0]-stamps[2]) > info_max_age_ns:
        return "stale_camera_info"
    if (not isinstance(rgb["frame_id"], str) or not rgb["frame_id"] or
            any(item["frame_id"] != rgb["frame_id"] for item in (depth, info))):
        return "frame_mismatch"
    for item, encodings in ((rgb, {"rgb8": 3, "bgr8": 3}), (depth, {"32FC1": 4, "16UC1": 2})):
        if not isinstance(item["encoding"], str) or item["encoding"] not in encodings:
            return "unsupported_encoding"
        if any(type(item[key]) is not int or item[key] <= 0 for key in ("width", "height", "step")):
            return "invalid_image_geometry"
        if (item["is_bigendian"] not in (0, 1) or item["data_sha256"] is None or
                item["step"] < item["width"]*encodings[item["encoding"]] or
                item["data_bytes"] != item["step"]*item["height"]):
            return "invalid_image_buffer"
    c = info["calibration"]
    if any((item["width"], item["height"]) != (c["width"], c["height"]) for item in (rgb, depth)):
        return "dimension_mismatch"
    for field, length in (("k", 9), ("r", 9), ("p", 12)):
        if not isinstance(c[field], list) or len(c[field]) != length:
            return "invalid_calibration"
    if not isinstance(c["d"], list):
        return "invalid_calibration"
    if any(type(v) not in (float, int) or not isfinite(v) for field in ("k", "d", "r", "p") for v in c[field]):
        return "invalid_calibration"
    try:
        Intrinsics(c["width"], c["height"], c["k"][0], c["k"][4], c["k"][2], c["k"][5]).validate()
    except (ValueError, TypeError):
        return "invalid_calibration"
    if any(abs(v) > 1e-8 for v in c["d"]):
        return "unsupported_distortion"
    roi = c["roi"]
    if (any(type(c[key]) is not int or c[key] not in (0, 1) for key in ("binning_x", "binning_y")) or
            any(type(roi[key]) is not int or roi[key] != 0 for key in ("x_offset", "y_offset")) or
            roi["width"] not in (0, c["width"]) or roi["height"] not in (0, c["height"]) or
            type(roi["do_rectify"]) is not bool or roi["do_rectify"]):
        return "unsupported_roi_or_binning"
    return None


class FrameEvidence:
    """One in-flight callback plus watermarks, never an unbounded frame history."""
    def __init__(self, config, slop_seconds=.05, info_max_age_seconds=2., session_id=None):
        config.validate()
        self.config = asdict(config)
        self.config_id = digest(self.config)
        self.session_id = session_id or str(uuid.uuid4())
        self.min_depth_pixels = config.min_depth_pixels
        self.slop_ns = round(slop_seconds*1e9)
        self.info_max_age_ns = round(info_max_age_seconds*1e9)
        self.epoch = None
        self.epoch_ready = False
        self.epoch_barrier_ns = None
        self.clock_ns = None
        self.clock_reset = False
        self.rgb_watermark = self.depth_watermark = None
        self.sequence = 0

    def invalidate_clock(self):
        """Called on a ROS clock jump even when no images are arriving."""
        self.epoch_ready = False
        self.clock_reset = True

    def check_clock(self, now_ns):
        if type(now_ns) is not int or now_ns < 0:
            return False
        if self.clock_ns is not None and now_ns < self.clock_ns:
            self.invalidate_clock()
        self.clock_ns = now_ns
        return True

    def on_epoch(self, epoch, now_ns):
        if not self.check_clock(now_ns):
            self.epoch_ready = False
            return
        if not isinstance(epoch, str) or not epoch.strip():
            self.epoch_ready = False
            return
        epoch = epoch.strip()
        if epoch == self.epoch:
            return  # Duplicate announcements cannot re-arm after a clock reset.
        self.epoch, self.epoch_ready, self.epoch_barrier_ns = epoch, True, now_ns
        self.clock_reset = False
        self.rgb_watermark = self.depth_watermark = None

    def begin(self, color, depth, info, now_ns, *, enabled, registered):
        self.sequence += 1
        clock_valid = self.check_clock(now_ns)
        rgb, dep, cam = image_metadata(color), image_metadata(depth), camera_metadata(info)
        reason = metadata_reason(rgb, dep, cam, self.slop_ns, self.info_max_age_ns)
        if not enabled:
            reason = "detector_disabled"
        elif not registered:
            reason = "depth_registration_unconfirmed"
        elif not clock_valid:
            reason = "invalid_ros_clock"
        elif self.clock_reset:
            reason = "clock_reset_requires_new_epoch"
        elif not self.epoch_ready:
            reason = "map_epoch_unavailable"
        elif reason is None:
            if min(rgb["stamp_ns"], dep["stamp_ns"]) <= self.epoch_barrier_ns:
                reason = "sample_before_epoch_barrier"
            elif max(rgb["stamp_ns"], dep["stamp_ns"]) > now_ns + 100_000_000:
                reason = "sample_in_future"
            elif ((self.rgb_watermark is not None and rgb["stamp_ns"] <= self.rgb_watermark) or
                  (self.depth_watermark is not None and dep["stamp_ns"] <= self.depth_watermark)):
                reason = "nonadvancing_sample"
        return {"schema_version": 1, "source": "red_rgbd_sim_baseline",
                "detector_session_id": self.session_id, "sequence": self.sequence,
                "map_epoch": self.epoch, "status": "rejected", "reason": reason,
                "rgb": rgb, "depth": dep, "camera_info": cam,
                "callback_stamp_ns": now_ns if clock_valid else None, "processed_stamp_ns": None,
                "detector_config": self.config.copy(), "detector_config_id": self.config_id,
                "processing": None,
                "hash_scope": "Original Image.data bytes, including padding; hashes do not prove source images were saved."}

    def finish(self, record, stats, now_ns, error=None):
        valid_clock = self.check_clock(now_ns)
        record["processed_stamp_ns"] = now_ns if valid_clock else None
        record["processing"] = finite_json(stats)
        reason = record["reason"] or error
        if not valid_clock or self.clock_reset:
            reason = "clock_reset_requires_new_epoch" if self.clock_reset else "invalid_ros_clock"
        elif record["map_epoch"] != self.epoch or not self.epoch_ready:
            reason = reason or "epoch_changed_during_processing"
        if reason is None:
            if stats is None:
                reason = "algorithm_not_completed"
            elif stats["valid_depth_pixels"] < self.min_depth_pixels:
                reason = "insufficient_frame_depth"
        if reason is None:
            record["status"] = "processed"
            self.rgb_watermark, self.depth_watermark = record["rgb"]["stamp_ns"], record["depth"]["stamp_ns"]
        record["reason"] = reason
        return record
