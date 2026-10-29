"""ROS-independent red component detection and robust optical-frame projection.

NumPy is required. OpenCV accelerates connected components when installed; the
small-image NumPy/Python fallback keeps offline tests portable.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from math import isfinite
import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None


@dataclass(frozen=True)
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self):
        if self.width <= 0 or self.height <= 0:
            raise ValueError("CameraInfo dimensions must be positive")
        if not all(isfinite(v) for v in (self.fx, self.fy, self.cx, self.cy)) or self.fx <= 0 or self.fy <= 0:
            raise ValueError("Intrinsics must be finite; fx/fy must be calibrated and positive")
        if not 0 <= self.cx < self.width or not 0 <= self.cy < self.height:
            raise ValueError("Principal point must lie within the baseline image")


@dataclass(frozen=True)
class DetectorConfig:
    hue_tolerance_degrees: float = 15.0
    min_saturation: float = 0.55
    min_value: float = 0.30
    min_area_pixels: int = 30
    min_depth_pixels: int = 8
    min_valid_depth_fraction: float = 0.5
    min_depth_m: float = 0.15
    max_depth_m: float = 6.0
    inner_fraction: float = 0.6

    def validate(self):
        if not all(isfinite(v) for v in vars(self).values()):
            raise ValueError("Detector configuration must be finite")
        if not 0 < self.hue_tolerance_degrees < 60:
            raise ValueError("Red hue tolerance must be between 0 and 60 degrees")
        if not all(0 < v <= 1 for v in (self.min_saturation, self.min_value, self.min_valid_depth_fraction, self.inner_fraction)):
            raise ValueError("Fractions must lie in (0,1]")
        if self.min_area_pixels < 1 or self.min_depth_pixels < 1 or not 0 < self.min_depth_m < self.max_depth_m:
            raise ValueError("Pixel counts and depth limits must be positive")


def validate_frame_metadata(*, color_stamp: float, depth_stamp: float, info_stamp: float,
                            color_frame: str, depth_frame: str, info_frame: str,
                            registered: bool, slop: float = 0.05, max_info_age: float = 2.0):
    if not registered:
        raise ValueError("Depth registration to the color camera must be explicitly confirmed")
    if not all(isfinite(v) and v >= 0 for v in (color_stamp, depth_stamp, info_stamp, slop, max_info_age)):
        raise ValueError("Timestamps and synchronization limits must be finite and non-negative")
    if abs(color_stamp - depth_stamp) > slop:
        raise ValueError("RGB and depth acquisition timestamps exceed sync slop")
    if abs(color_stamp - info_stamp) > max_info_age:
        raise ValueError("CameraInfo is stale or belongs to a different time interval")
    if not color_frame or color_frame != depth_frame or color_frame != info_frame:
        raise ValueError("RGB, registered depth, and CameraInfo must share one optical frame")


def red_mask(image: np.ndarray, encoding: str, config: DetectorConfig) -> np.ndarray:
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Color input must be H x W x 3 uint8")
    if encoding not in ("rgb8", "bgr8"):
        raise ValueError("Only rgb8 and bgr8 color encodings are supported")
    rgb = image if encoding == "rgb8" else image[..., ::-1]
    rgb = rgb.astype(np.float32) / 255.0
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    maximum, minimum = rgb.max(axis=2), rgb.min(axis=2)
    delta = maximum - minimum
    hue = np.zeros_like(maximum)
    nonzero = delta > 0
    red = nonzero & (maximum == r)
    green = nonzero & ~red & (maximum == g)
    blue = nonzero & ~red & ~green
    hue[red] = 60 * np.mod((g[red] - b[red]) / delta[red], 6)
    hue[green] = 60 * ((b[green] - r[green]) / delta[green] + 2)
    hue[blue] = 60 * ((r[blue] - g[blue]) / delta[blue] + 4)
    saturation = np.divide(delta, maximum, out=np.zeros_like(maximum), where=maximum > 0)
    return ((hue <= config.hue_tolerance_degrees) | (hue >= 360 - config.hue_tolerance_degrees)) & (saturation >= config.min_saturation) & (maximum >= config.min_value)


def components(mask: np.ndarray, min_area: int):
    """Yield component row/column indices; 8-connectivity for both backends."""
    if cv2 is not None:
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
        for label in range(1, count):
            if stats[label, cv2.CC_STAT_AREA] >= min_area:
                yield np.where(labels == label)
        return
    visited = np.zeros_like(mask, dtype=bool)
    height, width = mask.shape
    for y, x in zip(*np.where(mask)):
        if visited[y, x]:
            continue
        queue, pixels = deque([(int(y), int(x))]), []
        visited[y, x] = True
        while queue:
            py, px = queue.popleft()
            pixels.append((py, px))
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    ny, nx = py + dy, px + dx
                    if 0 <= ny < height and 0 <= nx < width and mask[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        queue.append((ny, nx))
        if len(pixels) >= min_area:
            coordinates = np.asarray(pixels)
            yield coordinates[:, 0], coordinates[:, 1]


def detect_red_objects(color: np.ndarray, depth: np.ndarray, intrinsics: Intrinsics,
                       depth_encoding: str, color_encoding: str = "rgb8",
                       config: DetectorConfig = DetectorConfig()) -> list[dict]:
    """Compatibility API: detection values and ordering remain unchanged."""
    return process_red_frame(color, depth, intrinsics, depth_encoding, color_encoding, config)[0]


def process_red_frame(color: np.ndarray, depth: np.ndarray, intrinsics: Intrinsics,
                      depth_encoding: str, color_encoding: str = "rgb8",
                      config: DetectorConfig = DetectorConfig()) -> tuple[list[dict], dict]:
    """Run once and retain input quality and depth-rejected component counts."""
    intrinsics.validate()
    config.validate()
    if depth.ndim != 2 or depth.shape != color.shape[:2] or depth.shape != (intrinsics.height, intrinsics.width):
        raise ValueError("Registered RGB, depth, and CameraInfo dimensions must match exactly")
    if depth_encoding == "32FC1" and depth.dtype == np.float32:
        depth_m = depth
    elif depth_encoding == "16UC1" and depth.dtype == np.uint16:
        depth_m = depth.astype(np.float32) * 0.001
    else:
        raise ValueError("Depth must be 32FC1 float32 meters or 16UC1 uint16 millimeters")
    mask = red_mask(color, color_encoding, config)
    detections = []
    stats = {"total_pixels": int(depth_m.size),
             "valid_depth_pixels": int(np.count_nonzero(np.isfinite(depth_m) &
                (depth_m >= config.min_depth_m) & (depth_m <= config.max_depth_m))),
             "candidate_count": 0, "detection_count": 0, "unlocalized_candidate_count": 0,
             "rejection_counts": {"too_few_inner_pixels": 0, "insufficient_valid_depth": 0,
                                  "mixed_depth": 0, "insufficient_depth_inliers": 0,
                                  "nonfinite_projection": 0}, "empty_reason": None}
    for ys, xs in components(mask, config.min_area_pixels):
        stats["candidate_count"] += 1
        # Restrict samples to red pixels inside the central component bounding
        # box, reducing mixed foreground/background depth along object edges.
        center_x, center_y = (xs.min() + xs.max()) / 2, (ys.min() + ys.max()) / 2
        half_x = (xs.max() - xs.min() + 1) * config.inner_fraction / 2
        half_y = (ys.max() - ys.min() + 1) * config.inner_fraction / 2
        inner = (np.abs(xs - center_x) <= half_x) & (np.abs(ys - center_y) <= half_y)
        xs, ys = xs[inner], ys[inner]
        if len(xs) < config.min_depth_pixels:
            stats["rejection_counts"]["too_few_inner_pixels"] += 1
            continue
        values = depth_m[ys, xs]
        valid = np.isfinite(values) & (values >= config.min_depth_m) & (values <= config.max_depth_m)
        if np.count_nonzero(valid) < config.min_depth_pixels or valid.mean() < config.min_valid_depth_fraction:
            stats["rejection_counts"]["insufficient_valid_depth"] += 1
            continue
        xs, ys, values = xs[valid], ys[valid], values[valid]
        median = float(np.median(values))
        mad = float(np.median(np.abs(values - median)))
        if mad > max(0.02, 0.10 * median):
            # A component containing two comparable depth surfaces is not a
            # reliable point observation; do not average foreground/background.
            stats["rejection_counts"]["mixed_depth"] += 1
            continue
        inliers = np.abs(values - median) <= max(0.05, 3 * 1.4826 * mad)
        if np.count_nonzero(inliers) < config.min_depth_pixels:
            stats["rejection_counts"]["insufficient_depth_inliers"] += 1
            continue
        u, v, z = float(np.median(xs[inliers])), float(np.median(ys[inliers])), float(np.median(values[inliers]))
        position = [(u - intrinsics.cx) * z / intrinsics.fx,
                    (v - intrinsics.cy) * z / intrinsics.fy, z]
        if not all(isfinite(value) for value in position):
            stats["rejection_counts"]["nonfinite_projection"] += 1
            continue
        # A baseline quality score, not a calibrated identity probability.
        confidence = float(min(0.95, 0.6 + 0.35 * valid.mean() * inliers.mean()))
        detections.append({"class_id": "red_object", "position": position, "confidence": confidence})
    stats["detection_count"] = len(detections)
    stats["unlocalized_candidate_count"] = sum(stats["rejection_counts"].values())
    if not detections:
        stats["empty_reason"] = "no_red_candidates" if not stats["candidate_count"] else "all_candidates_rejected"
    return detections, stats


def observation_payload(stamp: float, frame_id: str, detections: list[dict]) -> dict:
    if not isfinite(stamp) or stamp < 0 or not frame_id:
        raise ValueError("Observation must retain a finite acquisition timestamp and optical frame")
    for detection in detections:
        if len(detection["position"]) != 3 or not all(isfinite(x) for x in detection["position"]):
            raise ValueError("Detection positions must be finite XYZ meters")
        if not isfinite(detection["confidence"]) or not 0 <= detection["confidence"] <= 1:
            raise ValueError("Detection confidence must be finite within [0,1]")
    return {"schema_version": 1, "stamp": float(stamp), "frame_id": frame_id,
            "source": "red_rgbd_sim_baseline", "detections": detections}
