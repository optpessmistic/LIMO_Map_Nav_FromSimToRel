"""Offline RGB-D tests; no ROS, camera, Gazebo, or ground-truth object feed."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "limo_maze_perception"))
from limo_maze_perception import core
from limo_maze_perception.core import DetectorConfig, Intrinsics, detect_red_objects, observation_payload, validate_frame_metadata


class PerceptionCoreTests(unittest.TestCase):
    def setUp(self):
        self.rgb = np.zeros((60, 80, 3), np.uint8)
        self.rgb[20:40, 30:50] = [255, 0, 0]
        self.depth = np.full((60, 80), 2, np.float32)
        self.intrinsics = Intrinsics(80, 60, 100, 100, 39.5, 29.5)

    def detect(self, rgb=None, depth=None, encoding="32FC1", color_encoding="rgb8"):
        return detect_red_objects(self.rgb if rgb is None else rgb, self.depth if depth is None else depth,
                                  self.intrinsics, encoding, color_encoding)

    def test_center_projection_and_exact_contract(self):
        detections = self.detect()
        self.assertEqual(len(detections), 1)
        np.testing.assert_allclose(detections[0]["position"], [0, 0, 2])
        payload = observation_payload(12.345, "camera_optical_frame", detections)
        self.assertEqual(set(payload), {"schema_version", "stamp", "frame_id", "source", "detections"})
        self.assertEqual(payload["source"], "red_rgbd_sim_baseline")
        self.assertEqual(set(detections[0]), {"class_id", "position", "confidence"})
        self.assertNotIn("coverage", json.dumps(payload, allow_nan=False))

    def test_rgb_bgr_and_mm_m_are_consistent(self):
        expected = self.detect()
        bgr = self.rgb[..., ::-1].copy()
        millimeters = np.full((60, 80), 2000, np.uint16)
        actual = self.detect(bgr, millimeters, "16UC1", "bgr8")
        np.testing.assert_allclose(actual[0]["position"], expected[0]["position"], rtol=1e-6)

    def test_optical_axes_and_multiple_components(self):
        image = np.zeros_like(self.rgb)
        image[5:15, 5:15] = [255, 0, 0]
        image[40:50, 60:70] = [255, 0, 0]
        detections = self.detect(image)
        self.assertEqual(len(detections), 2)
        positions = sorted(d["position"] for d in detections)
        np.testing.assert_allclose(positions[0], [-0.6, -0.4, 2])
        np.testing.assert_allclose(positions[1], [0.5, 0.3, 2])

    def test_nonred_and_tiny_noise_do_not_trigger(self):
        image = np.zeros_like(self.rgb)
        image[20:40, 30:50] = [0, 255, 0]
        image[1:3, 1:3] = [255, 0, 0]
        self.assertEqual(self.detect(image), [])

    def test_red_hue_wrap_is_supported(self):
        self.rgb[20:40, 30:50] = [255, 0, 30]
        self.assertEqual(len(self.detect()), 1)

    def test_nan_infinity_zero_negative_and_far_depth_rejected(self):
        for value in (float("nan"), float("inf"), 0, -1, 100):
            with self.subTest(depth=value):
                self.assertEqual(self.detect(depth=np.full((60, 80), value, np.float32)), [])

    def test_depth_holes_and_minority_outliers_are_robust(self):
        self.depth[24:27, 34:37] = np.nan
        self.depth[29:32, 38:41] = 5.5
        detections = self.detect()
        self.assertEqual(len(detections), 1)
        self.assertAlmostEqual(detections[0]["position"][2], 2.0)

    def test_insufficient_or_mixed_depth_drops_observation(self):
        self.depth[:, :40] = 5.0
        self.assertEqual(self.detect(), [])
        self.depth[:] = np.nan
        self.depth[29, 39] = 2
        self.assertEqual(self.detect(), [])

    def test_dimension_dtype_encoding_and_calibration_validation(self):
        for depth, encoding in ((self.depth[:, :70], "32FC1"), (self.depth.astype(np.float64), "32FC1"),
                                (self.depth, "16UC1"), (self.depth, "8UC1")):
            with self.subTest(shape=depth.shape, encoding=encoding), self.assertRaises(ValueError):
                self.detect(depth=depth, encoding=encoding)
        for intrinsics in (Intrinsics(80, 60, 0, 100, 40, 30), Intrinsics(80, 60, float("nan"), 100, 40, 30)):
            with self.assertRaises(ValueError):
                detect_red_objects(self.rgb, self.depth, intrinsics, "32FC1")

    def test_registration_frame_and_acquisition_time_checks(self):
        valid = dict(color_stamp=10.0, depth_stamp=10.02, info_stamp=10.0,
                     color_frame="camera_optical_frame", depth_frame="camera_optical_frame",
                     info_frame="camera_optical_frame", registered=True)
        validate_frame_metadata(**valid)
        for override in ({"registered": False}, {"depth_frame": "depth_unaligned"}, {"color_frame": ""},
                         {"depth_stamp": 10.5}, {"info_stamp": 1}, {"color_stamp": float("nan")}, {"depth_stamp": -1}):
            with self.subTest(override=override), self.assertRaises(ValueError):
                validate_frame_metadata(**(valid | override))

    def test_numpy_fallback_agrees_with_default_backend(self):
        expected = self.detect()
        with patch.object(core, "cv2", None):
            self.assertEqual(self.detect(), expected)

    def test_invalid_serialization_never_emits_nonfinite_values(self):
        for stamp in (float("nan"), float("inf"), -1):
            with self.assertRaises(ValueError):
                observation_payload(stamp, "camera_optical_frame", [])
        with self.assertRaises(ValueError):
            observation_payload(1, "camera_optical_frame", [{"position": [0, 0, float("nan")], "confidence": .9}])


if __name__ == "__main__":
    unittest.main()
