"""Offline regressions: synthetic SDF/frames, no ROS, Gazebo, vendor or outputs."""
import argparse
from contextlib import redirect_stdout
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

try:
    import numpy as np
except ImportError:
    np = None

checker = None
if np is not None:
    source = Path(__file__).resolve().parents[1] / "scripts/check_scan_alignment.py"
    spec = importlib.util.spec_from_file_location("scan_alignment_under_test", source)
    checker = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(checker)


@unittest.skipIf(np is None, "NumPy is unavailable; offline scan-alignment tests explicitly skipped")
class ScanAlignmentTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="scan-alignment-tests-")
        self.root = Path(self.temporary.name).resolve()
        self.world = self.root / "world.sdf"
        self.model = self.root / "model.sdf"
        self.world.write_text("""<sdf version="1.9"><world name="test_world">
          <model name="wall"><static>true</static><pose>2 0 .5 0 0 0</pose>
            <link name="body">
              <collision name="collision"><geometry><box><size>.2 4 1</size></box></geometry></collision>
              <visual name="visual"><geometry><box><size>.2 4 1</size></box></geometry></visual>
            </link>
          </model>
        </world></sdf>""", encoding="utf-8")
        self.model.write_text("""<sdf version="1.9"><model name="limo" canonical_link="base_footprint">
          <link name="base_footprint"><sensor name="laser_sensor" type="gpu_lidar">
            <pose>.1 0 .2 0 0 0</pose>
            <lidar><scan><vertical><samples>1</samples><min_angle>0</min_angle><max_angle>0</max_angle></vertical></scan></lidar>
          </sensor></link>
        </model></sdf>""", encoding="utf-8")
        self.options = argparse.Namespace(odom_frame="odom", laser_frame="laser_link", lags_ms=[0])

    def tearDown(self):
        # Only remove the fresh directory owned by this test.
        self.assertTrue(self.root.is_relative_to(Path(tempfile.gettempdir()).resolve()))
        self.temporary.cleanup()

    def geometry(self):
        return checker.load_geometry(self.world, self.model, "limo", "laser_sensor")

    def edit_world(self, edit):
        document = ET.parse(self.world)
        edit(document.getroot())
        document.write(self.world, encoding="utf-8")

    def frames(self):
        # A stationary sensor at x=.1 sees the wall's x=1.9 face at 1.8m.
        scans, native, transforms = {}, {}, {}
        for stamp in (10_000_000, 30_000_000):
            scans[str(stamp)] = {"ranges": [1.8], "angle_min": 0., "angle_max": 0.,
                "angle_step": .01, "frame": "laser_link", "time_increment": 0., "scan_time": .1,
                "range_min": .05, "range_max": 8., "intensities": []}
            native[str(stamp)] = {"ranges": [1.8], "angle_min": 0., "angle_step": .01,
                                  "range_min": .05, "range_max": 8., "vertical_count": 1}
            transforms[str(stamp)] = {"odom": {"position": [.1, 0., .2],
                "quaternion": [0., 0., 0., 1.], "stamp_ns": stamp}}
        return {"native_scans": native, "ros_scans": scans, "transforms": transforms,
                "native_model_poses": {str(t): {"position": [0., 0., 0.], "quaternion": [0., 0., 0., 1.]}
                                       for t in (0, 20_000_000, 40_000_000)}}

    def run_main(self, replay, output):
        argv = ["check_scan_alignment.py", "--world", str(self.world), "--model-sdf", str(self.model),
                "--replay", str(replay), "--output", str(output), "--lags-ms=0"]
        with patch.object(sys, "argv", argv), redirect_stdout(io.StringIO()):
            return checker.main()

    def test_accepts_matching_static_boxes(self):
        boxes, local, name = self.geometry()
        self.assertEqual((len(boxes), name), (1, "test_world"))
        np.testing.assert_allclose(local[:3, 3], [.1, 0, .2])
        summary, rows, skipped = checker.analyze(self.frames(), boxes, local, self.options)
        self.assertEqual(summary["aligned_scan_count"], 2)
        self.assertEqual(skipped, [])
        self.assertLess(rows[0]["ray_error_m"]["rms"], 1e-12)

    def test_rejects_relative_frames(self):
        self.edit_world(lambda r: r.find("world/model/pose").set("relative_to", "world"))
        with self.assertRaisesRegex(checker.Unsupported, "Pose attributes"):
            self.geometry()

    def test_rejects_visual_collision_mismatch(self):
        def edit(root):
            root.find("world/model/link/visual/geometry/box/size").text = "1 1 1"
        self.edit_world(edit)
        with self.assertRaisesRegex(checker.Unsupported, "differ"):
            self.geometry()

    def test_rejects_dynamic_world_models(self):
        def edit(root):
            root.find("world/model/static").text = "false"
        self.edit_world(edit)
        with self.assertRaisesRegex(checker.Unsupported, "non-static"):
            self.geometry()

    def test_rejects_non_box_geometry(self):
        def edit(root):
            root.find("world/model/link/collision/geometry/box").tag = "sphere"
        self.edit_world(edit)
        with self.assertRaisesRegex(checker.Unsupported, "Only static box"):
            self.geometry()

    def test_rejects_custom_visibility(self):
        self.edit_world(lambda r: ET.SubElement(r.find("world/model/link/visual"), "visibility_flags"))
        with self.assertRaisesRegex(checker.Unsupported, "visibility_flags"):
            self.geometry()

    def test_keeps_odom_frames_without_map(self):
        boxes, local, _ = self.geometry()
        raw = self.frames()
        self.assertTrue(all(set(record) == {"odom"} for record in raw["transforms"].values()))
        summary, _, _ = checker.analyze(raw, boxes, local, self.options)
        self.assertEqual(summary["aligned_scan_count"], 2)

    def test_includes_sensor_local_rotation(self):
        document = ET.parse(self.model)
        document.getroot().find("model/link/sensor/pose").text = ".1 0 .2 0 0 .3"
        document.write(self.model, encoding="utf-8")
        boxes, local, _ = self.geometry()
        _, rows, _ = checker.analyze(self.frames(), boxes, local, self.options)
        for row in rows:
            self.assertTrue(math.isclose(row["world_minus_odom_sensor_yaw_rad"], .3, abs_tol=1e-12))

    def test_repeated_replay_cannot_upgrade_legacy_provenance(self):
        legacy = self.root / "legacy.json"
        legacy_frames = self.frames()
        for scan in legacy_frames["ros_scans"].values():
            for field in ("range_min", "range_max", "intensities"):
                scan.pop(field)
        legacy.write_text(json.dumps(legacy_frames), encoding="utf-8")
        first, second = self.root / "first.json", self.root / "second.json"
        self.assertEqual(self.run_main(legacy, first), 0)
        self.assertEqual(self.run_main(first, second), 0)
        for path in (first, second):
            report = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(report["geometry_provenance"], "legacy_unverified")
            self.assertTrue(any("Legacy replay" in line for line in report["limitations"]))
            self.assertEqual(report["raw"]["ros_scans"], legacy_frames["ros_scans"])
            self.assertTrue(all("range_min" not in scan and "range_max" not in scan
                                for scan in report["raw"]["ros_scans"].values()))

    def test_rejects_replay_geometry_hash_mismatch(self):
        source = self.root / "wrong_hash.json"
        source.write_text(json.dumps({"inputs": {"world": {"sha256": "wrong"},
                                                 "model_sdf": {"sha256": "wrong"}}}), encoding="utf-8")
        output = self.root / "rejected.json"
        self.assertEqual(self.run_main(source, output), 2)
        result = json.loads(output.read_text(encoding="utf-8"))
        self.assertEqual(result["status"], "unsupported")
        self.assertIn("hash mismatch", result["error"])

    def test_refuses_to_overwrite_output(self):
        output = self.root / "existing.json"
        output.write_bytes(b"existing evidence\n")
        with self.assertRaises(FileExistsError):
            self.run_main(self.root / "not_read.json", output)
        self.assertEqual(output.read_bytes(), b"existing evidence\n")


if __name__ == "__main__":
    unittest.main()
