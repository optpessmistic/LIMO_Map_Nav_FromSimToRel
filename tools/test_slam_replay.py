"""Synthetic input/cleanup regressions; no ROS, outputs, Gazebo or movement."""
from enum import IntEnum
from contextlib import redirect_stdout
import importlib.util
import io
import json
import math
from pathlib import Path
import signal
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

path = Path(__file__).resolve().parents[1]/"scripts/replay_slam_diagnostic.py"
spec = importlib.util.spec_from_file_location("slam_replay_test", path)
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


def fixture():
    scans, aligned = {}, []
    for i in range(41):
        ns = 1_000_000_000+i*125_000_000
        angle = max(0, min(i-8, 24))*.05
        q = [0., 0., math.sin(angle/2), math.cos(angle/2)]
        scans[str(ns)] = {"ranges": [1., 1.1, 1.2], "angle_min": -.1, "angle_max": .1,
            "angle_step": .1, "frame": "laser_link", "time_increment": 0., "scan_time": 0.}
        aligned.append({"stamp_ns": ns, "tf": {"odom_to_laser_link": {
            "stamp_ns": ns, "position": [.103*math.cos(angle), .103*math.sin(angle), .116],
            "quaternion": q}}})
    runtime = {"passed": True, "scan": {"range_min": .05, "range_max": 8.},
        "tf_chain": {"base_link->laser_link": {"xyz": [.103, 0., -.034],
                                               "quaternion_xyzw": [0., 0., 0., 1.]}}}
    return {"ros_scans": scans, "aligned": aligned}, runtime


def settings():
    return {"slam_toolbox": {"ros__parameters": {"use_sim_time": True, "base_frame": "base_link",
        "odom_frame": "odom", "map_frame": "map", "scan_topic": "/scan",
        "correlation_search_space_smear_deviation": .1, "scan_buffer_size": 10}}}


class ReplayInputTests(unittest.TestCase):
    def test_reconstructs_base_from_measured_sensor_at_multiple_yaws(self):
        data, runtime = fixture()
        samples, _, meta = replay.prepare_input(data, runtime)
        self.assertEqual(meta["reconstructed_range_fields"]["range_min"], len(data["ros_scans"]))
        self.assertEqual(meta["missing_intensities_count"], len(data["ros_scans"]))
        for sample in samples:
            p = sample["base"]["position"]
            self.assertAlmostEqual(p[0], 0., places=12)
            self.assertAlmostEqual(p[1], 0., places=12)
            self.assertAlmostEqual(p[2], .15, places=12)
            self.assertEqual(sample["scan"]["range_max"], 8.)
            self.assertEqual(sample["scan"]["intensities"], [])

    def test_preserves_optional_intensities_and_nonfinite_ranges(self):
        data, runtime = fixture()
        for scan in data["ros_scans"].values():
            scan["ranges"] = [float("nan"), float("inf"), float("-inf")]
            scan["intensities"] = [1., 2., 3.]
        samples, _, meta = replay.prepare_input(data, runtime)
        self.assertEqual(meta["missing_intensities_count"], 0)
        for sample in samples:
            self.assertTrue(math.isnan(sample["scan"]["ranges"][0]))
            self.assertEqual(sample["scan"]["ranges"][1:], [float("inf"), float("-inf")])
            self.assertEqual(sample["scan"]["intensities"], [1., 2., 3.])
        next(iter(data["ros_scans"].values()))["intensities"] = [1.]
        with self.assertRaises(ValueError): replay.prepare_input(data, runtime)

    def test_native_world_fields_cannot_affect_selection(self):
        data, runtime = fixture()
        before = replay.prepare_input(data, runtime)
        data.update(native_scans={"bad": "not a scan"}, native_model_poses={"bad": "not a pose"},
                    world_truth=object())
        self.assertEqual(before, replay.prepare_input(data, runtime))

    def test_half_segments_partition_same_measured_input(self):
        data, runtime = fixture()
        full = replay.prepare_input(data, runtime)[0]
        first = replay.prepare_input(data, runtime, "first_half")[0]
        second = replay.prepare_input(data, runtime, "second_half")[0]
        self.assertEqual(first+second, full)
        self.assertLess(first[-1]["stamp_ns"], second[0]["stamp_ns"])

    def test_new_collector_works_without_map_transform(self):
        data, runtime = fixture()
        newer = {"raw": {"ros_scans": data["ros_scans"], "transforms": {
            str(row["stamp_ns"]): {"odom": row["tf"]["odom_to_laser_link"]} for row in data["aligned"]}}}
        self.assertEqual(replay.prepare_input(data, runtime)[0], replay.prepare_input(newer, runtime)[0])

    def test_rejects_wrong_frame_time_angles_and_nonunit_quaternion(self):
        for field, value in (("frame", "world"), ("time_increment", .001), ("angle_max", .5)):
            data, runtime = fixture()
            next(iter(data["ros_scans"].values()))[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                replay.prepare_input(data, runtime)
        data, runtime = fixture()
        data["aligned"][0]["tf"]["odom_to_laser_link"]["quaternion"] = [0, 0, 0, 0]
        with self.assertRaises(ValueError): replay.prepare_input(data, runtime)

    def test_rejects_stale_tf_and_contradictory_range_metadata(self):
        data, runtime = fixture()
        data["aligned"][0]["tf"]["odom_to_laser_link"]["stamp_ns"] -= 1
        with self.assertRaises(ValueError): replay.prepare_input(data, runtime)
        data, runtime = fixture()
        next(iter(data["ros_scans"].values()))["range_max"] = 12.
        with self.assertRaises(ValueError): replay.prepare_input(data, runtime)

    def test_rejects_no_motion_or_sparse_evidence(self):
        data, runtime = fixture()
        for row in data["aligned"]:
            row["tf"]["odom_to_laser_link"]["quaternion"] = [0., 0., 0., 1.]
        with self.assertRaises(ValueError): replay.prepare_input(data, runtime)
        data, runtime = fixture()
        for row in data["aligned"][12:18]: del data["ros_scans"][str(row["stamp_ns"])]
        with self.assertRaises(ValueError): replay.prepare_input(data, runtime)

    def test_only_one_typed_scalar_override_and_fixed_interface(self):
        configured = replay.configure(settings(), ["correlation_search_space_smear_deviation=0.01"])
        self.assertEqual(configured["slam_toolbox"]["ros__parameters"]["correlation_search_space_smear_deviation"], .01)
        for overrides in (["scan_buffer_size=11", "scan_buffer_size=12"], ["use_sim_time=false"],
                          ["scan_buffer_size=1.5"], ["unknown=1"], ["correlation_search_space_smear_deviation=NaN"]):
            with self.subTest(overrides=overrides), self.assertRaises(ValueError):
                replay.configure(settings(), overrides)

    def test_requires_domain75_and_localhost(self):
        replay.check_environment({"ROS_DOMAIN_ID": "75", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"})
        for values in ({}, {"ROS_DOMAIN_ID": "73", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST"},
                       {"ROS_DOMAIN_ID": "75", "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET"}):
            with self.assertRaises(ValueError): replay.check_environment(values)

    def test_early_failure_saves_manifest_and_never_overwrites(self):
        with tempfile.TemporaryDirectory(prefix=".slam-replay-test-", dir=Path(__file__).resolve().parent) as temp:
            self.assertTrue(Path(temp).resolve().is_relative_to(Path(__file__).resolve().parent))
            output = Path(temp)/"result"
            args = ["--input", "missing.json", "--runtime-evidence", "missing-runtime.json",
                    "--params", "missing.yaml", "--output-dir", str(output)]
            with patch.dict(replay.os.environ, {"ROS_DOMAIN_ID": "73"}), redirect_stdout(io.StringIO()):
                self.assertEqual(replay.main(args), 2)
            report = output/"result.json"
            self.assertIn("ROS_DOMAIN_ID", json.loads(report.read_text())["error"])
            original = report.read_bytes()
            self.assertEqual(replay.main(args), 2)
            self.assertEqual(report.read_bytes(), original)

    def test_runtime_error_and_keyboard_interrupt_leave_failure_evidence(self):
        data, runtime = fixture()
        with tempfile.TemporaryDirectory(prefix=".slam-replay-test-", dir=Path(__file__).resolve().parent) as temp:
            root = Path(temp)
            for name, value in (("input.json", data), ("runtime.json", runtime), ("params.yaml", settings())):
                (root/name).write_text(json.dumps(value))
            for index, error in enumerate((RuntimeError("injected callback failure"), KeyboardInterrupt())):
                output = root/f"failure-{index}"
                args = ["--input", str(root/"input.json"), "--runtime-evidence", str(root/"runtime.json"),
                        "--params", str(root/"params.yaml"), "--output-dir", str(output)]
                with patch.dict(replay.os.environ, {"ROS_DOMAIN_ID":"75", "ROS_AUTOMATIC_DISCOVERY_RANGE":"LOCALHOST"}), \
                     patch.dict(sys.modules, {"yaml": SimpleNamespace(safe_load=json.loads, safe_dump=json.dumps)}), \
                     patch.object(replay, "run_replay", side_effect=error), redirect_stdout(io.StringIO()):
                    self.assertEqual(replay.main(args), 2)
                result = json.loads((output/"result.json").read_text())
                self.assertEqual(result["status"], "error")
                self.assertIn(type(error).__name__, result["error"])
                self.assertIn("sha256", result["inputs"]["recording"])
                self.assertTrue((output/"params.yaml").is_file())


class OwnedProcessTests(unittest.TestCase):
    def test_launcher_exit_does_not_hide_remaining_child(self):
        process = Mock(pid=1234)
        process.poll.return_value = 0
        class Signals(IntEnum):
            SIGINT=2; SIGTERM=15; SIGKILL=9
        with patch.object(replay, "signal", Signals), \
             patch.object(replay, "group_members", side_effect=[[{"pid":1235,"state":"S"}], [], []]), \
             patch.object(replay.os, "killpg", create=True) as kill:
            result = replay.stop_owned_group(process, waits=(0, 0, 0))
        kill.assert_called_once_with(1234, signal.SIGINT)
        self.assertTrue(result["confirmed_empty"])

    def test_inspection_failure_does_not_skip_escalation_or_claim_cleanup(self):
        process = Mock(pid=1234)
        process.poll.return_value = 0
        class Signals(IntEnum):
            SIGINT=2; SIGTERM=15; SIGKILL=9
        with patch.object(replay, "signal", Signals), \
             patch.object(replay, "group_members", side_effect=RuntimeError("ps failed")), \
             patch.object(replay.os, "killpg", create=True) as kill:
            result = replay.stop_owned_group(process, waits=(0, 0, 0))
        self.assertEqual(kill.call_count, 3)
        self.assertFalse(result["confirmed_empty"])


if __name__ == "__main__":
    unittest.main()
