"""Offline lifecycle regressions: fake readers only, no ROS/Gazebo imports/nodes."""
from contextlib import redirect_stdout
import importlib.util
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/capture_motion_evidence.py"
spec = importlib.util.spec_from_file_location("capture_motion_evidence", SCRIPT)
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def messages(index=1, moving=True):
    q = NS(x=0., y=0., z=0., w=1.)
    p = NS(x=.01 * index, y=0., z=0.)
    pose = NS(position=p, orientation=q)
    stamp = NS(sec=1, nanosec=index * 100_000_000, nsec=index * 100_000_000)
    native = NS(header=NS(stamp=stamp), pose=[NS(name="limo", id=17, **vars(pose))])
    odom = NS(header=NS(stamp=stamp), pose=NS(pose=pose), twist=NS(twist=NS(
        angular=NS(z=.1 if moving else 0.), linear=NS(x=0., y=0.))))
    return native, odom


class MissingTransform(Exception):
    pass


class FakeRuntime:
    """Same read-side surface as the script, plus deliberate lifecycle faults."""
    def __init__(self, mode="normal", output=None, *, abort_on_unsubscribe=False):
        self.mode, self.output = mode, output
        self.abort_on_unsubscribe = abort_on_unsubscribe
        self.active, self.now, self.spins = False, 0., 0
        self.order, self.shutdown_calls, self.init_options = [], 0, None
        self.native_callback = self.odom_callback = None
        self.runtime = NS(rclpy=self, SignalHandlerOptions=NS(NO="disabled"),
                          Parameter=lambda *a, **k: None, Duration=lambda **k: None,
                          Time=lambda **k: NS(**k), Odometry=object, Pose_V=object,
                          qos_profile_sensor_data=object(), TransformException=MissingTransform,
                          Buffer=self.make_buffer, TransformListener=self.make_listener,
                          GzNode=self.make_native)

    def init(self, **kwargs):
        self.init_options = kwargs
        self.active = True

    def ok(self):
        return self.active

    def shutdown(self):
        if not self.active:
            raise AssertionError("duplicate shutdown")
        self.order.append("ros_shutdown")
        self.shutdown_calls += 1
        self.active = False

    def create_node(self, *args, **kwargs):
        def subscribe(_kind, _topic, cb, _qos):
            self.odom_callback = cb
        return NS(create_subscription=subscribe, destroy_node=lambda: self.order.append("node_destroy"))

    def make_buffer(self, **kwargs):
        def lookup(*args):
            self.order.append("lookup")
            if self.mode == "alignment_error":
                raise RuntimeError("alignment failed")
            if self.mode == "missing_tf":
                raise MissingTransform("no map TF yet")
            return NS(transform=NS(rotation=NS(x=0., y=0., z=0., w=1.)))
        return NS(lookup_transform=lookup)

    def make_listener(self, *args):
        def unregister():
            self.order.append("listener_unregister")
            if self.mode == "cleanup_error":
                raise RuntimeError("listener cleanup failed")
        return NS(unregister=unregister)

    def make_native(self):
        runtime = self

        class Native:
            def subscribe(self, _kind, _topic, cb):
                runtime.native_callback = cb
                return runtime.mode != "subscription_failed"

            def unsubscribe(self, _topic):
                runtime.order.append("native_unsubscribe")
                # Evidence must already be complete JSON before native cleanup.
                saved = json.loads(runtime.output.read_text())
                assert saved["cleanup"]["status"] == "raw_snapshot_saved_before_native_cleanup"
                assert saved["partial"] is True
                assert saved["world_samples"]
                runtime.saved_at_unsubscribe = saved
                # Simulate a callback racing with unsubscribe; frozen report must not change.
                runtime.native_callback(messages(9)[0])
                if runtime.abort_on_unsubscribe:
                    os._exit(71)
                if runtime.mode == "unsubscribe_error":
                    raise RuntimeError("native unsubscribe failed")
                return True

            def __del__(self):
                runtime.order.append("native_release")

        return Native()

    def spin_once(self, node, timeout_sec):
        self.spins += 1
        self.now += .1
        native, odom = messages(self.spins, moving=self.mode != "no_motion")
        self.native_callback(native)
        self.odom_callback(odom)
        if self.spins == 2:
            if self.mode in ("sigint", "sigterm"):
                signal.raise_signal(signal.SIGINT if self.mode == "sigint" else signal.SIGTERM)
            elif self.mode == "keyboard_interrupt":
                raise KeyboardInterrupt()
            elif self.mode == "spin_error":
                raise RuntimeError("spin failed")
            elif self.mode == "native_callback_error":
                self.native_callback(NS())
            elif self.mode == "odom_callback_error":
                self.odom_callback(NS())
            elif self.mode == "context_stopped":
                self.active = False


class CaptureTests(unittest.TestCase):
    def run_capture(self, mode="normal"):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "initial-motion.json"
            rt = FakeRuntime(mode, output)
            previous = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
            with redirect_stdout(io.StringIO()):
                code = capture.main(["--duration", ".2", "--motion-wait", ".4", "--output", str(output)],
                                    runtime_factory=lambda: rt.runtime, monotonic=lambda: rt.now)
            report = json.loads(output.read_text())
            self.assertEqual(previous, {s: signal.getsignal(s) for s in previous})
            self.assertEqual(rt.init_options, {"signal_handler_options": "disabled"})
            self.assertFalse(report["automatic_pass_verdict"])
            self.assertTrue(report["read_only"])
            return code, report, rt

    def test_normal_window_preserves_legacy_samples_and_alignment(self):
        code, report, rt = self.run_capture()
        self.assertEqual(code, 0)
        self.assertFalse(report["partial"])
        self.assertEqual(report["reason"], "duration_complete")
        self.assertGreaterEqual(len(report["world_samples"]), 3)
        self.assertEqual(report["aligned_count"], len(report["world_samples"]))
        self.assertEqual(report["odom_summary"]["count"], len(report["odom_samples"]))
        self.assertEqual(rt.shutdown_calls, 1)
        self.assertEqual(report["cleanup"]["status"], "complete")
        self.assertLess(rt.order.index("native_unsubscribe"), rt.order.index("lookup"))
        self.assertLess(rt.order.index("native_release"), rt.order.index("listener_unregister"))
        self.assertLess(rt.order.index("listener_unregister"), rt.order.index("ros_shutdown"))
        self.assertLess(report["world_samples"][-1]["sim_s"], 1.9)

    def test_sigint_and_sigterm_preserve_both_streams_as_partial(self):
        for mode in ("sigint", "sigterm"):
            with self.subTest(mode=mode):
                code, report, rt = self.run_capture(mode)
                self.assertEqual(code, 2)
                self.assertTrue(report["partial"])
                self.assertEqual(report["reason"], "signal_interrupt")
                self.assertEqual(report["requested_signal"], mode.upper())
                self.assertEqual(len(report["world_samples"]), 2)
                self.assertEqual(len(report["odom_samples"]), 2)
                self.assertEqual(rt.shutdown_calls, 1)

    def test_sampling_exceptions_keep_preceding_samples(self):
        for mode, reason in (("spin_error", "sampling_error"), ("keyboard_interrupt", "keyboard_interrupt"),
                             ("native_callback_error", "sampling_error"), ("odom_callback_error", "sampling_error")):
            with self.subTest(mode=mode):
                code, report, rt = self.run_capture(mode)
                self.assertEqual(code, 2)
                self.assertTrue(report["partial"])
                self.assertEqual(report["reason"], reason)
                self.assertIsNotNone(report["error"])
                self.assertEqual(len(report["world_samples"]), 2)
                self.assertEqual(len(report["odom_samples"]), 2)
                self.assertEqual(rt.shutdown_calls, 1)

    def test_alignment_and_cleanup_errors_do_not_erase_raw_data(self):
        for mode in ("alignment_error", "cleanup_error", "unsubscribe_error"):
            with self.subTest(mode=mode):
                code, report, rt = self.run_capture(mode)
                self.assertEqual(code, 2)
                self.assertTrue(report["partial"])
                self.assertIsNotNone(report["error"])
                self.assertGreaterEqual(len(report["world_samples"]), 3)
                self.assertEqual(rt.shutdown_calls, 1)

    def test_already_stopped_context_is_not_shutdown_twice(self):
        code, report, rt = self.run_capture("context_stopped")
        self.assertEqual(code, 2)
        self.assertEqual(report["reason"], "ros_context_stopped")
        self.assertEqual(rt.shutdown_calls, 0)
        self.assertFalse(report["cleanup"]["ros_shutdown_called"])

    def test_motion_wait_timeout_is_not_full_motion_window(self):
        code, report, _ = self.run_capture("no_motion")
        self.assertEqual(code, 2)
        self.assertEqual(report["reason"], "motion_wait_timeout")
        self.assertFalse(report["motion_detected"])

    def test_missing_tf_is_retained_as_null_without_claiming_verdict(self):
        code, report, _ = self.run_capture("missing_tf")
        self.assertEqual(code, 0)
        self.assertTrue(report["aligned_samples"])
        self.assertIsNone(report["aligned_samples"][0]["map_base_yaw"])

    def test_failed_subscription_saves_empty_partial_and_releases_resources(self):
        code, report, rt = self.run_capture("subscription_failed")
        self.assertEqual(code, 2)
        self.assertEqual(report["world_samples"], [])
        self.assertIn("Could not subscribe", report["error"])
        self.assertNotIn("native_unsubscribe", rt.order)
        self.assertEqual(rt.shutdown_calls, 1)

    def test_import_failure_still_creates_valid_partial_report(self):
        def unavailable():
            raise ImportError("deliberately unavailable runtime")
        with tempfile.TemporaryDirectory() as tmp, redirect_stdout(io.StringIO()):
            output = Path(tmp) / "initial-motion.json"
            code = capture.main(["--output", str(output)], runtime_factory=unavailable)
            report = json.loads(output.read_text())
        self.assertEqual(code, 2)
        self.assertIn("ImportError", report["error"])
        self.assertEqual(report["world_summary"], {"count": 0})
        self.assertFalse(report["cleanup"]["ros_shutdown_called"])

    def test_existing_output_refused_without_loading_runtime(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "initial-motion.json"
            output.write_text("original")
            with self.assertRaises(SystemExit):
                capture.main(["--output", str(output)], runtime_factory=lambda: self.fail("must not import"))
            self.assertEqual(output.read_text(), "original")

    def test_abrupt_native_unsubscribe_exit_leaves_parseable_raw_snapshot(self):
        for mode in ("sigint", "normal"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as tmp:
                output = Path(tmp) / "initial-motion.json"
                code = ("import sys; from pathlib import Path; "
                        f"sys.path.insert(0, {str(Path(__file__).parent)!r}); "
                        "from test_capture_motion_evidence import capture, FakeRuntime; "
                        f"p=Path({str(output)!r}); r=FakeRuntime({mode!r}, p, abort_on_unsubscribe=True); "
                        "capture.main(['--duration', '.2', '--output', str(p)], runtime_factory=lambda:r.runtime, monotonic=lambda:r.now)")
                result = subprocess.run([sys.executable, "-B", "-c", code], capture_output=True, timeout=10)
                self.assertEqual(result.returncode, 71, result.stderr.decode())
                report = json.loads(output.read_text())
                self.assertEqual(report["reason"], "signal_interrupt" if mode == "sigint" else "duration_complete")
                self.assertTrue(report["partial"])
                self.assertGreaterEqual(len(report["world_samples"]), 2)
                self.assertGreaterEqual(len(report["odom_samples"]), 2)
                self.assertEqual(report["cleanup"]["status"], "raw_snapshot_saved_before_native_cleanup")


if __name__ == "__main__":
    unittest.main()
