"""Offline safety tests. Fake transport only; no Gazebo or ROS Node is created."""
from contextlib import redirect_stdout
from dataclasses import replace
import io
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/limo_sim_bringup/scripts'))

import spawn_once as mod


GOOD = [{"request_type": mod.REQ_TYPE, "response_type": mod.REP_TYPE, "socket_id": "one"}]


class Clock:
    def __init__(self): self.t = 0.
    def now(self): return self.t
    def sleep(self, seconds): self.t += seconds


class Transport:
    def __init__(self, clock, discoveries=None, reply=(True, True), discover_cost=0., request_cost=0.):
        self.clock, self.reply = clock, reply
        self.discoveries = list(discoveries or [GOOD])
        self.discover_cost, self.request_cost = discover_cost, request_cost
        self.polls = 0
        self.calls = []
    def discover(self, service):
        self.clock.sleep(self.discover_cost)
        item = self.discoveries[min(self.polls, len(self.discoveries)-1)]
        self.polls += 1
        if isinstance(item, BaseException): raise item
        return item
    def create(self, service, spec, timeout_ms):
        self.calls.append((service, spec, timeout_ms))
        self.clock.sleep(self.request_cost)
        if isinstance(self.reply, BaseException): raise self.reply
        return self.reply


class SpawnTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.spec = mod.SpawnSpec("limo_maze", "limo", "/unused.urdf", '<robot name="limo"/>',
                                  "fakehash", (1., 2., 3., .1, .2, .3),
                                  discovery_seconds=1., response_seconds=.5, total_seconds=1.5)
        self.intents = []
    def run_it(self, transport, persist=None, spec=None):
        result = mod.run_once(spec or self.spec, transport, monotonic=self.clock.now,
                              sleep=self.clock.sleep, persist_intent=persist or self.intents.append)
        self.assertLessEqual(len(transport.calls), 1)
        self.assertFalse(result["motion_authorized"])
        self.assertFalse(result["unique_instance_verified"])
        return result
    def test_discovery_waits_then_exactly_one_request(self):
        t = Transport(self.clock, [[], [], GOOD])
        r = self.run_it(t)
        self.assertTrue(r["success"])
        self.assertEqual(t.polls, 3)
        self.assertEqual(len(t.calls), 1)
        self.assertEqual(len(self.intents), 1)
        self.assertEqual(t.calls[0][0], "/world/limo_maze/create")
    def test_absent_service_exhausts_discovery_without_request(self):
        t = Transport(self.clock, [[]])
        r = self.run_it(t)
        self.assertEqual(r["status"], "discovery_deadline")
        self.assertEqual(t.calls, [])
        self.assertAlmostEqual(self.clock.t, 1.)
        self.assertEqual(self.intents, [])
    def test_ambiguous_providers_fail_before_create(self):
        t = Transport(self.clock, [GOOD+GOOD])
        self.assertEqual(self.run_it(t)["status"], "ambiguous_service_publishers")
        self.assertEqual(t.calls, [])
    def test_wrong_service_type_fails_before_create(self):
        t = Transport(self.clock, [[{"request_type": "wrong", "response_type": mod.REP_TYPE}]])
        self.assertEqual(self.run_it(t)["status"], "service_type_mismatch")
        self.assertEqual(t.calls, [])
    def test_false_transport_result_is_unknown_and_never_retried(self):
        t = Transport(self.clock, reply=(False, False))
        r = self.run_it(t)
        self.assertFalse(r["success"])
        self.assertEqual(r["create_outcome"], "unknown")
        self.assertEqual(r["request_attempts"], 1)
        self.assertEqual(len(t.calls), 1)
    def test_negative_reply_fails_without_retry(self):
        t = Transport(self.clock, reply=(True, False))
        r = self.run_it(t)
        self.assertEqual(r["status"], "create_rejected")
        self.assertEqual(r["create_outcome"], "negative_reply")
        self.assertEqual(len(t.calls), 1)
    def test_exception_after_attempt_is_unknown_not_retryable(self):
        t = Transport(self.clock, reply=RuntimeError("reply lost"))
        r = self.run_it(t)
        self.assertEqual(r["status"], "exception_after_attempt")
        self.assertEqual(r["create_outcome"], "unknown")
        self.assertEqual(len(t.calls), 1)
    def test_keyboard_interrupt_after_attempt_is_unknown(self):
        t = Transport(self.clock, reply=KeyboardInterrupt())
        r = self.run_it(t)
        self.assertEqual(r["status"], "exception_after_attempt")
        self.assertEqual(r["create_outcome"], "unknown")
    def test_intent_persistence_failure_prevents_request(self):
        def fail(_): raise OSError("disk full")
        t = Transport(self.clock)
        r = self.run_it(t, persist=fail)
        self.assertEqual(r["request_attempts"], 0)
        self.assertEqual(t.calls, [])
        self.assertEqual(r["create_outcome"], "not_sent")
    def test_intent_exists_before_transport_request(self):
        t = Transport(self.clock)
        original = t.create
        def verify(*args):
            self.assertEqual(len(self.intents), 1)
            return original(*args)
        t.create = verify
        self.assertTrue(self.run_it(t)["success"])
    def test_total_budget_clamps_request_timeout(self):
        t = Transport(self.clock, discover_cost=.75)
        r = self.run_it(t, spec=replace(self.spec, total_seconds=1.))
        self.assertTrue(r["success"])
        self.assertGreater(t.calls[0][2], 0)
        self.assertLessEqual(t.calls[0][2], 250)
    def test_slow_discovery_cannot_send_after_deadline(self):
        t = Transport(self.clock, discover_cost=1.2)
        self.assertEqual(self.run_it(t)["status"], "discovery_deadline")
        self.assertEqual(t.calls, [])
    def test_slow_intent_write_cannot_send_after_total_deadline(self):
        t = Transport(self.clock)
        def slow(_): self.clock.sleep(2.)
        r = self.run_it(t, persist=slow)
        self.assertEqual(r["status"], "total_deadline_after_intent")
        self.assertEqual(t.calls, [])
    def test_late_positive_response_not_reported_success(self):
        t = Transport(self.clock, request_cost=.6)
        r = self.run_it(t)
        self.assertFalse(r["success"])
        self.assertEqual(r["status"], "response_deadline_after_request")
        self.assertEqual(r["create_outcome"], "unknown")
    def test_truthy_non_bool_result_is_not_accepted(self):
        t = Transport(self.clock, reply=("false", True))
        self.assertEqual(self.run_it(t)["status"], "malformed_transport_result")
    def test_discovery_exception_prevents_request(self):
        t = Transport(self.clock, [OSError("discovery failed")])
        r = self.run_it(t)
        self.assertEqual(r["status"], "exception_before_attempt")
        self.assertEqual(t.calls, [])
    def test_request_contains_exact_xml_name_false_renaming_and_pose(self):
        def entity():
            return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(), orientation=SimpleNamespace()))
        req = mod.build_request(self.spec, entity)
        self.assertEqual(req.sdf, self.spec.model_xml)
        self.assertEqual(req.name, "limo")
        self.assertIs(req.allow_renaming, False)
        self.assertEqual((req.pose.position.x, req.pose.position.y, req.pose.position.z), (1.,2.,3.))
        q = req.pose.orientation
        self.assertAlmostEqual(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w, 1.)
        q0 = mod.quaternion((0.,0.,math.pi/2))
        self.assertAlmostEqual(q0[2], math.sqrt(.5))
        self.assertAlmostEqual(q0[3], math.sqrt(.5))


class InputAndCliTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="test-", dir=Path(__file__).parent)
        self.root = Path(self.tmp.name)
        self.addCleanup(self.tmp.cleanup)
        (self.root/"model.urdf").write_text('<robot name="limo"/>')
        self.path = self.root/"params.json"
        self.params = {"world": "limo_maze", "name": "limo", "model_file": "model.urdf",
                       "allow_renaming": False,
                       "pose": {k: 0. for k in ("x", "y", "z", "roll", "pitch", "yaw")}}
    def save(self): self.path.write_text(json.dumps(self.params))
    def test_bad_renaming_pose_and_names_are_rejected(self):
        for key, value in (("allow_renaming", True), ("name", "../limo"),
                           ("world", ""), ("response_seconds", float("nan"))):
            with self.subTest(key=key):
                old = dict(self.params)
                self.params[key] = value
                self.save()
                with self.assertRaises(ValueError): mod.load_spec(self.path)
                self.params = old
        self.params["pose"]["x"] = float("inf")
        self.save()
        with self.assertRaises(ValueError): mod.load_spec(self.path)
    def test_unexpanded_xacro_is_rejected(self):
        (self.root/"model.urdf").write_text('<robot name="limo"><link name="${foo}"/></robot>')
        self.save()
        with self.assertRaises(ValueError): mod.load_spec(self.path)
    def test_validated_only_does_not_construct_real_transport(self):
        self.save()
        def forbidden(): raise AssertionError("transport must not be constructed")
        with redirect_stdout(io.StringIO()):
            code = mod.main(["--params-file", str(self.path), "--report-dir", str(self.root/"report")],
                            transport_factory=forbidden)
        self.assertEqual(code, 0)
        self.assertEqual(json.loads((self.root/"report/report.json").read_text())["request_attempts"], 0)
    def test_same_report_directory_refuses_rerun_before_transport_construction(self):
        self.save()
        calls = []
        def factory():
            calls.append(True)
            return Transport(Clock())
        argv = ["--params-file", str(self.path), "--report-dir", str(self.root/"report"), "--execute"]
        with redirect_stdout(io.StringIO()): mod.main(argv, transport_factory=factory)
        with self.assertRaises(FileExistsError): mod.main(argv, transport_factory=factory)
        self.assertEqual(calls, [True])
        self.assertTrue((self.root/"report/attempt-intent.json").is_file())


if __name__ == "__main__":
    unittest.main(verbosity=2)
