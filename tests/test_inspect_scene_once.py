"""No network: fake scene transport and payloads only."""
from types import SimpleNamespace as NS
import unittest

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/limo_sim_bringup/scripts'))

import inspect_scene_once as mod
from test_spawn_once import Clock


def model(name, id, nested=(), deleted=False):
    return NS(name=name, id=id, model=nested, deleted=deleted)


def scene(*models, name="limo_maze"):
    return NS(name=name, model=models, SerializeToString=lambda: b"offline fake scene")


class FakeTransport:
    def __init__(self, result=True, payload=None, providers=1):
        self.result = result
        self.payload = payload or scene(model("limo", 17))
        self.calls = []
        self.providers = providers
    def discover(self, service):
        return [{"request_type": mod.SCENE_REQ, "response_type": mod.SCENE_REP,
                 "socket_id": str(i)} for i in range(self.providers)]
    def query(self, service, timeout_ms):
        self.calls.append((service, timeout_ms))
        return self.result, self.payload


class Tests(unittest.TestCase):
    def query(self, transport):
        clock = Clock()
        return mod.query_once(transport, "limo_maze", "limo", discovery_seconds=.3,
                              response_seconds=.3, total_seconds=.6,
                              monotonic=clock.now, sleep=clock.sleep)
    def test_single_named_model_unique_id_passes_snapshot_only(self):
        t = FakeTransport(payload=scene(model("wall", 8), model("limo", 17)))
        r = self.query(t)
        self.assertTrue(r["passed"])
        self.assertFalse(r["motion_authorized"])
        self.assertEqual(r["target_models"][0]["id"], 17)
        self.assertEqual(len(t.calls), 1)
        self.assertEqual(t.calls[0][0], "/world/limo_maze/scene/info")
    def test_same_name_distinct_ids_proves_multiple_instances(self):
        r = self.query(FakeTransport(payload=scene(model("limo", 17), model("limo", 23))))
        self.assertFalse(r["passed"])
        self.assertEqual(r["target_count"], 2)
    def test_repeated_id_is_ambiguous_not_unique(self):
        r = self.query(FakeTransport(payload=scene(model("limo", 17), model("other", 17))))
        self.assertFalse(r["passed"])
        self.assertIn("repeated_model_id_in_scene", r["reasons"])
    def test_zero_named_models_fails(self):
        self.assertFalse(self.query(FakeTransport(payload=scene(model("wall", 5))))["passed"])
    def test_deleted_model_does_not_satisfy_live_uniqueness(self):
        r = self.query(FakeTransport(payload=scene(model("limo", 17, deleted=True))))
        self.assertFalse(r["passed"])
        self.assertEqual(r["target_count"], 0)
    def test_nested_same_name_is_counted_not_hidden(self):
        r = self.query(FakeTransport(payload=scene(model("limo", 17, (model("limo", 23),)))))
        self.assertFalse(r["passed"])
        self.assertEqual(r["target_count"], 2)
    def test_wrong_world_fails(self):
        self.assertFalse(self.query(FakeTransport(payload=scene(model("limo",17), name="other")))["passed"])
    def test_ambiguous_publishers_prevent_query(self):
        t = FakeTransport(providers=2)
        r = self.query(t)
        self.assertEqual(r["status"], "ambiguous_service_publishers")
        self.assertEqual(t.calls, [])
    def test_absent_service_exhausts_budget_no_query(self):
        t = FakeTransport(providers=0)
        self.assertEqual(self.query(t)["status"], "discovery_deadline")
        self.assertEqual(t.calls, [])
    def test_false_transport_result_fails_without_retry(self):
        t = FakeTransport(result=False)
        r = self.query(t)
        self.assertFalse(r["passed"])
        self.assertEqual(r["status"], "transport_failed_or_timed_out")
        self.assertEqual(len(t.calls), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
