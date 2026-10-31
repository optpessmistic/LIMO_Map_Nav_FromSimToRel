#!/usr/bin/python3
"""Read-only native scene snapshot. Never sends create, goals, or velocity.

The result proves only the returned snapshot's model identity count. It does not
prove future uniqueness or validate odometry; those need independent observation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

from spawn_once import NAME, positive, write_new_json


SCENE_REQ = "gz.msgs.Empty"
SCENE_REP = "gz.msgs.Scene"


class SceneTransport:
    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.scene_pb2 import Scene
        from gz.msgs10.empty_pb2 import Empty
        self.node = Node()
        self.request_type, self.response_type = Empty, Scene

    def discover(self, service):
        if service not in self.node.service_list():
            return []
        def field(info, name):
            value = getattr(info, name)
            return value() if callable(value) else value
        return [{"request_type": field(info, "req_type_name"),
                 "response_type": field(info, "rep_type_name"),
                 "socket_id": field(info, "socket_id")}
                for info in self.node.service_info(service)]

    def query(self, service, timeout_ms):
        return self.node.request(service, self.request_type(), self.request_type,
                                 self.response_type, timeout_ms)


def inspect_payload(scene, world, name):
    rows = []
    def visit(models, ancestors):
        if len(ancestors) > 32:
            raise ValueError("scene model nesting exceeds audit bound")
        for model in models:
            path = (*ancestors, model.name)
            rows.append({"name": model.name, "id": int(model.id),
                         "path": list(path), "deleted": bool(model.deleted)})
            if len(rows) > 10000:
                raise ValueError("scene model count exceeds audit bound")
            visit(model.model, path)
    visit(scene.model, ())
    target = [r for r in rows if r["name"] == name and not r["deleted"]]
    ids = [r["id"] for r in rows if not r["deleted"]]
    reasons = []
    if scene.name and scene.name != world:
        reasons.append("scene_world_name_mismatch")
    if len(ids) != len(set(ids)):
        reasons.append("repeated_model_id_in_scene")
    if len(target) != 1:
        reasons.append("expected_exactly_one_named_model")
    elif target[0]["id"] <= 0:
        reasons.append("target_model_id_not_positive")
    elif len(target[0]["path"]) != 1:
        reasons.append("target_is_not_top_level_model")
    return {"scene_name": scene.name, "models": rows, "target_models": target,
            "target_count": len(target), "reasons": reasons,
            "unique_instance_verified_in_snapshot": not reasons,
            "scope": "one native scene response; does not establish future uniqueness"}


def query_once(transport, world, name, *, discovery_seconds=30., response_seconds=10.,
               total_seconds=40., monotonic=time.monotonic, sleep=time.sleep):
    if not NAME.fullmatch(world) or not NAME.fullmatch(name):
        raise ValueError("invalid world or model name")
    for key, value in (("discovery_seconds", discovery_seconds),
                       ("response_seconds", response_seconds), ("total_seconds", total_seconds)):
        positive(value, key, 420)
    start = monotonic()
    deadline = start+total_seconds
    discovery_deadline = min(deadline, start+discovery_seconds)
    service = f"/world/{world}/scene/info"
    report = {"schema_version": 1, "passed": False, "read_only": True,
              "motion_authorized": False, "service": service,
              "query_attempts": 0, "unique_instance_verified_in_snapshot": False,
              "status": "not_started", "service_publishers": []}
    try:
        while monotonic() < discovery_deadline:
            publishers = transport.discover(service)
            if monotonic() >= discovery_deadline:
                report["status"] = "discovery_deadline"
                return report
            if publishers:
                report["service_publishers"] = publishers
                if len(publishers) != 1:
                    report["status"] = "ambiguous_service_publishers"
                    return report
                if publishers[0].get("request_type") != SCENE_REQ or publishers[0].get("response_type") != SCENE_REP:
                    report["status"] = "service_type_mismatch"
                    return report
                break
            sleep(min(.1, max(0, discovery_deadline-monotonic())))
        else:
            report["status"] = "discovery_deadline"
            return report
        timeout_ms = int(min(response_seconds, deadline-monotonic())*1000)
        if timeout_ms < 1:
            report["status"] = "total_deadline_before_query"
            return report
        response_deadline = monotonic()+timeout_ms/1000
        report["query_attempts"] = 1
        report["request_timeout_ms"] = timeout_ms
        result, scene = transport.query(service, timeout_ms)
        if monotonic() > min(deadline, response_deadline):
            report["status"] = "response_deadline"
        elif result is not True:
            report["status"] = "transport_failed_or_timed_out"
        else:
            report.update(inspect_payload(scene, world, name))
            report["payload_sha256"] = hashlib.sha256(scene.SerializeToString()).hexdigest()
            report["passed"] = report["unique_instance_verified_in_snapshot"]
            report["status"] = "unique_model_in_snapshot" if report["passed"] else "scene_identity_check_failed"
        return report
    except (Exception, KeyboardInterrupt) as exc:
        report["status"] = "query_exception"
        report["error"] = f"{type(exc).__name__}: {exc}"
        return report
    finally:
        report["elapsed_wall_seconds"] = monotonic()-start


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True)
    parser.add_argument("--name", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--execute", action="store_true", help="perform read-only native service discovery and one scene query")
    args = parser.parse_args()
    if not args.execute:
        parser.error("pass --execute to perform the explicitly read-only query")
    # Reserve output before creating any transport object; refuse overwritten evidence.
    with args.output.open("x", encoding="utf-8") as output:
        try:
            report = query_once(SceneTransport(), args.world, args.name)
        except (Exception, KeyboardInterrupt) as exc:
            report = {"passed": False, "read_only": True, "query_attempts": 0,
                      "status": "initialization_failed", "error": f"{type(exc).__name__}: {exc}"}
        json.dump(report, output, ensure_ascii=False, indent=2, allow_nan=False)
        output.write("\n")
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
