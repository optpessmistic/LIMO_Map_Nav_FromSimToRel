#!/usr/bin/python3
"""Bounded discovery, at most one Gazebo create request.

Importing this module never imports Gazebo, constructs a Node, or sends traffic.
Only the explicit --execute CLI path constructs a transport Node. A synchronous
transport timeout means unknown outcome, not evidence that creation did not run.
An outer owner must enforce a hard process wall deadline if native calls stall.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import time
from typing import Callable
import xml.etree.ElementTree as ET


REQ_TYPE = "gz.msgs.EntityFactory"
REP_TYPE = "gz.msgs.Boolean"
NAME = re.compile(r"[A-Za-z][A-Za-z0-9_-]{0,127}\Z")


@dataclass(frozen=True)
class SpawnSpec:
    world: str
    name: str
    model_file: str
    model_xml: str
    model_sha256: str
    pose: tuple[float, float, float, float, float, float]
    discovery_seconds: float = 120.0
    response_seconds: float = 30.0
    total_seconds: float = 150.0
    poll_seconds: float = 0.1

    @property
    def service(self):
        return f"/world/{self.world}/create"

    def metadata(self):
        return {"world": self.world, "name": self.name,
                "model_file": self.model_file, "model_sha256": self.model_sha256,
                "pose_xyz_rpy": list(self.pose), "allow_renaming": False,
                "service": self.service, "discovery_seconds": self.discovery_seconds,
                "response_seconds": self.response_seconds,
                "total_seconds": self.total_seconds, "poll_seconds": self.poll_seconds}


def positive(value, name, upper):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite positive number")
    if not math.isfinite(value) or not 0 < value <= upper:
        raise ValueError(f"{name} must be > 0 and <= {upper}")
    return float(value)


def load_spec(path: Path):
    raw = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("parameter file must contain a JSON object")
    allowed = {"world", "name", "model_file", "pose", "allow_renaming",
               "discovery_seconds", "response_seconds", "total_seconds", "poll_seconds"}
    if set(raw) - allowed:
        raise ValueError(f"unknown parameters: {sorted(set(raw)-allowed)}")
    for field in ("world", "name"):
        if not isinstance(raw.get(field), str) or not NAME.fullmatch(raw[field]):
            raise ValueError(f"invalid {field}: use a simple explicit Gazebo name")
    if raw.get("allow_renaming") is not False:
        raise ValueError("allow_renaming must be explicitly false")
    if not isinstance(raw.get("model_file"), str) or not raw["model_file"]:
        raise ValueError("model_file is required")
    model_path = (path.parent / raw["model_file"]).resolve(strict=True)
    if not model_path.is_file() or model_path.stat().st_size > 5 * 1024**2:
        raise ValueError("model_file must be a regular file of at most 5 MiB")
    model_bytes = model_path.read_bytes()
    xml = model_bytes.decode("utf-8")
    root = ET.fromstring(xml)
    if root.tag == "robot":
        pass
    elif root.tag == "sdf" and len(root) == 1 and root[0].tag == "model":
        pass
    else:
        raise ValueError("expected an expanded URDF robot or an SDF with one top-level model")
    if any("xacro" in tag.tag for tag in root.iter()) or "${" in xml or "$(" in xml:
        raise ValueError("expand xacro/substitutions before invoking the spawn helper")
    pose = raw.get("pose")
    fields = ("x", "y", "z", "roll", "pitch", "yaw")
    if not isinstance(pose, dict) or set(pose) != set(fields):
        raise ValueError("pose must explicitly define x,y,z,roll,pitch,yaw")
    if any(isinstance(pose[k], bool) or not isinstance(pose[k], (int, float))
           or not math.isfinite(pose[k]) for k in fields):
        raise ValueError("all pose values must be finite numbers")
    budgets = {k: positive(raw.get(k, default), k, cap) for k, default, cap in (
        ("discovery_seconds", 120, 300), ("response_seconds", 30, 120),
        ("total_seconds", 150, 420), ("poll_seconds", .1, 1))}
    if min(budgets["discovery_seconds"], budgets["response_seconds"],
           budgets["total_seconds"]) < .001:
        raise ValueError("time budgets must allow at least one millisecond")
    return SpawnSpec(raw["world"], raw["name"], str(model_path), xml,
                     hashlib.sha256(model_bytes).hexdigest(),
                     tuple(float(pose[k]) for k in fields), **budgets)


def quaternion(rpy):
    r, p, y = (v / 2 for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def build_request(spec, entity_type):
    request = entity_type()
    # Embed exactly the bytes inspected above, avoiding a server-side file race.
    request.sdf = spec.model_xml
    request.name = spec.name
    request.allow_renaming = False
    request.pose.position.x, request.pose.position.y, request.pose.position.z = spec.pose[:3]
    q = quaternion(spec.pose[3:])
    (request.pose.orientation.x, request.pose.orientation.y,
     request.pose.orientation.z, request.pose.orientation.w) = q
    return request


class GazeboTransport:
    """Real adapter: instantiate only in an explicitly authorized CLI run."""
    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.entity_factory_pb2 import EntityFactory
        from gz.msgs10.boolean_pb2 import Boolean
        self.node = Node()
        self.entity_type, self.boolean_type = EntityFactory, Boolean

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

    def create(self, service, spec, timeout_ms):
        result, reply = self.node.request(service, build_request(spec, self.entity_type),
                                          self.entity_type, self.boolean_type, timeout_ms)
        return result, reply.data


def run_once(spec: SpawnSpec, transport, *, monotonic=time.monotonic,
             sleep=time.sleep, persist_intent: Callable[[dict], None]):
    start = monotonic()
    deadline = start + spec.total_seconds
    discovery_deadline = min(deadline, start + spec.discovery_seconds)
    report = {"schema_version": 1, "success": False, "status": "not_started",
              "request_attempts": 0, "create_outcome": "not_sent",
              "discovery_polls": 0, "service_publishers": [],
              "unique_instance_verified": False, "motion_authorized": False,
              "request_retry_allowed": False, "input": spec.metadata(),
              "deadline_enforcement": "cooperative discovery and native request timeout; outer hard process deadline required"}
    try:
        while monotonic() < discovery_deadline:
            report["discovery_polls"] += 1
            publishers = transport.discover(spec.service)
            if monotonic() >= discovery_deadline:
                report["status"] = "discovery_deadline"
                return report
            if publishers:
                report["service_publishers"] = publishers
                if len(publishers) != 1:
                    report["status"] = "ambiguous_service_publishers"
                    return report
                pub = publishers[0]
                if pub.get("request_type") != REQ_TYPE or pub.get("response_type") != REP_TYPE:
                    report["status"] = "service_type_mismatch"
                    return report
                break
            sleep(min(spec.poll_seconds, max(0, discovery_deadline-monotonic())))
        else:
            report["status"] = "discovery_deadline"
            return report
        timeout_ms = int(min(spec.response_seconds, deadline-monotonic())*1000)
        if timeout_ms < 1:
            report["status"] = "total_deadline_before_request"
            return report
        # This persistent intent must succeed BEFORE the sole mutating call.
        # If the process dies afterward, the outcome must remain unknown.
        persist_intent({"schema_version": 1, "service": spec.service,
                        "model_sha256": spec.model_sha256, "name": spec.name,
                        "allow_renaming": False, "request_attempt_upper_bound": 1,
                        "create_outcome": "unknown_until_final_report",
                        "request_retry_allowed": False, "timeout_ms": timeout_ms})
        if monotonic() >= deadline:
            report["status"] = "total_deadline_after_intent"
            return report
        # Recompute after durable I/O so writing evidence cannot overrun the budget.
        timeout_ms = int(min(spec.response_seconds, deadline-monotonic())*1000)
        if timeout_ms < 1:
            report["status"] = "total_deadline_before_request"
            return report
        report["request_timeout_ms"] = timeout_ms
        report["request_attempts"] = 1
        report["create_outcome"] = "unknown"
        response_deadline = monotonic() + timeout_ms/1000
        result, reply_data = transport.create(spec.service, spec, timeout_ms)
        if monotonic() >= deadline:
            report["status"] = "total_deadline_after_request"
        elif monotonic() > response_deadline:
            report["status"] = "response_deadline_after_request"
        elif type(result) is not bool or type(reply_data) is not bool:
            report["status"] = "malformed_transport_result"
        elif not result:
            report["status"] = "transport_failed_or_timed_out"
        elif not reply_data:
            report["status"] = "create_rejected"
            report["create_outcome"] = "negative_reply"
        else:
            report.update(success=True, status="create_acknowledged", create_outcome="positive_reply")
        return report
    except (Exception, KeyboardInterrupt) as exc:
        report["status"] = "exception_after_attempt" if report["request_attempts"] else "exception_before_attempt"
        report["error"] = f"{type(exc).__name__}: {exc}"
        return report
    finally:
        report["elapsed_wall_seconds"] = monotonic()-start


def write_new_json(path: Path, obj):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(obj, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def main(argv=None, *, transport_factory=GazeboTransport):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--params-file", type=Path, required=True)
    parser.add_argument("--report-dir", type=Path, required=True)
    parser.add_argument("--execute", action="store_true", help="permit exactly one real Gazebo create request")
    args = parser.parse_args(argv)
    spec = load_spec(args.params_file.resolve(strict=True))
    # Exclusive directory is a rerun guard and keeps earlier evidence intact.
    args.report_dir.mkdir(parents=False, exist_ok=False)
    write_new_json(args.report_dir/"inputs.json", spec.metadata())
    if not args.execute:
        report = {"success": False, "status": "validated_only", "request_attempts": 0,
                  "create_outcome": "not_sent", "motion_authorized": False}
    else:
        try:
            transport = transport_factory()
            report = run_once(spec, transport, persist_intent=lambda intent:
                              write_new_json(args.report_dir/"attempt-intent.json", intent))
        except (Exception, KeyboardInterrupt) as exc:
            report = {"success": False, "status": "transport_initialization_failed",
                      "request_attempts": 0, "create_outcome": "not_sent",
                      "error": f"{type(exc).__name__}: {exc}", "motion_authorized": False}
    write_new_json(args.report_dir/"report.json", report)
    print(json.dumps(report, ensure_ascii=False, allow_nan=False))
    return 0 if report["success"] else (0 if report["status"] == "validated_only" else 1)


if __name__ == "__main__":
    raise SystemExit(main())
