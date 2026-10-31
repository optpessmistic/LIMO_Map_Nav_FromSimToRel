"""Opt-in bounded-observation DDS/ActionServer regressions, without motion.

After sourcing Jazzy, run from the repository root in an otherwise empty domain:
  RUN_OBSERVATION_ROS_TEST=1 ROS_DOMAIN_ID=74 \
    ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST /usr/bin/python3 -B -m unittest \
    discover -s tests -p test_observation_ros.py -v

Default discovery explicitly skips these tests. The real MissionNode action
client talks to a controlled rclpy NavigateToPose ActionServer over DDS. A
server-side gate delays each real terminal result; mission callbacks, tokens,
UUIDs, Ledgers, planner and observation queue are not replaced. Clock, known
map, dynamic TF, stationary/moving odometry and processing evidence use DDS.
FrameEvidence and the actual red processing functions consume synthetic blank
RGB-D inputs; raw images themselves are not transported through DDS here.

The supplied map and poses are test fixtures, not SLAM/navigation/physical-stop
evidence. No Nav2/Gazebo is started, no Twist publisher exists, and passing does
not certify visual coverage, raw retention or a successful real-world mission.
Only helpers are reused from the inspection suite: its TestCase is not inherited
or exposed as another module-level TestCase (avoids rerunning its tests).
"""
import json
from collections import deque
from math import atan2
import os
from pathlib import Path
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests"))
import test_mission_inspection_ros as inspection_helpers

N = inspection_helpers.N
rclpy = inspection_helpers.rclpy
ENABLED = (os.environ.get("RUN_OBSERVATION_ROS_TEST") == "1" and
           os.environ.get("ROS_DOMAIN_ID") == "74" and
           os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") == "LOCALHOST" and
           rclpy is not None)


@unittest.skipUnless(ENABLED, "Set RUN_OBSERVATION_ROS_TEST=1 with Jazzy, domain 74 / LOCALHOST")
class ObservationActionDDSTests(unittest.TestCase):
    # Reuse sensor transport/processing helpers, never their synthetic action
    # terminal helper. These aliases do not inherit any other test_* methods.
    spin = inspection_helpers.RealMissionInspectionTests.spin
    tf = inspection_helpers.RealMissionInspectionTests.tf
    odom = inspection_helpers.RealMissionInspectionTests.odom
    publish_record = inspection_helpers.RealMissionInspectionTests.publish_record
    accepted = inspection_helpers.RealMissionInspectionTests.accepted

    def setUp(self):
        self.action_server = None
        self.server_goals, self.server_cancels = [], []
        self.pose = (1.55, 1.55, 0.)
        self.epoch = "observation-action-dds-epoch"
        self.sensor_trace = deque(maxlen=64)
        self.next_sensor_wall = None
        # Includes domain isolation assertion and cleanup even after setup fails.
        inspection_helpers.RealMissionInspectionTests.setUp(self)
        self.addCleanup(self.failure_diagnostics)  # Before cleanup_ros removes the evidence.

    def diagnostic_snapshot(self):
        now = time.monotonic()
        node = self.mission
        state = dict(node.inspection_odom)
        return {"wall": now, "clock_ns": node.get_clock().now().nanoseconds,
                "fixture_clock_ns": getattr(self, "now_ns", None), "phase": node.phase,
                "reason": node.reason, "odom": state,
                "odom_wall_age_s": None if state["received"] is None else now-state["received"],
                "inspection": node.inspection_snapshot(),
                "recent_events": node.events[-12:], "sensor_trace": list(self.sensor_trace)}

    def failure_diagnostics(self):
        result = self._outcome.result
        if self.mission is not None and any(test is self for test, _ in result.failures+result.errors):
            print("OBSERVATION_DDS_FAILURE " + json.dumps(self.diagnostic_snapshot(), allow_nan=False))

    def cleanup_ros(self):
        errors = []
        try:
            if self.mission is not None:
                self.mission._mission_timer.cancel()
            for entry in self.server_goals:
                if not entry["gate"].done():
                    entry["gate"].set_result("cancel" if entry["handle"].is_cancel_requested else "abort")
            if self.server_goals and self.executor is not None:
                self.spin(.15)
            if self.action_server is not None:
                self.action_server.destroy()
        except Exception as exc:
            errors.append(str(exc))
        finally:
            inspection_helpers.RealMissionInspectionTests.cleanup_ros(self)
        self.assertEqual(errors, [], "The fixture must retire its controlled server callbacks")

    async def execute_goal(self, handle):
        from nav2_msgs.action import NavigateToPose
        from rclpy.task import Future
        entry = {"handle": handle, "uuid": list(handle.goal_id.uuid),
                 "request": handle.request, "gate": Future(executor=self.executor),
                 "terminal": None}
        self.server_goals.append(entry)
        outcome = await entry["gate"]
        if outcome == "succeed":
            handle.succeed()
        elif outcome == "cancel":
            handle.canceled()
        else:
            handle.abort()
        entry["terminal"] = outcome
        return NavigateToPose.Result()

    def accept_cancel(self, handle):
        from rclpy.action import CancelResponse
        self.server_cancels.append(list(handle.goal_id.uuid))
        # Deliberately keep execute_goal suspended: ACK alone is not terminal.
        return CancelResponse.ACCEPT

    def create_mission(self, *, stage_timeout=30.):
        from geometry_msgs.msg import TransformStamped
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.action import ActionServer
        from rclpy.parameter import Parameter
        from rclpy.time import Time
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        from tf2_ros import TransformBroadcaster
        from limo_maze_mission import node as mission_module
        from limo_maze_perception.core import DetectorConfig
        from limo_maze_perception import evidence as evidence_module
        from test_perception_evidence import fixtures, stats
        # Load/initialize the actual image algorithm before enabling the mission
        # timer. No evidence record is made or selected by this warm-up.
        self.image_fixtures, self.image_stats, self.RosTime = fixtures, stats, Time
        _, _, _, warm_rgb, warm_depth = fixtures()
        stats(warm_rgb, warm_depth)
        self.assertEqual(Path(mission_module.__file__).resolve(),
                         ROOT / "src/limo_maze_mission/limo_maze_mission/node.py")
        self.assertEqual(Path(evidence_module.__file__).resolve(),
                         ROOT / "src/limo_maze_perception/limo_maze_perception/evidence.py")
        self.Clock, self.String = Clock, String
        self.Odometry, self.TransformStamped = Odometry, TransformStamped
        action_name = "/observation_test/navigate_to_pose"
        self.action_server = ActionServer(self.probe, NavigateToPose, action_name,
                                          execute_callback=self.execute_goal,
                                          cancel_callback=self.accept_cancel)
        params = {"use_sim_time": True, "autonomous": True,
                  "navigation_action": action_name,
                  "map_topic": "/observation_test/map",
                  "inspection_odom_topic": "/observation_test/odom",
                  "frame_processing_topic": "/observation_test/evidence",
                  "observation_topic": "/observation_test/targets",
                  "post_mapping_observation_mode": "bounded_processed_frames",
                  "require_processed_inspection": True,
                  "observation_max_stations": 1,
                  "observation_headings_per_station": 2,
                  "observation_max_dispatches": 2,
                  "observation_stage_timeout_s": stage_timeout,
                  "inspection_timeout_s": 8., "mission_timeout_s": 60.,
                  "return_timeout_s": 12., "cancel_timeout_s": 4.,
                  "home_frontier_return_reserve_s": 1.,
                  "output_dir": self.output.name}
        self.mission = mission_module.MissionNode(
            context=self.context, parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        self.mission._mission_timer.cancel()  # Fixture setup only; real timer runs below.
        self.executor.add_node(self.mission)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.clock_pub = self.probe.create_publisher(Clock, "/clock", 10)
        self.epoch_pub = self.probe.create_publisher(String, "/mission/map_epoch", latched)
        self.map_pub = self.probe.create_publisher(OccupancyGrid, params["map_topic"], latched)
        self.odom_pub = self.probe.create_publisher(Odometry, params["inspection_odom_topic"], 10)
        self.evidence_pub = self.probe.create_publisher(String, params["frame_processing_topic"], 10)
        self.tf_pub = TransformBroadcaster(self.probe)
        self.grid_message = OccupancyGrid()
        self.grid_message.header.frame_id = "map"
        self.grid_message.info.width = self.grid_message.info.height = 30
        self.grid_message.info.resolution = .1
        self.grid_message.info.origin.orientation.w = 1.
        self.grid_message.data = [100 if x in (0, 29) or y in (0, 29) else 0
                                  for y in range(30) for x in range(30)]
        self.spin(.4)
        self.now_ns = 10*N
        self.clock(self.now_ns)
        self.tf(self.now_ns, self.pose)
        self.epoch_pub.publish(String(data=self.epoch))
        self.publish_map()
        self.spin(.15)
        self.assertTrue(self.mission.map_epoch_ready)
        self.assertTrue(self.mission.inspection.ready)
        self.assertIsNotNone(self.mission.grid)
        self.wait_until(self.mission.nav.server_is_ready, "controlled action server discovery", pulse=False)
        self.producer = evidence_module.FrameEvidence(DetectorConfig())
        self.producer.on_epoch(self.epoch, self.now_ns)
        self.mission._mission_timer.reset()

    def publish_map(self):
        self.grid_message.header.stamp.sec, self.grid_message.header.stamp.nanosec = divmod(self.now_ns, N)
        self.map_pub.publish(self.grid_message)

    def delivered(self, predicate, label):
        # One send, bounded wait for DDS consumption: never resend a source
        # sample or replace a rejected frame. Only post-arrival windows require
        # the 0.2 s fixture bound below production's unchanged 0.3 s odom limit.
        # Initialization/navigation/cancel delivery does not certify a stop and
        # may wait up to 2 s for the executor/DDS; production deadlines still run.
        phase = self.mission.phase
        timeout = .2 if phase in ("settling", "inspecting", "home_verification") else 2.
        started = time.monotonic()
        trace = {"kind": "delivery_wait", "label": label, "phase_at_start": phase,
                 "timeout_s": timeout, "started_wall": started}
        self.sensor_trace.append(trace)
        end = started+timeout
        while not predicate() and time.monotonic() < end:
            self.spin(.002)
        trace["finished_wall"] = time.monotonic()
        if not predicate():
            self.fail(label+": "+json.dumps(self.diagnostic_snapshot(), allow_nan=False))

    def clock(self, ns):
        message = self.Clock()
        message.clock.sec, message.clock.nanosec = divmod(ns, N)
        trace = {"kind": "clock", "stamp_ns": ns, "published_wall": time.monotonic()}
        self.sensor_trace.append(trace)
        self.clock_pub.publish(message)
        # Publishing on a different topic is not an ordering guarantee. Odom
        # must not be sent ahead of the mission's consumed simulation clock.
        self.delivered(lambda: self.mission.get_clock().now().nanoseconds == ns,
                       "fixture /clock was not consumed")
        trace["consumed_wall"] = time.monotonic()

    def pulse(self, *, speed=0.):
        if self.next_sensor_wall is not None:
            self.spin(max(0., self.next_sensor_wall-time.monotonic()))
        self.now_ns += 100_000_000
        self.clock(self.now_ns)
        sent = time.monotonic()
        phase, token = self.mission.phase, self.mission.token
        self.tf(self.now_ns, self.pose)
        self.odom(self.now_ns, speed)
        self.publish_map()  # Keep map receipt fresh without inventing new cells.
        trace = {"kind": "sensor", "stamp_ns": self.now_ns, "sent_wall": sent,
                 "speed": speed, "phase_at_send": phase}
        self.sensor_trace.append(trace)
        self.next_sensor_wall = sent+.1
        post_arrival = ("settling", "inspecting", "home_verification")

        def sensors_consumed():
            # A goal may retire while this message is in flight. The production
            # callback intentionally ignores odometry outside post-arrival work.
            same_window = (phase in post_arrival and self.mission.phase in post_arrival and
                           self.mission.token == token)
            odom_ready = not same_window or self.mission.inspection_odom["stamp_ns"] == self.now_ns
            return odom_ready and self.mission.tf_buffer.can_transform(
                "map", "camera_optical_frame", self.RosTime(nanoseconds=self.now_ns))

        self.delivered(sensors_consumed, "fixture same-stamp TF/odometry was not consumed")
        state = self.mission.inspection_odom
        trace.update(delivered_wall=time.monotonic(), odom_stamp_ns=state["stamp_ns"],
                     odom_received_wall=state["received"])

    def record(self):
        stamp = self.now_ns
        start = time.monotonic()
        color, depth, info, rgb, depth_array = self.image_fixtures(stamp)
        record = self.producer.begin(color, depth, info, stamp, enabled=True, registered=True)
        record = self.producer.finish(record, self.image_stats(rgb, depth_array), stamp)
        self.sensor_trace.append({"kind": "processing", "stamp_ns": stamp,
                                  "started_wall": start, "finished_wall": time.monotonic()})
        self.assertEqual(record["status"], "processed", record)
        return record

    def wait_until(self, predicate, label, *, timeout=3., pulse=True):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic() < end:
            self.pulse() if pulse else self.spin(.02)
        self.assertTrue(predicate(), f"{label}; phase={self.mission.phase}, reason={self.mission.reason}")

    def goal_entry(self, index, *, kind):
        self.wait_until(lambda: len(self.server_goals) > index and self.mission.goal_handle is not None,
                        "real goal acceptance and UUID")
        entry = self.server_goals[index]
        self.assertEqual(self.mission.goal.kind, kind)
        self.assertEqual(list(self.mission.goal_handle.goal_id.uuid), entry["uuid"])
        request = entry["request"].pose
        self.assertEqual(request.header.frame_id, "map")
        self.assertAlmostEqual(request.pose.position.x, self.mission.goal.x)
        self.assertAlmostEqual(request.pose.position.y, self.mission.goal.y)
        q = request.pose.orientation
        self.assertAlmostEqual(atan2(2*q.w*q.z, 1-2*q.z*q.z), self.mission.goal.yaw)
        return entry

    def succeed(self, entry, *, home=False):
        self.pose = (self.mission.goal.x, self.mission.goal.y, self.mission.goal.yaw)
        self.pulse()  # Deliver the synthetic arrived pose before the action result.
        entry["gate"].set_result("succeed")
        self.wait_until(lambda: self.mission.phase == ("home_verification" if home else "settling"),
                        "DDS terminal result enters post-arrival verification")
        self.assertIsNone(self.mission.goal_handle)
        self.assertFalse(self.mission.send_pending)

    def verify_return_result(self):
        self.wait_until(lambda: self.mission.phase == "done", "fresh odometry verifies home", timeout=4.)
        saved = list(Path(self.output.name).glob("mission_*.json"))
        self.assertEqual(len(saved), 1)
        result = json.loads(saved[0].read_text())
        self.assertTrue(result["returned_pose_verified"])
        self.assertTrue(result["map_complete"])
        self.assertFalse(result["visual_coverage_complete"])
        self.assertFalse(result["mission_success"])
        self.assertEqual(self.mission.coverage.frames, 0)
        return result

    def test_two_headings_use_real_action_uuid_and_new_dds_windows_then_return(self):
        self.create_mission()
        previous_record = None
        queue = None
        for index in range(2):
            entry = self.goal_entry(index, kind="observation_queue")
            if queue is None:
                queue = self.mission.observation_queue
            self.assertIs(self.mission.observation_queue, queue)
            active = dict(self.mission.observation_active)
            self.assertEqual(active["nav_goal_uuid"], entry["uuid"])
            self.assertEqual(active["inspection_request_id"], "goal-"+str(self.mission.token))
            self.succeed(entry)
            self.assertEqual(self.mission.observation_active["nav_terminal_status"], 4)
            # Successful navigation alone, or fresh frames while moving, cannot
            # complete an action. The complete stop gate is still production code.
            for _ in range(2):
                self.pulse(speed=.03)
                self.publish_record(self.record())
            self.assertEqual(self.mission.phase, "settling")
            self.assertEqual(self.accepted(), 0)
            self.wait_until(lambda: self.mission.phase == "inspecting", "continuous stationary settling")
            ledger = self.mission.inspection
            self.assertEqual(ledger.request_id, active["inspection_request_id"])
            self.assertEqual(ledger.accepted_frames, 0)
            if previous_record is not None:
                temporal_rejections = ("sample_before_barrier", "stale_sample", "stale_processing")
                before = sum(ledger.rejection_counts.get(code, 0) for code in temporal_rejections)
                self.publish_record(previous_record)
                self.assertEqual(ledger.accepted_frames, 0)
                # A slower host may first hit the age guard. Both guards reject
                # the actual old packet without changing its source timestamps.
                after = sum(ledger.rejection_counts.get(code, 0) for code in temporal_rejections)
                self.assertEqual(after, before+1)
                self.assertEqual(ledger.request_id, active["inspection_request_id"])
            for ordinal in range(3):
                self.pulse()
                if self.mission.phase != "inspecting":
                    self.fail("sampling phase changed: "+json.dumps(self.diagnostic_snapshot(), allow_nan=False))
                record = self.record()
                if not self.mission.inspection_stopped(time.monotonic()):
                    self.fail("sampling lost stationary evidence: "+json.dumps(self.diagnostic_snapshot(), allow_nan=False))
                self.assertEqual(record["processing"]["detection_count"], 0)
                self.publish_record(record)
                if ordinal < 2:
                    if ledger.accepted_frames != ordinal+1:
                        self.fail("fixed frame was not accepted: "+json.dumps(self.diagnostic_snapshot(), allow_nan=False))
                    self.assertEqual(len(self.mission.observation_attempt_records), index)
                previous_record = record
            self.wait_until(lambda: len(self.mission.observation_attempt_records) == index+1,
                            "exactly three fresh frames retire this view")
            finished = self.mission.observation_attempt_records[index]
            self.assertEqual(finished["outcome"], "inspection_satisfied")
            self.assertEqual(finished["nav_goal_uuid"], entry["uuid"])
            self.assertEqual(finished["epoch"], self.epoch)
            self.assertEqual(finished["dispatch_id"], index+1)
            self.assertEqual(finished["nav_token"], active["nav_token"])
            self.assertEqual(finished["nav_terminal_status"], 4)
            evidence = finished["inspection"]
            self.assertEqual(evidence["request_id"], active["inspection_request_id"])
            self.assertEqual(evidence["goal_token"], active["nav_token"])
            self.assertEqual(evidence["expected_pose"], [active["goal"][k] for k in ("x", "y", "yaw")])
            self.assertEqual(evidence["accepted_frames"], 3)
            self.assertGreater(evidence["first_rgb_stamp_ns"], evidence["sampling_after_stamp_ns"])
            self.assertGreaterEqual(evidence["last_rgb_stamp_ns"]-evidence["first_rgb_stamp_ns"], 200_000_000)
            self.assertEqual(evidence["last_evidence"]["detection_count"], 0)
            self.assertFalse(evidence["visual_coverage_claimed"])
        home = self.goal_entry(2, kind="home")
        self.assertTrue(self.mission.observation_closed)
        self.assertEqual(queue.dispatches, 2)
        self.assertEqual(queue.snapshot()["state_counts"], {"completed": 2})
        attempts = self.mission.observation_attempt_records
        self.assertEqual(len({r["heading_index"] for r in attempts}), 2)
        self.assertEqual(len({r["nav_token"] for r in attempts}), 2)
        self.assertEqual(len({tuple(r["nav_goal_uuid"]) for r in attempts}), 2)
        self.succeed(home, home=True)
        result = self.verify_return_result()
        self.assertEqual(len(self.server_goals), 3)
        self.assertEqual(result["observation_actions"]["queue"]["dispatches"], 2)
        self.assertEqual(self.server_cancels, [])

    def test_paused_clock_stage_deadline_waits_for_owned_terminal_before_return(self):
        self.create_mission(stage_timeout=3.)
        entry = self.goal_entry(0, kind="observation_queue")
        queue, active = self.mission.observation_queue, dict(self.mission.observation_active)
        deadline, paused = self.mission.observation_deadline_wall, self.now_ns
        self.wait_until(lambda: self.mission.cancel_response, "real cancel ACK after stage wall deadline",
                        timeout=max(0., deadline-time.monotonic())+1., pulse=False)
        self.assertEqual(self.mission.get_clock().now().nanoseconds, paused)
        self.assertTrue(self.mission.observation_closed)
        self.assertEqual(self.mission.observation_close_reason, "bounded_observation_wall_timeout")
        self.assertEqual(self.server_cancels, [entry["uuid"]])
        self.assertTrue(self.mission.canceling)
        self.assertFalse(self.mission.cancel_terminal)
        self.assertIsNotNone(self.mission.goal_handle)
        self.assertEqual(self.mission.observation_active["action_id"], active["action_id"])
        self.assertEqual(self.mission.observation_attempt_records, [])
        self.assertIsNone(self.mission.return_started)
        self.spin(.25)
        self.assertEqual(len(self.server_goals), 1, "ACK cannot authorize the home goal")
        self.assertFalse(self.mission.cancel_terminal)
        self.assertIsNone(entry["terminal"])
        entry["gate"].set_result("cancel")  # Only now does the server report an owned terminal.
        home = self.goal_entry(1, kind="home")
        self.assertEqual(entry["terminal"], "cancel")
        self.assertIsNone(self.mission.observation_active)
        self.assertIs(self.mission.observation_queue, queue)
        self.assertEqual(queue.dispatches, 1)
        self.assertEqual(len(self.mission.observation_attempt_records), 1)
        failed = self.mission.observation_attempt_records[0]
        self.assertEqual(failed["outcome"], "canceled")
        self.assertEqual(failed["nav_goal_uuid"], entry["uuid"])
        self.assertEqual(failed["nav_terminal_status"], 5)
        self.assertIsNone(failed["inspection"])
        return_started, return_deadline = self.mission.return_started, self.mission.return_deadline_wall
        self.succeed(home, home=True)
        result = self.verify_return_result()
        self.assertEqual(len(self.server_goals), 2)
        self.assertEqual(len(self.mission.observation_attempt_records), 1)
        self.assertEqual(self.mission.observation_deadline_wall, deadline)
        self.assertEqual(self.mission.return_started, return_started)
        self.assertEqual(self.mission.return_deadline_wall, return_deadline)
        self.assertEqual(self.mission.return_attempts, 1)
        self.assertFalse(result["observation_actions"]["queue"]["all_planned_actions_completed"])


if __name__ == "__main__":
    unittest.main()
