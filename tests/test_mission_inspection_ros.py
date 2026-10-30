"""Opt-in DDS adapter regressions; no Nav2, Gazebo, or motion commands.

Run with Jazzy sourced, ROS_DOMAIN_ID=74 and LOCALHOST discovery. Synthetic
Clock, map, wheel odometry, dynamic TF and evidence messages traverse DDS.
The terminal action result is an explicit rclpy Future fixture, not a claim
that navigation or real braking occurred. Producer records use the real red
detector's processing/evidence functions, except deliberately altered replays.
"""
import json
from math import cos, sin
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src/limo_maze_mission"))
sys.path.insert(0, str(ROOT / "src/limo_maze_perception"))
sys.path.insert(0, str(ROOT / "tests"))

try:
    import rclpy
except ImportError:
    rclpy = None

N = 1_000_000_000


@unittest.skipUnless(rclpy is not None and os.environ.get("ROS_DOMAIN_ID") == "74" and
                     os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") == "LOCALHOST",
                     "DDS test requires ROS installed and explicit domain 74 / LOCALHOST")
class RealMissionInspectionTests(unittest.TestCase):
    def setUp(self):
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        self.context = self.executor = self.probe = self.mission = self.output = None
        # unittest does not call tearDown if setUp fails. Always release the
        # context/probe before an isolation assertion can fail.
        self.addCleanup(self.cleanup_ros)
        self.context = Context()
        rclpy.init(context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.probe = rclpy.create_node("mission_inspection_test_probe", context=self.context,
                                       enable_rosout=False)
        self.executor.add_node(self.probe)
        self.output = tempfile.TemporaryDirectory(prefix="limo-inspection-dds-")
        self.spin(.4)
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            names = self.probe.get_node_names()
            unrelated = [name for name in names if name != self.probe.get_name()]
            if names.count(self.probe.get_name()) != 1:
                unrelated.append("duplicate_or_undiscovered_test_probe")
            if not unrelated:
                break
            self.spin(.05)
        self.assertEqual(unrelated, [], "Domain 74 must be isolated")

    def cleanup_ros(self):
        errors = []
        for node in (self.mission, self.probe):
            if node is not None:
                try:
                    if self.executor is not None:
                        self.executor.remove_node(node)
                    node.destroy_node()
                except Exception as exc:
                    errors.append(str(exc))
        if self.executor is not None:
            try:
                self.executor.shutdown()
            except Exception as exc:
                errors.append(str(exc))
        if self.context is not None and self.context.ok():
            try:
                self.context.shutdown()
            except Exception as exc:
                errors.append(str(exc))
        if self.output is not None:
            self.output.cleanup()
        self.assertEqual(errors, [], "ROS cleanup must finish even after a fixture failure")

    def spin(self, seconds=.05):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.executor.spin_once(timeout_sec=min(.01, max(0., end-time.monotonic())))

    def create_mission(self, *, enabled=True, timeout=12.):
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        from tf2_ros import TransformBroadcaster
        from limo_maze_mission.node import MissionNode
        from limo_maze_perception.core import DetectorConfig
        from limo_maze_perception.evidence import FrameEvidence
        self.Clock, self.String = Clock, String
        self.Odometry, self.TransformStamped = Odometry, TransformStamped
        params = {"use_sim_time": True, "autonomous": False,
                  "map_topic": "/inspection_test/map",
                  "inspection_odom_topic": "/inspection_test/odom",
                  "frame_processing_topic": "/inspection_test/evidence",
                  "inspection_timeout_s": timeout, "output_dir": self.output.name}
        if enabled:
            params["require_processed_inspection"] = True
        self.mission = MissionNode(context=self.context,
                                   parameter_overrides=[Parameter(k, value=v) for k, v in params.items()])
        # Setup only: no timer observes half-created fixture state.
        self.mission._mission_timer.cancel()
        self.executor.add_node(self.mission)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.clock_pub = self.probe.create_publisher(Clock, "/clock", 10)
        self.epoch_pub = self.probe.create_publisher(String, "/mission/map_epoch", latched)
        self.map_pub = self.probe.create_publisher(OccupancyGrid, "/inspection_test/map", latched)
        self.odom_pub = self.probe.create_publisher(Odometry, "/inspection_test/odom", 10)
        self.evidence_pub = self.probe.create_publisher(String, "/inspection_test/evidence", 10)
        self.tf_pub = TransformBroadcaster(self.probe)
        self.spin(.4)
        self.now_ns = 10*N
        self.clock(self.now_ns)
        self.epoch_pub.publish(String(data="inspection-dds-epoch"))
        self.tf(self.now_ns)
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = self.mission.get_clock().now().to_msg()
        grid.info.width = grid.info.height = 80
        grid.info.resolution = .1
        grid.info.origin.position.x = grid.info.origin.position.y = -4.
        grid.info.origin.orientation.w = 1.
        grid.data = [0] * (80*80)
        self.map_pub.publish(grid)
        self.spin(.15)
        self.assertEqual(self.mission.get_clock().now().nanoseconds, self.now_ns)
        self.assertTrue(self.mission.map_epoch_ready)
        self.assertIsNotNone(self.mission.grid)
        self.producer = FrameEvidence(DetectorConfig())
        self.producer.on_epoch("inspection-dds-epoch", self.now_ns)

    def clock(self, ns):
        message = self.Clock()
        message.clock.sec, message.clock.nanosec = divmod(ns, N)
        self.clock_pub.publish(message)
        self.spin(.015)

    def tf(self, ns, pose=(1., 1., 0.), camera=True):
        transforms = []
        for parent, child, xyz, yaw in (
                ("map", "odom", (0., 0., 0.), 0.),
                ("odom", "base_link", (pose[0], pose[1], 0.), pose[2]),
                ("base_link", "camera_optical_frame", (.08, 0., .25), 0.)):
            if child == "camera_optical_frame" and not camera:
                continue
            message = self.TransformStamped()
            message.header.frame_id, message.child_frame_id = parent, child
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, N)
            message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = xyz
            message.transform.rotation.z, message.transform.rotation.w = sin(yaw/2), cos(yaw/2)
            transforms.append(message)
        self.tf_pub.sendTransform(transforms)

    def odom(self, ns, speed=0.):
        message = self.Odometry()
        message.header.frame_id, message.child_frame_id = "odom", "base_link"
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, N)
        message.pose.pose.orientation.w = 1.
        message.twist.twist.linear.x = float(speed)
        self.odom_pub.publish(message)

    def step(self, *, speed=0., pose=(1., 1., 0.), camera=True, delta_ns=100_000_000, wall=.07):
        self.now_ns += delta_ns
        self.clock(self.now_ns)
        self.tf(self.now_ns, pose, camera)
        self.odom(self.now_ns, speed)
        self.spin(wall)

    def terminal_success(self, *, returning=False):
        from action_msgs.msg import GoalStatus
        from rclpy.task import Future
        from limo_maze_mission.core import Goal
        node = self.mission
        node.home, node.started = ((1., 1., 0.) if returning else (0., 0., 0.)), time.monotonic()
        node.phase = "returning" if returning else "exploring"
        if returning:
            node.return_started = time.monotonic()
        node.goal = Goal(1., 1., 0., "home" if returning else "frontier", (1., 1.))
        node.token = 1
        future = Future()
        future.set_result(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED,
                                          result=SimpleNamespace(error_msg="")))
        node.on_goal_result(future, node.token)
        node._mission_timer.reset()
        self.spin(.12)
        self.assertEqual(node.phase, "home_verification" if returning else "settling")

    def settle(self):
        deadline = time.monotonic()+1.6
        while self.mission.phase == "settling" and time.monotonic() < deadline:
            self.step()
        self.assertEqual(self.mission.phase, "inspecting", self.mission.reason)

    def record(self, stamp=None):
        from test_perception_evidence import fixtures, stats
        stamp = self.now_ns if stamp is None else stamp
        color, depth, info, rgb, depth_array = fixtures(stamp)
        record = self.producer.begin(color, depth, info, self.now_ns, enabled=True, registered=True)
        record = self.producer.finish(record, stats(rgb, depth_array), self.now_ns)
        self.assertEqual(record["status"], "processed", record)
        return record

    def publish_record(self, record):
        self.evidence_pub.publish(self.String(data=json.dumps(record, allow_nan=False)))
        self.spin(.025)

    def accepted(self):
        return self.mission.inspection.snapshot()["accepted_frames"]

    def test_default_has_no_evidence_or_odometry_subscription(self):
        self.create_mission(enabled=False)
        self.assertIsNone(self.mission.inspection)
        self.assertIsNone(self.mission.inspection_clock_jump)
        self.assertEqual(self.evidence_pub.get_subscription_count(), 0)
        self.assertEqual(self.odom_pub.get_subscription_count(), 0)

    def test_invalid_inspection_timeout_fails_during_configuration(self):
        from rclpy.parameter import Parameter
        from limo_maze_mission.node import MissionNode
        # Keep a reference even if __init__ raises, so tearDown destroys its
        # already initialized rclpy Node and the context deterministically.
        self.mission = MissionNode.__new__(MissionNode)
        with self.assertRaisesRegex(ValueError, "inspection_timeout_s"):
            self.mission.__init__(context=self.context, parameter_overrides=[
                Parameter("require_processed_inspection", value=True),
                Parameter("inspection_timeout_s", value=301.)])

    def test_action_success_requires_pose_stationarity_and_new_processed_frames(self):
        self.create_mission()
        self.terminal_success()
        for _ in range(5):
            self.step(speed=.03)
            self.publish_record(self.record())
        self.assertEqual(self.mission.phase, "settling")
        self.assertEqual(self.accepted(), 0)
        for _ in range(7):
            self.step(pose=(1.12, 1., 0.))
            self.publish_record(self.record())
        self.assertEqual(self.mission.phase, "settling")
        self.assertEqual(self.accepted(), 0)
        self.settle()
        for index in range(3):
            self.step()
            self.publish_record(self.record())
            if index < 2:
                self.assertEqual(self.accepted(), index+1)
                self.assertFalse(any(e["event"] == "inspection_complete" for e in self.mission.events))
        self.spin(.12)
        complete = [e for e in self.mission.events if e["event"] == "inspection_complete"]
        self.assertEqual(len(complete), 1)
        evidence = complete[0]["evidence"]
        self.assertEqual(evidence["accepted_frames"], 3)
        self.assertGreaterEqual(evidence["last_rgb_stamp_ns"]-evidence["first_rgb_stamp_ns"], 200_000_000)
        self.assertFalse(evidence["visual_coverage_claimed"])
        self.assertEqual(evidence["last_evidence"]["base_pose"], [1., 1., 0.])
        self.assertAlmostEqual(evidence["last_evidence"]["camera_pose"][0], 1.08)
        self.assertEqual(self.mission.coverage.frames, 0)
        self.assertFalse(self.mission.visual_complete)

    def test_missing_sample_tf_old_samples_and_repeated_depth_do_not_count(self):
        self.create_mission()
        self.terminal_success()
        self.step()
        old_record = self.record()
        self.settle()
        self.publish_record(old_record)
        self.assertEqual(self.accepted(), 0)
        self.assertIn("sample_before_barrier", self.mission.inspection.snapshot()["rejection_counts"])
        nested_payload = "["*20000+"0"+"]"*20000
        self.assertLess(len(nested_payload), 65536)
        rejection_codes = ("invalid_json", "invalid_schema")
        before = sum(self.mission.inspection.snapshot()["rejection_counts"].get(code, 0)
                     for code in rejection_codes)
        self.evidence_pub.publish(self.String(data=nested_payload))
        self.spin(.025)
        self.assertEqual(self.mission.phase, "inspecting")
        self.assertEqual(self.accepted(), 0)
        after = sum(self.mission.inspection.snapshot()["rejection_counts"].get(code, 0)
                    for code in rejection_codes)
        self.assertEqual(after, before+1, "Deep JSON must be rejected across parser recursion limits")
        self.step(camera=False)
        self.publish_record(self.record())
        self.assertEqual(self.accepted(), 0)
        self.assertIn("sample_tf_unavailable", self.mission.inspection.snapshot()["rejection_counts"])
        self.step()
        first = self.record()
        self.publish_record(first)
        self.assertEqual(self.accepted(), 1)
        self.step(delta_ns=20_000_000)
        replay = self.record()
        # Adversarial transport fixture: a false processed claim with reused depth.
        replay["depth"]["stamp_ns"] = first["depth"]["stamp_ns"]
        self.publish_record(replay)
        self.assertEqual(self.accepted(), 1)
        self.assertIn("nonadvancing_sample", self.mission.inspection.snapshot()["rejection_counts"])
        deadline_before = self.mission.inspection_deadline
        self.step(speed=.03)
        self.assertEqual(self.mission.phase, "settling")
        self.assertEqual(self.accepted(), 0)
        self.assertEqual(self.mission.inspection_deadline, deadline_before)

    def test_paused_ros_clock_hits_wall_deadline_and_blocks_without_completion(self):
        self.create_mission(timeout=2.)
        self.terminal_success()
        self.settle()
        paused = self.mission.get_clock().now().nanoseconds
        deadline = time.monotonic()+2.5
        while self.mission.phase != "blocked" and time.monotonic() < deadline:
            self.spin(.05)
        self.assertEqual(self.mission.get_clock().now().nanoseconds, paused)
        self.assertEqual(self.mission.phase, "blocked", self.mission.reason)
        self.assertEqual(self.mission.reason, "inspection_wall_timeout_incomplete")
        self.assertFalse(any(e["event"] == "inspection_complete" for e in self.mission.events))
        self.assertEqual(self.accepted(), 0)
        saved = list(Path(self.output.name).glob("mission_*.json"))
        self.assertEqual(len(saved), 1)
        self.assertFalse(json.loads(saved[0].read_text())["mission_success"])

    def test_paused_ros_clock_cannot_verify_home_from_frozen_stop_evidence(self):
        self.create_mission()
        self.terminal_success(returning=True)
        for _ in range(9):
            self.step()
        self.assertEqual(self.mission.phase, "home_verification")
        self.assertIsNotNone(self.mission.home_stable_since)
        paused = self.mission.get_clock().now().nanoseconds
        self.spin(self.mission.cfg["home_stable_s"]+.3)
        self.assertEqual(self.mission.get_clock().now().nanoseconds, paused)
        self.assertEqual(self.mission.phase, "home_verification", self.mission.reason)
        self.assertIsNone(self.mission.home_stable_since)
        self.assertFalse(self.mission.result_written)
        self.assertEqual(list(Path(self.output.name).glob("mission_*.json")), [])

    def test_clock_jump_between_messages_invalidates_even_after_clock_recovers(self):
        self.create_mission()
        self.terminal_success()
        self.settle()
        self.step()
        self.publish_record(self.record())
        self.assertEqual(self.accepted(), 1)
        self.mission._mission_timer.cancel()
        prior = self.now_ns
        self.clock(5*N)
        self.clock(prior+100_000_000)
        self.assertTrue(self.mission.inspection_clock_invalid)
        self.mission._mission_timer.reset()
        self.spin(.3)
        self.assertEqual(self.mission.phase, "blocked", self.mission.reason)
        self.assertIn("clock", self.mission.reason)
        self.assertEqual(self.accepted(), 0)
        self.epoch_pub.publish(self.String(data="inspection-dds-new-epoch"))
        self.spin(.12)
        self.assertEqual(self.mission.phase, "blocked", "Epoch change requires a new mission")
        self.assertFalse(any(e["event"] == "inspection_complete" for e in self.mission.events))

    def test_prehome_clock_jump_requires_new_epoch_before_startup(self):
        self.create_mission()
        self.assertIsNone(self.mission.home)
        self.assertTrue(self.mission.inspection.ready)
        self.clock(5*N)
        self.clock(11*N)
        self.assertTrue(self.mission.inspection_clock_invalid)
        # Consume the pending jump through a DDS epoch callback before tick.
        # A replay of the same epoch must not unlock old queued evidence.
        self.epoch_pub.publish(self.String(data="inspection-dds-epoch"))
        self.spin(.05)
        self.assertFalse(self.mission.inspection.ready)
        self.mission._mission_timer.reset()
        self.spin(.15)
        self.assertIsNone(self.mission.home)
        self.assertEqual(self.mission.phase, "waiting")
        self.epoch_pub.publish(self.String(data="inspection-dds-new-epoch"))
        self.spin(.15)
        self.assertTrue(self.mission.inspection.ready)
        self.assertEqual(self.mission.inspection.epoch, "inspection-dds-new-epoch")
        self.assertFalse(self.mission.inspection_clock_invalid)
        self.assertEqual(self.mission.phase, "waiting", "There is still no navigation server in this fixture")


if __name__ == "__main__":
    unittest.main()
