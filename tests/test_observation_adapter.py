"""Execute draft adapter methods with fake ROS/Nav2 and REAL InspectionLedger.

No ROS imports or nodes: AST drops import statements only, keeping the complete
production-shaped class, its __init__, tick, callbacks, and safety guards. Robot
pose, TF, messages and action results are synthetic, not runtime validation.
"""
import ast
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as S
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/limo_maze_mission'))
from limo_maze_mission.core import Coverage, FailureBook, Goal, Grid, Planner, TargetStore, angle_delta
from limo_maze_mission.inspection import EvidenceError, InspectionLedger, InspectionPolicy, validate_record
from limo_maze_mission.observation import ObservationQueue, Policy as ObservationPolicy
from test_mission_inspection import frame


class TimeFixture:
    def __init__(self): self.wall, self.ns = 0., 10_000_000_000
    def monotonic(self): return self.wall
    def advance(self, dt=.1, ros=True):
        self.wall += dt
        if ros: self.ns += round(dt*1e9)
    def now(self): return S(nanoseconds=self.ns, to_msg=lambda: S(sec=self.ns//10**9, nanosec=self.ns%10**9))
    def create_jump_callback(self, *args, **kwargs): return None


class Future:
    def __init__(self): self.callbacks, self.value, self.done = [], None, False
    def add_done_callback(self, callback):
        self.callbacks.append(callback)
        if self.done: callback(self)
    def result(self): return self.value
    def resolve(self, value):
        self.value, self.done = value, True
        for callback in list(self.callbacks): callback(self)


class Handle:
    def __init__(self, id):
        self.accepted, self.goal_id = True, S(uuid=[id]*16)
        self.result_future, self.cancel_future = Future(), Future()
        self.cancel_calls = 0
    def get_result_async(self): return self.result_future
    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future


class Nav:
    def __init__(self): self.sent = []
    def server_is_ready(self): return True
    def send_goal_async(self, request):
        future = Future()
        self.sent.append((request, future))
        return future


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(message)


class FakeNode:
    def __init__(self, name, **kwargs):
        self.params = dict(kwargs.get('parameter_overrides', {}))
        self.clock = TIME
    def declare_parameter(self, key, value): self.params.setdefault(key, value)
    def get_parameter(self, key): return S(value=self.params[key])
    def create_subscription(self, *args, **kwargs): return S()
    def create_publisher(self, *args, **kwargs): return Publisher()
    def create_timer(self, *args, **kwargs): return S(cancel=lambda: None)
    def get_clock(self): return self.clock
    def get_logger(self): return S(info=lambda *a: None, warning=lambda *a: None, error=lambda *a: None)


def pose_stamped():
    return S(header=S(), pose=S(position=S(x=0., y=0., z=0.), orientation=S(x=0., y=0., z=0., w=1.)))


TIME = TimeFixture()
adapter = ModuleType('draft_observation_adapter')
adapter.__dict__.update({name: globals()[name] for name in (
    'asdict', 'json', 'Path', 'Coverage', 'FailureBook', 'Goal', 'Grid', 'Planner', 'TargetStore',
    'angle_delta', 'EvidenceError', 'InspectionLedger', 'InspectionPolicy', 'validate_record',
    'ObservationQueue', 'ObservationPolicy')})
adapter.__dict__.update({name: getattr(math, name) for name in ('atan2', 'cos', 'hypot', 'isfinite', 'sin')})
adapter.__dict__.update(Node=FakeNode, time=TIME, ActionClient=lambda *a: Nav(),
    Buffer=lambda **kw: S(), TransformListener=lambda *a: S(), Duration=lambda **kw: S(**kw),
    QoSProfile=lambda **kw: S(), DurabilityPolicy=S(TRANSIENT_LOCAL=1), ReliabilityPolicy=S(RELIABLE=1),
    OccupancyGrid=S, String=S, Odometry=S, Marker=S, MarkerArray=S, qos_profile_sensor_data=None,
    Clock=lambda **kw: S(), ClockType=S(STEADY_TIME=1), JumpThreshold=lambda **kw: S(),
    NavigateToPose=S(Goal=lambda: S()), PoseStamped=pose_stamped, TransformException=LookupError,
    GoalStatus=S(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6))
adapter.Marker = type('Marker', (), {'DELETEALL': 3})
source = ROOT/'src/limo_maze_mission/limo_maze_mission/node.py'
tree = ast.parse(source.read_text(encoding='utf-8'), filename=str(source))
tree.body = [item for item in tree.body if isinstance(item, (ast.ClassDef, ast.FunctionDef))]
exec(compile(tree, str(source), 'exec'), adapter.__dict__)


class AdapterTests(unittest.TestCase):
    def setUp(self):
        global TIME
        TIME = TimeFixture()
        adapter.time = TIME

    def mission(self, **overrides):
        params = {'autonomous': True, 'post_mapping_observation_mode': 'bounded_processed_frames',
                  'require_processed_inspection': True, 'observation_max_stations': 1,
                  'observation_headings_per_station': 2, 'observation_max_dispatches': 2}
        params.update(overrides)
        m = adapter.MissionNode(parameter_overrides=params)
        m.grid = Grid(30, 30, .1, 0., 0., 0., tuple(
            100 if x in (0, 29) or y in (0, 29) else 0 for y in range(30) for x in range(30)))
        m.synthetic_pose = (1.55, 1.55, .2)
        m.get_pose = lambda: m.synthetic_pose
        m.inspection_sample_transform = lambda frame_name, stamp: (
            (*m.synthetic_pose[:2], .15, 0., 0., math.sin(m.synthetic_pose[2]/2), math.cos(m.synthetic_pose[2]/2))
            if frame_name == m.cfg['base_frame'] else (1.634, 1.55, .18, 0., 0., 0., 1.))
        m.on_epoch(S(data='map-a'))
        m.home, m.started, m.phase, m.map_received = m.synthetic_pose, TIME.wall, 'exploring', TIME.wall
        m.saved_results = []
        m.save_result = lambda returned, final_pose=None: m.saved_results.append((returned, final_pose, m.observation_snapshot()))
        return m

    def tick(self, m, dt=0., ros=True):
        TIME.advance(dt, ros=ros)
        m.map_received = TIME.wall  # Explicit healthy-map fixture, independent of sensor evidence.
        m.tick()

    def odom(self, m, moving=False):
        stamp = S(sec=TIME.ns//10**9, nanosec=TIME.ns%10**9)
        m.on_inspection_odom(S(header=S(stamp=stamp, frame_id='odom'), child_frame_id='base_footprint',
            twist=S(twist=S(linear=S(x=.02 if moving else 0., y=0., z=0.), angular=S(x=0., y=0., z=0.)))))

    def start(self, m):
        self.tick(m)
        self.assertEqual(m.phase, 'observation_select')
        self.assertTrue(m.map_complete)
        self.tick(m)
        self.assertTrue(m.send_pending)
        self.assertEqual(m.goal.kind, 'observation_queue')

    def terminal(self, m, status=4):
        handle = Handle(m.token)
        m.nav.sent[-1][1].resolve(handle)
        if status == 4:
            m.synthetic_pose = (m.goal.x, m.goal.y, m.goal.yaw)
        handle.result_future.resolve(S(status=status, result=S(error_msg='fixture')))
        self.tick(m)
        return handle

    def settle(self, m):
        for _ in range(9):
            TIME.advance(.1)
            self.odom(m)
            self.tick(m)
            if m.phase == 'inspecting': return
        self.fail('stationary odometry should enable inspecting')

    def processed_window(self, m):
        for seq in range(1, 4):
            TIME.advance(.12)
            self.odom(m)
            record = frame(TIME.ns-20_000_000, seq=seq)
            m.on_frame_processing(S(data=json.dumps(record)))
            self.tick(m)

    def test_mode_defaults_off_and_cannot_fall_back_to_dwell(self):
        m = adapter.MissionNode()
        self.assertFalse(m.bounded_observation_enabled())
        self.assertIsNone(m.observation_snapshot())
        with self.assertRaises(ValueError):
            adapter.MissionNode(parameter_overrides={'post_mapping_observation_mode': 'bounded_processed_frames'})

    def test_ros_integer_uuid_is_serializable_in_active_and_completed_status(self):
        # Model a generated ROS integer scalar without adding a NumPy dependency
        # to this pure adapter test. Such values have int(), but no JSON encoder.
        class RosByte:
            def __init__(self, value): self.value = value
            def __int__(self): return self.value

        m = self.mission(); self.start(m)
        handle = Handle(m.token)
        values = [0, 1, 127, 128, 255] + list(range(11))
        handle.goal_id.uuid = [RosByte(value) for value in values]
        m.nav.sent[-1][1].resolve(handle)
        m.publish_status()
        active = json.loads(m.status_pub.messages[-1].data)['observation_actions']['active']
        self.assertEqual(active['nav_goal_uuid'], values)
        self.assertTrue(all(type(value) is int for value in m.observation_active['nav_goal_uuid']))
        m.synthetic_pose = (m.goal.x, m.goal.y, m.goal.yaw)
        handle.result_future.resolve(S(status=4, result=S(error_msg='fixture')))
        self.tick(m); self.settle(m); self.processed_window(m)
        recorded = json.loads(json.dumps(m.observation_attempt_records[0]))
        self.assertEqual(recorded['nav_goal_uuid'], values)

    def test_two_headings_zero_detections_real_ledgers_then_return(self):
        m = self.mission(camera_heading_offset_rad=.7)
        self.start(m)
        first_queue = m.observation_queue
        for _ in range(2):
            with patch.object(m.coverage, 'contains', side_effect=AssertionError('not action completion')):
                self.terminal(m)
                self.assertEqual(m.phase, 'settling')
                self.settle(m)
                self.processed_window(m)
        self.assertIs(m.observation_queue, first_queue)
        self.assertEqual(m.observation_queue.dispatches, 2)
        records = m.observation_attempt_records
        self.assertEqual(len(records), 2)
        self.assertEqual(len({r['heading_index'] for r in records}), 2)
        self.assertEqual(len({r['nav_token'] for r in records}), 2)
        for record in records:
            self.assertEqual(record['outcome'], 'inspection_satisfied')
            self.assertEqual(record['inspection']['accepted_frames'], 3)
            self.assertEqual(record['inspection']['last_evidence']['detection_count'], 0)
            view = first_queue.views[record['action_id']]
            self.assertEqual(record['goal']['yaw'], view.goal.yaw)  # No second camera offset.
        self.assertTrue(m.observation_closed)
        self.assertEqual(m.cancel_next, 'returning')
        self.assertFalse(m.visual_complete)
        self.assertFalse(m.failures.entries)

    def test_late_result_cannot_complete_next_heading(self):
        m = self.mission(); self.start(m)
        old_token = m.token
        self.terminal(m); self.settle(m); self.processed_window(m)
        new_active = dict(m.observation_active)
        future = Future(); future.resolve(S(status=4, result=S(error_msg='late')))
        m.on_goal_result(future, old_token)
        self.assertEqual(m.observation_active, new_active)
        self.assertTrue(m.send_pending)
        self.assertEqual(len(m.observation_attempt_records), 1)

    def test_navigation_failures_exhaust_finite_queue_without_failurebook(self):
        m = self.mission(); self.start(m)
        for _ in range(2): self.terminal(m, status=6)
        self.assertTrue(m.observation_closed)
        self.assertEqual(m.cancel_next, 'returning')
        self.assertEqual(m.observation_queue.dispatches, 2)
        self.assertTrue(all(r['outcome'] == 'navigation_failed' for r in m.observation_attempt_records))
        self.assertTrue(all(r['inspection'] is None for r in m.observation_attempt_records))
        self.assertFalse(m.failures.entries)

    def test_inspection_timeout_preserves_global_block_and_failure_evidence(self):
        m = self.mission(inspection_timeout_s=1.2); self.start(m); self.terminal(m)
        self.tick(m, 1.3)
        self.assertEqual(m.cancel_next, 'blocked')
        self.assertTrue(m.observation_closed)
        self.assertEqual(len(m.observation_attempt_records), 1)
        self.assertEqual(m.observation_attempt_records[0]['outcome'], 'inspection_failed')
        self.assertEqual(m.observation_queue.dispatches, 1)

    def test_epoch_change_keeps_owned_cancel_and_cannot_credit_late_success(self):
        m = self.mission(); self.start(m)
        handle = Handle(m.token); m.nav.sent[-1][1].resolve(handle)
        m.on_epoch(S(data='map-b'))
        self.assertTrue(m.canceling)
        self.assertEqual(handle.cancel_calls, 1)
        self.assertTrue(m.observation_queue.snapshot()['cancel_required'])
        self.assertFalse(m.coordinates_valid)
        handle.result_future.resolve(S(status=4, result=S(error_msg='late success')))
        self.tick(m)
        self.assertEqual(m.phase, 'blocked')
        self.assertFalse(m.observation_queue.snapshot()['all_planned_actions_completed'])
        self.assertEqual(m.observation_attempt_records[0]['outcome'], 'canceled')

    def test_stage_wall_deadline_runs_with_paused_clock_and_does_not_refill(self):
        m = self.mission(observation_stage_timeout_s=.5); self.start(m)
        handle = Handle(m.token); m.nav.sent[-1][1].resolve(handle)
        deadline = m.observation_deadline_wall
        self.tick(m, .6, ros=False)
        self.assertEqual(m.cancel_next, 'returning')
        self.assertTrue(m.observation_closed)
        self.assertEqual(m.observation_deadline_wall, deadline)
        self.assertEqual(len(m.nav.sent), 1)  # No new goal before owned terminal.
        handle.result_future.resolve(S(status=5, result=S(error_msg='cancel')))
        self.tick(m)
        self.assertEqual(m.phase, 'returning')
        self.assertEqual(m.observation_queue.dispatches, 1)

    def test_cancel_ack_without_terminal_does_not_release_next_view(self):
        m = self.mission(goal_timeout_s=.2); self.start(m)
        handle = Handle(m.token); m.nav.sent[-1][1].resolve(handle)
        self.tick(m, .3)
        handle.cancel_future.resolve(S(goals_canceling=[S(goal_id=handle.goal_id)]))
        self.tick(m)
        self.assertTrue(m.canceling)
        self.assertEqual(len(m.nav.sent), 1)
        handle.result_future.resolve(S(status=5, result=S(error_msg='cancel')))
        self.tick(m); self.tick(m)
        self.assertEqual(len(m.nav.sent), 2)
        self.assertEqual(m.observation_attempt_records[0]['outcome'], 'navigation_failed')

    def test_home_coverage_goal_cannot_reopen_closed_observation(self):
        m = self.mission(home_stable_s=.3); self.start(m)
        for _ in range(2): self.terminal(m, status=6)
        self.tick(m); self.tick(m)  # Cancel transition then home goal.
        self.assertEqual(m.goal.kind, 'home')
        self.terminal(m)
        m.detector_received, m.coverage.frames = TIME.wall, 1
        for _ in range(15):
            TIME.advance(.1); self.odom(m); self.tick(m)
            if m.phase == 'done': break
        self.assertEqual(m.phase, 'done')
        self.assertEqual(m.observation_queue.dispatches, 2)
        self.assertEqual(len(m.nav.sent), 3)
        self.assertFalse(m.visual_complete)
        self.assertTrue(m.saved_results[-1][0])

    def test_frames_before_settling_and_while_moving_cannot_complete(self):
        m = self.mission(); self.start(m); self.terminal(m)
        for i in range(3):
            TIME.advance(.12); self.odom(m, moving=True)
            m.on_frame_processing(S(data=json.dumps(frame(TIME.ns-20_000_000, seq=i+1))))
            self.tick(m)
        self.assertEqual(m.phase, 'settling')
        self.assertEqual(m.inspection.accepted_frames, 0)
        self.assertFalse(m.observation_attempt_records)

    def test_map_updates_do_not_replace_queue_or_replenish_budget(self):
        m = self.mission(); self.start(m)
        queue, deadline = m.observation_queue, m.observation_deadline_wall
        self.terminal(m, status=6)
        self.assertIs(m.observation_queue, queue)
        self.assertEqual(m.observation_deadline_wall, deadline)
        self.assertEqual(queue.dispatches, 2)
        self.terminal(m, status=6)
        # Even an explicit repeated stage-entry cannot unclose/rebuild it.
        m.begin_observation_stage(TIME.wall, m.synthetic_pose)
        self.assertIs(m.observation_queue, queue)
        self.assertTrue(m.observation_closed)
        self.assertEqual(queue.dispatches, 2)

    def test_clock_invalidation_before_sample_blocks_and_cannot_be_rearmed(self):
        m = self.mission(); self.start(m); self.terminal(m); self.settle(m)
        m.on_inspection_clock_jump()
        self.tick(m)
        self.assertEqual(m.cancel_next, 'blocked')
        self.assertTrue(m.observation_queue.invalid_reason)
        self.assertTrue(m.observation_closed)
        self.assertFalse(m.inspection.ready)
        m.on_epoch(S(data='map-a'))
        self.assertFalse(m.inspection.ready)

    def test_result_serialization_keeps_all_attempts_and_never_certifies_visual(self):
        import tempfile
        m = self.mission(); self.start(m)
        for _ in range(2): self.terminal(m, status=6)
        m.phase, m.result_written, m.map_complete = 'done', False, True
        m.visual_complete, m.cfg['target_definition_verified'] = True, True
        with tempfile.TemporaryDirectory(dir=ROOT) as directory:
            m.cfg['output_dir'] = directory
            # Restore ONLY serialization; navigation and ledger still ran above.
            with patch.object(adapter.time, 'strftime', lambda fmt: 'fixture', create=True):
                adapter.MissionNode.save_result(m, True, m.home)
            result = json.loads((Path(directory)/'mission_fixture.json').read_text())
        self.assertFalse(result['mission_success'])
        self.assertFalse(result['visual_coverage_complete'])
        self.assertTrue(result['returned_pose_verified'])
        self.assertEqual(len(result['observation_actions']['attempts']), 2)

    def test_stage_deadline_at_every_owned_phase_returns_once_after_terminal(self):
        for phase in ('pending_send', 'accepted_active', 'settling', 'inspecting'):
            with self.subTest(phase=phase):
                self.setUp()
                m = self.mission(observation_stage_timeout_s=2.)
                self.start(m)
                handle = None
                if phase == 'accepted_active':
                    handle = Handle(m.token); m.nav.sent[-1][1].resolve(handle)
                elif phase in ('settling', 'inspecting'):
                    self.terminal(m)
                    if phase == 'inspecting': self.settle(m)
                deadline = m.observation_deadline_wall
                self.tick(m, deadline-TIME.wall+.05, ros=False)
                self.assertTrue(m.observation_closed)
                self.assertEqual(m.cancel_next, 'returning')
                self.assertEqual(len(m.nav.sent), 1)
                if phase in ('pending_send', 'accepted_active'):
                    for _ in range(3): self.tick(m, .05, ros=False)
                    self.assertTrue(m.canceling)
                    self.assertIsNone(m.return_started)
                    self.assertEqual(len(m.nav.sent), 1)
                    if handle is None:
                        handle = Handle(m.token); m.nav.sent[-1][1].resolve(handle)
                    self.assertEqual(handle.cancel_calls, 1)
                    handle.result_future.resolve(S(status=5, result=S(error_msg='owned canceled')))
                self.tick(m)
                self.assertEqual(m.phase, 'returning')
                return_started = m.return_started
                self.tick(m)  # Exactly one home goal, now pending.
                self.assertEqual(m.return_attempts, 1)
                for _ in range(4): self.tick(m, .05, ros=False)
                self.assertEqual(m.return_started, return_started)
                self.assertEqual(m.return_attempts, 1)
                self.assertEqual(m.observation_deadline_wall, deadline)
                self.assertEqual(len(m.nav.sent), 2)
                self.assertEqual(len(m.observation_attempt_records), 1)
                self.assertEqual(sum(e['event'] == 'cancel_requested' and e['next_phase'] == 'returning'
                                     for e in m.events), 1)

    def test_completion_binding_mismatch_blocks_without_next_dispatch(self):
        for field in ('request_id', 'inspection_goal_token', 'dispatch_id', 'epoch', 'nav_terminal_status'):
            with self.subTest(field=field):
                self.setUp()
                m = self.mission(); self.start(m); self.terminal(m); self.settle(m)
                # Real ledger completes, but scheduler has not consumed it yet.
                for seq in range(1, 4):
                    TIME.advance(.12); self.odom(m)
                    m.on_frame_processing(S(data=json.dumps(frame(TIME.ns-20_000_000, seq=seq))))
                self.assertEqual(m.inspection.status, 'satisfied')
                if field == 'request_id': m.inspection.request_id = 'wrong-request'
                elif field == 'inspection_goal_token': m.inspection_goal_token += 1
                elif field == 'dispatch_id': m.observation_active['dispatch_id'] += 10
                elif field == 'epoch': m.observation_active['epoch'] = 'wrong-epoch'
                else: m.observation_active['nav_terminal_status'] = 6
                self.tick(m)
                self.assertTrue(m.canceling)
                self.assertEqual(m.cancel_next, 'blocked')
                self.assertTrue(m.observation_closed)
                self.assertEqual(m.observation_queue.dispatches, 1)
                self.assertEqual(len(m.nav.sent), 1)
                self.assertFalse(m.observation_queue.snapshot()['all_planned_actions_completed'])
                self.tick(m)
                self.assertEqual(m.phase, 'blocked')
                self.assertEqual(len(m.nav.sent), 1)

    def test_cancel_timeout_preserves_unknown_active_attempt_without_new_goal(self):
        m = self.mission(goal_timeout_s=.2, cancel_timeout_s=.4); self.start(m)
        self.tick(m, .3)
        self.tick(m, .5)
        self.assertEqual(m.phase, 'blocked')
        self.assertTrue(m.observation_closed)
        self.assertIsNotNone(m.observation_active)
        self.assertTrue(m.send_pending)
        self.assertEqual(len(m.nav.sent), 1)


if __name__ == '__main__':
    unittest.main(verbosity=2)
