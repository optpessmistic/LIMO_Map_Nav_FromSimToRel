"""Epoch regression with real block/cancel/result methods; no ROS or processes."""
import ast
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace as S
import unittest

import test_target_frame_association as fixtures
NODE_SOURCE = fixtures.NODE_SOURCE


class Future:
    def __init__(self):
        self.value, self.callbacks = None, []
    def result(self):
        return self.value
    def add_done_callback(self, callback):
        self.callbacks.append(callback)
    def resolve(self, value):
        self.value = value
        for callback in self.callbacks:
            callback(self)


class Handle:
    def __init__(self):
        self.accepted, self.cancel_calls = True, 0
        self.cancel_future, self.result_future = Future(), Future()
    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future
    def get_result_async(self):
        return self.result_future


class EpochCleanupTests(unittest.TestCase):
    def node(self):
        node, calls, warnings, published = fixtures.AdapterTests().node()
        node.phase = 'waiting'
        for offset in (0, 100_000_000, 200_000_000):
            node.on_observations(fixtures.AdapterTests().payload(
                [fixtures.detection(0.)], stamp={'sec': 1, 'nanosec': 123+offset}))
        self.assertEqual(len(node.targets.confirmed()), 1)
        tree = ast.parse((NODE_SOURCE).read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MissionNode')
        names = ('block', 'cancel_to', 'issue_cancel', 'on_cancel_response', 'on_goal_response', 'on_goal_result')
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
        clears, events = [], []
        env = {'Marker': type('Marker', (), {'DELETEALL': 3}), 'MarkerArray': S,
               'time': S(monotonic=lambda: 100.),
               'GoalStatus': S(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6)}
        exec(compile(ast.Module(body=methods, type_ignores=[]), 'actual_epoch_cleanup', 'exec'), env)
        for name in names:
            setattr(node, name, env[name].__get__(node))
        node.marker_pub = S(publish=clears.append)
        node.event = lambda name, **fields: events.append({'event': name, **fields})
        node.canceling, node.goal_handle, node.send_pending, node.token = False, None, False, 1
        node.cancel_response = node.cancel_terminal = False
        node.map_complete = node.visual_complete = True
        return node, calls, clears, events, published

    def test_prehome_confirmed_target_epoch_change_invalidates_and_clears_with_real_block(self):
        node, calls, clears, events, published = self.node()
        before = [asdict(target) for target in node.targets.targets]
        before_watermarks = dict(node.targets.frame_watermarks)
        node.on_epoch(S(data='map-b'))
        self.assertFalse(node.map_epoch_ready)
        self.assertFalse(node.coordinates_valid)
        self.assertFalse(node.map_complete)
        self.assertFalse(node.visual_complete)
        self.assertEqual(node.map_epoch, 'map-a')
        self.assertEqual(node.targets.frame_epoch, 'map-a')
        self.assertEqual(node.reason, 'target_map_epoch_changed_requires_new_mission')
        self.assertTrue(node.canceling)
        self.assertEqual(node.cancel_next, 'blocked')
        self.assertTrue(node.cancel_terminal)
        self.assertEqual(len(clears), 1)
        self.assertEqual([marker.action for marker in clears[0].markers], [3])
        self.assertEqual([asdict(target) for target in node.targets.targets], before)
        self.assertEqual(node.targets.frame_watermarks, before_watermarks)
        self.assertEqual(events[-1]['event'], 'cancel_requested')
        tf_calls, target_publications = len(calls), len(published)
        node.on_observations(fixtures.AdapterTests().payload([fixtures.detection(.18)], map_epoch='map-b'))
        self.assertEqual(len(calls), tf_calls)
        self.assertEqual(len(published), target_publications)
        self.assertEqual([asdict(target) for target in node.targets.targets], before)

    def test_repeated_or_terminal_phase_epoch_fault_clears_once_without_restarting_task(self):
        for phase in ('waiting', 'blocked', 'done'):
            with self.subTest(phase=phase):
                node, _, clears, events, _ = self.node()
                node.phase = phase
                node.on_epoch(S(data='map-b'))
                node.on_epoch(S(data='map-b'))
                self.assertFalse(node.coordinates_valid)
                self.assertFalse(node.map_epoch_ready)
                self.assertEqual(len(clears), 1)
                if phase in ('blocked', 'done'):
                    self.assertEqual(node.phase, phase)
                    self.assertEqual(events, [])
                else:
                    self.assertEqual(node.cancel_next, 'blocked')

    def test_pending_acceptance_keeps_owned_cancel_and_ack_is_not_terminal(self):
        node, _, clears, _, _ = self.node()
        node.send_pending = True
        node.on_epoch(S(data='map-b'))
        self.assertFalse(node.coordinates_valid)
        self.assertTrue(node.send_pending)
        self.assertFalse(node.cancel_terminal)
        response, handle = Future(), Handle()
        response.value = handle
        node.on_goal_response(response, node.token)
        self.assertFalse(node.send_pending)
        self.assertIs(node.goal_handle, handle)
        self.assertEqual(handle.cancel_calls, 1)
        handle.cancel_future.resolve(S(goals_canceling=[S()]))
        self.assertTrue(node.cancel_response)
        self.assertFalse(node.cancel_terminal)
        self.assertIs(node.goal_handle, handle)
        handle.result_future.resolve(S(status=5, result=S(error_msg='owned canceled')))
        self.assertTrue(node.cancel_terminal)
        self.assertIsNone(node.goal_handle)
        self.assertEqual(node.cancel_next, 'blocked')
        self.assertEqual(len(clears), 1)

    def test_same_epoch_remains_valid_and_noncoordinate_failure_does_not_clear(self):
        node, _, clears, _, _ = self.node()
        node.on_epoch(S(data='map-a'))
        self.assertTrue(node.map_epoch_ready)
        self.assertTrue(node.coordinates_valid)
        self.assertEqual(clears, [])
        node.block('map_stale')
        self.assertTrue(node.coordinates_valid)
        self.assertEqual(clears, [])
        self.assertEqual(node.cancel_next, 'blocked')


if __name__ == '__main__':
    unittest.main(verbosity=2)
