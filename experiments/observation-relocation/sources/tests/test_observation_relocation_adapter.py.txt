"""Relocated dispatches through the real adapter/ledger with synthetic ROS IO."""
import copy
from dataclasses import asdict, replace
import json
from types import SimpleNamespace as S
import unittest
from unittest.mock import patch

import test_observation_adapter as t


class RelocationAdapterTests(unittest.TestCase):
    def setUp(self):
        self.f = t.AdapterTests()
        self.f.setUp()

    def mission(self, **overrides):
        params = {'observation_relocation_radius_m': .15}
        params.update(overrides)
        return self.f.mission(**params)

    def freeze(self, m):
        self.f.tick(m)
        self.assertEqual(m.phase, 'observation_select')
        self.assertEqual(m.nav.sent, [])
        return m.observation_queue

    def update_map(self, m, *, stamp_ns=12_345_678_901, obstacle=True):
        grid = m.grid
        data = list(grid.data)
        if obstacle:
            first = next(iter(m.observation_queue.views.values()))
            cell = grid.to_cell(first.goal.x, first.goal.y)
            # 0.35 m from the original center: outside R=.25, inside
            # R+.05+half-diagonal. A neighbor 0.10 m away remains a safe endpoint.
            data[cell[1]*grid.width+cell[0]+4] = 100
        header = S(frame_id='map')
        if stamp_ns is not None:
            header.stamp = S(sec=stamp_ns//10**9, nanosec=stamp_ns%10**9)
        m.on_map(S(header=header, info=S(width=grid.width, height=grid.height, resolution=grid.resolution,
            origin=S(position=S(x=grid.origin_x, y=grid.origin_y),
                     orientation=S(x=0., y=0., z=0., w=1.))), data=data))

    def relocated(self, **overrides):
        m = self.mission(**overrides)
        queue = self.freeze(m)
        frozen = copy.deepcopy(queue.views)
        self.update_map(m)
        self.f.tick(m)
        self.assertTrue(m.send_pending)
        self.assertEqual(queue.views, frozen)
        planned = queue.views[m.observation_active['action_id']].goal
        self.assertNotEqual((m.goal.x, m.goal.y), (planned.x, planned.y))
        return m

    def process_one(self, m, sequence):
        t.TIME.advance(.12)
        self.f.odom(m)
        record = t.frame(t.TIME.ns-20_000_000, seq=sequence)
        m.on_frame_processing(S(data=json.dumps(record)))
        self.f.tick(m)
        return record

    def test_default_disabled_and_parameter_domain(self):
        m = t.adapter.MissionNode()
        self.assertEqual(m.cfg['observation_relocation_radius_m'], 0.)
        self.assertEqual(m.observation_policy.relocation_radius_m, 0.)
        for value in (-.01, float('nan'), float('inf'), True, '.15', .26, .51):
            with self.subTest(radius=value), self.assertRaises(ValueError):
                t.adapter.MissionNode(parameter_overrides={'observation_relocation_radius_m': value})
        with self.assertRaises(ValueError):
            t.adapter.MissionNode(parameter_overrides={
                'observation_station_spacing_m': .4, 'observation_relocation_radius_m': .11})
        edge = t.adapter.MissionNode(parameter_overrides={'observation_relocation_radius_m': .25})
        self.assertEqual(edge.observation_policy.relocation_radius_m, .25)

    def test_actual_map_stamp_and_missing_fixture_stamp_are_preserved(self):
        m = self.mission()
        queue = self.freeze(m)
        self.update_map(m, stamp_ns=None, obstacle=False)
        self.assertIsNone(m.map_stamp_ns)
        stamp = 12_345_678_901
        self.update_map(m, stamp_ns=stamp)
        selected = []
        original = queue.take_next
        def capture(*args, **kwargs):
            self.assertEqual(kwargs['map_stamp_ns'], stamp)
            decision = original(*args, **kwargs)
            selected.append(decision)
            return decision
        with patch.object(queue, 'take_next', side_effect=capture):
            self.f.tick(m)
        self.assertEqual(m.map_stamp_ns, stamp)
        self.assertEqual(m.observation_active['selection'], selected[0].validation)
        self.assertIsInstance(m.observation_active['selection'], dict)
        json.dumps(m.observation_active)  # Persistable provenance, not ROS objects.

    def test_relocated_nav_goal_and_new_ledger_use_actual_pose(self):
        m = self.relocated(camera_heading_offset_rad=.7)
        active = copy.deepcopy(m.observation_active)
        planned = m.observation_queue.views[active['action_id']].goal
        request = m.nav.sent[-1][0]
        self.assertEqual((request.pose.pose.position.x, request.pose.pose.position.y), (m.goal.x, m.goal.y))
        self.assertEqual(active['goal'], asdict(m.goal))
        self.assertEqual(active['planned_goal'], asdict(planned))
        self.assertEqual(m.goal.yaw, planned.yaw)  # BASE yaw; no second camera offset.
        self.assertEqual(m.observation_queue.active_view.goal, m.goal)
        self.assertTrue(m.observation_binding_valid())
        self.f.terminal(m)
        self.assertEqual(m.inspection.expected_pose, (m.goal.x, m.goal.y, m.goal.yaw))
        self.assertEqual(m.inspection.request_id, active['inspection_request_id'])
        self.assertFalse(m.inspection_pose_matches((planned.x, planned.y, planned.yaw)))
        self.f.settle(m)
        self.f.processed_window(m)
        record = m.observation_attempt_records[0]
        self.assertEqual(record['outcome'], 'inspection_satisfied')
        self.assertEqual(record['inspection']['expected_pose'], [active['goal'][k] for k in ('x', 'y', 'yaw')])
        self.assertEqual(record['inspection']['accepted_frames'], 3)
        self.assertEqual(record['planned_goal'], active['planned_goal'])
        self.assertEqual(record['selection'], active['selection'])
        self.assertFalse(m.visual_complete)

    def test_later_heading_can_relocate_without_rewriting_completed_station_view(self):
        m = self.mission()
        self.f.start(m)
        queue = m.observation_queue
        frozen, stations = copy.deepcopy(queue.views), tuple(queue.stations)
        first_active = copy.deepcopy(m.observation_active)
        self.assertEqual(first_active['goal'], first_active['planned_goal'])
        self.f.terminal(m)
        self.f.settle(m)
        self.process_one(m, 1)
        self.process_one(m, 2)
        self.update_map(m)
        old_record = self.process_one(m, 3)  # Completes old heading, then dispatches next.
        completed = copy.deepcopy(m.observation_attempt_records[0])
        second = copy.deepcopy(m.observation_active)
        self.assertEqual(completed['goal'], first_active['goal'])
        self.assertEqual(completed['planned_goal'], first_active['goal'])
        self.assertEqual(completed['station_id'], second['station_id'])
        self.assertNotEqual(completed['heading_index'], second['heading_index'])
        self.assertNotEqual(completed['nav_token'], second['nav_token'])
        self.assertNotEqual(completed['inspection_request_id'], second['inspection_request_id'])
        self.assertNotEqual(second['goal'], second['planned_goal'])
        self.assertEqual(queue.views, frozen)
        self.assertEqual(queue.stations, stations)
        self.assertEqual(queue.dispatches, 2)
        self.f.terminal(m)
        self.f.settle(m)
        m.on_frame_processing(S(data=json.dumps(old_record)))
        self.assertEqual(m.inspection.accepted_frames, 0)
        self.f.processed_window(m)
        self.assertEqual(m.observation_attempt_records[0], completed)
        self.assertEqual([r['outcome'] for r in m.observation_attempt_records],
                         ['inspection_satisfied', 'inspection_satisfied'])
        self.assertTrue(queue.snapshot()['all_planned_actions_completed'])
        self.assertFalse(m.visual_complete)

    def test_relocation_planning_crossing_original_deadline_cannot_send(self):
        m = self.mission(observation_stage_timeout_s=1.)
        queue = self.freeze(m)
        deadline, started = m.observation_deadline_wall, m.started
        self.update_map(m)
        original = queue.take_next
        selected = []
        def slow_selection(*args, **kwargs):
            decision = original(*args, **kwargs)
            self.assertEqual(decision.state, 'dispatch')
            self.assertNotEqual(decision.view.goal, queue.views[decision.view.action_id].goal)
            selected.append(decision)
            t.TIME.advance(1.1, ros=False)
            return decision
        with patch.object(queue, 'take_next', side_effect=slow_selection):
            self.f.tick(m)
        self.assertEqual(m.nav.sent, [])
        self.assertEqual(m.token, 0)
        self.assertEqual(queue.dispatches, 1)  # Reserved attempt is not refunded.
        self.assertEqual(m.observation_deadline_wall, deadline)
        self.assertEqual(m.started, started)
        self.assertTrue(m.observation_closed)
        self.assertIsNone(m.observation_active)
        self.assertEqual(m.cancel_next, 'returning')
        record = m.observation_attempt_records[0]
        self.assertEqual(record['outcome'], 'canceled')
        self.assertEqual(record['selection'], selected[0].validation)
        self.assertNotEqual(record['goal'], record['planned_goal'])
        self.assertIsNone(record['nav_token'])
        self.assertIsNone(record['dispatched_wall'])
        self.assertIsNone(record['inspection'])

    def test_active_dispatch_goal_mismatch_blocks_inspection_credit(self):
        for missing in (False, True):
            with self.subTest(missing=missing):
                self.setUp()
                m = self.relocated()
                queue = m.observation_queue
                if missing:
                    queue.active_view = None
                else:
                    queue.active_view = replace(queue.active_view,
                        goal=replace(queue.active_view.goal, x=queue.active_view.goal.x+.01))
                self.assertFalse(m.observation_binding_valid())
                self.f.terminal(m)
                self.assertTrue(m.observation_closed)
                self.assertIn('observation_binding_invalid', m.reason)
                self.assertFalse(queue.snapshot()['all_planned_actions_completed'])
                self.assertTrue(all(r['outcome'] != 'inspection_satisfied' for r in m.observation_attempt_records))


if __name__ == '__main__':
    unittest.main()
