"""Synthetic geometry and adapter tests only; no ROS/Gazebo/truth inputs."""
import ast
import copy
from dataclasses import asdict
import itertools
import json
import math
from pathlib import Path
import random
import sys
from types import SimpleNamespace as NS
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/limo_maze_mission'))
from limo_maze_mission import core
NODE_SOURCE = Path(__file__).resolve().parents[1]/'src/limo_maze_mission/limo_maze_mission/node.py'

N = 1_000_000_000


def detection(x, y=0., z=.1, cls='red_object', confidence=.95, instance=''):
    return {'class_id': cls, 'position': [x, y, z], 'confidence': confidence, 'instance_id': instance}


def frame(store, items, stamp=1, **kwargs):
    return store.add_frame(items, stamp*N, source=kwargs.get('source', 'detector'),
        sensor_frame=kwargs.get('sensor_frame', 'camera_optical_frame'), epoch=kwargs.get('epoch', 'map-a'))


def snapshot(store):
    return ([asdict(t) for t in store.targets], store.next_uid, store.frame_epoch, dict(store.frame_watermarks))


class AssociationTests(unittest.TestCase):
    def test_two_close_observations_remain_two_tracks_over_three_frames(self):
        store = core.TargetStore()
        for stamp in (1, 2, 3):
            result = frame(store, [detection(1., -.09), detection(1., .09)], stamp)
            self.assertTrue(result['accepted'])
            self.assertEqual(len({a['uid'] for a in result['assignments']}), 2)
            self.assertFalse(result['physical_identity_proven'])
        self.assertEqual(len(store.confirmed()), 2)
        self.assertEqual([t.hits for t in store.targets], [3, 3])
        self.assertEqual(store.merge_radius, .30)

    def test_detection_order_reversal_preserves_uids_positions_and_hits(self):
        a, b = core.TargetStore(), core.TargetStore()
        for stamp, offset in ((1, 0.), (2, .01), (3, -.01)):
            items = [detection(1.+offset, -.09), detection(1.+offset, .09)]
            frame(a, items, stamp)
            frame(b, list(reversed(items)), stamp)
            self.assertEqual(snapshot(a), snapshot(b))

    def test_new_close_second_detection_gets_a_new_track(self):
        store = core.TargetStore()
        frame(store, [detection(1., -.09)])
        result = frame(store, [detection(1., .09), detection(1., -.09)], 2)
        self.assertEqual((result['matched_tracks'], result['new_tracks']), (1, 1))
        self.assertEqual([t.hits for t in store.targets], [2, 1])

    def test_single_target_jitter_stays_one_track_with_original_weighting(self):
        store = core.TargetStore()
        for stamp, x in enumerate((1., 1.12, .9, 1.03), 1):
            frame(store, [detection(x)], stamp)
        self.assertEqual(len(store.targets), 1)
        self.assertEqual(store.targets[0].hits, 4)
        self.assertAlmostEqual(store.targets[0].x, (1+1.12+.9+1.03)/4)
        self.assertTrue(store.targets[0].confirmed)

    def test_cardinality_priority_avoids_greedy_nearest_trap(self):
        store = core.TargetStore()
        frame(store, [detection(0.), detection(.3)])
        result = frame(store, [detection(.15), detection(-.2)], 2)
        self.assertEqual((result['matched_tracks'], result['new_tracks']), (2, 0))
        self.assertEqual([a['uid'] for a in result['assignments']], [2, 1])

    def test_radius_gate_is_inclusive_and_outlier_creates_separate_unconfirmed_track(self):
        store = core.TargetStore()
        frame(store, [detection(0.)])
        self.assertEqual(frame(store, [detection(.3)], 2)['matched_tracks'], 1)
        self.assertEqual(frame(store, [detection(2.)], 3)['new_tracks'], 1)
        self.assertFalse(store.confirmed())

    def test_confirmation_window_expiry_resets_only_unconfirmed_track(self):
        store = core.TargetStore()
        frame(store, [detection(0.)], 1)
        frame(store, [detection(.02)], 2)
        frame(store, [detection(.04)], 10)
        self.assertEqual((store.targets[0].hits, store.targets[0].first_stamp), (1, 10.))
        self.assertAlmostEqual(store.targets[0].x, .04)
        for stamp in (11, 12, 99):
            frame(store, [detection(.04)], stamp)
        self.assertEqual(store.targets[0].hits, 4)
        self.assertTrue(store.targets[0].confirmed)

    def test_repeated_and_reordered_same_stamp_never_increment_or_create(self):
        store = core.TargetStore()
        items = [detection(0.), detection(.18)]
        frame(store, items)
        before = snapshot(store)
        for _ in range(4):
            result = frame(store, list(reversed(items)))
            self.assertEqual(result['reason'], 'duplicate_frame')
            self.assertEqual(snapshot(store), before)

    def test_conflicting_same_stamp_and_old_frame_are_atomic_rejections(self):
        store = core.TargetStore()
        frame(store, [detection(0.)], 2)
        before = snapshot(store)
        self.assertEqual(frame(store, [detection(9.)], 2)['reason'], 'conflicting_duplicate_frame')
        self.assertEqual(frame(store, [detection(9.)], 1)['reason'], 'nonadvancing_frame')
        self.assertEqual(snapshot(store), before)

    def test_cross_stream_equal_stamp_does_not_count_twice(self):
        store = core.TargetStore()
        frame(store, [detection(0.)], source='one')
        result = frame(store, [detection(0.)], source='two')
        self.assertFalse(result['assignments'][0]['hit_incremented'])
        self.assertEqual(store.targets[0].hits, 1)

    def test_multiple_classes_and_different_instance_ids_do_not_merge(self):
        store = core.TargetStore(min_hits=1)
        result = frame(store, [detection(0., cls='red'), detection(0., cls='blue'),
                               detection(.01, cls='tag', instance='A'), detection(.02, cls='tag', instance='B')])
        self.assertTrue(result['accepted'])
        self.assertEqual(len(store.confirmed()), 4)

    def test_reused_instance_id_cannot_jump_spatial_gate(self):
        store = core.TargetStore()
        frame(store, [detection(0., instance='A')])
        result = frame(store, [detection(2., instance='A')], 2)
        self.assertEqual(result['new_tracks'], 1)
        self.assertEqual(store.targets[0].x, 0.)

    def test_indistinguishable_or_duplicate_id_frame_fails_without_identity_claim(self):
        for observations, reason in (
            ([detection(0.), detection(0., confidence=.8)], 'indistinguishable_observations_in_frame'),
            ([detection(0., instance='A'), detection(.1, instance='A')], 'duplicate_instance_id_in_frame')):
            store = core.TargetStore()
            before = snapshot(store)
            self.assertEqual(frame(store, observations)['reason'], reason)
            self.assertEqual(snapshot(store), before)

    def test_low_confidence_ignored_and_empty_frames_have_no_target_count_prior(self):
        store = core.TargetStore()
        result = frame(store, [detection(0., confidence=.2), detection(.18)])
        self.assertEqual(len(result['ignored']), 1)
        self.assertEqual(len(store.targets), 1)
        self.assertTrue(frame(store, [], 2)['accepted'])
        self.assertEqual(len(store.targets), 1)

    def test_epoch_mismatch_preserves_original_track_and_watermark(self):
        store = core.TargetStore()
        frame(store, [detection(0.)])
        before = snapshot(store)
        self.assertEqual(frame(store, [detection(0.)], 2, epoch='map-b')['reason'],
                         'map_epoch_changed_requires_new_store')
        self.assertEqual(snapshot(store), before)

    def test_all_invalid_payloads_reject_without_partial_first_item_mutation(self):
        bad = [detection(math.nan), detection(math.inf), detection(1e6+1),
               detection(0., confidence=1.1), detection(0., cls=''),
               {'class_id': 'x', 'position': [1., 2.], 'confidence': .9},
               detection(0., confidence=True), detection(0., instance='x'*129)]
        for value in bad:
            store = core.TargetStore()
            before = snapshot(store)
            self.assertFalse(frame(store, [detection(0.), value])['accepted'])
            self.assertEqual(snapshot(store), before)

    def test_detection_track_and_stream_limits_are_atomic(self):
        store = core.TargetStore(max_detections=2, max_tracks=2, max_streams=1)
        before = snapshot(store)
        self.assertEqual(frame(store, [detection(i) for i in range(3)])['reason'], 'detection_count_limit')
        self.assertEqual(snapshot(store), before)
        frame(store, [detection(0.), detection(1.)])
        before = snapshot(store)
        self.assertEqual(frame(store, [detection(0.), detection(2.)], 2)['reason'], 'track_count_limit')
        self.assertEqual(frame(store, [], 2, source='other')['reason'], 'stream_count_limit')
        self.assertEqual(snapshot(store), before)

    def test_frame_identity_limits(self):
        for stamp in (True, -1, 2**63, .5):
            store = core.TargetStore()
            result = store.add_frame([], stamp, source='s', sensor_frame='f', epoch='e')
            self.assertEqual(result['reason'], 'invalid_frame_identity')
        self.assertEqual(frame(core.TargetStore(), [], source='s'*257)['reason'], 'invalid_frame_identity')

    def test_nanosecond_distinct_frames_advance_even_when_float_seconds_collapse(self):
        # Frame replay keys preserve integer identity; legacy per-track float
        # timestamps still conservatively avoid extra hits at collapsed values.
        store = core.TargetStore()
        first = 9_000_000_000_000_000_000
        for ns in (first, first+1):
            self.assertTrue(store.add_frame([detection(0.)], ns, source='s', sensor_frame='f', epoch='e')['accepted'])
        self.assertEqual(store.targets[0].hits, 1)

    def test_maximum_frame_and_track_capacity_are_supported_without_count_prior(self):
        store = core.TargetStore()
        for stamp, start in ((1, 0), (2, 100), (3, 200)):
            count = min(100, 256-start)
            result = frame(store, [detection(float(i)) for i in range(start, start+count)], stamp)
            self.assertTrue(result['accepted'])
        self.assertEqual(len(store.targets), 256)
        result = frame(store, [detection(float(i)) for i in range(100)], 4)
        self.assertEqual(result['matched_tracks'], 100)
        self.assertEqual(result['new_tracks'], 0)

    def test_legacy_single_add_confirmation_and_replay_semantics_remain(self):
        store = core.TargetStore()
        for stamp in (1., 2., 3.):
            store.add('object', (1, 1, .2), stamp, .9)
            store.add('object', (3, 1, .2), stamp, .9)
        self.assertEqual(len(store.confirmed()), 2)
        store.add('object', (1, 1, .2), 3., .9)
        self.assertEqual(store.targets[0].hits, 3)


class AssignmentTests(unittest.TestCase):
    def test_rectangular_solver_matches_exhaustive_small_matrices(self):
        rng = random.Random(317)
        for n in range(1, 5):
            for _ in range(15):
                m = n+1
                costs = [[float(rng.randrange(6)) for _ in range(m)] for _ in range(n)]
                result = core._minimum_cost_columns(costs)
                value = sum(costs[i][j] for i, j in enumerate(result))
                expected = min(sum(costs[i][j] for i, j in enumerate(p)) for p in itertools.permutations(range(m), n))
                self.assertEqual(value, expected)
                self.assertEqual(len(set(result)), n)
                self.assertEqual(result, core._minimum_cost_columns(costs))


class AdapterTests(unittest.TestCase):
    def node(self):
        tree = ast.parse((NODE_SOURCE).read_text())
        cls = next(item for item in tree.body if isinstance(item, ast.ClassDef) and item.name == 'MissionNode')
        names = ('on_observations', 'on_epoch', 'bounded_observation_enabled')
        methods = [item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name in names]
        env = {'json': json, 'isfinite': math.isfinite, 'hypot': math.hypot, 'atan2': math.atan2,
               'rotate': lambda q, p: p, 'Time': lambda **kw: NS(**kw), 'time': NS(monotonic=lambda: 100.),
               'TransformException': RuntimeError}
        exec(compile(ast.Module(body=methods, type_ignores=[]), 'draft_adapter', 'exec'), env)
        calls, warnings, published = [], [], []
        def lookup(target, source, time):
            calls.append(time.nanoseconds)
            return NS(transform=NS(translation=NS(x=0., y=0., z=0.), rotation=NS()))
        node = NS(grid=object(), phase='exploring', cancel_next=None, map_epoch_ready=True,
            coordinates_valid=True, map_epoch='map-a', targets=core.TargetStore(), home=None,
            observation_active=None,
            cfg={'observation_max_age_s': 1.5, 'map_frame': 'map'}, seconds=lambda: 1.5,
            tf_buffer=NS(lookup_transform=lookup), sources=set(),
            publish_targets=lambda: published.append(True), get_logger=lambda: NS(warning=warnings.append),
            block=lambda reason: warnings.append(reason))
        for name in names:
            setattr(node, name, env[name].__get__(node))
        return node, calls, warnings, published

    def payload(self, detections, **extra):
        return NS(data=json.dumps({'schema_version': 1, 'stamp': {'sec': 1, 'nanosec': 123},
            'frame_id': 'camera', 'source': 'detector', 'detections': detections, **extra}))

    def test_adapter_batches_two_objects_and_preserves_integer_tf_stamp(self):
        node, calls, warnings, published = self.node()
        node.on_observations(self.payload([detection(0.), detection(.18)]))
        self.assertEqual(calls, [1_000_000_123])
        self.assertEqual(len(node.targets.targets), 2)
        self.assertFalse(warnings)
        self.assertTrue(published)

    def test_bad_second_observation_does_not_partially_add_first(self):
        node, *_ = self.node()
        node.on_observations(self.payload([detection(0.), detection(math.nan)]))
        self.assertFalse(node.targets.targets)

    def test_epoch_and_coordinate_guard_prevents_any_tf_or_mutation(self):
        node, calls, *_ = self.node()
        node.coordinates_valid = False
        node.on_observations(self.payload([detection(0.)]))
        self.assertFalse(calls)
        node.coordinates_valid = True
        node.on_observations(self.payload([detection(0.)], map_epoch='old'))
        self.assertFalse(calls)

    def test_prehome_epoch_change_after_targets_bound_requires_new_mission(self):
        node, _, warnings, _ = self.node()
        node.on_observations(self.payload([detection(0.)]))
        node.on_epoch(NS(data='map-b'))
        self.assertFalse(node.map_epoch_ready)
        self.assertEqual(node.map_epoch, 'map-a')
        self.assertIn('target_map_epoch_changed_requires_new_mission', warnings)

    def test_out_of_bounds_numeric_stamp_is_cleanly_rejected(self):
        node, calls, warnings, _ = self.node()
        node.on_observations(self.payload([detection(0.)], stamp=1e308))
        self.assertFalse(calls)
        self.assertTrue(warnings)

    def test_existing_minimal_startup_adapter_without_target_store_still_accepts_epoch(self):
        node, *_ = self.node()
        del node.targets
        node.on_epoch(NS(data='map-b'))
        self.assertTrue(node.map_epoch_ready)
        self.assertEqual(node.map_epoch, 'map-b')


if __name__ == '__main__':
    unittest.main(verbosity=2)
