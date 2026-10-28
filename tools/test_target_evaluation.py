import contextlib
import copy
import io
import itertools
import json
import math
from pathlib import Path
import random
import tempfile
import unittest

from evaluate_targets import SE2, evaluate, load_json, main


def prediction(uid, x, y=0.0, class_id='red_object'):
    return {'uid': uid, 'class_id': class_id, 'x': x, 'y': y, 'z': .1, 'confirmed': True}


def truth(x, y=0.0, appearance='red_box_surrogate'):
    return {'x': x, 'y': y, 'z': .1, 'appearance': appearance}


def mission(predictions):
    return {'schema_version': 1, 'frame_id': 'map', 'map_epoch': 'trial-1',
            'coordinates_valid': True, 'status': 'done', 'mission_success': False,
            'targets': predictions}


def score(predictions, truths, threshold=.2, alignment=None):
    return evaluate(mission(predictions), {'targets_ground_truth': truths},
                    alignment=SE2(0, 0, 0) if alignment is None else alignment,
                    alignment_source='test fixture independently defined frames',
                    appearance_to_class={'red_box_surrogate': 'red_object'},
                    max_distance_m=threshold)


class TargetEvaluationTests(unittest.TestCase):
    def test_merged_instance_leaves_one_false_negative(self):
        result = score([prediction(1, .05)], [truth(0), truth(.1)])
        self.assertEqual((result['TP'], result['FP'], result['FN']), (1, 0, 1))
        self.assertEqual(result['precision'], 1.0)
        self.assertEqual(result['recall'], .5)

    def test_duplicate_predictions_are_false_positives(self):
        result = score([prediction(1, .01), prediction(2, .03)], [truth(0)])
        self.assertEqual((result['TP'], result['FP'], result['FN']), (1, 1, 0))
        self.assertEqual(result['unmatched_prediction_uids'], [2])
        self.assertEqual(result['precision'], .5)
        self.assertAlmostEqual(result['position_error_m']['mean'], .01)

    def test_cross_matching_prioritizes_cardinality_over_nearest_pair(self):
        # P1 can reach T0/T1, while P2 can only reach T0. Greedy P1->T0 loses one.
        result = score([prediction(1, .1), prediction(2, -.2)], [truth(0), truth(.4)], .31)
        self.assertEqual(result['TP'], 2)
        self.assertEqual({m['prediction_uid']: m['truth_index'] for m in result['matches']}, {1: 1, 2: 0})

    def test_equal_cardinality_minimizes_total_distance(self):
        result = score([prediction(1, .6), prediction(2, .1)], [truth(0), truth(1)], 2)
        self.assertEqual({m['prediction_uid']: m['truth_index'] for m in result['matches']}, {1: 1, 2: 0})
        self.assertAlmostEqual(sum(m['distance_m'] for m in result['matches']), .5)

    def test_zero_truth_and_zero_predictions_are_not_recall_success(self):
        result = score([], [])
        self.assertEqual((result['TP'], result['FP'], result['FN']), (0, 0, 0))
        self.assertIsNone(result['precision'])
        self.assertIsNone(result['recall'])
        self.assertTrue(result['zero_target_case'])
        self.assertTrue(result['target_inventory_exact'])
        self.assertTrue(all(v is None for v in result['position_error_m'].values()))
        self.assertFalse(result['reported_mission_success'])

    def test_zero_truth_with_predictions_counts_false_positives(self):
        result = score([prediction(1, 0)], [])
        self.assertEqual(result['FP'], 1)
        self.assertEqual(result['precision'], 0)
        self.assertIsNone(result['recall'])
        self.assertFalse(result['target_inventory_exact'])

    def test_no_predictions_means_all_truths_missed(self):
        result = score([], [truth(0), truth(1)])
        self.assertEqual(result['FN'], 2)
        self.assertIsNone(result['precision'])
        self.assertEqual(result['recall'], 0)

    def test_rotation_translation_direction(self):
        result = score([prediction(1, 1, 2)], [truth(8, 21)], .01, SE2(10, 20, math.pi/2))
        self.assertEqual(result['TP'], 1)
        self.assertAlmostEqual(result['position_error_m']['max'], 0)
        self.assertAlmostEqual(result['transformed_predictions'][0]['world_x'], 8)

    def test_threshold_inclusive_and_z_does_not_change_declared_planar_metric(self):
        p = prediction(1, .2)
        p['z'] = 5
        self.assertEqual(score([p], [truth(0)], .2)['TP'], 1)
        self.assertEqual(score([p], [truth(0)], .199)['TP'], 0)

    def test_unknown_prediction_class_is_never_mixed(self):
        result = score([prediction(1, 0, class_id='unknown')], [truth(0)])
        self.assertEqual((result['TP'], result['FP'], result['FN']), (0, 1, 1))

    def test_missing_truth_mapping_is_rejected_even_with_no_predictions(self):
        with self.assertRaisesRegex(ValueError, 'No explicit class mapping'):
            score([], [truth(0, appearance='unknown')])

    def test_unconfirmed_candidates_do_not_count_as_predictions(self):
        data = mission([])
        candidate = prediction(1, 0)
        candidate['confirmed'] = False
        data['unconfirmed_target_candidates'] = [candidate]
        result = self.evaluate_data(data)
        self.assertEqual(result['prediction_count'], 0)
        self.assertEqual(result['FN'], 1)

    @staticmethod
    def evaluate_data(data, metadata=None):
        return evaluate(data, {'targets_ground_truth': [truth(0)]} if metadata is None else metadata,
                        alignment=SE2(0, 0, 0), alignment_source='test',
                        appearance_to_class={'red_box_surrogate': 'red_object'}, max_distance_m=.2)

    def test_invalid_mission_coordinates_flags_or_running_status_are_rejected(self):
        for key, value in [('coordinates_valid', False), ('coordinates_valid', 1),
                           ('status', 'exploring'), ('schema_version', True),
                           ('mission_success', 'false'), ('frame_id', '')]:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                data = mission([prediction(1, 0)])
                data[key] = value
                self.evaluate_data(data)
        data = mission([prediction(1, 0)])
        data['targets'][0]['confirmed'] = False
        with self.assertRaises(ValueError):
            self.evaluate_data(data)

    def test_blocked_result_can_measure_partial_targets_without_passing_mission(self):
        data = mission([prediction(1, 0)])
        data['status'] = 'blocked'
        result = self.evaluate_data(data)
        self.assertEqual(result['TP'], 1)
        self.assertFalse(result['reported_mission_success'])

    def test_duplicate_prediction_and_candidate_uids_are_rejected(self):
        with self.assertRaisesRegex(ValueError, 'Duplicate target uid'):
            score([prediction(1, 0), prediction(1, 1)], [truth(0)])
        data = mission([prediction(1, 0)])
        candidate = prediction(1, 1)
        candidate['confirmed'] = False
        data['unconfirmed_target_candidates'] = [candidate]
        with self.assertRaisesRegex(ValueError, 'Duplicate target uid'):
            self.evaluate_data(data)

    def test_duplicate_ground_truth_uid_is_rejected(self):
        truths = [dict(truth(0), uid=5), dict(truth(1), uid=5)]
        with self.assertRaisesRegex(ValueError, 'Duplicate ground-truth uid'):
            score([], truths)

    def test_nonfinite_numbers_are_rejected_everywhere(self):
        for value in [math.nan, math.inf, -math.inf]:
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    score([prediction(1, value)], [truth(0)])
                with self.assertRaises(ValueError):
                    score([], [truth(value)])
                with self.assertRaises(ValueError):
                    SE2(value, 0, 0)
                with self.assertRaises(ValueError):
                    score([], [], value)
                data = mission([])
                data['events'] = [{'stamp': value}]
                with self.assertRaises(ValueError):
                    self.evaluate_data(data)
        with self.assertRaises(ValueError):
            score([prediction(1, True)], [])

    def test_json_nonfinite_overflow_and_duplicate_keys_are_rejected(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as tmp:
            path = Path(tmp)/'bad.json'
            for content in ['{"x":NaN}', '{"x":Infinity}', '{"x":1e309}', '{"x":1,"x":2}']:
                with self.subTest(content=content), self.assertRaises(ValueError):
                    path.write_text(content, encoding='utf-8')
                    load_json(path)

    def test_alignment_has_no_implicit_identity(self):
        with self.assertRaises(TypeError):
            SE2()
        with self.assertRaises(TypeError):
            evaluate(mission([]), {'targets_ground_truth': []},
                     appearance_to_class={}, max_distance_m=.2)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
            main(['mission.json', 'truth.json', '--max-distance-m', '.2'])
        self.assertEqual(raised.exception.code, 2)

    def test_cli_round_trip_and_refuses_output_overwrite(self):
        with tempfile.TemporaryDirectory(dir=Path(__file__).resolve().parent) as tmp:
            source, metadata, output = [Path(tmp)/name for name in ['mission.json', 'truth.json', 'score.json']]
            source.write_text(json.dumps(mission([prediction(1, 0)])), encoding='utf-8')
            metadata.write_text(json.dumps({'targets_ground_truth': [truth(1)]}), encoding='utf-8')
            args = [str(source), str(metadata), '--map-to-world-x', '1', '--map-to-world-y', '0',
                    '--map-to-world-yaw', '0', '--alignment-source', 'fixture',
                    '--class-map', 'red_box_surrogate=red_object', '--max-distance-m', '.2', '--output', str(output)]
            self.assertEqual(main(args), 0)
            self.assertEqual(load_json(output)['TP'], 1)
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                main(args)
            self.assertEqual(raised.exception.code, 2)

    def test_small_random_cases_agree_with_exhaustive_optimum(self):
        rng = random.Random(1923)
        # Independent enumeration verifies both objectives, including residual reassignment.
        for case in range(80):
            predictions = [prediction(i, rng.random(), rng.random()) for i in range(rng.randrange(5))]
            truths = [truth(rng.random(), rng.random()) for _ in range(rng.randrange(5))]
            threshold = .65
            best_count, best_cost = 0, 0.0
            for count in range(1, min(len(predictions), len(truths))+1):
                for pids in itertools.combinations(range(len(predictions)), count):
                    for tids in itertools.permutations(range(len(truths)), count):
                        distances = [math.hypot(predictions[i]['x']-truths[j]['x'], predictions[i]['y']-truths[j]['y'])
                                     for i, j in zip(pids, tids)]
                        if max(distances) <= threshold and (count > best_count or sum(distances) < best_cost):
                            best_count, best_cost = count, sum(distances)
            result = score(copy.deepcopy(predictions), truths, threshold)
            with self.subTest(case=case):
                self.assertEqual(result['TP'], best_count)
                self.assertAlmostEqual(sum(m['distance_m'] for m in result['matches']), best_cost)


if __name__ == '__main__':
    unittest.main()
