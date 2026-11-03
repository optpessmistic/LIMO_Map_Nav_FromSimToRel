#!/usr/bin/env python3
r"""Offline target inventory evaluation; never import this into a robot controller.

Usage (the zero transform below is an EXAMPLE, not an assumed alignment)::

    python tools/evaluate_targets.py mission.json worlds/example_maze.metadata.json \
        --map-to-world-x 0 --map-to-world-y 0 --map-to-world-yaw 0 \
        --alignment-source "independent surveyed start-frame calibration" \
        --class-map red_box_surrogate=red_object --max-distance-m 0.20

All SE(2) parameters are mandatory, in metres/radians, with direction
world_xy = R(yaw) @ map_xy + [x, y]. Obtain alignment independently of target
locations and for the same map session/epoch. This tool never fits alignment.
Matching uses planar XY distance, maximizes matched instances first, then
minimizes their total distance. All ground-truth appearances require an explicit
class mapping; unknown prediction classes remain false positives.

Only confirmed targets in terminal mission results with valid coordinates are
accepted. A failed/blocked mission can still have evaluable target measurements;
mission_success need not be true. Unconfirmed candidates are not predictions.
NaN, infinity, duplicate JSON keys and duplicate instance UIDs are rejected.

This is NOT evidence of map/visual coverage, autonomous execution, or return
home. Zero-truth recall and zero-prediction precision are null (undefined), not
100%. With no truths and no predictions, the inventory is exact but this alone
cannot establish that an unknown world was fully searched. Define the target
reference point and tolerance before trials: the present RGB-D baseline reports
a visible surface point whereas world metadata stores a box centre; no hidden
surface-to-centre correction is applied here.
"""

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import statistics
import sys


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a finite number')
    try:
        result = float(value)
    except OverflowError as exc:
        raise ValueError(f'{name} must be finite') from exc
    if not math.isfinite(result):
        raise ValueError(f'{name} must be finite')
    return result


def _text(value, name):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{name} must be a nonempty string')
    return value


def _validate_finite(value, path='input'):
    if isinstance(value, dict):
        for key, child in value.items():
            _validate_finite(child, f'{path}.{key}')
    elif isinstance(value, list):
        for i, child in enumerate(value):
            _validate_finite(child, f'{path}[{i}]')
    elif isinstance(value, (int, float)) and not isinstance(value, bool):
        _number(value, path)


def _unique_keys(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'Duplicate JSON key: {key}')
        result[key] = value
    return result


def load_json(path):
    """Reject nonstandard constants and numeric overflow, including metadata."""
    value = json.loads(Path(path).read_text(encoding='utf-8-sig'),
                       object_pairs_hook=_unique_keys)
    _validate_finite(value)
    return value


@dataclass(frozen=True)
class SE2:
    x: float
    y: float
    yaw: float

    def __post_init__(self):
        for name in ('x', 'y', 'yaw'):
            object.__setattr__(self, name, _number(getattr(self, name), name))

    def transform(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (_number(c*x-s*y+self.x, 'transformed x'),
                _number(s*x+c*y+self.y, 'transformed y'))


@dataclass
class _Edge:
    to: int
    reverse: int
    capacity: int
    cost: int


def _matching(predictions, truths, max_distance_m):
    """Unit-capacity min-cost max-flow, with Bellman-Ford residual paths.

    Successive shortest augmentations also allow reassignment of earlier
    matches. Stopping only when no augmenting path exists gives maximum
    cardinality; shortest paths give minimum total cost at that cardinality.
    Intended for small experimental inventories, not dense scene benchmarks.
    """
    n, m = len(predictions), len(truths)
    source, sink = n+m, n+m+1
    graph = [[] for _ in range(n+m+2)]

    def edge(u, v, cost):
        forward = _Edge(v, len(graph[v]), 1, cost)
        reverse = _Edge(u, len(graph[u]), 0, -cost)
        graph[u].append(forward)
        graph[v].append(reverse)
        return forward

    candidates, distances_by_pair = [], []
    for i, prediction in enumerate(predictions):
        edge(source, i, 0)
        for j, truth in enumerate(truths):
            if prediction['class_id'] != truth['class_id']:
                continue
            distance = _number(math.hypot(prediction['world_x']-truth['x'],
                                          prediction['world_y']-truth['y']), 'distance')
            if distance <= max_distance_m:
                distances_by_pair.append((i, j, distance))
    # Every finite float is a dyadic rational. A shared integer scale preserves
    # these costs exactly, avoiding spurious negative residual cycles caused by
    # floating-point addition/subtraction on almost-tied assignments.
    scale = max((d.as_integer_ratio()[1] for _, _, d in distances_by_pair), default=1)
    for i, j, distance in distances_by_pair:
        numerator, denominator = distance.as_integer_ratio()
        candidates.append((i, j, distance, edge(i, n+j, numerator*(scale//denominator))))
    for j in range(m):
        edge(n+j, sink, 0)

    while True:
        distances = [math.inf] * len(graph)
        previous = [None] * len(graph)
        distances[source] = 0
        for _ in range(len(graph)-1):
            changed = False
            for u, edges in enumerate(graph):
                if distances[u] == math.inf:
                    continue
                for k, candidate in enumerate(edges):
                    distance = distances[u]+candidate.cost
                    if candidate.capacity and distance < distances[candidate.to]:
                        distances[candidate.to] = distance
                        previous[candidate.to] = (u, k)
                        changed = True
            if not changed:
                break
        if previous[sink] is None:
            break
        v = sink
        while v != source:
            u, k = previous[v]
            selected = graph[u][k]
            selected.capacity -= 1
            graph[v][selected.reverse].capacity += 1
            v = u
    return [(i, j, distance) for i, j, distance, e in candidates if e.capacity == 0]


def _uid(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f'{name} must be a nonnegative integer')
    return value


def evaluate(mission, metadata, *, alignment, alignment_source,
             appearance_to_class, max_distance_m):
    """Return JSON-safe metrics. Alignment and class mappings have no defaults."""
    _validate_finite(mission, 'mission')
    _validate_finite(metadata, 'metadata')
    if not isinstance(alignment, SE2):
        raise ValueError('An explicit SE2 map-to-world alignment is required')
    _text(alignment_source, 'alignment_source')
    threshold = _number(max_distance_m, 'max_distance_m')
    if threshold <= 0:
        raise ValueError('max_distance_m must be positive')
    if not isinstance(mission, dict) or not isinstance(metadata, dict):
        raise ValueError('Mission and metadata must be JSON objects')
    if type(mission.get('schema_version')) is not int or mission['schema_version'] != 1:
        raise ValueError('Only mission schema_version 1 is supported')
    if mission.get('coordinates_valid') is not True:
        raise ValueError('Mission coordinates are invalid or unverified')
    if mission.get('status') not in ('done', 'blocked'):
        raise ValueError('Mission must be terminal (done or blocked)')
    if not isinstance(mission.get('mission_success'), bool):
        raise ValueError('mission_success must be a boolean; false is permitted')
    frame = _text(mission.get('frame_id'), 'mission.frame_id')
    if not isinstance(appearance_to_class, dict):
        raise ValueError('appearance_to_class must be an explicit mapping')
    for appearance, class_id in appearance_to_class.items():
        _text(appearance, 'appearance mapping key')
        _text(class_id, 'appearance mapping class')

    predictions, seen = [], set()
    for key, must_be_confirmed in [('targets', True), ('unconfirmed_target_candidates', False)]:
        targets = mission.get(key, [] if not must_be_confirmed else None)
        if not isinstance(targets, list):
            raise ValueError(f'mission.{key} must be an array')
        for target in targets:
            if not isinstance(target, dict):
                raise ValueError(f'mission.{key} entries must be objects')
            uid = _uid(target.get('uid'), 'target.uid')
            if uid in seen:
                raise ValueError(f'Duplicate target uid: {uid}')
            seen.add(uid)
            if target.get('confirmed') is not must_be_confirmed:
                raise ValueError(f'Invalid confirmed flag in mission.{key}')
            class_id = _text(target.get('class_id'), 'target.class_id')
            xyz = [_number(target.get(axis), 'target.'+axis) for axis in ('x', 'y', 'z')]
            if must_be_confirmed:
                x, y = alignment.transform(*xyz[:2])
                predictions.append({'uid': uid, 'class_id': class_id,
                                    'map_xyz': xyz, 'world_x': x, 'world_y': y})

    raw_truths = metadata.get('targets_ground_truth')
    if not isinstance(raw_truths, list):
        raise ValueError('metadata.targets_ground_truth must be an array')
    truths, truth_uids = [], set()
    for i, target in enumerate(raw_truths):
        if not isinstance(target, dict):
            raise ValueError('Ground-truth entries must be objects')
        if 'uid' in target:
            uid = _uid(target['uid'], 'ground_truth.uid')
            if uid in truth_uids:
                raise ValueError(f'Duplicate ground-truth uid: {uid}')
            truth_uids.add(uid)
        appearance = _text(target.get('appearance'), 'ground_truth.appearance')
        if appearance not in appearance_to_class:
            raise ValueError(f'No explicit class mapping for appearance: {appearance}')
        xyz = [_number(target.get(axis), 'ground_truth.'+axis) for axis in ('x', 'y', 'z')]
        truths.append({'index': i, 'class_id': appearance_to_class[appearance],
                       'appearance': appearance, 'x': xyz[0], 'y': xyz[1], 'z': xyz[2]})

    pairs = _matching(predictions, truths, threshold)
    matched_predictions = {i for i, _, _ in pairs}
    matched_truths = {j for _, j, _ in pairs}
    errors = [distance for _, _, distance in pairs]
    tp, fp, fn = len(pairs), len(predictions)-len(pairs), len(truths)-len(pairs)
    return {
        'schema_version': 1,
        'role': 'offline_target_evaluation_only',
        'mission_status': mission['status'],
        'reported_mission_success': mission['mission_success'],
        'map_frame': frame, 'map_epoch': mission.get('map_epoch'),
        'alignment': {'direction': 'map_to_world', 'x_m': alignment.x,
                      'y_m': alignment.y, 'yaw_rad': alignment.yaw,
                      'source': alignment_source, 'estimated_from_targets': False},
        'appearance_to_class': dict(appearance_to_class),
        'distance_metric': 'planar_xy_euclidean_m', 'max_distance_m': threshold,
        'prediction_count': len(predictions), 'ground_truth_count': len(truths),
        'TP': tp, 'FP': fp, 'FN': fn,
        'precision': tp/len(predictions) if predictions else None,
        'recall': tp/len(truths) if truths else None,
        'zero_target_case': not truths,
        'target_inventory_exact': fp == 0 and fn == 0,
        'position_error_m': {
            'mean': statistics.mean(errors) if errors else None,
            'median': statistics.median(errors) if errors else None,
            'rmse': math.hypot(*(e/math.sqrt(len(errors)) for e in errors)) if errors else None,
            'max': max(errors) if errors else None,
        },
        'matches': [{'prediction_uid': predictions[i]['uid'], 'truth_index': j,
                     'class_id': predictions[i]['class_id'], 'distance_m': distance}
                    for i, j, distance in pairs],
        'unmatched_prediction_uids': [p['uid'] for i, p in enumerate(predictions)
                                      if i not in matched_predictions],
        'unmatched_truth_indices': [j for j in range(len(truths)) if j not in matched_truths],
        'transformed_predictions': predictions,
        'limitations': [
            'Target matching does not assess map coverage, visual coverage, autonomy, or return home.',
            'Zero-truth recall and zero-prediction precision are undefined (null).',
            'Alignment provenance is supplied by the evaluator, not verified by this program.',
            'Error is planar; no surface-to-centre correction or target-height validation is applied.',
        ],
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('mission', type=Path)
    parser.add_argument('ground_truth', type=Path)
    parser.add_argument('--map-to-world-x', type=float, required=True)
    parser.add_argument('--map-to-world-y', type=float, required=True)
    parser.add_argument('--map-to-world-yaw', type=float, required=True)
    parser.add_argument('--alignment-source', required=True,
                        help='Independent calibration/anchor and map-session provenance; never target fitting')
    parser.add_argument('--class-map', action='append', default=[], metavar='APPEARANCE=CLASS',
                        help='Repeat for every ground-truth appearance; unknown predictions stay FP')
    parser.add_argument('--max-distance-m', type=float, required=True)
    parser.add_argument('--output', type=Path, help='New JSON file; otherwise stdout')
    args = parser.parse_args(argv)
    try:
        mapping = {}
        for entry in args.class_map:
            appearance, separator, class_id = entry.partition('=')
            if not separator or not appearance or not class_id or appearance in mapping:
                raise ValueError('Use unique --class-map APPEARANCE=CLASS entries')
            mapping[appearance] = class_id
        result = evaluate(load_json(args.mission), load_json(args.ground_truth),
                          alignment=SE2(args.map_to_world_x, args.map_to_world_y, args.map_to_world_yaw),
                          alignment_source=args.alignment_source, appearance_to_class=mapping,
                          max_distance_m=args.max_distance_m)
        encoded = json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False)+'\n'
        if args.output:
            with args.output.open('x', encoding='utf-8') as stream:
                stream.write(encoded)
        else:
            sys.stdout.write(encoded)
    except (ValueError, OSError) as exc:
        parser.exit(2, f'error: {exc}\n')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
