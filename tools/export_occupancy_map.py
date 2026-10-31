#!/usr/bin/env python3
"""Export a captured immutable OccupancyGrid to Nav2 trinary PGM/YAML.

No ROS, subscriptions, map changes, smoothing, or live service calls. A valid
snapshot may come from a report whose unrelated service export failed. New
prefixes only; the provenance file is published only after output readback.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
import os
from pathlib import Path
import re
import sys

MAX_SOURCE_BYTES = 64 * 1024 * 1024
MAX_CELLS = 4_000_000
PLANAR_EPS = 1e-9
YAML_FREE = 0.196
YAML_OCCUPIED = 0.65


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode('utf-8')


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate_json_key:' + key)
        result[key] = value
    return result


def finite(value, name):
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError('finite_number_required:' + name)
    return float(value)


def integer(value, name, low=0, high=2**63 - 1):
    if type(value) is not int or not low <= value <= high:
        raise ValueError('integer_out_of_range:' + name)
    return value


def vector(value, length, name):
    if type(value) is not list or len(value) != length:
        raise ValueError('vector_shape:' + name)
    return [finite(item, name) for item in value]


def reject_constant(value):
    raise ValueError('nonfinite_json:' + value)


def read_snapshot(path):
    path = Path(path).expanduser().resolve(strict=True)
    with path.open('rb') as stream:
        raw = stream.read(MAX_SOURCE_BYTES + 1)
    if len(raw) > MAX_SOURCE_BYTES:
        raise ValueError('source_size_limit')
    document = json.loads(raw, object_pairs_hook=unique_object, parse_constant=reject_constant)
    if type(document) is not dict:
        raise ValueError('source_object_required')
    snapshot = document.get('snapshot', document)
    if type(snapshot) is not dict or snapshot.get('valid') is not True or snapshot.get('errors', []) != []:
        raise ValueError('valid_snapshot_required')
    grid = snapshot.get('map')
    if type(grid) is not dict:
        raise ValueError('map_object_required')
    width = integer(grid.get('width'), 'width', 1, MAX_CELLS)
    height = integer(grid.get('height'), 'height', 1, MAX_CELLS)
    if width * height > MAX_CELLS:
        raise ValueError('cell_count_limit')
    resolution = finite(grid.get('resolution'), 'resolution')
    if resolution <= 0 or not math.isfinite(resolution * max(width, height)):
        raise ValueError('invalid_metric_extent')
    frame = grid.get('frame_id')
    if type(frame) is not str or not frame or frame != frame.strip() or any(ord(c) < 32 for c in frame):
        raise ValueError('invalid_frame_id')
    integer(grid.get('stamp_ns'), 'stamp_ns')
    if 'map_load_time_ns' in grid:
        integer(grid['map_load_time_ns'], 'map_load_time_ns')
    origin = grid.get('origin')
    if type(origin) is not dict or origin.get('valid_geometry') is not True:
        raise ValueError('valid_origin_geometry_required')
    xyz = vector(origin.get('xyz'), 3, 'origin.xyz')
    qx, qy, qz, qw = vector(origin.get('quaternion_xyzw'), 4, 'origin.quaternion_xyzw')
    if abs(qx*qx + qy*qy + qz*qz + qw*qw - 1) > 1e-6:
        raise ValueError('origin_quaternion_not_unit')
    if abs(xyz[2]) > PLANAR_EPS or abs(qx) > PLANAR_EPS or abs(qy) > PLANAR_EPS:
        raise ValueError('nonplanar_origin_not_representable_in_yaml')
    yaw = math.atan2(2*qz*qw, qw*qw - qz*qz)
    for x in (0., width * resolution):
        for y in (0., height * resolution):
            if not all(math.isfinite(value) for value in (
                    xyz[0] + math.cos(yaw)*x - math.sin(yaw)*y,
                    xyz[1] + math.sin(yaw)*x + math.cos(yaw)*y)):
                raise ValueError('nonfinite_world_extent')
    data = grid.get('data')
    if type(data) is not list or len(data) != width * height:
        raise ValueError('grid_data_length')
    if any(type(value) is not int or not -1 <= value <= 100 for value in data):
        raise ValueError('illegal_occupancy_cell')
    return path, raw, document, snapshot, grid, [xyz[0], xyz[1], yaw]


def classify(data, free_thresh, occupied_thresh):
    free = finite(free_thresh, 'free_threshold')
    occupied = finite(occupied_thresh, 'occupied_threshold')
    if not 0 <= free < occupied <= 1:
        raise ValueError('require_0_le_free_lt_occupied_le_1')
    # Nav2 map_io uses rint: nearest integer, ties to even under its default
    # rounding mode. Free is evaluated first, including coincident rounded cuts.
    low, high = round(100 * free), round(100 * occupied)
    cells = [-1 if value == -1 else 0 if value <= low else 100 if value >= high else -1 for value in data]
    return cells, {'free_threshold': free, 'occupied_threshold': occupied,
                   'free_max_cell_inclusive': low, 'occupied_min_cell_inclusive': high,
                   'rounding': 'nearest_integer_ties_to_even; free_test_first',
                   'unknown_and_intermediate_to': -1}


def write_new(path, raw):
    with path.open('xb') as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())


def read_yaml(path):
    """Read our restricted YAML emitter: plain keys with JSON-subset values.

    This is deliberately not a general-purpose YAML parser. These scalars and
    flow arrays are valid YAML accepted by Nav2's yaml-cpp loader.
    """
    fields = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        key, separator, value = line.partition(':')
        if not separator or key in fields:
            raise ValueError('invalid_or_duplicate_yaml_field')
        fields[key] = json.loads(value)
    return fields


def verify_outputs(pgm, yaml_path, metadata, expected_cells, width, height):
    """Reopen actual files and reconstruct grid cells using Nav2 trinary rules."""
    actual = read_yaml(yaml_path)
    if (json.dumps(actual, sort_keys=True, allow_nan=False) != json.dumps(metadata, sort_keys=True, allow_nan=False)
            or (yaml_path.parent / actual['image']).resolve() != pgm.resolve()):
        raise ValueError('yaml_readback_mismatch')
    raw = pgm.read_bytes()
    match = re.match(rb'P5\n([1-9][0-9]*) ([1-9][0-9]*)\n255\n', raw)
    if not match or (int(match[1]), int(match[2])) != (width, height):
        raise ValueError('pgm_header_mismatch')
    pixels = raw[match.end():]
    if len(pixels) != width * height or any(p not in (0, 205, 254) for p in pixels):
        raise ValueError('pgm_raster_mismatch')
    reconstructed = []
    for row in range(height - 1, -1, -1):
        for pixel in pixels[row*width:(row+1)*width]:
            probability = (255 - pixel) / 255
            reconstructed.append(100 if probability >= actual['occupied_thresh'] else
                                 0 if probability <= actual['free_thresh'] else -1)
    if reconstructed != expected_cells:
        raise ValueError('trinary_roundtrip_mismatch')
    return {'passed': True, 'method': 'reopened_P5_and_YAML; reconstructed_all_trinary_cells',
            'verified_cells': len(reconstructed), 'class_counts': dict(Counter(map(str, reconstructed))),
            'files': {path.name: {'bytes': path.stat().st_size, 'sha256': sha(path.read_bytes())}
                      for path in (pgm, yaml_path)}}


def export_snapshot(snapshot_path, output_prefix, free_thresh=0.25, occupied_thresh=0.65):
    source, raw, document, snapshot, grid, origin = read_snapshot(snapshot_path)
    cells, classification = classify(grid['data'], free_thresh, occupied_thresh)
    prefix = Path(output_prefix).expanduser().absolute()
    if not prefix.name or any(ord(c) < 32 for c in prefix.name) or not prefix.parent.is_dir():
        raise ValueError('output_prefix_requires_existing_directory_and_valid_name')
    if any(path.name == prefix.name or path.name.startswith(prefix.name + '.') for path in prefix.parent.iterdir()):
        raise FileExistsError('output_prefix_already_used:' + str(prefix))
    paths = {suffix: prefix.parent / (prefix.name + suffix)
             for suffix in ('.reservation.json', '.pgm', '.yaml', '.provenance.json', '.failure.json')}
    identity = {'source': str(source), 'source_bytes': len(raw), 'source_sha256': sha(raw)}
    # Exclusive reservation arbitrates concurrent exporters using the same
    # prefix. Every following file is also exclusive; partial failures remain.
    write_new(paths['.reservation.json'], json_bytes({'schema_version': 1, **identity, 'status': 'reserved'}))
    stage = 'write_pgm'
    try:
        width, height = grid['width'], grid['height']
        colors = {-1: 205, 0: 254, 100: 0}
        raster = bytes(colors[cells[row*width+column]]
                       for row in range(height-1, -1, -1) for column in range(width))
        write_new(paths['.pgm'], f'P5\n{width} {height}\n255\n'.encode('ascii') + raster)
        metadata = {'image': paths['.pgm'].name, 'mode': 'trinary', 'resolution': grid['resolution'],
                    'origin': origin, 'negate': 0, 'occupied_thresh': YAML_OCCUPIED, 'free_thresh': YAML_FREE}
        stage = 'write_yaml'
        write_new(paths['.yaml'], ''.join(f'{key}: {json.dumps(value, allow_nan=False)}\n'
                                        for key, value in metadata.items()).encode('utf-8'))
        stage = 'verify_readback'
        verification = verify_outputs(paths['.pgm'], paths['.yaml'], metadata, cells, width, height)
        report = {'schema_version': 1, 'passed': True, 'method': 'captured_occupancy_grid_offline_export',
            **identity, 'frame_id': grid['frame_id'], 'map_stamp_ns': grid['stamp_ns'],
            'map_metadata': {key: value for key, value in grid.items() if key != 'data'},
            'snapshot_capture': {key: snapshot[key] for key in ('valid', 'captured_ros_ns', 'captured_wall_s', 'epoch') if key in snapshot},
            'source_report_export_status': {key: document[key] for key in ('passed', 'errors', 'exports') if key in document},
            'source_grid_sha256': sha(json_bytes(grid)), 'input_classification': classification,
            'source_cell_counts': dict(Counter(map(str, grid['data']))), 'yaml': metadata,
            'conventions': {'grid_index': 'width*y+x; y=0 bottom row',
                'pgm_index': 'width*(height-1-y)+x; top row first',
                'origin': 'metric lower-left corner of cell (0,0); yaw in radians, no half-cell shift',
                'planar_tolerance': PLANAR_EPS, 'yaml_thresholds': 'fixed Nav2 trinary reload thresholds; distinct from input classification'},
            'verification': verification,
            'limitations': ['No live map service was called or repaired; source report is unchanged.',
                'Trinary export preserves unknowns and classifies intermediate probabilities as unknown; raw probabilities remain in source JSON.',
                'YAML has no frame field; use the recorded frame_id when configuring map_server.',
                'File readback does not prove complete mapping, visual coverage, navigation safety, or mission success.']}
        stage = 'write_provenance'
        encoded = json_bytes(report)
        write_new(paths['.provenance.json'], encoded)
        if paths['.provenance.json'].read_bytes() != encoded:
            raise ValueError('provenance_readback_mismatch')
        return report
    except Exception as exc:
        failure = {'schema_version': 1, 'passed': False, **identity, 'stage': stage,
                   'error': type(exc).__name__ + ': ' + str(exc),
                   'scope': 'Partial files retained; choose a new prefix. No successful export may be inferred.'}
        try:
            write_new(paths['.failure.json'], json_bytes(failure))
        except OSError:
            pass  # Preserve the original exception if the filesystem cannot record failure.
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--output-prefix', type=Path, required=True)
    parser.add_argument('--free-thresh', type=float, default=0.25, help='Input free classification threshold, inclusive after rounding to percent')
    parser.add_argument('--occupied-thresh', type=float, default=0.65, help='Input occupied classification threshold, inclusive after rounding to percent')
    args = parser.parse_args()
    try:
        result = export_snapshot(args.snapshot, args.output_prefix, args.free_thresh, args.occupied_thresh)
    except (OSError, ValueError, TypeError, KeyError) as exc:
        print(json.dumps({'passed': False, 'error': type(exc).__name__ + ': ' + str(exc)}), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
