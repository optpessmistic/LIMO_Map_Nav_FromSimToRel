"""Offline height-plane depth diagnostics from exact retained RGB-D evidence."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import time

from .layers import build_layers
from .retained import load_run


def uncertainty(value):
    if value == 'unknown':
        return None
    try:
        return float(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError('Use unknown or an explicit assumed nonnegative bound in metres') from exc


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-root', type=Path, required=True)
    p.add_argument('--mission', required=True, help='Exact mission filename within run-root/mission')
    p.add_argument('--output', type=Path, required=True, help='New directory outside the input run')
    p.add_argument('--floor-z', type=float, required=True, help='Explicit nominal plane z in the map frame')
    p.add_argument('--height-reference', required=True, help='Provenance/assumption, never a claim of measured ground')
    p.add_argument('--heights', type=float, nargs='+', required=True, help='Sample heights above that plane in metres')
    p.add_argument('--min-depth', type=float, required=True)
    p.add_argument('--max-depth', type=float, required=True)
    p.add_argument('--uncertainty-bound', type=uncertainty, required=True,
                   help='unknown gives nominal zero-margin comparisons; a number is an unverified assumed bound')
    p.add_argument('--depth-registered', action='store_true', required=True,
                   help='Explicitly declare that depth is registered to CameraInfo/RGB')
    p.add_argument('--depth-semantics', choices=['optical_z'], required=True)
    p.add_argument('--optical-axes', choices=['x_right_y_down_z_forward'], required=True)
    p.add_argument('--assumed-size', type=float, nargs=2, metavar=('WIDTH', 'HEIGHT'),
                   help='Optional hypothetical image-plane-parallel rectangle in metres')
    p.add_argument('--plot', action='store_true', help='Also render a PNG; requires matplotlib and numpy, exactly 3 heights')
    return p


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return {'bytes': path.stat().st_size, 'sha256': h.hexdigest()}


def json_file(path, value):
    with path.open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')


def main(argv=None):
    args = parser().parse_args(argv)
    output, run_root = args.output.absolute(), args.run_root.absolute()
    if output.exists() or output.is_symlink() or output.resolve().is_relative_to(run_root.resolve()):
        raise ValueError('Output must be a new directory outside the original run')
    if args.plot and len(args.heights) != 3:
        raise ValueError('The four-panel plot needs exactly three height layers')
    started = time.monotonic()
    loaded = load_run(run_root, args.mission)
    size = None if args.assumed_size is None else {
        'width_m': args.assumed_size[0], 'height_m': args.assumed_size[1],
        'orientation': 'parallel_image_plane'}
    data = build_layers(loaded['map_snapshot'], loaded['frames'],
        floor_z_m=args.floor_z, height_reference_provenance=args.height_reference,
        heights_m=args.heights, min_depth_m=args.min_depth, max_depth_m=args.max_depth,
        uncertainty_bound_m=args.uncertainty_bound, assumed_size_m=size,
        registered_to_camera=args.depth_registered, depth_semantics=args.depth_semantics,
        optical_axes=args.optical_axes)
    output.mkdir(parents=True, exist_ok=False)
    try:
        source_root = Path(__file__).resolve().parents[2]
        source_files = list(Path(__file__).parent.glob('*.py'))
        source_files.append(source_root/'tools/audit_frame_retention.py')
        source_files.extend((source_root/'src/limo_evidence_protocol/limo_evidence_protocol').glob('*.py'))
        sources = {p.relative_to(source_root).as_posix(): digest(p) for p in sorted(source_files)}
        inputs = {'schema_version': 1, 'run_root': str(run_root),
            'mission_filename': args.mission, 'map_epoch': loaded['mission']['map_epoch'],
            'files': loaded['inputs_hashes'], 'source_files': sources,
            'included_frames': [{k: f[k] for k in ('retention_id', 'request_sha256', 'source_files')}
                                for f in loaded['frames']],
            'excluded': loaded['excluded'],
            'original_audit': loaded['audit_original'], 'current_audit': loaded['audit_current']}
        json_file(output/'inputs.json', inputs)
        raw = (json.dumps(data, separators=(',', ':'), ensure_ascii=False, allow_nan=False)+'\n').encode('utf-8')
        with (output/'layers.json.gz').open('xb') as stream:
            stream.write(gzip.compress(raw, mtime=0))
        if args.plot:
            from .plot import plot_layers
            plot_layers(data, output/'visibility-diagnostic.png')
        flags = ('status', 'reason', 'map_complete', 'visual_coverage_complete',
                 'returned_pose_verified', 'mission_success')
        report = {'schema_version': 1, 'diagnostic_only': True,
            'recognition_coverage_verified': False, 'mission_success_claimed': False,
            'floor_reference_verified': False, 'historical_reprojection_verified': False,
            'depth_uncertainty_verified': False,
            'map_epoch': loaded['mission']['map_epoch'],
            'original_mission_flags': {k: loaded['mission'].get(k) for k in flags},
            'floor_z_m': data['floor_z_m'], 'height_reference_provenance': data['height_reference_provenance'],
            'uncertainty_bound_m': data['uncertainty_bound_m'],
            'sampled_depth_bounds_m': data['sampled_depth_bounds_m'],
            'depth_bounds_apply_to': data['depth_bounds_apply_to'],
            'assumed_size_m': data['assumed_size_m'],
            'frame_count': len(data['frames']), 'distinct_source_stamps': data['distinct_source_stamps'],
            'excluded_count': len(loaded['excluded']),
            'map_rectangle_cells': data['map']['width']*data['map']['height'],
            'classification_count': data['classification_count'],
            'layers': [{k: layer[k] for k in ('height_above_floor_m', 'map_z_m', 'counts', 'counts_by_map_class', 'classification_count', 'query_depth_relation_counts')}
                       for layer in data['layers']],
            'artifacts': {p.name: digest(p) for p in output.iterdir() if p.is_file()},
            'elapsed_monotonic_s': time.monotonic()-started,
            'scope': data['scope'], 'limitations': data['limitations']}
        json_file(output/'report.json', report)
    except Exception as exc:
        json_file(output/'failure.json', {'diagnostic_complete': False,
            'error_type': type(exc).__name__, 'error': str(exc),
            'scope': 'Partial output retained; no original experiment files modified'})
        raise
    print(json.dumps({'output': str(output), 'frame_count': report['frame_count'],
        'excluded_count': report['excluded_count'], 'classification_count': report['classification_count'],
        'recognition_coverage_verified': False, 'plot': args.plot}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
