"""Accumulate discrete depth-ray evidence on explicit height planes.

All cells of the supplied finite rectangle are kept, including unknown and
occupied cells. Historical sampled poses are not corrected for later SLAM
optimization. The result is a geometric diagnostic, never search acceptance.
"""
from collections import Counter
import math

from .geometry import classify_points, number, vector

STATES = ('behind_camera', 'outside_image', 'invalid_depth',
          'depth_ambiguous', 'occluded', 'depth_clear')
QUERY_DEPTH_RELATIONS = ('behind_camera', 'below_min_depth',
                         'within_depth_bounds', 'above_max_depth')


def map_geometry(snapshot):
    width, height = snapshot['width'], snapshot['height']
    if (type(width) is not int or type(height) is not int or
            width <= 0 or height <= 0 or width*height > 100000):
        raise ValueError('Map must contain 1..100000 cells')
    resolution = number(snapshot['resolution'], 'map resolution')
    if resolution <= 0:
        raise ValueError('Positive map resolution required')
    frame = snapshot['frame_id']
    if not isinstance(frame, str) or not frame or frame != frame.strip():
        raise ValueError('Explicit map frame required')
    origin = snapshot['origin']
    xyz = vector(origin['xyz'], 3, 'map origin')
    q = vector(origin['quaternion_xyzw'], 4, 'map orientation')
    if abs(sum(v*v for v in q)-1.) > 1e-6 or abs(q[0])+abs(q[1]) > 1e-8:
        raise ValueError('A unit planar map orientation is required')
    cells = snapshot['data']
    if len(cells) != width*height or any(type(v) is not int or not -1 <= v <= 100 for v in cells):
        raise ValueError('Complete raw occupancy rectangle required')
    yaw = 2*math.atan2(q[2], q[3])
    c, s = math.cos(yaw), math.sin(yaw)
    xy = [(xyz[0]+c*(x+.5)*resolution-s*(y+.5)*resolution,
           xyz[1]+s*(x+.5)*resolution+c*(y+.5)*resolution)
          for y in range(height) for x in range(width)]
    return {'width': width, 'height': height, 'resolution': resolution,
            'frame_id': frame, 'origin': {'xyz': xyz, 'quaternion_xyzw': q},
            'stamp_ns': snapshot.get('stamp_ns'), 'data': list(cells)}, xy


def map_class(value):
    return 'unknown' if value == -1 else 'free' if value == 0 else 'occupied' if value == 100 else 'intermediate'


def build_layers(snapshot, frames, *, floor_z_m, height_reference_provenance,
                 heights_m, min_depth_m, max_depth_m, uncertainty_bound_m,
                 registered_to_camera, depth_semantics, optical_axes,
                 assumed_size_m=None):
    """Return all per-point/per-frame states, plus readable aggregate labels.

    `nominal_clear` means at least one frame's four discrete measured rays were
    behind this sample point by more than the explicit assumed margin. Other
    frames' occluded/invalid/ambiguous evidence is retained, never overwritten.
    """
    grid, xy = map_geometry(snapshot)
    floor = number(floor_z_m, 'explicit floor z')
    if not isinstance(height_reference_provenance, str) or not height_reference_provenance.strip():
        raise ValueError('Explicit height-reference provenance required')
    if not isinstance(heights_m, (tuple, list)) or not 1 <= len(heights_m) <= 8:
        raise ValueError('One to eight explicit height layers required')
    heights = [number(h, 'height above reference') for h in heights_m]
    if len(set(heights)) != len(heights) or any(h < 0 for h in heights):
        raise ValueError('Distinct nonnegative heights required')
    if not isinstance(frames, (tuple, list)) or not 1 <= len(frames) <= 256:
        raise ValueError('One to 256 qualified source frames required')
    if len(frames)*len(xy)*len(heights) > 4000000:
        raise ValueError('Explicit four-million point/frame work limit exceeded')
    ids = [f['retention_id'] for f in frames]
    if len(set(ids)) != len(ids):
        raise ValueError('Duplicate retained frame identity')
    frame_metadata = []
    for f in frames:
        req, producer = f['request'], f['producer']
        frame_metadata.append({'retention_id': f['retention_id'],
            'request_sha256': f['request_sha256'],
            'camera_pose': list(req['tf']['camera_pose']),
            'rgb_stamp_ns': producer['rgb']['stamp_ns'],
            'depth_stamp_ns': producer['depth']['stamp_ns'],
            'detector_session_id': producer['detector_session_id'],
            'frame_id': producer['depth']['frame_id']})
    layers = []
    for height in heights:
        z = floor+height
        if not math.isfinite(z):
            raise ValueError('Height plane is not finite')
        points = [(x, y, z) for x, y in xy]
        supports = {state: [[] for _ in xy] for state in STATES}
        query_supports = {relation: [[] for _ in xy] for relation in QUERY_DEPTH_RELATIONS}
        best_margin, best_frame, best_size = [None]*len(xy), [None]*len(xy), [None]*len(xy)
        for frame_index, f in enumerate(frames):
            req, producer = f['request'], f['producer']
            dm = producer['depth']
            result = classify_points(f['camera_info'], {
                'parent_frame': grid['frame_id'], 'child_frame': dm['frame_id'],
                'stamp_ns': req['tf']['stamp_ns'], 'pose': req['tf']['camera_pose']},
                f['depth'], points, points_frame=grid['frame_id'],
                depth_frame=dm['frame_id'], depth_stamp_ns=dm['stamp_ns'],
                depth_encoding=dm['encoding'], depth_semantics=depth_semantics,
                registered_to_camera=registered_to_camera, optical_axes=optical_axes,
                min_depth_m=min_depth_m, max_depth_m=max_depth_m,
                uncertainty_bound_m=uncertainty_bound_m, camera_info_max_age_ns=0,
                assumed_size_m=assumed_size_m)
            for i, row in enumerate(result['rows']):
                supports[row['status']][i].append(frame_index)
                query_supports[row['query_depth_relation']][i].append(frame_index)
                lower = row['delta_min_m']
                if lower is not None and (best_margin[i] is None or lower > best_margin[i]):
                    best_margin[i], best_frame[i], best_size[i] = lower, frame_index, row['projected_size_px']
        states = []
        for i in range(len(xy)):
            states.append('nominal_clear' if supports['depth_clear'][i] else
                          'mixed_or_surface' if supports['depth_ambiguous'][i] else
                          'occluded' if supports['occluded'][i] else
                          'no_valid_depth' if supports['invalid_depth'][i] else 'no_view')
        counts_by_class = {}
        for value, state in zip(grid['data'], states):
            group = counts_by_class.setdefault(map_class(value), Counter())
            group[state] += 1
        layers.append({'height_above_floor_m': height, 'map_z_m': z,
            'states': states, 'counts': dict(Counter(states)),
            'counts_by_map_class': {key: dict(val) for key, val in counts_by_class.items()},
            'support_frame_indices': supports,
            'query_depth_support_frame_indices': query_supports,
            'query_depth_relation_counts': {relation: sum(map(len, indices))
                                            for relation, indices in query_supports.items()},
            'max_of_delta_min_m': best_margin, 'max_margin_frame_index': best_frame,
            'hypothetical_size_px_at_max_margin': best_size,
            'classification_count': sum(len(indices) for values in supports.values() for indices in values)})
    return {'schema_version': 1, 'diagnostic_only': True,
        'recognition_coverage_verified': False, 'mission_success_claimed': False,
        'floor_reference_verified': False, 'historical_reprojection_verified': False,
        'depth_uncertainty_verified': False, 'floor_z_m': floor,
        'height_reference_provenance': height_reference_provenance,
        'uncertainty_bound_m': uncertainty_bound_m,
        'sampled_depth_bounds_m': [min_depth_m, max_depth_m], 'depth_semantics': depth_semantics,
        'depth_bounds_apply_to': 'measured_surface_samples_only',
        'registered_to_camera_declared': registered_to_camera, 'optical_axes': optical_axes,
        'assumed_size_m': assumed_size_m, 'map': grid, 'frames': frame_metadata,
        'distinct_source_stamps': len({(f['detector_session_id'], f['frame_id'], f['depth_stamp_ns']) for f in frame_metadata}),
        'layers': layers,
        'classification_count': sum(layer['classification_count'] for layer in layers),
        'aggregation': 'Any nominal clear opportunity wins the display label; every source state remains in support_frame_indices.',
        'scope': 'Finite map rectangle and supplied retained frames only; no target positions/counts, no missing-frame inference, no map mutation.',
        'limitations': ['Discrete sample points/rays do not certify full object visibility or recognition.',
            'Depth registration and axial-Z semantics are explicit input declarations, not measured calibration.',
            'Depth bounds filter measured surfaces; query-point sensor/recognition range is not certified.',
            'Floor, depth and pose uncertainty have no measured bound in this diagnostic.',
            'Historical map poses have not been reprojected through the final SLAM graph.',
            'Map unknown and occupied cells remain in the denominator and keep their original map classification.']}
