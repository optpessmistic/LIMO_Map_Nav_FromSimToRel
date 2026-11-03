"""Single-frame depth diagnostics, stdlib only; no file/ROS/truth access.

CameraInfo uses the retained metadata shape: frame_id, stamp_ns, calibration
with width/height/k/d/r/p/roi/binning_x/binning_y/distortion_model. Pose input is
{parent_frame, child_frame, stamp_ns, pose:[x,y,z,qx,qy,qz,qw]} and explicitly
maps camera-optical coordinates INTO the points' frame. Points must each have
three coordinates in metres; no floor height or map.z=0 is inferred.

Only zero-distortion, unbinned/uncropped pinhole images with R=I and P=[K|0]
are accepted. Depth is an already decoded rectangular array. Raw byte length,
endianness, SHA/retention eligibility, and final-map historical reprojection
must be audited separately before real data can be used.

For a projected point use four ORIGINAL samples at floor(u/v)+(0/1), never
bilinear interpolation across an edge. Z is optical axial depth, not range.
delta_min/max = min/max(depth)-Z. Strict delta_min>epsilon is depth_clear;
delta_max<-epsilon is occluded; otherwise depth_ambiguous. Missing epsilon
uses zero only for nominal classification, never a verified-clear claim.
An invalid/out-of-image 2x2 stencil is invalid_depth. Projected point outside
the image is outside_image; Z<=0 is behind_camera.
Depth bounds filter measured surfaces, not query points. Each query's Z
relation to those bounds is recorded separately; no sensor range is certified.

An optional width/height is only a hypothetical rectangle parallel to the
image plane at the point's optical Z. It does not model object pose, partial
occlusion, recognizability, detector pixel thresholds or whole-object coverage.
"""
import math
from numbers import Integral, Real


def _finite_real(value):
    if isinstance(value, bool) or not isinstance(value, Real):
        return False
    try:
        return math.isfinite(value)
    except (OverflowError, ValueError):
        return False


def number(value, name):
    if not _finite_real(value):
        raise ValueError(name+' must be finite numeric')
    return float(value)


def integer(value, name, minimum=0):
    if isinstance(value, bool) or not isinstance(value, Integral) or value < minimum:
        raise ValueError(name+' must be an integer >= '+str(minimum))
    return int(value)


def _sequence_length(values, name):
    if isinstance(values, (str, bytes)):
        raise ValueError(name+' must be a sized array')
    try:
        return len(values)
    except TypeError as error:
        raise ValueError(name+' must be a sized array') from error


def vector(values, length, name):
    if _sequence_length(values, name) != length:
        raise ValueError(name+' requires exactly '+str(length)+' explicit coordinates')
    return [number(value, name) for value in values]


def frame_name(value):
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError('Explicit nonempty frame required')
    return value


def classify_points(camera_info, map_from_camera, depth, points_3d, *,
                    points_frame, depth_frame, depth_stamp_ns, depth_encoding,
                    depth_semantics, registered_to_camera, optical_axes,
                    min_depth_m, max_depth_m, uncertainty_bound_m,
                    camera_info_max_age_ns, assumed_size_m=None):
    """Return per-point statuses and raw margins; never recognition/coverage.

    Required conventions: depth_semantics='optical_z', registered_to_camera=True,
    optical_axes='x_right_y_down_z_forward'. 32FC1 values are metres; 16UC1 are
    integral millimetres. Optional assumed_size_m is
    {width_m, height_m, orientation:'parallel_image_plane'}.
    """
    if (depth_semantics != 'optical_z' or registered_to_camera is not True or
            optical_axes != 'x_right_y_down_z_forward'):
        raise ValueError('Explicit registered optical Z-forward axial depth required')
    if depth_encoding not in ('32FC1', '16UC1'):
        raise ValueError('Only decoded 32FC1 metres / 16UC1 millimetres are supported')
    parent = frame_name(points_frame)
    child = frame_name(camera_info['frame_id'])
    if (frame_name(map_from_camera['parent_frame']) != parent or
            frame_name(map_from_camera['child_frame']) != child or frame_name(depth_frame) != child):
        raise ValueError('Points, pose and depth/CameraInfo frames do not match')
    stamp = integer(depth_stamp_ns, 'depth stamp')
    if integer(map_from_camera['stamp_ns'], 'pose stamp') != stamp:
        raise ValueError('Single-frame pose must be at the exact depth stamp')
    info_age = abs(integer(camera_info['stamp_ns'], 'CameraInfo stamp')-stamp)
    if info_age > integer(camera_info_max_age_ns, 'CameraInfo age bound'):
        raise ValueError('CameraInfo outside explicit timestamp bound')
    near, far = number(min_depth_m, 'min depth'), number(max_depth_m, 'max depth')
    if not 0 < near < far:
        raise ValueError('Explicit positive ordered depth bounds required')
    epsilon = 0. if uncertainty_bound_m is None else number(uncertainty_bound_m, 'assumed uncertainty')
    if epsilon < 0:
        raise ValueError('Uncertainty bound must be nonnegative or None')
    c = camera_info['calibration']
    width, height = integer(c['width'], 'width', 2), integer(c['height'], 'height', 2)
    if width*height > 16000000 or _sequence_length(points_3d, 'sample points') > 100000:
        raise ValueError('Finite diagnostic work bound exceeded')
    if (_sequence_length(depth, 'depth') != height or
            any(_sequence_length(row, 'depth row') != width for row in depth)):
        raise ValueError('Decoded depth dimensions must equal CameraInfo')
    k, r, p = vector(c['k'], 9, 'K'), vector(c['r'], 9, 'R'), vector(c['p'], 12, 'P')
    if (k[0] <= 0 or k[4] <= 0 or [k[i] for i in (1, 3, 6, 7, 8)] != [0., 0., 0., 0., 1.] or
            r != [1., 0., 0., 0., 1., 0., 0., 0., 1.] or
            p != [k[0], 0., k[2], 0., 0., k[4], k[5], 0., 0., 0., 1., 0.]):
        raise ValueError('Require zero-skew pinhole K, R=I and P=[K|0]')
    if c['distortion_model'] not in ('', 'plumb_bob', 'rational_polynomial') or any(
            number(value, 'D') != 0 for value in c['d']):
        raise ValueError('Nonzero/unsupported distortion requires explicit rectification')
    roi = c['roi']
    if (integer(c['binning_x'], 'binning_x') not in (0, 1) or
            integer(c['binning_y'], 'binning_y') not in (0, 1) or
            integer(roi['x_offset'], 'roi x') != 0 or integer(roi['y_offset'], 'roi y') != 0 or
            integer(roi['width'], 'roi width') not in (0, width) or
            integer(roi['height'], 'roi height') not in (0, height) or roi['do_rectify'] is not False):
        raise ValueError('Cropped/binned/rectified ROI requires explicit intrinsic adjustment')
    pose = vector(map_from_camera['pose'], 7, 'camera pose')
    x, y, z, w = pose[3:]
    norm = math.sqrt(x*x+y*y+z*z+w*w)
    if not math.isfinite(norm) or abs(norm-1.) > 1e-6:
        raise ValueError('Finite unit camera quaternion required')
    x, y, z, w = (q/norm for q in (x, y, z, w))
    rotation = ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))
    size = None
    if assumed_size_m is not None:
        if assumed_size_m['orientation'] != 'parallel_image_plane':
            raise ValueError('Only an explicitly fronto-parallel hypothetical size is supported')
        size = [number(assumed_size_m[key], key) for key in ('width_m', 'height_m')]
        if min(size) <= 0:raise ValueError('Hypothetical size must be positive')
    rows = []
    for index, point in enumerate(points_3d):
        point = vector(point, 3, 'sample point')
        offset = [point[i]-pose[i] for i in range(3)]
        optical = [sum(rotation[j][i]*offset[j] for j in range(3)) for i in range(3)]
        if not all(math.isfinite(value) for value in optical):
            raise ValueError('Point transform is not finite')
        X, Y, Z = optical
        row = {'index':index, 'point_3d_m':point, 'point_optical_m':optical,
               'query_depth_relation': ('behind_camera' if Z <= 0 else
                   'below_min_depth' if Z < near else
                   'above_max_depth' if Z > far else 'within_depth_bounds'),
               'status':None, 'pixel_uv':None, 'depth_samples_m':None,
               'delta_min_m':None, 'delta_max_m':None, 'projected_size_px':None}
        rows.append(row)
        if Z <= 0:
            row['status'] = 'behind_camera'
            continue
        u, v = k[0]*(X/Z)+k[2], k[4]*(Y/Z)+k[5]
        if not all(math.isfinite(value) for value in (u, v)):
            row.update(status='outside_image', reason='nonfinite_projection')
            continue
        row['pixel_uv'] = [u, v]
        if size is not None:
            projected = [k[0]*size[0]/Z, k[4]*size[1]/Z]
            if all(math.isfinite(value) for value in projected):
                row['projected_size_px'] = {'width':projected[0], 'height':projected[1],
                    'assumption':'hypothetical rectangle parallel to image plane at optical Z'}
        if not (0 <= u < width and 0 <= v < height):
            row['status'] = 'outside_image'
            continue
        column, line = math.floor(u), math.floor(v)
        if column+1 >= width or line+1 >= height:
            row.update(status='invalid_depth', reason='2x2_stencil_outside_image')
            continue
        pixels = [(column+dx, line+dy) for dy in (0, 1) for dx in (0, 1)]
        samples, invalid = [], []
        for px, py in pixels:
            raw = depth[py][px]
            valid = _finite_real(raw)
            if depth_encoding == '16UC1':
                valid = valid and isinstance(raw, Integral) and 0 <= raw <= 65535
            metres = float(raw)*(0.001 if depth_encoding=='16UC1' else 1.) if valid else None
            valid = valid and near <= metres <= far
            samples.append(metres)
            if not valid:invalid.append({'pixel':[px, py], 'raw_repr':repr(raw)})
        row.update(depth_pixels=pixels, depth_samples_m=samples)
        if invalid:
            row.update(status='invalid_depth', reason='invalid_or_out_of_range_depth', invalid_samples=invalid)
            continue
        lower, upper = min(samples)-Z, max(samples)-Z
        row.update(delta_min_m=lower, delta_max_m=upper,
                   status='depth_clear' if lower>epsilon else 'occluded' if upper < -epsilon else 'depth_ambiguous')
    return {'schema_version':1, 'diagnostic_only':True, 'recognition_coverage_verified':False,
        'depth_visibility_verified':False, 'mission_success_claimed':False,
        'points_frame':parent, 'camera_optical_frame':child, 'depth_stamp_ns':stamp,
        'camera_info_age_ns':info_age, 'camera_pose_map_from_optical':pose,
        'height_reference':'explicit 3D point coordinates; no floor inferred',
        'depth_semantics':'optical_z', 'uncertainty_bound_m':uncertainty_bound_m,
        'sampled_depth_bounds_m':[near, far],
        'depth_bounds_apply_to':'measured_surface_samples_only',
        'comparison_mode':'nominal_zero_bound' if uncertainty_bound_m is None else 'assumed_bound',
        'uncertainty_bound_verified':False, 'rows':rows,
        'limitations':['A clear diagnostic point is neither a detected object nor whole-object/search coverage.',
            '2x2 samples are discrete rays; no unobserved surface/interval is filled or certified.',
            'Depth, pose uncertainty and image-registration error have not been calibrated here.',
            'Depth bounds filter measured surfaces; query-point sensor/recognition range is not certified.',
            'Input bytes, original strict audit eligibility and final-map reprojection require separate checks.']}
