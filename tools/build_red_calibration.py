#!/usr/bin/env python3
"""Generate 12 unrun RGB-D calibration fixtures, never an online search plan.

Standard library only; no ROS imports or process control. Model locations and
labels live in separate offline truth files. Refuses to overwrite an output dir.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[1]
REQUIRED_SYSTEMS = ('Physics', 'UserCommands', 'SceneBroadcaster', 'Sensors', 'Imu')
ROBOT_RADIUS = .25
GREY = '0.26 0.32 0.39 1'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)+'\n', encoding='utf-8')


def numbers(text, count):
    if not isinstance(text, str):
        raise ValueError('Missing literal numeric vector')
    result = tuple(float(x) for x in text.split())
    if len(result) != count or not all(math.isfinite(x) for x in result):
        raise ValueError('Expected finite numeric vector: '+str(text))
    return result


def one(parent, path):
    found = parent.findall(path)
    if len(found) != 1:
        raise ValueError('Expected exactly one literal XML element: '+path)
    return found[0]


def zero_pose(parent):
    poses = parent.findall('pose')
    if len(poses) > 1 or (poses and (poses[0].attrib or numbers(poses[0].text, 6) != (0.,)*6)):
        raise ValueError('Nonzero or frame-qualified local pose is unsupported')


def clip(camera, path):
    value = one(camera, path)
    near = numbers(one(value, 'near').text, 1)[0]
    far = numbers(one(value, 'far').text, 1)[0]
    if not 0 < near < far:
        raise ValueError('Invalid nominal camera clip geometry')
    return [near, far]


def load_reference(model_path, sensor_path):
    robot = ET.parse(model_path).getroot()
    origins = []
    for name, parent, child in [('base_footprint_joint', 'base_footprint', 'base_link'),
                               ('camera_joint', 'base_link', 'camera_link')]:
        joint = one(robot, "joint[@name='%s']" % name)
        if joint.get('type') != 'fixed':
            raise ValueError('Missing required fixed joint: '+name)
        if one(joint, 'parent').get('link') != parent or one(joint, 'child').get('link') != child:
            raise ValueError('Unexpected camera joint chain')
        origin = one(joint, 'origin')
        if numbers(origin.get('rpy', '0 0 0'), 3) != (0., 0., 0.):
            raise ValueError('Only the current unrotated camera mounting is supported')
        origins.append(numbers(origin.get('xyz'), 3))
    optical = one(robot, "joint[@name='camera_optical_joint']")
    if optical.get('type') != 'fixed':
        raise ValueError('Optical frame joint missing')
    if (one(optical, 'parent').get('link') != 'camera_link' or
            one(optical, 'child').get('link') != 'camera_optical_frame'):
        raise ValueError('Unexpected optical joint chain')
    o = one(optical, 'origin')
    if numbers(o.get('xyz'), 3) != (0., 0., 0.) or any(abs(a-b) > 1e-8 for a, b in
            zip(numbers(o.get('rpy'), 3), (-math.pi/2, 0., -math.pi/2))):
        raise ValueError('Optical convention must be x right, y down, z forward')
    sensor_root = ET.parse(sensor_path).getroot()
    # Do not silently select a conditional or duplicated unexpanded xacro sensor.
    gazebo = one(sensor_root, "gazebo[@reference='camera_link']")
    sensor = one(gazebo, "sensor[@type='rgbd_camera']")
    camera = one(sensor, 'camera')
    zero_pose(gazebo)
    zero_pose(sensor)
    zero_pose(camera)
    if (sensor.findtext('gz_frame_id') != 'camera_link' or
            camera.findtext('optical_frame_id') != 'camera_optical_frame'):
        raise ValueError('Camera sensor/optical frame convention differs')
    if any(camera.find(name) is not None for name in ('lens', 'intrinsics', 'distortion', 'projection')):
        raise ValueError('Custom camera projection/distortion is unsupported')
    width = int(one(camera, 'image/width').text)
    height = int(one(camera, 'image/height').text)
    hfov = numbers(one(camera, 'horizontal_fov').text, 1)[0]
    rgb_clip, depth_clip = clip(camera, 'clip'), clip(camera, 'depth_camera/clip')
    if not (width > 0 and height > 0 and 0 < hfov < math.pi):
        raise ValueError('Invalid nominal camera geometry')
    focal = width/(2*math.tan(hfov/2))
    smoke = REPO/'worlds/smoke_maze.world'
    world = ET.parse(smoke).getroot().find('world')
    target = one(world, "model[@name='target_surrogate_0']")
    link = one(target, 'link')
    zero_pose(link)
    for kind in ('collision', 'visual'):
        zero_pose(one(link, kind))
    size = numbers(one(link, 'collision/geometry/box/size').text, 3)
    if size != numbers(one(link, 'visual/geometry/box/size').text, 3):
        raise ValueError('Reference visual and collision dimensions differ')
    pose = one(target, 'pose')
    if pose.attrib:
        raise ValueError('Frame-qualified reference target pose is unsupported')
    center_z = numbers(pose.text, 6)[2]
    if not (0 < size[0] < size[1] < size[2] and abs(center_z-size[2]/2) < 1e-9):
        raise ValueError('Expected the current thin, floor-height red-box surrogate')
    rgba = numbers(one(link, 'visual/material/ambient').text, 4)
    if any(not 0 <= value <= 1 for value in rgba) or rgba != numbers(one(link, 'visual/material/diffuse').text, 4):
        raise ValueError('Reference target material must have matching bounded ambient/diffuse RGBA')
    return {'camera_world_xyz_nominal': [sum(v[i] for v in origins) for i in range(3)],
            'optical_axes_world': {'x': [0, -1, 0], 'y': [0, 0, -1], 'z': [1, 0, 0]},
            'width': width, 'height': height, 'horizontal_fov_rad': hfov,
            'nominal_intrinsics': {'fx': focal, 'fy': focal, 'cx': width/2, 'cy': height/2},
            'rgb_clip_m': rgb_clip, 'depth_clip_m': depth_clip,
            'target_size_xyz_m': size, 'target_center_z_m': center_z,
            'target_rgba': target.findtext('link/visual/material/ambient'),
            'source_sha256': {'robot_xacro': sha(model_path), 'sensor_xacro': sha(sensor_path),
                              'smoke_world': sha(smoke), 'world_builder': sha(REPO/'tools/build_gazebo_world.py')},
            'source_paths': {'robot_xacro': str(Path(model_path).resolve()), 'sensor_xacro': str(Path(sensor_path).resolve()),
                             'smoke_world': str(smoke), 'world_builder': str(REPO/'tools/build_gazebo_world.py')},
            'parser_scope': 'Literal direct joints and camera sensor only; no xacro expansion, nonzero sensor pose or custom projection support.',
            'runtime_calibration_verified': False,
            'projection_limits': 'Nominal pinhole/square pixels only. Runtime CameraInfo, sampled TF, rendered pixels and physical settled pose must be checked; these are not detections.'}


def cases():
    return [
        {'id': '01_blank', 'kind': 'blank'},
        *[{'id': '%02d_center_%04dmm' % (i+2, round(r*1000)), 'kind': 'single', 'range_m': r,
           'bearing_deg': 0., 'incidence_deg': 0.} for i, r in enumerate((.4, .8, 1.6, 3.2))],
        {'id': '06_left_35deg', 'kind': 'single', 'range_m': .8, 'bearing_deg': 35., 'incidence_deg': 0.},
        {'id': '07_right_35deg', 'kind': 'single', 'range_m': .8, 'bearing_deg': -35., 'incidence_deg': 0.},
        {'id': '08_side_45deg', 'kind': 'single', 'range_m': .8, 'bearing_deg': 0., 'incidence_deg': 45.},
        {'id': '09_side_75deg', 'kind': 'single', 'range_m': .8, 'bearing_deg': 0., 'incidence_deg': 75.},
        {'id': '10_red_panel_negative', 'kind': 'red_distractor'},
        {'id': '11_close_pair_180mm', 'kind': 'pair'},
        {'id': '12_partial_occlusion', 'kind': 'partial', 'range_m': .8, 'bearing_deg': 0., 'incidence_deg': 0.},
    ]


def local_xy(point, box):
    c, s = math.cos(box['yaw_rad']), math.sin(box['yaw_rad'])
    dx, dy = point[0]-box['xyz_m'][0], point[1]-box['xyz_m'][1]
    return c*dx+s*dy, -s*dx+c*dy


def distance_xy_to_box(point, box):
    x, y = local_xy(point, box)
    return math.hypot(max(0., abs(x)-box['size_xyz_m'][0]/2), max(0., abs(y)-box['size_xyz_m'][1]/2))


def boxes_penetrate(first, second, tolerance=1e-9):
    """Strict interior overlap of two yaw-only boxes; flush contact is allowed."""
    if (abs(first['xyz_m'][2]-second['xyz_m'][2]) >=
            (first['size_xyz_m'][2]+second['size_xyz_m'][2])/2-tolerance):
        return False
    def axes(box):
        c, s = math.cos(box['yaw_rad']), math.sin(box['yaw_rad'])
        return ((c, s), (-s, c))
    a_axes, b_axes = axes(first), axes(second)
    delta = [second['xyz_m'][i]-first['xyz_m'][i] for i in (0, 1)]
    for axis in (*a_axes, *b_axes):
        def radius(box, local_axes):
            return sum(abs(sum(a*b for a, b in zip(axis, direction)))*size/2
                       for direction, size in zip(local_axes, box['size_xyz_m'][:2]))
        if abs(sum(a*b for a, b in zip(axis, delta))) >= radius(first, a_axes)+radius(second, b_axes)-tolerance:
            return False
    return True


def ray_hits_box(start, end, box):
    """Closed 3-D segment vs yaw-oriented box; endpoint itself is excluded."""
    a = (*local_xy(start, box), start[2]-box['xyz_m'][2])
    b = (*local_xy(end, box), end[2]-box['xyz_m'][2])
    enter, leave = 0., 1.-1e-8
    for p, q, size in zip(a, b, box['size_xyz_m']):
        d, h = q-p, size/2
        if abs(d) < 1e-12:
            if abs(p) > h:
                return False
            continue
        low, high = sorted(((-h-p)/d, (h-p)/d))
        enter, leave = max(enter, low), min(leave, high)
        if enter > leave:
            return False
    return True


def front_point(target, lateral=0., vertical=0.):
    n = target['front_normal_world_xy']
    p = target['front_face_center_world_xyz_m']
    return p[0]-n[1]*lateral, p[1]+n[0]*lateral, p[2]+vertical


def nominal_project(point, ref):
    cx, cy, cz = ref['camera_world_xyz_nominal']
    depth = point[0]-cx
    k = ref['nominal_intrinsics']
    if depth <= 0:
        raise ValueError('Target behind nominal optical plane')
    return k['cx']-k['fx']*(point[1]-cy)/depth, k['cy']+k['fy']*(cz-point[2])/depth, depth


def build_case(case, ref, box_helper):
    template = ET.parse(REPO/'worlds/smoke_maze.world').getroot().find('world')
    root = ET.Element('sdf', version='1.9')
    world = ET.SubElement(root, 'world', name='limo_red_cal_'+case['id'])
    for child in template:
        if child.tag in ('plugin', 'gravity', 'scene', 'light'):
            world.append(copy.deepcopy(child))
    objects, targets = [], []

    def add(name, xyz, size, yaw=0., role='enclosure', color=GREY):
        box_helper(world, name, xyz, size, color)
        model = world.findall('model')[-1]
        model.find('pose').text = ' '.join(format(v, '.17g') for v in (*xyz, 0., 0., yaw))
        record = {'name': name, 'role': role, 'xyz_m': list(xyz), 'size_xyz_m': list(size),
                  'yaw_rad': yaw, 'rgba': list(numbers(color, 4))}
        objects.append(record)
        return record

    add('floor', (2., 0., -.05), (6., 6., .1), role='floor', color='0.72 0.72 0.72 1')
    add('room_back', (-1.05, 0., .3), (.1, 6., .6))
    add('room_front', (5.05, 0., .3), (.1, 6., .6))
    add('room_left', (2., 3.05, .3), (6.2, .1, .6))
    add('room_right', (2., -3.05, .3), (6.2, .1, .6))
    camera = ref['camera_world_xyz_nominal']
    size = ref['target_size_xyz_m']

    def target(name, r, bearing, incidence, mount=True):
        b, angle = math.radians(bearing), math.radians(bearing+180.+incidence)
        n = (math.cos(angle), math.sin(angle))
        face = (camera[0]+r*math.cos(b), camera[1]+r*math.sin(b), ref['target_center_z_m'])
        center = (face[0]-n[0]*size[0]/2, face[1]-n[1]*size[0]/2, face[2])
        record = add(name, center, size, angle, role='target', color=ref['target_rgba'])
        record.update(front_face_center_world_xyz_m=list(face), front_normal_world_xy=list(n),
                      desired_horizontal_camera_range_m=r, desired_horizontal_bearing_deg=bearing,
                      desired_front_incidence_deg=abs(incidence), class_id='red_box_surrogate',
                      pose_reference='Front face center; detector returns a visible-surface representative, not necessarily this point.')
        if mount:
            mount_center = (face[0]-n[0]*(size[0]+.05), face[1]-n[1]*(size[0]+.05), .3)
            backing = add(name+'_backing', mount_center, (.1, .8, .6), angle, role='mount')
            record['backing_name'] = backing['name']
        targets.append(record)

    kind = case['kind']
    if kind in ('single', 'partial'):
        target('red_box_0', case['range_m'], case['bearing_deg'], case['incidence_deg'])
    if kind == 'pair':
        for i, y in enumerate((-.09, .09)):
            target('red_box_'+str(i), math.hypot(.8, y), math.degrees(math.atan2(y, .8)),
                   -math.degrees(math.atan2(y, .8)), mount=False)
        backing = add('shared_backing', (camera[0]+.8+size[0]+.05, 0., .3), (.1, .8, .6), math.pi, role='mount')
        for item in targets:
            item['backing_name'] = backing['name']
    if kind == 'red_distractor':
        add('large_red_panel', (camera[0]+.815, 0., .2), (.03, .45, .4), math.pi,
            role='distractor', color=ref['target_rgba'])
    if kind == 'partial':
        add('grey_occluder', (camera[0]+.60, .017, .14), (.02, .026, .28), role='occluder')
    truth = {'schema_version': 1, 'role': 'simulator_and_offline_evaluator_only', 'case': case,
             'status': 'designed_not_rendered_or_tested', 'world_name': world.get('name'),
             'robot_spawn': {'x': 0., 'y': 0., 'z': 0., 'yaw': 0.},
             'camera_reference': ref, 'robot_check_radius_m': ROBOT_RADIUS,
             'targets_ground_truth': targets, 'target_count_offline_only': len(targets), 'all_static_boxes_offline_only': objects,
             'label_scope': 'Only the standard thin red boxes are positive surrogate targets; the oversized red panel is a declared negative for this shape-defined fixture, which the current color-only detector may not distinguish.',
             'final_target_definition_verified': False,
             'online_inputs_allowed': ['rendered RGB', 'registered depth', 'runtime CameraInfo', 'sample-time TF'],
             'online_inputs_forbidden': ['this truth JSON', 'instance count/poses/names/normals', 'world-derived navigation waypoints'],
             'effective_detection_range_claimed': False, 'autonomous_maze_success_claimed': False}
    return root, truth


def validate_case(root, truth):
    world = one(root, 'world')
    systems = [p.get('name', '').split('::')[-1] for p in world.findall('plugin')]
    if sorted(systems) != sorted(REQUIRED_SYSTEMS):
        raise ValueError('Required Gazebo systems missing or duplicated')
    if one(world, "plugin[@name='gz::sim::systems::Sensors']/render_engine").text != 'ogre2':
        raise ValueError('Expected ogre2 sensor rendering')
    models = world.findall('model')
    if len({m.get('name') for m in models}) != len(models):
        raise ValueError('Duplicate model names')
    boxes = truth['all_static_boxes_offline_only']
    by_name = {b['name']: b for b in boxes}
    if len(boxes) != len(models) or len(by_name) != len(boxes) or set(by_name) != {m.get('name') for m in models}:
        raise ValueError('Geometry/truth count differs')
    targets = truth['targets_ground_truth']
    if (truth['target_count_offline_only'] != len(targets) or
            len({t['name'] for t in targets}) != len(targets) or
            {t['name'] for t in targets} != {b['name'] for b in boxes if b['role'] == 'target'}):
        raise ValueError('Target labels differ from fixture roles/count')
    expected_count = {'blank': 0, 'red_distractor': 0, 'single': 1, 'pair': 2, 'partial': 1}
    if truth['case']['kind'] not in expected_count or len(targets) != expected_count[truth['case']['kind']]:
        raise ValueError('Case kind/positive target count mismatch')
    distractors = [b for b in boxes if b['role'] == 'distractor']
    if len(distractors) != int(truth['case']['kind'] == 'red_distractor'):
        raise ValueError('Case kind/distractor control count mismatch')
    minimum_clearance = math.inf
    for model in models:
        b = by_name[model.get('name')]
        pose_element = one(model, 'pose')
        if pose_element.attrib:
            raise ValueError('Frame-qualified fixture pose is unsupported')
        pose = numbers(pose_element.text, 6)
        if pose != tuple(b['xyz_m'])+(0., 0., b['yaw_rad']):
            raise ValueError('SDF/truth pose disagreement')
        link = one(model, 'link')
        zero_pose(link)
        for shape in ('collision', 'visual'):
            element = one(link, shape)
            zero_pose(element)
            actual = numbers(one(element, 'geometry/box/size').text, 3)
            if actual != tuple(b['size_xyz_m']) or min(actual) <= 0:
                raise ValueError('SDF/truth dimension disagreement')
        if model.findtext('static') != 'true':
            raise ValueError('Fixture object must be static')
        for color in ('ambient', 'diffuse'):
            if numbers(one(link, 'visual/material/'+color).text, 4) != tuple(b['rgba']):
                raise ValueError('SDF/truth material disagreement')
        if b['role'] != 'floor':
            clearance = distance_xy_to_box((0., 0.), b)-truth['robot_check_radius_m']
            minimum_clearance = min(minimum_clearance, clearance)
            if clearance <= 0:
                raise ValueError('Fixture intrudes into nominal robot envelope')
    for i, first in enumerate(boxes):
        for second in boxes[i+1:]:
            if boxes_penetrate(first, second):
                raise ValueError('Fixture solids penetrate: '+first['name']+' / '+second['name'])
    floors = [b for b in boxes if b['role'] == 'floor']
    if len(floors) != 1 or abs(floors[0]['xyz_m'][2]+floors[0]['size_xyz_m'][2]/2) > 1e-9:
        raise ValueError('Exactly one floor at nominal ground z=0 is required')
    local = local_xy((0., 0.), floors[0])
    if any(abs(value)+truth['robot_check_radius_m'] >= extent/2
           for value, extent in zip(local, floors[0]['size_xyz_m'][:2])):
        raise ValueError('Floor does not support the nominal robot envelope')
    ref, results = truth['camera_reference'], []
    camera = ref['camera_world_xyz_nominal']
    for target in truth['targets_ground_truth']:
        face, n = target['front_face_center_world_xyz_m'], target['front_normal_world_xy']
        actual = by_name[target['name']]
        if any(target[key] != actual[key] for key in ('xyz_m', 'size_xyz_m', 'yaw_rad')):
            raise ValueError('Target label geometry differs from its fixture box')
        expected_n = (math.cos(actual['yaw_rad']), math.sin(actual['yaw_rad']))
        expected_face = [actual['xyz_m'][i]+(expected_n[i]*actual['size_xyz_m'][0]/2 if i < 2 else 0.) for i in range(3)]
        if any(abs(a-b) > 1e-9 for a, b in zip(n, expected_n)) or any(abs(a-b) > 1e-9 for a, b in zip(face, expected_face)):
            raise ValueError('Target front face/normal disagrees with actual box geometry')
        dx, dy = face[0]-camera[0], face[1]-camera[1]
        radius = math.hypot(dx, dy)
        bearing = math.degrees(math.atan2(dy, dx))
        incidence = math.degrees(math.acos(max(-1., min(1., -(dx*n[0]+dy*n[1])/radius))))
        if incidence >= 90. or not 0 <= target['desired_front_incidence_deg'] < 90.:
            raise ValueError('Declared positive front face does not face the camera')
        if abs(radius-target['desired_horizontal_camera_range_m']) > 1e-8 or abs(bearing-target['desired_horizontal_bearing_deg']) > 1e-8 or abs(incidence-target['desired_front_incidence_deg']) > 1e-6:
            raise ValueError('Requested target range/bearing/incidence not achieved')
        if tuple(target['size_xyz_m']) != tuple(ref['target_size_xyz_m']) or abs(target['xyz_m'][2]-target['size_xyz_m'][2]/2) > 1e-9:
            raise ValueError('Positive target differs from the current surrogate shape/height')
        backing = by_name[target['backing_name']]
        backing_n = (math.cos(backing['yaw_rad']), math.sin(backing['yaw_rad']))
        if backing['role'] != 'mount' or any(abs(a-b) > 1e-9 for a, b in zip(n, backing_n)):
            raise ValueError('Backing orientation/role differs from target normal')
        expected = [face[i]-n[i]*(target['size_xyz_m'][0]+backing['size_xyz_m'][0]/2) for i in (0, 1)]
        # A shared backing may be displaced along its own tangent, never normal.
        if abs(sum((expected[i]-backing['xyz_m'][i])*n[i] for i in (0, 1))) > 1e-8:
            raise ValueError('Target back and backing front are not flush')
        back_center = [face[i]-n[i]*target['size_xyz_m'][0] for i in (0, 1)]
        if (abs(local_xy(back_center, backing)[1])+target['size_xyz_m'][1]/2 > backing['size_xyz_m'][1]/2+1e-9 or
                abs(target['xyz_m'][2]-backing['xyz_m'][2])+target['size_xyz_m'][2]/2 > backing['size_xyz_m'][2]/2+1e-9):
            raise ValueError('Backing does not contain the target back face')
        width, height = target['size_xyz_m'][1:]
        corners = [nominal_project(front_point(target, y, z), ref) for y in (-width/2, width/2) for z in (-height/2, height/2)]
        near = max(ref['depth_clip_m'][0], ref.get('rgb_clip_m', ref['depth_clip_m'])[0])
        far = min(ref['depth_clip_m'][1], ref.get('rgb_clip_m', ref['depth_clip_m'])[1])
        if any(not (0 <= u < ref['width'] and 0 <= v < ref['height'] and near < d < far) for u, v, d in corners):
            raise ValueError('Positive target front face clipped in nominal pinhole view')
        probes = [front_point(target, width*((i+.5)/12-.5), height*((j+.5)/20-.5)) for i in range(12) for j in range(20)]
        obstacles = [b for b in boxes if b['name'] != target['name']]
        visible = sum(not any(ray_hits_box(camera, p, b) for b in obstacles) for p in probes)
        fraction = visible/len(probes)
        if truth['case']['kind'] == 'partial':
            if not .2 < fraction < .8:
                raise ValueError('Occlusion fixture does not geometrically partially obscure target')
        elif fraction != 1.:
            raise ValueError('Unexpected fixture geometry occludes a positive target front face')
        results.append({'target_name_offline_only': target['name'], 'range_m': radius, 'bearing_deg': bearing,
                        'incidence_deg': incidence, 'nominal_front_bbox_uv': [min(p[0] for p in corners), min(p[1] for p in corners), max(p[0] for p in corners), max(p[1] for p in corners)],
                        'front_surface_probe_count': len(probes), 'geometric_visible_probe_fraction': fraction})
    pair = truth['targets_ground_truth']
    if truth['case']['kind'] == 'pair':
        separation = math.dist(pair[0]['front_face_center_world_xyz_m'], pair[1]['front_face_center_world_xyz_m'])
        if abs(separation-.18) > 1e-8 or separation <= ref['target_size_xyz_m'][1]:
            raise ValueError('Pair separation/physical gap invalid')
        projected = sorted(result['nominal_front_bbox_uv'] for result in results)
        if projected[0][2] >= projected[1][0]:
            raise ValueError('Pair front projections are not separated')
    control_results = []
    for control in distractors:
        n = (math.cos(control['yaw_rad']), math.sin(control['yaw_rad']))
        face = [control['xyz_m'][i]+(n[i]*control['size_xyz_m'][0]/2 if i < 2 else 0.) for i in range(3)]
        surface = {'front_normal_world_xy': n, 'front_face_center_world_xyz_m': face}
        width, height = control['size_xyz_m'][1:]
        corners = [nominal_project(front_point(surface, y, z), ref) for y in (-width/2, width/2) for z in (-height/2, height/2)]
        near = max(ref['depth_clip_m'][0], ref.get('rgb_clip_m', ref['depth_clip_m'])[0])
        far = min(ref['depth_clip_m'][1], ref.get('rgb_clip_m', ref['depth_clip_m'])[1])
        if any(not (0 <= u < ref['width'] and 0 <= v < ref['height'] and near < d < far) for u, v, d in corners):
            raise ValueError('Negative control clipped in nominal view')
        probes = [front_point(surface, width*((i+.5)/12-.5), height*((j+.5)/20-.5)) for i in range(12) for j in range(20)]
        visible = sum(not any(ray_hits_box(camera, p, b) for b in boxes if b['name'] != control['name']) for p in probes)
        if visible != len(probes):
            raise ValueError('Negative control is geometrically occluded')
        control_results.append({'name_offline_only': control['name'], 'front_surface_probe_count': len(probes),
                                'geometric_visible_probe_fraction': visible/len(probes)})
    return {'case': truth['case']['id'], 'geometry_constraints_passed': True,
            'minimum_nominal_robot_xy_clearance_m': minimum_clearance, 'targets': results,
            'negative_controls': control_results,
            'fixture_solid_penetration_count': 0, 'flush_contacts_allowed': True,
            'rendering_validated': False, 'perception_validated': False,
            'limits': 'Analytical fixture geometry only; surface probes do not establish rendered visibility, sensor validity, detector recall, calibrated range, complete search or physical robot safety.'}


def generate(output, model_path, sensor_path):
    if output.exists():
        raise ValueError('Refusing to overwrite existing output directory: '+str(output))
    ref = load_reference(model_path, sensor_path)
    spec = importlib.util.spec_from_file_location('limo_fixture_world_builder', REPO/'tools/build_gazebo_world.py')
    builder = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(builder)
    built = []
    for case in cases():
        root, truth = build_case(case, ref, builder.box)
        checks = validate_case(root, truth)
        built.append((case, root, truth, checks))
    output.mkdir(parents=True, exist_ok=False)
    (output/'worlds').mkdir()
    (output/'offline_truth').mkdir()
    manifest = {'schema_version': 1, 'status': 'designed_not_run', 'case_count': len(built),
                'generator_sha256': sha(__file__), 'input_reference': ref, 'cases': [],
                'runtime_test_results': None, 'effective_detection_range': None,
                'final_target_definition_verified': False,
                'label_scope': 'Experiment-defined thin red surrogate boxes only; the red-panel negative control is not a finalized course target class.'}
    for case, root, truth, checks in built:
        sdf_path = output/'worlds'/(case['id']+'.sdf')
        truth_path = output/'offline_truth'/(case['id']+'.json')
        ET.indent(root)
        ET.ElementTree(root).write(sdf_path, encoding='utf-8', xml_declaration=True)
        truth['world_sha256'] = sha(sdf_path)
        write_json(truth_path, truth)
        manifest['cases'].append({'id': case['id'], 'world': str(sdf_path.relative_to(output)),
                                  'offline_truth': str(truth_path.relative_to(output)), 'world_sha256': sha(sdf_path),
                                  'truth_sha256': sha(truth_path), 'geometry_checks': checks})
    write_json(output/'manifest.json', manifest)
    write_json(output/'geometry-checks.json', {'case_count': len(built), 'all_geometry_constraints_passed': True,
                                            'results': [item[3] for item in built], 'actual_render_or_detector_tests_run': False})
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--model-xacro', type=Path, required=True)
    parser.add_argument('--sensor-xacro', type=Path, required=True)
    args = parser.parse_args()
    manifest = generate(args.output.resolve(), args.model_xacro, args.sensor_xacro)
    print(json.dumps({'output': str(args.output.resolve()), 'case_count': manifest['case_count'],
                      'geometry_constraints_passed': True, 'rendering_or_detection_validated': False}))


if __name__ == '__main__':
    main()
