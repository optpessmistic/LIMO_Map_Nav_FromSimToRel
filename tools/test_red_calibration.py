#!/usr/bin/env python3
"""Pure geometry and deliberately broken fixture checks; no ROS/runtime imports."""
import importlib.util
import copy
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import xml.etree.ElementTree as ET

import build_red_calibration as scenes


spec = importlib.util.spec_from_file_location('fixture_builder', scenes.REPO/'tools/build_gazebo_world.py')
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


def reference():
    # A synthetic camera fixture; real-model parsing is checked by generation.
    focal = 640/(2*math.tan(1.3962634/2))
    return {'camera_world_xyz_nominal': [.084, 0., .18], 'width': 640, 'height': 480,
            'nominal_intrinsics': {'fx': focal, 'fy': focal, 'cx': 320., 'cy': 240.},
            'depth_clip_m': [.1, 8.], 'target_size_xyz_m': (.03, .06, .20),
            'target_center_z_m': .10, 'target_rgba': '0.9 0.05 0.05 1'}


def fixture(case_id):
    case = next(c for c in scenes.cases() if c['id'] == case_id)
    return scenes.build_case(case, reference(), builder.box)


def set_box(root, truth, name, xyz=None, size=None):
    """Change both SDF and truth so negative tests exercise physical checks."""
    box = next(b for b in truth['all_static_boxes_offline_only'] if b['name'] == name)
    model = root.find("world/model[@name='%s']" % name)
    if xyz is not None:
        box['xyz_m'] = list(xyz)
        model.find('pose').text = ' '.join(map(str, (*xyz, 0., 0., box['yaw_rad'])))
    if size is not None:
        box['size_xyz_m'] = list(size)
        for shape in ('collision', 'visual'):
            model.find('link/'+shape+'/geometry/box/size').text = ' '.join(map(str, size))


class GeometryChecks(unittest.TestCase):
    def test_ray_rotation_height_and_endpoint(self):
        # Long local X side becomes world Y after rotation: y=.7 hits; z=2 misses.
        box = {'xyz_m': [1., 0., 0.], 'size_xyz_m': [2., .2, 1.], 'yaw_rad': math.pi/2}
        self.assertTrue(scenes.ray_hits_box((0., .7, 0.), (2., .7, 0.), box))
        self.assertFalse(scenes.ray_hits_box((0., .7, 2.), (2., .7, 2.), box))
        # A fixture beginning at the target endpoint must not count as foreground.
        endpoint_box = {'xyz_m': [1.5, 0., 0.], 'size_xyz_m': [1., 1., 1.], 'yaw_rad': 0.}
        self.assertFalse(scenes.ray_hits_box((0., 0., 0.), (1., 0., 0.), endpoint_box))
        self.assertTrue(scenes.ray_hits_box((0., 0., 0.), (1.1, 0., 0.), endpoint_box))

    def test_lateral_bearing_sign_matches_image(self):
        left, left_truth = fixture('06_left_35deg')
        right, right_truth = fixture('07_right_35deg')
        l = scenes.validate_case(left, left_truth)['targets'][0]
        r = scenes.validate_case(right, right_truth)['targets'][0]
        self.assertLess(l['nominal_front_bbox_uv'][2], 320)
        self.assertGreater(r['nominal_front_bbox_uv'][0], 320)
        self.assertAlmostEqual(l['nominal_front_bbox_uv'][0]+r['nominal_front_bbox_uv'][2], 640)

    def test_blank_and_red_panel_are_declared_negative_controls(self):
        for case_id in ('01_blank', '10_red_panel_negative'):
            root, truth = fixture(case_id)
            self.assertEqual(truth['target_count_offline_only'], 0)
            self.assertEqual(scenes.validate_case(root, truth)['targets'], [])
        self.assertEqual(sum(b['role'] == 'distractor' for b in truth['all_static_boxes_offline_only']), 1)
        self.assertEqual(scenes.validate_case(root, truth)['negative_controls'][0]['geometric_visible_probe_fraction'], 1.)

    def test_partial_occlusion_and_pair_are_distinct_geometry(self):
        root, truth = fixture('12_partial_occlusion')
        partial = scenes.validate_case(root, truth)['targets'][0]
        self.assertGreater(partial['geometric_visible_probe_fraction'], .2)
        self.assertLess(partial['geometric_visible_probe_fraction'], .8)
        root, truth = fixture('11_close_pair_180mm')
        pair = scenes.validate_case(root, truth)['targets']
        self.assertEqual(len(pair), 2)
        self.assertTrue(all(p['geometric_visible_probe_fraction'] == 1. for p in pair))
        sorted_boxes = sorted(p['nominal_front_bbox_uv'] for p in pair)
        self.assertLess(sorted_boxes[0][2], sorted_boxes[1][0])

    def test_required_sensor_system_cannot_disappear(self):
        root, truth = fixture('01_blank')
        world = root.find('world')
        world.remove(world.find("plugin[@name='gz::sim::systems::Sensors']"))
        with self.assertRaisesRegex(ValueError, 'Required Gazebo systems'):
            scenes.validate_case(root, truth)

    def test_robot_intrusion_fails_even_when_sdf_matches_truth(self):
        root, truth = fixture('10_red_panel_negative')
        set_box(root, truth, 'large_red_panel', xyz=(.1, 0., .2))
        with self.assertRaisesRegex(ValueError, 'robot envelope'):
            scenes.validate_case(root, truth)

    def test_detached_mount_is_rejected(self):
        root, truth = fixture('03_center_0800mm')
        backing = next(b for b in truth['all_static_boxes_offline_only'] if b['role'] == 'mount')
        set_box(root, truth, backing['name'], xyz=(backing['xyz_m'][0]+.02, 0., .3))
        with self.assertRaisesRegex(ValueError, 'not flush'):
            scenes.validate_case(root, truth)

    def test_full_occlusion_is_not_called_partial(self):
        root, truth = fixture('12_partial_occlusion')
        set_box(root, truth, 'grey_occluder', xyz=(.684, 0., .14), size=(.02, .2, .28))
        with self.assertRaisesRegex(ValueError, 'partially obscure'):
            scenes.validate_case(root, truth)

    def test_out_of_view_fixture_is_rejected(self):
        case = {'id': 'bad_outside_view', 'kind': 'single', 'range_m': 1.,
                'bearing_deg': 60., 'incidence_deg': 0.}
        root, truth = scenes.build_case(case, reference(), builder.box)
        with self.assertRaisesRegex(ValueError, 'clipped'):
            scenes.validate_case(root, truth)

    def test_backing_tangent_offset_cannot_pass_normal_flush_check(self):
        root, truth = fixture('03_center_0800mm')
        backing = next(b for b in truth['all_static_boxes_offline_only'] if b['role'] == 'mount')
        set_box(root, truth, backing['name'], xyz=(backing['xyz_m'][0], 1., .3))
        with self.assertRaisesRegex(ValueError, 'contain the target back face'):
            scenes.validate_case(root, truth)

    def test_rotated_backing_cannot_claim_same_front_plane(self):
        root, truth = fixture('03_center_0800mm')
        backing = next(b for b in truth['all_static_boxes_offline_only'] if b['role'] == 'mount')
        backing['yaw_rad'] += .1
        set_box(root, truth, backing['name'], xyz=backing['xyz_m'])
        with self.assertRaises(ValueError):
            scenes.validate_case(root, truth)

    def test_front_truth_must_match_actual_box_not_just_requested_bearing(self):
        root, truth = fixture('03_center_0800mm')
        truth['targets_ground_truth'][0]['front_face_center_world_xyz_m'][0] += .05
        truth['targets_ground_truth'][0]['desired_horizontal_camera_range_m'] += .05
        with self.assertRaisesRegex(ValueError, 'actual box geometry'):
            scenes.validate_case(root, truth)

    def test_offscreen_fixture_solid_collision_is_rejected(self):
        root, truth = fixture('01_blank')
        set_box(root, truth, 'room_front', xyz=(5.05, .025, .3))
        with self.assertRaisesRegex(ValueError, 'solids penetrate'):
            scenes.validate_case(root, truth)

    def test_sat_box_overlap_rotation_height_and_flush_contact(self):
        a = {'xyz_m': [0., 0., 0.], 'size_xyz_m': [2., .2, 1.], 'yaw_rad': math.pi/4}
        b = {'xyz_m': [0., 0., 0.], 'size_xyz_m': [2., .2, 1.], 'yaw_rad': -math.pi/4}
        self.assertTrue(scenes.boxes_penetrate(a, b))
        b['xyz_m'][2] = 1.
        self.assertFalse(scenes.boxes_penetrate(a, b))
        b['xyz_m'] = [3., 0., 0.]
        self.assertFalse(scenes.boxes_penetrate(a, b))
        a = {'xyz_m': [0., 0., 0.], 'size_xyz_m': [1., 1., 1.], 'yaw_rad': 0.}
        b = {**a, 'xyz_m': [1., 0., 0.]}
        self.assertFalse(scenes.boxes_penetrate(a, b))
        b['xyz_m'][0] -= .01
        self.assertTrue(scenes.boxes_penetrate(a, b))

    def test_local_collision_pose_cannot_hide_changed_geometry(self):
        root, truth = fixture('03_center_0800mm')
        collision = root.find("world/model[@name='red_box_0']/link/collision")
        ET.SubElement(collision, 'pose').text = '0.4 0 0 0 0 0'
        with self.assertRaisesRegex(ValueError, 'local pose'):
            scenes.validate_case(root, truth)

    def test_facing_away_front_is_not_positive_visibility(self):
        case = {'id': 'backwards', 'kind': 'single', 'range_m': .8, 'bearing_deg': 0., 'incidence_deg': 120.}
        root, truth = scenes.build_case(case, reference(), builder.box)
        with self.assertRaisesRegex(ValueError, 'does not face'):
            scenes.validate_case(root, truth)

    def test_negative_panel_cannot_silently_leave_view(self):
        root, truth = fixture('10_red_panel_negative')
        set_box(root, truth, 'large_red_panel', xyz=(-.8, 0., .2))
        with self.assertRaisesRegex(ValueError, 'behind nominal optical plane'):
            scenes.validate_case(root, truth)

    def test_rgb_clip_also_limits_nominal_fixture(self):
        case = next(c for c in scenes.cases() if c['id'] == '05_center_3200mm')
        ref = reference()
        ref['rgb_clip_m'] = [.05, 2.]
        root, truth = scenes.build_case(case, ref, builder.box)
        with self.assertRaisesRegex(ValueError, 'clipped'):
            scenes.validate_case(root, truth)

    def test_label_and_material_disagreement_are_rejected(self):
        root, truth = fixture('01_blank')
        truth['target_count_offline_only'] = 1
        with self.assertRaisesRegex(ValueError, 'roles/count'):
            scenes.validate_case(root, truth)
        root, truth = fixture('01_blank')
        root.find("world/model[@name='floor']/link/visual/material/ambient").text = '1 0 0 1'
        with self.assertRaisesRegex(ValueError, 'material disagreement'):
            scenes.validate_case(root, truth)


ROBOT = '''<robot>
<joint name="base_footprint_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/><origin xyz="0 0 .15"/></joint>
<joint name="camera_joint" type="fixed"><parent link="base_link"/><child link="camera_link"/><origin xyz=".084 0 .03"/></joint>
<joint name="camera_optical_joint" type="fixed"><parent link="camera_link"/><child link="camera_optical_frame"/>
<origin xyz="0 0 0" rpy="-1.5707963267948966 0 -1.5707963267948966"/></joint></robot>'''
SENSOR = '''<robot><gazebo reference="camera_link"><sensor type="rgbd_camera" name="camera">
<gz_frame_id>camera_link</gz_frame_id><camera><optical_frame_id>camera_optical_frame</optical_frame_id>
<horizontal_fov>1.3962634</horizontal_fov><image><width>640</width><height>480</height></image>
<clip><near>.05</near><far>8</far></clip><depth_camera><clip><near>.1</near><far>8</far></clip></depth_camera>
</camera></sensor></gazebo></robot>'''


class SourceAndGenerationChecks(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='limo-red-calibration-')
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.model, self.sensor = self.directory/'robot.xacro', self.directory/'sensor.xacro'
        self.model.write_text(ROBOT)
        self.sensor.write_text(SENSOR)

    def test_reference_chain_and_source_hashes_are_not_assumed(self):
        ref = scenes.load_reference(self.model, self.sensor)
        self.assertEqual(ref['camera_world_xyz_nominal'], [.084, 0., .18])
        self.assertEqual(ref['rgb_clip_m'], [.05, 8.])
        self.assertEqual(ref['source_sha256']['sensor_xacro'], scenes.sha(self.sensor))
        self.assertFalse(ref['runtime_calibration_verified'])
        self.model.write_text(ROBOT.replace('.084 0 .03', '.09 0 .04'))
        ref = scenes.load_reference(self.model, self.sensor)
        self.assertEqual(ref['camera_world_xyz_nominal'], [.09, 0., .19])

    def test_sensor_and_camera_nonzero_or_qualified_pose_are_rejected(self):
        for path, attrs, value in (('gazebo/sensor', {}, '.1 0 0 0 0 0'),
                                   ('gazebo/sensor/camera', {}, '0 0 0 0 0 .2'),
                                   ('gazebo/sensor', {'relative_to': 'other'}, '0 0 0 0 0 0')):
            with self.subTest(path=path, attrs=attrs):
                root = ET.fromstring(SENSOR)
                ET.SubElement(root.find(path), 'pose', attrs).text = value
                ET.ElementTree(root).write(self.sensor)
                with self.assertRaisesRegex(ValueError, 'local pose'):
                    scenes.load_reference(self.model, self.sensor)

    def test_custom_projection_invalid_clip_and_unexpanded_xacro_fail_closed(self):
        variants = [SENSOR.replace('<camera>', '<camera><lens/>'),
                    SENSOR.replace('<far>8</far>', '<far>inf</far>'),
                    SENSOR.replace('<horizontal_fov>1.3962634', '<horizontal_fov>${camera_fov}'),
                    SENSOR.replace('<optical_frame_id>camera_optical_frame', '<optical_frame_id>wrong_frame')]
        for text in variants:
            with self.subTest(text=text):
                self.sensor.write_text(text)
                with self.assertRaises(ValueError):
                    scenes.load_reference(self.model, self.sensor)

    def test_duplicate_joints_and_rotated_mount_are_rejected(self):
        root = ET.fromstring(ROBOT)
        root.append(copy.deepcopy(root.find("joint[@name='camera_joint']")))
        ET.ElementTree(root).write(self.model)
        with self.assertRaisesRegex(ValueError, 'exactly one'):
            scenes.load_reference(self.model, self.sensor)
        self.model.write_text(ROBOT.replace('xyz=".084 0 .03"', 'xyz=".084 0 .03" rpy="0 0 .1"'))
        with self.assertRaisesRegex(ValueError, 'unrotated'):
            scenes.load_reference(self.model, self.sensor)

    def test_generate_all_twelve_to_new_directory_and_refuse_overwrite(self):
        output = self.directory/'generated'
        manifest = scenes.generate(output, self.model, self.sensor)
        self.assertEqual(manifest['case_count'], 12)
        self.assertIsNone(manifest['runtime_test_results'])
        self.assertIsNone(manifest['effective_detection_range'])
        self.assertFalse(manifest['final_target_definition_verified'])
        self.assertEqual(len(list((output/'worlds').glob('*.sdf'))), 12)
        self.assertEqual(len(list((output/'offline_truth').glob('*.json'))), 12)
        for case in manifest['cases']:
            self.assertEqual(case['world_sha256'], scenes.sha(output/case['world']))
            self.assertEqual(case['truth_sha256'], scenes.sha(output/case['offline_truth']))
            truth = json.loads((output/case['offline_truth']).read_text())
            self.assertFalse(truth['effective_detection_range_claimed'])
            self.assertFalse(truth['final_target_definition_verified'])
            self.assertEqual(truth['status'], 'designed_not_rendered_or_tested')
        before = scenes.sha(output/'manifest.json')
        with self.assertRaisesRegex(ValueError, 'overwrite'):
            scenes.generate(output, self.model, self.sensor)
        self.assertEqual(before, scenes.sha(output/'manifest.json'))


if __name__ == '__main__':
    unittest.main(verbosity=2)
