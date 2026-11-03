"""Independent height/occlusion and complete-domain aggregation checks."""
from copy import deepcopy
import math
import unittest

from tools.visibility.layers import build_layers


def inputs():
    grid = {'width': 2, 'height': 2, 'resolution': .2, 'frame_id': 'map',
        'origin': {'xyz': [-.2, -.2, 0.], 'quaternion_xyzw': [0., 0., 0., 1.]},
        'data': [-1, 0, 100, 50], 'stamp_ns': 10}
    camera = {'frame_id': 'optical', 'stamp_ns': 10, 'calibration': {
        'width': 5, 'height': 5, 'k': [1., 0., 2., 0., 1., 2., 0., 0., 1.],
        'p': [1., 0., 2., 0., 0., 1., 2., 0., 0., 0., 1., 0.],
        'r': [1., 0., 0., 0., 1., 0., 0., 0., 1.], 'd': [0.]*5,
        'distortion_model': 'plumb_bob', 'binning_x': 0, 'binning_y': 0,
        'roi': {'x_offset': 0, 'y_offset': 0, 'width': 0, 'height': 0, 'do_rectify': False}}}
    frame = {'retention_id': 'frame-1', 'request_sha256': 'a'*64,
        'camera_info': camera, 'request': {'tf': {'camera_pose': [0., 0., 0., 0., 0., 0., 1.], 'stamp_ns': 10}},
        'producer': {'rgb': {'stamp_ns': 10}, 'depth': {'frame_id': 'optical', 'stamp_ns': 10, 'encoding': '32FC1'},
            'detector_session_id': 'session-1'}, 'depth': [[3.]*5 for _ in range(5)]}
    params = dict(floor_z_m=0., height_reference_provenance='synthetic explicit plane',
        heights_m=[1., 2., 4.], min_depth_m=.1, max_depth_m=8., uncertainty_bound_m=None,
        registered_to_camera=True, depth_semantics='optical_z', optical_axes='x_right_y_down_z_forward')
    return grid, [frame], params


class LayersTests(unittest.TestCase):
    def test_all_map_classes_remain_and_height_changes_occlusion(self):
        grid, frames, params = inputs()
        out = build_layers(grid, frames, **params)
        self.assertEqual(out['map']['data'], [-1, 0, 100, 50])
        self.assertEqual([p['states'] for p in out['layers']],
            [['nominal_clear']*4, ['nominal_clear']*4, ['occluded']*4])
        self.assertEqual(out['classification_count'], 12)
        for plane in out['layers']:
            self.assertEqual(set(plane['counts_by_map_class']), {'unknown', 'free', 'occupied', 'intermediate'})
        self.assertFalse(out['recognition_coverage_verified'])
        self.assertFalse(out['historical_reprojection_verified'])
        self.assertFalse(out['floor_reference_verified'])

    def test_clear_display_preserves_other_frames_invalid_and_occluded_support(self):
        grid, frames, params = inputs()
        invalid, obstacle = deepcopy(frames[0]), deepcopy(frames[0])
        invalid['retention_id'], obstacle['retention_id'] = 'frame-2', 'frame-3'
        invalid['depth'] = [[math.nan]*5 for _ in range(5)]
        obstacle['depth'] = [[.5]*5 for _ in range(5)]
        out = build_layers(grid, frames+[invalid, obstacle], **params)
        supports = out['layers'][0]['support_frame_indices']
        self.assertEqual(out['layers'][0]['states'], ['nominal_clear']*4)
        for i in range(4):
            self.assertEqual(supports['depth_clear'][i], [0])
            self.assertEqual(supports['invalid_depth'][i], [1])
            self.assertEqual(supports['occluded'][i], [2])
        self.assertEqual(out['classification_count'], 36)
        self.assertEqual(out['distinct_source_stamps'], 1)  # repeated source stamps are explicit

    def test_assumed_uncertainty_preserves_surface_ambiguity(self):
        grid, frames, params = inputs()
        params.update(heights_m=[3.], uncertainty_bound_m=.02)
        out = build_layers(grid, frames, **params)
        self.assertEqual(out['layers'][0]['states'], ['mixed_or_surface']*4)
        self.assertFalse(out['depth_uncertainty_verified'])

    def test_query_range_supports_preserve_every_point_frame_without_cropping(self):
        grid, frames, params = inputs()
        params.update(floor_z_m=-1., heights_m=[0., 1.05, 2., 10.])
        out = build_layers(grid, frames, **params)
        relations = ['behind_camera', 'below_min_depth', 'within_depth_bounds', 'above_max_depth']
        for plane, relation in zip(out['layers'], relations):
            counts = plane['query_depth_relation_counts']
            self.assertEqual(sum(counts.values()), plane['classification_count'])
            self.assertEqual(counts[relation], 4)
            self.assertEqual(plane['query_depth_support_frame_indices'][relation], [[0]]*4)
        self.assertEqual(out['classification_count'], 16)
        self.assertEqual(out['depth_bounds_apply_to'], 'measured_surface_samples_only')

    def test_explicit_floor_is_applied_without_inferring_from_map_origin(self):
        grid, frames, params = inputs()
        grid['origin']['xyz'][2] = 100.
        params['floor_z_m'] = -2.
        out = build_layers(grid, frames, **params)
        self.assertEqual([p['map_z_m'] for p in out['layers']], [-1., 0., 2.])
        self.assertEqual(out['layers'][0]['states'], ['no_view']*4)
        self.assertEqual(out['layers'][2]['states'], ['nominal_clear']*4)

    def test_missing_domain_invalid_inputs_are_not_silently_trimmed(self):
        grid, frames, params = inputs()
        with self.assertRaises(ValueError):
            build_layers(grid, [], **params)
        with self.assertRaises(ValueError):
            build_layers(grid, frames+frames, **params)
        bad = deepcopy(grid)
        bad['data'] = bad['data'][:-1]
        with self.assertRaises(ValueError):
            build_layers(bad, frames, **params)
        bad = deepcopy(grid)
        bad['origin']['quaternion_xyzw'] = [math.sin(.1), 0., 0., math.cos(.1)]
        with self.assertRaises(ValueError):
            build_layers(bad, frames, **params)
        with self.assertRaises(ValueError):
            build_layers(grid, frames, **{**params, 'registered_to_camera': False})


if __name__ == '__main__':
    unittest.main(verbosity=2)
