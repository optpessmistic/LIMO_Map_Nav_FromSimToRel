"""Small synthetic projection cases only; no ROS, raw images or target truth."""
from copy import deepcopy
import json
import math
import unittest

from tools.visibility.geometry import classify_points


class DepthVisibilityTests(unittest.TestCase):
    def setUp(self):
        self.info = {'frame_id':'test_optical', 'stamp_ns':100,
            'calibration':{'width':8, 'height':6, 'k':[2.,0.,3.,0.,2.,2.,0.,0.,1.],
                'r':[1.,0.,0.,0.,1.,0.,0.,0.,1.],
                'p':[2.,0.,3.,0.,0.,2.,2.,0.,0.,0.,1.,0.], 'd':[0.,0.,0.,0.,0.],
                'distortion_model':'plumb_bob','binning_x':0,'binning_y':0,
                'roi':{'x_offset':0,'y_offset':0,'width':0,'height':0,'do_rectify':False}}}
        self.pose = {'parent_frame':'test_map', 'child_frame':'test_optical',
                     'stamp_ns':100, 'pose':[0.,0.,0.,0.,0.,0.,1.]}
        self.depth = [[3.]*8 for _ in range(6)]
        self.options = dict(points_frame='test_map',depth_frame='test_optical',depth_stamp_ns=100,
            depth_encoding='32FC1',depth_semantics='optical_z',registered_to_camera=True,
            optical_axes='x_right_y_down_z_forward',min_depth_m=.1,max_depth_m=6.,
            uncertainty_bound_m=None,camera_info_max_age_ns=0)

    def evaluate(self, points, **changes):
        return classify_points(self.info,self.pose,self.depth,points,**(self.options|changes))

    def test_required_states_and_optical_y_down(self):
        report = self.evaluate([[0,0,-1],[0,0,0],[10,0,1],[0,0,2],[0,0,4],[0,1,2]])
        self.assertEqual([row['status'] for row in report['rows']],
                         ['behind_camera','behind_camera','outside_image','depth_clear','occluded','depth_clear'])
        self.assertEqual(report['rows'][-1]['pixel_uv'],[3.,3.])
        self.assertFalse(report['recognition_coverage_verified'])
        self.assertFalse(report['depth_visibility_verified'])
        self.assertFalse(report['mission_success_claimed'])

    def test_inverse_translated_rotated_map_from_camera(self):
        self.pose['pose'] = [10.,20.,30.,0.,math.sqrt(.5),0.,math.sqrt(.5)]
        row = self.evaluate([[12.,20.,30.]])['rows'][0]
        for actual, expected in zip(row['point_optical_m'],[0.,0.,2.]):
            self.assertAlmostEqual(actual,expected)
        self.assertAlmostEqual(row['pixel_uv'][0],3.)
        self.assertEqual(row['status'],'depth_clear')

    def test_axial_depth_not_euclidean_range(self):
        self.depth = [[2.1]*8 for _ in range(6)]
        row = self.evaluate([[2.,0.,2.]])['rows'][0]
        self.assertGreater(math.sqrt(8),2.1)
        self.assertEqual(row['status'],'depth_clear')
        self.assertAlmostEqual(row['delta_min_m'],.1)

    def test_16bit_millimetres_match_metres(self):
        expected = self.evaluate([[0,0,2.]])['rows'][0]
        self.depth = [[3000]*8 for _ in range(6)]
        actual = self.evaluate([[0,0,2.]],depth_encoding='16UC1')['rows'][0]
        self.assertEqual(actual,expected)

    def test_query_range_is_recorded_separately_from_measured_surface_bounds(self):
        rows = self.evaluate([[0,0,z] for z in (-1., .05, .1, 6., 7.)])['rows']
        self.assertEqual([r['query_depth_relation'] for r in rows],
            ['behind_camera', 'below_min_depth', 'within_depth_bounds',
             'within_depth_bounds', 'above_max_depth'])
        self.assertEqual([r['status'] for r in rows],
            ['behind_camera', 'depth_clear', 'depth_clear', 'occluded', 'occluded'])

    def test_invalid_samples_are_never_filled_with_far_depth(self):
        for value in (float('nan'),float('inf'),0.,-1.,6.1,None,True):
            with self.subTest(value=value):
                self.depth[2][3] = value
                report = self.evaluate([[0,0,2.]])
                self.assertEqual(report['rows'][0]['status'],'invalid_depth')
                json.dumps(report,allow_nan=False)

    def test_mixed_depth_edge_is_ambiguous_not_interpolated_clear(self):
        self.depth[2][3] = 1.
        row = self.evaluate([[.25,.25,2.]])['rows'][0]
        self.assertEqual(row['status'],'depth_ambiguous')
        self.assertEqual(row['depth_samples_m'],[1.,3.,3.,3.])
        self.assertEqual((row['delta_min_m'],row['delta_max_m']),(-1.,1.))

    def test_surface_and_assumed_uncertainty_are_not_verified_clear(self):
        self.assertEqual(self.evaluate([[0,0,3.]])['rows'][0]['status'],'depth_ambiguous')
        report = self.evaluate([[0,0,2.95]],uncertainty_bound_m=.1)
        self.assertEqual(report['rows'][0]['status'],'depth_ambiguous')
        self.assertEqual(report['comparison_mode'],'assumed_bound')
        self.assertFalse(report['uncertainty_bound_verified'])

    def test_image_edge_stencil_is_unknown_and_not_clamped(self):
        row = self.evaluate([[4.,0.,2.]])['rows'][0]  # u=7, inside image, lacks adjacent pixel.
        self.assertEqual(row['pixel_uv'],[7.,2.])
        self.assertEqual(row['status'],'invalid_depth')
        self.assertEqual(row['reason'],'2x2_stencil_outside_image')

    def test_explicit_3d_height_and_size_assumption(self):
        with self.assertRaisesRegex(ValueError,'explicit coordinates'):
            self.evaluate([[1.,2.]])
        self.pose['pose'][2] = .03
        row = self.evaluate([[0.,0.,2.03]],assumed_size_m={
            'width_m':.06,'height_m':.20,'orientation':'parallel_image_plane'})['rows'][0]
        self.assertEqual(row['point_3d_m'][2],2.03)
        self.assertAlmostEqual(row['point_optical_m'][2],2.)
        self.assertAlmostEqual(row['projected_size_px']['width'],.06)
        self.assertAlmostEqual(row['projected_size_px']['height'],.20)

    def test_frame_stamp_registration_and_range_semantics_rejected(self):
        for change in ({'points_frame':'other'},{'depth_frame':'other'},{'depth_stamp_ns':101},
                       {'registered_to_camera':False},{'depth_semantics':'euclidean_range'},
                       {'optical_axes':'x_forward_y_left_z_up'},{'uncertainty_bound_m':-.1}):
            with self.subTest(change=change),self.assertRaises(ValueError):
                self.evaluate([[0,0,2]],**change)
        self.info['stamp_ns'] = 98
        with self.assertRaises(ValueError):self.evaluate([[0,0,2]])
        self.assertEqual(self.evaluate([[0,0,2]],camera_info_max_age_ns=2)['camera_info_age_ns'],2)

    def test_unsupported_camera_models_and_bad_quaternions_rejected(self):
        original = deepcopy(self.info)
        for key, value in (('d',[.1,0,0,0,0]),('binning_x',2),
                           ('r',[0.]*9),('p',[0.]*12),('k',[0.]*9)):
            with self.subTest(key=key),self.assertRaises(ValueError):
                self.info = deepcopy(original)
                self.info['calibration'][key] = value
                self.evaluate([[0,0,2]])
        self.info = original
        for quaternion in ([0,0,0,0],[0,0,0,2],[0,0,float('nan'),1]):
            with self.subTest(quaternion=quaternion),self.assertRaises(ValueError):
                self.pose['pose'][3:] = quaternion
                self.evaluate([[0,0,2]])

    def test_input_shape_and_nonfinite_geometry_fail_without_guessing(self):
        for points in (None, '0,0,2', [None], [[0.,0.,float('inf')]], [[0,0,10**400]]):
            with self.subTest(points=points), self.assertRaises(ValueError):
                self.evaluate(points)
        for depth in (None, self.depth[:-1], [None]*6, [[3.]*7 for _ in range(6)]):
            with self.subTest(depth=depth), self.assertRaises(ValueError):
                self.depth = depth
                self.evaluate([[0,0,2]])

    def test_work_bounds_are_checked_before_array_contents(self):
        class TooManyPoints:
            def __len__(self):
                return 100001

            def __iter__(self):
                raise AssertionError('Oversized input must not be traversed')

        with self.assertRaisesRegex(ValueError, 'work bound'):
            self.evaluate(TooManyPoints())
        self.info['calibration'].update(width=4001, height=4000)
        with self.assertRaisesRegex(ValueError, 'work bound'):
            self.evaluate([])

    def test_epsilon_equality_is_ambiguous_on_both_sides(self):
        rows = self.evaluate([[0,0,2.75],[0,0,3.25],[0,0,2.5],[0,0,3.5]],
                             uncertainty_bound_m=.25)['rows']
        self.assertEqual([row['status'] for row in rows],
                         ['depth_ambiguous','depth_ambiguous','depth_clear','occluded'])

    def test_depth_range_endpoints_and_unsigned_encoding_boundaries(self):
        for value in (.1, 6.):
            with self.subTest(endpoint=value):
                self.depth = [[value]*8 for _ in range(6)]
                self.assertNotEqual(self.evaluate([[0,0,2]])['rows'][0]['status'], 'invalid_depth')
        self.depth = [[3000]*8 for _ in range(6)]
        for value in (3000., -1, 65536, 10**400):
            with self.subTest(unsigned=value):
                self.depth[2][3] = value
                row = self.evaluate([[0,0,2]], depth_encoding='16UC1', max_depth_m=70.)['rows'][0]
                self.assertEqual(row['status'], 'invalid_depth')
                json.dumps(row, allow_nan=False)


if __name__ == '__main__':
    unittest.main(verbosity=2)
