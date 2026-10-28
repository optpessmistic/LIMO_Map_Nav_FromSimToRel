import math
import unittest
from export_annotated_map import world_to_image, image_contains_world_point


class ProjectionTests(unittest.TestCase):
    def test_ros_bottom_left_to_image_top_left(self):
        self.assertEqual(world_to_image(1.05, 2.05, [1, 2, 0], .1, 20),
                         (0.5000000000000004, 19.5))

    def test_rotated_map(self):
        x, y = world_to_image(9.85, 20.25, [10, 20, math.pi/2], .1, 30)
        self.assertAlmostEqual(x, 2.5)
        self.assertAlmostEqual(y, 28.5)

    def test_invalid_geometry(self):
        for resolution in [0, -1, math.nan]:
            with self.assertRaises(ValueError):
                world_to_image(1, 1, [0, 0, 0], resolution, 100)

    def test_included_lower_edge_and_excluded_upper_edge(self):
        lower = world_to_image(0, 0, [0, 0, 0], 1, 20)
        upper = world_to_image(0, 20, [0, 0, 0], 1, 20)
        self.assertTrue(image_contains_world_point(*lower, 30, 20))
        self.assertFalse(image_contains_world_point(*upper, 30, 20))
        self.assertFalse(image_contains_world_point(30, 10, 30, 20))

    def test_invalid_height_and_nonfinite_target(self):
        for height in (0, -1, math.nan):
            with self.assertRaises(ValueError):
                world_to_image(0, 0, [0, 0, 0], 1, height)
        with self.assertRaises(ValueError):
            world_to_image(math.inf, 0, [0, 0, 0], 1, 20)


if __name__ == '__main__':
    unittest.main()
