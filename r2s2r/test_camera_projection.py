"""Camera-projection coloring for cloud_slam (USD camera axes: +X fwd, +Y left, +Z up)."""
import importlib.util
import math
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
_spec = importlib.util.spec_from_file_location(
    'camera_projection',
    ROOT/'ros2_ws/src/sim_bridge_ros2/sim_bridge_ros2/camera_projection.py')
cp = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cp)

INTR = {'fx': 700.0, 'fy': 700.0, 'width': 1600, 'height': 1200}
IDENT = np.array([1., 0., 0., 0.])


class CameraProjectionTest(unittest.TestCase):
    def test_center_and_usd_axes(self):
        pts = np.array([[5., 0., 0.], [5., 1., 0.], [5., 0., 1.]])
        u, v, depth, mask = cp.project_points(pts, [0, 0, 0], IDENT, INTR)
        self.assertTrue(mask.all())
        self.assertAlmostEqual(u[0], 800.0)
        self.assertAlmostEqual(v[0], 600.0)
        self.assertLess(u[1], 800.0)   # +Y is left
        self.assertLess(v[2], 600.0)   # +Z is up
        np.testing.assert_allclose(depth, 5.0)

    def test_depth_and_fov_rejects(self):
        pts = np.array([[-1., 0., 0.], [0.1, 0., 0.], [5., 100., 0.]])
        _, _, _, mask = cp.project_points(pts, [0, 0, 0], IDENT, INTR, min_depth=0.15)
        self.assertFalse(mask.any())

    def test_body_rotation_is_applied(self):
        yaw90 = np.array([math.cos(math.pi/4), 0., 0., math.sin(math.pi/4)])
        pts = np.array([[0., 5., 0.]])
        u, v, _, mask = cp.project_points(pts, [0, 0, 0], yaw90, INTR)
        self.assertTrue(mask[0])
        self.assertAlmostEqual(u[0], 800.0)
        self.assertAlmostEqual(v[0], 600.0)

    def test_camera_offset_composes(self):
        body = ([1., 0., 0.], IDENT)
        cam = cp.compose_transform(body[0], body[1], [0.3, 0., 0.1], IDENT)
        pts = np.array([[6.3, 0., 0.1]])
        u, v, _, mask = cp.project_points(pts, cam[0], cam[1], INTR)
        self.assertTrue(mask[0])
        self.assertAlmostEqual(u[0], 800.0)
        self.assertAlmostEqual(v[0], 600.0)

    def test_z_buffer_keeps_nearest_surface(self):
        img = np.full((1200, 1600, 3), 77, np.uint8)
        pts = np.array([[5., 0., 0.], [2., 0., 0.]])
        colors, mask = cp.colorize_points(pts, img, [0, 0, 0], IDENT, INTR)
        self.assertEqual(int(mask.sum()), 1)
        self.assertTrue(mask[1])
        self.assertEqual(colors[1].tolist(), [77, 77, 77])

    def test_pixels_sample_by_projection(self):
        img = np.zeros((1200, 1600, 3), np.uint8)
        img[:, 400:] = [0, 200, 0]
        pts = np.array([[5., -1., 0.]])   # right of center -> pixel u > 800
        colors, mask = cp.colorize_points(pts, img, [0, 0, 0], IDENT, INTR)
        self.assertTrue(mask[0])
        self.assertEqual(colors[0].tolist(), [0, 200, 0])

    def test_nearest_stamp(self):
        stamps = [1.0, 1.1, 1.2]
        self.assertEqual(cp.nearest_stamp_index(stamps, 1.04), 0)
        self.assertEqual(cp.nearest_stamp_index(stamps, 1.16), 2)
        self.assertIsNone(cp.nearest_stamp_index([], 1.0))


if __name__ == '__main__':
    unittest.main()
