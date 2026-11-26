import unittest

from r2s2r.core.sensor_suite import scale_pinhole_intrinsics


class CameraScaleTest(unittest.TestCase):
    def test_half_scale(self):
        width, height, fx, fy = scale_pinhole_intrinsics(1600, 1296, 737.36, 737.29, 0.5)
        self.assertEqual((width, height), (800, 648))
        self.assertAlmostEqual(fx, 368.68)
        self.assertAlmostEqual(fy, 368.645)

    def test_minimum_size(self):
        width, height, _, _ = scale_pinhole_intrinsics(1600, 1296, 737.0, 737.0, 0.001)
        self.assertGreaterEqual(width, 2)
        self.assertGreaterEqual(height, 2)

    def test_rejects_bad_scale(self):
        for scale in (0.0, -0.5, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                scale_pinhole_intrinsics(1600, 1296, 737.0, 737.0, scale)


if __name__ == '__main__':
    unittest.main()
