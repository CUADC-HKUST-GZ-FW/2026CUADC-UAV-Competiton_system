#!/usr/bin/env python3
import math
import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, ROOT)

from geo_bridge.projection import CameraModel, offset_to_latlon, project_pixel_to_ground


class ProjectionTest(unittest.TestCase):
    def setUp(self):
        self.camera = CameraModel(width=1440, height=1080, horizontal_fov_deg=60.0)

    def test_center_pixel_is_under_the_aircraft_when_level(self):
        offset = project_pixel_to_ground(720, 540, 32.0, self.camera, heading_deg=0.0)
        self.assertIsNotNone(offset)
        self.assertEqual('uncalibrated_fov_model', offset.reason)
        self.assertAlmostEqual(0.0, offset.east_m, places=2)
        self.assertAlmostEqual(0.0, offset.north_m, places=2)
        self.assertAlmostEqual(32.0, offset.camera_height_m, places=2)
        self.assertAlmostEqual(32.0, offset.range_m, places=2)

    def test_right_of_center_goes_east_when_heading_north(self):
        offset = project_pixel_to_ground(900, 540, 32.0, self.camera, heading_deg=0.0)
        self.assertGreater(offset.east_m, 1.0)
        self.assertAlmostEqual(0.0, offset.north_m, delta=1.0)

    def test_offset_to_latlon_moves_north(self):
        lat, lon = offset_to_latlon(22.88, 113.49, 0.0, 111.32)
        self.assertAlmostEqual(22.881, lat, places=5)
        self.assertAlmostEqual(113.49, lon, places=5)

    def test_rejects_tiny_height(self):
        offset = project_pixel_to_ground(720, 540, 0.05, self.camera)
        self.assertEqual('height_too_small', offset.reason)


if __name__ == '__main__':
    unittest.main()
