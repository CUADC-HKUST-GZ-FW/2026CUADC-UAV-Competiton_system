#!/usr/bin/env python3
import os
import sys
import unittest

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, ROOT)

from geo_bridge.telemetry import TelemetryBuffer


class TelemetryBufferTest(unittest.TestCase):
    def test_merges_rel_alt_and_global_into_latest_sample(self):
        buf = TelemetryBuffer()
        buf.update_rel_alt(1.0, 32.5)
        buf.update_global(1.1, 22.8847501, 113.4961007, 45.0)
        sample = buf.latest
        self.assertEqual(32.5, sample.relative_alt_m)
        self.assertAlmostEqual(22.8847501, sample.latitude)
        self.assertAlmostEqual(113.4961007, sample.longitude)
        self.assertEqual(45.0, sample.amsl_m)
        self.assertEqual('global', sample.source)
        self.assertEqual(1, buf.rel_alt_count)
        self.assertEqual(1, buf.global_count)

    def test_nearest_returns_closest_wall_clock_sample(self):
        buf = TelemetryBuffer()
        buf.update_rel_alt(1.00, 10.0)
        buf.update_global(1.20, 22.0, 113.0, 40.0)
        sample, age = buf.nearest(1.18)
        self.assertIsNotNone(sample)
        self.assertEqual('global', sample.source)
        self.assertAlmostEqual(-0.02, age, places=6)


if __name__ == '__main__':
    unittest.main()
