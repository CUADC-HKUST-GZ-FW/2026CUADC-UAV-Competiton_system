#!/usr/bin/env python3
import os
import sys
import tempfile
import unittest

import numpy as np

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, ROOT)

from geo_bridge.recorder import FrameSidecar, OverlayRecorder, sidecar_path_for_video
from geo_bridge.telemetry import TelemetrySample


class SidecarTest(unittest.TestCase):
    def test_sidecar_path_shares_video_stem(self):
        self.assertEqual(
            '/tmp/geo_overlay_20260817_221900.jsonl',
            sidecar_path_for_video('/tmp/geo_overlay_20260817_221900.mp4'),
        )

    def test_append_only_after_successful_video_write(self):
        with tempfile.TemporaryDirectory() as tmp:
            video_path = os.path.join(tmp, 'clip.mp4')
            sidecar = FrameSidecar(sidecar_path_for_video(video_path))
            recorder = OverlayRecorder(video_path, fps=15.0)
            frame = np.zeros((64, 80, 3), dtype=np.uint8)
            sample = TelemetrySample(
                wall_time=1.0,
                relative_alt_m=32.5,
                latitude=22.8847501,
                longitude=113.4961007,
                amsl_m=45.0,
                source='global',
            )
            indexes = []
            for _ in range(3):
                index = recorder.write(frame)
                self.assertIsNotNone(index)
                sidecar.append(index, sample, now=1.2)
                indexes.append(index)
            sidecar.close()
            recorder.close()
            self.assertEqual([0, 1, 2], indexes)
            with open(sidecar_path_for_video(video_path), encoding='utf-8') as handle:
                rows = [__import__('json').loads(line) for line in handle if line.strip()]
            self.assertEqual(3, len(rows))
            self.assertEqual(list(range(3)), [row['frame_index'] for row in rows])
            self.assertEqual(22.8847501, rows[0]['latitude'])
            self.assertEqual(113.4961007, rows[0]['longitude'])
            self.assertEqual(32.5, rows[0]['relative_alt_m'])
            self.assertEqual(45.0, rows[0]['amsl_m'])


if __name__ == '__main__':
    unittest.main()
