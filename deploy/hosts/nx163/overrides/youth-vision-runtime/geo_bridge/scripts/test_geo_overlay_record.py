#!/usr/bin/env python3
"""Bench recorder: vision overlay JPEG -> MP4 + 1:1 lat/lon/alt sidecar JSONL.

Does not open the MVS camera. Reads overlays/latest.jpg from youth_vision_runner.
Use --mock-* on the bench. Flight recording should use the ROS geo_bridge node.
"""

import argparse
import json
import os
import sys
import time
from datetime import datetime

import cv2

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from geo_bridge.recorder import FrameSidecar, OverlayRecorder, sidecar_path_for_video
from geo_bridge.telemetry import TelemetrySample


def parse_args():
    parser = argparse.ArgumentParser(
        description='Record overlay frames and a per-frame lat/lon/alt sidecar'
    )
    parser.add_argument(
        '--overlay-jpeg',
        default='/home/nx163/youth-vision-runtime/overlays/latest.jpg',
    )
    parser.add_argument(
        '--output-dir',
        default='/home/nx163/youth-vision-runtime/logs/geo_recordings',
    )
    parser.add_argument('--duration-sec', type=float, default=30.0)
    parser.add_argument('--fps', type=float, default=15.0)
    parser.add_argument('--mock-alt', type=float, default=None, help='relative altitude meters')
    parser.add_argument('--mock-amsl', type=float, default=None, help='AMSL meters')
    parser.add_argument('--mock-lat', type=float, default=22.8847501)
    parser.add_argument('--mock-lon', type=float, default=113.4961007)
    return parser.parse_args()


def mock_sample(args):
    return TelemetrySample(
        wall_time=time.time(),
        relative_alt_m=args.mock_alt,
        latitude=args.mock_lat,
        longitude=args.mock_lon,
        amsl_m=args.mock_amsl,
        source='mock',
    )


def main():
    args = parse_args()
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    os.makedirs(args.output_dir, exist_ok=True)
    video_path = os.path.join(args.output_dir, f'geo_overlay_test_{stamp}.mp4')
    sidecar_path = sidecar_path_for_video(video_path)
    recorder = OverlayRecorder(video_path, fps=args.fps)
    sidecar = FrameSidecar(sidecar_path)
    last_mtime = None
    started = time.time()

    print(f'recording={video_path}')
    print(f'sidecar={sidecar_path}')
    print(f'mock_lat={args.mock_lat} mock_lon={args.mock_lon} mock_alt={args.mock_alt}')
    print(f'overlay_jpeg={args.overlay_jpeg}')

    try:
        while time.time() - started < args.duration_sec:
            if os.path.exists(args.overlay_jpeg):
                mtime = os.path.getmtime(args.overlay_jpeg)
                if last_mtime is None or mtime > last_mtime:
                    frame = cv2.imread(args.overlay_jpeg)
                    if frame is not None:
                        last_mtime = mtime
                        now = time.time()
                        sample = mock_sample(args)
                        frame_index = recorder.write(frame)
                        if frame_index is not None:
                            sidecar.append(frame_index, sample, now=now)
            time.sleep(1.0 / max(args.fps, 1.0))
    except KeyboardInterrupt:
        pass
    finally:
        video_summary = recorder.summary()
        sidecar_summary = sidecar.summary()
        sidecar.close()
        recorder.close()
        print(
            json.dumps(
                {
                    'video': video_summary['path'],
                    'sidecar': sidecar_summary['path'],
                    'frames': video_summary['frames'],
                    'sidecar_rows': sidecar_summary['rows'],
                },
                ensure_ascii=False,
            )
        )
        if video_summary['frames'] == 0:
            print('no overlay frames were captured; start the vision pipeline first', file=sys.stderr)
            return 1
        if video_summary['frames'] != sidecar_summary['rows']:
            print('sidecar row count does not match video frames', file=sys.stderr)
            return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
