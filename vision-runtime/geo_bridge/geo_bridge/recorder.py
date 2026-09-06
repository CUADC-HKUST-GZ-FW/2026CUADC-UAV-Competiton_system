"""Write overlay frames to an MP4 and a 1:1 per-frame sidecar JSONL."""

import json
import os
import time

import cv2


class OverlayRecorder:
    def __init__(self, output_path, fps=15.0):
        self.output_path = output_path
        self.fps = float(fps)
        self._writer = None
        self._size = None
        self.frames = 0
        self.started_at = None

    def _ensure(self, frame):
        height, width = frame.shape[:2]
        size = (width, height)
        if self._writer is not None and self._size == size:
            return True
        self.close()
        os.makedirs(os.path.dirname(self.output_path) or '.', exist_ok=True)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(self.output_path, fourcc, self.fps, size)
        if not writer.isOpened():
            return False
        self._writer = writer
        self._size = size
        self.started_at = time.time()
        return True

    def write(self, frame):
        """Append one frame. Returns 0-based frame_index, or None on failure."""
        if frame is None or frame.size == 0:
            return None
        if not self._ensure(frame):
            return None
        self._writer.write(frame)
        index = self.frames
        self.frames += 1
        return index

    def close(self):
        if self._writer is not None:
            self._writer.release()
            self._writer = None

    def summary(self):
        elapsed = 0.0 if self.started_at is None else time.time() - self.started_at
        return {
            'path': self.output_path,
            'frames': self.frames,
            'elapsed_sec': elapsed,
        }


class FrameSidecar:
    """One JSON object per MP4 frame, same stem as the video file."""

    def __init__(self, path):
        self.path = path
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        self._handle = open(path, 'w', encoding='utf-8')
        self.rows = 0

    def append(self, frame_index, sample, now=None, extra=None):
        now = time.time() if now is None else float(now)
        age = None
        latitude = longitude = relative_alt_m = amsl_m = source = None
        if sample is not None:
            latitude = sample.latitude
            longitude = sample.longitude
            relative_alt_m = sample.relative_alt_m
            amsl_m = sample.amsl_m
            source = sample.source
            if sample.wall_time > 0.0:
                age = now - sample.wall_time
        payload = {
            'frame_index': int(frame_index),
            'timestamp_unix_ms': int(round(now * 1000.0)),
            'latitude': latitude,
            'longitude': longitude,
            'relative_alt_m': relative_alt_m,
            'amsl_m': amsl_m,
            'telemetry_age_sec': age,
            'source': source or 'none',
        }
        if extra:
            payload.update(extra)
        self._handle.write(json.dumps(payload, ensure_ascii=False) + '\n')
        self._handle.flush()
        self.rows += 1
        return payload

    def close(self):
        if self._handle is not None:
            self._handle.close()
            self._handle = None

    def summary(self):
        return {'path': self.path, 'rows': self.rows}


def sidecar_path_for_video(video_path):
    root, _ext = os.path.splitext(video_path)
    return root + '.jsonl'
