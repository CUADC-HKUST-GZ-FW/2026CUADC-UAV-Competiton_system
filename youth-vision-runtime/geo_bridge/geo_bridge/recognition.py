"""Follow the youth-vision-runtime JSONL recognition log."""

import json
import os
from typing import Iterator, Optional


class RecognitionTail:
    """Append-only reader that survives log rotation used by the pipeline scripts."""

    def __init__(self, path):
        self.path = path
        self._handle = None
        self._inode = None
        self._offset = 0

    def close(self):
        if self._handle is not None:
            self._handle.close()
            self._handle = None

    def _open_if_needed(self):
        if not self.path or not os.path.exists(self.path):
            return False
        stat = os.stat(self.path)
        if self._handle is None or self._inode != stat.st_ino:
            self.close()
            self._handle = open(self.path, 'r', encoding='utf-8')
            self._inode = stat.st_ino
            self._offset = 0
        if stat.st_size < self._offset:
            self._handle.seek(0)
            self._offset = 0
        elif self._handle.tell() != self._offset:
            self._handle.seek(self._offset)
        return True

    def poll(self) -> Iterator[dict]:
        if not self._open_if_needed():
            return
        while True:
            line = self._handle.readline()
            if not line:
                break
            self._offset = self._handle.tell()
            text = line.strip()
            if not text:
                continue
            try:
                record = json.loads(text)
            except json.JSONDecodeError:
                continue
            if record.get('record_type') == 'status':
                continue
            yield record


def detection_pixel(record, index=0):
    """Prefer the three-point centroid, then the ROI center."""
    detections = record.get('detections') or []
    item = detections[index] if index < len(detections) else record
    keypoints = item.get('keypoints') or record.get('keypoints') or []
    points = []
    for keypoint in keypoints[:3]:
        if isinstance(keypoint, (list, tuple)) and len(keypoint) >= 2:
            points.append((float(keypoint[0]), float(keypoint[1])))
    if points:
        return (
            sum(point[0] for point in points) / len(points),
            sum(point[1] for point in points) / len(points),
        )
    roi = item.get('roi') or record.get('roi')
    if isinstance(roi, (list, tuple)) and len(roi) >= 4:
        return (float(roi[0]) + float(roi[2]) * 0.5, float(roi[1]) + float(roi[3]) * 0.5)
    return None


def arrow_image_heading_deg(record, index=0):
    """Image-plane heading of tip-from-base, 0 at +x, clockwise like image y-down."""
    detections = record.get('detections') or []
    item = detections[index] if index < len(detections) else record
    keypoints = item.get('keypoints') or record.get('keypoints') or []
    if len(keypoints) < 3:
        return None
    try:
        tip = keypoints[0]
        left = keypoints[1]
        right = keypoints[2]
        mid_x = 0.5 * (float(left[0]) + float(right[0]))
        mid_y = 0.5 * (float(left[1]) + float(right[1]))
        dx = float(tip[0]) - mid_x
        dy = float(tip[1]) - mid_y
    except (TypeError, ValueError, IndexError):
        return None
    if dx == 0.0 and dy == 0.0:
        return None
    import math
    return (math.degrees(math.atan2(dx, -dy)) + 360.0) % 360.0


def record_wall_time(record) -> Optional[float]:
    stamp_ms = record.get('timestamp_unix_ms')
    if stamp_ms is None:
        return None
    try:
        return float(stamp_ms) / 1000.0
    except (TypeError, ValueError):
        return None
