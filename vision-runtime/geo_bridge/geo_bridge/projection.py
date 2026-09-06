"""Pinhole ray versus a flat home-level ground plane.

Not used by the live geo_bridge node. Kept as a library for later target geolocation.

Frames used by this module:

* Image / OpenCV: origin top-left, x right, y down.
* Camera optical: x right, y down, z along the lens axis.
* Body NED: x forward, y right, z down.
* Local NED: same axes, origin at the aircraft, ground at z = height.

Default nadir mount (top of the image toward the nose):

    body_x = -cam_y
    body_y =  cam_x
    body_z =  cam_z
"""

import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class CameraModel:
    width: int = 1440
    height: int = 1080
    fx: Optional[float] = None
    fy: Optional[float] = None
    cx: Optional[float] = None
    cy: Optional[float] = None
    horizontal_fov_deg: float = 60.0
    mount_roll_deg: float = 0.0
    mount_pitch_deg: float = 0.0
    mount_yaw_deg: float = 0.0
    mount_z_m: float = 0.0

    def focal_lengths(self):
        if self.fx and self.fy:
            return float(self.fx), float(self.fy)
        fov = math.radians(self.horizontal_fov_deg)
        fx = float(self.width) / (2.0 * math.tan(fov * 0.5))
        fy = fx
        return fx, fy

    def principal(self):
        cx = float(self.cx) if self.cx is not None else self.width * 0.5
        cy = float(self.cy) if self.cy is not None else self.height * 0.5
        return cx, cy


@dataclass
class GroundOffset:
    east_m: float
    north_m: float
    down_m: float
    range_m: float
    horizontal_m: float
    camera_height_m: float
    calibrated: bool
    reason: str = 'ok'


def _rotz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _roty(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rotx(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _matmul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _matvec(matrix, vector):
    return tuple(sum(matrix[i][j] * vector[j] for j in range(3)) for i in range(3))


# Optical (x right, y down, z forward) -> body NED for a belly nadir camera
# whose image top points to the aircraft nose.
NADIR_CAM_TO_BODY = (
    (0.0, -1.0, 0.0),
    (1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0),
)


def camera_height_m(relative_alt_m, mount_z_m=0.0):
    return float(relative_alt_m) - float(mount_z_m)


def project_pixel_to_ground(
    u,
    v,
    relative_alt_m,
    camera,
    heading_deg=0.0,
    roll_deg=0.0,
    pitch_deg=0.0,
):
    """Intersect one pixel ray with the home-level ground plane."""
    if relative_alt_m is None or not math.isfinite(relative_alt_m):
        return None
    height = camera_height_m(relative_alt_m, camera.mount_z_m)
    if height <= 0.2:
        return GroundOffset(0.0, 0.0, height, abs(height), 0.0, height, False, 'height_too_small')

    fx, fy = camera.focal_lengths()
    cx, cy = camera.principal()
    ray_cam = ((u - cx) / fx, (v - cy) / fy, 1.0)
    norm = math.sqrt(sum(value * value for value in ray_cam))
    ray_cam = tuple(value / norm for value in ray_cam)

    mount = _matmul(
        _matmul(_rotz(camera.mount_yaw_deg), _roty(camera.mount_pitch_deg)),
        _rotx(camera.mount_roll_deg),
    )
    ray_body = _matvec(_matmul(mount, NADIR_CAM_TO_BODY), ray_cam)
    aircraft = _matmul(_matmul(_rotz(heading_deg or 0.0), _roty(pitch_deg or 0.0)), _rotx(roll_deg or 0.0))
    ray_ned = _matvec(aircraft, ray_body)

    down = ray_ned[2]
    if down <= 1.0e-6:
        return GroundOffset(0.0, 0.0, height, float('inf'), float('inf'), height, False, 'ray_not_toward_ground')

    scale = height / down
    north = scale * ray_ned[0]
    east = scale * ray_ned[1]
    horizontal = math.hypot(east, north)
    range_m = math.hypot(horizontal, height)
    calibrated = camera.fx is not None and camera.fy is not None
    return GroundOffset(
        east_m=east,
        north_m=north,
        down_m=height,
        range_m=range_m,
        horizontal_m=horizontal,
        camera_height_m=height,
        calibrated=calibrated,
        reason='ok' if calibrated else 'uncalibrated_fov_model',
    )


def offset_to_latlon(latitude, longitude, east_m, north_m):
    if latitude is None or longitude is None:
        return None, None
    if not all(math.isfinite(value) for value in (latitude, longitude, east_m, north_m)):
        return None, None
    dlat = north_m / 111320.0
    scale = 111320.0 * max(math.cos(math.radians(latitude)), 1.0e-6)
    dlon = east_m / scale
    return latitude + dlat, longitude + dlon


def imu_quat_to_roll_pitch(x, y, z, w):
    """Extract roll/pitch from a ROS ENU IMU quaternion. Yaw is taken from heading."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))
    return roll, pitch
