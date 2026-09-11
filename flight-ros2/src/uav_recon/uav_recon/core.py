"""Pure geometry and fusion helpers for target geolocation."""

from collections import deque
from dataclasses import dataclass, field
import math
import statistics
from typing import Callable, Deque, Generic, List, Optional, Sequence, Tuple, TypeVar


EARTH_A_M = 6378137.0
EARTH_E2 = 6.69437999014e-3
T = TypeVar('T')
EMPTY_TARGET_LABELS = frozenset(('empty', 'blank'))


def is_empty_target_label(label: object) -> bool:
    """Return whether a classifier label represents the empty target class."""
    normalized = str(label).strip().casefold()
    return (
        normalized in EMPTY_TARGET_LABELS
        or normalized.endswith('\u7a7a\u6807\u9776')
    )


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize3(v: Sequence[float]) -> Tuple[float, float, float]:
    length = math.sqrt(sum(x * x for x in v))
    if length <= 1e-12:
        raise ValueError('zero-length vector')
    return tuple(x / length for x in v)


def mat_vec(matrix: Sequence[Sequence[float]], vector: Sequence[float]) -> Tuple[float, float, float]:
    return tuple(sum(matrix[row][col] * vector[col] for col in range(3)) for row in range(3))


def mat_mul(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]):
    return tuple(
        tuple(sum(a[row][k] * b[k][col] for k in range(3)) for col in range(3))
        for row in range(3)
    )


def quat_normalize(q: Sequence[float]) -> Tuple[float, float, float, float]:
    length = math.sqrt(sum(x * x for x in q))
    if length <= 1e-12:
        raise ValueError('zero-length quaternion')
    return tuple(x / length for x in q)


def quat_slerp(a: Sequence[float], b: Sequence[float], ratio: float):
    qa = quat_normalize(a)
    qb = quat_normalize(b)
    dot = sum(x * y for x, y in zip(qa, qb))
    if dot < 0.0:
        qb = tuple(-x for x in qb)
        dot = -dot
    dot = clamp(dot, -1.0, 1.0)
    if dot > 0.9995:
        return quat_normalize(tuple(x + ratio * (y - x) for x, y in zip(qa, qb)))
    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    wa = math.sin((1.0 - ratio) * theta) / sin_theta
    wb = math.sin(ratio * theta) / sin_theta
    return tuple(wa * x + wb * y for x, y in zip(qa, qb))


def quat_to_matrix_xyzw(q: Sequence[float]):
    x, y, z, w = quat_normalize(q)
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def camera_to_body_matrix(forward_tilt_deg: float, left_tilt_deg: float = 0.0):
    """Return camera optical (right/down/forward) to body FLU rotation.

    Positive forward tilt moves the optical axis toward body forward. Positive
    left tilt rotates it toward body left around the body forward axis.
    """
    angle = math.radians(-forward_tilt_deg)
    c, s = math.cos(angle), math.sin(angle)
    pitch_forward = ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
    angle = math.radians(left_tilt_deg)
    c, s = math.cos(angle), math.sin(angle)
    tilt_left = ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))
    optical_down = ((0.0, -1.0, 0.0), (-1.0, 0.0, 0.0), (0.0, 0.0, -1.0))
    return mat_mul(tilt_left, mat_mul(pitch_forward, optical_down))


def undistort_normalized(
    u: float,
    v: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    distortion: Sequence[float],
):
    xd, yd = (u - cx) / fx, (v - cy) / fy
    if not distortion or max(abs(x) for x in distortion) < 1e-12:
        return xd, yd
    values = list(distortion) + [0.0] * (5 - len(distortion))
    k1, k2, p1, p2, k3 = values[:5]
    x, y = xd, yd
    for _ in range(8):
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        if abs(radial) < 1e-9:
            break
        x = (xd - dx) / radial
        y = (yd - dy) / radial
    return x, y


def enu_to_geodetic(latitude_deg: float, longitude_deg: float, east_m: float, north_m: float):
    latitude = math.radians(latitude_deg)
    sin_lat = math.sin(latitude)
    denom = math.sqrt(1.0 - EARTH_E2 * sin_lat * sin_lat)
    radius_east = EARTH_A_M / denom
    radius_north = EARTH_A_M * (1.0 - EARTH_E2) / (denom ** 3)
    lat = latitude_deg + math.degrees(north_m / radius_north)
    lon = longitude_deg + math.degrees(east_m / (radius_east * math.cos(latitude)))
    return lat, lon


def geodetic_delta_m(latitude0: float, longitude0: float, latitude: float, longitude: float):
    latitude_r = math.radians(latitude0)
    sin_lat = math.sin(latitude_r)
    denom = math.sqrt(1.0 - EARTH_E2 * sin_lat * sin_lat)
    radius_east = EARTH_A_M / denom
    radius_north = EARTH_A_M * (1.0 - EARTH_E2) / (denom ** 3)
    east = math.radians(longitude - longitude0) * radius_east * math.cos(latitude_r)
    north = math.radians(latitude - latitude0) * radius_north
    return east, north


@dataclass
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: Sequence[float]
    forward_tilt_deg: float
    offset_flu_m: Sequence[float]
    left_tilt_deg: float = 0.0


@dataclass
class Projection:
    latitude: float
    longitude: float
    altitude_msl_m: float
    east_offset_m: float
    north_offset_m: float
    slant_range_m: float


def project_pixel_to_ground(
    pixel: Sequence[float],
    aircraft_lla: Sequence[float],
    body_to_enu_quaternion: Sequence[float],
    ground_altitude_msl_m: float,
    camera: CameraModel,
) -> Projection:
    x, y = undistort_normalized(
        pixel[0], pixel[1], camera.fx, camera.fy, camera.cx, camera.cy, camera.distortion
    )
    ray_camera = normalize3((x, y, 1.0))
    body_to_enu = quat_to_matrix_xyzw(body_to_enu_quaternion)
    ray_body = mat_vec(
        camera_to_body_matrix(camera.forward_tilt_deg, camera.left_tilt_deg),
        ray_camera,
    )
    ray_enu = normalize3(mat_vec(body_to_enu, ray_body))
    camera_offset_enu = mat_vec(body_to_enu, camera.offset_flu_m)
    camera_altitude = aircraft_lla[2] + camera_offset_enu[2]
    if ray_enu[2] >= -1e-4:
        raise ValueError('camera ray does not intersect the ground below the aircraft')
    distance = (ground_altitude_msl_m - camera_altitude) / ray_enu[2]
    if distance <= 0.0:
        raise ValueError('ground altitude is above the camera ray origin')
    east = camera_offset_enu[0] + distance * ray_enu[0]
    north = camera_offset_enu[1] + distance * ray_enu[1]
    latitude, longitude = enu_to_geodetic(aircraft_lla[0], aircraft_lla[1], east, north)
    return Projection(latitude, longitude, ground_altitude_msl_m, east, north, distance)


@dataclass
class TimedValue(Generic[T]):
    timestamp: float
    value: T


class TimedBuffer(Generic[T]):
    def __init__(self, duration_sec: float = 10.0):
        self.duration_sec = duration_sec
        self.values: Deque[TimedValue[T]] = deque()

    def add(self, timestamp: float, value: T):
        self.values.append(TimedValue(timestamp, value))
        cutoff = timestamp - self.duration_sec
        while self.values and self.values[0].timestamp < cutoff:
            self.values.popleft()

    def interpolate(self, timestamp: float, fn: Callable[[T, T, float], T], max_gap_sec: float) -> Optional[T]:
        if not self.values:
            return None
        before = None
        after = None
        for item in self.values:
            if item.timestamp <= timestamp:
                before = item
            if item.timestamp >= timestamp:
                after = item
                break
        if before and after:
            if timestamp - before.timestamp > max_gap_sec or after.timestamp - timestamp > max_gap_sec:
                return None
            if after.timestamp == before.timestamp:
                return before.value
            ratio = (timestamp - before.timestamp) / (after.timestamp - before.timestamp)
            return fn(before.value, after.value, ratio)
        nearest = before or after
        if nearest and abs(nearest.timestamp - timestamp) <= max_gap_sec:
            return nearest.value
        return None

    def nearest(self, timestamp: float, max_gap_sec: float) -> Optional[T]:
        if not self.values:
            return None
        item = min(self.values, key=lambda sample: abs(sample.timestamp - timestamp))
        return item.value if abs(item.timestamp - timestamp) <= max_gap_sec else None


def lerp_tuple(a: Sequence[float], b: Sequence[float], ratio: float):
    return tuple(x + ratio * (y - x) for x, y in zip(a, b))


@dataclass
class Observation:
    timestamp: float
    latitude: float
    longitude: float
    altitude_msl_m: float
    label: str
    class_id: int
    confidence: float
    pose_score: float
    frame_path: str
    crop_path: str
    horizontal_sigma_m: float = 0.0
    center_distance_norm: float = 0.0


def image_center_weight(
    center_distance_norm: float,
    minimum_weight: float = 0.25,
    power: float = 1.0,
) -> float:
    """Return a bounded weight that decreases toward the image boundary."""
    distance = min(1.0, max(0.0, float(center_distance_norm)))
    floor = min(1.0, max(0.0, float(minimum_weight)))
    exponent = max(0.01, float(power))
    return floor + (1.0 - floor) * ((1.0 - distance) ** exponent)


@dataclass
class Track:
    target_id: str
    observations: List[Observation] = field(default_factory=list)

    def center(self):
        return (
            statistics.median(item.latitude for item in self.observations),
            statistics.median(item.longitude for item in self.observations),
        )

    def add(self, observation: Observation):
        self.observations.append(observation)

    def fuse(
        self,
        single_observation_sigma_m: float,
        center_weight_minimum: float = 0.25,
        center_weight_power: float = 1.0,
    ):
        lat0, lon0 = self.center()
        points = []
        for item in self.observations:
            east, north = geodetic_delta_m(lat0, lon0, item.latitude, item.longitude)
            points.append((item, east, north))
        median_east = statistics.median(point[1] for point in points)
        median_north = statistics.median(point[2] for point in points)
        residuals = [math.hypot(point[1] - median_east, point[2] - median_north) for point in points]
        median_residual = statistics.median(residuals)
        mad = statistics.median(abs(value - median_residual) for value in residuals)
        cutoff = max(0.35, median_residual + 3.5 * max(mad, 0.05))
        kept = [point for point, residual in zip(points, residuals) if residual <= cutoff]
        if not kept:
            kept = points
        center_weights = [
            image_center_weight(
                item.center_distance_norm,
                center_weight_minimum,
                center_weight_power,
            )
            for item, _, _ in kept
        ]
        weights = [
            max(0.01, item.confidence * item.pose_score * center_weight)
            for (item, _, _), center_weight in zip(kept, center_weights)
        ]
        total_weight = sum(weights)
        east = sum(weight * point[1] for weight, point in zip(weights, kept)) / total_weight
        north = sum(weight * point[2] for weight, point in zip(weights, kept)) / total_weight
        latitude, longitude = enu_to_geodetic(lat0, lon0, east, north)
        scatter_sq = sum(
            weight * ((point[1] - east) ** 2 + (point[2] - north) ** 2)
            for weight, point in zip(weights, kept)
        ) / total_weight
        input_sigma_sq = sum(
            weight * max(single_observation_sigma_m, point[0].horizontal_sigma_m) ** 2
            for weight, point in zip(weights, kept)
        ) / total_weight
        radius95 = 2.45 * math.sqrt(scatter_sq + input_sigma_sq)
        votes = {}
        for item, _, _ in kept:
            votes[item.label] = votes.get(item.label, 0.0) + max(0.01, item.confidence * item.pose_score)
        label = max(votes, key=votes.get)
        consensus = votes[label] / sum(votes.values())
        label_items = [point[0] for point in kept if point[0].label == label]
        best = max(label_items, key=lambda item: item.confidence * item.pose_score)
        return {
            'latitude': latitude,
            'longitude': longitude,
            'altitude_msl_m': statistics.median(item.altitude_msl_m for item, _, _ in kept),
            'horizontal_radius_95_m': radius95,
            'label': label,
            'class_id': best.class_id,
            'confidence': max(item.confidence for item in label_items),
            'label_consensus': consensus,
            'observation_count': len(kept),
            'observation_span_sec': max(item.timestamp for item, _, _ in kept) - min(item.timestamp for item, _, _ in kept),
            'mean_center_weight': sum(center_weights) / len(center_weights),
            'best': best,
        }


class TrackManager:
    def __init__(self, association_radius_m: float):
        self.association_radius_m = association_radius_m
        self.tracks: List[Track] = []

    def add(self, observation: Observation) -> Track:
        closest = None
        closest_distance = float('inf')
        for track in self.tracks:
            lat, lon = track.center()
            east, north = geodetic_delta_m(lat, lon, observation.latitude, observation.longitude)
            distance = math.hypot(east, north)
            if distance < closest_distance:
                closest, closest_distance = track, distance
        if closest is None or closest_distance > self.association_radius_m:
            closest = Track(f'target_{len(self.tracks) + 1:03d}')
            self.tracks.append(closest)
        closest.add(observation)
        return closest
