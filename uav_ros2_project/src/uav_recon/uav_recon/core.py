"""Pure geometry and fusion helpers for target geolocation."""

from collections import deque
from dataclasses import dataclass, field
import math
import statistics
from typing import Callable, Deque, Dict, Generic, List, Optional, Sequence, Tuple, TypeVar


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


def propagate_geodetic_with_local_delta(
    anchor_lla: Sequence[float],
    anchor_local_enu_m: Sequence[float],
    frame_local_enu_m: Sequence[float],
):
    """Propagate a global RTK anchor using an EKF-local ENU displacement."""
    east = frame_local_enu_m[0] - anchor_local_enu_m[0]
    north = frame_local_enu_m[1] - anchor_local_enu_m[1]
    up = frame_local_enu_m[2] - anchor_local_enu_m[2]
    latitude, longitude = enu_to_geodetic(anchor_lla[0], anchor_lla[1], east, north)
    return latitude, longitude, anchor_lla[2] + up


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

    def bracket(self, timestamp: float):
        """Return samples around timestamp, scanning from the newest sample."""
        before = None
        after = None
        for item in reversed(self.values):
            if item.timestamp == timestamp:
                return item, item
            if item.timestamp > timestamp:
                after = item
                continue
            before = item
            break
        return before, after

    def interpolate(
        self,
        timestamp: float,
        fn: Callable[[T, T, float], T],
        max_gap_sec: float,
    ) -> Optional[T]:
        before, after = self.bracket(timestamp)
        if before and after:
            if (
                timestamp - before.timestamp > max_gap_sec
                or after.timestamp - timestamp > max_gap_sec
            ):
                return None
            if after.timestamp == before.timestamp:
                return before.value
            ratio = (timestamp - before.timestamp) / (after.timestamp - before.timestamp)
            return fn(before.value, after.value, ratio)
        nearest = before or after
        if nearest and abs(nearest.timestamp - timestamp) <= max_gap_sec:
            return nearest.value
        return None

    def interpolate_bracketed(
        self,
        timestamp: float,
        fn: Callable[[T, T, float], T],
        max_gap_sec: float,
    ) -> Optional[T]:
        """Interpolate only when samples exist on both sides of the timestamp."""
        before, after = self.bracket(timestamp)
        if before is None or after is None:
            return None
        if timestamp - before.timestamp > max_gap_sec or after.timestamp - timestamp > max_gap_sec:
            return None
        if after.timestamp == before.timestamp:
            return before.value
        ratio = (timestamp - before.timestamp) / (after.timestamp - before.timestamp)
        return fn(before.value, after.value, ratio)

    def has_sample_at_or_after(self, timestamp: float) -> bool:
        return bool(self.values and self.values[-1].timestamp >= timestamp)

    def latest_at_or_before(self, timestamp: float, max_age_sec: float) -> Optional[TimedValue[T]]:
        for item in reversed(self.values):
            if item.timestamp <= timestamp:
                return item if timestamp - item.timestamp <= max_age_sec else None
        return None

    def nearest_sample(self, timestamp: float, max_gap_sec: float) -> Optional[TimedValue[T]]:
        before, after = self.bracket(timestamp)
        candidates = [item for item in (before, after) if item is not None]
        if not candidates:
            return None
        item = min(candidates, key=lambda sample: abs(sample.timestamp - timestamp))
        return item if abs(item.timestamp - timestamp) <= max_gap_sec else None

    def nearest(self, timestamp: float, max_gap_sec: float) -> Optional[T]:
        item = self.nearest_sample(timestamp, max_gap_sec)
        return None if item is None else item.value


def lerp_tuple(a: Sequence[float], b: Sequence[float], ratio: float):
    return tuple(x + ratio * (y - x) for x, y in zip(a, b))


def hermite_tuple(
    position0: Sequence[float],
    velocity0: Sequence[float],
    position1: Sequence[float],
    velocity1: Sequence[float],
    ratio: float,
    duration_sec: float,
):
    """Interpolate position using endpoint positions and velocities."""
    t = clamp(ratio, 0.0, 1.0)
    h00 = 2.0 * t ** 3 - 3.0 * t ** 2 + 1.0
    h10 = t ** 3 - 2.0 * t ** 2 + t
    h01 = -2.0 * t ** 3 + 3.0 * t ** 2
    h11 = t ** 3 - t ** 2
    return tuple(
        h00 * p0 + h10 * duration_sec * v0 + h01 * p1 + h11 * duration_sec * v1
        for p0, v0, p1, v1 in zip(position0, velocity0, position1, velocity1)
    )


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
    position_source: str = 'global_interpolation'
    global_anchor_age_sec: float = 0.0
    local_position_method: str = 'none'
    local_delta_enu_m: Sequence[float] = field(default_factory=lambda: (0.0, 0.0, 0.0))
    frame_number: int = -1
    source_sequence: int = 0
    center_px: Sequence[float] = field(default_factory=lambda: (0.0, 0.0))


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
        label_counts = {}
        label_scores = {}
        best_by_label = {}
        for item, _, _ in kept:
            label_counts[item.label] = label_counts.get(item.label, 0) + 1
            label_scores[item.label] = label_scores.get(item.label, 0.0) + max(
                0.01,
                item.confidence * item.pose_score,
            )
            current_best = best_by_label.get(item.label)
            if current_best is None or (
                item.confidence
                * item.pose_score
                * image_center_weight(
                    item.center_distance_norm,
                    center_weight_minimum,
                    center_weight_power,
                )
                > current_best.confidence
                * current_best.pose_score
                * image_center_weight(
                    current_best.center_distance_norm,
                    center_weight_minimum,
                    center_weight_power,
                )
            ):
                best_by_label[item.label] = item
        label = max(
            label_counts,
            key=lambda item: (label_counts[item], label_scores[item], item),
        )
        consensus = label_counts[label] / sum(label_counts.values())
        label_items = [point[0] for point in kept if point[0].label == label]
        best = best_by_label[label]
        return {
            'latitude': latitude,
            'longitude': longitude,
            'altitude_msl_m': statistics.median(item.altitude_msl_m for item, _, _ in kept),
            'horizontal_radius_95_m': radius95,
            'label': label,
            'class_id': best.class_id,
            'confidence': max(item.confidence for item in label_items),
            'label_consensus': consensus,
            'label_counts': label_counts,
            'label_scores': label_scores,
            'best_by_label': best_by_label,
            'observation_count': len(kept),
            'raw_observation_count': len(points),
            'rejected_observation_count': len(points) - len(kept),
            'observation_span_sec': max(item.timestamp for item, _, _ in kept) - min(item.timestamp for item, _, _ in kept),
            'observation_start_timestamp': min(item.timestamp for item, _, _ in kept),
            'observation_end_timestamp': max(item.timestamp for item, _, _ in kept),
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


@dataclass
class FramePacket:
    packet_id: str
    label: str
    observations: List[Observation] = field(default_factory=list)
    last_timestamp: float = 0.0
    last_center_px: Sequence[float] = field(default_factory=lambda: (0.0, 0.0))
    last_frame_number: int = -1

    def add(self, observation: Observation):
        self.observations.append(observation)
        self.last_timestamp = observation.timestamp
        self.last_center_px = tuple(float(value) for value in observation.center_px[:2])
        self.last_frame_number = observation.frame_number

    def fuse(
        self,
        single_observation_sigma_m: float,
        center_weight_minimum: float = 0.25,
        center_weight_power: float = 1.0,
    ):
        fused = Track(self.packet_id, list(self.observations)).fuse(
            single_observation_sigma_m,
            center_weight_minimum,
            center_weight_power,
        )
        fused['packet_ids'] = [self.packet_id]
        fused['packet_count'] = 1
        return fused


@dataclass
class FramePacketGroup:
    label: str
    packets: List[FramePacket]
    closure_reason: str = 'gap_timeout'


class PixelFramePacketManager:
    """Build pixel-continuous tracks before label voting or geolocation fusion."""

    def __init__(
        self,
        gap_timeout_sec: float = 0.20,
        pixel_gate_base_px: float = 50.0,
        pixel_gate_rate_px_per_sec: float = 1300.0,
        pixel_gate_max_px: float = 200.0,
        max_observations: int = 300,
        association_gap_sec: Optional[float] = None,
    ):
        self.gap_timeout_sec = max(0.01, float(gap_timeout_sec))
        if association_gap_sec is None:
            association_gap_sec = self.gap_timeout_sec
        self.association_gap_sec = min(
            self.gap_timeout_sec,
            max(0.01, float(association_gap_sec)),
        )
        self.pixel_gate_base_px = max(0.0, float(pixel_gate_base_px))
        self.pixel_gate_rate_px_per_sec = max(0.0, float(pixel_gate_rate_px_per_sec))
        self.pixel_gate_max_px = max(1.0, float(pixel_gate_max_px))
        self.max_observations = max(1, int(max_observations))
        self.active: List[FramePacket] = []
        self.pending: List[FramePacket] = []
        self.last_observation_timestamp: Optional[float] = None
        self.packet_counter = 0
        self.last_assignments: List[dict] = []

    def pixel_gate(self, delta_sec: float) -> float:
        return min(
            self.pixel_gate_max_px,
            self.pixel_gate_base_px
            + self.pixel_gate_rate_px_per_sec * max(0.0, float(delta_sec)),
        )

    @staticmethod
    def _pixel_distance(observation: Observation, packet: FramePacket) -> float:
        return math.hypot(
            float(observation.center_px[0]) - float(packet.last_center_px[0]),
            float(observation.center_px[1]) - float(packet.last_center_px[1]),
        )

    def _new_packet(self, observation: Observation) -> FramePacket:
        self.packet_counter += 1
        packet = FramePacket(
            f'packet_{self.packet_counter:04d}',
            str(observation.label),
        )
        self.active.append(packet)
        return packet

    def _new_packet_reason(self, observation: Observation, candidate_packet_ids):
        if not self.active:
            return 'no_active_packet', {}

        available = [
            packet
            for packet in self.active
            if len(packet.observations) < self.max_observations
        ]
        if not available:
            return 'max_observations', {}

        time_eligible = []
        same_frame = []
        for packet in available:
            delta_sec = observation.timestamp - packet.last_timestamp
            if delta_sec < 0.0 or delta_sec > self.association_gap_sec:
                continue
            if (
                observation.frame_number >= 0
                and packet.last_frame_number == observation.frame_number
            ):
                same_frame.append(packet)
                continue
            distance_px = self._pixel_distance(observation, packet)
            gate_px = self.pixel_gate(delta_sec)
            time_eligible.append((distance_px - gate_px, distance_px, gate_px, delta_sec, packet))

        if time_eligible:
            excess_px, distance_px, gate_px, delta_sec, packet = min(
                time_eligible,
                key=lambda item: (item[0], item[1]),
            )
            if packet.packet_id in candidate_packet_ids:
                reason = 'one_to_one_conflict'
            else:
                reason = 'pixel_gate_exceeded'
            return reason, {
                'previous_packet_id': packet.packet_id,
                'delta_sec': delta_sec,
                'distance_px': distance_px,
                'gate_px': gate_px,
                'excess_px': excess_px,
            }
        if same_frame:
            return 'same_frame', {
                'previous_packet_ids': [packet.packet_id for packet in same_frame],
            }

        nearest = min(
            available,
            key=lambda packet: abs(observation.timestamp - packet.last_timestamp),
        )
        return 'association_gap_exceeded', {
            'previous_packet_id': nearest.packet_id,
            'delta_sec': observation.timestamp - nearest.last_timestamp,
            'association_gap_sec': self.association_gap_sec,
        }

    def add_frame(self, observations: Sequence[Observation]) -> List[FramePacket]:
        """Assign one frame's detections to active packets one-to-one.

        Matching is label agnostic. Global greedy assignment over normalized
        pixel residuals prevents detection iteration order from attaching two
        same-frame targets to one packet or stealing the nearest track.
        """
        observations = list(observations)
        for observation in observations:
            if len(observation.center_px) < 2:
                raise ValueError('pixel packet observation requires center_px')
        if not observations:
            self.last_assignments = []
            return []

        edges = []
        candidate_packet_ids = [set() for _ in observations]
        for observation_index, observation in enumerate(observations):
            for packet_index, packet in enumerate(self.active):
                if len(packet.observations) >= self.max_observations:
                    continue
                delta_sec = observation.timestamp - packet.last_timestamp
                if delta_sec < 0.0 or delta_sec > self.association_gap_sec:
                    continue
                if (
                    observation.frame_number >= 0
                    and packet.last_frame_number == observation.frame_number
                ):
                    continue
                distance_px = self._pixel_distance(observation, packet)
                gate_px = self.pixel_gate(delta_sec)
                if distance_px <= gate_px:
                    candidate_packet_ids[observation_index].add(packet.packet_id)
                    edges.append((
                        distance_px / gate_px,
                        distance_px,
                        observation_index,
                        packet_index,
                        delta_sec,
                        gate_px,
                    ))

        matches = {}
        used_packets = set()
        for normalized, distance_px, observation_index, packet_index, delta_sec, gate_px in sorted(edges):
            if observation_index in matches or packet_index in used_packets:
                continue
            matches[observation_index] = (
                packet_index,
                normalized,
                distance_px,
                delta_sec,
                gate_px,
            )
            used_packets.add(packet_index)

        assignments = []
        details = []
        for observation_index, observation in enumerate(observations):
            match = matches.get(observation_index)
            if match is None:
                reason, detail = self._new_packet_reason(
                    observation,
                    candidate_packet_ids[observation_index],
                )
                packet = self._new_packet(observation)
                details.append({
                    'action': 'new_packet',
                    'packet_id': packet.packet_id,
                    'reason': reason,
                    **detail,
                })
            else:
                packet_index, normalized, distance_px, delta_sec, gate_px = match
                packet = self.active[packet_index]
                details.append({
                    'action': 'matched',
                    'packet_id': packet.packet_id,
                    'delta_sec': delta_sec,
                    'distance_px': distance_px,
                    'gate_px': gate_px,
                    'normalized_distance': normalized,
                })
            packet.add(observation)
            assignments.append(packet)

        self.last_assignments = details
        self.last_observation_timestamp = max(
            observation.timestamp for observation in observations
        )
        return assignments

    def add(self, observation: Observation) -> FramePacket:
        return self.add_frame([observation])[0]

    def advance(self, timestamp: float) -> List[FramePacketGroup]:
        still_active = []
        for packet in self.active:
            if timestamp - packet.last_timestamp > self.gap_timeout_sec:
                self.pending.append(packet)
            else:
                still_active.append(packet)
        self.active = still_active

        if (
            self.pending
            and not self.active
            and self.last_observation_timestamp is not None
            and timestamp - self.last_observation_timestamp > self.gap_timeout_sec
        ):
            return self._take_group('gap_timeout')
        return []

    def take_limit_reached_group(self) -> List[FramePacketGroup]:
        if not any(
            len(packet.observations) >= self.max_observations
            for packet in self.active
        ):
            return []
        self.pending.extend(self.active)
        self.active = []
        return self._take_group('max_observations')

    def _take_group(self, closure_reason: str) -> List[FramePacketGroup]:
        packets = self.pending
        self.pending = []
        self.last_observation_timestamp = None
        labels = {
            observation.label
            for packet in packets
            for observation in packet.observations
        }
        label = next(iter(labels)) if len(labels) == 1 else 'mixed'
        return [FramePacketGroup(label, packets, closure_reason)]

    @property
    def active_packet_count(self) -> int:
        return len(self.active)

    @property
    def pending_packet_count(self) -> int:
        return len(self.pending)


def _merge_packet_candidates(left, right):
    left_count = int(left['observation_count'])
    right_count = int(right['observation_count'])
    total_count = left_count + right_count
    east, north = geodetic_delta_m(
        left['latitude'],
        left['longitude'],
        right['latitude'],
        right['longitude'],
    )
    right_ratio = right_count / total_count
    latitude, longitude = enu_to_geodetic(
        left['latitude'],
        left['longitude'],
        east * right_ratio,
        north * right_ratio,
    )
    left_distance = math.hypot(east * right_ratio, north * right_ratio)
    right_distance = math.hypot(east * (1.0 - right_ratio), north * (1.0 - right_ratio))
    left_sigma = float(left['horizontal_radius_95_m']) / 2.45
    right_sigma = float(right['horizontal_radius_95_m']) / 2.45
    variance = (
        left_count * (left_sigma ** 2 + left_distance ** 2)
        + right_count * (right_sigma ** 2 + right_distance ** 2)
    ) / total_count
    label_counts = dict(left.get('label_counts', {left['label']: left_count}))
    for label, count in right.get(
        'label_counts',
        {right['label']: right_count},
    ).items():
        label_counts[label] = label_counts.get(label, 0) + int(count)
    label_scores = dict(
        left.get('label_scores', {left['label']: float(left['confidence'])})
    )
    for label, score in right.get(
        'label_scores',
        {right['label']: float(right['confidence'])},
    ).items():
        label_scores[label] = label_scores.get(label, 0.0) + float(score)
    best_by_label = dict(left.get('best_by_label', {left['label']: left['best']}))
    for label, observation in right.get(
        'best_by_label',
        {right['label']: right['best']},
    ).items():
        current = best_by_label.get(label)
        if current is None or (
            observation.confidence * observation.pose_score
            > current.confidence * current.pose_score
        ):
            best_by_label[label] = observation
    label = max(
        label_counts,
        key=lambda item: (label_counts[item], label_scores[item], item),
    )
    best = best_by_label[label]
    return {
        **left,
        'latitude': latitude,
        'longitude': longitude,
        'altitude_msl_m': (
            left_count * float(left['altitude_msl_m'])
            + right_count * float(right['altitude_msl_m'])
        ) / total_count,
        'horizontal_radius_95_m': 2.45 * math.sqrt(variance),
        'label': label,
        'class_id': best.class_id,
        'confidence': best.confidence,
        'label_consensus': label_counts[label] / sum(label_counts.values()),
        'label_counts': label_counts,
        'label_scores': label_scores,
        'best_by_label': best_by_label,
        'observation_count': total_count,
        'raw_observation_count': (
            int(left.get('raw_observation_count', left_count))
            + int(right.get('raw_observation_count', right_count))
        ),
        'rejected_observation_count': (
            int(left.get('rejected_observation_count', 0))
            + int(right.get('rejected_observation_count', 0))
        ),
        'observation_start_timestamp': min(
            float(left['observation_start_timestamp']),
            float(right['observation_start_timestamp']),
        ),
        'observation_end_timestamp': max(
            float(left['observation_end_timestamp']),
            float(right['observation_end_timestamp']),
        ),
        'observation_span_sec': max(
            float(left['observation_end_timestamp']),
            float(right['observation_end_timestamp']),
        ) - min(
            float(left['observation_start_timestamp']),
            float(right['observation_start_timestamp']),
        ),
        'mean_center_weight': (
            left_count * float(left['mean_center_weight'])
            + right_count * float(right['mean_center_weight'])
        ) / total_count,
        'best': best,
        'packet_ids': list(left.get('packet_ids', [])) + list(right.get('packet_ids', [])),
        'packet_count': int(left.get('packet_count', 1)) + int(right.get('packet_count', 1)),
    }


def resolve_packet_candidates(
    candidates: Sequence[dict],
    merge_distance_m: float = 1.5,
    distinct_distance_m: float = 10.0,
):
    """Merge, reject, or separate finalized packet coordinates."""
    merge_distance_m = max(0.0, float(merge_distance_m))
    distinct_distance_m = max(merge_distance_m, float(distinct_distance_m))
    ordered = sorted(
        (dict(candidate) for candidate in candidates),
        key=lambda item: (
            int(item['observation_count']),
            -float(item['horizontal_radius_95_m']),
            float(item['confidence']),
        ),
        reverse=True,
    )
    winners = []
    decisions = []
    for candidate in ordered:
        distances = []
        for index, winner in enumerate(winners):
            east, north = geodetic_delta_m(
                winner['latitude'],
                winner['longitude'],
                candidate['latitude'],
                candidate['longitude'],
            )
            distances.append((math.hypot(east, north), index))

        merge_matches = [item for item in distances if item[0] <= merge_distance_m]
        if merge_matches:
            distance, index = min(merge_matches)
            previous_ids = list(winners[index].get('packet_ids', []))
            candidate_ids = list(candidate.get('packet_ids', []))
            winners[index] = _merge_packet_candidates(winners[index], candidate)
            decisions.append({
                'action': 'merge',
                'distance_m': distance,
                'kept_packet_ids': previous_ids,
                'merged_packet_ids': candidate_ids,
            })
            continue

        conflicts = [item for item in distances if item[0] < distinct_distance_m]
        if conflicts:
            distance, index = min(conflicts)
            decisions.append({
                'action': 'discard_smaller_packet',
                'distance_m': distance,
                'kept_packet_ids': list(winners[index].get('packet_ids', [])),
                'discarded_packet_ids': list(candidate.get('packet_ids', [])),
            })
            continue

        winners.append(candidate)
        decisions.append({
            'action': 'keep_distinct',
            'packet_ids': list(candidate.get('packet_ids', [])),
        })
    return winners, decisions
