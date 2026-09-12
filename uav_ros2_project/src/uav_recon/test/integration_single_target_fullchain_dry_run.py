#!/usr/bin/env python3
import json
import math
import os
from pathlib import Path
import tempfile
import time

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import GPSRAW, State
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String

from uav_fcu_interface.fcu_interface_mavros_node import FcuInterfaceMavrosNode
from uav_interfaces.msg import ReconTarget, TargetCommand
from uav_mission_manager.mission_manager_node import MissionManagerNode, MissionState
from uav_recon.recon_node import ReconGeolocatorNode
from uav_vision_bridge.vision_target_bridge_node import VisionTargetBridge


AIRCRAFT_LAT = 22.88190000
AIRCRAFT_LON = 113.48830000
AIRCRAFT_ALT_MSL_M = 35.0
TARGET_HEADING_DEG = 90.0


class IntegrationSource(Node):
    def __init__(self):
        super().__init__('single_target_manifest_source')
        self.position_pub = self.create_publisher(
            NavSatFix, '/mavros/global_position/global', 10
        )
        self.pose_pub = self.create_publisher(
            PoseStamped, '/mavros/local_position/pose', 10
        )
        self.gps_pub = self.create_publisher(
            GPSRAW, '/mavros/gpsstatus/gps1/raw', 10
        )

    def publish_telemetry(self):
        stamp = self.get_clock().now().to_msg()

        position = NavSatFix()
        position.header.stamp = stamp
        position.status.status = NavSatStatus.STATUS_FIX
        position.latitude = AIRCRAFT_LAT
        position.longitude = AIRCRAFT_LON
        position.altitude = AIRCRAFT_ALT_MSL_M

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.pose.orientation.w = 1.0

        gps = GPSRAW()
        gps.header.stamp = stamp
        gps.fix_type = GPSRAW.GPS_FIX_TYPE_RTK_FIXED
        gps.satellites_visible = 20
        gps.lat = int(round(AIRCRAFT_LAT * 1.0e7))
        gps.lon = int(round(AIRCRAFT_LON * 1.0e7))
        gps.h_acc = 100

        self.position_pub.publish(position)
        self.pose_pub.publish(pose)
        self.gps_pub.publish(gps)
        return stamp.sec * 1_000_000_000 + stamp.nanosec


class IntegrationProbe(Node):
    def __init__(self):
        super().__init__('single_target_manifest_probe')
        self.recon_results = []
        self.target_commands = []
        self.mission_states = []
        self.summary_events = []
        self.create_subscription(
            ReconTarget,
            '/vision/recon_result',
            self.recon_results.append,
            10,
        )
        self.create_subscription(
            TargetCommand,
            '/vision/target_command',
            self.target_commands.append,
            10,
        )

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            '/mission/safety_state',
            lambda msg: self.mission_states.append(msg.data),
            state_qos,
        )
        self.create_subscription(
            String,
            '/fcu/mission_summary_event',
            lambda msg: self.summary_events.append(json.loads(msg.data)),
            10,
        )


def distance_m(lat1, lon1, lat2, lon2):
    radius_m = 6378137.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = (
        math.sin(d_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2.0) ** 2
    )
    return radius_m * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def make_manager_ready(manager):
    now = time.monotonic()
    with manager._lock:
        manager.fcu_connected = True
        manager.fcu_armed = True
        manager.fcu_mode = 'AUTO'
        manager.fcu_system_status = 3
        manager.actual_system_id = manager.expected_system_id
        manager.actual_component_id = manager.expected_component_id
        manager.last_fcu_state_time = now
        manager.last_heartbeat_time = now
        manager.heartbeat_count = manager.heartbeat_required_count
        manager.heartbeat_sequence_start = (
            now - manager.heartbeat_stable_duration_sec
        )
        manager.last_vehicle_info_time = now
        manager.last_position_time = now
        manager.position_healthy = True
        manager.last_gps_time = now
        manager.gps_healthy = True
        manager.last_ekf_time = now
        manager.ekf_healthy = True
        manager.last_sensor_time = now
        manager.sensor_health = True
        if not manager.transition_to(MissionState.STANDBY, 'isolated_mock_ready'):
            raise RuntimeError('mission manager could not enter STANDBY')


def make_fcu_ready(fcu):
    if not fcu.dry_run_goto or fcu.allow_mission_upload:
        raise RuntimeError('integration test requires dry-run with uploads disabled')
    state = State()
    state.connected = True
    state.armed = True
    state.mode = 'AUTO'
    fcu.current_state = state

    gps = NavSatFix()
    gps.status.status = NavSatStatus.STATUS_FIX
    gps.latitude = AIRCRAFT_LAT
    gps.longitude = AIRCRAFT_LON
    gps.altitude = AIRCRAFT_ALT_MSL_M
    fcu.current_gps = gps


def spin_for(executor, duration_sec):
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)


def main():
    with tempfile.TemporaryDirectory(prefix='single_target_fullchain_') as directory:
        root = Path(directory)
        crop_root = root / 'crops'
        output_root = root / 'output'
        crop_root.mkdir()
        frame_path = root / 'frame.jpg'
        crop_path = crop_root / 'crop_00.jpg'
        manifest_path = crop_root / 'manifest.json'
        frame_path.write_bytes(b'\xff\xd8\xff\xd9')
        crop_path.write_bytes(b'\xff\xd8\xff\xd9')

        rclpy.init(args=[
            '--ros-args',
            '-p', f'manifest_path:={manifest_path}',
            '-p', f'frame_source_path:={frame_path}',
            '-p', f'output_root:={output_root}',
            '-p', 'manifest_poll_hz:=120.0',
            '-p', 'calibration_valid:=true',
            '-p', 'calibration_width:=1440',
            '-p', 'calibration_height:=1080',
            '-p', 'fx:=1828.0308235733144',
            '-p', 'fy:=1827.57772607314',
            '-p', 'cx:=769.3836267628353',
            '-p', 'cy:=543.4421171826395',
            '-p', 'camera_forward_tilt_deg:=20.0',
            '-p', 'camera_offset_flu_m:=[0.418,0.0,-0.09]',
            '-p', 'ground_altitude_mode:=fixed_msl',
            '-p', 'fixed_ground_altitude_msl_m:=0.0',
            '-p', 'minimum_observations:=5',
            '-p', 'minimum_observation_span_sec:=0.20',
            '-p', 'max_horizontal_radius_95_m:=6.0',
        ])
        executor = MultiThreadedExecutor(num_threads=4)
        nodes = []
        try:
            fcu = FcuInterfaceMavrosNode()
            make_fcu_ready(fcu)
            manager = MissionManagerNode()
            recon = ReconGeolocatorNode()
            bridge = VisionTargetBridge(
                parameter_overrides=[
                    Parameter(
                        'heading_deg', Parameter.Type.DOUBLE, TARGET_HEADING_DEG
                    ),
                    Parameter('auto_execute', Parameter.Type.BOOL, True),
                    Parameter(
                        'automation_step_timeout_sec', Parameter.Type.DOUBLE, 4.0
                    ),
                    Parameter(
                        'automation_total_timeout_sec', Parameter.Type.DOUBLE, 8.0
                    ),
                ]
            )
            source = IntegrationSource()
            probe = IntegrationProbe()
            nodes = [fcu, manager, recon, bridge, source, probe]
            for node in nodes:
                executor.add_node(node)

            make_manager_ready(manager)
            spin_for(executor, 0.5)
            if not manager.goto_client.service_is_ready():
                raise RuntimeError('isolated /fcu/goto_global service was not discovered')
            if bridge.mission_state != 'STANDBY':
                raise RuntimeError('bridge did not receive the latched STANDBY state')

            for sequence in range(7):
                capture_ns = source.publish_telemetry()
                spin_for(executor, 0.05)
                manifest = {
                    'frame': sequence,
                    'capture_timestamp_unix_ns': capture_ns,
                    'capture_monotonic_ns': time.monotonic_ns(),
                    'capture_clock_source': 'isolated_test',
                    'source_sequence': sequence,
                    'frame_width': 1440,
                    'frame_height': 1080,
                    'mode': 'digit',
                    'count': 1,
                    'crops': [{
                        'rank': 0,
                        'detection_index': 0,
                        'src': 'crop_00.jpg',
                        'center': [769.3836267628353, 543.4421171826395],
                        'class_id': 85,
                        'class_label': '85',
                        'class_prob': 0.99,
                        'pose_score': 0.98,
                    }],
                }
                manifest_path.write_text(json.dumps(manifest), encoding='utf-8')
                spin_for(executor, 0.08)

            deadline = time.monotonic() + 4.0
            dry_run_event = None
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
                dry_run_event = next(
                    (
                        event
                        for event in probe.summary_events
                        if event.get('event') == 'composite_mission_dry_run'
                    ),
                    None,
                )
                if dry_run_event is not None:
                    break

            confirmed = [result for result in probe.recon_results if result.valid]
            if not confirmed:
                raise RuntimeError('geolocator produced no confirmed ReconTarget')
            if len(probe.target_commands) != 1:
                raise RuntimeError(
                    f'expected one TargetCommand, got {len(probe.target_commands)}'
                )
            if dry_run_event is None:
                raise RuntimeError('FCU dry-run route planning event was not produced')

            recon_result = confirmed[0]
            command = probe.target_commands[0]
            if abs(command.latitude - recon_result.latitude) > 1.0e-10:
                raise RuntimeError('latitude changed while crossing the bridge')
            if abs(command.longitude - recon_result.longitude) > 1.0e-10:
                raise RuntimeError('longitude changed while crossing the bridge')
            if abs(dry_run_event['target_lat'] - command.latitude) > 1.0e-10:
                raise RuntimeError('FCU planner did not receive bridge latitude')
            if abs(dry_run_event['target_lon'] - command.longitude) > 1.0e-10:
                raise RuntimeError('FCU planner did not receive bridge longitude')

            points = dry_run_event['points']
            distances = {
                name: round(
                    distance_m(
                        command.latitude,
                        command.longitude,
                        point['lat'],
                        point['lon'],
                    ),
                    3,
                )
                for name, point in points.items()
                if name in {'A', 'B', 'R', 'C', 'D'}
            }
            output = {
                'safe_isolation': {
                    'ros_domain_id': int(os.environ.get('ROS_DOMAIN_ID', '-1')),
                    'dry_run_goto': fcu.dry_run_goto,
                    'allow_mission_upload': fcu.allow_mission_upload,
                    'real_mavros_write_attempted': False,
                },
                'vision_manifest': {
                    'frames_injected': 7,
                    'label': '85',
                    'pose_score': 0.98,
                    'class_prob': 0.99,
                    'pixel_center': [769.3836267628353, 543.4421171826395],
                },
                'recon_geolocator': {
                    'confirmed_count': len(confirmed),
                    'target_id': recon_result.target_id,
                    'status': recon_result.status,
                    'observation_count': recon_result.observation_count,
                    'latitude': recon_result.latitude,
                    'longitude': recon_result.longitude,
                    'horizontal_radius_95_m': recon_result.horizontal_radius_95_m,
                },
                'bridge': {
                    'published_count': len(probe.target_commands),
                    'latitude': command.latitude,
                    'longitude': command.longitude,
                    'heading_deg': command.heading_deg,
                    'phase': bridge.automation_phase,
                },
                'mission_manager': {
                    'state': manager.state.value,
                    'attack_executed_latched': manager.attack_executed,
                    'active_target': dict(manager.active_target),
                },
                'route_rewrite_dry_run': {
                    'event': dry_run_event['event'],
                    'target_lat': dry_run_event['target_lat'],
                    'target_lon': dry_run_event['target_lon'],
                    'heading_deg': dry_run_event['heading_deg'],
                    'release_command_enabled': dry_run_event[
                        'release_command_enabled'
                    ],
                    'points': points,
                    'distance_from_target_m': distances,
                    'upload_attempted': dry_run_event['upload_attempted'],
                },
            }
            print(
                'SAFE_FULLCHAIN_RESULT='
                + json.dumps(output, ensure_ascii=False, sort_keys=True)
            )
        finally:
            for node in reversed(nodes):
                executor.remove_node(node)
                node.destroy_node()
            executor.shutdown()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
