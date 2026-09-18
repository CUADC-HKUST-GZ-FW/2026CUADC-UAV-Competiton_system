#!/usr/bin/env python3
"""Exercise competition selection through an isolated mission dry-run."""

import json
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import time

from sensor_msgs.msg import NavSatFix, NavSatStatus
from mavros_msgs.msg import State
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from uav_fcu_interface.fcu_interface_mavros_node import FcuInterfaceMavrosNode
from uav_interfaces.msg import ReconTarget, TargetCommand
from uav_mission_manager.mission_manager_node import (
    MissionManagerNode,
    MissionState,
)
from uav_vision_bridge.vision_target_bridge_node import VisionTargetBridge


def find_runtime_scripts():
    candidates = []
    configured_root = os.environ.get('YOUTH_RUNTIME_ROOT')
    if configured_root:
        candidates.append(Path(configured_root) / 'scripts')
    candidates.extend(
        [
            Path.home() / 'youth-vision-runtime' / 'scripts',
            Path(__file__).resolve().parents[4]
            / 'youth-vision-runtime'
            / 'scripts',
        ]
    )
    for candidate in candidates:
        if (candidate / 'competition_selector.py').is_file():
            return candidate
    raise RuntimeError(
        'competition_selector.py not found; set YOUTH_RUNTIME_ROOT'
    )


RUNTIME_SCRIPTS = find_runtime_scripts()
sys.path.insert(0, str(RUNTIME_SCRIPTS))
from competition_selector import CompetitionSelector  # noqa: E402


AIRCRAFT_LAT = 22.88190000
AIRCRAFT_LON = 113.48830000
AIRCRAFT_ALT_MSL_M = 35.0


class IntegrationProbe(Node):
    def __init__(self):
        super().__init__('competition_fullchain_probe')
        self.selected = []
        self.target_commands = []
        self.summary_events = []
        self.create_subscription(
            ReconTarget,
            '/vision/competition_selected_target',
            self.selected.append,
            10,
        )
        self.create_subscription(
            TargetCommand,
            '/vision/target_command',
            self.target_commands.append,
            10,
        )
        self.create_subscription(
            String,
            '/fcu/mission_summary_event',
            lambda msg: self.summary_events.append(json.loads(msg.data)),
            10,
        )


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
        manager.current_mission_seq = 0
        manager.current_mission_count = 12
        manager.last_mission_waypoints_time = now
        if not manager.transition_to(
            MissionState.STANDBY,
            'isolated_mock_ready',
        ):
            raise RuntimeError('mission manager could not enter STANDBY')


def refresh_manager_health(manager):
    now = time.monotonic()
    with manager._lock:
        manager.last_fcu_state_time = now
        manager.last_heartbeat_time = now
        manager.last_vehicle_info_time = now
        manager.last_position_time = now
        manager.last_gps_time = now
        manager.last_ekf_time = now
        manager.last_sensor_time = now
        manager.last_mission_waypoints_time = now


def make_fcu_ready(fcu):
    if not fcu.dry_run_goto or fcu.allow_mission_upload:
        raise RuntimeError(
            'integration test requires uploads to remain disabled'
        )
    if fcu.insert_wp_index != 5 or fcu.resume_wp_index != 10:
        raise RuntimeError('mission splice parameters are stale')
    if abs(fcu.d_offset_m - 50.0) > 1.0e-9:
        raise RuntimeError('D-point offset is stale')

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


def write_result(
    root,
    target_id,
    label,
    class_id,
    latitude,
    longitude,
    observations,
):
    target_root = root / target_id
    target_root.mkdir()
    payload = {
        'target_id': target_id,
        'status': 'finalized',
        'valid': True,
        'rtk_fixed': True,
        'observation_count': observations,
        'frame_path': str(target_root / 'frame.jpg'),
        'crop_path': str(target_root / 'crop.jpg'),
        'recognition': {
            'label': label,
            'class_id': class_id,
            'confidence': 0.98,
            'label_consensus': 0.95,
        },
        'coordinate': {
            'latitude': latitude,
            'longitude': longitude,
            'altitude_msl_m': 0.0,
            'horizontal_radius_95_m': 0.8,
        },
    }
    (target_root / 'result.json').write_text(
        json.dumps(payload, ensure_ascii=False),
        encoding='utf-8',
    )


def spin_for(executor, manager, duration_sec):
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        refresh_manager_health(manager)
        executor.spin_once(timeout_sec=0.02)


def main():
    with tempfile.TemporaryDirectory(
        prefix='competition_fullchain_'
    ) as directory:
        root = Path(directory)
        # Four finalized packets represent three physical targets. The weak
        # five-metre neighbour must be suppressed before the highest-value
        # image target is sent through the flight bridge.
        write_result(root, 'target_a', '坦克', 3, 22.8819000, 113.4883000, 42)
        write_result(
            root,
            'target_a_weak',
            '战斗机',
            5,
            22.8819000,
            113.4883488,
            12,
        )
        write_result(root, 'target_b', '战斗机', 5, 22.8819000, 113.4884464, 38)
        write_result(root, 'target_c', '轰炸机', 9, 22.8819000, 113.4885928, 36)

        rclpy.init(args=[
            '--ros-args',
            '-p', 'insert_wp_index:=5',
            '-p', 'resume_wp_index:=10',
            '-p', 'd_offset_m:=50.0',
        ])
        executor = MultiThreadedExecutor(num_threads=4)
        nodes = []
        try:
            fcu = FcuInterfaceMavrosNode()
            make_fcu_ready(fcu)
            manager = MissionManagerNode()
            selector = CompetitionSelector(SimpleNamespace(
                mode='image',
                session_root=root,
                required_targets=3,
                dedup_radius_m=10.0,
                settle_sec=1.0,
                allow_confirmed=False,
                poll_period_sec=0.05,
            ))
            bridge = VisionTargetBridge(parameter_overrides=[
                Parameter(
                    'source_topic',
                    Parameter.Type.STRING,
                    '/vision/competition_selected_target',
                ),
                Parameter('heading_deg', Parameter.Type.DOUBLE, 90.0),
                Parameter('auto_execute', Parameter.Type.BOOL, True),
                Parameter(
                    'automation_step_timeout_sec', Parameter.Type.DOUBLE, 4.0
                ),
                Parameter(
                    'automation_total_timeout_sec', Parameter.Type.DOUBLE, 8.0
                ),
            ])
            probe = IntegrationProbe()
            nodes = [fcu, manager, selector, bridge, probe]
            for node in nodes:
                executor.add_node(node)

            make_manager_ready(manager)
            spin_for(executor, manager, 0.5)
            if not manager.goto_client.service_is_ready():
                raise RuntimeError(
                    '/fcu/goto_global service was not discovered'
                )
            if bridge.mission_state != 'STANDBY':
                raise RuntimeError('bridge did not receive the STANDBY state')

            deadline = time.monotonic() + 6.0
            dry_run_event = None
            while time.monotonic() < deadline:
                spin_for(executor, manager, 0.1)
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

            if not selector.output_path.is_file():
                raise RuntimeError('competition_selected.json was not written')
            decision = json.loads(
                selector.output_path.read_text(encoding='utf-8')
            )
            selected = decision['selected_target']
            if (
                selected['target_id'] != 'target_c'
                or selected['label'] != '轰炸机'
            ):
                raise RuntimeError(
                    f'unexpected competition selection: {selected}'
                )
            if len(decision['candidates_used']) != 3:
                raise RuntimeError(
                    'selector did not retain exactly three targets'
                )
            suppressed_ids = {
                item['target_id'] for item in decision['suppressed_candidates']
            }
            if 'target_a_weak' not in suppressed_ids:
                raise RuntimeError('nearby weak packet was not suppressed')
            if not probe.selected:
                raise RuntimeError('selector published no ReconTarget')
            if len(probe.target_commands) != 1:
                raise RuntimeError(
                    'expected one TargetCommand, got '
                    f'{len(probe.target_commands)}'
                )
            if dry_run_event is None:
                raise RuntimeError('FCU dry-run route event was not produced')

            command = probe.target_commands[0]
            selected_coordinate = selected['coordinate']
            if (
                abs(command.latitude - selected_coordinate['latitude'])
                > 1.0e-10
            ):
                raise RuntimeError('selected latitude changed in bridge')
            if (
                abs(command.longitude - selected_coordinate['longitude'])
                > 1.0e-10
            ):
                raise RuntimeError('selected longitude changed in bridge')
            if abs(dry_run_event['target_lat'] - command.latitude) > 1.0e-10:
                raise RuntimeError('FCU planner received a different latitude')
            if abs(dry_run_event['target_lon'] - command.longitude) > 1.0e-10:
                raise RuntimeError(
                    'FCU planner received a different longitude'
                )

            ros_domain_id = int(os.environ.get('ROS_DOMAIN_ID', '-1'))
            output = {
                'safe_isolation': {
                    'ros_domain_id': ros_domain_id,
                    'dry_run_goto': fcu.dry_run_goto,
                    'allow_mission_upload': fcu.allow_mission_upload,
                    'real_mavros_write_attempted': False,
                },
                'selector': {
                    'eligible_nonempty_targets': decision[
                        'eligible_nonempty_targets'
                    ],
                    'suppressed_target_ids': sorted(suppressed_ids),
                    'selection_rule': decision['selection_rule'],
                    'selected_target_id': selected['target_id'],
                    'selected_label': selected['label'],
                },
                'bridge': {
                    'target_command_count': len(probe.target_commands),
                    'phase': bridge.automation_phase,
                    'latitude': command.latitude,
                    'longitude': command.longitude,
                },
                'route_rewrite_dry_run': {
                    'event': dry_run_event['event'],
                    'target_lat': dry_run_event['target_lat'],
                    'target_lon': dry_run_event['target_lon'],
                    'upload_attempted': dry_run_event['upload_attempted'],
                    'release_command_enabled': dry_run_event[
                        'release_command_enabled'
                    ],
                },
            }
            print(
                'SAFE_COMPETITION_FULLCHAIN_RESULT='
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
