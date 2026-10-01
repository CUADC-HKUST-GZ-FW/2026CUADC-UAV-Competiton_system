#!/usr/bin/env python3
"""Exercise partial competition fallback without touching a real mission."""

import json
import os
from pathlib import Path
from types import SimpleNamespace
import sys
import tempfile
import time

from mavros_msgs.msg import State, Waypoint, WaypointList
from sensor_msgs.msg import NavSatFix, NavSatStatus
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
        super().__init__('competition_partial_fallback_probe')
        self.selected = []
        self.target_commands = []
        self.summary_events = []
        self.mission_publisher = self.create_publisher(
            WaypointList,
            '/mavros/mission/waypoints',
            10,
        )
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

    def publish_mission_seq(self, current_seq):
        message = WaypointList()
        message.current_seq = current_seq
        message.waypoints = [Waypoint() for _ in range(12)]
        self.mission_publisher.publish(message)


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
        raise RuntimeError('test requires dry-run with mission upload disabled')

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
    longitude,
    observations,
    radius_m,
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
            'class_id': 0,
            'confidence': 0.98,
            'label_consensus': 0.95,
        },
        'coordinate': {
            'latitude': AIRCRAFT_LAT,
            'longitude': longitude,
            'altitude_msl_m': 0.0,
            'horizontal_radius_95_m': radius_m,
        },
    }
    (target_root / 'result.json').write_text(
        json.dumps(payload, ensure_ascii=True),
        encoding='utf-8',
    )


def spin_for(executor, duration_sec, manager=None):
    deadline = time.monotonic() + duration_sec
    while time.monotonic() < deadline:
        if manager is not None:
            refresh_manager_health(manager)
        executor.spin_once(timeout_sec=0.02)


def selector_args(root):
    return SimpleNamespace(
        mode='digit',
        session_root=root,
        required_targets=3,
        dedup_radius_m=10.0,
        settle_sec=1.0,
        allow_confirmed=False,
        poll_period_sec=0.05,
        fallback_commit_wp_index=4,
        submission_deadline_wp_index=5,
    )


def run_fallback_two(root):
    write_result(root, 'target_many_frames', '45', 113.4883000, 80, 1.4)
    write_result(root, 'target_precise', '83', 113.4884464, 8, 0.4)

    rclpy.init(args=[
        '--ros-args',
        '-p', 'insert_wp_index:=5',
        '-p', 'resume_wp_index:=10',
        '-p', 'release_offset_m:=46.0',
        '-p', 'd_offset_m:=50.0',
    ])
    executor = MultiThreadedExecutor(num_threads=4)
    nodes = []
    try:
        fcu = FcuInterfaceMavrosNode()
        make_fcu_ready(fcu)
        manager = MissionManagerNode()
        selector = CompetitionSelector(selector_args(root))
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
        spin_for(executor, 0.6, manager)
        if not manager.goto_client.service_is_ready():
            raise RuntimeError('/fcu/goto_global service was not discovered')
        if bridge.mission_state != 'STANDBY':
            raise RuntimeError('bridge did not receive the STANDBY state')

        probe.publish_mission_seq(3)
        spin_for(executor, 1.3, manager)
        if selector.selection is not None or probe.target_commands:
            raise RuntimeError('partial result was published before waypoint 4')

        probe.publish_mission_seq(4)
        deadline = time.monotonic() + 6.0
        dry_run_event = None
        while time.monotonic() < deadline:
            spin_for(executor, 0.1, manager)
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
        if decision['selection_rule'] != 'fallback_minimum_r95':
            raise RuntimeError(f'unexpected fallback rule: {decision}')
        if selected['target_id'] != 'target_precise':
            raise RuntimeError(f'lowest-radius result was not chosen: {selected}')
        if len(probe.target_commands) != 1:
            raise RuntimeError(
                f'expected one TargetCommand, got {len(probe.target_commands)}'
            )
        if dry_run_event is None:
            raise RuntimeError('FCU dry-run route event was not produced')
        if dry_run_event['upload_attempted']:
            raise RuntimeError('dry-run unexpectedly attempted mission upload')

        return {
            'scenario': 'fallback_two',
            'selection_rule': decision['selection_rule'],
            'selected_target_id': selected['target_id'],
            'selected_radius_m': selected['coordinate'][
                'horizontal_radius_95_m'
            ],
            'target_command_count': len(probe.target_commands),
            'route_event': dry_run_event['event'],
            'mission_upload_attempted': dry_run_event['upload_attempted'],
        }
    finally:
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


def run_base_zero(root):
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=2)
    nodes = []
    try:
        selector = CompetitionSelector(selector_args(root))
        probe = IntegrationProbe()
        nodes = [selector, probe]
        for node in nodes:
            executor.add_node(node)

        spin_for(executor, 0.4)
        probe.publish_mission_seq(4)
        spin_for(executor, 0.4)
        if selector.selection is not None or probe.selected:
            raise RuntimeError('zero-target result published at waypoint 4')

        probe.publish_mission_seq(5)
        spin_for(executor, 0.4)
        if not selector.selection_closed:
            raise RuntimeError('zero-target selection did not close at waypoint 5')
        if selector.selection is not None or probe.selected:
            raise RuntimeError('zero-target scenario published a target')
        if selector.output_path.exists():
            raise RuntimeError('zero-target scenario wrote a selection artifact')

        return {
            'scenario': 'base_zero',
            'selection_closed': selector.selection_closed,
            'selected_message_count': len(probe.selected),
            'target_command_count': len(probe.target_commands),
            'base_mission_preserved': True,
        }
    finally:
        for node in reversed(nodes):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    scenario = os.environ.get(
        'COMPETITION_FALLBACK_SCENARIO',
        'fallback_two',
    )
    with tempfile.TemporaryDirectory(
        prefix='competition_partial_fallback_'
    ) as directory:
        root = Path(directory)
        if scenario == 'fallback_two':
            output = run_fallback_two(root)
        elif scenario == 'base_zero':
            output = run_base_zero(root)
        else:
            raise RuntimeError(f'unsupported scenario: {scenario}')

    output['ros_domain_id'] = int(os.environ.get('ROS_DOMAIN_ID', '-1'))
    print(
        'SAFE_COMPETITION_PARTIAL_FALLBACK_RESULT='
        + json.dumps(output, ensure_ascii=True, sort_keys=True)
    )


if __name__ == '__main__':
    main()
