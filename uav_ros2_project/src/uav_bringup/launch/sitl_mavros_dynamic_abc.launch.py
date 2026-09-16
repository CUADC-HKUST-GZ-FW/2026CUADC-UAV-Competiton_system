"""SITL bringup with MAVROS, JSON target input, and unified per-run logging."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from uav_bringup.flight_logging import create_logging_setup, declare_logging_arguments
from uav_bringup.launch_params import (
    COMMON_FCU_ARGUMENT_DEFAULTS,
    COMMON_MISSION_MANAGER_ARGUMENT_DEFAULTS,
    common_launch_configurations,
    declare_common_arguments,
)


# ============================================================================
# SITL-only editable defaults
#
# Shared REAL/SITL values live in shared_launch_params.py. Everything below is
# specific to the SITL path or is currently configured only by SITL.
# ============================================================================
SITL_ONLY_ARGUMENT_DEFAULTS = {
    'fcu_url': 'udp://127.0.0.1:14550@14555',
    'vision_heading_deg': '0.0',
}

# Default JSON result root: ~/uav_ros2_ws/test_data/vision_result
SITL_VISION_RESULT_ROOT_DEFAULT = PathJoinSubstitution([
    EnvironmentVariable('HOME'),
    'uav_ros2_ws',
    'test_data',
    'vision_result',
])

SITL_TARGET_GATE_DEFAULTS = {
    'target_id': 'target_001',
    'manager_state_topic': '/mission_manager/state',
    'target_valid_topic': '/mission_manager/target_valid',
    'required_manager_state': 'STANDBY',
    'standby_stable_sec': 0.5,
    'standby_timeout_sec': 30.0,
    'publish_period_sec': 0.3,
    'max_publish_attempts': 3,
}

SITL_FCU_ONLY_DEFAULTS = {
    'dry_run_goto': False,
    'allow_mission_upload': True,
    # SITL executes real mission operations to validate the complete flow.
    'control_mode': 'REAL_CONTROL',
    'enable_real_payload_release': True,
}

SITL_MISSION_MANAGER_ONLY_DEFAULTS = {
    # SITL does not require the real aircraft's EKF telemetry.
    'ekf_required': False,
    # MAVROS/SITL startup may temporarily lack telemetry.
    'heartbeat_timeout_sec': 15.0,
    'fcu_state_timeout_sec': 15.0,
    'position_timeout_sec': 15.0,
    'gps_timeout_sec': 15.0,
    'sensor_timeout_sec': 15.0,
}


def _declare_sitl_only_arguments():
    return [
        DeclareLaunchArgument(
            'fcu_url',
            default_value=SITL_ONLY_ARGUMENT_DEFAULTS['fcu_url'],
        ),
        DeclareLaunchArgument(
            'vision_result_root',
            default_value=SITL_VISION_RESULT_ROOT_DEFAULT,
            description=(
                'SITL JSON vision result root containing target_001/result.json.'
            ),
        ),
        DeclareLaunchArgument(
            'vision_heading_deg',
            default_value=SITL_ONLY_ARGUMENT_DEFAULTS['vision_heading_deg'],
        ),
    ]


def _prefixed(prefix, values):
    """把节点参数放进 manifest，同时不重复定义参数值。"""
    return {f'{prefix}.{name}': value for name, value in values.items()}


def generate_launch_description():
    # ========================================================================
    # Shared values come only from shared_launch_params.py
    # ========================================================================
    common = common_launch_configurations()

    use_sim_time = common['use_sim_time']
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = common['gcs_url']
    target_system_id = common['target_system_id']
    target_component_id = common['target_component_id']

    vision_result_root = LaunchConfiguration('vision_result_root')
    vision_heading_deg = LaunchConfiguration('vision_heading_deg')

    # ========================================================================
    # Public config-file paths
    # ========================================================================
    mission_safety_config = PathJoinSubstitution([
        FindPackageShare('uav_mission_manager'),
        'config',
        'mission_safety.yaml',
    ])

    payload_monitor_config = PathJoinSubstitution([
        FindPackageShare('uav_payload'),
        'config',
        'payload_monitor.yaml',
    ])

    # ========================================================================
    # SITL Target Gate parameters
    # ========================================================================
    target_gate_parameters = {
        'use_sim_time': use_sim_time,
        'result_root': vision_result_root,
        'heading_deg': vision_heading_deg,
        **SITL_TARGET_GATE_DEFAULTS,
    }

    # ========================================================================
    # FCU Interface parameters
    # ========================================================================
    fcu_parameters = {
        'use_sim_time': use_sim_time,
        **SITL_FCU_ONLY_DEFAULTS,
        # Every parameter below reads its default from shared_launch_params.py.
        **{
            name: common[name]
            for name in COMMON_FCU_ARGUMENT_DEFAULTS
        },
        'target_system_id': target_system_id,
        'target_component_id': target_component_id,
    }

    # ========================================================================
    # Mission Manager parameters
    # ========================================================================
    mission_manager_parameters = {
        'use_sim_time': use_sim_time,
        'insert_wp_index': common['insert_wp_index'],
        **{
            name: common[name]
            for name in COMMON_MISSION_MANAGER_ARGUMENT_DEFAULTS
        },
        **SITL_MISSION_MANAGER_ONLY_DEFAULTS,
    }

    # ========================================================================
    # Payload Monitor parameters
    # ========================================================================
    payload_monitor_parameters = {
        'use_sim_time': use_sim_time,
    }

    # ========================================================================
    # Unified Logging
    # ========================================================================
    manifest_fields = {
        'fcu_url': fcu_url,
        'gcs_url': gcs_url,
        'target_system_id': target_system_id,
        'target_component_id': target_component_id,
        'vision_result_root': vision_result_root,
        'vision_heading_deg': vision_heading_deg,
        'mission_manager.config_file': mission_safety_config,
        'payload_monitor.config_file': payload_monitor_config,
        **_prefixed('target_gate', target_gate_parameters),
        **_prefixed('fcu', fcu_parameters),
        **_prefixed('mission_manager', mission_manager_parameters),
        **_prefixed('payload_monitor', payload_monitor_parameters),
    }

    logging_setup = create_logging_setup('SITL', manifest_fields)

    # ========================================================================
    # MAVROS
    # ========================================================================
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'apm.launch',
            ])
        ),
        launch_arguments={
            'fcu_url': fcu_url,
            'gcs_url': gcs_url,
            'tgt_system': target_system_id,
            'tgt_component': target_component_id,
        }.items(),
    )

    # ========================================================================
    # Nodes
    # ========================================================================
    sitl_target_gate = Node(
        package='uav_vision_bridge',
        executable='sitl_target_gate_node',
        name='sitl_target_gate_node',
        output='both',
        parameters=[target_gate_parameters],
    )

    fcu_interface = Node(
        package='uav_fcu_interface',
        executable='fcu_interface_mavros_node',
        name='fcu_interface_mavros_node',
        output='both',
        parameters=[fcu_parameters],
    )

    mission_manager = Node(
        package='uav_mission_manager',
        executable='mission_manager_node',
        name='mission_manager_node',
        output='both',
        parameters=[mission_safety_config, mission_manager_parameters],
    )

    payload_monitor = Node(
        package='uav_payload',
        executable='payload_monitor_node',
        name='payload_monitor_node',
        output='both',
        parameters=[payload_monitor_config, payload_monitor_parameters],
    )

    flight_summary_logger = Node(
        package='uav_bringup',
        executable='flight_summary_logger_node',
        name='flight_summary_logger_node',
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time,
            'aburcd_update_metrics_enabled': common[
                'aburcd_update_metrics_enabled'
            ],
            'dynamic_r_enabled': common['dynamic_r_enabled'],
        }],
    )

    arguments = (
        declare_common_arguments()
        + _declare_sitl_only_arguments()
        + declare_logging_arguments()
    )

    return LaunchDescription(
        arguments
        + [
            logging_setup,
            LogInfo(
                msg=(
                    '[SITL] MAVROS is included in this launch; '
                    'do not start another MAVROS instance separately.'
                )
            ),
            mavros,
            sitl_target_gate,
            fcu_interface,
            mission_manager,
            payload_monitor,
            flight_summary_logger,
        ]
    )
