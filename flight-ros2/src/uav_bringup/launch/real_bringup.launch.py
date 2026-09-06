"""Fail-closed real-hardware bringup with unified per-flight logging."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from uav_bringup.flight_logging import create_logging_setup, declare_logging_arguments
from uav_bringup.launch_params import (
    COMMON_FCU_ARGUMENT_DEFAULTS,
    common_launch_configurations,
    declare_common_arguments,
)


# ============================================================================
# REAL-only editable defaults
#
# Only values that are specific to real-hardware operation, or are currently
# configured only by REAL, stay here. Shared REAL/SITL values live in
# shared_launch_params.py.
# ============================================================================
REAL_ONLY_ARGUMENT_DEFAULTS = {
    # MAVROS real FCU endpoint.
    'fcu_url': 'udp://0.0.0.0:15001@192.168.144.14:15001',

    # Real-flight safety / actuation controls.
    'dry_run': 'false',
    'enable_real_payload_release': 'true',
    'allow_mission_upload': 'true',

    # Mission service controls currently configured only by REAL.
    'service_availability_timeout_sec': '2.0',
    'mission_service_retry_count': '3',
    'mission_service_retry_delay_sec': '0.15',
    'clear_mission_before_full_push': 'false',

    # A-B trajectory check controls currently configured only by REAL.
    'b_check_end_distance_from_a_m': '50.0',
    'b_check_max_heading_error_deg': '20.0',
    'b_check_convergence_tolerance_m': '1.5',
    'b_check_min_converging_ratio': '0.5',
}


def _declare_real_only_arguments():
    return [
        DeclareLaunchArgument(name, default_value=default_value)
        for name, default_value in REAL_ONLY_ARGUMENT_DEFAULTS.items()
    ]


def _real_only_launch_configurations():
    return {
        name: LaunchConfiguration(name)
        for name in REAL_ONLY_ARGUMENT_DEFAULTS
    }


def _prefixed(prefix, values):
    """把节点实际参数放进 manifest，而不重新定义参数。"""
    return {f'{prefix}.{name}': value for name, value in values.items()}


def _log_real_profile(context):
    """打印本次实飞关键启动参数。"""
    fcu_url = LaunchConfiguration('fcu_url').perform(context)
    endpoint_text = fcu_url.removeprefix('udp://')

    if '@' in endpoint_text:
        listen_endpoint, remote_endpoint = endpoint_text.split('@', 1)
    else:
        listen_endpoint = endpoint_text
        remote_endpoint = '(peer learned from UDP)'

    values = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            'target_system_id',
            'target_component_id',
            'use_sim_time',
            'dry_run',
            'allow_mission_upload',
            'enable_real_payload_release',
        )
    }

    return [
        LogInfo(msg='========== REAL FLIGHT PROFILE =========='),
        LogInfo(msg=f'MAVROS FCU URL: {fcu_url}'),
        LogInfo(msg=f'MAVROS listen: {listen_endpoint}'),
        LogInfo(msg=f'Remote FCU: {remote_endpoint or "(not fixed)"}'),
        LogInfo(msg=f'target_system_id: {values["target_system_id"]}'),
        LogInfo(msg=f'target_component_id: {values["target_component_id"]}'),
        LogInfo(msg=f'use_sim_time: {values["use_sim_time"]}'),
        LogInfo(msg=f'dry_run: {values["dry_run"]}'),
        LogInfo(msg=f'allow_mission_upload: {values["allow_mission_upload"]}'),
        LogInfo(
            msg=(
                'enable_real_payload_release: '
                + values['enable_real_payload_release']
            )
        ),
        LogInfo(
            msg=(
                'Startup itself performs no arming, AUTO switch, mission upload, '
                'or payload actuation.'
            )
        ),
        LogInfo(msg='========================================='),
    ]


def generate_launch_description():
    # ========================================================================
    # Shared values come only from shared_launch_params.py
    # ========================================================================
    common = common_launch_configurations()
    real_only = _real_only_launch_configurations()

    fcu_url = real_only['fcu_url']
    gcs_url = common['gcs_url']
    target_system_id = common['target_system_id']
    target_component_id = common['target_component_id']
    use_sim_time = common['use_sim_time']

    dry_run = real_only['dry_run']
    allow_mission_upload = real_only['allow_mission_upload']
    enable_real_payload_release = real_only['enable_real_payload_release']

    # ========================================================================
    # Control Mode
    # ========================================================================
    control_mode = PythonExpression([
        "'REAL_CONTROL' if '",
        enable_real_payload_release,
        "'.lower() == 'true' else 'DRY_RUN'",
    ])

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
    # FCU Interface parameters
    # ========================================================================
    fcu_parameters = {
        'use_sim_time': use_sim_time,
        'dry_run_goto': dry_run,
        'allow_mission_upload': allow_mission_upload,
        'control_mode': control_mode,
        'enable_real_payload_release': enable_real_payload_release,

        # Every parameter below reads its default from shared_launch_params.py.
        **{
            name: common[name]
            for name in COMMON_FCU_ARGUMENT_DEFAULTS
        },

        # REAL-only FCU parameters.
        'service_availability_timeout_sec': real_only['service_availability_timeout_sec'],
        'mission_service_retry_count': real_only['mission_service_retry_count'],
        'mission_service_retry_delay_sec': real_only['mission_service_retry_delay_sec'],
        'clear_mission_before_full_push': real_only['clear_mission_before_full_push'],
        'b_check_end_distance_from_a_m': real_only['b_check_end_distance_from_a_m'],
        'b_check_max_heading_error_deg': real_only['b_check_max_heading_error_deg'],
        'b_check_convergence_tolerance_m': real_only['b_check_convergence_tolerance_m'],
        'b_check_min_converging_ratio': real_only['b_check_min_converging_ratio'],
    }

    # ========================================================================
    # Mission Manager parameters
    # ========================================================================
    mission_manager_parameters = {
        'use_sim_time': use_sim_time,
        'expected_system_id': target_system_id,
        'expected_component_id': target_component_id,
        'acceptance_radius_m': common['acceptance_radius_m'],
    }

    # ========================================================================
    # Payload Monitor parameters
    # ========================================================================
    payload_monitor_parameters = {
        'use_sim_time': use_sim_time,
        # Keep payload telemetry validation aligned with the command that the
        # FCU interface inserts into the composite mission.
        'servo_channel': common['servo_channel'],
        'release_pwm': common['release_pwm'],
    }

    # ========================================================================
    # Unified Logging
    # ========================================================================
    manifest_fields = {
        'fcu_url': fcu_url,
        'gcs_url': gcs_url,
        'target_system_id': target_system_id,
        'target_component_id': target_component_id,
        'target_input_mode': 'DIRECT_TOPIC:/vision/target_command',
        'mission_manager.config_file': mission_safety_config,
        'payload_monitor.config_file': payload_monitor_config,
        **_prefixed('fcu', fcu_parameters),
        **_prefixed('mission_manager', mission_manager_parameters),
        **_prefixed('payload_monitor', payload_monitor_parameters),
    }

    logging_setup = create_logging_setup('REAL', manifest_fields)

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
    fcu_interface = Node(
        package='uav_fcu_interface',
        executable='fcu_interface_mavros_node',
        name='fcu_interface_mavros_node',
        output='both',
        on_exit=[LogInfo(msg='[REAL FLIGHT PROFILE] FCU interface exited')],
        parameters=[fcu_parameters],
    )

    mission_manager = Node(
        package='uav_mission_manager',
        executable='mission_manager_node',
        name='mission_manager_node',
        output='both',
        on_exit=[LogInfo(msg='[REAL FLIGHT PROFILE] MissionManager exited')],
        parameters=[mission_safety_config, mission_manager_parameters],
    )

    payload_monitor = Node(
        package='uav_payload',
        executable='payload_monitor_node',
        name='payload_monitor_node',
        output='both',
        on_exit=[LogInfo(msg='[REAL FLIGHT PROFILE] PayloadMonitor exited')],
        parameters=[payload_monitor_config, payload_monitor_parameters],
    )

    flight_summary_logger = Node(
        package='uav_bringup',
        executable='flight_summary_logger_node',
        name='flight_summary_logger_node',
        output='both',
        on_exit=[LogInfo(msg='[REAL FLIGHT PROFILE] FlightSummaryLogger exited')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Shared arguments are declared from one common file; REAL-only arguments
    # are declared here; logging arguments continue to come from flight_logging.
    arguments = (
        declare_common_arguments()
        + _declare_real_only_arguments()
        + declare_logging_arguments()
    )

    return LaunchDescription(
        arguments
        + [
            logging_setup,
            OpaqueFunction(function=_log_real_profile),
            LogInfo(
                msg=(
                    '[REAL] Target coordinates are expected directly on '
                    '/vision/target_command; no JSON target bridge is started.'
                )
            ),
            mavros,
            fcu_interface,
            mission_manager,
            payload_monitor,
            flight_summary_logger,
        ]
    )
