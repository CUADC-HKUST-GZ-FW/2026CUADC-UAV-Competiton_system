"""Shared REAL/SITL per-run logging support for uav_bringup launch files."""

from datetime import datetime
import getpass
import json
import os
import re
import socket

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration


# Deliberately exclude camera/image topics and other high-bandwidth streams.
# Add a topic here once and both REAL and SITL runs will record it.
BAG_TOPICS = [
    '/rosout',
    '/vision/target_command',
    '/mission/active_task_id',
    '/mission/safety_state',
    '/mission/safety_status',
    '/fcu/composite_mission_complete',
    '/fcu/mission_summary_event',
    '/payload/monitor/status',
    '/mavros/state',
    '/mavros/global_position/global',
    '/mavros/global_position/raw/fix',
    '/mavros/gpsstatus/gps1/raw',
    '/mavros/estimator_status',
    '/mavros/sys_status',
    '/mavros/mission/waypoints',
    '/mavros/mission/reached',
    '/mavros/rc/out',
    '/mavros/vfr_hud',
]

BAG_TOPIC_REGEX = '^(' + '|'.join(re.escape(topic) for topic in BAG_TOPICS) + ')$'


def declare_logging_arguments():
    """Launch arguments shared by REAL and SITL bringup."""
    return [
        DeclareLaunchArgument(
            'flight_log_root',
            default_value='~/uav_flight_logs',
        ),
        # Empty => automatically generated timestamped ID.
        DeclareLaunchArgument('flight_id', default_value=''),
        DeclareLaunchArgument('record_flight_bag', default_value='true'),
    ]


def create_logging_setup(profile, manifest_fields):
    """Return the OpaqueFunction that prepares one run's log products."""
    return OpaqueFunction(
        function=_setup_logging,
        kwargs={
            'profile': profile,
            'manifest_fields': dict(manifest_fields),
        },
    )


def _as_bool(value):
    return str(value).strip().lower() in {'1', 'true', 'yes', 'on'}


def _resolve(context, value):
    """Resolve a Launch Substitution or convert a literal to text."""
    if hasattr(value, 'perform'):
        return value.perform(context)
    return str(value)


def _safe_profile(value):
    profile = str(value).strip().upper()
    if profile not in {'REAL', 'SITL'}:
        raise RuntimeError(f'Unsupported logging profile: {profile!r}')
    return profile


def _make_unique_run_directory(root_dir, profile, requested_flight_id):
    started_at = datetime.now().astimezone()
    if requested_flight_id.strip():
        base_id = requested_flight_id.strip()
    else:
        base_id = (
            f'{profile.lower()}_'
            f'{started_at.strftime("%Y%m%d_%H%M%S")}_'
            f'{started_at.microsecond // 1000:03d}'
        )

    profile_root = os.path.join(root_dir, profile.lower())
    os.makedirs(profile_root, exist_ok=True)

    flight_id = base_id
    suffix = 0
    while True:
        run_dir = os.path.join(profile_root, flight_id)
        try:
            os.makedirs(run_dir, exist_ok=False)
            return flight_id, run_dir, started_at
        except FileExistsError:
            suffix += 1
            flight_id = f'{base_id}_{suffix:02d}'


def _setup_logging(context, profile, manifest_fields):
    profile = _safe_profile(profile)

    root_dir = os.path.abspath(
        os.path.expanduser(LaunchConfiguration('flight_log_root').perform(context))
    )
    requested_flight_id = LaunchConfiguration('flight_id').perform(context)
    record_bag = _as_bool(
        LaunchConfiguration('record_flight_bag').perform(context)
    )

    flight_id, run_dir, started_at = _make_unique_run_directory(
        root_dir,
        profile,
        requested_flight_id,
    )

    # Human-facing layout:
    #   run_manifest.json
    #   all_nodes.log
    #   mission_summary.log
    #   node_logs/<readable-node-name>.log
    #   rosbag/<rosbag files>
    #
    # ROS 2's native PID/timestamp log files are intentionally NOT redirected
    # into this run directory.  They remain under the normal ~/.ros/log tree.
    # node_logs/ is produced from /rosout using readable node/logger names.
    manifest_path = os.path.join(run_dir, 'run_manifest.json')
    all_nodes_log_path = os.path.join(run_dir, 'all_nodes.log')
    summary_log_path = os.path.join(run_dir, 'mission_summary.log')
    node_log_dir = os.path.join(run_dir, 'node_logs')
    rosbag_dir = os.path.join(run_dir, 'rosbag')

    os.makedirs(node_log_dir, exist_ok=True)
    # Do NOT create rosbag_dir. ros2 bag record expects to create it itself.

    resolved_manifest_fields = {
        key: _resolve(context, value)
        for key, value in manifest_fields.items()
    }

    manifest = {
        'schema_version': 3,
        'profile': profile,
        'flight_id': flight_id,
        'started_at_local': started_at.isoformat(timespec='milliseconds'),
        'hostname': socket.gethostname(),
        'username': getpass.getuser(),
        'pid': os.getpid(),
        'ros_distro': os.environ.get('ROS_DISTRO', ''),
        'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', ''),
        'run_directory': run_dir,
        'run_manifest_path': manifest_path,
        'all_nodes_log_path': all_nodes_log_path,
        'mission_summary_path': summary_log_path,
        # Backward-compatible manifest key retained for existing analysis tools.
        'flight_summary_path': summary_log_path,
        'node_log_directory': node_log_dir,
        'rosbag_directory': rosbag_dir if record_bag else None,
        'native_ros_log_directory': os.path.expanduser('~/.ros/log'),
        'record_flight_bag': record_bag,
        'bag_topics': BAG_TOPICS,
        'launch_configuration': resolved_manifest_fields,
    }

    with open(manifest_path, 'w', encoding='utf-8') as manifest_file:
        json.dump(manifest, manifest_file, ensure_ascii=False, indent=2)
        manifest_file.write('\n')

    actions = [
        # Do not set ROS_LOG_DIR here.  Setting it to node_logs/ would make ROS 2
        # create unreadable python3_<pid>_<timestamp>.log files beside our named
        # per-node logs.  Native ROS logs therefore stay in ~/.ros/log.
        SetEnvironmentVariable('UAV_RUN_DIR', run_dir),
        SetEnvironmentVariable(
            'UAV_FLIGHT_SUMMARY_PATH',
            summary_log_path,
        ),
        LogInfo(msg='========== UAV RUN LOGGING =============='),
        LogInfo(msg=f'profile: {profile}'),
        LogInfo(msg=f'flight_id: {flight_id}'),
        LogInfo(msg=f'run directory: {run_dir}'),
        LogInfo(msg=f'run manifest: {manifest_path}'),
        LogInfo(msg=f'all-node chronological log: {all_nodes_log_path}'),
        LogInfo(msg=f'mission summary log: {summary_log_path}'),
        LogInfo(msg=f'named node logs: {node_log_dir}'),
        LogInfo(msg=f'rosbag enabled: {str(record_bag).lower()}'),
        LogInfo(msg='========================================='),
    ]

    # One collector writes BOTH:
    #   1) all_nodes.log in chronological receive order;
    #   2) node_logs/<node-or-logger-name>.log, with readable stable filenames.
    actions.append(
        ExecuteProcess(
            cmd=[
                'python3',
                '-m',
                'uav_bringup.rosout_log_collector_node',
                '--all-nodes-log',
                all_nodes_log_path,
                '--node-log-dir',
                node_log_dir,
            ],
            output='screen',
        )
    )

    if record_bag:
        # Regex mode discovers matching topics as they appear, so recording can
        # start before MAVROS and the mission nodes finish initializing.
        actions.append(
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'bag',
                    'record',
                    '-o',
                    rosbag_dir,
                    '-e',
                    BAG_TOPIC_REGEX,
                ],
                output='screen',
            )
        )
    else:
        actions.append(LogInfo(msg='[UAV RUN LOGGING] rosbag recording disabled'))

    return actions
