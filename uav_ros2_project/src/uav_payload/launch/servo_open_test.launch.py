"""Start standalone MAVROS plus the read-only servo-open and GPS logger."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create MAVROS and the logger without starting mission/control nodes."""
    arguments = [
        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://0.0.0.0:15001@192.168.144.14:15001',
        ),
        DeclareLaunchArgument('gcs_url', default_value=''),
        DeclareLaunchArgument('target_system_id', default_value='1'),
        DeclareLaunchArgument('target_component_id', default_value='1'),
        DeclareLaunchArgument('servo_channel', default_value='7'),
        DeclareLaunchArgument('release_pwm', default_value='1900'),
        DeclareLaunchArgument('safe_pwm', default_value='1350'),
        DeclareLaunchArgument('pwm_tolerance_us', default_value='20'),
        DeclareLaunchArgument('required_open_samples', default_value='3'),
        DeclareLaunchArgument('required_closed_samples', default_value='3'),
        DeclareLaunchArgument('gps_stale_timeout_s', default_value='1.0'),
        DeclareLaunchArgument(
            'output_root',
            default_value='~/uav_flight_logs/servo_open_test',
        ),
    ]
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'apm.launch',
            ])
        ),
        launch_arguments={
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'tgt_system': LaunchConfiguration('target_system_id'),
            'tgt_component': LaunchConfiguration('target_component_id'),
        }.items(),
    )
    logger = Node(
        package='uav_payload',
        executable='servo_open_logger_node',
        name='servo_open_logger_node',
        output='both',
        parameters=[{
            'servo_channel': LaunchConfiguration('servo_channel'),
            'release_pwm': LaunchConfiguration('release_pwm'),
            'safe_pwm': LaunchConfiguration('safe_pwm'),
            'pwm_tolerance_us': LaunchConfiguration('pwm_tolerance_us'),
            'required_open_samples': LaunchConfiguration('required_open_samples'),
            'required_closed_samples': LaunchConfiguration('required_closed_samples'),
            'gps_stale_timeout_s': LaunchConfiguration('gps_stale_timeout_s'),
            'output_root': LaunchConfiguration('output_root'),
        }],
    )
    return LaunchDescription(arguments + [mavros, logger])
