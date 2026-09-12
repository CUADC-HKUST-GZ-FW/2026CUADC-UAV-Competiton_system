from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    fcu_url = LaunchConfiguration('fcu_url')
    output_root = LaunchConfiguration('output_root')
    fixed_relative_altitude_m = LaunchConfiguration('fixed_relative_altitude_m')
    max_horizontal_radius_95_m = LaunchConfiguration('max_horizontal_radius_95_m')
    mavros_launch = os.path.join(
        get_package_share_directory('mavros'), 'launch', 'apm.launch'
    )
    recon_config = os.path.join(
        get_package_share_directory('uav_recon'), 'config', 'recon.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'fcu_url',
            default_value='udp://0.0.0.0:15001@192.168.144.14:15001',
        ),
        DeclareLaunchArgument(
            'output_root',
            default_value='/home/nx163/youth-vision-runtime/recon_results/current',
        ),
        DeclareLaunchArgument('fixed_relative_altitude_m'),
        DeclareLaunchArgument(
            'max_horizontal_radius_95_m',
            default_value='0.5',
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch),
            launch_arguments={
                'fcu_url': fcu_url,
                'gcs_url': '',
                'tgt_system': '1',
                'tgt_component': '1',
            }.items(),
        ),
        Node(
            package='uav_recon',
            executable='recon_geolocator_node',
            name='recon_geolocator',
            output='screen',
            parameters=[
                recon_config,
                {
                    'output_root': output_root,
                    'ground_altitude_mode': 'fixed_relative',
                    'fixed_relative_altitude_m': ParameterValue(
                        fixed_relative_altitude_m, value_type=float
                    ),
                    'max_horizontal_radius_95_m': ParameterValue(
                        max_horizontal_radius_95_m, value_type=float
                    ),
                },
            ],
        ),
    ])
