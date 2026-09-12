import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('uav_recon'),
        'config',
        'recon.yaml',
    )
    output_root = LaunchConfiguration('output_root')
    minimum_gps_fix_type = LaunchConfiguration('minimum_gps_fix_type')

    return LaunchDescription([
        DeclareLaunchArgument(
            'output_root',
            default_value=(
                '/home/nx163/youth-vision-runtime/'
                'recon_results/current'
            ),
        ),
        DeclareLaunchArgument(
            'minimum_gps_fix_type',
            default_value='6',
        ),
        Node(
            package='uav_recon',
            executable='recon_geolocator_node',
            name='recon_geolocator',
            output='screen',
            parameters=[
                config,
                {
                    'output_root': output_root,
                    # Keep the current recon_node.py unchanged. Expose the
                    # orchestration-facing name and map it to its existing
                    # parameter.
                    'rtk_fixed_min_fix_type': ParameterValue(
                        minimum_gps_fix_type,
                        value_type=int,
                    ),
                },
            ],
        ),
    ])
