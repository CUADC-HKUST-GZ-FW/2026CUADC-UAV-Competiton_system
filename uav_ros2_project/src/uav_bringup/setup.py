from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'uav_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch/config files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'wp'), glob('wp/*.waypoints') + glob('wp/*.waypoint')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yvv',
    maintainer_email='yvvvvvvvvv@126.com',
    description='Bringup launch files for the UAV ROS2 mission framework',
    license='TODO',
    entry_points={
        'console_scripts': [
            'flight_summary_logger_node = uav_bringup.flight_summary_logger_node:main',
        ],
    },
)
