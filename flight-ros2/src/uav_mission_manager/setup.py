import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_mission_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.md'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yvv',
    maintainer_email='yvvvvvvvvv@126.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mission_manager_node = uav_mission_manager.mission_manager_node:main',
        ],
    },
)
