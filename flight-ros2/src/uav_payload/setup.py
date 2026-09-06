from glob import glob

from setuptools import find_packages, setup

package_name = 'uav_payload'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
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
            'payload_sim_node = uav_payload.payload_sim_node:main',
            'payload_monitor_node = uav_payload.payload_monitor_node:main',
        ],
    },
)
