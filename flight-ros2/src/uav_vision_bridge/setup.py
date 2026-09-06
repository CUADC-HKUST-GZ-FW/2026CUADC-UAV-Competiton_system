from setuptools import find_packages, setup

package_name = 'uav_vision_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'vision_target_bridge_node = uav_vision_bridge.vision_target_bridge_node:main',
            'sitl_target_gate_node = uav_vision_bridge.sitl_target_gate_node:main',
        ],
    },
)
