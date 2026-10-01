Modified files for SITL-only MAVROS param plugin disable:

1. uav_bringup/launch/sitl_mavros_dynamic_abc.launch.py
2. uav_bringup/config/sitl_mavros_pluginlists.yaml   (new)
3. uav_bringup/setup.py

REAL bringup is unchanged.

After replacing files, rebuild:
  cd ~/2026CUADC-UAV-Competiton_system/uav_ros2_project
  colcon build --packages-select uav_bringup
  source install/setup.bash

Then launch as before:
  ros2 launch uav_bringup sitl_mavros_dynamic_abc.launch.py

Expected MAVROS startup behavior:
  - should NOT show: Plugin param created / Plugin param initialized
  - should NOT show: PR: parameters list received
  - should still show waypoint/global_position/sys_status/etc. plugins
