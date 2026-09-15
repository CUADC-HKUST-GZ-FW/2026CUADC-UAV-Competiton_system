#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_SETUP:-/home/nx163/uav_ros2_project/install/setup.bash}"
CONFIG="${CONFIG:-$ROOT/geo_bridge/config/geo_bridge.yaml}"

if [[ ! -r "$ROS_SETUP" ]]; then
  echo "[GEO] ROS setup not found: $ROS_SETUP" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
if [[ -r "$WS_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$WS_SETUP"
fi
set -u

export PYTHONPATH="$ROOT/geo_bridge${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONUNBUFFERED=1

exec python3 -m geo_bridge.node --ros-args --params-file "$CONFIG"
