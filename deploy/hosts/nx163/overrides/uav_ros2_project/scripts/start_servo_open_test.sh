#!/usr/bin/env bash

set -euo pipefail

readonly UAV_ENTRY_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly UAV_SSH_DETACH_HELPER="${UAV_ENTRY_SCRIPT_DIR}/lib/ssh_detach.sh"

if [[ -r "${UAV_SSH_DETACH_HELPER}" ]]; then
    source "${UAV_SSH_DETACH_HELPER}"
elif [[ -n "${SSH_CONNECTION:-}" || -n "${SSH_TTY:-}" ]] \
    && [[ "${UAV_FOREGROUND:-0}" != "1" ]]; then
    echo "[DETACHED][ERROR] missing helper: ${UAV_SSH_DETACH_HELPER}" >&2
    exit 1
fi

readonly ROS_SETUP="/opt/ros/humble/setup.bash"
readonly WS_ROOT="/home/nx163/uav_ros2_project"
readonly WS_SETUP="${WS_ROOT}/install/setup.bash"
readonly SERVO_CHANNEL="${1:-7}"

if [[ ! "${SERVO_CHANNEL}" =~ ^([1-9]|1[0-6])$ ]]; then
    echo "[SERVO_TEST][ERROR] channel must be an integer from 1 to 16" >&2
    exit 2
fi
if [[ ! -r "${ROS_SETUP}" ]]; then
    echo "[SERVO_TEST][ERROR] ROS setup not found: ${ROS_SETUP}" >&2
    exit 1
fi
if [[ ! -r "${WS_SETUP}" ]]; then
    echo "[SERVO_TEST][ERROR] workspace setup not found: ${WS_SETUP}" >&2
    exit 1
fi

if declare -F uav_maybe_detach_from_ssh >/dev/null; then
    detach_rc=0
    uav_maybe_detach_from_ssh "$0" "$@" || detach_rc=$?
    if [[ "${detach_rc}" == "200" ]]; then
        exit 0
    elif [[ "${detach_rc}" != "0" ]]; then
        exit "${detach_rc}"
    fi
fi

set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PYTHONUNBUFFERED=1

if ros2 node list 2>/dev/null | grep -Fxq '/servo_open_logger_node'; then
    echo "[SERVO_TEST][ERROR] servo_open_logger_node is already running" >&2
    exit 1
fi
if ros2 node list 2>/dev/null | grep -Fxq '/mavros'; then
    echo "[SERVO_TEST][ERROR] MAVROS is already running" >&2
    echo "[SERVO_TEST][ERROR] stop the other startup mode before this standalone test" >&2
    exit 1
fi

echo "[SERVO_TEST] standalone read-only mode; starting MAVROS and logger"
echo "[SERVO_TEST] mission manager, reconnaissance and control nodes stay disabled"
echo "[SERVO_TEST] channel=${SERVO_CHANNEL}"
echo "[SERVO_TEST] waiting for closed PWM before detecting the first opening"
cd "${WS_ROOT}"
exec ros2 launch uav_payload servo_open_test.launch.py \
    servo_channel:="${SERVO_CHANNEL}"
