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
readonly RECOGNITION_CONFIG="${WS_ROOT}/config/recognition_mode.conf"
readonly RECON_START_SCRIPT="/home/nx163/youth-vision-runtime/scripts/start_recon_pipeline.sh"

if [[ ! -r "${ROS_SETUP}" ]]; then
    echo "[STARTUP][ERROR] ROS setup not found: ${ROS_SETUP}" >&2
    exit 1
fi

if [[ ! -d "${WS_ROOT}" ]]; then
    echo "[STARTUP][ERROR] workspace not found: ${WS_ROOT}" >&2
    exit 1
fi

if [[ ! -r "${WS_SETUP}" ]]; then
    echo "[STARTUP][ERROR] workspace setup not found: ${WS_SETUP}" >&2
    exit 1
fi

# Some ROS setup scripts reference variables that may not exist under nounset.
set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PYTHONUNBUFFERED=1

cd "${WS_ROOT}"

if ! command -v ros2 >/dev/null 2>&1; then
    echo "[STARTUP][ERROR] ros2 command unavailable after sourcing setup files" >&2
    exit 1
fi

if [[ ! -r "${RECOGNITION_CONFIG}" ]]; then
    echo "[vision] ERROR recognition config not found: ${RECOGNITION_CONFIG}" >&2
    exit 1
fi

source "${RECOGNITION_CONFIG}"
RECOGNITION_MODE="${RECOGNITION_MODE:-}"

case "${RECOGNITION_MODE}" in
    digit|image|none)
        ;;
    *)
        echo "[vision] ERROR invalid recognition mode: ${RECOGNITION_MODE}" >&2
        exit 1
        ;;
esac

if declare -F uav_maybe_detach_from_ssh >/dev/null; then
    detach_rc=0
    uav_maybe_detach_from_ssh "$0" "$@" || detach_rc=$?
    if [[ "${detach_rc}" == "200" ]]; then
        exit 0
    elif [[ "${detach_rc}" != "0" ]]; then
        exit "${detach_rc}"
    fi
fi

case "${RECOGNITION_MODE}" in
    digit|image)
        echo "[vision] recognition_mode=${RECOGNITION_MODE}"
        echo "[vision] starting ${RECOGNITION_MODE} recognition pipeline"

        if [[ ! -x "${RECON_START_SCRIPT}" ]]; then
            echo "[vision] ERROR recognition startup script not executable: ${RECON_START_SCRIPT}" >&2
            exit 1
        fi

        set +e
        "${RECON_START_SCRIPT}" "${RECOGNITION_MODE}" 1
        recon_rc=$?
        set -e

        if [[ ${recon_rc} -ne 0 ]]; then
            echo "[vision] ERROR recognition pipeline start failed mode=${RECOGNITION_MODE} rc=${recon_rc}" >&2
            exit "${recon_rc}"
        fi

        echo "[vision] recognition pipeline command succeeded mode=${RECOGNITION_MODE}"
        ;;

    none)
        echo "[vision] recognition_mode=none"
        echo "[vision] recognition disabled"
        ;;

    *)
        echo "[vision] ERROR invalid recognition mode: ${RECOGNITION_MODE}" >&2
        exit 1
        ;;
esac

echo "[STARTUP] workspace=${WS_ROOT} ros_domain_id=${ROS_DOMAIN_ID}"
echo "[STARTUP] launching uav_bringup real_bringup.launch.py"

# Replace the shell process so systemd signals reach ros2 launch directly.
exec ros2 launch uav_bringup real_bringup.launch.py
