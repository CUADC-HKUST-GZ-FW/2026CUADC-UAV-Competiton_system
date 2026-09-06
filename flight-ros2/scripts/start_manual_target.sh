#!/usr/bin/env bash
set -Eeuo pipefail

# 使用格式（路径略）：./start_manual_target.sh <纬度> <经度> <航向角>

readonly ROS_SETUP="/opt/ros/humble/setup.bash"
readonly WS_ROOT="/home/nx163/uav_ros2_project"
readonly WS_SETUP="${WS_ROOT}/install/setup.bash"

# ============================================================
# 参数
# ============================================================

if [[ "$#" -ne 3 ]]; then
    echo "Usage:"
    echo "  $0 <latitude> <longitude> <heading_deg>"
    echo
    echo "Example:"
    echo "  $0 30.123456 120.654321 90"
    exit 2
fi

LATITUDE="$1"
LONGITUDE="$2"
HEADING_DEG="$3"

# 与现有 fusion 脚本保持类似的数字格式检查
number_regex='^-?[0-9]+([.][0-9]+)?$'

if ! [[ "${LATITUDE}" =~ ${number_regex} ]]; then
    echo "[ERROR] invalid latitude: ${LATITUDE}" >&2
    exit 2
fi

if ! [[ "${LONGITUDE}" =~ ${number_regex} ]]; then
    echo "[ERROR] invalid longitude: ${LONGITUDE}" >&2
    exit 2
fi

if ! [[ "${HEADING_DEG}" =~ ${number_regex} ]]; then
    echo "[ERROR] invalid heading_deg: ${HEADING_DEG}" >&2
    exit 2
fi

# 经纬度范围检查
if ! awk -v v="${LATITUDE}" 'BEGIN { exit !(v >= -90 && v <= 90) }'; then
    echo "[ERROR] latitude must be in [-90, 90]" >&2
    exit 2
fi

if ! awk -v v="${LONGITUDE}" 'BEGIN { exit !(v >= -180 && v <= 180) }'; then
    echo "[ERROR] longitude must be in [-180, 180]" >&2
    exit 2
fi


# ============================================================
# 环境检查
# ============================================================

if [[ ! -r "${ROS_SETUP}" ]]; then
    echo "[ERROR] ROS setup not found: ${ROS_SETUP}" >&2
    exit 1
fi

if [[ ! -r "${WS_SETUP}" ]]; then
    echo "[ERROR] workspace setup not found: ${WS_SETUP}" >&2
    exit 1
fi

# 防止已经有 systemd bringup 在运行
if systemctl is-active --quiet uav-bringup.service; then
    echo "[ERROR] uav-bringup.service is already active." >&2
    echo "Stop it before manual startup." >&2
    exit 1
fi

set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PYTHONUNBUFFERED=1

cd "${WS_ROOT}"


# ============================================================
# 清理逻辑
# ============================================================

LAUNCH_PID=""
CLEANUP_DONE=0

cleanup() {
    if [[ "${CLEANUP_DONE}" == "1" ]]; then
        return
    fi

    CLEANUP_DONE=1
    set +e

    if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo
        echo "[SHUTDOWN] stopping UAV stack..."

        # ros2 launch 位于独立进程组中
        kill -INT -- "-${LAUNCH_PID}" 2>/dev/null \
            || kill -INT "${LAUNCH_PID}" 2>/dev/null \
            || true

        for _ in {1..50}; do
            kill -0 "${LAUNCH_PID}" 2>/dev/null || break
            sleep 0.1
        done

        if kill -0 "${LAUNCH_PID}" 2>/dev/null; then
            kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null \
                || kill -TERM "${LAUNCH_PID}" 2>/dev/null \
                || true
        fi
    fi
}

trap cleanup EXIT INT TERM


# ============================================================
# 显示本次任务
# ============================================================

echo "=========================================="
echo " Manual Target Mission"
echo "=========================================="
echo "latitude    = ${LATITUDE}"
echo "longitude   = ${LONGITUDE}"
echo "heading_deg = ${HEADING_DEG}"
echo "vision      = DISABLED"
echo "=========================================="


# ============================================================
# 启动完整 UAV bringup
#
# 注意：
# 不启动 recognition
# 不启动 recon
# 不启动 vision_target_bridge
# ============================================================

echo "[STARTUP] launching real UAV bringup..."

setsid ros2 launch uav_bringup real_bringup.launch.py &
LAUNCH_PID=$!

echo "[STARTUP] ros2 launch pid=${LAUNCH_PID}"


# ============================================================
# 等待 Mission Manager / FCU Interface / target topic 就绪
# ============================================================

echo "[STARTUP] waiting for mission system..."

deadline=$((SECONDS + 30))

while true; do

    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
        echo "[ERROR] real_bringup exited during startup" >&2
        wait "${LAUNCH_PID}" || true
        exit 1
    fi

    node_list="$(
        ros2 node list --no-daemon --spin-time 3.0 2>/dev/null || true
    )"

    target_info="$(
        ros2 topic info /vision/target_command 2>/dev/null || true
    )"

    target_subscribers="$(
        awk '/Subscription count:/ {print $3; exit}' <<<"${target_info}"
    )"
    target_subscribers="${target_subscribers:-0}"

    service_list="$(
        ros2 service list 2>/dev/null || true
    )"

    mission_ready=0
    fcu_ready=0
    goto_ready=0

    if grep -Fqx "/mission_manager_node" <<<"${node_list}"; then
        mission_ready=1
    fi

    if grep -Fqx "/fcu_interface_mavros_node" <<<"${node_list}"; then
        fcu_ready=1
    fi

    if grep -Fqx "/fcu/goto_global" <<<"${service_list}"; then
        goto_ready=1
    fi

    if [[ "${mission_ready}" == "1" ]] \
        && [[ "${fcu_ready}" == "1" ]] \
        && [[ "${goto_ready}" == "1" ]] \
        && (( target_subscribers >= 1 )); then

        break
    fi

    if (( SECONDS >= deadline )); then
        echo "[ERROR] mission system was not ready within 30 seconds" >&2
        echo "mission_manager=${mission_ready}" >&2
        echo "fcu_interface=${fcu_ready}" >&2
        echo "goto_global=${goto_ready}" >&2
        echo "target_subscribers=${target_subscribers}" >&2
        exit 1
    fi

    sleep 1
done


# ============================================================
# 发布人工指定目标
# ============================================================

echo "[TARGET] mission system ready"
echo "[TARGET] publishing target command..."

ros2 topic pub --once \
    --node-name manual_target_publisher_node \
    --keep-alive 2.0 \
    /vision/target_command \
    uav_interfaces/msg/TargetCommand \
    "{latitude: ${LATITUDE}, longitude: ${LONGITUDE}, heading_deg: ${HEADING_DEG}}"

echo
echo "=========================================="
echo " Target command sent"
echo "=========================================="
echo "latitude    = ${LATITUDE}"
echo "longitude   = ${LONGITUDE}"
echo "heading_deg = ${HEADING_DEG}"
echo "=========================================="
echo
echo "UAV stack is still running."
echo "Press Ctrl+C to stop."

# 保持启动脚本运行
wait "${LAUNCH_PID}"