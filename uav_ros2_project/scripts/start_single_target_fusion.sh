#!/usr/bin/env bash
set -Eeuo pipefail

readonly UAV_ENTRY_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly UAV_SSH_DETACH_HELPER="${UAV_ENTRY_SCRIPT_DIR}/lib/ssh_detach.sh"

if [[ -r "${UAV_SSH_DETACH_HELPER}" ]]; then
    source "${UAV_SSH_DETACH_HELPER}"
elif [[ -n "${SSH_CONNECTION:-}" || -n "${SSH_TTY:-}" ]] \
    && [[ "${UAV_FOREGROUND:-0}" != "1" ]]; then
    echo "[DETACHED][ERROR] missing helper: ${UAV_SSH_DETACH_HELPER}" >&2
    exit 1
fi

readonly ROOT="/home/nx163/youth-vision-runtime"
readonly ROS_WS="/home/nx163/uav_ros2_project"
readonly ROS_SETUP="/opt/ros/humble/setup.bash"
readonly WS_SETUP="${ROS_WS}/install/setup.bash"
readonly MODE="${1:-}"
readonly HEADING_INPUT="${2:-}"
readonly RUN_DIR="${ROOT}/run"
readonly LOG_DIR="${ROOT}/logs/recon"
readonly LOCK_FILE="${RUN_DIR}/single_target_fusion.lock"
readonly LOCK_OWNER="${RUN_DIR}/single_target_fusion.lock.owner"

FAILURE_REASON="startup_not_completed"
LOCK_ACQUIRED=0
CLEANUP_STARTED=0
LOCK_FD=""

declare -a CHILD_PIDS=()
declare -a CHILD_NAMES=()
declare -a CHILD_PIDFILES=()

fail() {
    FAILURE_REASON="$*"
    echo "ERROR: ${FAILURE_REASON}" >&2
    exit 1
}

process_start_ticks() {
    local pid="$1"
    awk '{print $22}' "/proc/${pid}/stat" 2>/dev/null || true
}

acquire_singleton_lock() {
    local owner_description="metadata_unavailable"

    exec {LOCK_FD}>"${LOCK_FILE}"
    if ! flock --nonblock "${LOCK_FD}"; then
        if [[ -s "${LOCK_OWNER}" ]]; then
            owner_description="$(tr '\n' ' ' <"${LOCK_OWNER}")"
        fi
        echo "single-target fusion is already running: ${owner_description}" >&2
        exec {LOCK_FD}>&-
        LOCK_FD=""
        return 73
    fi

    LOCK_ACQUIRED=1
    printf 'pid=%s boot_id=%s start_ticks=%s script=%s session=pending\n' \
        "$$" \
        "$(cat /proc/sys/kernel/random/boot_id)" \
        "$(process_start_ticks "$$")" \
        "$0" \
        >"${LOCK_OWNER}"
    return 0
}

child_group_alive() {
    local pid="$1"
    kill -0 -- "-${pid}" 2>/dev/null || kill -0 "${pid}" 2>/dev/null
}

signal_child_group() {
    local signal_name="$1"
    local pid="$2"
    kill -s "${signal_name}" -- "-${pid}" 2>/dev/null \
        || kill -s "${signal_name}" "${pid}" 2>/dev/null \
        || true
}

register_child() {
    local pid="$1"
    local name="$2"
    local pidfile="$3"
    CHILD_PIDS+=("${pid}")
    CHILD_NAMES+=("${name}")
    CHILD_PIDFILES+=("${pidfile}")
    printf '%s\n' "${pid}" >"${pidfile}"
    echo "child_started name=${name} pid=${pid} pidfile=${pidfile}"
}

cleanup() {
    local exit_code=$?
    local alive index pid owner_pid
    if [[ "${CLEANUP_STARTED}" == "1" ]]; then
        return
    fi
    CLEANUP_STARTED=1
    trap - EXIT ERR INT TERM HUP
    set +e

    echo "cleanup_started reason=${FAILURE_REASON} exit_code=${exit_code}"

    for ((index=${#CHILD_PIDS[@]} - 1; index >= 0; index--)); do
        pid="${CHILD_PIDS[index]}"
        if child_group_alive "${pid}"; then
            echo "child_signal name=${CHILD_NAMES[index]} pid=${pid} signal=INT"
            signal_child_group INT "${pid}"
        fi
    done

    for _ in {1..100}; do
        alive=0
        for pid in "${CHILD_PIDS[@]}"; do
            if child_group_alive "${pid}"; then
                alive=1
                break
            fi
        done
        [[ "${alive}" == "0" ]] && break
        sleep 0.1
    done

    for ((index=${#CHILD_PIDS[@]} - 1; index >= 0; index--)); do
        pid="${CHILD_PIDS[index]}"
        if child_group_alive "${pid}"; then
            echo "child_signal name=${CHILD_NAMES[index]} pid=${pid} signal=TERM"
            signal_child_group TERM "${pid}"
        fi
    done

    for _ in {1..50}; do
        alive=0
        for pid in "${CHILD_PIDS[@]}"; do
            if child_group_alive "${pid}"; then
                alive=1
                break
            fi
        done
        [[ "${alive}" == "0" ]] && break
        sleep 0.1
    done

    for ((index=${#CHILD_PIDS[@]} - 1; index >= 0; index--)); do
        pid="${CHILD_PIDS[index]}"
        if child_group_alive "${pid}"; then
            echo "child_signal name=${CHILD_NAMES[index]} pid=${pid} signal=KILL"
            signal_child_group KILL "${pid}"
        fi
    done

    for index in "${!CHILD_PIDFILES[@]}"; do
        rm -f -- "${CHILD_PIDFILES[index]}"
    done

    if [[ "${LOCK_ACQUIRED}" == "1" ]]; then
        owner_pid="$(sed -n 's/^pid=\([0-9][0-9]*\).*/\1/p' "${LOCK_OWNER}" 2>/dev/null)"
        if [[ "${owner_pid}" == "$$" ]]; then
            rm -f -- "${LOCK_OWNER}"
        fi
        flock --unlock "${LOCK_FD}" 2>/dev/null || true
        exec {LOCK_FD}>&-
        LOCK_FD=""
        LOCK_ACQUIRED=0
    fi

    echo "cleanup_completed reason=${FAILURE_REASON}"
}

on_error() {
    local exit_code="$1"
    local line_number="$2"
    local command="$3"
    if [[ "${FAILURE_REASON}" == "startup_not_completed" ]] \
        || [[ "${FAILURE_REASON}" == "running" ]]; then
        FAILURE_REASON="command_failed rc=${exit_code} line=${line_number} command=${command}"
    fi
}

node_list_snapshot() {
    # A fresh non-daemon ROS CLI process needs enough time to discover the
    # complete DDS graph on the Jetson.  The default 0.1 s window can see
    # topics while still missing their nodes and causes intermittent false
    # startup failures.
    ros2 node list --no-daemon --spin-time 3.0 2>/dev/null || true
}

node_count_in_snapshot() {
    local snapshot="$1"
    local node_name="$2"
    grep -Fxc -- "${node_name}" <<<"${snapshot}" || true
}

assert_no_residual_processes() {
    local pattern description matches
    while IFS='|' read -r description pattern; do
        matches="$(
            ps -eo pid=,comm=,args= \
                | awk -v pattern="${pattern}" \
                    '$2 !~ /^(bash|sh|ssh|sshd|grep|pgrep|awk|sed|timeout)$/ \
                    && index($0, pattern) { print }'
        )"
        if [[ -n "${matches}" ]]; then
            echo "${matches}" >&2
            fail "residual ${description} process is already running"
        fi
    done <<'EOF'
MAVROS|/opt/ros/humble/lib/mavros/mavros_node
flight bringup|ros2 launch uav_bringup real_bringup.launch.py
FCU interface|/uav_fcu_interface/fcu_interface_mavros_node
vision runner|/native/build/youth_vision_runner
recon geolocator|/uav_recon/recon_geolocator_node
vision target bridge|/uav_vision_bridge/vision_target_bridge_node
mission manager|/uav_mission_manager/mission_manager_node
EOF
}

if [[ "$#" -ne 2 ]] || {
    [[ "${MODE}" != "digit" ]] && [[ "${MODE}" != "image" ]];
}; then
    echo "usage: $0 [digit|image] [heading_deg]" >&2
    exit 2
fi
if ! [[ "${HEADING_INPUT}" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
    echo "heading_deg must be a finite number" >&2
    exit 2
fi

# Force integer-looking input such as "0" to a finite ROS double parameter.
heading_result=0
HEADING_DEG="$(
    LC_NUMERIC=C awk -v value="${HEADING_INPUT}" '
        BEGIN {
            number = value + 0.0
            if (number > 1.0e308 || number < -1.0e308) {
                exit 1
            }
            printf "%.6f", number
        }
    '
)" || heading_result=$?
if [[ "${heading_result}" != "0" ]]; then
    echo "heading_deg is outside the finite numeric range" >&2
    exit 2
fi
readonly HEADING_DEG

if declare -F uav_maybe_detach_from_ssh >/dev/null; then
    detach_rc=0
    uav_maybe_detach_from_ssh "$0" "$@" || detach_rc=$?
    if [[ "${detach_rc}" == "200" ]]; then
        exit 0
    elif [[ "${detach_rc}" != "0" ]]; then
        exit "${detach_rc}"
    fi
fi

mkdir -p \
    "${RUN_DIR}" \
    "${LOG_DIR}" \
    "${ROOT}/logs/recognition" \
    "${ROOT}/recon_results/sessions"

lock_result=0
acquire_singleton_lock || lock_result=$?
if [[ "${lock_result}" != "0" ]]; then
    exit "${lock_result}"
fi

trap cleanup EXIT
trap 'on_error "$?" "$LINENO" "$BASH_COMMAND"' ERR
trap 'FAILURE_REASON="received_signal_INT"; exit 130' INT
trap 'FAILURE_REASON="received_signal_TERM"; exit 143' TERM
trap 'FAILURE_REASON="received_signal_HUP"; exit 129' HUP

readonly STAMP="$(date +%Y%m%d_%H%M%S)"
readonly SESSION_ID="${STAMP}_${MODE}_fusion"
readonly RESULT_DIR="${ROOT}/recon_results/sessions/${SESSION_ID}"
readonly FLIGHT_LOG="${LOG_DIR}/flight_${SESSION_ID}.log"
readonly VISION_LOG="${LOG_DIR}/vision_${SESSION_ID}.log"
readonly RECON_LOG="${LOG_DIR}/ros_${SESSION_ID}.log"
readonly BRIDGE_LOG="${LOG_DIR}/single_target_fusion_bridge_${STAMP}.log"
readonly EVENT_LOG="${ROOT}/logs/recognition/recognition_events_${SESSION_ID}.jsonl"
readonly ORCHESTRATOR_LOG="${LOG_DIR}/orchestrator_${SESSION_ID}.log"

exec > >(
    trap '' INT TERM HUP
    exec tee -a "${ORCHESTRATOR_LOG}"
) 2>&1

printf 'pid=%s boot_id=%s start_ticks=%s script=%s session=%s\n' \
    "$$" \
    "$(cat /proc/sys/kernel/random/boot_id)" \
    "$(process_start_ticks "$$")" \
    "$0" \
    "${SESSION_ID}" \
    >"${LOCK_OWNER}"

echo "startup_begin timestamp=$(date -Is) session_id=${SESSION_ID} mode=${MODE} heading_deg=${HEADING_DEG}"

for required in \
    "${ROS_SETUP}" \
    "${WS_SETUP}" \
    "${ROOT}/configs/youth_pipeline.yaml" \
    "${ROOT}/native/build/youth_vision_runner"; do
    if [[ ! -e "${required}" ]]; then
        fail "required path is missing: ${required}"
    fi
done

if systemctl is-active --quiet uav-bringup.service; then
    fail "uav-bringup.service is active; stop it before manual startup"
fi

assert_no_residual_processes

if ss -H -lunp 2>/dev/null | awk '$4 ~ /:15001$/ { found=1 } END { exit !found }'; then
    fail "UDP port 15001 is already in use"
fi

set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PYTHONUNBUFFERED=1

startup_node_snapshot="$(node_list_snapshot)"
if [[ "$(node_count_in_snapshot "${startup_node_snapshot}" /mission_manager_node)" != "0" ]]; then
    fail "ROS graph already contains /mission_manager_node"
fi
if [[ "$(node_count_in_snapshot "${startup_node_snapshot}" /recon_geolocator)" != "0" ]]; then
    fail "ROS graph already contains /recon_geolocator"
fi
if [[ "$(node_count_in_snapshot "${startup_node_snapshot}" /vision_target_bridge_node)" != "0" ]]; then
    fail "ROS graph already contains /vision_target_bridge_node"
fi
if [[ "$(node_count_in_snapshot "${startup_node_snapshot}" /fcu_interface_mavros_node)" != "0" ]]; then
    fail "ROS graph already contains /fcu_interface_mavros_node"
fi

setsid bash -lc \
    "source '${ROS_SETUP}'; source '${WS_SETUP}'; exec ros2 launch uav_bringup real_bringup.launch.py dry_run:=false allow_mission_upload:=true allow_mode_change:=true enable_real_payload_release:=true" \
    >"${FLIGHT_LOG}" 2>&1 </dev/null {LOCK_FD}>&- &
flight_pid=$!
register_child "${flight_pid}" "flight_bringup" "${RUN_DIR}/manual_flight_bringup.pid"

deadline=$((SECONDS + 30))
while true; do
    flight_node_snapshot="$(node_list_snapshot)"
    mission_manager_count="$(
        node_count_in_snapshot "${flight_node_snapshot}" /mission_manager_node
    )"
    if ((mission_manager_count > 1)); then
        fail "duplicate /mission_manager_node instances detected count=${mission_manager_count}"
    fi
    if [[ "${mission_manager_count}" == "1" ]]; then
        break
    fi
    if ! child_group_alive "${flight_pid}"; then
        tail -40 "${FLIGHT_LOG}" >&2 || true
        fail "manual flight bringup exited during startup"
    fi
    if ((SECONDS >= deadline)); then
        tail -40 "${FLIGHT_LOG}" >&2 || true
        fail "mission_manager_node was not ready within 30 seconds"
    fi
    sleep 1
done

mavros_count="$(pgrep -xc mavros_node 2>/dev/null || true)"
if [[ "${mavros_count}" != "1" ]]; then
    fail "expected exactly one MAVROS process, found ${mavros_count}"
fi

mkdir -p "${RESULT_DIR}"
echo "${MODE}" >"${ROOT}/configs/youth_runtime_mode.txt"
ln -sfn "sessions/${SESSION_ID}" "${ROOT}/recon_results/latest"

# Preserve untouched camera frames for later review and training. Recording is
# bounded and is skipped when the configured output volume has less than 2 GiB.
if [[ "${YOUTH_SAVE_RAW_VIDEO:-1}" == "1" && -z "${YOUTH_RECORD_FILE:-}" ]]; then
    raw_dir="${YOUTH_RAW_VIDEO_DIR:-/home/nx163/camera_recordings}"
    mkdir -p "${raw_dir}"
    available_kb="$(df -Pk "${raw_dir}" | awk 'NR==2 {print $4}')"
    if [[ -n "${available_kb}" && "${available_kb}" -ge "${YOUTH_RAW_MIN_FREE_KB:-2097152}" ]]; then
        export YOUTH_RECORD_FILE="${raw_dir}/camera_${STAMP}_1440x1080_${MODE}_single_fusion_raw_no_overlay.mp4"
        export YOUTH_RECORD_DURATION_SEC="${YOUTH_RECORD_DURATION_SEC:-600}"
        export YOUTH_RECORD_FPS="${YOUTH_RECORD_FPS:-60}"
        export YOUTH_RECORD_BITRATE_KBPS="${YOUTH_RECORD_BITRATE_KBPS:-12000}"
    else
        echo "warning: raw recording disabled because ${raw_dir} has insufficient free space" >&2
    fi
fi

vision_args=(
    --config "${ROOT}/configs/youth_pipeline.yaml"
    --class-mode "${MODE}"
)
if [[ -n "${YOUTH_RECORD_FILE:-}" ]]; then
    vision_args+=(
        --record-file "${YOUTH_RECORD_FILE}"
        --record-duration-sec "${YOUTH_RECORD_DURATION_SEC:-600}"
        --record-fps "${YOUTH_RECORD_FPS:-60}"
        --record-bitrate-kbps "${YOUTH_RECORD_BITRATE_KBPS:-12000}"
    )
fi

sudo -n /usr/sbin/nvpmodel -m 0 >/dev/null
sudo -n /usr/bin/jetson_clocks >/dev/null

# The manifest is a one-frame handoff, not persistent reconnaissance history.
# Remove the previous process's final frame before the new producer starts.
startup_manifest="${ROOT}/overlays/latest_crops/manifest_fast.json"
rm -f -- "${startup_manifest}"
echo "startup_manifest_fence=armed path=${startup_manifest}"

setsid env \
    YOUTH_SESSION_ID="${SESSION_ID}" \
    YOUTH_RECOGNITION_LOG="${EVENT_LOG}" \
    LD_LIBRARY_PATH="/opt/MVS/lib/aarch64:/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/nvidia" \
    "${ROOT}/native/build/youth_vision_runner" \
    "${vision_args[@]}" \
    >"${VISION_LOG}" 2>&1 </dev/null {LOCK_FD}>&- &
vision_pid=$!
register_child "${vision_pid}" "vision_runner" "${RUN_DIR}/recon_vision.pid"

setsid bash -lc \
    "source '${ROS_SETUP}'; source '${WS_SETUP}'; exec ros2 launch uav_recon recon.launch.py output_root:='${RESULT_DIR}' minimum_gps_fix_type:=6" \
    >"${RECON_LOG}" 2>&1 </dev/null {LOCK_FD}>&- &
recon_pid=$!
register_child "${recon_pid}" "recon_geolocator" "${RUN_DIR}/recon_ros.pid"

setsid bash -lc \
    "source '${ROS_SETUP}'; source '${WS_SETUP}'; exec ros2 run uav_vision_bridge vision_target_bridge_node --ros-args -p heading_deg:='${HEADING_DEG}' -p auto_execute:=true -p target_selection_candidate_count:=2 -p candidate_collection_timeout_sec:=10.0" \
    >"${BRIDGE_LOG}" 2>&1 </dev/null {LOCK_FD}>&- &
bridge_pid=$!
register_child "${bridge_pid}" "vision_target_bridge" "${RUN_DIR}/single_target_fusion_bridge.pid"

sleep 3
for index in "${!CHILD_PIDS[@]}"; do
    if ! child_group_alive "${CHILD_PIDS[index]}"; then
        fail "${CHILD_NAMES[index]} exited during startup"
    fi
done

deadline=$((SECONDS + 30))
while true; do
    graph_node_snapshot="$(node_list_snapshot)"
    recon_node_count="$(
        node_count_in_snapshot "${graph_node_snapshot}" /recon_geolocator
    )"
    bridge_node_count="$(
        node_count_in_snapshot "${graph_node_snapshot}" /vision_target_bridge_node
    )"
    mission_manager_count="$(
        node_count_in_snapshot "${graph_node_snapshot}" /mission_manager_node
    )"
    fcu_interface_count="$(
        node_count_in_snapshot "${graph_node_snapshot}" /fcu_interface_mavros_node
    )"

    if ((recon_node_count > 1 || bridge_node_count > 1 \
        || mission_manager_count > 1 || fcu_interface_count > 1)); then
        fail "duplicate ROS nodes detected mission_manager=${mission_manager_count} fcu_interface=${fcu_interface_count} recon=${recon_node_count} bridge=${bridge_node_count}"
    fi

    recon_info="$(ros2 topic info /vision/recon_result 2>/dev/null || true)"
    target_info="$(ros2 topic info /vision/target_command 2>/dev/null || true)"
    recon_publishers="$(awk '/Publisher count:/ {print $3; exit}' <<<"${recon_info}")"
    recon_subscribers="$(awk '/Subscription count:/ {print $3; exit}' <<<"${recon_info}")"
    target_publishers="$(awk '/Publisher count:/ {print $3; exit}' <<<"${target_info}")"
    target_subscribers="$(awk '/Subscription count:/ {print $3; exit}' <<<"${target_info}")"
    recon_publishers="${recon_publishers:-0}"
    recon_subscribers="${recon_subscribers:-0}"
    target_publishers="${target_publishers:-0}"
    target_subscribers="${target_subscribers:-0}"

    # Each command topic must have one authoritative publisher. Subscribers
    # may legitimately fan out to required consumers, rosbag and diagnostics.
    if ((recon_publishers > 1 || target_publishers > 1)); then
        fail "duplicate ROS publishers detected recon_pub=${recon_publishers} recon_sub=${recon_subscribers} target_pub=${target_publishers} target_sub=${target_subscribers}"
    fi

    service_list="$(ros2 service list 2>/dev/null || true)"
    for legacy_service in \
        /mission/enable \
        /mission/confirm_target; do
        if grep -Fqx -- "${legacy_service}" <<<"${service_list}"; then
            fail "legacy mission-manager service detected: ${legacy_service}"
        fi
    done

    services_ready=1
    for required_service in \
        /mission/disable \
        /fcu/goto_global; do
        if ! grep -Fqx -- "${required_service}" <<<"${service_list}"; then
            services_ready=0
        fi
    done

    if [[ "${recon_node_count}" == "1" ]] \
        && [[ "${bridge_node_count}" == "1" ]] \
        && [[ "${mission_manager_count}" == "1" ]] \
        && [[ "${fcu_interface_count}" == "1" ]] \
        && [[ "${recon_publishers}" == "1" ]] \
        && [[ "${recon_subscribers}" -ge 1 ]] \
        && [[ "${target_publishers}" == "1" ]] \
        && [[ "${target_subscribers}" -ge 1 ]] \
        && [[ "${services_ready}" == "1" ]]; then
        break
    fi

    for index in "${!CHILD_PIDS[@]}"; do
        if ! child_group_alive "${CHILD_PIDS[index]}"; then
            fail "${CHILD_NAMES[index]} exited while waiting for the ROS graph"
        fi
    done

    if ((SECONDS >= deadline)); then
        fail "vision-to-flight ROS graph was not ready within 30 seconds nodes=${mission_manager_count}/${fcu_interface_count}/${recon_node_count}/${bridge_node_count} recon=${recon_publishers}/${recon_subscribers} target=${target_publishers}/${target_subscribers} services_ready=${services_ready}"
    fi
    sleep 1
done

FAILURE_REASON="running"
echo "single-target fusion started"
echo "mode=${MODE}"
echo "heading_deg=${HEADING_DEG}"
echo "session_id=${SESSION_ID}"
echo "mavros_count=${mavros_count}"
echo "orchestrator_log=${ORCHESTRATOR_LOG}"
echo "flight_log=${FLIGHT_LOG}"
echo "vision_log=${VISION_LOG}"
echo "recon_log=${RECON_LOG}"
echo "bridge_log=${BRIDGE_LOG}"
if [[ -n "${YOUTH_RECORD_FILE:-}" ]]; then
    echo "record_file=${YOUTH_RECORD_FILE}"
    echo "record_duration_sec=${YOUTH_RECORD_DURATION_SEC:-600}"
    echo "record_content=raw_camera_frames_without_model_overlay"
fi
if [[ "${UAV_DETACHED:-0}" == "1" ]]; then
    echo "stop_command=${UAV_ENTRY_SCRIPT_DIR}/stop_detached_uav.sh single"
else
    echo "press Ctrl+C to stop all processes started by this command"
fi

while true; do
    for index in "${!CHILD_PIDS[@]}"; do
        if ! child_group_alive "${CHILD_PIDS[index]}"; then
            fail "${CHILD_NAMES[index]} exited; stopping the fusion stack"
        fi
    done
    mavros_count="$(pgrep -xc mavros_node 2>/dev/null || true)"
    if [[ "${mavros_count}" != "1" ]]; then
        fail "runtime MAVROS instance count changed to ${mavros_count}"
    fi
    sleep 1
done
