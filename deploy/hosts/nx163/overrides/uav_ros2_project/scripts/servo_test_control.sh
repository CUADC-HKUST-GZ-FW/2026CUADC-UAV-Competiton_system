#!/usr/bin/env bash

set -euo pipefail

readonly ROS_SETUP="/opt/ros/humble/setup.bash"
readonly WS_ROOT="/home/nx163/uav_ros2_project"
readonly WS_SETUP="${WS_ROOT}/install/setup.bash"
readonly ACTION="${1:-}"
readonly SERVO_CHANNEL="${2:-7}"
readonly SAFE_PWM=1350
readonly RELEASE_PWM=1900
readonly PWM_TOLERANCE=20
readonly SAMPLE_TIMEOUT_SEC=4
readonly CONFIRM_TIMEOUT_SEC=15
readonly CONTROL_LOG="${HOME}/uav_flight_logs/servo_open_test/manual_control.log"

usage() {
    cat <<'EOF'
Ground-only servo control through the MAVROS instance started by
start_servo_open_test.sh.

Usage:
  servo_test_control.sh open [channel]
  servo_test_control.sh close [channel]
  servo_test_control.sh status [channel]
  servo_test_control.sh --help

Defaults:
  channel=7, open=1900 us, close=1350 us, tolerance=20 us

The open action requires an interactive OPEN confirmation. This tool refuses
to move the servo while the FCU is armed. It never changes FCU parameters and
does not provide an automatic cycling mode.
EOF
}

fail() {
    echo "[GROUND_SERVO][ERROR] $*" >&2
    exit 1
}

warn() {
    echo "[GROUND_SERVO][WARN] $*" >&2
}

log_result() {
    local result="$1"
    local observed="${2:-unknown}"
    mkdir -p -- "$(dirname -- "${CONTROL_LOG}")"
    printf '%s action=%s channel=%s target_pwm=%s result=%s observed=%s\n' \
        "$(date -Is)" "${ACTION}" "${SERVO_CHANNEL}" "${TARGET_PWM:-none}" \
        "${result}" "${observed}" >>"${CONTROL_LOG}"
}

if [[ "${ACTION}" == "--help" || "${ACTION}" == "-h" ]]; then
    usage
    exit 0
fi
if [[ $# -gt 2 ]]; then
    usage >&2
    exit 2
fi
case "${ACTION}" in
    open|close|status) ;;
    *)
        usage >&2
        exit 2
        ;;
esac
if [[ ! "${SERVO_CHANNEL}" =~ ^([1-9]|1[0-6])$ ]]; then
    echo "[GROUND_SERVO][ERROR] channel must be an integer from 1 to 16" >&2
    exit 2
fi
if [[ ! -r "${ROS_SETUP}" ]]; then
    fail "ROS setup not found: ${ROS_SETUP}"
fi
if [[ ! -r "${WS_SETUP}" ]]; then
    fail "workspace setup not found: ${WS_SETUP}"
fi

set +u
source "${ROS_SETUP}"
source "${WS_SETUP}"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PYTHONUNBUFFERED=1

for command in ros2 timeout python3; do
    command -v "${command}" >/dev/null 2>&1 || fail "required command not found: ${command}"
done

node_snapshot="$(ros2 node list 2>/dev/null || true)"
logger_running=false
if grep -Fxq '/servo_open_logger_node' <<<"${node_snapshot}"; then
    logger_running=true
fi

# MAVROS 2 runs as several component nodes under /mavros rather than as one
# node named exactly /mavros.  Check the telemetry interfaces this tool needs
# instead of relying on a distribution-specific node name.
topic_snapshot="$(ros2 topic list 2>/dev/null || true)"
if ! grep -Fxq '/mavros/state' <<<"${topic_snapshot}"; then
    fail "MAVROS state topic is unavailable; start the standalone servo test first"
fi
if ! grep -Fxq '/mavros/rc/out' <<<"${topic_snapshot}"; then
    fail "MAVROS RC output topic is unavailable; start the standalone servo test first"
fi

read_state() {
    local state_output
    if ! state_output="$(
        timeout "${SAMPLE_TIMEOUT_SEC}s" \
            ros2 topic echo --once /mavros/state mavros_msgs/msg/State \
            2>/dev/null
    )"; then
        fail "no /mavros/state message received"
    fi
    if grep -Eq '^connected:[[:space:]]+true$' <<<"${state_output}"; then
        FCU_CONNECTED=true
    else
        FCU_CONNECTED=false
    fi
    if grep -Eq '^armed:[[:space:]]+true$' <<<"${state_output}"; then
        FCU_ARMED=true
    else
        FCU_ARMED=false
    fi
}

read_pwm() {
    local rc_output
    if ! rc_output="$(
        timeout "${SAMPLE_TIMEOUT_SEC}s" \
            ros2 topic echo --once /mavros/rc/out mavros_msgs/msg/RCOut \
            2>/dev/null
    )"; then
        return 1
    fi
    printf '%s\n' "${rc_output}" | python3 -c '
import re
import sys

channel = int(sys.argv[1])
text = sys.stdin.read()
if "channels:" not in text:
    raise SystemExit(1)
payload = text.split("channels:", 1)[1].split("\n---", 1)[0]
values = [int(value) for value in re.findall(r"-?\d+", payload)]
if channel < 1 or channel > len(values):
    raise SystemExit(1)
print(values[channel - 1])
' "${SERVO_CHANNEL}"
}

classify_pwm() {
    local pwm="$1"
    local safe_delta=$((pwm - SAFE_PWM))
    local release_delta=$((pwm - RELEASE_PWM))
    (( safe_delta < 0 )) && safe_delta=$((-safe_delta))
    (( release_delta < 0 )) && release_delta=$((-release_delta))
    if (( safe_delta <= PWM_TOLERANCE )); then
        echo CLOSED
    elif (( release_delta <= PWM_TOLERANCE )); then
        echo OPEN
    else
        echo INTERMEDIATE
    fi
}

read_state

if [[ "${ACTION}" == "status" ]]; then
    echo "FCU connected: ${FCU_CONNECTED}"
    echo "FCU armed: ${FCU_ARMED}"
    echo "Servo logger: ${logger_running}"
    echo "Channel: ${SERVO_CHANNEL}"
    if pwm="$(read_pwm)"; then
        echo "Observed PWM: ${pwm}"
        echo "Expected close PWM: ${SAFE_PWM}"
        echo "Expected open PWM: ${RELEASE_PWM}"
        echo "State: $(classify_pwm "${pwm}")"
        log_result status "${pwm}"
        exit 0
    fi
    echo "Observed PWM: unavailable"
    echo "State: UNAVAILABLE"
    log_result unavailable
    exit 1
fi

if [[ "${FCU_CONNECTED}" != "true" ]]; then
    fail "FCU is not connected"
fi
if [[ "${FCU_ARMED}" == "true" ]]; then
    fail "FCU is armed; ground servo control is prohibited"
fi

if [[ "${ACTION}" == "open" && "${logger_running}" != "true" ]]; then
    fail "standalone servo logger is not running"
fi
if [[ "${ACTION}" == "close" && "${logger_running}" != "true" ]]; then
    warn "standalone servo logger is not running; allowing close for safe recovery"
fi

service_type="$(ros2 service type /mavros/cmd/command 2>/dev/null || true)"
if [[ "${service_type}" != "mavros_msgs/srv/CommandLong" ]]; then
    fail "/mavros/cmd/command is unavailable or has unexpected type: ${service_type:-none}"
fi

if [[ "${ACTION}" == "open" ]]; then
    readonly TARGET_PWM="${RELEASE_PWM}"
    if [[ ! -t 0 || ! -t 1 ]]; then
        fail "open requires an interactive terminal confirmation"
    fi
    echo "GROUND TEST ONLY"
    echo "Aircraft must be disarmed. Remove the propeller or disconnect propulsion power."
    echo "Keep hands clear of the servo and release mechanism."
    read -r -p "Type OPEN to command channel ${SERVO_CHANNEL} to ${TARGET_PWM} us: " answer
    if [[ "${answer}" != "OPEN" ]]; then
        echo "[GROUND_SERVO] cancelled"
        exit 3
    fi
else
    readonly TARGET_PWM="${SAFE_PWM}"
fi

echo "[GROUND_SERVO] sending action=${ACTION} channel=${SERVO_CHANNEL} target=${TARGET_PWM}"
request="{broadcast: false, command: 183, confirmation: 0, param1: ${SERVO_CHANNEL}.0, param2: ${TARGET_PWM}.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
if ! response="$(
    timeout 10s ros2 service call \
        /mavros/cmd/command mavros_msgs/srv/CommandLong "${request}" 2>&1
)"; then
    log_result command_failed
    fail "CommandLong call failed: ${response}"
fi
echo "${response}"
if ! grep -Eiq 'success[=:][[:space:]]*true' <<<"${response}"; then
    log_result command_rejected
    fail "flight controller did not accept MAV_CMD_DO_SET_SERVO"
fi

deadline=$((SECONDS + CONFIRM_TIMEOUT_SEC))
consecutive=0
observations=()
while (( SECONDS < deadline )); do
    if pwm="$(read_pwm)"; then
        observations+=("${pwm}")
        if (( ${#observations[@]} > 12 )); then
            observations=("${observations[@]: -12}")
        fi
        delta=$((pwm - TARGET_PWM))
        (( delta < 0 )) && delta=$((-delta))
        if (( delta <= PWM_TOLERANCE )); then
            consecutive=$((consecutive + 1))
            if (( consecutive >= 3 )); then
                echo "[GROUND_SERVO][OK] action=${ACTION} channel=${SERVO_CHANNEL} target=${TARGET_PWM}"
                echo "[GROUND_SERVO][OK] recent_observations=${observations[*]}"
                echo "[GROUND_SERVO][OK] FCU output confirmed; visually verify mechanical movement"
                log_result confirmed "${observations[*]}"
                exit 0
            fi
        else
            consecutive=0
        fi
    fi
done

log_result output_unconfirmed "${observations[*]:-none}"
fail "command accepted but three matching PWM frames were not confirmed; recent=${observations[*]:-none}"
