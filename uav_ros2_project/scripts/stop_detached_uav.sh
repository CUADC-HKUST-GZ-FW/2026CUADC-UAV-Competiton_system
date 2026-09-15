#!/usr/bin/env bash
set -euo pipefail

readonly SELECTOR="${1:-}"
readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"
readonly RUN_DIR="${UAV_DETACHED_RUN_DIR:-${HOME}/uav_flight_logs/run}"
readonly CURRENT_BOOT_ID="$(cat /proc/sys/kernel/random/boot_id)"

case "${SELECTOR}" in
    all|start_uav|start_single_target_fusion|start_competition_target_fusion|start_manual_target)
        selector_name="${SELECTOR}"
        ;;
    uav)
        selector_name="start_uav"
        ;;
    single)
        selector_name="start_single_target_fusion"
        ;;
    competition)
        selector_name="start_competition_target_fusion"
        ;;
    manual)
        selector_name="start_manual_target"
        ;;
    *)
        echo "usage: $0 <uav|single|competition|manual|all>" >&2
        exit 2
        ;;
esac
readonly SELECTOR_NAME="${selector_name}"

shopt -s nullglob
state_files=("${RUN_DIR}"/*.state)
declare -a ACTIVE_PIDS=()
declare -a ACTIVE_STATES=()

for state_file in "${state_files[@]}"; do
    unset pid boot_id start_ticks script
    while IFS='=' read -r key value; do
        case "${key}" in
            pid) pid="${value}" ;;
            boot_id) boot_id="${value}" ;;
            start_ticks) start_ticks="${value}" ;;
            script) script="${value}" ;;
        esac
    done <"${state_file}"

    script_path="${script:-}"
    script_name="$(basename -- "${script_path:-unknown}")"
    script_name="${script_name%.sh}"
    if [[ "${SELECTOR_NAME}" != "all" && "${script_name}" != "${SELECTOR_NAME}" ]]; then
        continue
    fi
    case "${script_path}" in
        "${REPO_ROOT}/scripts/start_uav.sh"|\
        "${REPO_ROOT}/scripts/start_single_target_fusion.sh"|\
        "${REPO_ROOT}/scripts/start_competition_target_fusion.sh"|\
        "${REPO_ROOT}/scripts/start_manual_target.sh")
            ;;
        *)
            echo "refusing untrusted state file: ${state_file}" >&2
            continue
            ;;
    esac

    if [[ ! "${pid:-}" =~ ^[0-9]+$ ]] \
        || [[ "${boot_id:-}" != "${CURRENT_BOOT_ID}" ]] \
        || [[ ! -r "/proc/${pid}/stat" ]] \
        || [[ "$(awk '{print $22}' "/proc/${pid}/stat" 2>/dev/null || true)" != "${start_ticks:-}" ]] \
        || ! kill -0 "${pid}" 2>/dev/null; then
        echo "stale state ignored: ${state_file}"
        continue
    fi

    ACTIVE_PIDS+=("${pid}")
    ACTIVE_STATES+=("${state_file}")
    echo "sending SIGINT entry=${script_name} pid=${pid}"
    kill -INT "${pid}"
done

if [[ "${#ACTIVE_PIDS[@]}" == "0" ]]; then
    echo "no validated detached process found for selector=${SELECTOR_NAME}"
    exit 3
fi

result=0
for index in "${!ACTIVE_PIDS[@]}"; do
    pid="${ACTIVE_PIDS[index]}"
    stopped=0
    for _ in {1..300}; do
        if ! kill -0 "${pid}" 2>/dev/null; then
            stopped=1
            break
        fi
        sleep 0.1
    done
    if [[ "${stopped}" == "1" ]]; then
        rm -f -- "${ACTIVE_STATES[index]}"
        echo "stopped pid=${pid}"
    else
        echo "process did not stop within 30 seconds; no force signal sent pid=${pid}" >&2
        result=1
    fi
done

exit "${result}"
