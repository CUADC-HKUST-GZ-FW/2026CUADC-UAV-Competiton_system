#!/usr/bin/env bash
set -euo pipefail

readonly SELECTOR="${1:-all}"
readonly RUN_DIR="${UAV_DETACHED_RUN_DIR:-${HOME}/uav_flight_logs/run}"
readonly CURRENT_BOOT_ID="$(cat /proc/sys/kernel/random/boot_id)"

case "${SELECTOR}" in
    all|start_uav|start_single_target_fusion|start_competition_target_fusion|start_manual_target)
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
        echo "usage: $0 [all|uav|single|competition|manual]" >&2
        exit 2
        ;;
esac
readonly SELECTOR_NAME="${selector_name:-${SELECTOR}}"

shopt -s nullglob
state_files=("${RUN_DIR}"/*.state)
matched=0

for state_file in "${state_files[@]}"; do
    unset pid boot_id start_ticks script started_at log
    while IFS='=' read -r key value; do
        case "${key}" in
            pid) pid="${value}" ;;
            boot_id) boot_id="${value}" ;;
            start_ticks) start_ticks="${value}" ;;
            script) script="${value}" ;;
            started_at) started_at="${value}" ;;
            log) log="${value}" ;;
        esac
    done <"${state_file}"

    script_name="$(basename -- "${script:-unknown}")"
    script_name="${script_name%.sh}"
    if [[ "${SELECTOR_NAME}" != "all" && "${script_name}" != "${SELECTOR_NAME}" ]]; then
        continue
    fi
    matched=1

    state="stale"
    process_line=""
    if [[ "${pid:-}" =~ ^[0-9]+$ ]] \
        && [[ "${boot_id:-}" == "${CURRENT_BOOT_ID}" ]] \
        && [[ -r "/proc/${pid}/stat" ]] \
        && [[ "$(awk '{print $22}' "/proc/${pid}/stat" 2>/dev/null || true)" == "${start_ticks:-}" ]] \
        && kill -0 "${pid}" 2>/dev/null; then
        state="running"
        process_line="$(ps -o pid=,ppid=,sid=,pgid=,tty=,stat=,lstart=,cmd= -p "${pid}" 2>/dev/null || true)"
    fi

    echo "entry=${script_name} state=${state} pid=${pid:-unknown} started_at=${started_at:-unknown}"
    echo "state_file=${state_file}"
    echo "launcher_log=${log:-unknown}"
    if [[ -n "${process_line}" ]]; then
        echo "process=${process_line}"
    fi
done

if [[ "${matched}" == "0" ]]; then
    echo "no detached state found for selector=${SELECTOR_NAME}"
fi
