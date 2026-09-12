#!/usr/bin/env bash

# Shared SSH-only detachment for real-flight entry points. The caller handles
# return code 200 by exiting after the detached child has been accepted.

uav_detached_process_start_ticks() {
    local pid="$1"
    awk '{print $22}' "/proc/${pid}/stat" 2>/dev/null || true
}

uav_write_detached_state() {
    local entrypoint="$1"
    local run_dir="$2"
    local base state_file temp_file

    base="$(basename -- "${entrypoint}")"
    base="${base%.sh}"
    state_file="${run_dir}/${base}.$$.state"
    temp_file="${state_file}.tmp"

    if ! mkdir -p -- "${run_dir}"; then
        echo "[DETACHED][ERROR] cannot create state directory: ${run_dir}" >&2
        return 1
    fi

    if ! printf 'schema=1\npid=%s\nppid=%s\nboot_id=%s\nstart_ticks=%s\nscript=%s\nstarted_at=%s\nlog=%s\n' \
        "$$" \
        "${PPID}" \
        "$(cat /proc/sys/kernel/random/boot_id)" \
        "$(uav_detached_process_start_ticks "$$")" \
        "${entrypoint}" \
        "$(date -Is)" \
        "${UAV_DETACHED_LAUNCH_LOG:-}" \
        >"${temp_file}"; then
        echo "[DETACHED][ERROR] cannot write state file: ${temp_file}" >&2
        return 1
    fi

    chmod 600 "${temp_file}" 2>/dev/null || true
    mv -f -- "${temp_file}" "${state_file}"
    export UAV_DETACHED_STATE_FILE="${state_file}"
}

uav_maybe_detach_from_ssh() {
    local entrypoint="$1"
    shift

    entrypoint="$(readlink -f -- "${entrypoint}")"
    if [[ -z "${entrypoint}" ]]; then
        echo "[DETACHED][ERROR] cannot resolve startup entry point" >&2
        return 1
    fi

    if [[ "${UAV_DETACHED:-0}" == "1" ]]; then
        uav_write_detached_state \
            "${entrypoint}" \
            "${UAV_DETACHED_RUN_DIR:-${HOME}/uav_flight_logs/run}"
        return $?
    fi

    if [[ "${UAV_FOREGROUND:-0}" == "1" ]]; then
        return 0
    fi
    if [[ -z "${SSH_CONNECTION:-}" && -z "${SSH_TTY:-}" ]]; then
        return 0
    fi
    if ! command -v setsid >/dev/null 2>&1; then
        echo "[DETACHED][ERROR] setsid is not installed" >&2
        return 1
    fi

    local script_dir repo_root base stamp log_dir run_dir launch_log
    script_dir="$(dirname -- "${entrypoint}")"
    repo_root="$(cd -- "${script_dir}/.." && pwd -P)"
    base="$(basename -- "${entrypoint}")"
    base="${base%.sh}"
    stamp="$(date +%Y%m%d_%H%M%S)"
    log_dir="${UAV_DETACHED_LOG_DIR:-${HOME}/uav_flight_logs/launcher}"
    run_dir="${UAV_DETACHED_RUN_DIR:-${HOME}/uav_flight_logs/run}"
    launch_log="${log_dir}/${base}_${stamp}_ssh$$.log"

    if ! mkdir -p -- "${log_dir}" "${run_dir}"; then
        echo "[DETACHED][ERROR] cannot create detached runtime directories" >&2
        return 1
    fi
    chmod 700 "${log_dir}" "${run_dir}" 2>/dev/null || true
    if ! touch -- "${launch_log}"; then
        echo "[DETACHED][ERROR] cannot create launcher log: ${launch_log}" >&2
        return 1
    fi
    chmod 600 "${launch_log}" 2>/dev/null || true

    if ! setsid -f env \
        UAV_DETACHED=1 \
        UAV_DETACHED_LAUNCH_LOG="${launch_log}" \
        UAV_DETACHED_RUN_DIR="${run_dir}" \
        "${entrypoint}" "$@" \
        </dev/null >>"${launch_log}" 2>&1; then
        echo "[DETACHED][ERROR] setsid failed; see ${launch_log}" >&2
        return 1
    fi

    echo "[DETACHED] startup request accepted"
    echo "[DETACHED] launcher_log=${launch_log}"
    echo "[DETACHED] status=${repo_root}/scripts/status_detached_uav.sh ${base}"
    echo "[DETACHED] stop=${repo_root}/scripts/stop_detached_uav.sh ${base}"
    return 200
}
