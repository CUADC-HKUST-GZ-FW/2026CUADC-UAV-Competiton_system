#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ROOT="${ROOT:-$(cd -- "${SCRIPT_DIR}/.." && pwd -P)}"
RUN_DIR="${ROOT}/run"
UAV_STOP_SCRIPT="${UAV_STOP_SCRIPT:-${HOME}/uav_ros2_project/scripts/stop_detached_uav.sh}"
SYSTEMD_UNIT="${YOUTH_VISION_SYSTEMD_UNIT:-youth-vision.service}"
CURRENT_BOOT_ID="$(cat /proc/sys/kernel/random/boot_id)"
result=0

process_start_ticks() {
  local pid="$1"
  awk '{print $22}' "/proc/${pid}/stat" 2>/dev/null || true
}

wait_for_exit() {
  local pid="$1"
  local attempts="$2"
  for _ in $(seq 1 "${attempts}"); do
    kill -0 "${pid}" 2>/dev/null || return 0
    sleep 0.1
  done
  return 1
}

stop_systemd_chain() {
  if ! systemctl is-active --quiet "${SYSTEMD_UNIT}" 2>/dev/null; then
    return 0
  fi

  echo "stopping active systemd unit ${SYSTEMD_UNIT}"
  if ! sudo -n systemctl stop "${SYSTEMD_UNIT}"; then
    echo "ERROR: ${SYSTEMD_UNIT} is active; run: sudo systemctl stop ${SYSTEMD_UNIT}" >&2
    exit 1
  fi
  for _ in $(seq 1 300); do
    systemctl is-active --quiet "${SYSTEMD_UNIT}" 2>/dev/null || return 0
    sleep 0.1
  done
  echo "ERROR: ${SYSTEMD_UNIT} did not stop within 30 seconds" >&2
  exit 1
}

stop_detached_chain() {
  local output rc
  [[ -x "${UAV_STOP_SCRIPT}" ]] || return 0

  set +e
  output="$("${UAV_STOP_SCRIPT}" single 2>&1)"
  rc=$?
  set -e
  if [[ -n "${output}" ]]; then
    printf '%s\n' "${output}"
  fi
  case "${rc}" in
    0|3) ;;
    *)
      echo "ERROR: detached single-target stop failed rc=${rc}" >&2
      result=1
      ;;
  esac
}

stop_lock_owner() {
  local owner_file="${RUN_DIR}/single_target_fusion.lock.owner"
  local pid="" boot_id="" start_ticks="" script="" key value cmdline
  [[ -s "${owner_file}" ]] || return 0

  while IFS='=' read -r key value; do
    case "${key}" in
      pid) pid="${value%% *}" ;;
      boot_id) boot_id="${value%% *}" ;;
      start_ticks) start_ticks="${value%% *}" ;;
      script) script="${value%% *}" ;;
    esac
  done < <(tr ' ' '\n' <"${owner_file}")

  if [[ ! "${pid}" =~ ^[0-9]+$ ]] \
      || [[ "${boot_id}" != "${CURRENT_BOOT_ID}" ]] \
      || [[ "$(process_start_ticks "${pid}")" != "${start_ticks}" ]] \
      || [[ "$(basename -- "${script:-unknown}")" != "start_single_target_fusion.sh" ]]; then
    rm -f -- "${owner_file}"
    return 0
  fi

  cmdline="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
  if [[ "${cmdline}" != *start_single_target_fusion.sh* ]]; then
    echo "ERROR: refusing to signal lock owner pid=${pid}; command does not match" >&2
    result=1
    return 0
  fi

  echo "sending SIGINT to single-target fusion owner pid=${pid}"
  kill -INT "${pid}" 2>/dev/null || true
  if ! wait_for_exit "${pid}" 300; then
    echo "sending SIGTERM to single-target fusion owner pid=${pid}"
    kill -TERM "${pid}" 2>/dev/null || true
    if ! wait_for_exit "${pid}" 100; then
      echo "ERROR: single-target fusion owner did not stop pid=${pid}" >&2
      result=1
    fi
  fi
}

stop_group() {
  local pid_file="$1"
  local expected="$2"
  local pid cmdline
  [[ -f "${pid_file}" ]] || return 0

  pid="$(cat "${pid_file}" 2>/dev/null || true)"
  if [[ ! "${pid}" =~ ^[0-9]+$ ]] || ! kill -0 "${pid}" 2>/dev/null; then
    rm -f -- "${pid_file}"
    return 0
  fi

  cmdline="$(tr '\0' ' ' <"/proc/${pid}/cmdline" 2>/dev/null || true)"
  if [[ ! "${cmdline}" =~ ${expected} ]]; then
    echo "stale pidfile ignored: ${pid_file} pid=${pid}" >&2
    rm -f -- "${pid_file}"
    return 0
  fi

  echo "stopping pid=${pid} pidfile=${pid_file}"
  kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
  if ! wait_for_exit "${pid}" 100; then
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
    if ! wait_for_exit "${pid}" 50; then
      kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
      if ! wait_for_exit "${pid}" 20; then
        echo "ERROR: process did not stop pid=${pid} pidfile=${pid_file}" >&2
        result=1
        return 0
      fi
    fi
  fi
  rm -f -- "${pid_file}"
}

stop_systemd_chain
stop_detached_chain
stop_lock_owner

# Legacy standalone recon/web entry points do not have a supervising parent.
stop_group "${RUN_DIR}/recon_web.pid" 'live_capture_server\.py'
stop_group "${RUN_DIR}/recon_results_dashboard.pid" 'recon_results_dashboard\.py'
stop_group "${RUN_DIR}/competition_selector.pid" 'competition_selector\.py|competition_selected\.sh'
stop_group "${RUN_DIR}/single_target_fusion_bridge.pid" 'vision_target_bridge'
stop_group "${RUN_DIR}/recon_ros.pid" 'recon_geolocator|recon(_with_mavros|_static_with_mavros)?\.launch\.py'
stop_group "${RUN_DIR}/recon_vision.pid" 'youth_vision_runner'
stop_group "${RUN_DIR}/manual_flight_bringup.pid" 'start_uav\.sh|uav_bringup|real_bringup\.launch\.py'

if [[ "${result}" != "0" ]]; then
  echo "recon pipeline stop incomplete" >&2
  exit "${result}"
fi

echo "recon pipeline and single-target fusion stopped"
