#!/usr/bin/env bash
set -euo pipefail

UNIT_NAME="youth-vision.service"
SOURCE_UNIT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/youth-vision.service}"
TARGET_UNIT="/etc/systemd/system/${UNIT_NAME}"

if [[ ${EUID} -ne 0 ]]; then
  echo "Run with sudo: sudo $0${1:+ $1}" >&2
  exit 1
fi

if [[ ! -f "${SOURCE_UNIT}" ]]; then
  echo "Unit file not found: ${SOURCE_UNIT}" >&2
  exit 1
fi

if [[ ! -x /home/nx163/uav_ros2_project/scripts/start_single_target_fusion.sh ]]; then
  echo "Target startup script is missing or not executable" >&2
  exit 1
fi

if [[ -f "${TARGET_UNIT}" ]]; then
  backup="${TARGET_UNIT}.bak_$(date +%Y%m%d_%H%M%S)"
  cp -a "${TARGET_UNIT}" "${backup}"
  echo "Backed up existing unit to ${backup}"
fi

install -o root -g root -m 0644 "${SOURCE_UNIT}" "${TARGET_UNIT}"
systemctl daemon-reload
systemctl enable "${UNIT_NAME}"

echo "Installed ${TARGET_UNIT}"
systemctl show "${UNIT_NAME}" \
  -p FragmentPath -p UnitFileState -p ActiveState -p SubState -p ExecStart
echo "The service was enabled but not started. It will run on the next boot."
