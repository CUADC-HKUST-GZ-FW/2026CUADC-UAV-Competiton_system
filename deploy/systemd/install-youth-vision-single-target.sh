#!/usr/bin/env bash
set -euo pipefail

UNIT_NAME="youth-vision.service"
SOURCE_UNIT="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/youth-vision.service}"
TARGET_UNIT="/etc/systemd/system/${UNIT_NAME}"
SERVICE_USER="${UAV_SERVICE_USER:-${SUDO_USER:-}}"
HEADING_DEG="${YOUTH_VISION_HEADING_DEG:-0}"

if [[ ${EUID} -ne 0 ]]; then
  echo "Run with sudo: sudo $0${1:+ $1}" >&2
  exit 1
fi

if [[ ! -f "${SOURCE_UNIT}" ]]; then
  echo "Unit file not found: ${SOURCE_UNIT}" >&2
  exit 1
fi

if [[ -z "${SERVICE_USER}" || "${SERVICE_USER}" == "root" ]]; then
  echo "Unable to determine the Jetson user. Run with sudo from that account or set UAV_SERVICE_USER." >&2
  exit 1
fi

passwd_entry="$(getent passwd "${SERVICE_USER}" || true)"
SERVICE_HOME="$(cut -d: -f6 <<<"${passwd_entry}")"
if [[ -z "${SERVICE_HOME}" || ! -d "${SERVICE_HOME}" ]]; then
  echo "Home directory not found for user ${SERVICE_USER}" >&2
  exit 1
fi

if [[ ! "${HEADING_DEG}" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
  echo "YOUTH_VISION_HEADING_DEG must be numeric: ${HEADING_DEG}" >&2
  exit 1
fi

PROJECT_DIR="${UAV_PROJECT_DIR:-${SERVICE_HOME}/uav_ros2_project}"
START_SCRIPT="${PROJECT_DIR}/scripts/start_single_target_fusion.sh"
if [[ ! -x "${START_SCRIPT}" ]]; then
  echo "Target startup script is missing or not executable: ${START_SCRIPT}" >&2
  exit 1
fi

rendered_unit="$(mktemp --suffix=.service)"
trap 'rm -f "${rendered_unit}"' EXIT
unit_content="$(<"${SOURCE_UNIT}")"
unit_content="${unit_content//@UAV_USER@/${SERVICE_USER}}"
unit_content="${unit_content//@UAV_HOME@/${SERVICE_HOME}}"
unit_content="${unit_content//@UAV_PROJECT_DIR@/${PROJECT_DIR}}"
unit_content="${unit_content//@HEADING_DEG@/${HEADING_DEG}}"
printf '%s\n' "${unit_content}" >"${rendered_unit}"

if grep -Eq '@[A-Z0-9_]+@' "${rendered_unit}"; then
  echo "Unresolved placeholder in rendered unit" >&2
  grep -En '@[A-Z0-9_]+@' "${rendered_unit}" >&2
  exit 1
fi

systemd-analyze verify "${rendered_unit}"

if [[ -f "${TARGET_UNIT}" ]]; then
  backup="${TARGET_UNIT}.bak_$(date +%Y%m%d_%H%M%S)"
  cp -a "${TARGET_UNIT}" "${backup}"
  echo "Backed up existing unit to ${backup}"
fi

install -o root -g root -m 0644 "${rendered_unit}" "${TARGET_UNIT}"
systemctl daemon-reload
systemctl enable "${UNIT_NAME}"

echo "Installed ${TARGET_UNIT}"
echo "Service user: ${SERVICE_USER}"
echo "Startup command: ${START_SCRIPT} image ${HEADING_DEG}"
systemctl show "${UNIT_NAME}" \
  -p FragmentPath -p UnitFileState -p ActiveState -p SubState -p ExecStart
if systemctl is-active --quiet "${UNIT_NAME}"; then
  echo "The service is enabled and already running; this install did not restart it."
else
  echo "The service was enabled but not started. It will run on the next boot."
fi
