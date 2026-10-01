#!/usr/bin/env bash
set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -L -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -L)"
readonly SCRIPT_PATH="${SCRIPT_DIR}/${BASH_SOURCE[0]##*/}"
readonly PROJECT_ROOT="$(dirname -- "${SCRIPT_DIR}")"

usage() {
    cat >&2 <<EOF
usage: ${0##*/} [single|competition] [digit|image] [heading_deg] [fixed|dynamic|shadow] [--restart|--no-restart]

Examples:
  ${0##*/} single digit
  ${0##*/} competition digit 0 fixed
  ${0##*/} competition image 0 dynamic --restart
EOF
}

if [[ "$#" -lt 2 || "$#" -gt 5 ]]; then
    usage
    exit 2
fi

readonly CHAIN="$1"
readonly MODE="$2"
readonly HEADING_DEG="${3:-0}"
readonly RELEASE_POINT_MODE="${4:-fixed}"
readonly APPLY_ACTION="${5:---restart}"

case "${CHAIN}" in
    single)
        ENTRY_SCRIPT="${SCRIPT_DIR}/start_single_target_fusion.sh"
        DESCRIPTION="single-target"
        ;;
    competition)
        ENTRY_SCRIPT="${SCRIPT_DIR}/start_competition_target_fusion.sh"
        DESCRIPTION="competition three-target"
        ;;
    *)
        echo "chain must be single or competition" >&2
        exit 2
        ;;
esac

case "${MODE}" in
    digit|image) ;;
    *) echo "mode must be digit or image" >&2; exit 2 ;;
esac

if ! [[ "${HEADING_DEG}" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
    echo "heading_deg must be a finite number" >&2
    exit 2
fi

case "${RELEASE_POINT_MODE}" in
    fixed|dynamic|shadow) ;;
    *) echo "release point mode must be fixed, dynamic, or shadow" >&2; exit 2 ;;
esac

case "${APPLY_ACTION}" in
    --restart|--no-restart) ;;
    *) echo "last argument must be --restart or --no-restart" >&2; exit 2 ;;
esac

if [[ ! -x "${ENTRY_SCRIPT}" ]]; then
    echo "entry script is missing or not executable: ${ENTRY_SCRIPT}" >&2
    exit 1
fi

if ((EUID != 0)); then
    exec sudo -- "${SCRIPT_PATH}" "$@"
fi

readonly SERVICE_USER="$(stat -c '%U' -- "${PROJECT_ROOT}")"
readonly SERVICE_HOME="$(getent passwd "${SERVICE_USER}" | cut -d: -f6)"
readonly UNIT_PATH="/etc/systemd/system/youth-vision.service"
readonly OBSOLETE_OVERRIDE="/etc/systemd/system/youth-vision.service.d/competition-image.conf"
UNIT_TEMP_DIR="$(mktemp -d /tmp/youth-vision-unit.XXXXXX)"
readonly UNIT_TEMP_DIR
UNIT_TEMP="${UNIT_TEMP_DIR}/youth-vision.service"
readonly UNIT_TEMP
trap 'rm -rf -- "${UNIT_TEMP_DIR}"' EXIT

if [[ -z "${SERVICE_HOME}" || ! -d "${SERVICE_HOME}" ]]; then
    echo "cannot determine home directory for ${SERVICE_USER}" >&2
    exit 1
fi

cat >"${UNIT_TEMP}" <<EOF
[Unit]
Description=${SERVICE_USER} ${DESCRIPTION} ${MODE} recognition and UAV fusion chain
After=network-online.target time-set.target
Wants=network-online.target time-set.target

[Service]
Type=simple
User=${SERVICE_USER}
Environment=HOME=${SERVICE_HOME}
Environment=UAV_FOREGROUND=1
WorkingDirectory=${PROJECT_ROOT}
ExecStart=${ENTRY_SCRIPT} ${MODE} ${HEADING_DEG} --release-point-mode ${RELEASE_POINT_MODE}
Restart=on-failure
RestartSec=10
KillMode=mixed
TimeoutStopSec=30
StandardOutput=journal
StandardError=journal
SyslogIdentifier=youth-vision

[Install]
WantedBy=multi-user.target
EOF

systemd-analyze verify "${UNIT_TEMP}"
install -m 0644 -- "${UNIT_TEMP}" "${UNIT_PATH}"
rm -f -- "${OBSOLETE_OVERRIDE}"
systemctl daemon-reload
systemctl enable youth-vision.service >/dev/null

if [[ "${APPLY_ACTION}" == "--restart" ]]; then
    systemctl restart youth-vision.service
fi

echo "configured youth-vision.service"
echo "chain=${CHAIN}"
echo "mode=${MODE}"
echo "heading_deg=${HEADING_DEG}"
echo "release_point_mode=${RELEASE_POINT_MODE}"
echo "action=${APPLY_ACTION}"
systemctl show youth-vision.service \
    -p UnitFileState -p ActiveState -p SubState -p ExecStart
