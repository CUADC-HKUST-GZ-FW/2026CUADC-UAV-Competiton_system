#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
    cat <<'EOF'
Usage: ./deploy/sync_local_jetson.sh [--check|--apply]

Run this from a clone of 2026CUADC-UAV-Competiton_system on NX163 or NX164.
--check is the default and only previews changed files.
--apply backs up overwritten files and updates the live runtime directories.
EOF
}

mode="check"
case "${1:---check}" in
    --check) mode="check" ;;
    --apply) mode="apply" ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
esac

for command in git rsync sed find getent; do
    command -v "${command}" >/dev/null 2>&1 || {
        echo "[ERROR] required command not found: ${command}" >&2
        exit 1
    }
done

readonly REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
readonly DEVICE_USER="$(id -un)"
readonly DEVICE_HOME="$(getent passwd "${DEVICE_USER}" | cut -d: -f6)"
readonly UAV_SOURCE="${REPO_ROOT}/uav_ros2_project"
readonly VISION_SOURCE="${REPO_ROOT}/youth-vision-runtime"
readonly UAV_DEST="${DEVICE_HOME}/uav_ros2_project"
readonly VISION_DEST="${DEVICE_HOME}/youth-vision-runtime"
readonly COMMIT="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
readonly STAMP="$(date +%Y%m%d_%H%M%S)"
readonly BACKUP_ROOT="${DEVICE_HOME}/deployment_backups/${STAMP}_${COMMIT:0:12}"

for source_dir in "${UAV_SOURCE}" "${VISION_SOURCE}"; do
    [[ -d "${source_dir}" ]] || {
        echo "[ERROR] source directory missing: ${source_dir}" >&2
        exit 1
    }
done

if [[ "${UAV_SOURCE}" == "${UAV_DEST}" || "${VISION_SOURCE}" == "${VISION_DEST}" ]]; then
    echo "[ERROR] repository clone must not be the live runtime directory." >&2
    exit 1
fi

camera_serial=""
active_vision_config="${VISION_DEST}/configs/youth_pipeline.yaml"
if [[ -f "${active_vision_config}" ]]; then
    camera_serial="$(
        sed -n 's/^[[:space:]]*mvs_serial:[[:space:]]*//p' \
            "${active_vision_config}" | head -n 1
    )"
fi

common_excludes=(
    --exclude=.git/
    --exclude=build/
    --exclude=install/
    --exclude=log/
    --exclude=logs/
    --exclude=run/
    --exclude=__pycache__/
    --exclude='*.pyc'
)

uav_excludes=(
    "${common_excludes[@]}"
    --exclude=.codex_backups/
    --exclude='*.tlog'
    --exclude='*.tlog.raw'
)

vision_excludes=(
    "${common_excludes[@]}"
    --exclude=engines/
    --exclude=weights/
    --exclude=onnx/
    --exclude=models/
    --exclude=overlays/
    --exclude=recon_results/
    --exclude=collections/
    --exclude=offline_inference/
    --exclude=staging/
    --exclude='*.mp4'
    --exclude='*.engine'
)

sync_tree() {
    local source_dir="$1"
    local destination_dir="$2"
    local backup_dir="$3"
    shift 3
    local -a excludes=("$@")
    local -a args=(-a --itemize-changes "${excludes[@]}")

    if [[ "${mode}" == "check" ]]; then
        args+=(--dry-run)
    else
        mkdir -p -- "${destination_dir}" "${backup_dir}"
        args+=(--backup --backup-dir="${backup_dir}")
    fi
    rsync "${args[@]}" "${source_dir}/" "${destination_dir}/"
}

echo "[INFO] repository=${REPO_ROOT}"
echo "[INFO] commit=${COMMIT}"
echo "[INFO] device=${DEVICE_USER} home=${DEVICE_HOME} mode=${mode}"

sync_tree \
    "${UAV_SOURCE}" \
    "${UAV_DEST}" \
    "${BACKUP_ROOT}/uav_ros2_project" \
    "${uav_excludes[@]}"
sync_tree \
    "${VISION_SOURCE}" \
    "${VISION_DEST}" \
    "${BACKUP_ROOT}/youth-vision-runtime" \
    "${vision_excludes[@]}"

if [[ "${mode}" == "check" ]]; then
    echo "[CHECK] preview complete; no file was changed."
    exit 0
fi

# The Git baseline mirrors NX163. Render only the home prefix in deployed text
# files so the same commit can run under /home/nx164 without a source fork.
if [[ "${DEVICE_HOME}" != "/home/nx163" ]]; then
    while IFS= read -r -d '' path; do
        sed -i "s#/home/nx163/#${DEVICE_HOME}/#g" "${path}"
    done < <(
        find "${UAV_DEST}" "${VISION_DEST}" -type f \
            \( -name '*.sh' -o -name '*.py' -o -name '*.cpp' \
               -o -name '*.hpp' -o -name '*.h' -o -name '*.yaml' \
               -o -name '*.yml' -o -name '*.xml' -o -name '*.service' \
               -o -name '*.md' -o -name '*.txt' \) -print0
    )
fi

# A camera serial is hardware identity, not shared source configuration.
if [[ -n "${camera_serial}" && -f "${active_vision_config}" ]]; then
    sed -i \
        "s/^[[:space:]]*mvs_serial:.*/mvs_serial: ${camera_serial}/" \
        "${active_vision_config}"
fi

mkdir -p -- "${DEVICE_HOME}/.local/state/cuadc-uav"
cat >"${DEVICE_HOME}/.local/state/cuadc-uav/deployed.env" <<EOF
REPOSITORY=CUADC-HKUST-GZ-FW/2026CUADC-UAV-Competiton_system
COMMIT=${COMMIT}
DEPLOYED_AT=$(date --iso-8601=seconds)
DEVICE_USER=${DEVICE_USER}
BACKUP_ROOT=${BACKUP_ROOT}
EOF

echo "[OK] live directories updated from ${COMMIT}."
echo "[OK] overwritten files backed up under ${BACKUP_ROOT}."
echo "[NEXT] rebuild uav_ros2_project and run the approved dry-run checks."
