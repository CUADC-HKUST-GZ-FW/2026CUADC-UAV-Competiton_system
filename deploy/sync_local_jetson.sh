#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
    cat <<'EOF'
Usage: ./deploy/sync_local_jetson.sh [--check|--apply] [--all|--flight-only|--vision-only]

Run this from a clone of 2026CUADC-UAV-Competiton_system on NX163 or NX164.
--check is the default and only previews changed files.
--apply backs up overwritten files and updates the live runtime directories.
--all is the default and syncs flight plus vision code.
--flight-only (alias: --uav-only) syncs only uav_ros2_project.
--vision-only syncs only youth-vision-runtime.
EOF
}

mode=""
scope=""
while (($#)); do
    case "$1" in
        --check|--apply)
            if [[ -n "${mode}" ]]; then
                echo "[ERROR] select only one deployment mode." >&2
                usage >&2
                exit 2
            fi
            mode="${1#--}"
            ;;
        --all)
            requested_scope="all"
            ;;
        --flight-only|--uav-only)
            requested_scope="uav"
            ;;
        --vision-only)
            requested_scope="vision"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            usage >&2
            exit 2
            ;;
    esac
    if [[ -n "${requested_scope:-}" ]]; then
        if [[ -n "${scope}" ]]; then
            echo "[ERROR] select only one deployment scope." >&2
            usage >&2
            exit 2
        fi
        scope="${requested_scope}"
        unset requested_scope
    fi
    shift
done
mode="${mode:-check}"
scope="${scope:-all}"

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
readonly STATE_FILE="${DEVICE_HOME}/.local/state/cuadc-uav/deployed.env"

sync_uav=false
sync_vision=false
case "${scope}" in
    all)
        sync_uav=true
        sync_vision=true
        ;;
    uav) sync_uav=true ;;
    vision) sync_vision=true ;;
esac

source_dirs=()
if [[ "${sync_uav}" == true ]]; then
    source_dirs+=("${UAV_SOURCE}")
fi
if [[ "${sync_vision}" == true ]]; then
    source_dirs+=("${VISION_SOURCE}")
fi
for source_dir in "${source_dirs[@]}"; do
    [[ -d "${source_dir}" ]] || {
        echo "[ERROR] source directory missing: ${source_dir}" >&2
        exit 1
    }
done

if [[ "${sync_uav}" == true && "${UAV_SOURCE}" == "${UAV_DEST}" ]] || \
   [[ "${sync_vision}" == true && "${VISION_SOURCE}" == "${VISION_DEST}" ]]; then
    echo "[ERROR] repository clone must not be the live runtime directory." >&2
    exit 1
fi

camera_serial=""
active_vision_config="${VISION_DEST}/configs/youth_pipeline.yaml"
vision_engine_keys=(
    det_engine
    image_cls_engine
    digit_cls_engine
)
declare -A vision_engine_values=()
if [[ "${sync_vision}" == true && -f "${active_vision_config}" ]]; then
    camera_serial="$(
        sed -n 's/^[[:space:]]*mvs_serial:[[:space:]]*//p' \
            "${active_vision_config}" | head -n 1
    )"
    for key in "${vision_engine_keys[@]}"; do
        value="$(
            sed -n "s/^[[:space:]]*${key}:[[:space:]]*//p" \
                "${active_vision_config}" | head -n 1
        )"
        if [[ -n "${value}" && -f "${value}" ]]; then
            vision_engine_values["${key}"]="${value}"
        elif [[ -n "${value}" ]]; then
            echo "[WARN] active ${key} does not exist and will not be preserved: ${value}" >&2
        fi
    done
fi

# Camera calibration and installation geometry belong to the physical device.
# Preserve them while shared algorithms and operating thresholds are updated.
active_recon_config="${UAV_DEST}/src/uav_recon/config/recon.yaml"
hardware_recon_keys=(
    calibration_valid
    calibration_width
    calibration_height
    fx
    fy
    cx
    cy
    distortion
    camera_forward_tilt_deg
    camera_left_tilt_deg
    camera_offset_flu_m
)
declare -A hardware_recon_values=()
if [[ "${sync_uav}" == true && -f "${active_recon_config}" ]]; then
    for key in "${hardware_recon_keys[@]}"; do
        value="$(
            sed -n "s/^[[:space:]]*${key}:[[:space:]]*//p" \
                "${active_recon_config}" | head -n 1
        )"
        if [[ -n "${value}" ]]; then
            hardware_recon_values["${key}"]="${value}"
        fi
    done
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
echo "[INFO] device=${DEVICE_USER} home=${DEVICE_HOME} mode=${mode} scope=${scope}"

if [[ "${sync_uav}" == true ]]; then
    sync_tree \
        "${UAV_SOURCE}" \
        "${UAV_DEST}" \
        "${BACKUP_ROOT}/uav_ros2_project" \
        "${uav_excludes[@]}"
fi
if [[ "${sync_vision}" == true ]]; then
    sync_tree \
        "${VISION_SOURCE}" \
        "${VISION_DEST}" \
        "${BACKUP_ROOT}/youth-vision-runtime" \
        "${vision_excludes[@]}"
fi

if [[ "${mode}" == "check" ]]; then
    echo "[CHECK] preview complete; no file was changed."
    exit 0
fi

# The Git baseline mirrors NX163. Render only the home prefix in deployed text
# files so the same commit can run under /home/nx164 without a source fork.
if [[ "${DEVICE_HOME}" != "/home/nx163" ]]; then
    render_roots=()
    if [[ "${sync_uav}" == true ]]; then
        render_roots+=("${UAV_DEST}")
    fi
    if [[ "${sync_vision}" == true ]]; then
        render_roots+=("${VISION_DEST}")
    fi
    while IFS= read -r -d '' path; do
        sed -i "s#/home/nx163/#${DEVICE_HOME}/#g" "${path}"
    done < <(
        find "${render_roots[@]}" -type f \
            \( -name '*.sh' -o -name '*.py' -o -name '*.cpp' \
               -o -name '*.hpp' -o -name '*.h' -o -name '*.yaml' \
               -o -name '*.yml' -o -name '*.xml' -o -name '*.service' \
               -o -name '*.md' -o -name '*.txt' \) -print0
    )
fi

# A camera serial is hardware identity, not shared source configuration.
if [[ "${sync_vision}" == true && -n "${camera_serial}" && -f "${active_vision_config}" ]]; then
    sed -i \
        "s/^[[:space:]]*mvs_serial:.*/mvs_serial: ${camera_serial}/" \
        "${active_vision_config}"
fi

# TensorRT plans are built for the local Jetson and are intentionally excluded
# from rsync. Keep every valid active engine path instead of replacing it with
# the NX163 source basename.
if [[ "${sync_vision}" == true && -f "${active_vision_config}" ]]; then
    for key in "${vision_engine_keys[@]}"; do
        if [[ -v "vision_engine_values[${key}]" ]]; then
            value="${vision_engine_values[${key}]}"
            sed -i \
                "s#^\([[:space:]]*${key}:[[:space:]]*\).*#\1${value}#" \
                "${active_vision_config}"
        fi
    done
fi

if [[ "${sync_uav}" == true && -f "${active_recon_config}" ]]; then
    for key in "${hardware_recon_keys[@]}"; do
        if [[ -v "hardware_recon_values[${key}]" ]]; then
            value="${hardware_recon_values[${key}]}"
            sed -i \
                "s#^\([[:space:]]*${key}:[[:space:]]*\).*#\1${value}#" \
                "${active_recon_config}"
        fi
    done
fi

state_value() {
    local key="$1"
    if [[ -f "${STATE_FILE}" ]]; then
        sed -n "s/^${key}=//p" "${STATE_FILE}" | tail -n 1
    fi
}

previous_uav_source="$(state_value UAV_SOURCE)"
previous_vision_source="$(state_value VISION_SOURCE)"
previous_built_commit="$(state_value BUILT_COMMIT)"
previous_built_at="$(state_value BUILT_AT)"

case "${scope}" in
    all)
        uav_source="git:${COMMIT}"
        vision_source="git:${COMMIT}"
        ;;
    uav)
        uav_source="git:${COMMIT}"
        vision_source="${previous_vision_source:-preserved-local}"
        ;;
    vision)
        uav_source="${previous_uav_source:-preserved-local}"
        vision_source="git:${COMMIT}"
        ;;
esac

mkdir -p -- "${DEVICE_HOME}/.local/state/cuadc-uav"
{
cat <<EOF
REPOSITORY=CUADC-HKUST-GZ-FW/2026CUADC-UAV-Competiton_system
COMMIT=${COMMIT}
DEPLOYED_AT=$(date --iso-8601=seconds)
DEPLOY_SCOPE=${scope}
DEVICE_USER=${DEVICE_USER}
BACKUP_ROOT=${BACKUP_ROOT}
UAV_SOURCE=${uav_source}
VISION_SOURCE=${vision_source}
EOF
if [[ "${scope}" == "vision" && -n "${previous_built_commit}" ]]; then
    echo "BUILT_COMMIT=${previous_built_commit}"
    if [[ -n "${previous_built_at}" ]]; then
        echo "BUILT_AT=${previous_built_at}"
    fi
fi
} >"${STATE_FILE}"

echo "[OK] scope=${scope} live code updated from ${COMMIT}."
echo "[OK] overwritten files backed up under ${BACKUP_ROOT}."
if [[ "${sync_vision}" == true ]]; then
    echo "[OK] device camera serial and TensorRT engine paths preserved."
fi
if [[ "${sync_uav}" == true ]]; then
    echo "[OK] device intrinsics, distortion, and extrinsics preserved."
    echo "[NEXT] rebuild uav_ros2_project and run the approved dry-run checks."
else
    echo "[NEXT] run the approved vision checks; no ROS 2 rebuild is required."
fi
