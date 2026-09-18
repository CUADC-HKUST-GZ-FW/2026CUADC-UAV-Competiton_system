#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
    cat <<'EOF'
Usage: ./deploy/update_jetson.sh [--check|--apply|--apply-local] [--all|--flight-only|--vision-only]

Update NX163 or NX164 from origin/main.
  --check  Fetch and preview the deployment without changing live files.
  --apply  Fast-forward main, back up and sync live files, then rebuild selected code.
  --apply-local  Deploy the current clean commit without contacting GitHub.
  --all  Update flight and vision code, rebuild native vision if needed, then rebuild ROS 2 (default).
  --flight-only  Update only uav_ros2_project and rebuild ROS 2.
  --vision-only  Update youth-vision-runtime and rebuild the native runner if needed, without rebuilding ROS 2.

This script never starts, stops, or restarts flight/vision processes.
Device camera identity, intrinsics, distortion, and extrinsics are preserved.
EOF
}

mode=""
scope=""
while (($#)); do
    case "$1" in
        --check|--apply|--apply-local)
            if [[ -n "${mode}" ]]; then
                echo "[ERROR] select only one update mode." >&2
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
            echo "[ERROR] select only one update scope." >&2
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

case "${scope}" in
    all) scope_flag="--all" ;;
    uav) scope_flag="--flight-only" ;;
    vision) scope_flag="--vision-only" ;;
esac

required_commands=(git rsync getent sed tail)
if [[ "${scope}" != "uav" ]]; then
    required_commands+=(awk make pgrep sha256sum)
fi
if [[ "${scope}" != "vision" ]]; then
    required_commands+=(colcon)
fi
for command in "${required_commands[@]}"; do
    command -v "${command}" >/dev/null 2>&1 || {
        echo "[ERROR] required command not found: ${command}" >&2
        exit 1
    }
done

readonly REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
readonly DEVICE_USER="$(id -un)"
readonly DEVICE_HOME="$(getent passwd "${DEVICE_USER}" | cut -d: -f6)"
readonly LIVE_UAV_ROOT="${DEVICE_HOME}/uav_ros2_project"
readonly LIVE_VISION_ROOT="${DEVICE_HOME}/youth-vision-runtime"
readonly VISION_NATIVE_DIR="${LIVE_VISION_ROOT}/native"
readonly VISION_NATIVE_SOURCE="${VISION_NATIVE_DIR}/youth_vision_runner.cpp"
readonly VISION_NATIVE_MAKEFILE="${VISION_NATIVE_DIR}/Makefile"
readonly VISION_NATIVE_BINARY="${VISION_NATIVE_DIR}/build/youth_vision_runner"
readonly STATE_FILE="${DEVICE_HOME}/.local/state/cuadc-uav/deployed.env"

state_value() {
    local key="$1"
    if [[ -f "${STATE_FILE}" ]]; then
        sed -n "s/^${key}=//p" "${STATE_FILE}" | tail -n 1
    fi
}

state_set() {
    local key="$1"
    local value="$2"
    local temporary="${STATE_FILE}.tmp.$$"
    awk -F= -v key="${key}" '$1 != key' "${STATE_FILE}" >"${temporary}"
    printf '%s=%s\n' "${key}" "${value}" >>"${temporary}"
    mv -f -- "${temporary}" "${STATE_FILE}"
}

vision_native_source_hash() {
    [[ -f "${VISION_NATIVE_SOURCE}" && -f "${VISION_NATIVE_MAKEFILE}" ]] || return 1
    {
        sha256sum "${VISION_NATIVE_SOURCE}" | awk '{print $1}'
        sha256sum "${VISION_NATIVE_MAKEFILE}" | awk '{print $1}'
    } | sha256sum | awk '{print $1}'
}

vision_native_binary_hash() {
    [[ -f "${VISION_NATIVE_BINARY}" ]] || return 1
    sha256sum "${VISION_NATIVE_BINARY}" | awk '{print $1}'
}

case "${DEVICE_USER}" in
    nx163|nx164) ;;
    *)
        echo "[ERROR] run this on NX163 or NX164, not as ${DEVICE_USER}." >&2
        exit 1
        ;;
esac

if [[ -n "$(git -C "${REPO_ROOT}" status --porcelain --untracked-files=no)" ]]; then
    echo "[ERROR] tracked files in the deployment clone have local changes." >&2
    echo "[ERROR] commit or restore them before updating from origin/main." >&2
    exit 1
fi

branch="$(git -C "${REPO_ROOT}" branch --show-current)"
if [[ "${branch}" != "main" ]]; then
    echo "[ERROR] deployment clone must be on main; current branch=${branch:-detached}." >&2
    exit 1
fi

if [[ "${mode}" != "apply-local" ]]; then
    echo "[INFO] fetching origin/main..."
    git -C "${REPO_ROOT}" fetch origin main
fi

if [[ "${mode}" == "check" ]]; then
    current="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
    target="$(git -C "${REPO_ROOT}" rev-parse origin/main)"
    echo "[CHECK] current=${current}"
    echo "[CHECK] target=${target}"
    echo "[CHECK] scope=${scope}"
    git -C "${REPO_ROOT}" log --oneline --decorate "HEAD..origin/main"
    "${REPO_ROOT}/deploy/sync_local_jetson.sh" --check "${scope_flag}"
    exit 0
fi

if [[ "${mode}" == "apply" ]]; then
    echo "[INFO] fast-forwarding main..."
    git -C "${REPO_ROOT}" merge --ff-only origin/main
else
    echo "[INFO] deploying current local commit without a GitHub fetch."
fi

previous_vision_native_source_sha256=""
previous_vision_native_binary_sha256=""
previous_vision_native_built_commit=""
previous_vision_native_built_at=""
vision_native_source_before=""
if [[ "${scope}" != "uav" ]]; then
    previous_vision_native_source_sha256="$(state_value VISION_NATIVE_SOURCE_SHA256)"
    previous_vision_native_binary_sha256="$(state_value VISION_NATIVE_BINARY_SHA256)"
    previous_vision_native_built_commit="$(state_value VISION_NATIVE_BUILT_COMMIT)"
    previous_vision_native_built_at="$(state_value VISION_NATIVE_BUILT_AT)"
    vision_native_source_before="$(vision_native_source_hash 2>/dev/null || true)"
fi

echo "[INFO] backing up and syncing live runtime trees..."
"${REPO_ROOT}/deploy/sync_local_jetson.sh" --apply "${scope_flag}"

readonly COMMIT="$(git -C "${REPO_ROOT}" rev-parse HEAD)"

if [[ "${scope}" != "uav" ]]; then
    vision_native_source_after="$(vision_native_source_hash)"
    vision_native_binary_after="$(vision_native_binary_hash 2>/dev/null || true)"
    rebuild_vision_native=false
    rebuild_reasons=()

    if [[ -z "${vision_native_binary_after}" ]]; then
        rebuild_vision_native=true
        rebuild_reasons+=("binary-missing")
    fi
    if [[ "${vision_native_source_before}" != "${vision_native_source_after}" ]]; then
        rebuild_vision_native=true
        rebuild_reasons+=("source-updated")
    fi
    if [[ -z "${previous_vision_native_source_sha256}" || \
          -z "${previous_vision_native_binary_sha256}" ]]; then
        rebuild_vision_native=true
        rebuild_reasons+=("manifest-missing")
    elif [[ "${previous_vision_native_source_sha256}" != "${vision_native_source_after}" ]]; then
        rebuild_vision_native=true
        rebuild_reasons+=("source-manifest-mismatch")
    elif [[ "${previous_vision_native_binary_sha256}" != "${vision_native_binary_after}" ]]; then
        rebuild_vision_native=true
        rebuild_reasons+=("binary-manifest-mismatch")
    fi

    if [[ "${rebuild_vision_native}" == true ]]; then
        if pgrep -f "^${VISION_NATIVE_BINARY}( |$)" >/dev/null 2>&1; then
            echo "[ERROR] youth_vision_runner is running; stop it before rebuilding." >&2
            exit 1
        fi

        staging_relative="build/.sync-staging-$$"
        staging_dir="${VISION_NATIVE_DIR}/${staging_relative}"
        staging_binary="${staging_dir}/youth_vision_runner"
        echo "[INFO] rebuilding native vision runner: ${rebuild_reasons[*]}"
        if ! make -C "${VISION_NATIVE_DIR}" \
            BUILD_DIR="${staging_relative}" \
            "${staging_relative}/youth_vision_runner"; then
            rm -f -- "${staging_binary}"
            rmdir -- "${staging_dir}" 2>/dev/null || true
            echo "[ERROR] native vision build failed; active binary was not changed." >&2
            exit 1
        fi
        if [[ ! -x "${staging_binary}" ]]; then
            rm -f -- "${staging_binary}"
            rmdir -- "${staging_dir}" 2>/dev/null || true
            echo "[ERROR] native vision build produced no executable." >&2
            exit 1
        fi

        backup_root="$(state_value BACKUP_ROOT)"
        binary_backup_dir="${backup_root}/youth-vision-runtime/native/build"
        mkdir -p -- "${binary_backup_dir}"
        if [[ -f "${VISION_NATIVE_BINARY}" ]]; then
            cp -a -- "${VISION_NATIVE_BINARY}" \
                "${binary_backup_dir}/youth_vision_runner.pre_rebuild"
        fi
        chmod 755 -- "${staging_binary}"
        mv -f -- "${staging_binary}" "${VISION_NATIVE_BINARY}"
        rmdir -- "${staging_dir}"
        vision_native_binary_after="$(vision_native_binary_hash)"
        previous_vision_native_built_commit="${COMMIT}"
        previous_vision_native_built_at="$(date --iso-8601=seconds)"
    else
        echo "[INFO] native vision runner already matches its recorded source hash."
    fi

    state_set VISION_BUILD_STATUS ready
    state_set VISION_NATIVE_SOURCE_SHA256 "${vision_native_source_after}"
    state_set VISION_NATIVE_BINARY_SHA256 "${vision_native_binary_after}"
    state_set VISION_NATIVE_BUILT_COMMIT "${previous_vision_native_built_commit}"
    state_set VISION_NATIVE_BUILT_AT "${previous_vision_native_built_at}"
    state_set VISION_NATIVE_VALIDATED_COMMIT "${COMMIT}"

    echo "[OK] native vision source=${vision_native_source_after}"
    echo "[OK] native vision binary=${vision_native_binary_after}"

    if [[ "${scope}" == "vision" ]]; then
        echo "[OK] ${DEVICE_USER} vision code is at ${COMMIT}."
        echo "[NOTE] ROS 2 was not rebuilt because flight code was not selected."
        echo "[NOTE] no service or flight process was restarted."
        exit 0
    fi
fi

ros_setup=""
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
    ros_setup="/opt/ros/humble/setup.bash"
else
    echo "[ERROR] ROS 2 setup.bash was not found." >&2
    exit 1
fi

echo "[INFO] rebuilding ${LIVE_UAV_ROOT}..."
# setup.bash is supplied by ROS 2 and intentionally modifies the shell environment.
# shellcheck disable=SC1090
set +u
source "${ros_setup}"
set -u
cd "${LIVE_UAV_ROOT}"
colcon build --symlink-install

{
    echo "BUILT_COMMIT=${COMMIT}"
    echo "BUILT_AT=$(date --iso-8601=seconds)"
    echo "UAV_BUILT_COMMIT=${COMMIT}"
} >>"${STATE_FILE}"

echo "[OK] ${DEVICE_USER} selected scope=${scope} and ROS install are at ${COMMIT}."
echo "[NOTE] no service or flight process was restarted."
