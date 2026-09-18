#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
    cat <<'EOF'
Usage: ./deploy/update_jetson.sh [--check|--apply|--apply-local] [--all|--flight-only|--vision-only]

Update NX163 or NX164 from origin/main.
  --check  Fetch and preview the deployment without changing live files.
  --apply  Fast-forward main, back up and sync live files, then rebuild ROS 2.
  --apply-local  Deploy the current clean commit without contacting GitHub.
  --all  Update flight and vision code (default).
  --flight-only  Update only uav_ros2_project and rebuild ROS 2.
  --vision-only  Update only youth-vision-runtime without rebuilding ROS 2.

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

required_commands=(git rsync getent)
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

echo "[INFO] backing up and syncing live runtime trees..."
"${REPO_ROOT}/deploy/sync_local_jetson.sh" --apply "${scope_flag}"

readonly COMMIT="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
readonly STATE_FILE="${DEVICE_HOME}/.local/state/cuadc-uav/deployed.env"

if [[ "${scope}" == "vision" ]]; then
    echo "[OK] ${DEVICE_USER} vision code is at ${COMMIT}."
    echo "[NOTE] ROS 2 was not rebuilt because flight code was not selected."
    echo "[NOTE] no service or flight process was restarted."
    exit 0
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
