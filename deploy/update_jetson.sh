#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
    cat <<'EOF'
Usage: ./deploy/update_jetson.sh [--check|--apply|--apply-local]

Update NX163 or NX164 from origin/main.
  --check  Fetch and preview the deployment without changing live files.
  --apply  Fast-forward main, back up and sync live files, then rebuild ROS 2.
  --apply-local  Deploy the current clean commit without contacting GitHub.

This script never starts, stops, or restarts flight/vision processes.
Device camera identity, intrinsics, distortion, and extrinsics are preserved.
EOF
}

mode="check"
case "${1:---check}" in
    --check) mode="check" ;;
    --apply) mode="apply" ;;
    --apply-local) mode="apply-local" ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
esac

for command in git rsync colcon getent; do
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
    git -C "${REPO_ROOT}" log --oneline --decorate "HEAD..origin/main"
    "${REPO_ROOT}/deploy/sync_local_jetson.sh" --check
    exit 0
fi

if [[ "${mode}" == "apply" ]]; then
    echo "[INFO] fast-forwarding main..."
    git -C "${REPO_ROOT}" merge --ff-only origin/main
else
    echo "[INFO] deploying current local commit without a GitHub fetch."
fi

echo "[INFO] backing up and syncing live runtime trees..."
"${REPO_ROOT}/deploy/sync_local_jetson.sh" --apply

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
source "${ros_setup}"
cd "${LIVE_UAV_ROOT}"
colcon build --symlink-install

readonly COMMIT="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
readonly STATE_FILE="${DEVICE_HOME}/.local/state/cuadc-uav/deployed.env"
{
    echo "BUILT_COMMIT=${COMMIT}"
    echo "BUILT_AT=$(date --iso-8601=seconds)"
} >>"${STATE_FILE}"

echo "[OK] ${DEVICE_USER} live code and ROS install are at ${COMMIT}."
echo "[NOTE] no service or flight process was restarted."
