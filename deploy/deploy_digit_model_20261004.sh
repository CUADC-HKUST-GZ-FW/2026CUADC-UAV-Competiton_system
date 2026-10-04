#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
    cat <<'EOF'
Usage:
  ./deploy/deploy_digit_model_20261004.sh --check
  ./deploy/deploy_digit_model_20261004.sh --apply
  ./deploy/deploy_digit_model_20261004.sh --rollback BACKUP_DIRECTORY

Run this from a fresh clone on NX163 or NX164. The script detects the login
user, builds a device-specific TensorRT engine, backs up the active digit
model configuration, and updates only digit_cls_engine.

It never starts, stops, or restarts flight or vision processes.
EOF
}

mode=""
rollback_dir=""
while (($#)); do
    case "$1" in
        --check|--apply)
            [[ -z "${mode}" ]] || { usage >&2; exit 2; }
            mode="${1#--}"
            ;;
        --rollback)
            [[ -z "${mode}" && $# -ge 2 ]] || { usage >&2; exit 2; }
            mode="rollback"
            rollback_dir="$2"
            shift
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
    shift
done
mode="${mode:-check}"

required_commands=(awk basename cp cut date dirname find getent grep id install mkdir mv pgrep readlink rm sed sha256sum sort stat xargs)
for command_name in "${required_commands[@]}"; do
    command -v "${command_name}" >/dev/null 2>&1 || {
        echo "[ERROR] required command not found: ${command_name}" >&2
        exit 1
    }
done

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
readonly REPO_ROOT
readonly RELEASE_ID="20261004_digit_large101_v4_conservative"
readonly RELEASE_DIR="${REPO_ROOT}/models/releases/${RELEASE_ID}"
readonly MODEL_BASENAME="digit_cls_large101_1003_v4_conservative_best"
readonly SOURCE_PT="${RELEASE_DIR}/${MODEL_BASENAME}.pt"
readonly SOURCE_ONNX="${RELEASE_DIR}/${MODEL_BASENAME}.onnx"
readonly EXPECTED_PT_SHA="74873fef6f87716465de1b090c07e60abc47e10e4e72fbfd40c3e3eb7bbaf89c"
readonly EXPECTED_ONNX_SHA="a707fc46520cd942bc1b97fe5b1b2d0dd942608978f1cba32e363ae84c075ef2"
DEVICE_USER="$(id -un)"
readonly DEVICE_USER
DEVICE_HOME="$(getent passwd "${DEVICE_USER}" | cut -d: -f6)"
readonly DEVICE_HOME
readonly LIVE_ROOT="${DEVICE_HOME}/youth-vision-runtime"
readonly ACTIVE_CONFIG="${LIVE_ROOT}/configs/youth_pipeline.yaml"
readonly MODEL_PATH="${LIVE_ROOT}/models/${MODEL_BASENAME}.pt"
readonly ONNX_PATH="${LIVE_ROOT}/onnx/${MODEL_BASENAME}.onnx"
readonly ENGINE_PATH="${LIVE_ROOT}/engines/${MODEL_BASENAME}_fp16_${DEVICE_USER}.engine"
readonly TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
readonly STATE_FILE="${DEVICE_HOME}/.local/state/cuadc-uav/digit_model.env"
readonly BACKUP_ROOT="${DEVICE_HOME}/deployment_backups/models"

case "${DEVICE_USER}" in
    nx163|nx164) ;;
    *)
        echo "[ERROR] run as nx163 or nx164, not ${DEVICE_USER}." >&2
        exit 1
        ;;
esac

verify_sha() {
    local file="$1"
    local expected="$2"
    local label="$3"
    [[ -f "${file}" ]] || { echo "[ERROR] missing ${label}: ${file}" >&2; exit 1; }
    local actual
    actual="$(sha256sum "${file}" | awk '{print $1}')"
    [[ "${actual}" == "${expected}" ]] || {
        echo "[ERROR] ${label} SHA-256 mismatch: expected=${expected} actual=${actual}" >&2
        exit 1
    }
}

active_processes() {
    pgrep -af 'youth_vision_runner|start_recon_pipeline([.]sh)?|start_competition_target_fusion([.]sh)?|start_single_target_fusion([.]sh)?' || true
}

require_stopped() {
    local running
    running="$(active_processes)"
    if [[ -n "${running}" ]]; then
        echo "[ERROR] vision or flight processes are still running:" >&2
        echo "${running}" >&2
        echo "[ERROR] stop them on the ground before changing the active engine." >&2
        exit 1
    fi
}

read_active_engine() {
    sed -n 's/^[[:space:]]*digit_cls_engine:[[:space:]]*//p' "${ACTIVE_CONFIG}" | tail -n 1
}

verify_sha "${SOURCE_PT}" "${EXPECTED_PT_SHA}" "release PT"
verify_sha "${SOURCE_ONNX}" "${EXPECTED_ONNX_SHA}" "release ONNX"
[[ -d "${LIVE_ROOT}" ]] || { echo "[ERROR] live runtime missing: ${LIVE_ROOT}" >&2; exit 1; }
[[ -f "${ACTIVE_CONFIG}" ]] || { echo "[ERROR] active config missing: ${ACTIVE_CONFIG}" >&2; exit 1; }

if [[ "${mode}" == "check" ]]; then
    current_engine="$(read_active_engine)"
    echo "[CHECK] release=${RELEASE_ID}"
    echo "[CHECK] device=${DEVICE_USER} home=${DEVICE_HOME}"
    echo "[CHECK] current_engine=${current_engine:-<unset>}"
    echo "[CHECK] target_engine=${ENGINE_PATH}"
    echo "[CHECK] source_pt_sha256=${EXPECTED_PT_SHA}"
    echo "[CHECK] source_onnx_sha256=${EXPECTED_ONNX_SHA}"
    if [[ -x "${TRTEXEC}" ]]; then
        echo "[CHECK] trtexec=${TRTEXEC}"
    else
        echo "[ERROR] trtexec is not executable: ${TRTEXEC}" >&2
        exit 1
    fi
    running="$(active_processes)"
    if [[ -n "${running}" ]]; then
        echo "[CHECK] active processes must be stopped before --apply:"
        echo "${running}"
    else
        echo "[CHECK] no active vision or flight process found"
    fi
    exit 0
fi

if [[ "${mode}" == "rollback" ]]; then
    require_stopped
    resolved_backup="$(readlink -f -- "${rollback_dir}")"
    resolved_root="$(readlink -f -- "${BACKUP_ROOT}")"
    [[ "${resolved_backup}" == "${resolved_root}/"* ]] || {
        echo "[ERROR] rollback directory must be inside ${BACKUP_ROOT}" >&2
        exit 1
    }
    [[ -f "${resolved_backup}/backup.env" && -f "${resolved_backup}/config/youth_pipeline.yaml" ]] || {
        echo "[ERROR] incomplete backup: ${resolved_backup}" >&2
        exit 1
    }
    # shellcheck source=/dev/null
    source "${resolved_backup}/backup.env"
    [[ "${BACKUP_DEVICE_USER}" == "${DEVICE_USER}" ]] || {
        echo "[ERROR] backup belongs to ${BACKUP_DEVICE_USER}, not ${DEVICE_USER}." >&2
        exit 1
    }
    if [[ -n "${PREVIOUS_ENGINE_FILE}" && -f "${resolved_backup}/engine/${PREVIOUS_ENGINE_FILE}" ]]; then
        install -m 0644 "${resolved_backup}/engine/${PREVIOUS_ENGINE_FILE}" "${PREVIOUS_ENGINE_PATH}"
    fi
    cp -p -- "${resolved_backup}/config/youth_pipeline.yaml" "${ACTIVE_CONFIG}"
    echo "[OK] restored config for ${DEVICE_USER} from ${resolved_backup}"
    echo "[OK] active digit engine=$(read_active_engine)"
    echo "[NEXT] run the normal no-propeller startup and full-chain checks."
    exit 0
fi

require_stopped
[[ -x "${TRTEXEC}" ]] || { echo "[ERROR] trtexec is not executable: ${TRTEXEC}" >&2; exit 1; }
config_key_count="$(grep -Ec '^[[:space:]]*digit_cls_engine:' "${ACTIVE_CONFIG}")"
[[ "${config_key_count}" == "1" ]] || {
    echo "[ERROR] expected exactly one digit_cls_engine entry, found ${config_key_count}." >&2
    exit 1
}

STAMP="$(date +%Y%m%d_%H%M%S)"
readonly STAMP
readonly BACKUP_DIR="${BACKUP_ROOT}/${STAMP}_${DEVICE_USER}_before_${RELEASE_ID}"
PREVIOUS_ENGINE_PATH="$(read_active_engine)"
readonly PREVIOUS_ENGINE_PATH
previous_engine_file=""
if [[ -n "${PREVIOUS_ENGINE_PATH}" ]]; then
    previous_engine_file="$(basename -- "${PREVIOUS_ENGINE_PATH}")"
fi

mkdir -p -- "${BACKUP_DIR}/config" "${BACKUP_DIR}/engine" "${BACKUP_DIR}/models" "${BACKUP_DIR}/onnx"
cp -p -- "${ACTIVE_CONFIG}" "${BACKUP_DIR}/config/youth_pipeline.yaml"
if [[ -n "${PREVIOUS_ENGINE_PATH}" && -f "${PREVIOUS_ENGINE_PATH}" ]]; then
    cp -p -- "${PREVIOUS_ENGINE_PATH}" "${BACKUP_DIR}/engine/${previous_engine_file}"
fi
old_model="${LIVE_ROOT}/models/digit_cls_balanced101_0903_night_v2_best.pt"
[[ -f "${old_model}" ]] && cp -p -- "${old_model}" "${BACKUP_DIR}/models/"
old_onnx="${LIVE_ROOT}/onnx/digit_cls_balanced101_0903_night_v2_best.onnx"
[[ -f "${old_onnx}" ]] && cp -p -- "${old_onnx}" "${BACKUP_DIR}/onnx/"
{
    printf 'BACKUP_DEVICE_USER=%q\n' "${DEVICE_USER}"
    printf 'PREVIOUS_ENGINE_PATH=%q\n' "${PREVIOUS_ENGINE_PATH}"
    printf 'PREVIOUS_ENGINE_FILE=%q\n' "${previous_engine_file}"
    printf 'RELEASE_ID=%q\n' "${RELEASE_ID}"
} >"${BACKUP_DIR}/backup.env"
checksum_tmp="${BACKUP_DIR}.SHA256SUMS.$$"
(
    cd -- "${BACKUP_DIR}"
    find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum
) >"${checksum_tmp}"
mv -f -- "${checksum_tmp}" "${BACKUP_DIR}/SHA256SUMS"

mkdir -p -- "${LIVE_ROOT}/models" "${LIVE_ROOT}/onnx" "${LIVE_ROOT}/engines"
install -m 0644 "${SOURCE_PT}" "${MODEL_PATH}"
install -m 0644 "${SOURCE_ONNX}" "${ONNX_PATH}"
verify_sha "${MODEL_PATH}" "${EXPECTED_PT_SHA}" "installed PT"
verify_sha "${ONNX_PATH}" "${EXPECTED_ONNX_SHA}" "installed ONNX"

temporary_engine="${ENGINE_PATH}.building.$$"
temporary_config="${ACTIVE_CONFIG}.building.$$"
cleanup() {
    rm -f -- "${temporary_engine:-}" "${temporary_config:-}"
}
trap cleanup EXIT

"${TRTEXEC}" \
    --onnx="${ONNX_PATH}" \
    --saveEngine="${temporary_engine}" \
    --fp16 \
    --memPoolSize=workspace:1024 \
    --builderOptimizationLevel=3

[[ -s "${temporary_engine}" ]] || { echo "[ERROR] TensorRT engine was not created." >&2; exit 1; }
mv -f -- "${temporary_engine}" "${ENGINE_PATH}"

cp -p -- "${ACTIVE_CONFIG}" "${temporary_config}"
sed -E -i "s|^([[:space:]]*digit_cls_engine:)[[:space:]]*.*$|\\1 ${ENGINE_PATH}|" "${temporary_config}"
updated_engine="$(sed -n 's/^[[:space:]]*digit_cls_engine:[[:space:]]*//p' "${temporary_config}" | tail -n 1)"
[[ "${updated_engine}" == "${ENGINE_PATH}" ]] || {
    echo "[ERROR] failed to update digit_cls_engine atomically." >&2
    exit 1
}
mv -f -- "${temporary_config}" "${ACTIVE_CONFIG}"

engine_sha="$(sha256sum "${ENGINE_PATH}" | awk '{print $1}')"
mkdir -p -- "$(dirname -- "${STATE_FILE}")"
{
    printf 'RELEASE_ID=%q\n' "${RELEASE_ID}"
    printf 'DEVICE_USER=%q\n' "${DEVICE_USER}"
    printf 'PT_PATH=%q\n' "${MODEL_PATH}"
    printf 'PT_SHA256=%q\n' "${EXPECTED_PT_SHA}"
    printf 'ONNX_PATH=%q\n' "${ONNX_PATH}"
    printf 'ONNX_SHA256=%q\n' "${EXPECTED_ONNX_SHA}"
    printf 'ENGINE_PATH=%q\n' "${ENGINE_PATH}"
    printf 'ENGINE_SHA256=%q\n' "${engine_sha}"
    printf 'BACKUP_DIR=%q\n' "${BACKUP_DIR}"
    printf 'DEPLOYED_AT=%q\n' "$(date --iso-8601=seconds)"
} >"${STATE_FILE}"

echo "[OK] deployed ${RELEASE_ID} on ${DEVICE_USER}"
echo "[OK] active_config=${ACTIVE_CONFIG}"
echo "[OK] active_engine=$(read_active_engine)"
echo "[OK] engine_sha256=${engine_sha}"
echo "[OK] backup=${BACKUP_DIR}"
echo "[OK] state=${STATE_FILE}"
echo "[NEXT] run the normal no-propeller startup and full-chain checks."
