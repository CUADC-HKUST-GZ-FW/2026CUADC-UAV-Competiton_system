#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
START_SCRIPT="${REPO_ROOT}/scripts/start_uav_sitl_4pane.sh"
STOP_SCRIPT="${REPO_ROOT}/scripts/stop_uav_sitl_4pane.sh"
CONFIG_EXAMPLE="${REPO_ROOT}/config/sitl_local.env.example"
LOCAL_CONFIG="${REPO_ROOT}/config/sitl_local.env"
BASHRC="${HOME}/.bashrc"
MARKER_BEGIN="# >>> UAV SITL four-pane shortcuts >>>"
MARKER_END="# <<< UAV SITL four-pane shortcuts <<<"

if [[ ! -f "${START_SCRIPT}" ]]; then
    echo "[错误] 找不到启动脚本：${START_SCRIPT}" >&2
    exit 1
fi

if [[ ! -f "${STOP_SCRIPT}" ]]; then
    echo "[错误] 找不到停止脚本：${STOP_SCRIPT}" >&2
    exit 1
fi

chmod +x "${START_SCRIPT}" "${STOP_SCRIPT}"

if [[ ! -f "${LOCAL_CONFIG}" && -f "${CONFIG_EXAMPLE}" ]]; then
    cp "${CONFIG_EXAMPLE}" "${LOCAL_CONFIG}"
    echo "[信息] 已创建本机配置：${LOCAL_CONFIG}"
fi

touch "${BASHRC}"
tmp_bashrc="$(mktemp)"
sed "/^${MARKER_BEGIN}$/,/^${MARKER_END}$/d" "${BASHRC}" > "${tmp_bashrc}"
cat >> "${tmp_bashrc}" <<EOF
${MARKER_BEGIN}
alias uav_start4='${START_SCRIPT}'
alias uav_restart4='${START_SCRIPT} --restart'
alias uav_stop4='${STOP_SCRIPT}'
alias uav_attach4='tmux attach -t uav_sitl_4pane'
alias uav_config4='nano ${LOCAL_CONFIG}'
${MARKER_END}
EOF
mv "${tmp_bashrc}" "${BASHRC}"

echo "[信息] SITL 四分屏快捷命令已安装。"
echo
echo "请执行："
echo
echo "source ~/.bashrc"
echo
echo "然后可使用："
echo
echo "uav_config4"
echo "uav_start4"
echo "uav_restart4"
echo "uav_stop4"
echo "uav_attach4"
