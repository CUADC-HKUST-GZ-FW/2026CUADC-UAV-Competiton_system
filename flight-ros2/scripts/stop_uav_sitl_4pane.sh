#!/usr/bin/env bash
set -Eeuo pipefail

SESSION="${UAV_TMUX_SESSION:-uav_sitl_4pane}"

if tmux has-session -t "${SESSION}" 2>/dev/null; then
    tmux list-panes -t "${SESSION}" -F "#{pane_id}" | while read -r pane_id; do
        tmux send-keys -t "${pane_id}" C-c
    done
    sleep 2
    tmux kill-session -t "${SESSION}"
    echo "[信息] 已停止。"
else
    echo "[信息] 没有运行中的 uav_sitl_4pane"
fi
