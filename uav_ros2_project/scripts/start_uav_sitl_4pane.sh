#!/usr/bin/env bash
export DISPLAY=:0
set -Eeuo pipefail

# 为兼容你现有的 uav_start4 / uav_restart4 / uav_stop4 / uav_attach4，
# session 名保持不变；实际界面现在只有 3 个 pane。
SESSION="uav_sitl_4pane"

ARDUPILOT_DIR="/home/yanyan/ardupilot"
ROS_WS="/home/yanyan/uav_ros2_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"

PARAM_FILE="/home/yanyan/uav_param_filter/filter_result_v3/params_to_import.param"
WAYPOINT_FILE="/home/yanyan/uav_ros2_ws/wp/test2.waypoints"

LOCATION="TEST"
FRAME="plane"
MAVROS_FCU_URL="udp://127.0.0.1:14550@14555"

SITL_TIMEOUT=180
MAVROS_TIMEOUT=90
COMMAND_DELAY=2

die(){ echo "[错误] $*" >&2; exit 1; }
info(){ echo "[信息] $*"; }

require_command(){ command -v "$1" >/dev/null 2>&1 || die "未找到命令：$1"; }
require_file(){ [[ -f "$1" ]] || die "文件不存在：$1"; }
require_dir(){ [[ -d "$1" ]] || die "目录不存在：$1"; }

send_to_pane(){
    local pane_id="$1"
    shift
    tmux send-keys -t "$pane_id" "$*" C-m
}

wait_for_sitl_prompt(){
    local elapsed=0
    info "等待 SITL 出现 MANUAL>……"
    while (( elapsed < SITL_TIMEOUT )); do
        if tmux capture-pane -p -S -3000 -t "$PANE_SITL" | grep -qE '(^|[[:space:]])MANUAL>'; then
            return 0
        fi
        sleep 1
        ((elapsed+=1))
    done
    return 1
}

mavros_connected(){
    bash -lc "
      set +u
      source '${ROS_SETUP}' >/dev/null 2>&1
      source '${ROS_WS}/install/setup.bash' >/dev/null 2>&1
      timeout 3s ros2 topic echo --once /mavros/state 2>/dev/null
    " | grep -q 'connected: true'
}

wait_for_mavros(){
    local elapsed=0
    info "等待 MAVROS connected=true……"
    while (( elapsed < MAVROS_TIMEOUT )); do
        mavros_connected && return 0
        sleep 2
        ((elapsed+=2))
    done
    return 1
}

require_command tmux
require_command timeout
require_dir "$ARDUPILOT_DIR"
require_dir "$ROS_WS"
require_file "$ROS_SETUP"
require_file "$ROS_WS/install/setup.bash"
require_file "$PARAM_FILE"
require_file "$WAYPOINT_FILE"
require_file "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"

if tmux has-session -t "$SESSION" 2>/dev/null; then
    if [[ "${1:-}" == "--restart" ]]; then
        tmux kill-session -t "$SESSION"
    else
        exec tmux attach-session -t "$SESSION"
    fi
fi

# 只创建 3 个 pane：
#   1 SITL / MAVProxy
#   2 MAVROS
#   3 uav_bringup
PANE_SITL="$(tmux -f ~/.tmux.conf new-session -d -s "$SESSION" -n "uav" -P -F '#{pane_id}')"
PANE_MAVROS="$(tmux split-window -h -t "$PANE_SITL" -P -F '#{pane_id}')"
PANE_BRINGUP="$(tmux split-window -v -t "$PANE_SITL" -P -F '#{pane_id}')"

tmux select-layout -t "${SESSION}:uav" tiled
tmux set-option -t "$SESSION" mouse on
tmux set-window-option -t "${SESSION}:uav" mode-keys vi
tmux set-option -t "$SESSION" history-limit 50000
tmux set-window-option -t "${SESSION}:uav" remain-on-exit on
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format ' #{pane_title} '

tmux select-pane -t "$PANE_SITL" -T "1 SITL / MAVProxy"
tmux select-pane -t "$PANE_MAVROS" -T "2 MAVROS"
tmux select-pane -t "$PANE_BRINGUP" -T "3 uav_bringup"

SITL_COMMAND="cd '${ARDUPILOT_DIR}' && ./Tools/autotest/sim_vehicle.py -v Plane -f '${FRAME}' --console --map -L '${LOCATION}' -w --add-param-file='${PARAM_FILE}'"
send_to_pane "$PANE_SITL" "$SITL_COMMAND"

if ! wait_for_sitl_prompt; then
    tmux select-pane -t "$PANE_SITL"
    exec tmux attach-session -t "$SESSION"
fi

MAVROS_COMMAND="cd '${ROS_WS}' && source '${ROS_SETUP}' && source install/setup.bash && ros2 launch mavros apm.launch fcu_url:='${MAVROS_FCU_URL}'"
send_to_pane "$PANE_MAVROS" "$MAVROS_COMMAND"

if ! wait_for_mavros; then
    tmux select-pane -t "$PANE_MAVROS"
    exec tmux attach-session -t "$SESSION"
fi

BRINGUP_COMMAND="cd '${ROS_WS}' && \
source '${ROS_SETUP}' && \
source install/setup.bash && \
ros2 launch uav_bringup sitl_mavros_dynamic_abc.launch.py 2>&1 | tee ~/bringup_log_\$(date +%Y%m%d_%H%M%S).log"

send_to_pane "$PANE_BRINGUP" "$BRINGUP_COMMAND"

# 目标发布现在由 sitl_target_gate_node 在 mission_manager 到达 STANDBY 后
# 自动读取 result.json 并发布，因此不再创建第 4 个 TargetCommand pane，
# 也不再手工 ros2 topic pub /vision/target_command。

send_to_pane "$PANE_SITL" "wp load ${WAYPOINT_FILE}"
sleep "$COMMAND_DELAY"
send_to_pane "$PANE_SITL" "wp list"
sleep "$COMMAND_DELAY"
send_to_pane "$PANE_SITL" "mode AUTO"
sleep "$COMMAND_DELAY"
send_to_pane "$PANE_SITL" "arm throttle"
sleep "$COMMAND_DELAY"

tmux select-layout -t "${SESSION}:uav" tiled
tmux select-pane -t "$PANE_SITL"

cat <<'EOF'

三分屏已启动：
  1：SITL / MAVProxy
  2：MAVROS
  3：uav_bringup

TargetCommand 第 4 屏已取消。
目标由 sitl_target_gate_node 自动读取 result.json 并发布。

鼠标点击面板可切换。
Ctrl+b 后按方向键可切换。
Ctrl+b 后按 z 可放大/恢复。
Ctrl+b 后按 d 可暂时离开。
EOF

exec tmux attach-session -t "$SESSION"
