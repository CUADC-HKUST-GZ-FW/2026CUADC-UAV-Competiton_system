#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
ROS_WS="${ROS_WS:-/home/nx163/uav_ros2_project}"
MODE="${1:-}"
CONFIG="${CONFIG:-$ROOT/configs/youth_pipeline.yaml}"
RUN_DIR="$ROOT/run"
LOG_DIR="$ROOT/logs/recon"
PID_FILE="$RUN_DIR/competition_selector.pid"

if [[ "$MODE" != "digit" && "$MODE" != "image" ]]; then
  echo "usage: $0 [digit|image]" >&2
  exit 2
fi
if [[ -f "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
  echo "competition reconnaissance is already running" >&2
  exit 1
fi
if pgrep -f 'vision_target_bridge_node' >/dev/null 2>&1; then
  echo "refusing to start: a flight-control vision bridge is running" >&2
  echo "competition reconnaissance must not publish into an active flight bridge" >&2
  exit 1
fi

mavros_count="$(pgrep -fc '/opt/ros/humble/lib/mavros/mavros_node' || true)"
# RAW_CAPTURE_DEFAULT_0903_COMPETITION
# Enable the native pre-overlay recorder for both startup branches. The saved
# MP4 is made from untouched camera frames and is suitable for later labeling.
if [[ "${YOUTH_SAVE_RAW_VIDEO:-1}" == "1" && -z "${YOUTH_RECORD_FILE:-}" ]]; then
  raw_dir="${YOUTH_RAW_VIDEO_DIR:-/home/nx163/camera_recordings}"
  mkdir -p "$raw_dir"
  available_kb="$(df -Pk "$raw_dir" | awk 'NR==2 {print $4}')"
  if [[ -n "$available_kb" && "$available_kb" -ge "${YOUTH_RAW_MIN_FREE_KB:-2097152}" ]]; then
    raw_stamp="$(date +%Y%m%d_%H%M%S)"
    export YOUTH_RECORD_FILE="$raw_dir/camera_${raw_stamp}_1440x1080_${MODE}_raw_no_overlay.mp4"
    export YOUTH_RECORD_DURATION_SEC="${YOUTH_RECORD_DURATION_SEC:-600}"
    export YOUTH_RECORD_FPS="${YOUTH_RECORD_FPS:-60}"
    export YOUTH_RECORD_BITRATE_KBPS="${YOUTH_RECORD_BITRATE_KBPS:-12000}"
  else
    echo "warning: raw recording disabled because $raw_dir has insufficient free space" >&2
  fi
fi

if (( mavros_count > 1 )); then
  echo "refusing to start: expected at most one MAVROS process, found $mavros_count" >&2
  exit 1
fi

if (( mavros_count == 0 )); then
  "$ROOT/scripts/start_recon_pipeline.sh" "$MODE" 0
  mavros_source="standalone_recon_pipeline"
else
  for pid_file in "$RUN_DIR/recon_vision.pid" "$RUN_DIR/recon_ros.pid"; do
    if [[ -f "$pid_file" ]] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
      echo "recon pipeline is already running; use stop_recon_pipeline.sh first" >&2
      exit 1
    fi
  done

  mkdir -p "$RUN_DIR" "$LOG_DIR" "$ROOT/logs/recognition" "$ROOT/recon_results/sessions"
  stamp="$(date +%Y%m%d_%H%M%S)"
  session_id="${stamp}_${MODE}_competition_recon"
  result_dir="$ROOT/recon_results/sessions/$session_id"
  vision_log="$LOG_DIR/vision_${session_id}.log"
  ros_log="$LOG_DIR/ros_${session_id}.log"
  event_log="$ROOT/logs/recognition/recognition_events_${session_id}.jsonl"
  echo "$MODE" > "$ROOT/configs/youth_runtime_mode.txt"
  ln -sfn "sessions/$session_id" "$ROOT/recon_results/latest"

  vision_args=(--config "$CONFIG" --class-mode "$MODE")
  if [[ -n "${YOUTH_RECORD_FILE:-}" ]]; then
    vision_args+=(
      --record-file "$YOUTH_RECORD_FILE"
      --record-duration-sec "${YOUTH_RECORD_DURATION_SEC:-600}"
      --record-fps "${YOUTH_RECORD_FPS:-60}"
      --record-bitrate-kbps "${YOUTH_RECORD_BITRATE_KBPS:-12000}"
    )
  fi

  sudo -n /usr/sbin/nvpmodel -m 0 >/dev/null
  sudo -n /usr/bin/jetson_clocks >/dev/null

  setsid env \
    YOUTH_SESSION_ID="$session_id" \
    YOUTH_RECOGNITION_LOG="$event_log" \
    LD_LIBRARY_PATH="/opt/MVS/lib/aarch64:/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/nvidia" \
    "$ROOT/native/build/youth_vision_runner" \
      "${vision_args[@]}" \
    > "$vision_log" 2>&1 </dev/null &
  vision_pid=$!
  echo "$vision_pid" > "$RUN_DIR/recon_vision.pid"

  recon_command=(
    ros2 run uav_recon recon_geolocator_node
    --ros-args
    --params-file "$ROS_WS/install/uav_recon/share/uav_recon/config/recon.yaml"
    -p "output_root:=$result_dir"
  )
  if [[ -n "${RECON_STATIC_HEIGHT_M:-}" ]]; then
    recon_command+=(
      -p tracking_mode:=legacy_geo_cluster
      -p ground_altitude_mode:=fixed_relative
      -p "fixed_relative_altitude_m:=$RECON_STATIC_HEIGHT_M"
      -p "max_horizontal_radius_95_m:=${RECON_MAX_HORIZONTAL_RADIUS_95_M:-0.5}"
    )
  fi

  setsid bash -lc 'source /opt/ros/humble/setup.bash; source "$1/install/setup.bash"; shift; exec "$@"' bash "$ROS_WS" "${recon_command[@]}" \
    > "$ros_log" 2>&1 </dev/null &
  ros_pid=$!
  echo "$ros_pid" > "$RUN_DIR/recon_ros.pid"

  sleep 3
  if ! kill -0 "$vision_pid" 2>/dev/null || ! kill -0 "$ros_pid" 2>/dev/null; then
    "$ROOT/scripts/stop_recon_pipeline.sh" >/dev/null 2>&1 || true
    echo "shared-MAVROS competition recon startup failed" >&2
    tail -30 "$vision_log" >&2 || true
    tail -30 "$ros_log" >&2 || true
    exit 1
  fi
  mavros_source="existing_shared_process"
fi

session_root="$(readlink -f "$ROOT/recon_results/latest")"
session_id="$(basename "$session_root")"
selector_log="$LOG_DIR/competition_selector_${session_id}.log"
selector_status_arg=""
if [[ -n "${RECON_STATIC_HEIGHT_M:-}" ]]; then
  selector_status_arg=" --allow-confirmed"
fi

setsid bash -lc "source /opt/ros/humble/setup.bash; source '$ROS_WS/install/setup.bash'; exec python3 '$ROOT/scripts/competition_selector.py' --mode '$MODE' --session-root '$session_root' --required-targets 3 --dedup-radius-m 3.0 --settle-sec 1.0${selector_status_arg}" \
  > "$selector_log" 2>&1 </dev/null &
selector_pid=$!
echo "$selector_pid" > "$PID_FILE"

sleep 2
if ! kill -0 "$selector_pid" 2>/dev/null; then
  "$ROOT/scripts/stop_recon_pipeline.sh" >/dev/null 2>&1 || true
  echo "competition selector failed to start" >&2
  tail -40 "$selector_log" >&2 || true
  exit 1
fi

echo "competition reconnaissance started"
echo "mode=$MODE"
echo "session_id=$session_id"
echo "all_results=$session_root/target_*/result.json"
echo "selected_result=$session_root/competition_selected.json"
echo "selected_topic=/vision/competition_selected_target"
echo "selector_log=$selector_log"
echo "mavros_source=$mavros_source"
echo "flight_control_started_by_this_script=false"
echo "flight_command_published=false"
if [[ -n "${RECON_STATIC_HEIGHT_M:-}" ]]; then
  echo "ground_altitude_mode=fixed_relative"
  echo "fixed_relative_altitude_m=$RECON_STATIC_HEIGHT_M"
  echo "max_horizontal_radius_95_m=${RECON_MAX_HORIZONTAL_RADIUS_95_M:-0.5}"
fi
if [[ -n "${YOUTH_RECORD_FILE:-}" ]]; then
  echo "record_file=$YOUTH_RECORD_FILE"
  echo "record_duration_sec=${YOUTH_RECORD_DURATION_SEC:-600}"

  echo "record_content=raw_camera_frames_without_model_overlay"
fi
