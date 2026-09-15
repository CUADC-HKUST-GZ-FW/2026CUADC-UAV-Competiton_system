#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
ROS_WS="${ROS_WS:-/home/nx163/uav_ros2_project}"
MODE="${1:-digit}"
EXPECTED_TARGETS="${2:-0}"
FCU_URL="${FCU_URL:-udp://0.0.0.0:15001@192.168.144.14:15001}"
CONFIG="${CONFIG:-$ROOT/configs/youth_pipeline.yaml}"
RUN_DIR="$ROOT/run"
LOG_DIR="$ROOT/logs/recon"

if [[ "$MODE" != "digit" && "$MODE" != "image" ]]; then
  echo "usage: $0 [digit|image] [expected_targets]" >&2
  exit 2
fi
if ! [[ "$EXPECTED_TARGETS" =~ ^[0-9]+$ ]] || (( EXPECTED_TARGETS > 20 )); then
  echo "expected_targets must be an integer from 0 to 20; 0 means open-ended" >&2
  exit 2
fi

for pid_file in "$RUN_DIR/recon_vision.pid" "$RUN_DIR/recon_ros.pid"; do
  if [[ -f "$pid_file" ]] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
    echo "recon pipeline is already running; use stop_recon_pipeline.sh first" >&2
    exit 1
  fi
done

mkdir -p "$RUN_DIR" "$LOG_DIR" "$ROOT/logs/recognition" "$ROOT/recon_results/sessions"
stamp="$(date +%Y%m%d_%H%M%S)"
session_id="${stamp}_${MODE}_recon"
result_dir="$ROOT/recon_results/sessions/$session_id"
vision_log="$LOG_DIR/vision_${session_id}.log"
ros_log="$LOG_DIR/ros_${session_id}.log"
event_log="$ROOT/logs/recognition/recognition_events_${session_id}.jsonl"

# RAW_CAPTURE_DEFAULT_0903_START
# The native recorder receives the camera frame before draw_overlay() mutates
# the display frame, so this MP4 contains no Pose boxes, labels, or FPS text.
if [[ "${YOUTH_SAVE_RAW_VIDEO:-1}" == "1" && -z "${YOUTH_RECORD_FILE:-}" ]]; then
  raw_dir="${YOUTH_RAW_VIDEO_DIR:-/home/nx163/camera_recordings}"
  mkdir -p "$raw_dir"
  available_kb="$(df -Pk "$raw_dir" | awk 'NR==2 {print $4}')"
  if [[ -n "$available_kb" && "$available_kb" -ge "${YOUTH_RAW_MIN_FREE_KB:-2097152}" ]]; then
    export YOUTH_RECORD_FILE="$raw_dir/camera_${stamp}_1440x1080_${MODE}_raw_no_overlay.mp4"
    export YOUTH_RECORD_DURATION_SEC="${YOUTH_RECORD_DURATION_SEC:-600}"
    export YOUTH_RECORD_FPS="${YOUTH_RECORD_FPS:-60}"
    export YOUTH_RECORD_BITRATE_KBPS="${YOUTH_RECORD_BITRATE_KBPS:-12000}"
  else
    echo "warning: raw recording disabled because $raw_dir has insufficient free space" >&2
  fi
fi
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

recon_launch=(
  ros2 launch uav_recon recon_with_mavros.launch.py
  "fcu_url:=$FCU_URL"
  "output_root:=$result_dir"
  "expected_targets:=$EXPECTED_TARGETS"
)
if [[ -n "${RECON_STATIC_HEIGHT_M:-}" ]]; then
  recon_launch=(
    ros2 launch uav_recon recon_static_with_mavros.launch.py
    "fcu_url:=$FCU_URL"
    "output_root:=$result_dir"
    "fixed_relative_altitude_m:=$RECON_STATIC_HEIGHT_M"
    "max_horizontal_radius_95_m:=${RECON_MAX_HORIZONTAL_RADIUS_95_M:-0.5}"
  )
fi

setsid bash -lc 'source /opt/ros/humble/setup.bash; source "$1/install/setup.bash"; shift; exec "$@"' bash "$ROS_WS" "${recon_launch[@]}" \
  > "$ros_log" 2>&1 </dev/null &
ros_pid=$!
echo "$ros_pid" > "$RUN_DIR/recon_ros.pid"

sleep 3
if ! kill -0 "$vision_pid" 2>/dev/null || ! kill -0 "$ros_pid" 2>/dev/null; then
  "$ROOT/scripts/stop_recon_pipeline.sh" >/dev/null 2>&1 || true
  echo "recon startup failed" >&2
  tail -30 "$vision_log" >&2 || true
  tail -30 "$ros_log" >&2 || true
  exit 1
fi

echo "recon pipeline started"
echo "mode=$MODE"
echo "session_id=$session_id"
echo "result_dir=$result_dir"
echo "vision_log=$vision_log"
echo "ros_log=$ros_log"
echo "status_file=$result_dir/status.json"
echo "confirmed_targets_file=$result_dir/confirmed_targets.json"
echo "expected_targets=$EXPECTED_TARGETS"
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
