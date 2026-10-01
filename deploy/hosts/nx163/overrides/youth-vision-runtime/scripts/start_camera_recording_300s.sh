#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
OUTPUT_DIR="${OUTPUT_DIR:-/home/nx163/camera_recordings}"
LOG_DIR="$ROOT/logs"
RECORDER="$ROOT/scripts/record_camera_300s.py"

mkdir -p "$OUTPUT_DIR" "$LOG_DIR"

if [[ -f "$LOG_DIR/camera_recording.pid" ]]; then
  active_pid="$(cat "$LOG_DIR/camera_recording.pid" 2>/dev/null || true)"
  active_command="$(ps -p "$active_pid" -o args= 2>/dev/null || true)"
  if [[ "$active_command" == *"$RECORDER"* ]]; then
    echo "recording already running: pid=$active_pid" >&2
    exit 1
  fi
  rm -f "$LOG_DIR/camera_recording.pid"
fi

"$ROOT/scripts/stop_youth_pipeline.sh" >/dev/null
sleep 1

stamp="$(date +%Y%m%d_%H%M%S)"
output="$OUTPUT_DIR/camera_${stamp}_1440x1080_300s.mp4"
log="$LOG_DIR/camera_record_${stamp}.log"

nohup env \
  MVCAM_COMMON_RUNENV=/opt/MVS/lib \
  LD_LIBRARY_PATH=/opt/MVS/lib/aarch64:/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/nvidia \
  python3 -u "$RECORDER" \
    --duration 300 \
    --fps 60 \
    --bitrate-kbps 12000 \
    --output "$output" \
  >"$log" 2>&1 </dev/null &
pid=$!
echo "$pid" > "$LOG_DIR/camera_recording.pid"
ln -sfn "$log" "$LOG_DIR/camera_record_latest.log"

sleep 2
if ! kill -0 "$pid" 2>/dev/null; then
  cat "$log" >&2
  rm -f "$LOG_DIR/camera_recording.pid"
  exit 1
fi

echo "recording started"
echo "pid=$pid"
echo "video=$output"
echo "log=$log"
echo "automatic_stop_seconds=300"
