#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
MODE="${1:-digit}"
CONFIG="${CONFIG:-$ROOT/configs/youth_pipeline.yaml}"
LOG_DIR="$ROOT/logs"
SESSION_DIR="$LOG_DIR/sessions"
EVENT_DIR="$LOG_DIR/recognition"
LIB_PATH="/opt/MVS/lib/aarch64:/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/nvidia"

if [[ "$MODE" != "digit" && "$MODE" != "image" ]]; then
  echo "usage: $0 [digit|image]" >&2
  exit 2
fi

if [[ -f "$LOG_DIR/camera_recording.pid" ]]; then
  recording_pid="$(cat "$LOG_DIR/camera_recording.pid" 2>/dev/null || true)"
  recording_command="$(ps -p "$recording_pid" -o args= 2>/dev/null || true)"
  if [[ "$recording_command" == *"$ROOT/scripts/record_camera_150s.py"* ]]; then
    echo "camera recording is still running: pid=$recording_pid" >&2
    echo "wait for recording to finish before starting inference" >&2
    exit 1
  fi
  rm -f "$LOG_DIR/camera_recording.pid"
fi

"$ROOT/scripts/stop_youth_pipeline.sh" >/dev/null
sleep 1

sudo -n /usr/sbin/nvpmodel -m 0 >/dev/null
sudo -n /usr/bin/jetson_clocks >/dev/null

mkdir -p "$LOG_DIR" "$SESSION_DIR" "$EVENT_DIR"
echo "$MODE" > "$ROOT/configs/youth_runtime_mode.txt"

stamp="$(date +%Y%m%d_%H%M%S)"
session_id="${stamp}_${MODE}_headless"
session_log="$SESSION_DIR/youth_pipeline_${session_id}.log"
event_log="$EVENT_DIR/recognition_events_${session_id}.jsonl"
legacy_log="$LOG_DIR/youth_pipeline_${MODE}.log"

if [[ -f "$legacy_log" && ! -L "$legacy_log" ]]; then
  mv "$legacy_log" "$SESSION_DIR/youth_pipeline_${MODE}_before_headless_${stamp}.log"
fi

: > "$session_log"
: > "$event_log"
ln -sfn "$session_log" "$legacy_log"
ln -sfn "$event_log" "$LOG_DIR/recognition_events_${MODE}.jsonl"
ln -sfn "$event_log" "$LOG_DIR/recognition_events_latest.jsonl"
rm -f "$LOG_DIR/live_capture_server.pid"

nohup env \
  YOUTH_SESSION_ID="$session_id" \
  YOUTH_RECOGNITION_LOG="$event_log" \
  LD_LIBRARY_PATH="$LIB_PATH" \
  "$ROOT/native/build/youth_vision_runner" \
    --config "$CONFIG" \
    --class-mode "$MODE" \
    --headless \
  > "$session_log" 2>&1 </dev/null &
pid=$!
echo "$pid" > "$LOG_DIR/youth_pipeline.pid"

for _ in $(seq 1 20); do
  if grep -q '"fps":[1-9]' "$session_log" 2>/dev/null; then
    echo "headless pipeline ready: mode=$MODE pid=$pid"
    echo "session_log=$session_log"
    echo "recognition_log=$event_log"
    exit 0
  fi
  if ! kill -0 "$pid" 2>/dev/null; then
    tail -30 "$session_log" >&2
    rm -f "$LOG_DIR/youth_pipeline.pid"
    exit 1
  fi
  sleep 1
done

echo "headless pipeline started; warmup is still in progress"
echo "pid=$pid"
echo "session_log=$session_log"
echo "recognition_log=$event_log"
