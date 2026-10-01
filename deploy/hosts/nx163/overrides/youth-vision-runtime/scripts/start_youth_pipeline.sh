#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
MODE="${1:-digit}"
WEB_PORT="${WEB_PORT:-8000}"
LOG_DIR="$ROOT/logs"
SESSION_DIR="$LOG_DIR/sessions"
EVENT_DIR="$LOG_DIR/recognition"
LIB_PATH="/opt/MVS/lib/aarch64:/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/nvidia"

if [[ "$MODE" != "digit" && "$MODE" != "image" ]]; then
  echo "usage: $0 [digit|image]" >&2
  exit 2
fi

"$ROOT/scripts/stop_youth_pipeline.sh"
sleep 3

sudo -n /usr/sbin/nvpmodel -m 0 >/dev/null
sudo -n /usr/bin/jetson_clocks >/dev/null

mkdir -p "$LOG_DIR" "$SESSION_DIR" "$EVENT_DIR" "$ROOT/overlays/latest_crops" "$ROOT/collections/digit_inputs"
echo "$MODE" > "$ROOT/configs/youth_runtime_mode.txt"

stamp="$(date +%Y%m%d_%H%M%S)"
session_id="${stamp}_${MODE}"
session_log="$SESSION_DIR/youth_pipeline_${session_id}.log"
event_log="$EVENT_DIR/recognition_events_${session_id}.jsonl"
web_log="$SESSION_DIR/live_capture_server_${stamp}.log"
legacy_log="$LOG_DIR/youth_pipeline_${MODE}.log"
legacy_web_log="$LOG_DIR/live_capture_server.log"

if [[ -f "$legacy_log" && ! -L "$legacy_log" ]]; then
  mv "$legacy_log" "$SESSION_DIR/youth_pipeline_${MODE}_before_full_logging_${stamp}.log"
fi
if [[ -f "$legacy_web_log" && ! -L "$legacy_web_log" ]]; then
  mv "$legacy_web_log" "$SESSION_DIR/live_capture_server_before_full_logging_${stamp}.log"
fi

: > "$session_log"
: > "$event_log"
: > "$web_log"
ln -sfn "$session_log" "$legacy_log"
ln -sfn "$event_log" "$LOG_DIR/recognition_events_${MODE}.jsonl"
ln -sfn "$event_log" "$LOG_DIR/recognition_events_latest.jsonl"
ln -sfn "$web_log" "$legacy_web_log"

nohup env \
  YOUTH_MAX_PERF=0 \
  YOUTH_SESSION_ID="$session_id" \
  YOUTH_RECOGNITION_LOG="$event_log" \
  LD_LIBRARY_PATH="$LIB_PATH" \
  "$ROOT/run_youth_vision.sh" "$MODE" \
  > "$session_log" 2>&1 </dev/null &
echo $! > "$LOG_DIR/youth_pipeline.pid"

nohup python3 "$ROOT/scripts/live_capture_server.py" \
  --bind 0.0.0.0 \
  --port "$WEB_PORT" \
  --web-root "$ROOT/overlays" \
  --capture-root "$ROOT/collections/digit_inputs" \
  > "$web_log" 2>&1 </dev/null &
echo $! > "$LOG_DIR/live_capture_server.pid"

for _ in $(seq 1 20); do
  if grep -q '"fps":[1-9]' "$session_log" 2>/dev/null; then
    echo "pipeline ready: mode=$MODE port=$WEB_PORT"
    echo "session_log=$session_log"
    echo "recognition_log=$event_log"
    exit 0
  fi
  if ! kill -0 "$(cat "$LOG_DIR/youth_pipeline.pid")" 2>/dev/null; then
    tail -30 "$session_log" >&2
    exit 1
  fi
  sleep 1
done

echo "pipeline started; warmup is still in progress"
echo "session_log=$session_log"
echo "recognition_log=$event_log"
