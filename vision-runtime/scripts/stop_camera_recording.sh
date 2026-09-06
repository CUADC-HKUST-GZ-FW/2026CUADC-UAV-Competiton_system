#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
RECORDER="$ROOT/scripts/record_camera_300s.py"
PID_FILE="$ROOT/logs/camera_recording.pid"
pid=""

if [[ -f "$PID_FILE" ]]; then
  candidate="$(cat "$PID_FILE" 2>/dev/null || true)"
  command_line="$(ps -p "$candidate" -o args= 2>/dev/null || true)"
  if [[ "$command_line" == *"$RECORDER"* ]]; then
    pid="$candidate"
  fi
fi

if [[ -z "$pid" ]]; then
  pid="$(pgrep -f "^python3 -u $RECORDER " | head -1 || true)"
fi

if [[ -z "$pid" ]]; then
  rm -f "$PID_FILE"
  echo "recording is not running"
  exit 0
fi

kill -TERM "$pid"
for _ in $(seq 1 60); do
  if ! kill -0 "$pid" 2>/dev/null; then
    rm -f "$PID_FILE"
    echo "recording stopped and MP4 finalized: pid=$pid"
    exit 0
  fi
  sleep 0.25
done

echo "recording did not finalize within 15 seconds: pid=$pid" >&2
echo "do not use kill -9 unless losing the current MP4 is acceptable" >&2
exit 1
