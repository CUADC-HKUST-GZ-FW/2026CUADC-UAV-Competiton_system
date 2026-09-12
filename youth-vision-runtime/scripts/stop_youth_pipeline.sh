#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"

stop_matching() {
  local pattern="$1"
  local pids
  pids="$(pgrep -f "$pattern" || true)"
  [[ -z "$pids" ]] && return 0

  kill -TERM $pids 2>/dev/null || true
  for _ in $(seq 1 20); do
    sleep 0.25
    pids="$(pgrep -f "$pattern" || true)"
    [[ -z "$pids" ]] && return 0
  done

  echo "process did not stop cleanly: $pattern" >&2
  return 1
}

stop_matching "^$ROOT/native/build/youth_vision_runner "
stop_matching "^python3 $ROOT/scripts/live_capture_server.py "
stop_matching "^python3 /home/nx163/camera_focus_server.py$"
stop_matching "python3 -m geo_bridge.node"
stop_matching "$ROOT/geo_bridge/scripts/test_geo_overlay_record.py"

rm -f \
  "$ROOT/logs/youth_pipeline.pid" \
  "$ROOT/logs/live_capture_server.pid" \
  "$ROOT/logs/geo_bridge.pid"
echo "pipeline stopped"
