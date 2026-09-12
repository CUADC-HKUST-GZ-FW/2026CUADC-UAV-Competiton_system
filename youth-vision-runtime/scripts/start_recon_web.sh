#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
MODE="${1:-digit}"
WEB_PORT="${WEB_PORT:-8000}"
RUN_DIR="$ROOT/run"
LOG_DIR="$ROOT/logs/recon"

"$ROOT/scripts/start_recon_pipeline.sh" "$MODE"
session_id="$(basename "$(readlink -f "$ROOT/recon_results/latest")")"
web_log="$LOG_DIR/web_${session_id}.log"

setsid python3 "$ROOT/scripts/live_capture_server.py" \
  --bind 0.0.0.0 \
  --port "$WEB_PORT" \
  --web-root "$ROOT/overlays" \
  --capture-root "$ROOT/collections/digit_inputs" \
  --recon-root "$ROOT/recon_results/latest" \
  > "$web_log" 2>&1 </dev/null &
web_pid=$!
echo "$web_pid" > "$RUN_DIR/recon_web.pid"

sleep 1
if ! kill -0 "$web_pid" 2>/dev/null; then
  "$ROOT/scripts/stop_recon_pipeline.sh" >/dev/null 2>&1 || true
  tail -30 "$web_log" >&2 || true
  exit 1
fi

echo "recon web ready"
echo "usb_url=http://192.168.55.1:$WEB_PORT/index_recon.html"
echo "wifi_url=http://$(hostname -I | awk '{print $1}'):$WEB_PORT/index_recon.html"
echo "web_log=$web_log"
