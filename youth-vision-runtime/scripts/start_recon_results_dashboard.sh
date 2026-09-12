#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
PORT="${PORT:-8092}"
RADIUS_M="${RADIUS_M:-8.0}"
PID_FILE="$ROOT/run/recon_results_dashboard.pid"
LOG_FILE="$ROOT/logs/recon_results_dashboard.log"

mkdir -p "$ROOT/run" "$ROOT/logs"
if [[ -f "$PID_FILE" ]]; then
  old_pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ -n "$old_pid" ]] && kill -0 "$old_pid" 2>/dev/null; then
    kill "$old_pid" 2>/dev/null || true
    sleep 0.3
  fi
  rm -f "$PID_FILE"
fi

setsid python3 "$ROOT/scripts/recon_results_dashboard.py" \
  --bind 0.0.0.0 \
  --port "$PORT" \
  --root "$ROOT/recon_results" \
  --dedup-radius-m "$RADIUS_M" \
  >"$LOG_FILE" 2>&1 </dev/null &
pid=$!
echo "$pid" > "$PID_FILE"
sleep 0.5

if ! kill -0 "$pid" 2>/dev/null; then
  tail -n 30 "$LOG_FILE" >&2 || true
  exit 1
fi

echo "侦察结果面板已启动"
echo "USB:  http://192.168.55.1:$PORT/"
for address in $(hostname -I); do
  [[ "$address" == 127.* ]] && continue
  echo "网络: http://$address:$PORT/"
done
echo "去重半径: $RADIUS_M m"
echo "日志: $LOG_FILE"
