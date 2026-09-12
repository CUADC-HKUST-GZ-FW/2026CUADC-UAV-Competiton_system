#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
PID_FILE="$ROOT/run/recon_results_dashboard.pid"

if [[ ! -f "$PID_FILE" ]]; then
  echo "侦察结果面板未运行"
  exit 0
fi

pid="$(cat "$PID_FILE" 2>/dev/null || true)"
if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
  kill "$pid"
  for _ in {1..20}; do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.1
  done
fi
rm -f "$PID_FILE"
echo "侦察结果面板已关闭"
