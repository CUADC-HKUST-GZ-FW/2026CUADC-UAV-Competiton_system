#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
LOG="$ROOT/logs/recognition_events_latest.jsonl"
LINES="${LINES:-20}"

if [[ ! -e "$LOG" ]]; then
  echo "no recognition log exists; start the realtime pipeline first" >&2
  exit 1
fi

echo "following $(readlink -f "$LOG")" >&2
exec tail -n "$LINES" -F "$LOG"
