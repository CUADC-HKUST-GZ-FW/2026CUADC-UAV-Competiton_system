#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
MODE="$(cat "$ROOT/configs/youth_runtime_mode.txt" 2>/dev/null || echo digit)"

echo "mode=$MODE"
pgrep -af "^$ROOT/native/build/youth_vision_runner " || echo "runner=stopped"
pgrep -af "^python3 $ROOT/scripts/live_capture_server.py " || echo "web=stopped"
pgrep -af "python3 -m geo_bridge.node" || echo "geo_bridge=stopped"
echo "session_log=$(readlink -f "$ROOT/logs/youth_pipeline_${MODE}.log" 2>/dev/null || echo unavailable)"
echo "recognition_log=$(readlink -f "$ROOT/logs/recognition_events_latest.jsonl" 2>/dev/null || echo unavailable)"
echo "geo_log=$(readlink -f "$ROOT/logs/geo_bridge_latest.log" 2>/dev/null || echo unavailable)"
echo "fused_log=$ROOT/logs/geo_bridge/fused_latest.jsonl"
echo "latest_status:"
tail -1 "$ROOT/logs/youth_pipeline_${MODE}.log" 2>/dev/null || true
echo "latest_recognition:"
tail -1 "$ROOT/logs/recognition_events_latest.jsonl" 2>/dev/null || true
echo "latest_geo:"
tail -1 "$ROOT/logs/geo_bridge_latest.log" 2>/dev/null || true
