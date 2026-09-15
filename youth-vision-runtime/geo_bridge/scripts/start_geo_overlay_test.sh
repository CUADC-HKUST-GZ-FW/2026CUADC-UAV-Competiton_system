#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
DURATION="${1:-20}"
MOCK_ALT="${MOCK_ALT:-32.5}"
LOG_DIR="$ROOT/logs/geo_recordings"

mkdir -p "$LOG_DIR"
stamp="$(date +%Y%m%d_%H%M%S)"
log="$LOG_DIR/geo_overlay_test_${stamp}.log"

export PYTHONPATH="$ROOT/geo_bridge${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONUNBUFFERED=1

echo "bench recorder: mock lat/lon/alt duration=${DURATION}s mock_alt=${MOCK_ALT}m"
exec python3 -u "$ROOT/geo_bridge/scripts/test_geo_overlay_record.py" \
  --mock-alt "$MOCK_ALT" \
  --duration-sec "$DURATION"
