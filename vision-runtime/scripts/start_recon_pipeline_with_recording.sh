#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
MODE="${1:-digit}"
DURATION="${2:-300}"
EXPECTED_TARGETS="${3:-0}"
OUTPUT_DIR="${OUTPUT_DIR:-/home/nx163/camera_recordings}"

if [[ "$MODE" != "digit" && "$MODE" != "image" ]]; then
  echo "usage: $0 [digit|image] [duration_seconds] [expected_targets]" >&2
  exit 2
fi
if ! [[ "$DURATION" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "duration_seconds must be numeric" >&2
  exit 2
fi
if ! awk -v duration="$DURATION" 'BEGIN { exit !(duration >= 1 && duration <= 300) }'; then
  echo "duration_seconds must be between 1 and 300" >&2
  exit 2
fi
if ! [[ "$EXPECTED_TARGETS" =~ ^[0-9]+$ ]] || (( EXPECTED_TARGETS > 20 )); then
  echo "expected_targets must be an integer from 0 to 20; 0 means open-ended" >&2
  exit 2
fi

mkdir -p "$OUTPUT_DIR"
available_kb="$(df -Pk "$OUTPUT_DIR" | awk 'NR==2 {print $4}')"
if [[ -z "$available_kb" || "$available_kb" -lt 2097152 ]]; then
  echo "recording requires at least 2 GiB free in $OUTPUT_DIR" >&2
  exit 1
fi

stamp="$(date +%Y%m%d_%H%M%S)"
record_file="$OUTPUT_DIR/camera_${stamp}_1440x1080_${MODE}_recon.mp4"

export YOUTH_RECORD_FILE="$record_file"
export YOUTH_RECORD_DURATION_SEC="$DURATION"
export YOUTH_RECORD_FPS="${YOUTH_RECORD_FPS:-60}"
export YOUTH_RECORD_BITRATE_KBPS="${YOUTH_RECORD_BITRATE_KBPS:-12000}"

exec "$ROOT/scripts/start_recon_pipeline.sh" "$MODE" "$EXPECTED_TARGETS"
