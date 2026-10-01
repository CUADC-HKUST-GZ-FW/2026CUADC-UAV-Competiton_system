#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
CONFIG="${CONFIG:-$ROOT/configs/youth_pipeline.yaml}"
MODE="${1:-image}"
SOURCE="${2:-}"

if [[ "$MODE" != "image" && "$MODE" != "digit" ]]; then
  echo "mode must be image or digit" >&2
  exit 2
fi

if [[ "${YOUTH_MAX_PERF:-1}" == "1" ]]; then
  if ! python3 "$ROOT/scripts/enable_max_performance.py"; then
    echo "warning: max performance setup failed; continuing with current clocks" >&2
  fi
fi

echo "$MODE" > "$ROOT/configs/youth_runtime_mode.txt"

cmd=(
  "$ROOT/native/build/youth_vision_runner"
  --config "$CONFIG"
  --class-mode "$MODE"
)

if [[ -n "$SOURCE" ]]; then
  cmd+=(--source "$SOURCE")
fi

exec "${cmd[@]}"
