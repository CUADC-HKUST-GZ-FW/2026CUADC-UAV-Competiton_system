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
