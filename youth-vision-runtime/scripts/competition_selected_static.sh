#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
MODE="${1:-}"
HEIGHT_M="${2:-}"

if [[ "$MODE" != "digit" && "$MODE" != "image" ]]; then
  echo "usage: $0 [digit|image] FIXED_RELATIVE_HEIGHT_M" >&2
  exit 2
fi

if ! python3 - "$HEIGHT_M" <<'PY'
import math
import sys

try:
    height = float(sys.argv[1])
except (IndexError, ValueError):
    raise SystemExit(1)
raise SystemExit(0 if math.isfinite(height) and 0.0 < height <= 1000.0 else 1)
PY
then
  echo "fixed relative height must be a finite number in (0, 1000] metres" >&2
  exit 2
fi

export RECON_STATIC_HEIGHT_M="$HEIGHT_M"
export RECON_MAX_HORIZONTAL_RADIUS_95_M="0.5"

exec "$ROOT/scripts/competition_selected.sh" "$MODE"
