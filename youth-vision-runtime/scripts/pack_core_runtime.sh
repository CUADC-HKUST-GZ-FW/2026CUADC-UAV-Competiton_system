#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
LIST="${LIST:-$ROOT/core_runtime_files.txt}"
OUT="${1:-$HOME/youth-vision-core-runtime.tar.gz}"

if [[ ! -f "$LIST" ]]; then
  echo "missing file list: $LIST" >&2
  exit 1
fi

mapfile -t files < <(grep -vE '^\s*(#|$)' "$LIST")
if [[ ${#files[@]} -eq 0 ]]; then
  echo "file list is empty: $LIST" >&2
  exit 1
fi

missing=0
for f in "${files[@]}"; do
  if [[ ! -e "$ROOT/$f" ]]; then
    echo "missing: $f" >&2
    missing=1
  fi
done
if [[ "$missing" -ne 0 ]]; then
  exit 1
fi

mkdir -p "$(dirname "$OUT")"
tar -C "$ROOT" -czf "$OUT" "${files[@]}"
echo "packed ${#files[@]} files -> $OUT"
ls -lh "$OUT"
