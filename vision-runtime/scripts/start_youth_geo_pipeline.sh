#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
MODE="${1:-digit}"
RECORD="${RECORD:-1}"
WEB_PORT="${WEB_PORT:-8000}"
LOG_DIR="$ROOT/logs"
GEO_DIR="$LOG_DIR/geo_bridge"
RECORD_DIR="$LOG_DIR/geo_recordings"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_SETUP:-/home/nx163/uav_ros2_project/install/setup.bash}"
CONFIG="${CONFIG:-$ROOT/geo_bridge/config/geo_bridge.yaml}"

if [[ "$MODE" != "digit" && "$MODE" != "image" ]]; then
  echo "usage: $0 [digit|image]" >&2
  exit 2
fi

shift || true
while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-record) RECORD=0 ;;
    --record) RECORD=1 ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
  shift
done

mkdir -p "$GEO_DIR" "$RECORD_DIR"
"$ROOT/scripts/start_youth_pipeline.sh" "$MODE"

stamp="$(date +%Y%m%d_%H%M%S)"
geo_log="$GEO_DIR/geo_bridge_${stamp}_${MODE}.log"
fused_log="$GEO_DIR/fused_${stamp}_${MODE}.jsonl"
ln -sfn "$geo_log" "$LOG_DIR/geo_bridge_latest.log"
ln -sfn "$fused_log" "$GEO_DIR/fused_latest.jsonl"
: > "$geo_log"
: > "$fused_log"

if [[ ! -r "$ROS_SETUP" ]]; then
  echo "[GEO] ROS setup not found: $ROS_SETUP" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
if [[ -r "$WS_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$WS_SETUP"
fi
set -u

export PYTHONPATH="$ROOT/geo_bridge${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONUNBUFFERED=1

extra_args=(-p "fused_log:=$fused_log")
if [[ "$RECORD" != "1" ]]; then
  extra_args+=(-p record_video:=false)
fi

nohup python3 -m geo_bridge.node \
  --ros-args \
  --params-file "$CONFIG" \
  "${extra_args[@]}" \
  > "$geo_log" 2>&1 </dev/null &
echo $! > "$LOG_DIR/geo_bridge.pid"

sleep 2
if ! kill -0 "$(cat "$LOG_DIR/geo_bridge.pid")" 2>/dev/null; then
  tail -30 "$geo_log" >&2
  rm -f "$LOG_DIR/geo_bridge.pid"
  exit 1
fi

echo "geo pipeline ready: mode=$MODE record=$RECORD port=$WEB_PORT"
echo "recognition_log=$LOG_DIR/recognition_events_latest.jsonl"
echo "geo_log=$geo_log"
echo "annotated_jpeg=$ROOT/overlays/latest_geo.jpg"
echo "recordings=$RECORD_DIR"
echo "fused_log=$GEO_DIR/fused_latest.jsonl"
