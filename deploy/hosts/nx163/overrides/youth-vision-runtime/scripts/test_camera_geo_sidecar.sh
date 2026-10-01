#!/usr/bin/env bash
set -euo pipefail

# Same camera grab as start_camera_recording_300s.sh (1440x1080, 60 fps, 300 s,
# NVIDIA H.264), plus a per-frame lat/lon/alt JSONL written in the same loop.
# Video and sidecar are stored in one directory.

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_SETUP:-/home/nx163/uav_ros2_project/install/setup.bash}"
OUTPUT_ROOT="${OUTPUT_ROOT:-$ROOT/logs/geo_sidecar_tests}"
LOG_DIR="$ROOT/logs"
RECORDER="$ROOT/scripts/record_camera_300s.py"
DURATION_SEC="${DURATION_SEC:-300}"

usage() {
  echo "usage: $0 [--duration SEC]" >&2
  echo "  records 1440x1080 camera frames for up to 300 seconds" >&2
  echo "  and writes a matching lat/lon/alt JSONL beside the MP4" >&2
  exit 2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)
      DURATION_SEC="$2"
      shift 2
      ;;
    --duration=*)
      DURATION_SEC="${1#--duration=}"
      shift
      ;;
    -h|--help)
      usage
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage
      ;;
  esac
done

if ! [[ "$DURATION_SEC" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  echo "duration must be a number of seconds" >&2
  exit 2
fi

python3 - "$DURATION_SEC" <<'PY'
import sys
value = float(sys.argv[1])
if not 1.0 <= value <= 300.0:
    raise SystemExit("duration must be between 1 and 300 seconds")
PY

if [[ ! -r "$ROS_SETUP" ]]; then
  echo "ROS setup not found: $ROS_SETUP" >&2
  echo "start MAVROS / real_bringup first; see TEST_CAMERA_GEO_SIDECAR.md" >&2
  exit 1
fi

mkdir -p "$OUTPUT_ROOT" "$LOG_DIR"

if [[ -f "$LOG_DIR/camera_recording.pid" ]]; then
  active_pid="$(cat "$LOG_DIR/camera_recording.pid" 2>/dev/null || true)"
  active_command="$(ps -p "$active_pid" -o args= 2>/dev/null || true)"
  if [[ "$active_command" == *"$RECORDER"* ]]; then
    echo "recording already running: pid=$active_pid" >&2
    exit 1
  fi
  rm -f "$LOG_DIR/camera_recording.pid"
fi

"$ROOT/scripts/stop_youth_pipeline.sh" >/dev/null
sleep 1

stamp="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$OUTPUT_ROOT/run_${stamp}"
mkdir -p "$OUT_DIR"
output="$OUT_DIR/camera_${stamp}_1440x1080_300s.mp4"
sidecar="${output%.mp4}.jsonl"
log="$OUT_DIR/camera_geo_record.log"

set +u
# shellcheck disable=SC1090
source "$ROS_SETUP"
if [[ -r "$WS_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$WS_SETUP"
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PYTHONPATH="$ROOT/geo_bridge${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONUNBUFFERED=1
export MVCAM_COMMON_RUNENV="${MVCAM_COMMON_RUNENV:-/opt/MVS/lib}"
export LD_LIBRARY_PATH="/opt/MVS/lib/aarch64:/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu/nvidia${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

nohup python3 -u "$RECORDER" \
  --duration "$DURATION_SEC" \
  --fps 60 \
  --bitrate-kbps 12000 \
  --output "$output" \
  --sidecar "$sidecar" \
  >"$log" 2>&1 </dev/null &
pid=$!
echo "$pid" > "$LOG_DIR/camera_recording.pid"
ln -sfn "$log" "$LOG_DIR/camera_geo_record_latest.log"

sleep 2
if ! kill -0 "$pid" 2>/dev/null; then
  cat "$log" >&2
  rm -f "$LOG_DIR/camera_recording.pid"
  exit 1
fi

echo "recording started"
echo "pid=$pid"
echo "output_dir=$OUT_DIR"
echo "video=$output"
echo "sidecar=$sidecar"
echo "log=$log"
echo "automatic_stop_seconds=$DURATION_SEC"
echo "stop_early=$ROOT/scripts/stop_camera_recording.sh"
echo "follow_log=tail -f $log"
