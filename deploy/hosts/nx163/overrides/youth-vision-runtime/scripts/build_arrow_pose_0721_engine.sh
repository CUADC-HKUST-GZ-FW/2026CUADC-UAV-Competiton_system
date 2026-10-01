#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/nx163/youth-vision-runtime}"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"

"$TRTEXEC" \
  --onnx="$ROOT/onnx/arrow_pose_0721_1024x768.onnx" \
  --saveEngine="$ROOT/engines/arrow_pose_0721_1024x768_fp16.raw.engine" \
  --fp16 \
  --memPoolSize=workspace:2048 \
  --builderOptimizationLevel=3
