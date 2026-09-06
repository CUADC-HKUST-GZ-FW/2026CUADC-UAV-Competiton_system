#!/usr/bin/env bash
set -euo pipefail

ROOT="${ROOT:-/home/hkustgz26/youth-vision-runtime}"
TRTEXEC="${TRTEXEC:-/usr/src/tensorrt/bin/trtexec}"
mkdir -p "$ROOT/engines"

"$TRTEXEC" \
  --onnx="$ROOT/onnx/arrow_pose_1536.onnx" \
  --saveEngine="$ROOT/engines/arrow_pose_1536_fp16.raw.engine" \
  --fp16 \
  --memPoolSize=workspace:2048 \
  --builderOptimizationLevel=0

"$TRTEXEC" \
  --onnx="$ROOT/onnx/digit_cls_128.onnx" \
  --saveEngine="$ROOT/engines/digit_cls_128_fp16.raw.engine" \
  --fp16 \
  --memPoolSize=workspace:1024

"$TRTEXEC" \
  --onnx="$ROOT/onnx/image_cls_640.onnx" \
  --saveEngine="$ROOT/engines/image_cls_640_fp16.raw.engine" \
  --fp16 \
  --memPoolSize=workspace:2048
