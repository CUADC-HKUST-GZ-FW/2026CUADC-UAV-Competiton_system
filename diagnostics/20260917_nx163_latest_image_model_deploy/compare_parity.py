#!/usr/bin/env python3
"""Compare the candidate ONNX output with the NX163 FP16 TensorRT engine."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import onnxruntime as ort


ROOT = Path(__file__).resolve().parents[2]
AUDIT_DIR = Path(__file__).resolve().parent
INPUT_DIR = ROOT / "diagnostics" / "20260907_backend_parity_audit" / "inputs" / "image"
TRT_DIR = AUDIT_DIR / "tensorrt_outputs"
ONNX_PATH = (
    ROOT
    / "runs"
    / "real_blank_gloo_0915"
    / "head_tuned_20260915_220620"
    / "candidate_package_not_deployed"
    / "image_real_blank_gloo_best.onnx"
)
LABELS_PATH = (
    ROOT
    / "github_publish"
    / "2026CUADC-UAV-Competiton_system"
    / "youth-vision-runtime"
    / "configs"
    / "youth_classes_image.txt"
)


def main() -> None:
    labels = LABELS_PATH.read_text(encoding="utf-8").splitlines()
    session = ort.InferenceSession(str(ONNX_PATH), providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    rows: list[dict[str, object]] = []

    for input_path in sorted(INPUT_DIR.glob("*.bin")):
        tensor = np.fromfile(input_path, dtype=np.float32).reshape(1, 3, 128, 128)
        onnx_scores = session.run(None, {input_name: tensor})[0].reshape(-1).astype(np.float32)
        trt_payload = json.loads((TRT_DIR / f"{input_path.stem}.json").read_text(encoding="utf-8"))
        trt_scores = np.asarray(trt_payload[0]["values"], dtype=np.float32)
        if onnx_scores.shape != trt_scores.shape or len(labels) != onnx_scores.size:
            raise RuntimeError(f"shape mismatch for {input_path.name}")

        onnx_id = int(np.argmax(onnx_scores))
        trt_id = int(np.argmax(trt_scores))
        rows.append(
            {
                "sample": input_path.stem,
                "truth": input_path.stem.split("__", 1)[0],
                "onnx_id": onnx_id,
                "onnx_label": labels[onnx_id],
                "onnx_score": float(onnx_scores[onnx_id]),
                "trt_id": trt_id,
                "trt_label": labels[trt_id],
                "trt_score": float(trt_scores[trt_id]),
                "top1_match": onnx_id == trt_id,
                "max_abs_error": float(np.max(np.abs(onnx_scores - trt_scores))),
            }
        )

    result = {
        "sample_count": len(rows),
        "top1_matches": sum(bool(row["top1_match"]) for row in rows),
        "maximum_abs_error": max(float(row["max_abs_error"]) for row in rows),
        "rows": rows,
    }
    (AUDIT_DIR / "parity.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps({key: value for key, value in result.items() if key != "rows"}, indent=2))


if __name__ == "__main__":
    main()
