#!/usr/bin/env python3
"""Offline D435i depth-file to ABS Ray-Pred frame.

This entry reads one saved .npy depth image. It does not access a camera SDK,
ROS, DDS, or shared memory. The binary .frame is a file serialization of
FrameHeader followed by the 11 float32 log2-distance payload values.
"""

from __future__ import annotations

import argparse
import json
import math
import struct
import time
from pathlib import Path

import numpy as np
from ray_predictor import CAM_H as INPUT_H, CAM_W as INPUT_W
from ray_predictor import HEADER as RAY_HEADER
from ray_predictor import RAY_COUNT, RAY_MAX_M, RAY_MIN_M

RAY_ANGLES_DEG = [-45.0 + 9.0 * i for i in range(RAY_COUNT)]
RAY_MAGIC = 0x415253594143544F
RAY_VERSION = 2
RAY_PAYLOAD = struct.Struct("<11f")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Infer ABS 11-ray values from one saved .npy depth image; no live device access."
    )
    parser.add_argument("--input", required=True, type=Path, help="single 2-D .npy depth image")
    parser.add_argument("--unit", required=True, choices=("m", "z16"),
                        help="m: metric depth; z16: uint16 raw depth units")
    parser.add_argument("--z16-scale-m", type=float,
                        help="meters per Z16 unit; required for --unit z16, no implicit default")
    parser.add_argument("--model", required=True, type=Path,
                        help="explicit TorchScript Ray-Pred model; there is no fallback")
    parser.add_argument("--output-prefix", required=True, type=Path,
                        help="writes PREFIX.json and, only for a valid prediction, PREFIX.frame")
    return parser.parse_args()


def read_depth(path: Path, unit: str, z16_scale_m: float | None) -> tuple[np.ndarray, np.ndarray, dict]:
    raw = np.load(path, allow_pickle=False)
    if raw.ndim != 2 or raw.dtype.kind not in "uif":
        raise ValueError("depth .npy must be a 2-D integer or floating array")

    if unit == "z16":
        if raw.dtype != np.uint16:
            raise ValueError("--unit z16 requires a uint16 .npy array")
        if z16_scale_m is None or not math.isfinite(z16_scale_m) or z16_scale_m <= 0.0:
            raise ValueError("--unit z16 requires an explicit positive --z16-scale-m")
        depth_m = raw.astype(np.float32) * np.float32(z16_scale_m)
        valid = raw > 0
        scale_m = float(z16_scale_m)
    else:
        if z16_scale_m is not None:
            raise ValueError("--z16-scale-m is only valid with --unit z16")
        depth_m = raw.astype(np.float32)
        valid = np.isfinite(depth_m) & (depth_m > 0.0)
        scale_m = 1.0

    # Z16 zero and non-finite/non-positive metric values are unknown, not free
    # space. This decision occurs before range clipping.
    valid &= np.isfinite(depth_m) & (depth_m > 0.0)
    metadata = {
        "input_path": str(path.resolve()),
        "input_dtype": str(raw.dtype),
        "input_shape_hw": [int(raw.shape[0]), int(raw.shape[1])],
        "input_unit": unit,
        "z16_scale_m_per_unit": scale_m if unit == "z16" else None,
        "valid_pixel_count": int(valid.sum()),
        "invalid_pixel_count": int(valid.size - valid.sum()),
        "valid_pixel_fraction": float(valid.mean()),
    }
    return depth_m, valid, metadata


def downsample_depth_centers(depth_m: np.ndarray, valid: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    height, width = depth_m.shape
    if width * INPUT_H != height * INPUT_W:
        raise ValueError(
            "input aspect ratio must be 16:9 to match the trained 160x90 view; "
            "crop/warp policy is not implicit"
        )
    if width < INPUT_W or height < INPUT_H:
        raise ValueError("input must be at least 160x90; this adapter does not upsample depth")

    sampled = np.empty((INPUT_H, INPUT_W), dtype=np.float32)
    sampled_valid = np.empty((INPUT_H, INPUT_W), dtype=bool)
    for out_y in range(INPUT_H):
        y0 = out_y * height // INPUT_H
        y1 = (out_y + 1) * height // INPUT_H
        sample_y = min(height - 1, ((2 * out_y + 1) * height) // (2 * INPUT_H))
        for out_x in range(INPUT_W):
            x0 = out_x * width // INPUT_W
            x1 = (out_x + 1) * width // INPUT_W
            sample_x = min(width - 1, ((2 * out_x + 1) * width) // (2 * INPUT_W))
            sampled[out_y, out_x] = depth_m[sample_y, sample_x]
            # Do not lose invalid data hidden elsewhere in this output pixel's
            # source footprint.
            sampled_valid[out_y, out_x] = bool(valid[y0:y1, x0:x1].all())
    return sampled, sampled_valid


def main() -> int:
    args = parse_args()
    if not args.input.is_file():
        raise FileNotFoundError("depth input does not exist: {}".format(args.input))
    if not args.model.is_file():
        raise FileNotFoundError("explicit model does not exist: {}".format(args.model))

    prefix = args.output_prefix
    prefix.parent.mkdir(parents=True, exist_ok=True)
    json_path = Path(str(prefix) + ".json")
    frame_path = Path(str(prefix) + ".frame")
    if json_path.exists() or frame_path.exists():
        raise FileExistsError("output exists; choose a new --output-prefix")

    depth_m, valid, input_info = read_depth(args.input, args.unit, args.z16_scale_m)
    resized_m, resized_valid = downsample_depth_centers(depth_m, valid)
    input_info["model_input_shape_hwc"] = [INPUT_H, INPUT_W, 3]
    input_info["model_input_resize"] = "nearest pixel-center sample; all source pixels in each footprint must be valid"
    input_info["resized_valid_pixel_count"] = int(resized_valid.sum())
    input_info["resized_invalid_pixel_count"] = int(resized_valid.size - resized_valid.sum())

    report = {
        "status": "rejected_invalid_depth",
        "model_path": str(args.model.resolve()),
        "input": input_info,
        "timestamps": {
            "capture_monotonic_ns": None,
            "computer_receive_monotonic_ns": None,
            "inference_started_monotonic_ns": None,
            "inference_complete_monotonic_ns": None,
        },
        "inference_ms": None,
        "protocol": {
            "magic_hex": "0x{:016x}".format(RAY_MAGIC),
            "version": RAY_VERSION,
            "sequence": None,
            "payload_count": RAY_COUNT,
            "payload_dtype": "float32",
            "payload_unit": "log2(m)",
            "ray_angles_deg_increasing": RAY_ANGLES_DEG,
            "frame_file": None,
        },
        "prediction_valid": False,
        "rays_log2_m": None,
        "rays_m": None,
    }

    if not resized_valid.all():
        json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print("[RayOffline] rejected: {} resized pixels contain unknown depth; wrote {}".format(
            input_info["resized_invalid_pixel_count"], json_path))
        return 2

    import torch

    torch.set_num_threads(1)
    model = torch.jit.load(str(args.model.resolve()), map_location="cpu")
    model.eval()
    clipped_m = np.clip(resized_m, RAY_MIN_M, RAY_MAX_M)
    depth_log = np.log2(clipped_m).astype(np.float32)
    # Use a writable byte buffer instead of NumPy's Torch bridge. The host's
    # Torch build warns that it was compiled against NumPy 1.x while the
    # available NumPy is 2.x; the byte-buffer path keeps this small offline
    # conversion functional without changing installed packages.
    input_tensor = torch.frombuffer(bytearray(depth_log.tobytes()), dtype=torch.float32)
    input_tensor = input_tensor.reshape(INPUT_H, INPUT_W).unsqueeze(0).unsqueeze(0).repeat(1, 3, 1, 1)
    if tuple(input_tensor.shape) != (1, 3, INPUT_H, INPUT_W):
        raise RuntimeError("internal model input shape mismatch: {}".format(tuple(input_tensor.shape)))

    inference_started_ns = time.monotonic_ns()
    with torch.inference_mode():
        output = model(input_tensor)
    if not isinstance(output, torch.Tensor) or tuple(output.shape) != (1, RAY_COUNT):
        raise RuntimeError("model output must be (1, 11), got {}".format(
            tuple(output.shape) if isinstance(output, torch.Tensor) else type(output)))
    rays_log2 = np.asarray(output[0].detach().cpu().tolist(), dtype=np.float32)
    if not np.isfinite(rays_log2).all():
        raise RuntimeError("model returned non-finite ray values")
    rays_log2 = np.clip(rays_log2, math.log2(RAY_MIN_M), math.log2(RAY_MAX_M)).astype(np.float32)
    rays_m = np.exp2(rays_log2).astype(np.float32)
    inference_complete_ns = time.monotonic_ns()

    # The frame is a file-only serialization. It is never mapped to /dev/shm.
    # A saved input has no live source timestamp, so this local completion time
    # does not establish sensor freshness or controller suitability.
    sequence = 2  # stable/even single-frame sequence in the v2 seqlock contract
    frame_bytes = RAY_HEADER.pack(RAY_MAGIC, RAY_VERSION, sequence, inference_complete_ns)
    frame_bytes += RAY_PAYLOAD.pack(*[float(x) for x in rays_log2])
    with frame_path.open("xb") as stream:
        stream.write(frame_bytes)

    report["status"] = "prediction_written_offline"
    report["timestamps"]["inference_started_monotonic_ns"] = inference_started_ns
    report["timestamps"]["inference_complete_monotonic_ns"] = inference_complete_ns
    report["inference_ms"] = (inference_complete_ns - inference_started_ns) / 1e6
    report["protocol"]["sequence"] = sequence
    report["protocol"]["frame_file"] = str(frame_path.resolve())
    report["prediction_valid"] = True
    report["rays_log2_m"] = [float(x) for x in rays_log2]
    report["rays_m"] = [float(x) for x in rays_m]
    report["model_input"] = {
        "shape_nchw": [1, 3, INPUT_H, INPUT_W],
        "depth_clip_m": [RAY_MIN_M, RAY_MAX_M],
        "transform": "log2(m), one channel repeated three times",
        "tensor_min": float(depth_log.min()),
        "tensor_max": float(depth_log.max()),
    }
    json_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print("[RayOffline] wrote {} and {} (offline only; no shm/device access)".format(frame_path, json_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
