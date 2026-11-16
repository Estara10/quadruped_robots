#!/usr/bin/env python3
"""Offline-only depth-hole input sensitivity analyses.

The default input compares nine saved frames with a source-equivalent 2.58.4
mode-1 implementation. ``--native-fill-capture`` instead consumes saved raw
and actual SDK-filled pairs without running a filter. No mode accesses hardware,
ROS/DDS, a controller, or shared memory.
"""

from __future__ import annotations

import csv
import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from realsense_depth_to_abs_rays import downsample_depth_centers

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/ray_pred_mujoco_finetuned_best.pt"
DEFAULT_OUT = ROOT / "logs/real_go2/20261112_d435i_hole_candidate_ab"
INPUTS = {
    "cabinet_replacement": ROOT / "logs/real_go2/20261112_d435i_swap_compare/orin_capture/go2_d435i_swap_capture_depth_only",
    "position1": ROOT / "logs/real_go2/20261112_d435i_two_position_scene_compare/position1/go2_d435i_position1_capture_data",
    "position2": ROOT / "logs/real_go2/20261112_d435i_two_position_scene_compare/position2/go2_d435i_position2_capture_data",
}
H, W = 270, 480
MODEL_H, MODEL_W = 90, 160
RAY_MIN_M, RAY_MAX_M = 0.1, 6.0
RAY_ANGLES_DEG = [-45.0 + 9.0 * i for i in range(11)]


def infer_11(model, depth_m: np.ndarray) -> np.ndarray:
    import torch

    clipped = np.clip(depth_m, RAY_MIN_M, RAY_MAX_M)
    log_depth = np.log2(clipped).astype(np.float32)
    # Avoid the host's NumPy 2 / Torch 2.0 NumPy bridge warning.
    tensor = torch.frombuffer(bytearray(log_depth.tobytes()), dtype=torch.float32)
    tensor = tensor.reshape(1, 1, MODEL_H, MODEL_W).repeat(1, 3, 1, 1)
    with torch.inference_mode():
        out = model(tensor)
    if not isinstance(out, torch.Tensor) or tuple(out.shape) != (1, 11):
        raise RuntimeError(f"expected model output (1, 11), got {getattr(out, 'shape', type(out))}")
    rays = np.asarray(out[0].tolist(), dtype=np.float32)
    if not np.isfinite(rays).all():
        raise RuntimeError("model returned nonfinite rays")
    return np.exp2(np.clip(rays, math.log2(RAY_MIN_M), math.log2(RAY_MAX_M))).astype(np.float32)


def librealsense_2584_mode1_source(depth_u16: np.ndarray) -> np.ndarray:
    """Translate v2.58.4 holes_fill_farest(uint16_t*) loop over a copied image.

    The upstream function scans top-to-bottom and left-to-right, fills only
    zero-valued interior pixels, and takes the maximum of its five indexed
    neighbors (above, upper-left, left, lower-left, below). Border pixels remain
    unchanged. This preserves the implementation's in-place scan behavior.
    """
    h, w = depth_u16.shape
    out = depth_u16.copy().reshape(-1)
    for y in range(1, h - 1):
        p = y * w + 1
        for _x in range(1, w):
            if out[p] == 0:
                value = int(out[p - w])
                for q in (p - w - 1, p - 1, p + w - 1, p + w):
                    value = max(value, int(out[q]))
                out[p] = value
            p += 1
    return out.reshape(h, w)


def analyze_saved_native_fill(model, capture_dir: Path, output_dir: Path, model_path: Path) -> int:
    """Compare saved Orin SDK raw/filled pairs; does not run any filter."""
    metadata = {}
    for line in (capture_dir / "metadata.txt").read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            metadata[key.strip()] = value.strip()
    width, height = int(metadata["width"]), int(metadata["height"])
    scale = float(metadata["scale"])
    if width != W or height != H:
        raise ValueError(f"saved capture is {width}x{height}; expected {W}x{H}")
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError(f"invalid depth scale in metadata: {scale}")

    with (capture_dir / "frames.csv").open(newline="", encoding="utf-8") as f:
        saved_rows = [row for row in csv.DictReader(f) if row["saved"] == "1"]
    if len(saved_rows) != 3:
        raise ValueError(f"expected exactly 3 saved frame pairs, got {len(saved_rows)}")

    per_frame = []
    ray_rows = []
    for row in saved_rows:
        index = int(row["i"])
        raw_path = capture_dir / f"{index}_depth.u16"
        filled_path = capture_dir / f"{index}_filled.u16"
        raw = np.fromfile(raw_path, dtype="<u2")
        filled = np.fromfile(filled_path, dtype="<u2")
        if raw.size != width * height or filled.size != width * height:
            raise ValueError(f"unexpected saved frame size at sample {index}")
        raw, filled = raw.reshape(height, width), filled.reshape(height, width)
        raw_valid = raw > 0
        filled_valid = filled > 0
        added_mask = (~raw_valid) & filled_valid
        remaining_mask = ~filled_valid
        changed_measured = int(np.count_nonzero(filled[raw_valid] != raw[raw_valid]))

        raw_m = raw.astype(np.float32) * np.float32(scale)
        filled_m = filled.astype(np.float32) * np.float32(scale)
        a_m = raw_m.copy()
        a_m[~raw_valid] = np.float32(6.0)
        b_m = filled_m.copy()
        b_m[remaining_mask] = np.float32(6.0)
        a_input, a_footprint_valid = downsample_depth_centers(a_m, np.ones_like(raw_valid))
        b_input, b_footprint_valid = downsample_depth_centers(b_m, np.ones_like(raw_valid))
        _, strict_a = downsample_depth_centers(raw_m, raw_valid)
        _, strict_b = downsample_depth_centers(filled_m, filled_valid)
        if not a_footprint_valid.all() or not b_footprint_valid.all():
            raise RuntimeError("diagnostic replacement unexpectedly left invalid model cells")

        sample_y = ((2 * np.arange(MODEL_H) + 1) * height) // (2 * MODEL_H)
        sample_x = ((2 * np.arange(MODEL_W) + 1) * width) // (2 * MODEL_W)
        raw_sample_valid = raw_valid[np.ix_(sample_y, sample_x)]
        added_sample = added_mask[np.ix_(sample_y, sample_x)]
        remaining_sample = remaining_mask[np.ix_(sample_y, sample_x)]
        center = np.s_[height // 2 - 20:height // 2 + 20,
                       width // 2 - 20:width // 2 + 20]

        start = time.perf_counter()
        a_rays = infer_11(model, a_input)
        a_infer_ms = (time.perf_counter() - start) * 1000.0
        start = time.perf_counter()
        b_rays = infer_11(model, b_input)
        b_infer_ms = (time.perf_counter() - start) * 1000.0
        delta = b_rays - a_rays
        record = {
            "sample_index": index,
            "frame_number": int(row["frame_number"]),
            "device_timestamp_ms": float(row["device_ms"]),
            "raw_file": raw_path.name,
            "sdk_filled_file": filled_path.name,
            "raw_zero_fraction_full": float((~raw_valid).mean()),
            "sdk_filled_fraction_full": float(added_mask.mean()),
            "remaining_zero_fraction_full": float(remaining_mask.mean()),
            "raw_zero_fraction_center40": float((~raw_valid[center]).mean()),
            "sdk_filled_fraction_center40": float(added_mask[center].mean()),
            "remaining_zero_fraction_center40": float(remaining_mask[center].mean()),
            "model_cells": MODEL_H * MODEL_W,
            "model_a_measured_cells": int(raw_sample_valid.sum()),
            "model_a_6m_placeholder_cells": int((~raw_sample_valid).sum()),
            "model_a_measured_fraction": float(raw_sample_valid.mean()),
            "model_a_6m_placeholder_fraction": float((~raw_sample_valid).mean()),
            "model_measured_cells": int(raw_sample_valid.sum()),
            "model_sdk_filled_cells": int(added_sample.sum()),
            "model_6m_placeholder_cells": int(remaining_sample.sum()),
            "model_measured_fraction": float(raw_sample_valid.mean()),
            "model_sdk_filled_fraction": float(added_sample.mean()),
            "model_6m_placeholder_fraction": float(remaining_sample.mean()),
            "model_unknown_before_fill_fraction": float((~raw_sample_valid).mean()),
            "strict_adapter_rejects_raw": bool(not strict_a.all()),
            "strict_adapter_rejects_sdk_filled": bool(not strict_b.all()),
            "strict_invalid_model_cells_raw": int((~strict_a).sum()),
            "strict_invalid_model_cells_sdk_filled": int((~strict_b).sum()),
            "original_valid_pixels_changed_by_sdk": changed_measured,
            "inference_a_ms": a_infer_ms,
            "inference_b_ms": b_infer_ms,
            "rays_a_m": [float(v) for v in a_rays],
            "rays_b_m": [float(v) for v in b_rays],
            "delta_b_minus_a_m": [float(v) for v in delta],
        }
        per_frame.append(record)
        for ray_index, angle in enumerate(RAY_ANGLES_DEG):
            ray_rows.append({
                "sample_index": index,
                "frame_number": int(row["frame_number"]),
                "ray_index": ray_index,
                "label_angle_deg": angle,
                "a_raw_zero_to_6m_distance_m": float(a_rays[ray_index]),
                "b_sdk_fill_then_remaining_zero_to_6m_distance_m": float(b_rays[ray_index]),
                "b_minus_a_m": float(delta[ray_index]),
                "diagnostic_only_not_for_control": True,
            })

    delta = np.asarray([r["delta_b_minus_a_m"] for r in per_frame], dtype=np.float64)
    a_rays = np.asarray([r["rays_a_m"] for r in per_frame], dtype=np.float64)
    b_rays = np.asarray([r["rays_b_m"] for r in per_frame], dtype=np.float64)
    per_ray = []
    for i, angle in enumerate(RAY_ANGLES_DEG):
        d = delta[:, i]
        per_ray.append({
            "ray_index": i,
            "label_angle_deg": angle,
            "mean_a_m": float(a_rays[:, i].mean()),
            "mean_b_m": float(b_rays[:, i].mean()),
            "mean_delta_b_minus_a_m": float(d.mean()),
            "mean_absolute_delta_m": float(np.abs(d).mean()),
            "max_absolute_delta_m": float(np.abs(d).max()),
            "frames_b_farther": int((d > 0).sum()),
            "frames_b_nearer": int((d < 0).sum()),
            "frames_equal": int((d == 0).sum()),
            "a_frame_range_m": [float(a_rays[:, i].min()), float(a_rays[:, i].max())],
            "b_frame_range_m": [float(b_rays[:, i].min()), float(b_rays[:, i].max())],
        })

    with (output_dir / "ab_per_ray.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(ray_rows[0]))
        writer.writeheader()
        writer.writerows(ray_rows)
    with (output_dir / "per_frame_masks_and_rays.csv").open("w", newline="", encoding="utf-8") as f:
        fields = [k for k, v in per_frame[0].items() if not isinstance(v, list)]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows([{k: v for k, v in r.items() if not isinstance(v, list)} for r in per_frame])

    summary = {
        "purpose": "Three saved real SDK pairs; input sensitivity only, not range accuracy or control validation.",
        "source": {
            "capture_dir": str(capture_dir.resolve()),
            "camera_serial": metadata.get("serial"),
            "width": width,
            "height": height,
            "stored_format": "little-endian uint16 Z16 copy",
            "depth_scale_m_per_unit": scale,
            "pair_count": len(per_frame),
            "pairing": "frames.csv rows marked saved=1; saved SDK output is consumed directly; no filter is rerun",
        },
        "model": {
            "path": str(model_path.resolve()),
            "input": "160x90 center sample; clip 0.1-6m; log2(m); repeat into 3 channels",
            "ray_labels_deg": RAY_ANGLES_DEG,
            "view_contract": "training virtual camera 102 deg HFOV; D435i approximately 88.9 deg; output labels remain -45 to +45 deg; no inference remapping",
        },
        "candidates": {
            "A": "raw zeros replaced with 6m, then existing preprocessing and model",
            "B": "saved actual SDK mode-1 output; remaining zeros replaced with 6m, then identical preprocessing and model",
            "masks": "original raw valid pixels, SDK-added pixels (raw zero and filled nonzero), remaining unknown (filled zero) kept distinct; model receives no mask channel",
        },
        "aggregate": {
            "mean_absolute_ray_change_over_33_frame_ray_pairs_m": float(np.abs(delta).mean()),
            "maximum_absolute_ray_change_m": float(np.abs(delta).max()),
            "mean_signed_change_b_minus_a_m": float(delta.mean()),
            "b_farther_frame_ray_pairs": int((delta > 0).sum()),
            "b_nearer_frame_ray_pairs": int((delta < 0).sum()),
            "equal_frame_ray_pairs": int((delta == 0).sum()),
            "mean_raw_unknown_full_image_fraction": float(np.mean([r["raw_zero_fraction_full"] for r in per_frame])),
            "mean_sdk_filled_fraction_full_image": float(np.mean([r["sdk_filled_fraction_full"] for r in per_frame])),
            "mean_remaining_unknown_full_image_fraction": float(np.mean([r["remaining_zero_fraction_full"] for r in per_frame])),
            "mean_model_cell_fractions": {
                "candidate_A_raw_plus_6m": {
                    "measured": float(np.mean([r["model_a_measured_fraction"] for r in per_frame])),
                    "6m_placeholder": float(np.mean([r["model_a_6m_placeholder_fraction"] for r in per_frame])),
                },
                "candidate_B_sdk_fill_then_6m": {
                    "measured": float(np.mean([r["model_measured_fraction"] for r in per_frame])),
                    "sdk_filled": float(np.mean([r["model_sdk_filled_fraction"] for r in per_frame])),
                    "6m_placeholder": float(np.mean([r["model_6m_placeholder_fraction"] for r in per_frame])),
                },
            },
            "original_valid_pixels_changed_by_sdk_total": int(sum(r["original_valid_pixels_changed_by_sdk"] for r in per_frame)),
            "strict_raw_rejected_frames": int(sum(r["strict_adapter_rejects_raw"] for r in per_frame)),
            "strict_sdk_filled_rejected_frames": int(sum(r["strict_adapter_rejects_sdk_filled"] for r in per_frame)),
        },
        "per_ray_summary": per_ray,
        "per_frame": per_frame,
        "limitations": [
            "Three frames are a short sample from one static observation, not long-term stability evidence.",
            "No independent obstacle/range labels are available; prediction changes cannot be ranked as better or safer.",
            "The model has no validity-mask input; all diagnostic predictions use explicit 6m placeholders for residual zeros.",
            "The model training virtual-camera HFOV is 102 deg while the D435i is approximately 88.9 deg; ray labels remain fixed at -45 to +45 deg.",
        ],
        "diagnostic_only_not_for_control": True,
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")

    # Compact chart: each group of three bars is one ray's three saved frames.
    from PIL import Image, ImageDraw
    chart = Image.new("RGB", (1320, 520), "white")
    draw = ImageDraw.Draw(chart)
    draw.text((45, 16), "Native SDK hole-fill effect on predicted distance (B - A, m); diagnostic only", fill="black")
    left, right, top, bottom = 70, 1290, 70, 440
    max_abs = max(0.02, float(np.abs(delta).max()))
    y0 = (top + bottom) // 2
    draw.line((left, y0, right, y0), fill=(25, 25, 25), width=2)
    for i, angle in enumerate(RAY_ANGLES_DEG):
        center_x = left + int((i + 0.5) * (right - left) / len(RAY_ANGLES_DEG))
        for frame_idx in range(3):
            value = float(delta[frame_idx, i])
            x = center_x + (frame_idx - 1) * 12
            y = y0 - int(value / max_abs * (bottom - top) * 0.47)
            color = (190, 65, 65) if value > 0 else (65, 100, 175)
            draw.rectangle((x - 4, min(y, y0), x + 4, max(y, y0)), fill=color)
        draw.text((center_x - 14, bottom + 12), str(angle), fill="black")
    draw.text((left, bottom + 38), "ray label angle (deg); three bars per ray = saved frames 179 / 299 / 419", fill="black")
    chart.save(output_dir / "ray_delta_by_frame.png")
    print(json.dumps({"output": str(output_dir), "aggregate": summary["aggregate"]}, indent=2))
    return 0


def main() -> int:
    import torch

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT,
                        help="new output directory; existing directories are not overwritten")
    parser.add_argument("--model", type=Path, default=MODEL,
                        help="explicit TorchScript model path; defaults to the current soft-safety weight")
    parser.add_argument("--native-fill-capture", type=Path,
                        help="analyze saved raw/SDK-filled frame pairs directly; no filtering is run")
    args = parser.parse_args()
    model_path = args.model if args.model.is_absolute() else ROOT / args.model
    if not model_path.is_file():
        raise FileNotFoundError(model_path)
    torch.set_num_threads(1)
    model = torch.jit.load(str(model_path), map_location="cpu")
    model.eval()
    output_dir = args.output_dir if args.output_dir.is_absolute() else ROOT / args.output_dir
    output_dir.mkdir(parents=True, exist_ok=False)
    if args.native_fill_capture is not None:
        capture_dir = args.native_fill_capture
        if not capture_dir.is_absolute():
            capture_dir = ROOT / capture_dir
        return analyze_saved_native_fill(model, capture_dir, output_dir, model_path)
    rows = []
    summaries = {}
    all_frames = []
    figure_examples = []

    for group, directory in INPUTS.items():
        with (directory / "frames.csv").open(newline="", encoding="utf-8") as f:
            frame_meta = list(csv.DictReader(f))
        group_rows = []
        for meta in frame_meta:
            sample = int(meta["sample"])
            scale = float(meta["scale_m"])
            raw_path = directory / f"{sample}_depth.u16"
            raw = np.fromfile(raw_path, dtype="<u2")
            if raw.size != W * H:
                raise RuntimeError(f"unexpected byte count/sample shape: {raw_path} {raw.size}")
            raw = raw.reshape(H, W)
            valid = raw > 0
            depth_m = raw.astype(np.float32) * np.float32(scale)
            sampled, strict_valid = downsample_depth_centers(depth_m, valid)

            direct6_m = depth_m.copy()
            direct6_m[~valid] = np.float32(6.0)
            direct6_sampled, direct6_valid = downsample_depth_centers(
                direct6_m, np.ones_like(valid, dtype=bool))
            if not direct6_valid.all():
                raise RuntimeError("6m replacement unexpectedly left invalid model cells")
            rays_m = infer_11(model, direct6_sampled)
            filtered = librealsense_2584_mode1_source(raw)
            filtered_valid = filtered > 0
            mode1_filled = (~valid) & filtered_valid
            remaining_unknown = ~filtered_valid
            changed_valid = int(np.count_nonzero(filtered[valid] != raw[valid]))
            mode1_depth_before_6m = filtered.astype(np.float32) * np.float32(scale)
            _, mode1_strict_valid = downsample_depth_centers(mode1_depth_before_6m, filtered_valid)
            mode1_depth_m = filtered.astype(np.float32) * np.float32(scale)
            mode1_depth_m[remaining_unknown] = np.float32(6.0)
            mode1_sampled, mode1_valid_after_fill6 = downsample_depth_centers(
                mode1_depth_m, np.ones_like(valid, dtype=bool))
            if not mode1_valid_after_fill6.all():
                raise RuntimeError("mode-1 candidate retained an invalid model input")
            rays_mode1 = infer_11(model, mode1_sampled)

            center_y0, center_x0 = H // 2 - 20, W // 2 - 20
            center_y1, center_x1 = center_y0 + 40, center_x0 + 40
            sample_y = ((2 * np.arange(MODEL_H) + 1) * H) // (2 * MODEL_H)
            sample_x = ((2 * np.arange(MODEL_W) + 1) * W) // (2 * MODEL_W)
            raw_sample_unknown = ~valid[np.ix_(sample_y, sample_x)]
            fill_sample = mode1_filled[np.ix_(sample_y, sample_x)]
            remaining_sample = remaining_unknown[np.ix_(sample_y, sample_x)]
            # Each output cell corresponds to an exact 3x3 source footprint.
            raw_footprint_unknown = np.zeros((MODEL_H, MODEL_W), dtype=bool)
            fill_footprint = np.zeros_like(raw_footprint_unknown)
            remaining_footprint = np.zeros_like(raw_footprint_unknown)
            for oy in range(MODEL_H):
                for ox in range(MODEL_W):
                    ys = slice(oy * 3, oy * 3 + 3)
                    xs = slice(ox * 3, ox * 3 + 3)
                    raw_footprint_unknown[oy, ox] = bool((~valid[ys, xs]).any())
                    fill_footprint[oy, ox] = bool(mode1_filled[ys, xs].any())
                    remaining_footprint[oy, ox] = bool(remaining_unknown[ys, xs].any())
            center_masks = {
                "full_image": {
                    "raw_unknown": ~valid,
                    "filled": mode1_filled,
                    "remaining": remaining_unknown,
                },
                "center_40x40": {
                    "raw_unknown": (~valid)[center_y0:center_y1, center_x0:center_x1],
                    "filled": mode1_filled[center_y0:center_y1, center_x0:center_x1],
                    "remaining": remaining_unknown[center_y0:center_y1, center_x0:center_x1],
                },
            }

            def subset_stats(raw_mask: np.ndarray, filled_mask: np.ndarray, remain_mask: np.ndarray) -> dict:
                n = int(raw_mask.size)
                raw_n = int(raw_mask.sum())
                fill_n = int(filled_mask.sum())
                rem_n = int(remain_mask.sum())
                return {
                    "cell_count": n,
                    "raw_unknown_cells": raw_n,
                    "raw_unknown_fraction": raw_n / n,
                    "mode1_filled_cells": fill_n,
                    "mode1_filled_fraction_of_raw_unknown": fill_n / raw_n if raw_n else 0.0,
                    "remaining_unknown_cells_before_6m": rem_n,
                    "remaining_unknown_fraction_of_raw_unknown": rem_n / raw_n if raw_n else 0.0,
                }

            source_region_stats = {}
            for region, masks in center_masks.items():
                # Regions are described as masks over source pixels.
                n = int(masks["raw_unknown"].size)
                raw_n = int(masks["raw_unknown"].sum())
                fill_n = int(masks["filled"].sum())
                rem_n = int(masks["remaining"].sum())
                source_region_stats[region] = {
                    "pixel_count": n,
                    "raw_unknown_pixels": raw_n,
                    "raw_unknown_fraction": raw_n / n,
                    "mode1_filled_pixels": fill_n,
                    "mode1_filled_fraction_of_region": fill_n / n,
                    "mode1_filled_fraction_of_raw_unknown": fill_n / raw_n if raw_n else 0.0,
                    "remaining_unknown_pixels_before_6m": rem_n,
                    "remaining_unknown_fraction_of_region": rem_n / n,
                    "remaining_unknown_fraction_of_raw_unknown": rem_n / raw_n if raw_n else 0.0,
                }
            source_region_stats["model_sample_centers"] = subset_stats(
                raw_sample_unknown, fill_sample, remaining_sample)
            source_region_stats["model_3x3_footprints_any_unknown"] = subset_stats(
                raw_footprint_unknown, fill_footprint, remaining_footprint)

            record = {
                "group": group,
                "sample": sample,
                "source_file": str(raw_path.relative_to(ROOT)),
                "frame_number": int(meta.get("frame", meta.get("depth_frame", sample))),
                "timestamp_ms": float(meta["timestamp_ms"]),
                "resolution_wh": [W, H],
                "format": "Z16 uint16 little-endian, stored raw frame",
                "depth_scale_m_per_unit": scale,
                "raw_valid_pixels": int(valid.sum()),
                "raw_zero_pixels": int((~valid).sum()),
                "raw_zero_fraction": float((~valid).mean()),
                "strict_model_invalid_pixels": int((~strict_valid).sum()),
                "strict_model_invalid_fraction": float((~strict_valid).mean()),
                "strict_adapter_rejects_frame": bool(not strict_valid.all()),
                "direct6_replaced_source_zero_pixels": int((~valid).sum()),
                "direct6_replaced_source_zero_fraction": float((~valid).mean()),
                "mode1_sdk_version_candidate": "2.58.4 source-equivalent; SDK runtime software-device execution unavailable (udev monitor init error)",
                "mode1_filled_pixels": int(mode1_filled.sum()),
                "mode1_filled_fraction_full_image": float(mode1_filled.mean()),
                "mode1_filled_fraction_of_raw_unknown": float(mode1_filled.sum() / max(1, (~valid).sum())),
                "mode1_remaining_zero_pixels_before_6m": int(remaining_unknown.sum()),
                "mode1_remaining_zero_fraction_full_image": float(remaining_unknown.mean()),
                "mode1_remaining_zero_fraction_of_raw_unknown": float(remaining_unknown.sum() / max(1, (~valid).sum())),
                "mode1_original_nonzero_pixels_changed": changed_valid,
                "mode1_strict_model_invalid_pixels": int((~mode1_strict_valid).sum()),
                "mode1_strict_model_invalid_fraction": float((~mode1_strict_valid).mean()),
                "mode1_strict_adapter_rejects_frame": bool(not mode1_strict_valid.all()),
                "mask_statistics": source_region_stats,
                "diagnostic_only_not_for_control": True,
                "rays_m_direct6": [float(v) for v in rays_m],
                "rays_m_mode1_then_6m": [float(v) for v in rays_mode1],
                "rays_m_delta_mode1_minus_direct6": [float(v) for v in (rays_mode1 - rays_m)],
            }
            group_rows.append(record)
            all_frames.append(record)
            if sample == int(frame_meta[0]["sample"]):
                figure_examples.append({
                    "group": group,
                    "sample": sample,
                    "raw_m": depth_m,
                    "unknown": ~valid,
                    "filled": mode1_filled,
                    "remaining": remaining_unknown,
                })
            for ray_idx, angle in enumerate(RAY_ANGLES_DEG):
                rows.append({
                    "group": group,
                    "sample": sample,
                    "ray_index": ray_idx,
                    "angle_deg": angle,
                    "direct6_diagnostic_distance_m": float(rays_m[ray_idx]),
                    "mode1_2584_source_then_6m_diagnostic_distance_m": float(rays_mode1[ray_idx]),
                    "delta_mode1_minus_direct6_m": float(rays_mode1[ray_idx] - rays_m[ray_idx]),
                    "diagnostic_only_not_for_control": True,
                })

        summaries[group] = {
            "frame_count": len(group_rows),
            "mean_raw_zero_fraction": float(np.mean([r["raw_zero_fraction"] for r in group_rows])),
            "raw_zero_fraction_range": [
                float(min(r["raw_zero_fraction"] for r in group_rows)),
                float(max(r["raw_zero_fraction"] for r in group_rows)),
            ],
            "mean_strict_model_invalid_fraction": float(np.mean([r["strict_model_invalid_fraction"] for r in group_rows])),
            "strict_rejected_frames": int(sum(r["strict_adapter_rejects_frame"] for r in group_rows)),
            "mode1_strict_rejected_frames": int(sum(r["mode1_strict_adapter_rejects_frame"] for r in group_rows)),
            "mean_mode1_filled_fraction_full_image": float(np.mean([r["mode1_filled_fraction_full_image"] for r in group_rows])),
            "mean_mode1_filled_fraction_of_raw_unknown": float(np.mean([r["mode1_filled_fraction_of_raw_unknown"] for r in group_rows])),
            "mean_mode1_remaining_zero_fraction_full_image": float(np.mean([r["mode1_remaining_zero_fraction_full_image"] for r in group_rows])),
            "mean_mode1_remaining_zero_fraction_of_raw_unknown": float(np.mean([r["mode1_remaining_zero_fraction_of_raw_unknown"] for r in group_rows])),
            "original_nonzero_pixels_changed_total": int(sum(r["mode1_original_nonzero_pixels_changed"] for r in group_rows)),
            "direct6_rays_m_mean_by_index": np.mean(
                np.asarray([r["rays_m_direct6"] for r in group_rows], dtype=np.float64), axis=0
            ).tolist(),
            "mode1_then_6m_rays_m_mean_by_index": np.mean(
                np.asarray([r["rays_m_mode1_then_6m"] for r in group_rows], dtype=np.float64), axis=0
            ).tolist(),
            "mean_delta_mode1_minus_direct6_by_ray": np.mean(
                np.asarray([r["rays_m_delta_mode1_minus_direct6"] for r in group_rows], dtype=np.float64), axis=0
            ).tolist(),
        }

    with (output_dir / "ab_per_ray.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    per_frame_fields = [
        "group", "sample", "raw_zero_pixels", "raw_zero_fraction",
        "mode1_filled_pixels", "mode1_filled_fraction_full_image",
        "mode1_filled_fraction_of_raw_unknown", "mode1_remaining_zero_pixels_before_6m",
        "mode1_remaining_zero_fraction_full_image", "mode1_remaining_zero_fraction_of_raw_unknown",
        "mode1_original_nonzero_pixels_changed",
        "mode1_strict_invalid_model_cells", "mode1_strict_adapter_rejects_frame",
        "full_raw_unknown_fraction", "full_mode1_filled_pixels", "full_remaining_unknown_fraction",
        "center40_raw_unknown_fraction", "center40_mode1_filled_pixels", "center40_remaining_unknown_fraction",
        "model_sample_raw_unknown_fraction", "model_sample_filled_cells", "model_sample_remaining_unknown_fraction",
        "model_3x3_footprint_raw_unknown_fraction", "model_3x3_footprint_filled_cells",
        "model_3x3_footprint_remaining_unknown_fraction",
    ]
    with (output_dir / "per_frame_masks.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=per_frame_fields)
        writer.writeheader()
        for rec in all_frames:
            ms = rec["mask_statistics"]
            writer.writerow({
                "group": rec["group"], "sample": rec["sample"],
                "raw_zero_pixels": rec["raw_zero_pixels"], "raw_zero_fraction": rec["raw_zero_fraction"],
                "mode1_filled_pixels": rec["mode1_filled_pixels"],
                "mode1_filled_fraction_full_image": rec["mode1_filled_fraction_full_image"],
                "mode1_filled_fraction_of_raw_unknown": rec["mode1_filled_fraction_of_raw_unknown"],
                "mode1_remaining_zero_pixels_before_6m": rec["mode1_remaining_zero_pixels_before_6m"],
                "mode1_remaining_zero_fraction_full_image": rec["mode1_remaining_zero_fraction_full_image"],
                "mode1_remaining_zero_fraction_of_raw_unknown": rec["mode1_remaining_zero_fraction_of_raw_unknown"],
                "mode1_original_nonzero_pixels_changed": rec["mode1_original_nonzero_pixels_changed"],
                "mode1_strict_invalid_model_cells": rec["mode1_strict_model_invalid_pixels"],
                "mode1_strict_adapter_rejects_frame": rec["mode1_strict_adapter_rejects_frame"],
                "full_raw_unknown_fraction": ms["full_image"]["raw_unknown_fraction"],
                "full_mode1_filled_pixels": ms["full_image"]["mode1_filled_pixels"],
                "full_remaining_unknown_fraction": ms["full_image"]["remaining_unknown_fraction_of_region"],
                "center40_raw_unknown_fraction": ms["center_40x40"]["raw_unknown_fraction"],
                "center40_mode1_filled_pixels": ms["center_40x40"]["mode1_filled_pixels"],
                "center40_remaining_unknown_fraction": ms["center_40x40"]["remaining_unknown_fraction_of_region"],
                "model_sample_raw_unknown_fraction": ms["model_sample_centers"]["raw_unknown_fraction"],
                "model_sample_filled_cells": ms["model_sample_centers"]["mode1_filled_cells"],
                "model_sample_remaining_unknown_fraction": ms["model_sample_centers"]["remaining_unknown_fraction_of_raw_unknown"],
                "model_3x3_footprint_raw_unknown_fraction": ms["model_3x3_footprints_any_unknown"]["raw_unknown_fraction"],
                "model_3x3_footprint_filled_cells": ms["model_3x3_footprints_any_unknown"]["mode1_filled_cells"],
                "model_3x3_footprint_remaining_unknown_fraction": ms["model_3x3_footprints_any_unknown"]["remaining_unknown_fraction_of_raw_unknown"],
            })
    summary = {
        "purpose": "Offline diagnostic input sensitivity only; not accuracy, safety, or control validation.",
        "python": sys.version,
        "torch": torch.__version__,
        "device": "cpu",
        "model": str(model_path.resolve()),
        "model_input": {
            "shape_nchw": [1, 3, MODEL_H, MODEL_W],
            "source_to_model": "center sample from 480x270, clip 0.1-6m, log2(m), repeat to 3 channels",
            "unknown_A": "raw Z16 zero replaced with 6m before current preprocessing",
            "unknown_B": "mode-1 filled raw Z16 zeros retained as filled measurements only in candidate data; remaining zeros replaced with 6m before same current preprocessing; original raw validity mask retained separately",
            "virtual_training_hfov_deg": 102.0,
            "d435i_hfov_deg_approx": 88.9,
            "inference_fov_remapping": "none; adapter only center-samples, so physical view is not stretched or expanded",
            "ray_output_labels_deg": RAY_ANGLES_DEG,
        },
        "filter_comparison": {
            "requested": "librealsense hole_filling_filter mode=1; host candidate version 2.58.4",
            "installed_package_version": "ros-humble-librealsense2 2.58.4-1jammy.20260830.174155",
            "status": "source_equivalent_2.58.4_algorithm_applied; SDK runtime filter not invoked",
            "reason": "compiled SDK software-device test terminated when librealsense attempted to initialize a udev monitor; no hardware was opened. Analysis uses the version-pinned v2.58.4 holes_fill_farest implementation translated literally for uint16 Z16.",
            "mode_semantics": "farest_from_around; documentation describes choosing a neighboring depth furthest from camera; pinned implementation fills zero interior cells in place using max of five indexed samples (above, upper-left, left, lower-left, below); top/bottom/left border unchanged",
            "2_51_1_comparison": "official 2.51.1 release/API notes do not expose the internal implementation; the long-standing API description defines mode 1 as farthest among up/down/left/right neighbors. Exact source-level equivalence between 2.51.1 and 2.58.4 is not verified here.",
            "source_urls": [
                "https://github.com/realsenseai/librealsense/blob/v2.58.4/src/proc/hole-filling-filter.h",
                "https://github.com/realsenseai/librealsense/blob/v2.58.4/src/proc/hole-filling-filter.cpp",
                "https://github.com/realsenseai/librealsense/wiki/API-Changes",
            ],
        },
        "groups": summaries,
        "per_frame": all_frames,
        "per_ray_csv": str((output_dir / "ab_per_ray.csv").resolve()),
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    from PIL import Image, ImageDraw

    panel_w, panel_h, header_h = W, H, 28
    titles = ["Raw depth (gray; black=zero)", "Original unknown (white=unknown)",
              "Mode 1 filled pixels", "Still unknown before 6m"]
    canvas = Image.new("RGB", (panel_w * 4, (panel_h + header_h) * len(figure_examples)), "white")
    draw = ImageDraw.Draw(canvas)
    for row_idx, ex in enumerate(figure_examples):
        y0 = row_idx * (panel_h + header_h)
        raw = ex["raw_m"]
        shade = np.clip((raw - RAY_MIN_M) / (RAY_MAX_M - RAY_MIN_M), 0.0, 1.0)
        gray = np.where(ex["unknown"], 0, np.asarray(shade * 255, dtype=np.uint8))
        raw_img = Image.fromarray(gray, mode="L").convert("RGB")
        mask_img = Image.fromarray((ex["unknown"].astype(np.uint8) * 255), mode="L").convert("RGB")
        filled_rgb = np.zeros((H, W, 3), dtype=np.uint8)
        filled_rgb[ex["filled"]] = (40, 190, 80)
        remain_rgb = np.zeros((H, W, 3), dtype=np.uint8)
        remain_rgb[ex["remaining"]] = (220, 45, 45)
        panels = [raw_img, mask_img, Image.fromarray(filled_rgb), Image.fromarray(remain_rgb)]
        for col, panel in enumerate(panels):
            x0 = col * panel_w
            canvas.paste(panel, (x0, y0 + header_h))
            draw.text((x0 + 4, y0 + 7), f"{ex['group']} - {titles[col]}", fill="black")
    canvas.save(output_dir / "depth_hole_masks.png")

    mean_delta = np.mean(np.asarray([r["rays_m_delta_mode1_minus_direct6"] for r in all_frames]), axis=0)
    plot_w, plot_h = 1100, 480
    chart = Image.new("RGB", (plot_w, plot_h), "white")
    d = ImageDraw.Draw(chart)
    left, right, top, bottom = 70, plot_w - 30, 55, plot_h - 65
    d.text((left, 15), "Mean B - A diagnostic distance, all 9 frames (positive = farther)", fill="black")
    max_abs = max(0.05, float(np.max(np.abs(mean_delta))))
    y_zero = int(top + (bottom - top) * 0.5)
    d.line((left, y_zero, right, y_zero), fill=(20, 20, 20), width=2)
    for idx, value in enumerate(mean_delta):
        x = left + int((idx + 0.5) * (right - left) / len(RAY_ANGLES_DEG))
        y = y_zero - int((value / max_abs) * (bottom - top) * 0.45)
        color = (195, 70, 70) if value > 0 else (70, 110, 180)
        d.rectangle((x - 18, min(y, y_zero), x + 18, max(y, y_zero)), fill=color)
        d.text((x - 14, bottom + 8), str(RAY_ANGLES_DEG[idx]), fill="black")
        d.text((x - 22, top - 20 if value >= 0 else bottom + 25), f"{value:+.2f}", fill=color)
    d.text((left, plot_h - 22), "ray label angle (deg; fixed -45 to +45)", fill="black")
    chart.save(output_dir / "mean_ray_delta.png")
    print(json.dumps({"output": str(output_dir), "groups": summaries, "filter_status": summary["filter_comparison"]["status"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
