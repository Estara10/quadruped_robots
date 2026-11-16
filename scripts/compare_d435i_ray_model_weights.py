#!/usr/bin/env python3
"""Offline comparison of three explicitly supplied ABS ray-model weights."""

from __future__ import annotations

import argparse
import csv
import json
import math
import pickle
import sys
from pathlib import Path

import numpy as np

from analyze_d435i_new_camera_hole_candidates import (
    H, W, MODEL_H, MODEL_W, RAY_ANGLES_DEG, RAY_MIN_M, RAY_MAX_M,
    downsample_depth_centers,
)

ROOT = Path(__file__).resolve().parents[1]
CAPTURE = ROOT / "logs/real_go2"
PAIRED_DIRS = {
    "cabinet_190cm_native_fill": CAPTURE / "20261112_d435i_cabinet_190cm_native_fill/capture",
    "native_fill_field": CAPTURE / "20261112_d435i_native_fill_field/capture",
}
RAW_DIRS = {
    "cabinet_replacement": CAPTURE / "20261112_d435i_swap_compare/orin_capture/go2_d435i_swap_capture_depth_only",
    "position1": CAPTURE / "20261112_d435i_two_position_scene_compare/position1/go2_d435i_position1_capture_data",
    "position2": CAPTURE / "20261112_d435i_two_position_scene_compare/position2/go2_d435i_position2_capture_data",
}
SIM_DIR = ROOT / "ABS/training/legged_gym/legged_gym/depth_data/mujoco_ray_pred/mujoco_ray_pred_medium_20260611"
MODEL_HFOV_DEG = 102.0
FALSE_SAFE_TARGET_M = 1.5
FALSE_SAFE_MARGIN_M = 0.5
FALSE_DANGER_TARGET_M = 3.0
FALSE_DANGER_MARGIN_M = 0.5


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", action="append", required=True, metavar="NAME=PATH",
                   help="TorchScript model; pass exactly base=..., medium=..., soft_safety=...")
    p.add_argument("--output-dir", type=Path, required=True,
                   help="new result directory; existing directories are rejected")
    return p.parse_args()


def parse_models(items: list[str]) -> dict[str, Path]:
    models: dict[str, Path] = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"model must be NAME=PATH: {item}")
        name, value = item.split("=", 1)
        if name in models or name not in ("base", "medium", "soft_safety"):
            raise ValueError(f"unexpected or duplicate model name: {name}")
        path = Path(value)
        models[name] = path if path.is_absolute() else ROOT / path
    if set(models) != {"base", "medium", "soft_safety"}:
        raise ValueError("provide exactly base, medium, and soft_safety models")
    for name, path in models.items():
        if not path.is_file():
            raise FileNotFoundError(f"{name} model does not exist: {path}")
    return models


def load_model(path: Path):
    import torch

    model = torch.jit.load(str(path), map_location="cpu")
    model.eval()
    if model.training:
        raise RuntimeError(f"model did not enter eval mode: {path}")
    return model


def infer_batch(model, depths_m: np.ndarray) -> np.ndarray:
    """Existing contract: clip metres, log2, repeat channel, output exp2 metres."""
    import torch

    clipped = np.clip(depths_m, RAY_MIN_M, RAY_MAX_M).astype(np.float32, copy=False)
    log_depth = np.log2(clipped).astype(np.float32, copy=False)
    tensor = torch.frombuffer(bytearray(np.ascontiguousarray(log_depth).tobytes()), dtype=torch.float32)
    tensor = tensor.reshape(-1, 1, MODEL_H, MODEL_W).repeat(1, 3, 1, 1)
    with torch.inference_mode():
        outputs = model(tensor)
    if not isinstance(outputs, torch.Tensor) or tuple(outputs.shape) != (len(depths_m), 11):
        raise RuntimeError(f"expected output (N,11), got {getattr(outputs, 'shape', type(outputs))}")
    logits = np.asarray(outputs.tolist(), dtype=np.float32)
    if not np.isfinite(logits).all():
        raise RuntimeError("model returned nonfinite log2 distances")
    meters = np.exp2(np.clip(logits, math.log2(RAY_MIN_M), math.log2(RAY_MAX_M)))
    return meters.astype(np.float32)


def read_kv(path: Path) -> dict[str, str]:
    values = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            k, v = line.split("=", 1)
            values[k.strip()] = v.strip()
    return values


def load_real_inputs() -> tuple[list[dict], list[dict]]:
    """Return per-frame real inputs and source metadata, preserving A/B provenance."""
    frames: list[dict] = []
    sources: list[dict] = []

    def add_raw(group: str, directory: Path, meta: dict, paired: bool) -> None:
        with (directory / "frames.csv").open(newline="", encoding="utf-8") as f:
            rows = list(csv.DictReader(f))
        if "saved" in rows[0]:
            rows = [r for r in rows if r["saved"] == "1"]
        for row in rows:
            sample = int(row.get("i", row.get("sample")))
            frame_number = int(row.get("frame_number", row.get("frame", row.get("depth_frame", sample))))
            scale = float(meta.get("scale", row.get("scale_m", "nan")))
            if not math.isfinite(scale) or scale <= 0:
                raise ValueError(f"missing/invalid scale for {group} sample {sample}")
            raw_path = directory / f"{sample}_depth.u16"
            raw = np.fromfile(raw_path, dtype="<u2")
            if raw.size != W * H:
                raise ValueError(f"unexpected image size: {raw_path} has {raw.size} pixels")
            raw = raw.reshape(H, W)
            raw_valid = raw > 0
            raw_m = raw.astype(np.float32) * np.float32(scale)
            direct = raw_m.copy()
            direct[~raw_valid] = np.float32(6.0)
            direct_sample, _ = downsample_depth_centers(direct, np.ones_like(raw_valid))
            _, strict_raw = downsample_depth_centers(raw_m, raw_valid)
            center = np.s_[H // 2 - 20:H // 2 + 20, W // 2 - 20:W // 2 + 20]
            candidates = [{"candidate": "A_raw_zero_to_6m", "model_input_m": direct_sample,
                           "measured_sample_fraction": float(raw_valid[
                               ((2*np.arange(MODEL_H)+1)*H)//(2*MODEL_H), :][:,
                               ((2*np.arange(MODEL_W)+1)*W)//(2*MODEL_W)].mean()),
                           "sdk_filled_sample_fraction": 0.0,
                           "six_m_placeholder_sample_fraction": float((~raw_valid[
                               ((2*np.arange(MODEL_H)+1)*H)//(2*MODEL_H), :][:,
                               ((2*np.arange(MODEL_W)+1)*W)//(2*MODEL_W)]).mean()),
                           "strict_invalid_cells": int((~strict_raw).sum()),
                           "strict_rejected": bool(not strict_raw.all())}]
            filled_path = directory / f"{sample}_filled.u16"
            if paired and filled_path.is_file():
                filled = np.fromfile(filled_path, dtype="<u2")
                if filled.size != W * H:
                    raise ValueError(f"unexpected image size: {filled_path}")
                filled = filled.reshape(H, W)
                if np.count_nonzero(filled[raw_valid] != raw[raw_valid]):
                    raise RuntimeError(f"SDK modified measured raw pixels: {group} sample {sample}")
                filled_valid = filled > 0
                added = (~raw_valid) & filled_valid
                remaining = ~filled_valid
                candidate_b_m = filled.astype(np.float32) * np.float32(scale)
                candidate_b_m[remaining] = np.float32(6.0)
                candidate_b, _ = downsample_depth_centers(candidate_b_m, np.ones_like(raw_valid))
                _, strict_b = downsample_depth_centers(filled.astype(np.float32) * np.float32(scale), filled_valid)
                sy = ((2 * np.arange(MODEL_H) + 1) * H) // (2 * MODEL_H)
                sx = ((2 * np.arange(MODEL_W) + 1) * W) // (2 * MODEL_W)
                sampled_raw_valid = raw_valid[np.ix_(sy, sx)]
                sampled_added = added[np.ix_(sy, sx)]
                sampled_remaining = remaining[np.ix_(sy, sx)]
                candidates.append({
                    "candidate": "B_sdk_native_fill_then_residual_zero_to_6m",
                    "model_input_m": candidate_b,
                    "measured_sample_fraction": float(sampled_raw_valid.mean()),
                    "sdk_filled_sample_fraction": float(sampled_added.mean()),
                    "six_m_placeholder_sample_fraction": float(sampled_remaining.mean()),
                    "strict_invalid_cells": int((~strict_b).sum()),
                    "strict_rejected": bool(not strict_b.all()),
                })
                filled_fraction = float(added.mean())
                remaining_fraction = float(remaining.mean())
            else:
                filled_fraction = None
                remaining_fraction = None
            frames.append({
                "group": group, "sample": sample, "frame_number": frame_number,
                "raw_path": str(raw_path.relative_to(ROOT)),
                "filled_path": str(filled_path.relative_to(ROOT)) if paired and filled_path.is_file() else None,
                "scale_m_per_unit": scale,
                "raw_unknown_fraction": float((~raw_valid).mean()),
                "raw_center40_unknown_fraction": float((~raw_valid[center]).mean()),
                "sdk_filled_full_fraction": filled_fraction,
                "remaining_unknown_full_fraction": remaining_fraction,
                "original_nonzero_changed": 0,
                "candidates": candidates,
            })
        sources.append({"group": group, "directory": str(directory.relative_to(ROOT)),
                        "metadata": meta, "saved_frame_count": len(rows),
                        "paired_sdk_fill": bool(paired and any((directory / f"{r.get('i',r.get('sample'))}_filled.u16").exists() for r in rows))})

    for group, directory in PAIRED_DIRS.items():
        if not directory.is_dir():
            raise FileNotFoundError(directory)
        meta = read_kv(directory / "metadata.txt")
        if int(meta["width"]) != W or int(meta["height"]) != H:
            raise ValueError(f"unexpected capture dimensions in {directory}")
        add_raw(group, directory, meta, paired=True)
    for group, directory in RAW_DIRS.items():
        if not directory.is_dir():
            raise FileNotFoundError(directory)
        add_raw(group, directory, {}, paired=False)
    return frames, sources


def run_real(models: dict[str, object], frames: list[dict], out_dir: Path) -> tuple[list[dict], dict]:
    inputs = [(frame, candidate) for frame in frames for candidate in frame["candidates"]]
    depths = np.stack([candidate["model_input_m"] for _, candidate in inputs])
    predictions: dict[str, np.ndarray] = {}
    for name, model in models.items():
        predictions[name] = infer_batch(model, depths)

    out_csv = out_dir / "real_per_frame_per_ray.csv"
    fields = ["group", "sample", "frame_number", "candidate", "model", "ray_index", "label_angle_deg",
              "predicted_distance_m", "raw_unknown_fraction", "sdk_filled_full_fraction",
              "remaining_unknown_full_fraction", "model_measured_fraction", "model_sdk_filled_fraction",
              "model_6m_placeholder_fraction", "strict_invalid_cells", "strict_rejected",
              "diagnostic_only_not_accuracy"]
    with out_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for item_index, (frame, candidate) in enumerate(inputs):
            frame["predictions"] = frame.get("predictions", {})
            key = candidate["candidate"]
            frame["predictions"].setdefault(key, {})
            for name, pred in predictions.items():
                rays = pred[item_index]
                frame["predictions"][key][name] = [float(v) for v in rays]
                for i, angle in enumerate(RAY_ANGLES_DEG):
                    writer.writerow({
                        "group": frame["group"], "sample": frame["sample"], "frame_number": frame["frame_number"],
                        "candidate": key, "model": name, "ray_index": i, "label_angle_deg": angle,
                        "predicted_distance_m": float(rays[i]),
                        "raw_unknown_fraction": frame["raw_unknown_fraction"],
                        "sdk_filled_full_fraction": frame["sdk_filled_full_fraction"],
                        "remaining_unknown_full_fraction": frame["remaining_unknown_full_fraction"],
                        "model_measured_fraction": candidate["measured_sample_fraction"],
                        "model_sdk_filled_fraction": candidate["sdk_filled_sample_fraction"],
                        "model_6m_placeholder_fraction": candidate["six_m_placeholder_sample_fraction"],
                        "strict_invalid_cells": candidate["strict_invalid_cells"],
                        "strict_rejected": candidate["strict_rejected"],
                        "diagnostic_only_not_accuracy": True,
                    })

    grouped = {}
    for frame in frames:
        for candidate in frame["candidates"]:
            c = candidate["candidate"]
            for model_name, rays in frame["predictions"][c].items():
                grouped.setdefault((frame["group"], c, model_name), []).append(rays)
    summary = []
    for (group, candidate, model_name), ray_sets in sorted(grouped.items()):
        arr = np.asarray(ray_sets, dtype=np.float64)
        summary.append({
            "group": group, "candidate": candidate, "model": model_name, "frame_count": len(arr),
            "mean_by_ray_m": arr.mean(axis=0).tolist(),
            "range_by_ray_m": [[float(arr[:, i].min()), float(arr[:, i].max())] for i in range(11)],
            "mean_distance_all_rays_m": float(arr.mean()),
            "mean_model_measured_fraction": float(np.mean([c["measured_sample_fraction"] for fr in frames if fr["group"] == group for c in fr["candidates"] if c["candidate"] == candidate])),
            "mean_model_sdk_filled_fraction": float(np.mean([c["sdk_filled_sample_fraction"] for fr in frames if fr["group"] == group for c in fr["candidates"] if c["candidate"] == candidate])),
            "mean_model_6m_placeholder_fraction": float(np.mean([c["six_m_placeholder_sample_fraction"] for fr in frames if fr["group"] == group for c in fr["candidates"] if c["candidate"] == candidate])),
        })
    return frames, {"csv": str(out_csv.resolve()), "groups": summary}


def run_labeled_sim(models: dict[str, object], out_dir: Path) -> dict:
    """Compare all three models on the fixed, labeled medium fine-tune source set."""
    with (SIM_DIR / "label.pkl").open("rb") as f:
        labels = pickle.load(f)
    image_paths = sorted(SIM_DIR.glob("*.npy"))
    if len(image_paths) != len(labels):
        raise RuntimeError(f"dataset image/label count mismatch: {len(image_paths)} images, {len(labels)} labels")
    batch_size = 32
    csv_path = out_dir / "medium_dataset_labeled_predictions.csv"
    fields = ["sample_id", "model", "ray_index", "label_angle_deg", "target_m", "predicted_m",
              "signed_error_pred_minus_target_m", "absolute_error_m", "source_dataset_used_for_finetune"]
    aggregate = {}
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for model_name, model in models.items():
            sum_abs = np.zeros(11, dtype=np.float64)
            sum_sq = np.zeros(11, dtype=np.float64)
            sum_err = np.zeros(11, dtype=np.float64)
            false_safe = np.zeros(11, dtype=np.int64)
            false_danger = np.zeros(11, dtype=np.int64)
            total = 0
            for offset in range(0, len(image_paths), batch_size):
                paths = image_paths[offset:offset + batch_size]
                image_list = []
                target_list = []
                for path in paths:
                    if path.stem not in labels:
                        raise KeyError(f"missing geometry label for {path.stem}")
                    img = np.load(path, allow_pickle=False).astype(np.float32)
                    target = np.asarray(labels[path.stem], dtype=np.float32)
                    if img.shape != (MODEL_H, MODEL_W) or target.shape != (11,):
                        raise ValueError(f"unexpected labeled sample shapes at {path.name}: {img.shape}, {target.shape}")
                    if not np.isfinite(img).all() or (img <= 0).any() or not np.isfinite(target).all():
                        raise ValueError(f"nonfinite/invalid labeled sample at {path.name}")
                    image_list.append(img)
                    target_list.append(target)
                images = np.stack(image_list)
                targets = np.stack(target_list)
                pred = infer_batch(model, images).astype(np.float64)
                err = pred - targets
                sum_abs += np.abs(err).sum(axis=0)
                sum_sq += np.square(err).sum(axis=0)
                sum_err += err.sum(axis=0)
                false_safe += ((targets <= FALSE_SAFE_TARGET_M) & (pred >= targets + FALSE_SAFE_MARGIN_M)).sum(axis=0)
                false_danger += ((targets >= FALSE_DANGER_TARGET_M) & (pred <= targets - FALSE_DANGER_MARGIN_M)).sum(axis=0)
                total += len(paths)
                for j, path in enumerate(paths):
                    for ray_i, angle in enumerate(RAY_ANGLES_DEG):
                        writer.writerow({
                            "sample_id": path.stem, "model": model_name, "ray_index": ray_i,
                            "label_angle_deg": angle, "target_m": float(targets[j, ray_i]),
                            "predicted_m": float(pred[j, ray_i]),
                            "signed_error_pred_minus_target_m": float(err[j, ray_i]),
                            "absolute_error_m": float(abs(err[j, ray_i])),
                            "source_dataset_used_for_finetune": True,
                        })
            aggregate[model_name] = {
                "sample_count": total,
                "mae_m": float(sum_abs.sum() / (total * 11)),
                "rmse_m": float(math.sqrt(sum_sq.sum() / (total * 11))),
                "bias_pred_minus_target_m": float(sum_err.sum() / (total * 11)),
                "per_ray_mae_m": (sum_abs / total).tolist(),
                "per_ray_bias_m": (sum_err / total).tolist(),
                "false_safe_by_ray_count": false_safe.tolist(),
                "false_danger_by_ray_count": false_danger.tolist(),
                "false_safe_total": int(false_safe.sum()),
                "false_danger_total": int(false_danger.sum()),
                "not_independent_validation": True,
            }
    return {"source_dataset": str(SIM_DIR.relative_to(ROOT)), "manifest": json.loads((SIM_DIR / "manifest.json").read_text()),
            "labels": "label.pkl, metric distances; same dataset appears in medium and soft_safety fine-tune manifests",
            "not_independent_validation": True, "csv": str(csv_path.resolve()), "models": aggregate}


def write_plot(real_summary: dict, output: Path) -> None:
    from PIL import Image, ImageDraw

    rows = [r for r in real_summary["groups"] if r["group"] == "cabinet_190cm_native_fill"]
    if not rows:
        return
    colors = {"base": (55, 110, 190), "medium": (45, 150, 85), "soft_safety": (195, 65, 65)}
    chart = Image.new("RGB", (1260, 650), "white")
    d = ImageDraw.Draw(chart)
    d.text((35, 15), "Cabinet capture: mean 11-ray predictions by model and input treatment (diagnostic only)", fill="black")
    left, right, top, bottom = 75, 1220, 70, 565
    ymin, ymax = 0.0, 3.0
    for tick in np.arange(0, 3.01, 0.5):
        y = bottom - int((tick - ymin) / (ymax - ymin) * (bottom - top))
        d.line((left, y, right, y), fill=(220, 220, 220), width=1)
        d.text((35, y - 7), f"{tick:.1f}m", fill=(50, 50, 50))
    for idx, angle in enumerate(RAY_ANGLES_DEG):
        x = left + int(idx * (right - left) / 10)
        d.text((x - 12, bottom + 8), str(int(angle)), fill="black")
    for model_name, color in colors.items():
        for candidate, style in (("A_raw_zero_to_6m", "A"), ("B_sdk_native_fill_then_residual_zero_to_6m", "B")):
            item = next((r for r in rows if r["model"] == model_name and r["candidate"] == candidate), None)
            if item is None:
                continue
            points = []
            for idx, value in enumerate(item["mean_by_ray_m"]):
                x = left + int(idx * (right - left) / 10)
                y = bottom - int((value - ymin) / (ymax - ymin) * (bottom - top))
                points.append((x, y))
            if len(points) > 1:
                d.line(points, fill=color, width=3 if style == "A" else 1)
            for x, y in points:
                d.ellipse((x - 4, y - 4, x + 4, y + 4), fill=color)
        d.text((left + (list(colors).index(model_name) * 180), 610), f"{model_name}: A solid / B thin", fill=color)
    d.text((left, 585), "Ray label angle (deg)", fill="black")
    chart.save(output)


def main() -> int:
    args = parse_args()
    model_paths = parse_models(args.model)
    out_dir = args.output_dir if args.output_dir.is_absolute() else ROOT / args.output_dir
    out_dir.mkdir(parents=True, exist_ok=False)
    import torch
    torch.set_num_threads(1)
    models = {name: load_model(path) for name, path in model_paths.items()}

    probe = np.full((1, MODEL_H, MODEL_W), 1.0, dtype=np.float32)
    for name, model in models.items():
        output = infer_batch(model, probe)
        if output.shape != (1, 11) or not np.isfinite(output).all():
            raise RuntimeError(f"inference contract failed for {name}")

    frames, sources = load_real_inputs()
    frames, real_summary = run_real(models, frames, out_dir)
    sim_summary = run_labeled_sim(models, out_dir)
    write_plot(real_summary, out_dir / "cabinet_11ray_model_comparison.png")

    medium_manifest_path = ROOT / "archive/ray_pred_finetune/mujoco_finetune_medium_20260611/manifest.json"
    soft_manifest_path = ROOT / "archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/manifest.json"
    summaries = {
        "purpose": "Compare three existing weights on identical saved real inputs and one existing labeled MuJoCo dataset; not a real-world accuracy certification.",
        "models": {name: {"path": str(path.resolve()), "torchscript": True, "eval_mode": not model.training,
                          "output_contract": "11 log2(m) values from model, exp2/clipped to metric distances for reporting"}
                   for (name, path), model in zip(model_paths.items(), models.values())},
        "training_manifests": {
            "medium": json.loads(medium_manifest_path.read_text()),
            "soft_safety": json.loads(soft_manifest_path.read_text()),
            "base_manifest": None,
            "base_note": "Base checkpoint is the scripted artifact saved by train_depth_resnet.py; no matching run manifest was found in the requested checkpoint directory.",
        },
        "preprocessing": {
            "real_depth": "uint16 Z16 scaled to metres from per-capture metadata/CSV; raw zeros are replaced with 6m for A; B consumes only saved SDK-filled pairs and replaces remaining zero with 6m.",
            "both_candidates": "same 160x90 center sampler; 0.1-6m clip; log2; repeat 3 channels; exp2 and metric clipping for reported rays.",
            "sdk_fill_mask": "original measured values, SDK-added values, and residual unknown are kept distinct in summaries; no mask is sent to the model.",
            "strict_adapter": "strict 3x3-footprint rejection count is recorded, but the diagnostic inference intentionally evaluates the requested A/B candidates.",
            "fov": {"training_virtual_camera_hfov_deg": MODEL_HFOV_DEG, "d435i_hfov_deg_approx": 88.9,
                    "ray_label_angles_deg": RAY_ANGLES_DEG, "inference_remapping": "none"},
        },
        "real_sources": sources,
        "real_summary": real_summary,
        "labeled_simulation_comparison": sim_summary,
        "limits": [
            "Cabinet distance is approximate and does not provide independent labels for all body-horizontal rays; camera optical center and ray origin differ.",
            "The labeled medium set is training data for the medium and soft_safety models; it is descriptive/in-sample and not independent validation.",
            "No saved split membership was found to isolate a common held-out subset across all three weights.",
            "No real D435i-style holes occurred in the synthetic training images; random erasure values are augmentation, not a measured camera-hole model.",
            "The approximately 89deg D435i field of view differs from the 102deg training render; the current predictor does not remap it.",
            "The A/B zero replacement is diagnostic only and cannot establish unknown-space safety.",
        ],
        "diagnostic_only_not_for_control": True,
    }
    (out_dir / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"output_dir": str(out_dir), "real_groups": len(real_summary["groups"]),
                      "sim_metrics": sim_summary["models"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
