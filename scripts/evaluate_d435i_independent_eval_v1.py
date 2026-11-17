#!/usr/bin/env python3
"""One-shot evaluation of four fixed ray checkpoints on the independent eval layouts.

Data: logs/real_go2/20261117_d435i_independent_eval_v1 (new layouts, generated after all
training and checkpoint selection; never used for training or selection).

Counting conventions are reported separately and not mixed:
  * record level: one row per input frame x ray (360 frames x 11 = 3960)
  * pair level:   one sample per unique pose-ray, clean/masked averaged (1980)
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from finetune_mujoco_ray_pred import RayDataset, RAY_MAX, RAY_MIN  # noqa: E402
from analyze_d435i_near_coverage_validation import (  # noqa: E402
    ANGLES, GROUPS, read_visibility, group_mask, metric_block,
)

EVAL_ROOT = ROOT / "logs/real_go2/20261117_d435i_independent_eval_v1"
MODELS = {
    "medium": ROOT / "archive/ray_pred_finetune/mujoco_finetune_medium_20260611/ray_pred_mujoco_finetuned_best.pt",
    "epoch13": ROOT / "logs/real_go2/20261117_d435i_fullnet_preparation/training_runs/d435i_fullnet_300epoch_20261117/ray_pred_mujoco_finetuned_best.pt",
    "near_coverage_epoch3": ROOT / "logs/real_go2/20261117_d435i_near_coverage_training/medium_fullnet_near_coverage_60ep/ray_pred_mujoco_finetuned_best.pt",
    "far_danger_0p5_epoch3": ROOT / "logs/real_go2/20261117_d435i_far_underestimate_training/medium_fullnet_far_danger_0p5_60ep/ray_pred_mujoco_finetuned_best.pt",
}
PAIR_GROUPS = ("all", "visible_target", "in_frame", "out_of_frame_surface_visible",
               "fully_invisible", "no_hit_within_range")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=EVAL_ROOT / "evaluation" / "four_model_comparison.json")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--batch-size", type=int, default=32)
    args = parser.parse_args()
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError(f"CUDA requested but unavailable: {args.device}")
    for path in MODELS.values():
        if not path.is_file():
            raise FileNotFoundError(path)

    data_dir = EVAL_ROOT / "data" / "validation"
    dataset = RayDataset([data_dir])
    visibility = read_visibility([EVAL_ROOT / "ray_projection_visibility.csv"])

    metadata, target_rows = [], []
    for image_path, label in dataset.items:
        stem = image_path.stem
        sample, flavor = stem.rsplit("_", 1)
        if flavor not in ("clean", "masked") or sample not in visibility:
            raise RuntimeError(f"invalid eval sample: {image_path}")
        for ray_index in range(11):
            vis = visibility[sample][ray_index]
            target_rows.append(float(np.clip(label[ray_index], RAY_MIN, RAY_MAX)))
            metadata.append({"sample": sample, "flavor": flavor, "ray_index": ray_index,
                             "angle_deg": ANGLES[ray_index],
                             "detailed_group": vis.get("detailed_group", vis["group"]),
                             "target_m": target_rows[-1]})
    targets_flat = np.asarray(target_rows, dtype=np.float64)
    groups_flat = np.asarray([m["detailed_group"] for m in metadata], dtype=object)

    preds = {}
    for name, model_path in MODELS.items():
        model = torch.jit.load(str(model_path), map_location=device).eval()
        matrix = np.empty((len(dataset), 11), dtype=np.float32)
        with torch.no_grad():
            for start in range(0, len(dataset), args.batch_size):
                stop = min(start + args.batch_size, len(dataset))
                depth = torch.stack([dataset[i][0] for i in range(start, stop)]).to(device)
                pred = torch.exp2(model(depth.unsqueeze(1).repeat(1, 3, 1, 1))).clamp(RAY_MIN, RAY_MAX)
                matrix[start:stop] = pred.cpu().numpy()
        preds[name] = matrix.reshape(-1).astype(np.float64)

    results = {
        "eval_set": str(EVAL_ROOT),
        "eval_set_summary": str(EVAL_ROOT / "summary.json"),
        "used_for_training_or_selection": False,
        "final_test_claim": False,
        "pose_ray_pairs": len(targets_flat) // 2,
        "input_frames": len(dataset),
        "record_level_ray_values": int(targets_flat.size),
        "models": {name: {"path": str(path), "bytes": path.stat().st_size, "sha256": sha256(path)}
                   for name, path in MODELS.items()},
        "record_level": {},
        "pair_level": {},
        "per_ray_visible_near": {},
    }

    for name, pred_flat in preds.items():
        record = {"all": metric_block(targets_flat, pred_flat, groups_flat)}
        for group in GROUPS:
            record[group] = metric_block(targets_flat, pred_flat, groups_flat, group_mask(groups_flat, group))
        for flavor in ("clean", "masked"):
            flavor_mask = np.asarray([m["flavor"] == flavor for m in metadata])
            record[f"input_{flavor}"] = metric_block(targets_flat, pred_flat, groups_flat, flavor_mask)
        results["record_level"][name] = record

        per_ray = {}
        for ray_index, angle in enumerate(ANGLES):
            ray_mask = np.asarray([m["ray_index"] == ray_index for m in metadata])
            per_ray[f"{ray_index}:{angle:+.0f}deg"] = {
                group: metric_block(targets_flat, pred_flat, groups_flat,
                                    ray_mask & group_mask(groups_flat, group))
                for group in ("visible_target", "in_frame", "out_of_frame_surface_visible", "fully_invisible")
            }
        results["per_ray_visible_near"][name] = per_ray

    pair_index: dict[tuple[str, int], dict[str, int]] = {}
    for i, meta in enumerate(metadata):
        pair_index.setdefault((meta["sample"], meta["ray_index"]), {})[meta["flavor"]] = i
    pair_keys = list(pair_index)
    pair_target = np.asarray([metadata[pair_index[k][next(iter(pair_index[k]))]]["target_m"]
                              for k in pair_keys], dtype=np.float64)
    pair_group = np.asarray([metadata[pair_index[k][next(iter(pair_index[k]))]]["detailed_group"]
                             for k in pair_keys], dtype=object)
    pair_visible = np.isin(pair_group, ["in_frame", "out_of_frame_surface_visible"])

    for name, pred_flat in preds.items():
        pair_pred = np.asarray([float(np.mean([pred_flat[i] for i in pair_index[k].values()]))
                                for k in pair_keys], dtype=np.float64)
        block = {}
        for group in PAIR_GROUPS:
            mask = (np.ones(len(pair_keys), dtype=bool) if group == "all" else
                    pair_visible if group == "visible_target" else pair_group == group)
            t, p = pair_target[mask], pair_pred[mask]
            if t.size == 0:
                block[group] = {"unique_pose_ray_pairs": 0}
                continue
            err = p - t
            near_pos = np.maximum(err[t <= 1.5], 0.0)
            block[group] = {
                "unique_pose_ray_pairs": int(t.size),
                "mae_m": float(np.mean(np.abs(err))),
                "bias_m": float(np.mean(err)),
                "near_target_le_1p5_count": int(np.count_nonzero(t <= 1.5)),
                "near_false_safe": int(np.count_nonzero((t <= 1.5) & (err >= 0.5))),
                "near_overestimate_p95_m": float(np.quantile(near_pos, 0.95)) if near_pos.size else None,
                "far_target_ge_3_count": int(np.count_nonzero(t >= 3.0)),
                "far_false_danger": int(np.count_nonzero((t >= 3.0) & (err <= -0.5))),
            }
        results["pair_level"][name] = block

    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.write_text(json.dumps(results, indent=2))

    header = (f"{'model':<24}{'MAE':>8}{'visNear_fs':>16}{'P95':>8}"
              f"{'far_under':>16}{'invisible_fs':>15}")
    print(header)
    for name in MODELS:
        r = results["record_level"][name]
        p = results["pair_level"][name]
        print(f"{name:<24}{r['all']['mae_m']:>8.3f}"
              f"{r['visible_target']['false_safe']:>7}/{r['visible_target']['near_target_le_1p5_count']:<7}"
              f"{(r['visible_target']['near_overestimate_p95_m'] or 0):>8.3f}"
              f"{r['all']['false_danger']:>7}/{r['all']['far_target_ge_3_count']:<7}"
              f"{r['fully_invisible']['false_safe']:>6}/{r['fully_invisible']['near_target_le_1p5_count']:<6}")
    print(f"\npair-level (n={len(pair_keys)}):")
    for name in MODELS:
        p = results["pair_level"][name]
        v, i = p["visible_target"], p["fully_invisible"]
        print(f"  {name:<24} mae={p['all']['mae_m']:.3f} "
              f"vis_near_fs={v['near_false_safe']}/{v['near_target_le_1p5_count']} "
              f"far_under={p['all']['far_false_danger']}/{p['all']['far_target_ge_3_count']} "
              f"invisible_near_fs={i.get('near_false_safe')}/{i.get('near_target_le_1p5_count')}")
    print(f"\nwritten: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
