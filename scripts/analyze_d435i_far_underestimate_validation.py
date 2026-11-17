#!/usr/bin/env python3
"""Single-factor comparison: far-underestimate loss weight 0 -> 0.5.

Reuses the fixed 20261117 combined validation split and the metric definitions of
analyze_d435i_near_coverage_validation.py. Reports both counting conventions:
  * record level: one row per input frame x ray (540 x 11 = 5940)
  * pair level:   one sample per unique pose-ray, clean/masked predictions averaged (2970)
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from finetune_mujoco_ray_pred import RayDataset, RAY_MAX, RAY_MIN  # noqa: E402
from analyze_d435i_near_coverage_validation import (  # noqa: E402
    BASE, SUPPLEMENT, MEDIUM, EPOCH13, NEW_CANDIDATE, ANGLES, GROUPS,
    read_visibility, group_mask, metric_block,
)

FAR_WEIGHT_CANDIDATE = (
    ROOT / "logs/real_go2/20261117_d435i_far_underestimate_training"
    / "medium_fullnet_far_danger_0p5_60ep/ray_pred_mujoco_finetuned_best.pt"
)
MODELS = {
    "medium": MEDIUM,
    "epoch13": EPOCH13,
    "near_coverage_epoch3": NEW_CANDIDATE,
    "far_danger_0p5_epoch3": FAR_WEIGHT_CANDIDATE,
}
BINS = ("near_le_1p5m", "mid_1p5_to_3m", "far_ge_3m")
GROUP_SET = ("all", *GROUPS)


def dist_bin(target: float) -> str:
    if target <= 1.5:
        return "near_le_1p5m"
    if target < 3.0:
        return "mid_1p5_to_3m"
    return "far_ge_3m"


def mean(values: list[float]) -> float | None:
    return float(np.mean(values)) if values else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--batch-size", type=int, default=32)
    args = parser.parse_args()
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError(f"CUDA requested but unavailable: {args.device}")
    for path in MODELS.values():
        if not path.is_file():
            raise FileNotFoundError(path)

    dataset = RayDataset([BASE / "data/validation", SUPPLEMENT / "data/validation"])
    visibility = read_visibility([BASE / "ray_projection_visibility.csv",
                                  SUPPLEMENT / "ray_projection_visibility.csv"])

    metadata, target_rows = [], []
    for image_path, label in dataset.items:
        stem = image_path.stem
        sample, flavor = stem.rsplit("_", 1)
        if flavor not in ("clean", "masked") or sample not in visibility:
            raise RuntimeError(f"invalid validation sample: {image_path}")
        source = "supplement_validation" if SUPPLEMENT.name in image_path.parts else "v2_validation"
        for ray_index in range(11):
            vis = visibility[sample][ray_index]
            detailed = vis.get("detailed_group", vis["group"])
            target_rows.append(float(np.clip(label[ray_index], RAY_MIN, RAY_MAX)))
            metadata.append({"sample": sample, "flavor": flavor, "ray_index": ray_index,
                             "angle_deg": ANGLES[ray_index], "detailed_group": detailed,
                             "source": source, "target_m": target_rows[-1]})
    targets_flat = np.asarray(target_rows, dtype=np.float32)
    groups_flat = np.asarray([m["detailed_group"] for m in metadata], dtype=object)

    preds = {}
    for name, model_path in MODELS.items():
        model = torch.jit.load(str(model_path), map_location=device).eval()
        pred_matrix = np.empty((len(dataset), 11), dtype=np.float32)
        with torch.no_grad():
            for start in range(0, len(dataset), args.batch_size):
                stop = min(start + args.batch_size, len(dataset))
                depth = torch.stack([dataset[i][0] for i in range(start, stop)]).to(device)
                pred = torch.exp2(model(depth.unsqueeze(1).repeat(1, 3, 1, 1))).clamp(RAY_MIN, RAY_MAX)
                pred_matrix[start:stop] = pred.cpu().numpy()
        preds[name] = pred_matrix.reshape(-1).astype(np.float64)

    summary = {
        "validation_only": True,
        "final_holdout_read_or_evaluated": False,
        "change": "false_danger_weight 0.0 -> 0.5; additive on base weight 1.0, so a far-target "
                  "underestimate (target>=3m and pred<target-0.5m) gets weight 1.5 (+50%)",
        "dataset_dirs": [str(BASE / "data/validation"), str(SUPPLEMENT / "data/validation")],
        "validation_poses": len(dataset) // 2,
        "input_frames": len(dataset),
        "record_level": {
            "observation_unit": "one input frame x ray (clean and masked counted separately)",
            "ray_values_per_model": int(targets_flat.size),
            "metric_contract": "near: target<=1.5m; near false-safe: pred-target>=0.5m; "
                               "near P95 over max(error,0); far underestimate: target>=3.0m and "
                               "target-pred>=0.5m",
            "models": {},
        },
        "pair_level": {
            "observation_unit": "unique pose-ray; clean/masked predictions averaged within the pair",
            "unique_pose_ray_pairs": int(targets_flat.size // 2),
            "models": {},
            "epoch3_to_far_danger_0p5": {},
        },
    }

    for name, pred_flat in preds.items():
        result = {"all": metric_block(targets_flat, pred_flat, groups_flat)}
        for group in GROUPS:
            result[group] = metric_block(targets_flat, pred_flat, groups_flat, group_mask(groups_flat, group))
        for source in ("v2_validation", "supplement_validation"):
            source_mask = np.asarray([m["source"] == source for m in metadata])
            result[source] = metric_block(targets_flat, pred_flat, groups_flat, source_mask)
        summary["record_level"]["models"][name] = result

        per_ray = {}
        for ray_index, angle in enumerate(ANGLES):
            ray_mask = np.asarray([m["ray_index"] == ray_index for m in metadata])
            per_ray[f"{ray_index}:{angle:+.0f}deg"] = {
                group: metric_block(targets_flat, pred_flat, groups_flat,
                                    ray_mask & group_mask(groups_flat, group))
                for group in ("visible_target", "in_frame", "out_of_frame_surface_visible", "fully_invisible")
            }
        summary["record_level"][f"per_ray_{name}"] = per_ray
        for flavor in ("clean", "masked"):
            flavor_mask = np.asarray([m["flavor"] == flavor for m in metadata])
            result[f"input_{flavor}"] = metric_block(targets_flat, pred_flat, groups_flat, flavor_mask)

    # Pair level: average the two input flavors, then count one unit per pose-ray.
    pair_index: dict[tuple[str, int], dict[str, int]] = {}
    for i, meta in enumerate(metadata):
        pair_index.setdefault((meta["sample"], meta["ray_index"]), {})[meta["flavor"]] = i
    pair_keys = list(pair_index)
    pair_target = np.asarray([metadata[pair_index[k][next(iter(pair_index[k]))]]["target_m"]
                              for k in pair_keys], dtype=np.float64)
    pair_group = np.asarray([metadata[pair_index[k][next(iter(pair_index[k]))]]["detailed_group"]
                             for k in pair_keys], dtype=object)
    pair_visible = np.isin(pair_group, ["in_frame", "out_of_frame_surface_visible"])

    def pair_mask(group: str) -> np.ndarray:
        if group == "all":
            return np.ones(len(pair_keys), dtype=bool)
        if group == "visible_target":
            return pair_visible
        return pair_group == group

    for name, pred_flat in preds.items():
        pair_pred = np.asarray([float(np.mean([pred_flat[i] for i in pair_index[k].values()]))
                                for k in pair_keys], dtype=np.float64)
        block = {}
        for group in GROUP_SET:
            gm = pair_mask(group)
            for bin_name in BINS:
                mask = gm & np.asarray([dist_bin(t) == bin_name for t in pair_target])
                t, p = pair_target[mask], pair_pred[mask]
                if t.size == 0:
                    block[f"{group}/{bin_name}"] = {"unique_pose_ray_pairs": 0}
                    continue
                err = p - t
                near_pos = np.maximum(err[t <= 1.5], 0.0)
                block[f"{group}/{bin_name}"] = {
                    "unique_pose_ray_pairs": int(t.size),
                    "mean_signed_error_m": float(np.mean(err)),
                    "mae_m": float(np.mean(np.abs(err))),
                    "mean_prediction_m": float(np.mean(p)),
                    "near_false_safe_count": int(np.count_nonzero((t <= 1.5) & (err >= 0.5))),
                    "near_denominator": int(np.count_nonzero(t <= 1.5)),
                    "near_overestimate_p95_m": float(np.quantile(near_pos, 0.95)) if near_pos.size else None,
                    "far_false_danger_count": int(np.count_nonzero((t >= 3.0) & (err <= -0.5))),
                    "far_denominator": int(np.count_nonzero(t >= 3.0)),
                }
        summary["pair_level"]["models"][name] = block

        # Clean/masked pairing on near targets: one pair per pose-ray, both flavors present.
        deltas, transitions = [], {"clean_only": 0, "masked_only": 0}
        for key in pair_keys:
            flavors = pair_index[key]
            if "clean" not in flavors or "masked" not in flavors:
                continue
            target = metadata[flavors["clean"]]["target_m"]
            if target > 1.5:
                continue
            clean_err = pred_flat[flavors["clean"]] - target
            masked_err = pred_flat[flavors["masked"]] - target
            deltas.append(abs(masked_err) - abs(clean_err))
            clean_fail, masked_fail = clean_err >= 0.5, masked_err >= 0.5
            transitions["clean_only"] += int(clean_fail and not masked_fail)
            transitions["masked_only"] += int(masked_fail and not clean_fail)
        summary["pair_level"].setdefault("clean_masked_near_pairs", {})[name] = {
            "near_pose_ray_pairs": len(deltas),
            "masked_minus_clean_abs_error_mean_m": float(np.mean(deltas)) if deltas else None,
            "masked_worse_pairs": int(np.count_nonzero(np.asarray(deltas) > 1e-9)),
            "masked_better_pairs": int(np.count_nonzero(np.asarray(deltas) < -1e-9)),
            "ties": int(np.count_nonzero(np.abs(np.asarray(deltas)) <= 1e-9)),
            "false_safe_clean_only": transitions["clean_only"],
            "false_safe_masked_only": transitions["masked_only"],
        }

    old = "near_coverage_epoch3"
    new = "far_danger_0p5_epoch3"
    old_pair = np.asarray([float(np.mean([preds[old][i] for i in pair_index[k].values()])) for k in pair_keys])
    new_pair = np.asarray([float(np.mean([preds[new][i] for i in pair_index[k].values()])) for k in pair_keys])
    for group in ("all", "visible_target", "in_frame", "out_of_frame_surface_visible", "fully_invisible"):
        gm = pair_mask(group)
        for bin_name in BINS:
            mask = gm & np.asarray([dist_bin(t) == bin_name for t in pair_target])
            if not mask.any():
                continue
            t = pair_target[mask]
            d_abs = np.abs(new_pair[mask] - t) - np.abs(old_pair[mask] - t)
            d_pred = new_pair[mask] - old_pair[mask]
            summary["pair_level"]["epoch3_to_far_danger_0p5"][f"{group}/{bin_name}"] = {
                "unique_pose_ray_pairs": int(mask.sum()),
                "absolute_error_better": int(np.count_nonzero(d_abs < -1e-9)),
                "absolute_error_worse": int(np.count_nonzero(d_abs > 1e-9)),
                "ties": int(np.count_nonzero(np.abs(d_abs) <= 1e-9)),
                "mean_prediction_shift_m": float(np.mean(d_pred)),
                "mean_abs_error_change_m": float(np.mean(d_abs)),
            }

    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    with (out / "per_sample_per_ray.csv").open("w", newline="", encoding="utf-8") as f:
        fields = (["sample", "source", "flavor", "ray_index", "ray_angle_deg", "label_m", "visibility"]
                  + [f"{n}_{kind}" for n in MODELS for kind in ("pred_m", "error_m")])
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for i, meta in enumerate(metadata):
            row = {"sample": meta["sample"], "source": meta["source"], "flavor": meta["flavor"],
                   "ray_index": meta["ray_index"], "ray_angle_deg": meta["angle_deg"],
                   "label_m": targets_flat[i], "visibility": meta["detailed_group"]}
            for name in MODELS:
                row[f"{name}_pred_m"] = float(preds[name][i])
                row[f"{name}_error_m"] = float(preds[name][i] - targets_flat[i])
            writer.writerow(row)
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    create_plot(summary, out / "far_underestimate_comparison.png", args.output_dir.name)
    brief = {}
    for name in MODELS:
        record = summary["record_level"]["models"][name]
        brief[name] = {
            group: {key: record[group][key] for key in
                    ("mae_m", "bias_m", "false_safe", "near_overestimate_p95_m",
                     "false_danger", "far_underestimate_ge_0p5_rate")}
            for group in ("all", "visible_target", "in_frame")
        }
        brief[name]["pair_all"] = {
            bin_name: {key: summary["pair_level"]["models"][name][f"all/{bin_name}"][key]
                       for key in ("unique_pose_ray_pairs", "mean_signed_error_m", "mae_m",
                                   "near_false_safe_count", "near_denominator",
                                   "far_false_danger_count", "far_denominator")}
            for bin_name in BINS
        }
    print(json.dumps(brief, indent=2))
    return 0


def create_plot(summary: dict, destination: Path, run_label: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = list(MODELS)
    labels = ["medium", "epoch13", "near-coverage epoch3", "far-weight 0.5 epoch3"]
    colors = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]
    fig, axes = plt.subplots(2, 2, figsize=(13.5, 9.0))
    record = summary["record_level"]["models"]

    # A: record-level event rates.
    ax = axes[0, 0]
    measures = [("visible_target", "false_safe_rate", "Visible near false-safe"),
                ("all", "far_underestimate_ge_0p5_rate", "Far underestimate")]
    x = np.arange(len(measures))
    width = 0.2
    for j, name in enumerate(names):
        values = [record[name][g][m] * 100 for g, m, _ in measures]
        bars = ax.bar(x + (j - 1.5) * width, values, width, color=colors[j], label=labels[j],
                      edgecolor="white", linewidth=0.8, zorder=3)
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, value + 1.5, f"{value:.1f}",
                    ha="center", va="bottom", fontsize=7.6, color="#3f3e39")
    ax.set_xticks(x, [label for _, _, label in measures])
    ax.set_ylabel("Percent of eligible ray records")
    ax.set_title("A. Record level (5940 input-ray records)", loc="left", fontsize=10.5)
    ax.set_ylim(0, 118)

    # B: pair-level signed error by distance bin.
    ax = axes[0, 1]
    bins = ["near_le_1p5m", "mid_1p5_to_3m", "far_ge_3m"]
    bin_labels = ["near ≤1.5 m", "mid 1.5–3 m", "far ≥3 m"]
    x = np.arange(len(bins))
    pair = summary["pair_level"]["models"]
    for j, name in enumerate(names):
        values = [pair[name][f"all/{b}"]["mean_signed_error_m"] for b in bins]
        bars = ax.bar(x + (j - 1.5) * width, values, width, color=colors[j], label=labels[j],
                      edgecolor="white", linewidth=0.8, zorder=3)
        if j == len(names) - 1:
            for bar, value in zip(bars, values):
                ax.text(bar.get_x() + bar.get_width() / 2, value + (0.10 if value >= 0 else -0.10),
                        f"{value:+.2f}", ha="center", va="bottom" if value >= 0 else "top",
                        fontsize=7.6, color="#3f3e39")
    ax.axhline(0, color="#8a8984", linewidth=0.9, zorder=2)
    ax.set_xticks(x, bin_labels)
    ax.set_ylabel("Mean signed error, predicted − label (m)")
    ax.set_title("B. Pair level (2970 pose-ray pairs, clean/masked averaged)", loc="left", fontsize=10.5)
    ax.set_ylim(-3.45, 0.45)

    # C: per-ray visible near false-safe rate.
    ax = axes[1, 0]
    ray_x = np.arange(11)
    for name, label, color in zip(names, labels, colors):
        per_ray = summary["record_level"][f"per_ray_{name}"]
        rates = [per_ray[f"{i}:{ANGLES[i]:+.0f}deg"]["visible_target"]["false_safe_rate"] * 100
                 for i in range(11)]
        ax.plot(ray_x, rates, marker="o", markersize=4.2, linewidth=2, color=color, label=label, zorder=3)
    ax.set_xticks(ray_x, [f"{a:+.0f}°" for a in ANGLES], fontsize=8.5)
    ax.set_ylabel("Percent of visible near records")
    ax.set_title("C. Visible near false-safe rate by beam", loc="left", fontsize=10.5)

    # D: clean vs masked near false-safe count.
    ax = axes[1, 1]
    x = np.arange(2)
    for j, name in enumerate(names):
        values = [record[name][f"input_{flavor}"]["false_safe"] for flavor in ("clean", "masked")]
        bars = ax.bar(x + (j - 1.5) * width, values, width, color=colors[j], label=labels[j],
                      edgecolor="white", linewidth=0.8, zorder=3)
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, value + 1.5, str(value),
                    ha="center", va="bottom", fontsize=7.6, color="#3f3e39")
    ax.set_xticks(x, ["clean", "masked"])
    ax.set_ylabel("Near false-safe records")
    ax.set_title("D. Clean vs masked (record level)", loc="left", fontsize=10.5)
    ax.set_ylim(0, 150)

    for ax in axes.flat:
        ax.grid(axis="y", alpha=0.25, linewidth=0.7, zorder=0)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    handles, legend_labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, legend_labels, loc="upper center", bbox_to_anchor=(0.5, 0.945),
               ncol=4, frameon=False)
    fig.suptitle(f"Far-underestimate loss weight single-factor comparison — {run_label}",
                 fontsize=11.5, y=0.985)
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    fig.savefig(destination, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
