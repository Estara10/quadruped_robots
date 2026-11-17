#!/usr/bin/env python3
"""Compare fixed D435i ray checkpoints on the combined near-coverage validation split."""
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


BASE = ROOT / "logs/real_go2/20261116_d435i_ray_adaptation_data/adaptation_v2_current_mount"
SUPPLEMENT = ROOT / "logs/real_go2/20261117_d435i_near_coverage_supplement_v1"
MEDIUM = ROOT / "archive/ray_pred_finetune/mujoco_finetune_medium_20260611/ray_pred_mujoco_finetuned_best.pt"
EPOCH13 = ROOT / "logs/real_go2/20261117_d435i_fullnet_preparation/training_runs/d435i_fullnet_300epoch_20261117/ray_pred_mujoco_finetuned_best.pt"
NEW_CANDIDATE = ROOT / "logs/real_go2/20261117_d435i_near_coverage_training/medium_fullnet_near_coverage_60ep/ray_pred_mujoco_finetuned_best.pt"
ANGLES = [-45. + 9. * i for i in range(11)]
GROUPS = ("visible_target", "in_frame", "out_of_frame_surface_visible", "fully_invisible", "no_hit_within_range")


def read_visibility(paths: list[Path]) -> dict:
    visibility = {}
    for path in paths:
        with path.open(newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                if row["hit_geom"] == "range_cap":
                    group = "no_hit_within_range"
                elif row["label_hitpoint_in_image"].lower() == "true":
                    group = "in_frame"
                elif row["obstacle_has_visible_pixels_in_render"].lower() == "true":
                    group = "out_of_frame_surface_visible"
                else:
                    group = "fully_invisible"
                sample = row["sample"]
                ray = int(row["ray_index"])
                value = {**row, "group": group}
                visibility.setdefault(sample, {})[ray] = value
    for sample, rays in visibility.items():
        for ray, value in list(rays.items()):
            if value["group"] in ("in_frame", "out_of_frame_surface_visible"):
                value = dict(value)
                value["group"] = "visible_target"
                # Keep the detailed class for the separate class group.
                value["detailed_group"] = rays[ray]["group"]
                rays[ray] = value
    # Restore detailed classes from source fields after marking the visible union.
    for sample, rays in visibility.items():
        for ray, value in rays.items():
            if value["group"] == "visible_target":
                value["detailed_group"] = (
                    "in_frame" if value["label_hitpoint_in_image"].lower() == "true"
                    else "out_of_frame_surface_visible"
                )
    return visibility


def group_mask(groups: np.ndarray, name: str) -> np.ndarray:
    if name == "visible_target":
        return np.isin(groups, ["in_frame", "out_of_frame_surface_visible"])
    return groups == name


def metric_block(target: np.ndarray, prediction: np.ndarray, groups: np.ndarray,
                 selected: np.ndarray | None = None) -> dict:
    if selected is None:
        selected = np.ones(target.shape, dtype=bool)
    target = target[selected]
    prediction = prediction[selected]
    groups = groups[selected]
    if target.size == 0:
        return {"ray_values": 0}
    error = prediction - target
    near = target <= 1.5
    near_pos = np.maximum(error[near], 0.)
    far = target >= 3.0
    return {
        "ray_values": int(target.size),
        "mae_m": float(np.mean(np.abs(error))),
        "rmse_m": float(np.sqrt(np.mean(error ** 2))),
        "bias_m": float(np.mean(error)),
        "near_target_le_1p5_count": int(near.sum()),
        "false_safe": int(np.count_nonzero(near & (error >= .5))),
        "false_safe_rate": float(np.count_nonzero(near & (error >= .5)) / max(1, near.sum())),
        "near_overestimate_p95_m": float(np.quantile(near_pos, .95)) if near_pos.size else None,
        "near_overestimate_max_m": float(np.max(near_pos)) if near_pos.size else None,
        "false_danger": int(np.count_nonzero(far & (error <= -.5))),
        "far_target_ge_3_count": int(far.sum()),
        "far_underestimate_ge_0p5_rate": float(np.count_nonzero(far & (error <= -.5)) / max(1, far.sum())),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--batch-size", type=int, default=32)
    args = parser.parse_args()
    device = torch.device(args.device)
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError(f"CUDA requested but unavailable: {args.device}")
    model_paths = {"medium": MEDIUM, "epoch13": EPOCH13, "near_coverage_epoch3": NEW_CANDIDATE}
    for path in [*model_paths.values(), BASE / "data/validation/label.pkl",
                 SUPPLEMENT / "data/validation/label.pkl"]:
        if not path.is_file():
            raise FileNotFoundError(path)
    dataset = RayDataset([BASE / "data/validation", SUPPLEMENT / "data/validation"])
    visibility_paths = [BASE / "ray_projection_visibility.csv", SUPPLEMENT / "ray_projection_visibility.csv"]
    visibility = read_visibility(visibility_paths)

    target_rows = []
    metadata = []
    for index, (image_path, label) in enumerate(dataset.items):
        stem = image_path.stem
        sample, flavor = stem.rsplit("_", 1)
        if flavor not in ("clean", "masked") or sample not in visibility:
            raise RuntimeError(f"invalid validation sample: {image_path}")
        source = "supplement_validation" if SUPPLEMENT.name in image_path.parts else "v2_validation"
        for ray_index in range(11):
            vis = visibility[sample][ray_index]
            original_group = vis["group"]
            detailed = vis.get("detailed_group", original_group)
            target_rows.append(float(np.clip(label[ray_index], RAY_MIN, RAY_MAX)))
            metadata.append({"sample": sample, "flavor": flavor, "ray_index": ray_index,
                             "angle_deg": ANGLES[ray_index], "group": original_group,
                             "detailed_group": detailed, "source": source,
                             "hit_geom": vis["hit_geom"],
                             "label_hitpoint_in_image": vis["label_hitpoint_in_image"],
                             "obstacle_has_visible_pixels_in_render": vis["obstacle_has_visible_pixels_in_render"]})
    targets_flat = np.asarray(target_rows, dtype=np.float32)
    groups_flat = np.asarray([m["detailed_group"] for m in metadata], dtype=object)
    sources_flat = np.asarray([m["source"] for m in metadata], dtype=object)
    sample_flavors = [(Path(item[0]).stem.rsplit("_", 1)[0], Path(item[0]).stem.rsplit("_", 1)[1])
                      for item in dataset.items]

    models = {}
    all_model_rows = {}
    for name, model_path in model_paths.items():
        model = torch.jit.load(str(model_path), map_location=device).eval()
        pred_matrix = np.empty((len(dataset), 11), dtype=np.float32)
        with torch.no_grad():
            for start in range(0, len(dataset), args.batch_size):
                stop = min(start + args.batch_size, len(dataset))
                depth = torch.stack([dataset[i][0] for i in range(start, stop)]).to(device)
                pred = torch.exp2(model(depth.unsqueeze(1).repeat(1, 3, 1, 1))).clamp(RAY_MIN, RAY_MAX)
                pred_matrix[start:stop] = pred.cpu().numpy()
        models[name] = pred_matrix
        pred_flat = pred_matrix.reshape(-1)
        all_model_rows[name] = pred_flat

    summary = {
        "validation_only": True,
        "final_holdout_read_or_evaluated": False,
        "dataset_dirs": [str(BASE / "data/validation"), str(SUPPLEMENT / "data/validation")],
        "validation_poses": len(dataset) // 2,
        "input_frames": len(dataset),
        "ray_values_per_model": int(targets_flat.size),
        "metric_contract": "false_safe: target<=1.5 and prediction-target>=0.5m; near P95 is quantile of max(error,0); far underestimation: target>=3.0 and target-prediction>=0.5m; clean/masked are paired inputs, not independent poses",
        "visibility_definition": "in_frame; out_of_frame_surface_visible; fully_invisible hit geometry with zero rendered pixels; no_hit_within_range is 6m range cap; visible_target is first two combined",
        "models": {},
        "clean_masked_pairs": {},
        "per_ray_near": {},
    }
    for name, pred_flat in all_model_rows.items():
        result = {"all": metric_block(targets_flat, pred_flat, groups_flat)}
        for group in GROUPS:
            result[group] = metric_block(targets_flat, pred_flat, groups_flat, group_mask(groups_flat, group))
        for source in ("v2_validation", "supplement_validation"):
            result[source] = metric_block(targets_flat, pred_flat, groups_flat, sources_flat == source)
        summary["models"][name] = result

        per_ray = {}
        for ray_index, angle in enumerate(ANGLES):
            indices = np.asarray([m["ray_index"] == ray_index for m in metadata])
            row = {}
            for group in ("visible_target", "in_frame", "out_of_frame_surface_visible", "fully_invisible"):
                row[group] = metric_block(targets_flat, pred_flat, groups_flat,
                                          indices & group_mask(groups_flat, group))
            per_ray[f"{ray_index}:{angle:+.0f}deg"] = row
        summary["per_ray_near"][name] = per_ray

        frame_rows = {}
        for flavor in ("clean", "masked"):
            frame_mask = np.asarray([m["flavor"] == flavor for m in metadata])
            frame_rows[flavor] = metric_block(targets_flat, pred_flat, groups_flat, frame_mask)
        by_pair = {}
        for i, (sample, flavor) in enumerate(sample_flavors):
            for ray_index in range(11):
                flat_index = i * 11 + ray_index
                if targets_flat[flat_index] <= 1.5:
                    by_pair.setdefault((sample, ray_index), {})[flavor] = {
                        "abs_error": abs(float(pred_flat[flat_index] - targets_flat[flat_index])),
                        "false_safe": bool(pred_flat[flat_index] - targets_flat[flat_index] >= .5),
                    }
        deltas = []
        clean_to_masked = masked_to_clean = 0
        clean_false_safe = masked_false_safe = 0
        complete = 0
        for pair in by_pair.values():
            if "clean" not in pair or "masked" not in pair:
                continue
            complete += 1
            deltas.append(pair["masked"]["abs_error"] - pair["clean"]["abs_error"])
            clean_false_safe += int(pair["clean"]["false_safe"])
            masked_false_safe += int(pair["masked"]["false_safe"])
            clean_to_masked += int(not pair["clean"]["false_safe"] and pair["masked"]["false_safe"])
            masked_to_clean += int(pair["clean"]["false_safe"] and not pair["masked"]["false_safe"])
        summary["clean_masked_pairs"][name] = {
            "near_pose_ray_pairs": complete,
            "clean": frame_rows["clean"],
            "masked": frame_rows["masked"],
            "masked_minus_clean_abs_error_mean_m": float(np.mean(deltas)) if deltas else None,
            "masked_worse_pairs": int(np.count_nonzero(np.asarray(deltas) > 1e-9)),
            "masked_better_pairs": int(np.count_nonzero(np.asarray(deltas) < -1e-9)),
            "ties": int(np.count_nonzero(np.abs(deltas) <= 1e-9)),
            "near_false_safe_clean": clean_false_safe,
            "near_false_safe_masked": masked_false_safe,
            "clean_to_masked_false_safe": clean_to_masked,
            "masked_to_clean_false_safe": masked_to_clean,
        }

    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    with (out / "per_sample_per_ray.csv").open("w", newline="", encoding="utf-8") as f:
        fields = ["sample", "source", "flavor", "ray_index", "ray_angle_deg", "label_m",
                  "visibility", "hit_geom", "label_hitpoint_in_image", "obstacle_has_visible_pixels_in_render",
                  "medium_pred_m", "medium_error_m", "epoch13_pred_m", "epoch13_error_m",
                  "near_coverage_epoch3_pred_m", "near_coverage_epoch3_error_m"]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for i, meta in enumerate(metadata):
            row = {"sample": meta["sample"], "source": meta["source"], "flavor": meta["flavor"],
                   "ray_index": meta["ray_index"], "ray_angle_deg": meta["angle_deg"],
                   "label_m": targets_flat[i], "visibility": meta["detailed_group"],
                   "hit_geom": meta["hit_geom"],
                   "label_hitpoint_in_image": meta["label_hitpoint_in_image"],
                   "obstacle_has_visible_pixels_in_render": meta["obstacle_has_visible_pixels_in_render"]}
            for name, pred_flat in all_model_rows.items():
                column = {"medium": "medium", "epoch13": "epoch13", "near_coverage_epoch3": "near_coverage_epoch3"}[name]
                row[f"{column}_pred_m"] = pred_flat[i]
                row[f"{column}_error_m"] = pred_flat[i] - targets_flat[i]
            writer.writerow(row)
    (out / "summary.json").write_text(json.dumps(summary, indent=2))
    create_plot(summary, out / "validation_comparison.png")
    print(json.dumps({name: {k: summary["models"][name][k] for k in
                             ("all", "visible_target", "in_frame", "out_of_frame_surface_visible", "fully_invisible")}
                      for name in models}, indent=2))
    return 0


def create_plot(summary: dict, destination: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = list(summary["models"])
    labels = ["medium", "epoch13", "new epoch3"]
    colors = ["#566573", "#c0392b", "#2471a3"]
    groups = ["in_frame", "out_of_frame_surface_visible", "fully_invisible"]
    group_labels = ["Hitpoint in frame", "Surface visible, hitpoint outside", "Fully invisible"]
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.2))
    x = np.arange(len(groups))
    width = .24
    for j, name in enumerate(names):
        values = [summary["models"][name][g]["false_safe_rate"] * 100 for g in groups]
        axes[0, 0].bar(x + (j - 1) * width, values, width, label=labels[j], color=colors[j])
        p95 = [summary["models"][name][g]["near_overestimate_p95_m"] for g in groups]
        axes[0, 1].bar(x + (j - 1) * width, p95, width, label=labels[j], color=colors[j])
    axes[0, 0].set_title("Near false-safe rate (error ≥ 0.5 m)")
    axes[0, 0].set_ylabel("Percent of near ray inputs")
    axes[0, 0].set_xticks(x, group_labels, rotation=15, ha="right")
    axes[0, 1].set_title("Near overestimate P95")
    axes[0, 1].set_ylabel("m")
    axes[0, 1].set_xticks(x, group_labels, rotation=15, ha="right")
    ray_x = np.arange(11)
    for name, label, color in zip(names, labels, colors):
        counts = [summary["per_ray_near"][name][f"{i}:{angle:+.0f}deg"]["visible_target"]["false_safe_rate"] * 100
                  for i, angle in enumerate(ANGLES)]
        axes[1, 0].plot(ray_x, counts, marker="o", label=label, color=color)
    axes[1, 0].set_title("Visible near false-safe rate by ray")
    axes[1, 0].set_ylabel("Percent")
    axes[1, 0].set_xticks(ray_x, [f"{a:+.0f}°" for a in ANGLES], rotation=35)
    for j, name in enumerate(names):
        metrics = summary["clean_masked_pairs"][name]
        clean_rate = metrics["clean"]["false_safe_rate"] * 100
        masked_rate = metrics["masked"]["false_safe_rate"] * 100
        axes[1, 1].bar(j - .16, clean_rate, .32, color=colors[j], alpha=.55)
        axes[1, 1].bar(j + .16, masked_rate, .32, color=colors[j])
    axes[1, 1].set_title("Near false-safe rate by input type")
    axes[1, 1].set_ylabel("Percent")
    axes[1, 1].set_xticks(np.arange(3), labels)
    axes[1, 1].legend(handles=[plt.Rectangle((0, 0), 1, 1, color="#888", alpha=.55, label="clean"),
                              plt.Rectangle((0, 0), 1, 1, color="#888", label="masked")])
    for ax in axes.flat:
        ax.grid(axis="y", alpha=.25)
    handles, legend_labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(handles, legend_labels, loc="upper center", ncol=3, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, .94))
    fig.savefig(destination, dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    raise SystemExit(main())
