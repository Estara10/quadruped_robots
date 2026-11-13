#!/usr/bin/env python3
"""Finetune Ray-Pred ResNet18 on MuJoCo depth/ray datasets."""

from __future__ import annotations

import argparse
import copy
import csv
import json
import pickle
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np
import torch
from torch.utils.data import DataLoader, Dataset, random_split

ROOT = Path.home() / "quadruped_robots"
DEFAULT_BASE_MODEL = (
    ROOT
    / "ABS"
    / "training"
    / "legged_gym"
    / "legged_gym"
    / "depth_logs"
    / "20260528-143154-resnet18-go2_depth"
    / "depth_lidar_model_20260528-143154_250.pt"
)
DEFAULT_OUTPUT_ROOT = ROOT / "logs" / "ray_pred_finetune"
RAY_MIN = 0.1
RAY_MAX = 6.0


class RayDataset(Dataset):
    def __init__(self, dataset_dirs: Sequence[Path]) -> None:
        self.items: List[Tuple[Path, np.ndarray]] = []
        for dataset_dir in dataset_dirs:
            label_path = dataset_dir / "label.pkl"
            if not label_path.exists():
                raise FileNotFoundError(label_path)
            with label_path.open("rb") as f:
                labels = pickle.load(f)
            for key, ray_m in labels.items():
                image_path = dataset_dir / f"{key}.npy"
                if image_path.exists():
                    self.items.append((image_path, np.asarray(ray_m, dtype=np.float32)))
        if not self.items:
            raise RuntimeError("No dataset samples found")

    def __len__(self) -> int:
        return len(self.items)

    def __getitem__(self, index: int):
        image_path, ray_m = self.items[index]
        depth_m = np.load(image_path, allow_pickle=True).astype(np.float32)
        depth_log = np.log2(np.clip(depth_m, RAY_MIN, RAY_MAX)).astype(np.float32)
        ray_log = np.log2(np.clip(ray_m, RAY_MIN, RAY_MAX)).astype(np.float32)
        return torch.from_numpy(depth_log), torch.from_numpy(ray_log)


def weighted_ray_loss(
    pred_log: torch.Tensor,
    target_log: torch.Tensor,
    near_weight: float,
    false_safe_weight: float,
    false_danger_weight: float,
) -> torch.Tensor:
    target_m = torch.clamp(torch.exp2(target_log), RAY_MIN, RAY_MAX)
    pred_m = torch.clamp(torch.exp2(pred_log), RAY_MIN, RAY_MAX)
    weights = torch.ones_like(target_log)
    weights = weights + near_weight * torch.clamp((1.5 - target_m) / 1.4, min=0.0, max=1.0)
    false_safe = (target_m <= 1.5) & (pred_m > target_m + 0.5)
    false_danger = (target_m >= 3.0) & (pred_m < target_m - 0.5)
    weights = weights + false_safe_weight * false_safe.float()
    weights = weights + false_danger_weight * false_danger.float()
    return torch.mean(weights * (pred_log - target_log) ** 2)


def eval_model(model, loader: DataLoader, device: torch.device) -> dict:
    model.eval()
    predictions = []
    targets = []
    with torch.no_grad():
        for depth_log, target_log in loader:
            depth_log = depth_log.to(device)
            target_log = target_log.to(device)
            inputs = depth_log.unsqueeze(1).repeat(1, 3, 1, 1)
            pred_log = model(inputs)
            targets.append(torch.clamp(torch.exp2(target_log), RAY_MIN, RAY_MAX).detach().cpu())
            predictions.append(torch.clamp(torch.exp2(pred_log), RAY_MIN, RAY_MAX).detach().cpu())
    target_m = torch.cat(targets)
    pred_m = torch.cat(predictions)
    err_m = pred_m - target_m
    near = target_m <= 1.5
    near_over = near & (err_m >= 0.5)
    far_under = (target_m >= 3.0) & (err_m <= -0.5)
    return {
        "ray_values": int(target_m.numel()),
        "mae_m": float(err_m.abs().mean().item()),
        "rmse_m": float(torch.sqrt(torch.mean(err_m ** 2)).item()),
        "bias_m": float(err_m.mean().item()),
        "max_abs_m": float(err_m.abs().max().item()),
        "per_ray_mae_m": err_m.abs().mean(0).tolist(),
        "per_ray_bias_m": err_m.mean(0).tolist(),
        "near_target_le_1p5_count": int(near.sum().item()),
        "false_safe": int(near_over.sum().item()),
        "false_safe_rate": float(near_over.sum().item() / max(1, near.sum().item())),
        "near_overestimate_p95_m": float(err_m[near].clamp(min=0).quantile(0.95).item()) if bool(near.any()) else None,
        "near_overestimate_max_m": float(err_m[near].clamp(min=0).max().item()) if bool(near.any()) else None,
        "per_ray_near_false_safe": near_over.sum(0).tolist(),
        "per_ray_near_eligible": near.sum(0).tolist(),
        "excessive_underestimate_ge_0p5_count": int((err_m <= -0.5).sum().item()),
        "false_danger": int(far_under.sum().item()),
        "false_danger_rate": float(far_under.sum().item() / max(1, (target_m >= 3.0).sum().item())),
    }


def model_score(metrics: dict, false_safe_penalty: float, false_danger_penalty: float, mae_weight: float) -> float:
    return (
        false_safe_penalty * float(metrics["false_safe"])
        + false_danger_penalty * float(metrics["false_danger"])
        + mae_weight * float(metrics["mae_m"])
    )


def eval_visibility_groups(model, dataset: RayDataset, device: torch.device, visibility_csv: Path,
                          batch_size: int) -> dict:
    """Report metrics separately for in-frame and out-of-frame ray hitpoints."""
    visibility = {}
    with visibility_csv.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            sample = row["sample"]
            visibility.setdefault(sample, np.zeros(11, dtype=bool))
            visibility[sample][int(row["ray_index"])] = row["label_hitpoint_in_image"].lower() == "true"

    grouped_rows = {"in_frame": [], "out_of_frame": []}
    model.eval()
    with torch.no_grad():
        for start in range(0, len(dataset), batch_size):
            indices = range(start, min(start + batch_size, len(dataset)))
            samples, depths, targets = [], [], []
            for index in indices:
                image_path, _ = dataset.items[index]
                stem = image_path.stem
                sample = stem.rsplit("_", 1)[0] if stem.endswith(("_clean", "_masked")) else stem
                if sample not in visibility:
                    raise KeyError(f"No visibility row for sample {sample} in {visibility_csv}")
                depth_log, target_log = dataset[index]
                samples.append(visibility[sample])
                depths.append(depth_log)
                targets.append(target_log)
            depth_batch = torch.stack(depths).to(device)
            target_batch = torch.clamp(torch.exp2(torch.stack(targets).to(device)), RAY_MIN, RAY_MAX)
            pred_batch = torch.clamp(
                torch.exp2(model(depth_batch.unsqueeze(1).repeat(1, 3, 1, 1))), RAY_MIN, RAY_MAX
            )
            error_batch = (pred_batch - target_batch).cpu().numpy()
            target_np = target_batch.cpu().numpy()
            for row_index, visible_mask in enumerate(samples):
                for ray_index, is_visible in enumerate(visible_mask):
                    grouped_rows["in_frame" if is_visible else "out_of_frame"].append(
                        (ray_index, float(target_np[row_index, ray_index]), float(error_batch[row_index, ray_index]))
                    )

    result = {}
    for name, rows in grouped_rows.items():
        if not rows:
            result[name] = {"ray_values": 0}
            continue
        by_ray = [[] for _ in range(11)]
        target = np.asarray([row[1] for row in rows], dtype=np.float64)
        error = np.asarray([row[2] for row in rows], dtype=np.float64)
        for ray_index, _, err in rows:
            by_ray[ray_index].append(err)
        near = target <= 1.5
        near_positive = np.maximum(error[near], 0.0)
        result[name] = {
            "ray_values": int(error.size),
            "mae_m": float(np.mean(np.abs(error))),
            "bias_m": float(np.mean(error)),
            "near_target_le_1p5_count": int(np.count_nonzero(near)),
            "false_safe": int(np.count_nonzero(near & (error >= .5))),
            "near_overestimate_p95_m": float(np.quantile(near_positive, .95)) if near_positive.size else None,
            "false_danger": int(np.count_nonzero((target >= 3.0) & (error <= -.5))),
            "per_ray_mae_m": [float(np.mean(np.abs(x))) if x else None for x in by_ray],
            "per_ray_bias_m": [float(np.mean(x)) if x else None for x in by_ray],
        }
    return result


def train(args: argparse.Namespace) -> Path:
    device = torch.device(args.device if args.device else ("cuda" if torch.cuda.is_available() else "cpu"))
    dataset_dirs = [Path(p).expanduser() for p in args.datasets]
    dataset = RayDataset(dataset_dirs)
    if args.validation_datasets:
        train_dataset = dataset
        val_dataset = RayDataset([Path(p).expanduser() for p in args.validation_datasets])
        train_size, val_size = len(train_dataset), len(val_dataset)
        split_mode = "explicit_disjoint_directories"
    else:
        val_size = max(1, int(len(dataset) * args.val_fraction))
        train_size = len(dataset) - val_size
        generator = torch.Generator().manual_seed(args.seed)
        train_dataset, val_dataset = random_split(dataset, [train_size, val_size], generator=generator)
        split_mode = "legacy_random_split"
    train_loader = DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True, num_workers=args.num_workers)
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers)

    model = torch.jit.load(str(args.base_model), map_location=device)
    if args.trainable == "layer4_fc":
        trainable = []
        for name, parameter in model.named_parameters():
            parameter.requires_grad_(name.startswith("resnet.layer4.") or name.startswith("resnet.fc."))
            if parameter.requires_grad:
                trainable.append(parameter)
        if not trainable:
            raise RuntimeError("layer4_fc selected but no matching TorchScript parameters were found")
        model.eval()
    else:
        for parameter in model.parameters():
            parameter.requires_grad_(True)
        trainable = list(model.parameters())
        model.train()
    optimizer = torch.optim.AdamW(trainable, lr=args.lr, weight_decay=args.weight_decay)

    output_dir = args.output_root / args.run_name
    output_dir.mkdir(parents=True, exist_ok=True)
    best_path = output_dir / "ray_pred_mujoco_finetuned_best.pt"
    last_path = output_dir / "ray_pred_mujoco_finetuned_last.pt"
    best_score = float("inf")
    best_metrics = None
    best_selection_key = None
    selected_epoch = None
    history = []
    baseline_metrics = eval_model(model, val_loader, device)
    baseline_visibility = (
        eval_visibility_groups(model, val_dataset, device, args.validation_visibility_csv, args.batch_size)
        if args.validation_visibility_csv else None
    )

    for epoch in range(1, args.epochs + 1):
        model.train() if args.trainable == "all" else model.eval()
        losses = []
        for depth_log, target_log in train_loader:
            depth_log = depth_log.to(device)
            target_log = target_log.to(device)
            inputs = depth_log.unsqueeze(1).repeat(1, 3, 1, 1)
            pred_log = model(inputs)
            loss = weighted_ray_loss(
                pred_log,
                target_log,
                args.near_weight,
                args.false_safe_weight,
                args.false_danger_weight,
            )
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), args.grad_clip)
            optimizer.step()
            losses.append(float(loss.item()))

        metrics = eval_model(model, val_loader, device)
        if args.validation_visibility_csv:
            metrics["visibility_groups"] = eval_visibility_groups(
                model, val_dataset, device, args.validation_visibility_csv, args.batch_size
            )
        metrics["epoch"] = epoch
        metrics["train_loss"] = float(np.mean(losses)) if losses else float("nan")
        if args.validation_datasets:
            metrics["eligible_for_selection"] = (
                metrics["false_safe"] <= baseline_metrics["false_safe"]
                and metrics["false_danger"] <= baseline_metrics["false_danger"]
                and (metrics["near_overestimate_p95_m"] or 0.0)
                <= (baseline_metrics["near_overestimate_p95_m"] or 0.0)
            )
            metrics["score"] = None
            selection_key = (metrics["mae_m"], metrics["near_overestimate_p95_m"] or 0.0)
        else:
            metrics["eligible_for_selection"] = True
            metrics["score"] = model_score(
                metrics,
                args.false_safe_score_penalty,
                args.false_danger_score_penalty,
                args.mae_score_weight,
            )
            selection_key = (metrics["score"],)
        history.append(metrics)
        print(
            f"[Finetune] epoch={epoch:03d} loss={metrics['train_loss']:.5f} "
            f"val_mae={metrics['mae_m']:.3f} false_safe={metrics['false_safe']} "
            f"false_danger={metrics['false_danger']} eligible={metrics['eligible_for_selection']} "
            f"score={metrics['score']}"
        )
        model_cpu = copy.deepcopy(model).to("cpu")
        torch.jit.save(model_cpu, str(last_path))
        if metrics["eligible_for_selection"] and (best_selection_key is None or selection_key < best_selection_key):
            best_selection_key = selection_key
            selected_epoch = epoch
            best_score = float(metrics["score"]) if metrics["score"] is not None else float(metrics["mae_m"])
            best_metrics = dict(metrics)
            torch.jit.save(model_cpu, str(best_path))

    test_result = None
    if args.test_datasets:
        test_dataset = RayDataset([Path(p).expanduser() for p in args.test_datasets])
        if best_path.is_file():
            selected_model = torch.jit.load(str(best_path), map_location=device).eval()
            test_loader = DataLoader(test_dataset, batch_size=args.batch_size, shuffle=False,
                                     num_workers=args.num_workers)
            baseline_model = torch.jit.load(str(args.base_model), map_location=device).eval()
            test_result = {
                "baseline": eval_model(baseline_model, test_loader, device),
                "candidate": eval_model(selected_model, test_loader, device),
                "used_for_selection": False,
            }
            if args.test_visibility_csv:
                test_result["baseline_visibility_groups"] = eval_visibility_groups(
                    baseline_model, test_dataset, device, args.test_visibility_csv, args.batch_size
                )
                test_result["candidate_visibility_groups"] = eval_visibility_groups(
                    selected_model, test_dataset, device, args.test_visibility_csv, args.batch_size
                )
        else:
            test_result = {"status": "not_run_no_epoch_passed_nonregression_gate",
                           "used_for_selection": False}

    manifest = {
        "base_model": str(args.base_model),
        "datasets": [str(p) for p in dataset_dirs],
        "validation_datasets": [str(Path(p).expanduser()) for p in args.validation_datasets],
        "test_datasets": [str(Path(p).expanduser()) for p in args.test_datasets],
        "validation_visibility_csv": str(args.validation_visibility_csv) if args.validation_visibility_csv else None,
        "test_visibility_csv": str(args.test_visibility_csv) if args.test_visibility_csv else None,
        "sample_count": len(dataset),
        "train_size": train_size,
        "val_size": val_size,
        "split_mode": split_mode,
        "device": str(device),
        "trainable": args.trainable,
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "lr": args.lr,
        "near_weight": args.near_weight,
        "false_safe_weight": args.false_safe_weight,
        "false_danger_weight": args.false_danger_weight,
        "false_safe_score_penalty": args.false_safe_score_penalty,
        "false_danger_score_penalty": args.false_danger_score_penalty,
        "mae_score_weight": args.mae_score_weight,
        "best_model": str(best_path),
        "last_model": str(last_path),
        "best_metrics": best_metrics,
        "baseline_validation_metrics": baseline_metrics,
        "baseline_validation_visibility_groups": baseline_visibility,
        "selected_epoch": selected_epoch,
        "selection_status": "selected" if selected_epoch is not None else "no_epoch_passed_nonregression_gate",
        "selection_rule": ("explicit validation: require false-safe count, false-danger count, and near overestimate P95 all no worse than base; among eligible epochs minimize overall MAE, tie-break by near overestimate P95" if args.validation_datasets else "legacy weighted score"),
        "heldout_test_metrics": test_result,
        "history": history,
    }
    with (output_dir / "manifest.json").open("w") as f:
        json.dump(manifest, f, indent=2)
    return best_path if best_path.is_file() else last_path


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--datasets", nargs="+", required=True, help="Dataset directories containing .npy files and label.pkl")
    parser.add_argument("--validation-datasets", nargs="*", default=[],
                        help="Optional disjoint validation directories; when set, --datasets are training-only")
    parser.add_argument("--test-datasets", nargs="*", default=[],
                        help="Optional held-out directories evaluated after checkpoint selection")
    parser.add_argument("--validation-visibility-csv", type=Path, default=None,
                        help="Optional ray_projection_visibility.csv for in-frame/out-of-frame validation metrics")
    parser.add_argument("--test-visibility-csv", type=Path, default=None,
                        help="Optional ray_projection_visibility.csv for held-out visibility metrics")
    parser.add_argument("--base-model", type=Path, default=DEFAULT_BASE_MODEL)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--run-name", default="mujoco_finetune_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--near-weight", type=float, default=4.0)
    parser.add_argument("--false-safe-weight", type=float, default=8.0)
    parser.add_argument("--false-danger-weight", type=float, default=0.0)
    parser.add_argument("--false-safe-score-penalty", type=float, default=1.0)
    parser.add_argument("--false-danger-score-penalty", type=float, default=0.0)
    parser.add_argument("--mae-score-weight", type=float, default=10.0)
    parser.add_argument("--grad-clip", type=float, default=1.0)
    parser.add_argument("--val-fraction", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--device", default="", help="Torch device override, e.g. cpu or cuda")
    parser.add_argument("--trainable", choices=("all", "layer4_fc"), default="all",
                        help="Parameters to update; layer4_fc matches the bounded pilot training scope")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    best_path = train(args)
    manifest_path = args.output_root / args.run_name / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    if manifest["selection_status"] == "selected":
        print(f"[Finetune] Selected model: {best_path}")
    else:
        print(f"[Finetune] No checkpoint passed the validation non-regression gate; last diagnostic weights: {best_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
