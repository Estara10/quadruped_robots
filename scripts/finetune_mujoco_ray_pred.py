#!/usr/bin/env python3
"""Finetune Ray-Pred ResNet18 on MuJoCo depth/ray datasets."""

from __future__ import annotations

import argparse
import copy
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


def weighted_ray_loss(pred_log: torch.Tensor, target_log: torch.Tensor, near_weight: float, false_safe_weight: float) -> torch.Tensor:
    target_m = torch.clamp(torch.exp2(target_log), RAY_MIN, RAY_MAX)
    pred_m = torch.clamp(torch.exp2(pred_log), RAY_MIN, RAY_MAX)
    weights = torch.ones_like(target_log)
    weights = weights + near_weight * torch.clamp((1.5 - target_m) / 1.4, min=0.0, max=1.0)
    false_safe = (target_m <= 1.5) & (pred_m > target_m + 0.5)
    weights = weights + false_safe_weight * false_safe.float()
    return torch.mean(weights * (pred_log - target_log) ** 2)


def eval_model(model, loader: DataLoader, device: torch.device) -> dict:
    model.eval()
    abs_errors = []
    false_safe = 0
    false_danger = 0
    with torch.no_grad():
        for depth_log, target_log in loader:
            depth_log = depth_log.to(device)
            target_log = target_log.to(device)
            inputs = depth_log.unsqueeze(1).repeat(1, 3, 1, 1)
            pred_log = model(inputs)
            target_m = torch.clamp(torch.exp2(target_log), RAY_MIN, RAY_MAX)
            pred_m = torch.clamp(torch.exp2(pred_log), RAY_MIN, RAY_MAX)
            err_m = pred_m - target_m
            abs_errors.append(torch.abs(err_m).detach().cpu())
            false_safe += int(((target_m <= 1.5) & (err_m >= 0.5)).sum().item())
            false_danger += int(((target_m >= 3.0) & (-err_m >= 0.5)).sum().item())
    all_abs = torch.cat([x.reshape(-1) for x in abs_errors])
    return {
        "mae_m": float(all_abs.mean().item()),
        "rmse_m": float(torch.sqrt(torch.mean(all_abs ** 2)).item()),
        "max_abs_m": float(all_abs.max().item()),
        "false_safe": false_safe,
        "false_danger": false_danger,
    }


def model_score(metrics: dict, false_safe_penalty: float, mae_weight: float) -> float:
    return false_safe_penalty * float(metrics["false_safe"]) + mae_weight * float(metrics["mae_m"])


def train(args: argparse.Namespace) -> Path:
    device = torch.device(args.device if args.device else ("cuda" if torch.cuda.is_available() else "cpu"))
    dataset_dirs = [Path(p).expanduser() for p in args.datasets]
    dataset = RayDataset(dataset_dirs)
    val_size = max(1, int(len(dataset) * args.val_fraction))
    train_size = len(dataset) - val_size
    generator = torch.Generator().manual_seed(args.seed)
    train_dataset, val_dataset = random_split(dataset, [train_size, val_size], generator=generator)
    train_loader = DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True, num_workers=args.num_workers)
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size, shuffle=False, num_workers=args.num_workers)

    model = torch.jit.load(str(args.base_model), map_location=device)
    model.train()
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=args.weight_decay)

    output_dir = args.output_root / args.run_name
    output_dir.mkdir(parents=True, exist_ok=True)
    best_path = output_dir / "ray_pred_mujoco_finetuned_best.pt"
    last_path = output_dir / "ray_pred_mujoco_finetuned_last.pt"
    best_score = float("inf")
    best_metrics = None
    history = []

    for epoch in range(1, args.epochs + 1):
        model.train()
        losses = []
        for depth_log, target_log in train_loader:
            depth_log = depth_log.to(device)
            target_log = target_log.to(device)
            inputs = depth_log.unsqueeze(1).repeat(1, 3, 1, 1)
            pred_log = model(inputs)
            loss = weighted_ray_loss(pred_log, target_log, args.near_weight, args.false_safe_weight)
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), args.grad_clip)
            optimizer.step()
            losses.append(float(loss.item()))

        metrics = eval_model(model, val_loader, device)
        metrics["epoch"] = epoch
        metrics["train_loss"] = float(np.mean(losses)) if losses else float("nan")
        metrics["score"] = model_score(metrics, args.false_safe_score_penalty, args.mae_score_weight)
        history.append(metrics)
        print(
            f"[Finetune] epoch={epoch:03d} loss={metrics['train_loss']:.5f} "
            f"val_mae={metrics['mae_m']:.3f} false_safe={metrics['false_safe']} "
            f"false_danger={metrics['false_danger']} score={metrics['score']:.3f}"
        )
        model_cpu = copy.deepcopy(model).to("cpu")
        torch.jit.save(model_cpu, str(last_path))
        if metrics["score"] < best_score:
            best_score = metrics["score"]
            best_metrics = dict(metrics)
            torch.jit.save(model_cpu, str(best_path))

    manifest = {
        "base_model": str(args.base_model),
        "datasets": [str(p) for p in dataset_dirs],
        "sample_count": len(dataset),
        "train_size": train_size,
        "val_size": val_size,
        "device": str(device),
        "epochs": args.epochs,
        "batch_size": args.batch_size,
        "lr": args.lr,
        "near_weight": args.near_weight,
        "false_safe_weight": args.false_safe_weight,
        "false_safe_score_penalty": args.false_safe_score_penalty,
        "mae_score_weight": args.mae_score_weight,
        "best_model": str(best_path),
        "last_model": str(last_path),
        "best_metrics": best_metrics,
        "history": history,
    }
    with (output_dir / "manifest.json").open("w") as f:
        json.dump(manifest, f, indent=2)
    return best_path


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--datasets", nargs="+", required=True, help="Dataset directories containing .npy files and label.pkl")
    parser.add_argument("--base-model", type=Path, default=DEFAULT_BASE_MODEL)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--run-name", default="mujoco_finetune_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--epochs", type=int, default=20)
    parser.add_argument("--batch-size", type=int, default=128)
    parser.add_argument("--lr", type=float, default=1e-4)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--near-weight", type=float, default=4.0)
    parser.add_argument("--false-safe-weight", type=float, default=8.0)
    parser.add_argument("--false-safe-score-penalty", type=float, default=1.0)
    parser.add_argument("--mae-score-weight", type=float, default=10.0)
    parser.add_argument("--grad-clip", type=float, default=1.0)
    parser.add_argument("--val-fraction", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--num-workers", type=int, default=0)
    parser.add_argument("--device", default="", help="Torch device override, e.g. cpu or cuda")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    best_path = train(args)
    print(f"[Finetune] Best model: {best_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
