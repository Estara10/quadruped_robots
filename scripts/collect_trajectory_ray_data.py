#!/usr/bin/env python3
"""Collect depth + geometric-ray trajectory data from a running MuJoCo simulation.

Usage (after starting geometric-ray sim with scene_obstacle):
  source /home/lidio/anaconda3/etc/profile.d/conda.sh && conda activate abs
  MUJOCO_GL=egl python3 scripts/collect_trajectory_ray_data.py \
    --scene scene_obstacle.xml \
    --dataset-name traj_obstacle_$(date +%Y%m%d_%H%M%S) \
    --duration 35
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import mmap
import os
import pickle
import struct
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

ROOT = Path.home() / "quadruped_robots"
COMPARE_SCRIPT = ROOT / "scripts" / "compare_ray_pred.py"
DEFAULT_OUTPUT_ROOT = (
    ROOT / "ABS" / "training" / "legged_gym" / "legged_gym" / "depth_data" / "mujoco_traj"
)
QPOS_SHM = "/dev/shm/mujoco_qpos"
RAY2D_SHM = "/dev/shm/mujoco_ray2d"
RAY2D_COUNT = 11
RAY2D_SIZE = RAY2D_COUNT * 4
QPOS_COUNT = 19
QPOS_SIZE = QPOS_COUNT * 8


def load_compare_module():
    spec = importlib.util.spec_from_file_location("compare_ray_pred", COMPARE_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def open_shm(path: str, size: int) -> mmap.mmap:
    fd = os.open(path, os.O_RDONLY)
    buf = mmap.mmap(fd, size, mmap.MAP_SHARED, mmap.PROT_READ)
    os.close(fd)
    return buf


def read_qpos(buf: mmap.mmap) -> Optional[np.ndarray]:
    buf.seek(0)
    raw = buf.read(QPOS_SIZE)
    vals = np.array(struct.unpack(f"{QPOS_COUNT}d", raw), dtype=np.float64)
    if np.sum(np.abs(vals)) < 1e-6:
        return None
    return vals


def read_ray_m(buf: mmap.mmap) -> np.ndarray:
    buf.seek(0)
    raw = buf.read(RAY2D_SIZE)
    ray_log = np.array(struct.unpack(f"{RAY2D_COUNT}f", raw), dtype=np.float32)
    ray_m = np.exp2(ray_log)
    ray_m = np.clip(ray_m, 0.1, 6.0)
    return ray_m.astype(np.float32)


def collect(
    args: argparse.Namespace,
    mujoco,
    compare,
    model,
    data,
    renderer,
    body_id: int,
    qpos_buf: mmap.mmap,
    ray_buf: mmap.mmap,
    output_dir: Path,
) -> Dict[str, int]:
    labels: Dict[str, np.ndarray] = {}
    saved = 0
    skipped_stale = 0
    t_start = time.time()
    sample_interval = 1.0 / max(args.hz, 1)

    print(f"[TrajCollect] Starting collection, duration={args.duration}s, hz={args.hz}")
    last_qpos = None
    last_sample_t = 0.0

    while time.time() - t_start < args.duration + 1.0:
        qpos = read_qpos(qpos_buf)
        if qpos is None:
            skipped_stale += 1
            time.sleep(0.05)
            continue

        elapsed = time.time() - t_start
        if elapsed < args.skip_first_s:
            time.sleep(0.05)
            continue

        if elapsed - last_sample_t < sample_interval:
            time.sleep(0.02)
            continue

        if qpos is last_qpos:
            skipped_stale += 1
            time.sleep(0.05)
            continue

        nq = min(QPOS_COUNT, model.nq)
        data.qpos[:nq] = qpos[:nq]
        mujoco.mj_forward(model, data)

        cam = compare.camera_from_body(mujoco, data, body_id)
        renderer.update_scene(data, camera=cam)
        depth = renderer.render()
        depth = np.clip(depth, compare.RAY2D_MIN_DIST, compare.RAY2D_MAX_DIST).astype(np.float32)

        ray_m = read_ray_m(ray_buf)

        key = f"traj_{saved:06d}"
        np.save(output_dir / f"{key}.npy", depth)
        labels[key] = ray_m
        saved += 1
        last_qpos = qpos
        last_sample_t = elapsed

        if saved % 50 == 0:
            print(f"[TrajCollect] saved={saved}, elapsed={elapsed:.1f}s, skipped_stale={skipped_stale}")

        time.sleep(0.02)

    print(f"[TrajCollect] Done: saved={saved}, skipped_stale={skipped_stale}")
    return labels


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default="scene_obstacle.xml")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--dataset-name", default="traj_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--duration", type=float, default=35.0)
    parser.add_argument("--hz", type=float, default=5.0)
    parser.add_argument("--skip-first-s", type=float, default=7.0)
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    import mujoco

    compare = load_compare_module()

    scene_path = compare.resolve_scene(args.scene)
    if not scene_path.exists():
        raise FileNotFoundError(scene_path)

    output_dir = args.output_root / args.dataset_name
    if output_dir.exists() and not args.overwrite:
        raise FileExistsError(f"{output_dir} exists; pass --overwrite")
    output_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    body_id = compare.find_body_id(mujoco, model)
    renderer = compare.make_depth_renderer(mujoco, model)

    qpos_buf = open_shm(QPOS_SHM, QPOS_SIZE)
    ray_buf = open_shm(RAY2D_SHM, RAY2D_SIZE)

    print(f"[TrajCollect] Scene: {scene_path}")
    print(f"[TrajCollect] Output: {output_dir}")
    print(f"[TrajCollect] Waiting for qpos data...")

    try:
        labels = collect(args, mujoco, compare, model, data, renderer, body_id, qpos_buf, ray_buf, output_dir)
    finally:
        renderer.close()
        qpos_buf.close()
        ray_buf.close()

    if not labels:
        print("[TrajCollect] No samples collected")
        return 1

    with (output_dir / "label.pkl").open("wb") as f:
        pickle.dump(labels, f)

    manifest = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "scene": args.scene,
        "sample_count": len(labels),
        "duration_s": args.duration,
        "hz": args.hz,
        "skip_first_s": args.skip_first_s,
        "camera_size": [compare.CAM_W, compare.CAM_H],
        "camera_hfov_deg": compare.CAMERA_HFOV_DEG,
        "camera_local_pos": compare.CAMERA_LOCAL_POS.tolist(),
    }
    with (output_dir / "manifest.json").open("w") as f:
        json.dump(manifest, f, indent=2)

    print(f"[TrajCollect] Saved {len(labels)} samples to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
