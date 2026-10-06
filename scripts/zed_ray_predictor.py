#!/usr/bin/env python3
"""Real-robot ZED depth camera → ResNet18 → /mujoco_ray2d ray prediction.

Replaces the MuJoCo geometric ray2d on the real Go2.
Adapted from the original ABS publisher_depthimg_linvel.py.

Usage (on Go2):
  python3 scripts/zed_ray_predictor.py

Environment:
  RAY_PRED_MODEL: override model path (default: soft_safety finetuned)
"""

import mmap
import numpy as np
import os
import struct
import sys
import time
from pathlib import Path

HOME = Path.home()
ROOT = HOME / "quadruped_robots"
DEFAULT_MODEL = str(
    ROOT / "logs" / "ray_pred_finetune" / "mujoco_finetune_soft_safety_20260611"
    / "ray_pred_mujoco_finetuned_best.pt"
)
FALLBACK_MODEL = str(
    ROOT / "ABS" / "training" / "legged_gym" / "legged_gym" / "depth_logs"
    / "20260528-143154-resnet18-go2_depth" / "depth_lidar_model_20260528-143154_250.pt"
)

CAM_W, CAM_H = 160, 90
RAY2D_COUNT = 11
RAY2D_MAX_DIST = 6.0
RAY2D_MIN_DIST = 0.1
RAY2D_SHM = "/mujoco_ray2d"


def main():
    import pyzed.sl as sl
    import torch

    model_path = os.environ.get("RAY_PRED_MODEL", DEFAULT_MODEL)
    if not Path(model_path).exists():
        print(f"[ZED] Model not found: {model_path}")
        if Path(FALLBACK_MODEL).exists():
            model_path = FALLBACK_MODEL
            print(f"[ZED] Using fallback: {model_path}")
        else:
            raise FileNotFoundError(f"Neither {model_path} nor {FALLBACK_MODEL} found")

    print(f"[ZED] Loading model: {model_path}")
    model = torch.jit.load(model_path).cpu()
    model.eval()

    # Open /mujoco_ray2d shared memory
    ray2d_size = RAY2D_COUNT * 4
    ray2d_fd = os.open(f"/dev/shm{RAY2D_SHM}", os.O_RDWR | os.O_CREAT)
    os.ftruncate(ray2d_fd, ray2d_size)
    ray2d_buf = mmap.mmap(ray2d_fd, ray2d_size, mmap.MAP_SHARED,
                          mmap.PROT_READ | mmap.PROT_WRITE)
    os.close(ray2d_fd)
    print(f"[ZED] Opened {RAY2D_SHM}")

    # ZED camera init (matches original ABS config)
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    init_params.depth_maximum_distance = RAY2D_MAX_DIST
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ZED] Camera open failed: {status}")
        sys.exit(1)

    runtime_params = sl.RuntimeParameters()
    runtime_params.enable_fill_mode = True
    depth_mat = sl.Mat()
    resolution = sl.Resolution(CAM_W, CAM_H)

    print(f"[ZED] Camera ready, running at {CAM_W}x{CAM_H}")
    frame = 0
    try:
        while True:
            t0 = time.time()
            if zed.grab(runtime_params) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.001)
                continue

            zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH, sl.MEM.CPU, resolution)
            depth = np.array(depth_mat.get_data(), dtype=np.float32)
            depth[np.isinf(depth)] = RAY2D_MAX_DIST
            depth = np.nan_to_num(depth, nan=RAY2D_MAX_DIST)
            depth = np.clip(depth, RAY2D_MIN_DIST, RAY2D_MAX_DIST)

            depth_log = np.log2(depth).astype(np.float32)
            depth_tensor = torch.from_numpy(depth_log).unsqueeze(0).unsqueeze(0).repeat(1, 3, 1, 1)

            with torch.no_grad():
                rays = model(depth_tensor).squeeze(0)
            rays = torch.clamp(rays, np.log2(RAY2D_MIN_DIST), np.log2(RAY2D_MAX_DIST))
            rays_np = rays.numpy().astype(np.float32)

            ray2d_buf.seek(0)
            ray2d_buf.write(rays_np.tobytes())
            ray2d_buf.flush()

            frame += 1
            dt = (time.time() - t0) * 1000
            if frame % 100 == 0:
                rs = " ".join(f"{np.exp2(r):.2f}" for r in rays_np)
                print(f"[ZED] f={frame:5d} rays=[{rs}] dt={dt:.1f}ms")

    except KeyboardInterrupt:
        print(f"\n[ZED] Stopped ({frame} frames)")
    finally:
        ray2d_buf.close()
        zed.close()


if __name__ == "__main__":
    main()
