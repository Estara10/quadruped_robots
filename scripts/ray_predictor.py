#!/usr/bin/env python3
"""MuJoCo depth camera + ResNet18 ray prediction.

Paper's perception pipeline for simulation:
  1. Render depth from robot-mounted camera (160×90)
  2. ResNet18 → 11 log2 ray distances
  3. Write to /mujoco_ray2d shared memory

Usage:
  source /home/lidio/anaconda3/etc/profile.d/conda.sh && conda activate abs
  MUJOCO_SCENE=scene.xml python scripts/ray_predictor.py
"""

import os
import sys
import time
import struct
import mmap
import ctypes
import numpy as np
from pathlib import Path

# ── Paths ─────────────────────────────────────────────────────
HOME = Path.home()
MUJOCO_DIR = HOME / "quadruped_robots" / "unitree_mujoco"
ROBOT = "go2"
SCENE = os.environ.get("MUJOCO_SCENE", "scene.xml")
SCENE_PATH = str(MUJOCO_DIR / "unitree_robots" / ROBOT / SCENE)

DEFAULT_MODEL_PATH = str(HOME / "quadruped_robots" / "logs" / "ray_pred_finetune" /
    "mujoco_finetune_soft_safety_20260611" / "ray_pred_mujoco_finetuned_best.pt")
FALLBACK_MODEL_PATH = str(HOME / "quadruped_robots" / "ABS" / "training" / "legged_gym" /
    "legged_gym" / "depth_logs" / "20260528-143154-resnet18-go2_depth" /
    "depth_lidar_model_20260528-143154_250.pt")
MODEL_PATH = os.environ.get("RAY_PRED_MODEL", DEFAULT_MODEL_PATH)

# ── Parameters ─────────────────────────────────────────────────
CAM_W, CAM_H = 160, 90          # matches ZED publisher resolution
CAMERA_HFOV_DEG = 102.0          # matches Isaac Gym depth_cam.hfov
CAMERA_LOCAL_POS = np.array([0.0, 0.0, 0.27], dtype=np.float64)
CAMERA_LOOKAHEAD_M = 3.0
RAY2D_COUNT = 11
RAY2D_MAX_DIST = 6.0
RAY2D_MIN_DIST = 0.1
RAY2D_SHM = "/mujoco_ray2d"
QPOS_SHM = "/mujoco_qpos"       # 19 doubles from C++ bridge (full qpos)
QPOS_COUNT = 19                 # 7 free + 12 joints
FPS = 30                        # rendering rate


def mujoco_fovy_from_hfov(hfov_deg: float, width: int = CAM_W, height: int = CAM_H) -> float:
    hfov = np.radians(hfov_deg)
    vfov = 2.0 * np.arctan(np.tan(hfov * 0.5) * (height / width))
    return float(np.degrees(vfov))


def main():
    import mujoco
    import torch

    print(f"[RayPredictor] Scene: {SCENE_PATH}")
    if not Path(MODEL_PATH).exists() and Path(FALLBACK_MODEL_PATH).exists():
        print(f"[RayPredictor] Model not found, fallback to: {FALLBACK_MODEL_PATH}")
        model_path = FALLBACK_MODEL_PATH
    else:
        model_path = MODEL_PATH
    model = mujoco.MjModel.from_xml_path(SCENE_PATH)
    data = mujoco.MjData(model)

    # Offscreen renderer with depth enabled
    model.vis.global_.fovy = mujoco_fovy_from_hfov(CAMERA_HFOV_DEG)
    renderer = mujoco.Renderer(model, CAM_H, CAM_W)
    renderer.enable_depth_rendering()

    # Free camera positioned relative to robot each frame
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE

    # Load ResNet18 TorchScript model
    print(f"[RayPredictor] Loading model: {model_path}")
    print(f"[RayPredictor] Writing Ray-Pred rays to {RAY2D_SHM}")
    resnet = torch.jit.load(model_path)
    resnet.eval()

    # Open / create ray2d shared memory
    ray2d_size = RAY2D_COUNT * 4  # 11 floats
    ray2d_fd = os.open(f"/dev/shm{RAY2D_SHM}", os.O_RDWR)
    ray2d_buf = mmap.mmap(ray2d_fd, ray2d_size, mmap.MAP_SHARED,
                          mmap.PROT_READ | mmap.PROT_WRITE)
    os.close(ray2d_fd)
    print(f"[RayPredictor] Connected to {RAY2D_SHM}")

    # Try to open qpos shared memory (written by C++ bridge)
    qpos_buf = None
    try:
        qpos_size = QPOS_COUNT * 8
        qpos_fd = os.open(f"/dev/shm{QPOS_SHM}", os.O_RDONLY)
        qpos_buf = mmap.mmap(qpos_fd, qpos_size, mmap.MAP_SHARED, mmap.PROT_READ)
        os.close(qpos_fd)
        print(f"[RayPredictor] Connected to {QPOS_SHM}")
    except FileNotFoundError:
        print(f"[RayPredictor] {QPOS_SHM} not found — using default pose")

    # Find base_link body id for camera positioning
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if body_id < 0:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
    if body_id < 0:
        body_id = 1  # first non-world body
    print(f"[RayPredictor] base_link body id = {body_id}")

    print(f"[RayPredictor] Running at {FPS} Hz...")
    frame = 0
    period = 1.0 / FPS

    try:
        while True:
            t0 = time.time()

            # Sync qpos from C++ simulation if available
            if qpos_buf is not None:
                qpos_buf.seek(0)
                raw = qpos_buf.read(QPOS_COUNT * 8)
                vals = struct.unpack(f'{QPOS_COUNT}d', raw)
                # Only sync if data appears valid (not all zeros)
                if sum(abs(v) for v in vals) > 1e-6:
                    nq = min(QPOS_COUNT, model.nq)
                    for i in range(nq):
                        data.qpos[i] = vals[i]
                    mujoco.mj_forward(model, data)

            # Position camera at front of robot body
            body_xpos = data.xpos[body_id]
            body_xmat = data.xmat[body_id].reshape(3, 3) if hasattr(data.xmat[body_id], 'reshape') else \
                np.array([data.xmat[body_id*9+0], data.xmat[body_id*9+1], data.xmat[body_id*9+2],
                          data.xmat[body_id*9+3], data.xmat[body_id*9+4], data.xmat[body_id*9+5],
                          data.xmat[body_id*9+6], data.xmat[body_id*9+7], data.xmat[body_id*9+8]]).reshape(3, 3)

            # Camera matches Isaac Gym depth_cam: body frame (0, 0, 0.27), hfov=102°
            cam_world = body_xpos + body_xmat @ CAMERA_LOCAL_POS
            look_world = body_xpos + body_xmat @ np.array([CAMERA_LOOKAHEAD_M, 0.0, 0.0])

            # Compute camera parameters for MjvCamera
            diff = look_world - cam_world
            dist = np.linalg.norm(diff)
            cam.lookat[:] = look_world
            cam.distance = dist
            cam.azimuth = np.degrees(np.arctan2(diff[1], diff[0]))
            cam.elevation = np.degrees(np.arcsin(diff[2] / max(dist, 0.001)))

            # Render depth from robot camera
            renderer.update_scene(data, camera=cam)
            depth = renderer.render()  # (H, W) float32, meters

            # Log2 transform (matches training: np.log2(image), min=0.1m, max=6.0m)
            depth = np.clip(depth, RAY2D_MIN_DIST, RAY2D_MAX_DIST)
            depth_log = np.log2(depth).astype(np.float32)

            # ResNet18: 1×3×90×160
            depth_tensor = torch.from_numpy(depth_log)
            depth_tensor = depth_tensor.unsqueeze(0).unsqueeze(0)  # (1,1,90,160)
            depth_tensor = depth_tensor.repeat(1, 3, 1, 1)         # (1,3,90,160)

            with torch.no_grad():
                rays = resnet.forward(depth_tensor).squeeze(0)  # (11,)
            rays = torch.clamp(rays, np.log2(RAY2D_MIN_DIST), np.log2(RAY2D_MAX_DIST))
            rays_np = rays.numpy().astype(np.float32)

            # Write log2 ray distances to shared memory
            ray2d_buf.seek(0)
            ray2d_buf.write(rays_np.tobytes())
            ray2d_buf.flush()

            if frame % 30 == 0:
                rs = " ".join(f"{r:.2f}" for r in rays_np)
                dt = (time.time() - t0) * 1000
                print(f"[RayPredictor] f={frame:4d} rays=[{rs}] dt={dt:.1f}ms")

            frame += 1
            elapsed = time.time() - t0
            if elapsed < period:
                time.sleep(period - elapsed)

    except KeyboardInterrupt:
        print(f"\n[RayPredictor] Stopped ({frame} frames)")
    finally:
        ray2d_buf.close()


if __name__ == "__main__":
    main()
