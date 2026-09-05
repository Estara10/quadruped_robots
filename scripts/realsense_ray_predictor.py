#!/usr/bin/env python3
"""Real-robot Intel RealSense D435i depth camera → ResNet18 → /mujoco_ray2d ray prediction.

Replaces the MuJoCo geometric ray2d on the real Go2.
Mirrors scripts/zed_ray_predictor.py but uses pyrealsense2 instead of ZED SDK.

Usage (on Go2):
  python3 scripts/realsense_ray_predictor.py

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

# Camera config — must match Isaac Gym training setup.
# In training: camera pos = (0, 0, 0.27) in body frame, pointing forward.
# D435i HFOV ≈ 87° (stereo) / 69° (depth), but depth is from left IR — use depth HFOV.
CAM_POS = (0.0, 0.0, 0.27)       # body-frame camera position (x, y, z)
CAM_W, CAM_H = 160, 90             # inference resolution (from training)
# RealSense depth stream: we capture at higher res then downsample.
RS_CAPTURE_W, RS_CAPTURE_H = 640, 480
RS_FPS = 30

RAY2D_COUNT = 11
RAY2D_MAX_DIST = 6.0
RAY2D_MIN_DIST = 0.1
RAY2D_SHM = "/mujoco_ray2d"

# TODO: confirm with actual D435i depth HFOV (typ. ~87° for stereo depth, ~69° for left IR)
# Set this once you can read the depth intrinsics from the camera.
CAM_HFOV_DEG = 69.0


def resize_depth(depth: np.ndarray, target_w: int, target_h: int) -> np.ndarray:
    """Downsample depth map to target resolution with nearest-neighbour."""
    # Simple strided subsample — fast, no interpolation artifacts on depth edges.
    h, w = depth.shape
    if h == target_h and w == target_w:
        return depth
    h_step = max(1, h // target_h)
    w_step = max(1, w // target_w)
    return depth[::h_step, ::w_step][:target_h, :target_w]


def main():
    import pyrealsense2 as rs
    import torch

    model_path = os.environ.get("RAY_PRED_MODEL", DEFAULT_MODEL)
    if not Path(model_path).exists():
        print(f"[RS] Model not found: {model_path}")
        if Path(FALLBACK_MODEL).exists():
            model_path = FALLBACK_MODEL
            print(f"[RS] Using fallback: {model_path}")
        else:
            raise FileNotFoundError(f"Neither {model_path} nor {FALLBACK_MODEL} found")

    print(f"[RS] Loading model: {model_path}")
    model = torch.jit.load(model_path).cpu()
    model.eval()

    # Open /mujoco_ray2d shared memory
    ray2d_size = RAY2D_COUNT * 4
    ray2d_fd = os.open(f"/dev/shm{RAY2D_SHM}", os.O_RDWR | os.O_CREAT)
    os.ftruncate(ray2d_fd, ray2d_size)
    ray2d_buf = mmap.mmap(ray2d_fd, ray2d_size, mmap.MAP_SHARED,
                          mmap.PROT_READ | mmap.PROT_WRITE)
    os.close(ray2d_fd)
    print(f"[RS] Opened {RAY2D_SHM}")

    # RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, RS_CAPTURE_W, RS_CAPTURE_H, rs.format.z16, RS_FPS)
    profile = pipeline.start(config)

    # Get depth scale (convert raw uint16 -> metres)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"[RS] depth_scale = {depth_scale:.4f} m/unit")

    # Log intrinsics (help verify HFOV)
    intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    hfov_real = 2.0 * np.arctan2(intr.width * 0.5, intr.fx) * 180.0 / np.pi
    print(f"[RS] depth intrinsics: {intr.width}x{intr.height} fx={intr.fx:.1f} fy={intr.fy:.1f}")
    print(f"[RS] derived HFOV = {hfov_real:.1f} deg (config assumes {CAM_HFOV_DEG:.1f})")

    # Align depth to colour (not strictly needed, but gives less missing data at edges)
    align = rs.align(rs.stream.color)
    # Fallback: if colour stream not available, just use raw depth.
    has_color = False
    try:
        config.enable_stream(rs.stream.color, RS_CAPTURE_W, RS_CAPTURE_H, rs.format.bgr8, RS_FPS)
        has_color = True
    except Exception:
        print("[RS] No colour stream; using raw depth only")

    print(f"[RS] Camera ready, running at {CAM_W}x{CAM_H}")
    frame = 0
    try:
        while True:
            t0 = time.time()
            frameset = pipeline.wait_for_frames()

            if has_color:
                frameset = align.process(frameset)

            depth_frame = frameset.get_depth_frame()
            if not depth_frame:
                time.sleep(0.001)
                continue

            depth_raw = np.asanyarray(depth_frame.get_data(), dtype=np.uint16)
            depth_m = depth_raw.astype(np.float32) * depth_scale

            # Downsample to training resolution
            depth_m = resize_depth(depth_m, CAM_W, CAM_H)

            # Clamp and log2
            depth_m = np.clip(depth_m, RAY2D_MIN_DIST, RAY2D_MAX_DIST)
            # Treat zero as max range (no return)
            depth_m[depth_m <= 0] = RAY2D_MAX_DIST

            depth_log = np.log2(depth_m).astype(np.float32)
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
                rs_str = " ".join(f"{np.exp2(r):.2f}" for r in rays_np)
                print(f"[RS] f={frame:5d} rays=[{rs_str}] dt={dt:.1f}ms")

    except KeyboardInterrupt:
        print(f"\n[RS] Stopped ({frame} frames)")
    finally:
        ray2d_buf.close()
        pipeline.stop()


if __name__ == "__main__":
    main()
