#!/usr/bin/env python3
"""MuJoCo depth-camera ray predictor writing the versioned ABS ray frame."""

import mmap
import json
import os
import struct
import time
from pathlib import Path

import numpy as np

ROOT = Path.home() / "quadruped_robots"
MUJOCO_DIR = ROOT / "unitree_mujoco"
SCENE = os.environ.get("MUJOCO_SCENE", "scene_obstacle.xml")
SCENE_PATH = MUJOCO_DIR / "unitree_robots" / "go2" / SCENE
MODEL_PATH = Path(os.environ.get(
    "RAY_PRED_MODEL",
    ROOT / "archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/"
         "ray_pred_mujoco_finetuned_best.pt"))

CAM_W, CAM_H = 160, 90
CAMERA_HFOV_DEG = 102.0
CAMERA_LOCAL_POS = np.array([0.0, 0.0, 0.27], dtype=np.float64)
CAMERA_LOOKAHEAD_M = 3.0
RAY_COUNT = 11
RAY_MIN_M, RAY_MAX_M = 0.1, 6.0
FPS = 30
RAY_SHM = "/mujoco_ray2d"
RAY_HEADER_SHM = "/mujoco_ray2d_stamp"
QPOS_SHM = "/mujoco_qpos"
SIM_CLOCK_SHM = "/mujoco_sim_clock"
QPOS_COUNT = 19
HEADER = struct.Struct("<4Q")
SIM_CLOCK = struct.Struct("<4Qd")
RAY_BYTES = RAY_COUNT * 4
QPOS_BYTES = QPOS_COUNT * 8
SIM_CLOCK_MAX_AGE_NS = 100_000_000


def mujoco_fovy_from_hfov(hfov_deg, width=CAM_W, height=CAM_H):
    hfov = np.radians(hfov_deg)
    vfov = 2.0 * np.arctan(np.tan(hfov * 0.5) * (height / width))
    return float(np.degrees(vfov))


def read_qpos(qpos_buf):
    qpos_buf.seek(0)
    first = qpos_buf.read(QPOS_BYTES)
    qpos_buf.seek(0)
    second = qpos_buf.read(QPOS_BYTES)
    if len(first) != QPOS_BYTES or first != second:
        return None
    qpos = np.asarray(struct.unpack("<19d", first), dtype=np.float64)
    if not np.isfinite(qpos).all() or not np.any(qpos):
        return None
    return qpos


def read_sim_clock(clock_buf):
    clock_buf.seek(0)
    first = clock_buf.read(SIM_CLOCK.size)
    clock_buf.seek(0)
    second = clock_buf.read(SIM_CLOCK.size)
    if len(first) != SIM_CLOCK.size or first != second:
        return False
    magic, version, sequence, monotonic_ns, sim_time = SIM_CLOCK.unpack(first)
    age_ns = time.monotonic_ns() - monotonic_ns
    return (magic == 0x414253434C4F434B and version == 2 and sequence > 0 and
            sequence % 2 == 0 and age_ns >= 0 and age_ns <= SIM_CLOCK_MAX_AGE_NS and
            np.isfinite(sim_time))


def write_ray_frame(ray_buf, header_buf, rays):
    header_buf.seek(0)
    magic, version, sequence, _ = HEADER.unpack(header_buf.read(HEADER.size))
    if magic not in (0, 0x415253594143544F) or version not in (0, 2):
        raise RuntimeError("unexpected ray shared-memory header contract")
    if sequence & 1:
        sequence += 1
    struct.pack_into("<Q", header_buf, 16, sequence + 1)
    ray_buf.seek(0)
    ray_buf.write(rays.astype("<f4", copy=False).tobytes())
    struct.pack_into("<Q", header_buf, 0, 0x415253594143544F)
    struct.pack_into("<Q", header_buf, 8, 2)
    published_ns = time.monotonic_ns()
    struct.pack_into("<Q", header_buf, 24, published_ns)
    struct.pack_into("<Q", header_buf, 16, sequence + 2)
    return published_ns, time.monotonic_ns()


def main():
    import torch
    import mujoco

    if not SCENE_PATH.is_file():
        raise FileNotFoundError("MuJoCo scene is missing: {}".format(SCENE_PATH))
    if not MODEL_PATH.is_file():
        raise FileNotFoundError("explicit Ray-Pred model is missing: {}".format(MODEL_PATH))

    device = torch.device(os.environ.get("RAY_PRED_DEVICE", "cpu"))
    if device.type == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was explicitly requested for Ray-Pred but is unavailable")

    model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
    if model.nq != QPOS_COUNT:
        raise RuntimeError("qpos contract mismatch: scene nq={}, expected {}".format(model.nq, QPOS_COUNT))
    data = mujoco.MjData(model)
    model.vis.global_.fovy = mujoco_fovy_from_hfov(CAMERA_HFOV_DEG)
    renderer = mujoco.Renderer(model, CAM_H, CAM_W)
    renderer.enable_depth_rendering()

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    resnet = torch.jit.load(str(MODEL_PATH), map_location=device)
    resnet.eval()
    warmup_start_ns = time.monotonic_ns()
    warmup_input = torch.zeros((1, 3, CAM_H, CAM_W), dtype=torch.float32, device=device)
    with torch.inference_mode():
        for _ in range(3):
            warmup_output = resnet.forward(warmup_input)
        if device.type == "cuda":
            torch.cuda.synchronize(device)
    warmup_ms = (time.monotonic_ns() - warmup_start_ns) / 1e6
    cuda_device_name = torch.cuda.get_device_name(device) if device.type == "cuda" else None

    ray_fd = os.open("/dev/shm" + RAY_SHM, os.O_RDWR)
    header_fd = os.open("/dev/shm" + RAY_HEADER_SHM, os.O_RDWR)
    qpos_fd = os.open("/dev/shm" + QPOS_SHM, os.O_RDONLY)
    clock_fd = os.open("/dev/shm" + SIM_CLOCK_SHM, os.O_RDONLY)
    ray_buf = mmap.mmap(ray_fd, RAY_BYTES, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
    header_buf = mmap.mmap(header_fd, HEADER.size, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
    qpos_buf = mmap.mmap(qpos_fd, QPOS_BYTES, mmap.MAP_SHARED, mmap.PROT_READ)
    clock_buf = mmap.mmap(clock_fd, SIM_CLOCK.size, mmap.MAP_SHARED, mmap.PROT_READ)
    for fd in (ray_fd, header_fd, qpos_fd, clock_fd):
        os.close(fd)

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if body_id < 0:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
    if body_id < 0:
        raise RuntimeError("Go2 base body is absent from the selected scene")

    print("[RayPredictor] scene={} model={} ray_source=prediction header={}".format(
        SCENE_PATH, MODEL_PATH, RAY_HEADER_SHM), flush=True)
    print("[RayPredictorReady] device={} cuda_name={} torch_threads={} torch_interop_threads={} warmup_passes=3 warmup_ms={:.3f} renderer_backend={}".format(
        device, cuda_device_name, torch.get_num_threads(), torch.get_num_interop_threads(),
        warmup_ms, os.environ.get("MUJOCO_GL", "unknown")), flush=True)
    print("[RayPredictor] camera={}x{} hfov={}deg depth=m clip=[{},{}] log2; output=11 log2 rays, order -45deg..+45deg".format(
        CAM_W, CAM_H, CAMERA_HFOV_DEG, RAY_MIN_M, RAY_MAX_M), flush=True)
    frame = 0
    previous_publish_complete_ns = None
    period = 1.0 / FPS
    try:
        while True:
            started_ns = time.monotonic_ns()
            qpos_read_start_ns = started_ns
            qpos = read_qpos(qpos_buf)
            qpos_read_complete_ns = time.monotonic_ns()
            sim_clock_read_start_ns = qpos_read_complete_ns
            sim_clock_valid = read_sim_clock(clock_buf)
            sim_clock_read_complete_ns = time.monotonic_ns()
            if qpos is None or not sim_clock_valid:
                time.sleep(min(period, 0.01))
                continue

            data.qpos[:] = qpos
            mujoco.mj_forward(model, data)
            body_xpos = data.xpos[body_id]
            body_xmat = data.xmat[body_id].reshape(3, 3)
            cam_world = body_xpos + body_xmat @ CAMERA_LOCAL_POS
            look_world = body_xpos + body_xmat @ np.array([CAMERA_LOOKAHEAD_M, 0.0, 0.0])
            diff = look_world - cam_world
            distance = float(np.linalg.norm(diff))
            cam.lookat[:] = look_world
            cam.distance = distance
            cam.azimuth = np.degrees(np.arctan2(diff[1], diff[0]))
            cam.elevation = np.degrees(np.arcsin(diff[2] / max(distance, 0.001)))

            render_start = time.monotonic_ns()
            renderer.update_scene(data, camera=cam)
            depth_m = np.clip(renderer.render(), RAY_MIN_M, RAY_MAX_M)
            render_complete_ns = time.monotonic_ns()
            render_ns = render_complete_ns - render_start
            preprocess_start_ns = render_complete_ns
            depth_log = np.log2(depth_m).astype(np.float32)
            depth_tensor_cpu = torch.from_numpy(depth_log).unsqueeze(0).unsqueeze(0).repeat(1, 3, 1, 1)
            preprocess_complete_ns = time.monotonic_ns()
            transfer_start_ns = preprocess_complete_ns
            depth_tensor = depth_tensor_cpu.to(device)
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            transfer_complete_ns = time.monotonic_ns()
            infer_start = time.monotonic_ns()
            with torch.inference_mode():
                output = resnet.forward(depth_tensor)
            if device.type == "cuda":
                torch.cuda.synchronize(device)
            inference_complete_ns = time.monotonic_ns()
            infer_ns = inference_complete_ns - infer_start
            if not isinstance(output, torch.Tensor) or tuple(output.shape) != (1, RAY_COUNT):
                raise RuntimeError("model output must have shape (1, 11), got {}".format(
                    tuple(output.shape) if isinstance(output, torch.Tensor) else type(output)))
            output_copy_start_ns = time.monotonic_ns()
            rays = output.squeeze(0).detach().cpu().numpy().astype(np.float32)
            output_copy_complete_ns = time.monotonic_ns()
            if not np.isfinite(rays).all():
                raise RuntimeError("model produced non-finite ray values")
            rays = np.clip(rays, np.log2(RAY_MIN_M), np.log2(RAY_MAX_M))
            publish_start_ns = time.monotonic_ns()
            published_ns, publish_complete_ns = write_ray_frame(ray_buf, header_buf, rays)
            print("[RayPredictorFrame] " + json.dumps({
                "frame": frame, "qpos_read_start_monotonic_ns": qpos_read_start_ns,
                "qpos_read_complete_monotonic_ns": qpos_read_complete_ns,
                "sim_clock_read_start_monotonic_ns": sim_clock_read_start_ns,
                "sim_clock_read_complete_monotonic_ns": sim_clock_read_complete_ns,
                "render_start_monotonic_ns": render_start,
                "render_complete_monotonic_ns": render_complete_ns,
                "prediction_complete_monotonic_ns": output_copy_complete_ns,
                "published_monotonic_ns": published_ns,
                "publish_complete_monotonic_ns": publish_complete_ns,
                "loop_interval_ms": ((publish_complete_ns - previous_publish_complete_ns) / 1e6
                                     if previous_publish_complete_ns is not None else None),
                "qpos_read_ms": (qpos_read_complete_ns - qpos_read_start_ns) / 1e6,
                "sim_clock_read_ms": (sim_clock_read_complete_ns - sim_clock_read_start_ns) / 1e6,
                "render_ms": render_ns / 1e6, "inference_ms": infer_ns / 1e6,
                "preprocess_ms": (preprocess_complete_ns - preprocess_start_ns) / 1e6,
                "device_transfer_ms": (transfer_complete_ns - transfer_start_ns) / 1e6,
                "output_copy_ms": (output_copy_complete_ns - output_copy_start_ns) / 1e6,
                "publish_ms": (publish_complete_ns - publish_start_ns) / 1e6,
                "render_to_publish_ms": (published_ns - render_start) / 1e6,
                "prediction_to_publish_complete_ms": (publish_complete_ns - output_copy_complete_ns) / 1e6,
                "rays_log2": rays.tolist()}, separators=(",", ":")), flush=True)
            previous_publish_complete_ns = publish_complete_ns
            frame += 1
            remaining = period - ((time.monotonic_ns() - started_ns) / 1e9)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        print("[RayPredictor] stopped after {} frames".format(frame), flush=True)
    finally:
        renderer.close()
        for buf in (ray_buf, header_buf, qpos_buf, clock_buf):
            buf.close()


if __name__ == "__main__":
    main()
