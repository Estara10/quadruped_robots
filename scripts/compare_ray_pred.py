#!/usr/bin/env python3
"""Offline Ray-Pred vs MuJoCo geometric ray comparison."""

from __future__ import annotations

import argparse
import csv
import json
import math
import mmap
import os
import struct
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

ROOT = Path.home() / "quadruped_robots"
MUJOCO_DIR = ROOT / "unitree_mujoco"
SCENE_DIR = MUJOCO_DIR / "unitree_robots" / "go2"
DEFAULT_MODEL = (
    ROOT
    / "ABS"
    / "training"
    / "legged_gym"
    / "legged_gym"
    / "depth_logs"
    / "20260528-143154-resnet18-go2_depth"
    / "depth_lidar_model_20260528-143154_250.pt"
)
DEFAULT_OUTPUT_ROOT = ROOT / "logs" / "ray_pred_compare"

CAM_W = 160
CAM_H = 90
CAMERA_HFOV_DEG = 102.0
CAMERA_LOCAL_POS = np.array([0.0, 0.0, 0.27], dtype=np.float64)
CAMERA_LOOKAHEAD_M = 3.0
RAY2D_COUNT = 11
RAY2D_MAX_DIST = 6.0
RAY2D_MIN_DIST = 0.1
RAY2D_THETA_START = -math.pi / 4.0
RAY2D_THETA_STEP = math.pi / 20.0
RAY2D_X0 = -0.05
RAY2D_Y0 = 0.0
QPOS_COUNT = 19
QPOS_FORMAT = f"{QPOS_COUNT}d"
QPOS_SIZE = struct.calcsize(QPOS_FORMAT)


@dataclass
class PoseSample:
    index: int
    source: str
    x: float
    y: float
    z: float
    yaw: float
    qpos: Optional[np.ndarray] = None


@dataclass
class RayResult:
    sample: PoseSample
    target_log: np.ndarray
    pred_log: np.ndarray
    target_m: np.ndarray
    pred_m: np.ndarray
    depth_m: Optional[np.ndarray]
    mean_abs_log: float
    rmse_log: float
    max_abs_log: float
    mean_abs_m: float
    rmse_m: float
    max_abs_m: float
    false_safe_count: int
    false_danger_count: int
    mean_abs_log_flipped: float
    rmse_log_flipped: float
    max_abs_log_flipped: float
    mean_abs_m_flipped: float
    rmse_m_flipped: float
    max_abs_m_flipped: float
    false_safe_count_flipped: int
    false_danger_count_flipped: int
    inference_ms: float
    render_ms: float


def parse_float_list(value: str) -> List[float]:
    return [float(item.strip()) for item in value.split(",") if item.strip()]


def resolve_scene(scene: str) -> Path:
    path = Path(scene).expanduser()
    if path.is_absolute():
        return path
    return SCENE_DIR / scene


def find_body_id(mujoco, model) -> int:
    for name in ("base_link", "torso_link"):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id >= 0:
            return body_id
    return 1


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    half = yaw * 0.5
    return math.cos(half), 0.0, 0.0, math.sin(half)


def apply_sample(mujoco, model, data, sample: PoseSample) -> None:
    if sample.qpos is not None:
        nq = min(model.nq, len(sample.qpos))
        data.qpos[:nq] = sample.qpos[:nq]
    else:
        data.qpos[0] = sample.x
        data.qpos[1] = sample.y
        data.qpos[2] = sample.z
        qw, qx, qy, qz = yaw_to_quat(sample.yaw)
        data.qpos[3] = qw
        data.qpos[4] = qx
        data.qpos[5] = qy
        data.qpos[6] = qz
    mujoco.mj_forward(model, data)


def grid_samples(args: argparse.Namespace, initial_z: float) -> List[PoseSample]:
    if args.x_values:
        xs = parse_float_list(args.x_values)
    else:
        xs = np.linspace(args.x_min, args.x_max, args.samples).tolist()
    ys = parse_float_list(args.y_values)
    yaws = [math.radians(v) for v in parse_float_list(args.yaw_deg_values)]
    z = args.base_z if args.base_z is not None else initial_z

    samples: List[PoseSample] = []
    index = 0
    for yaw in yaws:
        for y in ys:
            for x in xs:
                samples.append(PoseSample(index=index, source="grid", x=x, y=y, z=z, yaw=yaw))
                index += 1
    return samples


def qpos_shm_samples(args: argparse.Namespace) -> List[PoseSample]:
    samples: List[PoseSample] = []
    fd = os.open(str(args.qpos_shm), os.O_RDONLY)
    try:
        buf = mmap.mmap(fd, QPOS_SIZE, mmap.MAP_SHARED, mmap.PROT_READ)
        try:
            for index in range(args.samples):
                buf.seek(0)
                raw = buf.read(QPOS_SIZE)
                qpos = np.array(struct.unpack(QPOS_FORMAT, raw), dtype=np.float64)
                if np.sum(np.abs(qpos)) > 1e-6:
                    samples.append(
                        PoseSample(
                            index=index,
                            source="qpos_shm",
                            x=float(qpos[0]),
                            y=float(qpos[1]),
                            z=float(qpos[2]),
                            yaw=float("nan"),
                            qpos=qpos,
                        )
                    )
                if index + 1 < args.samples:
                    time.sleep(args.interval)
        finally:
            buf.close()
    finally:
        os.close(fd)
    return samples


def obstacle_geom_ids(mujoco, model) -> Iterable[int]:
    blocked_types = {
        int(mujoco.mjtGeom.mjGEOM_PLANE),
        int(mujoco.mjtGeom.mjGEOM_HFIELD),
        int(mujoco.mjtGeom.mjGEOM_MESH),
    }
    allowed_types = {
        int(mujoco.mjtGeom.mjGEOM_BOX),
        int(mujoco.mjtGeom.mjGEOM_CYLINDER),
        int(mujoco.mjtGeom.mjGEOM_SPHERE),
        int(mujoco.mjtGeom.mjGEOM_CAPSULE),
        int(mujoco.mjtGeom.mjGEOM_ELLIPSOID),
    }
    for geom_id in range(model.ngeom):
        group = int(model.geom_group[geom_id])
        if group in (2, 3):
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
        if name == "floor":
            continue
        body_id = int(model.geom_bodyid[geom_id])
        if body_id > 0 and float(model.body_mass[body_id]) > 0.0:
            continue
        geom_type = int(model.geom_type[geom_id])
        if geom_type in blocked_types or geom_type not in allowed_types:
            continue
        yield geom_id


def ray_box_distance(origin: np.ndarray, direction: np.ndarray, center: np.ndarray, xmat: np.ndarray, size: np.ndarray) -> float:
    dx = origin[0] - center[0]
    dy = origin[1] - center[1]
    local_x = xmat[0] * dx + xmat[3] * dy
    local_y = xmat[1] * dx + xmat[4] * dy
    dir_x = xmat[0] * direction[0] + xmat[3] * direction[1]
    dir_y = xmat[1] * direction[0] + xmat[4] * direction[1]
    t_min = -math.inf
    t_max = math.inf

    for pos, ray_dir, half_size in ((local_x, dir_x, size[0]), (local_y, dir_y, size[1])):
        if abs(ray_dir) < 1e-9:
            if pos < -half_size or pos > half_size:
                return math.inf
            continue
        t1 = (-half_size - pos) / ray_dir
        t2 = (half_size - pos) / ray_dir
        if t1 > t2:
            t1, t2 = t2, t1
        t_min = max(t_min, t1)
        t_max = min(t_max, t2)

    if t_max >= max(0.0, t_min):
        return t_min if t_min >= 0.0 else t_max
    return math.inf


def ray_round_distance(
    mujoco,
    model,
    geom_id: int,
    ray_x0: float,
    ray_y0: float,
    ctheta: float,
    stheta: float,
    center: np.ndarray,
    size: np.ndarray,
) -> float:
    geom_type = int(model.geom_type[geom_id])
    radius = float(size[0])
    if geom_type == int(mujoco.mjtGeom.mjGEOM_ELLIPSOID):
        radius = max(float(size[0]), float(size[1]))
    elif geom_type not in (
        int(mujoco.mjtGeom.mjGEOM_CYLINDER),
        int(mujoco.mjtGeom.mjGEOM_SPHERE),
        int(mujoco.mjtGeom.mjGEOM_CAPSULE),
    ):
        radius = max(float(size[0]), float(size[1]))

    gx = float(center[0])
    gy = float(center[1])
    distance_to_line = abs(stheta * gx - ctheta * gy - stheta * ray_x0 + ctheta * ray_y0)
    if distance_to_line >= radius:
        return math.inf

    center_distance_sq = (gx - ray_x0) * (gx - ray_x0) + (gy - ray_y0) * (gy - ray_y0)
    d_0p = math.sqrt(max(0.0, center_distance_sq - distance_to_line * distance_to_line))
    semi_arc = math.sqrt(max(0.0, radius * radius - distance_to_line * distance_to_line))
    ray_distance = d_0p - semi_arc
    check_dir = ctheta * (gx - ray_x0) + stheta * (gy - ray_y0)
    if check_dir <= 0:
        return math.inf
    return ray_distance


def compute_geometric_ray_log(mujoco, model, data, body_id: int) -> np.ndarray:
    body_xpos = data.xpos[body_id]
    body_xmat = data.xmat[body_id]
    body_yaw = math.atan2(float(body_xmat[3]), float(body_xmat[0]))
    ray_x0 = float(body_xpos[0]) + RAY2D_X0 * math.cos(body_yaw) - RAY2D_Y0 * math.sin(body_yaw)
    ray_y0 = float(body_xpos[1]) + RAY2D_X0 * math.sin(body_yaw) + RAY2D_Y0 * math.cos(body_yaw)
    origin = np.array([ray_x0, ray_y0], dtype=np.float64)

    rays = np.full(RAY2D_COUNT, math.log2(RAY2D_MAX_DIST), dtype=np.float32)
    geom_ids = list(obstacle_geom_ids(mujoco, model))
    for ray_index in range(RAY2D_COUNT):
        theta = RAY2D_THETA_START + ray_index * RAY2D_THETA_STEP
        world_theta = theta + body_yaw
        ctheta = math.cos(world_theta)
        stheta = math.sin(world_theta)
        direction = np.array([ctheta, stheta], dtype=np.float64)
        best_dist = RAY2D_MAX_DIST

        for geom_id in geom_ids:
            geom_type = int(model.geom_type[geom_id])
            center = data.geom_xpos[geom_id]
            size = model.geom_size[geom_id]
            if geom_type == int(mujoco.mjtGeom.mjGEOM_BOX):
                distance = ray_box_distance(origin, direction, center, data.geom_xmat[geom_id], size)
            else:
                distance = ray_round_distance(mujoco, model, geom_id, ray_x0, ray_y0, ctheta, stheta, center, size)
            if not math.isfinite(distance):
                continue
            distance = max(distance, RAY2D_MIN_DIST)
            if distance < best_dist:
                best_dist = float(distance)
        rays[ray_index] = math.log2(best_dist)
    return rays


def mujoco_fovy_from_hfov(hfov_deg: float, width: int = CAM_W, height: int = CAM_H) -> float:
    hfov = math.radians(hfov_deg)
    vfov = 2.0 * math.atan(math.tan(hfov * 0.5) * (height / width))
    return math.degrees(vfov)


def make_depth_renderer(mujoco, model):
    model.vis.global_.fovy = mujoco_fovy_from_hfov(CAMERA_HFOV_DEG)
    renderer = mujoco.Renderer(model, CAM_H, CAM_W)
    renderer.enable_depth_rendering()
    return renderer


def camera_from_body(mujoco, data, body_id: int):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    body_xpos = data.xpos[body_id]
    body_xmat = data.xmat[body_id].reshape(3, 3)
    cam_world = body_xpos + body_xmat @ CAMERA_LOCAL_POS
    look_world = body_xpos + body_xmat @ np.array([CAMERA_LOOKAHEAD_M, 0.0, 0.0])
    diff = look_world - cam_world
    distance = float(np.linalg.norm(diff))
    cam.lookat[:] = look_world
    cam.distance = distance
    cam.azimuth = math.degrees(math.atan2(float(diff[1]), float(diff[0])))
    cam.elevation = math.degrees(math.asin(float(diff[2]) / max(distance, 0.001)))
    return cam


def predict_ray_log(
    torch,
    resnet,
    renderer,
    data,
    body_id: int,
    device: str,
    keep_depth: bool,
) -> Tuple[np.ndarray, Optional[np.ndarray], float, float]:
    cam = camera_from_body(__import__("mujoco"), data, body_id)
    t0 = time.time()
    renderer.update_scene(data, camera=cam)
    depth = renderer.render()
    render_ms = (time.time() - t0) * 1000.0

    depth = np.clip(depth, RAY2D_MIN_DIST, RAY2D_MAX_DIST)
    depth_log = np.log2(depth).astype(np.float32)
    depth_tensor = torch.from_numpy(depth_log).unsqueeze(0).unsqueeze(0).repeat(1, 3, 1, 1).to(device)

    t1 = time.time()
    with torch.no_grad():
        rays = resnet.forward(depth_tensor).squeeze(0)
    inference_ms = (time.time() - t1) * 1000.0
    depth_m = depth.astype(np.float32).copy() if keep_depth else None
    return rays.detach().cpu().numpy().astype(np.float32), depth_m, render_ms, inference_ms


def error_metrics(pred_log: np.ndarray, target_log: np.ndarray, args: argparse.Namespace) -> Dict[str, object]:
    pred_m = np.clip(np.exp2(pred_log), RAY2D_MIN_DIST, RAY2D_MAX_DIST)
    target_m = np.clip(np.exp2(target_log), RAY2D_MIN_DIST, RAY2D_MAX_DIST)
    err_log = pred_log - target_log
    err_m = pred_m - target_m
    false_safe = np.logical_and(target_m <= args.near_distance, err_m >= args.false_safe_margin)
    false_danger = np.logical_and(target_m >= args.clear_distance, -err_m >= args.false_danger_margin)
    return {
        "pred_m": pred_m,
        "mean_abs_log": float(np.mean(np.abs(err_log))),
        "rmse_log": float(np.sqrt(np.mean(err_log * err_log))),
        "max_abs_log": float(np.max(np.abs(err_log))),
        "mean_abs_m": float(np.mean(np.abs(err_m))),
        "rmse_m": float(np.sqrt(np.mean(err_m * err_m))),
        "max_abs_m": float(np.max(np.abs(err_m))),
        "false_safe_count": int(np.count_nonzero(false_safe)),
        "false_danger_count": int(np.count_nonzero(false_danger)),
    }


def evaluate_sample(
    mujoco,
    torch,
    model,
    data,
    renderer,
    resnet,
    body_id: int,
    sample: PoseSample,
    args: argparse.Namespace,
) -> RayResult:
    apply_sample(mujoco, model, data, sample)
    target_log = compute_geometric_ray_log(mujoco, model, data, body_id)
    pred_log, depth_m, render_ms, inference_ms = predict_ray_log(
        torch, resnet, renderer, data, body_id, args.device, args.save_worst > 0
    )
    target_m = np.clip(np.exp2(target_log), RAY2D_MIN_DIST, RAY2D_MAX_DIST)
    metrics = error_metrics(pred_log, target_log, args)
    flipped_metrics = error_metrics(pred_log[::-1].copy(), target_log, args)
    return RayResult(
        sample=sample,
        target_log=target_log,
        pred_log=pred_log,
        target_m=target_m,
        pred_m=metrics["pred_m"],
        depth_m=depth_m,
        mean_abs_log=metrics["mean_abs_log"],
        rmse_log=metrics["rmse_log"],
        max_abs_log=metrics["max_abs_log"],
        mean_abs_m=metrics["mean_abs_m"],
        rmse_m=metrics["rmse_m"],
        max_abs_m=metrics["max_abs_m"],
        false_safe_count=metrics["false_safe_count"],
        false_danger_count=metrics["false_danger_count"],
        mean_abs_log_flipped=flipped_metrics["mean_abs_log"],
        rmse_log_flipped=flipped_metrics["rmse_log"],
        max_abs_log_flipped=flipped_metrics["max_abs_log"],
        mean_abs_m_flipped=flipped_metrics["mean_abs_m"],
        rmse_m_flipped=flipped_metrics["rmse_m"],
        max_abs_m_flipped=flipped_metrics["max_abs_m"],
        false_safe_count_flipped=flipped_metrics["false_safe_count"],
        false_danger_count_flipped=flipped_metrics["false_danger_count"],
        inference_ms=inference_ms,
        render_ms=render_ms,
    )


def result_to_row(result: RayResult) -> Dict[str, object]:
    row: Dict[str, object] = {
        "sample_index": result.sample.index,
        "source": result.sample.source,
        "x": result.sample.x,
        "y": result.sample.y,
        "z": result.sample.z,
        "yaw_rad": result.sample.yaw,
        "mean_abs_log2": result.mean_abs_log,
        "rmse_log2": result.rmse_log,
        "max_abs_log2": result.max_abs_log,
        "mean_abs_m": result.mean_abs_m,
        "rmse_m": result.rmse_m,
        "max_abs_m": result.max_abs_m,
        "min_target_m": float(np.min(result.target_m)),
        "min_pred_m": float(np.min(result.pred_m)),
        "false_safe_count": result.false_safe_count,
        "false_danger_count": result.false_danger_count,
        "mean_abs_m_flipped": result.mean_abs_m_flipped,
        "rmse_m_flipped": result.rmse_m_flipped,
        "max_abs_m_flipped": result.max_abs_m_flipped,
        "false_safe_count_flipped": result.false_safe_count_flipped,
        "false_danger_count_flipped": result.false_danger_count_flipped,
        "render_ms": result.render_ms,
        "inference_ms": result.inference_ms,
    }
    for ray_index in range(RAY2D_COUNT):
        row[f"target_log2_{ray_index:02d}"] = float(result.target_log[ray_index])
        row[f"pred_log2_{ray_index:02d}"] = float(result.pred_log[ray_index])
        row[f"target_m_{ray_index:02d}"] = float(result.target_m[ray_index])
        row[f"pred_m_{ray_index:02d}"] = float(result.pred_m[ray_index])
        row[f"err_m_{ray_index:02d}"] = float(result.pred_m[ray_index] - result.target_m[ray_index])
    return row


def write_csv(results: Sequence[RayResult], csv_path: Path) -> None:
    rows = [result_to_row(result) for result in results]
    fieldnames = list(rows[0].keys())
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def aggregate(results: Sequence[RayResult]) -> Dict[str, object]:
    target_m = np.stack([result.target_m for result in results], axis=0)
    pred_m = np.stack([result.pred_m for result in results], axis=0)
    pred_m_flipped = np.stack([np.clip(np.exp2(result.pred_log[::-1]), RAY2D_MIN_DIST, RAY2D_MAX_DIST) for result in results], axis=0)
    target_log = np.stack([result.target_log for result in results], axis=0)
    pred_log = np.stack([result.pred_log for result in results], axis=0)
    pred_log_flipped = np.stack([result.pred_log[::-1] for result in results], axis=0)
    err_m = pred_m - target_m
    err_m_flipped = pred_m_flipped - target_m
    err_log = pred_log - target_log
    err_log_flipped = pred_log_flipped - target_log
    return {
        "samples": len(results),
        "mean_abs_log2": float(np.mean(np.abs(err_log))),
        "rmse_log2": float(np.sqrt(np.mean(err_log * err_log))),
        "max_abs_log2": float(np.max(np.abs(err_log))),
        "mean_abs_m": float(np.mean(np.abs(err_m))),
        "rmse_m": float(np.sqrt(np.mean(err_m * err_m))),
        "max_abs_m": float(np.max(np.abs(err_m))),
        "false_safe_total": int(sum(result.false_safe_count for result in results)),
        "false_danger_total": int(sum(result.false_danger_count for result in results)),
        "mean_abs_log2_flipped": float(np.mean(np.abs(err_log_flipped))),
        "rmse_log2_flipped": float(np.sqrt(np.mean(err_log_flipped * err_log_flipped))),
        "max_abs_log2_flipped": float(np.max(np.abs(err_log_flipped))),
        "mean_abs_m_flipped": float(np.mean(np.abs(err_m_flipped))),
        "rmse_m_flipped": float(np.sqrt(np.mean(err_m_flipped * err_m_flipped))),
        "max_abs_m_flipped": float(np.max(np.abs(err_m_flipped))),
        "false_safe_total_flipped": int(sum(result.false_safe_count_flipped for result in results)),
        "false_danger_total_flipped": int(sum(result.false_danger_count_flipped for result in results)),
        "mean_render_ms": float(np.mean([result.render_ms for result in results])),
        "mean_inference_ms": float(np.mean([result.inference_ms for result in results])),
        "per_ray_mean_abs_m": np.mean(np.abs(err_m), axis=0).tolist(),
        "per_ray_max_abs_m": np.max(np.abs(err_m), axis=0).tolist(),
        "per_ray_bias_m": np.mean(err_m, axis=0).tolist(),
        "per_ray_mean_target_m": np.mean(target_m, axis=0).tolist(),
        "per_ray_mean_pred_m": np.mean(pred_m, axis=0).tolist(),
    }


def write_markdown(summary: Dict[str, object], args: argparse.Namespace, md_path: Path, csv_path: Path) -> None:
    lines = [
        "# Ray-Pred Offline Comparison",
        "",
        f"Generated: {datetime.now().isoformat(timespec='seconds')}",
        f"Scene: `{args.scene_path}`",
        f"Model: `{args.model}`",
        f"CSV: `{csv_path}`",
        "",
        "## Overall",
        "",
        f"- Samples: **{summary['samples']}**",
        f"- Mean abs error: **{summary['mean_abs_m']:.3f} m** / **{summary['mean_abs_log2']:.3f} log2**",
        f"- RMSE: **{summary['rmse_m']:.3f} m** / **{summary['rmse_log2']:.3f} log2**",
        f"- Max abs error: **{summary['max_abs_m']:.3f} m** / **{summary['max_abs_log2']:.3f} log2**",
        f"- False-safe rays: **{summary['false_safe_total']}**",
        f"- False-danger rays: **{summary['false_danger_total']}**",
        f"- Flipped mean abs error: **{summary['mean_abs_m_flipped']:.3f} m** / **{summary['mean_abs_log2_flipped']:.3f} log2**",
        f"- Flipped false-safe/danger rays: **{summary['false_safe_total_flipped']} / {summary['false_danger_total_flipped']}**",
        f"- Mean render/inference: **{summary['mean_render_ms']:.1f} ms / {summary['mean_inference_ms']:.1f} ms**",
        "",
        "## Per-Ray Metrics",
        "",
        "| Ray | Mean target(m) | Mean pred(m) | Bias(m) | MAE(m) | Max abs(m) |",
        "|---:|---:|---:|---:|---:|---:|",
    ]
    per_ray_target = summary["per_ray_mean_target_m"]
    per_ray_pred = summary["per_ray_mean_pred_m"]
    per_ray_bias = summary["per_ray_bias_m"]
    per_ray_mae = summary["per_ray_mean_abs_m"]
    per_ray_max = summary["per_ray_max_abs_m"]
    for ray_index in range(RAY2D_COUNT):
        lines.append(
            f"| {ray_index} | {per_ray_target[ray_index]:.3f} | {per_ray_pred[ray_index]:.3f} | "
            f"{per_ray_bias[ray_index]:.3f} | {per_ray_mae[ray_index]:.3f} | {per_ray_max[ray_index]:.3f} |"
        )
    lines.extend(
        [
            "",
            "## Interpretation",
            "",
            f"- False-safe: target <= {args.near_distance:.2f}m and prediction is >= target + {args.false_safe_margin:.2f}m.",
            f"- False-danger: target >= {args.clear_distance:.2f}m and prediction is <= target - {args.false_danger_margin:.2f}m.",
            "- Flipped metrics compare the same prediction after reversing ray order; a large improvement indicates a likely left-right ray-order mismatch.",
            "- This script never writes `/mujoco_ray2d`; it only compares Ray-Pred output with an in-process geometric target.",
        ]
    )
    md_path.write_text("\n".join(lines) + "\n")


def write_worst_visualizations(results: Sequence[RayResult], output_dir: Path, count: int) -> List[str]:
    if count <= 0:
        return []
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return []

    viz_dir = output_dir / "worst_samples"
    viz_dir.mkdir(parents=True, exist_ok=True)
    saved: List[str] = []
    worst_results = sorted(results, key=lambda result: result.mean_abs_m, reverse=True)[:count]
    ray_indices = np.arange(RAY2D_COUNT)
    for rank, result in enumerate(worst_results, start=1):
        prefix = f"rank_{rank:02d}_sample_{result.sample.index:03d}"
        if result.depth_m is not None:
            depth_path = viz_dir / f"{prefix}_depth.png"
            plt.figure(figsize=(8, 4.5))
            plt.imshow(result.depth_m, cmap="viridis", vmin=RAY2D_MIN_DIST, vmax=RAY2D_MAX_DIST)
            plt.colorbar(label="depth(m)")
            plt.title(
                f"sample {result.sample.index} depth, x={result.sample.x:.2f}, "
                f"y={result.sample.y:.2f}, yaw={result.sample.yaw:.2f}"
            )
            plt.tight_layout()
            plt.savefig(depth_path, dpi=150)
            plt.close()
            saved.append(str(depth_path))

        rays_path = viz_dir / f"{prefix}_rays.png"
        plt.figure(figsize=(8, 4.5))
        plt.plot(ray_indices, result.target_m, marker="o", label="geometric target")
        plt.plot(ray_indices, result.pred_m, marker="o", label="ray-pred")
        plt.plot(ray_indices, np.clip(np.exp2(result.pred_log[::-1]), RAY2D_MIN_DIST, RAY2D_MAX_DIST), marker="o", label="ray-pred flipped")
        plt.ylim(0.0, RAY2D_MAX_DIST + 0.25)
        plt.xlabel("ray index (-45° to +45°)")
        plt.ylabel("distance(m)")
        plt.title(
            f"sample {result.sample.index}: MAE={result.mean_abs_m:.2f}m, "
            f"flip={result.mean_abs_m_flipped:.2f}m"
        )
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        plt.savefig(rays_path, dpi=150)
        plt.close()
        saved.append(str(rays_path))
    return saved


def write_manifest(summary: Dict[str, object], args: argparse.Namespace, output_dir: Path, visualizations: Sequence[str]) -> None:
    manifest = {
        "scene": str(args.scene_path),
        "model": str(args.model),
        "source": args.source,
        "samples": args.samples,
        "device": args.device,
        "camera_size": [CAM_W, CAM_H],
        "camera_hfov_deg": CAMERA_HFOV_DEG,
        "camera_fovy_deg": mujoco_fovy_from_hfov(CAMERA_HFOV_DEG),
        "camera_local_pos": CAMERA_LOCAL_POS.tolist(),
        "ray_count": RAY2D_COUNT,
        "ray_distance_range_m": [RAY2D_MIN_DIST, RAY2D_MAX_DIST],
        "thresholds": {
            "near_distance": args.near_distance,
            "clear_distance": args.clear_distance,
            "false_safe_margin": args.false_safe_margin,
            "false_danger_margin": args.false_danger_margin,
            "min_target_clearance": args.min_target_clearance,
        },
        "visualizations": list(visualizations),
        "summary": summary,
    }
    with (output_dir / "manifest.json").open("w") as f:
        json.dump(manifest, f, indent=2)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", default="scene_obstacle.xml", help="Scene XML filename under go2/, or absolute path")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL, help="TorchScript Ray-Pred model path")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--session-name", default=datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--source", choices=("grid", "qpos-shm"), default="grid")
    parser.add_argument("--samples", type=int, default=15, help="Grid x samples, or qpos-shm reads")
    parser.add_argument("--x-min", type=float, default=0.0)
    parser.add_argument("--x-max", type=float, default=7.0)
    parser.add_argument("--x-values", default="", help="Comma-separated x positions; overrides x-min/x-max/samples")
    parser.add_argument("--y-values", default="0.0", help="Comma-separated y positions")
    parser.add_argument("--yaw-deg-values", default="0.0", help="Comma-separated yaw angles in degrees")
    parser.add_argument("--base-z", type=float, default=None, help="Override base z for grid samples")
    parser.add_argument("--qpos-shm", type=Path, default=Path("/dev/shm/mujoco_qpos"))
    parser.add_argument("--interval", type=float, default=0.1, help="Seconds between qpos-shm reads")
    parser.add_argument("--device", default="cpu", help="Torch device, e.g. cpu or cuda")
    parser.add_argument("--near-distance", type=float, default=1.5)
    parser.add_argument("--clear-distance", type=float, default=3.0)
    parser.add_argument("--false-safe-margin", type=float, default=0.5)
    parser.add_argument("--false-danger-margin", type=float, default=0.5)
    parser.add_argument("--save-worst", type=int, default=3, help="Save depth/ray plots for N worst samples; 0 disables plots")
    parser.add_argument("--min-target-clearance", type=float, default=0.0, help="Drop samples with min geometric ray below this distance")
    args = parser.parse_args(argv)
    args.scene_path = resolve_scene(args.scene)
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    import mujoco
    import torch

    if not args.scene_path.exists():
        raise FileNotFoundError(args.scene_path)
    if not args.model.exists():
        raise FileNotFoundError(args.model)

    model = mujoco.MjModel.from_xml_path(str(args.scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    body_id = find_body_id(mujoco, model)
    renderer = make_depth_renderer(mujoco, model)

    print(f"[RayCompare] Scene: {args.scene_path}")
    print(f"[RayCompare] Model: {args.model}")
    print("[RayCompare] Safety: not opening or writing /dev/shm/mujoco_ray2d")
    resnet = torch.jit.load(str(args.model), map_location=args.device)
    resnet.eval()

    if args.source == "qpos-shm":
        samples = qpos_shm_samples(args)
    else:
        samples = grid_samples(args, float(data.qpos[2]))
    if not samples:
        raise RuntimeError("No valid samples collected")

    output_dir = args.output_root / args.session_name
    output_dir.mkdir(parents=True, exist_ok=True)
    results: List[RayResult] = []
    try:
        for sample in samples:
            result = evaluate_sample(mujoco, torch, model, data, renderer, resnet, body_id, sample, args)
            if float(np.min(result.target_m)) < args.min_target_clearance:
                print(
                    f"[RayCompare] sample={sample.index:03d} skipped: "
                    f"min_target={float(np.min(result.target_m)):.3f}m < {args.min_target_clearance:.3f}m"
                )
                continue
            results.append(result)
            print(
                f"[RayCompare] sample={sample.index:03d} "
                f"mae={result.mean_abs_m:.3f}m max={result.max_abs_m:.3f}m "
                f"false_safe={result.false_safe_count} false_danger={result.false_danger_count}"
            )
    finally:
        renderer.close()

    if not results:
        raise RuntimeError("No samples left after min-target-clearance filtering")

    csv_path = output_dir / "ray_pred_compare.csv"
    md_path = output_dir / "ray_pred_compare.md"
    write_csv(results, csv_path)
    summary = aggregate(results)
    visualizations = write_worst_visualizations(results, output_dir, args.save_worst)
    write_markdown(summary, args, md_path, csv_path)
    write_manifest(summary, args, output_dir, visualizations)
    print(f"[RayCompare] CSV: {csv_path}")
    print(f"[RayCompare] Report: {md_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
