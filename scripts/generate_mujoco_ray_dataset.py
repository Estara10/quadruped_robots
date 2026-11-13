#!/usr/bin/env python3
"""Generate MuJoCo depth images and geometric ray labels for Ray-Pred finetuning."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import pickle
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence

import numpy as np

ROOT = Path.home() / "quadruped_robots"
COMPARE_SCRIPT = ROOT / "scripts" / "compare_ray_pred.py"
DEFAULT_OUTPUT_ROOT = ROOT / "ABS" / "training" / "legged_gym" / "legged_gym" / "depth_data" / "mujoco_ray_pred"
DEFAULT_SCENES = ["scene_obstacle.xml", "scene_terrain.xml", "scene_slope_obstacle.xml"]


def load_compare_module():
    spec = importlib.util.spec_from_file_location("compare_ray_pred", COMPARE_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def parse_float_list(value: str) -> List[float]:
    return [float(item.strip()) for item in value.split(",") if item.strip()]


def sample_grid(args: argparse.Namespace) -> List[tuple[float, float, float]]:
    xs = parse_float_list(args.x_values) if args.x_values else np.linspace(args.x_min, args.x_max, args.x_samples).tolist()
    ys = parse_float_list(args.y_values)
    yaws = [math.radians(v) for v in parse_float_list(args.yaw_deg_values)]
    return [(float(x), float(y), float(yaw)) for yaw in yaws for y in ys for x in xs]


def render_depth(mujoco, compare, model, data, renderer, body_id: int, camera_config=None,
                 return_native: bool = False) -> np.ndarray:
    cam = compare.camera_from_body(mujoco, data, body_id, camera_config)
    renderer.update_scene(data, camera=cam)
    if camera_config is not None:
        # MuJoCo's free camera is centered by default. Shift the near-plane
        # frustum so its optical-axis projection lands at the supplied K=(fx,
        # fy, ppx, ppy), using image coordinates measured at pixel centers.
        # MuJoCo 3.2.3 averages both virtual eyes for mono rendering. Updating
        # just camera[0] halves the intended principal-point displacement.
        for gl_camera in renderer.scene.camera:
            near = float(gl_camera.frustum_near)
            gl_camera.frustum_center = (
                ((camera_config.width - 1.0) * 0.5 - camera_config.ppx)
                * near / camera_config.fx
            )
            gl_camera.frustum_top = (camera_config.ppy + 0.5) * near / camera_config.fy
            gl_camera.frustum_bottom = (
                camera_config.ppy + 0.5 - camera_config.height
            ) * near / camera_config.fy
    depth = renderer.render()
    if camera_config is not None and not return_native:
        from realsense_depth_to_abs_rays import downsample_depth_centers

        valid = np.ones(depth.shape, dtype=bool)
        depth, _ = downsample_depth_centers(depth.astype(np.float32), valid)
    return np.clip(depth, compare.RAY2D_MIN_DIST, compare.RAY2D_MAX_DIST).astype(np.float32)


def generate_scene_samples(
    mujoco,
    compare,
    scene: str,
    scene_index: int,
    poses: Sequence[tuple[float, float, float]],
    args: argparse.Namespace,
    output_dir: Path,
    labels: Dict[str, np.ndarray],
    camera_config=None,
) -> Dict[str, int]:
    scene_path = compare.resolve_scene(scene)
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    body_id = compare.find_body_id(mujoco, model)
    renderer = compare.make_depth_renderer(mujoco, model, camera_config)
    base_z = args.base_z if args.base_z is not None else float(data.qpos[2])
    saved = 0
    skipped_clearance = 0

    try:
        for pose_index, (x, y, yaw) in enumerate(poses):
            sample_index = scene_index * len(poses) + pose_index
            sample = compare.PoseSample(
                index=sample_index,
                source="mujoco_dataset",
                x=x,
                y=y,
                z=base_z,
                yaw=yaw,
            )
            compare.apply_sample(mujoco, model, data, sample)
            ray_log = compare.compute_geometric_ray_log(
                mujoco, model, data, body_id, Path(scene).name
            )
            ray_m = np.clip(np.exp2(ray_log), compare.RAY2D_MIN_DIST, compare.RAY2D_MAX_DIST).astype(np.float32)
            min_ray = float(np.min(ray_m))
            if min_ray < args.min_target_clearance:
                skipped_clearance += 1
                continue
            depth = render_depth(
                mujoco, compare, model, data, renderer, body_id, camera_config,
                return_native=args.save_native_depth,
            )
            key = f"scene{scene_index:02d}_pose{pose_index:05d}"
            if args.save_native_depth:
                from realsense_depth_to_abs_rays import downsample_depth_centers

                np.save(output_dir / f"{key}_native.npy", depth)
                depth, _ = downsample_depth_centers(depth, np.ones(depth.shape, dtype=bool))
            np.save(output_dir / f"{key}.npy", depth)
            labels[key] = ray_m
            saved += 1
            if saved % args.log_interval == 0:
                print(f"[Dataset] {scene}: saved={saved} skipped_clearance={skipped_clearance}")
    finally:
        renderer.close()

    return {"saved": saved, "skipped_clearance": skipped_clearance}


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenes", nargs="+", default=DEFAULT_SCENES)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--dataset-name", default="mujoco_go2_ray_pred_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--x-min", type=float, default=0.0)
    parser.add_argument("--x-max", type=float, default=7.0)
    parser.add_argument("--x-samples", type=int, default=40)
    parser.add_argument("--x-values", default="")
    parser.add_argument("--y-values", default="-0.75,-0.5,-0.25,0.0,0.25,0.5,0.75")
    parser.add_argument("--yaw-deg-values", default="-20,-10,0,10,20")
    parser.add_argument("--base-z", type=float, default=None)
    parser.add_argument("--min-target-clearance", type=float, default=0.35)
    parser.add_argument("--log-interval", type=int, default=100)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--camera-position-m", nargs=3, type=float,
                        metavar=("X", "Y", "Z"),
                        help="explicit camera origin in base_link coordinates; provide with both other camera options")
    parser.add_argument("--camera-pitch-down-deg", type=float,
                        help="pitch downward relative to base_link +x / horizontal plane")
    parser.add_argument("--depth-intrinsics", nargs=6, type=float,
                        metavar=("WIDTH", "HEIGHT", "FX", "FY", "PPX", "PPY"),
                        help="native depth image dimensions and pixel intrinsics; render native size then exact center-sample to 160x90")
    parser.add_argument("--save-native-depth", action="store_true",
                        help="also save native-resolution float32 depth before the existing center sampler; requires explicit camera geometry")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    import mujoco

    compare = load_compare_module()
    supplied = (args.camera_position_m is not None,
                args.camera_pitch_down_deg is not None,
                args.depth_intrinsics is not None)
    if any(supplied) and not all(supplied):
        raise ValueError("camera position, pitch, and depth intrinsics must be supplied together")
    camera_config = None
    if all(supplied):
        width_f, height_f, fx, fy, ppx, ppy = args.depth_intrinsics
        if width_f != int(width_f) or height_f != int(height_f):
            raise ValueError("depth image width and height must be integers")
        camera_config = compare.CameraRenderConfig(
            position_body_m=tuple(args.camera_position_m),
            pitch_down_deg=args.camera_pitch_down_deg,
            width=int(width_f), height=int(height_f),
            fx=fx, fy=fy, ppx=ppx, ppy=ppy,
        )
        camera_config.validate()
    if args.save_native_depth and camera_config is None:
        raise ValueError("--save-native-depth requires explicit camera position, pitch, and intrinsics")
    output_dir = args.output_root / args.dataset_name
    if output_dir.exists() and not args.overwrite:
        raise FileExistsError(f"{output_dir} exists; pass --overwrite or choose another --dataset-name")
    output_dir.mkdir(parents=True, exist_ok=True)

    poses = sample_grid(args)
    labels: Dict[str, np.ndarray] = {}
    scene_stats = {}
    print(f"[Dataset] Output: {output_dir}")
    print(f"[Dataset] Scenes: {args.scenes}")
    print(f"[Dataset] Poses per scene before filtering: {len(poses)}")

    for scene_index, scene in enumerate(args.scenes):
        stats = generate_scene_samples(
            mujoco, compare, scene, scene_index, poses, args, output_dir, labels, camera_config
        )
        scene_stats[scene] = stats
        print(f"[Dataset] {scene}: {stats}")

    with (output_dir / "label.pkl").open("wb") as f:
        pickle.dump(labels, f)

    if camera_config is not None:
        manifest_hfov = math.degrees(2.0 * math.atan(camera_config.width / (2.0 * camera_config.fx)))
        manifest_fovy = math.degrees(2.0 * math.atan(camera_config.height / (2.0 * camera_config.fy)))
        manifest_camera_pos = list(camera_config.position_body_m)
    else:
        manifest_hfov = compare.CAMERA_HFOV_DEG
        manifest_fovy = compare.mujoco_fovy_from_hfov(compare.CAMERA_HFOV_DEG)
        manifest_camera_pos = compare.CAMERA_LOCAL_POS.tolist()

    manifest = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "scenes": args.scenes,
        "poses_per_scene_before_filtering": len(poses),
        "sample_count": len(labels),
        "min_target_clearance": args.min_target_clearance,
        "model_input_size": [compare.CAM_W, compare.CAM_H],
        # Preserve the legacy manifest fields for existing default-mode readers.
        "camera_size": [compare.CAM_W, compare.CAM_H],
        "camera_hfov_deg": manifest_hfov,
        "camera_fovy_deg": manifest_fovy,
        "camera_local_pos": manifest_camera_pos,
        "camera_render": ({
            "source_size": [camera_config.width, camera_config.height],
            "camera_shell_center_assumed_optical_origin_base_link_m": list(camera_config.position_body_m),
            "origin_reference_assumption": "housing center used as a proxy; housing-to-optical-center structural offset is unknown",
            "pitch_down_relative_base_x_deg": camera_config.pitch_down_deg,
            "roll_yaw": "assumed zero; not independently measured",
            "intrinsics_fx_fy_ppx_ppy": [camera_config.fx, camera_config.fy,
                                           camera_config.ppx, camera_config.ppy],
            "derived_hfov_deg": math.degrees(2.0 * math.atan(
                camera_config.width / (2.0 * camera_config.fx))),
            "derived_fovy_deg": math.degrees(2.0 * math.atan(
                camera_config.height / (2.0 * camera_config.fy))),
            "principal_point": "off-axis frustum_center/top/bottom are set from ppx/ppy after scene update; source pixels use center coordinates",
            "to_model_input": "native render then exact existing center-pixel sampler to 160x90; no resize or image stretch",
        } if camera_config is not None else {
            "source_size": [compare.CAM_W, compare.CAM_H],
            "camera_shell_center_assumed_optical_origin_base_link_m": compare.CAMERA_LOCAL_POS.tolist(),
            "hfov_deg": compare.CAMERA_HFOV_DEG,
            "fovy_deg": compare.mujoco_fovy_from_hfov(compare.CAMERA_HFOV_DEG),
            "to_model_input": "legacy 160x90 render; existing default behavior",
        }),
        "ray_distance_range_m": [compare.RAY2D_MIN_DIST, compare.RAY2D_MAX_DIST],
        "scene_stats": scene_stats,
        "save_native_depth": bool(args.save_native_depth),
    }
    with (output_dir / "manifest.json").open("w") as f:
        json.dump(manifest, f, indent=2)
    print(f"[Dataset] Saved samples: {len(labels)}")
    print(f"[Dataset] Labels: {output_dir / 'label.pkl'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
