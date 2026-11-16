#!/usr/bin/env python3
"""Prepare a bounded, split-by-layout D435i ray-adaptation dataset (offline).

Reuses the existing renderer, geometric label function, center sampler and
librealsense 2.58.4 source-equivalent hole fill. It does not train, touch the
network, connect to a device, or write shared memory.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import pickle
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import mujoco  # noqa: E402

import compare_ray_pred as compare  # noqa: E402
import generate_mujoco_ray_dataset as generation  # noqa: E402
from analyze_d435i_new_camera_hole_candidates import librealsense_2584_mode1_source  # noqa: E402
from realsense_depth_to_abs_rays import downsample_depth_centers  # noqa: E402

CAMERA = compare.CameraRenderConfig(
    position_body_m=(.305, 0., .140),
    pitch_down_deg=4.0,
    width=480, height=270,
    fx=244.134, fy=244.134, ppx=245.186, ppy=134.22,
)
PITCH_RANGE_DEG = (3., 5.)
BASE_Z_RANGE_M = (.35, .36)
RAY_ANGLES = [-45. + 9. * i for i in range(11)]
POSE_X_RANGE = (-.8, 3.5)
POSE_Y_RANGE = (-1., 1.)
POSE_YAW_RANGE = (-.5, .5)
BIN_NEAR = (.45, 1.5)
BIN_MID = (1.5, 3.0)
BIN_FAR = (3.0, 5.999)
MIN_CLEARANCE_M = .45
MAX_CAP_BEAMS = {  # reject poses whose view is mostly unexplained 6 m caps
    "near": 5, "mid": 6, "far": 6,
}

MASK_GROUPS = {
    "train": [
        "logs/real_go2/20261113_d435i_remounted_cabinet/capture",
        "logs/real_go2/20261113_remount_adaptation_capture/capture",
        "logs/real_go2/20261113_d435i_existing_mount_standing_cabinet/capture",
        "logs/real_go2/20261113_d435i_1m_distance_capture/capture",
    ],
    "validation": ["logs/real_go2/20261113_d435i_80cm_distance_capture/capture"],
    "test": [
        "logs/real_go2/20261113_d435i_2m_distance_capture/capture/data_20261113_143447",
        "logs/real_go2/20261112_d435i_native_fill_field/capture",
    ],
}


def box(name: str, x: float, y: float, sx: float, sy: float, z: float = .6, sz: float = .6) -> str:
    return (f'<geom name="{name}" type="box" pos="{x:.4f} {y:.4f} {z:.4f}" '
            f'size="{sx:.4f} {sy:.4f} {sz:.4f}" rgba=".62 .62 .62 1"/>')


def cyl(name: str, x: float, y: float, radius: float, half_height: float = .6, z: float = .6) -> str:
    return (f'<geom name="{name}" type="cylinder" pos="{x:.4f} {y:.4f} {z:.4f}" '
            f'size="{radius:.4f} {half_height:.4f}" rgba=".55 .55 .55 1"/>')


def wall(name: str, x: float, half_y: float = 4.0) -> str:
    return (f'<geom name="{name}" type="box" pos="{x:.4f} 0 1" '
            f'size=".05 {half_y:.3f} 1" rgba=".5 .5 .5 1"/>')


def random_obstacles(seed: int, wall_x: float | None) -> list[str]:
    rng = np.random.default_rng(seed)
    parts = [wall("backdrop", wall_x)] if wall_x is not None else []
    for j in range(7):
        x, y = rng.uniform(1.2, 5.2), rng.uniform(-2.1, 2.1)
        sx, sy = rng.uniform(.12, .4), rng.uniform(.12, .4)
        if j % 2:
            parts.append(box(f"box{j}", x, y, sx, sy))
        else:
            parts.append(cyl(f"cyl{j}", x, y, sx))
    return parts


def build_layouts() -> list[dict]:
    """Whole layouts own a split; quota = fraction of near/mid/far pose classes."""
    return [
        {"split": "train", "name": "train_near_center", "count": 105, "quota": (.55, .30, .15), "parts": [
            wall("backdrop", 5.2), box("near_center", .95, .10, .18, .25),
            box("left_near", 1.35, .75, .15, .20), box("right_mid", 2.6, -.9, .25, .30),
            box("left_far", 3.8, 1.2, .30, .30), cyl("right_cyl", 3.2, -1.4, .25)]},
        {"split": "train", "name": "train_near_left", "count": 105, "quota": (.55, .30, .15), "parts": [
            wall("backdrop", 5.4), box("left_near", 1.0, .55, .18, .22),
            box("left_mid", 1.9, 1.0, .20, .25), cyl("center", 1.6, -.15, .18),
            box("right_far", 3.4, -.95, .30, .30), cyl("right_far_cyl", 4.2, -.4, .28)]},
        {"split": "train", "name": "train_near_right", "count": 105, "quota": (.55, .30, .15), "parts": [
            wall("backdrop", 5.4), box("right_near", 1.0, -.55, .18, .22),
            box("right_mid", 1.9, -1.0, .20, .25), cyl("center", 1.6, .15, .18),
            box("left_far", 3.4, .95, .30, .30), cyl("left_far_cyl", 4.2, .4, .28)]},
        {"split": "train", "name": "train_occlusion_pair", "count": 105, "quota": (.50, .30, .20), "parts": [
            wall("backdrop", 5.8), box("front", 1.0, 0., .15, .25), box("behind", 2.0, 0., .15, .25),
            box("left", 2.4, .9, .15, .25), box("right", 1.9, -.8, .20, .20),
            cyl("cyl", 3.5, .4, .30)]},
        {"split": "train", "name": "train_corridor", "count": 105, "quota": (.55, .30, .15), "parts": [
            '<geom name="side_left" type="box" pos="2.0 2.6 1" size="2.4 .05 1" rgba=".5 .5 .5 1"/>',
            '<geom name="side_right" type="box" pos="2.0 -2.6 1" size="2.4 .05 1" rgba=".5 .5 .5 1"/>',
            '<geom name="end_wall" type="box" pos="5.0 0 1" size=".05 3.5 1" rgba=".5 .5 .5 1"/>',
            box("center_near", 1.6, .15, .18, .22), box("right_mid", 2.9, -.2, .20, .25),
            box("left_far", 4.0, .3, .25, .30), cyl("left_cyl", 2.3, .9, .25)]},
        {"split": "train", "name": "train_open_mixed", "count": 105, "quota": (.30, .35, .35), "parts": [
            wall("backdrop", 5.6), box("a", 2.2, -.6, .20, .25), box("b", 3.6, .9, .25, .25),
            cyl("c", 4.6, -.4, .30), box("d", 3.0, -1.3, .25, .30)]},
        {"split": "train", "name": "train_random_a", "count": 105, "quota": (.50, .30, .20), "parts":
            random_obstacles(20261115, 5.2)},
        {"split": "train", "name": "train_random_b", "count": 105, "quota": (.45, .35, .20), "parts":
            random_obstacles(20261116, 5.8)},
        {"split": "validation", "name": "validation_mixed", "count": 90, "quota": (.45, .35, .20), "parts": [
            wall("backdrop", 5.4), box("left_near", 1.1, .6, .18, .22),
            box("center", 1.6, -.15, .20, .25), box("right_mid", 2.4, .9, .20, .25),
            cyl("right_cyl", 2.9, -.8, .25), box("far", 4.0, .3, .30, .30)]},
        {"split": "validation", "name": "validation_open", "count": 90, "quota": (.30, .35, .35), "parts": [
            wall("backdrop", 5.0), box("a", 2.5, .7, .20, .25), box("b", 3.4, -.8, .25, .30),
            cyl("c", 4.2, .4, .30), box("d", 4.5, -.2, .20, .20)]},
        {"split": "test", "name": "test_mixed", "count": 90, "quota": (.45, .35, .20), "parts": [
            wall("backdrop", 5.6), box("right_near", .9, -.25, .18, .22),
            box("left_mid", 1.4, .7, .15, .20), box("front", 1.8, .1, .15, .22),
            box("behind", 2.6, .1, .15, .22), cyl("right_cyl", 3.3, -.9, .28)]},
        {"split": "test", "name": "test_open", "count": 90, "quota": (.30, .35, .35), "parts": [
            wall("backdrop", 5.9), box("a", 2.8, .8, .20, .25), box("b", 3.9, -.9, .25, .30),
            cyl("c", 4.8, .2, .30), box("d", 4.4, -.3, .20, .20)]},
    ]


DIAGNOSTIC_TARGETS = [0.25, 0.35]
CHECK_TARGETS = [(0., 1.0), (18., 1.2), (-18., 1.4), (45., 2.0), (-45., 2.2)]


def scene_xml(parts: list[str]) -> str:
    return (
        '<mujoco model="d435i_ray_adaptation"><compiler angle="radian"/>'
        '<visual><global offwidth="480" offheight="270"/></visual>'
        '<worldbody><geom name="floor" type="plane" size="20 20 .1" rgba=".25 .25 .25 1"/>'
        + "".join(parts)
        + '<body name="base_link" pos="0 0 .355"><freejoint/>'
          '<inertial pos="0 0 0" mass="1" diaginertia=".01 .01 .01"/></body></worldbody></mujoco>'
    )


def target_xml(angle_deg: float, distance: float) -> str:
    """Finite box whose front face meets the given ABS ray at distance."""
    angle = math.radians(angle_deg)
    half_depth, half_width = .08, .12
    cx = -.05 + (distance + half_depth) * math.cos(angle)
    cy = (distance + half_depth) * math.sin(angle)
    return (f'<geom name="target" type="box" pos="{cx:.9f} {cy:.9f} .70" '
            f'euler="0 0 {angle:.12f}" size="{half_depth} {half_width} .70" '
            'rgba=".65 .65 .65 1"/>')


def projected_pixel(point_world, body_pos, body_rot, config: compare.CameraRenderConfig):
    cam = body_pos + body_rot @ np.asarray(config.position_body_m)
    delta = body_rot.T @ (point_world - cam)
    pitch = math.radians(config.pitch_down_deg)
    optical = np.asarray([
        -delta[1],
        -math.sin(pitch) * delta[0] - math.cos(pitch) * delta[2],
        math.cos(pitch) * delta[0] - math.sin(pitch) * delta[2],
    ])
    if optical[2] <= 0:
        return math.nan, math.nan, float(optical[2])
    return (config.fx * optical[0] / optical[2] + config.ppx,
            config.fy * optical[1] / optical[2] + config.ppy,
            float(optical[2]))


def classify(label_m: np.ndarray) -> str | None:
    near = int(np.count_nonzero((label_m >= BIN_NEAR[0]) & (label_m < BIN_NEAR[1])))
    mid = int(np.count_nonzero((label_m >= BIN_MID[0]) & (label_m < BIN_MID[1])))
    far = int(np.count_nonzero((label_m >= BIN_FAR[0]) & (label_m < BIN_FAR[1])))
    cap = int(np.count_nonzero(label_m >= 5.999))
    for name, ok in (
        ("near", near >= 2),
        ("mid", near < 2 and mid >= 2),
        ("far", near < 2 and mid < 2 and far >= 2),
    ):
        if ok and cap <= MAX_CAP_BEAMS[name]:
            return name
    return None


def quota_counts(total: int, quota: tuple[float, float, float]) -> dict[str, int]:
    near = int(round(total * quota[0]))
    mid = int(round(total * quota[1]))
    return {"near": near, "mid": mid, "far": total - near - mid}


def inside_obstacle(model, data, body_id: int, scene_filename: str) -> bool:
    point = data.xpos[body_id, :2]
    inside = False
    for gid in compare.obstacle_geom_ids(mujoco, model, scene_filename):
        delta = point - data.geom_xpos[gid, :2]
        if int(model.geom_type[gid]) == int(mujoco.mjtGeom.mjGEOM_BOX):
            inside |= bool(np.all(np.abs(delta) <= model.geom_size[gid, :2] + .2))
        else:
            inside |= bool(np.linalg.norm(delta) <= model.geom_size[gid, 0] + .2)
    return inside


def set_candidate_frustum(renderer, config: compare.CameraRenderConfig) -> None:
    # MuJoCo 3.2.3 mono rendering averages both virtual eyes; set both.
    for gl_camera in renderer.scene.camera:
        near = float(gl_camera.frustum_near)
        gl_camera.frustum_center = ((config.width - 1.) * .5 - config.ppx) * near / config.fx
        gl_camera.frustum_top = (config.ppy + .5) * near / config.fy
        gl_camera.frustum_bottom = (config.ppy + .5 - config.height) * near / config.fy


def render_native_checked(model, data, renderer, body_id: int, config) -> np.ndarray:
    """Render native depth; retry once if the first frame comes back empty."""
    for attempt in range(2):
        native = generation.render_depth(mujoco, compare, model, data, renderer, body_id,
                                         config, return_native=True)
        if float((native < .1005).mean()) <= .95:
            return native
        if attempt == 0:
            print("[prepare] empty render detected; retrying once", flush=True)
    raise RuntimeError("render returned an empty frame twice; refusing to save it")


def warm_up(model, data, depth_renderer, seg_renderer) -> None:
    data.qpos[:3] = 0.
    data.qpos[2] = .355
    data.qpos[3:7] = (1., 0., 0., 0.)
    mujoco.mj_forward(model, data)
    depth_renderer.update_scene(data, camera=compare.camera_from_body(
        mujoco, data, compare.find_body_id(mujoco, model), CAMERA))
    set_candidate_frustum(depth_renderer, CAMERA)
    depth_renderer.render()
    seg_renderer.update_scene(data, camera=compare.camera_from_body(
        mujoco, data, compare.find_body_id(mujoco, model), CAMERA))
    set_candidate_frustum(seg_renderer, CAMERA)
    seg_renderer.render()


def visibility_rows(model, data, body_id, scene_name, key, split, pose, config,
                     label_m, clean_input, seg_renderer):
    seg_renderer.update_scene(data, camera=compare.camera_from_body(
        mujoco, data, body_id, config))
    set_candidate_frustum(seg_renderer, config)
    seg = seg_renderer.render()
    obs_ids = list(compare.obstacle_geom_ids(mujoco, model, Path(scene_name).name))
    visible_geom = {gid: int(np.count_nonzero((seg[:, :, 0] == gid) &
                                              (seg[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))))
                    for gid in obs_ids}
    body_pos = data.xpos[body_id].copy()
    body_rot = data.xmat[body_id].reshape(3, 3).copy()
    origin = body_pos + body_rot @ np.asarray([compare.RAY2D_X0, compare.RAY2D_Y0, 0.])
    rows = []
    for ray_index, angle_deg in enumerate(RAY_ANGLES):
        angle = math.radians(angle_deg)
        direction = body_rot @ np.asarray([math.cos(angle), math.sin(angle), 0.])
        gid = np.asarray([-1], dtype=np.int32)
        hit_distance = float(mujoco.mj_ray(model, data, origin, direction, None, 1, body_id, gid))
        if hit_distance < 0 or hit_distance > 6.:
            target_m, hit_name, u, v, zcam = 6., "range_cap", math.nan, math.nan, math.nan
            inside, sampled_depth, sample_u, sample_v = False, math.nan, -1, -1
            object_visible, visible_pixel_count, seg_at_hit = False, 0, -1
        else:
            target_m = max(.1, hit_distance)
            geom_id = int(gid[0])
            hit = origin + direction * hit_distance
            u, v, zcam = projected_pixel(hit, body_pos, body_rot, config)
            inside = bool(zcam > 0 and -.5 <= u < config.width - .5 and -.5 <= v < config.height - .5)
            hit_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom{geom_id}"
            visible_pixel_count = visible_geom.get(geom_id, 0)
            object_visible = visible_pixel_count > 0
            if inside:
                model_x = int(np.clip(np.rint((u - 1.) / 3.), 0, 159))
                model_y = int(np.clip(np.rint((v - 1.) / 3.), 0, 89))
                sample_u, sample_v = 3 * model_x + 1, 3 * model_y + 1
                sampled_depth = float(clean_input[model_y, model_x])
                px, py = int(np.clip(np.rint(u), 0, config.width - 1)), int(np.clip(np.rint(v), 0, config.height - 1))
                seg_at_hit = int(seg[py, px, 0])
            else:
                sampled_depth, sample_u, sample_v, seg_at_hit = math.nan, -1, -1, -1
        if abs(target_m - float(label_m[ray_index])) > 1e-4:
            raise RuntimeError(f"label mismatch {scene_name} {key} ray {angle_deg}: "
                               f"{target_m} vs {label_m[ray_index]}")
        rows.append({
            "split": split, "scene": scene_name, "sample": key,
            "pose_x_m": pose.x, "pose_y_m": pose.y, "pose_z_m": pose.z, "pose_yaw_deg": math.degrees(pose.yaw),
            "pitch_down_deg": config.pitch_down_deg,
            "ray_index": ray_index, "ray_angle_deg": angle_deg,
            "label_m": float(label_m[ray_index]), "hit_geom": hit_name,
            "label_hit_u_px": float(u), "label_hit_v_px": float(v),
            "label_hitpoint_in_image": inside,
            "obstacle_has_visible_pixels_in_render": object_visible,
            "visible_geometry_pixels": visible_pixel_count,
            "segmentation_geom_at_hitpoint_pixel": seg_at_hit,
            "nearest_center_sampler_source_u_px": sample_u,
            "nearest_center_sampler_source_v_px": sample_v,
            "model_input_depth_at_hitpoint_m": sampled_depth,
            "projected_hit_optical_z_m": zcam,
        })
    return rows


def save_pair(destination: Path, key: str, split: str, masks, mask_counter: dict,
              native, label_m, labels: dict, records: dict, pose, scene_name):
    raw = np.rint(native * 1000.).clip(0, 65535).astype("<u2")
    clean, _ = downsample_depth_centers(raw.astype(np.float32) * .001, np.ones(raw.shape, dtype=bool))
    group = masks[split]
    mask_path, raw_valid = group[mask_counter[split] % len(group)]
    mask_counter[split] += 1
    masked_u16 = raw.copy()
    masked_u16[~raw_valid] = 0
    filled = librealsense_2584_mode1_source(masked_u16)
    added = (~raw_valid) & (filled > 0)
    remaining = filled == 0
    prepared = filled.astype(np.float32) * .001
    prepared[remaining] = 6.
    masked, _ = downsample_depth_centers(prepared, np.ones(prepared.shape, dtype=bool))
    sampled_unknown, _ = downsample_depth_centers(remaining.astype(np.float32), np.ones(prepared.shape, dtype=bool))
    for kind, image in (("clean", clean), ("masked", masked)):
        name = f"{key}_{kind}"
        np.save(destination / f"{name}.npy", image)
        labels[name] = label_m
    records[key] = {
        "key": key, "split": split, "layout": scene_name, "pose": {
            "x": pose.x, "y": pose.y, "z": pose.z, "yaw_rad": pose.yaw,
            "pitch_down_deg": None},
        "mask_source": str(mask_path.relative_to(ROOT)),
        "paired_clean_masked_same_label": True,
        "raw_unknown_fraction": float((~raw_valid).mean()),
        "sdk_mode1_filled_fraction": float(added.mean()),
        "remaining_unknown_fraction": float(remaining.mean()),
        "model_input_unknown_fraction_after_fill": float(sampled_unknown.mean()),
        "augmentation_note": "real captured zero mask transplanted onto synthetic geometry; not a sensor physics simulation",
    }
    return clean, masked


def run_layout(mujoco_module, compare_module, generation_module, layout, output, masks,
               mask_counter, rng, max_attempts, poses_override, all_rows, records):
    scene_path = output / "scenes" / f"{layout['name']}.xml"
    scene_path.write_text(scene_xml(layout["parts"]))
    model = mujoco_module.MjModel.from_xml_path(str(scene_path))
    data = mujoco_module.MjData(model)
    mujoco_module.mj_forward(model, data)
    body_id = compare_module.find_body_id(mujoco_module, model)
    config = CAMERA
    depth_renderer = compare_module.make_depth_renderer(mujoco_module, model, config)
    seg_renderer = mujoco_module.Renderer(model, height=270, width=480)
    seg_renderer.enable_segmentation_rendering()
    warm_up(model, data, depth_renderer, seg_renderer)
    split = layout["split"]
    destination = output / "data" / split
    destination.mkdir(parents=True, exist_ok=True)
    labels = {}
    label_path = destination / "label.pkl"
    if label_path.exists():
        with label_path.open("rb") as f:
            labels = pickle.load(f)
    total = poses_override or layout["count"]
    quota = quota_counts(total, layout["quota"])
    accepted = {"near": 0, "mid": 0, "far": 0}
    accepted_total = 0
    attempts = 0
    try:
        while any(accepted[c] < quota[c] for c in accepted) and attempts < max_attempts:
            attempts += 1
            x = float(rng.uniform(*POSE_X_RANGE))
            y = float(rng.uniform(*POSE_Y_RANGE))
            yaw = float(rng.uniform(*POSE_YAW_RANGE))
            z = float(rng.uniform(*BASE_Z_RANGE_M))
            pitch = float(rng.uniform(*PITCH_RANGE_DEG))
            pose = compare_module.PoseSample(attempts, "d435i_ray_adaptation", x, y, z, yaw)
            compare_module.apply_sample(mujoco_module, model, data, pose)
            if inside_obstacle(model, data, body_id, scene_path.name):
                continue
            label_m = np.exp2(compare_module.compute_geometric_ray_log(
                mujoco_module, model, data, body_id, scene_path.name)).astype(np.float32)
            if float(label_m.min()) < MIN_CLEARANCE_M:
                continue
            cls = classify(label_m)
            if cls is None or accepted[cls] >= quota[cls]:
                continue
            pose_config = replace(config, pitch_down_deg=pitch)
            native = render_native_checked(model, data, depth_renderer, body_id, pose_config)
            key = f"{layout['name']}_pose{accepted_total:03d}"
            clean, _masked = save_pair(destination, key, split, masks, mask_counter,
                                       native, label_m, labels, records, pose, layout["name"])
            records[key]["pose"]["pitch_down_deg"] = pitch
            records[key]["pose_class"] = cls
            rows = visibility_rows(model, data, body_id, scene_path.name, key, split,
                                   pose, pose_config, label_m, clean, seg_renderer)
            all_rows.extend(rows)
            accepted[cls] += 1
            accepted_total += 1
            print(f"[prepare] {layout['name']} {key} class={cls} attempts={attempts}", flush=True)
    finally:
        depth_renderer.close()
        seg_renderer.close()
        seg_renderer._mjr_context = None
    if any(accepted[c] < quota[c] for c in accepted):
        raise RuntimeError(f"{layout['name']}: unmet quota {accepted} vs {quota} after {attempts} attempts")
    with label_path.open("wb") as f:
        pickle.dump(labels, f)
    return {"scene": layout["name"], "split": split, "attempts": attempts,
            "accepted": accepted, "images": 2 * sum(accepted.values())}


def run_diagnostics(output):
    """0.1-0.45 m is a visibility diagnostic only; labels keep the ABS 0.1 m floor."""
    diag_dir = output / "diagnostic"
    diag_dir.mkdir(parents=True, exist_ok=True)
    results = {"use": "visibility diagnostic only; excluded from train/validation/test",
               "samples": []}
    for distance in DIAGNOSTIC_TARGETS:
        name = f"diagnostic_center_{distance:.2f}m"
        scene_path = output / "scenes" / f"{name}.xml"
        scene_path.write_text(scene_xml([wall("backdrop", 4.0), target_xml(0., distance)]))
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        data = mujoco.MjData(model)
        body_id = compare.find_body_id(mujoco, model)
        pose = compare.PoseSample(0, "diagnostic", 0., 0., .36, 0.)
        compare.apply_sample(mujoco, model, data, pose)
        config = replace(CAMERA, pitch_down_deg=4.)
        renderer = compare.make_depth_renderer(mujoco, model, config)
        detail_renderer = mujoco.Renderer(model, height=270, width=480)
        detail_renderer.enable_segmentation_rendering()
        try:
            warm_up(model, data, renderer, detail_renderer)
            compare.apply_sample(mujoco, model, data, pose)
            native = render_native_checked(model, data, renderer, body_id, config)
            native_raw = np.rint(native * 1000.).clip(0, 65535).astype("<u2")
            clean, _ = downsample_depth_centers(native_raw.astype(np.float32) * .001,
                                                np.ones(native_raw.shape, dtype=bool))
            label_m = np.exp2(compare.compute_geometric_ray_log(
                mujoco, model, data, body_id, scene_path.name)).astype(np.float32)
            rows = visibility_rows(model, data, body_id, scene_path.name, name, "diagnostic",
                                   pose, config, label_m, clean, detail_renderer)
            np.save(diag_dir / f"{name}_clean.npy", clean)
            center = rows[5]
            results["samples"].append({
                "sample": name, "target_distance_m": distance,
                "label_m": float(label_m[5]),
                "center_hit_geom": center["hit_geom"],
                "label_hitpoint_in_image": center["label_hitpoint_in_image"],
                "label_hit_u_px": center["label_hit_u_px"],
                "label_hit_v_px": center["label_hit_v_px"],
                "projected_hit_optical_z_m": center["projected_hit_optical_z_m"],
                "obstacle_has_visible_pixels_in_render": center["obstacle_has_visible_pixels_in_render"],
            })
        finally:
            renderer.close()
            detail_renderer.close()
            detail_renderer._mjr_context = None
    (diag_dir / "diagnostic.json").write_text(json.dumps(results, indent=2))


def run_geometry_checks(output):
    """A few known angles/distances, one target per scene, to check sign/order/projection."""
    check_dir = output / "geometry_checks"
    check_dir.mkdir(parents=True, exist_ok=True)
    entries = []
    for angle_deg, distance in CHECK_TARGETS:
        scene_path = check_dir / f"check_{angle_deg:+.0f}_{distance:.1f}m.xml"
        # No backdrop: every other beam must stay at the 6 m range cap.
        scene_path.write_text(scene_xml([target_xml(angle_deg, distance)]))
        model = mujoco.MjModel.from_xml_path(str(scene_path))
        data = mujoco.MjData(model)
        body_id = compare.find_body_id(mujoco, model)
        pose = compare.PoseSample(0, "geometry_check", 0., 0., .36, 0.)
        compare.apply_sample(mujoco, model, data, pose)
        config = replace(CAMERA, pitch_down_deg=4.)
        renderer = compare.make_depth_renderer(mujoco, model, config)
        detail_renderer = mujoco.Renderer(model, height=270, width=480)
        detail_renderer.enable_segmentation_rendering()
        try:
            warm_up(model, data, renderer, detail_renderer)
            compare.apply_sample(mujoco, model, data, pose)
            native = render_native_checked(model, data, renderer, body_id, config)
            raw = np.rint(native * 1000.).clip(0, 65535).astype("<u2")
            clean, _ = downsample_depth_centers(raw.astype(np.float32) * .001,
                                                np.ones(raw.shape, dtype=bool))
            label_m = np.exp2(compare.compute_geometric_ray_log(
                mujoco, model, data, body_id, scene_path.name)).astype(np.float32)
            rows = visibility_rows(model, data, body_id, scene_path.name, scene_path.stem,
                                   "geometry_check", pose, config, label_m, clean, detail_renderer)
            ray_index = int(round((angle_deg + 45.) / 9.))
            row = rows[ray_index]
            hit_expected = all(abs(float(label_m[i]) - 6.) < 1e-3 for i in range(11) if i != ray_index)
            entry = {
                "angle_deg": angle_deg, "distance_m": distance, "ray_index": ray_index,
                "label_m": float(label_m[ray_index]),
                "label_matches_distance": bool(abs(float(label_m[ray_index]) - distance) < 1e-3),
                "other_beams_are_range_cap": hit_expected,
                "hit_geom": row["hit_geom"],
                "hit_u_px": row["label_hit_u_px"], "hit_v_px": row["label_hit_v_px"],
                "hitpoint_in_image": row["label_hitpoint_in_image"],
                "u_side_consistent": bool((angle_deg > 0 and row["label_hit_u_px"] < CAMERA.ppx) or
                                          (angle_deg < 0 and row["label_hit_u_px"] > CAMERA.ppx) or
                                          (angle_deg == 0 and abs(row["label_hit_u_px"] - CAMERA.ppx) < 2.)),
                "model_input_depth_at_hitpoint_m": row["model_input_depth_at_hitpoint_m"],
                "projected_hit_optical_z_m": row["projected_hit_optical_z_m"],
            }
            entries.append(entry)
            if not (entry["label_matches_distance"] and entry["other_beams_are_range_cap"]
                    and entry["u_side_consistent"]):
                raise RuntimeError(f"geometry check failed: {entry}")
        finally:
            renderer.close()
            detail_renderer.close()
            detail_renderer._mjr_context = None
    (check_dir / "geometry_checks.json").write_text(json.dumps({
        "camera": {"housing_proxy_base_m": list(CAMERA.position_body_m), "pitch_down_deg": 4.,
                   "base_z_m": .36,
                   "intrinsics": [CAMERA.width, CAMERA.height, CAMERA.fx, CAMERA.fy, CAMERA.ppx, CAMERA.ppy]},
        "rays_deg": RAY_ANGLES, "entries": entries,
        "note": "one synthetic target per scene; checks label distance, beam index/sign and projection, not real-camera accuracy",
    }, indent=2))


def summarize(output, all_rows, records, layout_stats, masks, elapsed_s, bytes_saved):
    splits = ("train", "validation", "test")
    bin_counts = {s: {"below_0p45": 0, "near_045_1p5": 0, "mid_1p5_3": 0,
                      "far_3_6": 0, "cap_6": 0} for s in splits}
    pose_classes = {s: {"near": 0, "mid": 0, "far": 0} for s in splits}
    for record in records.values():
        pose_classes[record["split"]][record["pose_class"]] += 1
    per_ray = {s: [{"count": 0, "near": 0, "cap": 0, "sum": 0., "min": 99., "max": 0.} for _ in RAY_ANGLES]
               for s in splits}
    for row in all_rows:
        if row["split"] not in splits:
            continue
        d = row["label_m"]
        key = ("cap_6" if d >= 5.999 else "far_3_6" if d >= BIN_FAR[0]
               else "mid_1p5_3" if d >= BIN_MID[0] else "near_045_1p5" if d >= BIN_NEAR[0] else "below_0p45")
        bin_counts[row["split"]][key] += 1
        stat = per_ray[row["split"]][row["ray_index"]]
        stat["count"] += 1
        stat["near"] += int(row["label_m"] < 1.5)
        stat["cap"] += int(d >= 5.999)
        stat["sum"] += d
        stat["min"] = min(stat["min"], d)
        stat["max"] = max(stat["max"], d)
    for s in splits:
        for stat in per_ray[s]:
            stat["mean"] = stat["sum"] / max(1, stat["count"]) if stat["count"] else None
            stat["min"] = stat["min"] if stat["count"] else None
            stat["max"] = stat["max"] if stat["count"] else None
            stat.pop("sum", None)
    hit_rows = [r for r in all_rows if r["split"] in splits and r["hit_geom"] != "range_cap"]
    outside = [r for r in hit_rows if not r["label_hitpoint_in_image"]]
    visibility = {
        "hit_label_count": len(hit_rows),
        "hitpoint_inside_image_count": len(hit_rows) - len(outside),
        "hitpoint_outside_image_count": len(outside),
        "outside_hitpoint_but_some_obstacle_surface_visible": sum(
            r["obstacle_has_visible_pixels_in_render"] for r in outside),
        "outside_hitpoint_and_no_obstacle_pixels_in_render": sum(
            not r["obstacle_has_visible_pixels_in_render"] for r in outside),
        "inside_hitpoint_but_hit_geom_pixel_absent": sum(
            r["segmentation_geom_at_hitpoint_pixel"] < 0 for r in hit_rows if r["label_hitpoint_in_image"]),
        "method": "MuJoCo segmentation rendering with candidate K; each split counted separately",
    }
    per_split_visibility = {}
    for s in splits:
        rows = [r for r in hit_rows if r["split"] == s]
        out = [r for r in rows if not r["label_hitpoint_in_image"]]
        per_split_visibility[s] = {
            "hit_label_count": len(rows), "inside": len(rows) - len(out), "outside": len(out),
            "outside_but_surface_visible": sum(r["obstacle_has_visible_pixels_in_render"] for r in out),
            "outside_and_no_surface": sum(not r["obstacle_has_visible_pixels_in_render"] for r in out),
        }
    mask_stats = {}
    for s in splits:
        vals = [r for r in records.values() if r["split"] == s]
        mask_stats[s] = {
            "poses": len(vals),
            "mask_sources_used": len({r["mask_source"] for r in vals}),
            "raw_unknown_fraction_mean": float(np.mean([r["raw_unknown_fraction"] for r in vals])),
            "raw_unknown_fraction_min": float(np.min([r["raw_unknown_fraction"] for r in vals])),
            "raw_unknown_fraction_max": float(np.max([r["raw_unknown_fraction"] for r in vals])),
            "filled_fraction_mean": float(np.mean([r["sdk_mode1_filled_fraction"] for r in vals])),
            "remaining_unknown_fraction_mean": float(np.mean([r["remaining_unknown_fraction"] for r in vals])),
            "model_input_unknown_fraction_mean": float(
                np.mean([r["model_input_unknown_fraction_after_fill"] for r in vals])),
        }
    total_rays = {s: sum(stat["count"] for stat in per_ray[s]) for s in splits}
    summary = {
        "training_performed": False,
        "dataset_kind": "offline geometry/mask adaptation candidate for a later bounded training run",
        "elapsed_seconds": elapsed_s,
        "bytes_on_disk": bytes_saved,
        "poses_total": len(records),
        "images_total": 2 * len(records),
        "poses_by_split": {s: sum(1 for r in records.values() if r["split"] == s) for s in splits},
        "layout_stats": layout_stats,
        "pose_class_counts": pose_classes,
        "label_bin_counts": bin_counts,
        "label_bin_fraction": {s: {k: v / max(1, total_rays[s]) for k, v in bin_counts[s].items()}
                               for s in splits},
        "cap_fraction": {s: bin_counts[s]["cap_6"] / max(1, total_rays[s]) for s in splits},
        "near_fraction": {s: bin_counts[s]["near_045_1p5"] / max(1, total_rays[s]) for s in splits},
        "per_ray_label_stats": per_ray,
        "hitpoint_visibility": visibility,
        "per_split_visibility": per_split_visibility,
        "mask_augmentation": mask_stats,
        "camera": {"housing_proxy_base_m": list(CAMERA.position_body_m),
                   "pitch_range_deg": list(PITCH_RANGE_DEG),
                   "base_height_range_m": list(BASE_Z_RANGE_M),
                   "depth_intrinsics": [CAMERA.width, CAMERA.height, CAMERA.fx, CAMERA.fy,
                                        CAMERA.ppx, CAMERA.ppy]},
        "contract": {"rays_deg": RAY_ANGLES, "ray_origin_base_m": [compare.RAY2D_X0, compare.RAY2D_Y0],
                     "label_range_m": [compare.RAY2D_MIN_DIST, compare.RAY2D_MAX_DIST],
                     "pose_uniform_ranges": {"x": list(POSE_X_RANGE), "y": list(POSE_Y_RANGE),
                                             "yaw_rad": list(POSE_YAW_RANGE)}},
        "processing": {
            "training_and_inference_values": "native Z metres -> mm Z16 -> real zero mask -> librealsense 2.58.4 source-equivalent mode1 fill -> remaining zero to 6 m -> centre sample (3i+1,3j+1) -> clip 0.1-6 m -> log2 -> 3 channels",
            "field_candidate_equivalent": "depth mm * 0.001 -> device SDK mode1 fill -> centre sample -> residual zero to 6 m -> clip/log2/3 channels",
            "field_default_strict_path": "rejects a frame when the centre pixel is still unknown; the 6 m replacement is not the default",
        },
        "limitations": [
            "housing centre is only an optical-origin proxy; pitch and base height are candidates, not precision extrinsic calibration",
            "mask transplant is an augmentation, not a sensor physics simulation; mask frames are reused across poses",
            "residual unknown filled to 6 m is a candidate numeric policy, not measured free space",
            "all labels come from scene geometry; no neural prediction is used as a label",
            "0.1-0.45 m is a separate visibility diagnostic and is not part of train/validation/test",
            "synthetic held-out layouts are development data, not real-world accuracy evidence",
        ],
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2))
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True,
                        help="new directory; the script refuses to overwrite")
    parser.add_argument("--poses-per-layout", type=int, default=None,
                        help="probe override; default uses each layout's planned count")
    parser.add_argument("--seed", type=int, default=20261115)
    parser.add_argument("--max-attempts", type=int, default=40000)
    parser.add_argument("--skip-geometry-checks", action="store_true")
    args = parser.parse_args(argv)
    out = args.output_root.expanduser().resolve()
    if out.exists():
        raise FileExistsError(out)
    out.mkdir(parents=True)
    (out / "scenes").mkdir()
    rng = np.random.default_rng(args.seed)
    masks = {}
    for split, groups in MASK_GROUPS.items():
        masks[split] = []
        for group in groups:
            for path in sorted((ROOT / group).glob("*_depth.u16")):
                mask = np.fromfile(path, dtype="<u2")
                if mask.size != 480 * 270:
                    raise ValueError(f"mask dimensions invalid: {path}")
                masks[split].append((path, mask.reshape(270, 480) > 0))
        if not masks[split]:
            raise ValueError(f"no raw masks for split {split}")
    mask_counter = {s: 0 for s in masks}
    all_rows, records, layout_stats = [], {}, []
    import time
    t0 = time.time()
    for layout in build_layouts():
        stats = run_layout(mujoco, compare, generation, layout, out, masks, mask_counter,
                           rng, args.max_attempts, args.poses_per_layout, all_rows, records)
        layout_stats.append(stats)
    run_diagnostics(out)
    if not args.skip_geometry_checks:
        run_geometry_checks(out)
    elapsed = time.time() - t0
    with (out / "ray_projection_visibility.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)
    (out / "data_records.json").write_text(json.dumps(records, indent=2))
    bytes_saved = sum(p.stat().st_size for p in out.rglob("*") if p.is_file())
    summary = summarize(out, all_rows, records, layout_stats, masks, elapsed, bytes_saved)
    print(json.dumps({k: summary[k] for k in ("poses_total", "images_total", "poses_by_split",
                                              "label_bin_fraction", "cap_fraction", "near_fraction",
                                              "hitpoint_visibility", "elapsed_seconds", "bytes_on_disk")},
                     indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
