#!/usr/bin/env python3
"""Diagnose fixed near-coverage validation errors from saved predictions and renders.

This is a one-off offline audit: it reads existing validation predictions, re-renders
only their near-visible false-safe poses for pixel/geometry correspondence, and never
loads or evaluates a model checkpoint.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
V2 = ROOT / "logs/real_go2/20261116_d435i_ray_adaptation_data/adaptation_v2_current_mount"
SUPP = ROOT / "logs/real_go2/20261117_d435i_near_coverage_supplement_v1"
PRED = ROOT / "logs/real_go2/20261117_d435i_near_coverage_training/medium_fullnet_near_coverage_60ep/validation_comparison/per_sample_per_ray.csv"
CAMERA_POS_BODY = np.array([.300, .020, .132], dtype=np.float64)
PITCH_DOWN_DEG = 3.0
W, H, FX, FY, PPX, PPY = 480, 270, 244.134, 244.134, 245.186, 134.22
RAY_X0, RAY_Y0 = -.05, 0.
RAY_ANGLES = np.arange(-45., 46., 9.)
RAY_MIN, RAY_MAX = .1, 6.
HEIGHT_BAND_M = (-.10, .10)  # Body-frame slab around the horizontal ABS ray plane.
ANGLE_TOL_DEG = math.degrees(math.atan(.5 / (FX / 3.)))  # Half one 160x90 input pixel at image center.
MODEL_NAMES = ("medium", "epoch13", "near_coverage_epoch3")


def dist_bin(target: float) -> str:
    if target <= 1.5:
        return "near_le_1p5m"
    if target < 3.:
        return "mid_1p5_to_3m"
    return "far_ge_3m_including_6m_cap"


def mean(values):
    return float(np.mean(values)) if values else None


def median(values):
    return float(np.median(values)) if values else None


def group_match(visibility, group):
    if group == "all":
        return True
    if group == "visible_target":
        return visibility in ("in_frame", "out_of_frame_surface_visible")
    return visibility == group


def read_inputs():
    predictions = list(csv.DictReader(PRED.open(newline="", encoding="utf-8")))
    sources = {"v2_validation": V2, "supplement_validation": SUPP}
    visibility, records = {}, {}
    for source, root in sources.items():
        for row in csv.DictReader((root / "ray_projection_visibility.csv").open(newline="", encoding="utf-8")):
            visibility[(source, row["sample"], int(row["ray_index"]))] = row
        records[source] = json.loads((root / "data_records.json").read_text(encoding="utf-8"))
    return predictions, sources, visibility, records


def pair_predictions(rows):
    pairs = {}
    for row in rows:
        key = (row["source"], row["sample"], int(row["ray_index"]))
        pair = pairs.setdefault(key, {"target_m": float(row["label_m"]), "visibility": row["visibility"],
                                      "scene": row["sample"].rsplit("_pose", 1)[0], "flavors": {}})
        pair["flavors"][row["flavor"]] = row
    return pairs


def pair_error(row_pair, model):
    rows = list(row_pair["flavors"].values())
    return float(np.mean([float(r[f"{model}_error_m"]) for r in rows]))


def error_summary(rows):
    pairs = pair_predictions(rows)
    summary = {"observation_unit": "unique pose-ray pair; clean/masked predictions averaged within each pair",
               "models": {}, "old_epoch13_to_new": {}}
    for model in MODEL_NAMES:
        summary["models"][model] = {}
        for group in ("all", "visible_target", "in_frame", "out_of_frame_surface_visible", "fully_invisible", "no_hit_within_range"):
            for bin_name in ("near_le_1p5m", "mid_1p5_to_3m", "far_ge_3m_including_6m_cap"):
                ps = [p for p in pairs.values()
                      if group_match(p["visibility"], group)
                      and dist_bin(p["target_m"]) == bin_name]
                errors = [pair_error(p, model) for p in ps]
                predictions = [float(np.mean([float(r[f"{model}_pred_m"]) for r in p["flavors"].values()])) for p in ps]
                targets = [p["target_m"] for p in ps]
                eligible = [abs(e) for e, t in zip(errors, targets) if t <= 1.5]
                false_safe = sum(e >= .5 and t <= 1.5 for e, t in zip(errors, targets))
                false_danger = sum(e <= -.5 and t >= 3. for e, t in zip(errors, targets))
                key = f"{group}/{bin_name}"
                summary["models"][model][key] = {
                    "unique_pose_ray_pairs": len(ps), "mean_signed_error_m": mean(errors),
                    "mae_m": mean([abs(e) for e in errors]), "near_false_safe_count": int(false_safe),
                    "near_false_safe_denominator": sum(t <= 1.5 for t in targets),
                    "near_overestimate_p95_m": float(np.quantile(eligible, .95)) if eligible else None,
                    "near_overestimate_max_m": max(eligible) if eligible else None,
                    "far_false_danger_count": int(false_danger),
                    "far_false_danger_denominator": sum(t >= 3. for t in targets),
                    "mean_prediction_m": mean(predictions),
                }
    # Paired comparison by one geometry/ray, rather than counting the two input flavors as trials.
    for group in ("all", "visible_target", "in_frame", "out_of_frame_surface_visible", "fully_invisible"):
        for bin_name in ("near_le_1p5m", "mid_1p5_to_3m", "far_ge_3m_including_6m_cap"):
            ps = [p for p in pairs.values() if group_match(p["visibility"], group)
                  and dist_bin(p["target_m"]) == bin_name]
            delta_abs, delta_pred = [], []
            better = worse = tied = 0
            for p in ps:
                old = pair_error(p, "epoch13")
                new = pair_error(p, "near_coverage_epoch3")
                d = abs(new) - abs(old)
                delta_abs.append(d)
                delta_pred.append(new - old)
                better += d < -1e-9
                worse += d > 1e-9
                tied += abs(d) <= 1e-9
            summary["old_epoch13_to_new"][f"{group}/{bin_name}"] = {
                "unique_pose_ray_pairs": len(ps), "absolute_error_better": int(better),
                "absolute_error_worse": int(worse), "ties": int(tied),
                "mean_abs_error_change_m": mean(delta_abs),
                "mean_prediction_shift_m": mean(delta_pred),
            }
    summary["layout_near_visible"] = {}
    for layout in sorted({p["scene"] for p in pairs.values()}):
        ps = [p for p in pairs.values() if p["scene"] == layout and p["target_m"] <= 1.5
              and p["visibility"] in ("in_frame", "out_of_frame_surface_visible")]
        if not ps:
            continue
        summary["layout_near_visible"][layout] = {"unique_pose_ray_pairs": len(ps), "models": {}}
        for model in MODEL_NAMES:
            errors = [pair_error(p, model) for p in ps]
            summary["layout_near_visible"][layout]["models"][model] = {
                "mean_signed_error_m": mean(errors), "mae_m": mean([abs(e) for e in errors]),
                "false_safe_count": sum(e >= .5 for e in errors),
            }
    summary["per_ray_visible_near"] = {}
    for ray in range(11):
        ps = [p for p in pairs.values() if int(p["flavors"][next(iter(p["flavors"]))]["ray_index"]) == ray
              and p["target_m"] <= 1.5 and group_match(p["visibility"], "visible_target")]
        row = {"angle_deg": -45 + 9 * ray, "unique_pose_ray_pairs": len(ps), "models": {}}
        for model in MODEL_NAMES:
            errs = [pair_error(p, model) for p in ps]
            predictions = [float(np.mean([float(r[f"{model}_pred_m"]) for r in p["flavors"].values()])) for p in ps]
            row["models"][model] = {"mean_signed_error_m": mean(errs), "mae_m": mean([abs(e) for e in errs]),
                                     "false_safe_count": sum(e >= .5 for e in errs),
                                     "false_safe_denominator": len(errs), "mean_prediction_m": mean(predictions)}
        old, new = [pair_error(p, "epoch13") for p in ps], [pair_error(p, "near_coverage_epoch3") for p in ps]
        row["epoch13_to_new"] = {"mean_prediction_shift_m": mean([b-a for a,b in zip(old,new)]),
                                  "absolute_error_better": sum(abs(b)<abs(a) for a,b in zip(old,new)),
                                  "absolute_error_worse": sum(abs(b)>abs(a) for a,b in zip(old,new))}
        summary["per_ray_visible_near"][f"{ray}:{-45+9*ray:+d}deg"] = row
    summary["clean_masked_visible_near"] = {}
    by_key = defaultdict(dict)
    for r in rows:
        if r["visibility"] in ("in_frame", "out_of_frame_surface_visible") and float(r["label_m"]) <= 1.5:
            by_key[(r["source"], r["sample"], r["ray_index"])][r["flavor"]] = r
    for model in MODEL_NAMES:
        diffs, flips = [], {"clean_only_false_safe": 0, "masked_only_false_safe": 0}
        for flavors in by_key.values():
            if set(flavors) != {"clean", "masked"}:
                continue
            ce = float(flavors["clean"][f"{model}_error_m"])
            me = float(flavors["masked"][f"{model}_error_m"])
            diffs.append(abs(me) - abs(ce))
            flips["clean_only_false_safe"] += ce >= .5 and me < .5
            flips["masked_only_false_safe"] += ce < .5 and me >= .5
        summary["clean_masked_visible_near"][model] = {
            "unique_pose_ray_pairs": len(diffs), "clean_to_masked_mean_abs_error_change_m": mean(diffs),
            "masked_abs_error_better_pairs": sum(x < -1e-9 for x in diffs),
            "masked_abs_error_worse_pairs": sum(x > 1e-9 for x in diffs), **flips,
        }
    return summary, pairs


def choose_controls(rows, failure_keys):
    pools = defaultdict(list)
    for r in rows:
        if r["visibility"] not in ("in_frame", "out_of_frame_surface_visible") or float(r["label_m"]) > 1.5:
            continue
        key = (r["source"], r["sample"], int(r["ray_index"]))
        if key in failure_keys:
            continue
        candidates = [q for q in rows if q["source"] == r["source"] and q["sample"] == r["sample"]
                      and q["ray_index"] == r["ray_index"]]
        if len(candidates) != 2 or any(float(q["near_coverage_epoch3_error_m"]) >= .5 for q in candidates):
            continue
        err = abs(float(np.mean([float(q["near_coverage_epoch3_error_m"]) for q in candidates])))
        pools[(r["source"], r["visibility"])].append((err, key))
    controls = []
    for _, candidates in pools.items():
        controls.append(min(candidates)[1])
    return controls


def mode1_fill(raw_u16):
    # Exact source-equivalent implementation reused from the earlier D435i diagnostic.
    h, w = raw_u16.shape
    out = raw_u16.copy().reshape(-1)
    for y in range(1, h - 1):
        p = y * w + 1
        for _ in range(1, w):
            if out[p] == 0:
                out[p] = max(int(out[p-w]), int(out[p-w-1]), int(out[p-1]),
                             int(out[p+w-1]), int(out[p+w]))
            p += 1
    return out.reshape(h, w)


def render_cases(case_keys, sources, visibility, records, controls, out_dir):
    import mujoco
    import compare_ray_pred as compare
    from prepare_d435i_ray_adaptation_data import projected_pixel, set_candidate_frustum
    from realsense_depth_to_abs_rays import downsample_depth_centers

    config = compare.CameraRenderConfig(tuple(CAMERA_POS_BODY), PITCH_DOWN_DEG, W, H, FX, FY, PPX, PPY)
    config.validate()
    case_by_pose = defaultdict(list)
    for key in case_keys + controls:
        case_by_pose[(key[0], key[1])].append(key)
    result = {}
    for (source, sample), cases in case_by_pose.items():
        root = sources[source]
        rec = records[source][sample]
        scene_name = visibility[(source, sample, cases[0][2])]["scene"]
        model = mujoco.MjModel.from_xml_path(str(root / "scenes" / scene_name))
        data = mujoco.MjData(model)
        body_id = compare.find_body_id(mujoco, model)
        pose = rec["pose"]
        pose_sample = compare.PoseSample(index=0, source="validation_diagnosis", x=pose["x"], y=pose["y"],
                                         z=pose["z"], yaw=pose["yaw_rad"])
        compare.apply_sample(mujoco, model, data, pose_sample)
        depth_renderer = compare.make_depth_renderer(mujoco, model, config)
        seg_renderer = mujoco.Renderer(model, height=H, width=W)
        seg_renderer.enable_segmentation_rendering()
        cam = compare.camera_from_body(mujoco, data, body_id, config)
        depth_renderer.update_scene(data, camera=cam)
        set_candidate_frustum(depth_renderer, config)
        native = np.clip(depth_renderer.render(), .1, 6.).astype(np.float32)
        seg_renderer.update_scene(data, camera=cam)
        set_candidate_frustum(seg_renderer, config)
        seg = seg_renderer.render()
        raw_u16 = np.rint(native * 1000.).clip(0, 65535).astype("<u2")
        clean_render, _ = downsample_depth_centers(raw_u16.astype(np.float32) * .001,
                                                     np.ones(raw_u16.shape, dtype=bool))
        clean_render = np.clip(clean_render, .1, 6.).astype(np.float32)
        clean_dir = root / "data/validation"
        clean_input = np.load(clean_dir / f"{sample}_clean.npy", allow_pickle=False)
        masked_input = np.load(clean_dir / f"{sample}_masked.npy", allow_pickle=False)
        mask_path = ROOT / rec["mask_source"]
        raw_mask = np.fromfile(mask_path, dtype="<u2").reshape(H, W)
        raw_valid = raw_mask > 0
        masked_u16 = raw_u16.copy()
        masked_u16[~raw_valid] = 0
        filled_u16 = mode1_fill(masked_u16)
        mode1_filled = (~raw_valid) & (filled_u16 > 0)
        residual_unknown = filled_u16 == 0
        masked_m = filled_u16.astype(np.float32) * .001
        masked_m[residual_unknown] = 6.
        masked_render, _ = downsample_depth_centers(masked_m, np.ones(masked_m.shape, dtype=bool))
        masked_render = np.clip(masked_render, .1, 6.).astype(np.float32)
        pix_diff_clean = float(np.max(np.abs(clean_render - clean_input)))
        pix_diff_masked = float(np.max(np.abs(masked_render - masked_input)))

        # Point diagnostics: all original valid rendered depth pixels, no object labels used.
        vv, uu = np.indices((H, W), dtype=np.float64)
        zc = native.astype(np.float64)
        xc = (uu - PPX) / FX * zc
        yc = (vv - PPY) / FY * zc
        pitch = math.radians(PITCH_DOWN_DEG)
        xb = CAMERA_POS_BODY[0] + math.cos(pitch) * zc - math.sin(pitch) * yc
        yb = CAMERA_POS_BODY[1] - xc
        zb = CAMERA_POS_BODY[2] - math.cos(pitch) * yc - math.sin(pitch) * zc
        points_valid = np.isfinite(zc) & (zc > .1) & (zc < 6.) & (zb >= HEIGHT_BAND_M[0]) & (zb <= HEIGHT_BAND_M[1])
        xrel, yrel = xb - RAY_X0, yb - RAY_Y0
        point_ranges = np.hypot(xrel, yrel)
        bearings = np.degrees(np.arctan2(yrel, xrel))
        cam_pos_world = data.xpos[body_id].copy() + data.xmat[body_id].reshape(3, 3) @ CAMERA_POS_BODY
        body_rot = data.xmat[body_id].reshape(3, 3).copy()
        body_pos = data.xpos[body_id].copy()

        for key in cases:
            ray = key[2]
            vis = visibility[key]
            geom_name = vis["hit_geom"]
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name) if geom_name != "range_cap" else -1
            native_target = ((seg[:, :, 0] == geom_id) &
                             (seg[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))) if geom_id >= 0 else np.zeros((H, W), bool)
            target_grid = native_target[1::3, 1::3]
            native_count = int(native_target.sum())
            grid_count = int(target_grid.sum())
            unknown_target_pixels = int((native_target & ~raw_valid).sum())
            fill_target_pixels = int((native_target & mode1_filled).sum())
            residual_target_pixels = int((native_target & residual_unknown).sum())
            target_grid_count = max(grid_count, 1)
            grid_unknown = int((target_grid & ~raw_valid[1::3, 1::3]).sum())
            grid_filled = int((target_grid & mode1_filled[1::3, 1::3]).sum())
            grid_resid = int((target_grid & residual_unknown[1::3, 1::3]).sum())
            u = float(vis["label_hit_u_px"])
            v = float(vis["label_hit_v_px"])
            hit_inside = vis["label_hitpoint_in_image"].lower() == "true"
            if not hit_inside:
                category = "hitpoint_outside_surface_visible" if native_count else "hitpoint_outside_no_surface_pixels"
            elif native_count == 0:
                category = "hitpoint_in_frame_but_target_geom_not_rendered"
            elif grid_count == 0:
                category = "native_target_disappears_after_center_sampling"
            elif grid_count <= 4:
                category = "sparse_center_sample_1_to_4_cells"
            else:
                category = "target_retained_in_model_grid"

            if hit_inside:
                hit_px_x = int(np.clip(np.rint(u), 0, W-1))
                hit_px_y = int(np.clip(np.rint(v), 0, H-1))
                ix = int(np.clip(np.rint((u - 1.) / 3.), 0, 159))
                iy = int(np.clip(np.rint((v - 1.) / 3.), 0, 89))
                sample_u, sample_v = 3 * ix + 1, 3 * iy + 1
                hit_native_depth = float(native[hit_px_y, hit_px_x])
                hit_sample_depth = float(clean_input[iy, ix])
                sampled_seg_at_hit = int(seg[sample_v, sample_u, 0])
            else:
                hit_px_x = hit_px_y = sample_u = sample_v = ix = iy = sampled_seg_at_hit = None
                hit_native_depth = hit_sample_depth = None

            angle = float(vis["ray_angle_deg"])
            # One half-width of a center-sampled input cell around the named ray.
            delta = (bearings - angle + 180.) % 360. - 180.
            point_sel = points_valid & (np.abs(delta) <= ANGLE_TOL_DEG) & (point_ranges > 0) & (point_ranges <= RAY_MAX)
            selected_ranges = point_ranges[point_sel]
            if selected_ranges.size:
                nearest_idx = np.argmin(np.where(point_sel, point_ranges, np.inf))
                nearest_y, nearest_x = np.unravel_index(nearest_idx, point_ranges.shape)
                closest_t = float(point_ranges[nearest_y, nearest_x])
                lateral = abs(float(xrel[nearest_y, nearest_x] * math.sin(math.radians(angle)) -
                                    yrel[nearest_y, nearest_x] * math.cos(math.radians(angle))))
            else:
                closest_t, lateral = None, None

            image_path = clean_dir / f"{sample}_clean.npy"
            masked_path = clean_dir / f"{sample}_masked.npy"
            row = {
                "source": source, "sample": sample, "ray_index": ray, "ray_angle_deg": angle,
                "target_m": float(vis["label_m"]), "candidate_clean_error_m": None,
                "candidate_masked_error_m": None, "visibility": vis["label_hitpoint_in_image"],
                "obstacle_has_surface_pixels": int(native_count > 0), "category": category,
                "target_geometry_pixels_480x270": native_count,
                "target_geometry_cells_160x90": grid_count,
                "sample_retention_fraction": grid_count / native_count if native_count else None,
                "target_pixels_raw_unknown_count": unknown_target_pixels,
                "target_pixels_mode1_filled_count": fill_target_pixels,
                "target_pixels_residual_6m_count": residual_target_pixels,
                "target_grid_raw_unknown_count": grid_unknown,
                "target_grid_mode1_filled_count": grid_filled,
                "target_grid_residual_6m_count": grid_resid,
                "target_grid_coverage_denominator": grid_count,
                "target_grid_raw_unknown_fraction": grid_unknown / target_grid_count if grid_count else None,
                "target_grid_mode1_filled_fraction": grid_filled / target_grid_count if grid_count else None,
                "target_grid_residual_6m_fraction": grid_resid / target_grid_count if grid_count else None,
                "record_raw_unknown_fraction_full_frame": rec["raw_unknown_fraction"],
                "record_mode1_filled_fraction_full_frame": rec["sdk_mode1_filled_fraction"],
                "record_residual_6m_fraction_full_frame": rec["remaining_unknown_fraction"],
                "sampled_clean_depth_max_diff_m": pix_diff_clean,
                "sampled_masked_depth_max_diff_m": pix_diff_masked,
                "projected_hit_optical_z_m": float(vis["projected_hit_optical_z_m"]) if vis["projected_hit_optical_z_m"] else None,
                "nearest_hit_pixel_uv": [hit_px_x, hit_px_y] if hit_inside else None,
                "nearest_center_sample_source_uv": [sample_u, sample_v] if hit_inside else None,
                "nearest_center_sample_grid_xy": [ix, iy] if hit_inside else None,
                "nearest_hit_pixel_depth_m": hit_native_depth,
                "nearest_center_sample_depth_m": hit_sample_depth,
                "nearest_center_sample_geom_id": sampled_seg_at_hit,
                "hit_geom_name": geom_name,
                "camera_intrinsics": [W, H, FX, FY, PPX, PPY],
                "camera_position_base_candidate_m": CAMERA_POS_BODY.tolist(),
                "camera_pitch_down_deg": PITCH_DOWN_DEG,
                "ray_origin_base_m": [RAY_X0, RAY_Y0, 0.],
                "pointcloud_height_band_base_m": list(HEIGHT_BAND_M),
                "pointcloud_ray_angle_tolerance_deg": ANGLE_TOL_DEG,
                "original_valid_depth_points_in_bin": int(selected_ranges.size),
                "pointcloud_min_along_horizontal_range_m": closest_t,
                "pointcloud_lateral_offset_at_min_m": lateral,
                "pointcloud_evidence_status": "approximate_points_available" if selected_ranges.size else "unknown_no_valid_points_in_bin",
                "hit_pixel_uv": [u, v], "label_hitpoint_in_frame": hit_inside,
            }
            result[key] = {"row": row, "native": native, "seg": seg[:, :, 0], "target_mask": native_target,
                           "target_grid": target_grid, "clean": clean_input, "masked": masked_input,
                           "mask_valid": raw_valid, "mode1_filled": mode1_filled,
                           "residual_unknown": residual_unknown, "model": model,
                           "point_sel": point_sel, "body_pos": body_pos, "body_rot": body_rot,
                           "cam_pos_world": cam_pos_world}
        depth_renderer.close()
        seg_renderer.close()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    out_dir = args.output_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=False)
    rows, sources, visibility, records = read_inputs()
    pairs = pair_predictions(rows)
    errors, _ = error_summary(rows)
    failures = []
    for key, pair in pairs.items():
        if pair["visibility"] not in ("in_frame", "out_of_frame_surface_visible") or pair["target_m"] > 1.5:
            continue
        errs = [float(r["near_coverage_epoch3_error_m"]) for r in pair["flavors"].values()]
        if any(e >= .5 for e in errs):
            failures.append(key)
    controls = choose_controls(rows, set(failures))
    rendered = render_cases(failures, sources, visibility, records, controls, out_dir)
    for key in failures + controls:
        r = next(q for q in rows if q["source"] == key[0] and q["sample"] == key[1]
                 and int(q["ray_index"]) == key[2] and q["flavor"] == "clean")
        rendered[key]["row"]["candidate_clean_error_m"] = float(r["near_coverage_epoch3_error_m"])
        masked = next((q for q in rows if q["source"] == key[0] and q["sample"] == key[1]
                       and int(q["ray_index"]) == key[2] and q["flavor"] == "masked"), None)
        rendered[key]["row"]["candidate_masked_error_m"] = float(masked["near_coverage_epoch3_error_m"]) if masked else None
        rendered[key]["row"]["case_role"] = "false_safe_geometry" if key in failures else "illustrative_non_false_safe_control"

    categories = Counter(rendered[k]["row"]["category"] for k in failures)
    group_summary = {}
    for category, count in categories.items():
        keys = [k for k in failures if rendered[k]["row"]["category"] == category]
        group_summary[category] = {
            "unique_pose_ray_cases": count,
            "clean_or_masked_false_safe_rows": sum(sum(1 for r in rows if r["source"] == k[0] and r["sample"] == k[1]
                                                       and int(r["ray_index"]) == k[2] and r["flavor"] in pairs[k]["flavors"]
                                                       and float(r["near_coverage_epoch3_error_m"]) >= .5) for k in keys),
            "mean_original_target_pixels": mean([rendered[k]["row"]["target_geometry_pixels_480x270"] for k in keys]),
            "mean_sampled_target_cells": mean([rendered[k]["row"]["target_geometry_cells_160x90"] for k in keys]),
            "sampled_target_cells_zero": sum(rendered[k]["row"]["target_geometry_cells_160x90"] == 0 for k in keys),
            "target_grid_unknown_fraction_median": median([rendered[k]["row"]["target_grid_raw_unknown_fraction"] for k in keys
                                                             if rendered[k]["row"]["target_grid_raw_unknown_fraction"] is not None]),
            "target_grid_mode1_fill_fraction_median": median([rendered[k]["row"]["target_grid_mode1_filled_fraction"] for k in keys
                                                                if rendered[k]["row"]["target_grid_mode1_filled_fraction"] is not None]),
            "target_grid_residual_6m_fraction_median": median([rendered[k]["row"]["target_grid_residual_6m_fraction"] for k in keys
                                                                if rendered[k]["row"]["target_grid_residual_6m_fraction"] is not None]),
        }
    by_angle = Counter(float(visibility[k]["ray_angle_deg"]) for k in failures)
    by_layout = Counter(rendered[k]["row"]["sample"].rsplit("_pose", 1)[0] for k in failures)

    # A few deterministic representatives: median residual magnitude per observed category,
    # paired with the nearest saved non-false-safe example from the same visibility class.
    representatives = []
    for category, keys in groupby_category(failures, rendered):
        keys = sorted(keys, key=lambda k: abs(float(rendered[k]["row"]["candidate_clean_error_m"] or 0.)))
        representatives.append(keys[len(keys)//2])
    rep_controls = []
    for fail_key in representatives:
        frow = rendered[fail_key]["row"]
        candidates = [k for k in controls if k[0] == fail_key[0]
                      and visibility[k]["label_hitpoint_in_image"].lower() == str(frow["label_hitpoint_in_frame"]).lower()]
        if not candidates:
            candidates = [k for k in controls if k[0] == fail_key[0]]
        if candidates:
            rep_controls.append(min(candidates, key=lambda k: abs(float(visibility[k]["label_m"]) - frow["target_m"])))
    reps_all = list(dict.fromkeys(representatives + rep_controls))
    # Renderer returned detailed image buffers only for preselected failures and controls.
    write_case_figure(reps_all, rendered, rows, visibility, out_dir)

    case_rows = [rendered[k]["row"] for k in failures]
    with (out_dir / "remaining_false_safe_geometry.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(case_rows[0].keys()))
        w.writeheader(); w.writerows(case_rows)
    with (out_dir / "representative_cases.csv").open("w", newline="", encoding="utf-8") as f:
        selected_rows = [rendered[k]["row"] for k in reps_all]
        w = csv.DictWriter(f, fieldnames=list(selected_rows[0].keys()))
        w.writeheader(); w.writerows(selected_rows)

    # Per-bin/group metrics from paired geometry; distinct input flavors are kept in their own summary.
    with (out_dir / "grouped_error_deltas.csv").open("w", newline="", encoding="utf-8") as f:
        fields = ["group_distance", "unique_pose_ray_pairs", "mean_signed_error_m", "mae_m",
                  "near_false_safe_count", "near_false_safe_denominator", "far_false_danger_count",
                  "far_false_danger_denominator", "mean_prediction_shift_epoch13_to_new_m",
                  "absolute_error_better", "absolute_error_worse", "ties"]
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader()
        for group, m in errors["models"].items():
            for key, metric in m.items():
                delta = errors["old_epoch13_to_new"].get(key)
                w.writerow({"group_distance": f"{group}/{key.split('/',1)[1]}", **{k: metric.get(k) for k in fields[1:8]},
                            **({"mean_prediction_shift_epoch13_to_new_m": delta["mean_prediction_shift_m"],
                                "absolute_error_better": delta["absolute_error_better"],
                                "absolute_error_worse": delta["absolute_error_worse"], "ties": delta["ties"]} if delta else {})})

    summary = {
        "scope": "existing combined v2+near-coverage validation only; no model inference; no final holdout accessed",
        "input_prediction_rows": len(rows), "unique_pose_ray_pairs": len(pairs),
        "visible_near_unique_pose_ray_pairs": sum(p["target_m"] <= 1.5 and p["visibility"] in
                                                   ("in_frame", "out_of_frame_surface_visible") for p in pairs.values()),
        "visible_near_false_safe_rows_epoch3": sum(float(r["near_coverage_epoch3_error_m"]) >= .5 and
                                                    float(r["label_m"]) <= 1.5 and r["visibility"] in
                                                    ("in_frame", "out_of_frame_surface_visible") for r in rows),
        "visible_near_false_safe_unique_geometry_cases": len(failures),
        "paired_error_and_shift_summary": errors,
        "failure_categories_unique_pose_ray": group_summary,
        "failure_by_ray_unique_pose_ray": {f"{int(a):+d}deg": n for a, n in sorted(by_angle.items())},
        "failure_by_layout_unique_pose_ray": dict(by_layout),
        "failure_flavors": {
            "clean_and_masked_fail": sum(set(pairs[k]["flavors"]) == {"clean", "masked"} and
                                          all(float(pairs[k]["flavors"][x]["near_coverage_epoch3_error_m"]) >= .5 for x in ("clean", "masked"))
                                          for k in failures),
            "clean_only_fail": sum(float(pairs[k]["flavors"]["clean"]["near_coverage_epoch3_error_m"]) >= .5 and
                                   float(pairs[k]["flavors"].get("masked", {}).get("near_coverage_epoch3_error_m", 0)) < .5 for k in failures),
            "masked_only_fail": sum(float(pairs[k]["flavors"].get("clean", {}).get("near_coverage_epoch3_error_m", 0)) < .5 and
                                    float(pairs[k]["flavors"]["masked"]["near_coverage_epoch3_error_m"]) >= .5 for k in failures),
        },
        "pointcloud_diagnostic_contract": {
            "depth": "original rendered optical-Z depth, quantized to mm only to match saved clean inputs; no hole fill or 6m placeholders",
            "camera_to_body": "inverse of current generator's 3-degree pitch and camera position candidate (.300,.020,.132)m",
            "base_z_band_m": list(HEIGHT_BAND_M), "ray_angle_tolerance_deg": ANGLE_TOL_DEG,
            "angle_basis": "half-width of one 160x90 center-sampled model pixel at image center from fx/3",
            "distance": "horizontal projection from ABS ray origin (-.05,0,0) along the horizontal ray; lateral offset separately retained",
            "selection": "all valid original-depth points in the fixed height and angular bins; obstacle labels are not used to choose points",
            "meaning": "approximate point evidence only; discretization/occlusion may yield unknown; not a continuous-ray truth",
        },
        "representative_case_keys": [list(k) for k in reps_all],
        "mask_reconstruction": "uses the augmentation mask recorded in data_records and librealsense 2.58.4 source-equivalent mode1; reports raw unknown/fill/residual-6m intersections with the target geometry",
        "full_frame_mask_ratio_is_context_only": True,
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({k: summary[k] for k in ("input_prediction_rows", "unique_pose_ray_pairs",
                 "visible_near_unique_pose_ray_pairs", "visible_near_false_safe_rows_epoch3",
                 "visible_near_false_safe_unique_geometry_cases", "failure_categories_unique_pose_ray",
                 "failure_flavors", "representative_case_keys")}, indent=2))


def groupby_category(keys, rendered):
    out = defaultdict(list)
    for key in keys:
        out[rendered[key]["row"]["category"]].append(key)
    return out.items()


def write_case_figure(keys, rendered, rows, visibility, out_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if not keys:
        return
    fig, axes = plt.subplots(len(keys), 4, figsize=(15, max(3.2, 2.5*len(keys))), squeeze=False)
    for i, key in enumerate(keys):
        item = rendered[key]
        r = item["row"]
        sample, ray, source = key[1], key[2], key[0]
        prediction_rows = [q for q in rows if q["source"] == source and q["sample"] == sample and
                           int(q["ray_index"]) == ray]
        pred_text = ", ".join(f"{q['flavor']} {float(q['near_coverage_epoch3_pred_m']):.2f}m"
                               for q in prediction_rows)
        title = f"{r['case_role']}: {r['category']} | ray {r['ray_angle_deg']:+.0f}°, label {r['target_m']:.2f}m; {pred_text}"
        axes[i, 0].imshow(item["native"], cmap="viridis", vmin=.1, vmax=6., origin="upper")
        axes[i, 0].contour(item["target_mask"].astype(float), levels=[.5], colors="red", linewidths=.6)
        if r["label_hitpoint_in_frame"]:
            axes[i, 0].plot(*r["hit_pixel_uv"], "r+", ms=8, mew=1.5)
        axes[i, 0].set_title("480×270 original depth; target geom outline")
        axes[i, 1].imshow(item["target_grid"], cmap="gray_r", origin="upper", vmin=0, vmax=1, aspect="auto")
        axes[i, 1].set_title(f"160×90 target occupancy ({r['target_geometry_cells_160x90']} cells)")
        axes[i, 2].imshow(item["clean"], cmap="viridis", vmin=.1, vmax=6., origin="upper", aspect="auto")
        axes[i, 2].contour(item["target_grid"].astype(float), levels=[.5], colors="red", linewidths=.6)
        axes[i, 2].set_title("saved clean input")
        axes[i, 3].imshow(item["masked"], cmap="viridis", vmin=.1, vmax=6., origin="upper", aspect="auto")
        axes[i, 3].contour(item["target_grid"].astype(float), levels=[.5], colors="red", linewidths=.6)
        axes[i, 3].set_title(f"saved masked; unknown/fill/6m cells {r['target_grid_raw_unknown_count']}/"
                             f"{r['target_grid_mode1_filled_count']}/{r['target_grid_residual_6m_count']}")
        axes[i, 0].set_ylabel(title, fontsize=8)
        for ax in axes[i]:
            ax.set_xticks([]); ax.set_yticks([])
    fig.tight_layout()
    fig.savefig(out_dir / "representative_failure_and_control_cases.png", dpi=140)
    plt.close(fig)


if __name__ == "__main__":
    main()
