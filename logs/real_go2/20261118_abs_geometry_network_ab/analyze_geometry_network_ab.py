#!/usr/bin/env python3
"""Offline full-image raw-depth vs fixed epoch13 ray comparison."""
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = Path(__file__).resolve().parent
BUCKET = ROOT / "logs/real_go2/20261118_bucket_wall_static"
BOARD = ROOT / "logs/real_go2/20261118_standing_board119_B0bEMC"
EPOCH13 = ROOT / "logs/real_go2/20261117_d435i_fullnet_preparation/training_runs/d435i_fullnet_300epoch_20261117/ray_pred_mujoco_finetuned_best.pt"
RA_MODEL = ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/ra_value.pt"

ANGLES = np.arange(-45.0, 46.0, 9.0)
RAY_ORIGIN = np.array([-0.05, 0.0], dtype=np.float64)
MOUNT = np.array([0.300, 0.020, 0.132], dtype=np.float64)
PITCH = math.radians(3.0)
HEIGHT = (-0.10, 0.10)
ANGLE_TOL_DEG = math.degrees(math.atan(0.5 / (244.134 / 3.0)))
MIN_POINTS = 3
DEPTH_RANGE = (0.10, 6.0)
FRAME_IDS = (0, 240, 480)


def read_meta(path):
    return dict(line.split("=", 1) for line in path.read_text().splitlines() if "=" in line)


def load_geom(depth_path, meta):
    h, w = int(meta["height"]), int(meta["width"])
    z = np.fromfile(depth_path, dtype="<u2").reshape(h, w).astype(np.float64) * float(meta["scale"])
    vv, uu = np.indices((h, w), dtype=np.float64)
    fx, fy, cx, cy = (float(meta[k]) for k in ("fx", "fy", "ppx", "ppy"))
    xc = (uu - cx) * z / fx
    yc = (vv - cy) * z / fy
    xb = MOUNT[0] + math.cos(PITCH) * z - math.sin(PITCH) * yc
    yb = MOUNT[1] - xc
    zb = MOUNT[2] - math.cos(PITCH) * yc - math.sin(PITCH) * z
    valid = np.isfinite(z) & (z > 0.0) & (z >= DEPTH_RANGE[0]) & (z <= DEPTH_RANGE[1])
    xrel, yrel = xb - RAY_ORIGIN[0], yb - RAY_ORIGIN[1]
    radii = np.hypot(xrel, yrel)
    bearings = np.degrees(np.arctan2(yrel, xrel))
    distances, counts = [], []
    for angle in ANGLES:
        delta = (bearings - angle + 180.0) % 360.0 - 180.0
        use = (valid & (zb >= HEIGHT[0]) & (zb <= HEIGHT[1]) &
               (np.abs(delta) <= ANGLE_TOL_DEG) &
               (radii >= DEPTH_RANGE[0]) & (radii <= DEPTH_RANGE[1]))
        count = int(use.sum())
        ray_rad = math.radians(float(angle))
        along = xrel[use] * math.cos(ray_rad) + yrel[use] * math.sin(ray_rad)
        distances.append(float(np.min(along)) if count >= MIN_POINTS else None)
        counts.append(count)
    return distances, counts


def board_model_inference(meta):
    """Only the 8 unsaved beams; stored comparison values are checked separately."""
    sys.path.insert(0, str(ROOT / "scripts"))
    from go2_depth_ray_observer import load_model, prepare_input, infer_log_depth
    model, _ = load_model(EPOCH13)
    output = {}
    for frame_id in FRAME_IDS:
        h, w = int(meta["height"]), int(meta["width"])
        filled = np.fromfile(BOARD / "capture" / f"{frame_id}_filled.u16", dtype="<u2").reshape(h, w)
        depth_m = filled.astype(np.float32) * float(meta["scale"])
        inp, _, sampled_valid = prepare_input(depth_m, filled != 0, 6.0)
        if inp is None:
            raise RuntimeError("board comparison preprocessing unexpectedly rejected its filled frame")
        rays, _ = infer_log_depth(model, inp)
        output[frame_id] = rays.astype(float).tolist()
    return output


def main():
    bucket_cmp = json.loads((BUCKET / "comparison.json").read_text())
    board_cmp = json.loads((BOARD / "comparison.json").read_text())
    expected_model_rel = EPOCH13.relative_to(ROOT).as_posix()
    if Path(bucket_cmp["models"][1]).as_posix() != expected_model_rel:
        raise RuntimeError("bucket comparison checkpoint differs from fixed epoch13 path")
    bucket_by_group = {int(g["group"]): g for g in bucket_cmp["groups"]}
    bucket_meta = {g: read_meta(BUCKET / f"group{g}/capture/metadata.txt") for g in range(1, 5)}
    board_meta = read_meta(BOARD / "capture/metadata.txt")
    board_missing_prediction = board_model_inference(board_meta)

    samples = []
    for group in range(1, 5):
        meta = bucket_meta[group]
        data = bucket_by_group[group]
        for frame_id in FRAME_IDS:
            stored = next(x for x in data["representative_frames"] if int(x["frame"]) == frame_id)
            raw_path = BUCKET / f"group{group}/capture/{frame_id}_depth.u16"
            geom, counts = load_geom(raw_path, meta)
            network = [float(x) for x in stored["epoch13_m"]]
            for i, angle in enumerate(ANGLES):
                samples.append({
                    "capture": "bucket_wall_static", "group": group, "frame": frame_id,
                    "scene": data["target"], "ray_index": i, "angle_deg": float(angle),
                    "geometry_m": geom[i], "valid_points": counts[i],
                    "network_epoch13_m": network[i],
                    "geometry_log2_m": math.log2(geom[i]) if geom[i] is not None else None,
                    "network_epoch13_log2_m": math.log2(network[i]),
                    "network_minus_geometry_m": network[i] - geom[i] if geom[i] is not None else None,
                    "network_minus_geometry_log2": math.log2(network[i] / geom[i]) if geom[i] is not None else None,
                    "network_source": "comparison.json saved frame output",
                    "reference_m": None, "network_error_m": None, "geometry_error_m": None,
                })

    stored_board = board_cmp["frames_by_angle"]
    model_path = board_cmp["models"][1]
    if Path(model_path).as_posix() != expected_model_rel:
        raise RuntimeError(f"board comparison checkpoint differs from fixed epoch13 path: {model_path}")
    board_stored_delta = []
    for frame_id in FRAME_IDS:
        geom, counts = load_geom(BOARD / "capture" / f"{frame_id}_depth.u16", board_meta)
        network = list(board_missing_prediction[frame_id])
        for angle_key, records in stored_board.items():
            row = next(x for x in records if int(x["frame"]) == frame_id)
            ri = int(round((float(angle_key) + 45.0) / 9.0))
            stored_val = float(row["epoch13_m"])
            board_stored_delta.append({"frame": frame_id, "angle_deg": float(angle_key),
                                       "stored_m": stored_val, "recomputed_m": network[ri],
                                       "absolute_delta_m": abs(stored_val - network[ri])})
            network[ri] = stored_val
        for i, angle in enumerate(ANGLES):
            ref = float(board_cmp["ray_reference_m"].get(str(int(angle)))) if str(int(angle)) in board_cmp["ray_reference_m"] else None
            samples.append({
                "capture": "standing_board119", "group": "board119", "frame": frame_id,
                "scene": "vertical board; tape-derived body-ray references only at -18/0/+18",
                "ray_index": i, "angle_deg": float(angle), "geometry_m": geom[i],
                "valid_points": counts[i], "network_epoch13_m": network[i],
                "geometry_log2_m": math.log2(geom[i]) if geom[i] is not None else None,
                "network_epoch13_log2_m": math.log2(network[i]),
                "network_minus_geometry_m": network[i] - geom[i] if geom[i] is not None else None,
                "network_minus_geometry_log2": math.log2(network[i] / geom[i]) if geom[i] is not None else None,
                "network_source": "stored comparison.json at -18/0/+18; fixed epoch13 inference for other beams",
                "reference_m": ref,
                "network_error_m": (network[i] - ref) if ref is not None else None,
                "geometry_error_m": (geom[i] - ref) if ref is not None and geom[i] is not None else None,
            })

    fields = list(samples[0].keys())
    with (OUT / "frame_ray_ab.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(samples)

    counts_by_ray = []
    for i, angle in enumerate(ANGLES):
        subset = [r for r in samples if r["ray_index"] == i]
        valid = [r for r in subset if r["geometry_m"] is not None]
        deltas_m = [r["network_minus_geometry_m"] for r in valid]
        deltas_log2 = [r["network_minus_geometry_log2"] for r in valid]
        counts_by_ray.append({"ray_index": i, "angle_deg": float(angle), "valid_frames": len(valid),
                              "total_frames": len(subset), "valid_rate": len(valid) / len(subset),
                              "point_counts": [r["valid_points"] for r in subset],
                              "mean_network_minus_geometry_m": float(np.mean(deltas_m)) if deltas_m else None,
                              "mean_abs_network_geometry_difference_m": float(np.mean(np.abs(deltas_m))) if deltas_m else None,
                              "mean_network_minus_geometry_log2": float(np.mean(deltas_log2)) if deltas_log2 else None})
    full_frames = []
    for capture, group, frame in sorted({(r["capture"], r["group"], r["frame"]) for r in samples}, key=str):
        rows = [r for r in samples if r["capture"] == capture and r["group"] == group and r["frame"] == frame]
        valid = sum(r["geometry_m"] is not None for r in rows)
        full_frames.append({"capture": capture, "group": group, "frame": frame,
                            "valid_rays": valid, "complete_11": valid == 11})

    board_focus = []
    for angle in (-18, 0, 18):
        ri = int(round((angle + 45) / 9))
        rows = [r for r in samples if r["capture"] == "standing_board119" and r["ray_index"] == ri]
        board_focus.append({"angle_deg": angle, "ray_index": ri,
                            "reference_m": rows[0]["reference_m"],
                            "geometry_m_by_frame": [r["geometry_m"] for r in rows],
                            "geometry_error_m_by_frame": [r["geometry_error_m"] for r in rows],
                            "network_epoch13_m_by_frame": [r["network_epoch13_m"] for r in rows],
                            "network_error_m_by_frame": [r["network_error_m"] for r in rows],
                            "valid_points_by_frame": [r["valid_points"] for r in rows]})
    result = {
        "purpose": "Offline diagnostic only; not a safety or control approval.",
        "network_baseline": {"label": "old epoch13 network baseline; not original ABS weight",
                             "path": str(EPOCH13.relative_to(ROOT)),
                             "selected_checkpoint": "300-epoch run's preselected epoch13 best checkpoint; existing outputs used where present"},
        "geometry_method": {"raw_depth_only": True, "input_depth_scale_m_per_unit": 0.001,
                            "mount_candidate_body_m": MOUNT.tolist(), "pitch_down_deg": 3.0,
                            "ray_origin_body_m": [-0.05, 0.0, 0.0], "ray_angles_deg": ANGLES.tolist(),
                            "height_band_body_z_m": list(HEIGHT), "angle_tolerance_deg": ANGLE_TOL_DEG,
                            "minimum_raw_pixels": MIN_POINTS, "valid_depth_m": list(DEPTH_RANGE),
                            "distance": "nearest horizontal along-ray projection of raw backprojected points in angular/height bin",
                            "limitations": ["candidate mount is approximate", "finite pixel bin is not continuous-ray truth", "all depth surfaces count; no semantic/obstacle segmentation"]},
        "frames": 15, "full_geometry_11_frames": sum(x["complete_11"] for x in full_frames),
        "geometry_full_frame_rate": sum(x["complete_11"] for x in full_frames) / len(full_frames),
        "per_ray_effective_rate": counts_by_ray,
        "full_frame_status": full_frames,
        "board_tape_comparison": board_focus,
        "board_stored_vs_recomputed_epoch13": board_stored_delta,
        "ra": {"synchronous_ra_state_available": False,
               "ra_scores_computed": False,
               "reason": "No同期 complete RA observation/switch state is present in these camera captures; geometric vectors are not complete across all 11 rays, so the fixed-vector RA sensitivity path is not run."},
        "samples": samples,
    }
    (OUT / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({"output": str(OUT), "rows": len(samples), "complete_frames": result["full_geometry_11_frames"],
                      "board_saved_prediction_max_delta_m": max(x["absolute_delta_m"] for x in board_stored_delta)}, indent=2))


if __name__ == "__main__":
    main()
