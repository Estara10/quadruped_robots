#!/usr/bin/env python3
"""Generate a reproducible, stratified 60-scene S6-09 static map library.

Reference routes are geometry checks and illustrations only; they are never
passed to the policy. Geometry primitives and MuJoCo/catalog fingerprinting
reuse the S6-08 production-oriented implementation.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import random
import shutil
import sys

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import generate_s6_08_scenes as geom  # noqa: E402

GENERATOR_VERSION = "s6-09-mapgen/1.1"
GEN_DIR = ROOT / "docs/thesis_project/designs/S6-09"
SCENE_DIR = geom.SCENE_DIR
CATALOG = ROOT / "common/abs_scene_catalog.def"
LAYOUTS = GEN_DIR / "layouts.json"
REPORT_DIR = ROOT / "docs/thesis_project/evidence/S6-09"

DENSITIES = {
    # Non-overlapping occupied-area bands; gaps between bands avoid borderline
    # reassignment. Density is descriptive, not a claim of policy difficulty.
    "sparse": {"fraction": (0.060, 0.090), "min_gap_m": 0.28, "size_scale": 0.92},
    "medium": {"fraction": (0.095, 0.120), "min_gap_m": 0.16, "size_scale": 1.04},
    "dense": {"fraction": (0.130, 0.200), "min_gap_m": 0.10, "size_scale": 1.30},
}
DENSITY_ORDER = tuple(DENSITIES)
STRUCTURES = (
    "single_side_bypass",
    "alternating_bypass",
    "offset_gates",
    "route_fork",
    "irregular_cluster",
)
STRUCTURE_LABELS = {
    "single_side_bypass": "单侧绕行",
    "alternating_bypass": "连续交替绕行",
    "offset_gates": "错位通道口",
    "route_fork": "多路线分叉",
    "irregular_cluster": "不规则障碍群",
}
DENSITY_CODES = {"sparse": "s", "medium": "m", "dense": "d"}
STRUCTURE_CODES = {
    "single_side_bypass": "b", "alternating_bypass": "a", "offset_gates": "o",
    "route_fork": "f", "irregular_cluster": "i",
}
SPLIT_CODES = {"development": "d", "held_out": "h"}
AREA_M2 = ((geom.OBSTACLE_BOUNDS[0][1] - geom.OBSTACLE_BOUNDS[0][0]) *
           (geom.OBSTACLE_BOUNDS[1][1] - geom.OBSTACLE_BOUNDS[1][0]))
GATE_XS = (1.30, 3.50, 5.70)
GATE_WIDTH_M = 1.95
GATE_HALF_WIDTH_M = GATE_WIDTH_M / 2
MAX_LAYOUT_ATTEMPTS = 96
MAX_FILL_ATTEMPTS = 1800


def route_length(path: list[tuple[float, float]]) -> float:
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def interpolate_route(points: list[tuple[float, float]], step: float = 0.05) -> list[tuple[float, float]]:
    """Sample a waypoint polyline at no more than `step` metres."""
    result = []
    for a, b in zip(points, points[1:]):
        count = max(1, math.ceil(math.dist(a, b) / step))
        for i in range(count):
            t = i / count
            result.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t))
    result.append(points[-1])
    return result


def gate_wall(rng: random.Random, first_idx: int, x: float, center_y: float,
              gate_width: float = GATE_WIDTH_M, half_x: float = 0.12) -> list[dict]:
    """Build two finite box walls with a measured opening between them."""
    (ymin, ymax) = geom.OBSTACLE_BOUNDS[1]
    low_edge, high_edge = center_y - gate_width / 2, center_y + gate_width / 2
    out = []
    for idx, lo, hi in ((first_idx, ymin, low_edge), (first_idx + 1, high_edge, ymax)):
        if hi - lo <= 0.20:
            raise ValueError("gate opening leaves an undersized wall fragment")
        out.append({
            "name": f"ppt_obstacle_{idx:02d}", "shape": "box",
            "x": round(x, 6), "y": round((lo + hi) / 2, 6),
            "half_x": half_x, "half_y": round((hi - lo) / 2, 6),
            "yaw_rad": 0.0, "height_m": round(rng.uniform(0.32, 0.62), 6),
            "role": "gate_wall",
        })
    return out


def _structure_layout(density: str, structure: str, seed: int,
                      rng: random.Random) -> tuple[list[dict], list[list[tuple[float, float]]], dict]:
    obs: list[dict] = []
    paths: list[list[tuple[float, float]]] = []
    details: dict = {}
    if structure == "single_side_bypass":
        side = rng.choice((-1, 1))
        gate_y = side * 1.40
        # A single side opening spans the lane; the opposite side is closed to
        # the declared search bounds by this barrier.
        (ymin, ymax) = geom.OBSTACLE_BOUNDS[1]
        lo, hi = (ymin, gate_y - GATE_HALF_WIDTH_M) if side > 0 else (gate_y + GATE_HALF_WIDTH_M, ymax)
        obs.append({
            "name": "ppt_obstacle_01", "shape": "box", "x": 3.50,
            "y": round((lo + hi) / 2, 6), "half_x": 0.15,
            "half_y": round((hi - lo) / 2, 6), "yaw_rad": 0.0,
            "height_m": round(rng.uniform(0.32, 0.62), 6), "role": "side_barrier",
        })
        path = interpolate_route([geom.START, (2.60, gate_y), (3.50, gate_y), (4.40, gate_y), geom.GOAL])
        paths = [path]
        actual_opening = (ymax - (gate_y - GATE_HALF_WIDTH_M) if side > 0 else
                          (gate_y + GATE_HALF_WIDTH_M) - ymin)
        details = {"selected_side": "left" if side > 0 else "right",
                   "gate_center_y_m": gate_y,
                   "nominal_opening_gap_m": GATE_WIDTH_M,
                   "actual_gate_opening_width_m": round(actual_opening, 6)}
    elif structure in ("alternating_bypass", "offset_gates"):
        if structure == "alternating_bypass":
            magnitude = rng.uniform(1.30, 1.48)
            centers = [rng.choice((-1, 1)) * magnitude for _ in GATE_XS]
            while centers[0] * centers[1] >= 0 or centers[1] * centers[2] >= 0:
                centers = [rng.choice((-1, 1)) * magnitude for _ in GATE_XS]
        else:
            # Offset openings vary in both direction and lateral position but
            # are not constrained to an alternating pattern.
            centers = [rng.choice((-1.42, -0.95, 0.85, 1.38)) for _ in GATE_XS]
            if len({round(x, 2) for x in centers}) < 3:
                centers = [1.35, 0.12, -1.20]
        for i, (x, center) in enumerate(zip(GATE_XS, centers)):
            wall_half_x = {"sparse": 0.10, "medium": 0.14, "dense": 0.25}[density]
            obs.extend(gate_wall(rng, 1 + 2 * i, x, center, half_x=wall_half_x))
        waypoints = [geom.START]
        for x, center in zip(GATE_XS, centers):
            waypoints.extend(((x - 0.90, center), (x, center), (x + 0.90, center)))
        waypoints.append(geom.GOAL)
        paths = [interpolate_route(waypoints)]
        details = {"gate_centers_y_m": centers, "gate_width_m": GATE_WIDTH_M}
    elif structure == "route_fork":
        # Two islands form two genuinely separate corridors: one reference
        # passes above both, the other below both. The alternatives remain
        # separated by nearly 3 m across both obstacle regions.
        offset = rng.uniform(1.40, 1.44)
        fork_xs = (2.30, 4.70)
        half_x = {"sparse": 0.48, "medium": 0.70, "dense": 0.95}[density]
        half_y = {"sparse": 0.40, "medium": 0.50, "dense": 0.60}[density]
        for index, x in enumerate(fork_xs, 1):
            o = geom.make_obstacle(rng, index, "box", x, 0.0, 1.0)
            o.update({"x": x, "y": 0.0, "half_x": half_x, "half_y": half_y,
                      "yaw_rad": 0.0, "role": "fork_choice_island"})
            obs.append(o)
        paths = [
            interpolate_route([geom.START, (0.55, offset), (2.30, offset),
                               (4.70, offset), (6.45, offset), geom.GOAL]),
            interpolate_route([geom.START, (0.55, -offset), (2.30, -offset),
                               (4.70, -offset), (6.45, -offset), geom.GOAL]),
        ]
        details = {"route_count": 2, "route_lateral_offset_m": offset,
                   "route_01_bypasses": {"ppt_obstacle_01": "upper", "ppt_obstacle_02": "upper"},
                   "route_02_bypasses": {"ppt_obstacle_01": "lower", "ppt_obstacle_02": "lower"},
                   "route_choice_obstacle_centers_x_m": list(fork_xs)}
    elif structure == "irregular_cluster":
        # Deliberately no route-conditioned obstacle placement. All obstacles
        # are sampled in the field first; connectivity is checked afterwards.
        details = {"placement_method": "unstructured_uniform_random_then_connectivity_check"}
    else:
        raise ValueError(f"unknown structure {structure}")
    return obs, paths, details


def _clear_of_paths(candidate: dict, paths: list[list[tuple[float, float]]]) -> bool:
    return all(geom.local_polyline_shape_distance(path, candidate) >= geom.REQUIRED_CENTER_CLEARANCE_M
               for path in paths)


def _candidate(rng: random.Random, density: str, index: int) -> dict:
    scale = DENSITIES[density]["size_scale"]
    shape = rng.choices(("box", "cylinder", "rotated_bar"), (0.50, 0.32, 0.18))[0]
    x = rng.uniform(0.65, 6.35)
    y = rng.uniform(-2.85, 2.85)
    return geom.make_obstacle(rng, index, shape, x, y, scale)


def _path_payload(paths: list[list[tuple[float, float]]], obstacles: list[dict]) -> list[dict]:
    payload = []
    for index, path in enumerate(paths, 1):
        metrics = geom.path_metrics(path, obstacles)
        metrics["reference_path_length_m"] = route_length(path)
        metrics["minimum_actual_obstacle_clearance_m"] = geom.path_clearance(path, obstacles)
        metrics["envelope_margin_excess_width_m"] = 2 * (
            metrics["minimum_actual_obstacle_clearance_m"] - geom.REQUIRED_CENTER_CLEARANCE_M)
        payload.append({"route_id": f"route_{index:02d}",
                        "reference_path_m": [list(p) for p in path],
                        "path_check": metrics})
    return payload


def _choose_irregular_path(obstacles: list[dict]) -> list[tuple[float, float]] | None:
    raw = geom.grid_path(obstacles)
    if raw is None:
        return None
    path = geom.simplify_path(raw, obstacles)
    if not all(geom.segment_clear(a, b, obstacles) for a, b in zip(path, path[1:])):
        return None
    return path


def _check_structure(structure: str, paths: list[list[tuple[float, float]]], details: dict) -> bool:
    if structure == "single_side_bypass":
        side = 1 if details["selected_side"] == "left" else -1
        return all(side * p[1] >= -1e-8 for p in paths[0] if 1.0 < p[0] < 6.0)
    if structure == "alternating_bypass":
        ys = details["gate_centers_y_m"]
        return ys[0] * ys[1] < 0 and ys[1] * ys[2] < 0
    if structure == "offset_gates":
        ys = details["gate_centers_y_m"]
        return max(ys) - min(ys) >= 1.5 and len(set(ys)) >= 3
    if structure == "route_fork":
        upper, lower = paths
        def y_at(path, x):
            for a, b in zip(path, path[1:]):
                if a[0] <= x <= b[0]:
                    t = (x - a[0]) / (b[0] - a[0])
                    return a[1] + t * (b[1] - a[1])
            raise ValueError(f"x={x} outside monotone reference route")
        return (len(paths) == 2 and
                all(abs(y_at(upper, x) - y_at(lower, x)) >= 2.5
                    for x in (2.30, 4.70)) and
                route_length(upper) > 7.25 and route_length(lower) > 7.25)
    return structure == "irregular_cluster"


def generate_scene(density: str, structure: str, split: str, replica: int) -> dict:
    if split not in ("development", "held_out") or replica not in (1, 2):
        raise ValueError("split must be development/held_out and replica must be 1 or 2")
    d_i, s_i = DENSITY_ORDER.index(density), STRUCTURES.index(structure)
    # Separate numeric seed namespaces prevent accidental dev/holdout reuse.
    seed = (6_900_000 if split == "development" else 7_900_000) + d_i * 100_000 + s_i * 1_000 + replica
    scene_slug = f"random_s6_09_{density}_{structure}_{'dev' if split == 'development' else 'holdout'}_{replica:02d}"
    # The C++ collision snapshot has a fixed 32-byte scene_id field. Keep a
    # short, injective identity in that field while preserving the descriptive
    # filename and preview slug. Codes are documented in MAP_RULES.md.
    scene_id = (f"s609_{DENSITY_CODES[density]}{STRUCTURE_CODES[structure]}_"
                f"{SPLIT_CODES[split]}{replica:02d}")
    area_lo, area_hi = DENSITIES[density]["fraction"]
    gap = DENSITIES[density]["min_gap_m"]
    for attempt in range(MAX_LAYOUT_ATTEMPTS):
        rng = random.Random(seed + attempt * 10_000_019)
        obstacles, paths, details = _structure_layout(density, structure, seed, rng)
        placed = len(obstacles)
        if structure == "irregular_cluster":
            paths = []
        # Bounded random fill; structured routes are protected geometrically.
        failed = False
        for _ in range(MAX_FILL_ATTEMPTS):
            area_fraction = sum(geom.obstacle_area(o) for o in obstacles) / AREA_M2
            if area_fraction >= area_lo:
                break
            candidate = _candidate(rng, density, placed + 1)
            if not geom.in_bounds(candidate) or not geom.clear_start_goal(candidate):
                continue
            if any(geom.shape_distance(candidate, old) < gap for old in obstacles):
                continue
            if paths and not _clear_of_paths(candidate, paths):
                continue
            obstacles.append(candidate)
            placed += 1
        else:
            failed = True
        if failed:
            last = "area_lower_bound_not_reached_with_bounded_fill"
            continue
        area_fraction = sum(geom.obstacle_area(o) for o in obstacles) / AREA_M2
        if not area_lo <= area_fraction < area_hi:
            last = f"area_fraction_outside_band:{area_fraction:.5f}"
            continue
        if not all(geom.in_bounds(o) and geom.clear_start_goal(o) for o in obstacles):
            last = "bounds_or_start_goal_clearance"
            continue
        if not geom.no_overlap_and_spacing(obstacles, gap):
            last = "overlap_or_minimum_spacing"
            continue
        if structure == "irregular_cluster":
            route = _choose_irregular_path(obstacles)
            if route is None:
                last = "random_cluster_no_center_path_on_5cm_grid"
                continue
            paths = [route]
        else:
            if any(not all(geom.segment_clear(a, b, obstacles)
                           for a, b in zip(path, path[1:])) for path in paths):
                last = "reference_route_inflated_clearance_failed"
                continue
        if not _check_structure(structure, paths, details):
            last = "route_structure_invariant_failed"
            continue
        blockers = geom.line_blockers(obstacles)
        if not blockers:
            last = "direct_start_goal_line_not_blocked"
            continue
        # Independently require generic start-to-goal connectivity for every map.
        if geom.grid_path(obstacles) is None:
            last = "no_connected_center_path_on_5cm_grid"
            continue
        path_records = _path_payload(paths, obstacles)
        if any(not rec["path_check"]["connected"] or
               rec["path_check"]["minimum_actual_obstacle_clearance_m"] < geom.REQUIRED_CENTER_CLEARANCE_M
               for rec in path_records):
            last = "route_payload_check_failed"
            continue
        pair_gaps = [geom.shape_distance(obstacles[i], obstacles[j])
                     for i in range(len(obstacles)) for j in range(i + 1, len(obstacles))]
        actual_two_sided_gaps = details.get("actual_gate_opening_width_m", details.get("gate_width_m"))
        return {
            "scene_id": scene_id,
            "scene_slug": scene_slug,
            "model_name": scene_slug,
            "replica": replica,
            "filename": f"scene_{scene_slug}.xml",
            "preview": f"{scene_slug}.png",
            "generator_version": GENERATOR_VERSION,
            "generation_seed": seed,
            "generation_attempt": attempt + 1,
            "run_seed": None,
            "split": split,
            "density": density,
            "route_structure": structure,
            "route_structure_label_zh": STRUCTURE_LABELS[structure],
            "structure_parameters": details,
            "density_definition": {
                "obstacle_footprint_area_fraction": area_fraction,
                "target_area_fraction_exclusive": [area_lo, area_hi],
                "minimum_obstacle_spacing_m": min(pair_gaps) if pair_gaps else None,
                "configured_minimum_spacing_m": gap,
                "minimum_route_centerline_corridor_width_m": min(
                    rec["path_check"]["minimum_centerline_corridor_width_m"] for rec in path_records),
                "reference_route_lengths_m": [rec["path_check"]["reference_path_length_m"] for rec in path_records],
                "detour_ratios": [rec["path_check"]["reference_path_length_m"] / math.dist(geom.START, geom.GOAL)
                                  for rec in path_records],
            },
            "obstacle_bounds_m": {"x": list(geom.OBSTACLE_BOUNDS[0]), "y": list(geom.OBSTACLE_BOUNDS[1])},
            "path_search_bounds_m": {"x": list(geom.PATH_BOUNDS[0]), "y": list(geom.PATH_BOUNDS[1])},
            "start_m": list(geom.START), "goal_m": list(geom.GOAL),
            "obstacle_count": len(obstacles), "obstacles": obstacles,
            "routes": path_records,
            "direct_line_blocked": True,
            "direct_line_blocking_obstacles": blockers,
            "path_check_settings": {
                "grid_resolution_m": geom.GRID_M,
                "continuous_check_step_m": geom.PATH_CHECK_STEP_M,
                "go2_planar_envelope_radius_m": geom.GO2_ENVELOPE_RADIUS_M,
                "engineering_margin_m": geom.ENGINEERING_MARGIN_M,
                "required_centerline_clearance_m": geom.REQUIRED_CENTER_CLEARANCE_M,
                "start_goal_clearance_m": geom.START_GOAL_CLEARANCE_M,
                "route_centerline_width_definition": "2 * minimum obstacle clearance along reference route; not actual obstacle-to-obstacle gap",
                "actual_gate_opening_width_m": actual_two_sided_gaps,
            },
        }
    raise RuntimeError(f"{scene_id}: bounded generation failed after {MAX_LAYOUT_ATTEMPTS} attempts; last={last}")


def font(size: int):
    try:
        return ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", size)
    except OSError:
        return ImageFont.load_default()


def render_scene(scene: dict) -> Image.Image:
    image = Image.new("RGB", (1200, 900), "white")
    d = ImageDraw.Draw(image)
    xb, yb = geom.OBSTACLE_BOUNDS
    plot = (90, 115, 1110, 790)
    sx, sy = (plot[2] - plot[0]) / (xb[1] - xb[0]), (plot[3] - plot[1]) / (yb[1] - yb[0])
    def pix(p):
        return (round(plot[0] + (p[0] - xb[0]) * sx), round(plot[3] - (p[1] - yb[0]) * sy))
    d.text((90, 20), f"{scene['scene_slug']} · {scene['density']} · {STRUCTURE_LABELS[scene['route_structure']]} · {scene['split']} · seed {scene['generation_seed']}", fill="#17212b", font=font(20))
    dd = scene["density_definition"]
    d.text((90, 55), f"占地 {dd['obstacle_footprint_area_fraction']:.1%}（互斥分档） · 最小障碍间距 {dd['minimum_obstacle_spacing_m']:.2f} m · 障碍 {scene['obstacle_count']} 个", fill="#475569", font=font(15))
    d.rectangle(plot, outline="#46515b", width=2)
    # Draw the conservative robot-center envelope behind the reference routes.
    colors = ("#087e83", "#7c3aed", "#d97706")
    for index, record in enumerate(scene["routes"]):
        path = [tuple(p) for p in record["reference_path_m"]]
        px = [pix(p) for p in path]
        width = max(2, round(2 * geom.REQUIRED_CENTER_CLEARANCE_M * (sx + sy) / 2))
        d.line(px, fill="#d9efec", width=width, joint="curve")
        d.line(px, fill=colors[index % len(colors)], width=4, joint="curve")
    # Start/goal direct line.
    p0, p1 = pix(geom.START), pix(geom.GOAL)
    distance = math.dist(p0, p1)
    for off in range(0, int(distance), 28):
        t0, t1 = off / distance, min(off + 16, distance) / distance
        d.line((round(p0[0] + (p1[0]-p0[0])*t0), round(p0[1] + (p1[1]-p0[1])*t0),
                round(p0[0] + (p1[0]-p0[0])*t1), round(p0[1] + (p1[1]-p0[1])*t1)), fill="#e28a23", width=3)
    for obstacle in scene["obstacles"]:
        color = "#bb3c37" if obstacle["name"] in scene["direct_line_blocking_obstacles"] else "#59636d"
        if geom.is_box(obstacle):
            pts = [pix(p) for p in geom.box_vertices(obstacle)]
            d.polygon(pts, fill=color, outline="#25313c")
        else:
            cx, cy = pix(geom.geom_xy(obstacle)); r = obstacle["radius"]
            d.ellipse((round(cx-r*sx), round(cy-r*sy), round(cx+r*sx), round(cy+r*sy)), fill=color, outline="#25313c", width=2)
    for point, label, color in ((geom.START, "START", "#21864a"), (geom.GOAL, "GOAL", "#2357a5")):
        x, y = pix(point); d.ellipse((x-9, y-9, x+9, y+9), fill=color, outline="white", width=2)
        d.text((x+10, y-18), label, fill=color, font=font(14))
    d.text((90, 820), "浅青带：Go2 0.65 m 保守包络 + 0.10 m 工程余量；彩线仅为几何参考路线，不是机器人轨迹/策略输入。", fill="#334155", font=font(15))
    d.text((90, 850), f"边界 x=[{xb[0]:g},{xb[1]:g}] m, y=[{yb[0]:g},{yb[1]:g}] m；比例 x={1/sx:.4f} m/px, y={1/sy:.4f} m/px", fill="#334155", font=font(13))
    return image


def overview(images: list[tuple[dict, Image.Image]], split: str) -> Image.Image:
    # Five structure columns × six rows (two replicas across each density).
    thumb_w, thumb_h = 360, 270
    margin, header_h = 18, 56
    canvas = Image.new("RGB", (margin + 5 * (thumb_w + margin), header_h + 6 * (thumb_h + margin)), "#f2f4f7")
    draw = ImageDraw.Draw(canvas)
    draw.text((margin, 12), f"S6-09 {split} maps · 30 geometry-only previews", fill="#17212b", font=font(22))
    by_key = {(s["density"], s["route_structure"], int(s["replica"])): im for s, im in images}
    row = 0
    for density in DENSITY_ORDER:
        for replica in (1, 2):
            for col, structure in enumerate(STRUCTURES):
                x = margin + col * (thumb_w + margin)
                y = header_h + row * (thumb_h + margin)
                resampling = getattr(Image, "Resampling", Image)
                im = by_key[(density, structure, replica)].resize((thumb_w, thumb_h), resampling.LANCZOS)
                canvas.paste(im, (x, y))
            row += 1
    return canvas


def _catalog_row(scene: dict, xml_path: Path) -> dict:
    return geom.catalog_row(scene, xml_path)


def register_catalog(rows: list[dict]) -> None:
    text = CATALOG.read_text(encoding="utf-8")
    old_lines = text.splitlines()
    lines = []
    index = 0
    while index < len(old_lines):
        if old_lines[index].startswith("ABS_SCENE(") and (
                '"scene_random_s6_09_' in old_lines[index] or '"s609_' in old_lines[index]):
            index += 3
            continue
        lines.append(old_lines[index])
        index += 1
    for row in rows:
        lines.extend([
            f'ABS_SCENE("{row["filename"]}", "{row["scene_id"]}", {row["obstacle_count"]}, kNamedPptObstacles,',
            f'          "{row["root_sha256"]}", "{row["closure_sha256"]}",',
            f'          "{row["model_fingerprint"]}")',
        ])
    CATALOG.write_text("\n".join(lines) + "\n", encoding="utf-8")


def generate_all() -> list[dict]:
    scenes = []
    for density in DENSITY_ORDER:
        for structure in STRUCTURES:
            for split, reps in (("development", (1, 2)), ("held_out", (1, 2))):
                for replica in reps:
                    scene = generate_scene(density, structure, split, replica)
                    scenes.append(scene)
                    print(f"generated {len(scenes):02d}/60 {scene['scene_id']} seed={scene['generation_seed']} "
                          f"attempt={scene['generation_attempt']} area={scene['density_definition']['obstacle_footprint_area_fraction']:.4f}",
                          flush=True)
    return scenes


def metadata(scenes: list[dict]) -> dict:
    return {
        "schema_version": 1,
        "generator_version": GENERATOR_VERSION,
        "scene_count": len(scenes),
        "stratification": {"density": list(DENSITY_ORDER), "route_structure": list(STRUCTURES),
                           "replicas_per_cell": 4, "development_per_cell": 2,
                           "held_out_per_cell": 2},
        "development_seed_namespace": "6900000-7199999",
        "held_out_seed_namespace": "7900000-8199999",
        "generation_seeds": {scene["scene_id"]: scene["generation_seed"] for scene in scenes},
        "run_seeds": "separate from generation seeds; no robot run is authorized in S6-09",
        "density_bands_exclusive": {name: list(spec["fraction"]) for name, spec in DENSITIES.items()},
        "bounds_m": {"obstacle": {"x": list(geom.OBSTACLE_BOUNDS[0]), "y": list(geom.OBSTACLE_BOUNDS[1])},
                     "path_search": {"x": list(geom.PATH_BOUNDS[0]), "y": list(geom.PATH_BOUNDS[1])}},
        "start_m": list(geom.START), "goal_m": list(geom.GOAL),
        "go2_planar_envelope_radius_m": geom.GO2_ENVELOPE_RADIUS_M,
        "engineering_margin_m": geom.ENGINEERING_MARGIN_M,
        "required_centerline_clearance_m": geom.REQUIRED_CENTER_CLEARANCE_M,
        "envelope_basis": "inherited S6-08 conservatism; go2.xml approximate fully-extended leg reach; not gait validated",
        "start_goal_clearance_m": geom.START_GOAL_CLEARANCE_M,
        "obstacle_height_range_m": [0.32, 0.62],
        "grid_resolution_m": geom.GRID_M,
        "continuous_path_check_step_m": geom.PATH_CHECK_STEP_M,
        "direct_line_blocked_count": sum(scene["direct_line_blocked"] for scene in scenes),
        "scenes": scenes,
    }


def write_outputs(scenes: list[dict]) -> None:
    GEN_DIR.mkdir(parents=True, exist_ok=True)
    SCENE_DIR.mkdir(parents=True, exist_ok=True)
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    rendered = []
    for scene in scenes:
        xml_path = SCENE_DIR / scene["filename"]
        xml_path.write_text(geom.render_xml(scene), encoding="utf-8")
        mujoco_model = geom.mujoco.MjModel.from_xml_path(str(xml_path))
        del mujoco_model
        image = render_scene(scene)
        image.save(GEN_DIR / scene["preview"], format="PNG", optimize=True)
        rendered.append((scene, image))
    LAYOUTS.write_text(json.dumps(metadata(scenes), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    # Representative artwork: choose first development scene for each structure.
    for structure in STRUCTURES:
        item = next((scene, image) for scene, image in rendered
                    if scene["route_structure"] == structure and scene["density"] == "medium" and scene["split"] == "development")
        item[1].save(GEN_DIR / f"representative_{structure}.png", format="PNG", optimize=True)
    for split in ("development", "held_out"):
        overview([item for item in rendered if item[0]["split"] == split], split).save(
            GEN_DIR / f"overview_{split}_30.png", format="PNG", optimize=True)
    rows = [_catalog_row(scene, SCENE_DIR / scene["filename"]) for scene in scenes]
    register_catalog(rows)


def verify_outputs(scenes: list[dict]) -> list[dict]:
    if not LAYOUTS.is_file():
        raise RuntimeError(f"missing layout source {LAYOUTS}")
    old = json.loads(LAYOUTS.read_text(encoding="utf-8"))
    expected_doc = metadata(scenes)
    if old != expected_doc:
        raise RuntimeError("layout metadata differs from deterministic regeneration")
    scene_ids = [scene["scene_id"] for scene in scenes]
    if len(set(scene_ids)) != 60 or any(
            not scene_id.isascii() or len(scene_id.encode("ascii")) >= 32
            for scene_id in scene_ids):
        raise RuntimeError("S6-09 internal scene IDs must be unique and NUL-terminated in 32-byte field")
    scene_catalog = __import__("scripts.abs_scene", fromlist=["scene_catalog"]).scene_catalog()
    catalog_ids = {spec.scene_id for spec in scene_catalog.values()}
    if len(catalog_ids) != len(scene_catalog) or not set(scene_ids).issubset(catalog_ids):
        raise RuntimeError("S6-09 short IDs are not unique in the complete production scene catalog")
    rows = []
    rendered = []
    for scene in scenes:
        xml_path = SCENE_DIR / scene["filename"]
        expected_xml = geom.render_xml(scene)
        if not xml_path.is_file() or xml_path.read_text(encoding="utf-8") != expected_xml:
            raise RuntimeError(f"XML differs from deterministic regeneration: {scene['filename']}")
        image_path = GEN_DIR / scene["preview"]
        expected_image = render_scene(scene)
        if not image_path.is_file() or Image.open(image_path).convert("RGB").tobytes() != expected_image.tobytes():
            raise RuntimeError(f"preview differs from deterministic regeneration: {scene['preview']}")
        rendered.append((scene, expected_image))
        model = geom.mujoco.MjModel.from_xml_path(str(xml_path))
        obstacle_ids = []
        for geom_id in range(model.ngeom):
            name = geom.mujoco.mj_id2name(model, geom.mujoco.mjtObj.mjOBJ_GEOM, geom_id)
            if name and name.startswith("ppt_obstacle_"):
                obstacle_ids.append(name)
        if len(obstacle_ids) != scene["obstacle_count"] or set(obstacle_ids) != {o["name"] for o in scene["obstacles"]}:
            raise RuntimeError(f"compiled geom set mismatch: {scene['scene_id']}")
        spec = __import__("scripts.abs_scene", fromlist=["resolve_scene"])
        resolved = spec.resolve_scene(scene["filename"])
        if resolved.scene_id != scene["scene_id"] or resolved.obstacle_count != len(obstacle_ids):
            raise RuntimeError(f"Python production scene resolution mismatch: {scene['scene_id']}")
        rows.append(_catalog_row(scene, xml_path))
    for structure in STRUCTURES:
        scene, expected_image = next((s, im) for s, im in rendered
                                     if s["route_structure"] == structure and
                                     s["density"] == "medium" and s["split"] == "development")
        rep_path = GEN_DIR / f"representative_{structure}.png"
        if not rep_path.is_file() or Image.open(rep_path).convert("RGB").tobytes() != expected_image.tobytes():
            raise RuntimeError(f"representative preview differs: {structure}")
    for split in ("development", "held_out"):
        expected_overview = overview([item for item in rendered if item[0]["split"] == split], split)
        overview_path = GEN_DIR / f"overview_{split}_30.png"
        if not overview_path.is_file() or Image.open(overview_path).convert("RGB").tobytes() != expected_overview.tobytes():
            raise RuntimeError(f"overview differs: {split}")
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--generate", action="store_true")
    mode.add_argument("--check", action="store_true")
    args = parser.parse_args()
    scenes = generate_all()
    if len(scenes) != 60:
        raise RuntimeError(f"expected 60 scenes, got {len(scenes)}")
    if args.generate:
        write_outputs(scenes)
    rows = verify_outputs(scenes)
    print(f"S6-09 {GENERATOR_VERSION}: {len(scenes)} scenes, 30 development, 30 held-out")
    print(f"direct_start_goal_line_blocked={sum(s['direct_line_blocked'] for s in scenes)}/60")
    print(f"production_scene_bindings_verified={len(rows)}/60")
    print(f"catalog_rows={len(rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
