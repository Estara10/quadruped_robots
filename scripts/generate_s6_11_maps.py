#!/usr/bin/env python3
"""Generate the S6-11 local-obstacle revision of the S6-09 map library.

The old S6-09 files and catalog entries remain intact. This script writes a
versioned 60-scene manifest made from the retained 36 scenes and 24 new local
layouts. Runtime model fingerprints are supplied only by the C++ MuJoCo
production helper, never by this Python generator.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import random
import re
import sys
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import generate_s6_08_scenes as geom  # noqa: E402

GEN_DIR = ROOT / "docs/thesis_project/designs/S6-11"
EVIDENCE_DIR = ROOT / "docs/thesis_project/evidence/S6-11"
SCENE_DIR = geom.SCENE_DIR
CATALOG = ROOT / "common/abs_scene_catalog.def"
OLD_LAYOUT = ROOT / "docs/thesis_project/designs/S6-09/layouts.json"
LAYOUTS = GEN_DIR / "layouts_s6_11.json"
LEGACY_EXCLUDED = GEN_DIR / "legacy_24_excluded.json"
MANIFEST = GEN_DIR / "scene_manifest_s6_11.json"

GENERATOR_VERSION = "s6-11-local-obstacles/1.0"
DENSITIES = {
    "sparse": {"fraction": (0.060, 0.090), "min_gap_m": 0.28},
    "medium": {"fraction": (0.095, 0.120), "min_gap_m": 0.16},
    "dense": {"fraction": (0.130, 0.200), "min_gap_m": 0.10},
}
DENSITY_ORDER = ("sparse", "medium", "dense")
NEW_STRUCTURES = ("short_local_bypass", "forward_offset_short_obstacles")
ALL_STRUCTURES = (
    "single_side_bypass", "alternating_bypass", "offset_gates",
    *NEW_STRUCTURES,
)
NEW_LABELS = {
    "short_local_bypass": "短障碍局部绕行",
    "forward_offset_short_obstacles": "前向错列短障碍",
}
OLD_LABELS = {
    "single_side_bypass": "单侧绕行",
    "alternating_bypass": "连续交替绕行",
    "offset_gates": "错位通道口",
}
DENSITY_CODE = {"sparse": "s", "medium": "m", "dense": "d"}
SPLIT_CODE = {"development": "d", "held_out": "h"}
NEW_STRUCTURE_CODE = {"short_local_bypass": "l", "forward_offset_short_obstacles": "q"}
AREA_M2 = ((geom.OBSTACLE_BOUNDS[0][1] - geom.OBSTACLE_BOUNDS[0][0]) *
           (geom.OBSTACLE_BOUNDS[1][1] - geom.OBSTACLE_BOUNDS[1][0]))
MAX_LAYOUT_ATTEMPTS = 96
MAX_FILL_ATTEMPTS = 2400
LOCAL_MAX_PROJECTED_LENGTH_M = 0.85
MAX_REFERENCE_LATERAL_OFFSET_M = 1.55
MAX_LOCAL_HEADING_DEG = 50.0
FILL_CLEARANCE_M = geom.REQUIRED_CENTER_CLEARANCE_M + 0.04
SEED_BASE = {"development": 9_110_000, "held_out": 9_210_000}


def json_read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def support_x(o: dict) -> float:
    if o["shape"] == "cylinder":
        return float(o["radius"])
    a = float(o.get("yaw_rad", 0.0))
    return abs(math.cos(a)) * float(o["half_x"]) + abs(math.sin(a)) * float(o["half_y"])


def support_y(o: dict) -> float:
    if o["shape"] == "cylinder":
        return float(o["radius"])
    a = float(o.get("yaw_rad", 0.0))
    return abs(math.sin(a)) * float(o["half_x"]) + abs(math.cos(a)) * float(o["half_y"])


def projected_length_x(o: dict) -> float:
    return 2.0 * support_x(o)


def projected_length_y(o: dict) -> float:
    return 2.0 * support_y(o)


def projected_area(o: dict) -> float:
    return geom.obstacle_area(o)


def make_short_obstacle(rng: random.Random, index: int, x: float, y: float,
                        preferred: str | None = None) -> dict:
    shape = preferred or rng.choices(("box", "cylinder", "rotated_bar"), (0.48, 0.30, 0.22))[0]
    height = round(rng.uniform(0.32, 0.62), 6)
    if shape == "cylinder":
        return {"name": f"ppt_obstacle_{index:02d}", "shape": shape,
                "x": round(x, 6), "y": round(y, 6),
                "radius": round(rng.uniform(0.22, 0.29), 6),
                "yaw_rad": 0.0, "height_m": height, "role": "short_local_obstacle"}
    if shape == "rotated_bar":
        result = {"name": f"ppt_obstacle_{index:02d}", "shape": shape,
                  "x": round(x, 6), "y": round(y, 6),
                  "half_x": round(rng.uniform(0.27, 0.37), 6),
                  "half_y": round(rng.uniform(0.13, 0.20), 6),
                  "yaw_rad": round(rng.uniform(-0.20, 0.20), 6),
                  "height_m": height, "role": "short_local_obstacle"}
    else:
        result = {"name": f"ppt_obstacle_{index:02d}", "shape": "box",
                  "x": round(x, 6), "y": round(y, 6),
                  "half_x": round(rng.uniform(0.25, 0.35), 6),
                  "half_y": round(rng.uniform(0.20, 0.28), 6),
                  "yaw_rad": round(rng.uniform(-0.22, 0.22), 6),
                  "height_m": height, "role": "short_local_obstacle"}
    return result


def grid_forward_path(obstacles: list[dict], step: float = geom.GRID_M) -> list[tuple[float, float]] | None:
    """Find a forward-monotone route on the existing 5 cm inflated grid.

    Each x slice permits at most one y grid step, so every raw segment is
    forward-going and has <=45 degree heading. Actual shapes are checked again
    continuously after path simplification.
    """
    x0, x1 = geom.PATH_BOUNDS[0]
    y0, y1 = -1.8, 1.8
    nx = round((x1 - x0) / step) + 1
    ny = round((y1 - y0) / step) + 1
    ys = [y0 + j * step for j in range(ny)]
    xs = [x0 + i * step for i in range(nx)]
    safety = geom.REQUIRED_CENTER_CLEARANCE_M + step * math.sqrt(2.0) / 2.0
    free = [[all(geom.point_shape_distance((x, y), o) >= safety for o in obstacles)
             for y in ys] for x in xs]
    sy = round((0.0 - y0) / step)
    if not free[0][sy]:
        return None
    # State includes the last lateral step; transition cost gently discourages
    # abrupt steering reversals without changing the required robot envelope.
    inf = float("inf")
    costs = [[[inf] * 3 for _ in range(ny)] for _ in range(nx)]
    parents: list[list[list[tuple[int, int] | None]]] = [
        [[None] * 3 for _ in range(ny)] for _ in range(nx)]
    for direction in (-1, 0, 1):
        costs[0][sy][direction + 1] = 0.0
    for ix in range(nx - 1):
        for iy, y in enumerate(ys):
            if not free[ix][iy]:
                continue
            for prev in (-1, 0, 1):
                old = costs[ix][iy][prev + 1]
                if not math.isfinite(old):
                    continue
                for dy in (-1, 0, 1):
                    jy = iy + dy
                    if not 0 <= jy < ny or not free[ix + 1][jy]:
                        continue
                    turn_penalty = 0.018 * abs(dy - prev)
                    lateral_penalty = 0.004 * abs(ys[jy])
                    new = old + math.hypot(step, dy * step) + turn_penalty + lateral_penalty
                    if new < costs[ix + 1][jy][dy + 1]:
                        costs[ix + 1][jy][dy + 1] = new
                        parents[ix + 1][jy][dy + 1] = (iy, prev + 1)
    gy = round((0.0 - y0) / step)
    candidates = [(costs[-1][gy][d], d) for d in range(3)]
    cost, direction_index = min(candidates)
    if not math.isfinite(cost):
        return None
    iy, di = gy, direction_index
    nodes = [(nx - 1, iy)]
    for ix in range(nx - 1, 0, -1):
        parent = parents[ix][iy][di]
        if parent is None:
            return None
        iy, di = parent
        nodes.append((ix - 1, iy))
    nodes.reverse()
    raw = [(xs[ix], ys[iy]) for ix, iy in nodes]
    path = geom.simplify_path(raw, obstacles)
    if any(b[0] <= a[0] for a, b in zip(path, path[1:])):
        return None
    if not all(geom.segment_clear(a, b, obstacles) for a, b in zip(path, path[1:])):
        return None
    if max(abs(y) for _, y in path) > MAX_REFERENCE_LATERAL_OFFSET_M:
        return None
    headings = [abs(math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])))
                for a, b in zip(path, path[1:])]
    if headings and max(headings) > MAX_LOCAL_HEADING_DEG:
        return None
    return path


def base_obstacles(rng: random.Random, density: str, structure: str) -> list[dict]:
    if structure == "short_local_bypass":
        count = 2 if density == "sparse" else 3
        centers = [2.05, 4.95] if count == 2 else [1.65, 3.50, 5.35]
        jitter = rng.uniform(-0.10, 0.10)
        return [make_short_obstacle(rng, i + 1, x + jitter, 0.0)
                for i, x in enumerate(centers)]
    centers_x = (1.65, 3.50, 5.35)
    centers_y = [rng.choice((-0.76, -0.68)), rng.choice((0.68, 0.76)),
                 rng.choice((-0.76, -0.68))]
    obstacles = []
    for i, (x, y) in enumerate(zip(centers_x, centers_y), 1):
        o = make_short_obstacle(rng, i, x + rng.uniform(-0.08, 0.08), y)
        # This class makes several small local centerline adjustments. Keep
        # each footprint on the selected side of the forward ray bundle.
        if o["shape"] == "cylinder":
            o["radius"] = min(o["radius"], 0.25)
        else:
            o["half_y"] = min(o["half_y"], 0.23)
        obstacles.append(o)
    return obstacles


def within_density(obstacles: list[dict], density: str) -> bool:
    area_fraction = sum(projected_area(o) for o in obstacles) / AREA_M2
    low, high = DENSITIES[density]["fraction"]
    return low <= area_fraction < high


def generate_new_scene(density: str, structure: str, split: str, replica: int) -> dict:
    dindex = DENSITY_ORDER.index(density)
    sindex = NEW_STRUCTURES.index(structure)
    seed = SEED_BASE[split] + dindex * 10_000 + sindex * 1_000 + replica
    stem = (f"random_s6_11_{density}_{structure}_"
            f"{'dev' if split == 'development' else 'holdout'}_{replica:02d}")
    scene_id = f"s611_{DENSITY_CODE[density]}{NEW_STRUCTURE_CODE[structure]}_{SPLIT_CODE[split]}{replica:02d}"
    low, high = DENSITIES[density]["fraction"]
    min_gap = DENSITIES[density]["min_gap_m"]
    last_reason = "bounded_resampling_exhausted"
    for attempt in range(MAX_LAYOUT_ATTEMPTS):
        rng = random.Random(seed + attempt * 1_000_003)
        obstacles = base_obstacles(rng, density, structure)
        if not all(geom.in_bounds(o) and geom.clear_start_goal(o) for o in obstacles):
            last_reason = "base_bounds_or_endpoint_clearance"
            continue
        if not geom.no_overlap_and_spacing(obstacles, min_gap):
            last_reason = "base_obstacle_spacing"
            continue
        path = grid_forward_path(obstacles)
        if path is None:
            last_reason = "base_layout_has_no_forward_local_route"
            continue
        placement_failed = False
        fill_attempts = 0
        for fill_attempts in range(1, MAX_FILL_ATTEMPTS + 1):
            fraction = sum(projected_area(o) for o in obstacles) / AREA_M2
            if fraction >= low:
                break
            idx = len(obstacles) + 1
            shape = rng.choices(("box", "cylinder", "rotated_bar"), (0.48, 0.30, 0.22))[0]
            candidate = make_short_obstacle(rng, idx, rng.uniform(0.55, 6.45),
                                            rng.uniform(-2.88, 2.88), shape)
            candidate["role"] = "density_fill_off_reference_route"
            if not geom.in_bounds(candidate) or not geom.clear_start_goal(candidate):
                continue
            if projected_length_x(candidate) > LOCAL_MAX_PROJECTED_LENGTH_M or projected_length_y(candidate) > 0.75:
                continue
            if any(geom.shape_distance(candidate, old) < min_gap for old in obstacles):
                continue
            if geom.local_polyline_shape_distance(path, candidate, FILL_CLEARANCE_M) < FILL_CLEARANCE_M:
                continue
            if fraction + projected_area(candidate) / AREA_M2 >= high:
                continue
            obstacles.append(candidate)
        else:
            placement_failed = True
        if placement_failed or not within_density(obstacles, density):
            last_reason = "bounded_fill_could_not_reach_disjoint_density_band"
            continue
        if not all(geom.in_bounds(o) and geom.clear_start_goal(o) for o in obstacles):
            last_reason = "bounds_or_endpoint_clearance"
            continue
        if not geom.no_overlap_and_spacing(obstacles, min_gap):
            last_reason = "overlap_or_density_minimum_spacing"
            continue
        if not all(geom.segment_clear(a, b, obstacles) for a, b in zip(path, path[1:])):
            last_reason = "reference_path_clearance"
            continue
        blockers = geom.line_blockers(obstacles)
        area_fraction = sum(projected_area(o) for o in obstacles) / AREA_M2
        pair_gaps = [geom.shape_distance(obstacles[i], obstacles[j])
                     for i in range(len(obstacles)) for j in range(i + 1, len(obstacles))]
        metrics = geom.path_metrics(path, obstacles)
        max_forward_extent = max(projected_length_x(o) for o in obstacles)
        max_lateral_extent = max(projected_length_y(o) for o in obstacles)
        if max_forward_extent > LOCAL_MAX_PROJECTED_LENGTH_M + 1e-9 or max_lateral_extent > 0.75 + 1e-9:
            last_reason = "obstacle_exceeds_local_footprint_limit"
            continue
        route = {
            "route_id": "forward_local_reference_only",
            "reference_path_m": [list(p) for p in path],
            "path_check": metrics,
        }
        # Summarize axis progress and the local passage point around every
        # primary obstacle. These are geometric descriptors, not policy input.
        primary = obstacles[:(2 if structure == "short_local_bypass" and density == "sparse" else 3)]
        passes = []
        for o in primary:
            ox, oy = float(o["x"]), float(o["y"])
            nearest = min(path, key=lambda p: abs(p[0] - ox))
            before = max((p for p in path if p[0] <= ox), key=lambda p: p[0], default=path[0])
            after = min((p for p in path if p[0] >= ox), key=lambda p: p[0], default=path[-1])
            passes.append({"obstacle": o["name"], "center_m": [ox, oy],
                           "reference_near_point_m": list(nearest),
                           "entry_x_m": before[0], "exit_x_m": after[0],
                           "positive_goalward_progress": after[0] >= before[0]})
        scene = {
            "scene_id": scene_id,
            "scene_slug": stem,
            "model_name": stem,
            "filename": f"scene_{stem}.xml",
            "preview": f"{stem}.png",
            "generator_version": GENERATOR_VERSION,
            "generation_seed": seed,
            "generation_attempt": attempt + 1,
            "fill_attempts_used": fill_attempts,
            "replica": replica,
            "run_seed": None,
            "split": split,
            "density": density,
            "route_structure": structure,
            "route_structure_label_zh": NEW_LABELS[structure],
            "structure_parameters": {
                "primary_obstacle_count": len(primary),
                "local_bypass_records": passes,
                "max_allowed_forward_projected_length_m": LOCAL_MAX_PROJECTED_LENGTH_M,
                "max_allowed_lateral_projected_length_m": 0.75,
                "max_reference_lateral_offset_m": MAX_REFERENCE_LATERAL_OFFSET_M,
                "max_reference_heading_deg": MAX_LOCAL_HEADING_DEG,
                "entry_exit_direction": "x-monotone toward goal; reference passage advances in +x",
            },
            "density_definition": {
                "obstacle_footprint_area_fraction": area_fraction,
                "target_area_fraction_exclusive": [low, high],
                "minimum_obstacle_spacing_m": min(pair_gaps),
                "configured_minimum_spacing_m": min_gap,
                "minimum_reference_centerline_clearance_m": metrics["minimum_geometric_obstacle_clearance_m"],
                "minimum_centerline_corridor_width_m": metrics["minimum_centerline_corridor_width_m"],
                "reference_route_length_m": metrics["reference_path_length_m"],
                "detour_ratio": metrics["reference_path_length_m"] / math.dist(geom.START, geom.GOAL),
            },
            "obstacle_bounds_m": {"x": list(geom.OBSTACLE_BOUNDS[0]), "y": list(geom.OBSTACLE_BOUNDS[1])},
            "path_search_bounds_m": {"x": list(geom.PATH_BOUNDS[0]), "y": list(geom.PATH_BOUNDS[1])},
            "start_m": list(geom.START), "goal_m": list(geom.GOAL),
            "obstacle_count": len(obstacles),
            "obstacles": obstacles,
            "routes": [route],
            "direct_line_blocked": bool(blockers),
            "direct_line_blocking_obstacles": blockers,
            "path_check": metrics,
            "path_check_settings": {
                "grid_resolution_m": geom.GRID_M,
                "continuous_check_step_m": geom.PATH_CHECK_STEP_M,
                "go2_planar_envelope_radius_m": geom.GO2_ENVELOPE_RADIUS_M,
                "engineering_margin_m": geom.ENGINEERING_MARGIN_M,
                "required_centerline_clearance_m": geom.REQUIRED_CENTER_CLEARANCE_M,
                "start_goal_clearance_m": geom.START_GOAL_CLEARANCE_M,
                "route_centerline_width_definition": "2 * minimum reference centerline obstacle clearance; not actual wall-to-wall gap",
            },
        }
        return scene
    raise RuntimeError(f"{scene_id}: bounded resampling exhausted ({MAX_LAYOUT_ATTEMPTS}); last={last_reason}")


def all_new_scenes() -> list[dict]:
    result = []
    for density in DENSITY_ORDER:
        for structure in NEW_STRUCTURES:
            for split in ("development", "held_out"):
                for replica in (1, 2):
                    scene = generate_new_scene(density, structure, split, replica)
                    result.append(scene)
                    print(f"generated {len(result):02d}/24 {scene['scene_id']} "
                          f"seed={scene['generation_seed']} attempt={scene['generation_attempt']} "
                          f"area={scene['density_definition']['obstacle_footprint_area_fraction']:.4f}", flush=True)
    return result


def retained_and_excluded(old: dict) -> tuple[list[dict], list[dict]]:
    retained, excluded = [], []
    for scene in old["scenes"]:
        if scene["route_structure"] in OLD_LABELS:
            retained.append(copy.deepcopy(scene))
        else:
            excluded.append(copy.deepcopy(scene))
    if (len(retained), len(excluded)) != (36, 24):
        raise RuntimeError(f"expected 36 retained + 24 excluded, got {len(retained)} + {len(excluded)}")
    return retained, excluded


def metadata(old: dict, new_scenes: list[dict], retained: list[dict], excluded: list[dict]) -> dict:
    scenes = retained + new_scenes
    return {
        "schema_version": 1,
        "library_version": "s6-11-mapset/1.0",
        "generator_version": GENERATOR_VERSION,
        "scene_count": len(scenes),
        "development_count": sum(s["split"] == "development" for s in scenes),
        "held_out_count": sum(s["split"] == "held_out" for s in scenes),
        "stratification": {"density": list(DENSITY_ORDER), "route_structure": list(ALL_STRUCTURES),
                           "replicas_per_density_structure": 4, "development_per_cell": 2,
                           "held_out_per_cell": 2},
        "generation_seed_namespaces": {"new_development": "9110000-9130999", "new_held_out": "9210000-9230999"},
        "run_seeds": "independent from generation seeds; no policy run is authorized by S6-11",
        "density_bands_exclusive": {name: list(DENSITIES[name]["fraction"]) for name in DENSITY_ORDER},
        "bounds_m": {"obstacle": {"x": list(geom.OBSTACLE_BOUNDS[0]), "y": list(geom.OBSTACLE_BOUNDS[1])},
                     "path_search": {"x": list(geom.PATH_BOUNDS[0]), "y": list(geom.PATH_BOUNDS[1])}},
        "start_m": list(geom.START), "goal_m": list(geom.GOAL),
        "go2_planar_envelope_radius_m": geom.GO2_ENVELOPE_RADIUS_M,
        "engineering_margin_m": geom.ENGINEERING_MARGIN_M,
        "required_centerline_clearance_m": geom.REQUIRED_CENTER_CLEARANCE_M,
        "local_obstacle_limits": {"forward_projected_length_max_m": LOCAL_MAX_PROJECTED_LENGTH_M,
                                   "lateral_projected_length_max_m": 0.75,
                                   "reference_lateral_offset_max_m": MAX_REFERENCE_LATERAL_OFFSET_M,
                                   "reference_heading_change_max_deg": MAX_LOCAL_HEADING_DEG},
        "s6_09_source_generator_version": old["generator_version"],
        "history_excluded_scene_ids": [s["scene_id"] for s in excluded],
        "scenes": scenes,
    }


def legacy_excluded_doc(excluded: list[dict], new_scenes: list[dict]) -> dict:
    replacement = {}
    new_by = {(s["density"], s["route_structure"], s["split"], s["replica"]): s for s in new_scenes}
    for old in excluded:
        route = ("short_local_bypass" if old["route_structure"] == "route_fork"
                 else "forward_offset_short_obstacles")
        new = new_by[(old["density"], route, old["split"], old["replica"])]
        replacement[old["scene_id"]] = {
            "historical_filename": old["filename"],
            "historical_preview": old["preview"],
            "historical_route_structure": old["route_structure"],
            "replacement_scene_id": new["scene_id"],
            "replacement_filename": new["filename"],
            "replacement_preview": new["preview"],
            "split": old["split"], "density": old["density"], "replica": old["replica"],
        }
    return {"version": "s6-11-history-crosswalk/1.0",
            "note": "Historical maps/files/IDs/previews/runs remain resolvable and are excluded from the S6-11 main-candidate manifest.",
            "scene_count": len(excluded), "scenes": replacement}


def font(size: int):
    try:
        return ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", size)
    except OSError:
        return ImageFont.load_default()


def render_scene(scene: dict) -> Image.Image:
    image = Image.new("RGB", (1200, 900), "white")
    draw = ImageDraw.Draw(image)
    xb, yb = geom.OBSTACLE_BOUNDS
    plot = (90, 115, 1110, 790)
    sx, sy = (plot[2] - plot[0]) / (xb[1] - xb[0]), (plot[3] - plot[1]) / (yb[1] - yb[0])
    def pix(p):
        return (round(plot[0] + (p[0] - xb[0]) * sx), round(plot[3] - (p[1] - yb[0]) * sy))
    draw.text((90, 18), f"{scene['scene_slug']} · {scene['density']} · {scene['scene_id']} · seed {scene['generation_seed']}", fill="#17212b", font=font(17))
    dd = scene["density_definition"]
    draw.text((90, 52), f"占地 {dd['obstacle_footprint_area_fraction']:.1%} · 最小间距 {dd['minimum_obstacle_spacing_m']:.2f} m · 障碍 {scene['obstacle_count']} 个", fill="#475569", font=font(15))
    draw.rectangle(plot, outline="#46515b", width=2)
    # FOV boundary from the actual ray contract: +/-45 degrees, 6 m range.
    origin = pix((geom.START[0] - 0.05, geom.START[1]))
    for sign in (-1, 1):
        # The 6 m ray endpoint lies outside the declared map bounds. Clip its
        # display to the y boundary rather than drawing across the page margin.
        end = pix((geom.START[0] - 0.05 + 3.0, sign * 3.0))
        draw.line((origin, end), fill="#c4ced8", width=2)
    path = [tuple(p) for p in scene["routes"][0]["reference_path_m"]]
    px = [pix(p) for p in path]
    width = max(2, round(2 * geom.REQUIRED_CENTER_CLEARANCE_M * (sx + sy) / 2))
    draw.line(px, fill="#d9efec", width=width, joint="curve")
    draw.line(px, fill="#087e83" if scene["route_structure"] == NEW_STRUCTURES[0] else "#7c3aed", width=4, joint="curve")
    p0, p1 = pix(geom.START), pix(geom.GOAL)
    distance = math.dist(p0, p1)
    for off in range(0, int(distance), 28):
        t0, t1 = off / distance, min(off + 16, distance) / distance
        draw.line((round(p0[0] + (p1[0]-p0[0])*t0), round(p0[1] + (p1[1]-p0[1])*t0),
                   round(p0[0] + (p1[0]-p0[0])*t1), round(p0[1] + (p1[1]-p0[1])*t1)), fill="#e28a23", width=3)
    blocker_names = set(scene["direct_line_blocking_obstacles"])
    for o in scene["obstacles"]:
        color = "#bb3c37" if o["name"] in blocker_names else "#59636d"
        if geom.is_box(o):
            pts = [pix(p) for p in geom.box_vertices(o)]
            draw.polygon(pts, fill=color, outline="#25313c")
        else:
            cx, cy = pix((o["x"], o["y"])); rx, ry = o["radius"] * sx, o["radius"] * sy
            draw.ellipse((round(cx-rx), round(cy-ry), round(cx+rx), round(cy+ry)), fill=color, outline="#25313c", width=2)
    for point, label, color in ((geom.START, "START", "#21864a"), (geom.GOAL, "GOAL", "#2357a5")):
        x, y = pix(point); draw.ellipse((x-9, y-9, x+9, y+9), fill=color, outline="white", width=2)
        draw.text((x+10, y-18), label, fill=color, font=font(14))
    draw.text((90, 817), "浅青带：Go2 0.65 m 包络 + 0.10 m 余量；彩线为几何参考路线，不是机器人轨迹或策略输入。灰线：起点±45°射线视场边界。", fill="#334155", font=font(14))
    draw.text((90, 850), f"障碍边界 x=[{xb[0]:g},{xb[1]:g}] m, y=[{yb[0]:g},{yb[1]:g}] m；图中坐标按比例绘制。", fill="#334155", font=font(13))
    return image


def make_overview(scenes: list[dict], split: str, preview_images: dict[str, Image.Image]) -> Image.Image:
    thumb_w, thumb_h, margin, header_h = 360, 270, 18, 56
    canvas = Image.new("RGB", (margin + 5 * (thumb_w + margin), header_h + 6 * (thumb_h + margin)), "#f2f4f7")
    draw = ImageDraw.Draw(canvas)
    draw.text((margin, 12), f"S6-11 {split} · 30 geometry-only map previews", fill="#17212b", font=font(22))
    by_key = {(s["density"], s["route_structure"], int(s["replica"])): preview_images[s["scene_id"]] for s in scenes}
    structures = list(ALL_STRUCTURES)
    row = 0
    for density in DENSITY_ORDER:
        for replica in (1, 2):
            for col, structure in enumerate(structures):
                x, y = margin + col * (thumb_w + margin), header_h + row * (thumb_h + margin)
                im = by_key[(density, structure, replica)]
                resampling = getattr(Image, "Resampling", Image)
                canvas.paste(im.resize((thumb_w, thumb_h), resampling.LANCZOS), (x, y))
            row += 1
    return canvas


def write_markdown_tables(new_scenes: list[dict], excluded: list[dict], retained: list[dict]) -> None:
    new_by = {(s["density"], s["route_structure"], s["split"], s["replica"]): s for s in new_scenes}
    cross = ["# S6-11 旧新版 24 张替换对应表", "", "旧地图及身份仍保留；以下旧ID只列入历史排除清单，不进入 S6-11 新版60张清单。替换只对应相同密度、split和replica，不表示几何等价。", "",
             "|旧结构|旧 ID|旧地图|新结构|新 ID|新地图|划分|密度|", "|---|---|---|---|---|---|---|---|"]
    for old in excluded:
        route = "short_local_bypass" if old["route_structure"] == "route_fork" else "forward_offset_short_obstacles"
        new = new_by[(old["density"], route, old["split"], old["replica"])]
        cross.append(f"|{OLD_LABELS.get(old['route_structure'], old['route_structure'])}|`{old['scene_id']}`|`{old['filename']}`|{NEW_LABELS[route]}|`{new['scene_id']}`|`{new['filename']}`|{old['split']}|{old['density']}|")
    (GEN_DIR / "replacement_crosswalk.md").write_text("\n".join(cross) + "\n", encoding="utf-8")

    review = ["# S6-11 其余36张静态局部绕行规则审查", "", "只做静态几何核对，未按机器人策略表现筛选或改图。S6-11的新局部短障碍规则为：最大障碍前向投影0.85 m、横向投影0.75 m；保留Go2包络和工程余量后的中心净距0.75 m；参考路径向前单调、最大侧偏1.55 m、单段方向变化不超过50°。下表按旧图保存的真实障碍轮廓与参考路径，报告长构件、最大侧偏、最大段角和是否后退。长构件/大段角说明几何任务与新版规则不同，需要 Director 决定是否继续保留于主候选；本任务不扩展改图。", "",
              "|旧 ID|路线类别|密度|最长前/横向投影 m|超0.85m构件数|参考最大侧偏 m|最大段角 °|后退路径长度 m|直线阻挡数|静态判断|", "|---|---|---|---:|---:|---:|---:|---:|---:|---|"]
    for scene in retained:
        spans_x = [projected_length_x(o) for o in scene["obstacles"]]
        spans_y = [projected_length_y(o) for o in scene["obstacles"]]
        spans = [max(x, y) for x, y in zip(spans_x, spans_y)]
        long_count = sum(v > LOCAL_MAX_PROJECTED_LENGTH_M for v in spans)
        max_span = max(spans, default=0.0)
        route = [tuple(p) for p in scene["routes"][0]["reference_path_m"]]
        max_y = max((abs(p[1]) for p in route), default=0.0)
        headings = [abs(math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])))
                    for a, b in zip(route, route[1:])]
        max_heading = max(headings, default=0.0)
        backward = sum(max(0.0, a[0] - b[0]) for a, b in zip(route, route[1:]))
        status = ("需Director审阅：存在长横向构件或大转角；不等同于策略不可通过" if
                  long_count or max_heading > MAX_LOCAL_HEADING_DEG or max_y > MAX_REFERENCE_LATERAL_OFFSET_M
                  else "未触发上述静态标记；仍非策略观测/可达证明")
        review.append(f"|`{scene['scene_id']}`|{OLD_LABELS[scene['route_structure']]}|{scene['density']}|{max(spans_x, default=0.0):.3f}/{max(spans_y, default=0.0):.3f}|{long_count}|{max_y:.3f}|{max_heading:.1f}|{backward:.3f}|{len(scene.get('direct_line_blocking_obstacles', []))}|{status}|")
    (GEN_DIR / "retained_36_static_review.md").write_text("\n".join(review) + "\n", encoding="utf-8")


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def closure_sha256(root_xml: Path) -> str:
    # Same relative-path/include closure convention as scripts/abs_scene.py.
    records: dict[str, str] = {}
    visited: set[Path] = set()
    tags = {"mesh", "hfield", "texture", "skin"}
    def add(path: Path) -> None:
        path = path.resolve()
        path.relative_to(root_xml.parent.resolve())
        if not path.is_file():
            raise ValueError(f"missing scene closure file: {path}")
        records[path.relative_to(ROOT).as_posix()] = sha256_file(path)
    def visit(xml: Path) -> None:
        xml = xml.resolve()
        xml.relative_to(root_xml.parent.resolve())
        if xml in visited:
            return
        visited.add(xml)
        add(xml)
        doc = ET.parse(xml).getroot()
        for el in doc.iter():
            if el.tag == "include" and el.get("file"):
                visit(xml.parent / el.get("file", ""))
            elif el.tag in tags and el.get("file"):
                asset = xml.parent / "assets" / el.get("file", "")
                if not asset.exists():
                    asset = xml.parent / el.get("file", "")
                add(asset)
    visit(root_xml)
    payload = [{"path": p, "sha256": h} for p, h in sorted(records.items())]
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()).hexdigest()


def catalog_rows(scenes: list[dict], fingerprints: dict[str, str] | None = None) -> list[dict]:
    rows = []
    fingerprints = fingerprints or {}
    for scene in scenes:
        path = SCENE_DIR / scene["filename"]
        fp = fingerprints.get(scene["filename"], "0" * 64)
        rows.append({"filename": scene["filename"], "scene_id": scene["scene_id"],
                     "obstacle_count": scene["obstacle_count"], "root_sha256": sha256_file(path),
                     "closure_sha256": closure_sha256(path), "model_fingerprint": fp})
    return rows


def append_or_replace_catalog(rows: list[dict], replace_fingerprints_only: bool = False,
                             upsert: bool = False) -> None:
    text = CATALOG.read_text(encoding="utf-8")
    lines = text.splitlines()
    row_by_filename = {r["filename"]: r for r in rows}
    out = []
    index = 0
    found = set()
    while index < len(lines):
        line = lines[index]
        m = re.match(r'ABS_SCENE\("([^\"]+)"', line)
        if m and m.group(1) in row_by_filename:
            row = row_by_filename[m.group(1)]
            found.add(row["filename"])
            if replace_fingerprints_only:
                out.append(line)
                out.append(lines[index + 1])
                out.append(f'          "{row["model_fingerprint"]}")')
            else:
                out.extend([
                    f'ABS_SCENE("{row["filename"]}", "{row["scene_id"]}", {row["obstacle_count"]}, kNamedPptObstacles,',
                    f'          "{row["root_sha256"]}", "{row["closure_sha256"]}",',
                    f'          "{row["model_fingerprint"]}")',
                ])
            index += 3
        else:
            out.append(line)
            index += 1
    if replace_fingerprints_only:
        missing = set(row_by_filename) - found
        if missing:
            raise RuntimeError(f"catalog entries missing for fingerprints: {sorted(missing)}")
    else:
        existing = set(row_by_filename) & found
        if existing and not upsert:
            raise RuntimeError(f"new catalog filenames already exist: {sorted(existing)}")
        for row in rows:
            if row["filename"] in found:
                continue
            out.extend([
                f'ABS_SCENE("{row["filename"]}", "{row["scene_id"]}", {row["obstacle_count"]}, kNamedPptObstacles,',
                f'          "{row["root_sha256"]}", "{row["closure_sha256"]}",',
                f'          "{row["model_fingerprint"]}")',
            ])
    CATALOG.write_text("\n".join(out) + "\n", encoding="utf-8")


def write_outputs(new_scenes: list[dict], old: dict, retained: list[dict], excluded: list[dict]) -> None:
    GEN_DIR.mkdir(parents=True, exist_ok=True)
    EVIDENCE_DIR.mkdir(parents=True, exist_ok=True)
    SCENE_DIR.mkdir(parents=True, exist_ok=True)
    preview_images = {}
    for scene in new_scenes:
        xml_path = SCENE_DIR / scene["filename"]
        xml_path.write_text(geom.render_xml(scene), encoding="utf-8")
        # Python MuJoCo here checks syntax only. Runtime fingerprint is assigned
        # later by the installed C++ production MuJoCo/probe target.
        geom.mujoco.MjModel.from_xml_path(str(xml_path))
        image = render_scene(scene)
        image.save(GEN_DIR / scene["preview"], format="PNG", optimize=True)
        preview_images[scene["scene_id"]] = image
    legacy = legacy_excluded_doc(excluded, new_scenes)
    LEGACY_EXCLUDED.write_text(json.dumps(legacy, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    doc = metadata(old, new_scenes, retained, excluded)
    LAYOUTS.write_text(json.dumps(doc, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    active = doc["scenes"]
    MANIFEST.write_text(json.dumps({k: v for k, v in doc.items() if k != "scenes"} |
                                   {"active_scenes": [{"scene_id": s["scene_id"], "filename": s["filename"],
                                                       "preview": s["preview"], "density": s["density"],
                                                       "route_structure": s["route_structure"], "split": s["split"],
                                                       "generation_seed": s["generation_seed"]} for s in active]},
                                   indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    for structure in NEW_STRUCTURES:
        scene = next(s for s in new_scenes if s["density"] == "medium" and s["split"] == "development" and s["replica"] == 1 and s["route_structure"] == structure)
        preview_images[scene["scene_id"]].save(GEN_DIR / f"representative_{structure}.png", format="PNG", optimize=True)
    for split in ("development", "held_out"):
        group = [s for s in active if s["split"] == split]
        images = {}
        for scene in group:
            if scene["scene_id"] not in preview_images:
                src = ROOT / "docs/thesis_project/designs/S6-09" / scene["preview"]
                images[scene["scene_id"]] = Image.open(src).convert("RGB")
            else:
                images[scene["scene_id"]] = preview_images[scene["scene_id"]]
        make_overview(group, split, images).save(GEN_DIR / f"overview_{split}_30.png", format="PNG", optimize=True)
    write_markdown_tables(new_scenes, excluded, retained)
    # Preserve runtime fingerprints on deterministic regeneration; first-time
    # generation uses placeholders until the C++ MuJoCo probe supplies them.
    existing_fingerprints = {}
    catalog_text = CATALOG.read_text(encoding="utf-8").splitlines()
    for i, line in enumerate(catalog_text):
        match = re.match(r'ABS_SCENE\("([^"]+)"', line)
        if match and i + 2 < len(catalog_text):
            fingerprint = re.search(r'"([0-9a-f]{64})"\)\s*$', catalog_text[i + 2])
            if fingerprint:
                existing_fingerprints[match.group(1)] = fingerprint.group(1)
    fingerprints = {s["filename"]: existing_fingerprints[s["filename"]]
                    for s in new_scenes if s["filename"] in existing_fingerprints}
    rows = catalog_rows(new_scenes, fingerprints)
    append_or_replace_catalog(rows, upsert=True)


def verify_outputs(new_scenes: list[dict], old: dict, retained: list[dict], excluded: list[dict]) -> dict:
    sys.path.insert(0, str(ROOT / "scripts"))
    import abs_scene
    doc = json_read(LAYOUTS)
    expected = metadata(old, new_scenes, retained, excluded)
    if doc != expected:
        raise RuntimeError("S6-11 layout data does not match deterministic seeded generation")
    if len(doc["scenes"]) != 60 or doc["development_count"] != 30 or doc["held_out_count"] != 30:
        raise RuntimeError("S6-11 library count/split mismatch")
    ids = [s["scene_id"] for s in doc["scenes"]]
    if len(set(ids)) != 60 or any(not x.isascii() or len(x.encode("ascii")) >= 32 for x in ids):
        raise RuntimeError("S6-11 scene identities are non-unique or exceed 31 ASCII bytes")
    old_ids = {s["scene_id"] for s in old["scenes"]}
    if set(ids) & {s["scene_id"] for s in excluded} or len(old_ids - set(ids)) != 24:
        raise RuntimeError("old excluded maps are mixed into the new library or old count drifted")
    active_old = [s for s in doc["scenes"] if s["route_structure"] in OLD_LABELS]
    if len(active_old) != 36 or any(s not in old["scenes"] for s in active_old):
        raise RuntimeError("retained S6-09 scene records changed")
    expected_cells = {(d, st, split): 2 for d in DENSITY_ORDER for st in ALL_STRUCTURES
                      for split in ("development", "held_out")}
    actual_cells = {}
    for scene in doc["scenes"]:
        key = (scene["density"], scene["route_structure"], scene["split"])
        actual_cells[key] = actual_cells.get(key, 0) + 1
    if actual_cells != expected_cells:
        raise RuntimeError(f"manifest stratification mismatch: {actual_cells}")
    if len({s["generation_seed"] for s in doc["scenes"] if s["split"] == "development"} &
           {s["generation_seed"] for s in doc["scenes"] if s["split"] == "held_out"}):
        raise RuntimeError("development and held-out generation seeds overlap")
    if sum(bool(s.get("direct_line_blocked")) for s in doc["scenes"]) < 40:
        raise RuntimeError("fewer than 40 active maps block the start-goal line")
    for scene in excluded:
        spec = abs_scene.resolve_scene(scene["filename"])
        if (spec.scene_id != scene["scene_id"] or spec.obstacle_count != scene["obstacle_count"] or
                not (ROOT / "docs/thesis_project/designs/S6-09" / scene["preview"]).is_file()):
            raise RuntimeError(f"historical excluded scene no longer resolves: {scene['scene_id']}")
    for scene in new_scenes:
        xml = SCENE_DIR / scene["filename"]
        if not xml.is_file() or xml.read_text(encoding="utf-8") != geom.render_xml(scene):
            raise RuntimeError(f"XML missing or not deterministic: {scene['filename']}")
        geom.mujoco.MjModel.from_xml_path(str(xml))
        area_fraction = scene["density_definition"]["obstacle_footprint_area_fraction"]
        if not (DENSITIES[scene["density"]]["fraction"][0] <= area_fraction <
                DENSITIES[scene["density"]]["fraction"][1]):
            raise RuntimeError(f"density band mismatch: {scene['scene_id']}")
        if any(not geom.in_bounds(o) or not geom.clear_start_goal(o) for o in scene["obstacles"]):
            raise RuntimeError(f"obstacle out of bounds or endpoint clearance: {scene['scene_id']}")
        if any(not (0.32 <= float(o["height_m"]) <= 0.62) for o in scene["obstacles"]):
            raise RuntimeError(f"obstacle height outside declared existing band: {scene['scene_id']}")
        if not geom.no_overlap_and_spacing(scene["obstacles"], DENSITIES[scene["density"]]["min_gap_m"]):
            raise RuntimeError(f"overlap/minimum spacing failure: {scene['scene_id']}")
        path = [tuple(p) for p in scene["routes"][0]["reference_path_m"]]
        if scene["path_check"]["minimum_geometric_obstacle_clearance_m"] < geom.REQUIRED_CENTER_CLEARANCE_M - 1e-9:
            raise RuntimeError(f"stored route envelope clearance below requirement: {scene['scene_id']}")
        if not all(geom.segment_clear(a, b, scene["obstacles"]) for a, b in zip(path, path[1:])):
            raise RuntimeError(f"continuous envelope path failure: {scene['scene_id']}")
        if any(b[0] <= a[0] for a, b in zip(path, path[1:])) or max(abs(y) for _, y in path) > MAX_REFERENCE_LATERAL_OFFSET_M:
            raise RuntimeError(f"route is not forward-local: {scene['scene_id']}")
        if max(projected_length_x(o) for o in scene["obstacles"]) > LOCAL_MAX_PROJECTED_LENGTH_M + 1e-9:
            raise RuntimeError(f"local obstacle length limit failed: {scene['scene_id']}")
        if max(projected_length_y(o) for o in scene["obstacles"]) > 0.75 + 1e-9:
            raise RuntimeError(f"local obstacle width limit failed: {scene['scene_id']}")
        image_path = GEN_DIR / scene["preview"]
        if not image_path.is_file() or Image.open(image_path).convert("RGB").tobytes() != render_scene(scene).tobytes():
            raise RuntimeError(f"preview mismatch: {scene['scene_id']}")
    cross = json_read(LEGACY_EXCLUDED)
    if cross["scene_count"] != 24 or set(cross["scenes"]) != {s["scene_id"] for s in excluded}:
        raise RuntimeError("old/new 24-scene crosswalk mismatch")
    # Check production registry names/short IDs and root/closure binding. The
    # C++ runtime probe below is authoritative for model fingerprints.
    bindings = []
    for scene in new_scenes:
        spec = abs_scene.resolve_scene(scene["filename"])
        if spec.scene_id != scene["scene_id"] or spec.obstacle_count != scene["obstacle_count"]:
            raise RuntimeError(f"production Python scene binding mismatch: {scene['scene_id']}")
        if spec.model_fingerprint == "0" * 64:
            raise RuntimeError(f"runtime fingerprint not yet filled: {scene['scene_id']}")
        bindings.append(scene["filename"])
    return {"new_scenes": len(new_scenes), "retained_scenes": len(active_old),
            "new_manifest_scenes": len(doc["scenes"]), "development": doc["development_count"],
            "held_out": doc["held_out_count"], "new_bindings": len(bindings)}


def apply_fingerprints(path: Path, new_scenes: list[dict]) -> int:
    fingerprints = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line.startswith("SCENE "):
            continue
        fields = dict(token.split("=", 1) for token in line.split()[1:] if "=" in token)
        if "filename" in fields and "fingerprint" in fields:
            fingerprints[fields["filename"]] = fields["fingerprint"]
    expected = {s["filename"] for s in new_scenes}
    if set(fingerprints) != expected or any(re.fullmatch(r"[0-9a-f]{64}", f) is None for f in fingerprints.values()):
        raise RuntimeError(f"C++ fingerprint output incomplete: got {len(fingerprints)}/{len(expected)}")
    rows = catalog_rows(new_scenes, fingerprints)
    append_or_replace_catalog(rows, replace_fingerprints_only=True)
    return len(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--generate", action="store_true")
    modes.add_argument("--apply-fingerprints", type=Path)
    modes.add_argument("--check", action="store_true")
    args = parser.parse_args()
    old = json_read(OLD_LAYOUT)
    retained, excluded = retained_and_excluded(old)
    new_scenes = all_new_scenes()
    if args.generate:
        write_outputs(new_scenes, old, retained, excluded)
        print(f"wrote 24 new local scenes and versioned 60-scene manifest; catalog fingerprint placeholders require C++ production values")
        return 0
    if args.apply_fingerprints:
        print(f"applied production C++ runtime fingerprints to {apply_fingerprints(args.apply_fingerprints, new_scenes)} new catalog rows")
        return 0
    result = verify_outputs(new_scenes, old, retained, excluded)
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
