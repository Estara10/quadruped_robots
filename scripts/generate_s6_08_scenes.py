#!/usr/bin/env python3
"""Generate and geometrically validate reproducible S6-08 Go2 scenes.

The curve-based two-sided placement idea follows ABS testbed.py's cubic route
and safe-distance obstacle placement. Unlike that reference, this tool uses
the repository's Go2 MJCF, exact rotated-box/circle geometry, a footprint
inflation, and bounded grid connectivity checks. Reference paths are display
and validation artifacts only; they are never controller inputs.
"""
from __future__ import annotations

import argparse
import hashlib
import heapq
import json
import math
from pathlib import Path
import random
import struct
import xml.etree.ElementTree as ET

import mujoco
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
SCENE_DIR = ROOT / "unitree_mujoco/unitree_robots/go2"
OUT_DIR = ROOT / "docs/thesis_project/evidence/S6-08"
GEN_DIR = ROOT / "docs/thesis_project/designs/S6-08"
LAYOUTS = GEN_DIR / "generated_scenes.json"
GENERATOR_VERSION = "s6-08-mapgen/1.0"
START = (0.0, 0.0)
GOAL = (7.0, 0.0)
OBSTACLE_BOUNDS = ((-0.5, 7.5), (-3.0, 3.0))
PATH_BOUNDS = ((0.0, 7.0), (-2.9, 2.9))
GRID_M = 0.05
PATH_CHECK_STEP_M = 0.025
GO2_ENVELOPE_RADIUS_M = 0.65
ENGINEERING_MARGIN_M = 0.10
REQUIRED_CENTER_CLEARANCE_M = GO2_ENVELOPE_RADIUS_M + ENGINEERING_MARGIN_M
START_GOAL_CLEARANCE_M = 0.95
SPECS = {
    "sparse": {"count": 8, "gap_m": 0.28, "size_scale": 1.0, "area_fraction": [0.05, 0.09], "minimum_corridor_width_m": 1.50, "seed": 608001},
    "medium": {"count": 12, "gap_m": 0.12, "size_scale": 1.06, "area_fraction": [0.085, 0.14], "minimum_corridor_width_m": 1.50, "seed": 608003},
    "dense": {"count": 14, "gap_m": 0.06, "size_scale": 1.28, "area_fraction": [0.14, 0.22], "minimum_corridor_width_m": 1.50, "seed": 608005},
}


def geom_xy(o: dict) -> tuple[float, float]:
    return float(o["x"]), float(o["y"])


def is_box(o: dict) -> bool:
    return o["shape"] in ("box", "rotated_bar")


def box_vertices(o: dict) -> list[tuple[float, float]]:
    x, y = geom_xy(o)
    hx, hy, a = float(o["half_x"]), float(o["half_y"]), float(o["yaw_rad"])
    c, s = math.cos(a), math.sin(a)
    return [(x + c * lx - s * ly, y + s * lx + c * ly)
            for lx, ly in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy))]


def support_y(o: dict) -> float:
    if not is_box(o):
        return float(o["radius"])
    a = float(o["yaw_rad"])
    return abs(math.sin(a)) * float(o["half_x"]) + abs(math.cos(a)) * float(o["half_y"])


def support_x(o: dict) -> float:
    if not is_box(o):
        return float(o["radius"])
    a = float(o["yaw_rad"])
    return abs(math.cos(a)) * float(o["half_x"]) + abs(math.sin(a)) * float(o["half_y"])


def obstacle_area(o: dict) -> float:
    return (4.0 * float(o["half_x"]) * float(o["half_y"]) if is_box(o)
            else math.pi * float(o["radius"]) ** 2)


def point_segment_distance(p: tuple[float, float], a: tuple[float, float],
                           b: tuple[float, float]) -> float:
    vx, vy = b[0] - a[0], b[1] - a[1]
    denom = vx * vx + vy * vy
    t = 0.0 if denom == 0 else max(0.0, min(1.0,
        ((p[0] - a[0]) * vx + (p[1] - a[1]) * vy) / denom))
    return math.hypot(p[0] - a[0] - t * vx, p[1] - a[1] - t * vy)


def point_box_signed_distance(p: tuple[float, float], o: dict) -> float:
    x, y = geom_xy(o)
    dx, dy = p[0] - x, p[1] - y
    a = float(o["yaw_rad"])
    lx, ly = math.cos(a) * dx + math.sin(a) * dy, -math.sin(a) * dx + math.cos(a) * dy
    qx, qy = abs(lx) - float(o["half_x"]), abs(ly) - float(o["half_y"])
    outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
    return outside if outside > 0 else max(qx, qy)


def point_shape_distance(p: tuple[float, float], o: dict) -> float:
    if is_box(o):
        return point_box_signed_distance(p, o)
    return math.hypot(p[0] - float(o["x"]), p[1] - float(o["y"])) - float(o["radius"])


def orient(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segments_intersect(a: tuple[float, float], b: tuple[float, float],
                       c: tuple[float, float], d: tuple[float, float]) -> bool:
    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    eps = 1e-12
    if ((o1 > eps and o2 < -eps) or (o1 < -eps and o2 > eps)) and \
       ((o3 > eps and o4 < -eps) or (o3 < -eps and o4 > eps)):
        return True
    def on(p, q, r):
        return abs(orient(p, q, r)) <= eps and min(p[0], r[0]) - eps <= q[0] <= max(p[0], r[0]) + eps and \
               min(p[1], r[1]) - eps <= q[1] <= max(p[1], r[1]) + eps
    return on(a, c, b) or on(a, d, b) or on(c, a, d) or on(c, b, d)


def segment_segment_distance(a: tuple[float, float], b: tuple[float, float],
                             c: tuple[float, float], d: tuple[float, float]) -> float:
    if segments_intersect(a, b, c, d):
        return 0.0
    return min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
               point_segment_distance(c, a, b), point_segment_distance(d, a, b))


def shape_distance(a: dict, b: dict) -> float:
    if not is_box(a) and not is_box(b):
        return math.hypot(float(a["x"]) - float(b["x"]),
                          float(a["y"]) - float(b["y"])) - float(a["radius"]) - float(b["radius"])
    if is_box(a) and not is_box(b):
        return point_box_signed_distance(geom_xy(b), a) - float(b["radius"])
    if is_box(b) and not is_box(a):
        return point_box_signed_distance(geom_xy(a), b) - float(a["radius"])
    va, vb = box_vertices(a), box_vertices(b)
    axes = []
    for vertices in (va, vb):
        for i in range(2):
            dx, dy = vertices[i + 1][0] - vertices[i][0], vertices[i + 1][1] - vertices[i][1]
            axes.append((-dy, dx))
    overlap = True
    best_gap = -math.inf
    for ax, ay in axes:
        norm = math.hypot(ax, ay)
        ax, ay = ax / norm, ay / norm
        pa = [x * ax + y * ay for x, y in va]
        pb = [x * ax + y * ay for x, y in vb]
        gap = max(min(pb) - max(pa), min(pa) - max(pb))
        if gap > 0:
            overlap = False
            best_gap = max(best_gap, gap)
    if overlap:
        return -1e-12
    return min(segment_segment_distance(va[i], va[(i + 1) % 4], vb[j], vb[(j + 1) % 4])
               for i in range(4) for j in range(4))


def segment_shape_distance(a: tuple[float, float], b: tuple[float, float], o: dict) -> float:
    if not is_box(o):
        return point_segment_distance(geom_xy(o), a, b) - float(o["radius"])
    vertices = box_vertices(o)
    if point_box_signed_distance(a, o) <= 0 or point_box_signed_distance(b, o) <= 0:
        return 0.0
    return min(segment_segment_distance(a, b, vertices[i], vertices[(i + 1) % 4]) for i in range(4))


def cubic_route(y1: float, y2: float) -> list[tuple[float, float]]:
    result = []
    n = round((GOAL[0] - START[0]) / PATH_CHECK_STEP_M)
    for i in range(n + 1):
        t = i / n
        x = START[0] + (GOAL[0] - START[0]) * t
        y = 3 * (1 - t) ** 2 * t * y1 + 3 * (1 - t) * t * t * y2
        result.append((x, y))
    return result


def polyline_shape_distance(path: list[tuple[float, float]], o: dict) -> float:
    return min(segment_shape_distance(a, b, o) for a, b in zip(path, path[1:]))


def local_polyline_shape_distance(path: list[tuple[float, float]], o: dict,
                                  clearance: float = REQUIRED_CENTER_CLEARANCE_M) -> float:
    """Fast exact distance to route segments whose x ranges can be nearby.

    Generated routes are x-monotone. Any segment farther than the shape's x
    support plus the required clearance cannot intersect its inflated contour.
    Full-route checking is still performed before accepting each completed map.
    """
    lo = float(o["x"]) - support_x(o) - clearance - 0.05
    hi = float(o["x"]) + support_x(o) + clearance + 0.05
    segments = [(a, b) for a, b in zip(path, path[1:])
                if max(a[0], b[0]) >= lo and min(a[0], b[0]) <= hi]
    if not segments:
        return math.inf
    return min(segment_shape_distance(a, b, o) for a, b in segments)


def path_clearance(path: list[tuple[float, float]], obstacles: list[dict]) -> float:
    if not obstacles:
        return math.inf
    return min(polyline_shape_distance(path, o) for o in obstacles)


def line_blockers(obstacles: list[dict]) -> list[str]:
    out = []
    a, b = START, GOAL
    for o in obstacles:
        if segment_shape_distance(a, b, o) <= 1e-9:
            out.append(o["name"])
    return out


def in_bounds(o: dict) -> bool:
    xb, yb = OBSTACLE_BOUNDS
    return (float(o["x"]) - support_x(o) >= xb[0] - 1e-9 and
            float(o["x"]) + support_x(o) <= xb[1] + 1e-9 and
            float(o["y"]) - support_y(o) >= yb[0] - 1e-9 and
            float(o["y"]) + support_y(o) <= yb[1] + 1e-9)


def clear_start_goal(o: dict) -> bool:
    return all(point_shape_distance(point, o) >= START_GOAL_CLEARANCE_M
               for point in (START, GOAL))


def no_overlap_and_spacing(obstacles: list[dict], gap: float) -> bool:
    return all(shape_distance(obstacles[i], obstacles[j]) >= gap
               for i in range(len(obstacles)) for j in range(i + 1, len(obstacles)))


def grid_path(obstacles: list[dict], resolution: float = GRID_M) -> list[tuple[float, float]] | None:
    (xmin, xmax), (ymin, ymax) = PATH_BOUNDS
    nx = round((xmax - xmin) / resolution) + 1
    ny = round((ymax - ymin) / resolution) + 1
    conservative = REQUIRED_CENTER_CLEARANCE_M + resolution * math.sqrt(2) / 2
    blocked: set[tuple[int, int]] = set()
    for ix in range(nx):
        x = xmin + ix * resolution
        for iy in range(ny):
            y = ymin + iy * resolution
            p = (x, y)
            if any(point_shape_distance(p, o) < conservative for o in obstacles):
                blocked.add((ix, iy))
    start = (round((START[0] - xmin) / resolution), round((START[1] - ymin) / resolution))
    goal = (round((GOAL[0] - xmin) / resolution), round((GOAL[1] - ymin) / resolution))
    if start in blocked or goal in blocked:
        return None
    neighbors = ((-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
                 (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2)))
    frontier = [(0.0, start)]
    came: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
    costs = {start: 0.0}
    while frontier:
        _, cur = heapq.heappop(frontier)
        if cur == goal:
            cells = []
            while cur is not None:
                cells.append(cur)
                cur = came[cur]
            cells.reverse()
            return [(xmin + ix * resolution, ymin + iy * resolution) for ix, iy in cells]
        for dx, dy, step in neighbors:
            nxt = (cur[0] + dx, cur[1] + dy)
            if not (0 <= nxt[0] < nx and 0 <= nxt[1] < ny) or nxt in blocked:
                continue
            if dx and dy and ((cur[0] + dx, cur[1]) in blocked or (cur[0], cur[1] + dy) in blocked):
                continue
            new_cost = costs[cur] + step
            if new_cost >= costs.get(nxt, math.inf):
                continue
            costs[nxt] = new_cost
            came[nxt] = cur
            heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
            heapq.heappush(frontier, (new_cost + heuristic, nxt))
    return None


def segment_clear(a: tuple[float, float], b: tuple[float, float], obstacles: list[dict]) -> bool:
    return all(segment_shape_distance(a, b, o) >= REQUIRED_CENTER_CLEARANCE_M
               for o in obstacles)


def simplify_path(path: list[tuple[float, float]], obstacles: list[dict]) -> list[tuple[float, float]]:
    if len(path) < 3:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not segment_clear(path[i], path[j], obstacles):
            j -= 1
        out.append(path[j])
        i = j
    return out


def path_samples(path: list[tuple[float, float]], step: float = PATH_CHECK_STEP_M):
    for a, b in zip(path, path[1:]):
        d = math.dist(a, b)
        n = max(1, math.ceil(d / step))
        for i in range(n):
            t = i / n
            yield (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
    yield path[-1]


def path_metrics(path: list[tuple[float, float]], obstacles: list[dict]) -> dict:
    samples = list(path_samples(path))
    clearance = min(point_shape_distance(p, o) for p in samples for o in obstacles)
    blockers = line_blockers(obstacles)
    length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    endpoint_tolerance_m = 1e-9
    return {
        "connected": (math.dist(path[0], START) <= endpoint_tolerance_m and
                      math.dist(path[-1], GOAL) <= endpoint_tolerance_m),
        "endpoint_tolerance_m": endpoint_tolerance_m,
        "grid_resolution_m": GRID_M,
        "continuous_path_check_step_m": PATH_CHECK_STEP_M,
        "go2_planar_envelope_radius_m": GO2_ENVELOPE_RADIUS_M,
        "engineering_clearance_margin_m": ENGINEERING_MARGIN_M,
        "required_obstacle_clearance_m": REQUIRED_CENTER_CLEARANCE_M,
        "minimum_geometric_obstacle_clearance_m": clearance,
        "minimum_centerline_corridor_width_m": 2.0 * clearance,
        "residual_width_after_envelope_margin_m": 2.0 * (clearance - REQUIRED_CENTER_CLEARANCE_M),
        "reference_path_length_m": length,
        "reference_path_point_count": len(path),
        "start_goal_straight_blocked": bool(blockers),
        "start_goal_straight_blocking_obstacles": blockers,
    }


def make_obstacle(rng: random.Random, index: int, shape: str, x: float, y: float,
                  size_scale: float = 1.0) -> dict:
    o = {"name": f"ppt_obstacle_{index:02d}", "shape": shape, "x": x, "y": y,
         "height_m": round(rng.uniform(0.32, 0.62), 6)}
    if shape == "cylinder":
        o["radius"] = round(rng.uniform(0.30, 0.38) * size_scale, 6)
        o["yaw_rad"] = 0.0
    elif shape == "rotated_bar":
        o["half_x"] = round(rng.uniform(0.44, 0.58) * size_scale, 6)
        o["half_y"] = round(rng.uniform(0.105, 0.15) * size_scale, 6)
        o["yaw_rad"] = round(rng.uniform(-0.9, 0.9), 6)
    else:
        o["half_x"] = round(rng.uniform(0.31, 0.44) * size_scale, 6)
        o["half_y"] = round(rng.uniform(0.25, 0.34) * size_scale, 6)
        o["yaw_rad"] = round(rng.uniform(-math.pi, math.pi), 6)
    return o


def gen_scene(density: str, ordinal: int) -> dict:
    spec = SPECS[density]
    seed = spec["seed"] + ordinal
    scene_id = f"s6_08_{density}_{ordinal + 1:02d}"
    rng = random.Random(seed)
    last_reason = "bounded_resampling_exhausted"
    for route_attempt in range(96):
        sign = rng.choice((-1.0, 1.0))
        y1 = sign * rng.uniform(1.65, 1.90)
        y2 = sign * rng.uniform(1.65, 1.90)
        route = cubic_route(y1, y2)
        n_obstacles = int(spec["count"])
        line_xs = sorted((rng.uniform(1.95, 2.15), rng.uniform(4.85, 5.05)))
        obstacles = []
        # Two guaranteed, finite-segment line blockers; their placement follows
        # the random-route/two-sided reference method and remains within bounds.
        for index, x in enumerate(line_xs, 1):
            # Low-profile horizontal bars cross the finite direct line while
            # allowing the reference route to pass above/below with clearance.
            o = make_obstacle(rng, index, "rotated_bar", x, 0.0, float(spec["size_scale"]))
            o["half_x"], o["half_y"], o["yaw_rad"] = 0.22 * spec["size_scale"], 0.07 * spec["size_scale"], 0.0
            obstacles.append(o)
        if any(polyline_shape_distance(route, o) < REQUIRED_CENTER_CLEARANCE_M + 0.02 for o in obstacles):
            last_reason = "random_route_cannot_clear_line_blockers"
            continue
        if not all(clear_start_goal(o) and in_bounds(o) for o in obstacles):
            last_reason = "line_blocker_violates_start_goal_or_bounds"
            continue
        # Place roughly balanced counts on each side of the generated route.
        remaining = n_obstacles - 2
        tight_side_quota = min(4, max(2, math.ceil(remaining * 0.4)))
        side_quota = {sign: tight_side_quota, -sign: remaining - tight_side_quota}
        placed = len(obstacles)
        placement_failed = False
        attempts_per_obstacle = 1400
        for side in (sign, -sign):
            for _ in range(side_quota[side]):
                success = False
                for _try in range(attempts_per_obstacle):
                    idx = placed + 1
                    forced_shape = "rotated_bar" if idx == 3 else rng.choices(
                        ("box", "cylinder", "rotated_bar"), weights=(0.52, 0.32, 0.16))[0]
                    curve_x = rng.uniform(0.75, 6.25)
                    curve_i = min(len(route) - 1, round(curve_x / PATH_CHECK_STEP_M))
                    curve_y = route[curve_i][1]
                    cand = make_obstacle(rng, idx, forced_shape, curve_x,
                                         curve_y + side * rng.uniform(0.0, 1.0), float(spec["size_scale"]))
                    # Offset using the exact rotated projection plus the same
                    # center clearance that defines path feasibility.
                    cand["y"] = curve_y + side * (REQUIRED_CENTER_CLEARANCE_M + support_y(cand) +
                                                   spec["gap_m"] + rng.uniform(0.03, 1.25))
                    cand["x"] = round(cand["x"], 6)
                    cand["y"] = round(cand["y"], 6)
                    if not in_bounds(cand) or not clear_start_goal(cand):
                        continue
                    if any(shape_distance(cand, old) < spec["gap_m"] for old in obstacles):
                        continue
                    if local_polyline_shape_distance(route, cand) < REQUIRED_CENTER_CLEARANCE_M:
                        continue
                    obstacles.append(cand)
                    placed += 1
                    success = True
                    break
                if not success:
                    placement_failed = True
                    last_reason = f"could_not_place_side_{'upper' if side > 0 else 'lower'}_obstacle_{placed + 1}"
                    break
            if placement_failed:
                break
        if placement_failed or len(obstacles) != n_obstacles:
            continue
        if not no_overlap_and_spacing(obstacles, spec["gap_m"]):
            last_reason = "overlap_or_minimum_spacing_failure"
            continue
        direct_blockers = line_blockers(obstacles)
        if len(direct_blockers) < 1:
            last_reason = "no_start_goal_line_blocker"
            continue
        grid = grid_path(obstacles)
        if grid is None:
            last_reason = "no_connected_center_path_on_5cm_grid"
            continue
        path = simplify_path(grid, obstacles)
        if not all(segment_clear(a, b, obstacles) for a, b in zip(path, path[1:])):
            last_reason = "simplified_reference_path_intersects_inflated_obstacle"
            continue
        metrics = path_metrics(path, obstacles)
        if metrics["minimum_geometric_obstacle_clearance_m"] < REQUIRED_CENTER_CLEARANCE_M:
            last_reason = "reference_path_clearance_below_go2_envelope"
            continue
        if metrics["minimum_centerline_corridor_width_m"] < spec["minimum_corridor_width_m"]:
            last_reason = "reference_corridor_below_density_class_minimum"
            continue
        area_fraction = sum(obstacle_area(o) for o in obstacles) / \
                        ((OBSTACLE_BOUNDS[0][1] - OBSTACLE_BOUNDS[0][0]) *
                         (OBSTACLE_BOUNDS[1][1] - OBSTACLE_BOUNDS[1][0]))
        low, high = spec["area_fraction"]
        if not low <= area_fraction <= high:
            last_reason = f"occupied_area_fraction_outside_{low}_{high}:{area_fraction:.6f}"
            continue
        # A simple spacing descriptor complements occupied area and bottleneck width.
        pair_gaps = [shape_distance(obstacles[i], obstacles[j])
                     for i in range(len(obstacles)) for j in range(i + 1, len(obstacles))]
        return {
            "scene_id": scene_id,
            "filename": f"scene_{scene_id}.xml",
            "preview": f"{scene_id}.png",
            "generator_version": GENERATOR_VERSION,
            "generation_seed": seed,
            "run_seed": None,
            "density": density,
            "density_definition": {
                "obstacle_footprint_area_fraction": area_fraction,
                "target_area_fraction": [low, high],
                "minimum_obstacle_spacing_m": min(pair_gaps),
                "configured_minimum_spacing_m": spec["gap_m"],
                "minimum_reference_corridor_width_m": metrics["minimum_centerline_corridor_width_m"],
                "configured_minimum_reference_corridor_width_m": spec["minimum_corridor_width_m"],
            },
            "obstacle_bounds_m": {"x": list(OBSTACLE_BOUNDS[0]), "y": list(OBSTACLE_BOUNDS[1])},
            "path_search_bounds_m": {"x": list(PATH_BOUNDS[0]), "y": list(PATH_BOUNDS[1])},
            "start_m": list(START),
            "goal_m": list(GOAL),
            "route_control_y_m": [y1, y2],
            "obstacle_count": len(obstacles),
            "obstacles": obstacles,
            "reference_path_m": [list(p) for p in path],
            "path_check": metrics,
        }
    raise RuntimeError(f"{scene_id}: bounded generation failed after 96 route attempts; last={last_reason}")


def xml_float(x: float) -> str:
    return format(float(x), ".17g")


def render_xml(scene: dict) -> str:
    center_x = sum(OBSTACLE_BOUNDS[0]) / 2
    center_y = sum(OBSTACLE_BOUNDS[1]) / 2
    extent = max(OBSTACLE_BOUNDS[0][1] - OBSTACLE_BOUNDS[0][0],
                 OBSTACLE_BOUNDS[1][1] - OBSTACLE_BOUNDS[1][0])
    lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        f'<mujoco model="go2 {scene.get("model_name", scene["scene_id"])}">',
        '  <include file="go2.xml" />',
        f'  <statistic center="{xml_float(center_x)} {xml_float(center_y)} 0.1" extent="{xml_float(extent)}" />',
        '  <visual><headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0" />',
        '    <rgba haze="0.15 0.25 0.35 1" /><global azimuth="135" elevation="-40" /></visual>',
        '  <asset>',
        '    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072" />',
        '    <texture type="2d" name="s6_08_groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300" />',
        '    <material name="s6_08_groundplane" texture="s6_08_groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2" />',
        '  </asset>',
        '  <worldbody><light pos="1 0 7" dir="0 0 -1" directional="true" />',
        f'    <geom name="floor" type="plane" pos="{xml_float(center_x)} {xml_float(center_y)} 0" size="4 3.5 0.05" material="s6_08_groundplane" />',
    ]
    for o in scene["obstacles"]:
        z, h = float(o["height_m"]) / 2, float(o["height_m"])
        if is_box(o):
            size = f'{xml_float(o["half_x"])} {xml_float(o["half_y"])} {xml_float(h / 2)}'
            lines.append(f'    <geom name="{o["name"]}" type="box" pos="{xml_float(o["x"])} {xml_float(o["y"])} {xml_float(z)}" euler="0 0 {xml_float(o["yaw_rad"])}" size="{size}" rgba="0.26 0.28 0.31 1" />')
        else:
            lines.append(f'    <geom name="{o["name"]}" type="cylinder" pos="{xml_float(o["x"])} {xml_float(o["y"])} {xml_float(z)}" size="{xml_float(o["radius"])} {xml_float(h / 2)}" rgba="0.26 0.28 0.31 1" />')
    lines.extend(['  </worldbody>', '</mujoco>', ''])
    xml = "\n".join(lines)
    ET.fromstring(xml)
    return xml


def render_preview(scene: dict) -> Image.Image:
    image = Image.new("RGB", (1200, 900), "white")
    d = ImageDraw.Draw(image)
    try:
        font = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 17)
        small = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 13)
        title = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 21)
    except OSError:
        font = small = title = ImageFont.load_default()
    xb, yb = OBSTACLE_BOUNDS
    box = (105, 90, 1095, 800)
    sx = (box[2] - box[0]) / (xb[1] - xb[0])
    sy = (box[3] - box[1]) / (yb[1] - yb[0])
    def pix(p):
        return (round(box[0] + (p[0] - xb[0]) * sx), round(box[1] + (yb[1] - p[1]) * sy))
    d.text((105, 22), f'{scene["scene_id"]} · {scene["density"]} · seed {scene["generation_seed"]}', fill="#17212b", font=title)
    d.text((105, 55), f'占地比例 {scene["density_definition"]["obstacle_footprint_area_fraction"]:.1%} · '
           f'最小障碍间距 {scene["density_definition"]["minimum_obstacle_spacing_m"]:.2f} m · '
           f'通行包络余量后净宽 {scene["path_check"]["residual_width_after_envelope_margin_m"]:.3f} m',
           fill="#475569", font=small)
    d.rectangle(box, outline="#46515b", width=2)
    # Show the permitted robot-center corridor around the reference path; this is
    # geometry-only visualization, not a controller path command.
    corridor_width = 2 * REQUIRED_CENTER_CLEARANCE_M
    path = [tuple(p) for p in scene["reference_path_m"]]
    px_path = [pix(p) for p in path]
    d.line(px_path, fill="#cbe8e4", width=round(corridor_width * (sx + sy) / 2), joint="curve")
    for p in px_path:
        r = round(corridor_width * (sx + sy) / 4)
        d.ellipse((p[0]-r, p[1]-r, p[0]+r, p[1]+r), fill="#cbe8e4")
    d.line(px_path, fill="#087e83", width=4, joint="curve")
    p0, p1 = pix(START), pix(GOAL)
    dist = math.dist(p0, p1)
    dash_px = 18
    for off in range(0, int(dist), dash_px * 2):
        t0, t1 = off / dist, min(off + dash_px, dist) / dist
        d.line((round(p0[0] + (p1[0]-p0[0])*t0), round(p0[1] + (p1[1]-p0[1])*t0),
                round(p0[0] + (p1[0]-p0[0])*t1), round(p0[1] + (p1[1]-p0[1])*t1)), fill="#e28a23", width=3)
    blockers = set(scene["path_check"]["start_goal_straight_blocking_obstacles"])
    for o in scene["obstacles"]:
        color = "#bb3c37" if o["name"] in blockers else "#59636d"
        outline = "#7f1d1d" if o["name"] in blockers else "#25313c"
        if is_box(o):
            poly = [pix(p) for p in box_vertices(o)]
            d.polygon(poly, fill=color, outline=outline)
            d.line(poly + [poly[0]], fill=outline, width=2)
        else:
            cx, cy = pix(geom_xy(o)); rx = float(o["radius"]) * sx; ry = float(o["radius"]) * sy
            d.ellipse((round(cx-rx), round(cy-ry), round(cx+rx), round(cy+ry)), fill=color, outline=outline, width=2)
    for point, label, color in ((START, "START", "#21864a"), (GOAL, "GOAL", "#2357a5")):
        x, y = pix(point); d.ellipse((x-9, y-9, x+9, y+9), fill=color, outline="white", width=2)
        d.text((x+10, y-18), label, fill=color, font=small)
    d.text((105, 817), "红色：阻挡直线的障碍；青色：参考路线及 Go2 包络+余量；橙色虚线：起点至目标直线。路线不输入策略。", fill="#334155", font=font)
    d.text((105, 850), f'坐标边界 x=[{xb[0]:g},{xb[1]:g}] m, y=[{yb[0]:g},{yb[1]:g}] m；比例 x={1/sx:.4f} m/px, y={1/sy:.4f} m/px', fill="#334155", font=small)
    return image


def model_fingerprint(model: mujoco.MjModel) -> str:
    b = bytearray(b"abs-go2-collision-model-fingerprint/v1\0")
    def u32(v): b.extend(struct.pack("<I", int(v) & 0xffffffff))
    def i32(v): b.extend(struct.pack("<i", int(v)))
    def f64(v): b.extend(struct.pack("<d", float(v)))
    def string(v):
        raw = (v or "").encode()
        u32(len(raw)); b.extend(raw)
    u32(model.nbody); u32(model.ngeom)
    for i in range(model.ngeom):
        body = int(model.geom_bodyid[i])
        u32(i); i32(model.geom_type[i]); i32(body); i32(model.geom_group[i])
        i32(model.geom_contype[i]); i32(model.geom_conaffinity[i])
        for v in model.geom_pos[i]: f64(v)
        for v in model.geom_quat[i]: f64(v)
        for v in model.geom_size[i]: f64(v)
        string(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i))
        string(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body))
    return hashlib.sha256(b).hexdigest()


def closure_digest(root_xml: Path) -> str:
    # Keep the same include and mesh/asset traversal as scripts/abs_scene.py.
    records_by_path: dict[str, str] = {}
    visited: set[Path] = set()
    file_tags = {"mesh", "hfield", "texture", "skin"}
    def add(path: Path) -> None:
        path = path.resolve()
        path.relative_to(root_xml.parent.resolve())
        if not path.is_file():
            raise ValueError(f"scene closure asset missing: {path}")
        records_by_path[path.relative_to(ROOT).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
    def visit(xml_path: Path) -> None:
        xml_path = xml_path.resolve()
        xml_path.relative_to(root_xml.parent.resolve())
        if xml_path in visited:
            return
        visited.add(xml_path)
        add(xml_path)
        tree = ET.parse(xml_path).getroot()
        for element in tree.iter():
            if element.tag == "include" and element.get("file"):
                visit(xml_path.parent / element.get("file", ""))
            elif element.tag in file_tags and element.get("file"):
                asset = xml_path.parent / "assets" / element.get("file", "")
                if not asset.exists():
                    asset = xml_path.parent / element.get("file", "")
                add(asset)
    visit(root_xml)
    records = [{"path": path, "sha256": digest} for path, digest in sorted(records_by_path.items())]
    payload = json.dumps(records, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()
    return hashlib.sha256(payload).hexdigest()


def catalog_row(scene: dict, xml_path: Path) -> dict:
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    return {
        "filename": scene["filename"], "scene_id": scene["scene_id"],
        "obstacle_count": scene["obstacle_count"],
        "root_sha256": hashlib.sha256(xml_path.read_bytes()).hexdigest(),
        "closure_sha256": closure_digest(xml_path),
        "model_fingerprint": model_fingerprint(model),
    }


def sample_density(density: str, first_ordinal: int = 0, count: int = 2) -> list[dict]:
    return [gen_scene(density, ordinal) for ordinal in range(first_ordinal, first_ordinal + count)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="validate existing generated outputs without writing")
    parser.add_argument("--print-catalog-rows", action="store_true", help="also print production catalog identities")
    parser.add_argument("--count-per-density", type=int, default=2,
                        help="maps generated per density (default: 2; final library can use 20)")
    parser.add_argument("--first-ordinal", type=int, default=0,
                        help="first zero-based ordinal/seed offset per density")
    args = parser.parse_args()
    if args.count_per_density < 1 or args.first_ordinal < 0:
        parser.error("--count-per-density must be positive and --first-ordinal nonnegative")
    scenes = []
    for density in ("sparse", "medium", "dense"):
        scenes.extend(sample_density(density, args.first_ordinal, args.count_per_density))
    doc = {
        "schema_version": 1, "generator_version": GENERATOR_VERSION,
        "generation_seeds": {scene["scene_id"]: scene["generation_seed"] for scene in scenes},
        "run_seeds": "separate from generation seeds; set per diagnostic run",
        "robot_center_envelope_basis": {
            "planar_envelope_radius_m": GO2_ENVELOPE_RADIUS_M,
            "source": "go2.xml: hip x offset 0.1934 m; thigh/calf link lengths 0.213 m each; foot sphere radius 0.027 m; circular envelope conservatively rounds the approximately 0.646 m straightened leg reach down to 0.65 m and also covers the 0.335 m front trunk/collision extent.",
            "engineering_margin_m": ENGINEERING_MARGIN_M,
            "note": "The envelope is a conservative geometry-based planning assumption, not a validated walking gait footprint."
        },
        "obstacle_bounds_m": {"x": list(OBSTACLE_BOUNDS[0]), "y": list(OBSTACLE_BOUNDS[1])},
        "default_start_m": list(START), "default_goal_m": list(GOAL),
        "sample_only": True, "not_for_held_out_test": True,
        "scenes": scenes,
    }
    if not args.check:
        GEN_DIR.mkdir(parents=True, exist_ok=True)
        SCENE_DIR.mkdir(parents=True, exist_ok=True)
        for scene in scenes:
            path = SCENE_DIR / scene["filename"]
            xml = render_xml(scene)
            path.write_text(xml, encoding="utf-8")
            preview = render_preview(scene)
            preview.save(GEN_DIR / scene["preview"], format="PNG", optimize=True)
            # Compile the actual MJCF now; failures do not produce registered layouts.
            mujoco.MjModel.from_xml_path(str(path))
        LAYOUTS.write_text(json.dumps(doc, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    else:
        if not LAYOUTS.is_file():
            raise SystemExit(f"missing layout source: {LAYOUTS}")
        old = json.loads(LAYOUTS.read_text(encoding="utf-8"))
        old_by_id = {s["scene_id"]: s for s in old.get("scenes", [])}
        if set(old_by_id) != {s["scene_id"] for s in scenes}:
            raise SystemExit("scene IDs differ from deterministic regeneration")
        for scene in scenes:
            expected_xml = render_xml(scene)
            path = SCENE_DIR / scene["filename"]
            if not path.is_file() or path.read_text(encoding="utf-8") != expected_xml:
                raise SystemExit(f"generated XML differs: {path}")
            if old_by_id[scene["scene_id"]] != scene:
                raise SystemExit(f"generated layout differs: {scene['scene_id']}")
            actual_image = Image.open(GEN_DIR / scene["preview"]).convert("RGB")
            if actual_image.tobytes() != render_preview(scene).tobytes():
                raise SystemExit(f"generated preview differs: {scene['preview']}")
            mujoco.MjModel.from_xml_path(str(path))
    print(f"generator={GENERATOR_VERSION} scenes={len(scenes)} grid={GRID_M:.3f}m path_sample={PATH_CHECK_STEP_M:.3f}m")
    for scene in scenes:
        blockers = scene["path_check"]["start_goal_straight_blocking_obstacles"]
        print(f"{scene['filename']} seed={scene['generation_seed']} density={scene['density']} "
              f"count={scene['obstacle_count']} area={scene['density_definition']['obstacle_footprint_area_fraction']:.4f} "
              f"min_gap={scene['density_definition']['minimum_obstacle_spacing_m']:.3f} "
              f"corridor={scene['path_check']['minimum_centerline_corridor_width_m']:.3f} "
              f"blockers={len(blockers)} connected=true")
        if args.print_catalog_rows:
            row = catalog_row(scene, SCENE_DIR / scene["filename"])
            print("CATALOG " + json.dumps(row, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
