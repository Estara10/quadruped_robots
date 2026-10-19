#!/usr/bin/env python3
"""Create the four M1 stage-2 geometry-only old/new comparison samples.

This intentionally generates only two structures x sparse/medium x one
development replica. It does not register production scenes or generate the
60-map library. Images show the old S6-09 layout next to its M1 prototype.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import generate_s6_08_scenes as geom  # noqa: E402

OLD_LAYOUTS = ROOT / "docs/thesis_project/designs/S6-11/layouts_s6_11.json"
OUT = ROOT / "docs/thesis_project/designs/M1/stage2_samples"
RULES = ROOT / "docs/thesis_project/designs/M1/MAP_RULES.md"
SEED_BASE = 10151000
START = (0.0, 0.0)
GOAL = (7.5, 0.0)
OBSTACLE_BOUNDS = ((-0.5, 8.0), (-3.0, 3.0))
FIELD_AREA_M2 = 8.5 * 6.0
MIN_PAIR_GAP_M = 0.50
MIN_PATH_CLEARANCE_M = 0.75
MAX_OBSTACLE_PROJECTION_X_M = 0.85
MAX_OBSTACLE_PROJECTION_Y_M = 0.75

# The paths are analytic, x-monotone and use the geometry-only reference line.
# A moderate lateral amplitude creates a broad S; the offset route uses three
# smaller alternating lobes. Neither path is sent to the controller.
def reference_path(kind: str) -> list[tuple[float, float]]:
    n = 600
    amp = 1.18 if kind == "gentle_s" else 0.94
    cycles = 2.0 if kind == "gentle_s" else 3.0
    return [(GOAL[0] * i / n,
             amp * math.sin(cycles * math.pi * i / n)) for i in range(n + 1)]


def projected_x(o: dict) -> float:
    if o["shape"] == "cylinder":
        return float(o["radius"])
    a = float(o["yaw_rad"])
    return abs(math.cos(a)) * float(o["half_x"]) + abs(math.sin(a)) * float(o["half_y"])


def projected_y(o: dict) -> float:
    if o["shape"] == "cylinder":
        return float(o["radius"])
    a = float(o["yaw_rad"])
    return abs(math.sin(a)) * float(o["half_x"]) + abs(math.cos(a)) * float(o["half_y"])


def new_obstacles(kind: str, density: str) -> list[dict]:
    if kind == "gentle_s":
        obstacles = [
            {"name": "ppt_obstacle_01", "shape": "box", "x": 1.875, "y": 0.0,
             "half_x": 0.16, "half_y": 0.16, "yaw_rad": 0.0,
             "height_m": 0.48, "role": "gentle_s_primary"},
            {"name": "ppt_obstacle_02", "shape": "cylinder", "x": 5.625, "y": 0.0,
             "radius": 0.18, "yaw_rad": 0.0,
             "height_m": 0.44, "role": "gentle_s_primary"},
            {"name": "ppt_obstacle_03", "shape": "cylinder", "x": 4.65, "y": 2.40,
             "radius": 0.18, "yaw_rad": 0.0,
             "height_m": 0.40, "role": "sparse_fill"},
        ]
    elif kind == "offset_route":
        obstacles = [
            {"name": f"ppt_obstacle_{i:02d}", "shape": "rotated_bar", "x": x, "y": y,
             "half_x": 0.10, "half_y": 0.37, "yaw_rad": 0.0,
             "height_m": 0.44, "role": "offset_primary"}
            for i, (x, y) in enumerate(zip((1.25, 3.75, 6.25), (-0.36, 0.36, -0.36)), 1)
        ]
    else:
        raise ValueError(kind)
    if density == "medium":
        fill_positions = (((0.75, 2.35), (2.80, -2.40), (4.65, -2.40),
                           (6.60, 2.38), (7.45, -2.38), (1.70, -2.40)) if kind == "gentle_s" else
                          ((0.75, 2.35), (2.55, -2.40), (4.45, 2.40),
                           (6.80, -2.40), (7.45, 2.38)))
        fillers = [{"name": f"ppt_obstacle_{i+4:02d}", "shape": "cylinder", "x": x, "y": y,
                    "radius": 0.20, "yaw_rad": 0.0, "height_m": 0.40, "role": "medium_fill"}
                   for i, (x, y) in enumerate(fill_positions)]
        obstacles.extend(fillers)
    elif density == "sparse":
        # Off-route cylinders bring the area fraction into the
        # proposed open-map sparse band without occupying the reference lane.
        positions = ((0.75, -2.35),) if kind == "gentle_s" else (
            (0.75, 2.35), (4.65, -2.40), (7.45, 2.35))
        for pos in positions:
            obstacles.append({"name": f"ppt_obstacle_{len(obstacles)+1:02d}", "shape": "cylinder",
                              "x": pos[0], "y": pos[1], "radius": 0.20, "yaw_rad": 0.0,
                              "height_m": 0.40, "role": "sparse_fill"})
    return obstacles


def angle_metrics(path: list[tuple[float, float]]) -> tuple[float, float]:
    headings = [math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
                for a, b in zip(path, path[1:])]
    max_abs = max(abs(v) for v in headings)
    delta = [abs((b - a + 180.0) % 360.0 - 180.0) for a, b in zip(headings, headings[1:])]
    return max_abs, max(delta, default=0.0)


def make_scene(old: dict, kind: str, density: str, seed: int) -> dict:
    # Align the shared exact geometry helpers with this M1 candidate field.
    geom.START = START
    geom.GOAL = GOAL
    geom.OBSTACLE_BOUNDS = OBSTACLE_BOUNDS
    geom.PATH_BOUNDS = ((START[0], GOAL[0]), (-2.9, 2.9))
    geom.START_GOAL_CLEARANCE_M = 0.95
    obstacles = new_obstacles(kind, density)
    path = reference_path(kind)
    spacing = min(geom.shape_distance(a, b) for i, a in enumerate(obstacles)
                  for b in obstacles[i + 1:]) if len(obstacles) > 1 else math.inf
    core = sorted((o for o in obstacles if "primary" in o["role"]), key=lambda o: float(o["x"]))
    core_longitudinal_gaps = [float(b["x"]) - projected_x(b) -
                              (float(a["x"]) + projected_x(a))
                              for a, b in zip(core, core[1:])]
    clearance = geom.path_clearance(path, obstacles)
    max_heading, max_heading_change = angle_metrics(path)
    grid_path = geom.grid_path(obstacles, resolution=0.05)
    assert grid_path is not None, f"no 0.75m-envelope connected route: {old['scene_id']}"
    assert spacing >= MIN_PAIR_GAP_M, ("inter-object gap below 0.50m",
                                       old["scene_id"], spacing)
    assert clearance >= MIN_PATH_CLEARANCE_M, ("route clearance below 0.75m",
                                               old["scene_id"], clearance)
    assert max_heading <= (45.0 if kind == "gentle_s" else 50.0)
    for o in obstacles:
        assert all(float(o[k]) - support >= bound[0] - 1e-9 and
                   float(o[k]) + support <= bound[1] + 1e-9
                   for k, support, bound in (("x", projected_x(o), OBSTACLE_BOUNDS[0]),
                                             ("y", projected_y(o), OBSTACLE_BOUNDS[1])))
        assert projected_x(o) * 2 <= MAX_OBSTACLE_PROJECTION_X_M + 1e-9
        assert projected_y(o) * 2 <= MAX_OBSTACLE_PROJECTION_Y_M + 1e-9
        assert all(geom.point_shape_distance(point, o) >= 0.95 for point in (START, GOAL))
    assert geom.no_overlap_and_spacing(obstacles, MIN_PAIR_GAP_M)
    direct_blockers = [o["name"] for o in obstacles
                       if geom.segment_shape_distance(START, GOAL, o) <= 1e-9]
    # Two of the three core obstacles intersect the point-robot centerline.
    assert len(direct_blockers) >= 2
    route_length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
    area = sum(geom.obstacle_area(o) for o in obstacles) / FIELD_AREA_M2
    band_lo, band_hi = {"sparse": (0.008, 0.019), "medium": (0.019, 0.032),
                        "dense": (0.032, 0.050)}[density]
    assert band_lo <= area < band_hi, ("candidate density outside proposed band",
                                       old["scene_id"], area, band_lo, band_hi)
    return {
        "scene_id": ("m1_s_sp_d01" if kind == "gentle_s" and density == "sparse" else
                     "m1_s_md_d01" if kind == "gentle_s" else
                     "m1_o_sp_d01" if density == "sparse" else "m1_o_md_d01"),
        "scene_slug": f"m1_{kind}_{density}_dev_01",
        "source_scene_id": old["scene_id"],
        "density": density,
        "density_band_proposal": [band_lo, band_hi],
        "route_structure": kind,
        "split": "development_sample_only",
        "generator_version": "m1-stage2-prototype/1.0",
        "generation_seed": None,
        "fixed_prototype_recipe_id": seed,
        "random_sampling_used": False,
        "run_seed": None,
        "start_m": list(START), "goal_m": list(GOAL),
        "field_bounds_m": {"x": list(OBSTACLE_BOUNDS[0]), "y": list(OBSTACLE_BOUNDS[1])},
        "field_is_physical_wall": False,
        "obstacle_count": len(obstacles),
        "obstacles": obstacles,
        "reference_path_m": [[round(x, 6), round(y, 6)] for x, y in path],
        "path_check": {
            "connected": True, "grid_connectivity_0_05m": bool(grid_path),
            "go2_envelope_radius_m": 0.65, "engineering_margin_m": 0.10,
            "required_centerline_clearance_m": 0.75,
            "minimum_edge_clearance_m": clearance,
            "extra_centerline_corridor_width_m": max(0.0, 2 * (clearance - 0.75)),
            "extra_corridor_definition": "2*(reference-path minimum edge clearance - 0.75m); not the actual obstacle-to-obstacle opening",
            "reference_path_length_m": route_length,
            "detour_ratio": route_length / (GOAL[0] - START[0]),
            "max_heading_from_forward_deg": max_heading,
            "max_heading_change_between_samples_deg": max_heading_change,
            "actual_minimum_obstacle_edge_spacing_m": spacing,
            "primary_obstacle_longitudinal_edge_gaps_m": core_longitudinal_gaps,
            "grid_resolution_m": 0.05,
            "continuous_shape_check_step_m": 0.025,
            "direct_line_blocked": bool(direct_blockers),
            "direct_line_blocking_obstacles": direct_blockers,
            "route_geometry_method": "analytic sinusoidal reference polyline sampled at 0.0125m; actual rotated box/circle distance checked with shared geometry helpers",
        },
        "footprint_area_fraction": area,
        "obstacle_projection_max_m": {"x": max(projected_x(o) * 2 for o in obstacles),
                                       "y": max(projected_y(o) * 2 for o in obstacles)},
        "core_obstacle_centers_m": [[o["x"], o["y"]] for o in obstacles if "primary" in o["role"]],
        "filename": f"scene_{'m1_gentle_s' if kind == 'gentle_s' else 'm1_offset'}_{density}_dev_01.xml",
        "preview": f"comparison_{kind}_{density}.png",
        "legacy_scene": {"scene_id": old["scene_id"], "filename": old["filename"],
                         "preview": old["preview"], "goal_m": old["goal_m"],
                         "bounds_m": old["obstacle_bounds_m"],
                         "obstacle_count": old["obstacle_count"]},
    }


def font(size: int):
    try:
        return ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", size)
    except OSError:
        return ImageFont.load_default()


def vertices(o: dict):
    return geom.box_vertices(o)


def render_panel(draw: ImageDraw.ImageDraw, scene: dict, rect: tuple[int, int, int, int],
                 title: str, old: bool = False) -> dict:
    x0, y0, x1, y1 = rect
    xb, yb = ( (-0.5, 7.5), (-3.0, 3.0) ) if old else OBSTACLE_BOUNDS
    plot = (x0 + 80, y0 + 92, x1 - 28, y1 - 124)
    sx, sy = (plot[2] - plot[0]) / (xb[1] - xb[0]), (plot[3] - plot[1]) / (yb[1] - yb[0])
    def pix(p):
        return (round(plot[0] + (p[0] - xb[0]) * sx), round(plot[3] - (p[1] - yb[0]) * sy))
    draw.text((x0 + 24, y0 + 16), title, fill="#17212b", font=font(23))
    draw.rectangle(plot, outline="#46515b", width=2)
    # Meter ticks make the displayed world scale explicit.
    for xv in range(math.ceil(xb[0]), math.floor(xb[1]) + 1):
        tx, ty = pix((xv, yb[0]))
        draw.line((tx, plot[3], tx, plot[3] + 6), fill="#46515b", width=1)
        draw.text((tx - 8, plot[3] + 8), str(xv), fill="#475569", font=font(11))
    for yv in range(math.ceil(yb[0]), math.floor(yb[1]) + 1):
        tx, ty = pix((xb[0], yv))
        draw.line((plot[0] - 6, ty, plot[0], ty), fill="#46515b", width=1)
        draw.text((plot[0] - 34, ty - 7), str(yv), fill="#475569", font=font(11))
    path = scene.get("reference_path_m") if not old else scene["routes"][0]["reference_path_m"]
    path_px = [pix(tuple(p)) for p in path]
    corridor_px = round(2 * 0.75 * (sx + sy) / 2)
    draw.line(path_px, fill="#cdebe6", width=corridor_px, joint="curve")
    draw.line(path_px, fill="#087e83" if not old else "#7c3aed", width=4, joint="curve")
    start, goal = (START, GOAL) if not old else (tuple(scene["start_m"]), tuple(scene["goal_m"]))
    ps, pg = pix(start), pix(goal)
    draw.line((*ps, *pg), fill="#e28a23", width=3)
    obs = scene["obstacles"]
    for i, o in enumerate(obs, 1):
        color = "#bb3c37" if old and o.get("role") == "gate_wall" else (
            "#bb3c37" if (not old and o["name"] in scene["path_check"]["direct_line_blocking_obstacles"]) else "#59636d")
        if geom.is_box(o):
            pts = [pix(p) for p in vertices(o)]
            draw.polygon(pts, fill=color, outline="#25313c")
        else:
            cx, cy = pix((o["x"], o["y"]))
            rad = float(o["radius"])
            draw.ellipse((cx-rad*sx, cy-rad*sy, cx+rad*sx, cy+rad*sy), fill=color, outline="#25313c", width=2)
        center = pix((o["x"], o["y"]))
        draw.text((center[0]+4, center[1]-16), f"{i:02d}", fill="white", font=font(12), stroke_width=1, stroke_fill="#222")
    for point, label, col in ((start, "START", "#21864a"), (goal, "GOAL", "#2357a5")):
        cx, cy = pix(point)
        draw.ellipse((cx-8, cy-8, cx+8, cy+8), fill=col, outline="white", width=2)
        draw.text((cx+9, cy-18), label, fill=col, font=font(13))
    draw.text((x0 + 24, y1 - 106), f"目标 {start[0]:.1f}→{goal[0]:.1f} m · 边界 x[{xb[0]:g},{xb[1]:g}], y[{yb[0]:g},{yb[1]:g}] m · 障碍 {len(obs)} 个",
              fill="#334155", font=font(15))
    if old:
        route_points = [tuple(p) for p in path]
        clear = geom.path_clearance(route_points, obs)
        length = sum(math.dist(a, b) for a, b in zip(route_points, route_points[1:]))
        headings, turns = angle_metrics(route_points)
        spacing = min(geom.shape_distance(a, b) for i, a in enumerate(obs) for b in obs[i+1:])
        max_px = max(projected_x(o) * 2 for o in obs)
        max_py = max(projected_y(o) * 2 for o in obs)
        core_groups: dict[float, list[dict]] = {}
        for o in obs:
            if o.get("role") == "gate_wall":
                core_groups.setdefault(round(float(o["x"]), 6), []).append(o)
        xs = sorted(core_groups)
        long_gaps = [b - a - max(projected_x(o) for o in core_groups[a]) -
                     max(projected_x(o) for o in core_groups[b]) for a, b in zip(xs, xs[1:])]
        min_long_gap = min(long_gaps) if long_gaps else math.nan
        info = (f"舊ID {scene['scene_id']} · 路長 {length:.2f}m · 最大方向角 {headings:.1f}° · "
                f"最小參考線障礙淨距 {clear:.2f}m · 最近物體間距 {spacing:.2f}m")
        size_info = (f"最大實際投影 x/y={max_px:.2f}/{max_py:.2f}m · 核心牆組縱向淨距 {min_long_gap:.2f}m")
    else:
        pc = scene["path_check"]
        max_px = max(projected_x(o) * 2 for o in obs)
        max_py = max(projected_y(o) * 2 for o in obs)
        gaps = pc["primary_obstacle_longitudinal_edge_gaps_m"]
        min_long_gap = min(gaps) if gaps else math.nan
        info = (f"新ID {scene['scene_id']} · 路長 {pc['reference_path_length_m']:.2f}m · 最大方向角 {pc['max_heading_from_forward_deg']:.1f}° · "
                f"最小參考線障礙淨距 {pc['minimum_edge_clearance_m']:.2f}m · 最近物體間距 {pc['actual_minimum_obstacle_edge_spacing_m']:.2f}m")
        size_info = (f"最大實際投影 x/y={max_px:.2f}/{max_py:.2f}m · 核心障礙縱向淨距 {min_long_gap:.2f}m")
    draw.text((x0 + 24, y1 - 77), info, fill="#334155", font=font(14))
    draw.text((x0 + 24, y1 - 58), size_info, fill="#475569", font=font(13))
    # All objects are visible; a compact numbered inventory follows each panel.
    if old:
        core = sum(1 for o in obs if o.get("role") == "gate_wall")
        gate_groups: dict[float, list[dict]] = {}
        for o in obs:
            if o.get("role") == "gate_wall":
                gate_groups.setdefault(round(float(o["x"]), 6), []).append(o)
        gate_openings = []
        for group in gate_groups.values():
            ordered = sorted(group, key=lambda o: float(o["y"]))
            gate_openings.extend(max(0.0, float(b["y"]) - geom.support_y(b) -
                                     (float(a["y"]) + geom.support_y(a)))
                                 for a, b in zip(ordered, ordered[1:]))
        min_gate_open = min(gate_openings) if gate_openings else math.nan
        open_text = f"红色为长墙段（{core}段）· 最窄实体门口 {min_gate_open:.2f}m · 浅带=0.75m/侧几何包络"
    else:
        extra = scene["path_check"]["extra_centerline_corridor_width_m"]
        open_text = f"红色为直线阻挡短物体；参考线额外包络宽 {extra:.2f}m（不是实物间隙）"
    draw.text((x0 + 24, y1 - 38), open_text, fill="#64748b", font=font(12))
    return {"plot_bounds": [*xb, *yb]}


def write_xml(scene: dict) -> str:
    # M1 samples are unregistered prototypes. Relative include keeps the Go2
    # model available without copying or changing model assets.
    include = "../../../../../unitree_mujoco/unitree_robots/go2/go2.xml"
    center_x = (OBSTACLE_BOUNDS[0][0] + OBSTACLE_BOUNDS[0][1]) / 2
    extent = max(OBSTACLE_BOUNDS[0][1]-OBSTACLE_BOUNDS[0][0],
                 OBSTACLE_BOUNDS[1][1]-OBSTACLE_BOUNDS[1][0])
    lines = ["<?xml version=\"1.0\" encoding=\"utf-8\"?>",
             f"<mujoco model=\"{scene['scene_slug']}\">",
             f"  <include file=\"{include}\" />",
             f"  <statistic center=\"{center_x:g} 0 0.1\" extent=\"{extent:g}\" />",
             "  <worldbody>",
             f"    <geom name=\"floor\" type=\"plane\" pos=\"{center_x:g} 0 0\" size=\"4.25 3.0 0.05\" rgba=\"0.2 0.3 0.4 1\" />"]
    for o in scene["obstacles"]:
        h = float(o["height_m"])
        if geom.is_box(o):
            lines.append(f"    <geom name=\"{o['name']}\" type=\"box\" pos=\"{o['x']:g} {o['y']:g} {h/2:g}\" euler=\"0 0 {o['yaw_rad']:g}\" size=\"{o['half_x']:g} {o['half_y']:g} {h/2:g}\" rgba=\"0.26 0.28 0.31 1\" />")
        else:
            lines.append(f"    <geom name=\"{o['name']}\" type=\"cylinder\" pos=\"{o['x']:g} {o['y']:g} {h/2:g}\" size=\"{o['radius']:g} {h/2:g}\" rgba=\"0.26 0.28 0.31 1\" />")
    lines.extend(("  </worldbody>", "</mujoco>", ""))
    text = "\n".join(lines)
    ET.fromstring(text)
    if not (OUT / include).resolve().is_file():
        raise FileNotFoundError(f"prototype model include does not resolve: {OUT / include}")
    return text


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    source = json.loads(OLD_LAYOUTS.read_text(encoding="utf-8"))
    old_by = {x["scene_id"]: x for x in source["scenes"]}
    specs = [
        ("s609_sa_d01", "gentle_s", "sparse", SEED_BASE + 1),
        ("s609_ma_d01", "gentle_s", "medium", SEED_BASE + 2),
        ("s609_so_d01", "offset_route", "sparse", SEED_BASE + 3),
        ("s609_mo_d01", "offset_route", "medium", SEED_BASE + 4),
    ]
    scenes = [make_scene(old_by[oldid], kind, density, seed)
              for oldid, kind, density, seed in specs]
    (OUT / "m1_stage2_samples.json").write_text(
        json.dumps({"version": "m1-stage2-prototypes/1.0", "status": "user_review_pending",
                    "field_bounds_m": {"x": list(OBSTACLE_BOUNDS[0]), "y": list(OBSTACLE_BOUNDS[1])},
                    "goal_m": list(GOAL), "scenes": scenes}, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8")
    comparison = []
    for (old_id, kind, density, _), new in zip(specs, scenes):
        old = copy.deepcopy(old_by[old_id])
        canvas = Image.new("RGB", (2200, 1040), "#f4f6f8")
        draw = ImageDraw.Draw(canvas)
        title = ("缓和 S 型" if kind == "gentle_s" else "错位短障碍路线") + f" · {density} · 旧 / 新几何对照"
        draw.text((45, 20), title, fill="#111827", font=font(29))
        draw.text((45, 58), "几何审图样例；浅色带表示机器人中心可用包络检查，不是策略路线输入或机器人实测轨迹。", fill="#475569", font=font(16))
        render_panel(draw, old, (20, 105, 1080, 1000), "旧版 · S6-09 历史地图", old=True)
        render_panel(draw, new, (1100, 105, 2180, 1000), "新版候选 · M1 阶段2样例")
        path = OUT / new["preview"]
        canvas.save(path)
        new["comparison_image"] = path.name
        (OUT / new["filename"]).write_text(write_xml(new), encoding="utf-8")
        comparison.append({"old_scene_id": old_id, "old_filename": old["filename"],
                           "new_scene_id": new["scene_id"], "new_filename": new["filename"],
                           "comparison_image": path.name})
    # Add image refs to the JSON after the list has all output filenames.
    (OUT / "m1_stage2_samples.json").write_text(
        json.dumps({"version": "m1-stage2-prototypes/1.0", "status": "user_review_pending",
                    "field_bounds_m": {"x": list(OBSTACLE_BOUNDS[0]), "y": list(OBSTACLE_BOUNDS[1])},
                    "goal_m": list(GOAL), "scenes": scenes, "old_new_pairs": comparison},
                   indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"sample_count": len(scenes), "maps": [
        {"id": s["scene_id"], "old": s["source_scene_id"],
         "footprint_pct": round(100 * s["footprint_area_fraction"], 3),
         "length_m": round(s["path_check"]["reference_path_length_m"], 3),
         "clearance_m": round(s["path_check"]["minimum_edge_clearance_m"], 3),
         "min_gap_m": round(s["path_check"]["actual_minimum_obstacle_edge_spacing_m"], 3),
         "max_angle_deg": round(s["path_check"]["max_heading_from_forward_deg"], 2),
         "direct_line_blocked": s["path_check"]["direct_line_blocked"]}
        for s in scenes], "output": str(OUT)}, indent=2))


if __name__ == "__main__":
    main()
