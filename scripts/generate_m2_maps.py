#!/usr/bin/env python3
"""Create six independent M2 development candidates, without policy runs.

Uses the existing exact shape-distance helpers. M1 is never regenerated.
Reference routes are geometric witnesses, not commands supplied to the robot.
"""
from __future__ import annotations

import csv
import argparse
import json
import math
from pathlib import Path
import random

from PIL import Image, ImageDraw
import generate_m1_library as m1

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/thesis_project/designs/M2/candidate_v1"
VERSION = "m2-development-candidates/0.1"

# Coordinates describe distinct local challenge structures, not long walls.
# Every recipe is completed with short off-axis objects near its forward route.
RECIPES = [
    ("连续正面短障碍", "连续接近风险；检查恢复后再次遇障碍的表现", [
        (1.75, 0.00, "cylinder"), (3.65, 0.12, "box"), (5.65, -0.10, "cylinder")]),
    ("左右错列混合障碍", "左右偏置交替；增加局部转向与模式切换需求", [
        (1.90, -0.12, "box"), (3.75, -1.25, "cylinder"), (5.60, 0.12, "box")]),
    ("两段偏置开口", "两处宽开口偏向不同侧；没有横贯场地的墙", [
        (1.90, -0.12, "box"), (1.90, 2.15, "cylinder"),
        (5.60, 0.12, "box"), (5.60, -2.15, "cylinder")]),
    ("斜箱与圆柱交替", "短斜箱与圆柱轮廓交替；检查不同入射角下避障", [
        (1.80, 0.02, "rotated_bar"), (3.80, -0.18, "cylinder"),
        (5.75, 0.16, "rotated_bar")]),
    ("后段连续障碍", "前段开放、后段两次接近风险；观察高速接近后恢复", [
        (2.90, 0.00, "cylinder"), (4.70, -0.18, "box"), (6.20, 0.22, "cylinder")]),
    ("混合错位局部绕行", "三个局部阻挡配合侧向物体；保持较宽转向空间", [
        (1.90, 0.12, "cylinder"), (3.75, 1.85, "rotated_bar"),
        (5.60, -0.12, "box")]),
]


def make_obstacle(rng, index, x, y, shape, role):
    o = {"name": f"ppt_obstacle_{index:02d}", "shape": shape,
         "x": round(x, 6), "y": round(y, 6), "role": role,
         "height_m": round(rng.uniform(0.38, 0.58), 6)}
    if shape == "cylinder":
        o.update(radius=round(rng.uniform(0.23, 0.30), 6), yaw_rad=0.0)
    else:
        o.update(half_x=round(rng.uniform(0.23, 0.33), 6),
                 half_y=round(rng.uniform(0.18, 0.25), 6),
                 yaw_rad=round(rng.uniform(0.25, 0.50) * rng.choice((-1, 1))
                               if shape == "rotated_bar" else rng.uniform(-0.16, 0.16), 6))
    return o


def scene(number, label, intent, recipe):
    seed = 2026101600 + number
    for attempt in range(1, 101):
        rng = random.Random(seed + attempt * 100003)
        obs = [make_obstacle(rng, i, x + rng.uniform(-0.06, 0.06),
                             y + rng.uniform(-0.04, 0.04), shape, "primary")
               for i, (x, y, shape) in enumerate(recipe, 1)]
        if not all(m1.within_limits(o) for o in obs):
            continue
        if not m1.geom.no_overlap_and_spacing(obs, 0.60):
            continue
        path = reference_path(number, obs)
        if path is None:
            continue
        # Add local flank obstacles, rather than filling only map edges.
        for _ in range(160):
            if len(obs) >= 9 + (number % 2):
                break
            x = rng.uniform(1.1, 6.5)
            y = rng.choice((-1, 1)) * rng.uniform(0.85, 2.45)
            cand = make_obstacle(rng, len(obs) + 1, x, y,
                                 rng.choice(("box", "cylinder")), "flank")
            if (m1.within_limits(cand)
                    and m1.geom.no_overlap_and_spacing(obs + [cand], 0.60)
                    and m1.geom.path_clearance(path, [cand]) >= 0.80):
                obs.append(cand)
        if len(obs) < 8:
            continue
        # Recompute the route through the entire final obstacle set.
        path = reference_path(number, obs)
        if path is None or max(abs(y) for _, y in path) > 1.55:
            continue
        clearance = m1.geom.path_clearance(path, obs)
        if clearance < 0.75:
            continue
        blockers = m1.geom.line_blockers(obs)
        if not blockers:
            continue
        metrics = m1.geom.path_metrics(path, obs)
        headings = [abs(math.degrees(math.atan2(b[1]-a[1], b[0]-a[0])))
                    for a, b in zip(path, path[1:])]
        metrics["max_forward_heading_deg"] = max(headings, default=0)
        if metrics["max_forward_heading_deg"] > 45.000001:
            continue
        return {"scene_id": f"m2d{number:02d}", "filename": f"m2dev{number:02d}.xml",
                "preview": f"m2dev{number:02d}.png", "display_name": f"M2-{number:02d}",
                "model_name": f"m2dev{number:02d}", "generation_seed": seed,
                "generation_attempt": attempt, "generator_version": VERSION,
                "split": "development_candidate", "density": "challenge_candidate",
                "route_structure_label_zh": label, "challenge_intent": intent,
                "difficulty_verified_by_policy": False,
                "start_m": list(m1.START), "goal_m": list(m1.GOAL),
                "field_bounds_m": {"x": [-0.5, 8.0], "y": [-3.0, 3.0]},
                "field_is_physical_wall": False, "obstacle_count": len(obs),
                "obstacles": obs, "direct_line_blocking_obstacles": blockers,
                "direct_line_blocked": True,
                "routes": [{"route_id": "geometry_only", "reference_path_m": path}],
                "path_check": metrics,
                "density_definition": {
                    "obstacle_footprint_area_fraction": m1.area(obs)/m1.FIELD_AREA,
                    "minimum_obstacle_spacing_m": min(m1.geom.shape_distance(a,b)
                        for i,a in enumerate(obs) for b in obs[i+1:])}}
    raise RuntimeError(f"No feasible candidate for M2-{number:02d} in bounded generation")


def reference_path(number, obstacles):
    if number in (2, 3, 6):
        sign = -1 if number == 6 else 1
        path = [(7.5*i/600, sign*1.10*math.sin(2*math.pi*i/600))
                for i in range(601)]
        return path if m1.geom.path_clearance(path, obstacles) >= 0.75 else None
    return m1.s611.grid_forward_path(obstacles)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runtime-fingerprints", type=Path,
                        help="Register already-generated XML using production C++ identity output")
    args = parser.parse_args()
    if args.runtime_fingerprints:
        register(args.runtime_fingerprints)
        return
    m1.configure_geometry()
    OUT.mkdir(parents=True, exist_ok=True)
    scenes = [scene(i, *recipe) for i, recipe in enumerate(RECIPES, 1)]
    # Render at equal metric scale: x and y share one pixels-per-metre value.
    m1.BOUNDS = ((-0.5, 8.0), (-3.0, 3.0))
    images = []
    for s in scenes:
        im = render(s)
        im.save(OUT / s["preview"])
        images.append(im)
        (m1.SCENE_DIR / s["filename"]).write_text(m1.geom.render_xml(s), encoding="utf-8")
        print(f"{s['display_name']} {s['obstacle_count']} obstacles; "
              f"clearance={s['path_check']['minimum_geometric_obstacle_clearance_m']:.3f}m; "
              f"route={s['path_check']['reference_path_length_m']:.2f}m", flush=True)
    overview = Image.new("RGB", (1500, 1680), "#edf1f4")
    d = ImageDraw.Draw(overview)
    d.text((25, 12), "M2：6张独立开发候选图 · 几何通路已生成，策略难度未验证",
           fill="#17212b", font=m1.font(24))
    for i, im in enumerate(images):
        tile = im.resize((740, 540))
        overview.paste(tile, (5 + (i % 2)*750, 50 + (i // 2)*540))
    overview.save(OUT / "overview_m2_6.png")
    (OUT / "layouts.json").write_text(json.dumps({"mapset_version": VERSION,
        "purpose": "round-two parameter-development candidates; not an independent final test set",
        "policy_runs": 0, "m1_modified": False, "scenes": scenes}, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    with (OUT / "scene_index.csv").open("w", newline="", encoding="utf-8") as f:
        keys = ("filename", "scene_id", "route_structure_label_zh", "challenge_intent",
                "generation_seed", "obstacle_count")
        w = csv.DictWriter(f, fieldnames=keys); w.writeheader()
        w.writerows({k: s[k] for k in keys} for s in scenes)


def register(identity_file):
    doc = json.loads((OUT / "layouts.json").read_text(encoding="utf-8"))
    entries = {}
    for line in identity_file.read_text(encoding="utf-8").splitlines():
        name, fingerprint, version = line.split()
        if len(fingerprint) != 64 or any(c not in "0123456789abcdef" for c in fingerprint):
            raise ValueError("Malformed runtime model fingerprint")
        if name in entries:
            raise ValueError("Duplicate runtime identity")
        entries[name] = (fingerprint, version)
    if set(entries) != {s["filename"] for s in doc["scenes"]}:
        raise ValueError("Runtime identities must cover exactly these six M2 candidates")
    rows = m1.s611.catalog_rows(doc["scenes"], {name:v[0] for name,v in entries.items()})
    m1.s611.append_or_replace_catalog(rows, upsert=True)
    for s,row in zip(doc["scenes"], rows):
        s["production_scene_binding"] = {**row, "runtime_mujoco_version": entries[s["filename"]][1]}
    doc["registration"] = "catalog registered; simulator rebuild required; no policy run"
    (OUT / "layouts.json").write_text(json.dumps(doc, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print("Registered six M2 scene bindings using production MuJoCo fingerprints")


def render(s):
    im = Image.new("RGB", (1080, 790), "white")
    d = ImageDraw.Draw(im)
    scale, left, top = 105, 95, 100
    def p(q): return (left+(q[0]+0.5)*scale, top+(3.0-q[1])*scale)
    d.text((40, 15), f"{s['display_name']} · {s['route_structure_label_zh']}",
           fill="#17212b", font=m1.font(26))
    d.text((40, 55), f"障碍{s['obstacle_count']}个 · 最小物体间距 "
           f"{s['density_definition']['minimum_obstacle_spacing_m']:.2f}m · 种子{s['generation_seed']}",
           fill="#475569", font=m1.font(17))
    for x in range(9):
        d.line((p((x,-3)),p((x,3))),fill="#edf0f2")
    for y in range(-3,4):
        d.line((p((-.5,y)),p((8,y))),fill="#edf0f2")
    route = s["routes"][0]["reference_path_m"]
    points = [p(q) for q in route]
    radius = 0.75*scale
    d.line(points, fill="#d8eeeb", width=round(2*radius), joint="curve")
    for x,y in points:
        d.ellipse((x-radius,y-radius,x+radius,y+radius),fill="#d8eeeb")
    # The declared field boundary below is not a physical wall.
    d.line(points,fill="#087e83",width=3)
    d.line((p(m1.START),p(m1.GOAL)),fill="#df9636",width=2)
    blockers = set(s["direct_line_blocking_obstacles"])
    for o in s["obstacles"]:
        color = "#bd4540" if o["name"] in blockers else "#586672"
        if m1.geom.is_box(o):
            d.polygon([p(q) for q in m1.geom.box_vertices(o)],fill=color,outline="#263238")
        else:
            x,y=p((o["x"],o["y"]));r=o["radius"]*scale
            d.ellipse((x-r,y-r,x+r,y+r),fill=color,outline="#263238")
        d.text(p((o["x"]+.03,o["y"]+.04)),o["name"][-2:],fill="white",font=m1.font(12))
    d.rectangle((p((-.5,3)),p((8,-3))),outline="#475569",width=2)
    for q,label,color in ((m1.START,"START", "#21864a"),(m1.GOAL,"GOAL", "#2357a5")):
        x,y=p(q);d.ellipse((x-7,y-7,x+7,y+7),fill=color)
        d.text((x-20,y+12),label,fill=color,font=m1.font(15))
    d.text((40, 745), "等比例俯视；浅青带半宽0.75m。参考线仅作几何通路证明，不输入策略。",fill="#334155",font=m1.font(17))
    return im


if __name__ == "__main__":
    main()
