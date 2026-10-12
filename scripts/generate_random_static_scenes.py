#!/usr/bin/env python3
"""Render fixed layouts as deterministic Go2 MJCF scenes and geometry previews."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
LAYOUTS = ROOT / "docs/thesis_project/designs/random_static_scenes/layouts.json"
SCENE_DIR = ROOT / "unitree_mujoco/unitree_robots/go2"
PREVIEW_DIR = ROOT / "docs/thesis_project/designs/random_static_scenes"


def f(value: float) -> str:
    return format(float(value), ".17g")


def point_segment_distance(point: tuple[float, float], a: list[float], b: list[float]) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) /
                         (dx * dx + dy * dy)))
    return math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy)


def shapes_overlap(a: dict, b: dict) -> bool:
    """Exact positive-area overlap check for axis-aligned boxes and cylinders."""
    if a["shape"] == b["shape"] == "box":
        return (abs(float(a["x"]) - float(b["x"])) < float(a["half_x"]) + float(b["half_x"]) and
                abs(float(a["y"]) - float(b["y"])) < float(a["half_y"]) + float(b["half_y"]))
    if a["shape"] == b["shape"] == "cylinder":
        return math.hypot(float(a["x"]) - float(b["x"]), float(a["y"]) - float(b["y"])) < float(a["radius"]) + float(b["radius"])
    box, cylinder = (a, b) if a["shape"] == "box" else (b, a)
    dx = max(abs(float(cylinder["x"]) - float(box["x"])) - float(box["half_x"]), 0.0)
    dy = max(abs(float(cylinder["y"]) - float(box["y"])) - float(box["half_y"]), 0.0)
    return dx * dx + dy * dy < float(cylinder["radius"]) ** 2


def validate_layout(document: dict, scene: dict) -> None:
    waypoints = scene["path_waypoints_m"]
    width = float(document["reserved_corridor_width_m"])
    for index, obstacle in enumerate(scene["obstacles"], 1):
        # A circumscribed circle is a conservative XY envelope for both shapes.
        radius = (float(obstacle["radius"]) if obstacle["shape"] == "cylinder" else
                  math.hypot(float(obstacle["half_x"]), float(obstacle["half_y"])))
        center = (float(obstacle["x"]), float(obstacle["y"]))
        clearance = min(point_segment_distance(center, a, b)
                        for a, b in zip(waypoints, waypoints[1:])) - radius
        if clearance <= width / 2:
            raise ValueError(f"{scene['name']} obstacle {index} invades reserved corridor: {clearance:.9f} m")
    for index, obstacle in enumerate(scene["obstacles"]):
        for other_index in range(index + 1, len(scene["obstacles"])):
            if shapes_overlap(obstacle, scene["obstacles"][other_index]):
                raise ValueError(f"{scene['name']} obstacles {index + 1} and {other_index + 1} overlap")


def bounds_overruns(document: dict, scene: dict) -> list[dict]:
    x_bounds = document["world_bounds_m"]["x"]
    y_bounds = document["world_bounds_m"]["y"]
    overruns = []
    for index, obstacle in enumerate(scene["obstacles"], 1):
        radius = float(obstacle.get("radius", 0.0))
        half_x = radius if radius else float(obstacle["half_x"])
        half_y = radius if radius else float(obstacle["half_y"])
        outside = {
            "x_min_m": max(0.0, x_bounds[0] - (obstacle["x"] - half_x)),
            "x_max_m": max(0.0, (obstacle["x"] + half_x) - x_bounds[1]),
            "y_min_m": max(0.0, y_bounds[0] - (obstacle["y"] - half_y)),
            "y_max_m": max(0.0, (obstacle["y"] + half_y) - y_bounds[1]),
        }
        outside = {axis: delta for axis, delta in outside.items() if delta > 0.0}
        if outside:
            overruns.append({"obstacle_index_1based": index, "overrun_m": outside})
    return overruns


def blocking_obstacles(scene: dict) -> dict[int, tuple[float, float]]:
    """Return 1-based obstacle IDs and their positive-width x span on y=0."""
    spans = {}
    for index, obstacle in enumerate(scene["obstacles"], 1):
        y = float(obstacle["y"])
        if obstacle["shape"] == "cylinder":
            radius = float(obstacle["radius"])
            if abs(y) < radius:
                half_span = math.sqrt(radius * radius - y * y)
                spans[index] = (float(obstacle["x"]) - half_span,
                                float(obstacle["x"]) + half_span)
        else:
            if abs(y) < float(obstacle["half_y"]):
                half_span = float(obstacle["half_x"])
                spans[index] = (float(obstacle["x"]) - half_span,
                                float(obstacle["x"]) + half_span)
    return spans


def render_preview(document: dict, scene: dict) -> Image.Image:
    """Draw exact top-down shapes, the direct line, and the reserved bypass lane."""
    image = Image.new("RGB", (1120, 880), "white")
    draw = ImageDraw.Draw(image)
    try:
        font = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 18)
        small = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 14)
        title_font = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 22)
    except OSError:
        font = small = title_font = ImageFont.load_default()
    xb, yb = document["world_bounds_m"]["x"], document["world_bounds_m"]["y"]
    left, top, right, bottom = 100, 100, 1020, 790
    sx = (right - left) / (xb[1] - xb[0])
    sy = (bottom - top) / (yb[1] - yb[0])
    def xy(x: float, y: float) -> tuple[int, int]:
        return (round(left + (x - xb[0]) * sx), round(top + (yb[1] - y) * sy))
    draw.text((left, 24), scene["title"], fill="#17212b", font=title_font)
    draw.text((left, 58), f"{document.get('layout_revision', 'unversioned')} · seed {scene['seed']} · "
              f"reserved bypass lane {document['reserved_corridor_width_m']:.2f} m (geometry only; not controller input)",
              fill="#475569", font=small)
    draw.rectangle((left, top, right, bottom), outline="#46515b", width=2)

    # Preserve-channel centerline and its 1.10 m footprint are display-only geometry.
    waypoints = [tuple(map(float, p)) for p in scene["path_waypoints_m"]]
    lane_points = [xy(*p) for p in waypoints]
    lane_px = round(float(document["reserved_corridor_width_m"]) * (sx + sy) / 2)
    draw.line(lane_points, fill="#c7e8e6", width=lane_px, joint="curve")
    for point in lane_points:
        radius = lane_px // 2
        draw.ellipse((point[0]-radius, point[1]-radius, point[0]+radius, point[1]+radius), fill="#c7e8e6")
    draw.line(lane_points, fill="#167c80", width=2, joint="curve")

    # Finite start-to-goal segment, not an infinite line.
    start, goal = tuple(document["start_m"]), tuple(document["goal_m"])
    p0, p1 = xy(*start), xy(*goal)
    dash = 15
    total = math.dist(p0, p1)
    for offset in range(0, int(total), 2 * dash):
        a = offset / total
        b = min(offset + dash, total) / total
        draw.line((round(p0[0] + (p1[0]-p0[0])*a), round(p0[1] + (p1[1]-p0[1])*a),
                   round(p0[0] + (p1[0]-p0[0])*b), round(p0[1] + (p1[1]-p0[1])*b)),
                  fill="#e38b26", width=3)

    blockers = blocking_obstacles(scene)
    for index, obstacle in enumerate(scene["obstacles"], 1):
        x, y = float(obstacle["x"]), float(obstacle["y"])
        color = "#bd3c37" if index in blockers else "#555d66"
        outline = "#7f1d1d" if index in blockers else "#26323d"
        if obstacle["shape"] == "cylinder":
            rx = float(obstacle["radius"]) * sx
            ry = float(obstacle["radius"]) * sy
            px, py = xy(x, y)
            draw.ellipse((round(px-rx), round(py-ry), round(px+rx), round(py+ry)), fill=color, outline=outline, width=2)
        else:
            a = xy(x-float(obstacle["half_x"]), y+float(obstacle["half_y"]))
            b = xy(x+float(obstacle["half_x"]), y-float(obstacle["half_y"]))
            draw.rectangle((*a, *b), fill=color, outline=outline, width=2)
        px, py = xy(x, y)
        draw.text((px+5, py+4), str(index), fill="white", font=small, stroke_width=1, stroke_fill=outline)

    for point, label, color in ((start, "START", "#21864a"), (goal, "GOAL", "#2357a5")):
        px, py = xy(*point)
        draw.ellipse((px-8, py-8, px+8, py+8), fill=color, outline="white", width=2)
        draw.text((px+10, py-17), label, fill=color, font=small)
    draw.text((left, 812), "Orange dashed: start→goal segment   Red: blocking obstacle   Teal: reserved bypass centerline / 1.10 m corridor",
              fill="#334155", font=font)
    draw.text((left, 842), f"Line blockers: {', '.join(map(str, blockers)) if blockers else 'none'}",
              fill="#7f1d1d" if blockers else "#475569", font=font)
    return image


def render(document: dict, scene: dict) -> str:
    validate_layout(document, scene)
    center_x = (document["world_bounds_m"]["x"][0] + document["world_bounds_m"]["x"][1]) / 2
    center_y = (document["world_bounds_m"]["y"][0] + document["world_bounds_m"]["y"][1]) / 2
    extent = max(document["world_bounds_m"]["x"][1] - document["world_bounds_m"]["x"][0],
                 document["world_bounds_m"]["y"][1] - document["world_bounds_m"]["y"][0])
    lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        f'<mujoco model="go2 {scene["name"]}">',
        '  <include file="go2.xml" />',
        f'  <statistic center="{f(center_x)} {f(center_y)} 0.1" extent="{f(extent)}" />',
        '  <visual>',
        '    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0" />',
        '    <rgba haze="0.15 0.25 0.35 1" />',
        '    <global azimuth="135" elevation="-40" />',
        '  </visual>',
        '  <asset>',
        '    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072" />',
        '    <texture type="2d" name="random_groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300" />',
        '    <material name="random_groundplane" texture="random_groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2" />',
        '  </asset>',
        '  <worldbody>',
        '    <light pos="1 0 7" dir="0 0 -1" directional="true" />',
        f'    <geom name="floor" type="plane" pos="{f(center_x)} {f(center_y)} 0" size="{f(extent / 2)} {f((document["world_bounds_m"]["y"][1] - document["world_bounds_m"]["y"][0]) / 2 + 0.5)} 0.05" material="random_groundplane" />',
    ]
    for index, obstacle in enumerate(scene["obstacles"], 1):
        name = f"ppt_obstacle_{index:02d}"
        x, y, z = float(obstacle["x"]), float(obstacle["y"]), float(obstacle["height"]) / 2
        if obstacle["shape"] == "box":
            size = (float(obstacle["half_x"]), float(obstacle["half_y"]), float(obstacle["height"]) / 2)
            lines.append(f'    <geom name="{name}" type="box" pos="{f(x)} {f(y)} {f(z)}" size="{f(size[0])} {f(size[1])} {f(size[2])}" rgba="0.26 0.28 0.31 1" />')
        elif obstacle["shape"] == "cylinder":
            lines.append(f'    <geom name="{name}" type="cylinder" pos="{f(x)} {f(y)} {f(z)}" size="{f(obstacle["radius"])} {f(float(obstacle["height"]) / 2)}" rgba="0.26 0.28 0.31 1" />')
        else:
            raise ValueError(f"unsupported shape {obstacle['shape']!r}")
    lines.extend(['  </worldbody>', '</mujoco>', ''])
    xml = "\n".join(lines)
    ET.fromstring(xml)
    return xml


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="check generated files without writing")
    args = parser.parse_args()
    document = json.loads(LAYOUTS.read_text(encoding="utf-8"))
    scenes = document["scenes"]
    if len(scenes) != 5:
        raise ValueError(f"expected five frozen layouts, found {len(scenes)}")
    for index, scene in enumerate(scenes, 1):
        if scene["name"] != f"random_scene_{index:02d}":
            raise ValueError(f"unexpected layout order/name: {scene['name']}")
        path = SCENE_DIR / f"scene_random_{index:02d}.xml"
        rendered = render(document, scene)
        preview_path = PREVIEW_DIR / scene["preview"]
        preview = render_preview(document, scene)
        expected_blockers = {1: [1], 2: [3], 3: [4], 4: [15], 5: []}[index]
        actual_blockers = list(blocking_obstacles(scene))
        if actual_blockers != expected_blockers:
            raise ValueError(f"{scene['name']} line blockers {actual_blockers} != expected {expected_blockers}")
        if args.check:
            if not path.is_file() or path.read_text(encoding="utf-8") != rendered:
                raise SystemExit(f"generated scene differs: {path}")
            if not preview_path.is_file() or Image.open(preview_path).convert("RGB").tobytes() != preview.tobytes():
                raise SystemExit(f"generated preview differs: {preview_path}")
        else:
            path.write_text(rendered, encoding="utf-8")
            preview.save(preview_path, format="PNG", optimize=True)
        overruns = bounds_overruns(document, scene)
        note = f" declared_bounds_overrun={overruns}" if overruns else " declared_bounds_overrun=none"
        print(f"{path.name}: seed={scene['seed']} obstacles={len(scene['obstacles'])} "
              f"line_blockers={actual_blockers} preview={'ok' if args.check else 'written'} "
              f"check={'ok' if args.check else 'written'}{note}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
