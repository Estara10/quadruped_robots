#!/usr/bin/env python3
"""Render the five frozen layouts.json entries as deterministic Go2 MJCF scenes."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
LAYOUTS = ROOT / "docs/thesis_project/designs/random_static_scenes/layouts.json"
SCENE_DIR = ROOT / "unitree_mujoco/unitree_robots/go2"


def f(value: float) -> str:
    return format(float(value), ".17g")


def point_segment_distance(point: tuple[float, float], a: list[float], b: list[float]) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    t = max(0.0, min(1.0, ((point[0] - a[0]) * dx + (point[1] - a[1]) * dy) /
                         (dx * dx + dy * dy)))
    return math.hypot(point[0] - a[0] - t * dx, point[1] - a[1] - t * dy)


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
        if args.check:
            if not path.is_file() or path.read_text(encoding="utf-8") != rendered:
                raise SystemExit(f"generated scene differs: {path}")
        else:
            path.write_text(rendered, encoding="utf-8")
        overruns = bounds_overruns(document, scene)
        note = f" declared_bounds_overrun={overruns}" if overruns else " declared_bounds_overrun=none"
        print(f"{path.name}: seed={scene['seed']} obstacles={len(scene['obstacles'])} check={'ok' if args.check else 'written'}{note}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
