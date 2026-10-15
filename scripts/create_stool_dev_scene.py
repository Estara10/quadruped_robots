#!/usr/bin/env python3
"""Create one fixed-stool development scene; no controller or policy run."""
from __future__ import annotations

import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw

import generate_s6_08_scenes as geom
import generate_s6_11_maps as library

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "docs/thesis_project/designs/S6-11/stool_sample"
FILENAME = "scene_stool_dev_01.xml"
SCENE_ID = "stool_dev_01"


def create():
    # Dimensions in metres; no joints means these geoms are fixed to the world.
    stools = [(2.3, 0.0, 0.0), (4.8, -0.65, math.radians(18))]
    pieces, footprints = [], []
    for number, (x, y, yaw) in enumerate(stools, 1):
        prefix = f"ppt_obstacle_stool{number}"
        seat = dict(name=prefix + "_seat", shape="box", x=x, y=y,
                    half_x=0.30, half_y=0.30, yaw_rad=yaw, height_m=0.06,
                    z_m=0.49, role="seat", furniture=f"stool{number}")
        pieces.append(seat)
        footprints.append(seat)
        for leg, (lx, ly) in enumerate(((-.23, -.23), (-.23, .23), (.23, -.23), (.23, .23)), 1):
            pieces.append(dict(name=prefix + f"_leg{leg}", shape="box",
                               x=x + lx*math.cos(yaw) - ly*math.sin(yaw),
                               y=y + lx*math.sin(yaw) + ly*math.cos(yaw),
                               half_x=.035, half_y=.035, yaw_rad=yaw,
                               height_m=.46, z_m=.23, role="leg", furniture=f"stool{number}"))
    route = [(0., 0.), (1.25, 1.10), (5.85, 1.10), (7., 0.)]
    # Check continuous segment distances against the conservative full seat footprints.
    clearance = min(geom.segment_shape_distance(a, b, o)
                    for a, b in zip(route, route[1:]) for o in footprints)
    if clearance < geom.REQUIRED_CENTER_CLEARANCE_M:
        raise ValueError(f"insufficient route clearance: {clearance}")
    for o in pieces:
        for x, y in geom.box_vertices(o):
            if not (-.5 <= x <= 7.5 and -3 <= y <= 3):
                raise ValueError("stool outside bounds")
        if min(geom.point_shape_distance(p, o) for p in (geom.START, geom.GOAL)) < .95:
            raise ValueError("endpoint clearance")
    scene = dict(filename=FILENAME, scene_id=SCENE_ID, scene_slug="fixed stools development 01",
                 density="development sample; not in 60-map library", generation_seed=None,
                 split="development only", obstacle_count=len(pieces), furniture_count=2,
                 obstacles=pieces, routes=[dict(reference_path_m=route)],
                 route_structure="short_local_bypass",
                 density_definition=dict(obstacle_footprint_area_fraction=.72/48,
                                         minimum_obstacle_spacing_m=geom.shape_distance(*footprints)),
                 direct_line_blocking_obstacles=[footprints[0]["name"]],
                 minimum_route_clearance_m=clearance,
                 ray_semantics="Existing 2D XY projection of each seat and leg; conservative occupied footprint, no under-seat navigation.",
                 collision_semantics="Ten physical boxes: two seats and eight legs; no hidden solid volume beneath seats.",
                 dimensions_m=dict(seat_width=.6, seat_depth=.6, seat_thickness=.06,
                                   total_height=.52, leg_width=.07, leg_height=.46))
    OUT.mkdir(parents=True, exist_ok=True)
    xml = ET.fromstring(geom.render_xml(scene))
    world = xml.find("worldbody")
    by_name = {o["name"]: o for o in pieces}
    for node in world.findall("geom"):
        if node.get("name") in by_name:
            o = by_name[node.get("name")]
            node.set("pos", f'{o["x"]:.9f} {o["y"]:.9f} {o["z_m"]:.9f}')
            node.set("rgba", "0.65 0.38 0.17 1" if o["role"] == "seat" else "0.28 0.18 0.10 1")
    ET.indent(xml)
    (geom.SCENE_DIR / FILENAME).write_text(ET.tostring(xml, encoding="unicode") + "\n")
    # Register without replacing a previously calibrated C++ fingerprint on regeneration.
    existing = None
    try:
        import abs_scene
        existing = abs_scene.resolve_scene(FILENAME).model_fingerprint
    except (ValueError, RuntimeError, FileNotFoundError):
        pass
    rows = library.catalog_rows([scene], {FILENAME: existing} if existing else None)
    library.append_or_replace_catalog(rows, upsert=True)
    (OUT / "layout.json").write_text(json.dumps({"scenes": [scene]}, ensure_ascii=False, indent=2) + "\n")
    library.render_scene(scene).save(OUT / "top_view.png")

    # Oblique schematic uses the same three-dimensional box corners as the XML.
    image = Image.new("RGB", (1200, 850), "#f5f7fa")
    draw = ImageDraw.Draw(image)
    def project(x, y, z):
        return (round(200 + 100*x + 40*y), round(500 + 22*x - 48*y - 190*z))
    draw.polygon([project(x, y, 0) for x, y in [(-.5,-3),(7.5,-3),(7.5,3),(-.5,3)]], fill="#e6ebee", outline="#9aa7b1")
    for o in sorted(pieces, key=lambda v: (v["z_m"], -v["x"]-v["y"])):
        bottom = [project(x,y,o["z_m"]-o["height_m"]/2) for x,y in geom.box_vertices(o)]
        top = [project(x,y,o["z_m"]+o["height_m"]/2) for x,y in geom.box_vertices(o)]
        for i in range(4):
            j = (i+1)%4
            draw.polygon([bottom[i],bottom[j],top[j],top[i]], fill="#80532f", outline="#493622")
        draw.polygon(top, fill="#b8844e", outline="#493622")
    for (x,y), label in [(geom.START,"START"),(geom.GOAL,"GOAL")]:
        px,py=project(x,y,0)
        draw.ellipse((px-7,py-7,px+7,py+7), fill="#21864a")
        draw.text((px+10,py-22),label,fill="#17212b",font=library.font(18))
    draw.text((45,25),"静态四脚凳开发样例：2张凳子，10个真实碰撞构件",fill="#17212b",font=library.font(25))
    draw.text((45,70),"座面 0.60×0.60 m；总高 0.52 m；凳腿宽 0.07 m。示意图，不是仿真截图。",fill="#334155",font=library.font(18))
    draw.text((45,795),"当前射线使用座面/凳腿的二维投影，保守禁止钻凳；实际碰撞保留凳腿间空隙。",fill="#334155",font=library.font(18))
    image.save(OUT / "stools_oblique.png")
    print(f"created {FILENAME} id={SCENE_ID} geoms=10 route_clearance={clearance:.3f}m")


if __name__ == "__main__":
    create()
