#!/usr/bin/env python3
"""Create four deterministic S6-11 common-object development variants.

No robot or policy is started. Collision objects are static primitive MuJoCo
geometries. Ray input continues to use the production 2-D XY projection.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import generate_s6_08_scenes as geom  # noqa: E402
import generate_s6_11_maps as maps  # noqa: E402

LAYOUTS = ROOT / "docs/thesis_project/designs/S6-11/layouts_s6_11.json"
OUT = ROOT / "docs/thesis_project/designs/S6-11/common_object_variants"
SCENE_DIR = geom.SCENE_DIR
GENERATOR_VERSION = "s6-11-common-objects/1.0"

# source scene -> (source geom name, object kind, instance number)
SUBSTITUTIONS = {
    "s611_sl_d01": [
        ("ppt_obstacle_01", "stool", 1),
        ("ppt_obstacle_02", "small_table", 2),
    ],
    "s611_ml_d01": [
        ("ppt_obstacle_01", "stool", 1),
        ("ppt_obstacle_02", "office_desk", 2),
        ("ppt_obstacle_03", "trash_bin", 3),
    ],
    "s611_sq_d01": [
        ("ppt_obstacle_01", "planter", 1),
        ("ppt_obstacle_02", "trash_bin", 2),
        ("ppt_obstacle_03", "bollard", 3),
    ],
    "s611_mq_d01": [
        ("ppt_obstacle_01", "small_table", 1),
        ("ppt_obstacle_02", "planter", 2),
        ("ppt_obstacle_03", "bollard", 3),
    ],
}
IDS = {
    "s611_sl_d01": ("scene_s6_11_sparse_local_objects_dev_01.xml", "s611_obj_sl_d01"),
    "s611_ml_d01": ("scene_s6_11_medium_local_objects_dev_01.xml", "s611_obj_ml_d01"),
    "s611_sq_d01": ("scene_s6_11_sparse_offset_objects_dev_01.xml", "s611_obj_sq_d01"),
    "s611_mq_d01": ("scene_s6_11_medium_offset_objects_dev_01.xml", "s611_obj_mq_d01"),
}

OBJECT_LABELS = {
    "stool": "四脚凳", "small_table": "小桌", "office_desk": "办公桌",
    "trash_bin": "垃圾桶", "planter": "花盆", "bollard": "路桩",
}
COLORS = {
    "stool": (0.68, 0.40, 0.18, 1), "small_table": (0.62, 0.35, 0.15, 1),
    "office_desk": (0.48, 0.29, 0.16, 1), "trash_bin": (0.18, 0.30, 0.36, 1),
    "planter": (0.50, 0.27, 0.18, 1), "bollard": (0.83, 0.56, 0.12, 1),
}


def transform(x: float, y: float, dx: float, dy: float, yaw: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return x + dx * c - dy * s, y + dx * s + dy * c


def make_piece(name: str, shape: str, x: float, y: float, yaw: float,
               height: float, z: float, role: str, object_id: str,
               half_x: float | None = None, half_y: float | None = None,
               radius: float | None = None) -> dict:
    out = {"name": name, "shape": shape, "x": round(x, 6), "y": round(y, 6),
           "yaw_rad": round(yaw, 6), "height_m": round(height, 6), "z_m": round(z, 6),
           "role": role, "object_id": object_id}
    if half_x is not None:
        out["half_x"], out["half_y"] = round(half_x, 6), round(half_y, 6)
    if radius is not None:
        out["radius"] = round(radius, 6)
    return out


def build_object(kind: str, instance: int, source: dict) -> dict:
    x, y, yaw = float(source["x"]), float(source["y"]), float(source.get("yaw_rad", 0))
    oid = f"obj{instance:02d}_{kind}"
    prefix = f"ppt_obstacle_{oid}"
    pieces: list[dict] = []
    if kind in {"stool", "small_table", "office_desk"}:
        if kind == "stool":
            width, depth, total_h, top_t, leg_w, lx, ly = .52, .52, .50, .055, .06, .20, .20
            top_role, leg_role = "seat", "leg"
        elif kind == "small_table":
            width, depth, total_h, top_t, leg_w, lx, ly = .48, .42, .48, .05, .045, .18, .15
            top_role, leg_role = "tabletop", "leg"
        else:
            width, depth, total_h, top_t, leg_w, lx, ly = .58, .50, .72, .06, .05, .235, .19
            top_role, leg_role = "desktop", "leg"
        pieces.append(make_piece(prefix + "_" + top_role, "box", x, y, yaw,
                                 top_t, total_h - top_t / 2, top_role, oid,
                                 width / 2, depth / 2))
        leg_h = total_h - top_t
        leg_index = 0
        for dx in (-lx, lx):
            for dy in (-ly, ly):
                leg_index += 1
                px, py = transform(x, y, dx, dy, yaw)
                pieces.append(make_piece(prefix + f"_leg{leg_index}", "box", px, py, yaw,
                                         leg_h, leg_h / 2, leg_role, oid, leg_w / 2, leg_w / 2))
        dimensions = {"top_width_m": width, "top_depth_m": depth,
                      "total_height_m": total_h, "top_thickness_m": top_t,
                      "leg_cross_section_m": leg_w, "legs": 4}
    else:
        radius, height = {"trash_bin": (.18, .48), "planter": (.20, .44),
                          "bollard": (.12, .65)}[kind]
        pieces.append(make_piece(prefix + "_proxy", "cylinder", x, y, 0,
                                 height, height / 2, "proxy", oid, radius=radius))
        dimensions = {"proxy_shape": "vertical cylinder", "radius_m": radius,
                      "height_m": height,
                      "simplification": "uniform cylinder; rim/taper/handles/details omitted"}
    return {"object_id": oid, "kind": kind, "label_zh": OBJECT_LABELS[kind],
            "center_m": [round(x, 6), round(y, 6)], "yaw_rad": round(yaw, 6),
            "dimensions_m": dimensions, "source_obstacle": source["name"],
            "components": [p["name"] for p in pieces]}, pieces


def point_inside_shape(p: tuple[float, float], o: dict) -> bool:
    if geom.is_box(o):
        return geom.point_box_signed_distance(p, o) <= 0
    return math.hypot(p[0] - float(o["x"]), p[1] - float(o["y"])) <= float(o["radius"])


def union_area_1cm(obstacles: list[dict]) -> float:
    # 1 cm centre-sampled occupancy estimate, restricted to each shape's AABB.
    dx = dy = .01
    xmin, xmax = geom.OBSTACLE_BOUNDS[0]
    ymin, ymax = geom.OBSTACLE_BOUNDS[1]
    occupied: set[tuple[int,int]] = set()
    nx, ny = round((xmax-xmin)/dx), round((ymax-ymin)/dy)
    for o in obstacles:
        if geom.is_box(o):
            xs=[p[0] for p in geom.box_vertices(o)]; ys=[p[1] for p in geom.box_vertices(o)]
            xlo,xhi,ylo,yhi=min(xs),max(xs),min(ys),max(ys)
        else:
            xlo,xhi=float(o["x"])-o["radius"],float(o["x"])+o["radius"]
            ylo,yhi=float(o["y"])-o["radius"],float(o["y"])+o["radius"]
        ix0=max(0,math.floor((xlo-xmin)/dx)); ix1=min(nx,math.ceil((xhi-xmin)/dx))
        iy0=max(0,math.floor((ylo-ymin)/dy)); iy1=min(ny,math.ceil((yhi-ymin)/dy))
        for ix in range(ix0,ix1):
            x=xmin+(ix+.5)*dx
            for iy in range(iy0,iy1):
                y=ymin+(iy+.5)*dy
                if point_inside_shape((x,y),o): occupied.add((ix,iy))
    return len(occupied)*dx*dy


def make_xml(scene: dict) -> str:
    xml = ET.fromstring(geom.render_xml(scene))
    by_name = {o["name"]: o for o in scene["obstacles"]}
    for node in xml.find("worldbody").findall("geom"):
        name = node.get("name")
        if name not in by_name:
            continue
        o = by_name[name]
        node.set("pos", f'{o["x"]:.9f} {o["y"]:.9f} {o["z_m"]:.9f}')
        rgba = COLORS.get(o.get("object_kind"), (0.32, 0.36, 0.40, 1))
        node.set("rgba", " ".join(f"{v:g}" for v in rgba))
        node.set("contype", "1")
        node.set("conaffinity", "1")
    ET.indent(xml)
    return ET.tostring(xml, encoding="unicode") + "\n"


def render_top(scene: dict) -> Image.Image:
    image = Image.new("RGB", (1200, 900), "white")
    d = ImageDraw.Draw(image)
    xb, yb = geom.OBSTACLE_BOUNDS
    plot = (95, 110, 1105, 790)
    sx, sy = (plot[2] - plot[0]) / (xb[1] - xb[0]), (plot[3] - plot[1]) / (yb[1] - yb[0])
    def pix(p):
        return (round(plot[0] + (p[0] - xb[0]) * sx), round(plot[3] - (p[1] - yb[0]) * sy))
    d.text((95, 18), f'{scene["scene_id"]} · {scene["density"]} · 来自 {scene["source_scene_id"]}',
           fill="#17212b", font=maps.font(19))
    d.text((95, 54), f'物体 {scene["object_count"]} 件 · 碰撞构件 {scene["obstacle_count"]} 个 · 包络余量后净宽 {scene["path_check"]["residual_width_after_envelope_margin_m"]:.2f} m',
           fill="#475569", font=maps.font(15))
    d.rectangle(plot, outline="#46515b", width=2)
    route = [tuple(p) for p in scene["reference_path_m"]]
    rp = [pix(p) for p in route]
    corridor_px = round(2 * geom.REQUIRED_CENTER_CLEARANCE_M * (sx + sy) / 2)
    d.line(rp, fill="#d7ede8", width=corridor_px, joint="curve")
    d.line(rp, fill="#087e83", width=4, joint="curve")
    p0, p1 = pix(geom.START), pix(geom.GOAL)
    d.line((p0, p1), fill="#e28a23", width=2)
    object_map = {o["name"]: o for o in scene["obstacles"]}
    for obj in scene["objects"]:
        comps = [object_map[name] for name in obj["components"]]
        color = COLORS[obj["kind"]][:3]
        rgb = tuple(round(255 * c) for c in color)
        for o in comps:
            if geom.is_box(o):
                d.polygon([pix(p) for p in geom.box_vertices(o)], fill=rgb, outline="#202b33")
            else:
                cx, cy = pix((o["x"], o["y"]))
                rx, ry = round(o["radius"] * sx), round(o["radius"] * sy)
                d.ellipse((cx-rx, cy-ry, cx+rx, cy+ry), fill=rgb, outline="#202b33", width=2)
        tx, ty = pix(tuple(obj["center_m"]))
        d.text((tx+7, ty-20), obj["label_zh"], fill="#111827", font=maps.font(13))
    for point, label, color in ((geom.START, "START", "#21864a"), (geom.GOAL, "GOAL", "#2357a5")):
        x, y = pix(point); d.ellipse((x-8,y-8,x+8,y+8), fill=color, outline="white", width=2)
        d.text((x+10,y-18), label, fill=color, font=maps.font(13))
    d.text((95, 818), "浅青带：Go2 0.65 m 包络 + 0.10 m 余量；参考路线只作几何核验，不是策略输入。", fill="#334155", font=maps.font(14))
    d.text((95, 850), "二维射线将各碰撞构件投影到XY；不是深度相机或实机感知验证。坐标单位：米。", fill="#334155", font=maps.font(14))
    return image


def project3d(x: float, y: float, z: float) -> tuple[int, int]:
    return round(220 + 105*x + 44*y), round(620 + 23*x - 50*y - 330*z)


def render_oblique(scene: dict) -> Image.Image:
    image = Image.new("RGB", (1200, 850), "#f5f7fa")
    d = ImageDraw.Draw(image)
    ground = [project3d(x,y,0) for x,y in ((-.5,-3),(7.5,-3),(7.5,3),(-.5,3))]
    d.polygon(ground, fill="#e5eaee", outline="#9aa7b1")
    lookup = {o["name"]: o for o in scene["obstacles"]}
    draw_items = []
    for obj in scene["objects"]:
        for piece_name in obj["components"]:
            o = lookup[piece_name]
            draw_items.append((float(o["z_m"]), obj, o))
    for _, obj, o in sorted(draw_items, key=lambda row: row[0]):
        col = tuple(round(255*c) for c in COLORS[obj["kind"]][:3])
        if geom.is_box(o):
            bottom = [project3d(x,y,o["z_m"]-o["height_m"]/2) for x,y in geom.box_vertices(o)]
            top = [project3d(x,y,o["z_m"]+o["height_m"]/2) for x,y in geom.box_vertices(o)]
            for i in range(4):
                j=(i+1)%4
                d.polygon([bottom[i],bottom[j],top[j],top[i]], fill=col, outline="#26313a")
            d.polygon(top, fill=tuple(min(255,v+24) for v in col), outline="#26313a")
        else:
            x,y=float(o["x"]),float(o["y"]); r=float(o["radius"]); h=float(o["height_m"])
            low=[project3d(x+r*math.cos(t),y+r*math.sin(t),0) for t in [i*math.pi/12 for i in range(24)]]
            high=[project3d(x+r*math.cos(t),y+r*math.sin(t),h) for t in [i*math.pi/12 for i in range(24)]]
            # Faceted cylinder side and elliptical top; a schematic of the proxy.
            for i in range(12):
                j=(i+1)%12
                d.polygon([low[2*i],low[2*j],high[2*j],high[2*i]], fill=col, outline="#26313a")
            d.polygon(high, fill=tuple(min(255,v+24) for v in col), outline="#26313a")
    d.text((45,24), f'{scene["scene_id"]} · 三维静态几何示意', fill="#17212b", font=maps.font(24))
    d.text((45,65), "基本几何碰撞代理；固定在世界坐标；非 MuJoCo 截图。桌面/座面下方未填充。", fill="#334155", font=maps.font(17))
    y=105
    for obj in scene["objects"]:
        d.text((45,y), f'{obj["label_zh"]} ({obj["object_id"]})：{len(obj["components"])} 个碰撞构件', fill="#17212b", font=maps.font(16)); y+=25
    return image


def validate(scene: dict) -> None:
    obs = scene["obstacles"]
    names = [o["name"] for o in obs]
    if len(names) != len(set(names)) or any(not n.startswith("ppt_obstacle_") for n in names):
        raise ValueError(f'{scene["scene_id"]}: duplicate/unselectable geometry name')
    if len(scene["scene_id"].encode("ascii")) >= 32:
        raise ValueError("scene_id must be unique ASCII shorter than 32 bytes")
    objects = {o["object_id"] for o in obs}
    if len(objects) != scene["object_count"]:
        raise ValueError("object count mismatch")
    for o in obs:
        if not geom.in_bounds(o):
            raise ValueError(f'out_of_bounds:{o["name"]}')
        if min(geom.point_shape_distance(point, o) for point in (geom.START, geom.GOAL)) < geom.START_GOAL_CLEARANCE_M:
            raise ValueError(f'endpoint_clearance:{o["name"]}')
    owners = {o["name"]: o["object_id"] for o in obs}
    for i, a in enumerate(obs):
        for b in obs[i+1:]:
            if owners[a["name"]] == owners[b["name"]]:
                continue  # intentional contact/overlap within one furniture object
            if geom.shape_distance(a,b) < -1e-8:
                raise ValueError(f'unexpected_overlap:{a["name"]}:{b["name"]}')
    route=scene["reference_path_m"]
    if math.dist(route[0],geom.START)>1e-6 or math.dist(route[-1],geom.GOAL)>1e-6:
        raise ValueError("route_endpoints")
    min_clear=min(geom.segment_shape_distance(a,b,o) for a,b in zip(route,route[1:]) for o in obs)
    if min_clear + 1e-8 < geom.REQUIRED_CENTER_CLEARANCE_M:
        raise ValueError(f'{scene["scene_id"]}:reference_clearance:{min_clear:.4f}m')
    scene["path_check"].update({
        "minimum_geometric_obstacle_clearance_m": min_clear,
        "minimum_centerline_corridor_width_m": 2*min_clear,
        "required_obstacle_clearance_m": geom.REQUIRED_CENTER_CLEARANCE_M,
        "go2_planar_envelope_radius_m": geom.GO2_ENVELOPE_RADIUS_M,
        "engineering_clearance_margin_m": geom.ENGINEERING_MARGIN_M,
        "residual_width_after_envelope_margin_m": 2*(min_clear-geom.REQUIRED_CENTER_CLEARANCE_M),
        "route_checked_against_actual_component_projections": True,
    })
    blockers=geom.line_blockers(obs)
    scene["direct_line_blocking_obstacles"]=blockers
    if scene["route_structure"] == "short_local_bypass" and not blockers:
        raise ValueError("expected direct line blockers in local bypass variant")
    if scene["route_structure"] == "forward_offset_short_obstacles" and not blockers:
        # Base offset maps are not required to block the centerline; preserve that source property.
        scene["direct_line_blocking_obstacles"]=[]


def generate() -> list[dict]:
    doc=json.loads(LAYOUTS.read_text(encoding="utf-8"))
    byid={s["scene_id"]:s for s in doc["scenes"]}
    OUT.mkdir(parents=True,exist_ok=True)
    SCENE_DIR.mkdir(parents=True,exist_ok=True)
    variants=[]
    for source_id, substitutions in SUBSTITUTIONS.items():
        source=byid[source_id]
        filename, scene_id=IDS[source_id]
        removed={name for name,_,_ in substitutions}
        source_obs={o["name"]:o for o in source["obstacles"]}
        if not removed.issubset(source_obs):
            raise ValueError(f"missing source obstacle in {source_id}")
        obstacles=[copy.deepcopy(o) for o in source["obstacles"] if o["name"] not in removed]
        objects=[]
        for source_name,kind,instance in substitutions:
            obj,pieces=build_object(kind,instance,source_obs[source_name])
            objects.append(obj)
            for p in pieces:
                p["object_kind"]=kind
            obstacles.extend(pieces)
        for o in obstacles:
            o.setdefault("object_id", "obstacle_" + o["name"].replace("ppt_obstacle_", ""))
            o.setdefault("object_kind", "simple_obstacle")
            o.setdefault("z_m", o["height_m"]/2)
        total_object_count = len({o["object_id"] for o in obstacles})
        source_path=source["routes"][0]["reference_path_m"]
        scene={
            "filename":filename,"scene_id":scene_id,"scene_slug":filename.removeprefix("scene_").removesuffix(".xml"),
            "model_name":scene_id,"source_scene_id":source_id,"source_filename":source["filename"],
            "generator_version":GENERATOR_VERSION,"generation_seed":source["generation_seed"],
            "variant_random_seed":None,"variant_method":"deterministic in-place object substitution; no random relocation",
            "split":"development_variant_only_not_in_60_map_library","density":source["density"],
            "route_structure":source["route_structure"],"route_structure_label_zh":source["route_structure_label_zh"],
            "start_m":source["start_m"],"goal_m":source["goal_m"],
            "obstacle_bounds_m":source["obstacle_bounds_m"],"path_search_bounds_m":source["path_search_bounds_m"],
            "object_count":total_object_count,"common_object_count":len(objects),
            "simple_obstacle_count":total_object_count-len(objects),
            "objects":objects,"obstacle_count":len(obstacles),"obstacles":obstacles,
            "reference_path_m":source_path,
            "reference_path_adjustment_y_m":0.0,
            "ray_projection_semantics":"Production 2-D XY projection of every selected collision geom. Tabletops/seats conservatively block their full XY footprint; no under-furniture route is represented. This is not depth-camera or real-robot perception evidence.",
            "collision_semantics":"Static world collision primitives remain individually named by parent object and part; table/stool undersides are not filled by hidden collision volumes.",
            "replacement_mapping":[{"source_obstacle":sname,"object_id":obj["object_id"],"kind":kind,
                                    "center_m":[source_obs[sname]["x"],source_obs[sname]["y"]],
                                    "source_shape":source_obs[sname]["shape"],"dimensions_m":obj["dimensions_m"],
                                    "components":obj["components"]}
                                   for (sname,kind,_),obj in zip(substitutions,objects)],
            "path_check":{"grid_resolution_m":.05,"continuous_path_check_step_m":.025,
                          "start_goal_clearance_required_m":geom.START_GOAL_CLEARANCE_M},
            "density_definition":{"source_density_label":source["density"],
                                   "approx_xy_union_area_m2_1cm_grid":None,
                                   "approx_xy_union_area_fraction_1cm_grid":None,
                                   "measurement":"1 cm centre-sampled union of XY collision projections"},
        }
        # Keep the source path if it clears the larger furniture projections.
        # Otherwise make the smallest bounded uniform lateral shift of interior
        # waypoints; endpoints stay fixed and the conservative envelope is unchanged.
        candidate_paths=[(0.0,source_path)]
        for step in range(1,13):
            delta=step*.025
            for signed in (delta,-delta):
                candidate=[]
                for i,p in enumerate(source_path):
                    candidate.append([p[0],p[1] if i in (0,len(source_path)-1) else round(p[1]+signed,6)])
                candidate_paths.append((signed,candidate))
        chosen=None
        for delta,candidate in candidate_paths:
            if any(not (geom.PATH_BOUNDS[1][0] <= p[1] <= geom.PATH_BOUNDS[1][1]) for p in candidate):
                continue
            clear=min(geom.segment_shape_distance(tuple(a),tuple(b),o)
                      for a,b in zip(candidate,candidate[1:]) for o in obstacles)
            if clear+1e-8>=geom.REQUIRED_CENTER_CLEARANCE_M:
                chosen=(delta,candidate)
                break
        if chosen is None:
            raise ValueError(f'{scene_id}: bounded reference-route adjustment failed')
        scene["reference_path_adjustment_y_m"]=chosen[0]
        scene["reference_path_m"]=chosen[1]
        validate(scene)
        area=union_area_1cm(obstacles)
        scene["density_definition"]["approx_xy_union_area_m2_1cm_grid"]=round(area,4)
        scene["density_definition"]["approx_xy_union_area_fraction_1cm_grid"]=round(area/48,6)
        xml=make_xml(scene)
        ET.fromstring(xml)
        (SCENE_DIR/filename).write_text(xml,encoding="utf-8")
        (OUT/(scene_id+"_layout.json")).write_text(json.dumps(scene,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
        render_top(scene).save(OUT/(scene_id+"_top.png"))
        render_oblique(scene).save(OUT/(scene_id+"_oblique_schematic.png"))
        variants.append(scene)
    (OUT/"layouts_common_objects.json").write_text(json.dumps({"schema_version":1,"generator_version":GENERATOR_VERSION,
        "map_library_membership":"independent development variants; excluded from the 60-map manifest and all held-out splits",
        "scene_count":len(variants),"scenes":variants},ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    # Four-up overview, each panel links visually to the per-scene images.
    overview=Image.new("RGB",(1600,1200),"#eef1f4")
    d=ImageDraw.Draw(overview)
    d.text((35,18),"S6-11 常见物体开发变体 · 示意总览",fill="#17212b",font=maps.font(26))
    for i,scene in enumerate(variants):
        x=20+(i%2)*790; y=72+(i//2)*555
        top=Image.open(OUT/(scene["scene_id"]+"_top.png")).convert("RGB").resize((760,430))
        overview.paste(top,(x,y))
        d.text((x,y+435),f'{scene["scene_id"]}: 家具/常见物体{scene["common_object_count"]}件 + 简单障碍{scene["simple_obstacle_count"]}件 / {scene["obstacle_count"]}个碰撞构件',fill="#17212b",font=maps.font(17))
        d.text((x,y+465),"斜视图见对应 _oblique_schematic.png（非MuJoCo截图）",fill="#475569",font=maps.font(15))
    overview.save(OUT/"common_object_variants_overview.png")
    return variants


if __name__ == "__main__":
    for row in generate():
        print(f'{row["scene_id"]} {row["filename"]} objects={row["object_count"]} geoms={row["obstacle_count"]} clearance={row["path_check"]["minimum_geometric_obstacle_clearance_m"]:.3f}m area={row["density_definition"]["approx_xy_union_area_fraction_1cm_grid"]:.3%}')
