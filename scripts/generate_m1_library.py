#!/usr/bin/env python3
"""Generate the reproducible M1 60-map candidate library.

Reference routes are geometric feasibility aids only and are never supplied to
the policy. The development/held-out seeds use separate namespaces.
"""
from __future__ import annotations

import copy
import json
import math
import random
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import generate_s6_08_scenes as geom
import generate_s6_11_maps as s611

OUT = ROOT / "docs/thesis_project/designs/M1/library_v1"
SCENE_DIR = ROOT / "unitree_mujoco/unitree_robots/go2"
EVIDENCE = ROOT / "docs/thesis_project/evidence/M1"
GENERATOR_VERSION = "m1-map-library/1.0"
START, GOAL = (0.0, 0.0), (7.5, 0.0)
BOUNDS = ((-0.5, 8.0), (-3.0, 3.0))
FIELD_AREA = (BOUNDS[0][1]-BOUNDS[0][0])*(BOUNDS[1][1]-BOUNDS[1][0])
DENSITIES = {"sparse": (0.008, 0.019), "medium": (0.019, 0.032), "dense": (0.032, 0.050)}
STRUCTURES = ("single_side_bypass", "gentle_s_bypass", "staggered_channel",
              "short_local_bypass", "forward_offset_obstacles")
LABELS = {"single_side_bypass":"短障碍单侧绕行", "gentle_s_bypass":"缓和S型连续绕行",
          "staggered_channel":"错位短障碍通道", "short_local_bypass":"短障碍局部绕行",
          "forward_offset_obstacles":"前向错列短障碍"}
CODES = {"single_side_bypass":"b", "gentle_s_bypass":"s", "staggered_channel":"o",
         "short_local_bypass":"l", "forward_offset_obstacles":"f"}
DC = {"sparse":"s", "medium":"m", "dense":"d"}
SC = {"development":"d", "held_out":"h"}
SEED_BASE = {"development": 261015000, "held_out": 271015000}
MIN_CLEARANCE = 0.75
MIN_GAP = 0.50
MAX_X, MAX_Y = 0.85, 0.75
MAX_ATTEMPTS = 500

def public_name(density, structure, split, replica):
    """Stable user-facing numbering; internal identities remain historical M1 IDs."""
    number = list(DENSITIES).index(density) * 10 + STRUCTURES.index(structure) * 2 + replica
    return f"{'dev' if split == 'development' else 'holdout'}{number:02d}"

def configure_geometry():
    geom.START, geom.GOAL = START, GOAL
    geom.OBSTACLE_BOUNDS = BOUNDS
    geom.PATH_BOUNDS = ((0.0, 7.5), (-2.9, 2.9))
    geom.START_GOAL_CLEARANCE_M = 0.95
    geom.REQUIRED_CENTER_CLEARANCE_M = MIN_CLEARANCE
    geom.GRID_M = 0.05
    geom.PATH_CHECK_STEP_M = 0.025
    s611.geom.START, s611.geom.GOAL = START, GOAL
    s611.geom.OBSTACLE_BOUNDS = BOUNDS
    s611.geom.PATH_BOUNDS = ((0.0, 7.5), (-2.9, 2.9))
    s611.geom.START_GOAL_CLEARANCE_M = 0.95
    s611.AREA_M2 = FIELD_AREA

def obstacle(rng: random.Random, idx: int, x: float, y: float, role: str) -> dict:
    shape = rng.choices(("box", "cylinder", "rotated_bar"), (0.43, 0.28, 0.29))[0]
    height = round(rng.uniform(0.32, 0.62), 6)
    name = f"ppt_obstacle_{idx:02d}"
    if shape == "cylinder":
        return {"name":name,"shape":shape,"x":round(x,6),"y":round(y,6),
                "radius":round(rng.uniform(0.20,0.29),6),"yaw_rad":0.0,
                "height_m":height,"role":role}
    return {"name":name,"shape":shape,"x":round(x,6),"y":round(y,6),
            "half_x":round(rng.uniform(0.20,0.39),6),"half_y":round(rng.uniform(0.14,0.33),6),
            "yaw_rad":round(rng.uniform(-0.45,0.45),6),"height_m":height,"role":role}

def area(obs): return sum(geom.obstacle_area(o) for o in obs)

def s_path(rng):
    amp = rng.uniform(0.86, 1.18)
    phase = rng.uniform(-0.12, 0.12)
    n=601
    return [(7.5*i/(n-1), amp*math.sin(2*math.pi*(i/(n-1)+phase)) - amp*math.sin(2*math.pi*phase))
            for i in range(n)]

def core_layout(rng, structure, density):
    ncore = {"sparse":2,"medium":3,"dense":4}[density]
    xs = [rng.uniform(1.65,2.35), rng.uniform(4.65,5.55), rng.uniform(2.8,3.8),
          rng.uniform(5.9,6.65)][:ncore]
    if structure == "single_side_bypass":
        ncore = {"sparse":1,"medium":2,"dense":3}[density]
        xs = sorted(rng.uniform(2.0,5.9) for _ in range(ncore))
        ys = [0.0]*ncore
    elif structure == "gentle_s_bypass":
        xs = [rng.uniform(1.75,2.25), rng.uniform(5.25,5.75)]
        # Centerline objects are locally passed on alternating sides by the
        # displayed feasible S reference; policy remains free to bypass.
        ys = [0.0,0.0]
    elif structure == "staggered_channel":
        xs = [rng.uniform(1.55,1.95),rng.uniform(3.55,3.95),rng.uniform(5.65,6.05)]
        ys = [rng.choice((-0.48,-0.34)),rng.choice((0.34,0.48)),rng.choice((-0.48,-0.34))]
    elif structure == "short_local_bypass":
        xs = [rng.uniform(2.0,2.8),rng.uniform(4.8,5.8)][:ncore]
        ys = [rng.uniform(-0.20,0.20) for _ in xs]
    elif structure == "forward_offset_obstacles":
        xs = sorted(rng.uniform(1.45,6.35) for _ in range({"sparse":2,"medium":3,"dense":4}[density]))
        ys = [rng.choice((-0.55,-0.42,0.42,0.55)) for _ in xs]
    out=[]
    for i,(x,y) in enumerate(zip(xs,ys),1):
        # Vary all central obstacles (position, dimensions, shape, orientation)
        # instead of manufacturing density only with edge fill.
        o=obstacle(rng,i,x+rng.uniform(-0.16,0.16),y+rng.uniform(-0.12,0.12),"primary")
        out.append(o)
    return out

def make_path(rng, structure, obstacles):
    if structure == "gentle_s_bypass":
        path=s_path(rng)
        return path if geom.path_clearance(path,obstacles) >= MIN_CLEARANCE else None
    path=s611.grid_forward_path(obstacles)
    if not path or max(abs(p[1]) for p in path)>1.55:
        return None
    headings=[abs(math.degrees(math.atan2(b[1]-a[1],b[0]-a[0]))) for a,b in zip(path,path[1:])]
    if max(headings,default=0)>50.0:
        return None
    return path

def within_limits(o):
    return (2*geom.support_x(o) <= MAX_X+1e-9 and
            2*geom.support_y(o) <= MAX_Y+1e-9 and geom.in_bounds(o) and
            geom.clear_start_goal(o))

def one_scene(density, structure, split, replica):
    cell = DENSITIES[density]
    seed=SEED_BASE[split]+list(DENSITIES).index(density)*100000+STRUCTURES.index(structure)*1000+replica
    low,high=cell
    for attempt in range(1,MAX_ATTEMPTS+1):
        rng=random.Random(seed+attempt*10000019)
        core=core_layout(rng,structure,density)
        if not all(within_limits(o) for o in core) or not geom.no_overlap_and_spacing(core,MIN_GAP):
            continue
        path=make_path(rng,structure,core)
        if path is None: continue
        obs=core[:]
        # Fill toward the density band with randomly located/oriented obstacles.
        # The full primary configuration changes by seed and density.
        for idx in range(100):
            fraction=area(obs)/FIELD_AREA
            if fraction >= low + (high-low)*0.55: break
            if len(obs)>=24: break
            x=rng.uniform(0.65,7.35); y=rng.choice((-1,1))*rng.uniform(1.25,2.8)
            cand=obstacle(rng,len(obs)+1,x,y,"density_obstacle")
            if not within_limits(cand): continue
            if not geom.no_overlap_and_spacing(obs+[cand],MIN_GAP): continue
            if geom.path_clearance(path,obs+[cand]) < MIN_CLEARANCE: continue
            obs.append(cand)
        fraction=area(obs)/FIELD_AREA
        if not low <= fraction < high: continue
        if len(obs)<2 or not geom.no_overlap_and_spacing(obs,MIN_GAP): continue
        if not all(within_limits(o) for o in obs): continue
        clearance=geom.path_clearance(path,obs)
        if clearance < MIN_CLEARANCE: continue
        blockers=geom.line_blockers(obs)
        metrics=geom.path_metrics(path,obs)
        sid=f"m1{DC[density]}{CODES[structure]}{SC[split]}{replica:02d}"
        slug=f"m1_{density}_{structure}_{'dev' if split=='development' else 'holdout'}_{replica:02d}"
        short_name=public_name(density,structure,split,replica)
        return {"scene_id":sid,"scene_slug":slug,"model_name":slug,
                "filename":f"{short_name}.xml","preview":f"{short_name}.png","display_name":short_name,
                "generator_version":GENERATOR_VERSION,"generation_seed":seed,
                "generation_attempt":attempt,"run_seed":None,"split":split,"density":density,
                "route_structure":structure,"route_structure_label_zh":LABELS[structure],
                "start_m":list(START),"goal_m":list(GOAL),"field_bounds_m":{"x":list(BOUNDS[0]),"y":list(BOUNDS[1])},
                "field_is_physical_wall":False,"obstacle_count":len(obs),"obstacles":obs,
                "reference_route_type":"feasible_reference_route_only",
                "routes":[{"route_id":"feasible_reference_route_only","reference_path_m":[[round(x,6),round(y,6)] for x,y in path],"path_check":metrics}],
                "direct_line_blocked":bool(blockers),"direct_line_blocking_obstacles":blockers,
                "density_definition":{"obstacle_footprint_area_fraction":fraction,"target_area_fraction_exclusive":[low,high],
                    "minimum_obstacle_spacing_m":min(geom.shape_distance(a,b) for i,a in enumerate(obs) for b in obs[i+1:]),
                    "reference_route_length_m":metrics["reference_path_length_m"],
                    "detour_ratio":metrics["reference_path_length_m"]/7.5,
                    "minimum_reference_centerline_clearance_m":clearance,
                    "minimum_centerline_corridor_width_m":metrics["minimum_centerline_corridor_width_m"]},
                "path_check":metrics,"path_check_settings":{"grid_resolution_m":0.05,"continuous_check_step_m":0.025,
                    "go2_planar_envelope_radius_m":0.65,"engineering_margin_m":0.10,
                    "required_centerline_clearance_m":MIN_CLEARANCE,"start_goal_clearance_m":0.95,
                    "route_label_note":"feasible reference route; policy may take a more direct local bypass; route is not policy input"}}
    raise RuntimeError(f"bounded resampling exhausted: {density}/{structure}/{split}/{replica}")

def font(size):
    try:return ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",size)
    except OSError:return ImageFont.load_default()

def render(scene):
    im=Image.new("RGB",(900,680),"white");d=ImageDraw.Draw(im)
    x0,y0,x1,y1=80,65,850,595
    sx=(x1-x0)/(BOUNDS[0][1]-BOUNDS[0][0]); sy=(y1-y0)/(BOUNDS[1][1]-BOUNDS[1][0])
    def p(q):return (round(x0+(q[0]-BOUNDS[0][0])*sx),round(y1-(q[1]-BOUNDS[1][0])*sy))
    d.text((45,14),f"{scene.get('display_name',scene['scene_id'])} · {scene['route_structure_label_zh']} · {scene['density']} · seed {scene['generation_seed']}",fill="#17212b",font=font(18))
    d.rectangle((x0,y0,x1,y1),outline="#475569",width=2)
    route=scene["routes"][0]["reference_path_m"]; rp=[p(q) for q in route]
    w=round(2*MIN_CLEARANCE*(sx+sy)/2)
    d.line(rp,fill="#d8eeeb",width=w,joint="curve");d.line(rp,fill="#087e83",width=4,joint="curve")
    d.line((*p(START),*p(GOAL)),fill="#e28a23",width=2)
    blockers=set(scene["direct_line_blocking_obstacles"])
    for o in scene["obstacles"]:
        col="#bb3c37" if o["name"] in blockers else "#59636d"
        if geom.is_box(o):d.polygon([p(v) for v in geom.box_vertices(o)],fill=col,outline="#222")
        else:
            cx,cy=p((o["x"],o["y"]));r=o["radius"]
            d.ellipse((cx-r*sx,cy-r*sy,cx+r*sx,cy+r*sy),fill=col,outline="#222")
    for q,label,col in ((START,"START","#21864a"),(GOAL,"GOAL","#2357a5")):
        x,y=p(q);d.ellipse((x-7,y-7,x+7,y+7),fill=col);d.text((x+8,y-16),label,fill=col,font=font(13))
    d.text((48,610),"浅青带：Go2 0.65m保守包络+0.10m工程余量；青线为可行参考路线，不是策略输入或实际轨迹。",fill="#334155",font=font(14))
    d.text((48,640),f"x=[-0.5,8.0]m，y=[-3,3]m；占地={scene['density_definition']['obstacle_footprint_area_fraction']:.2%}，障碍数={scene['obstacle_count']}。",fill="#334155",font=font(13))
    return im

def main():
    configure_geometry(); OUT.mkdir(parents=True,exist_ok=True); scenes=[]
    for density in DENSITIES:
        for structure in STRUCTURES:
            for split in ("development","held_out"):
                for replica in (1,2):
                    s=one_scene(density,structure,split,replica);scenes.append(s)
                    print(f"{len(scenes):02d}/60 {s['scene_id']} area={s['density_definition']['obstacle_footprint_area_fraction']:.3%} n={s['obstacle_count']} tries={s['generation_attempt']}",flush=True)
    assert len({s['scene_id'] for s in scenes})==60
    assert sum(s['split']=='development' for s in scenes)==30 and sum(s['split']=='held_out' for s in scenes)==30
    assert sum(s['direct_line_blocked'] for s in scenes)>=40
    for s in scenes:
        path=SCENE_DIR/s['filename'];path.write_text(geom.render_xml(s),encoding='utf-8')
        ET.parse(path); render(s).save(OUT/s['preview'],format='PNG',optimize=True)
    doc={"schema_version":1,"library_version":"m1-mapset/1.0","naming_revision":"dev-holdout/1.0","generator_version":GENERATOR_VERSION,
         "seed_namespaces":SEED_BASE,"seed_reproducible":True,"run_seed":"separate from generation seed",
         "scene_count":60,"development_count":30,"held_out_count":30,
         "stratification":{"density":list(DENSITIES),"route_structure":list(STRUCTURES),"per_cell":4,"development_per_cell":2,"held_out_per_cell":2},
         "density_bands_exclusive":{k:list(v) for k,v in DENSITIES.items()},"start_m":list(START),"goal_m":list(GOAL),
         "field_bounds_m":{"x":list(BOUNDS[0]),"y":list(BOUNDS[1])},"field_is_physical_wall":False,
         "go2_envelope_radius_m":0.65,"engineering_margin_m":0.10,"required_route_clearance_m":0.75,
         "development_heldout_split_fixed_before_policy_runs":True,"scenes":scenes}
    (OUT/"layouts.json").write_text(json.dumps(doc,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    for split in ("development","held_out"):
        selected=[s for s in scenes if s['split']==split]
        sheets=Image.new('RGB',(5*300,6*230),'#f2f4f7')
        for idx,s in enumerate(selected):
            thumb=render(s).resize((290,218));sheets.paste(thumb,((idx%5)*300,(idx//5)*230))
        sheets.save(OUT/f"overview_{split}_30.png",optimize=True)
    # Keep the fixed split/order manifest readable and stable.
    (OUT/"scene_index.csv").write_text("scene_id,filename,density,route_structure,split,replica,seed,goal_x,goal_y,sim_limit_s,wall_cap_s\n"+"".join(
        f"{s['scene_id']},{s['filename']},{s['density']},{s['route_structure']},{s['split']},{int(s['scene_id'][-2:])},{s['generation_seed']},7.5,0,20,120\n" for s in scenes),encoding='utf-8')
    # Register bulk-generated rows in the shared production catalog. Fingerprints
    # are placeholders until the actual C++ MuJoCo executable computes them.
    # Regeneration must not overwrite production C++ fingerprints with placeholders.
    import abs_scene
    fingerprints={s['filename']:abs_scene.SCENES[s['filename']].model_fingerprint
                  for s in scenes if s['filename'] in abs_scene.SCENES}
    rows=s611.catalog_rows(scenes,fingerprints)
    s611.append_or_replace_catalog(rows,upsert=True)
    print(f"WROTE {OUT} scenes=60 dev=30 held_out=30 line_blocked={sum(s['direct_line_blocked'] for s in scenes)}",flush=True)
if __name__=='__main__':main()
