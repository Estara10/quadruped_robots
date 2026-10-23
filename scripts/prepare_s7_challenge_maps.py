#!/usr/bin/env python3
"""Prepare M3: 15 retained development variants and 15 historical failure layouts."""
import argparse
import copy
import csv
import json
import math
import random
import re
from pathlib import Path

from PIL import Image, ImageDraw
import generate_m2_maps as g

ROOT = g.ROOT
OUT = ROOT / 'docs/thesis_project/designs/M3'
SOURCE = ROOT / 'docs/thesis_project/designs/M2/selected_v2/layouts.json'
VERSION = 's7-challenge-test/1.4'

# User-requested forward opening: keep obstacle sizes; move only these three layouts.
INITIAL_OPENING_EDITS = {
    'challenge02.xml': {1:(2.8,-1.4), 2:(2.8,1.4), 3:(5.1,0), 4:(5.7,1.75)},
    'challenge04.xml': {1:(2.8,-1.45), 2:(2.8,1.45), 3:(4.7,0), 4:(6,-1.3),
                        8:(4.1,2.6), 9:(6.7,2.5)},
    'challenge06.xml': {1:(2.9,1.45), 2:(2.9,-1.45), 3:(4.9,0), 4:(6,1.25),
                        7:(4.1,2.4), 9:(5.4,2.5), 8:(6.8,2.4)},
}


def initial_rays(obstacles):
    """Static geometric illustration of the existing 11 rays at nominal start pose."""
    origin = (-0.05, 0.0)
    samples = []
    for angle in range(-45, 46, 9):
        a = math.radians(angle)
        def endpoint(distance):
            return (origin[0]+distance*math.cos(a), distance*math.sin(a))
        if all(g.m1.geom.segment_shape_distance(origin, endpoint(6.0), o) > 0
               for o in obstacles):
            distance = 6.0
        else:
            low, high = 0.0, 6.0
            for _ in range(22):
                mid = (low+high)/2
                if any(g.m1.geom.segment_shape_distance(origin, endpoint(mid), o) <= 0
                       for o in obstacles):
                    high = mid
                else:
                    low = mid
            distance = high
        samples.append(dict(angle_deg=angle, distance_m=round(distance,6)))
    return samples


def apply_initial_opening(s):
    edits = INITIAL_OPENING_EDITS.get(s['filename'])
    if not edits:
        return
    obs = s['obstacles']
    moves = []
    for index, xy in edits.items():
        o = obs[index-1]
        moves.append(dict(obstacle=o['name'], before_xy_m=[o['x'],o['y']], after_xy_m=list(xy)))
        o['x'], o['y'] = xy
    if s['filename'] == 'challenge04.xml':
        obs.append(dict(name='ppt_obstacle_11', shape='cylinder', x=5.8, y=0.3,
                        radius=0.20, yaw_rad=0.0, height_m=0.50,
                        role='manual_added_local_obstacle'))
    s['obstacle_count'] = len(obs)
    if g.m1.geom.path_clearance([(0,0),(2.5,0)],obs) < 0.75:
        raise ValueError('Initial straight corridor is too narrow')
    old_bounds = g.m1.geom.PATH_BOUNDS
    try:
        g.m1.geom.PATH_BOUNDS = ((2.5,7.5),(-2.9,2.9))
        tail = g.m1.s611.grid_forward_path(obs)
    finally:
        g.m1.geom.PATH_BOUNDS = old_bounds
    if tail is None:
        raise ValueError('No forward route after the initial opening')
    rays = initial_rays(obs)
    if min(r['distance_m'] for r in rays if abs(r['angle_deg']) <= 9) < 3.0:
        raise ValueError('Initial central ray opening is obstructed')
    s['routes'] = [dict(route_id='geometry_only', reference_path_m=[(0,0)]+tail)]
    s['geometry_copied_without_perturbation'] = False
    s['layout_revision'] = 'initial-forward-opening-20261023'
    s['route_structure_label_zh'] = '前向宽开口 + 后段局部绕障'
    if s['filename'] == 'challenge04.xml':
        s['layout_revision'] = 'left-opening-center-cylinder-20261023'
        s['route_structure_label_zh'] = '前向宽开口 + 左侧绕障 + 中段圆柱'
        s['manual_added_obstacle'] = dict(obs[-1])
    s['initial_opening'] = dict(
        initial_straight_length_m=2.5, required_center_clearance_m=0.75,
        nominal_start_origin_xy_m=[-0.05,0.0], initial_rays=rays, moves=moves,
        note='Static nominal-pose geometry only; not live sensor or policy evidence. Source historical failure rates do not transfer to edited layouts.')


def render_scene(s):
    im = g.render(s)
    if 'initial_opening' not in s:
        return im
    d = ImageDraw.Draw(im)
    def pixel(x,y):
        return (95+(x+0.5)*105, 100+(3-y)*105)
    origin = pixel(-0.05,0)
    for ray in s['initial_opening']['initial_rays']:
        a = math.radians(ray['angle_deg'])
        length = min(ray['distance_m'],3.8)
        d.line((origin,pixel(-0.05+length*math.cos(a),length*math.sin(a))),
               fill='#c58532',width=1)
    d.line((pixel(0,0),pixel(2.5,0)),fill='#2563eb',width=5)
    d.text((40,710),'蓝段：前2.5m通行段；橙线：名义起点11束静态射线示意',
           fill='#1e40af',font=g.m1.font(17))
    return im


def variant(template, number, replica):
    seed = 2026102300 + number
    rng = random.Random(seed)
    route = template['routes'][0]['reference_path_m']
    for attempt in range(1, 2001):
        obs = copy.deepcopy(template['obstacles'])
        for o in obs:
            o['x'] = round(o['x'] + rng.uniform(-0.22, 0.22), 6)
            o['y'] = round(o['y'] + rng.uniform(-0.18, 0.18), 6)
            if g.m1.geom.is_box(o):
                o['half_x'] = round(o['half_x'] * rng.uniform(0.92, 1.08), 6)
                o['half_y'] = round(o['half_y'] * rng.uniform(0.92, 1.08), 6)
                o['yaw_rad'] = round(o['yaw_rad'] + rng.uniform(-0.12, 0.12), 6)
            else:
                o['radius'] = round(o['radius'] * rng.uniform(0.92, 1.08), 6)
        if not all(g.m1.within_limits(o) for o in obs):
            continue
        if not g.m1.geom.no_overlap_and_spacing(obs, 0.50):
            continue
        blockers = g.m1.geom.line_blockers(obs)
        if not blockers or g.m1.geom.path_clearance(route, obs) < 0.75:
            continue
        metrics = g.m1.geom.path_metrics(route, obs)
        if not metrics['connected']:
            continue
        name = f'challenge{number:02d}'
        return dict(
            filename=name+'.xml', scene_id=f's7ch{number:02d}', model_name=name,
            display_name=name+' · source '+template['filename'], preview=name+'.png',
            mapset_version=VERSION, split='challenge_test',
            source_filename=template['filename'], source_difficulty=template['difficulty_label'],
            template_replica=replica, template_family=template['filename'],
            generation_seed=seed, generation_attempt=attempt,
            route_structure_label_zh=template['route_structure_label_zh'],
            start_m=list(g.m1.START), goal_m=list(g.m1.GOAL),
            field_bounds_m={'x': [-0.5, 8.0], 'y': [-3.0, 3.0]},
            field_is_physical_wall=False, obstacle_count=len(obs), obstacles=obs,
            routes=[dict(route_id='geometry_only', reference_path_m=route)],
            path_check=metrics, direct_line_blocked=True,
            direct_line_blocking_obstacles=blockers,
            difficulty_verified_by_policy=False,
            density_definition=dict(
                obstacle_footprint_area_fraction=g.m1.area(obs)/g.m1.FIELD_AREA,
                minimum_obstacle_spacing_m=min(g.m1.geom.shape_distance(a,b)
                    for i,a in enumerate(obs) for b in obs[i+1:])))
    raise RuntimeError(f'No feasible variant for {template["filename"]} replica {replica}')


def historical_scene(row):
    library = json.loads((ROOT / 'docs/thesis_project/designs/M1/library_v1/layouts.json').read_text())
    source = next(s for s in library['scenes'] if s['filename'] == row['source_filename'])
    s = copy.deepcopy(source)
    s.pop('production_scene_binding', None)
    name = Path(row['replacement_filename']).stem
    number = int(name.removeprefix('challenge'))
    s.update(filename=name+'.xml', scene_id=f's7ch{number:02d}', model_name=name,
             display_name=name+' · source '+source['filename'], preview=name+'.png',
             mapset_version=VERSION, split='historically_selected_challenge',
             source_filename=source['filename'], source_difficulty='historical_nonarrival',
             template_family=source['filename'], template_replica=1,
             selection_basis=dict(row), geometry_copied_without_perturbation=True,
             difficulty_verified_by_policy=False)
    # Keep the actual source layout: failures describe this geometry, not a new perturbation.
    apply_initial_opening(s)
    path = s['routes'][0]['reference_path_m']
    obs = s['obstacles']
    if not all(g.m1.within_limits(o) for o in obs):
        raise ValueError(f'{source["filename"]}: invalid obstacle dimensions or bounds')
    if not g.m1.geom.no_overlap_and_spacing(obs, 0.50):
        raise ValueError(f'{source["filename"]}: insufficient obstacle separation')
    if g.m1.geom.path_clearance(path, obs) < 0.75:
        raise ValueError(f'{source["filename"]}: insufficient route clearance')
    s['path_check'] = g.m1.geom.path_metrics(path, obs)
    s['direct_line_blocking_obstacles'] = g.m1.geom.line_blockers(obs)
    s['direct_line_blocked'] = bool(s['direct_line_blocking_obstacles'])
    s['density_definition'] = dict(
        obstacle_footprint_area_fraction=g.m1.area(obs)/g.m1.FIELD_AREA,
        minimum_obstacle_spacing_m=min(g.m1.geom.shape_distance(a,b)
            for i,a in enumerate(obs) for b in obs[i+1:]))
    if not s['path_check']['connected']:
        raise ValueError(f'{source["filename"]}: disconnected reference route')
    return s


def generate():
    g.m1.configure_geometry()
    templates = [s for s in json.loads(SOURCE.read_text())['scenes']
                 if s['difficulty_label'] in ('middle', 'difficult')]
    if len(templates) != 15:
        raise ValueError('Expected 6 middle and 9 difficult templates')
    scenes = [variant(t, 2*i+1, 1) for i,t in enumerate(templates)]
    with (OUT / 'historical_selection.csv').open() as f:
        selections = list(csv.DictReader(f))
    if len(selections) != 15:
        raise ValueError('Expected 15 historical replacements')
    scenes += [historical_scene(row) for row in selections]
    scenes.sort(key=lambda s: s['filename'])
    # Reject geometry clones, including M1/M2 aliases with different filenames.
    signatures = [json.dumps([{k:v for k,v in o.items() if k in
                    ('shape','x','y','radius','half_x','half_y','yaw_rad','height_m')}
                    for o in s['obstacles']],sort_keys=True) for s in scenes]
    if len(set(signatures)) != 30:
        raise ValueError('Duplicate geometry in M3')
    OUT.mkdir(parents=True, exist_ok=True)
    overview = Image.new('RGB', (1830, 1680), '#edf1f4')
    draw = ImageDraw.Draw(overview)
    draw.text((20, 12), 'M3 v1.4 · 04增加中段圆柱 · 参考线不输入策略',
              fill='#17212b', font=g.m1.font(24))
    for i,s in enumerate(scenes):
        (g.m1.SCENE_DIR/s['filename']).write_text(g.m1.geom.render_xml(s))
        im = render_scene(s)
        im.save(OUT/s['preview'])
        overview.paste(im.resize((360,264)), (10+(i%5)*364, 60+(i//5)*270))
        print(f'{s["filename"]} <- {s["source_filename"]}, clearance={s["path_check"]["minimum_geometric_obstacle_clearance_m"]:.3f}', flush=True)
    overview.save(OUT/'overview_30.png')
    doc = dict(mapset_version=VERSION, scenes=scenes, scene_count=30,
               registration_status='pending production MuJoCo registration',
               policy_runs=0, original_holdouts_modified=False,
               purpose='Post-result challenge development: 15 retained variants plus 15 layouts selected by pooled historical nonarrival, not group wins.',
               inference_limit='Historically selected, partly reused maps; not an independent holdout. Geometry uniqueness is not independent route-family diversity.')
    (OUT/'layouts.json').write_text(json.dumps(doc,ensure_ascii=False,indent=2)+'\n')
    keys = ['filename','scene_id','source_filename','source_difficulty','template_replica','generation_seed','obstacle_count']
    with (OUT/'scene_index.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=keys); w.writeheader()
        w.writerows({k:s[k] for k in keys} for s in scenes)
    # Catalog entries are required by the existing physics reader; no new identity mechanism.
    g.m1.s611.append_or_replace_catalog(g.m1.s611.catalog_rows(scenes), upsert=True)


def register(path):
    doc=json.loads((OUT/'layouts.json').read_text())
    found={}
    for line in path.read_text().splitlines():
        m=re.fullmatch(r'SCENE filename=(\S+) id=(\S+) obstacle_count=(\d+) fingerprint=([0-9a-f]{64}) mujoco_version=(\S+)',line)
        if m: found[m[1]]=(m[2],int(m[3]),m[4],m[5])
    if set(found) != {s['filename'] for s in doc['scenes']}:
        raise ValueError('Runtime loading must cover these 30 scenes')
    for s in doc['scenes']:
        sid,count,fp,version=found[s['filename']]
        if (sid,count)!=(s['scene_id'],s['obstacle_count']):
            raise ValueError(s['filename'])
        row=g.m1.s611.catalog_rows([s],{s['filename']:fp})[0]
        g.m1.s611.append_or_replace_catalog([row],upsert=True)
        s['production_scene_binding']={**row,'runtime_mujoco_version':version}
    doc['registration_status']='registered using production MuJoCo; no policy runs'
    (OUT/'layouts.json').write_text(json.dumps(doc,ensure_ascii=False,indent=2)+'\n')
    (OUT/'runtime_identities.txt').write_text(path.read_text())


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--register-from',type=Path)
    args=p.parse_args()
    register(args.register_from) if args.register_from else generate()
