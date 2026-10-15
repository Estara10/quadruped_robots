#!/usr/bin/env python3
"""Read-only deterministic geometry, stratification and production-binding checks for M1."""
from __future__ import annotations
import json, math, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"scripts"))
import generate_m1_library as gen
import abs_scene

def main():
    gen.configure_geometry()
    lib=gen.OUT/"layouts.json";doc=json.loads(lib.read_text(encoding="utf-8"));scenes=doc["scenes"]
    assert doc["library_version"]=="m1-mapset/1.0" and len(scenes)==60
    ids=[s["scene_id"] for s in scenes];assert len(set(ids))==60
    assert all(sid.isascii() and len(sid.encode("ascii"))<32 for sid in ids)
    cells={}
    seeds={}
    primary_variants={}
    for s in scenes:
        key=(s["density"],s["route_structure"],s["split"]);cells[key]=cells.get(key,0)+1
        seed=s["generation_seed"];assert seed not in seeds.setdefault(s["split"],set());seeds[s["split"]].add(seed)
        replica=int(s["scene_id"][-2:]);expected=gen.one_scene(s["density"],s["route_structure"],s["split"],replica)
        for field,value in expected.items():
            assert s.get(field)==value,(s["scene_id"],field,"seed regeneration mismatch")
        assert s["start_m"]==[0.0,0.0] and s["goal_m"]==[7.5,0.0]
        obs=s["obstacles"];path=[tuple(p) for p in s["routes"][0]["reference_path_m"]]
        assert len(obs)==s["obstacle_count"] and gen.area(obs)/gen.FIELD_AREA==s["density_definition"]["obstacle_footprint_area_fraction"]
        assert gen.DENSITIES[s["density"]][0] <= gen.area(obs)/gen.FIELD_AREA < gen.DENSITIES[s["density"]][1]
        assert all(gen.within_limits(o) for o in obs)
        assert gen.geom.no_overlap_and_spacing(obs,gen.MIN_GAP)
        assert gen.geom.path_clearance(path,obs)>=gen.MIN_CLEARANCE-1e-9
        assert all(gen.geom.point_shape_distance(gen.START,o)>=0.95 for o in obs)
        assert all(gen.geom.point_shape_distance(gen.GOAL,o)>=0.95 for o in obs)
        expected_blockers=gen.geom.line_blockers(obs)
        assert bool(expected_blockers)==s["direct_line_blocked"]
        assert set(expected_blockers)==set(s["direct_line_blocking_obstacles"])
        binding=s.get("production_scene_binding",{})
        spec=abs_scene.resolve_scene(s["filename"])
        assert (spec.scene_id,spec.obstacle_count)==(s["scene_id"],len(obs))
        assert binding.get("runtime_model_fingerprint")==spec.model_fingerprint
        assert binding.get("root_sha256")==spec.root_sha256 and binding.get("closure_sha256")==spec.closure_sha256
        pgen=Path(s["filename"])
        primary=tuple((round(float(o["x"]),3),round(float(o["y"]),3),
                       round(float(o.get("half_x",o.get("radius",0))),3),
                       round(float(o.get("half_y",o.get("radius",0))),3),round(float(o.get("yaw_rad",0)),3))
                      for o in obs if o["role"]=="primary")
        primary_variants.setdefault((s["route_structure"],s["density"]),set()).add(primary)
    assert len(cells)==30 and all(v==2 for v in cells.values()),cells
    dev={s["generation_seed"] for s in scenes if s["split"]=="development"}
    held={s["generation_seed"] for s in scenes if s["split"]=="held_out"}
    assert len(dev)==30 and len(held)==30 and not dev&held
    assert sum(s["direct_line_blocked"] for s in scenes)>=40
    assert all(len(v)>=3 for v in primary_variants.values()),"primary geometry variation inadequate"
    print(f"M1 verify PASS: 60 layouts, 30 dev/30 heldout, 30 strata, line-blocked={sum(s['direct_line_blocked'] for s in scenes)}, deterministic XML/layout/bindings valid")

if __name__=="__main__": main()
