#!/usr/bin/env python3
"""Focused offline checks for S6-08 generated layouts and path rejection."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import generate_s6_08_scenes as gen  # noqa: E402


def main() -> int:
    source = json.loads(gen.LAYOUTS.read_text(encoding="utf-8"))
    scenes = source["scenes"]
    assert len(scenes) == 6
    assert {s["density"] for s in scenes} == {"sparse", "medium", "dense"}
    assert all(sum(x["density"] == d for x in scenes) == 2 for d in ("sparse", "medium", "dense"))
    for scene in scenes:
        regenerated = gen.gen_scene(scene["density"], int(scene["scene_id"][-2:]) - 1)
        assert regenerated == scene, scene["scene_id"]
        obstacles = scene["obstacles"]
        assert all(gen.in_bounds(o) and gen.clear_start_goal(o) for o in obstacles)
        assert gen.no_overlap_and_spacing(obstacles, gen.SPECS[scene["density"]]["gap_m"])
        assert scene["path_check"]["connected"]
        assert len(gen.line_blockers(obstacles)) >= 1
        route = [tuple(p) for p in scene["reference_path_m"]]
        assert all(gen.segment_clear(a, b, obstacles) for a, b in zip(route, route[1:]))
        assert gen.path_clearance(route, obstacles) >= gen.REQUIRED_CENTER_CLEARANCE_M
        assert gen.obstacle_area(obstacles[0]) > 0
    # A transverse, inflated wall across the entire search region must be rejected.
    wall = [{"name": "blocking_wall", "shape": "box", "x": 3.5, "y": 0.0,
             "half_x": 0.10, "half_y": 4.0, "yaw_rad": 0.0}]
    assert gen.grid_path(wall) is None
    print("S6-08 geometry checks passed: 6 deterministic maps, bounds/clearance/spacing,")
    print("line blockers, continuous reference-route clearance, and no-route rejection.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
