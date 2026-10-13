#!/usr/bin/env python3
"""Focused offline invariants for the generated S6-09 map library."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import generate_s6_09_scenes as gen  # noqa: E402


def main() -> int:
    layouts = json.loads(gen.LAYOUTS.read_text(encoding="utf-8"))
    scenes = layouts["scenes"]
    assert len(scenes) == 60
    assert sum(s["split"] == "development" for s in scenes) == 30
    assert sum(s["split"] == "held_out" for s in scenes) == 30
    assert len({s["generation_seed"] for s in scenes}) == 60
    assert {s["split"] for s in scenes} == {"development", "held_out"}

    combo_counts = {}
    blocker_count = 0
    for scene in scenes:
        key = (scene["density"], scene["route_structure"])
        combo_counts[key] = combo_counts.get(key, 0) + 1
        assert scene["start_m"] == [0.0, 0.0] and scene["goal_m"] == [7.0, 0.0]
        assert scene["direct_line_blocked"] is True
        assert scene["direct_line_blocking_obstacles"]
        blocker_count += 1
        density = scene["density_definition"]["obstacle_footprint_area_fraction"]
        low, high = scene["density_definition"]["target_area_fraction_exclusive"]
        assert low <= density < high
        obstacles = scene["obstacles"]
        assert len(obstacles) == scene["obstacle_count"]
        assert all(0.32 <= o["height_m"] <= 0.62 for o in obstacles)
        assert all(gen.geom.in_bounds(o) and gen.geom.clear_start_goal(o) for o in obstacles)
        assert gen.geom.no_overlap_and_spacing(obstacles, gen.DENSITIES[scene["density"]]["min_gap_m"])
        routes = scene["routes"]
        expected_route_count = 2 if scene["route_structure"] == "route_fork" else 1
        assert len(routes) == expected_route_count
        for route in routes:
            path = [tuple(p) for p in route["reference_path_m"]]
            assert route["path_check"]["connected"]
            assert all(gen.geom.segment_clear(a, b, obstacles) for a, b in zip(path, path[1:]))
            assert gen.geom.path_clearance(path, obstacles) >= gen.geom.REQUIRED_CENTER_CLEARANCE_M
        structure = scene["route_structure"]
        details = scene["structure_parameters"]
        if structure == "alternating_bypass":
            centers = details["gate_centers_y_m"]
            assert centers[0] * centers[1] < 0 and centers[1] * centers[2] < 0
        elif structure == "offset_gates":
            centers = details["gate_centers_y_m"]
            assert len(set(centers)) == 3 and max(centers) - min(centers) >= 1.5
        elif structure == "route_fork":
            assert len(routes) == 2
            assert details["route_01_bypasses"] != details["route_02_bypasses"]
            assert abs(routes[0]["reference_path_m"][100][1] - routes[1]["reference_path_m"][100][1]) >= 2.5
        elif structure == "irregular_cluster":
            assert details["placement_method"] == "unstructured_uniform_random_then_connectivity_check"
    assert all(value == 4 for value in combo_counts.values())
    assert len(combo_counts) == 15
    assert blocker_count >= 40

    dev = {s["generation_seed"] for s in scenes if s["split"] == "development"}
    held = {s["generation_seed"] for s in scenes if s["split"] == "held_out"}
    assert not (dev & held)
    assert len({s["scene_id"] for s in scenes}) == 60
    assert all(s["scene_id"].isascii() and len(s["scene_id"].encode("ascii")) < 32
               for s in scenes)
    assert len({s["filename"] for s in scenes}) == 60
    assert len({s["preview"] for s in scenes}) == 60
    print("S6-09 library checks passed: 60 maps; 15 balanced density/structure cells;")
    print("30/30 development/held-out; disjoint seeds; geometry, route, and direct-line checks passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
