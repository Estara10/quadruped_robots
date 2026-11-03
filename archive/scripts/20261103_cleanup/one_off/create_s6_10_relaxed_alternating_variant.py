#!/usr/bin/env python3
"""Create and geometry-check the one S6-10 relaxed alternating-gate variant.

This derives from the S6-09 dense alternating development map. It does not
modify the 60-scene S6-09 library or its held-out split.
"""
from __future__ import annotations

import copy
import importlib
import itertools
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import generate_s6_08_scenes as geom  # noqa: E402
import generate_s6_09_scenes as mapgen  # noqa: E402
import abs_scene  # noqa: E402

SOURCE_FILENAME = "scene_random_s6_09_dense_alternating_bypass_dev_01.xml"
FILENAME = "scene_random_s6_10_dense_alternating_relaxed_dev_01.xml"
SCENE_ID = "s610_dar_d01"
SCENE_SLUG = "S6-10 relaxed alternating dev01"
LAYOUT_OUT = ROOT / "docs/thesis_project/designs/S6-10/variant_layout.json"
PREVIEW_OUT = ROOT / "docs/thesis_project/designs/S6-10/s6_10_relaxed_alternating_dev01.png"
SCENE_OUT = geom.SCENE_DIR / FILENAME
CATALOG = ROOT / "common/abs_scene_catalog.def"
REQUIRED_GAP_M = 0.10
GATE_XS = (1.20, 3.50, 5.80)
GATE_CENTERS_Y = (1.20, -1.20, 1.20)
GATE_WIDTH_M = 2.35
WALL_HALF_X_M = 0.175  # original 0.25 m reduced by 30% along x/travel
ROUTE_OFFSET_M = 0.85
GENERATION_SEED = 7_101_001  # source layout seed; this is a deterministic derivative


def length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def path_y_at_x(path, x):
    values = []
    for a, b in zip(path, path[1:]):
        if min(a[0], b[0]) - 1e-9 <= x <= max(a[0], b[0]) + 1e-9:
            if abs(b[0] - a[0]) < 1e-12:
                values.extend((a[1], b[1]))
            else:
                t = (x - a[0]) / (b[0] - a[0])
                values.append(a[1] + t * (b[1] - a[1]))
    if not values:
        raise ValueError(f"reference path does not cross gate x={x}")
    return sum(values) / len(values)


def build_variant():
    library = json.loads((ROOT / "docs/thesis_project/designs/S6-09/layouts.json").read_text())
    source = next(s for s in library["scenes"] if s["filename"] == SOURCE_FILENAME)
    scene = copy.deepcopy(source)
    scene.update({
        "scene_id": SCENE_ID,
        "scene_slug": SCENE_SLUG,
        "model_name": SCENE_SLUG,
        "filename": FILENAME,
        "preview": PREVIEW_OUT.name,
        "generator_version": "s6-10-relaxed-alternating/1.0",
        "generation_seed": GENERATION_SEED,
        "generation_attempt": 1,
        "run_seed": None,
        "split": "dev variant; outside S6-09 library",
        "density": "dense-derived-relaxed",
        "variant_of": {"filename": SOURCE_FILENAME, "scene_id": source["scene_id"],
                        "generation_seed": source["generation_seed"]},
    })
    ymin, ymax = geom.OBSTACLE_BOUNDS[1]
    obstacles = []
    for gate, (x, center) in enumerate(zip(GATE_XS, GATE_CENTERS_Y)):
        low, high = center - GATE_WIDTH_M / 2, center + GATE_WIDTH_M / 2
        if low - ymin <= 0.20 or ymax - high <= 0.20:
            raise ValueError("widened gate creates an undersized wall fragment")
        for index, lo, hi in ((1 + 2 * gate, ymin, low), (2 + 2 * gate, high, ymax)):
            old = source["obstacles"][index - 1]
            obstacles.append({
                "name": old["name"], "shape": "box", "x": x,
                "y": (lo + hi) / 2, "half_x": WALL_HALF_X_M,
                "half_y": (hi - lo) / 2, "yaw_rad": 0.0,
                "height_m": old["height_m"], "role": "gate_wall",
            })
    obstacles.append(copy.deepcopy(source["obstacles"][6]))  # retain the cylinder and its identity
    scene["obstacles"] = obstacles
    scene["obstacle_count"] = len(obstacles)
    scene["start_m"] = list(geom.START)
    scene["goal_m"] = list(geom.GOAL)

    path = [geom.START]
    for x, center in zip(GATE_XS, GATE_CENTERS_Y):
        path.extend(((x - ROUTE_OFFSET_M, center), (x, center), (x + ROUTE_OFFSET_M, center)))
    path.append(geom.GOAL)
    reference = mapgen.interpolate_route(path, step=0.025)
    blockers = geom.line_blockers(obstacles)
    pair_distances = [(geom.shape_distance(a, b), a["name"], b["name"])
                      for a, b in itertools.combinations(obstacles, 2)]
    min_pair = min(pair_distances)
    min_clearance = geom.path_clearance(reference, obstacles)
    min_spacing = min_pair[0]
    checks = {
        "all_obstacles_within_bounds": all(geom.in_bounds(o) for o in obstacles),
        "start_goal_clearance_at_least_0_95m": all(geom.clear_start_goal(o) for o in obstacles),
        "no_overlap_and_minimum_0_10m_obstacle_gap": geom.no_overlap_and_spacing(obstacles, REQUIRED_GAP_M),
        "start_goal_straight_is_blocked": bool(blockers),
        "three_gate_centers_alternate_left_right_left": list(GATE_CENTERS_Y) == [1.20, -1.20, 1.20],
        "reference_path_crosses_each_gate_center": True,
        "reference_path_continuously_clears_0_75m_envelope": min_clearance >= geom.REQUIRED_CENTER_CLEARANCE_M,
        "grid_connectivity_exists": geom.grid_path(obstacles) is not None,
    }
    crossings = [path_y_at_x(reference, x) for x in GATE_XS]
    for x, center, crossing in zip(GATE_XS, GATE_CENTERS_Y, crossings):
        low, high = center - GATE_WIDTH_M / 2, center + GATE_WIDTH_M / 2
        if crossing < low + geom.REQUIRED_CENTER_CLEARANCE_M or crossing > high - geom.REQUIRED_CENTER_CLEARANCE_M:
            checks["reference_path_crosses_each_gate_center"] = False
    if not all(checks.values()):
        raise ValueError(f"variant geometry rejected: {checks}; clearance={min_clearance}")

    old_area = sum(geom.obstacle_area(o) for o in source["obstacles"])
    new_area = sum(geom.obstacle_area(o) for o in obstacles)
    scene["routes"] = [{"route_id": "route_01", "reference_path_m": [list(p) for p in reference],
                        "path_check": geom.path_metrics(reference, obstacles)}]
    scene["direct_line_blocking_obstacles"] = blockers
    scene["structure_parameters"] = {"gate_centers_y_m": list(GATE_CENTERS_Y),
                                      "gate_width_m": GATE_WIDTH_M,
                                      "gate_x_m": list(GATE_XS),
                                      "longitudinal_wall_half_thickness_m": WALL_HALF_X_M}
    scene["density_definition"] = {
        "source_density": "dense",
        "obstacle_footprint_area_fraction": new_area / 48.0,
        "source_obstacle_footprint_area_fraction": old_area / 48.0,
        "minimum_obstacle_spacing_m": min_spacing,
        "minimum_obstacle_spacing_pair": [min_pair[1], min_pair[2]],
        "source_minimum_obstacle_spacing_m": source["density_definition"]["minimum_obstacle_spacing_m"],
        "minimum_route_centerline_corridor_width_m": 2 * min_clearance,
        "reference_route_lengths_m": [length(reference)],
        "detour_ratios": [length(reference) / 7.0],
    }
    scene["path_check_settings"] = {
        "grid_resolution_m": geom.GRID_M,
        "continuous_check_step_m": geom.PATH_CHECK_STEP_M,
        "go2_planar_envelope_radius_m": geom.GO2_ENVELOPE_RADIUS_M,
        "engineering_margin_m": geom.ENGINEERING_MARGIN_M,
        "required_centerline_clearance_m": geom.REQUIRED_CENTER_CLEARANCE_M,
        "start_goal_clearance_m": geom.START_GOAL_CLEARANCE_M,
        "route_centerline_width_definition": "twice minimum geometric clearance of reference route; not the physical gate opening",
        "actual_gate_opening_width_m": [GATE_WIDTH_M] * 3,
    }
    scene["variant_geometry_comparison"] = {
        "longitudinal_wall_full_thickness_m": {"before": 0.50, "after": 2 * WALL_HALF_X_M,
                                                "relative_reduction": 0.30},
        "gate_opening_width_m": {"before": source["structure_parameters"]["gate_width_m"],
                                 "after": GATE_WIDTH_M},
        "gate_center_x_m": {"before": [1.3, 3.5, 5.7], "after": list(GATE_XS)},
        "clear_run_between_adjacent_gate_faces_m": {
            "before": 2.2 - 2 * 0.25,
            "after": (GATE_XS[1] - GATE_XS[0]) - 2 * WALL_HALF_X_M,
            "second_to_third_after": (GATE_XS[2] - GATE_XS[1]) - 2 * WALL_HALF_X_M,
        },
        "minimum_obstacle_pair_clearance_m": {"before": source["density_definition"]["minimum_obstacle_spacing_m"],
                                              "after": min_spacing, "closest_pair": [min_pair[1], min_pair[2]]},
        "reference_path_minimum_obstacle_clearance_m": {
            "before": source["routes"][0]["path_check"]["minimum_geometric_obstacle_clearance_m"],
            "after": min_clearance,
        },
        "start_goal_obstacle_edge_clearance_x_m": {
            "before": [1.3 - 0.25, 7.0 - (5.7 + 0.25)],
            "after": [GATE_XS[0] - WALL_HALF_X_M, 7.0 - (GATE_XS[-1] + WALL_HALF_X_M)],
        },
    }
    scene["geometry_checks"] = dict(checks)
    scene["geometry_checks"].update({
        "actual_obstacle_pair_clearance_m": min_spacing,
        "actual_obstacle_pair_closest_names": [min_pair[1], min_pair[2]],
        "minimum_reference_route_clearance_m": min_clearance,
        "reference_route_length_m": length(reference),
        "reference_gate_center_crossings_y_m": crossings,
        "line_blocking_obstacles": blockers,
        "obstacle_count": len(obstacles),
        "actual_occupancy_fraction": new_area / 48.0,
        "occupancy_note": "dense-derived relaxed variant; actual occupancy is reported and this is not a density-bin sample",
    })
    return scene


def main():
    catalog = abs_scene.scene_catalog()
    if SCENE_ID in {spec.scene_id for name, spec in catalog.items() if name != FILENAME}:
        raise ValueError("variant internal scene ID collides with a different catalog entry")
    scene = build_variant()
    SCENE_OUT.write_text(geom.render_xml(scene), encoding="utf-8")
    scene["catalog_row"] = geom.catalog_row(scene, SCENE_OUT)
    source_lines = CATALOG.read_text(encoding="utf-8").splitlines()
    lines = []
    index = 0
    while index < len(source_lines):
        line = source_lines[index]
        if line.startswith("ABS_SCENE("):
            block = [line]
            while not block[-1].rstrip().endswith(")"):
                index += 1
                if index >= len(source_lines):
                    raise ValueError("truncated ABS_SCENE catalog entry")
                block.append(source_lines[index])
            if not block:
                raise ValueError("truncated ABS_SCENE catalog entry")
            if not line.startswith(f'ABS_SCENE("{FILENAME}"'):
                lines.extend(block)
            index += 1
        else:
            lines.append(line)
            index += 1
    CATALOG.write_text("\n".join(lines).rstrip() + "\n" +
                       f'ABS_SCENE("{FILENAME}", "{SCENE_ID}", {len(scene["obstacles"])}, kNamedPptObstacles,\n'
                       f'          "{scene["catalog_row"]["root_sha256"]}", "{scene["catalog_row"]["closure_sha256"]}",\n'
                       f'          "{scene["catalog_row"]["model_fingerprint"]}")\n', encoding="utf-8")
    PREVIEW_OUT.parent.mkdir(parents=True, exist_ok=True)
    preview_scene = copy.deepcopy(scene)
    preview_scene.update({"scene_slug": "S6-10 relaxed alt", "density": "dense-derived",
                          "split": "dev variant"})
    mapgen.render_scene(preview_scene).save(PREVIEW_OUT)
    scene["preview_path"] = PREVIEW_OUT.relative_to(ROOT).as_posix()
    LAYOUT_OUT.write_text(json.dumps(scene, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    importlib.reload(abs_scene)
    resolved = abs_scene.resolve_scene(FILENAME)
    if resolved.scene_id != SCENE_ID or resolved.obstacle_count != len(scene["obstacles"]):
        raise RuntimeError("production Python scene resolver disagrees with variant registration")
    print(json.dumps({"scene_id": SCENE_ID, "filename": FILENAME,
                      "layout": LAYOUT_OUT.relative_to(ROOT).as_posix(),
                      "preview": PREVIEW_OUT.relative_to(ROOT).as_posix(),
                      "catalog_binding": resolved.binding(),
                      "geometry_checks": scene["geometry_checks"]}, indent=2))


if __name__ == "__main__":
    main()
