#!/usr/bin/env python3
"""R1 challenge15 six-layout A/C H=0.02 confirmation batch."""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import manual_r1_ac_hold_confirm_120 as core  # noqa: E402

MAPS = tuple(f"r1_ch15_left_x{i:02d}_v2.xml" for i in range(1, 7))
LEVELS = (("A", None), ("C", 0.02))
MAPSET = ROOT / "docs/thesis_project/designs/R1/challenge15_left_route_v2/layouts.json"

core.PARENT = ROOT / "logs/09_R1_switch_constraints/ch15_ac_120"
core.REPORT_PARENT = ROOT / "docs/thesis_project/evidence/R1/CH15_AC_120"
core.ANALYZER_SCRIPT = ROOT / "scripts/analyze_r1_ch15_ac_120.py"
core.MAPS = MAPS
core.LEVELS = LEVELS
core.REPEATS = 10
core.SCHEDULE_SEED = 2026110201
core.CLASSIFICATION = "R1 challenge15六张同源布局A/C H=0.02开发比较，非独立验证"
core.BASE_SCENE_STARTS = 120
core.MAX_SCENE_STARTS = 123


def verify_schedule(items: list[dict], meta: dict) -> dict:
    if len(items) != 120:
        raise ValueError("schedule does not contain 120 items")
    keys = [(item["map"], item["repeat"], core.level_id(item["group"], item["hold_s"]))
            for item in items]
    if len(set(keys)) != 120:
        raise ValueError("schedule keys are not unique")
    per_level = core.Counter(core.level_id(item["group"], item["hold_s"]) for item in items)
    cells = core.Counter((item["map"], core.level_id(item["group"], item["hold_s"])) for item in items)
    rounds = core.Counter(item["repeat"] for item in items)
    if set(per_level.values()) != {60} or len(cells) != 12 or set(cells.values()) != {10}:
        raise ValueError("expected two 60-run settings and twelve 10-run map-setting cells")
    if set(rounds) != set(range(1, 11)) or set(rounds.values()) != {12}:
        raise ValueError("expected ten rounds of twelve runs")
    expected = {(map_name, core.level_id(group, hold))
                for map_name in MAPS for group, hold in LEVELS}
    if any({(item["map"], core.level_id(item["group"], item["hold_s"]))
            for item in items if item["repeat"] == repeat} != expected
           for repeat in range(1, 11)):
        raise ValueError("each round must contain every map-setting combination once")
    positions = core.Counter((core.level_id(item["group"], item["hold_s"]), item["position"])
                             for item in items)
    return {"items": 120, "per_level": dict(per_level), "cells": len(cells),
            "rounds": {str(index): rounds[index] for index in range(1, 11)},
            "position_counts": {f"{key[0]}/{key[1]}": value for key, value in sorted(positions.items())},
            "base_levels": meta["base_levels"], "shifts": meta["shifts"]}


def create_plan(root: Path) -> dict:
    from abs_scene import resolve_scene
    layout = core.load_json(MAPSET)
    if layout.get("mapset_version") != "r1-challenge15-left-route-v2/1":
        raise RuntimeError("challenge15 mapset version differs from this batch")
    by_name = {item["filename"]: item for item in layout["scenes"]}
    if set(by_name) != set(MAPS):
        raise RuntimeError("challenge15 layout metadata does not contain exactly six maps")
    contract = {}
    for filename in MAPS:
        record = by_name[filename]
        spec = resolve_scene(filename)
        binding = record["production_scene_binding"]
        if (spec.scene_id != record["scene_id"] or spec.scene_id != binding["scene_id"] or
                spec.obstacle_count != 11 or binding["obstacle_count"] != 11 or
                any(binding[key] != getattr(spec, key) for key in
                    ("root_sha256", "closure_sha256", "model_fingerprint"))):
            raise RuntimeError(f"registered scene identity differs from layouts metadata: {filename}")
        contract[filename] = record

    blocks, meta = core.make_schedule()
    items = []
    for block in blocks:
        record = contract[block["map"]]
        spec = resolve_scene(block["map"])
        items.append({
            "sequence": block["sequence"],
            "plan_item_id": f"{block['sequence']:03d}_r{block['repeat']}_{spec.scene_id}_{core.level_id(block['group'], block['hold_s'])}",
            "repeat": block["repeat"], "block_index": block["block_index"],
            "position": block["position"], "map": block["map"],
            "scene_id": spec.scene_id, "difficulty_label": record["difficulty_label"],
            "source_set": record["source_set"], "source_filename": record["source_filename"],
            "layout_revision": record["layout_revision"], "field_bounds_m": record["field_bounds_m"],
            "xml_sha256": record["production_scene_binding"]["root_sha256"],
            "group": block["group"], "hold_s": block["hold_s"],
            "hold_consumed": block["group"] == "C",
            "cli_hold_fallback_s": block["hold_s"] if block["group"] == "C" else core.A_CLI_HOLD_FALLBACK,
            "cli_exit_fallback": core.CLI_EXIT_FALLBACK,
            "scene": block["map"], "goal_world_m": list(core.GOAL),
            "entry_threshold": core.E, "sim_limit_s": core.SIM_LIMIT_S,
            "wall_cap_s": core.WALL_CAP_S,
            "standing_observation_sim_s": core.STANDING_SIM_S,
            "standing_observation_wall_cap_s": core.STANDING_WALL_CAP_S,
            "status": "NOT_STARTED", "attempts": []})
    verification = verify_schedule(items, meta)
    plan = {
        "schema": "r1-ch15-ac-120-v1", "batch_id": root.name,
        "classification": core.CLASSIFICATION,
        "authority": "user-confirmed CH15_AC_120_EXECUTOR_PROMPT.md",
        "created_at": core.now(),
        "versions": {**core.smoke.record_versions(),
            "batch_scripts": {"runner_sha256": core.sha(Path(__file__)),
                              "shared_runner_sha256": core.sha(ROOT / "scripts/manual_r1_ac_hold_confirm_120.py"),
                              "analyzer_sha256": core.sha(core.ANALYZER_SCRIPT)}},
        "mapset": {"maps": list(MAPS), "layout_path": str(MAPSET.resolve()),
                   "mapset_version": layout["mapset_version"],
                   "design_source": "docs/thesis_project/designs/R1/challenge15_left_route_v2"},
        "contract": {"levels": [{"level": core.level_id(group, hold), "group": group, "hold_s": hold}
                                  for group, hold in LEVELS],
                     "entry_threshold_E": core.E, "single_threshold_exit": core.E,
                     "cli_exit_threshold_fallback": core.CLI_EXIT_FALLBACK,
                     "a_cli_hold_fallback_s": core.A_CLI_HOLD_FALLBACK,
                     "goal_world_m": list(core.GOAL), "arrival_radius_m": 0.5,
                     "sim_limit_s": core.SIM_LIMIT_S, "wall_cap_s": core.WALL_CAP_S,
                     "standing_observation_sim_s": core.STANDING_SIM_S,
                     "standing_wall_cap_s": core.STANDING_WALL_CAP_S,
                     "hysteresis": False, "A_hold_enabled": False, "C_hold_enabled": True},
        "schedule": {"seed": core.SCHEDULE_SEED, "meta": meta, "verification": verification},
        "rendering": {"backend": "GLFW", "requested_mode": "hidden",
                      "env": "ABS_MUJOCO_NO_WINDOW=1", "rviz": "start_rviz:=false",
                      "framebuffer_capture": False},
        "budget": {"base_scene_starts": 120, "max_scene_starts": core.MAX_SCENE_STARTS,
                   "max_starts_per_item": core.MAX_STARTS_PER_ITEM,
                   "max_engineering_repairs": core.MAX_ENGINEERING_REPAIRS,
                   "max_orchestration_corrections": core.MAX_ORCHESTRATION_CORRECTIONS,
                   "max_batch_bytes": core.MAX_BATCH_BYTES, "min_free_bytes": core.MIN_FREE_BYTES},
        "counters": {"confirmed_scene_starts": 0, "engineering_repairs": 0,
                     "orchestration_corrections": 0}, "items": items}
    core.atomic_json(root / core.PLAN_FILE, plan)
    return plan


core.verify_schedule = verify_schedule
core.create_plan = create_plan

if __name__ == "__main__":
    raise SystemExit(core.main())
