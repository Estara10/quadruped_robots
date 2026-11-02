#!/usr/bin/env python3
"""R1 single-layout challenge15 A/C hold-time comparison (40 runs)."""
from __future__ import annotations

import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import manual_r1_ac_hold_confirm_120 as core  # noqa: E402

MAPS = ("r1_ch15_left_x05_v2.xml",)
LEVELS = (("A", None), ("C", 0.02), ("C", 0.10), ("C", 0.12))
MAPSET = ROOT / "docs/thesis_project/designs/R1/challenge15_left_route_v2/layouts.json"

core.PARENT = ROOT / "logs/09_R1_switch_constraints/ch15_x05_hold_40"
core.REPORT_PARENT = ROOT / "docs/thesis_project/evidence/R1/CH15_X05_HOLD_40"
core.ANALYZER_SCRIPT = ROOT / "scripts/analyze_r1_ch15_x05_hold_40.py"
core.MAPS = MAPS
core.LEVELS = LEVELS
core.REPEATS = 10
core.SCHEDULE_SEED = 2026110202
core.CLASSIFICATION = "R1 challenge15 x05单图A/C保持时间局部比较，非独立验证"
core.BASE_SCENE_STARTS = 40
core.MAX_SCENE_STARTS = 42
core.MAX_ENGINEERING_REPAIRS = 2
core.MAX_BATCH_BYTES = 1 * 1024**3


def verify_schedule(items: list[dict], meta: dict) -> dict:
    if len(items) != 40:
        raise ValueError("schedule does not contain 40 items")
    level_key = core.level_id
    keys = [(item["map"], item["repeat"], level_key(item["group"], item["hold_s"]))
            for item in items]
    if len(set(keys)) != 40:
        raise ValueError("schedule keys are not unique")
    per_level = Counter(level_key(item["group"], item["hold_s"]) for item in items)
    if set(per_level.values()) != {10}:
        raise ValueError(f"expected ten runs per setting: {per_level}")
    rounds = Counter(item["repeat"] for item in items)
    if set(rounds) != set(range(1, 11)) or set(rounds.values()) != {4}:
        raise ValueError("expected ten rounds of four runs")
    expected = {(MAPS[0], level_key(group, hold)) for group, hold in LEVELS}
    if any({(item["map"], level_key(item["group"], item["hold_s"]))
            for item in items if item["repeat"] == repeat} != expected
           for repeat in range(1, 11)):
        raise ValueError("each round must contain all four settings exactly once")
    positions = Counter((level_key(item["group"], item["hold_s"]), item["position"])
                        for item in items)
    return {"items": 40, "per_level": dict(per_level), "rounds": dict(rounds),
            "position_counts": {f"{key[0]}/{key[1]}": value for key, value in sorted(positions.items())},
            "base_levels": meta["base_levels"], "shifts": meta["shifts"]}


def create_plan(root: Path) -> dict:
    from abs_scene import resolve_scene

    layout = core.load_json(MAPSET)
    if layout.get("mapset_version") != "r1-challenge15-left-route-v2/1":
        raise RuntimeError("challenge15 mapset version differs from the installed batch map")
    by_name = {item["filename"]: item for item in layout["scenes"]}
    if MAPS[0] not in by_name:
        raise RuntimeError(f"x05 is missing from the registered map metadata: {MAPS[0]}")
    record = by_name[MAPS[0]]
    spec = resolve_scene(MAPS[0])
    binding = record["production_scene_binding"]
    if (spec.scene_id != "r1ch15x05" or spec.scene_id != record["scene_id"] or
            spec.scene_id != binding["scene_id"] or spec.obstacle_count != 11 or
            binding["obstacle_count"] != 11 or
            any(binding[key] != getattr(spec, key) for key in
                ("root_sha256", "closure_sha256", "model_fingerprint"))):
        raise RuntimeError("registered x05 scene identity differs from its installed model")

    blocks, meta = core.make_schedule()
    items = []
    for block in blocks:
        items.append({
            "sequence": block["sequence"],
            "plan_item_id": f"{block['sequence']:03d}_r{block['repeat']}_{spec.scene_id}_{core.level_id(block['group'], block['hold_s'])}",
            "repeat": block["repeat"], "block_index": block["block_index"],
            "position": block["position"], "map": block["map"],
            "scene_id": spec.scene_id, "difficulty_label": record["difficulty_label"],
            "source_set": record["source_set"], "source_filename": record["source_filename"],
            "layout_revision": record["layout_revision"], "field_bounds_m": record["field_bounds_m"],
            "xml_sha256": binding["root_sha256"],
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
        "schema": "r1-ch15-x05-hold-40-v1", "batch_id": root.name,
        "classification": core.CLASSIFICATION,
        "authority": "user-confirmed R1图05 A/C保持时间比较40次 prompt",
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
        "budget": {"base_scene_starts": 40, "max_scene_starts": core.MAX_SCENE_STARTS,
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
