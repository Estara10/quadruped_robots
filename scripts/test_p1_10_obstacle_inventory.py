#!/usr/bin/env python3
"""Offline tests for P1-10 historical obstacle-map formalization."""

from __future__ import annotations

import copy
import hashlib
import json
import shutil
import sys
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))
import p1_10_obstacle_inventory as inventory  # noqa: E402


EVIDENCE = REPO / "docs/evidence/P1-10/historical_five_map_inventory_20260903.json"
PAIR = REPO / "docs/evidence/P1-10/replay_pair_20260903_saved_record_closure/pair_manifest.json"
FLAT_SUITE = REPO / "scenarios/p1_10/scenario_suite_manifest.json"


def load_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def test_five_maps_and_closures():
    evidence = load_json(EVIDENCE)
    assert [item["map_file"] for item in evidence["maps"]] == list(inventory.MAPS)
    assert all(item["status"] == "INVENTORIED" for item in evidence["maps"])
    assert all(item["closure"]["files"] and not item["errors"] for item in evidence["maps"])
    assert all(all(file["present"] and file["sha256"] is not None and file["bytes"] > 0 for file in item["closure"]["files"]) for item in evidence["maps"])


def test_geom_metadata_matches_xml_and_candidates():
    evidence = load_json(EVIDENCE)
    for index, entry in enumerate(evidence["maps"], start=1):
        current = inventory.inventory_map(entry["map_file"])
        assert current["obstacles"] == entry["obstacles"]
        scenario = load_json(REPO / "scenarios/p1_10" / f"obstacle_test{index}.json")
        assert inventory.validate_candidate(scenario, entry) == []
        assert scenario["obstacle_layout"]["objects"]
        assert scenario["obstacle_layout"]["status"] == "DECLARED_FROM_XML_OFFLINE_ONLY"


def test_scene_obstacle_is_mechanical_alias():
    test1 = REPO / "unitree_mujoco/unitree_robots/go2/scene_test1.xml"
    alias = REPO / "unitree_mujoco/unitree_robots/go2/scene_obstacle.xml"
    assert test1.read_bytes() == alias.read_bytes()
    assert hashlib.sha256(test1.read_bytes()).hexdigest() == hashlib.sha256(alias.read_bytes()).hexdigest()
    evidence = load_json(EVIDENCE)
    assert evidence["deduplication"]["scene_obstacle.xml"]["formal_scenario_id"] == "obstacle_test1"


def test_root_mutation_fails_closed():
    evidence = load_json(EVIDENCE)
    expected = evidence["maps"][0]
    with tempfile.TemporaryDirectory(prefix="p1_10_map_mutation_") as temp:
        root = Path(temp) / "scene_test1.xml"
        root.write_bytes((REPO / expected["root_xml"]).read_bytes())
        text = root.read_text(encoding="utf-8").replace("4.77821", "4.77822", 1)
        root.write_text(text, encoding="utf-8")
        assert inventory.sha256_file(root)[0] != expected["root_xml_sha256"]


def test_asset_mutation_fails_closed():
    evidence = load_json(EVIDENCE)
    expected = evidence["maps"][0]
    asset = next(item for item in expected["closure"]["files"] if item["role"].endswith("_asset"))
    with tempfile.TemporaryDirectory(prefix="p1_10_asset_mutation_") as temp:
        root_dir = Path(temp) / "go2"
        shutil.copytree(REPO / "unitree_mujoco/unitree_robots/go2", root_dir)
        target = root_dir / Path(asset["path"]).relative_to("unitree_mujoco/unitree_robots/go2")
        target.write_bytes(target.read_bytes() + b"\nmutation")
        mutated = inventory.resolve_closure(root_dir / "scene_test1.xml")
        assert mutated["closure_sha256"] != expected["model_closure_sha256"]


def test_goal_qpos_and_obstacle_metadata_mutations_fail_closed():
    evidence = load_json(EVIDENCE)
    entry = evidence["maps"][0]
    scenario = load_json(REPO / "scenarios/p1_10/obstacle_test1.json")
    changed_goal = copy.deepcopy(scenario)
    changed_goal["goal"]["world_xy_m"][0] = 8.0
    assert inventory.validate_candidate(changed_goal, entry)
    changed_qpos = copy.deepcopy(scenario)
    changed_qpos["initial_state"]["qpos"][0] = 0.1
    assert inventory.validate_candidate(changed_qpos, entry)
    changed_metadata = copy.deepcopy(scenario)
    changed_metadata["obstacle_layout"]["objects"][0]["type"] = "sphere"
    assert inventory.validate_candidate(changed_metadata, entry)


def test_unsupported_candidates_cannot_enter_flat_capture():
    evidence = load_json(EVIDENCE)
    flat_suite = load_json(FLAT_SUITE)
    flat_ids = {item["scenario_id"] for item in flat_suite["scenarios"]}
    assert all(item["status"] == "UNSUPPORTED" for item in evidence["scenario_suite"]["formal_scenarios"])
    assert not ({"obstacle_test1", "obstacle_test2", "obstacle_test3", "obstacle_test4", "obstacle_test5"} & flat_ids)
    assert all(item["capture_eligibility"]["status"] == "UNSUPPORTED" for item in (load_json(REPO / "scenarios/p1_10" / f"obstacle_test{i}.json") for i in range(1, 6)))


def test_flat_suite_and_frozen_pair_unchanged():
    pair = load_json(PAIR)
    suite_sha = hashlib.sha256(FLAT_SUITE.read_bytes()).hexdigest()
    assert suite_sha == pair["scenario"]["suite_manifest_sha256"]
    assert pair["status_at_freeze"] == "FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW"


def test_ray_contract_fail_closed_boundaries():
    scenario = load_json(REPO / "scenarios/p1_10/obstacle_test1.json")
    ray = scenario["ray_authority"]
    assert inventory.validate_ray_contract(ray) == []
    for key, bad in (("status", None), ("stored_units", "meters"), ("frame", None)):
        changed = copy.deepcopy(ray)
        changed[key] = bad
        assert inventory.validate_ray_contract(changed)
    changed = copy.deepcopy(ray)
    changed["rays"]["count"] = 10
    assert inventory.validate_ray_contract(changed)


def test_terrain_is_extra_not_counted():
    evidence = load_json(EVIDENCE)
    assert evidence["extra_candidate"]["map_file"] == "scene_terrain.xml"
    assert evidence["extra_candidate"]["map_file"] not in [item["map_file"] for item in evidence["maps"]]


if __name__ == "__main__":
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_")]
    for test in tests:
        test()
        print(f"PASS {test.__name__}")
    print(f"{len(tests)} offline obstacle-inventory tests PASS")
