#!/usr/bin/env python3
"""Generate the deterministic S7 candidate run matrix from the frozen M1 index."""

from __future__ import annotations

import argparse
import csv
import json
import random
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
M1_DIR = ROOT / "docs/thesis_project/designs/M1/library_v1"
DEFAULT_OUT = ROOT / "docs/thesis_project/designs/S7"
SEED = 2026102001
BOOTSTRAP_SEED = 2026102002
WILLIAMS = {
    "W0": ("A", "B", "D", "C"),
    "W1": ("B", "C", "A", "D"),
    "W2": ("C", "D", "B", "A"),
    "W3": ("D", "A", "C", "B"),
}
PARAMS = {
    "A": {"E": -0.05, "X": None, "H_s": None},
    "B": {"E": -0.05, "X": -0.10, "H_s": None},
    "C": {"E": -0.05, "X": None, "H_s": 0.20},
    "D": {"E": -0.05, "X": -0.10, "H_s": 0.20},
}


def read_m1():
    with (M1_DIR / "scene_index.csv").open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    maps = [r for r in rows if r["split"] == "held_out"]
    with (M1_DIR / "layouts.json").open(encoding="utf-8") as f:
        layout_index = json.load(f)
    return maps, layout_index


def validate_m1(maps, layout_index):
    assert len(maps) == 30, f"expected 30 held-out maps, got {len(maps)}"
    assert len({r["display_name"] for r in maps}) == 30
    assert len({r["scene_id"] for r in maps}) == 30
    assert len({r["filename"] for r in maps}) == 30
    assert {r["display_name"] for r in maps} == {
        f"holdout{i:02d}" for i in range(1, 31)
    }
    assert layout_index["library_version"] == "m1-mapset/1.0"
    assert layout_index["field_is_physical_wall"] is False
    bounds = layout_index["field_bounds_m"]
    assert bounds == {"x": [-0.5, 8.0], "y": [-3.0, 3.0]}
    assert layout_index["start_m"] == [0.0, 0.0]
    assert layout_index["goal_m"] == [7.5, 0.0]
    layout_scenes = {
        scene["display_name"]: scene
        for scene in layout_index["scenes"]
        if scene["split"] == "held_out"
    }
    assert len(layout_scenes) == 30
    for row in maps:
        scene = layout_scenes[row["display_name"]]
        assert scene["scene_id"] == row["scene_id"]
        assert scene["filename"] == row["filename"]
        assert scene["density"] == row["density"]
        assert scene["route_structure"] == row["route_structure"]
        assert scene["goal_m"] == [float(row["goal_x"]), float(row["goal_y"])]
        assert scene["field_bounds_m"] == bounds
        assert scene["field_is_physical_wall"] is False
    assert {float(r["goal_x"]) for r in maps} == {7.5}
    assert {float(r["goal_y"]) for r in maps} == {0.0}
    assert {float(r["sim_limit_s"]) for r in maps} == {20.0}
    assert {float(r["wall_cap_s"]) for r in maps} == {120.0}
    return layout_index["library_version"], bounds


def build_matrix(seed=SEED):
    maps, layout_index = read_m1()
    map_version, bounds = validate_m1(maps, layout_index)
    rng = random.Random(seed)
    shuffled_maps = sorted(maps, key=lambda r: r["display_name"])
    rng.shuffle(shuffled_maps)

    blocks = []
    for j, map_row in enumerate(shuffled_maps):
        left = f"W{j % 4}"
        right = f"W{(j + 2) % 4}"
        # The two sequences form an opposite pair; a seeded fair bit assigns
        # which one is repeat 1. Each map's two repeats therefore differ.
        seq_by_repeat = {1: left, 2: right}
        if rng.randrange(2):
            seq_by_repeat = {1: right, 2: left}
        for repeat in (1, 2):
            blocks.append(
                {
                    "map": map_row,
                    "repeat_block": repeat,
                    "williams_sequence": seq_by_repeat[repeat],
                    "groups": WILLIAMS[seq_by_repeat[repeat]],
                }
            )
    rng.shuffle(blocks)

    records = []
    global_order = 0
    for block_order, block in enumerate(blocks, start=1):
        m = block["map"]
        for position, group in enumerate(block["groups"], start=1):
            global_order += 1
            p = PARAMS[group]
            records.append(
                {
                    "plan_item_id": f"S7-{global_order:03d}",
                    "global_order": global_order,
                    "block_order": block_order,
                    "display_name": m["display_name"],
                    "filename": m["filename"],
                    "scene_id": m["scene_id"],
                    "map_version": map_version,
                    "split": "held_out",
                    "density": m["density"],
                    "route_structure": m["route_structure"],
                    "repeat_block": block["repeat_block"],
                    "williams_sequence_id": block["williams_sequence"],
                    "within_block_position": position,
                    "group": group,
                    "E": p["E"],
                    "X": p["X"],
                    "H_s": p["H_s"],
                    "start_x_m": 0.0,
                    "start_y_m": 0.0,
                    "goal_x_m": float(m["goal_x"]),
                    "goal_y_m": float(m["goal_y"]),
                    "arrival_radius_m": 0.5,
                    "sim_limit_s": float(m["sim_limit_s"]),
                    "wall_cap_s": float(m["wall_cap_s"]),
                    "standing_observation_sim_s": 2.0,
                    "standing_wall_cap_s": 30.0,
                    "mujoco_window": "hidden",
                    "rviz_enabled": False,
                    "schedule_seed": seed,
                }
            )

    metadata = {
        "matrix_id": "S7-formal-240-candidate-v1",
        "status": "CANDIDATE_FROZEN_PENDING_DIRECTOR_REVIEW",
        "authorized_to_run": False,
        "schedule_seed": seed,
        "bootstrap_seed": BOOTSTRAP_SEED,
        "map_version": map_version,
        "scene_index": "docs/thesis_project/designs/M1/library_v1/scene_index.csv",
        "layouts": "docs/thesis_project/designs/M1/library_v1/layouts.json",
        "field_bounds_m": bounds,
        "field_is_physical_wall": False,
        "williams_sequences": {k: list(v) for k, v in WILLIAMS.items()},
        "sequence_assignment": "shuffle 30 map rows; for shuffled index j use W[j mod 4] and W[(j+2) mod 4], seeded fair bit assigns these to repeats 1/2; shuffle 60 complete blocks; preserve each block's group order",
        "seed_scope": "schedule ordering only; does not seed MuJoCo physics, policy randomness, or reset/initial state",
        "repeat_definition": "two independent reset/stand/mission blocks within each map; not exact initial-state replay",
        "groups": PARAMS,
        "records": records,
    }
    validate_matrix(records)
    return metadata


def validate_matrix(records):
    assert len(records) == 240
    assert len({r["plan_item_id"] for r in records}) == 240
    assert [r["global_order"] for r in records] == list(range(1, 241))
    by_map_repeat = Counter((r["display_name"], r["repeat_block"]) for r in records)
    assert len(by_map_repeat) == 60 and set(by_map_repeat.values()) == {4}
    assert all(
        {r["group"] for r in records if (r["display_name"], r["repeat_block"]) == key}
        == {"A", "B", "C", "D"}
        for key in by_map_repeat
    )
    by_map = Counter(r["display_name"] for r in records)
    assert len(by_map) == 30 and set(by_map.values()) == {8}
    by_group = Counter(r["group"] for r in records)
    assert by_group == {g: 60 for g in "ABCD"}
    by_seq = Counter(r["williams_sequence_id"] for r in records if r["within_block_position"] == 1)
    assert by_seq == {w: 15 for w in WILLIAMS}
    by_group_position = Counter((r["group"], r["within_block_position"]) for r in records)
    assert all(by_group_position[(g, p)] == 15 for g in "ABCD" for p in range(1, 5))
    map_sequences = {}
    for r in records:
        map_sequences.setdefault(r["display_name"], {})[r["repeat_block"]] = r["williams_sequence_id"]
    assert all(v[1] != v[2] for v in map_sequences.values())
    transitions = Counter()
    for r in records:
        if r["within_block_position"] < 4:
            next_row = next(
                x for x in records
                if x["block_order"] == r["block_order"]
                and x["within_block_position"] == r["within_block_position"] + 1
            )
            transitions[(r["group"], next_row["group"])] += 1
    assert len(transitions) == 12 and set(transitions.values()) == {15}
    for r in records:
        assert r["split"] == "held_out"
        assert (r["E"], r["X"], r["H_s"]) == (
            PARAMS[r["group"]]["E"], PARAMS[r["group"]]["X"], PARAMS[r["group"]]["H_s"]
        )
        assert r["goal_x_m"] == 7.5 and r["goal_y_m"] == 0.0
        assert r["sim_limit_s"] == 20.0 and r["wall_cap_s"] == 120.0
        assert r["arrival_radius_m"] == 0.5
        assert r["standing_observation_sim_s"] == 2.0 and r["standing_wall_cap_s"] == 30.0
        assert r["mujoco_window"] == "hidden" and r["rviz_enabled"] is False


CSV_FIELDS = [
    "plan_item_id", "global_order", "block_order", "display_name", "filename",
    "scene_id", "map_version", "split", "density", "route_structure", "repeat_block",
    "williams_sequence_id", "within_block_position", "group", "E", "X", "H_s",
    "start_x_m", "start_y_m", "goal_x_m", "goal_y_m", "arrival_radius_m",
    "sim_limit_s", "wall_cap_s", "standing_observation_sim_s", "standing_wall_cap_s",
    "mujoco_window", "rviz_enabled", "schedule_seed",
]


def write_outputs(data, out_dir):
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "formal_matrix_240.json").write_text(
        json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    with (out_dir / "formal_matrix_240.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(data["records"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()
    matrix = build_matrix()
    # Determinism check is performed before writing; no runtime is started.
    assert matrix == build_matrix()
    write_outputs(matrix, args.output_dir)
    print(f"wrote {len(matrix['records'])} candidate rows to {args.output_dir}")
    print("validation: 240 unique items; 30x2 blocks; 60/group; 15/sequence; 15/group-position; 15/directed transition")
    print("status: candidate only; not authorized to run")


if __name__ == "__main__":
    main()
