#!/usr/bin/env python3
"""Targeted offline checks for the frozen T1 400-item batch path."""
from __future__ import annotations

import os
import sys
import csv
import tempfile
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import manual_t1_expanded_400 as batch  # noqa: E402
import s1_run_diagnostic as diagnostic  # noqa: E402


def main() -> int:
    catalog = batch.scene_catalog()
    first, selected = batch.make_schedule(catalog)
    second, selected_again = batch.make_schedule(catalog)
    batch.validate_schedule(first, catalog)
    identity = lambda rows: [[r[k] for k in ("plan_item_id", "map", "scene_id", "repeat", "group", "exit_threshold_X")] for r in rows]
    assert identity(first) == identity(second)
    assert selected == selected_again and len(selected) == 8
    assert len(first) == 400 and len({r["plan_item_id"] for r in first}) == 400
    assert sum(r["group"] == "A" for r in first) == 50
    assert sum(r["group"] == "B" for r in first) == 350
    for m in catalog:
        for x in batch.X_LEVELS:
            rows = [r for r in first if r["map"] == m["filename"] and r["exit_threshold_X"] == x]
            assert len(rows) == 5
    selected_rows = [r for r in first if r["screenshot_preselected"]]
    assert {r["exit_threshold_X"] for r in selected_rows} == set(batch.X_LEVELS)
    assert len({r["map"] for r in selected_rows}) == 1 and all(r["repeat"] == 1 for r in selected_rows)

    # CSV round-tripping turns booleans into strings. Verify the actual summary
    # path preserves TRUE/FALSE/UNKNOWN safety counts and map-cluster inputs.
    fake_rows = []
    outcomes = ("True", "False", "UNKNOWN")
    for map_index, m in enumerate(catalog):
        for arm_index, x in enumerate(batch.X_LEVELS):
            fake_rows.append({
                "map": m["filename"], "scene_id": m["scene_id"], "density": m["density"],
                "route_structure": m["route_structure"], "exit_threshold_X": f"{x:.2f}",
                "status": "VALID_TASK_RESULT", "record_validity": "VALID", "attempts": "1",
                "terminal_result": "ARRIVED", "motion_collision_result": outcomes[(map_index+arm_index)%3],
                "motion_fall_result": outcomes[(map_index+arm_index+1)%3],
                "elapsed_task_duration_s": "8.0", "short_reverse_per_valid_min": "2.0",
                "recovery_share_valid_motion_time": "0.25",
            })
    with tempfile.TemporaryDirectory(prefix="t1_400_summary_") as tmp:
        summary_paths = batch.final_tables_and_cluster_ci(Path(tmp), fake_rows)
        assert summary_paths["map_threshold_rows"] == 80
        with (Path(tmp)/"map_threshold_summary_80.csv").open(encoding="utf-8") as stream:
            summary_rows = list(csv.DictReader(stream))
        first_summary = next(r for r in summary_rows if r["map"] == "dev01.xml" and r["exit_threshold_X"] == "-0.05")
        assert first_summary["collision_true_n"] == "1"
        assert first_summary["collision_false_n"] == "0"
        assert first_summary["collision_unknown_n"] == "0"
        assert first_summary["fall_true_n"] == "0"
        assert first_summary["fall_false_n"] == "1"
        assert first_summary["fall_unknown_n"] == "0"
        ci_rows = list(csv.DictReader((Path(tmp)/"map_cluster_bootstrap_ci.csv").open(encoding="utf-8")))
        assert any(r["contrast"] == "paired_B_minus_A" and r["map_clusters_available"] == "10" for r in ci_rows)

    sentinel = SimpleNamespace(scene_id="id", root_sha256="root", closure_sha256="closure", model_fingerprint="model")
    old_live = os.environ.get("ABS_MUJOCO_FRAMEBUFFER_CAPTURE")
    old_stand = os.environ.get("ABS_MUJOCO_STANDING_FRAMEBUFFER_CAPTURE")
    try:
        os.environ["ABS_MUJOCO_FRAMEBUFFER_CAPTURE"] = "/tmp/parent-live.ppm"
        os.environ["ABS_MUJOCO_STANDING_FRAMEBUFFER_CAPTURE"] = "/tmp/parent-stand.ppm"
        disabled = diagnostic.build_env("capture", Path("/tmp/overlay"), ":0", "/tmp/Xauthority",
                                        sentinel, Path("/tmp/run"), capture_framebuffer=False)
        assert "ABS_MUJOCO_FRAMEBUFFER_CAPTURE" not in disabled
        assert "ABS_MUJOCO_STANDING_FRAMEBUFFER_CAPTURE" not in disabled
        enabled = diagnostic.build_env("capture", Path("/tmp/overlay"), ":0", "/tmp/Xauthority",
                                       sentinel, Path("/tmp/run"), capture_framebuffer=True)
        assert enabled["ABS_MUJOCO_FRAMEBUFFER_CAPTURE"].endswith("/live_panel.ppm")
        assert enabled["ABS_MUJOCO_STANDING_FRAMEBUFFER_CAPTURE"].endswith("/standing_hold.ppm")
    finally:
        if old_live is None: os.environ.pop("ABS_MUJOCO_FRAMEBUFFER_CAPTURE", None)
        else: os.environ["ABS_MUJOCO_FRAMEBUFFER_CAPTURE"] = old_live
        if old_stand is None: os.environ.pop("ABS_MUJOCO_STANDING_FRAMEBUFFER_CAPTURE", None)
        else: os.environ["ABS_MUJOCO_STANDING_FRAMEBUFFER_CAPTURE"] = old_stand
    print("PASS: 400 unique seeded items; 10x8x5 allocation; A/B mapping; fixed eight screenshots; screenshot suppression and manual default preserved")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
