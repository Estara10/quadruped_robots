#!/usr/bin/env python3
"""Focused offline checks for the M2 144-run plan and fixed score."""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import manual_t1_m2_round2 as batch  # noqa: E402
import score_t1_m2_round2 as scoring  # noqa: E402
from s1_run_diagnostic import overlay_configuration_matches  # noqa: E402


def main() -> int:
    catalog = batch.load_catalog()
    plan1, screenshots1 = batch.make_schedule(catalog)
    plan2, screenshots2 = batch.make_schedule(catalog)
    batch.validate_schedule(plan1, catalog)
    assert len(plan1) == 144 and plan1 == plan2
    assert screenshots1 == screenshots2 and len(screenshots1) == 8
    assert set(batch.XS) == {-0.06, -0.07, -0.08, -0.09, -0.10, -0.11, -0.12, -0.13}
    assert all(row["group"] == "B" and row["hold_enabled"] is False for row in plan1)
    for scene in batch.MAPS:
        assert all(sum(r["map"] == scene and r["exit_threshold_X"] == x for r in plan1) == 3 for x in batch.XS)
    chosen = [row for row in plan1 if row["screenshot_preselected"]]
    assert len(chosen) == 8 and len({row["map"] for row in chosen}) == 1
    assert all(row["repeat"] == 1 for row in chosen) and {row["exit_threshold_X"] for row in chosen} == set(batch.XS)

    for x in batch.XS:
        overlay = batch.ROOT / "logs/t1/m2_exit_round2/20261016_174118_5/overlays" / f"B_{x:.2f}"
        config = overlay / "share/go2_description/config/abs/config.yaml"
        text = config.read_text(encoding="utf-8")
        assert overlay_configuration_matches(text, "B", -0.05, x, 0.30, 7.5, 0.0)
        assert "recovery_min_hold_s:" not in text
        assert batch.legacy.manual.verify_overlay_detached(batch.SOURCE, overlay)
    # YAML/numeric tolerance: decimal renderings represent the same parameter value.
    sample = (batch.ROOT / "logs/t1/m2_exit_round2/20261016_174118_5/overlays/B_-0.10/share/go2_description/config/abs/config.yaml").read_text()
    assert overlay_configuration_matches(sample.replace("-0.10000000000000001", "-0.1"),
                                         "B", -0.05, -0.10, 0.30, 7.5, 0.0)

    base = {"sequence": "1", "plan_item_id": "fixture", "map": "m2dev01.xml", "scene_id": "m2d01",
        "group": "B", "exit_threshold_X": "-0.10", "repeat": "1", "status": "VALID_TASK_RESULT",
        "record_validity": "VALID", "terminal_result": "ARRIVED", "motion_collision_result": "False",
        "elapsed_task_duration_s": "10", "valid_motion_duration_s": "60",
        "short_reverse_switch_count": "257.8", "short_reverse_switch_coverage": "complete",
        "short_reverse_per_valid_min": str(257.8), "attempts": "1"}
    scored = scoring.score_row(base)
    assert scored["score_status"] == "COMPLETE" and abs(scored["total_score_100"]-85.0) < 1e-9
    assert abs(scored["stability_score_0_1"]-0.5) < 1e-9
    unknown = dict(base, motion_collision_result="UNKNOWN")
    assert scoring.score_row(unknown)["score_status"] == "UNKNOWN"
    print("PASS: seeded 144-row M2 schedule, 6x8x3 balance, screenshot cap, detached B overlays, no hold, numeric parser, weighted score and UNKNOWN handling")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
