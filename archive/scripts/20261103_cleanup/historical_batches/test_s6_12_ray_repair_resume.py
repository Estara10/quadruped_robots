#!/usr/bin/env python3
"""Targeted offline checks for the narrowly authorized row-66 continuation gate."""
from __future__ import annotations

import copy
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import manual_s6_12_coarse as batch  # noqa: E402


def must_reject(plan: dict, root: Path) -> None:
    try:
        batch.validate_authorized_ray_repair_resume(plan, root)
    except RuntimeError:
        return
    raise AssertionError("invalid resume state was accepted")


def main() -> None:
    root = ROOT / "logs/s6_12/b_exit_coarse/20261015_192706_3"
    test_fall_analysis_final_13_gate()


def test_fall_analysis_final_13_gate() -> None:
    root = ROOT / "logs/s6_12/b_exit_coarse/20261015_192706_3"
    completed = json.loads((root / "s6_12_coarse_plan.json").read_text(encoding="utf-8"))
    # Recreate the narrowly authorized prelaunch state from the completed
    # frozen plan: rows 1-83 with verified outcomes, untouched rows 84-96.
    plan = copy.deepcopy(completed)
    plan["status"] = "STOPPED"
    plan["stop_sequence"] = 83
    for row in plan["runs"][83:]:
        row["status"] = "NOT_STARTED"
        row["attempts"] = []
        for field in ("started_at", "finished_at", "exit_code", "run_id", "run_directory",
                      "terminal_result", "record_validity", "metrics", "stop_reason"):
            row.pop(field, None)
    row = batch.validate_authorized_fall_analysis_final_13(plan, root)
    assert row["sequence"] == 83 and row["fall_time_reanalysis"]["motion_fall"] is True
    variants = []
    changed = copy.deepcopy(plan); changed["runs"][81]["status"] = "NOT_STARTED"; variants.append(changed)
    changed = copy.deepcopy(plan); changed["runs"][82]["fall_time_reanalysis"]["motion_fall"] = False; variants.append(changed)
    changed = copy.deepcopy(plan); changed["runs"][82]["run_id"] = "other"; variants.append(changed)
    changed = copy.deepcopy(plan); changed["runs"][83]["status"] = "RUNNING"; variants.append(changed)
    changed = copy.deepcopy(plan); changed["runs"][95]["attempts"].append({"run_id": "duplicate"}); variants.append(changed)
    for variant in variants:
        try:
            batch.validate_authorized_fall_analysis_final_13(variant, root)
        except RuntimeError:
            continue
        raise AssertionError("invalid fall-analysis continuation state was accepted")
    try:
        batch.validate_authorized_fall_analysis_final_13(copy.deepcopy(plan), root.parent)
    except RuntimeError:
        pass
    else:
        raise AssertionError("different batch root was accepted")
    print("S6-12 row-83 fall-analysis continuation gate PASS: exact 1-83 prefix and untouched 84-96 accepted; mutated prefix, evidence, run identity, suffix, and batch root rejected")


if __name__ == "__main__":
    main()
