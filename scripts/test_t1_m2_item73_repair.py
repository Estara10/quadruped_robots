#!/usr/bin/env python3
"""Focused offline checks for the single authorized M2 item-073 retry."""
from __future__ import annotations

import copy
import json
import shutil
import sys
import tempfile
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import manual_t1_m2_round2 as batch  # noqa: E402


def main() -> int:
    source_root = ROOT / "logs/t1/m2_exit_round2/20261016_174309_9"
    original = json.loads((source_root / batch.PLAN_NAME).read_text(encoding="utf-8"))
    catalog = batch.load_catalog()
    facts = batch.validate_item73_repair_and_continue(original, source_root, catalog)
    assert facts["pending_sequences"] == list(range(73, 145))
    assert facts["pending_count"] == 72
    assert facts["accounting"]["completed_valid_plan_items"] == 72
    assert facts["accounting"]["confirmed_mujoco_scene_starts"] == 74
    assert facts["accounting"]["confirmed_no_scene_orchestration_failures"] == 4
    assert facts["budget"]["projected_scene_starts_after_additional"] == 146
    budget = copy.deepcopy(original)
    budget["scene_start_budget_policy"] = {"max_actual_scene_starts": 146}
    assert batch.validate_scene_budget(budget, additional_scene_starts=72)[
        "projected_scene_starts_after_additional"] == 146
    try:
        batch.validate_scene_budget(budget, additional_scene_starts=73)
    except RuntimeError:
        pass
    else:
        raise AssertionError("a 147th scene start was allowed")
    original_item73_dir = Path(original["runs"][72]["attempts"][0]["run_directory"])
    original_context = (original_item73_dir / "run_context.json").read_bytes()
    original_runtime = (original_item73_dir / "runtime_record.jsonl").read_bytes()

    with tempfile.TemporaryDirectory(prefix="t1-item73-entry-") as temp:
        temp_parent = Path(temp)
        batch_parent = temp_parent / "batches"
        root = batch_parent / "20261016_174309_9"
        root.mkdir(parents=True)
        shutil.copytree(source_root / "runs", root / "runs")
        plan = copy.deepcopy(original)
        batch.atomic_json(root / batch.PLAN_NAME, plan)
        (root / "glfw_preflight_item73_repair.json").write_text(json.dumps({
            "status": "PASS", "environment": {"DISPLAY": ":0",
                "XAUTHORITY": "/run/user/1000/gdm/Xauthority", "MUJOCO_GL": "glfw"},
            "vendor": "test", "renderer": "test", "version": "test"}), encoding="utf-8")
        batch.PARENT = batch_parent
        original_verify = batch.verify_preparation_recovery_freeze
        batch.verify_preparation_recovery_freeze = lambda p, c, **kw: {
            "continuation_code_freeze": batch.code_freeze(), "offline_fixture": True}
        spawned = []
        def no_child(command, **kwargs):
            spawned.append(command)
            raise OSError("mock: launcher refused before child creation")
        try:
            result = batch.main(["--batch-root", str(root), "--resume",
                "--authorized-item-073-repair-and-completion", "--display", ":0",
                "--xauthority", "/run/user/1000/gdm/Xauthority",
                "--plot-python", "/home/lidio/anaconda3/bin/python3"],
                test_hooks={"allow_fixture_root": True, "popen_factory": no_child})
        finally:
            batch.verify_preparation_recovery_freeze = original_verify
        assert result == 1 and len(spawned) == 1
        command = spawned[0]
        assert command[command.index("--scene") + 1] == "m2dev01.xml"
        assert command[command.index("--exit-threshold") + 1] == "-0.12"
        updated = json.loads((root / batch.PLAN_NAME).read_text(encoding="utf-8"))
        item73 = updated["runs"][72]
        assert [x["attempt_number"] for x in item73["attempts"]] == [1, 2]
        assert item73["attempts"][0]["run_id"] == facts["original_item73_run_id"]
        assert item73["attempts"][1]["scene_start_status"] == "CONFIRMED_NOT_STARTED"
        assert (root / "runs" / item73["plan_item_id"] / "attempt_02").is_dir()
        assert (original_item73_dir / "run_context.json").read_bytes() == original_context
        assert (original_item73_dir / "runtime_record.jsonl").read_bytes() == original_runtime
        assert updated["scene_start_budget_policy"]["max_actual_scene_starts"] == 146
        assert batch.scene_budget_accounting(updated)["confirmed_mujoco_scene_starts"] == 74
    print("PASS: item 073 raw-evidence gate, 146/147 scene budget, production main-entry dispatch, isolated attempt_02 allocation, and no-child failure accounting")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
