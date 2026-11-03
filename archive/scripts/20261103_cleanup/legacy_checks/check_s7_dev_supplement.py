#!/usr/bin/env python3
"""Targeted offline checks for the two-run m02 A/D dev supplement.

No MuJoCo, ROS, controller, model inference, or real scene start is invoked.
All batches live under logs/07_formal_preparation/dev_supplement_m02_AD.
"""
from __future__ import annotations

import json
import subprocess
import sys
import uuid
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import s7_dev_supplement as dev  # noqa: E402
import s7_formal_runner as runner  # noqa: E402
import s7_formal_analysis as analysis  # noqa: E402

OUT = dev.OUTPUT_ROOT / "readiness_check"
RESULTS: list[dict[str, Any]] = []


def record(name: str, passed: bool, detail: Any) -> None:
    RESULTS.append({"check": name, "status": "PASS" if passed else "FAIL",
                    "detail": detail})
    print(f"[{'PASS' if passed else 'FAIL'}] {name}: {detail}", flush=True)


def fake_summary(*, terminal: str = "ARRIVED", safe: bool | str = True,
                 cleanup: bool = True, duration: float = 8.0) -> dict[str, Any]:
    return {
        "run_id": "dev-fixture",
        "navigation_terminal_result": terminal,
        "terminal_result": terminal,
        "record_validity": "VALID",
        "task_window_safety": {
            "nonfoot_collision_failure": terminal == "COLLISION_TERMINATION",
            "fall": terminal == "FALL_TERMINATION",
            "coverage_complete_for_motion_interval": True,
        },
        "declared_boundary_physics_observation": {
            "status": "BASE_LINK_IN_BOUNDS_PHYSICS_STEPS_OBSERVED"},
        "rl_task_metric_boundary": {
            "start_cycle": 0, "end_cycle": 400, "elapsed_sim_s": duration,
            "valid_continuous_sim_s": duration, "cycle_identity_complete": True,
            "terminal_frame_payload_complete": True,
        },
        "safety_task_completion": {
            "status": safe,
            "standing_and_cleanup": {
                "standing_observation": {"complete": True},
                "normal_shutdown": cleanup, "shutdown_complete": cleanup,
                "process_exit_code": 0 if cleanup else None,
            },
        },
        "recovery_share_valid_motion_time": 0.1,
        "recovery_intervals": [{"duration_s": 0.2, "censored": False}],
        "short_reverse_switch_count": 4,
        "short_reverse_switch_coverage": "complete",
        "valid_motion_duration_s": duration,
        "transitions": [
            {"sim_clock_valid": True, "sim_time_s": 1.0, "mode_before": 0, "mode_after": 1},
            {"sim_clock_valid": True, "sim_time_s": 1.2, "mode_before": 1, "mode_after": 0},
        ],
        "target_distance_start_m": 7.5, "target_distance_final_m": 0.4,
        "foot_contact_count": 0, "foot_impact_count": 0,
    }


class DevFakeExecutor:
    def __init__(self, behaviors: list[str]):
        self.behaviors = list(behaviors)
        self.calls = 0

    def __call__(self, command: list[str], log_path: Path) -> dict[str, Any]:
        output_root = Path(command[command.index("--output-root") + 1])
        attempt_dir = output_root.parent
        behavior = self.behaviors.pop(0) if self.behaviors else "arrived"
        self.calls += 1
        log_path.parent.mkdir(parents=True, exist_ok=True)
        if behavior == "pre_scene":
            log_path.write_text("SCENE_PREFLIGHT_REJECTED dev fixture\n", encoding="utf-8")
            return {"stdout_result": {"result": "SCENE_PREFLIGHT_REJECTED",
                                      "reason": "dev fixture"},
                    "returncode": 2, "interrupted": False}
        if behavior == "unknown":
            log_path.write_text("no scene evidence\n", encoding="utf-8")
            return {"stdout_result": None, "returncode": None, "interrupted": False}

        group = command[command.index("--group") + 1]
        scene = command[command.index("--scene") + 1]
        entry = float(command[command.index("--entry-threshold") + 1])
        exit_arg = float(command[command.index("--exit-threshold") + 1])
        hold_arg = float(command[command.index("--hold-s") + 1])
        goal_x = float(command[command.index("--goal-x") + 1])
        goal_y = float(command[command.index("--goal-y") + 1])
        observed_group = "B" if behavior == "wrong_group" else group
        observed_exit = exit_arg if group in {"B", "D"} else entry
        observed_hold = hold_arg if group in {"C", "D"} else 0.0
        terminal = ("COLLISION_TERMINATION" if behavior == "collision" else
                    "DIAGNOSTIC_SIM_TIME_LIMIT" if behavior == "timeout" else
                    "ARRIVED")
        safe = False if behavior in {"collision", "timeout"} else True
        cleanup = behavior != "cleanup_unconfirmed"

        output_root.mkdir(parents=True, exist_ok=True)
        run_id = uuid.uuid4().hex
        run_dir = output_root / run_id
        run_dir.mkdir()
        (run_dir / "mujoco.log").write_text(
            "[MUJOCO-WINDOW] backend=GLFW requested=hidden visible=0\n"
            "MuJoCo version 3.3.3\n", encoding="utf-8")
        context = {
            "run_id": run_id, "session_id": 1,
            "settings": {
                "scene_id": "m2v2_m02", "group": group,
                "goal_world_m": [goal_x, goal_y], "entry_threshold": entry,
                "exit_threshold": observed_exit, "recovery_min_hold_s": observed_hold,
                "hysteresis": "true" if group in {"B", "D"} else "false",
                "hold": "true" if group in {"C", "D"} else "false",
            },
            "runtime_config_observed": {
                "group": observed_group, "entry_threshold": entry,
                "exit_threshold": observed_exit, "recovery_min_hold_s": observed_hold,
                "hysteresis": observed_group in {"B", "D"},
                "hold": observed_group in {"C", "D"},
                "switching_mode": {"A": 1, "B": 2, "C": 3, "D": 4}[observed_group],
                "session_id": 1,
                "source_binding": {
                    "group": observed_group, "controller_pid": 123, "writer_pid": 123,
                    "config": f"/dev-overlay/{observed_group}/config.yaml",
                    "package_share": f"/dev-overlay/{observed_group}",
                    "goal_world_m": [goal_x, goal_y],
                },
            },
            "runtime_source_validation": {
                "status": "confirmed", "reason": "all_source_events_match",
                "evidence": {"group": observed_group, "controller_pid": 123,
                             "writer_pid": 123},
            },
            "preflight": {
                "candidate_config_group_verified": observed_group,
                "candidate_config_entry_E_verified": True,
                "candidate_config_goal_x_m_verified": True,
                "candidate_config_legacy_fields_absent": True,
                "candidate_config_path": f"/dev-overlay/{observed_group}/config.yaml",
                "collision_reader_scene_binding": {"scene_id": "m2v2_m02"},
            },
            "lifecycle": {
                "mujoco_process_started_monotonic_ns": 1,
                "task_start": {"rl_step": 0, "session_id": 1},
            },
            "result_contract": {"classification": terminal},
        }
        terminal_record = {
            "run_id": run_id, "run_terminal_result": terminal,
            "normal_shutdown": cleanup, "shutdown_complete": cleanup,
            "process_exit_code": 0 if cleanup else None, "external_safety_events": [],
        }
        process_facts = {"exit_code": 0, "shutdown_complete": cleanup,
                         "closeout_complete": cleanup, "forced_termination": False}
        summary = fake_summary(terminal=terminal, safe=safe, cleanup=cleanup)
        summary["run_id"] = run_id
        for name, value in (("run_context.json", context), ("terminal.json", terminal_record),
                            ("process_facts.json", process_facts), ("metrics.json", summary)):
            (run_dir / name).write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
        return {"stdout_result": {"run_id": run_id, "result": terminal,
                                  "process_facts": process_facts},
                "returncode": 0, "interrupted": False}


def clean_batch(name: str) -> Path:
    import shutil
    path = OUT / "batches" / name
    if path.exists():
        shutil.rmtree(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def run_fake(name: str, behaviors: list[str], *, resume: bool = False,
             disk_probe=lambda: 50.0) -> tuple[dict[str, Any], Path]:
    records, meta, errors = dev.load_dev_plan()
    assert not errors, errors
    batch_root = (OUT / "batches" / name) if resume else clean_batch(name)
    auth = runner.read_json(dev.AUTHORIZATION_PATH)
    fake = DevFakeExecutor(behaviors)
    result = dev.run_dev_batch(
        batch_root=batch_root, batch_id=auth["batch_id"], records=records, meta=meta,
        authorization=auth, authorization_path=dev.AUTHORIZATION_PATH,
        executor=fake, analyze_run=None, resume=resume, stop_after_scene_starts=1,
        disk_probe=disk_probe,
        overlay_preparer=lambda root: {"status": "DEV_FIXTURE_OVERLAY", "groups": {}})
    result["executor_calls"] = fake.calls
    return result, batch_root


def check_plan() -> None:
    records, meta, errors = dev.load_dev_plan()
    validation = dev.validate_dev_plan(records, meta)
    groups = [row["group"] for row in sorted(records, key=lambda item: item["global_order"])]
    passed = (not errors and validation["status"] == "PASS" and len(records) == 2
              and groups == ["A", "D"] and all(row["filename"] == "m02.xml"
                                               for row in records)
              and all(row["scene_id"] == "m2v2_m02" for row in records)
              and meta["not_formal_data"] is True
              and meta["holdout_prohibited"] is True
              and not any("holdout" in row["filename"] for row in records))
    record("01_dev_plan_two_items", passed, {
        "plan_sha256": meta.get("plan_sha256"), "validation": validation,
        "load_errors": errors,
    })


def check_authorization_isolation() -> None:
    records, meta, errors = dev.load_dev_plan()
    assert not errors
    binding = dev.dev_binding(records, meta)
    auth = runner.read_json(dev.AUTHORIZATION_PATH)
    dev_check = dev.validate_dev_authorization(auth, binding, auth["batch_id"])
    formal_records, formal_meta, formal_errors = runner.load_frozen_plan()
    formal_binding = runner.matrix_binding(formal_records, formal_meta)
    formal_check = runner.validate_authorization(auth, formal_binding, batch_id=auth["batch_id"])
    batch_root = OUT / "refusal" / "batch"
    missing = subprocess.run([
        "/usr/bin/python3", str(ROOT / "scripts/run_s7_dev_supplement.py"), "run",
        "--batch-root", str(batch_root), "--batch-id", auth["batch_id"],
        "--authorization", str(OUT / "refusal" / "missing_auth.json"),
    ], cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        timeout=120, check=False)
    dev_fixture = dict(auth)
    dev_fixture["fixture_only"] = True
    fixture_check = dev.validate_dev_authorization(dev_fixture, binding, auth["batch_id"])
    passed = (dev_check["status"] == "PASS"
              and formal_check["status"] == "REJECTED"
              and missing.returncode == 2
              and "missing_or_unreadable_dev_authorization" in missing.stdout
              and not batch_root.exists()
              and fixture_check["status"] == "REJECTED")
    record("02_authorization_scope_isolation", passed, {
        "dev_check": dev_check["status"], "formal_check": formal_check["status"],
        "fixture_check": fixture_check["status"], "missing_exit": missing.returncode,
        "batch_dir_created": batch_root.exists(),
        "formal_load_errors": formal_errors,
    })


def check_first_run_stages_a() -> None:
    result, batch_root = run_fake("first_a", ["arrived"])
    ledger = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"]
    state = runner.read_json(batch_root / "batch_state.json")
    audit = runner.audit_ledger(
        dev.load_dev_plan()[0], ledger, max_substitutions=0, max_per_plan_item=1)
    passed = (result["status"] == "STAGED" and result["dispatched"] == 1
              and result.get("staged_reason") == "staged_after_1_scene_start"
              and len(ledger) == 1 and ledger[0]["group"] == "A"
              and ledger[0]["scene_created"] is True
              and ledger[0]["run_id"] == ledger[0]["run_directory"].split("/")[-1]
              and state.get("stage") == "STAGED_AFTER_SCENE_START"
              and state.get("next_pending") == ["DEV-M02-D"]
              and any("staged_after_1_scene_start" in str(item.get("reason"))
                      for item in runner.read_json(batch_root / "ledger" / "attempts.json")["checkpoints"])
              and audit["counts"]["actual_scene_starts"] == 1)
    record("03_first_run_only_a_and_checkpoint", passed, {
        "status": result["status"], "dispatched": result["dispatched"],
        "staged_reason": result.get("staged_reason"), "ledger": [
            {key: row.get(key) for key in ("plan_item_id", "group", "scene_created",
                                            "run_id", "raw_terminal", "record_validity")}
            for row in ledger], "state_stage": state.get("stage"),
        "next_pending": state.get("next_pending"),
    })


def check_resume_runs_d_only() -> None:
    first, batch_root = run_fake("resume_d", ["arrived"])
    before = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"]
    a_before = next(row for row in before if row["group"] == "A")
    a_run_id = a_before["run_id"]
    a_count_before = sum(1 for row in before if row["group"] == "A" and row["scene_created"] is True)
    second, _ = run_fake("resume_d", ["arrived"], resume=True)
    after = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"]
    a_after = [row for row in after if row["group"] == "A" and row["scene_created"] is True]
    d_after = [row for row in after if row["group"] == "D" and row["scene_created"] is True]
    order = [row["group"] for row in after if row["scene_created"] is True]
    passed = (first["status"] == "STAGED" and second["status"] == "COMPLETE"
              and second["dispatched"] == 1
              and len(a_after) == 1 and len(d_after) == 1
              and a_after[0]["run_id"] == a_run_id
              and a_count_before == 1 and order == ["A", "D"]
              and len({row["run_id"] for row in after if row["run_id"]}) == 2)
    record("04_resume_runs_d_without_rerunning_a", passed, {
        "first_status": first["status"], "second_status": second["status"],
        "second_dispatched": second["dispatched"], "orders": order,
        "A_scene_starts": len(a_after), "D_scene_starts": len(d_after),
        "A_run_id_unchanged": a_after[0]["run_id"] == a_run_id if a_after else None,
    })


def check_legal_collision_is_result() -> None:
    first, batch_root = run_fake("collision_a", ["collision"])
    a = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"][0]
    second, _ = run_fake("collision_a", ["arrived"], resume=True)
    after = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"]
    passed = (first["status"] == "STAGED" and a["raw_terminal"] == "COLLISION_TERMINATION"
              and a["safety_task_completion"] is False
              and second["status"] == "COMPLETE"
              and [row["group"] for row in after if row["scene_created"] is True] == ["A", "D"])
    record("05_legal_collision_is_result_not_retry", passed, {
        "first_status": first["status"], "A_terminal": a["raw_terminal"],
        "A_safety_task_completion": a["safety_task_completion"],
        "second_status": second["status"],
        "groups": [row["group"] for row in after if row["scene_created"] is True],
    })


def check_legal_timeout_is_result() -> None:
    first, batch_root = run_fake("timeout_a", ["timeout"])
    a = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"][0]
    second, _ = run_fake("timeout_a", ["arrived"], resume=True)
    after = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"]
    passed = (first["status"] == "STAGED"
              and a["raw_terminal"] == "DIAGNOSTIC_SIM_TIME_LIMIT"
              and a["safety_task_completion"] is False
              and second["status"] == "COMPLETE"
              and [row["group"] for row in after if row["scene_created"] is True] == ["A", "D"])
    record("06b_legal_timeout_is_result_not_retry", passed, {
        "first_status": first["status"], "A_terminal": a["raw_terminal"],
        "A_safety_task_completion": a["safety_task_completion"],
        "second_status": second["status"],
        "groups": [row["group"] for row in after if row["scene_created"] is True],
    })


def check_no_auto_retry() -> None:
    first, batch_root = run_fake("pre_scene_a", ["pre_scene"])
    second, _ = run_fake("pre_scene_a", [], resume=True)
    after = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"]
    passed = (first["status"] == "PAUSED"
              and first.get("pause_reason") == "pre_scene_failure_no_auto_retry"
              and second["status"] == "PAUSED"
              and second["dispatched"] == 0
              and len(after) == 1 and after[0]["scene_created"] is False)
    record("06_pre_scene_failure_no_auto_retry", passed, {
        "first_status": first["status"], "first_pause": first.get("pause_reason"),
        "second_status": second["status"], "second_dispatched": second["dispatched"],
        "scene_created": after[0]["scene_created"],
    })


def check_config_mismatch() -> None:
    result, batch_root = run_fake("wrong_group", ["wrong_group"])
    attempt = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"][0]
    consumption = attempt.get("runtime_config_consumption") or {}
    passed = (result["status"] == "PAUSED"
              and result.get("pause_reason") == "runtime_config_consumption_not_confirmed"
              and attempt["scene_created"] is True
              and any("observed_group:B!=A" in error
                      for error in consumption.get("errors", [])))
    record("07_config_consumption_mismatch_pauses", passed, {
        "status": result["status"], "pause": result.get("pause_reason"),
        "consumption_status": consumption.get("status"),
        "errors": consumption.get("errors"),
    })


def check_quota_schema() -> None:
    records, meta, _ = dev.load_dev_plan()
    binding = dev.dev_binding(records, meta)
    auth = runner.read_json(dev.AUTHORIZATION_PATH)
    too_many = dict(auth, max_scene_starts=3)
    substitution = dict(auth, max_substitutions=1)
    fixture = dict(auth, fixture_only=True)
    passed = (dev.validate_dev_authorization(too_many, binding, auth["batch_id"])["status"] == "REJECTED"
              and dev.validate_dev_authorization(substitution, binding, auth["batch_id"])["status"] == "REJECTED"
              and dev.validate_dev_authorization(fixture, binding, auth["batch_id"])["status"] == "REJECTED")
    record("08_dev_quota_and_no_fixture_bypass", passed, {
        "too_many": dev.validate_dev_authorization(too_many, binding, auth["batch_id"]),
        "substitution": dev.validate_dev_authorization(substitution, binding, auth["batch_id"]),
        "fixture": dev.validate_dev_authorization(fixture, binding, auth["batch_id"]),
    })


def check_analysis_consistency() -> None:
    first, batch_root = run_fake("analysis", ["arrived"])
    second, _ = run_fake("analysis", ["arrived"], resume=True)
    report = dev.analyze_dev_batch(batch_root, output_dir=OUT / "analysis")
    passed = (first["status"] == "STAGED" and second["status"] == "COMPLETE"
              and report.get("data_class") == "NON_FORMAL_DEV_SUPPLEMENT_DATA"
              and report.get("consistency_status") == "PASS"
              and report.get("A_before_D") is True
              and report.get("A_scene_starts") == 1
              and report.get("D_scene_starts") == 1
              and report.get("manifest", {}).get("data_class") == dev.DATA_CLASS
              and all(row["scene_run_binding_ok"] and row["config_consumption_ok"]
                      and row["cleanup_confirmed"]
                      for row in report.get("attempt_consistency", [])))
    record("09_analysis_consistency", passed, {
        "report_status": report.get("consistency_status"),
        "issues": report.get("consistency_issues"),
        "A_before_D": report.get("A_before_D"),
        "A_starts": report.get("A_scene_starts"), "D_starts": report.get("D_scene_starts"),
        "data_class": report.get("data_class"),
        "attempts": len(report.get("attempt_consistency", [])),
    })


def check_status_query() -> None:
    first, batch_root = run_fake("status", ["arrived"])
    records, meta, _ = dev.load_dev_plan()
    status = runner.status(batch_root, records, max_substitutions=0, max_per_plan_item=1)
    passed = (first["status"] == "STAGED" and status["status"] == "OK"
              and status["batch_state"].get("stage") == "STAGED_AFTER_SCENE_START"
              and status["batch_state"].get("next_pending") == ["DEV-M02-D"]
              and status["audit"]["counts"]["actual_scene_starts"] == 1)
    record("10_status_query", passed, {
        "runner_status": status["status"],
        "stage": status["batch_state"].get("stage"),
        "next_pending": status["batch_state"].get("next_pending"),
        "actual_scene_starts": status["audit"]["counts"]["actual_scene_starts"],
    })


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "batches").mkdir(parents=True, exist_ok=True)
    checks = [
        check_plan, check_authorization_isolation, check_first_run_stages_a,
        check_resume_runs_d_only, check_legal_collision_is_result,
        check_legal_timeout_is_result, check_no_auto_retry,
        check_config_mismatch, check_quota_schema, check_analysis_consistency,
        check_status_query,
    ]
    for check in checks:
        try:
            check()
        except Exception as error:  # noqa: BLE001
            import traceback
            record(check.__name__, False, {"exception": f"{type(error).__name__}: {error}",
                                           "traceback": traceback.format_exc()})
    summary = {
        "schema": "s7-dev-supplement-readiness-check-v1",
        "scope": "offline only; no MuJoCo, ROS, controller, model inference, or scene start",
        "data_class": "NON_FORMAL_DEV_SUPPLEMENT_DATA",
        "checks": RESULTS,
        "passed": sum(item["status"] == "PASS" for item in RESULTS),
        "failed": sum(item["status"] == "FAIL" for item in RESULTS),
        "total": len(RESULTS),
    }
    runner.write_json_atomic(OUT / "check_results.json", summary)
    print(json.dumps({"passed": summary["passed"], "failed": summary["failed"],
                      "total": summary["total"]}, ensure_ascii=False))
    return 0 if summary["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
