#!/usr/bin/env python3
"""Targeted T2 attempt-accounting review; writes only independent derived output."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from formal_attempt_accounting import audit_attempt_ledger  # noqa: E402


H180 = ROOT / "logs/02_T2_minimum_hold/hold_scan_180/20261018_192507_t2scan180"
C48 = ROOT / "logs/02_T2_minimum_hold/combined_check_48/20261019_1358_01_seed2026101901"
HFUNC = ROOT / "logs/02_T2_minimum_hold/hold_function_check/20261018_190259_t2hold"


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def get_attempt_facts() -> tuple[list[dict[str, Any]], list[dict[str, str]]]:
    rows: list[dict[str, Any]] = []
    correction_events: list[dict[str, str]] = []

    # T2 180: failed-but-started SYSTEM_SAFETY_ABORT and its one authorized historical substitute.
    base34 = H180 / "runs/034_r1_m2v2_d07_C_H0.20"
    r1 = base34 / "result/C/rep_01/1699cb934b7a4c73b2a666867eadf353"
    r2 = base34 / "attempt_02/result/C/rep_01/8a6cbbe58c3a4ad0af10f0928fbb856e"
    c1, t1, m1, p1 = (read_json(r1 / f) for f in ("run_context.json", "terminal.json", "metrics.json", "process_facts.json"))
    c2, t2, m2, p2 = (read_json(r2 / f) for f in ("run_context.json", "terminal.json", "metrics.json", "process_facts.json"))
    event = (t1.get("external_safety_events") or [{}])[0]
    diagnostic = event.get("ray_read_diagnostics", {})
    assert c1["run_id"] == t1["run_id"] == m1["run_id"] == "1699cb934b7a4c73b2a666867eadf353"
    assert c2["run_id"] == t2["run_id"] == m2["run_id"] == "8a6cbbe58c3a4ad0af10f0928fbb856e"
    assert c1["lifecycle"]["task_start"]["rl_step"] == 0
    assert t1["run_terminal_result"] == "SYSTEM_SAFETY_ABORT"
    assert event.get("event") == "POLICY_SAFETY_VETO"
    assert diagnostic.get("ray_reason") == "odd_sequence_writer_in_progress"
    assert diagnostic.get("attempts") == 16 and diagnostic.get("read_duration_ns") == 12545
    assert t1["shutdown_complete"] and t1["process_exit_code"] == 0 and not t1["forced_termination"]
    assert t2["run_terminal_result"] == "DIAGNOSTIC_SIM_TIME_LIMIT"
    assert m1["record_validity"] == m2["record_validity"] == "VALID"
    attempt01 = {
        "attempt_id": "t2h180-034-scene-attempt-1", "plan_item_id": "034_r1_m2v2_d07_C_H0.20",
        "kind": "scenario", "group": "C", "map": "d07.xml", "parameters": "E=-0.05; H=0.20s",
        "attempt_number": 1, "output_directory": str(base34 / "result"), "run_directory": str(r1),
        "scene_created": True, "run_id": t1["run_id"], "entered_rl": True,
        "raw_terminal": t1["run_terminal_result"], "record_validity": m1["record_validity"],
        "failure_stage": "motion; cycle 147 ray read failed: odd_sequence_writer_in_progress after 16 tries/12.545us; exact FAULTED frame persisted",
        "cause_evidence": {"class": "infrastructure_trigger_observed", "references": [
            "terminal.external_safety_events[0].ray_read_diagnostics",
            "terminal.external_safety_events[0].fault_frame_ref"],
            "independent_infrastructure_corroboration": False},
        "replaces_attempt_id": None, "is_replacement": False, "replacement_authorized": False,
        "process_exit_code": t1["process_exit_code"], "shutdown_complete": t1["shutdown_complete"],
        "forced_termination": t1["forced_termination"],
        "safety_task_completion": False,
        "cleanup_evidence": "terminal: shutdown_complete=True, exit_code=0, forced_termination=False",
        "post_detection_physics_is_exact_terminal_boundary": False,
        "evidence": str((r1 / "terminal.json").relative_to(ROOT)),
    }
    attempt02 = {
        "attempt_id": "t2h180-034-scene-attempt-2", "plan_item_id": "034_r1_m2v2_d07_C_H0.20",
        "kind": "scenario", "group": "C", "map": "d07.xml", "parameters": "E=-0.05; H=0.20s",
        "attempt_number": 2, "output_directory": str(base34 / "attempt_02/result"), "run_directory": str(r2),
        "scene_created": True, "run_id": t2["run_id"], "entered_rl": True,
        "raw_terminal": t2["run_terminal_result"], "record_validity": m2["record_validity"],
        "failure_stage": "task limit at 20 sim s; 4.436 m from goal",
        "cause_evidence": {"class": "none", "references": []},
        "replaces_attempt_id": attempt01["attempt_id"], "is_replacement": True,
        "replacement_authorized": True,
        "safety_task_completion": False,
        "process_exit_code": t2["process_exit_code"], "shutdown_complete": t2["shutdown_complete"],
        "forced_termination": t2["forced_termination"],
        "cleanup_evidence": "terminal: shutdown_complete=True, exit_code=0, forced_termination=False",
        "evidence": str((r2 / "terminal.json").relative_to(ROOT)),
    }
    rows.extend((attempt01, attempt02))

    # T2 48, plan item 001: two no-scene invocations then exactly one A scene run.
    batch = read_json(C48 / "batch_plan.json")
    item = next(x for x in batch["schedule"] if x["plan_item_id"] == "001_r1_e03_A")
    fails = batch["pre_scene_failures"]
    assert len(fails) == 2 and batch["status_counts"]["actual_scene_starts"] == 48
    assert batch["status_counts"]["pre_scene_orchestration_failures"] == 2
    assert [x["kind"] for x in item["attempts"]] == ["pre_scene_cli_orchestration", "runner_resume_orchestration", "scenario"]
    assert item["attempts"][0]["mujoco_process_started"] is False
    assert item["attempts"][1]["mujoco_process_started"] is False
    arun = item["attempts"][2]
    assert arun["run_id"] == item["run_id"] == "16465c346b8a4fd1b85c4548169fd6a2"
    rows.extend([
        {"attempt_id": "t2c48-001-invocation-1", "plan_item_id": item["plan_item_id"],
         "kind": "pre_scene_orchestration", "group": "A", "map": "e03.xml", "parameters": "E=-0.05",
         "invocation_number": 1, "scene_created": False, "run_id": None, "entered_rl": False,
         "raw_terminal": None, "record_validity": "NOT_APPLICABLE", "failure_stage": "pre-scene CLI validation",
         "cause_evidence": {"class": "parameter_dispatch_error", "references": [fails[0]["evidence"]]},
         "replaces_attempt_id": None, "is_replacement": False, "replacement_authorized": False,
         "process_exit_code": None, "shutdown_complete": True, "cleanup_evidence": "process_inventory_after_failure=[]",
         "evidence": fails[0]["evidence"]},
        {"attempt_id": "t2c48-001-invocation-2", "plan_item_id": item["plan_item_id"],
         "kind": "runner_resume_orchestration", "group": "A", "map": "e03.xml", "parameters": "E=-0.05",
         "invocation_number": 2, "scene_created": False, "run_id": None, "entered_rl": False,
         "raw_terminal": None, "record_validity": "NOT_APPLICABLE", "failure_stage": "resume directory allocation",
         "cause_evidence": {"class": "FileExistsError before child launch", "references": [
             "logs/02_T2_minimum_hold/combined_check_48/20261019_1358_01_seed2026101901/runner_resume_failure_02.json"]},
         "replaces_attempt_id": None, "is_replacement": False, "replacement_authorized": False,
         "process_exit_code": None, "shutdown_complete": True, "cleanup_evidence": "no CLI child/MuJoCo process created"},
        {"attempt_id": "t2c48-001-scene-attempt-1", "plan_item_id": item["plan_item_id"],
         "kind": "scenario", "group": "A", "map": "e03.xml", "parameters": "E=-0.05; no X/H fields",
         "invocation_number": 3, "attempt_number": 1, "output_directory": arun["output_directory"],
         "run_directory": arun["run_directory"], "scene_created": True, "run_id": arun["run_id"],
         "entered_rl": True, "raw_terminal": arun["terminal"], "record_validity": arun["record_validity"],
         "failure_stage": "none; valid selected A result", "cause_evidence": {"class": "none", "references": []},
         "replaces_attempt_id": None, "is_replacement": False, "replacement_authorized": False,
         "safety_task_completion": "UNKNOWN", "process_exit_code": arun["returncode"],
         "shutdown_complete": arun["process_shutdown_complete"], "cleanup_evidence": "closeout_complete=True; process inventory empty",
         "evidence": str(Path(arun["run_directory"]).relative_to(ROOT))},
    ])

    # T2 four-run function check, A was recovered by rereading the original record; no new scene attempt.
    fdir = HFUNC / "runs/01_A_no_hold/result/A/rep_01/4f452f21f03b4d8a9736716701a9f7da"
    fm, ft, fp = (read_json(fdir / f) for f in ("metrics.json", "terminal.json", "process_facts.json"))
    assert fm["record_validity"] == "VALID" and ft["run_terminal_result"] == "ARRIVED"
    assert ft["process_exit_code"] == 0 and ft["shutdown_complete"] is True
    rows.append({"attempt_id": "t2func-A-scene-attempt-1", "plan_item_id": "t2-hold-function-A-m02",
                 "kind": "scenario", "group": "A", "map": "m02.xml", "parameters": "E=-0.05; hold disabled",
                 "attempt_number": 1, "output_directory": str(HFUNC / "runs/01_A_no_hold/result"),
                 "run_directory": str(fdir), "scene_created": True, "run_id": ft["run_id"], "entered_rl": True,
                 "raw_terminal": ft["run_terminal_result"], "record_validity": fm["record_validity"],
                 "failure_stage": "initial reader lookup misclassified group A as missing; corrected from original plan/context/record",
                 "cause_evidence": {"class": "reader_group_filter_bug", "references": [
                     "docs/thesis_project/evidence/T2/hold_function_check/T2_HOLD_FUNCTION_CHECK_EXECUTION_REPORT.md"]},
                 "replaces_attempt_id": None, "is_replacement": False, "replacement_authorized": False,
                 "safety_task_completion": "UNKNOWN", "process_exit_code": ft["process_exit_code"],
                 "shutdown_complete": ft["shutdown_complete"], "forced_termination": fp["forced_termination"],
                 "cleanup_evidence": "shutdown_complete=True; process_exit_code=0",
                 "evidence": str((fdir / "terminal.json").relative_to(ROOT))})
    correction_events.append({"batch": "T2 hold function check", "plan_item_id": "t2-hold-function-A-m02",
                              "event": "read-side classification repair", "scenario_attempts_before": "1",
                              "scenario_attempts_after": "1", "run_id": ft["run_id"],
                              "effect": "original valid A was restored; no scenario rerun"})
    return rows, correction_events


def constructed_checks() -> dict[str, Any]:
    checks: list[dict[str, Any]] = []

    def run(name: str, plans: list[dict[str, Any]], rows: list[dict[str, Any]], expected: str,
            assertions: dict[str, Any] | None = None) -> None:
        result = audit_attempt_ledger(plans, rows)
        if result["status"] != expected:
            raise AssertionError(f"{name}: expected {expected}, got {result['status']} {result['errors']}")
        for key, value in (assertions or {}).items():
            actual: Any = result
            for part in key.split("."):
                actual = actual[int(part)] if isinstance(actual, list) else actual[part]
            if actual != value:
                raise AssertionError(f"{name}: {key} expected {value!r}, got {actual!r}")
        checks.append({"case": name, "expected_status": expected, "actual_status": result["status"],
                       "errors": result["errors"], "counts": result["counts"],
                       "plan_item_outcomes": result["plan_item_outcomes"],
                       "selected": [{"item": x["plan_item_id"], "run_id": x["selected_run_id"],
                                     "status": x["plan_status"]} for x in result["plan_items"]]})

    def plan(i: int, group: str = "A") -> dict[str, Any]:
        return {"plan_item_id": f"p{i:03d}", "group": group}

    def row(attempt: str, item: str, **kwargs: Any) -> dict[str, Any]:
        base = {"attempt_id": attempt, "plan_item_id": item, "group": "A", "kind": "scenario",
                "scene_created": True, "run_id": f"run-{attempt}", "run_directory": f"/tmp/{attempt}/run",
                "output_directory": f"/tmp/{attempt}", "entered_rl": True,
                "raw_terminal": "ARRIVED", "record_validity": "VALID",
                "safety_task_completion": True, "is_replacement": False,
                "replacement_authorized": False}
        base.update(kwargs)
        return base

    run("valid_arrival_not_repeated", [plan(1)], [row("a1", "p001"), row("a1", "p001")], "PASS",
        {"counts.actual_scene_starts": 1, "counts.invocation_attempts": 1})
    for terminal in ("COLLISION_TERMINATION", "FALL_TERMINATION", "DIAGNOSTIC_SIM_TIME_LIMIT"):
        run(f"legal_{terminal}_not_replaced", [plan(1)],
            [row("a1", "p001", raw_terminal=terminal, safety_task_completion=False)], "PASS",
            {"counts.actual_scene_starts": 1, "counts.replacement_scene_starts": 0})
    run("no_scene_invocation_does_not_consume_scene_budget", [plan(1)], [
        {"attempt_id": "i1", "plan_item_id": "p001", "group": "A", "scene_created": False,
         "run_id": None, "entered_rl": False}, row("a2", "p001")], "PASS",
        {"counts.invocation_attempts": 2, "counts.actual_scene_starts": 1})
    run("started_without_run_id_consumes_scene_budget", [plan(1)], [
        row("a1", "p001", run_id=None, record_validity="UNKNOWN", raw_terminal=None)], "PASS",
        {"counts.actual_scene_starts": 1, "counts.unique_run_ids": 0,
         "plan_items.0.plan_status": "UNRESOLVED_SCENE_ATTEMPT"})
    run("group_A_uses_its_bound_run_not_other_group_directory", [plan(1, "A")],
        [row("a1", "p001", group="A", run_id="actual-A-run", raw_terminal="ARRIVED")], "PASS",
        {"plan_items.0.selected_run_id": "actual-A-run"})
    first = row("a1", "p001", raw_terminal="STARTUP_FAILURE", record_validity="INVALID",
                entered_rl=False, is_replacement=False,
                attempt_number=1,
                cause_evidence={"class": "infrastructure_failure_confirmed", "references": ["event log"]})
    replacement = row("a2", "p001", run_id="replacement-run", raw_terminal="ARRIVED",
                      replaces_attempt_id="a1", is_replacement=True, replacement_authorized=True,
                      attempt_number=2,
                      output_directory="/tmp/a2", run_directory="/tmp/a2/replacement-run")
    run("replacement_bound_to_correct_plan_and_new_directory", [plan(1)], [first, replacement], "PASS",
        {"counts.actual_scene_starts": 2, "plan_items.0.selected_run_id": "replacement-run",
         "first_actual_scene_only_sensitivity.plan_item_outcomes.NOT_STARTED": 1})
    run("same_run_id_cannot_be_used_by_two_items", [plan(1), plan(2)],
        [row("a1", "p001", run_id="reused"), row("a2", "p002", run_id="reused")], "REJECTED")
    run("wrong_plan_replacement_reference_rejected", [plan(1), plan(2)], [
        first, row("a2", "p002", attempt_number=2, replaces_attempt_id="a1", is_replacement=True,
                   replacement_authorized=True, output_directory="/tmp/a2", run_directory="/tmp/a2/r")], "REJECTED")
    run("same_item_second_replacement_rejected", [plan(1)], [first, replacement,
        row("a3", "p001", attempt_number=3, replaces_attempt_id="a1", is_replacement=True, replacement_authorized=True,
            output_directory="/tmp/a3", run_directory="/tmp/a3/r")], "REJECTED")
    run("replacement_reusing_failed_directory_rejected", [plan(1)], [first,
        row("a2", "p001", attempt_number=2, run_directory=first["run_directory"], replaces_attempt_id="a1",
            is_replacement=True, replacement_authorized=True)], "REJECTED")
    run("replacement_without_own_directory_evidence_rejected", [plan(1)], [first,
        row("a2", "p001", attempt_number=2, replaces_attempt_id="a1", is_replacement=True,
            replacement_authorized=True, output_directory=None, run_directory=None)], "REJECTED")
    reversed_first = dict(first, attempt_number=2)
    reversed_replacement = dict(replacement, attempt_number=1)
    run("replacement_must_follow_its_failed_attempt", [plan(1)], [reversed_first, reversed_replacement], "REJECTED")
    system_abort = row("a1", "p001", attempt_number=1, raw_terminal="SYSTEM_SAFETY_ABORT", safety_task_completion=False,
                       cause_evidence={"class": "infrastructure_trigger_observed", "references": ["reader log"],
                                       "independent_infrastructure_corroboration": False})
    run("valid_system_safety_abort_is_plan_result_and_not_replaceable", [plan(1)], [system_abort], "PASS",
        {"plan_items.0.selected_run_id": "run-a1",
         "plan_item_outcomes.KNOWN_NOT_COMPLETE": 1,
         "safe_completion_bounds_over_all_plan_items.potential_upper": 0.0})
    run("system_abort_without_independent_infra_corroboration_not_replaced", [plan(1)],
        [system_abort, row("a2", "p001", attempt_number=2, replaces_attempt_id="a1", is_replacement=True,
                           replacement_authorized=True, output_directory="/tmp/a2", run_directory="/tmp/a2/r")],
        "REJECTED", {"plan_items.0.selected_run_id": "run-a1",
                     "first_actual_scene_only_sensitivity.plan_item_outcomes.KNOWN_NOT_COMPLETE": 1})
    run("invalid_record_keeps_independently_confirmed_safety_failure", [plan(1)], [
        row("a1", "p001", raw_terminal="COLLISION_TERMINATION", record_validity="INVALID",
            safety_task_completion="UNKNOWN", confirmed_task_safety_failure=True,
            safety_failure_evidence={"references": ["same-run task-window collision record"]},
            cause_evidence={"class": "recording_failure_confirmed", "references": ["record error"]})], "PASS",
        {"plan_items.0.plan_status": "KNOWN_SAFETY_FAILURE_RECORD_INCOMPLETE",
         "safe_completion_bounds_over_all_plan_items.potential_upper": 0.0,
         "first_actual_scene_only_sensitivity.plan_item_outcomes.KNOWN_NOT_COMPLETE": 1})
    run("safety_failure_claim_without_evidence_rejected", [plan(1)], [
        row("a1", "p001", record_validity="INVALID", safety_task_completion="UNKNOWN",
            confirmed_task_safety_failure=True, safety_failure_evidence={})], "REJECTED")
    confirmed_failure = row("a1", "p001", attempt_number=1, raw_terminal="COLLISION_TERMINATION",
                            record_validity="INVALID", safety_task_completion="UNKNOWN",
                            confirmed_task_safety_failure=True,
                            safety_failure_evidence={"references": ["task-window collision event"]},
                            cause_evidence={"class": "recording_failure_confirmed", "references": ["record error"]})
    run("replacement_cannot_erase_confirmed_safety_event", [plan(1)], [confirmed_failure,
        row("a2", "p001", attempt_number=2, replaces_attempt_id="a1", is_replacement=True,
            replacement_authorized=True, output_directory="/tmp/a2", run_directory="/tmp/a2/run")], "REJECTED")
    cap_plans = [plan(i) for i in range(1, 14)]
    cap_rows: list[dict[str, Any]] = []
    for i in range(1, 14):
        a = row(f"x{i}-1", f"p{i:03d}", raw_terminal="STARTUP_FAILURE", record_validity="INVALID",
                entered_rl=False, attempt_number=1,
                cause_evidence={"class": "infrastructure_failure_confirmed", "references": ["log"]})
        b = row(f"x{i}-2", f"p{i:03d}", replaces_attempt_id=f"x{i}-1", is_replacement=True,
                attempt_number=2,
                replacement_authorized=True, output_directory=f"/tmp/x{i}-2", run_directory=f"/tmp/x{i}-2/run")
        cap_rows.extend([a, b])
    run("batch_substitution_cap_12_enforced", cap_plans, cap_rows, "REJECTED")
    group_plans = [plan(i, g) for i, g in enumerate("ABCD", 1)]
    group_rows = [row(f"g{g}", f"p{i:03d}", group=g, run_id=f"run-{g}") for i, g in enumerate("ABCD", 1)]
    run("same_rules_apply_to_all_groups", group_plans, group_rows, "PASS",
        {"counts.plan_items": 4, "counts.actual_scene_starts": 4})
    return {"status": "PASS", "case_count": len(checks), "cases": checks,
            "scope": "constructed attempt ledgers only; no subprocess or scene execution"}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    facts, corrections = get_attempt_facts()
    historical_plans = [
        {"plan_item_id": "034_r1_m2v2_d07_C_H0.20", "group": "C"},
        {"plan_item_id": "001_r1_e03_A", "group": "A"},
        {"plan_item_id": "t2-hold-function-A-m02", "group": "A"},
    ]
    # Existing T2 choices are represented without authorizing or revising them.
    historical_rows = []
    for row in facts:
        r = dict(row)
        if r["plan_item_id"] == "034_r1_m2v2_d07_C_H0.20" and r["attempt_id"] == "t2h180-034-scene-attempt-1":
            r["raw_terminal"] = "SYSTEM_SAFETY_ABORT"
            r["record_validity"] = "VALID"
        elif r["plan_item_id"] == "034_r1_m2v2_d07_C_H0.20" and r.get("is_replacement"):
            # Historical replacement is auditable, but is not endorsed as a future automatic rule.
            r["replacement_authorized"] = True
        elif r["plan_item_id"] == "001_r1_e03_A" and r["kind"] == "scenario":
            r["safety_task_completion"] = "UNKNOWN"
        elif r["plan_item_id"] == "t2-hold-function-A-m02":
            r["safety_task_completion"] = "UNKNOWN"
        historical_rows.append(r)
    historical_audit = audit_attempt_ledger(historical_plans, historical_rows)
    # C34 historical replacement is expected to be rejected under current rule because independent
    # publisher-side corroboration is missing; expose that as the formal-rule sensitivity, not a data rewrite.
    checks = constructed_checks()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "attempt_accounting_check.json").write_text(json.dumps({
        "status": "PASS" if checks["status"] == "PASS" else "FAIL",
        "historical_fact_row_count": len(facts),
        "historical_rule_audit_status": historical_audit["status"],
        "historical_rule_audit_errors": historical_audit["errors"],
        "historical_rule_audit_counts": historical_audit["counts"],
        "historical_rule_audit_plan_item_outcomes": historical_audit["plan_item_outcomes"],
        "historical_rule_audit_safety_completion_bounds": historical_audit["safe_completion_bounds_over_all_plan_items"],
        "historical_rule_audit_plan_items": historical_audit["plan_items"],
        "historical_rule_audit_note": "Current formal rule does not auto-approve the historical d07/C substitute: controller-side read failure is directly observed, but independent publisher-side corroboration at the event is missing. Both actual starts remain in the all-start ledger.",
        "historical_attempt_facts": facts,
        "reader_correction_events": corrections,
        "constructed_checks": checks,
    }, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    fields = ["attempt_id", "batch", "plan_item_id", "group", "map", "parameters", "kind",
              "attempt_number", "invocation_number", "scene_created", "run_id", "entered_rl",
              "raw_terminal", "record_validity", "safety_task_completion", "failure_stage", "classification",
              "scene_start_consumed", "replacement_eligible", "replacement_reason", "cause_evidence", "replaces_attempt_id",
              "is_replacement", "replacement_authorized", "confirmed_task_safety_failure",
              "safety_failure_evidence", "denominator_scope", "process_exit_code",
              "shutdown_complete", "forced_termination", "cleanup_evidence", "evidence"]
    with (args.output / "attempt_facts.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in facts:
            output = {**row, "batch": "T2-180" if row["plan_item_id"].startswith("034_") else
                     "T2-48" if row["plan_item_id"].startswith("001_") else "T2-function"}
            classification = audit_attempt_ledger(
                [{"plan_item_id": row["plan_item_id"], "group": row.get("group")}], [row]
            )["attempt_facts"][0]["classification"]
            output["classification"] = classification.get("status")
            output["scene_start_consumed"] = classification.get("scene_start_consumed")
            output["replacement_eligible"] = classification.get("replacement_eligible")
            output["replacement_reason"] = classification.get("replacement_reason", "")
            if row.get("scene_created") is not True:
                output["denominator_scope"] = "orchestration invocation only; no scene-start or navigation denominator"
            elif row["plan_item_id"].startswith("034_") and row.get("is_replacement") is True:
                output["denominator_scope"] = "all-start reliability; historically selected plan-item result; not newly authorized by this audit"
            elif row["plan_item_id"].startswith("034_"):
                output["denominator_scope"] = "all-start reliability; first-start sensitivity; historical original attempt retained"
            elif row["plan_item_id"].startswith("001_"):
                output["denominator_scope"] = "all-start reliability and one T2-48 plan-item result"
            else:
                output["denominator_scope"] = "T2 functional check only; not a formal 240-item denominator"
            for key in ("cause_evidence", "safety_failure_evidence"):
                if key in output:
                    output[key] = json.dumps(output[key], ensure_ascii=False, sort_keys=True)
            writer.writerow(output)
    with (args.output / "reader_correction_events.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(corrections[0]))
        writer.writeheader(); writer.writerows(corrections)
    print(f"status={checks['status']} constructed_cases={checks['case_count']} historical_rows={len(facts)} audit={historical_audit['status']}")


if __name__ == "__main__":
    main()
