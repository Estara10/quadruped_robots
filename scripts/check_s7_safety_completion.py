#!/usr/bin/env python3
"""Small S7-01 safety-completion check using the production run analyzer."""
from __future__ import annotations

import argparse
import copy
import csv
import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from run_record import load_record  # noqa: E402
from s1_analyze_run import calculate, motion_safety  # noqa: E402
from safety_task_completion import (  # noqa: E402
    assess_task_identity,
    classify_safety_task_completion,
)


REAL_CASES = {
    "R1": ("logs/02_T2_minimum_hold/combined_check_48/20261019_1358_01_seed2026101901/runs/001_r1_e03_A/invocation_03/producer/A/rep_01/16465c346b8a4fd1b85c4548169fd6a2",
           True),
    "R2": ("logs/04_B1_recording_validation/runs/m1dod01_D/D/rep_01/0f148c54215e409fbbcacb23955510de",
           "UNKNOWN"),
    "R3": ("logs/02_T2_minimum_hold/combined_check_48/20261019_1358_01_seed2026101901/runs/023_r1_d02_C/producer/C/rep_01/b40d5daa14b1403a8d0f050df55cd683",
           False),
    "R4": ("logs/02_T2_minimum_hold/hold_scan_180/20261018_192507_t2scan180/runs/101_r2_m2v2_d07_C_H0.10/result/C/rep_01/2547e48a82304e9ca0aac05ce518e92d",
           False),
    "R5": ("logs/02_T2_minimum_hold/combined_check_48/20261019_1358_01_seed2026101901/runs/044_r2_m06_D/producer/D/rep_01/c91ce19ba1d14b53b0b14ddaf9207d97",
           False),
    "R6": ("logs/02_T2_minimum_hold/hold_scan_180/20261018_192507_t2scan180/runs/034_r1_m2v2_d07_C_H0.20/result/C/rep_01/1699cb934b7a4c73b2a666867eadf353",
           False),
    "R7": ("logs/02_T2_minimum_hold/combined_check_48/20261019_1358_01_seed2026101901/runs/033_r2_d05_C/producer/C/rep_01/fd11b43b53d04ff1ab26cf3c0e6274f4",
           True),
    "R8": ("logs/02_T2_minimum_hold/hold_scan_180/20261018_192507_t2scan180/runs/153_r2_m2v2_m03_C_H0.30/result/C/rep_01/9348a1844e444336b3c6dfa48c2bf2d5",
           False),
    "R9": ("logs/01_T1_exit_threshold/legacy_coarse_S6_12/b_exit_coarse/20261015_192706_3/runs/083_m1dfd01_x05_r1/result/A/rep_01/8fecf8c227d54c199a63b7530e51eab0",
           False),
}


def _fixture(result: str = "ARRIVED") -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]]:
    ref = {"session_id": 11, "rl_step": 3, "source_sequence": 8,
           "monotonic_ns": 200, "sim_time_s": 2.0, "same_snapshot_persisted": True}
    context = {
        "run_id": "CONSTRUCTED",
        "session_id": 11,
        "capture_id": "capture-test",
        "settings": {"scene_id": "test_scene", "runtime_model_fingerprint": "model-test",
                     "arrival_threshold_m": 0.5, "diagnostic_sim_limit_after_motion_s": 20.0},
        "result_contract": {"classification": result},
        "lifecycle": {
            "task_start": {"session_id": 11, "rl_step": 0, "sim_time_s": 1.0},
            "arrival_observed": {"distance_m": 0.2, "threshold_m": 0.5,
                                  "session_id": 11, "rl_step": 3, "sim_time_s": 2.0},
            "terminal_frame_ref": ref,
            "standing_observation": {"status": "not part of motion window"},
        },
    }
    terminal = {"run_id": "CONSTRUCTED", "capture_id": "capture-test",
                "run_terminal_result": result, "run_terminal_frame_ref": dict(ref),
                "normal_shutdown": True, "shutdown_complete": True, "process_exit_code": 0}
    summary = {
        "run_id": "CONSTRUCTED", "scene_id": "test_scene", "record_validity": "VALID",
        "record_validity_reasons": [], "terminal_result": result, "command_consumption": "UNKNOWN",
        "rl_task_metric_boundary": {"definition": "cycle0 through terminal inclusive",
                                     "start_cycle": 0, "end_cycle": 3,
                                     "cycle_identity_complete": True,
                                     "terminal_frame_payload_complete": True},
        "task_window_safety": {"nonfoot_collision_failure": False, "fall": False,
                               "coverage_complete_for_motion_interval": True,
                               "fall_result_basis": "complete history",
                               "foot_contact_count": 0, "foot_impact_count": 0,
                               "foot_contact_events": [], "foot_impact_events": [],
                               "foot_impact_unknown_overlaps_motion": False},
        "declared_boundary_physics_observation": {
            "status": "BASE_LINK_IN_BOUNDS_PHYSICS_STEPS_OBSERVED",
            "scope": "base_link XY at every covered PhysicsLoop step"},
    }
    identity = {"status": "VALID", "reasons": [], "terminal_frame_ref_match": True}
    return summary, context, terminal, identity


def constructed_cases() -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []

    # These event arrays deliberately sit outside the already-produced task-window
    # decision. They exercise only the thin composite adapter, not event detection.
    summary, context, terminal, identity = _fixture()
    summary["capture_fall_events"] = [{"confirmed": True, "closed": True,
                                       "confirmed_sim_time": 0.5, "end_sim_time": 0.8}]
    context["lifecycle"]["task_start"]["sim_time_s"] = 1.0
    out.append({"case_id": "C1", "expected": True, "summary": summary,
                "context": context, "terminal": terminal, "identity": identity,
                "note": "pre-task fall was closed before task_start; task-window fall remains false"})

    summary, context, terminal, identity = _fixture()
    terminal["post_terminal_fall_events"] = [{"confirmed": True, "confirmed_sim_time": 2.2}]
    out.append({"case_id": "C2", "expected": True, "summary": summary,
                "context": context, "terminal": terminal, "identity": identity,
                "note": "post-terminal fall event is not injected into task-window safety"})

    summary, context, terminal, identity = _fixture()
    summary["task_window_safety"]["nonfoot_collision_failure"] = True
    summary["task_window_safety"]["collision"] = True
    out.append({"case_id": "C3", "expected": False, "summary": summary,
                "context": context, "terminal": terminal, "identity": identity,
                "note": "ARRIVED plus confirmed task-window nonfoot collision is false and contract-conflicting"})

    summary, context, terminal, identity = _fixture()
    summary["declared_boundary_physics_observation"] = {"status": "UNKNOWN",
                                                         "reason": "coverage_gap"}
    out.append({"case_id": "C4", "expected": "UNKNOWN", "summary": summary,
                "context": context, "terminal": terminal, "identity": identity,
                "note": "bounds coverage gap prevents confirmed safe completion"})

    summary, context, terminal, identity = _fixture()
    terminal["run_terminal_frame_ref"]["rl_step"] = 4
    identity = assess_task_identity(summary, context, terminal, task_frames=[
        {"session_id": 11, "collision_snapshot": {"capture_id": "capture-test",
         "scenario_id": "test_scene", "runtime_model_fingerprint": "model-test"}}
    ])
    out.append({"case_id": "C5", "expected": "UNKNOWN", "summary": summary,
                "context": context, "terminal": terminal, "identity": identity,
                "note": "terminal reference cycle mismatch is an identity conflict"})

    summary, context, terminal, identity = _fixture()
    summary["record_validity"] = "INVALID"
    summary["record_validity_reasons"] = ["constructed_invalid_record"]
    out.append({"case_id": "C6", "expected": "UNKNOWN", "summary": summary,
                "context": context, "terminal": terminal, "identity": identity,
                "note": "invalid record is not converted to safe success or safety failure"})

    summary, context, terminal, identity = _fixture()
    summary["task_window_safety"]["fall"] = "UNKNOWN"
    summary["task_window_safety"]["fall_result_basis"] = "constructed incomplete fall history"
    out.append({"case_id": "C7", "expected": "UNKNOWN", "summary": summary,
                "context": context, "terminal": terminal, "identity": identity,
                "note": "ARRIVED with fall UNKNOWN remains unknown despite other clear dimensions"})

    summary, context, terminal, identity = _fixture()
    summary["task_window_safety"]["fall"] = True
    summary["task_window_safety"]["fall_events"] = [{"confirmed": True, "confirmed_sim_time": 1.5}]
    out.append({"case_id": "C8", "expected": False, "summary": summary,
                "context": context, "terminal": terminal, "identity": identity,
                "note": "ARRIVED plus confirmed task-window fall is false and contract-conflicting"})

    summary, context, terminal, identity = _fixture()
    terminal["external_safety_events"] = [{"event": "POLICY_SAFETY_VETO",
                                           "controller_session_id": 11,
                                           "controller_policy_cycle": 2}]
    out.append({"case_id": "C9", "expected": False, "summary": summary,
                "context": context, "terminal": terminal, "identity": identity,
                "note": "ARRIVED with a same-session safety veto cycle inside the task window is false and conflicting"})

    summary, context, terminal, identity = _fixture()
    terminal["external_safety_events"] = [{"event": "POLICY_SAFETY_VETO",
                                           "controller_session_id": 11,
                                           "controller_policy_cycle": 4}]
    out.append({"case_id": "C10", "expected": True, "summary": summary,
                "context": context, "terminal": terminal, "identity": identity,
                "note": "same-session safety veto after the terminal cycle does not contaminate task-window classification"})

    summary, context, terminal, identity = _fixture("SYSTEM_SAFETY_ABORT")
    summary["task_window_safety"]["nonfoot_collision_failure"] = "UNKNOWN"
    summary["task_window_safety"]["fall"] = "UNKNOWN"
    summary["task_window_safety"]["coverage_complete_for_motion_interval"] = False
    out.append({"case_id": "C11", "expected": False, "summary": summary,
                "context": context, "terminal": terminal, "identity": identity,
                "note": "started task with system safety abort is not complete even if physical details stay unknown"})

    return out


def production_post_terminal_event_check() -> dict[str, Any]:
    """Exercise production motion_safety with a later, same-run producer snapshot."""
    relative, _expected = REAL_CASES["R1"]
    run_dir = ROOT / relative
    context = json.loads((run_dir / "run_context.json").read_text(encoding="utf-8"))
    record = load_record(str(run_dir / "runtime_record.jsonl"))
    terminal_ref = context["lifecycle"]["terminal_frame_ref"]
    start_s = context["lifecycle"]["task_start"]["sim_time_s"]
    end_s = context["lifecycle"]["arrival_observed"]["sim_time_s"]
    frames = [line["payload"] for line in record.frames
              if line.get("status") == "LIVE" and isinstance(line.get("payload"), dict)]
    task_frames = [frame for frame in frames
                   if frame.get("session_id") == terminal_ref["session_id"] and
                   frame.get("rl_entered") == 1 and frame.get("rl_active") == 1 and
                   isinstance(frame.get("rl_step"), int) and
                   frame["rl_step"] <= terminal_ref["rl_step"]]
    if not task_frames or task_frames[-1].get("rl_step") != terminal_ref["rl_step"]:
        raise AssertionError("R1 task-frame boundary unavailable for post-terminal probe")
    task_start_snapshot = task_frames[0].get("collision_snapshot")
    task_terminal_snapshot = task_frames[-1].get("collision_snapshot")
    later = dict(task_terminal_snapshot)
    later["sequence"] = task_terminal_snapshot["sequence"] + 2
    later["physics_step"] = task_terminal_snapshot["physics_step"] + 1
    later["sim_time"] = end_s + 0.02
    later["monotonic_ns"] = task_terminal_snapshot["monotonic_ns"] + 1_000_000
    post_event = {"start_physics_step": later["physics_step"],
                  "end_physics_step": later["physics_step"],
                  "confirmed_physics_step": later["physics_step"],
                  "start_sim_time": end_s + 0.01,
                  "end_sim_time": end_s + 0.01,
                  "confirmed_sim_time": end_s + 0.01,
                  "closed": True, "confirmed": True}
    later["fall_history"] = list(later.get("fall_history") or []) + [post_event]
    later["fall_history_count"] = len(later["fall_history"])
    pre_task_snapshot = context["result_contract"]["task_window_safety_boundary"]["snapshot"]
    result = motion_safety(task_frames, start_s, end_s, later, [pre_task_snapshot],
                           task_start_snapshot, task_terminal_snapshot)
    return {"case_id": "P1", "source_run_id": context["run_id"],
            "expected_fall": False, "actual_fall": result.get("fall"),
            "coverage_complete": result.get("coverage_complete_for_motion_interval"),
            "task_end_sim_time_s": end_s,
            "constructed_post_terminal_confirmation_sim_time_s": post_event["confirmed_sim_time"],
            "event_in_task_window": bool(result.get("fall_events")),
            "result_basis": result.get("fall_result_basis"),
            "scope": "production motion_safety() on a copy of the existing R1 frames plus one synthetic later publisher snapshot"}


def production_coverage_checks() -> list[dict[str, Any]]:
    """Exercise contact, invalid-posture, overflow, and publisher-gap inputs."""
    relative, _expected = REAL_CASES["R1"]
    run_dir = ROOT / relative
    context = json.loads((run_dir / "run_context.json").read_text(encoding="utf-8"))
    record = load_record(str(run_dir / "runtime_record.jsonl"))
    ref = context["lifecycle"]["terminal_frame_ref"]
    start_s = context["lifecycle"]["task_start"]["sim_time_s"]
    end_s = context["lifecycle"]["arrival_observed"]["sim_time_s"]
    payloads = [line["payload"] for line in record.frames
                if line.get("status") == "LIVE" and isinstance(line.get("payload"), dict)]
    task_frames = [frame for frame in payloads
                   if frame.get("session_id") == ref["session_id"] and
                   frame.get("rl_entered") == 1 and frame.get("rl_active") == 1 and
                   isinstance(frame.get("rl_step"), int) and frame["rl_step"] <= ref["rl_step"]]
    start_snapshot = task_frames[0]["collision_snapshot"]
    end_snapshot = task_frames[-1]["collision_snapshot"]
    pre_snapshot = context["result_contract"]["task_window_safety_boundary"]["snapshot"]

    def snapshot_variant() -> dict[str, Any]:
        later = copy.deepcopy(end_snapshot)
        later["sequence"] = end_snapshot["sequence"] + 2
        later["physics_step"] = end_snapshot["physics_step"] + 1
        later["sim_time"] = end_s + 0.02
        later["monotonic_ns"] = end_snapshot["monotonic_ns"] + 1_000_000
        return later

    variants: list[tuple[str, str, Any, Any, Any]] = []
    later = snapshot_variant()
    t = start_s + (end_s - start_s) / 2
    later.update({"unknown_contact_steps": 1, "first_unknown_contact_step": later["physics_step"],
                  "last_unknown_contact_step": later["physics_step"],
                  "first_unknown_contact_sim_time": t, "last_unknown_contact_sim_time": t})
    variants.append(("P2_unknown_contact", "UNKNOWN", False, later, True))

    later = snapshot_variant()
    later.update({"invalid_posture_steps": 1, "first_invalid_posture_step": later["physics_step"],
                  "last_invalid_posture_step": later["physics_step"],
                  "first_invalid_posture_sim_time": t, "last_invalid_posture_sim_time": t})
    variants.append(("P3_invalid_posture", False, "UNKNOWN", later, True))

    later = snapshot_variant()
    later["fall_history_overflow"] = True
    variants.append(("P4_fall_history_overflow", "UNKNOWN", "UNKNOWN", later, False))

    later = snapshot_variant()
    later["physics_coverage_complete"] = False
    variants.append(("P5_physics_coverage_gap", "UNKNOWN", "UNKNOWN", later, False))

    results = []
    for case_id, expected_collision, expected_fall, later_snapshot, expected_complete in variants:
        result = motion_safety(task_frames, start_s, end_s, later_snapshot, [pre_snapshot],
                               start_snapshot, end_snapshot)
        actual = (result.get("collision"), result.get("fall"),
                  result.get("coverage_complete_for_motion_interval"))
        expected = (expected_collision, expected_fall, expected_complete)
        results.append({"case_id": case_id, "expected": expected, "actual": actual,
                        "pass": actual == expected,
                        "unclassified_contact_overlaps_motion": result.get("unclassified_contact_overlaps_motion"),
                        "invalid_posture_overlaps_motion": result.get("invalid_posture_overlaps_motion"),
                        "coverage_reason": result.get("coverage_reason"),
                        "scope": "production motion_safety() over copied R1 task frames and a synthetic later snapshot"})
    return results


def unknown_sensitivity_examples() -> list[dict[str, Any]]:
    """Check candidate-minus-reference bounds for high- and low-is-better rates."""
    def interval(positive: int, negative: int, unknown: int) -> tuple[float, float]:
        total = positive + negative + unknown
        if total <= 0:
            raise ValueError("empty planned denominator")
        return positive / total, (positive + unknown) / total

    def row(case: str, metric: str, direction: str,
            candidate: tuple[int, int, int], reference: tuple[int, int, int]) -> dict[str, Any]:
        c = interval(*candidate)
        r = interval(*reference)
        lower, upper = c[0] - r[1], c[1] - r[0]
        worst, best = (lower, upper) if direction == "higher_is_better" else (upper, lower)
        return {"case_id": case, "metric": metric, "direction": direction,
                "candidate_counts_positive_negative_unknown": candidate,
                "reference_counts_positive_negative_unknown": reference,
                "candidate_interval": c, "reference_interval": r,
                "candidate_minus_reference_math_interval": (lower, upper),
                "most_unfavorable": worst, "most_favorable": best,
                "interpretation": "missing-outcome sensitivity range, not a bootstrap confidence interval"}

    return [
        row("U1", "safety_completion", "higher_is_better", (2, 1, 1), (1, 2, 1)),
        row("U2", "safety_completion", "higher_is_better", (2, 2, 0), (1, 1, 2)),
        row("U3", "safety_completion", "higher_is_better", (2, 1, 1), (2, 2, 0)),
        row("U4", "safety_completion", "higher_is_better", (3, 1, 0), (2, 2, 0)),
        row("U5", "nonfoot_collision", "lower_is_better", (1, 2, 1), (1, 1, 2)),
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    results: list[dict[str, Any]] = []
    failures: list[str] = []
    for case_id, (relative, expected) in REAL_CASES.items():
        run_dir = ROOT / relative
        summary = calculate(run_dir / "runtime_record.jsonl", run_dir / "run_context.json")
        stored_metrics_path = run_dir / "metrics.json"
        stored_metrics = json.loads(stored_metrics_path.read_text(encoding="utf-8")) if stored_metrics_path.exists() else {}
        actual = summary.get("safety_task_completion", {}).get("status", "UNKNOWN")
        if actual != expected:
            failures.append(f"{case_id}: expected {expected}, got {actual}")
        results.append({"case_id": case_id, "run_id": summary.get("run_id"),
                        "source": relative, "expected": expected, "actual": actual,
                        "record_validity": summary.get("record_validity"),
                        "terminal_result": summary.get("terminal_result"),
                        "stored_metrics_fall": (stored_metrics.get("task_window_safety", {}).get("fall")
                                                 if isinstance(stored_metrics.get("task_window_safety"), dict) else None),
                        "current_production_fall": summary.get("task_window_safety", {}).get("fall"),
                        "raw_terminal": summary.get("safety_task_completion", {}).get("raw_terminal"),
                        "task_source_identity": summary.get("task_source_identity"),
                        "task_safety_dimensions": summary.get("safety_task_completion", {}).get("task_safety_dimensions"),
                        "reasons": summary.get("safety_task_completion", {}).get("reasons"),
                        "standing_and_cleanup": summary.get("safety_task_completion", {}).get("standing_and_cleanup")})

    constructed_results = []
    for case in constructed_cases():
        actual = classify_safety_task_completion(case["summary"], case["context"],
                                                 case["terminal"], case["identity"])["status"]
        repeated = classify_safety_task_completion(case["summary"], case["context"],
                                                   case["terminal"], case["identity"])["status"]
        if actual != case["expected"] or repeated != actual:
            failures.append(f"{case['case_id']}: expected {case['expected']}, got {actual}, repeated {repeated}")
        constructed_results.append({"case_id": case["case_id"], "expected": case["expected"],
                                    "actual": actual, "repeat_identical": repeated == actual,
                                    "note": case["note"],
                                    "identity_status": case["identity"].get("status"),
                                    "reasons": classify_safety_task_completion(
                                        case["summary"], case["context"], case["terminal"],
                                        case["identity"])["reasons"]})

    post_terminal_check = production_post_terminal_event_check()
    if post_terminal_check["actual_fall"] != post_terminal_check["expected_fall"]:
        failures.append(f"{post_terminal_check['case_id']}: expected fall={post_terminal_check['expected_fall']}, got {post_terminal_check['actual_fall']}")
    coverage_checks = production_coverage_checks()
    failures.extend(f"{item['case_id']}: production safety result mismatch" for item in coverage_checks
                    if not item["pass"])
    sensitivity = unknown_sensitivity_examples()
    for item in sensitivity:
        lower, upper = item["candidate_minus_reference_math_interval"]
        if lower > upper or item["most_unfavorable"] not in (lower, upper):
            failures.append(f"{item['case_id']}: invalid UNKNOWN sensitivity bounds")

    result = {"status": "PASS" if not failures else "FAIL",
              "scope": "targeted offline production calculate() reads plus explicit adapter fixtures; no source records modified",
              "real_cases": results, "constructed_cases": constructed_results,
              "production_post_terminal_event_check": post_terminal_check,
              "production_coverage_checks": coverage_checks,
              "unknown_sensitivity_examples": sensitivity,
              "case_counts_are_not_performance_estimates": True, "failures": failures}
    (args.output / "classification_check_results.json").write_text(
        json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    with (args.output / "classification_check.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["case_id", "run_id", "kind", "expected", "actual",
                                                   "record_validity", "terminal_result", "reasons"])
        writer.writeheader()
        for row in results:
            writer.writerow({"case_id": row["case_id"], "run_id": row["run_id"],
                             "kind": "existing run", "expected": row["expected"],
                             "actual": row["actual"], "record_validity": row["record_validity"],
                             "terminal_result": row["terminal_result"],
                             "reasons": ";".join(row["reasons"] or [])})
        for row in constructed_results:
            writer.writerow({"case_id": row["case_id"], "run_id": "CONSTRUCTED_" + row["case_id"],
                             "kind": "constructed adapter input", "expected": row["expected"],
                             "actual": row["actual"], "record_validity": "synthetic",
                             "terminal_result": "synthetic", "reasons": ";".join(row["reasons"])})
    print(f"status={result['status']} real={len(results)} constructed={len(constructed_results)} output={args.output}")
    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
