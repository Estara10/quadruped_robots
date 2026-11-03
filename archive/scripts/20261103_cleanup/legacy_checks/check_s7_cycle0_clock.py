#!/usr/bin/env python3
"""Targeted offline review of the A-run cycle0 sim-clock gap.

Reads the existing A/D records and the controller-authored S6-04 trace. It does
not start MuJoCo/ROS, does not write raw records, and does not enable the
decision-clock override in the production analysis path.
"""
from __future__ import annotations

import hashlib
import json
import math
import shutil
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from s1_analyze_run import (  # noqa: E402
    SIM_CLOCK_PUBLICATION_FRESHNESS_NS, calculate, resolve_decision_start_clock,
)
from safety_task_completion import classify_safety_task_completion  # noqa: E402
import s7_dev_supplement as dev  # noqa: E402
import s7_formal_analysis as analysis  # noqa: E402

OUT = ROOT / "logs/07_formal_preparation/cycle0_clock_review"
A = (ROOT / "logs/07_formal_preparation/dev_supplement_m02_AD/dev-m02-ad-20261021/"
          "runs/DEV-M02-A/attempt_01/result/798d518a8fb84d419cb00987ed6a8538")
D = (ROOT / "logs/07_formal_preparation/dev_supplement_m02_AD/dev-m02-ad-20261021/"
          "runs/DEV-M02-D/attempt_01/result/5859b72dc8994e8a9e9d6ad0e0cd8e2d")
RESULTS: list[dict[str, Any]] = []


def record(name: str, passed: bool, detail: Any) -> None:
    RESULTS.append({"check": name, "status": "PASS" if passed else "FAIL",
                    "detail": detail})
    print(f"[{'PASS' if passed else 'FAIL'}] {name}: {detail}", flush=True)


def sha(path: Path) -> str | None:
    if not path.is_file():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def summary_of(run: Path, override: bool) -> dict:
    return calculate(run / "runtime_record.jsonl", run / "run_context.json",
                     decision_clock_override=override)


def check_a_decision_vs_publication() -> dict:
    context = load(A / "run_context.json")
    record_file = A / "runtime_record.jsonl"
    frames = []
    with record_file.open(encoding="utf-8") as handle:
        for line in handle:
            item = json.loads(line)
            if item.get("kind") == "frame" and isinstance(item.get("payload"), dict):
                frames.append(item["payload"])
    decision = resolve_decision_start_clock(context, A / "run_context.json", frames)
    first = next(frame for frame in frames
                 if frame.get("session_id") == decision.get("session_id")
                 and frame.get("rl_step") == decision.get("rl_step"))
    passed = (
        decision.get("status") == "VALID_AT_DECISION"
        and decision.get("sim_time_s") == 7.126
        and decision.get("frame_sim_clock_valid") is False
        and decision.get("frame_sim_clock_status") == 2
        and decision.get("frame_sim_clock_age_ns") == 108_484_356
        and first.get("sim_clock_age_ns") >= SIM_CLOCK_PUBLICATION_FRESHNESS_NS
        and abs(float(first.get("sim_time_s")) - float(decision.get("sim_time_s"))) <= 1e-9
    )
    record("01_A_decision_valid_publication_stale", passed, {
        "decision": decision, "first_frame": {
            key: first.get(key) for key in (
                "sim_time_s", "sim_clock_valid", "sim_clock_status",
                "sim_clock_age_ns", "sim_clock_monotonic_ns", "monotonic_ns",
                "source_sequence")
        }})
    return decision


def check_a_default_vs_override(decision: dict) -> tuple[dict, dict]:
    before = {name: sha(A / name) for name in (
        "runtime_record.jsonl", "run_context.json", "terminal.json", "metrics.json")}
    default = summary_of(A, False)
    override = summary_of(A, True)
    after = {name: sha(A / name) for name in (
        "runtime_record.jsonl", "run_context.json", "terminal.json", "metrics.json")}
    default_ok = (
        default.get("safety_task_completion_status") == "UNKNOWN"
        and default.get("task_start_sim_time_s") is None
        and default.get("elapsed_task_duration_s") is None
        and default.get("timing_clock_override_applied") is False
        and default.get("raw_invalid_clock_frames") == 1
        and default.get("invalid_clock_frames") == 1
        and default.get("short_reverse_switch_coverage") == "partial_or_unknown"
        and "arrival_or_task_start_sim_time_missing" in default.get(
            "safety_task_completion", {}).get("reasons", [])
    )
    override_ok = (
        override.get("timing_clock_override_applied") is True
        and override.get("task_start_sim_time_s") == 7.126
        and abs(float(override.get("elapsed_task_duration_s") or 0) - 6.502) <= 1e-9
        and override.get("valid_motion_duration_s") == override.get("elapsed_task_duration_s")
        and override.get("raw_invalid_clock_frames") == 1
        and override.get("invalid_clock_frames") == 0
        and override.get("short_reverse_switch_coverage") == "complete"
        and override.get("safety_task_completion_status") is True
        and override.get("safety_task_completion", {}).get(
            "task_safety_dimensions", {}).get("foot_events", {}).get("impact_count") == 1
        and override.get("rl_task_metric_boundary", {}).get("start_cycle") == 0
        and override.get("task_start_clock", {}).get("rl_step") == 0
    )
    record("02_A_default_unchanged", default_ok, {
        "safe": default.get("safety_task_completion_status"),
        "task_start": default.get("task_start_sim_time_s"),
        "elapsed": default.get("elapsed_task_duration_s"),
        "raw_invalid": default.get("raw_invalid_clock_frames"),
        "timing_invalid": default.get("invalid_clock_frames"),
        "short_reverse": default.get("short_reverse_switch_coverage"),
    })
    record("03_A_override_restores_timing_but_not_raw", override_ok, {
        "safe": override.get("safety_task_completion_status"),
        "task_start": override.get("task_start_sim_time_s"),
        "elapsed": override.get("elapsed_task_duration_s"),
        "raw_invalid": override.get("raw_invalid_clock_frames"),
        "timing_invalid": override.get("invalid_clock_frames"),
        "short_reverse": override.get("short_reverse_switch_coverage"),
        "start_cycle": override.get("rl_task_metric_boundary", {}).get("start_cycle"),
        "foot_impact_count": override.get("safety_task_completion", {}).get(
            "task_safety_dimensions", {}).get("foot_events", {}).get("impact_count"),
        "raw_hashes_unchanged": before == after,
    })
    record("04_A_raw_files_unchanged", before == after, {"before": before, "after": after})
    record("05_A_trace_and_sample_identity", decision.get("status") == "VALID_AT_DECISION"
           and decision.get("session_id") == context_session(A)
           and decision.get("rl_step") == 0, decision)
    return default, override


def context_session(run: Path) -> int | None:
    context = load(run / "run_context.json")
    start = context.get("lifecycle", {}).get("task_start", {})
    return start.get("session_id")


def check_d_control() -> None:
    default = summary_of(D, False)
    override = summary_of(D, True)
    decision = default.get("task_start_clock", {})
    passed = (
        decision.get("status") == "FRAME_VALID"
        and default.get("safety_task_completion_status") is True
        and override.get("safety_task_completion_status") is True
        and override.get("timing_clock_override_applied") is False
        and abs(float(default.get("elapsed_task_duration_s") or 0) - 6.6) <= 1e-9
        and abs(float(override.get("elapsed_task_duration_s") or 0) - 6.6) <= 1e-9
    )
    record("06_D_valid_frame_no_override", passed, {
        "task_start_clock": decision,
        "default_safe": default.get("safety_task_completion_status"),
        "override_safe": override.get("safety_task_completion_status"),
        "default_elapsed": default.get("elapsed_task_duration_s"),
        "override_elapsed": override.get("elapsed_task_duration_s"),
    })


def make_case(name: str, *, session: int = 1, cycle: int = 0,
              frame_sim: float | None = 5.0, frame_valid: bool = False,
              frame_status: int = 2, frame_age: int | None = 108_000_000,
              trace_rows: list[dict] | None = None,
              trace_text: str | None = None) -> Path:
    directory = OUT / "constructed" / name
    if directory.exists():
        shutil.rmtree(directory)
    directory.mkdir(parents=True)
    context = {
        "lifecycle": {"task_start": {
            "session_id": session, "rl_step": cycle, "sim_clock_valid": frame_valid,
            "sim_time_s": frame_sim if frame_valid else None,
        }},
    }
    (directory / "run_context.json").write_text(json.dumps(context) + "\n", encoding="utf-8")
    if trace_text is None:
        rows = trace_rows or [{
            "schema": "s6-04-controller-startup-v1", "session_id": session,
            "policy_cycle": cycle, "controller_update_steady_ns": 1000,
            "controller_update_gain_step": 3, "sim_clock_valid": True,
            "sim_time_s": 5.0,
        }]
        trace_text = "".join("[S6-04-TRACE] " + json.dumps(row) + "\n" for row in rows)
    (directory / "ros_launch.log").write_text(trace_text, encoding="utf-8")
    return directory


def resolve_case(directory: Path, *, frame_sim: float | None = 5.0,
                 frame_valid: bool = False, frame_status: int = 2,
                 frame_age: int | None = 108_000_000,
                 session: int = 1, cycle: int = 0) -> dict:
    context = load(directory / "run_context.json")
    frame = {"session_id": session, "rl_step": cycle,
             "sim_clock_valid": frame_valid,
             "sim_clock_status": frame_status,
             "sim_clock_age_ns": frame_age,
             "sim_time_s": frame_sim,
             "monotonic_ns": 200_000_000}
    return resolve_decision_start_clock(context, directory / "run_context.json", [frame])


def check_constructed_rejections() -> None:
    valid = resolve_case(make_case("valid"))
    no_trace_dir = make_case("no_trace", trace_text="")
    (no_trace_dir / "ros_launch.log").unlink()
    no_trace = resolve_case(no_trace_dir)
    empty_trace = resolve_case(make_case("empty_trace", trace_text=""))
    wrong_session = resolve_case(make_case("wrong_session", trace_rows=[{
        "schema": "s6-04-controller-startup-v1", "session_id": 2,
        "policy_cycle": 0, "sim_clock_valid": True, "sim_time_s": 5.0,
        "controller_update_steady_ns": 1000}]))
    wrong_cycle = resolve_case(make_case("wrong_cycle", trace_rows=[{
        "schema": "s6-04-controller-startup-v1", "session_id": 1,
        "policy_cycle": 1, "sim_clock_valid": True, "sim_time_s": 5.0,
        "controller_update_steady_ns": 1000}]))
    sample_mismatch = resolve_case(make_case("sample_mismatch", trace_rows=[{
        "schema": "s6-04-controller-startup-v1", "session_id": 1,
        "policy_cycle": 0, "sim_clock_valid": True, "sim_time_s": 5.1,
        "controller_update_steady_ns": 1000}]))
    invalid_at_capture = resolve_case(make_case("invalid_at_capture", trace_rows=[{
        "schema": "s6-04-controller-startup-v1", "session_id": 1,
        "policy_cycle": 0, "sim_clock_valid": False, "sim_time_s": None,
        "controller_update_steady_ns": 1000}]))
    reset_frame = resolve_case(make_case("reset_frame"), frame_status=3)
    no_age = resolve_case(make_case("no_age"), frame_age=0)
    no_sim_time = resolve_case(make_case("no_sim_time"), frame_sim=None)
    frame_valid = resolve_case(make_case("frame_valid"), frame_valid=True)
    missing_start_dir = make_case("missing_start")
    context = {"lifecycle": {}}
    (missing_start_dir / "run_context.json").write_text(json.dumps(context) + "\n",
                                                        encoding="utf-8")
    missing_start = resolve_decision_start_clock(
        context, missing_start_dir / "run_context.json", [])
    passed = (
        valid["status"] == "VALID_AT_DECISION"
        and no_trace["status"] == "CONTROLLER_TRACE_LOG_MISSING"
        and empty_trace["status"] == "CONTROLLER_TRACE_CYCLE_MISSING"
        and wrong_session["status"] == "CONTROLLER_TRACE_CYCLE_MISSING"
        and wrong_cycle["status"] == "CONTROLLER_TRACE_CYCLE_MISSING"
        and sample_mismatch["status"] == "DECISION_SAMPLE_MISMATCH"
        and invalid_at_capture["status"] == "DECISION_CLOCK_INVALID_AT_CAPTURE"
        and reset_frame["status"] == "FRAME_NOT_PUBLICATION_STALE"
        and no_age["status"] == "FRAME_STALE_WITHOUT_PUBLICATION_AGE"
        and no_sim_time["status"] == "FRAME_SIM_TIME_MISSING"
        and frame_valid["status"] == "FRAME_VALID"
        and missing_start["status"] == "MISSING_TASK_START"
    )
    record("07_constructed_rejections", passed, {
        "valid": valid["status"], "no_trace": no_trace["status"],
        "empty_trace": empty_trace["status"],
        "wrong_session": wrong_session["status"], "wrong_cycle": wrong_cycle["status"],
        "sample_mismatch": sample_mismatch["status"],
        "invalid_at_capture": invalid_at_capture["status"],
        "reset_frame": reset_frame["status"], "no_age": no_age["status"],
        "no_sim_time": no_sim_time["status"], "frame_valid": frame_valid["status"],
        "missing_start": missing_start["status"],
    })


def check_group_shared_path() -> None:
    outcomes = {}
    for group in "ABCD":
        directory = make_case(f"group_{group}")
        context = load(directory / "run_context.json")
        context["settings"] = {"group": group}
        (directory / "run_context.json").write_text(json.dumps(context) + "\n",
                                                    encoding="utf-8")
        outcomes[group] = resolve_case(directory)["status"]
    passed = all(value == "VALID_AT_DECISION" for value in outcomes.values())
    record("08_ABCD_shared_decision_path", passed, outcomes)


def check_no_false_safety_clearing(default: dict) -> None:
    context = load(A / "run_context.json")
    terminal = load(A / "terminal.json")
    identity = default["task_source_identity"]
    summary = json.loads(json.dumps(default))
    summary["task_window_safety"]["fall"] = "UNKNOWN"
    summary["task_window_safety"]["coverage_complete_for_motion_interval"] = False
    summary["task_window_safety"]["coverage_reason"] = "fall_history_source_incomplete"
    summary["safety_task_completion"] = {}
    with_override = classify_safety_task_completion(
        summary, context, terminal, identity,
        start_clock_override=default["task_start_clock"])
    summary_fall = json.loads(json.dumps(default))
    summary_fall["task_window_safety"]["fall"] = True
    summary_fall["task_window_safety"]["coverage_complete_for_motion_interval"] = True
    summary_fall["safety_task_completion"] = {}
    fall_override = classify_safety_task_completion(
        summary_fall, context, terminal, identity,
        start_clock_override=default["task_start_clock"])
    passed = (with_override["status"] == "UNKNOWN"
              and fall_override["status"] is False
              and "fall" in str(with_override.get("reasons")))
    record("09_decision_clock_does_not_clear_unknown_or_fall", passed, {
        "unknown_fall": {"status": with_override["status"],
                         "reasons": with_override.get("reasons")},
        "confirmed_fall": {"status": fall_override["status"],
                           "reasons": fall_override.get("reasons")},
    })


def build_timeline() -> dict:
    timeline = {}
    for label, run in (("A", A), ("D", D)):
        context = load(run / "run_context.json")
        terminal = load(run / "terminal.json")
        metrics = load(run / "metrics.json")
        lifecycle = context.get("lifecycle", {})
        frames = []
        with (run / "runtime_record.jsonl").open(encoding="utf-8") as handle:
            for line in handle:
                item = json.loads(line)
                if item.get("kind") == "frame" and isinstance(item.get("payload"), dict):
                    frames.append(item["payload"])
        first_active = next((frame for frame in frames
                             if frame.get("rl_entered") == 1 and frame.get("rl_active") == 1),
                            None)
        timeline[label] = {
            "run_id": context.get("run_id"),
            "group": context.get("settings", {}).get("group"),
            "mujoco_process_started_monotonic_ns": lifecycle.get(
                "mujoco_process_started_monotonic_ns"),
            "mujoco_authority_live_monotonic_ns": lifecycle.get(
                "mujoco_authority_live_monotonic_ns"),
            "standing_admission_monotonic_ns": lifecycle.get(
                "standing_admission_monotonic_ns"),
            "rl_entry_request_monotonic_ns": lifecycle.get("rl_entry_request_monotonic_ns"),
            "controller_state_rl_entry": lifecycle.get("first_policy_cycle_monotonic_ns"),
            "first_active_frame": {
                key: (first_active or {}).get(key) for key in (
                    "session_id", "rl_step", "sim_time_s", "sim_clock_valid",
                    "sim_clock_status", "sim_clock_age_ns", "sim_clock_monotonic_ns",
                    "monotonic_ns", "source_sequence")
            },
            "task_start": lifecycle.get("task_start"),
            "terminal": terminal.get("run_terminal_result"),
            "process_exit_code": terminal.get("process_exit_code"),
            "shutdown_complete": terminal.get("shutdown_complete"),
            "safety_task_completion": metrics.get("safety_task_completion_status"),
        }
    return timeline


def check_formal_unified_path() -> None:
    records, meta, errors = dev.load_dev_plan()
    assert not errors, errors
    ledger = load(dev.OUTPUT_ROOT / "dev-m02-ad-20261021" / "ledger" / "attempts.json")
    audit = dev.runner.audit_ledger(records, ledger["attempts"],
                                    max_substitutions=0, max_per_plan_item=1)
    summaries, provenance = analysis.recompute_scene_start_summaries(
        records, audit, output_dir=OUT / "formal_unified_recompute",
        decision_clock_override=True)
    views = analysis.build_views(records, audit, summaries)
    outcomes = {row["plan_item_id"]: row["outcome"]
                for row in views["primary_plan_item"]["rows"]}
    sensitivity = analysis.reconstruction_sensitivity(
        records, audit, summaries, output_dir=OUT / "formal_unified_recompute")
    passed = (
        provenance.get("decision_clock_reconstruction", {}).get("enabled") is True
        and outcomes.get("DEV-M02-A") == "CONFIRMED_SAFE_COMPLETE"
        and outcomes.get("DEV-M02-D") == "CONFIRMED_SAFE_COMPLETE"
        and sensitivity.get("reconstruction_runs") == 1
        and sensitivity.get("safety_changed_runs") == 1
        and sensitivity.get("timing_changed_runs") == 1
        and sensitivity.get("unresolved_runs") == 0
    )
    record("10_formal_analysis_uses_unified_reconstruction", passed, {
        "provenance": provenance.get("decision_clock_reconstruction"),
        "outcomes": outcomes,
        "sensitivity": {key: sensitivity.get(key) for key in (
            "reconstruction_runs", "safety_changed_runs", "timing_changed_runs",
            "unresolved_runs")},
    })


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    decision = check_a_decision_vs_publication()
    default, override = check_a_default_vs_override(decision)
    check_d_control()
    check_constructed_rejections()
    check_group_shared_path()
    check_no_false_safety_clearing(default)
    check_formal_unified_path()
    timeline = build_timeline()
    payload = {
        "schema": "s7-cycle0-clock-review-v1",
        "scope": "offline read-only; no scene, ROS, model inference, or raw-record write",
        "A": {
            "run_id": "798d518a8fb84d419cb00987ed6a8538",
            "decision_start_clock": decision,
            "default_analysis": {
                "safety_task_completion": default.get("safety_task_completion_status"),
                "task_start_sim_time_s": default.get("task_start_sim_time_s"),
                "elapsed_task_duration_s": default.get("elapsed_task_duration_s"),
                "raw_invalid_clock_frames": default.get("raw_invalid_clock_frames"),
                "short_reverse_switch_coverage": default.get(
                    "short_reverse_switch_coverage"),
                "reasons": default.get("safety_task_completion", {}).get("reasons"),
            },
            "override_analysis": {
                "safety_task_completion": override.get("safety_task_completion_status"),
                "task_start_sim_time_s": override.get("task_start_sim_time_s"),
                "elapsed_task_duration_s": override.get("elapsed_task_duration_s"),
                "raw_invalid_clock_frames": override.get("raw_invalid_clock_frames"),
                "timing_invalid_clock_frames": override.get("invalid_clock_frames"),
                "short_reverse_switch_coverage": override.get(
                    "short_reverse_switch_coverage"),
                "dimensions": {
                    key: (value.get("value") if isinstance(value, dict) else value)
                    for key, value in (override.get("safety_task_completion", {}).get(
                        "task_safety_dimensions", {}) or {}).items()
                    if key != "foot_events"
                },
                "foot_events": (override.get("safety_task_completion", {}).get(
                    "task_safety_dimensions", {}) or {}).get("foot_events"),
                "reasons": override.get("safety_task_completion", {}).get("reasons"),
            },
        },
        "timeline": timeline,
        "checks": RESULTS,
        "passed": sum(item["status"] == "PASS" for item in RESULTS),
        "failed": sum(item["status"] == "FAIL" for item in RESULTS),
        "total": len(RESULTS),
        "production_default_unchanged": True,
        "raw_records_written": False,
    }
    (OUT / "check_results.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"passed": payload["passed"], "failed": payload["failed"],
                      "total": payload["total"]}, ensure_ascii=False))
    return 0 if payload["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
