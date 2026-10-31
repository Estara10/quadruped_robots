#!/usr/bin/env python3
"""Targeted offline joint test for the S7 formal runner and analyzer.

No MuJoCo, ROS, model inference, controller, or real scene-start function is
invoked.  Fixture batches live under logs/07_formal_preparation/runner_readiness.
"""
from __future__ import annotations

import json
import subprocess
import sys
import uuid
from collections import Counter
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import s7_formal_analysis as analysis  # noqa: E402
import s7_formal_runner as runner  # noqa: E402
from formal_attempt_accounting import audit_attempt_ledger, classify_attempt  # noqa: E402

OFFLINE = ROOT / "logs/07_formal_preparation/runner_readiness"
OUT = OFFLINE / "offline_joint_test"
FIXTURES = OUT / "constructed"
RESULTS: list[dict[str, Any]] = []


def record(name: str, passed: bool, detail: Any) -> None:
    RESULTS.append({"check": name, "status": "PASS" if passed else "FAIL",
                    "detail": detail})
    print(f"[{'PASS' if passed else 'FAIL'}] {name}: {detail}", flush=True)


def fixture_meta(matrix_id: str = "S7-OFFLINE-FIXTURE") -> dict[str, Any]:
    return {
        "matrix_id": matrix_id,
        "status": runner.FROZEN_STATUS,
        "authorized_to_run": False,
        "schedule_seed": 2026102001,
        "bootstrap_seed": 2026102002,
        "map_version": "m1-mapset/1.0",
        "groups": {group: {"E": p["E"], "X": p["X"], "H_s": p["H"]}
                   for group, p in runner.GROUP_PARAMETERS.items()},
        "williams_sequences": runner.WILLIAMS,
        "field_bounds_m": {"x": [-0.5, 8.0], "y": [-3.0, 3.0]},
        "field_is_physical_wall": False,
    }


def fixture_plan(maps: int = 1, blocks_per_map: int = 1) -> tuple[list[dict], dict]:
    records: list[dict] = []
    order = 1
    block_order = 0
    sequences = ["W0", "W2", "W1", "W3"]
    for map_index in range(1, maps + 1):
        for block in range(1, blocks_per_map + 1):
            block_order += 1
            sequence_name = sequences[(map_index + block - 2) % len(sequences)]
            for position, group in enumerate(runner.WILLIAMS[sequence_name], start=1):
                params = runner.GROUP_PARAMETERS[group]
                records.append(runner.normalize_record({
                    "plan_item_id": f"FIX-{order:03d}",
                    "global_order": order,
                    "block_order": block_order,
                    "display_name": f"fixture_map_{map_index:02d}",
                    "filename": f"holdout{map_index:02d}.xml",
                    "scene_id": f"fixture_scene_{map_index:02d}",
                    "map_version": "m1-mapset/1.0",
                    "split": "held_out",
                    "density": "sparse",
                    "route_structure": "fixture",
                    "repeat_block": block,
                    "williams_sequence_id": sequence_name,
                    "within_block_position": position,
                    "group": group,
                    "E": params["E"], "X": params["X"], "H_s": params["H"],
                    "start_x_m": 0.0, "start_y_m": 0.0,
                    "goal_x_m": 7.5, "goal_y_m": 0.0,
                    "arrival_radius_m": 0.5, "sim_limit_s": 20.0, "wall_cap_s": 120.0,
                    "standing_observation_sim_s": 2.0, "standing_wall_cap_s": 30.0,
                    "mujoco_window": "hidden", "rviz_enabled": False,
                    "schedule_seed": 2026102001,
                }))
                order += 1
    return records, fixture_meta()


def fixture_binding(plan: list[dict], meta: dict) -> dict[str, Any]:
    return {
        "matrix_id": meta["matrix_id"], "matrix_status": meta["status"],
        "plan_sha256": runner.plan_digest(plan), "csv_sha256": None, "json_sha256": None,
        "map_version": meta["map_version"], "schedule_seed": meta["schedule_seed"],
        "bootstrap_seed": meta["bootstrap_seed"], "plan_items": len(plan),
        "authorized_to_run": False,
    }


def fixture_auth(binding: dict[str, Any], batch_id: str, replacements: list[dict] | None = None
                 ) -> dict[str, Any]:
    return {
        "schema": runner.AUTHORIZATION_SCHEMA,
        "authorization_id": f"OFFLINE-FIXTURE-{batch_id}",
        "authorized_to_run": True,
        "fixture_only": True,
        "batch_id": batch_id,
        "matrix_id": binding["matrix_id"],
        "plan_sha256": binding["plan_sha256"],
        "base_scene_starts": runner.BASE_SCENE_STARTS,
        "max_substitutions": runner.MAX_SUBSTITUTIONS,
        "max_total_scene_starts": runner.MAX_TOTAL_SCENE_STARTS,
        "batch_budget_gib": runner.BATCH_BUDGET_GIB,
        "min_free_gib": runner.MIN_FREE_GIB,
        "director_review": {"reference": "OFFLINE FIXTURE ONLY - NOT A RUN AUTHORIZATION",
                            "date": "2026-10-21"},
        "replacements": replacements or [],
    }


def fake_summary(*, terminal: str = "ARRIVED", safe: bool | str = True,
                 collision: bool = False, fall: bool = False, out_of_bounds: bool = False,
                 cleanup: bool = True, duration: float = 8.0,
                 recovery_share: float = 0.10) -> dict[str, Any]:
    return {
        "run_id": "fixture",
        "navigation_terminal_result": terminal,
        "terminal_result": terminal,
        "record_validity": "VALID",
        "task_window_safety": {
            "nonfoot_collision_failure": collision, "fall": fall,
            "coverage_complete_for_motion_interval": True,
        },
        "declared_boundary_physics_observation": {
            "status": ("BASE_LINK_OUT_OF_BOUNDS_PHYSICS_STEP_OBSERVED" if out_of_bounds
                       else "BASE_LINK_IN_BOUNDS_PHYSICS_STEPS_OBSERVED")},
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
        "recovery_share_valid_motion_time": recovery_share,
        "recovery_intervals": [
            {"duration_s": 0.2, "censored": False}, {"duration_s": 0.4, "censored": False},
        ],
        "short_reverse_switch_count": 4,
        "short_reverse_switch_coverage": "complete",
        "valid_motion_duration_s": duration,
        "transitions": [
            {"sim_clock_valid": True, "sim_time_s": 1.0, "mode_before": 0, "mode_after": 1},
            {"sim_clock_valid": True, "sim_time_s": 1.2, "mode_before": 1, "mode_after": 0},
            {"sim_clock_valid": True, "sim_time_s": 1.5, "mode_before": 0, "mode_after": 1},
        ],
        "target_distance_start_m": 7.5, "target_distance_final_m": 0.4,
        "foot_contact_count": 0, "foot_impact_count": 0,
    }


class FakeExecutor:
    """Writes only constructed files; never imports or calls the diagnostic."""

    def __init__(self, behaviors: list[str]):
        self.behaviors = list(behaviors)
        self.calls = 0

    def __call__(self, command: list[str], log_path: Path) -> dict[str, Any]:
        raise AssertionError("FakeExecutor.__call__ must never be used as the real dispatcher")

    def dispatch(self, command: list[str], log_path: Path) -> dict[str, Any]:
        output_root = Path(command[command.index("--output-root") + 1])
        attempt_dir = output_root.parent
        behavior = self.behaviors.pop(0) if self.behaviors else "entered_arrived"
        self.calls += 1
        log_path.parent.mkdir(parents=True, exist_ok=True)
        if behavior == "pre_scene":
            log_path.write_text("SCENE_PREFLIGHT_REJECTED fixture\n", encoding="utf-8")
            return {"stdout_result": {"result": "SCENE_PREFLIGHT_REJECTED", "reason": "fixture"},
                    "returncode": 2, "interrupted": False}
        if behavior == "unknown":
            log_path.write_text("no scene evidence fixture\n", encoding="utf-8")
            return {"stdout_result": None, "returncode": None, "interrupted": False}
        if behavior == "scene_no_run_id":
            log_path.write_text("[MUJOCO-WINDOW] backend=GLFW requested=hidden visible=0\n"
                                "MuJoCo version 3.3.3\n", encoding="utf-8")
            return {"stdout_result": None, "returncode": None, "interrupted": False}

        terminal = ("COLLISION_TERMINATION" if behavior == "entered_collision" else
                    "DIAGNOSTIC_SIM_TIME_LIMIT" if behavior == "entered_timeout" else
                    "SYSTEM_SAFETY_ABORT" if behavior == "entered_abort" else "ARRIVED")
        safe = ("UNKNOWN" if behavior == "entered_unknown_safety" else
                False if behavior in {"entered_collision", "entered_timeout", "entered_abort"}
                else True)
        cleanup = behavior != "entered_cleanup_unconfirmed"
        group = command[command.index("--group") + 1]
        observed_group = "B" if behavior == "entered_wrong_group" else group
        scene = command[command.index("--scene") + 1]
        entry = float(command[command.index("--entry-threshold") + 1])
        exit_value = float(command[command.index("--exit-threshold") + 1])
        hold_value = float(command[command.index("--hold-s") + 1])
        goal_x = float(command[command.index("--goal-x") + 1])
        goal_y = float(command[command.index("--goal-y") + 1])
        scene_id = f"fixture_scene_{int(scene[7:9]):02d}"
        output_root.mkdir(parents=True, exist_ok=True)
        run_id = uuid.uuid4().hex
        run_dir = output_root / run_id
        run_dir.mkdir()
        (run_dir / "mujoco.log").write_text(
            "[MUJOCO-WINDOW] backend=GLFW requested=hidden visible=0\nMuJoCo version 3.3.3\n",
            encoding="utf-8")
        context = {
            "run_id": run_id,
            "session_id": 1,
            "settings": {
                "scene_id": scene_id, "group": group,
                "goal_world_m": [goal_x, goal_y],
                "entry_threshold": entry,
                "exit_threshold": exit_value if group in {"B", "D"} else entry,
                "recovery_min_hold_s": hold_value if group in {"C", "D"} else 0.0,
                "hysteresis": "true" if group in {"B", "D"} else "false",
                "hold": "true" if group in {"C", "D"} else "false",
            },
            "runtime_config_observed": {
                "group": observed_group, "entry_threshold": entry,
                "exit_threshold": exit_value if observed_group in {"B", "D"} else entry,
                "recovery_min_hold_s": hold_value if observed_group in {"C", "D"} else 0.0,
                "hysteresis": observed_group in {"B", "D"}, "hold": observed_group in {"C", "D"},
                "switching_mode": {"A": 1, "B": 2, "C": 3, "D": 4}[observed_group],
                "session_id": 1,
                "source_binding": {
                    "group": observed_group, "controller_pid": 123, "writer_pid": 123,
                    "config": f"/fixture/overlays/{observed_group}/config.yaml",
                    "package_share": f"/fixture/overlays/{observed_group}",
                    "goal_world_m": [goal_x, goal_y],
                },
            },
            "runtime_source_validation": {
                "status": "confirmed", "reason": "all_source_events_match",
                "evidence": {"group": observed_group, "controller_pid": 123, "writer_pid": 123},
            },
            "preflight": {
                "candidate_config_group_verified": observed_group,
                "candidate_config_entry_E_verified": True,
                "candidate_config_goal_x_m_verified": True,
                "candidate_config_legacy_fields_absent": True,
                "candidate_config_path": f"/fixture/overlays/{observed_group}/config.yaml",
                "collision_reader_scene_binding": {"scene_id": scene_id},
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
            "process_exit_code": 0 if cleanup else None,
            "external_safety_events": (
                [{"event": "POLICY_SAFETY_VETO", "ray_read_diagnostics": {"attempts": 16}}]
                if behavior == "entered_abort" else []),
        }
        process_facts = {"exit_code": 0, "shutdown_complete": cleanup,
                         "closeout_complete": cleanup, "forced_termination": False}
        summary = fake_summary(
            terminal=terminal, safe=safe,
            collision=terminal == "COLLISION_TERMINATION",
            fall=False, cleanup=cleanup)
        summary["run_id"] = run_id
        for name, value in (("run_context.json", context), ("terminal.json", terminal_record),
                            ("process_facts.json", process_facts), ("metrics.json", summary)):
            (run_dir / name).write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")
        return {"stdout_result": {"run_id": run_id, "result": terminal,
                                  "process_facts": process_facts},
                "returncode": 0, "interrupted": False}


def run_fixture(name: str, plan: list[dict], meta: dict[str, Any], behaviors: list[str], *,
                disk_probe=lambda: 50.0, max_new_starts: int | None = None,
                resume: bool = False, allow_fixture: bool = True,
                replacements: list[dict] | None = None,
                authorization_override: dict | None = None) -> tuple[dict[str, Any], Path]:
    batch_root = FIXTURES / name
    if batch_root.exists():
        import shutil
        shutil.rmtree(batch_root)
    batch_root.mkdir(parents=True, exist_ok=True)
    binding = fixture_binding(plan, meta)
    batch_id = f"fixture-{name}"
    authorization = authorization_override or fixture_auth(binding, batch_id, replacements)
    authorization["batch_id"] = batch_id
    authorization["plan_sha256"] = binding["plan_sha256"]
    authorization["matrix_id"] = binding["matrix_id"]
    authorization_dir = FIXTURES / "_authorizations"
    authorization_dir.mkdir(parents=True, exist_ok=True)
    authorization_path = authorization_dir / f"{name}.json"
    authorization_path.write_text(json.dumps(authorization, indent=2) + "\n", encoding="utf-8")
    fake = FakeExecutor(behaviors)
    result = runner.run_batch(
        batch_root=batch_root, batch_id=batch_id, plan=plan, meta=meta, binding=binding,
        authorization=authorization, authorization_path=authorization_path,
        executor=fake.dispatch, analyze_run=None, disk_probe=disk_probe,
        allow_fixture=allow_fixture, resume=resume, require_full_plan=False,
        overlay_preparer=lambda root: {"status": "FIXTURE_OVERLAY", "groups": {}},
        max_new_starts=max_new_starts)
    result["executor_calls"] = fake.calls
    return result, batch_root


def attempt_row(plan_item: dict[str, Any], *, number: int, run_id: str,
                output_directory: str, terminal: str = "ARRIVED",
                validity: str = "VALID", safe: bool | str = True,
                scene_created: bool = True, entered_rl: bool = True,
                is_replacement: bool = False, replaces: str | None = None,
                authorized: bool = False, cause: dict | None = None,
                confirmed_safety_failure: bool = False,
                authorization_id: str | None = None) -> dict[str, Any]:
    run_directory = str(Path(output_directory) / run_id)
    return {
        "attempt_id": f"{plan_item['plan_item_id']}-attempt-{number}",
        "plan_item_id": plan_item["plan_item_id"], "group": plan_item["group"],
        "invocation_number": number, "scene_created": scene_created,
        "run_id": run_id, "entered_rl": entered_rl, "raw_terminal": terminal,
        "record_validity": validity, "safety_task_completion": safe,
        "output_directory": output_directory, "run_directory": run_directory,
        "is_replacement": is_replacement, "replaces_attempt_id": replaces,
        "replacement_authorized": authorized,
        "replacement_authorization_id": authorization_id,
        "cause_evidence": cause or {"class": "none", "references": [],
                                     "independent_infrastructure_corroboration": False},
        "confirmed_task_safety_failure": confirmed_safety_failure,
        "safety_failure_evidence": ({"references": ["fixture"]}
                                    if confirmed_safety_failure else {}),
    }


def check_plan() -> None:
    records, meta, errors = runner.load_frozen_plan()
    validation = runner.validate_plan(records, meta, require_full=True)
    binding = runner.matrix_binding(records, meta)
    groups = Counter(item["group"] for item in records)
    orders = [item["global_order"] for item in records]
    passed = (not errors and validation["status"] == "PASS" and len(records) == 240
              and groups == {"A": 60, "B": 60, "C": 60, "D": 60}
              and orders == list(range(1, 241)) and binding["authorized_to_run"] is False)
    record("01_frozen_plan_read_complete", passed, {
        "plan_items": len(records), "groups": dict(groups),
        "validation": validation["status"], "load_errors": errors,
        "plan_sha256": binding["plan_sha256"],
        "transitions_balanced": set(validation["transitions"].values()) == {15},
    })


def check_overlays() -> None:
    root = OUT / "overlay_check"
    if root.exists():
        import shutil
        shutil.rmtree(root)
    report = runner.prepare_group_overlays(root)
    fields: dict[str, Any] = {}
    passed = True
    for group, value in report["groups"].items():
        text = (Path(value["config_path"])).read_text(encoding="utf-8")
        switching = text.split("  switching:", 1)[1].split("\n\n", 1)[0]
        has_exit = "exit_threshold:" in switching
        has_hold = "recovery_min_hold_s:" in switching
        fields[group] = {"has_exit": has_exit, "has_hold": has_hold,
                         "status": value["status"],
                         "goal_x_7_5": "goal_x: 7.5" in text,
                         "goal_y_0": "goal_y: 0" in text}
        if group in {"A", "C"} and has_exit:
            passed = False
        if group in {"B", "D"} and not has_exit:
            passed = False
        if group in {"A", "B"} and has_hold:
            passed = False
        if group in {"C", "D"} and not has_hold:
            passed = False
    record("02_group_overlay_fields", passed, fields)


def check_refusals() -> None:
    plan, meta = fixture_plan()
    binding = fixture_binding(plan, meta)
    batch_root = OUT / "refusal" / "batch"
    os_auth = OUT / "refusal" / "fixture_auth.json"
    os_auth.parent.mkdir(parents=True, exist_ok=True)
    auth = fixture_auth(binding, "production-refusal")
    os_auth.write_text(json.dumps(auth, indent=2) + "\n", encoding="utf-8")

    def cli(*arguments: str) -> tuple[int, str]:
        result = subprocess.run(["/usr/bin/python3", str(ROOT / "scripts/run_s7_formal_240.py"),
                                 "run", *arguments], cwd=ROOT, text=True,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                timeout=120, check=False)
        return result.returncode, result.stdout

    code_missing, out_missing = cli("--batch-root", str(batch_root), "--batch-id", "refusal")
    code_fixture, out_fixture = cli("--batch-root", str(batch_root), "--batch-id", "refusal",
                                    "--authorization", str(os_auth))
    core = runner.run_batch(
        batch_root=batch_root, batch_id="refusal", plan=plan, meta=meta, binding=binding,
        authorization=auth, authorization_path=os_auth,
        executor=lambda command, log: (_ for _ in ()).throw(AssertionError("dispatch called")),
        allow_fixture=False, require_full_plan=False,
        overlay_preparer=lambda root: {"status": "FIXTURE_OVERLAY", "groups": {}})
    passed = (code_missing == 2 and "missing_explicit_batch_authorization" in out_missing
              and code_fixture == 2 and "fixture_authorization_rejected" in out_fixture
              and core["status"] == "REFUSED"
              and not batch_root.exists())
    record("03_default_and_missing_authorization_refused", passed, {
        "missing_exit": code_missing, "fixture_exit": code_fixture,
        "core_status": core["status"], "batch_dir_created": batch_root.exists(),
    })


def check_legal_terminal_no_rerun() -> None:
    plan, meta = fixture_plan(maps=1, blocks_per_map=2)  # 8 items
    first, batch_root = run_fixture(
        "legal_terminal_no_rerun", plan, meta, ["entered_arrived"] * len(plan))
    attempts_before = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"]

    def forbidden(command, log):
        raise AssertionError("legal terminal was re-dispatched")

    binding = fixture_binding(plan, meta)
    authorization = runner.read_json(FIXTURES / "_authorizations" / "legal_terminal_no_rerun.json")
    resumed = runner.run_batch(
        batch_root=batch_root, batch_id="fixture-legal_terminal_no_rerun", plan=plan,
        meta=meta, binding=binding, authorization=authorization,
        authorization_path=FIXTURES / "_authorizations" / "legal_terminal_no_rerun.json",
        executor=forbidden, analyze_run=None, allow_fixture=True, resume=True,
        require_full_plan=False, overlay_preparer=lambda root: {"status": "FIXTURE_OVERLAY",
                                                                "groups": {}})
    attempts_after = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"]
    passed = (first["status"] == "COMPLETE" and first["dispatched"] == len(plan)
              and resumed["status"] == "COMPLETE" and resumed["dispatched"] == 0
              and len(attempts_after) == len(attempts_before))
    record("04_valid_terminal_not_rerun", passed, {
        "first_status": first["status"], "first_dispatched": first["dispatched"],
        "resume_status": resumed["status"], "resume_dispatched": resumed["dispatched"],
        "attempts_before": len(attempts_before), "attempts_after": len(attempts_after),
    })


def check_attempt_fact_classes() -> None:
    plan, meta = fixture_plan()  # 4 items
    outcomes = {}
    for name, behavior in (("pre_scene", "pre_scene"),
                           ("no_run_id", "scene_no_run_id"),
                           ("entered_rl", "entered_arrived")):
        result, batch_root = run_fixture(f"fact_{name}", plan, meta, [behavior],
                                         max_new_starts=1)
        attempts = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"]
        outcomes[name] = {
            "run_status": result["status"], "pause": result.get("pause_reason"),
            "attempts": [{key: row.get(key) for key in (
                "scene_created", "run_id", "entered_rl", "raw_terminal",
                "record_validity", "runner_status")} for row in attempts],
        }
    audit_merged = audit_attempt_ledger(plan, runner.read_json(
        FIXTURES / "fact_no_run_id" / "ledger" / "attempts.json")["attempts"])
    passed = (
        outcomes["pre_scene"]["attempts"][0]["scene_created"] is False
        and outcomes["pre_scene"]["attempts"][0]["run_id"] is None
        and outcomes["pre_scene"]["attempts"][0]["runner_status"] == "PRE_SCENE_FAILURE"
        and outcomes["no_run_id"]["attempts"][0]["scene_created"] is True
        and outcomes["no_run_id"]["attempts"][0]["run_id"] is None
        and outcomes["no_run_id"]["attempts"][0]["runner_status"] == "SCENE_STARTED"
        and outcomes["no_run_id"]["attempts"][0]["entered_rl"] is None
        and outcomes["entered_rl"]["attempts"][0]["scene_created"] is True
        and outcomes["entered_rl"]["attempts"][0]["entered_rl"] is True
        and audit_merged["counts"]["actual_scene_starts"] == 1
    )
    record("05_scene_fact_classes", passed, outcomes)


def check_interrupted_resume() -> None:
    plan, meta = fixture_plan(maps=1, blocks_per_map=2)  # 8 items
    first, batch_root = run_fixture(
        "interrupted_resume", plan, meta, ["entered_arrived"] * 2, max_new_starts=2)
    ledger = runner.read_json(batch_root / "ledger" / "attempts.json")
    before_ids = [row["attempt_id"] for row in ledger["attempts"]]
    before_runs = [row.get("run_id") for row in ledger["attempts"]]
    # Simulate a hard interruption after a later attempt was durably marked as
    # an intent with an unknown scene-creation fact.
    interrupted_item = plan[2]
    unknown = attempt_row(interrupted_item, number=1, run_id="pending",
                          output_directory=str(batch_root / "runs" / interrupted_item["plan_item_id"]
                                               / "attempt_01" / "result"))
    unknown["run_id"] = None
    unknown["run_directory"] = None
    unknown["scene_created"] = None
    unknown["entered_rl"] = None
    unknown["raw_terminal"] = None
    unknown["record_validity"] = "UNKNOWN"
    unknown["output_directory"] = str(batch_root / "runs" / interrupted_item["plan_item_id"]
                                      / "attempt_01" / "result")
    ledger["attempts"].append(unknown)
    runner.write_json_atomic(batch_root / "ledger" / "attempts.json", ledger)

    def forbidden(command, log):
        raise AssertionError("unknown scene fact was auto-retried")

    binding = fixture_binding(plan, meta)
    authorization = runner.read_json(FIXTURES / "_authorizations" / "interrupted_resume.json")
    resumed = runner.run_batch(
        batch_root=batch_root, batch_id="fixture-interrupted_resume", plan=plan,
        meta=meta, binding=binding, authorization=authorization,
        authorization_path=FIXTURES / "_authorizations" / "interrupted_resume.json",
        executor=forbidden, analyze_run=None, allow_fixture=True, resume=True,
        require_full_plan=False, overlay_preparer=lambda root: {"status": "FIXTURE_OVERLAY",
                                                                "groups": {}})
    after = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"]
    passed = (first["status"] == "PAUSED"
              and first.get("pause_reason") == "scene_start_cap_reached"
              and resumed["status"] == "PAUSED"
              and resumed.get("pause_reason") == "scene_creation_unknown"
              and resumed["dispatched"] == 0
              and [row.get("run_id") for row in after if row.get("run_id")] == before_runs
              and len([row for row in after if row.get("scene_created") is True]) == 2)
    record("06_interruption_resume_no_duplicate", passed, {
        "first_status": first["status"], "first_pause": first.get("pause_reason"),
        "resume_status": resumed["status"], "resume_pause": resumed.get("pause_reason"),
        "resume_dispatched": resumed["dispatched"],
        "completed_attempts_before": len(before_ids),
        "scene_starts_after": len([row for row in after if row.get("scene_created") is True]),
    })


def check_unknown_scene_fact() -> None:
    plan, meta = fixture_plan()
    first, batch_root = run_fixture("unknown_scene_fact", plan, meta, ["unknown"],
                                    max_new_starts=1)
    attempts = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"]
    binding = fixture_binding(plan, meta)

    def forbidden(command, log):
        raise AssertionError("unknown fact was retried")

    resumed = runner.run_batch(
        batch_root=batch_root, batch_id="fixture-unknown_scene_fact", plan=plan,
        meta=meta, binding=binding,
        authorization=runner.read_json(FIXTURES / "_authorizations" / "unknown_scene_fact.json"),
        authorization_path=FIXTURES / "_authorizations" / "unknown_scene_fact.json",
        executor=forbidden, analyze_run=None, allow_fixture=True, resume=True,
        require_full_plan=False, overlay_preparer=lambda root: {"status": "FIXTURE_OVERLAY",
                                                                "groups": {}})
    passed = (first["status"] == "PAUSED"
              and first.get("pause_reason") == "scene_creation_unknown_after_launch"
              and attempts[0]["scene_created"] is None
              and resumed["status"] == "PAUSED"
              and resumed.get("pause_reason") == "scene_creation_unknown"
              and resumed["dispatched"] == 0)
    record("07_scene_creation_unknown_pauses", passed, {
        "first_pause": first.get("pause_reason"), "scene_created": attempts[0]["scene_created"],
        "resume_pause": resumed.get("pause_reason"), "resume_dispatched": resumed["dispatched"],
    })


def check_bad_bindings() -> None:
    plan, meta = fixture_plan()  # 4 items
    item1, item2 = plan[0], plan[1]
    base_dir = str(FIXTURES / "bad_bindings" / "runs")
    prior = attempt_row(item1, number=1, run_id="run-a", output_directory=f"{base_dir}/a",
                        terminal="SYSTEM_SAFETY_ABORT", validity="INVALID",
                        entered_rl=True)
    replacement = attempt_row(item1, number=2, run_id="run-b", output_directory=f"{base_dir}/b",
                              is_replacement=True, replaces=prior["attempt_id"], authorized=True)
    wrong_owner = attempt_row(item2, number=2, run_id="run-c", output_directory=f"{base_dir}/c",
                              is_replacement=True, replaces=prior["attempt_id"], authorized=True)
    reused = attempt_row(item2, number=1, run_id="run-a", output_directory=f"{base_dir}/d")
    outside = attempt_row(item1, number=2, run_id="run-e", output_directory=f"{base_dir}/e",
                          is_replacement=True, replaces=prior["attempt_id"], authorized=True)
    outside["run_directory"] = str(FIXTURES / "elsewhere" / "run-e")
    cases = {}
    cases["wrong_replacement_owner"] = audit_attempt_ledger(
        plan, [prior, wrong_owner])["errors"]
    cases["run_id_reused"] = audit_attempt_ledger(
        plan, [attempt_row(item1, number=1, run_id="run-a", output_directory=f"{base_dir}/a"),
               reused])["errors"]
    cases["replacement_directory_outside_attempt"] = runner.audit_ledger(
        plan, [prior, outside])["errors"]
    cases["wrong_run_directory"] = runner.audit_ledger(
        plan, [prior, {**replacement, "run_directory": str(FIXTURES / "elsewhere" / "x")}]
    )["errors"]
    passed = (
        "replacement_wrong_plan_or_missing_prior_attempt:" + wrong_owner["attempt_id"]
        in cases["wrong_replacement_owner"]
        and any(error.startswith("run_id_reused:run-a") for error in cases["run_id_reused"])
        and any(error.startswith("replacement_run_outside_own_attempt_directory")
                for error in cases["replacement_directory_outside_attempt"])
        and any(error.startswith("run_directory_outside_attempt_directory")
                for error in cases["wrong_run_directory"])
    )
    record("08_bad_directory_run_id_replacement_binding", passed, cases)


def check_replacement_quotas() -> None:
    plan, meta = fixture_plan()  # 4 items
    item = plan[0]
    prior = attempt_row(item, number=1, run_id="prior-1", output_directory="/fixture/a",
                        terminal="SYSTEM_SAFETY_ABORT", validity="INVALID",
                        cause={"class": "infrastructure_trigger_observed",
                               "references": ["fixture"], "independent_infrastructure_corroboration": True})
    replacement1 = attempt_row(item, number=2, run_id="rep-1", output_directory="/fixture/b",
                               is_replacement=True, replaces=prior["attempt_id"], authorized=True)
    replacement2 = attempt_row(item, number=3, run_id="rep-2", output_directory="/fixture/c",
                               is_replacement=True, replaces=prior["attempt_id"], authorized=True)
    second_per_item = audit_attempt_ledger(plan, [prior, replacement1, replacement2])["errors"]

    batch_plan = []
    attempts = []
    for index in range(13):
        mini, _ = fixture_plan()
        target = dict(mini[0])
        target["plan_item_id"] = f"Q-{index:02d}"
        target["global_order"] = index + 1
        target["block_order"] = index + 1
        target["within_block_position"] = (index % 4) + 1
        batch_plan.append(target)
        prior_row = attempt_row(target, number=1, run_id=f"q-prior-{index}",
                                output_directory=f"/fixture/q{index}/a",
                                terminal="SYSTEM_SAFETY_ABORT", validity="INVALID",
                                cause={"class": "infrastructure_trigger_observed",
                                       "references": ["fixture"],
                                       "independent_infrastructure_corroboration": True})
        replacement = attempt_row(target, number=2, run_id=f"q-rep-{index}",
                                  output_directory=f"/fixture/q{index}/b",
                                  is_replacement=True, replaces=prior_row["attempt_id"],
                                  authorized=True)
        attempts.extend([prior_row] + ([replacement] if index < 13 else []))
    batch_errors = audit_attempt_ledger(batch_plan, attempts)["errors"]
    passed = ("plan_item_substitution_quota_exceeded:" + item["plan_item_id"]
              in second_per_item
              and any(error.startswith("batch_substitution_quota_exceeded:13")
                      for error in batch_errors))
    record("09_replacement_quota_rules", passed, {
        "second_same_item": second_per_item, "batch_13": batch_errors})


def check_safety_abort_no_auto_replacement() -> None:
    plan, meta = fixture_plan()
    item = plan[0]
    abort = attempt_row(item, number=1, run_id="abort-run", output_directory="/fixture/abort",
                        terminal="SYSTEM_SAFETY_ABORT", validity="INVALID",
                        cause={"class": "infrastructure_trigger_observed",
                               "references": ["terminal ray diagnostics"],
                               "independent_infrastructure_corroboration": False})
    classified = classify_attempt(abort)
    batch_root = FIXTURES / "safety_abort_no_replacement"
    if batch_root.exists():
        import shutil
        shutil.rmtree(batch_root)
    batch_root.mkdir(parents=True)
    (batch_root / "ledger").mkdir()
    runner.write_json_atomic(batch_root / "ledger" / "attempts.json", {
        "schema": runner.LEDGER_SCHEMA, "batch_id": "fixture-safety-abort",
        "attempts": [abort], "events": [], "checkpoints": []})
    (batch_root / "batch_manifest.json").write_text(json.dumps({
        "schema": runner.MANIFEST_SCHEMA, "batch_id": "fixture-safety-abort",
        "matrix": fixture_binding(plan, meta),
        "version_record": runner.version_record()}, indent=2) + "\n", encoding="utf-8")
    binding = fixture_binding(plan, meta)
    authorization = fixture_auth(binding, "fixture-safety-abort", replacements=[{
        "plan_item_id": item["plan_item_id"], "replaces_attempt_id": abort["attempt_id"],
        "reason": "fixture", "authorized_by": "Director",
        "authorized_date": "2026-10-21", "authorization_id": "fixture-replacement",
        "evidence_references": ["terminal ray diagnostics"],
        "independent_infrastructure_corroboration": True,
    }])
    auth_path = FIXTURES / "_authorizations" / "safety_abort.json"
    auth_path.write_text(json.dumps(authorization, indent=2) + "\n", encoding="utf-8")

    def forbidden(command, log):
        raise AssertionError("unsupported safety-abort replacement was dispatched")

    result = runner.run_batch(
        batch_root=batch_root, batch_id="fixture-safety-abort", plan=plan, meta=meta,
        binding=binding, authorization=authorization, authorization_path=auth_path,
        executor=forbidden, analyze_run=None, allow_fixture=True, resume=True,
        require_full_plan=False, overlay_preparer=lambda root: {"status": "FIXTURE_OVERLAY",
                                                                "groups": {}})
    passed = (classified["replacement_eligible"] is False
              and result["status"] == "PAUSED" and result["dispatched"] == 0
              and "unresolved:SYSTEM_SAFETY_ABORT" in str(result.get("pause_reason")))
    record("10_safety_abort_without_corroboration_not_replaced", passed, {
        "replacement_eligible": classified["replacement_eligible"],
        "replacement_reason": classified.get("replacement_reason"),
        "runner_status": result["status"], "pause_reason": result.get("pause_reason"),
        "dispatched": result["dispatched"],
    })


def build_view_fixture() -> tuple[list[dict], dict, dict, dict]:
    plan, meta = fixture_plan(maps=1, blocks_per_map=2)  # 8 items
    attempts: list[dict] = []
    summaries: dict[str, dict] = {}
    for index, item in enumerate(plan):
        safe = "UNKNOWN" if index == 0 else (index % 3 != 0)
        terminal = "ARRIVED" if safe is not False else "COLLISION_TERMINATION"
        row = attempt_row(item, number=1, run_id=f"view-run-{index}",
                          output_directory=f"/fixture/view/{index}", terminal=terminal,
                          safe=safe, confirmed_safety_failure=(safe is False))
        attempts.append(row)
        summaries[row["attempt_id"]] = fake_summary(
            terminal=terminal, safe=safe, collision=(safe is False),
            cleanup=True)
    audit = runner.audit_ledger(plan, attempts)
    return plan, meta, audit, summaries


def check_three_views() -> None:
    plan, meta, audit, summaries = build_view_fixture()
    views = analysis.build_views(plan, audit, summaries)
    primary = views["primary_plan_item"]
    total = sum(primary["outcome_counts"].values())
    all_starts = views["all_actual_scene_starts"]
    first = views["first_actual_scene_only_sensitivity"]
    passed = (total == len(plan) and all_starts["counts"]["actual_scene_starts"] == len(plan)
              and first["denominator_plan_items"] == len(plan)
              and len(first.get("rows", [])) == len(plan)
              and sum(first.get("outcome_counts", {}).values()) == len(plan)
              and "UNKNOWN" not in primary["outcome_counts"]
              and primary["outcome_counts"].get("SAFETY_UNKNOWN", 0) >= 1
              and primary["outcome_counts"].get("KNOWN_NOT_COMPLETE", 0) >= 1)
    record("11_three_views_denominators", passed, {
        "primary_outcomes": primary["outcome_counts"], "plan_items": len(plan),
        "all_start_counts": all_starts["counts"],
        "first_view_denominator": first["denominator_plan_items"],
        "first_rows": len(first.get("rows", [])),
        "first_outcomes": first.get("outcome_counts"),
    })


def check_unknown_and_invalid_rules() -> None:
    base_plan, meta = fixture_plan()
    plan = [dict(base_plan[0], plan_item_id=f"U-{index}", global_order=index + 1)
            for index in range(4)]
    item_unknown, item_invalid, item_failure, item_not_started = plan
    attempts = [
        attempt_row(item_unknown, number=1, run_id="u-1", output_directory="/f/u",
                    safe="UNKNOWN"),
        attempt_row(item_invalid, number=1, run_id="i-1", output_directory="/f/i",
                    terminal="COLLISION_TERMINATION", validity="INVALID", safe=False,
                    confirmed_safety_failure=True),
        attempt_row(item_failure, number=1, run_id="f-1", output_directory="/f/f",
                    terminal="FALL_TERMINATION", safe=False,
                    confirmed_safety_failure=True),
    ]
    audit = runner.audit_ledger(plan, attempts)
    views = analysis.build_views(plan, audit, {})
    outcomes = {row["plan_item_id"]: row["outcome"]
                for row in views["primary_plan_item"]["rows"]}
    group = views["primary_plan_item"]["per_group"][item_unknown["group"]]
    passed = (
        outcomes[item_unknown["plan_item_id"]] == "SAFETY_UNKNOWN"
        and outcomes[item_invalid["plan_item_id"]] == "KNOWN_NOT_COMPLETE"
        and outcomes[item_failure["plan_item_id"]] == "KNOWN_NOT_COMPLETE"
        and outcomes[item_not_started["plan_item_id"]] == "NOT_STARTED"
        and group["safe_completion_lower_over_all_plan_items"] == 0.0
        and group["safe_completion_upper_over_all_plan_items"] == 0.5
    )
    record("12_unknown_invalid_not_started_vs_confirmed_failure", passed, {
        "outcomes": outcomes, "group_bounds": group,
    })


def check_task_cleanup_separation() -> None:
    plan, meta = fixture_plan()
    item = plan[0]
    task_failed_cleanup_ok = attempt_row(item, number=1, run_id="sep-1",
                                         output_directory="/f/sep1",
                                         terminal="COLLISION_TERMINATION", safe=False,
                                         confirmed_safety_failure=True)
    task_ok_cleanup_failed = attempt_row(plan[1], number=1, run_id="sep-2",
                                         output_directory="/f/sep2", safe=True)
    audit = runner.audit_ledger(plan, [task_failed_cleanup_ok, task_ok_cleanup_failed])
    summaries = {
        task_failed_cleanup_ok["attempt_id"]: fake_summary(
            terminal="COLLISION_TERMINATION", safe=False, collision=True, cleanup=True),
        task_ok_cleanup_failed["attempt_id"]: fake_summary(safe=True, cleanup=False),
    }
    report = analysis.build_report(plan=plan, audit=audit, summary_by_attempt=summaries,
                                   replicates=20, seed=2026102002)
    rows = {row["plan_item_id"]: row for row in report["views"]["primary_plan_item"]["rows"]}
    first = rows[item["plan_item_id"]]
    second = rows[plan[1]["plan_item_id"]]
    passed = (first["outcome"] == "KNOWN_NOT_COMPLETE"
              and first["metrics"]["cleanup_complete"] is True
              and second["outcome"] == "CONFIRMED_SAFE_COMPLETE"
              and second["metrics"]["cleanup_complete"] is False
              and second["metrics"]["safe_complete"] is True)
    record("13_task_and_cleanup_windows_separate", passed, {
        "task_failed_cleanup_ok": {"outcome": first["outcome"],
                                   "cleanup": first["metrics"]["cleanup_complete"]},
        "task_ok_cleanup_failed": {"outcome": second["outcome"],
                                   "cleanup": second["metrics"]["cleanup_complete"],
                                   "safe_complete": second["metrics"]["safe_complete"]},
    })


def check_resource_guards() -> None:
    plan, meta = fixture_plan()
    disk, _ = run_fixture("disk_guard", plan, meta, ["entered_arrived"],
                          disk_probe=lambda: 14.0, max_new_starts=1)
    # Quota path: 12 authorized replacement starts already exist; the 13th is
    # pending and must be refused before the executor is called.
    quota_plan, quota_meta = fixture_plan(maps=4, blocks_per_map=1)  # 16 items
    attempts: list[dict] = []
    quota_entries: list[dict] = []
    for index, item in enumerate(quota_plan):
        if index < 12:
            prior = attempt_row(item, number=1, run_id=f"qp-{index}",
                                output_directory=f"/fixture/quota/{index}/a",
                                terminal="SYSTEM_SAFETY_ABORT", validity="INVALID",
                                cause={"class": "infrastructure_trigger_observed",
                                       "references": ["fixture"],
                                       "independent_infrastructure_corroboration": True})
            attempts.append(prior)
            replacement_id = f"fixture-quota-{index}"
            attempts.append(attempt_row(item, number=2, run_id=f"qr-{index}",
                                        output_directory=f"/fixture/quota/{index}/b",
                                        is_replacement=True,
                                        replaces=prior["attempt_id"], authorized=True,
                                        authorization_id=replacement_id))
            quota_entries.append({
                "plan_item_id": item["plan_item_id"],
                "replaces_attempt_id": prior["attempt_id"],
                "reason": "fixture", "authorized_by": "Director",
                "authorized_date": "2026-10-21",
                "authorization_id": replacement_id,
                "evidence_references": ["fixture"],
                "independent_infrastructure_corroboration": True,
            })
        elif index == 12:
            attempts.append(attempt_row(item, number=1, run_id="qv-12",
                                        output_directory="/fixture/quota/12/v"))
        elif index == 13:
            prior = attempt_row(item, number=1, run_id="qp-13",
                                output_directory="/fixture/quota/13/a",
                                terminal="SYSTEM_SAFETY_ABORT", validity="INVALID",
                                cause={"class": "infrastructure_trigger_observed",
                                       "references": ["fixture"],
                                       "independent_infrastructure_corroboration": True})
            attempts.append(prior)
            quota_entries.append({
                "plan_item_id": item["plan_item_id"],
                "replaces_attempt_id": prior["attempt_id"],
                "reason": "fixture", "authorized_by": "Director",
                "authorized_date": "2026-10-21", "authorization_id": "fixture-quota-13",
                "evidence_references": ["fixture"],
                "independent_infrastructure_corroboration": True,
            })
        else:
            attempts.append(attempt_row(item, number=1, run_id=f"qv-{index}",
                                        output_directory=f"/fixture/quota/{index}/v"))
    batch_root = FIXTURES / "quota_guard"
    if batch_root.exists():
        import shutil
        shutil.rmtree(batch_root)
    batch_root.mkdir(parents=True)
    (batch_root / "ledger").mkdir()
    runner.write_json_atomic(batch_root / "ledger" / "attempts.json", {
        "schema": runner.LEDGER_SCHEMA, "batch_id": "fixture-quota-guard",
        "attempts": attempts, "events": [], "checkpoints": []})
    binding = fixture_binding(quota_plan, quota_meta)
    (batch_root / "batch_manifest.json").write_text(json.dumps({
        "schema": runner.MANIFEST_SCHEMA, "batch_id": "fixture-quota-guard",
        "matrix": binding, "version_record": runner.version_record()}, indent=2) + "\n",
        encoding="utf-8")
    authorization = fixture_auth(binding, "fixture-quota-guard",
                                 replacements=quota_entries)
    auth_path = FIXTURES / "_authorizations" / "quota_guard.json"
    auth_path.write_text(json.dumps(authorization, indent=2) + "\n", encoding="utf-8")

    def forbidden(command, log):
        raise AssertionError("quota-exhausted replacement dispatched")

    quota = runner.run_batch(
        batch_root=batch_root, batch_id="fixture-quota-guard", plan=quota_plan,
        meta=quota_meta, binding=binding, authorization=authorization,
        authorization_path=auth_path, executor=forbidden, analyze_run=None,
        allow_fixture=True, resume=True, require_full_plan=False,
        overlay_preparer=lambda root: {"status": "FIXTURE_OVERLAY", "groups": {}})
    passed = (disk["status"] == "PAUSED" and disk.get("pause_reason") == "free_space_below_guard"
              and disk["dispatched"] == 0
              and quota["status"] == "PAUSED"
              and quota.get("pause_reason") == "substitution_quota_exhausted"
              and quota["dispatched"] == 0)
    record("14_disk_and_quota_stop_dispatch", passed, {
        "disk": {"status": disk["status"], "pause": disk.get("pause_reason"),
                 "dispatched": disk["dispatched"]},
        "quota": {"status": quota["status"], "pause": quota.get("pause_reason"),
                  "dispatched": quota["dispatched"]},
    })


def check_deterministic_summary() -> None:
    plan, meta, audit, summaries = build_view_fixture()
    report1 = analysis.build_report(plan=plan, audit=audit, summary_by_attempt=summaries,
                                    replicates=200, seed=2026102002)
    report2 = analysis.build_report(plan=plan, audit=audit, summary_by_attempt=summaries,
                                    replicates=200, seed=2026102002)
    first = json.dumps(report1, sort_keys=True, ensure_ascii=False)
    second = json.dumps(report2, sort_keys=True, ensure_ascii=False)
    passed = runner.sha256_json(report1) == runner.sha256_json(report2) and first == second
    record("15_repeated_summary_identical", passed, {
        "sha256": runner.sha256_json(report1)})


def check_comparisons_and_endpoints() -> None:
    plan, meta = fixture_plan(maps=4, blocks_per_map=1)  # 16 items
    safe_truth = {
        ("fixture_map_01", "A"): True, ("fixture_map_01", "B"): True,
        ("fixture_map_01", "C"): True, ("fixture_map_01", "D"): True,
        ("fixture_map_02", "A"): True, ("fixture_map_02", "B"): True,
        ("fixture_map_02", "C"): False, ("fixture_map_02", "D"): True,
        ("fixture_map_03", "A"): False, ("fixture_map_03", "B"): True,
        ("fixture_map_03", "C"): False, ("fixture_map_03", "D"): True,
        ("fixture_map_04", "A"): "UNKNOWN", ("fixture_map_04", "B"): True,
        ("fixture_map_04", "C"): False, ("fixture_map_04", "D"): False,
    }
    attempts = []
    summaries = {}
    for index, item in enumerate(plan):
        value = safe_truth[(item["display_name"], item["group"])]
        terminal = "ARRIVED" if value is True else (
            "COLLISION_TERMINATION" if value is False else "ARRIVED")
        row = attempt_row(item, number=1, run_id=f"cmp-{index}",
                          output_directory=f"/fixture/cmp/{index}", terminal=terminal,
                          safe=value)
        attempts.append(row)
        summaries[row["attempt_id"]] = fake_summary(
            terminal=terminal, safe=value, collision=(value is False), duration=8.0,
            recovery_share=0.1)
    audit = runner.audit_ledger(plan, attempts)
    report = analysis.build_report(plan=plan, audit=audit, summary_by_attempt=summaries,
                                   replicates=50, seed=2026102002)
    example = {
        "label": "NON_FORMAL_CONSTRUCTED_DATA",
        "warning": "Constructed 4-map x 4-group fixture. This is not an experimental result "
                   "and must never be merged with logs/08 formal data.",
        "report": report,
    }
    runner.write_json_atomic(OUT / "example_three_layer_summary_NON_FORMAL.json", example)
    metrics = report["comparisons"]["metrics"]["safe_complete"]["comparisons"]
    expected = {"B-A": (50.0, 25.0, 50.0), "C-A": (-25.0, -50.0, -25.0),
                "D-B": (-25.0, -25.0, -25.0), "D-C": (50.0, 50.0, 50.0)}
    passed = True
    observed = {}
    for key, (point, lower, upper) in expected.items():
        block = metrics[key]
        scenario = block["scenario_unknown_bounds"]
        observed[key] = {"point": block["point"], "scenario": scenario}
        passed = passed and abs((block["point"] or 0) - point) < 1e-12
        passed = passed and abs(scenario["lower"] - lower) < 1e-12
        passed = passed and abs(scenario["upper"] - upper) < 1e-12
        passed = passed and abs(scenario["worst_case"] - lower) < 1e-12
    interaction = metrics[analysis.INTERACTION_KEY]
    observed[analysis.INTERACTION_KEY] = {"point": interaction["point"],
                                          "ci": interaction["bootstrap_95"]}
    passed = passed and abs(interaction["point"]) < 1e-12
    record("16_comparisons_interaction_scenario_endpoints", passed, observed)


def check_real_records() -> None:
    import s1_analyze_run
    run_ids = {
        "R1": "16465c346b8a4fd1b85c4548169fd6a2",
        "R6": "1699cb934b7a4c73b2a666867eadf353",
    }
    expected = {"R1": True, "R6": False}
    output: dict[str, Any] = {}
    passed = True
    out_dir = OUT / "real_records"
    out_dir.mkdir(parents=True, exist_ok=True)
    for label, run_id in run_ids.items():
        matches = list((ROOT / "logs").rglob(f"{run_id}/run_context.json"))
        if not matches:
            output[label] = {"status": "NOT_FOUND", "run_id": run_id}
            passed = False
            continue
        run_dir = matches[0].parent
        summary = s1_analyze_run.calculate(run_dir / "runtime_record.jsonl",
                                           run_dir / "run_context.json")
        metrics = analysis.extract_metrics(summary)
        (out_dir / f"{label}_{run_id}.json").write_text(
            json.dumps(summary, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
            encoding="utf-8")
        output[label] = {
            "run_id": run_id, "run_directory": str(run_dir),
            "safe_complete": metrics["safe_complete"],
            "terminal": metrics["navigation_terminal_result"],
            "record_validity": metrics["record_validity"],
            "cleanup_complete": metrics["cleanup_complete"],
        }
        passed = passed and metrics["safe_complete"] is expected[label]
    record("17_real_development_record_read", passed, output)


def check_reconcile_completed_from_disk() -> None:
    plan, meta = fixture_plan(maps=2, blocks_per_map=2)  # 16 items
    first, batch_root = run_fixture(
        "reconcile_completed", plan, meta, ["entered_arrived"], max_new_starts=1)
    item = plan[1]
    attempt_dir = batch_root / "runs" / item["plan_item_id"] / "attempt_01"
    attempt_dir.mkdir(parents=True, exist_ok=True)
    command = runner.build_diagnostic_command(
        item, overlay=batch_root / "overlays" / item["group"],
        output_root=attempt_dir / "result")
    fake = FakeExecutor(["entered_arrived"])
    outcome = fake.dispatch(command, attempt_dir / "console.log")
    written_run_id = outcome["stdout_result"]["run_id"]
    ledger = runner.read_json(batch_root / "ledger" / "attempts.json")
    ledger["attempts"].append({
        **{key: None for key in ("scene_created", "run_id", "entered_rl", "raw_terminal")},
        "attempt_id": f"fixture-reconcile_completed:{item['plan_item_id']}:inv01",
        "plan_item_id": item["plan_item_id"], "group": item["group"],
        "invocation_number": 1, "kind": "scenario_invocation",
        "record_validity": "UNKNOWN", "output_directory": str(attempt_dir / "result"),
        "is_replacement": False, "replaces_attempt_id": None,
        "replacement_authorized": False, "runner_status": "INTENT",
        "cause_evidence": {"class": "not_yet_known", "references": [],
                           "independent_infrastructure_corroboration": False},
    })
    runner.write_json_atomic(batch_root / "ledger" / "attempts.json", ledger)
    binding = fixture_binding(plan, meta)
    authorization = runner.read_json(FIXTURES / "_authorizations" / "reconcile_completed.json")
    resume_executor = FakeExecutor(["entered_arrived"] * (len(plan) - 2))
    resumed = runner.run_batch(
        batch_root=batch_root, batch_id="fixture-reconcile_completed", plan=plan,
        meta=meta, binding=binding, authorization=authorization,
        authorization_path=FIXTURES / "_authorizations" / "reconcile_completed.json",
        executor=resume_executor.dispatch, analyze_run=None, allow_fixture=True,
        resume=True, require_full_plan=False,
        overlay_preparer=lambda root: {"status": "FIXTURE_OVERLAY", "groups": {}})
    after = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"]
    item_attempts = [row for row in after if row["plan_item_id"] == item["plan_item_id"]]
    audit = runner.audit_ledger(plan, after)
    passed = (first["status"] == "PAUSED" and first["dispatched"] == 1
              and resumed["status"] == "COMPLETE"
              and resumed["dispatched"] == len(plan) - 2
              and len(item_attempts) == 1
              and item_attempts[0]["run_id"] == written_run_id
              and item_attempts[0]["scene_created"] is True
              and audit["counts"]["actual_scene_starts"] == len(plan)
              and audit["counts"]["unique_run_ids"] == len(plan))
    record("18_reconcile_completed_from_disk", passed, {
        "first_dispatched": first["dispatched"], "resume_dispatched": resumed["dispatched"],
        "item_attempts": len(item_attempts), "item_run_id_matches": (
            item_attempts[0].get("run_id") == written_run_id if item_attempts else None),
        "actual_scene_starts": audit["counts"]["actual_scene_starts"],
        "unique_run_ids": audit["counts"]["unique_run_ids"],
        "resume_status": resumed["status"],
    })


def write_fixture_matrix(plan: list[dict], meta: dict) -> tuple[Path, Path]:
    import csv
    directory = FIXTURES / "_fixture_matrix"
    directory.mkdir(parents=True, exist_ok=True)
    csv_path = directory / "formal_matrix_fixture.csv"
    json_path = directory / "formal_matrix_fixture.json"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=runner.PLAN_FIELDS)
        writer.writeheader()
        for record in plan:
            writer.writerow({key: ("" if record.get(key) is None else record.get(key))
                             for key in runner.PLAN_FIELDS})
    payload = {**meta, "authorized_to_run": False, "records": plan}
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
                         encoding="utf-8")
    return csv_path, json_path


def check_analysis_cli() -> None:
    plan, meta = fixture_plan(maps=1, blocks_per_map=2)  # 8 items, same as check 04
    batch_root = FIXTURES / "legal_terminal_no_rerun"
    csv_path, json_path = write_fixture_matrix(plan, meta)
    outputs = []
    for index in (1, 2):
        out_dir = OUT / f"analysis_cli_{index}"
        result = subprocess.run([
            "/usr/bin/python3", str(ROOT / "scripts/analyze_s7_formal_240.py"),
            "--batch-root", str(batch_root), "--matrix-csv", str(csv_path),
            "--matrix-json", str(json_path), "--use-stored-metrics",
            "--allow-constructed-plan", "--replicates", "100",
            "--output-dir", str(out_dir),
        ], cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            timeout=300, check=False)
        outputs.append((result.returncode, result.stdout, out_dir))
    first = (outputs[0][2] / "formal_summary.json")
    second = (outputs[1][2] / "formal_summary.json")
    identical = (first.is_file() and second.is_file()
                 and first.read_bytes() == second.read_bytes())
    published = all((outputs[0][2] / name).is_file() for name in
                    ("formal_summary.json", "primary_plan_items.csv",
                     "first_scene_sensitivity.csv", "comparisons.csv", "SUMMARY.md"))
    passed = (all(code == 0 for code, _, _ in outputs) and identical and published)
    record("19_analysis_cli_constructed_repeatable", passed, {
        "exit_codes": [code for code, _, _ in outputs],
        "identical_formal_summary": identical,
        "outputs_published": published,
        "output_dirs": [str(item[2]) for item in outputs],
    })


def check_replacement_first_start_retained() -> None:
    plan, meta = fixture_plan()  # 4 items
    item = plan[0]
    prior = attempt_row(
        item, number=1, run_id="first-abort", output_directory="/fixture/first/a",
        terminal="SYSTEM_SAFETY_ABORT", validity="INVALID", safe=False,
        cause={"class": "infrastructure_trigger_observed",
               "references": ["fixture independent evidence"],
               "independent_infrastructure_corroboration": True})
    replacement = attempt_row(
        item, number=2, run_id="substitute-arrived", output_directory="/fixture/first/b",
        terminal="ARRIVED", validity="VALID", safe=True, is_replacement=True,
        replaces=prior["attempt_id"], authorized=True,
        authorization_id="fixture-first-replacement")
    audit = runner.audit_ledger(plan, [prior, replacement])
    summaries = {
        prior["attempt_id"]: fake_summary(terminal="SYSTEM_SAFETY_ABORT", safe=False,
                                          collision=False),
        replacement["attempt_id"]: fake_summary(terminal="ARRIVED", safe=True),
    }
    report = analysis.build_report(plan=plan, audit=audit, summary_by_attempt=summaries,
                                   replicates=20, seed=2026102002)
    primary = next(row for row in report["views"]["primary_plan_item"]["rows"]
                   if row["plan_item_id"] == item["plan_item_id"])
    first = next(row for row in report["views"]["first_actual_scene_only_sensitivity"]["rows"]
                 if row["plan_item_id"] == item["plan_item_id"])
    starts = report["views"]["all_actual_scene_starts"]
    passed = (audit["status"] == "PASS"
              and primary["outcome"] == "CONFIRMED_SAFE_COMPLETE"
              and first["outcome"] == "KNOWN_NOT_COMPLETE"
              and first["first_scene_terminal"] == "SYSTEM_SAFETY_ABORT"
              and starts["counts"]["actual_scene_starts"] == 2
              and starts["counts"]["replacement_scene_starts"] == 1
              and starts["class_counts"].get("SYSTEM_SAFETY_ABORT", 0) == 1)
    record("20_first_start_failure_not_erased_by_substitute", passed, {
        "audit_status": audit["status"], "primary_outcome": primary["outcome"],
        "first_outcome": first["outcome"],
        "first_terminal": first["first_scene_terminal"],
        "all_start_counts": starts["counts"],
        "all_start_classes": starts["class_counts"],
    })


def check_runtime_consumption_mismatch() -> None:
    plan, meta = fixture_plan()  # first item is group A
    result, batch_root = run_fixture("runtime_consumption_mismatch", plan, meta,
                                     ["entered_wrong_group"], max_new_starts=1)
    attempts = runner.read_json(batch_root / "ledger" / "attempts.json")["attempts"]
    consumption = attempts[0].get("runtime_config_consumption") or {}
    errors = consumption.get("errors") or []
    passed = (result["status"] == "PAUSED"
              and result.get("pause_reason") == "runtime_config_consumption_not_confirmed"
              and result["dispatched"] == 1
              and attempts[0].get("scene_created") is True
              and any(error.startswith("observed_group:B!=A") for error in errors))
    record("21_runtime_consumption_mismatch_pauses", passed, {
        "runner_status": result["status"], "pause_reason": result.get("pause_reason"),
        "consumption_status": consumption.get("status"), "errors": errors,
    })


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    FIXTURES.mkdir(parents=True, exist_ok=True)
    checks = [
        check_plan, check_overlays, check_refusals, check_legal_terminal_no_rerun,
        check_attempt_fact_classes, check_interrupted_resume, check_unknown_scene_fact,
        check_bad_bindings, check_replacement_quotas,
        check_safety_abort_no_auto_replacement, check_three_views,
        check_unknown_and_invalid_rules, check_task_cleanup_separation,
        check_resource_guards, check_deterministic_summary,
        check_comparisons_and_endpoints, check_real_records,
        check_reconcile_completed_from_disk, check_analysis_cli,
        check_replacement_first_start_retained, check_runtime_consumption_mismatch,
    ]
    for check in checks:
        try:
            check()
        except Exception as error:  # noqa: BLE001 - a failed check must stay visible
            import traceback
            record(check.__name__, False, {"exception": f"{type(error).__name__}: {error}",
                                           "traceback": traceback.format_exc()})
    summary = {
        "schema": "s7-runner-readiness-offline-joint-test-v1",
        "scope": "offline only; no MuJoCo, ROS, controller, model inference, or scene start",
        "checks": RESULTS,
        "passed": sum(item["status"] == "PASS" for item in RESULTS),
        "failed": sum(item["status"] == "FAIL" for item in RESULTS),
        "total": len(RESULTS),
        "fixture_authorization_note": "fixture_only records are rejected by the production CLI",
    }
    runner.write_json_atomic(OUT / "check_results.json", summary)
    print(json.dumps({"passed": summary["passed"], "failed": summary["failed"],
                      "total": summary["total"]}, ensure_ascii=False))
    return 0 if summary["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
