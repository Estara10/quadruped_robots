#!/usr/bin/env python3
"""Focused offline checks for exact pre-RL standing rejection evidence."""
from __future__ import annotations

import sys
import tempfile
import os
import json
import copy
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import s1_run_diagnostic as diagnostic  # noqa: E402
import manual_t1_m2_round2 as batch  # noqa: E402


def snap(sequence: int, sim_time: float, *, coverage: int = 1, unknown: int = 0):
    return SimpleNamespace(
        version=5, sequence=sequence, monotonic_ns=1000 + sequence,
        physics_step=sequence // 2, sim_time=sim_time, capture_id="capture-test",
        scenario_id="scene-test", runtime_model_fingerprint="f" * 64,
        authoritative=1, physics_coverage_complete=coverage, unknown_contacts=unknown,
        invalid_posture_steps=0, robot_obstacle_contacts=0, foot_ground_contacts=4,
        ground_contacts=4, base_height_m=0.31, base_roll_rad=0.0,
        base_pitch_rad=0.0, fall_candidate=0, fall_confirmed=0, fall_history=[],
        fall_history_overflow=0, collision_history_overflow=0,
        first_invalid_posture_sim_time=0.0, last_invalid_posture_sim_time=0.0,
    )


def run_monitor(samples, statuses=None, cap=1.0):
    scene = SimpleNamespace(model_fingerprint="f" * 64, scene_id="scene-test",
                            binding=lambda: {"scene_id": "scene-test"})
    fd, ready_path = tempfile.mkstemp(prefix="t1-prep-", suffix=".log")
    os.close(fd)
    ready_log = Path(ready_path)
    ready_log.write_text("[BaseFixedStand]: [STAND-READY] command3_gate=passed\n", encoding="utf-8")
    original = (diagnostic.read_collision_snapshot_diagnostic, diagnostic.classify_snapshot,
                diagnostic.now_ns, diagnostic.collision_snapshot_payload)
    queue = iter(samples)
    status_queue = iter(statuses or [diagnostic.CollisionStatus.LIVE] * len(samples))
    last_sample = samples[-1] if samples else None
    def read():
        return b"fixture", {"read_status": "READ", "sequence": getattr(last_sample, "sequence", None)}
    def classify(*args, **kwargs):
        status = next(status_queue, diagnostic.CollisionStatus.LIVE)
        sample = next(queue, last_sample)
        return status, sample if status is diagnostic.CollisionStatus.LIVE else None
    ticks = iter(range(5000, 1000000))
    diagnostic.read_collision_snapshot_diagnostic = read
    diagnostic.classify_snapshot = classify
    diagnostic.now_ns = lambda: next(ticks)
    diagnostic.collision_snapshot_payload = lambda status, item: {
        "status": status.value, "physics_coverage_complete": bool(item.physics_coverage_complete),
        "unknown_contacts": item.unknown_contacts}
    try:
        return diagnostic.confirm_pre_rl_standing(
            "capture-test", scene, wall_cap_s=cap, stable_sim_s=0.50,
            ros_log_path=ready_log, last_command=2)
    finally:
        (diagnostic.read_collision_snapshot_diagnostic, diagnostic.classify_snapshot,
         diagnostic.now_ns, diagnostic.collision_snapshot_payload) = original
        ready_log.unlink(missing_ok=True)


def main() -> int:
    plan_path = ROOT / "logs/t1/m2_exit_round2/20261016_174309_9/m2_round2_plan.json"
    stopped_plan = json.loads(plan_path.read_text(encoding="utf-8"))
    plan = copy.deepcopy(stopped_plan)
    catalog = batch.load_catalog()

    facts = batch.validate_final_139_resume(plan, plan_path.parent, catalog)
    assert facts["accounting"]["completed_valid_plan_items"] == 5
    assert facts["accounting"]["confirmed_mujoco_scene_starts"] == 6
    assert facts["accounting"]["confirmed_no_scene_orchestration_failures"] == 4
    assert facts["pending_sequences"] == list(range(6, 145))
    assert facts["pending_count"] == 139
    assert facts["budget"]["projected_scene_starts_after_additional"] == 145
    assert facts["accounting"]["duplicate_summary_sources_excluded"] == [
        "engineering_attempt_history", "engineering_retry_history"]
    amended = copy.deepcopy(plan)
    amended["scene_start_budget_policy"] = {"max_actual_scene_starts": 145}
    assert batch.validate_scene_budget(amended, additional_scene_starts=139)["projected_scene_starts_after_additional"] == 145
    try:
        batch.validate_scene_budget(amended, additional_scene_starts=140)
    except RuntimeError:
        pass
    else:
        raise AssertionError("scene budget allowed a 146th start")

    # Ledger semantics: confirmed pre-scene failures do not consume; unknown starts reserve;
    # duplicate history summaries do not add to either count.
    no_scene = copy.deepcopy(amended)
    no_scene.setdefault("pre_scene_start_attempts", []).append({"scene_started": False})
    no_scene["engineering_attempt_history"] = [{"scene_started": True}]
    before = batch.scene_budget_accounting(no_scene)
    assert before["confirmed_no_scene_orchestration_failures"] == facts["accounting"]["confirmed_no_scene_orchestration_failures"] + 1
    assert before["scene_budget_consumed_or_reserved"] == 6
    unknown = copy.deepcopy(no_scene)
    unknown["runs"][6]["attempts"] = [{"child_process_spawned": True, "scene_start_status": "UNKNOWN"}]
    reserved = batch.scene_budget_accounting(unknown)
    assert reserved["scene_starts_unknown_conservatively_reserved"] == 1
    assert reserved["scene_budget_consumed_or_reserved"] == 7

    with mock.patch.object(batch.subprocess, "Popen", side_effect=AssertionError("must not spawn")):
        try:
            batch.run_one(plan_path.parent, amended, amended["runs"][0], SimpleNamespace())
        except RuntimeError as error:
            assert "refusing duplicate or nonfresh run" in str(error)
        else:
            raise AssertionError("resume helper allowed a prior valid item to be rerun")
    authorized = copy.deepcopy(amended)
    authorized["continuation_code_freeze"] = {"offline_gate": "marker"}
    with mock.patch.object(batch, "code_freeze", side_effect=RuntimeError("reached runtime freeze gate")):
        try:
            batch.run_one(plan_path.parent, authorized, authorized["runs"][5], SimpleNamespace(),
                          authorized_engineering_retry=True, authorized_final_139_resume=True)
        except RuntimeError as error:
            assert "reached runtime freeze gate" in str(error)
        else:
            raise AssertionError("item 006 authorization did not reach the runtime freeze guard")

    # Exercise production argparse -> recovery validator -> dispatcher -> run_one ->
    # attempt_02 allocation. The injected Popen fails before creating any child process.
    with tempfile.TemporaryDirectory(prefix="t1-m2-main-entry-") as temporary:
        fixture_root = Path(temporary) / "20261016_174309_9"
        fixture_root.mkdir()
        (fixture_root / "runs").mkdir()
        (fixture_root / batch.PLAN_NAME).write_text(
            json.dumps(stopped_plan, indent=2), encoding="utf-8")
        preflight_source = plan_path.parent / "glfw_preflight_final_139.json"
        preflight_target = fixture_root / "glfw_preflight_scene_budget_resume.json"
        preflight_target.write_bytes(preflight_source.read_bytes())
        spawned = []
        def no_child(command, **kwargs):
            spawned.append((command, kwargs))
            raise OSError("mock: child process intentionally not created")
        exit_code = batch.main([
            "--batch-root", str(fixture_root), "--resume", "--authorized-final-139-resume",
            "--display", ":0", "--xauthority", "/run/user/1000/gdm/Xauthority",
            "--plot-python", "/home/lidio/anaconda3/bin/python3"],
            test_hooks={"allow_fixture_root": True, "popen_factory": no_child})
        assert exit_code == 1
        assert len(spawned) == 1
        command = spawned[0][0]
        assert command[command.index("--groups") + 1] == "B"
        assert command[command.index("--scene") + 1] == "m2dev06.xml"
        assert command[command.index("--exit-threshold") + 1] == "-0.12"
        assert command[command.index("--goal-x") + 1] == "7.5"
        assert command[command.index("--prepared-overlay") + 1].endswith("B_-0.12")
        temp_plan = json.loads((fixture_root / batch.PLAN_NAME).read_text(encoding="utf-8"))
        item6 = temp_plan["runs"][5]
        assert [attempt["attempt_number"] for attempt in item6["attempts"]] == [1, 2]
        assert item6["attempts"][1]["scene_start_status"] == "CONFIRMED_NOT_STARTED"
        assert item6["attempts"][1]["child_process_spawned"] is False
        assert (fixture_root / "runs" / item6["plan_item_id"] / "attempt_02").is_dir()
        assert not (fixture_root / "runs" / item6["plan_item_id"] / "attempt_01").exists()
        assert temp_plan["runs"][0]["attempts"] == stopped_plan["runs"][0]["attempts"]
        assert temp_plan["scene_start_budget_policy"]["max_actual_scene_starts"] == 145
        assert batch.scene_budget_accounting(temp_plan)["confirmed_mujoco_scene_starts"] == 6
        assert batch.scene_budget_accounting(temp_plan)["confirmed_no_scene_orchestration_failures"] == 5
        reentry_facts = batch.validate_final_139_resume(temp_plan, fixture_root, catalog,
                                                         allow_fixture_root=True)
        assert reentry_facts["stop_sequence"] == 6
        assert reentry_facts["new_confirmed_no_scene_failures"] == 1
        repeated = copy.deepcopy(temp_plan)
        repeated["runs"][5]["attempts"].append({"attempt_number": 3,
            "status": "STOPPED_FAILURE", "scene_start_status": "CONFIRMED_NOT_STARTED",
            "child_process_spawned": False,
            "started_at": "2099-01-01T00:00:00+00:00",
            "failure_signature": item6["attempts"][1]["failure_signature"],
            "failure_reasons": ["mock: child process intentionally not created"]})
        try:
            batch.validate_final_139_resume(repeated, fixture_root, catalog, allow_fixture_root=True)
        except RuntimeError as error:
            assert "same pre-scene orchestration failure has repeated" in str(error)
        else:
            raise AssertionError("identical second pre-scene failure was not rejected")

    with tempfile.TemporaryDirectory(prefix="t1-m2-attempt-path-") as temporary:
        item_root = Path(temporary) / "006_r1_m2d06_x12"
        assert batch.allocate_attempt_directory(item_root, 1) == item_root
        assert item_root.is_dir() and not list(item_root.iterdir())
        retry = batch.allocate_attempt_directory(item_root, 2)
        assert retry == item_root / "attempt_02" and retry.is_dir()
        try:
            batch.allocate_attempt_directory(item_root, 2)
        except FileExistsError:
            pass
        else:
            raise AssertionError("attempt allocation overwrote an existing directory")

    valid = snap(10, 1.0)
    assert diagnostic.standing_physics_sample_failure(valid) is None
    assert "physics_coverage_complete" in diagnostic.standing_physics_sample_failure(
        snap(12, 1.1, coverage=0))
    assert "unknown_contacts" in diagnostic.standing_physics_sample_failure(
        snap(14, 1.2, unknown=1))
    assert "sim_time_nonfinite" in diagnostic.standing_physics_sample_failure(
        snap(16, float("nan")))

    stable = run_monitor([snap(10, 1.0), snap(12, 1.25), snap(14, 1.51)])
    assert stable["status"] == "STABLE"
    assert stable["stable_elapsed_sim_s"] >= 0.50
    assert stable["last_collision_snapshot"]["physics_coverage_complete"] is True
    assert stable["observations"][-1]["physics_diagnostic"]["sample_age_ns"] >= 0

    invalid_coverage = run_monitor([snap(20, 2.0, coverage=0)], cap=0.025)
    assert invalid_coverage["status"] == "PREPARATION_FAILURE"
    assert "physics_coverage_complete" in invalid_coverage["first_rejected_sample"]["reason"]
    assert invalid_coverage["last_rejected_sample"]["physics_diagnostic"]["physics_coverage_complete"] == 0

    unknown = run_monitor([snap(30, 3.0, unknown=1)], cap=0.025)
    assert unknown["status"] == "PREPARATION_FAILURE"
    assert "unknown_contacts" in unknown["first_rejected_sample"]["reason"]
    assert unknown["last_rejected_sample"]["physics_diagnostic"]["unknown_contacts"] == 1

    not_live = run_monitor([snap(40, 4.0)], statuses=[diagnostic.CollisionStatus.INVALID])
    assert not_live["status"] == "PREPARATION_FAILURE"
    assert not_live["physics_failure"]["snapshot_status"] == "INVALID"
    assert not_live["physics_failure"]["expected_capture_id"] == "capture-test"
    print("PASS: production pre-RL checks, scene-start accounting, 145/146 budget boundary, and full main-entry mock through item 006 attempt_02 allocation")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
