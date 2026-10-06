#!/usr/bin/env python3
"""Filesystem-layout tests for the fail-closed P1-10 comparison CLI."""

from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import sys

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

import p1_10_saved_record_compare as comparator  # noqa: E402
from p1_08_baseline_capture import write_p1_10_context  # noqa: E402


PAIR_REL = Path("docs/evidence/P1-10/replay_pair_20260903_saved_record_closure")
SOURCE_CONTEXT = REPO / "docs/evidence/P1-10/replay_pair_20260903/scenario_resolved_manifest.json"


def _write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _payload(index: int, *, session_id: int = 900, numeric_delta: float = 0.0) -> dict:
    policy_state = 1 if index == 1 else 0
    return {
        "session_id": session_id, "source_sequence": 2 + index * 2,
        "rl_step": index + 1, "ray_age_ns": 10 + index,
        "monotonic_ns": 1000 + index * 10, "source": 1,
        "controller_active": 1, "rl_entered": 1, "rl_active": 1,
        "safety_faulted": 0, "policy_state": policy_state,
        "policy_state_name": "RECOVERY" if policy_state else "AGILE",
        "ray_origin": 1, "ray_valid": 1, "collision_origin": 0,
        "torque_saturated_computed": 0, "ra_value": -0.2,
        "lin_vel": [0.1 + index, 0.2, 0.0],
        "command": [0.5, 0.0, 0.0],
        "world_pose": [1.0 + index + (numeric_delta if index == 2 else 0.0), 0.0, 0.01 * index],
        "ray2d": [2.0 + index] * 11, "action_raw": [0.1 + index] * 12,
        "action_clipped": [0.2 + index] * 12, "joint_target_rad": [0.3 + index] * 12,
        "torque_nm": [0.4 + index] * 12, "torque_saturated": [0.0] * 12,
    }


def _write_record(path: Path, *, run_id: str, session_id: int = 900, numeric_delta: float = 0.0) -> None:
    frames = []
    for index in range(3):
        frames.append({
            "kind": "frame", "run_id": run_id, "status": "LIVE",
            "recorded_at_ns": 1001 + index,
            "payload": _payload(index, session_id=session_id, numeric_delta=numeric_delta),
            "availability": {
                "session_id": True, "source_sequence": True, "rl_step": True,
                "ray_age_ns": True, "monotonic_ns": True, "ra_value": True,
                "lin_vel": True, "command": True, "world_pose": True,
                "ray2d": True, "action_raw": True, "action_clipped": True,
                "joint_target_rad": True, "torque_nm": True,
                "torque_saturated": False, "collision": False,
            },
        })
    lines = [
        {"kind": "meta", "record_format_version": 2, "run_id": run_id,
         "source": "/dev/shm/mujoco_rt_frame", "created_at_ns": 1},
        *frames,
        {"kind": "terminal", "run_id": run_id, "frames_observed": 3,
         "first_frame_time_ns": 1000, "last_frame_time_ns": 1020, "duration_ns": 20,
         "last_session_id": session_id, "process_exit_code": 0,
         "forced_termination": False, "shutdown_complete": True,
         "shutdown_request_source": "SIGINT", "normal_shutdown": True,
         "termination_reason": "FRAMES_ENDED_RC0", "safety_fault_last": False,
         "safety_fault_seen": False, "reached_goal": "UNKNOWN",
         "timeout": "UNKNOWN", "collision_events": "UNKNOWN",
         "fall_events": "UNKNOWN", "fact_validation_errors": []},
    ]
    path.write_text("".join(json.dumps(line, sort_keys=True) + "\n" for line in lines), encoding="utf-8")


def _write_process_facts(path: Path, *, run_id: str) -> None:
    def child(pid: int) -> dict:
        return {
            "exit_code": 0, "not_launched": False, "escalated": False,
            "pid": pid, "pgid": pid, "signals": [{"signal": "SIGINT", "delivered": True}],
            "cleanup_errors": [],
        }

    context = json.loads(SOURCE_CONTEXT.read_text(encoding="utf-8"))
    _write_json(path, {
        "run_id": run_id, "scene": "scene_flat.xml", "exit_code": 0,
        "shutdown_complete": True, "forced_termination": False,
        "shutdown_request_source": "SIGINT", "cleanup_error_count": 0,
        "p1_10_context": context, "child.mujoco": child(101),
        "child.ros2_launch": child(202),
    })


def _make_layout(root: Path) -> tuple[Path, Path, Path]:
    repo = root / "repo"
    pair_dir = repo / PAIR_REL
    pair_dir.mkdir(parents=True)
    pair = json.loads((REPO / PAIR_REL / "pair_manifest.json").read_text(encoding="utf-8"))
    _write_json(pair_dir / "pair_manifest.json", pair)
    for label, run_id, session_id in (("run_A", "run-a", 900), ("run_B", "run-b", 901)):
        run_dir = pair_dir / label
        run_dir.mkdir()
        context = json.loads(SOURCE_CONTEXT.read_text(encoding="utf-8"))
        _write_json(run_dir / "scenario_resolved_manifest.json", context)
        write_p1_10_context(run_dir / "p1_10_context.json", context)
        _write_process_facts(run_dir / "process_facts.json", run_id=run_id)
        _write_record(run_dir / "runtime_record.jsonl", run_id=run_id, session_id=session_id)
    return repo, pair_dir, pair_dir / "run_A"


class SavedRecordComparatorFilesystemTests(unittest.TestCase):
    def _run(self, repo: Path, pair_dir: Path) -> int:
        with mock.patch.object(comparator, "REPO", repo):
            return comparator.main(["--pair-dir", str(pair_dir)])

    def test_valid_real_pair_layout_passes_and_outputs_are_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo, pair_dir, _ = _make_layout(Path(tmp))
            self.assertEqual(self._run(repo, pair_dir), 0)
            for name in (
                "canonical_identity_input.json", "canonical_identity_output.json",
                "diff_report.json", "saved_record_comparison_report.md",
            ):
                self.assertTrue((pair_dir / name).is_file(), name)

    def test_production_context_writer_to_comparator_passes(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo, pair_dir, _ = _make_layout(Path(tmp))
            context = json.loads((pair_dir / "run_A/p1_10_context.json").read_text(encoding="utf-8"))
            self.assertEqual(context["scene"]["root_xml_sha256"], comparator.EXPECTED_BINDING["scene_root_sha256"])
            self.assertEqual(context["scene"]["model_closure_sha256"], comparator.EXPECTED_BINDING["model_closure_sha256"])
            self.assertEqual(self._run(repo, pair_dir), 0)

    def test_excluded_metadata_differences_pass(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo, pair_dir, _ = _make_layout(Path(tmp))
            _write_record(pair_dir / "run_B/runtime_record.jsonl", run_id="run-b", session_id=901)
            self.assertEqual(self._run(repo, pair_dir), 0)

    def test_numeric_difference_fails_and_reports_diagnostics(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo, pair_dir, _ = _make_layout(Path(tmp))
            _write_record(pair_dir / "run_B/runtime_record.jsonl", run_id="run-b", session_id=901, numeric_delta=0.25)
            self.assertEqual(self._run(repo, pair_dir), 1)
            report = json.loads((pair_dir / "diff_report.json").read_text(encoding="utf-8"))
            self.assertGreater(report["difference_count"], 0)
            self.assertTrue(any(item["nonzero_count"] for item in report["numeric_diagnostics"]))

    def test_process_facts_unknown_none_and_semantic_failures_reject(self):
        mutations = {
            "none": lambda facts: facts.update(exit_code=None),
            "unknown": lambda facts: facts.update(exit_code="UNKNOWN"),
            "nonzero": lambda facts: facts.update(exit_code=3),
            "shutdown_incomplete": lambda facts: facts.update(shutdown_complete=False),
            "forced": lambda facts: facts.update(forced_termination=True),
            "bad_source": lambda facts: facts.update(shutdown_request_source="SIGTERM"),
            "child_wait": lambda facts: facts["child.mujoco"].update(exit_code=7),
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as tmp:
                repo, pair_dir, run_a = _make_layout(Path(tmp))
                facts_path = run_a / "process_facts.json"
                facts = json.loads(facts_path.read_text(encoding="utf-8"))
                mutate(facts)
                _write_json(facts_path, facts)
                self.assertEqual(self._run(repo, pair_dir), 2)

    def test_terminal_missing_duplicate_position_and_semantics_reject(self):
        for mode in ("missing_field", "duplicate", "not_final", "normal_false", "forced", "bad_source"):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as tmp:
                repo, pair_dir, run_a = _make_layout(Path(tmp))
                record_path = run_a / "runtime_record.jsonl"
                lines = [json.loads(line) for line in record_path.read_text().splitlines()]
                if mode == "missing_field":
                    del lines[-1]["normal_shutdown"]
                elif mode == "duplicate":
                    lines.insert(-1, copy.deepcopy(lines[-1]))
                elif mode == "not_final":
                    lines.append({"kind": "frame", "run_id": "run-a", "status": "MISSING", "payload": None, "availability": None})
                elif mode == "normal_false":
                    lines[-1]["normal_shutdown"] = False
                elif mode == "forced":
                    lines[-1]["forced_termination"] = True
                else:
                    lines[-1]["shutdown_request_source"] = "SIGTERM"
                record_path.write_text("".join(json.dumps(line) + "\n" for line in lines), encoding="utf-8")
                self.assertEqual(self._run(repo, pair_dir), 2)

    def test_both_runs_with_invalid_termination_reason_reject(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo, pair_dir, _ = _make_layout(Path(tmp))
            for label in ("run_A", "run_B"):
                path = pair_dir / label / "runtime_record.jsonl"
                lines = [json.loads(line) for line in path.read_text().splitlines()]
                lines[-1]["termination_reason"] = "NOT_A_RECORDER_REASON"
                path.write_text("".join(json.dumps(line) + "\n" for line in lines), encoding="utf-8")
            self.assertEqual(self._run(repo, pair_dir), 2)

    def test_terminal_domain_type_and_value_rejections(self):
        mutations = {
            "termination_reason_type": ("termination_reason", 7),
            "termination_reason_value": ("termination_reason", "NOT_A_RECORDER_REASON"),
            "safety_fault_seen_type": ("safety_fault_seen", "false"),
            "safety_fault_seen_value": ("safety_fault_seen", 0),
            "safety_fault_last_type": ("safety_fault_last", None),
            "safety_fault_last_value": ("safety_fault_last", 1),
            "reached_goal_type": ("reached_goal", True),
            "reached_goal_value": ("reached_goal", "FALSE"),
            "timeout_type": ("timeout", 0),
            "timeout_value": ("timeout", "FALSE"),
            "collision_events_type": ("collision_events", False),
            "collision_events_value": ("collision_events", "0"),
            "fall_events_type": ("fall_events", 0),
            "fall_events_value": ("fall_events", "FALSE"),
        }
        for name, (field, value) in mutations.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as tmp:
                repo, pair_dir, run_a = _make_layout(Path(tmp))
                path = run_a / "runtime_record.jsonl"
                lines = [json.loads(line) for line in path.read_text().splitlines()]
                lines[-1][field] = value
                path.write_text("".join(json.dumps(line) + "\n" for line in lines), encoding="utf-8")
                self.assertEqual(self._run(repo, pair_dir), 2)

    def test_context_drift_rejects(self):
        mutations = {
            "scenario": ("scenario_id", "other"),
            "variant": ("variant_binding", {"label": "paper-faithful"}),
            "baseline": ("baseline", {"manifest_sha256": "drift"}),
            "seed": ("seeds", {"root_seed": 9}),
            "window": ("launch_contract", {"window_s": 24.0}),
            "initial_state": ("initial_state_source", {"kind": "keyframe0"}),
        }
        for name, (key, value) in mutations.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as tmp:
                repo, pair_dir, run_a = _make_layout(Path(tmp))
                path = run_a / "p1_10_context.json"
                context = json.loads(path.read_text(encoding="utf-8"))
                if isinstance(value, dict):
                    context[key] = {**context[key], **value}
                else:
                    context[key] = value
                _write_json(path, context)
                self.assertEqual(self._run(repo, pair_dir), 2)

    def test_external_run_path_and_wrong_pair_dir_reject(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo, pair_dir, _ = _make_layout(Path(tmp))
            pair_path = pair_dir / "pair_manifest.json"
            pair = json.loads(pair_path.read_text(encoding="utf-8"))
            pair["planned_runs"]["run_a"]["directory"] = "../external"
            _write_json(pair_path, pair)
            self.assertEqual(self._run(repo, pair_dir), 2)
            wrong = repo / "docs/evidence/P1-10/wrong_pair"
            wrong.mkdir(parents=True)
            self.assertEqual(self._run(repo, wrong), 2)

    def test_pair_run_and_required_artifact_symlinks_reject(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo, pair_dir, run_a = _make_layout(Path(tmp))
            real_pair = pair_dir.with_name("real_pair")
            pair_dir.rename(real_pair)
            pair_dir.symlink_to(real_pair, target_is_directory=True)
            self.assertEqual(self._run(repo, pair_dir), 2)

        with tempfile.TemporaryDirectory() as tmp:
            repo, pair_dir, run_a = _make_layout(Path(tmp))
            real_run = pair_dir / "external_run_A"
            run_a.rename(real_run)
            run_a.symlink_to(real_run, target_is_directory=True)
            self.assertEqual(self._run(repo, pair_dir), 2)

        for filename in comparator.REQUIRED_RUN_FILES:
            with self.subTest(filename=filename), tempfile.TemporaryDirectory() as tmp:
                repo, pair_dir, run_a = _make_layout(Path(tmp))
                artifact = run_a / filename
                external = Path(tmp) / f"external_{filename.replace('/', '_')}"
                external.write_bytes(artifact.read_bytes())
                artifact.unlink()
                artifact.symlink_to(external)
                self.assertEqual(self._run(repo, pair_dir), 2)

    def test_missing_required_files_reject(self):
        for filename in comparator.REQUIRED_RUN_FILES:
            with self.subTest(filename=filename), tempfile.TemporaryDirectory() as tmp:
                repo, pair_dir, run_a = _make_layout(Path(tmp))
                (run_a / filename).unlink()
                self.assertEqual(self._run(repo, pair_dir), 2)

    def test_binding_missing_none_unknown_and_wrong_type_reject(self):
        mutations = {
            "missing": lambda context: context.pop("scenario_id"),
            "none": lambda context: context.update(scenario_id=None),
            "unknown": lambda context: context.update(scenario_id="UNKNOWN"),
            "wrong_type": lambda context: context.update(seeds={**context["seeds"], "root_seed": "20260902"}),
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as tmp:
                repo, pair_dir, run_a = _make_layout(Path(tmp))
                path = run_a / "p1_10_context.json"
                context = json.loads(path.read_text(encoding="utf-8"))
                mutate(context)
                _write_json(path, context)
                self.assertEqual(self._run(repo, pair_dir), 2)

    def test_full_binding_fields_missing_or_invalid_reject(self):
        fields = {
            "scene_root_sha256": ("scene", "root_xml_sha256"),
            "model_closure_sha256": ("scene", "model_closure_sha256"),
            "initial_state_qpos_sha256": ("initial_state", "qpos_sha256"),
        }
        for field, (section, key) in fields.items():
            for mode, value in (
                ("none", None), ("unknown", "UNKNOWN"), ("wrong_type", 123),
                ("wrong_hash", "0" * 64),
            ):
                with self.subTest(field=field, mode=mode), tempfile.TemporaryDirectory() as tmp:
                    repo, pair_dir, run_a = _make_layout(Path(tmp))
                    path = run_a / "p1_10_context.json"
                    context = json.loads(path.read_text(encoding="utf-8"))
                    context[section][key] = value
                    _write_json(path, context)
                    self.assertEqual(self._run(repo, pair_dir), 2)
            with self.subTest(field=field, mode="missing"), tempfile.TemporaryDirectory() as tmp:
                repo, pair_dir, run_a = _make_layout(Path(tmp))
                path = run_a / "p1_10_context.json"
                context = json.loads(path.read_text(encoding="utf-8"))
                context[section].pop(key)
                _write_json(path, context)
                self.assertEqual(self._run(repo, pair_dir), 2)

    def test_process_facts_binding_missing_is_not_filled_from_other_context(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo, pair_dir, run_a = _make_layout(Path(tmp))
            facts_path = run_a / "process_facts.json"
            facts = json.loads(facts_path.read_text(encoding="utf-8"))
            facts["p1_10_context"].pop("scenario_id")
            _write_json(facts_path, facts)
            self.assertEqual(self._run(repo, pair_dir), 2)

    def test_required_json_parse_failure_rejects(self):
        for filename in ("process_facts.json", "scenario_resolved_manifest.json", "p1_10_context.json"):
            with self.subTest(filename=filename), tempfile.TemporaryDirectory() as tmp:
                repo, pair_dir, run_a = _make_layout(Path(tmp))
                (run_a / filename).write_text("{not-json}\n", encoding="utf-8")
                self.assertEqual(self._run(repo, pair_dir), 2)

    def test_output_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            repo, pair_dir, _ = _make_layout(Path(tmp))
            self.assertEqual(self._run(repo, pair_dir), 0)
            before = (pair_dir / "canonical_identity_output.json").read_bytes()
            self.assertEqual(self._run(repo, pair_dir), 2)
            self.assertEqual((pair_dir / "canonical_identity_output.json").read_bytes(), before)


if __name__ == "__main__":
    unittest.main(verbosity=2)
