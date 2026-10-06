#!/usr/bin/env python3
"""Offline fail-closed rejection tests for the P1-09 runtime → P1-02 binding.

Covers: source forgery / synthetic / legacy / non-LIVE frame, session change,
sequence/time rollback, duplicate / misplaced terminal, missing facts,
orchestrator-facts vs record-terminal contradiction, and safety-fault never
becomes SUCCESS. The positive case writes a formal run whose P1-02 validator
verdict is INVALID (missing authoritative data sources), never VALID/SUCCESS.
"""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from typing import Any, Dict, List

from abs_rt_frame import RuntimeFrame
from formal_runtime_binding import BindingError, bind_runtime_record
from run_record import frame_availability, frame_payload

_64 = "a" * 64


def make_payload(
    sequence: int,
    rl_step: int,
    monotonic: int,
    session: int = 7,
    source: int = 1,
    policy_state: int = 0,
    safety_faulted: int = 0,
) -> Dict[str, Any]:
    frame = RuntimeFrame(
        magic=0x414253525446524D, version=1, sequence=sequence,
        monotonic_ns=monotonic, session_id=session, rl_step=rl_step, ray_age_ns=1,
        source=source, controller_active=1, rl_entered=1, rl_active=1,
        safety_faulted=safety_faulted, policy_state=policy_state, ray_origin=1,
        ray_valid=1, collision_origin=0, torque_saturated_computed=0, reserved_pad=0,
        ra_value=-0.9, lin_vel=(0.3, 0.2, 0.0), command=(0.5, 0.5, 0.1),
        world_pose=(1.0, 2.0, 0.3), ray2d=tuple(float(i) for i in range(11)),
        action_raw=tuple(float(i) for i in range(12)),
        action_clipped=tuple(float(i) for i in range(12)),
        joint_target_rad=tuple(float(i) for i in range(12)),
        torque_nm=tuple(float(i) for i in range(12)),
        torque_saturated=tuple(0.0 for _ in range(12)),
    )
    return frame_payload(frame)


def live_line(payload: Dict[str, Any], run_id: str) -> Dict[str, Any]:
    return {"kind": "frame", "run_id": run_id, "status": "LIVE",
            "recorded_at_ns": payload["monotonic_ns"] + 1, "payload": payload,
            "availability": frame_availability(_to_frame(payload))}


def _to_frame(payload: Dict[str, Any]) -> RuntimeFrame:
    return RuntimeFrame(
        magic=0x414253525446524D, version=1, sequence=payload["source_sequence"],
        monotonic_ns=payload["monotonic_ns"], session_id=payload["session_id"],
        rl_step=payload["rl_step"], ray_age_ns=payload["ray_age_ns"],
        source=payload["source"], controller_active=payload["controller_active"],
        rl_entered=payload["rl_entered"], rl_active=payload["rl_active"],
        safety_faulted=payload["safety_faulted"], policy_state=payload["policy_state"],
        ray_origin=payload["ray_origin"], ray_valid=payload["ray_valid"],
        collision_origin=payload["collision_origin"],
        torque_saturated_computed=payload["torque_saturated_computed"], reserved_pad=0,
        ra_value=payload["ra_value"], lin_vel=tuple(payload["lin_vel"]),
        command=tuple(payload["command"]), world_pose=tuple(payload["world_pose"]),
        ray2d=tuple(payload["ray2d"]), action_raw=tuple(payload["action_raw"]),
        action_clipped=tuple(payload["action_clipped"]),
        joint_target_rad=tuple(payload["joint_target_rad"]),
        torque_nm=tuple(payload["torque_nm"]), torque_saturated=tuple(payload["torque_saturated"]),
    )


def terminal(run_id: str, *, first_ns: int = 100, last_ns: int = 120, frames: int = 3,
             session: int = 7, process_exit_code: int = 0, forced_termination: bool = False,
             shutdown_complete: bool = True, shutdown_request_source: str = "SIGINT",
             safety_fault_seen: bool = False, **overrides: Any) -> Dict[str, Any]:
    base: Dict[str, Any] = {
        "kind": "terminal", "run_id": run_id, "frames_observed": frames,
        "first_frame_time_ns": first_ns, "last_frame_time_ns": last_ns,
        "duration_ns": last_ns - first_ns, "last_session_id": session,
        "last_policy_state": "AGILE", "safety_fault_seen": safety_fault_seen,
        "safety_fault_last": safety_fault_seen, "last_ra_value": -0.9,
        "simulation_time_s": "UNKNOWN", "simulation_time_s_reason": "x",
        "reached_goal": "UNKNOWN", "reached_goal_reason": "x",
        "timeout": "UNKNOWN", "timeout_reason": "x",
        "collision_events": "UNKNOWN", "collision_events_reason": "x",
        "fall_events": "UNKNOWN", "fall_events_reason": "x",
        "process_exit_code": process_exit_code, "forced_termination": forced_termination,
        "shutdown_request_source": shutdown_request_source,
        "shutdown_complete": shutdown_complete, "fact_validation_errors": [],
        "normal_shutdown": True, "termination_reason": "FRAMES_ENDED_RC0",
    }
    base.update(overrides)
    return base


def write_record(path: Path, lines: List[Dict[str, Any]]) -> Path:
    path.write_text("".join(json.dumps(line, sort_keys=True) + "\n" for line in lines), encoding="utf-8")
    return path


def valid_record(path: Path, run_id: str = "run_test") -> Path:
    """A record that is VALID in its own chain (3 LIVE authoritative frames)."""
    p1 = make_payload(2, 1, 100)
    p2 = make_payload(4, 2, 110)
    p3 = make_payload(6, 3, 120)
    lines = [
        {"kind": "meta", "record_format_version": 2, "run_id": run_id,
         "source": "/dev/shm/mujoco_rt_frame", "created_at_ns": 1},
        live_line(p1, run_id), live_line(p2, run_id), live_line(p3, run_id),
        terminal(run_id, first_ns=100, last_ns=120),
    ]
    return write_record(path, lines)


def minimal_context() -> Dict[str, Any]:
    return {
        "created_at": "2026-08-30T00:00:00+00:00",
        "variant": "stabilized",
        "git": {"commit": _64, "branch": "test", "dirty_state": "clean"},
        "models": {
            "agile_policy": {"path": "/m/a.pt", "sha256": _64, "source_provenance": "UNKNOWN"},
            "ra_value": {"path": "/m/r.pt", "sha256": _64, "source_provenance": "UNKNOWN"},
            "recovery_policy": {"path": "/m/rec.pt", "sha256": _64, "source_provenance": "UNKNOWN"},
        },
        "effective_config": {"path": "/cfg.yaml", "sha256": _64},
        "environment": {
            "mujoco_binary_path": "/bin", "mujoco_binary_sha256": _64,
            "mujoco_version": "3.3.3", "timestep_s": 0.002, "solver": "Newton",
            "go2_mjcf_path": "/g.xml", "go2_mjcf_sha256": _64,
            "go2_assets_sha256": _64, "hardware_mode": "simulation",
        },
        "scenario": {"id": "scene_flat", "path": "/s.xml", "sha256": _64,
                     "metadata": {"schema": "v1", "obstacle_count": 0}},
        "seeds": {"root_seed": 1, "sources": {"scene_generator": 1, "controller_goal": 1,
                                               "perception": 1, "evaluator": 1}},
        "perception": {"source": "mujoco_ray2d", "version": "1", "sha256": _64,
                       "frame_contract_version": "1"},
        "rates_hz": {"controller": 200.0, "pd": 500.0, "policy": 500.0, "ra": 500.0, "perception": 50.0},
        "thresholds": {"arrival_region_m": 1.0, "arrival_hold_s": 0.5, "fall_height_m": 0.35,
                       "fall_angle_rad": 1.3, "collision_definition_id": "c1",
                       "ra_entry_threshold": -0.05, "ra_exit_threshold": -0.05},
    }


class BindingRejectionTests(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = Path(self._tmp.name)

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def _bind(self, record_path: Path, facts: Dict[str, Any] | None = None,
              context: Dict[str, Any] | None = None):
        facts_path = None
        if facts is not None:
            facts_path = self.dir / "facts.json"
            facts_path.write_text(json.dumps(facts), encoding="utf-8")
        return bind_runtime_record(str(record_path), self.dir / "run", context or minimal_context(),
                                   facts_path=str(facts_path) if facts_path else None)

    # ------------------------------------------------------------- positive
    def test_valid_record_writes_formal_invalid_verdict(self):
        record = valid_record(self.dir / "ok.jsonl")
        verdict = self._bind(record)
        self.assertEqual(verdict["episode_state"], "INVALID")
        self.assertTrue(verdict["validator_completed"])
        self.assertNotIn("SUCCESS", str(verdict["reasons"]))
        self.assertIn("non_numeric_telemetry_clock", verdict["reasons"])
        self.assertEqual(verdict["runtime_live_frames"], 3)
        self.assertEqual(verdict["runtime_session_id"], 7)
        # summary is never SUCCESS and terminal outcome is UNKNOWN
        summary = json.loads((self.dir / "run" / "summary.json").read_text())
        self.assertNotEqual(summary["terminal_outcome"], "SUCCESS")
        self.assertEqual(summary["validity"], "INVALID")
        self.assertEqual(summary["run_id"], verdict["formal_run_id"])

    # ------------------------------------------------------ source forgery
    def test_synthetic_frame_rejected(self):
        record = self.dir / "syn.jsonl"
        p1 = make_payload(2, 1, 100, source=2)  # SYNTHETIC_TEST
        lines = [{"kind": "meta", "record_format_version": 2, "run_id": "r",
                  "source": "/dev/shm/mujoco_rt_frame", "created_at_ns": 1},
                 live_line(p1, "r"), terminal("r", first_ns=100, last_ns=100)]
        write_record(record, lines)
        with self.assertRaises(BindingError):
            self._bind(record)

    def test_legacy_frame_rejected(self):
        record = self.dir / "leg.jsonl"
        p1 = make_payload(2, 1, 100, source=3)  # LEGACY_ONLY
        lines = [{"kind": "meta", "record_format_version": 2, "run_id": "r",
                  "source": "/dev/shm/mujoco_rt_frame", "created_at_ns": 1},
                 live_line(p1, "r"), terminal("r", first_ns=100, last_ns=100)]
        write_record(record, lines)
        with self.assertRaises(BindingError):
            self._bind(record)

    def test_non_live_frame_rejected(self):
        record = self.dir / "nl.jsonl"
        p1 = make_payload(2, 1, 100)
        lines = [{"kind": "meta", "record_format_version": 2, "run_id": "r",
                  "source": "/dev/shm/mujoco_rt_frame", "created_at_ns": 1},
                 {"kind": "frame", "run_id": "r", "status": "INVALID", "payload": None, "availability": None},
                 terminal("r", first_ns=100, last_ns=100)]
        write_record(record, lines)
        with self.assertRaises(BindingError):
            self._bind(record)

    # ------------------------------------------------------ continuity
    def test_session_change_rejected(self):
        record = self.dir / "sess.jsonl"
        p1 = make_payload(2, 1, 100, session=7)
        p2 = make_payload(4, 2, 110, session=8)
        lines = [{"kind": "meta", "record_format_version": 2, "run_id": "r",
                  "source": "/dev/shm/mujoco_rt_frame", "created_at_ns": 1},
                 live_line(p1, "r"), live_line(p2, "r"),
                 terminal("r", first_ns=100, last_ns=110)]
        write_record(record, lines)
        with self.assertRaises(BindingError):
            self._bind(record)

    def test_sequence_rollback_rejected(self):
        record = self.dir / "seq.jsonl"
        p1 = make_payload(6, 1, 100)
        p2 = make_payload(4, 2, 110)  # sequence decreases
        lines = [{"kind": "meta", "record_format_version": 2, "run_id": "r",
                  "source": "/dev/shm/mujoco_rt_frame", "created_at_ns": 1},
                 live_line(p1, "r"), live_line(p2, "r"),
                 terminal("r", first_ns=100, last_ns=110)]
        write_record(record, lines)
        with self.assertRaises(BindingError):
            self._bind(record)

    def test_monotonic_rollback_rejected(self):
        record = self.dir / "mono.jsonl"
        p1 = make_payload(2, 1, 120)
        p2 = make_payload(4, 2, 110)  # monotonic decreases
        lines = [{"kind": "meta", "record_format_version": 2, "run_id": "r",
                  "source": "/dev/shm/mujoco_rt_frame", "created_at_ns": 1},
                 live_line(p1, "r"), live_line(p2, "r"),
                 terminal("r", first_ns=110, last_ns=120)]
        write_record(record, lines)
        with self.assertRaises(BindingError):
            self._bind(record)

    # ------------------------------------------------------ terminal misuse
    def test_duplicate_terminal_rejected(self):
        record = self.dir / "dup.jsonl"
        p1 = make_payload(2, 1, 100)
        lines = [{"kind": "meta", "record_format_version": 2, "run_id": "r",
                  "source": "/dev/shm/mujoco_rt_frame", "created_at_ns": 1},
                 live_line(p1, "r"), terminal("r", first_ns=100, last_ns=100),
                 terminal("r", first_ns=100, last_ns=100)]
        write_record(record, lines)
        with self.assertRaises(BindingError):
            self._bind(record)

    def test_misplaced_terminal_rejected(self):
        record = self.dir / "mis.jsonl"
        p1 = make_payload(2, 1, 100)
        p2 = make_payload(4, 2, 110)
        lines = [{"kind": "meta", "record_format_version": 2, "run_id": "r",
                  "source": "/dev/shm/mujoco_rt_frame", "created_at_ns": 1},
                 live_line(p1, "r"), terminal("r", first_ns=100, last_ns=100),
                 live_line(p2, "r")]
        write_record(record, lines)
        with self.assertRaises(BindingError):
            self._bind(record)

    # ------------------------------------------------------ facts
    def test_missing_facts_rejected(self):
        record = self.dir / "misfacts.jsonl"
        p1 = make_payload(2, 1, 100)
        lines = [{"kind": "meta", "record_format_version": 2, "run_id": "r",
                  "source": "/dev/shm/mujoco_rt_frame", "created_at_ns": 1},
                 live_line(p1, "r"),
                 terminal("r", first_ns=100, last_ns=100, process_exit_code=None)]
        write_record(record, lines)
        with self.assertRaises(BindingError):
            self._bind(record)

    def test_facts_contradiction_rejected(self):
        record = valid_record(self.dir / "contra.jsonl")
        # facts sidecar says forced_termination=True; record terminal says False
        with self.assertRaises(BindingError):
            self._bind(record, facts={"forced_termination": True})

    # ------------------------------------------------------ safety never SUCCESS
    def test_safety_fault_never_success(self):
        record = self.dir / "fault.jsonl"
        p1 = make_payload(2, 1, 100, safety_faulted=1)
        p2 = make_payload(4, 2, 110)
        p3 = make_payload(6, 3, 120)
        lines = [{"kind": "meta", "record_format_version": 2, "run_id": "r",
                  "source": "/dev/shm/mujoco_rt_frame", "created_at_ns": 1},
                 live_line(p1, "r"), live_line(p2, "r"), live_line(p3, "r"),
                 terminal("r", first_ns=100, last_ns=120, safety_fault_seen=True)]
        write_record(record, lines)
        verdict = self._bind(record)
        self.assertEqual(verdict["episode_state"], "INVALID")
        summary = json.loads((self.dir / "run" / "summary.json").read_text())
        self.assertNotEqual(summary["terminal_outcome"], "SUCCESS")
        self.assertEqual(summary["terminal_outcome"], "UNKNOWN")


if __name__ == "__main__":
    unittest.main()
