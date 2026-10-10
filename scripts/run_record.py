#!/usr/bin/env python3
"""Per-run raw record chain for the real-time runtime frame (P1-09 runtime record).

This module is the data-chain completion for P1-09: it continuously saves the
FULL payload of the authoritative runtime frame (``/dev/shm/mujoco_rt_frame``,
produced by ``StateRL::writeRtFrame``) into one per-run record, then computes a
post-run summary from that saved record only.

It is deliberately NOT the P1-02 ``FormalRunWriter`` chain:

- ``formal_rt_frame_recorder.py`` and ``formal_experiment_contract.py`` keep
  their reviewed fail-closed boundaries. They refuse to fabricate formal
  telemetry/summary fields whose authority is missing, and this module does not
  bypass them.
- This record is a RAW, self-describing per-run archive. It stores whatever the
  frame actually contained plus an explicit ``availability`` map derived from the
  controller's own flags. A field the controller does not compute is recorded as
  unavailable/UNKNOWN, never as a fabricated number.
- No mock/default/synthetic value is used to fill a field. ``UNKNOWN``/``None``
  are explicit and must appear verbatim in the record.

Fail-closed record validity (Reviewer REJECT fix):

- The whole record is VALID only when EVERY present frame line is LIVE and
  AUTHORITATIVE_RUNTIME. Any INVALID / SYNTHETIC / LEGACY / UNKNOWN_ORIGIN /
  STALE / malformed / non-authoritative frame invalidates the entire record;
  bad frames are never filtered out to keep a VALID label.
- ``MISSING`` lines (the shared-memory file held no frame at that poll) are
  recorded gaps, not bad frames; they do not by themselves invalidate. They
  still contribute no payload.
- Cross-frame continuity is enforced across payload-bearing frame lines:
  ``session_id`` unchanged, ``source_sequence`` / ``rl_step`` /
  ``monotonic_ns`` strictly increasing. Any violation invalidates the record,
  and a negative ``duration_ns`` invalidates it too.
- meta / frames / terminal must share one ``run_id``; the terminal line must be
  unique and be the final line of the record.
- A safety fault is ``safety_faulted`` OR ``policy_state == FAULTED``; it enters
  outcome precedence ahead of forced/nonzero-exit causes.
- Process facts (``exit_code``, ``forced_termination``, ``shutdown_complete``,
  ``shutdown_request_source``) are strictly type-checked at ``finalize``. No
  implicit bool conversion: the string ``"false"`` is never treated as True. A
  fact of an invalid type is recorded UNKNOWN and flags the record INVALID via
  ``fact_validation_errors``.

Data-origin boundary:

- A frame whose ``source == AUTHORITATIVE_RUNTIME`` is real runtime data.
  ``classify_frame`` (from ``abs_rt_frame``) is the single classifier; only a
  LIVE/STALE snapshot carries a payload, and the status is stored verbatim.
- Synthetic/legacy/unknown-origin fixtures can never be promoted: a record with
  any such frame is INVALID and is never a runtime outcome.

No MuJoCo, ROS2, benchmark, formal episode, pilot, or real robot is launched
here. The production loop only reads the fixed shared-memory path through the
existing reader.
"""

from __future__ import annotations

import json
import math
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

from abs_rt_frame import (
    COLLISION_UNAVAILABLE,
    DEFAULT_STALE_TIMEOUT_NS,
    FrameStatus,
    POLICY_FAULTED,
    RuntimeFrame,
    SOURCE_AUTHORITATIVE_RUNTIME,
    classify_frame,
    read_shm_frame,
)
from abs_collision import (
    CAPTURE_ID_RE, HEX64_RE, CollisionSnapshot, CollisionStatus,
    MODEL_CLOSURE_SHA256, SCENARIO_ID, SCENE_ROOT_SHA256, classify_snapshot,
    read_collision_snapshot,
)
from abs_scene import LEGACY_BINDING, SCENES
from p1_10_common_start import CommonStartError, validate_anchor

RECORD_FORMAT_VERSION = 2
FRAME_SOURCE_PATH = "/dev/shm/mujoco_rt_frame"
RUN_RECORD_KIND_META = "meta"
RUN_RECORD_KIND_FRAME = "frame"
RUN_RECORD_KIND_TERMINAL = "terminal"

# Accepted frame statuses form an explicit whitelist: LIVE (valid payload) and
# MISSING (recorded gap, no payload). ANY other status — including the known
# invalid set below AND any unknown string, empty string, null, or wrong type —
# is rejected fail-closed and invalidates the whole record.
_KNOWN_NON_LIVE_STATUSES = frozenset(
    {
        FrameStatus.INVALID.value,
        FrameStatus.UNKNOWN_ORIGIN.value,
        FrameStatus.LEGACY.value,
        FrameStatus.SYNTHETIC.value,
        FrameStatus.STALE.value,
    }
)
_ACCEPTED_FRAME_STATUSES = frozenset({FrameStatus.LIVE.value, FrameStatus.MISSING.value})

# LIVE payload schema. Version 2 of the runtime frame adds strategy-event and
# simulator-clock association fields; availability flags remain explicit.
_INT_FIELDS = (
    "session_id", "source_sequence", "rl_step", "ray_age_ns", "monotonic_ns",
    "policy_cycle_id", "sim_clock_sequence", "sim_clock_monotonic_ns",
    "sim_clock_segment_id", "sim_clock_age_ns", "risk_evaluation_ns",
    "risk_condition_entered", "transition_reason", "mode_before", "mode_after",
    "switching_mode", "action_source", "sim_clock_status",
    "source", "controller_active", "rl_entered", "rl_active", "safety_faulted",
    "policy_state", "ray_origin", "ray_valid", "collision_origin",
    "torque_saturated_computed",
)
_BOOL_DOMAINS = frozenset(
    {"controller_active", "rl_entered", "rl_active", "safety_faulted",
     "ray_valid", "torque_saturated_computed", "risk_condition_met",
     "policy_mode_changed", "sim_clock_valid"}
)
_INT_ENUM_DOMAINS = {
    "source": (0, 1, 2, 3),
    "policy_state": (0, 1, 2),
    "ray_origin": (0, 1),
    "collision_origin": (0,),
    "mode_before": (0, 1, 2),
    "mode_after": (0, 1, 2),
    "switching_mode": (0, 1),
    "action_source": (0, 1, 2, 3),
    "transition_reason": (0, 1, 2, 3, 4),
    "risk_condition_entered": (0, 1, 2),
    "sim_clock_status": (0, 1, 2, 3, 4),
}
_VECTOR_FIELDS = {
    "lin_vel": 3, "command": 3, "world_pose": 3, "ray2d": 11,
    "action_raw": 12, "action_clipped": 12, "joint_target_rad": 12,
    "torque_nm": 12, "torque_saturated": 12,
}
_COLLISION_CLASSES = {0, 1, 2, 3, 4, 5}

# Human reason for each terminal field that has no authoritative source today.
UNKNOWN_REASON_SIM_TIME = "no valid MuJoCo clock sample was associated with the recorded policy frame"
UNKNOWN_REASON_REACHED_GOAL = "no authoritative reached-goal computation exists in the current runtime frame"
UNKNOWN_REASON_TIMEOUT = "no authoritative timeout marker exists in the current runtime frame"
UNKNOWN_REASON_COLLISION = "versioned collision snapshot is missing, stale, invalid, has unknown contacts, or lacks contiguous publisher coverage"
UNKNOWN_REASON_FALL = "versioned posture source is missing, stale, invalid, or lacks contiguous publisher coverage"


def _now_ns() -> int:
    """Steady-clock ns in the same domain as the C++ writer (steady_clock)."""
    return time.monotonic_ns()


def _policy_name(policy_state: int) -> str:
    return {0: "AGILE", 1: "RECOVERY", 2: "FAULTED"}.get(policy_state, "UNKNOWN")


def _finite_list(values: Sequence[float]) -> List[float]:
    return [float(v) for v in values]


def frame_payload(frame: RuntimeFrame) -> Dict[str, Any]:
    """Full raw payload of one frame snapshot, preserved verbatim.

    Every controller-computed value is stored as-is (including the raw
    ``torque_saturated`` bytes). Availability is carried separately in
    ``frame_availability`` so consumers never mistake flag-0 fields for results.
    """
    return {
        "session_id": frame.session_id,
        "source_sequence": frame.sequence,
        "rl_step": frame.rl_step,
        "ray_age_ns": frame.ray_age_ns,
        "monotonic_ns": frame.monotonic_ns,
        "policy_cycle_id": frame.rl_step,
        "sim_time_s": frame.sim_time_s if frame.sim_clock_sequence != 0 else None,
        "sim_clock_sequence": frame.sim_clock_sequence,
        "sim_clock_monotonic_ns": frame.sim_clock_monotonic_ns,
        "sim_clock_segment_id": frame.sim_clock_segment_id,
        "sim_clock_age_ns": frame.sim_clock_age_ns,
        "sim_clock_status": frame.sim_clock_status,
        "sim_clock_valid": bool(frame.sim_clock_valid),
        "mode_before": frame.mode_before,
        "mode_after": frame.policy_state,
        "switching_mode": frame.switching_mode,
        "action_source": frame.action_source,
        "transition_reason": frame.transition_reason,
        "risk_condition_met": bool(frame.risk_condition_met),
        "risk_condition_entered": frame.risk_condition_entered,
        "risk_evaluation_ns": frame.risk_evaluation_ns,
        "risk_condition_entered_ns": frame.risk_condition_entered_ns or None,
        "policy_mode_changed": bool(frame.policy_mode_changed),
        "mode_change_ns": frame.mode_change_ns or None,
        "entry_threshold": float(frame.entry_threshold),
        "exit_threshold": float(frame.exit_threshold),
        "source": frame.source,
        "controller_active": frame.controller_active,
        "rl_entered": frame.rl_entered,
        "rl_active": frame.rl_active,
        "safety_faulted": frame.safety_faulted,
        "policy_state": frame.policy_state,
        "policy_state_name": _policy_name(frame.policy_state),
        "ray_origin": frame.ray_origin,
        "ray_valid": frame.ray_valid,
        "collision_origin": frame.collision_origin,
        "torque_saturated_computed": frame.torque_saturated_computed,
        "ra_value": frame.ra_value,
        "lin_vel": _finite_list(frame.lin_vel),
        "command": _finite_list(frame.command),
        "world_pose": _finite_list(frame.world_pose),
        "ray2d": _finite_list(frame.ray2d),
        "action_raw": _finite_list(frame.action_raw),
        "action_clipped": _finite_list(frame.action_clipped),
        "joint_target_rad": _finite_list(frame.joint_target_rad),
        "torque_nm": _finite_list(frame.torque_nm),
        "torque_saturated": _finite_list(frame.torque_saturated),
    }


def frame_availability(frame: RuntimeFrame) -> Dict[str, bool]:
    """Availability flags derived from the controller's own frame flags.

    ``torque_saturated`` is available only when ``torque_saturated_computed`` is
    set; ``collision`` only when ``collision_origin`` is not UNAVAILABLE (never
    today); ``ray2d`` only when ``ray_valid``. Everything else the controller
    computes every RL step is available when the frame is LIVE.
    """
    return {
        "session_id": True,
        "source_sequence": True,
        "rl_step": True,
        "ray_age_ns": True,
        "monotonic_ns": True,
        "policy_cycle_id": True,
        "sim_time_s": bool(frame.sim_clock_sequence),
        "sim_clock_sequence": bool(frame.sim_clock_sequence),
        "sim_clock_monotonic_ns": bool(frame.sim_clock_monotonic_ns),
        "sim_clock_segment_id": bool(frame.sim_clock_sequence),
        "sim_clock_age_ns": bool(frame.sim_clock_sequence),
        "sim_clock_status": True,
        "sim_clock_valid": True,
        "mode_before": True,
        "mode_after": True,
        "switching_mode": True,
        "action_source": frame.action_source != 0,
        "transition_reason": True,
        "risk_condition_met": True,
        "risk_condition_entered": True,
        "risk_evaluation_ns": True,
        "risk_condition_entered_ns": frame.risk_condition_entered_ns != 0,
        "policy_mode_changed": True,
        "mode_change_ns": frame.mode_change_ns != 0,
        "entry_threshold": True,
        "exit_threshold": True,
        "ra_value": True,
        "lin_vel": True,
        "command": True,
        "world_pose": True,
        "ray2d": bool(frame.ray_valid),
        "action_raw": True,
        "action_clipped": True,
        "joint_target_rad": True,
        "torque_nm": True,
        "torque_saturated": bool(frame.torque_saturated_computed),
        "collision": bool(frame.collision_origin != COLLISION_UNAVAILABLE),
    }


def collision_snapshot_payload(status: CollisionStatus,
                               snapshot: Optional[CollisionSnapshot]) -> Dict[str, Any]:
    """Serialize the versioned collision source without filling missing values."""
    if snapshot is None:
        return {"status": status.value, "available": False, "reason": status.value.lower()}
    payload: Dict[str, Any] = {
        "version": snapshot.version,
        "status": status.value,
        "available": status is CollisionStatus.LIVE,
        "reason": None if status is CollisionStatus.LIVE else status.value.lower(),
        "sequence": snapshot.sequence,
        "physics_step": snapshot.physics_step,
        "sim_time": snapshot.sim_time,
        "monotonic_ns": snapshot.monotonic_ns,
        "authoritative": bool(snapshot.authoritative),
        "current_collision": bool(snapshot.current_collision),
        "collision_edge": bool(snapshot.collision_edge),
        "classified_contacts": snapshot.classified_contacts,
        "unknown_contacts": snapshot.unknown_contacts,
        "robot_obstacle_contacts": snapshot.robot_obstacle_contacts,
        "ground_contacts": snapshot.ground_contacts,
        "self_contacts": snapshot.self_contacts,
        "other_contacts": snapshot.other_contacts,
        "last_contact_class": snapshot.last_contact_class,
        "foot_ground_contacts": snapshot.foot_ground_contacts,
        "nonfoot_ground_contacts": snapshot.nonfoot_ground_contacts,
        "collision_contact_steps": snapshot.collision_contact_steps,
        "collision_episode_count": snapshot.collision_episode_count,
        "last_collision_start_step": snapshot.last_collision_start_step,
        "last_collision_end_step": snapshot.last_collision_end_step,
        "collision_duration_s": snapshot.collision_duration_s,
        "last_collision_start_sim_time": snapshot.last_collision_start_sim_time,
        "last_collision_end_sim_time": snapshot.last_collision_end_sim_time,
        "collision_history_count": snapshot.collision_history_count,
        "collision_history_overflow": bool(snapshot.collision_history_overflow),
        "physics_coverage_complete": bool(snapshot.physics_coverage_complete),
        "collision_history": list(snapshot.collision_history),
        "base_height_m": snapshot.base_height_m,
        "base_roll_rad": snapshot.base_roll_rad,
        "base_pitch_rad": snapshot.base_pitch_rad,
        "fall_candidate": bool(snapshot.fall_candidate),
        "fall_confirmed": bool(snapshot.fall_confirmed),
        "fall_start_physics_step": snapshot.fall_start_physics_step,
        "fall_start_sim_time": snapshot.fall_start_sim_time,
        "fall_confirmed_physics_step": snapshot.fall_confirmed_physics_step,
        "fall_confirmed_sim_time": snapshot.fall_confirmed_sim_time,
        "unknown_contact_steps": snapshot.unknown_contact_steps,
        "unknown_contact_episodes": snapshot.unknown_contact_episodes,
        "first_unknown_contact_step": snapshot.first_unknown_contact_step,
        "last_unknown_contact_step": snapshot.last_unknown_contact_step,
        "first_unknown_contact_sim_time": snapshot.first_unknown_contact_sim_time,
        "last_unknown_contact_sim_time": snapshot.last_unknown_contact_sim_time,
        "invalid_posture_steps": snapshot.invalid_posture_steps,
        "invalid_posture_episodes": snapshot.invalid_posture_episodes,
        "first_invalid_posture_step": snapshot.first_invalid_posture_step,
        "last_invalid_posture_step": snapshot.last_invalid_posture_step,
        "first_invalid_posture_sim_time": snapshot.first_invalid_posture_sim_time,
        "last_invalid_posture_sim_time": snapshot.last_invalid_posture_sim_time,
        "fall_history_count": snapshot.fall_history_count,
        "fall_history_overflow": bool(snapshot.fall_history_overflow),
        "fall_history": list(snapshot.fall_history),
        "event_latches_complete": snapshot.event_latches_complete,
        "last_robot_geom_id": snapshot.last_robot_geom_id,
        "last_obstacle_geom_id": snapshot.last_obstacle_geom_id,
        "invalid_reason": snapshot.invalid_reason,
        "scenario_id": snapshot.scenario_id,
        "scene_root_sha256": snapshot.scene_root_sha256,
        "model_closure_sha256": snapshot.model_closure_sha256,
        "capture_id": snapshot.capture_id,
        "runtime_model_fingerprint": snapshot.runtime_model_fingerprint,
    }
    if snapshot.version == 5:
        payload.update({
            "foot_identity_valid": snapshot.foot_identity_valid == 1,
            "nonfoot_obstacle_contacts": snapshot.nonfoot_obstacle_contacts,
            "foot_obstacle_contacts": snapshot.foot_obstacle_contacts,
            "foot_contact_mask": snapshot.foot_contact_mask,
            "foot_impact_mask": snapshot.foot_impact_mask,
            "foot_force_valid_mask": snapshot.foot_force_valid_mask,
            "foot_impact_unknown_mask": snapshot.foot_impact_unknown_mask,
            "foot_contact_history_count": snapshot.foot_contact_history_count,
            "foot_contact_history_overflow": bool(snapshot.foot_contact_history_overflow),
            "foot_impact_history_count": snapshot.foot_impact_history_count,
            "foot_impact_history_overflow": bool(snapshot.foot_impact_history_overflow),
            "foot_contact_physics_steps": snapshot.foot_contact_physics_steps,
            "foot_contact_episode_count": snapshot.foot_contact_episode_count,
            "foot_contact_first_step": snapshot.foot_contact_first_step,
            "foot_contact_last_step": snapshot.foot_contact_last_step,
            "foot_impact_physics_steps": snapshot.foot_impact_physics_steps,
            "foot_impact_episode_count": snapshot.foot_impact_episode_count,
            "foot_impact_first_step": snapshot.foot_impact_first_step,
            "foot_impact_last_step": snapshot.foot_impact_last_step,
            "foot_impact_unknown_steps": snapshot.foot_impact_unknown_steps,
            "foot_impact_unknown_first_step": snapshot.foot_impact_unknown_first_step,
            "foot_impact_unknown_last_step": snapshot.foot_impact_unknown_last_step,
            "foot_contact_duration_s": snapshot.foot_contact_duration_s,
            "foot_contact_first_sim_time": snapshot.foot_contact_first_sim_time,
            "foot_contact_last_sim_time": snapshot.foot_contact_last_sim_time,
            "foot_impact_duration_s": snapshot.foot_impact_duration_s,
            "foot_impact_first_sim_time": snapshot.foot_impact_first_sim_time,
            "foot_impact_last_sim_time": snapshot.foot_impact_last_sim_time,
            "foot_impact_unknown_first_sim_time": snapshot.foot_impact_unknown_first_sim_time,
            "foot_impact_unknown_last_sim_time": snapshot.foot_impact_unknown_last_sim_time,
            "foot_world_force": snapshot.foot_world_force,
            "foot_fxy": snapshot.foot_fxy,
            "foot_abs_fz": snapshot.foot_abs_fz,
            "foot_impact_threshold": snapshot.foot_impact_threshold,
            "foot_contact_history": list(snapshot.foot_contact_history),
            "foot_impact_history": list(snapshot.foot_impact_history),
            "nonfoot_collision_bodies": list(snapshot.nonfoot_collision_bodies),
        })
    return payload


# ------------------------------------------------------------- strict fact types
def _strict_bool(value: Any, field_name: str, errors: List[str]) -> Optional[bool]:
    """Accept only a real ``bool`` (or None). No implicit bool conversion."""
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    errors.append(f"{field_name}: expected bool, got {type(value).__name__} (implicit conversion rejected)")
    return None


def _strict_int(value: Any, field_name: str, errors: List[str]) -> Optional[int]:
    """Accept only a real ``int`` (not bool, not str, not float)."""
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int):
        errors.append(f"{field_name}: expected int, got {type(value).__name__}")
        return None
    return value


def _strict_str(value: Any, field_name: str, errors: List[str]) -> Optional[str]:
    """Accept only a real ``str`` (or None)."""
    if value is None:
        return None
    if not isinstance(value, str):
        errors.append(f"{field_name}: expected str, got {type(value).__name__}")
        return None
    return value


def _validate_process_facts(facts: Mapping[str, Any]) -> Tuple[Dict[str, Any], List[str]]:
    """Strictly type-check process facts. Invalid types -> UNKNOWN + error.

    This is the fail-closed gate the Reviewer required: ``bool("false")`` is
    never used; a string, int, or float where a bool is expected is recorded
    UNKNOWN and reported as a validation error.
    """
    errors: List[str] = []
    validated: Dict[str, Any] = {
        "exit_code": _strict_int(facts.get("exit_code"), "process_exit_code", errors),
        "forced_termination": _strict_bool(facts.get("forced_termination"), "forced_termination", errors),
        "shutdown_complete": _strict_bool(facts.get("shutdown_complete"), "shutdown_complete", errors),
        "shutdown_request_source": _strict_str(facts.get("shutdown_request_source"), "shutdown_request_source", errors),
    }
    external_events = facts.get("external_safety_events", [])
    if not isinstance(external_events, list) or any(
        not isinstance(item, dict) or
        not all(isinstance(item.get(key), str) for key in ("event", "source", "message")) or
        type(item.get("observed_monotonic_ns")) is not int
        for item in external_events
    ):
        errors.append("external_safety_events: expected list of event/source/message/observed_monotonic_ns objects")
        external_events = []
    validated["external_safety_events"] = external_events
    validated["cleanup_events"] = facts.get("cleanup_events", [])
    if not isinstance(validated["cleanup_events"], list) or any(not isinstance(item, dict) for item in validated["cleanup_events"]):
        errors.append("cleanup_events: expected list of event objects")
        validated["cleanup_events"] = []
    validated["run_terminal_result"] = _strict_str(facts.get("run_terminal_result"), "run_terminal_result", errors)
    validated["run_terminal_monotonic_ns"] = _strict_int(facts.get("run_terminal_monotonic_ns"), "run_terminal_monotonic_ns", errors)
    return validated, errors


class RunRecordRecorder:
    """Write one per-run JSONL record of frame snapshots + a terminal block.

    The production loop reads the fixed source through ``read_shm_frame`` and
    classifies with ``classify_frame``; there is no arbitrary path or
    frame-dictionary injection API in production (fixtures are test-only).
    """

    def __init__(self, path: str, *, run_id: Optional[str] = None,
                 capture_id: Optional[str] = None,
                 expected_fingerprint: Optional[str] = None,
                 expected_scene_binding: Optional[Mapping[str, Any]] = None,
                 common_start_required: bool = False,
                 expected_initial_qpos_sha256: Optional[str] = None,
                 stale_timeout_ns: int = DEFAULT_STALE_TIMEOUT_NS):
        self.path = Path(path)
        self.run_id = run_id or uuid.uuid4().hex
        self.stale_timeout_ns = stale_timeout_ns
        if capture_id is not None and CAPTURE_ID_RE.fullmatch(capture_id) is None:
            raise ValueError("invalid capture_id")
        if expected_fingerprint is not None and HEX64_RE.fullmatch(expected_fingerprint) is None:
            raise ValueError("invalid expected_fingerprint")
        if not isinstance(common_start_required, bool):
            raise ValueError("common_start_required must be bool")
        if common_start_required and capture_id is None:
            raise ValueError("common-start record requires capture_id")
        if common_start_required and (expected_initial_qpos_sha256 is None or
                                      HEX64_RE.fullmatch(expected_initial_qpos_sha256) is None):
            raise ValueError("common-start record requires expected initial qpos sha256")
        self.capture_id = capture_id
        self.expected_fingerprint = expected_fingerprint
        self.scene_binding_explicit = expected_scene_binding is not None
        self.expected_scene_binding = dict(expected_scene_binding or LEGACY_BINDING)
        self.common_start_required = common_start_required
        self.expected_initial_qpos_sha256 = expected_initial_qpos_sha256
        self._start_anchor: Optional[Dict[str, Any]] = None
        self._fh = None  # type: Optional[object]
        self._frames = 0
        self._first_monotonic_ns: Optional[int] = None
        self._last_monotonic_ns: Optional[int] = None
        self._last_session_id: Optional[int] = None
        self._last_policy_state: Optional[int] = None
        self._last_ra_value: Optional[float] = None
        self._safety_fault_seen = False
        self._safety_fault_last = False
        self._collision_samples = 0
        self._collision_live_samples = 0
        self._collision_unknown_samples = 0
        self._collision_invalid_samples = 0
        self._collision_physics_gaps = 0
        self._collision_first_physics_step: Optional[int] = None
        self._collision_last_physics_step: Optional[int] = None
        self._collision_last_sim_time: Optional[float] = None
        self._collision_observed_event = False
        self._collision_episode_count_max = 0
        self._collision_contact_steps_max = 0
        self._collision_duration_s_max = 0.0
        self._collision_history_overflow = False
        self._collision_event_latches_complete = True
        self._collision_publisher_coverage_complete = True
        self._collision_last_payload: Optional[Dict[str, Any]] = None
        self._fall_confirmed_observed = False
        self._started = False
        self._stopped = False
        self._finalized = False

    # ------------------------------------------------------------- lifecycle
    def start(self) -> str:
        """Open the record file and write the meta line. Returns the run_id."""
        if self._started:
            raise RuntimeError("run record already started")
        self._fh = open(self.path, "w", encoding="utf-8")
        meta = {
            "kind": RUN_RECORD_KIND_META,
            "record_format_version": RECORD_FORMAT_VERSION,
            "run_id": self.run_id,
            "source": FRAME_SOURCE_PATH,
            "created_at_ns": _now_ns(),
        }
        if self.capture_id is not None:
            meta["capture_id"] = self.capture_id
        if self.scene_binding_explicit:
            meta["scene_binding"] = self.expected_scene_binding
        if self.common_start_required:
            meta["common_start_required"] = True
            meta["common_start_initial_qpos_sha256"] = self.expected_initial_qpos_sha256
        self._write_line(meta)
        self._started = True
        return self.run_id

    def close(self) -> None:
        if self._fh is not None:
            try:
                self._fh.flush()
            finally:
                self._fh.close()
                self._fh = None

    @property
    def state(self) -> str:
        """Two-phase lifecycle state: INITIAL / CAPTURING / STOPPED / FINALIZED."""
        if not self._started:
            return "INITIAL"
        if self._finalized:
            return "FINALIZED"
        if self._stopped:
            return "STOPPED"
        return "CAPTURING"

    @property
    def stopped(self) -> bool:
        return self._stopped

    @property
    def finalized(self) -> bool:
        return self._finalized

    def stop_sampling(self) -> None:
        """End the capture phase: stop sampling the runtime frame.

        The record stays unfinalized and no further frame line can be written.
        Calling it again is a harmless no-op. This is the boundary that keeps
        controller-exit INVALID frames out of the record: after this point the
        recorder no longer reads the shared-memory frame.
        """
        if not self._started:
            raise RuntimeError("run record not started")
        if self._finalized:
            raise RuntimeError("run record already finalized")
        self._stopped = True

    # ------------------------------------------------------------- recording
    def set_start_anchor(self, anchor: Mapping[str, Any]) -> None:
        """Attach the producer-side anchor before the first formal LIVE frame."""
        if not self._started or self._stopped or self._finalized:
            raise RuntimeError("cannot attach start anchor outside capture")
        if not self.common_start_required:
            raise RuntimeError("start anchor is only valid for a common-start record")
        try:
            validate_anchor(anchor,
                            expected_capture_id=self.capture_id or "",
                            expected_qpos_sha256=self.expected_initial_qpos_sha256 or "")
        except CommonStartError as exc:
            raise ValueError(str(exc)) from exc
        self._start_anchor = json.loads(json.dumps(dict(anchor), sort_keys=True))

    def record_snapshot(self, raw: bytes, now_ns: Optional[int] = None) -> Dict[str, Any]:
        """Classify one snapshot and append its frame line.

        Returns the stored line so callers/tests can assert the classification.
        A safety fault is recorded when ``safety_faulted`` OR
        ``policy_state == FAULTED``.
        """
        if not self._started or self._stopped or self._finalized:
            raise RuntimeError("run record not open for sampling: not started, sampling stopped, or already finalized")
        if now_ns is None:
            now_ns = _now_ns()
        status, frame = classify_frame(raw, now_ns, self.stale_timeout_ns)
        line: Dict[str, Any] = {
            "kind": RUN_RECORD_KIND_FRAME,
            "run_id": self.run_id,
            "status": status.value,
            "recorded_at_ns": now_ns,
        }
        if self.capture_id is not None:
            line["capture_id"] = self.capture_id
        if frame is None:
            line["payload"] = None
            line["availability"] = None
        else:
            line["payload"] = frame_payload(frame)
            line["availability"] = frame_availability(frame)
            if self.common_start_required and self._start_anchor is not None:
                line["start_anchor_ref"] = {
                    "capture_id": self._start_anchor["capture_id"],
                    "rl_session_id": self._start_anchor["rl_session_id"],
                    "first_physics_step": self._start_anchor["first_physics_step"],
                    "first_runtime_frame_sequence": self._start_anchor["first_runtime_frame_sequence"],
                }
            # The simulator can publish the next physics-step timestamp between
            # the policy-frame timestamp and this read. Validate collision age
            # against a fresh post-read host clock, not the earlier frame poll.
            collision_raw = read_collision_snapshot()
            collision_now_ns = max(now_ns, _now_ns())
            collision_status, collision_snapshot = classify_snapshot(
                collision_raw, collision_now_ns, self.stale_timeout_ns,
                expected_capture_id=self.capture_id,
                expected_fingerprint=self.expected_fingerprint,
                expected_scene_binding=(self.expected_scene_binding
                                        if self.scene_binding_explicit else None))
            line["payload"]["collision_snapshot"] = collision_snapshot_payload(
                collision_status, collision_snapshot)
            line["availability"]["collision_snapshot"] = collision_status is CollisionStatus.LIVE
            self._record_collision_sample(collision_status, collision_snapshot)
            self._frames += 1
            if self._first_monotonic_ns is None:
                self._first_monotonic_ns = frame.monotonic_ns
            self._last_monotonic_ns = frame.monotonic_ns
            self._last_session_id = frame.session_id
            self._last_policy_state = frame.policy_state
            self._last_ra_value = frame.ra_value
            faulted = bool(frame.safety_faulted) or frame.policy_state == POLICY_FAULTED
            if faulted:
                self._safety_fault_seen = True
                self._safety_fault_last = True
            elif status is FrameStatus.LIVE:
                self._safety_fault_last = False
        self._write_line(line)
        return line

    def _record_collision_sample(self, status: CollisionStatus,
                                 snapshot: Optional[CollisionSnapshot]) -> None:
        self._collision_samples += 1
        if status is not CollisionStatus.LIVE or snapshot is None:
            if status is CollisionStatus.INVALID:
                self._collision_invalid_samples += 1
            else:
                self._collision_unknown_samples += 1
            return
        self._collision_live_samples += 1
        self._collision_episode_count_max = max(self._collision_episode_count_max, snapshot.collision_episode_count)
        self._collision_contact_steps_max = max(self._collision_contact_steps_max, snapshot.collision_contact_steps)
        self._collision_duration_s_max = max(self._collision_duration_s_max, snapshot.collision_duration_s)
        self._collision_history_overflow = self._collision_history_overflow or bool(snapshot.collision_history_overflow)
        self._collision_publisher_coverage_complete = self._collision_publisher_coverage_complete and bool(snapshot.physics_coverage_complete)
        self._collision_event_latches_complete = self._collision_event_latches_complete and bool(snapshot.event_latches_complete)
        self._collision_last_payload = collision_snapshot_payload(status, snapshot)
        self._fall_confirmed_observed = self._fall_confirmed_observed or bool(snapshot.fall_confirmed) or any(
            isinstance(event, dict) and event.get("confirmed") is True
            for event in snapshot.fall_history)
        if self._collision_first_physics_step is None:
            self._collision_first_physics_step = snapshot.physics_step
        if snapshot.unknown_contacts > 0 or snapshot.unknown_contact_steps > 0 or not snapshot.event_latches_complete:
            self._collision_unknown_samples += 1
        if self._collision_last_physics_step is not None:
            gap = snapshot.physics_step - self._collision_last_physics_step - 1
            if gap < 0:
                self._collision_invalid_samples += 1
            else:
                self._collision_physics_gaps += gap
        self._collision_last_physics_step = snapshot.physics_step
        self._collision_last_sim_time = snapshot.sim_time
        if snapshot.collision_episode_count > 0 or snapshot.current_collision or snapshot.collision_edge:
            self._collision_observed_event = True

    # ------------------------------------------------------------- terminal
    def finalize(self, process_facts: Optional[Mapping[str, Any]] = None) -> Dict[str, Any]:
        """Append the terminal/event block from real, strictly-typed facts only.

        ``process_facts`` are the run's process-level observations supplied by
        the orchestrator (exit code, forced termination, request source,
        shutdown completeness). A fact of an invalid type is recorded UNKNOWN
        and reported in ``fact_validation_errors`` (which invalidates the
        record); it is never coerced (``bool("false")`` is never used).
        """
        if not self._started:
            raise RuntimeError("run record not started")
        if self._finalized:
            raise RuntimeError("run record already finalized; terminal already written")
        # Finalize may be reached directly from CAPTURING (implicit stop) or from
        # STOPPED. Either way sampling is over before the terminal is written.
        self._stopped = True
        facts = process_facts or {}
        validated, errors = _validate_process_facts(facts)

        external_safety_events = validated["external_safety_events"]
        observed_safety = self._safety_fault_seen or bool(external_safety_events)
        normal_shutdown, termination_reason = _compute_terminal(
            exit_code=validated["exit_code"],
            forced=validated["forced_termination"],
            shutdown_complete=validated["shutdown_complete"],
            safety_fault_seen=observed_safety,
            frames_observed=self._frames,
        )

        terminal_collision = next((event.get("collision_snapshot")
                                   for event in reversed(external_safety_events)
                                   if isinstance(event.get("collision_snapshot"), dict)), None)
        terminal_collision_validation = (
            _validate_collision_payload(terminal_collision,
                                        self.expected_scene_binding if self.scene_binding_explicit else None)
            if terminal_collision is not None else [])
        terminal_collision_live = (
            isinstance(terminal_collision, dict) and
            terminal_collision.get("status") == CollisionStatus.LIVE.value and
            terminal_collision.get("version") in (4, 5) and
            terminal_collision.get("capture_id") == self.capture_id and
            terminal_collision.get("runtime_model_fingerprint") == self.expected_fingerprint and
            not terminal_collision_validation)
        if terminal_collision_live:
            self._collision_last_payload = terminal_collision
            self._collision_observed_event = self._collision_observed_event or (
                terminal_collision.get("collision_episode_count", 0) > 0)
            self._fall_confirmed_observed = self._fall_confirmed_observed or (
                terminal_collision.get("fall_confirmed") is True)
            self._collision_episode_count_max = max(
                self._collision_episode_count_max, terminal_collision.get("collision_episode_count", 0))
            self._collision_contact_steps_max = max(
                self._collision_contact_steps_max, terminal_collision.get("collision_contact_steps", 0))
            self._collision_duration_s_max = max(
                self._collision_duration_s_max, terminal_collision.get("collision_duration_s", 0.0))
            self._collision_history_overflow = self._collision_history_overflow or (
                terminal_collision.get("collision_history_overflow") is True)
            self._collision_publisher_coverage_complete = (
                self._collision_publisher_coverage_complete and
                terminal_collision.get("physics_coverage_complete") is True)
            if (terminal_collision.get("unknown_contacts", 0) > 0 or
                    terminal_collision.get("unknown_contact_steps", 0) > 0 or
                    terminal_collision.get("event_latches_complete") is not True):
                self._collision_unknown_samples += 1
            self._collision_event_latches_complete = (
                self._collision_event_latches_complete and
                terminal_collision.get("event_latches_complete") is True)
            if terminal_collision.get("fall_history") and any(
                    isinstance(event, dict) and event.get("confirmed") is True
                    for event in terminal_collision["fall_history"]):
                self._fall_confirmed_observed = True
        elif terminal_collision is not None:
            self._collision_invalid_samples += 1

        sampled_collision_complete = (
            self._collision_live_samples > 0 and
            self._collision_unknown_samples == 0 and
            self._collision_invalid_samples == 0 and
            self._collision_publisher_coverage_complete and
            self._collision_event_latches_complete and
            not self._collision_history_overflow
        )
        v5_collision_semantics = isinstance(self._collision_last_payload, dict) and self._collision_last_payload.get("version") == 5
        if self._collision_observed_event:
            collision_events: Any = True
            collision_reason = ("authoritative non-foot robot-obstacle contact observed"
                                if v5_collision_semantics else
                                "authoritative robot-obstacle contact observed (historical layout semantics)")
        elif sampled_collision_complete:
            collision_events = False
            collision_reason = ("no non-foot robot-obstacle episode in the contiguous authoritative captured interval"
                                if v5_collision_semantics else
                                "no robot-obstacle episode in the contiguous authoritative captured interval")
        else:
            collision_events = "UNKNOWN"
            collision_reason = (
                "no accumulated obstacle episode in a continuous authoritative PhysicsLoop coverage interval"
                if sampled_collision_complete else UNKNOWN_REASON_COLLISION
            )

        terminal: Dict[str, Any] = {
            "kind": RUN_RECORD_KIND_TERMINAL,
            "run_id": self.run_id,
            "frames_observed": self._frames,
            "first_frame_time_ns": self._first_monotonic_ns,
            "last_frame_time_ns": self._last_monotonic_ns,
            "duration_ns": (
                self._last_monotonic_ns - self._first_monotonic_ns
                if self._first_monotonic_ns is not None and self._last_monotonic_ns is not None
                else None
            ),
            "last_session_id": self._last_session_id,
            "last_policy_state": None if self._last_policy_state is None else _policy_name(self._last_policy_state),
            "safety_fault_seen": observed_safety,
            "safety_fault_last": self._safety_fault_last,
            "external_safety_events": external_safety_events,
            "cleanup_events": validated["cleanup_events"],
            "run_terminal_result": validated["run_terminal_result"],
            "run_terminal_monotonic_ns": validated["run_terminal_monotonic_ns"],
            "last_ra_value": self._last_ra_value,
            # Terminal/event fields. Those with no authoritative source today are
            # recorded UNKNOWN with an explicit reason — never fabricated.
            "simulation_time_s": self._collision_last_sim_time if self._collision_live_samples else "UNKNOWN",
            "simulation_time_s_reason": (
                "authoritative collision snapshot sim_time from last sampled physics step"
                if self._collision_live_samples else UNKNOWN_REASON_SIM_TIME
            ),
            "reached_goal": "UNKNOWN",
            "reached_goal_reason": UNKNOWN_REASON_REACHED_GOAL,
            "timeout": "UNKNOWN",
            "timeout_reason": UNKNOWN_REASON_TIMEOUT,
            "collision_events": collision_events,
            "collision_events_reason": collision_reason,
            "process_exit_code": validated["exit_code"],
            "forced_termination": validated["forced_termination"],
            "shutdown_request_source": validated["shutdown_request_source"],
            "shutdown_complete": validated["shutdown_complete"],
            "fact_validation_errors": errors,
            "normal_shutdown": normal_shutdown,
            "termination_reason": termination_reason,
            "collision_coverage": {
                "samples": self._collision_samples,
                "live_samples": self._collision_live_samples,
                "unknown_samples": self._collision_unknown_samples,
                "invalid_samples": self._collision_invalid_samples,
                "physics_step_gaps": self._collision_physics_gaps,
                "producer_physics_coverage_complete": sampled_collision_complete,
                "publisher_physics_coverage_complete": self._collision_publisher_coverage_complete,
                "complete_for_captured_interval": sampled_collision_complete,
                "complete_for_motion_no_collision_claim": False,
                "observed_robot_obstacle_event": self._collision_observed_event,
                "episode_count_max": self._collision_episode_count_max,
                "contact_physics_steps_max": self._collision_contact_steps_max,
                "contact_duration_s_max": self._collision_duration_s_max,
                "event_history_overflow": self._collision_history_overflow,
                "last_snapshot": self._collision_last_payload,
            },
            "fall_events": (True if self._fall_confirmed_observed else
                            (False if sampled_collision_complete and not (
                                isinstance(self._collision_last_payload, dict) and
                                self._collision_last_payload.get("invalid_posture_steps", 0) > 0
                             ) else "UNKNOWN")),
            "fall_events_reason": (
                "confirmed posture rule observed" if self._fall_confirmed_observed else
                "no confirmed posture event in contiguous authoritative captured interval"
                if sampled_collision_complete and not (
                    isinstance(self._collision_last_payload, dict) and
                    self._collision_last_payload.get("invalid_posture_steps", 0) > 0
                ) else UNKNOWN_REASON_FALL),
        }
        if self.capture_id is not None:
            terminal["capture_id"] = self.capture_id
        if self.common_start_required:
            terminal["common_start_required"] = True
            terminal["start_anchor"] = self._start_anchor
            terminal["start_anchor_status"] = (
                "PRESENT" if self._start_anchor is not None else "MISSING")
        self._write_line(terminal)
        self._finalized = True
        self.close()
        return terminal

    def _write_line(self, line: Dict[str, Any]) -> None:
        if self._fh is None:
            raise RuntimeError("run record file not open")
        self._fh.write(json.dumps(line, sort_keys=True) + "\n")
        # Flush so the on-disk record always matches the logical lifecycle state
        # (frame lines visible during capture, no terminal before finalize) and
        # survives an unexpected exit.
        self._fh.flush()


def _compute_terminal(
    *,
    exit_code: Optional[int],
    forced: Optional[bool],
    shutdown_complete: Optional[bool],
    safety_fault_seen: bool,
    frames_observed: int,
) -> Tuple[Optional[bool], str]:
    """Compute normal-shutdown and termination-reason from real facts only.

    Priority for termination reason (most authoritative first): controller
    SAFETY_FAULT > FORCED_TERMINATION > NONZERO_EXIT > FRAMES_ENDED_RC0 > UNKNOWN.
    """
    if safety_fault_seen:
        termination_reason = "SAFETY_FAULT"
    elif forced is True:
        termination_reason = "FORCED_TERMINATION"
    elif exit_code is not None and exit_code != 0:
        termination_reason = "NONZERO_EXIT"
    elif (
        exit_code == 0
        and forced is False
        and frames_observed > 0
        and shutdown_complete is True
    ):
        termination_reason = "FRAMES_ENDED_RC0"
    else:
        termination_reason = "UNKNOWN"

    if forced is True or (exit_code is not None and exit_code != 0):
        normal_shutdown = False
    elif exit_code == 0 and forced is False and shutdown_complete is True:
        normal_shutdown = True
    else:
        normal_shutdown = None  # UNKNOWN: not provable from the record alone
    return normal_shutdown, termination_reason


# ================================================================== post-run


@dataclass
class RunRecordData:
    meta: Dict[str, Any]
    frames: List[Dict[str, Any]]
    terminal: Optional[Dict[str, Any]]
    parse_errors: List[str] = field(default_factory=list)
    terminal_count: int = 0
    terminal_is_last: Optional[bool] = None


def load_record(path: str) -> RunRecordData:
    """Read a run record file into its meta / frame / terminal components.

    A line that fails to parse is retained as a parse error; the summary treats
    the record as structurally invalid if any required component is missing or
    any line is unparsable. Terminal uniqueness/position is tracked so the
    summary can enforce the record boundary.
    """
    meta: Dict[str, Any] = {}
    frames: List[Dict[str, Any]] = []
    terminal: Optional[Dict[str, Any]] = None
    errors: List[str] = []
    terminal_count = 0
    last_line_was_terminal = False
    for index, raw_line in enumerate(Path(path).read_text(encoding="utf-8").splitlines()):
        if not raw_line.strip():
            continue
        try:
            obj = json.loads(raw_line)
        except ValueError as exc:
            errors.append(f"line {index + 1}: {exc}")
            last_line_was_terminal = False
            continue
        if not isinstance(obj, dict):
            errors.append(f"line {index + 1}: record line is not a JSON object")
            last_line_was_terminal = False
            continue
        kind = obj.get("kind")
        if kind == RUN_RECORD_KIND_META:
            meta = obj
            last_line_was_terminal = False
        elif kind == RUN_RECORD_KIND_FRAME:
            frames.append(obj)
            last_line_was_terminal = False
        elif kind == RUN_RECORD_KIND_TERMINAL:
            terminal_count += 1
            terminal = obj
            last_line_was_terminal = True
        else:
            errors.append(f"line {index + 1}: unrecognized kind {kind!r}")
            last_line_was_terminal = False
    return RunRecordData(
        meta=meta,
        frames=frames,
        terminal=terminal,
        parse_errors=errors,
        terminal_count=terminal_count,
        terminal_is_last=last_line_was_terminal,
    )


def _is_empty_or_null(value: Any) -> bool:
    """True when ``value`` is null or an empty mapping.

    A legitimate MISSING gap must carry no payload and no availability; any
    other value (including a non-empty dict) makes the frame malformed.
    """
    return value is None or (isinstance(value, dict) and len(value) == 0)


def _validate_live_payload(payload: Any) -> List[str]:
    """Full structural validation of one LIVE payload (Reviewer blocker 2).

    Checks every field this record actually stores and the summary statistics
    depend on: required keys present, scalar int/float types legal, boolean and
    enum domains legal, vector/list lengths exact, and every numeric value
    finite (NaN/Inf rejected). Returns traceable reason strings; an empty list
    means the payload is well-formed. It never raises on a malformed payload.
    """
    reasons: List[str] = []
    if not isinstance(payload, dict):
        return ["payload_not_object"]

    for key in _INT_FIELDS:
        if key not in payload:
            reasons.append(f"missing_field:{key}")
            continue
        value = payload[key]
        if isinstance(value, bool) or not isinstance(value, int):
            reasons.append(f"field_not_int:{key}")
    for key in _BOOL_DOMAINS:
        if payload.get(key) not in (0, 1):
            reasons.append(f"boolean_field_out_of_domain:{key}")
    for key, domain in _INT_ENUM_DOMAINS.items():
        if payload.get(key) not in domain:
            reasons.append(f"enum_field_out_of_domain:{key}")

    for key in ("risk_condition_entered_ns", "mode_change_ns"):
        value = payload.get(key)
        if value is not None and (isinstance(value, bool) or not isinstance(value, int) or value <= 0):
            reasons.append(f"event_timestamp_invalid:{key}")
    if payload.get("risk_condition_entered") == 1 and payload.get("risk_condition_entered_ns") is None:
        reasons.append("risk_edge_timestamp_missing")
    if payload.get("risk_condition_entered") != 1 and payload.get("risk_condition_entered_ns") is not None:
        reasons.append("unexpected_risk_edge_timestamp")
    if payload.get("policy_mode_changed") and payload.get("mode_change_ns") is None:
        reasons.append("mode_change_timestamp_missing")
    if not payload.get("policy_mode_changed") and payload.get("mode_change_ns") is not None:
        reasons.append("unexpected_mode_change_timestamp")

    sim_time = payload.get("sim_time_s")
    if payload.get("sim_clock_valid"):
        if isinstance(sim_time, bool) or not isinstance(sim_time, (int, float)) or not math.isfinite(float(sim_time)):
            reasons.append("valid_sim_time_missing_or_nonfinite")
        if payload.get("sim_clock_sequence", 0) == 0 or payload.get("sim_clock_monotonic_ns", 0) == 0:
            reasons.append("valid_sim_clock_source_fields_missing")
    elif sim_time is not None and (
        isinstance(sim_time, bool)
        or not isinstance(sim_time, (int, float))
        or not math.isfinite(float(sim_time))
    ):
        reasons.append("invalid_sim_time_sample_not_finite_number")

    for key in ("entry_threshold", "exit_threshold"):
        value = payload.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            reasons.append(f"threshold_invalid:{key}")

    if not isinstance(payload.get("policy_state_name"), str):
        reasons.append("policy_state_name_not_str")

    if "ra_value" not in payload:
        reasons.append("missing_field:ra_value")
    else:
        ra = payload["ra_value"]
        if isinstance(ra, bool) or not isinstance(ra, (int, float)) or not math.isfinite(float(ra)):
            reasons.append("ra_value_not_finite_number")

    for key, length in _VECTOR_FIELDS.items():
        if key not in payload:
            reasons.append(f"missing_field:{key}")
            continue
        vec = payload[key]
        if not isinstance(vec, list):
            reasons.append(f"field_not_list:{key}")
            continue
        if len(vec) != length:
            reasons.append(f"wrong_vector_length:{key}")
            continue
        for index, value in enumerate(vec):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                reasons.append(f"vector_not_finite:{key}[{index}]")
    return reasons


def _validate_collision_payload(value: Any,
                                expected_scene_binding: Optional[Mapping[str, Any]] = None) -> List[str]:
    """Validate an optional versioned collision snapshot.

    Historical P1-09 records may omit this field; omission is UNKNOWN. If the
    field is present, malformed authority data invalidates the record rather
    than becoming collision=false.
    """
    if value is None:
        return []
    if not isinstance(value, dict):
        return ["collision_snapshot_not_object"]
    reasons: List[str] = []
    status = value.get("status")
    if status not in {item.value for item in CollisionStatus}:
        return ["collision_snapshot_status_invalid"]
    if status == CollisionStatus.INVALID.value:
        return ["collision_snapshot_invalid_source"]
    version = value.get("version", 2)  # historical serialized v2 records did not repeat version
    if isinstance(version, bool) or not isinstance(version, int) or version not in (2, 3, 4, 5):
        reasons.append("collision_snapshot_version_invalid")
    available = value.get("available")
    if not isinstance(available, bool):
        reasons.append("collision_snapshot_available_not_bool")
    if status == CollisionStatus.LIVE.value:
        if available is not True or value.get("authoritative") is not True or value.get("reason") is not None:
            reasons.append("collision_snapshot_live_not_authoritative")
        for key in ("sequence", "physics_step", "monotonic_ns", "classified_contacts",
                    "unknown_contacts", "robot_obstacle_contacts", "ground_contacts",
                    "self_contacts", "other_contacts", "last_contact_class",
                    "last_robot_geom_id", "last_obstacle_geom_id", "invalid_reason"):
            item = value.get(key)
            if isinstance(item, bool) or not isinstance(item, int):
                reasons.append(f"collision_snapshot_{key}_not_int")
        sim_time = value.get("sim_time")
        if isinstance(sim_time, bool) or not isinstance(sim_time, (int, float)) or not math.isfinite(float(sim_time)):
            reasons.append("collision_snapshot_sim_time_invalid")
        for key in ("current_collision", "collision_edge"):
            if not isinstance(value.get(key), bool):
                reasons.append(f"collision_snapshot_{key}_not_bool")
        if value.get("last_contact_class") not in _COLLISION_CLASSES:
            reasons.append("collision_snapshot_class_invalid")
        if isinstance(value.get("sequence"), int) and not isinstance(value.get("sequence"), bool):
            if value["sequence"] <= 0 or value["sequence"] & 1:
                reasons.append("collision_snapshot_sequence_invalid")
        if isinstance(value.get("physics_step"), int) and not isinstance(value.get("physics_step"), bool) and value["physics_step"] <= 0:
            reasons.append("collision_snapshot_physics_step_invalid")
        if isinstance(value.get("monotonic_ns"), int) and not isinstance(value.get("monotonic_ns"), bool) and value["monotonic_ns"] <= 0:
            reasons.append("collision_snapshot_monotonic_invalid")
        if value.get("invalid_reason") != 0:
            reasons.append("collision_snapshot_invalid_reason")
        robot_contacts = value.get("robot_obstacle_contacts")
        if version == 5:
            nonfoot = value.get("nonfoot_obstacle_contacts")
            foot = value.get("foot_obstacle_contacts")
            if (isinstance(nonfoot, bool) or not isinstance(nonfoot, int) or nonfoot < 0 or
                    isinstance(foot, bool) or not isinstance(foot, int) or foot < 0 or
                    nonfoot + foot != robot_contacts or
                    value.get("current_collision") is not (nonfoot > 0)):
                reasons.append("collision_snapshot_v5_obstacle_partition_invalid")
        elif isinstance(robot_contacts, int) and not isinstance(robot_contacts, bool):
            if value.get("current_collision") is True and robot_contacts <= 0:
                reasons.append("collision_snapshot_current_without_obstacle_contact")
            if value.get("current_collision") is False and robot_contacts != 0:
                reasons.append("collision_snapshot_obstacle_contact_without_current")
        if value.get("collision_edge") is True and value.get("current_collision") is not True:
            reasons.append("collision_snapshot_edge_without_current")
        if version in (3, 4, 5):
            for key in ("foot_ground_contacts", "nonfoot_ground_contacts", "collision_contact_steps",
                        "collision_episode_count", "last_collision_start_step", "last_collision_end_step",
                        "fall_start_physics_step", "fall_confirmed_physics_step", "collision_history_count"):
                item = value.get(key)
                if isinstance(item, bool) or not isinstance(item, int) or item < 0:
                    reasons.append(f"collision_snapshot_{key}_invalid")
            for key in ("collision_duration_s", "last_collision_start_sim_time",
                        "last_collision_end_sim_time", "base_height_m", "base_roll_rad",
                        "base_pitch_rad", "fall_start_sim_time", "fall_confirmed_sim_time"):
                item = value.get(key)
                if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(float(item)):
                    reasons.append(f"collision_snapshot_{key}_invalid")
            for key in ("collision_history_overflow", "physics_coverage_complete",
                        "fall_candidate", "fall_confirmed"):
                if not isinstance(value.get(key), bool):
                    reasons.append(f"collision_snapshot_{key}_not_bool")
            if (isinstance(value.get("foot_ground_contacts"), int) and
                    isinstance(value.get("nonfoot_ground_contacts"), int) and
                    value["foot_ground_contacts"] + value["nonfoot_ground_contacts"] != value.get("ground_contacts")):
                reasons.append("collision_snapshot_ground_count_mismatch")
            if value.get("collision_history_overflow") is True and value.get("collision_episode_count", 0) <= value.get("collision_history_count", 0):
                reasons.append("collision_snapshot_overflow_without_truncation")
            history = value.get("collision_history")
            if not isinstance(history, list) or len(history) != value.get("collision_history_count"):
                reasons.append("collision_snapshot_history_count_mismatch")
            elif isinstance(history, list):
                for event in history:
                    if not isinstance(event, dict):
                        reasons.append("collision_snapshot_history_entry_not_object")
                        continue
                    for key in ("start_physics_step", "end_physics_step", "robot_geom_id", "obstacle_geom_id"):
                        if isinstance(event.get(key), bool) or not isinstance(event.get(key), int):
                            reasons.append(f"collision_snapshot_history_{key}_invalid")
                    for key in ("start_sim_time", "end_sim_time"):
                        item = event.get(key)
                        if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(float(item)):
                            reasons.append(f"collision_snapshot_history_{key}_invalid")
                    for key in ("robot_geom_name", "obstacle_geom_name"):
                        if not isinstance(event.get(key), str):
                            reasons.append(f"collision_snapshot_history_{key}_invalid")
                    if (isinstance(event.get("start_physics_step"), int) and
                            isinstance(event.get("end_physics_step"), int) and
                            event["end_physics_step"] < event["start_physics_step"]):
                        reasons.append("collision_snapshot_history_step_order_invalid")
            if version in (4, 5) and "event_latches_complete" in value:
                if not isinstance(value.get("event_latches_complete"), bool):
                    reasons.append("collision_snapshot_event_latches_flag_invalid")
                for key in ("unknown_contact_steps", "unknown_contact_episodes",
                            "first_unknown_contact_step", "last_unknown_contact_step",
                            "invalid_posture_steps", "invalid_posture_episodes",
                            "first_invalid_posture_step", "last_invalid_posture_step",
                            "fall_history_count"):
                    item = value.get(key)
                    if isinstance(item, bool) or not isinstance(item, int) or item < 0:
                        reasons.append(f"collision_snapshot_{key}_invalid")
                for key in ("first_unknown_contact_sim_time", "last_unknown_contact_sim_time",
                            "first_invalid_posture_sim_time", "last_invalid_posture_sim_time"):
                    item = value.get(key)
                    if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(float(item)):
                        reasons.append(f"collision_snapshot_{key}_invalid")
                fall_history = value.get("fall_history")
                if (not isinstance(fall_history, list) or
                        len(fall_history) != value.get("fall_history_count")):
                    reasons.append("collision_snapshot_fall_history_count_mismatch")
                elif isinstance(fall_history, list):
                    for event in fall_history:
                        if not isinstance(event, dict):
                            reasons.append("collision_snapshot_fall_history_entry_not_object")
                            continue
                        if (not isinstance(event.get("confirmed"), bool) or
                                not isinstance(event.get("closed"), bool)):
                            reasons.append("collision_snapshot_fall_history_flags_invalid")
                if not isinstance(value.get("fall_history_overflow"), bool):
                    reasons.append("collision_snapshot_fall_history_overflow_invalid")
            if version == 5:
                if value.get("foot_identity_valid") is not True:
                    reasons.append("collision_snapshot_foot_identity_invalid")
                for key in ("nonfoot_obstacle_contacts", "foot_obstacle_contacts",
                            "foot_contact_mask", "foot_impact_mask", "foot_force_valid_mask",
                            "foot_impact_unknown_mask", "foot_contact_history_count",
                            "foot_impact_history_count", "foot_contact_physics_steps",
                            "foot_contact_episode_count", "foot_impact_physics_steps",
                            "foot_impact_episode_count", "foot_impact_unknown_steps"):
                    item = value.get(key)
                    if isinstance(item, bool) or not isinstance(item, int) or item < 0:
                        reasons.append(f"collision_snapshot_{key}_invalid")
                for key in ("foot_contact_history_overflow", "foot_impact_history_overflow"):
                    if not isinstance(value.get(key), bool):
                        reasons.append(f"collision_snapshot_{key}_invalid")
                if value.get("foot_contact_mask", 0) > 0xF or value.get("foot_impact_mask", 0) > 0xF:
                    reasons.append("collision_snapshot_foot_mask_out_of_range")
                if (value.get("foot_impact_mask", 0) & ~value.get("foot_contact_mask", 0) or
                        value.get("foot_impact_unknown_mask", 0) & ~value.get("foot_contact_mask", 0)):
                    reasons.append("collision_snapshot_foot_impact_without_contact")
                for key in ("foot_contact_duration_s", "foot_contact_first_sim_time",
                            "foot_contact_last_sim_time", "foot_impact_duration_s",
                            "foot_impact_first_sim_time", "foot_impact_last_sim_time",
                            "foot_impact_unknown_first_sim_time", "foot_impact_unknown_last_sim_time"):
                    item = value.get(key)
                    if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(float(item)):
                        reasons.append(f"collision_snapshot_{key}_invalid")
                for key, width in (("foot_world_force", 4), ("foot_fxy", 4),
                                   ("foot_abs_fz", 4), ("foot_impact_threshold", 4)):
                    rows = value.get(key)
                    if not isinstance(rows, (list, tuple)) or len(rows) != width:
                        reasons.append(f"collision_snapshot_{key}_invalid")
                    elif key == "foot_world_force":
                        if any(not isinstance(row, (list, tuple)) or len(row) != 3 or
                               any(isinstance(number, bool) or not isinstance(number, (int, float)) or
                                   not math.isfinite(float(number)) for number in row) for row in rows):
                            reasons.append(f"collision_snapshot_{key}_invalid")
                    elif any(isinstance(number, bool) or not isinstance(number, (int, float)) or
                             not math.isfinite(float(number)) for number in rows):
                        reasons.append(f"collision_snapshot_{key}_invalid")
                for key, count_key in (("foot_contact_history", "foot_contact_history_count"),
                                       ("foot_impact_history", "foot_impact_history_count")):
                    events = value.get(key)
                    if not isinstance(events, list) or len(events) != value.get(count_key):
                        reasons.append(f"collision_snapshot_{key}_count_mismatch")
                        continue
                    for event in events:
                        if not isinstance(event, dict) or event.get("foot_geom_name") not in {"FL", "FR", "RL", "RR"}:
                            reasons.append(f"collision_snapshot_{key}_identity_invalid")
                            continue
                        for field in ("start_physics_step", "end_physics_step", "foot_geom_id", "obstacle_geom_id"):
                            if isinstance(event.get(field), bool) or not isinstance(event.get(field), int):
                                reasons.append(f"collision_snapshot_{key}_{field}_invalid")
                        for field in ("start_sim_time", "end_sim_time"):
                            number = event.get(field)
                            if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(float(number)):
                                reasons.append(f"collision_snapshot_{key}_{field}_invalid")
                        if not isinstance(event.get("closed"), bool) or not isinstance(event.get("force_known"), bool):
                            reasons.append(f"collision_snapshot_{key}_flags_invalid")
                        if not isinstance(event.get("obstacle_geom_name"), str) or not event.get("obstacle_geom_name"):
                            reasons.append(f"collision_snapshot_{key}_obstacle_name_missing")
                bodies = value.get("nonfoot_collision_bodies")
                if not isinstance(bodies, list) or len(bodies) != value.get("collision_history_count"):
                    reasons.append("collision_snapshot_nonfoot_body_count_mismatch")
                elif any(not isinstance(item, dict) or not isinstance(item.get("robot_body_id"), int) or
                         item.get("robot_body_id", -1) < 0 or not item.get("robot_body_name")
                         for item in bodies):
                    reasons.append("collision_snapshot_nonfoot_body_identity_invalid")
        count_names = ("robot_obstacle_contacts", "ground_contacts", "self_contacts", "other_contacts")
        if all(isinstance(value.get(name), int) and not isinstance(value.get(name), bool) for name in count_names):
            if sum(value[name] for name in count_names) != value.get("classified_contacts"):
                reasons.append("collision_snapshot_classified_count_mismatch")
        for key in ("scenario_id", "scene_root_sha256", "model_closure_sha256",
                    "capture_id", "runtime_model_fingerprint"):
            if not isinstance(value.get(key), str) or not value.get(key):
                reasons.append(f"collision_snapshot_{key}_invalid")
        binding = expected_scene_binding or LEGACY_BINDING
        if value.get("scenario_id") != binding.get("scene_id"):
            reasons.append("collision_snapshot_scenario_mismatch")
        if value.get("scene_root_sha256") != binding.get("root_sha256"):
            reasons.append("collision_snapshot_scene_root_mismatch")
        if value.get("model_closure_sha256") != binding.get("closure_sha256"):
            reasons.append("collision_snapshot_model_closure_mismatch")
        if (expected_scene_binding is not None and
                value.get("runtime_model_fingerprint") != binding.get("model_fingerprint")):
            reasons.append("collision_snapshot_scene_model_fingerprint_mismatch")
        if CAPTURE_ID_RE.fullmatch(value.get("capture_id", "")) is None:
            reasons.append("collision_snapshot_capture_id_invalid")
        if HEX64_RE.fullmatch(value.get("runtime_model_fingerprint", "")) is None:
            reasons.append("collision_snapshot_runtime_fingerprint_invalid")
    return reasons


def _validate_record_trust(data: RunRecordData) -> Tuple[bool, List[str], List[Dict[str, Any]]]:
    """Fail-closed validity of the saved record.

    The whole record is INVALID when: any present frame status is not in the
    explicit {LIVE, MISSING} whitelist (unknown/null/wrong-type statuses fail
    closed); any LIVE payload fails full schema validation; any frame is
    non-authoritative; cross-frame continuity breaks; run identity is
    inconsistent; the terminal is not a unique final line; the duration is
    negative; or process facts were malformed.

    Returns ``(valid, reasons, valid_live_payloads)`` so the summary can
    compute statistics only over payloads that already passed validation (it
    never touches an unvalidated payload field).
    """
    reasons: List[str] = []
    valid_live_payloads: List[Dict[str, Any]] = []

    # --- structural
    if not data.meta:
        reasons.append("missing_meta_line")
    if not data.frames:
        reasons.append("no_frame_lines")
    if data.terminal is None:
        reasons.append("missing_terminal_line")
    reasons.extend(data.parse_errors)

    scene_binding = data.meta.get("scene_binding")
    collision_scene_binding = scene_binding
    if scene_binding is None:
        scene_binding = LEGACY_BINDING  # pre-S2-04 historical records
    else:
        scene_id = scene_binding.get("scene_id") if isinstance(scene_binding, dict) else None
        catalog_binding = next((spec.binding() for spec in SCENES.values()
                                if spec.scene_id == scene_id), None)
        if catalog_binding is None or scene_binding != catalog_binding:
            reasons.append("meta_scene_binding_not_in_catalog")
            scene_binding = LEGACY_BINDING

    # --- run identity: meta / frames / terminal share one run_id
    meta_run_id = data.meta.get("run_id")
    if not meta_run_id:
        reasons.append("meta_missing_run_id")
    if data.terminal is not None and data.terminal.get("run_id") != meta_run_id:
        reasons.append("terminal_run_id_mismatch")
    for index, frame in enumerate(data.frames):
        if frame.get("run_id") != meta_run_id:
            reasons.append(f"frame_run_id_mismatch_or_missing:{index}")

    # A v2 collision authority record is bound to the harness-created capture
    # identity. Historical records without collision snapshots remain readable;
    # a record that carries authority data without this binding is invalid.
    capture_id = data.meta.get("capture_id")
    if capture_id is not None and CAPTURE_ID_RE.fullmatch(capture_id) is None:
        reasons.append("meta_capture_id_invalid")
    if data.terminal is not None and capture_id is not None and data.terminal.get("capture_id") != capture_id:
        reasons.append("terminal_capture_id_mismatch")
    for index, frame in enumerate(data.frames):
        if capture_id is not None and frame.get("capture_id") != capture_id:
            reasons.append(f"frame_capture_id_mismatch_or_missing:{index}")
        if (capture_id is None and isinstance(frame.get("payload"), dict) and
                isinstance(frame["payload"].get("collision_snapshot"), dict) and
                frame["payload"]["collision_snapshot"].get("status") == CollisionStatus.LIVE.value):
            reasons.append(f"collision_capture_identity_missing:{index}")

    # Common-start records have an additional producer-side boundary.  The
    # legacy record format remains readable when this opt-in field is absent;
    # an explicitly enabled record must carry a complete, bound anchor.
    common_start_required = data.meta.get("common_start_required", False)
    if not isinstance(common_start_required, bool):
        reasons.append("common_start_required_not_bool")
        common_start_required = False
    if common_start_required:
        expected_qpos = data.meta.get("common_start_initial_qpos_sha256")
        if not isinstance(expected_qpos, str) or HEX64_RE.fullmatch(expected_qpos) is None:
            reasons.append("common_start_initial_qpos_sha256_invalid")
        if capture_id is None:
            reasons.append("common_start_capture_id_missing")
        terminal_anchor = data.terminal.get("start_anchor") if data.terminal is not None else None
        if data.terminal is None or data.terminal.get("common_start_required") is not True:
            reasons.append("common_start_terminal_binding_missing")
        if data.terminal is not None and data.terminal.get("start_anchor_status") != "PRESENT":
            reasons.append("common_start_anchor_missing")
        if isinstance(expected_qpos, str) and HEX64_RE.fullmatch(expected_qpos) and capture_id is not None:
            try:
                validate_anchor(terminal_anchor,
                                expected_capture_id=capture_id,
                                expected_qpos_sha256=expected_qpos)
            except CommonStartError as exc:
                reasons.append(f"common_start_anchor_invalid:{exc}")

    # --- terminal uniqueness + boundary
    if data.terminal_count != 1:
        reasons.append("terminal_not_unique")
    if data.terminal_is_last is not True:
        reasons.append("terminal_not_at_record_boundary")

    # --- frame status whitelist + LIVE payload schema validation
    for index, frame in enumerate(data.frames):
        status = frame.get("status")
        if status == FrameStatus.LIVE.value:
            payload = frame.get("payload")
            availability = frame.get("availability")
            if not isinstance(payload, dict) or not isinstance(availability, dict):
                reasons.append(f"malformed_frame_payload:{index}")
                continue
            if payload.get("source") != SOURCE_AUTHORITATIVE_RUNTIME:
                reasons.append(f"non_authoritative_frame:{index}:{payload.get('source')}")
            payload_reasons = _validate_live_payload(payload)
            payload_reasons.extend(_validate_collision_payload(
                payload.get("collision_snapshot"), collision_scene_binding))
            collision = payload.get("collision_snapshot")
            if isinstance(collision, dict) and collision.get("status") == CollisionStatus.LIVE.value:
                if capture_id is None or collision.get("capture_id") != capture_id:
                    payload_reasons.append("collision_snapshot_capture_id_not_bound_to_record")
            if payload_reasons:
                reasons.append(f"invalid_payload:{index}:" + ",".join(payload_reasons))
            else:
                valid_live_payloads.append(payload)
        elif status == FrameStatus.MISSING.value:
            # A MISSING gap is legitimate only when it carries NO payload and NO
            # availability. Any non-empty payload/availability makes the frame
            # malformed and invalidates the whole record (never ignored).
            if not _is_empty_or_null(frame.get("payload")) or not _is_empty_or_null(frame.get("availability")):
                reasons.append(f"malformed_missing_frame:{index}")
            continue
        elif isinstance(status, str) and status in _KNOWN_NON_LIVE_STATUSES:
            reasons.append(f"non_live_frame_status:{index}:{status}")
        else:
            # Unknown / null / wrong-type status: never treated as MISSING.
            reasons.append(f"unknown_frame_status:{index}:{status!r}")

    if common_start_required:
        first_live = next((frame for frame in data.frames
                           if frame.get("status") == FrameStatus.LIVE.value), None)
        anchor = data.terminal.get("start_anchor") if data.terminal is not None else None
        ref = first_live.get("start_anchor_ref") if first_live is not None else None
        expected_ref = None
        if isinstance(anchor, dict):
            expected_ref = {
                "capture_id": anchor.get("capture_id"),
                "rl_session_id": anchor.get("rl_session_id"),
                "first_physics_step": anchor.get("first_physics_step"),
                "first_runtime_frame_sequence": anchor.get("first_runtime_frame_sequence"),
            }
        if first_live is None or not isinstance(ref, dict):
            reasons.append("common_start_first_live_frame_anchor_ref_missing")
        elif set(ref) != set(expected_ref or {}):
            reasons.append("common_start_first_live_frame_anchor_ref_fields_invalid")
        elif ref != expected_ref:
            reasons.append("common_start_first_live_frame_anchor_ref_mismatch")
        elif not isinstance(first_live.get("payload"), dict):
            reasons.append("common_start_first_live_frame_payload_missing")
        else:
            payload = first_live["payload"]
            if isinstance(anchor, dict):
                if payload.get("session_id") != anchor.get("rl_session_id"):
                    reasons.append("common_start_first_live_frame_session_mismatch")
                if payload.get("source_sequence", -1) < anchor.get("first_runtime_frame_sequence", 0):
                    reasons.append("common_start_first_live_frame_sequence_before_anchor")
                if payload.get("rl_step", -1) < anchor.get("first_runtime_frame_rl_step", 0):
                    reasons.append("common_start_first_live_frame_rl_step_before_anchor")
                if payload.get("monotonic_ns", -1) < anchor.get("first_runtime_frame_monotonic_ns", 0):
                    reasons.append("common_start_first_live_frame_time_before_anchor")

    # --- cross-frame continuity over validated payloads only
    previous_session = None
    previous_sequence = None
    previous_rl_step = None
    previous_monotonic = None
    for payload in valid_live_payloads:
        if previous_session is not None and payload["session_id"] != previous_session:
            reasons.append("session_id_changed")
        if previous_sequence is not None and payload["source_sequence"] <= previous_sequence:
            reasons.append("source_sequence_not_strictly_increasing")
        if previous_rl_step is not None and payload["rl_step"] <= previous_rl_step:
            reasons.append("rl_step_not_strictly_increasing")
        if previous_monotonic is not None and payload["monotonic_ns"] <= previous_monotonic:
            reasons.append("monotonic_time_not_strictly_increasing")
        previous_session = payload["session_id"]
        previous_sequence = payload["source_sequence"]
        previous_rl_step = payload["rl_step"]
        previous_monotonic = payload["monotonic_ns"]

    # --- duration and process-fact validity
    if data.terminal is not None:
        duration = data.terminal.get("duration_ns")
        if duration is not None and duration < 0:
            reasons.append("negative_duration")
        if data.terminal.get("fact_validation_errors"):
            reasons.append("malformed_process_facts")

    return not reasons, reasons, valid_live_payloads


def _is_valid_record(data: RunRecordData) -> Tuple[bool, List[str]]:
    """Backward-compatible alias: structural validity is part of trust validity."""
    valid, reasons, _ = _validate_record_trust(data)
    return valid, reasons


def _horizontal_speed(lin_vel: List[float]) -> float:
    return math.sqrt(lin_vel[0] ** 2 + lin_vel[1] ** 2)


def _stats(values: List[float]) -> Dict[str, Any]:
    if not values:
        return {"count": 0, "mean": None, "min": None, "max": None, "last": None}
    return {
        "count": len(values),
        "mean": sum(values) / len(values),
        "min": min(values),
        "max": max(values),
        "last": values[-1],
    }


def _frame_faulted(payload: Dict[str, Any]) -> bool:
    """A safety fault is safety_faulted OR policy_state == FAULTED."""
    return bool(payload.get("safety_faulted")) or payload.get("policy_state") == POLICY_FAULTED


def summarize_record(path: str) -> Dict[str, Any]:
    """Post-run summary computed from the saved record only.

    Every output below derives from the JSONL record written by
    ``RunRecordRecorder``. Statistics use only frames whose stored status is
    LIVE (fresh, coherent, authoritative) unless stated otherwise.
    """
    data = load_record(path)
    valid, trust_reasons, valid_live_payloads = _validate_record_trust(data)

    live_status_count = sum(1 for f in data.frames if f.get("status") == "LIVE")

    authoritative = bool(valid) and bool(valid_live_payloads)
    summary: Dict[str, Any] = {
        "record_validity": "VALID" if valid else "INVALID",
        "record_validity_reasons": trust_reasons,
        "authoritative_runtime_source": authoritative,
        "authoritative_reason": None if authoritative else (
            "no_live_frames_in_record" if valid else trust_reasons
        ),
        "meta": data.meta,
        "terminal": data.terminal,
        "frame_count": len(data.frames),
        "live_frame_count": live_status_count,
        "outcome": None,
        "outcome_reasons": [],
        "termination_reason": None,
        "normal_shutdown": None,
        "duration_ns": None,
        "velocity": {},
        "attitude_yaw": {},
        "collision": {},
        "recovery_usage": {},
        "ra_statistics": {},
        "safety_faults": {},
        "simulation_time_s": None,
    }

    if data.terminal is not None:
        summary["termination_reason"] = data.terminal.get("termination_reason")
        summary["normal_shutdown"] = data.terminal.get("normal_shutdown")
        summary["duration_ns"] = data.terminal.get("duration_ns")
        summary["simulation_time_s"] = data.terminal.get("simulation_time_s")

    # Statistics are computed only over payloads that already passed full schema
    # validation; a malformed payload invalidates the record and never reaches
    # this code path. The summary therefore cannot KeyError/TypeError.
    live_payloads = valid_live_payloads

    # --- velocity / attitude / RA / Recovery / safety from LIVE frames
    speeds: List[float] = []
    yaws: List[float] = []
    ra_values: List[float] = []
    recovery_steps: List[int] = []
    fault_steps: List[int] = []
    for payload in live_payloads:
        speeds.append(_horizontal_speed(payload["lin_vel"]))
        yaws.append(payload["world_pose"][2])
        ra_values.append(payload["ra_value"])
        if payload["policy_state"] == 1:
            recovery_steps.append(payload["rl_step"])
        if _frame_faulted(payload):
            fault_steps.append(payload["rl_step"])

    speed_stats = _stats(speeds)
    velocity = {
        "horizontal_speed_avg_m_s": speed_stats["mean"],
        "horizontal_speed_peak_m_s": speed_stats["max"],
        "lin_vel_samples": speed_stats["count"],
    }
    if live_payloads:
        for axis, name in ((0, "vx"), (1, "vy"), (2, "vz")):
            axis_values = [p["lin_vel"][axis] for p in live_payloads]
            velocity[f"{name}_avg_m_s"] = sum(axis_values) / len(axis_values)
            velocity[f"{name}_peak_m_s"] = max(abs(v) for v in axis_values)

    summary["velocity"] = velocity
    summary["attitude_yaw"] = {
        "recorded": bool(live_payloads),
        "note": "only yaw (world_pose[2]) is in the frame; roll/pitch have no source",
        "yaw_deg": _stats([math.degrees(y) for y in yaws]) if yaws else {"count": 0, "mean": None, "min": None, "max": None, "last": None},
    }

    collision_snapshots = [p.get("collision_snapshot") for p in live_payloads]
    collision_live = [item for item in collision_snapshots
                      if isinstance(item, dict) and item.get("status") == CollisionStatus.LIVE.value]
    collision_observed = any(
        isinstance(item, dict) and (item.get("collision_episode_count", 0) > 0 or
                                    item.get("current_collision") is True or item.get("collision_edge") is True)
        for item in collision_live
    )
    collision_unknown = any(
        not isinstance(item, dict) or item.get("status") != CollisionStatus.LIVE.value or
        item.get("unknown_contacts", 0) > 0
        for item in collision_snapshots
    ) if collision_snapshots else True
    collision_gaps = 0
    previous_physics_step = None
    for item in collision_live:
        step = item.get("physics_step")
        if isinstance(step, int) and previous_physics_step is not None:
            collision_gaps += max(0, step - previous_physics_step - 1)
        previous_physics_step = step
    sampled_collision_complete = bool(collision_live) and not collision_unknown and all(
        item.get("version", 2) in (4, 5) and item.get("physics_coverage_complete") is True and
        item.get("collision_history_overflow") is False for item in collision_live)
    # The simulator accumulates every PhysicsLoop step. Policy-frame gaps are
    # informational and do not erase that producer-side coverage.
    collision_available = collision_observed
    summary["collision"] = {
        "available": collision_available,
        "reason": None if collision_available else (
            "authoritative samples contain no contact, but capture has no complete episode coverage boundary"
            if sampled_collision_complete else UNKNOWN_REASON_COLLISION
        ),
        "event_count": max((item.get("collision_episode_count", 0) for item in collision_live), default=None)
            if collision_observed else (0 if sampled_collision_complete else None),
        "coverage": {
            "samples": len(collision_snapshots),
            "live_samples": len(collision_live),
            "unknown_contacts": sum(item.get("unknown_contacts", 0) for item in collision_live),
            "physics_step_gaps": collision_gaps,
            "sampled_steps_contiguous": sampled_collision_complete,
            "producer_physics_coverage_complete": sampled_collision_complete,
            "complete_for_no_collision_claim": False,
            "scope": "whole captured interval; motion boundary is evaluated by S1 analyzer",
        },
    }

    summary["recovery_usage"] = {
        "recovery_steps": len(recovery_steps),
        "total_live_steps": len(live_payloads),
        "recovery_fraction": (len(recovery_steps) / len(live_payloads)) if live_payloads else None,
        "transitions": _transition_count(live_payloads),
    }

    summary["ra_statistics"] = {
        **_stats(ra_values),
        "samples": len(ra_values),
    }

    summary["safety_faults"] = {
        "faulted_steps": len(fault_steps),
        "first_fault_rl_step": fault_steps[0] if fault_steps else None,
        "faulted": bool(fault_steps),
    }

    # --- outcome, from the record only
    outcome_reasons: List[str] = []
    if not valid:
        outcome = "INVALID"
        outcome_reasons.extend(trust_reasons)
    elif not live_payloads:
        outcome = "INVALID"
        outcome_reasons.append("no_live_frames_in_record")
    else:
        terminal = data.terminal or {}
        safety_fault = bool(fault_steps) or terminal.get("safety_fault_seen") is True
        if safety_fault:
            outcome = "FAILURE"
            outcome_reasons.append("safety_fault")
        elif terminal.get("forced_termination") is True:
            outcome = "FAILURE"
            outcome_reasons.append("forced_termination")
        elif terminal.get("process_exit_code") is not None and terminal.get("process_exit_code") != 0:
            outcome = "FAILURE"
            outcome_reasons.append("nonzero_exit")
        else:
            outcome = "UNKNOWN"
            if terminal.get("reached_goal") in (None, "UNKNOWN"):
                outcome_reasons.append("no_reached_goal_source")
            if terminal.get("timeout") in (None, "UNKNOWN"):
                outcome_reasons.append("no_timeout_source")
            if terminal.get("collision_events") in (None, "UNKNOWN"):
                outcome_reasons.append("no_collision_source")
            if terminal.get("fall_events") in (None, "UNKNOWN"):
                outcome_reasons.append("no_fall_source")

    summary["outcome"] = outcome
    summary["outcome_reasons"] = outcome_reasons
    return summary


def _transition_count(live_payloads: List[Dict[str, Any]]) -> int:
    """Count policy-state transitions across consecutive LIVE frames."""
    transitions = 0
    previous = None
    for payload in live_payloads:
        current = payload["policy_state"]
        if previous is not None and current != previous:
            transitions += 1
        previous = current
    return transitions


def report_record(path: str) -> None:
    """Human-readable report of ``summarize_record`` (used by the CLI)."""
    summary = summarize_record(path)
    print(f"record: {path}")
    print(f"  validity            : {summary['record_validity']}  {summary['record_validity_reasons']}")
    print(f"  authoritative source: {summary['authoritative_runtime_source']}")
    print(f"  frames / live       : {summary['frame_count']} / {summary['live_frame_count']}")
    print(f"  outcome             : {summary['outcome']}  {summary['outcome_reasons']}")
    print(f"  termination reason  : {summary['termination_reason']}")
    print(f"  normal shutdown     : {summary['normal_shutdown']}")
    print(f"  duration_ns         : {summary['duration_ns']}")
    print(f"  simulation_time_s   : {summary['simulation_time_s']}")
    print(f"  velocity            : {summary['velocity']}")
    print(f"  attitude yaw        : {summary['attitude_yaw']['yaw_deg']}")
    print(f"  collision           : {summary['collision']}")
    print(f"  recovery usage      : {summary['recovery_usage']}")
    print(f"  RA statistics       : {summary['ra_statistics']}")
    print(f"  safety faults       : {summary['safety_faults']}")


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        sys.exit("usage: python3 run_record.py <run-record.jsonl>")
    report_record(sys.argv[1])
