#!/usr/bin/env python3
"""Bounded, self-cleaning S1 nonformal safety-recording diagnostic."""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import select
import signal
import shlex
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from abs_rt_frame import (  # noqa: E402
    FRAME_VERSION, SIM_CLOCK_STALE, FrameStatus, classify_frame, read_shm_frame,
)
from abs_collision import (  # noqa: E402
    CollisionStatus, classify_snapshot, read_collision_snapshot,
    read_collision_snapshot_diagnostic,
)
from abs_scene import resolve_scene  # noqa: E402
from run_record import RunRecordRecorder, collision_snapshot_payload  # noqa: E402
from s1_runtime_source import verify_config_consumption, verify_source_binding  # noqa: E402

MUJOCO_DIR = ROOT / "unitree_mujoco"
MUJOCO_BIN = MUJOCO_DIR / "simulate/build2/unitree_mujoco"
ROS_WS = ROOT / "quadruped_ros2_control_humble"
LIBTORCH = Path.home() / "Libraries/libtorch-cpu-2.0.1/lib"
SDKLIB = Path.home() / "Libraries/unitree_sdk2/lib"
EMERGENCY_RE = re.compile(r"\[EMERGENCY\].*Forcing PASSIVE")
HARDSTOP_RE = re.compile(r"\[HARD-STOP\].*forcing PASSIVE")
HARDSTOP_CONFIRMED_RE = re.compile(
    r"\[HARD-STOP-CONFIRMED\].*command=(?P<command>\d+).*"
    r"state=PASSIVE.*steady_ns=(?P<steady_ns>\d+)")
S6_ENTRY_RE = re.compile(r"\[S6-04-ENTRY\].*session_id=(?P<session>\d+).*steady_ns=(?P<steady_ns>\d+)")
S6_TRACE_MARKER = "[S6-04-TRACE] "
S6_GAIN_TRACE_MARKER = "[S6-05-GAIN] "
S6_FALL_LOW_HEIGHT = 1
S6_FALL_TILT = 2
VETO_RE = re.compile(r"\[ABS-CONTRACT\].*PASSIVE")
SOURCE_EVIDENCE_WAIT_S = 3.0


class ConfigOnlyComplete(Exception):
    """Internal bounded stop after controller initialization consumed config."""


class PreparationOnlyComplete(Exception):
    """Internal stop after observing fixed-stand preparation, never entering RL."""


def now_ns() -> int:
    return time.monotonic_ns()


def select_strategy_terminal(safety_abort: bool, collision: bool, fall: bool,
                             arrived: bool, sim_timeout: bool) -> str | None:
    """Resolve same-observation terminal facts using the run contract order."""
    for active, result in (
        (safety_abort, "SYSTEM_SAFETY_ABORT"),
        (collision, "COLLISION_TERMINATION"),
        (fall, "FALL_TERMINATION"),
        (arrived, "ARRIVED"),
        (sim_timeout, "DIAGNOSTIC_SIM_TIME_LIMIT"),
    ):
        if active:
            return result
    return None


def normal_closeout_gate(terminal: str, safety_abort: bool, collision_events: list[dict],
                         fall_confirmed: bool, snapshot: dict | None) -> tuple[bool, str | None]:
    """Require healthy, authoritative terminal evidence before standing hold."""
    if terminal not in {"ARRIVED", "DIAGNOSTIC_SIM_TIME_LIMIT"}:
        return False, "terminal is not ARRIVED or controlled sim timeout"
    if safety_abort or collision_events or fall_confirmed:
        return False, "safety, collision, or confirmed fall veto"
    if not isinstance(snapshot, dict) or snapshot.get("status") != CollisionStatus.LIVE.value:
        return False, "physics snapshot is not LIVE"
    if snapshot.get("unknown_contacts") != 0:
        return False, "unknown contact coverage"
    if snapshot.get("fall_candidate") is not False or snapshot.get("fall_confirmed") is not False:
        return False, "fall state is uncertain or confirmed"
    if snapshot.get("robot_obstacle_contacts") != 0:
        return False, "robot-obstacle contact is present"
    return True, None


def standing_physics_sample_failure(snapshot: object) -> str | None:
    """Validate one live physics-authority sample during the standing hold."""
    if (not math.isfinite(snapshot.sim_time) or snapshot.physics_coverage_complete != 1 or
            snapshot.unknown_contacts != 0):
        return "invalid physics time, incomplete coverage, or unknown contact"
    if snapshot.fall_candidate or snapshot.fall_confirmed:
        return "fall candidate/confirmation during standing hold"
    if snapshot.robot_obstacle_contacts != 0:
        return "robot-obstacle contact during standing hold"
    if (snapshot.base_height_m < 0.25 or abs(snapshot.base_roll_rad) > 0.30 or
            abs(snapshot.base_pitch_rad) > 0.30 or snapshot.foot_ground_contacts < 2):
        return "standing posture/support condition lost"
    return None


def standing_hold_reached(start_sim_s: float, current_sim_s: float,
                           requested_sim_s: float) -> bool:
    return (math.isfinite(start_sim_s) and math.isfinite(current_sim_s) and
            math.isfinite(requested_sim_s) and current_sim_s >= start_sim_s and
            current_sim_s - start_sim_s >= requested_sim_s)


def startup_low_height_only_fall(snapshot: dict, entry_sim_s: float) -> tuple[bool, str]:
    """Allow the diagnostic supervisor delay only for a confirmed low-height-only episode."""
    if not isinstance(snapshot, dict) or snapshot.get("status") != CollisionStatus.LIVE.value:
        return False, "physics authority is not LIVE"
    if snapshot.get("version") != 5 or snapshot.get("physics_coverage_complete") is not True:
        return False, "v5 physics coverage is unavailable or incomplete"
    if snapshot.get("unknown_contacts", 0) != 0 or snapshot.get("invalid_posture_steps", 0) != 0:
        return False, "unknown contact or invalid posture data"
    flags = snapshot.get("fall_condition_flags")
    if type(flags) is not int or flags != S6_FALL_LOW_HEIGHT:
        return False, "fall cause is not proven to be low-height only"
    if snapshot.get("fall_candidate") is not True or snapshot.get("fall_confirmed") is not True:
        return False, "fall episode is not currently confirmed and active"
    posture = (snapshot.get("base_height_m"), snapshot.get("base_roll_rad"), snapshot.get("base_pitch_rad"))
    if (any(not isinstance(value, (int, float)) or not math.isfinite(float(value)) for value in posture) or
            float(posture[0]) >= 0.22 or
            abs(float(posture[1])) >= math.radians(60.0) or
            abs(float(posture[2])) >= math.radians(60.0)):
        return False, "current posture is not low-height-only"
    history = snapshot.get("fall_history")
    if not isinstance(history, list) or not history:
        return False, "confirmed fall event history is missing"
    latest = history[-1]
    if (not isinstance(latest, dict) or latest.get("confirmed") is not True or
            latest.get("closed") is not False or
            not isinstance(latest.get("start_sim_time"), (int, float)) or
            float(latest["start_sim_time"]) < entry_sim_s):
        return False, "active confirmed fall episode is not bound to this RL startup window"
    return True, "confirmed low-height-only episode is within the explicit startup observation window"


def startup_normal_posture(snapshot: dict) -> tuple[bool, str | None]:
    """Check one authoritative sample used by the 0.30 s recovered-posture dwell."""
    if not isinstance(snapshot, dict) or snapshot.get("status") != CollisionStatus.LIVE.value:
        return False, "physics authority is not LIVE"
    if snapshot.get("version") != 5 or snapshot.get("physics_coverage_complete") is not True:
        return False, "physics publisher coverage is incomplete"
    if snapshot.get("unknown_contacts", 0) != 0 or snapshot.get("invalid_posture_steps", 0) != 0:
        return False, "unknown contact or invalid posture data"
    values = (snapshot.get("base_height_m"), snapshot.get("base_roll_rad"), snapshot.get("base_pitch_rad"))
    if any(not isinstance(value, (int, float)) or not math.isfinite(float(value)) for value in values):
        return False, "posture sample is non-finite or absent"
    normal = (float(values[0]) >= 0.22 and abs(float(values[1])) < math.radians(60.0) and
              abs(float(values[2])) < math.radians(60.0) and
              snapshot.get("fall_candidate") is False and snapshot.get("fall_confirmed") is False)
    return normal, None if normal else "posture has not recovered or fall episode remains active"


def new_startup_observation_state(anchor: dict, requested_s: float, *,
                                 capture_id: str, model_fingerprint: str,
                                 scenario_id: str, session_id: int) -> dict:
    """Create the explicit manual-view monitor, clocked only by MuJoCo authority."""
    return {
        "anchor_sim_time_s": float(anchor["sim_time_s"]),
        "deadline_sim_time_s": float(anchor["sim_time_s"]) + float(requested_s),
        "requested_sim_s": float(requested_s),
        "capture_id": capture_id,
        "model_fingerprint": model_fingerprint,
        "scenario_id": scenario_id,
        "session_id": int(session_id),
        "last_sequence": int(anchor["sequence"]),
        "last_physics_step": int(anchor["physics_step"]),
        "last_monotonic_ns": int(anchor["monotonic_ns"]),
        "last_sim_time_s": float(anchor["sim_time_s"]),
        "low_height_seen": False,
        "low_height_tracker_confirmed": False,
        "recovered": False,
        "recovery_evidence_status": "PENDING",
        "recovery_evidence_reason": None,
        "normal_dwell_start_sim_s": None,
        "policy_frames": 0,
        "policy_clock_invalid_frames": 0,
        "policy_clock_invalid_sequences": [],
        "policy_stale_clock_frames": 0,
        "policy_read_status_counts": {},
        "policy_stale_sequences": [],
        "policy_nonlive_observations": 0,
        "policy_gaps": [],
        "physics_samples": 0,
        "physics_sample_gaps": 0,
        "events": [],
        "delay_low_height_fall": False,
        "last_physics_status": "LIVE",
        "last_policy_cycle": None,
        "terminal": None,
        "baseline_unknown_contact_steps": int(anchor.get("unknown_contact_steps", 0)),
        "baseline_invalid_posture_steps": int(anchor.get("invalid_posture_steps", 0)),
        "baseline_fall_history_count": int(anchor.get("fall_history_count", 0)),
        "last_fall_history_count": int(anchor.get("fall_history_count", 0)),
    }


def note_startup_policy_frame(state: dict, frame: object) -> dict:
    """Record policy coverage quality without making its clock a viewing clock."""
    cycle = int(frame.rl_step)
    previous = state["last_policy_cycle"]
    if previous is None:
        if cycle < 0:
            return {"status": "INVALID", "reason": "first active policy cycle is negative"}
        if cycle > 0:
            gap = {"previous_policy_cycle": None, "current_policy_cycle": cycle,
                   "missing_policy_cycles": cycle,
                   "session_id": int(frame.session_id),
                   "handling": "cycle 0 and intervening frames absent; recording incomplete; continue physics-clocked viewing"}
            state["policy_gaps"].append(gap)
            state["events"].append({"event": "policy_recording_gap", **gap})
    elif cycle <= previous:
        return {"status": "INVALID", "reason": "policy cycle did not advance monotonically"}
    elif cycle > previous + 1:
        gap = {"previous_policy_cycle": previous, "current_policy_cycle": cycle,
               "missing_policy_cycles": cycle - previous - 1,
               "session_id": int(frame.session_id),
               "handling": "recording incomplete; continue physics-clocked viewing; no interpolation"}
        state["policy_gaps"].append(gap)
        state["events"].append({"event": "policy_recording_gap", **gap})
    state["last_policy_cycle"] = cycle
    state["policy_frames"] += 1
    if not bool(frame.sim_clock_valid):
        sequence = int(frame.sequence)
        if sequence not in state["policy_clock_invalid_sequences"]:
            state["policy_clock_invalid_sequences"].append(sequence)
            state["policy_clock_invalid_frames"] += 1
            state["events"].append({"event": "policy_sim_clock_invalid",
                                    "policy_cycle": cycle,
                                    "sequence": sequence,
                                    "session_id": int(frame.session_id),
                                    "sim_clock_status": int(frame.sim_clock_status),
                                    "handling": "quality issue only; never replace policy time with physics time"})
            if int(frame.sim_clock_status) == SIM_CLOCK_STALE:
                state["policy_stale_clock_frames"] += 1
    return {"status": "OK"}


def note_startup_policy_read_status(state: dict, status: str, frame: object | None = None) -> None:
    """Count supervisor observations of non-live policy data separately from cycles."""
    counts = state["policy_read_status_counts"]
    counts[status] = counts.get(status, 0) + 1
    if status != FrameStatus.LIVE.value:
        state["policy_nonlive_observations"] += 1
    if status == FrameStatus.STALE.value and frame is not None:
        sequence = int(frame.sequence)
        if sequence not in state["policy_stale_sequences"]:
            state["policy_stale_sequences"].append(sequence)
    if frame is not None and not bool(frame.sim_clock_valid):
        sequence = int(frame.sequence)
        if sequence not in state["policy_clock_invalid_sequences"]:
            state["policy_clock_invalid_sequences"].append(sequence)
            state["policy_clock_invalid_frames"] += 1
            state["events"].append({"event": "policy_sim_clock_invalid",
                                    "policy_cycle": int(frame.rl_step),
                                    "sequence": sequence,
                                    "session_id": int(frame.session_id),
                                    "sim_clock_status": int(frame.sim_clock_status),
                                    "handling": "quality issue only; never replace policy time with physics time"})
            if int(frame.sim_clock_status) == SIM_CLOCK_STALE:
                state["policy_stale_clock_frames"] += 1


def advance_startup_observation(state: dict, physics_status: str,
                                snapshot: dict | None, observed_monotonic_ns: int) -> dict:
    """Advance the manual viewer from a same-run live MuJoCo physics snapshot.

    Policy-frame staleness/gaps are intentionally absent from this safety clock.
    The helper returns a terminal decision only for physics-source/safety failures
    or when the fixed physical-time observation window expires.
    """
    state["last_physics_status"] = physics_status

    def stop(reason: str, terminal: str) -> dict:
        if state.get("low_height_seen") and not state.get("recovered"):
            state["recovery_evidence_status"] = "INCOMPLETE"
            state["recovery_evidence_reason"] = reason
        result = {"status": "STOP", "reason": reason, "terminal": terminal,
                  "observed_monotonic_ns": observed_monotonic_ns,
                  "physics_status": physics_status}
        state["terminal"] = result
        state["events"].append({"event": "observation_stopped", **result})
        return result

    if physics_status != CollisionStatus.LIVE.value or not isinstance(snapshot, dict):
        return stop(f"MuJoCo physics authority is {physics_status}", "STARTUP_PHYSICS_SOURCE_FAILURE")
    if snapshot.get("version") != 5 or snapshot.get("authoritative") is not True:
        return stop("MuJoCo physics authority is not authoritative v5", "STARTUP_PHYSICS_SOURCE_FAILURE")
    if snapshot.get("physics_coverage_complete") is not True:
        return stop("MuJoCo physics event coverage is incomplete", "STARTUP_PHYSICS_SOURCE_FAILURE")
    if snapshot.get("event_latches_complete") is not True:
        return stop("MuJoCo event-latch coverage is incomplete", "STARTUP_PHYSICS_SOURCE_FAILURE")
    if (snapshot.get("collision_history_overflow") is True or
            snapshot.get("fall_history_overflow") is True):
        return stop("MuJoCo safety-event history overflowed", "STARTUP_PHYSICS_SOURCE_FAILURE")
    fall_history = snapshot.get("fall_history")
    fall_history_count = snapshot.get("fall_history_count")
    if (not isinstance(fall_history, list) or not isinstance(fall_history_count, int) or
            fall_history_count != len(fall_history) or
            fall_history_count < state["baseline_fall_history_count"] or
            fall_history_count < state["last_fall_history_count"]):
        return stop("MuJoCo fall history is missing, truncated, or regressed",
                    "STARTUP_RECOVERY_EVIDENCE_INCOMPLETE")
    last_fall_end = -math.inf
    for episode in fall_history:
        if not isinstance(episode, dict):
            return stop("MuJoCo fall history contains an invalid entry",
                        "STARTUP_RECOVERY_EVIDENCE_INCOMPLETE")
        start_s, end_s = episode.get("start_sim_time"), episode.get("end_sim_time")
        if (not isinstance(start_s, (int, float)) or not math.isfinite(float(start_s)) or
                not isinstance(end_s, (int, float)) or not math.isfinite(float(end_s)) or
                not isinstance(episode.get("closed"), bool) or float(end_s) < float(start_s) or
                float(start_s) < last_fall_end):
            return stop("MuJoCo fall history chronology or closure is invalid",
                        "STARTUP_RECOVERY_EVIDENCE_INCOMPLETE")
        last_fall_end = float(end_s)
    state["last_fall_history_count"] = fall_history_count
    if (snapshot.get("capture_id") != state["capture_id"] or
            snapshot.get("runtime_model_fingerprint") != state["model_fingerprint"] or
            snapshot.get("scenario_id") != state["scenario_id"]):
        return stop("MuJoCo physics source/run identity changed", "STARTUP_PHYSICS_SOURCE_FAILURE")
    sim_s = snapshot.get("sim_time")
    sequence = snapshot.get("sequence")
    physics_step = snapshot.get("physics_step")
    source_ns = snapshot.get("monotonic_ns")
    if (not isinstance(sim_s, (int, float)) or not math.isfinite(float(sim_s)) or
            not isinstance(sequence, int) or not isinstance(physics_step, int) or
            not isinstance(source_ns, int) or source_ns <= 0 or source_ns > observed_monotonic_ns):
        return stop("MuJoCo physics time/identity fields are invalid", "STARTUP_PHYSICS_SOURCE_FAILURE")
    if sequence < state["last_sequence"] or physics_step < state["last_physics_step"] or source_ns < state["last_monotonic_ns"]:
        return stop("MuJoCo physics sequence, step, or monotonic source regressed", "STARTUP_PHYSICS_SOURCE_FAILURE")
    if float(sim_s) < state["last_sim_time_s"] - 1e-9:
        return stop("MuJoCo simulation time regressed", "STARTUP_PHYSICS_SOURCE_FAILURE")
    if any(float(item["end_sim_time"]) > float(sim_s) + 1e-9 for item in fall_history):
        return stop("MuJoCo fall history extends beyond the current physics time",
                    "STARTUP_RECOVERY_EVIDENCE_INCOMPLETE")
    if sequence == state["last_sequence"]:
        return {"status": "CONTINUE", "reason": "same physics snapshot; no new sample",
                "elapsed_physics_s": max(0.0, float(sim_s) - state["anchor_sim_time_s"])}
    if source_ns <= state["last_monotonic_ns"]:
        return stop("new MuJoCo snapshot did not advance its monotonic timestamp", "STARTUP_PHYSICS_SOURCE_FAILURE")
    if physics_step > state["last_physics_step"] and float(sim_s) <= state["last_sim_time_s"] + 1e-12:
        # A paused simulator may publish the same physical step/time. A changed
        # physics step with no simulation-time advance is inconsistent.
        return stop("MuJoCo physics step advanced without simulation-time advance", "STARTUP_PHYSICS_SOURCE_FAILURE")
    if physics_step > state["last_physics_step"] + 1:
        state["physics_sample_gaps"] += physics_step - state["last_physics_step"] - 1
    state["last_sequence"] = sequence
    state["last_physics_step"] = physics_step
    state["last_monotonic_ns"] = source_ns
    state["last_sim_time_s"] = float(sim_s)
    state["physics_samples"] += 1

    unknown_steps = snapshot.get("unknown_contact_steps", 0)
    if (snapshot.get("unknown_contacts", 0) != 0 or
            unknown_steps > state["baseline_unknown_contact_steps"]):
        state["events"].append({"event": "recovery_evidence_unknown_contact",
                                "unknown_contacts": snapshot.get("unknown_contacts"),
                                "step_delta": unknown_steps - state["baseline_unknown_contact_steps"],
                                "first_step": snapshot.get("first_unknown_contact_step"),
                                "last_step": snapshot.get("last_unknown_contact_step"),
                                "first_sim_time_s": snapshot.get("first_unknown_contact_sim_time"),
                                "last_sim_time_s": snapshot.get("last_unknown_contact_sim_time")})
        return stop("MuJoCo contact classification became uncertain", "STARTUP_PHYSICS_SOURCE_FAILURE")
    invalid_steps = snapshot.get("invalid_posture_steps", 0)
    if invalid_steps > state["baseline_invalid_posture_steps"]:
        state["events"].append({"event": "recovery_evidence_invalid_posture",
                                "step_delta": invalid_steps - state["baseline_invalid_posture_steps"],
                                "first_step": snapshot.get("first_invalid_posture_step"),
                                "last_step": snapshot.get("last_invalid_posture_step"),
                                "first_sim_time_s": snapshot.get("first_invalid_posture_sim_time"),
                                "last_sim_time_s": snapshot.get("last_invalid_posture_sim_time")})
        return stop("MuJoCo posture source recorded invalid samples", "STARTUP_PHYSICS_SOURCE_FAILURE")
    flags = snapshot.get("fall_condition_flags")
    roll, pitch = snapshot.get("base_roll_rad"), snapshot.get("base_pitch_rad")
    height = snapshot.get("base_height_m")
    if (not isinstance(roll, (int, float)) or not math.isfinite(float(roll)) or
            not isinstance(pitch, (int, float)) or not math.isfinite(float(pitch))):
        return stop("MuJoCo posture values are unavailable", "STARTUP_PHYSICS_SOURCE_FAILURE")
    if not isinstance(height, (int, float)) or not math.isfinite(float(height)):
        return stop("MuJoCo base height is unavailable", "STARTUP_PHYSICS_SOURCE_FAILURE")
    if abs(float(roll)) >= math.radians(75.0) or abs(float(pitch)) >= math.radians(75.0):
        return stop("existing 75-degree body safety boundary reached", "SYSTEM_SAFETY_ABORT")
    collision_history = snapshot.get("collision_history")
    new_obstacle_collision = False
    if isinstance(collision_history, list):
        for item in collision_history:
            if not isinstance(item, dict):
                continue
            event_start, obstacle_id = item.get("start_sim_time"), item.get("obstacle_geom_id")
            if (isinstance(event_start, (int, float)) and math.isfinite(float(event_start)) and
                    event_start >= state["anchor_sim_time_s"] and
                    isinstance(obstacle_id, int) and obstacle_id >= 0):
                new_obstacle_collision = True
                break
    if (snapshot.get("robot_obstacle_contacts", 0) > 0 or
            snapshot.get("nonfoot_obstacle_contacts", 0) > 0 or new_obstacle_collision):
        return stop("robot-obstacle collision observed in MuJoCo authority", "COLLISION_TERMINATION")

    low_now = float(height) < 0.22
    history = fall_history
    latest = history[-1] if history else None
    latest_is_new_low_episode = (
        isinstance(latest, dict) and
        latest.get("start_sim_time", math.inf) >= state["anchor_sim_time_s"] and
        isinstance(flags, int) and bool(flags & S6_FALL_LOW_HEIGHT))
    if low_now or latest_is_new_low_episode:
        state["low_height_seen"] = True
    confirmed = snapshot.get("fall_confirmed") is True
    low_only_confirmed = confirmed and flags == S6_FALL_LOW_HEIGHT
    state["low_height_tracker_confirmed"] = bool(state["low_height_tracker_confirmed"] or low_only_confirmed)
    if confirmed and isinstance(flags, int) and flags & S6_FALL_TILT:
        return stop("PhysicsLoop confirmed a tilt-related fall", "FALL_TERMINATION")
    if confirmed and (state["recovered"] or not low_only_confirmed):
        return stop("ordinary confirmed fall after recovery or non-low-height fall", "FALL_TERMINATION")
    state["delay_low_height_fall"] = bool(low_only_confirmed and not state["recovered"])
    if state["delay_low_height_fall"] and not state.get("low_height_delay_logged"):
        state["low_height_delay_logged"] = True
        state["events"].append({"event": "confirmed_low_height_fall_terminal_delayed",
                                "sim_time_s": float(sim_s), "fall_condition_flags": flags,
                                "reason": "explicit bounded manual observation; tracker/event history unchanged"})

    normal, normal_reason = startup_normal_posture(snapshot)
    startup_falls = [item for item in history
                     if float(item["start_sim_time"]) >= state["anchor_sim_time_s"]]
    if normal and any(item.get("closed") is False for item in history):
        normal, normal_reason = False, "fall episode remains open"
    if state["low_height_seen"] and normal:
        if not startup_falls:
            state["recovery_evidence_status"] = "INCOMPLETE"
            state["recovery_evidence_reason"] = "no complete fall-history episode anchors the observed startup low-height event"
            normal, normal_reason = False, state["recovery_evidence_reason"]
        elif any(item.get("closed") is not True for item in startup_falls):
            state["recovery_evidence_status"] = "PENDING"
            state["recovery_evidence_reason"] = "startup fall episode has not closed"
            normal, normal_reason = False, state["recovery_evidence_reason"]
        else:
            # PhysicsLoop publishes cumulative, per-physical-step fall history.
            # Reader snapshots may skip steps; a complete publisher coverage flag,
            # complete history, and unchanged invalid/unknown accumulators bridge
            # that read gap without pretending the reader saw each physics step.
            latest_closed_end = max(float(item["end_sim_time"]) for item in startup_falls)
            previous_start = state["normal_dwell_start_sim_s"]
            if previous_start is None or abs(float(previous_start) - latest_closed_end) > 1e-9:
                state["normal_dwell_start_sim_s"] = latest_closed_end
                state["events"].append({"event": "normal_posture_dwell_anchored_to_closed_fall_history",
                                        "sim_time_s": latest_closed_end,
                                        "physics_step": max(int(item.get("end_physics_step", 0)) for item in startup_falls),
                                        "reader_physics_step": physics_step,
                                        "sampled_physics_steps_unobserved": state["physics_sample_gaps"],
                                        "producer_coverage_complete": True,
                                        "fall_history_complete": True,
                                        "invalid_posture_steps_since_anchor": int(snapshot.get("invalid_posture_steps", 0)) - state["baseline_invalid_posture_steps"],
                                        "unknown_contact_steps_since_anchor": int(snapshot.get("unknown_contact_steps", 0)) - state["baseline_unknown_contact_steps"]})
            state["recovery_evidence_status"] = "COMPLETE"
            state["recovery_evidence_reason"] = "continuous producer coverage and complete event accumulators show no later anomaly"
        if normal and float(sim_s) - float(state["normal_dwell_start_sim_s"]) >= 0.30:
            if not state["recovered"]:
                state["recovered"] = True
                state["recovery_sim_time_s"] = float(sim_s)
                state["events"].append({"event": "RECOVERED_AFTER_LOW_HEIGHT_EVENT",
                                        "sim_time_s": float(sim_s), "physics_step": physics_step,
                                        "continuous_normal_dwell_s": float(sim_s) - float(state["normal_dwell_start_sim_s"]),
                                        "dwell_origin": "closed fall-history end; producer covers every physical step",
                                        "low_height_event_retained": True,
                                        "safety_success_claimed": False})
    elif state["low_height_seen"]:
        state["recovery_evidence_status"] = "PENDING" if normal_reason == "fall episode remains open" else "INCOMPLETE"
        state["recovery_evidence_reason"] = normal_reason
        state["normal_dwell_start_sim_s"] = None

    elapsed = float(sim_s) - state["anchor_sim_time_s"]
    if float(sim_s) >= state["deadline_sim_time_s"]:
        if confirmed and state["delay_low_height_fall"]:
            return stop("four-second observation ended while a confirmed low-height episode remained active",
                        "FALL_TERMINATION")
        if state["low_height_seen"] and not state["recovered"]:
            terminal = ("STARTUP_OBSERVATION_UNRECOVERED_LIMIT"
                        if state.get("recovery_evidence_status") == "COMPLETE"
                        else "STARTUP_OBSERVATION_RECOVERY_UNKNOWN_LIMIT")
        else:
            terminal = "STARTUP_OBSERVATION_LIMIT"
        return stop("requested MuJoCo simulation-time observation limit reached", terminal)
    return {"status": "RECOVERED_CONTINUE" if state["recovered"] else "CONTINUE",
            "elapsed_physics_s": elapsed, "recovery_sim_time_s": state.get("recovery_sim_time_s"),
            "low_height_seen": state["low_height_seen"]}


def parse_startup_trace_lines(log_text: str) -> list[dict]:
    """Parse only controller-authored S6-04 JSON telemetry lines."""
    records = []
    for line in log_text.splitlines():
        marker = line.find(S6_TRACE_MARKER)
        if marker < 0:
            continue
        payload = json.loads(line[marker + len(S6_TRACE_MARKER):])
        if not isinstance(payload, dict) or payload.get("schema") != "s6-04-controller-startup-v1":
            raise ValueError("invalid S6-04 controller trace schema")
        records.append(payload)
    return records


def parse_startup_gain_trace_lines(log_text: str) -> list[dict]:
    """Parse controller-authored S6-05 entry and actual command-interface gain writes."""
    records = []
    for line in log_text.splitlines():
        marker = line.find(S6_GAIN_TRACE_MARKER)
        if marker < 0:
            continue
        payload = json.loads(line[marker + len(S6_GAIN_TRACE_MARKER):])
        if not isinstance(payload, dict) or payload.get("schema") != "s6-05-gain-v1":
            raise ValueError("invalid S6-05 startup gain trace schema")
        if payload.get("event") not in {"entry", "command_write", "transition_complete"}:
            raise ValueError("invalid S6-05 startup gain trace event")
        names = payload.get("joint_names")
        if not isinstance(names, list) or len(names) != 12 or len(set(names)) != 12:
            raise ValueError("S6-05 gain trace lacks 12 unique named joint channels")
        required_vectors = ("entry_kp", "entry_kd", "target_kp", "target_kd")
        if payload["event"] != "entry":
            required_vectors += ("written_kp", "written_kd")
        for field in required_vectors:
            values = payload.get(field)
            if not isinstance(values, list) or len(values) != 12 or any(
                    not isinstance(value, (int, float)) or not math.isfinite(value) for value in values):
                raise ValueError(f"invalid S6-05 gain vector: {field}")
        for field in ("session_id", "steady_ns"):
            if not isinstance(payload.get(field), int) or payload[field] <= 0:
                raise ValueError(f"invalid S6-05 gain trace identity: {field}")
        if payload["event"] != "entry":
            for field in ("update", "elapsed_s", "period_s", "alpha"):
                value = payload.get(field)
                if not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError(f"invalid S6-05 gain trace timing: {field}")
        records.append(payload)
    return records


def rl_entry_precondition_failure(readiness: dict) -> str | None:
    """Fail closed unless the shared recorder, standing, joint, and safety gates are ready."""
    required = {
        "recorder_ready": readiness.get("recorder_ready") is True,
        "standing_confirmed": readiness.get("standing_confirmed") is True,
        "joint_sample_valid": readiness.get("joint_sample_valid") is True,
        "joint_velocity_within_limit": readiness.get("joint_velocity_within_limit") is True,
        "pre_task_safety_boundary_live": readiness.get("pre_task_safety_boundary_live") is True,
    }
    missing = [name for name, passed in required.items() if not passed]
    return "RL entry refused; preparation gates failed: " + ", ".join(missing) if missing else None


def task_window_history_events(snapshot_payload: object, task_start_sim_s: float | None,
                               current_sim_s: float | None,
                               pre_task_physics_step: int | None) -> tuple[list[dict], list[dict]]:
    """Return cumulative collision/fall episodes that intersect the active task window."""
    if not isinstance(snapshot_payload, dict) or snapshot_payload.get("status") != CollisionStatus.LIVE.value:
        return [], []
    collisions = snapshot_payload.get("collision_history", [])
    falls = snapshot_payload.get("fall_history", [])
    if task_start_sim_s is not None and current_sim_s is not None:
        collisions = [event for event in collisions if isinstance(event, dict) and
                      event.get("end_sim_time", -math.inf) >= task_start_sim_s and
                      event.get("start_sim_time", math.inf) <= current_sim_s]
        falls = [event for event in falls if isinstance(event, dict) and
                 event.get("end_sim_time", -math.inf) >= task_start_sim_s and
                 event.get("start_sim_time", math.inf) <= current_sim_s]
    elif isinstance(pre_task_physics_step, int):
        # When the first active policy cycle has invalid sim_time, retain its
        # boundary and conservatively include any cumulative episode extending
        # past the final pre-RL physics step. Time-based metrics remain UNKNOWN.
        collisions = [event for event in collisions if isinstance(event, dict) and
                      event.get("end_physics_step", -1) > pre_task_physics_step]
        falls = [event for event in falls if isinstance(event, dict) and
                 event.get("end_physics_step", -1) > pre_task_physics_step]
    else:
        return [], []
    return list(collisions), list(falls)


def matching_processes(names: set[str]) -> list[dict]:
    """Find relevant processes by kernel comm name, avoiding cmdline self-matches."""
    found: list[dict] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            comm = (entry / "comm").read_text(encoding="utf-8").strip()
            if comm not in names:
                continue
            pid = int(entry.name)
            cmdline = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", errors="replace").strip()
            found.append({"pid": pid, "pgid": os.getpgid(pid), "comm": comm, "cmdline": cmdline})
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return found


def controller_node_inventory(node_list_output: str, source: str = "ros2 node list --no-daemon --spin-time 2") -> dict:
    """Count manager/controller names from one ROS graph snapshot without deduping."""
    expected = ("/controller_manager", "/rl_quadruped_controller")
    observed = [line.strip() for line in node_list_output.splitlines() if line.strip() in expected]
    counts = {name: observed.count(name) for name in expected}
    return {
        "source": source,
        "observed_expected_names": observed,
        "counts": counts,
        "unique": all(counts[name] == 1 for name in expected),
        "duplicates": [name for name in expected if counts[name] > 1],
        "missing": [name for name in expected if counts[name] == 0],
    }


def proc_environment(pid: int) -> dict[str, str]:
    """Read only ROS/runtime variables needed to explain a graph query."""
    wanted = {"ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY", "ROS_DISTRO",
              "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "PYTHONPATH", "PATH", "LD_LIBRARY_PATH"}
    values: dict[str, str] = {}
    try:
        raw = (Path("/proc") / str(pid) / "environ").read_bytes().split(b"\0")
        for item in raw:
            if b"=" not in item:
                continue
            key, value = item.split(b"=", 1)
            name = key.decode("utf-8", errors="replace")
            if name in wanted:
                values[name] = value.decode("utf-8", errors="replace")
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return {"status": "unavailable"}
    values.setdefault("ROS_DOMAIN_ID", "<unset; ROS default domain 0>")
    values.setdefault("RMW_IMPLEMENTATION", "<unset; RMW default selected by ROS>")
    return values


def proc_identity(pid: int) -> dict:
    identity: dict = {"pid": pid}
    try:
        identity["uid"] = (Path("/proc") / str(pid) / "status").read_text().split("Uid:", 1)[1].split()[0]
        identity["executable"] = os.readlink(f"/proc/{pid}/exe")
        identity["cmdline"] = ((Path("/proc") / str(pid) / "cmdline").read_bytes()
                               .replace(b"\0", b" ").decode("utf-8", errors="replace").strip())
        identity["process_environment"] = proc_environment(pid)
        maps = (Path("/proc") / str(pid) / "maps").read_text(encoding="utf-8", errors="replace")
        identity["rmw_libraries"] = sorted({line.split()[-1] for line in maps.splitlines()
                                             if "librmw_" in line and line.split()[-1].startswith("/")})
        identity["pgid"] = os.getpgid(pid)
    except (FileNotFoundError, PermissionError, ProcessLookupError, IndexError):
        identity["status"] = "process_exited_before_identity_capture"
    return identity


def process_descendants(root_pid: int) -> list[int]:
    """Return a bounded process-tree snapshot rooted at one ROS query wrapper."""
    parent_of: dict[int, int] = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry / "stat").read_text().rsplit(")", 1)[1].split()
            parent_of[int(entry.name)] = int(fields[1])
        except (FileNotFoundError, PermissionError, ProcessLookupError, IndexError, ValueError):
            continue
    result, frontier = [], [root_pid]
    while frontier:
        parent = frontier.pop(0)
        for child in (pid for pid, ppid in parent_of.items() if ppid == parent):
            if child not in result:
                result.append(child)
                frontier.append(child)
    return result


def ros_daemon_processes() -> list[dict]:
    """Snapshot ROS CLI daemon identities without signaling or modifying them."""
    found: list[dict] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            pid = int(entry.name)
            cmd = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", errors="replace")
            if "ros2 daemon" in cmd or "ros2cli.daemon" in cmd:
                found.append(proc_identity(pid))
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return found


def ros_query_observed(command: str, env: dict[str, str], timeout: float = 12.0) -> dict:
    """Run one sourced ROS CLI command and bind output, time, env and PID."""
    shell = (f"source /opt/ros/humble/setup.bash && source {ROS_WS}/install/setup.bash && "
             f"export AMENT_PREFIX_PATH={shlex.quote(env['S1_04_OVERLAY'])}:$AMENT_PREFIX_PATH && "
             "python3 -c 'import json,os,sys; keys=(\"ROS_DOMAIN_ID\",\"RMW_IMPLEMENTATION\",\"ROS_LOCALHOST_ONLY\",\"ROS_DISTRO\",\"AMENT_PREFIX_PATH\",\"COLCON_PREFIX_PATH\",\"PYTHONPATH\",\"PATH\",\"LD_LIBRARY_PATH\"); ros2=__import__(\"shutil\").which(\"ros2\"); shebang=open(ros2,encoding=\"utf-8\").readline().strip() if ros2 else None; "
             "print(\"__S6_ROS_QUERY_ENV__\"+json.dumps({\"environment_probe_python_only\":sys.executable,\"ros2_launcher\":ros2,\"ros2_shebang\":shebang,\"env\":{k:os.environ.get(k,\"<unset>\") for k in keys}},separators=(\",\",\":\")))' && "
             f"exec {command}")
    start_ns = now_ns()
    proc = subprocess.Popen(["bash", "-lc", shell], cwd=ROS_WS, env=env,
                            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    identity = None
    tree_identities: list[dict] = []
    for _ in range(30):
        time.sleep(0.01)
        identity = proc_identity(proc.pid)
        tree_identities = [proc_identity(pid) for pid in process_descendants(proc.pid)]
        if any("ros2" in item.get("cmdline", "") and item.get("rmw_libraries")
               for item in tree_identities):
            break
    try:
        output, _ = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        proc.kill()
        output, _ = proc.communicate()
        status = "timeout"
    else:
        status = "completed" if proc.returncode == 0 else "query_failed"
    finish_ns = now_ns()
    marker = next((line[len("__S6_ROS_QUERY_ENV__"):]
                   for line in output.splitlines() if line.startswith("__S6_ROS_QUERY_ENV__")), None)
    try:
        effective = json.loads(marker) if marker else None
    except json.JSONDecodeError:
        effective = {"parse_error": marker}
    query_output = "\n".join(line for line in output.splitlines()
                              if not line.startswith("__S6_ROS_QUERY_ENV__"))
    return {"command": command, "status": status, "returncode": proc.returncode,
            "start_monotonic_ns": start_ns, "end_monotonic_ns": finish_ns,
            "duration_ms": (finish_ns - start_ns) / 1e6,
            "query_process": identity, "query_process_tree": tree_identities,
            "effective_shell_environment": effective,
            "output": query_output}


def bounded_controller_graph_queries(env: dict[str, str], attempts: int = 5,
                                     retry_delay_s: float = 0.5) -> dict:
    """Require one fresh direct graph view; retry only missing discovery, never duplicates/errors."""
    daemon = ros_query_observed("ros2 node list", env, timeout=6.0)
    daemon["inventory"] = controller_node_inventory(daemon.get("output", ""),
                                                     "ros2 node list (ROS CLI daemon/cache)")
    daemon["daemon_processes"] = ros_daemon_processes()
    direct_attempts: list[dict] = []
    accepted = None
    stop_reason = None
    for attempt in range(1, attempts + 1):
        direct = ros_query_observed("ros2 node list --no-daemon --spin-time 2", env, timeout=8.0)
        inventory = controller_node_inventory(direct.get("output", ""),
                                              "ros2 node list --no-daemon --spin-time 2")
        direct.update({"attempt": attempt, "inventory": inventory})
        direct_attempts.append(direct)
        if direct.get("status") != "completed":
            stop_reason = "direct_query_failed"
            break
        if inventory["duplicates"]:
            stop_reason = "duplicate_controller_graph_names"
            break
        if inventory["unique"]:
            accepted = direct
            break
        if attempt < attempts:
            time.sleep(retry_delay_s)
    if accepted is None and stop_reason is None:
        stop_reason = "expected_nodes_missing_after_bounded_wait"
    return {"daemon_diagnostic": daemon, "direct_attempts": direct_attempts,
            "accepted_direct_attempt": accepted, "status": "confirmed_unique" if accepted else stop_reason,
            "attempt_limit": attempts, "retry_delay_s": retry_delay_s}


def mapped_library_paths(pid: int, soname: str) -> list[str]:
    """Return resolved paths for a named shared object mapped by one process."""
    paths = set()
    try:
        lines = (Path("/proc") / str(pid) / "maps").read_text(encoding="utf-8").splitlines()
    except (FileNotFoundError, PermissionError, ProcessLookupError) as exc:
        raise RuntimeError(f"cannot inspect controller process {pid} mappings: {exc}") from exc
    for line in lines:
        fields = line.split(None, 5)
        if len(fields) < 6:
            continue
        mapped = fields[5]
        if mapped.endswith(" (deleted)"):
            mapped = mapped[:-10]
        if Path(mapped).name == soname:
            paths.add(str(Path(mapped).resolve()))
    return sorted(paths)


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n")


def build_env(capture_id: str, overlay: Path, display: str, xauthority: str,
              scene: object, run_dir: Path) -> dict[str, str]:
    env = os.environ.copy()
    env["DISPLAY"] = display
    env["XAUTHORITY"] = xauthority
    env["S1_04_OVERLAY"] = str(overlay)
    env["LD_LIBRARY_PATH"] = f"{SDKLIB}:{LIBTORCH}:/usr/lib/x86_64-linux-gnu:{env.get('LD_LIBRARY_PATH', '')}"
    env["MUJOCO_GL"] = "glfw"
    env["ABS_P1_10_SCENARIO_ID"] = scene.scene_id
    env["ABS_P1_10_ROOT_XML_SHA256"] = scene.root_sha256
    env["ABS_P1_10_MODEL_CLOSURE_SHA256"] = scene.closure_sha256
    env["ABS_P1_10_CAPTURE_ID"] = capture_id
    env["ABS_P1_10_EXPECTED_MODEL_FINGERPRINT"] = scene.model_fingerprint
    resolved_run_dir = run_dir.resolve()
    env["ABS_MUJOCO_STANDING_PROMPT_MARKER"] = str(resolved_run_dir / "standing_prompt.marker")
    env["ABS_MUJOCO_STANDING_FRAMEBUFFER_CAPTURE"] = str(resolved_run_dir / "standing_hold.ppm")
    env["ABS_MUJOCO_FRAMEBUFFER_CAPTURE"] = str(resolved_run_dir / "live_panel.ppm")
    return env


def capture_one_joint_state(env: dict[str, str], timeout: float = 4.0) -> dict:
    """Read one actual joint position/velocity sample through the launch ROS graph."""
    helper = ROOT / "scripts/capture_joint_state_once.py"
    started_ns = now_ns()
    result = ros_command(f"/usr/bin/python3 {shlex.quote(str(helper))} --timeout-s 2.0", env,
                         timeout=timeout)
    ended_ns = now_ns()
    sample = None
    if result.returncode == 0:
        try:
            sample = json.loads(result.stdout.strip().splitlines()[-1])
        except (IndexError, json.JSONDecodeError):
            sample = None
    expected = {f"{leg}_{joint}_joint" for leg in ("FR", "FL", "RR", "RL")
                for joint in ("hip", "thigh", "calf")}
    valid = (isinstance(sample, dict) and len(sample.get("name", [])) == 12 and
             set(sample.get("name", [])) == expected and
             len(sample.get("position_rad", [])) == 12 and
             len(sample.get("velocity_rad_s", [])) == 12 and
             all(math.isfinite(float(value)) for value in
                 sample.get("position_rad", []) + sample.get("velocity_rad_s", [])))
    if valid:
        sample["joint_state_by_name"] = {
            name: {"position_rad": sample["position_rad"][index],
                   "velocity_rad_s": sample["velocity_rad_s"][index]}
            for index, name in enumerate(sample["name"])}
    return {"status": "captured" if valid else "unavailable",
            "capture_command_monotonic_ns": [started_ns, ended_ns],
            "sample": sample,
            "reason": None if valid else
                      f"joint-state sample unavailable (rc={result.returncode}): {result.stdout[-500:]}"}


def confirm_pre_rl_standing(capture_id: str, scene: object, wall_cap_s: float = 20.0,
                            stable_sim_s: float = 0.50, ros_log_path: Path | None = None,
                            last_command: int | None = None) -> dict:
    """Require a bounded healthy fixed-stand interval; observe confirmed falls to resolution."""
    started_ns = now_ns()
    deadline = time.monotonic() + wall_cap_s
    stable_start_s: float | None = None
    previous_sim_s: float | None = None
    previous_sequence: int | None = None
    observations: list[dict] = []
    resolved_setup_events: list[dict] = []
    preparation_events: list[dict] = []
    seen_sequences: set[int] = set()
    confirmed_fall_seen = False
    observed_fall_keys: set[tuple[int, float]] = set()
    resolved_fall_keys: set[tuple[int, float]] = set()

    def controller_sample() -> dict:
        state = "PASSIVE"
        last_stand_line = None
        stand_ready_line = None
        if ros_log_path is not None and ros_log_path.is_file():
            try:
                text = ros_log_path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                text = ""
            for line in text.splitlines():
                lower = line.lower()
                if "switched from passive to fixed down" in lower:
                    state = "FIXEDDOWN"
                elif "switched from fixed down to fixed stand" in lower:
                    state = "FIXEDSTAND"
                elif "switched from fixed stand to passive" in lower or "[hard-stop-confirmed]" in lower:
                    state = "PASSIVE"
                if "[stand-symm]" in lower:
                    last_stand_line = line.strip()
                if "[stand-ready]" in lower:
                    stand_ready_line = line.strip()
        return {"state_from_controller_log": state,
                "last_fixedstand_execution_log": last_stand_line,
                "stand_transition_ready": stand_ready_line is not None,
                "stand_transition_ready_log": stand_ready_line,
                "last_supervisor_command": last_command}

    def append_sample(sample: dict) -> None:
        if sample.get("sequence") in seen_sequences:
            return
        seen_sequences.add(sample.get("sequence"))
        observations.append(sample)

    while time.monotonic() < deadline:
        raw, diagnostic = read_collision_snapshot_diagnostic()
        validation_ns = now_ns()
        status, snap = classify_snapshot(raw, validation_ns, expected_capture_id=capture_id,
                                         expected_fingerprint=scene.model_fingerprint,
                                         expected_scene_binding=scene.binding())
        if status is not CollisionStatus.LIVE or snap is None:
            return {"status": "PREPARATION_FAILURE", "reason": f"physics authority {status.value} during pre-RL standing preparation",
                    "reader": diagnostic, "observations": observations,
                    "started_monotonic_ns": started_ns, "finished_monotonic_ns": now_ns(),
                    "wall_cap_s": wall_cap_s, "stable_required_sim_s": stable_sim_s}
        latest_fall = dict(snap.fall_history[-1]) if snap.fall_history else None
        failed = standing_physics_sample_failure(snap)
        controller = controller_sample()
        if failed is None and not controller["stand_transition_ready"]:
            failed = "controller fixed-stand ramp has not reached command-3 transition gate"
        if (failed is None and latest_fall is not None and latest_fall.get("confirmed") and
                not latest_fall.get("closed")):
            failed = "fall history retains a confirmed open episode"
        if snap.fall_confirmed:
            confirmed_fall_seen = True
            if latest_fall is not None:
                fall_key = (int(latest_fall.get("start_physics_step", 0)),
                            float(latest_fall.get("start_sim_time", 0.0)))
                if fall_key not in observed_fall_keys:
                    observed_fall_keys.add(fall_key)
                    preparation_events.append({"kind": "confirmed_fall", "phase": "preparation",
                        "start_sim_time_s": fall_key[1], "start_physics_step": fall_key[0],
                        "confirmed_sim_time_s": latest_fall.get("confirmed_sim_time"),
                        "confirmed_physics_step": latest_fall.get("confirmed_physics_step"),
                        "action": "continue_bounded_observation_but_refuse_RL_for_this_run"})
        elif latest_fall is not None and latest_fall.get("confirmed") and latest_fall.get("closed"):
            confirmed_fall_seen = True
            fall_key = (int(latest_fall.get("start_physics_step", 0)),
                        float(latest_fall.get("start_sim_time", 0.0)))
            if fall_key in observed_fall_keys and fall_key not in resolved_fall_keys:
                resolved_fall_keys.add(fall_key)
                preparation_events.append({"kind": "confirmed_fall_episode_closed_after_normal_posture",
                    "phase": "preparation", "start_sim_time_s": fall_key[1],
                    "start_physics_step": fall_key[0], "end_sim_time_s": latest_fall.get("end_sim_time"),
                    "end_physics_step": latest_fall.get("end_physics_step"),
                    "closed": True, "action": "continue_stability_observation_but_refuse_RL_for_this_run"})
        if failed is not None:
            stable_start_s = None
            previous_sim_s = snap.sim_time
            previous_sequence = snap.sequence
            append_sample({"sequence": snap.sequence, "physics_step": snap.physics_step,
                                 "sim_time_s": snap.sim_time, "status": "not_stable",
                                 "reason": failed, "base_height_m": snap.base_height_m,
                                 "base_roll_rad": snap.base_roll_rad, "base_pitch_rad": snap.base_pitch_rad,
                                 "foot_ground_contacts": snap.foot_ground_contacts,
                                 "fall_candidate": bool(snap.fall_candidate),
                                 "fall_confirmed": bool(snap.fall_confirmed),
                                 "fall_episode_active": bool(latest_fall and not latest_fall.get("closed", False)),
                                 "fall_episode": latest_fall,
                                 "invalid_posture_steps": snap.invalid_posture_steps,
                                 "first_invalid_posture_sim_time_s": snap.first_invalid_posture_sim_time,
                                 "last_invalid_posture_sim_time_s": snap.last_invalid_posture_sim_time,
                                 "controller": controller})
            if (snap.invalid_posture_steps > 0 and
                    (not preparation_events or preparation_events[-1].get("last_sim_time_s") != snap.last_invalid_posture_sim_time)):
                preparation_events.append({"kind": "invalid_posture_history", "phase": "preparation",
                                           "count_in_authority": snap.invalid_posture_steps,
                                           "first_sim_time_s": snap.first_invalid_posture_sim_time,
                                           "last_sim_time_s": snap.last_invalid_posture_sim_time,
                                           "action": "stability_window_must_begin_after_last_invalid_posture"})
            time.sleep(0.01)
            continue
        current_setup_invalid = (snap.invalid_posture_steps > 0 and stable_start_s is not None and
                                 snap.last_invalid_posture_sim_time > stable_start_s)
        if current_setup_invalid:
            resolved_setup_events.append({"kind": "invalid_posture", "last_sim_time_s": snap.last_invalid_posture_sim_time,
                                          "phase": "preparation", "action": "restart_stability_window"})
            stable_start_s = None
        if previous_sequence is not None and snap.sequence > previous_sequence and previous_sim_s is not None and snap.sim_time <= previous_sim_s:
            stable_start_s = None
        if stable_start_s is None:
            stable_start_s = snap.sim_time
        observation = {"sequence": snap.sequence, "physics_step": snap.physics_step,
                       "sim_time_s": snap.sim_time, "status": "stable_candidate",
                       "base_height_m": snap.base_height_m, "base_roll_rad": snap.base_roll_rad,
                       "base_pitch_rad": snap.base_pitch_rad,
                       "foot_ground_contacts": snap.foot_ground_contacts,
                       "fall_candidate": bool(snap.fall_candidate),
                       "fall_confirmed": bool(snap.fall_confirmed),
                       "fall_episode_active": bool(latest_fall and not latest_fall.get("closed", False)),
                       "fall_episode": latest_fall,
                       "unknown_contacts": snap.unknown_contacts,
                       "robot_obstacle_contacts": snap.robot_obstacle_contacts,
                       "controller": controller}
        append_sample(observation)
        if previous_sim_s is not None and snap.sim_time < previous_sim_s:
            return {"status": "PREPARATION_FAILURE", "reason": "simulation clock moved backward during pre-RL standing preparation",
                    "observations": observations, "started_monotonic_ns": started_ns,
                    "finished_monotonic_ns": now_ns(), "wall_cap_s": wall_cap_s,
                    "stable_required_sim_s": stable_sim_s}
        previous_sim_s, previous_sequence = snap.sim_time, snap.sequence
        if snap.sim_time - stable_start_s >= stable_sim_s:
            return {"status": "STABLE", "reason": None,
                    "recovered_after_confirmed_preparation_fall": confirmed_fall_seen,
                    "stable_sim_time_s": [stable_start_s, snap.sim_time],
                    "stable_elapsed_sim_s": snap.sim_time - stable_start_s,
                    "started_monotonic_ns": started_ns, "finished_monotonic_ns": now_ns(),
                    "wall_cap_s": wall_cap_s, "stable_required_sim_s": stable_sim_s,
                    "criteria": {"base_height_m_min": 0.25, "absolute_roll_rad_max": 0.30,
                                 "absolute_pitch_rad_max": 0.30, "foot_ground_contacts_min": 2,
                                 "physics_coverage_complete": True, "unknown_contacts": 0,
                                 "no_fall_candidate_or_confirmation": True,
                                 "no_robot_obstacle_contact": True,
                                 "controller_command3_gate_confirmed": True,
                                 "controller_command3_gate_log": controller["stand_transition_ready_log"]},
                    "resolved_setup_events": resolved_setup_events,
                    "preparation_events": preparation_events,
                    "posture_history_at_confirmation": {
                        "invalid_posture_steps": snap.invalid_posture_steps,
                        "first_invalid_posture_sim_time_s": snap.first_invalid_posture_sim_time,
                        "last_invalid_posture_sim_time_s": snap.last_invalid_posture_sim_time,
                        "all_observed_preparation_anomalies_retained": True},
                    "observations": observations,
                    "last_collision_snapshot": collision_snapshot_payload(status, snap)}
        time.sleep(0.01)
    latest = observations[-1] if observations else None
    return {"status": "PREPARATION_FAILURE", "reason": "stable standing interval not confirmed before wall cap",
            "started_monotonic_ns": started_ns, "finished_monotonic_ns": now_ns(),
            "wall_cap_s": wall_cap_s, "stable_required_sim_s": stable_sim_s,
            "resolved_setup_events": resolved_setup_events,
            "preparation_events": preparation_events, "observations": observations,
            "last_observation": latest}


def hold_setting_matches(switching_text: str, expected_s: float) -> bool:
    """Accept equivalent decimal spellings while rejecting malformed hold values."""
    match = re.search(
        r"(?m)^    recovery_min_hold_s:\s*([-+0-9.eE]+)\s*(?:#.*)?$",
        switching_text,
    )
    if match is None:
        return False
    try:
        actual = float(match.group(1))
    except ValueError:
        return False
    return math.isfinite(actual) and abs(actual - expected_s) <= 1e-6


def numeric_setting_matches(text: str, key: str, expected: float,
                            indentation: str = "    ", tolerance: float = 1e-9) -> bool:
    """Compare one YAML scalar numerically so equivalent decimal spellings agree."""
    if not math.isfinite(expected):
        return False
    pattern = rf"(?m)^{re.escape(indentation + key)}:\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*(?:#.*)?$"
    matches = re.findall(pattern, text)
    if len(matches) != 1:
        return False
    try:
        actual = float(matches[0])
    except ValueError:
        return False
    return math.isfinite(actual) and abs(actual - expected) <= tolerance


def overlay_configuration_matches(config_text: str, group: str, entry_threshold: float,
                                  exit_threshold: float, hold_duration_s: float,
                                  goal_x: float, goal_y: float = 0.0) -> bool:
    """Validate the effective group block and goal using numeric YAML scalar comparison."""
    match = re.search(r"(?ms)^  switching:\s*\n((?:^    .*\n|^\s*#.*\n)*)", config_text)
    if match is None:
        return False
    switching = match.group(1)
    group_matches = re.findall(r"(?m)^    group:\s*[\"']?([A-D])[\"']?\s*(?:#.*)?$", switching)
    if group_matches != [group]:
        return False
    has_exit = re.search(r"(?m)^    exit_threshold:", switching) is not None
    has_hold = re.search(r"(?m)^    recovery_min_hold_s:", switching) is not None
    if not numeric_setting_matches(switching, "entry_threshold", entry_threshold):
        return False
    if group in ("A", "C"):
        if has_exit:
            return False
    elif not has_exit or not numeric_setting_matches(switching, "exit_threshold", exit_threshold):
        return False
    if group in ("A", "B"):
        if has_hold:
            return False
    elif not has_hold or not hold_setting_matches(switching, hold_duration_s):
        return False
    legacy_keys = re.findall(r"(?m)^\s+(?:switching_mode|ra_threshold|recovery_hold_steps):", config_text)
    return (not legacy_keys and
            numeric_setting_matches(config_text, "goal_x", goal_x, indentation="  ") and
            numeric_setting_matches(config_text, "goal_y", goal_y, indentation="  "))


def ros_command(command: str, env: dict[str, str], timeout: float = 12.0) -> subprocess.CompletedProcess[str]:
    shell = (f"source /opt/ros/humble/setup.bash && source {ROS_WS}/install/setup.bash && "
             f"export AMENT_PREFIX_PATH={shlex.quote(env['S1_04_OVERLAY'])}:$AMENT_PREFIX_PATH && {command}")
    return subprocess.run(["bash", "-lc", shell], cwd=ROS_WS, env=env,
                          text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          timeout=timeout, check=False)


def publish(env: dict[str, str], command: int, lx: float = 0.0, ly: float = 0.0) -> dict:
    msg = f"{{command: {command}, lx: {lx}, ly: {ly}, rx: 0.0, ry: 0.0}}"
    result = ros_command(f"ros2 topic pub --once /control_input control_input_msgs/msg/Inputs '{msg}'", env)
    return {"command": command, "monotonic_ns": now_ns(), "returncode": result.returncode,
            "output": result.stdout[-1200:]}


def start_prepared_stop_publisher(env: dict[str, str]) -> tuple[subprocess.Popen[str], dict]:
    helper = ROOT / "scripts/s2_03_stop_publisher.py"
    shell = (f"source /opt/ros/humble/setup.bash && source {ROS_WS}/install/setup.bash && "
             f"export AMENT_PREFIX_PATH={shlex.quote(env['S1_04_OVERLAY'])}:$AMENT_PREFIX_PATH && "
             f"/usr/bin/python3 -u {shlex.quote(str(helper))}")
    proc = subprocess.Popen(["bash", "-lc", shell], cwd=ROS_WS, env=env, text=True,
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if proc.stdout is not None and select.select([proc.stdout], [], [], 0.1)[0]:
                line = proc.stdout.readline()
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if event.get("event") == "READY":
                    return proc, event
                if event.get("event") == "NOT_READY":
                    raise RuntimeError(f"prepared PASSIVE publisher did not become ready: {event}")
            if proc.poll() is not None:
                raise RuntimeError(f"prepared PASSIVE publisher exited before READY: rc={proc.returncode}")
        raise TimeoutError("prepared PASSIVE publisher READY timeout")
    except Exception:
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=2.0)
            except Exception:
                proc.kill()
                proc.wait(timeout=2.0)
        raise


def publish_prepared_command(proc: subprocess.Popen[str], token: str) -> dict:
    if proc.poll() is not None or proc.stdin is None or proc.stdout is None:
        raise RuntimeError("prepared control publisher is not alive")
    invoked_ns = now_ns()
    proc.stdin.write(f"{token}\n")
    proc.stdin.flush()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if select.select([proc.stdout], [], [], 0.1)[0]:
            line = proc.stdout.readline()
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if event.get("event") == "PUBLISHED" and event.get("token") == token:
                event["supervisor_invoked_monotonic_ns"] = invoked_ns
                event["supervisor_ack_received_monotonic_ns"] = now_ns()
                return event
            if event.get("event") in {"EOF", "NOT_READY"}:
                raise RuntimeError(f"prepared PASSIVE publisher failed: {event}")
    raise TimeoutError(f"prepared {token} publish acknowledgement timeout")


def publish_prepared_stop(proc: subprocess.Popen[str]) -> dict:
    """Backward-compatible command-1 request used by abnormal closeout."""
    return publish_prepared_command(proc, "STOP")


NORMAL_SHUTDOWN_RE = re.compile(r"\[NORMAL-SHUTDOWN\]\s+(.*)$")


def normal_shutdown_events(log_text: str, after_monotonic_ns: int) -> list[dict]:
    events: list[dict] = []
    for line in log_text.splitlines():
        match = NORMAL_SHUTDOWN_RE.search(line)
        if not match:
            continue
        fields = dict(re.findall(r"([a-z0-9_]+)=([^\s]+)", match.group(1)))
        try:
            steady_ns = int(fields["steady_ns"])
        except (KeyError, ValueError):
            continue
        if steady_ns < after_monotonic_ns:
            continue
        events.append({"event": "NORMAL_SHUTDOWN_STAGE", "steady_ns": steady_ns,
                       "phase": fields.get("phase", "UNKNOWN"), "fields": fields,
                       "log_line": line.strip()[-1600:]})
    events.sort(key=lambda event: event["steady_ns"])
    return events


def wait_controller_transition(log_file, ros_process: subprocess.Popen | None,
                               transition: str, timeout_s: float) -> str:
    """Wait for a specific startup FSM transition before sending the next command."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        log_file.flush()
        if ros_process is None or ros_process.poll() is not None:
            raise RuntimeError(f"ROS launch exited before controller transition {transition!r}")
        text = log_file.name and Path(log_file.name).read_text(encoding="utf-8", errors="replace")
        if transition in text:
            return transition
        time.sleep(0.05)
    raise TimeoutError(f"controller did not report startup transition {transition!r} within {timeout_s:g}s")


def perform_normal_shutdown(proc: subprocess.Popen[str], log_path: Path,
                            capture_id: str, scene: object,
                            ros_process: subprocess.Popen | None,
                            standing_prompt_marker: Path,
                            standing_observation_s: float,
                            wall_cap_s: float,
                            mujoco_process: subprocess.Popen | None = None) -> dict:
    """Decelerate, confirm standing, and observe a bounded standing hold."""
    request = publish_prepared_command(proc, "NORMAL_SHUTDOWN")
    request_ns = int(request["publish_start_monotonic_ns"])
    observed_physics: list[dict] = []
    standing_hold_physics: list[dict] = []
    seen_sequences: set[int] = set()
    deadline = time.monotonic() + wall_cap_s
    final_events: list[dict] = []
    failure_reason = None
    failure_physics_observation: dict | None = None
    hold_started_sim_s: float | None = None
    previous_sim_s: float | None = None
    last_sequence = -1
    while time.monotonic() < deadline:
        if ros_process is None or ros_process.poll() is not None:
            failure_reason = "ROS/controller process exited during normal closeout"
            break
        log_text = log_path.read_text(encoding="utf-8", errors="replace")
        final_events = normal_shutdown_events(log_text, request_ns)
        phases = [event["phase"] for event in final_events]
        if "FAILED" in phases or "ABORTED" in phases:
            failure_reason = f"controller reported normal closeout {phases[-1]}"
            break
        safety_line = next((line for line in log_text.splitlines()
                            if EMERGENCY_RE.search(line) or HARDSTOP_RE.search(line) or VETO_RE.search(line)), None)
        if safety_line is not None:
            failure_reason = "controller safety/hard-stop event during normal closeout: " + safety_line.strip()[-500:]
            break

        physics_raw, reader_diagnostic = read_collision_snapshot_diagnostic()
        validation_now_ns = now_ns()
        collision_status, snap = classify_snapshot(
            physics_raw, validation_now_ns, expected_capture_id=capture_id,
            expected_fingerprint=scene.model_fingerprint,
            expected_scene_binding=scene.binding())
        if collision_status is not CollisionStatus.LIVE or snap is None:
            failure_reason = f"collision/physics state became {collision_status.value} during closeout"
            try:
                stat = os.stat("/dev/shm/abs_collision_authority")
                shm_status = {"exists": True, "size": stat.st_size, "device": stat.st_dev,
                              "inode": stat.st_ino}
            except OSError as exc:
                shm_status = {"exists": False, "errno": exc.errno, "error": str(exc)}
            failure_physics_observation = {
                "classified_status": collision_status.value, "reader": reader_diagnostic,
                "mujoco_pid": mujoco_process.pid if mujoco_process is not None else None,
                "mujoco_returncode": mujoco_process.poll() if mujoco_process is not None else None,
                "controller_pid": ros_process.pid if ros_process is not None else None,
                "controller_returncode": ros_process.poll() if ros_process is not None else None,
                "shared_memory": shm_status, "observed_monotonic_ns": now_ns()}
            if snap is not None:
                failure_physics_observation["snapshot"] = {
                    "sequence": snap.sequence, "physics_step": snap.physics_step,
                    "sim_time_s": snap.sim_time, "snapshot_monotonic_ns": snap.monotonic_ns,
                    "age_ns_at_validation": validation_now_ns - snap.monotonic_ns,
                    "capture_id": snap.capture_id, "runtime_model_fingerprint": snap.runtime_model_fingerprint,
                    "scene_binding": scene.binding(), "authoritative": bool(snap.authoritative),
                    "physics_coverage_complete": bool(snap.physics_coverage_complete),
                    "collision_history_overflow": bool(snap.collision_history_overflow),
                    "fall_history_overflow": bool(snap.fall_history_overflow),
                }
            break
        if snap.sequence not in seen_sequences:
            if snap.sequence <= last_sequence:
                failure_reason = "physics snapshot sequence did not advance monotonically"
                break
            seen_sequences.add(snap.sequence)
            last_sequence = snap.sequence
            observed_physics.append({"sequence": snap.sequence, "physics_step": snap.physics_step,
                                     "sim_time_s": snap.sim_time, "base_height_m": snap.base_height_m,
                                     "base_roll_rad": snap.base_roll_rad,
                                     "base_pitch_rad": snap.base_pitch_rad,
                                     "ground_contacts": snap.ground_contacts,
                                     "foot_ground_contacts": snap.foot_ground_contacts,
                                     "physics_coverage_complete": bool(snap.physics_coverage_complete),
                                     "robot_obstacle_contacts": snap.robot_obstacle_contacts,
                                     "fall_candidate": bool(snap.fall_candidate),
                                     "fall_confirmed": bool(snap.fall_confirmed)})
            if "STANDING_HOLD" in phases:
                failure_reason = standing_physics_sample_failure(snap)
                if failure_reason is not None:
                    break
                if previous_sim_s is not None and snap.sim_time <= previous_sim_s:
                    failure_reason = "simulation time paused or moved backward during standing observation"
                    break
                if hold_started_sim_s is None:
                    hold_started_sim_s = snap.sim_time
                    standing_prompt_marker.touch()
                    print("[S2-07] 已到达，保持站立；正在观察仿真状态。", flush=True)
                previous_sim_s = snap.sim_time
                standing_hold_physics.append(observed_physics[-1])
        if snap.fall_confirmed:
            failure_reason = "fall confirmed by the live physics authority during closeout"
            break
        if snap.robot_obstacle_contacts > 0:
            failure_reason = "robot-obstacle contact observed by the live physics authority during closeout"
            break
        if (hold_started_sim_s is not None and previous_sim_s is not None and
                standing_hold_reached(hold_started_sim_s, previous_sim_s, standing_observation_s)):
            return {"request": request, "controller_stages": final_events,
                    "physics_observations": observed_physics,
                    "standing_hold_complete": True,
                    "standing_state": "FIXEDSTAND",
                    "standing_observation_sim_time_s": [hold_started_sim_s, previous_sim_s],
                    "standing_observation_requested_s": standing_observation_s,
                    "standing_observation_elapsed_s": previous_sim_s - hold_started_sim_s,
                    "standing_observation_samples": standing_hold_physics,
                    "failure_physics_observation": None,
                    "passive_requested": False,
                    "lowering_requested": False,
                    "confirmation_scope": "controller continuously checked standing feedback; physics authority supplied sampled posture/contact/coverage; no PASSIVE or lower command sent"}
        time.sleep(0.01)

    return {"request": request, "controller_stages": final_events,
            "physics_observations": observed_physics, "standing_hold_complete": False,
            "standing_state": "UNKNOWN",
            "standing_observation_sim_time_s": ([hold_started_sim_s, previous_sim_s]
                                                  if hold_started_sim_s is not None else None),
            "standing_observation_requested_s": standing_observation_s,
            "standing_observation_elapsed_s": (previous_sim_s - hold_started_sim_s
                                                 if hold_started_sim_s is not None and previous_sim_s is not None else 0.0),
            "standing_observation_samples": standing_hold_physics,
            "failure_physics_observation": failure_physics_observation,
            "failure_reason": failure_reason or "bounded normal closeout observation expired",
            "passive_requested": False, "lowering_requested": False,
            "confirmation_scope": "incomplete; standing hold was not fully observed"}


def hard_stop_confirmation(log_text: str, after_monotonic_ns: int) -> dict | None:
    """Return only a completed PASSIVE transition logged after this stop request."""
    for line in log_text.splitlines():
        match = HARDSTOP_CONFIRMED_RE.search(line)
        if not match or int(match.group("command")) not in (1, 9):
            continue
        steady_ns = int(match.group("steady_ns"))
        if steady_ns >= after_monotonic_ns:
            return {"event": "hard_stop_transition_complete", "command": int(match.group("command")),
                    "state": "PASSIVE", "controller_steady_monotonic_ns": steady_ns,
                    "message": line.strip()[-1000:]}
    return None


def verify_scene_authority_log(log_text: str, scene: object) -> dict:
    events = [line for line in log_text.splitlines() if "[ABS-SCENE-AUTHORITY]" in line]
    if len(events) != 1:
        raise RuntimeError(f"expected one scene authority log, found {len(events)}")
    fields = dict(re.findall(r"([a-z0-9_]+)=([^\s]+)", events[0]))
    expected = {"scene_id": scene.scene_id, "root_sha256": scene.root_sha256,
                "model_closure_sha256": scene.closure_sha256,
                "runtime_model_fingerprint": scene.model_fingerprint,
                "obstacle_count": str(scene.obstacle_count)}
    mismatch = {key: {"observed": fields.get(key), "expected": value}
                for key, value in expected.items() if fields.get(key) != value}
    if mismatch:
        raise RuntimeError(f"loaded MuJoCo scene authority mismatch: {mismatch}")
    return {"status": "confirmed", "fields": expected, "log_line": events[0].strip()}


class FrameCapture:
    def __init__(self, path: Path, run_id: str, capture_id: str, run_start_ns: int,
                 scene: object, candidate_mode: int, entry_threshold: float,
                 exit_threshold: float, hold_duration_s: float = 0.0):
        self.recorder = RunRecordRecorder(str(path), run_id=run_id, capture_id=capture_id,
                                          expected_fingerprint=scene.model_fingerprint,
                                          expected_scene_binding=scene.binding())
        self.stop = threading.Event()
        self.thread: threading.Thread | None = None  # Kept as an explicit no-worker state for closeout facts.
        self._lock = threading.Lock()
        self.error: str | None = None
        self.last_frame_key: tuple[int, int] | None = None
        self.unique_frames = 0
        self.bound_session_id: int | None = None
        self.source_mismatch: str | None = None
        self.last_frame_metadata: dict | None = None
        self.run_start_ns = run_start_ns
        self.pre_run_frames_ignored = 0
        self.candidate_mode = candidate_mode
        self.entry_threshold = entry_threshold
        self.exit_threshold = exit_threshold
        self.hold_duration_s = hold_duration_s

    def start(self) -> None:
        self.recorder.start()
        # The supervisor is the sole frame reader/writer. It records each exact
        # snapshot before evaluating it for a terminal event, so terminal capture
        # cannot race an asynchronous sampler or be overtaken by a later cycle.
        with self.recorder.path.open("r", encoding="utf-8") as stream:
            first_line = json.loads(stream.readline())
        if (first_line.get("kind") != "meta" or first_line.get("run_id") != self.recorder.run_id or
                first_line.get("capture_id") != self.recorder.capture_id):
            raise RuntimeError("recorder readiness readback did not match the active run/capture")

    def readiness_evidence(self) -> dict:
        size = self.recorder.path.stat().st_size
        return {"status": "ready_readable_and_writable", "path": str(self.recorder.path),
                "meta_bytes_flushed": size, "run_id": self.recorder.run_id,
                "capture_id": self.recorder.capture_id,
                "writer_open": self.recorder.state == "CAPTURING"}

    def observe(self, raw: bytes, status: FrameStatus, frame: object, sampled_ns: int) -> dict:
        """Persist the exact supervisor snapshot once, before terminal evaluation."""
        if status is not FrameStatus.LIVE or frame is None:
            return {"status": "not_live", "persisted": False}
        with self._lock:
            if self.stop.is_set() or self.recorder.stopped:
                self.error = "capture was stopped before the observed policy frame was persisted"
                return {"status": "failed", "persisted": False, "reason": self.error}
            if frame.session_id < self.run_start_ns:
                self.pre_run_frames_ignored += 1
                return {"status": "pre_run_ignored", "persisted": False}
            key = (frame.session_id, frame.sequence)
            if key == self.last_frame_key:
                return {"status": "already_persisted", "persisted": True,
                        "frame_ref": self.last_frame_metadata}
            if self.bound_session_id is None:
                self.bound_session_id = frame.session_id
            elif frame.session_id != self.bound_session_id:
                self.source_mismatch = f"runtime session changed {self.bound_session_id}->{frame.session_id}"
            if (frame.switching_mode != self.candidate_mode or
                    frame.version != FRAME_VERSION or
                    abs(frame.entry_threshold - self.entry_threshold) >= 1e-5 or
                    abs(frame.exit_threshold - self.exit_threshold) >= 1e-5 or
                    abs(frame.recovery_min_hold_s - self.hold_duration_s) >= 1e-5 or
                    bool(frame.hold_enabled) != (self.hold_duration_s > 0.0)):
                self.source_mismatch = (f"unexpected candidate/threshold: mode={frame.switching_mode}, "
                                        f"entry={frame.entry_threshold}, exit={frame.exit_threshold}, "
                                        f"hold={frame.recovery_min_hold_s}/{frame.hold_enabled}")
            ref = {"session_id": frame.session_id, "rl_step": frame.rl_step,
                   "source_sequence": frame.sequence,
                   "sim_time_s": frame.sim_time_s if frame.sim_clock_valid else None,
                   "monotonic_ns": frame.monotonic_ns,
                   "same_snapshot_persisted": True,
                   "policy_state": {0: "AGILE", 1: "RECOVERY", 2: "FAULTED"}.get(frame.policy_state, "UNKNOWN")}
            try:
                line = self.recorder.record_snapshot(raw, sampled_ns)
                payload = line.get("payload") or {}
                expected = (frame.session_id, frame.sequence, frame.rl_step, frame.monotonic_ns)
                stored = (payload.get("session_id"), payload.get("source_sequence"),
                          payload.get("rl_step"), payload.get("monotonic_ns"))
                required = ("ra_value", "mode_before", "mode_after", "action_source",
                            "action_raw", "action_clipped", "joint_target_rad", "world_pose",
                            "sim_time_s", "sim_clock_valid", "monotonic_ns")
                if line.get("status") != FrameStatus.LIVE.value or stored != expected or any(k not in payload for k in required):
                    self.error = f"persisted frame does not match observed snapshot: expected={expected}, stored={stored}"
                    return {"status": "failed", "persisted": False, "reason": self.error, "frame_ref": ref}
            except Exception as exc:
                self.error = f"{type(exc).__name__}: {exc}"
                return {"status": "failed", "persisted": False, "reason": self.error, "frame_ref": ref}
            self.last_frame_key = key
            self.last_frame_metadata = {**ref, "persisted": True}
            self.unique_frames += 1
            return {"status": "persisted", "persisted": True, "frame_ref": self.last_frame_metadata,
                    "collision_snapshot": payload.get("collision_snapshot")}

    def finish(self, facts: dict) -> dict:
        self.stop.set()
        if not self.recorder.stopped:
            self.recorder.stop_sampling()
        terminal = self.recorder.finalize(facts)
        return terminal


def finalize_run_closeout(out: Path, context: dict, process_facts: dict,
                          recorder: FrameCapture, capture_started: bool,
                          cleanup_processes) -> tuple[dict | None, list[dict]]:
    """Production closeout path: stop owned groups, finalize capture, persist artifacts.

    Artifact failures are collected and retried into run_context/process_facts. They
    cannot prevent the process cleanup callback from running because cleanup is the
    first closeout action.
    """
    errors: list[dict] = []
    terminal: dict | None = None
    try:
        cleanup_processes()
    except Exception as exc:
        errors.append({"stage": "process_cleanup", "error": f"{type(exc).__name__}: {exc}"})
    for item in process_facts.get("cleanup_errors", []):
        errors.append({"stage": f"process_cleanup:{item.get('stage', 'unknown')}",
                       "error": item.get("error", "cleanup error without detail")})
    if process_facts.get("shutdown_complete") is False:
        errors.append({"stage": "process_cleanup", "error": "owned process exit was not confirmed"})

    context.setdefault("lifecycle", {})
    try:
        write_json(out / "run_context.json", context)
    except Exception as exc:
        errors.append({"stage": "run_context_pre_finalize", "error": f"{type(exc).__name__}: {exc}"})

    if capture_started:
        try:
            terminal = recorder.finish(process_facts)
            context["lifecycle"]["terminal_record_finalized"] = True
        except Exception as exc:
            errors.append({"stage": "recorder_finalize", "error": f"{type(exc).__name__}: {exc}"})
            try:
                recorder.recorder.close()
            except Exception as close_exc:
                errors.append({"stage": "recorder_close", "error": f"{type(close_exc).__name__}: {close_exc}"})
        if terminal is not None:
            try:
                write_json(out / "terminal.json", terminal)
                context["lifecycle"]["terminal_json_saved"] = True
            except Exception as exc:
                errors.append({"stage": "terminal_json", "error": f"{type(exc).__name__}: {exc}"})
            if context.get("session_id") is None and terminal.get("last_session_id") is not None:
                context["session_id"] = terminal["last_session_id"]
    else:
        try:
            recorder.recorder.close()
        except Exception as exc:
            errors.append({"stage": "recorder_close", "error": f"{type(exc).__name__}: {exc}"})

    context["lifecycle"]["closeout_errors"] = errors
    process_facts["closeout_errors"] = errors
    process_facts["closeout_complete"] = not errors and (not capture_started or terminal is not None)
    # Persist process facts and context independently so one failed artifact does
    # not suppress the other. Retry both once with the accumulated error list.
    for path, value, stage in (
        (out / "process_facts.json", process_facts, "process_facts"),
        (out / "run_context.json", context, "run_context_final"),
    ):
        try:
            write_json(path, value)
        except Exception as exc:
            errors.append({"stage": stage, "error": f"{type(exc).__name__}: {exc}"})
    context["lifecycle"]["closeout_errors"] = errors
    process_facts["closeout_errors"] = errors
    process_facts["closeout_complete"] = not errors and (not capture_started or terminal is not None)
    for path, value in ((out / "process_facts.json", process_facts), (out / "run_context.json", context)):
        try:
            write_json(path, value)
        except Exception:
            # Last-resort persistence failures are returned to the caller/stdout;
            # do not raise past the already completed process cleanup.
            pass
    return terminal, errors


def extract_first_active_policy_state(ros_log_path: Path, record_path: Path) -> dict:
    """Pair the controller's measured state with the exact first active frame."""
    prefix = "[ABS-INITIAL-STATE] "
    candidates = []
    try:
        for line in ros_log_path.read_text(encoding="utf-8", errors="replace").splitlines():
            marker = line.find(prefix)
            if marker < 0:
                continue
            candidates.append(json.loads(line[marker + len(prefix):]))
    except (OSError, json.JSONDecodeError) as exc:
        return {"status": "UNKNOWN", "reason": f"initial_state_log_read_or_parse_failed:{exc}"}
    if len(candidates) != 1:
        return {"status": "UNKNOWN", "reason": "expected_exactly_one_first_active_state_log",
                "log_count": len(candidates)}
    state = candidates[0]
    try:
        frames = []
        with record_path.open("r", encoding="utf-8") as stream:
            for line in stream:
                item = json.loads(line)
                payload = item.get("payload")
                if (item.get("kind") == "frame" and item.get("status") == "LIVE" and
                        isinstance(payload, dict) and payload.get("rl_entered") == 1 and
                        payload.get("rl_active") == 1):
                    frames.append(payload)
    except (OSError, json.JSONDecodeError) as exc:
        return {"status": "UNKNOWN", "reason": f"runtime_record_read_or_parse_failed:{exc}"}
    if not frames:
        return {"status": "UNKNOWN", "reason": "no_persisted_active_policy_frame"}
    first = min(frames, key=lambda item: (item.get("session_id", math.inf), item.get("rl_step", math.inf)))
    identity = (state.get("session_id"), state.get("cycle_id"), state.get("frame_monotonic_ns"))
    frame_identity = (first.get("session_id"), first.get("rl_step"), first.get("monotonic_ns"))
    if state.get("cycle_id") != 0 or first.get("rl_step") != 0:
        return {"status": "UNKNOWN", "reason": "cycle_zero_runtime_frame_not_persisted",
                "logged_identity": list(identity), "record_identity": list(frame_identity),
                "same_cycle_runtime_frame_persisted_by_supervisor": False}
    exact_persisted_match = identity == frame_identity
    if not exact_persisted_match:
        return {"status": "UNKNOWN", "reason": "first_active_policy_cycle_not_persisted_as_the_same_runtime_frame",
                "logged_identity": list(identity), "record_identity": list(frame_identity),
                "same_cycle_runtime_frame_persisted_by_supervisor": False}
    required_payload = ("session_id", "source_sequence", "rl_step", "monotonic_ns",
                        "rl_entered", "rl_active", "ra_value", "mode_before", "mode_after",
                        "action_source", "action_raw", "action_clipped", "joint_target_rad",
                        "world_pose", "sim_time_s", "sim_clock_valid", "collision_snapshot")
    missing_payload = [name for name in required_payload if name not in first]
    if missing_payload:
        return {"status": "UNKNOWN", "reason": "cycle_zero_runtime_frame_incomplete",
                "missing_runtime_payload_fields": missing_payload,
                "logged_identity": list(identity), "record_identity": list(frame_identity),
                "same_cycle_runtime_frame_persisted_by_supervisor": False}
    arrays = (("joint_order", 12), ("joint_position_rad", 12), ("joint_velocity_rad_s", 12),
              ("imu_quaternion", 4), ("body_angular_velocity_rad_s", 3),
              ("body_linear_velocity_m_s", 3), ("world_position_xy_m", 2))
    for name, length in arrays:
        value = state.get(name)
        if not isinstance(value, list) or len(value) != length:
            return {"status": "UNKNOWN", "reason": f"invalid_initial_state_field:{name}"}
        if name != "joint_order" and any(not isinstance(x, (int, float)) or not math.isfinite(x) for x in value):
            return {"status": "UNKNOWN", "reason": f"nonfinite_initial_state_field:{name}"}
    if len(set(state["joint_order"])) != 12:
        return {"status": "UNKNOWN", "reason": "joint_order_is_not_unique"}
    if not isinstance(state.get("world_yaw_rad"), (int, float)) or not math.isfinite(state["world_yaw_rad"]):
        return {"status": "UNKNOWN", "reason": "invalid_world_yaw"}
    return {"status": "CAPTURED", "source": "StateRL measured input emitted synchronously after successful writeRtFrame for active cycle 0",
            "session_id": identity[0], "policy_cycle_id": identity[1],
            "frame_monotonic_ns": identity[2],
            "same_cycle_runtime_frame_persisted_by_supervisor": True,
            "first_persisted_active_cycle_id": frame_identity[1],
            "supervisor_initial_cycle_gap": 0,
            "sim_time_valid": state.get("sim_clock_valid"),
            "sim_time_s": state.get("sim_time_s"),
            "first_persisted_active_frame_identity": list(frame_identity),
            "joint_order": state["joint_order"], "joint_position_rad": state["joint_position_rad"],
            "joint_velocity_rad_s": state["joint_velocity_rad_s"],
            "imu_quaternion_order": state.get("imu_quaternion_order"),
            "imu_quaternion": state["imu_quaternion"],
            "body_angular_velocity_rad_s": state["body_angular_velocity_rad_s"],
            "body_linear_velocity_m_s": state["body_linear_velocity_m_s"],
            "world_position_xy_m": state["world_position_xy_m"], "world_yaw_rad": state["world_yaw_rad"]}


def run(args: argparse.Namespace) -> int:
    try:
        scene_spec = resolve_scene(args.scene)
    except (OSError, ValueError) as exc:
        print(json.dumps({"result": "SCENE_PREFLIGHT_REJECTED", "scene": args.scene,
                          "reason": str(exc)}, ensure_ascii=False))
        return 2
    run_id = uuid.uuid4().hex
    capture_id = "p1-10-capture-" + uuid.uuid4().hex
    out = args.output_root / run_id
    out.mkdir(parents=True, exist_ok=False)
    overlay = Path(args.overlay).resolve()
    group = args.group
    candidate = {"A": "paper_faithful_switch", "B": "hysteresis_switch",
                 "C": "minimum_hold_switch", "D": "combined_switch"}[group]
    candidate_mode = {"A": 1, "B": 2, "C": 3, "D": 4}[group]
    entry_threshold = args.entry_threshold
    exit_threshold = args.exit_threshold if group in ("B", "D") else entry_threshold
    hold_duration_s = args.hold_s if group in ("C", "D") else 0.0
    hysteresis = "true" if group in ("B", "D") else "false"
    hold = "true" if group in ("C", "D") else "false"
    env = build_env(capture_id, overlay, args.display, args.xauthority, scene_spec, out)
    if args.startup_gain_trace:
        env["ABS_S6_STARTUP_GAIN_TRACE"] = "1"
    if args.startup_recovery_observation_s > 0.0:
        (out / "startup_observation.enabled").write_text("explicit S6-04 diagnostic only\n", encoding="utf-8")
        env["ABS_S6_04_STARTUP_TRACE"] = "1"
    start_wall_ns = now_ns()
    recorder = FrameCapture(out / "runtime_record.jsonl", run_id, capture_id,
                            start_wall_ns, scene_spec, candidate_mode,
                            entry_threshold, exit_threshold, hold_duration_s)
    log_path = out / "ros_launch.log"
    mujoco_log_path = out / "mujoco.log"
    ros_log = log_path.open("w", encoding="utf-8")
    mujoco_log = mujoco_log_path.open("w", encoding="utf-8")
    mujoco: subprocess.Popen | None = None
    ros: subprocess.Popen | None = None
    stop_publisher: subprocess.Popen[str] | None = None
    post_terminal_shutdown: dict | None = None
    process_facts = {"exit_code": None, "forced_termination": False,
                     "shutdown_complete": False, "shutdown_request_source": "bounded_diagnostic_supervisor",
                     "external_safety_events": []}
    controls: list[dict] = []
    terminal_cause = "STARTUP_FAILURE"
    movement_start_sim_s: float | None = None
    task_start_sim_s: float | None = None
    task_start_physics_step: int | None = None
    sim_start_s: float | None = None
    first_pose: tuple[float, float] | None = None
    previous_pose_time_s: float | None = None
    first_frame_ns: int | None = None
    last_frame_key: tuple[int, int] | None = None
    frame_gap_count = 0
    safety_line_offset = 0
    startup_entry_marker: dict | None = None
    startup_clock_anchor: dict | None = None
    startup_monitor_state: dict | None = None
    startup_observation_event: dict | None = None
    start_event_ns: int | None = None
    finish_event_ns: int | None = None
    terminal_event_ns: int | None = None
    frame_rows = 0
    collision_live_seen = False
    startup_error: str | None = None
    capture_started = False
    bound_session_id: int | None = None
    source_mismatch: str | None = None
    source_wait_started: float | None = None
    terminal_frame_ref: dict | None = None
    terminal_summary: dict | None = None
    lifecycle_events: list[dict] = []
    motion_start_state_capture: dict = {"status": "not_started"}
    motion_start_capture_thread: threading.Thread | None = None

    context = {
        "schema": "s1-run-context-v1", "run_id": run_id, "capture_id": capture_id,
        "classification": ("controller initialization config-consumption preflight; no RL requested"
                            if args.config_only else
                            "bounded fixed-stand preparation observation; command 3 forbidden"
                            if args.prepare_only else args.classification),
        "created_monotonic_ns": start_wall_ns,
        "settings": {"scene": str(scene_spec.path), "scene_filename": scene_spec.filename,
                     "scene_id": scene_spec.scene_id,
                     "scene_obstacle_count": scene_spec.obstacle_count,
                     "scene_root_sha256": scene_spec.root_sha256,
                     "scene_model_closure_sha256": scene_spec.closure_sha256,
                     "runtime_model_fingerprint": scene_spec.model_fingerprint,
                     "candidate": candidate, "group": group,
                     "entry_threshold": entry_threshold, "exit_threshold": exit_threshold,
                     "hysteresis": hysteresis, "hold": hold,
                     "recovery_min_hold_s": hold_duration_s,
                     "goal_world_m": [args.goal_x, 0.0], "arrival_threshold_m": 0.5,
                     "diagnostic_sim_limit_after_motion_s": args.sim_limit_s,
                     "standing_observation_sim_s": args.standing_observation_sim_s,
                     "startup_recovery_observation_s": args.startup_recovery_observation_s,
                     "startup_gain_transition_trace_requested": args.startup_gain_trace,
                     "startup_gain_transition_expected_s": 0.30,
                     "startup_gain_transition_clock": "actual controller period accumulation; not sim_time or Recovery hold",
                     "startup_recovery_scope": ("low-height-only supervisor delay; tracker unchanged; simulation diagnostic only"
                                                  if args.startup_recovery_observation_s > 0 else "disabled"),
                     "standing_observation_wall_cap_s": args.standing_observation_wall_cap_s,
                     "controller_config": str(overlay / "share/go2_description/config/abs/config.yaml"),
                     "model_paths": [str(overlay / "share/go2_description/config/abs/policy.pt"),
                                     str(overlay / "share/go2_description/config/abs/ra_value.pt"),
                                     str(overlay / "share/go2_description/config/rec/policy.pt")],
                     "display": env.get("DISPLAY"),
                     "xauthority": env.get("XAUTHORITY"), "overlay": str(overlay),
                     "fall_rule": {"base_body": "base_link", "height_threshold_m": 0.22,
                                   "tilt_threshold_deg": 60.0, "duration_sim_s": 0.30,
                                   "combination": "height OR absolute roll OR absolute pitch",
                                   "duration_boundary": "candidate must hold continuously for >=0.30 valid sim seconds"},
                     "collision_scope": "PhysicsLoop mj_step; bound robot-obstacle contact episodes; ground, self, other, and unknown contacts retained separately"},
        "safety_result_contract": {
            "task_start": "first active policy decision frame with rl_entered=1 and rl_active=1",
            "legacy_motion_start": "first valid policy pose displacement >=0.10m from initial pose; auxiliary historical metric only",
            "collision_terminal": "immediate once an accumulated robot-obstacle episode intersects inclusive motion interval",
            "terminal_priority_same_observation": ["SYSTEM_SAFETY_ABORT", "COLLISION_TERMINATION",
                                                    "FALL_TERMINATION", "ARRIVED",
                                                    "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE"],
            "cleanup_is_not_motion": True,
            "policy_stop_request_is_not_stop_confirmation": True,
            "startup_recovery_observation": ("explicit one-off simulation diagnostic; no delay outside this invocation"
                                             if args.startup_recovery_observation_s > 0 else "disabled"),
            "mujoco_command_consumption": "UNKNOWN",
        },
        "lifecycle": {"task_start_event": "first active policy decision frame with rl_entered=1 and rl_active=1",
                      "task_start": None,
                      "motion_start_event": "first observed base displacement >=0.10m from first valid strategy-frame pose; auxiliary historical boundary",
                      "motion_start_sim_time_s": None,
                      "motion_start_interval_s": None,
                      "run_terminal_event": None,
                      "movement_end_monotonic_ns": None,
                      "cleanup_complete_monotonic_ns": None,
                      "terminal_frame_capture": {"status": "not_terminal", "complete": None},
                      "motion_and_cleanup_are_separate": True},
        "initial_state": {
            "reset_state": {"status": "available_from_scene_startup_probe",
                            "source": "mj_loadXML->mj_makeData->mj_forward for exact scene XML",
                            "scene": str(scene_spec.path),
                            "note": "construction/reset state; distinct from later measured motion-start state"},
            "motion_start_measured_state": {"status": "pending",
                                            "source": "authoritative runtime strategy frame plus /joint_states sample"},
            "pre_rl_joint_state": {"status": "pending",
                                   "source": "/joint_states during PASSIVE/FIXEDSTAND preparation; asynchronous to policy cycles"},
        },
        "commands": controls,
        "preflight": {"scene_binding": scene_spec.binding(),
                      "mujoco_launch_command": [str(MUJOCO_BIN), "-s", scene_spec.filename],
                      "ray_collision_geometry_selector_shared": True},
        "events": lifecycle_events,
    }
    write_json(out / "run_context.json", context)

    try:
        existing_processes = matching_processes({"ros2_control_node", "ros2_control_no", "controller_manager"})
        controller_process_lines = existing_processes
        if controller_process_lines:
            raise RuntimeError("pre-existing controller-manager process found; left untouched: " + " | ".join(str(item) for item in controller_process_lines))
        package_prefix = ros_command("ros2 pkg prefix go2_description", env)
        if package_prefix.returncode != 0 or Path(package_prefix.stdout.strip()).resolve() != overlay.resolve():
            raise RuntimeError(f"go2_description overlay was not selected: rc={package_prefix.returncode}, prefix={package_prefix.stdout.strip()!r}")
        controller_prefix = ros_command("ros2 pkg prefix rl_quadruped_controller", env)
        expected_controller_prefix = (ROS_WS / "install/rl_quadruped_controller").resolve()
        if controller_prefix.returncode != 0 or Path(controller_prefix.stdout.strip()).resolve() != expected_controller_prefix:
            raise RuntimeError(f"controller package prefix mismatch in launch shell: rc={controller_prefix.returncode}, prefix={controller_prefix.stdout.strip()!r}")
        share_check = ros_command(
            "python3 -c 'from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory(\"go2_description\"))'", env)
        if share_check.returncode != 0 or Path(share_check.stdout.strip()).resolve() != (overlay / "share/go2_description").resolve():
            raise RuntimeError(f"launch-equivalent ament share lookup differs from overlay: {share_check.stdout.strip()!r}")
        config_path = overlay / "share/go2_description/config/abs/config.yaml"
        config_text = config_path.read_text(encoding="utf-8")
        if not overlay_configuration_matches(config_text, group, entry_threshold,
                                             exit_threshold, hold_duration_s,
                                             args.goal_x, 0.0):
            raise RuntimeError(f"overlay config does not satisfy abs.switching {group} contract or expected goal")
        missing_models = [path for path in context["settings"]["model_paths"] if not Path(path).is_file()]
        if missing_models:
            raise RuntimeError("expected deployment model path missing: " + ", ".join(missing_models))
        context["preflight"] = {"controller_processes_before_launch": controller_process_lines,
                                "launch_shell_go2_description_prefix": package_prefix.stdout.strip(),
                                "launch_shell_controller_prefix": controller_prefix.stdout.strip(),
                                "ament_share_lookup_same_shell": share_check.stdout.strip(),
                                "candidate_config_path": str(config_path),
                                "candidate_config_group_verified": group,
                                "candidate_config_entry_E_verified": True,
                                "candidate_config_legacy_fields_absent": True,
                                "candidate_config_goal_x_m_verified": True,
                                "launch_command": "source /opt/ros/humble/setup.bash; source workspace/install/setup.bash; export the same AMENT_PREFIX_PATH; ros2 launch ..."}
        write_json(out / "run_context.json", context)
        recorder.start()
        capture_started = True
        recorder_readiness = recorder.readiness_evidence()
        context["preflight"]["recorder_readiness"] = {**recorder_readiness,
                                                      "monotonic_ns": now_ns(),
                                                      "required_before_rl": True}
        lifecycle_events.append({"phase": "startup_preparation", "event": "recorder_ready",
                                 "monotonic_ns": context["preflight"]["recorder_readiness"]["monotonic_ns"],
                                 "evidence": recorder_readiness})
        mujoco = subprocess.Popen([str(MUJOCO_BIN), "-s", scene_spec.filename], cwd=MUJOCO_DIR,
                                  env=env, stdout=mujoco_log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        time.sleep(3.0)
        if mujoco.poll() is not None:
            raise RuntimeError(f"MuJoCo exited at startup with code {mujoco.returncode}")
        observed_mujoco_exe = os.readlink(f"/proc/{mujoco.pid}/exe")
        context["preflight"]["mujoco_executable"] = {
            "launch_path": str(MUJOCO_BIN.resolve()), "pid": mujoco.pid,
            "observed_proc_exe": observed_mujoco_exe,
            "matches_expected_build": Path(observed_mujoco_exe).resolve() == MUJOCO_BIN.resolve(),
        }
        if not context["preflight"]["mujoco_executable"]["matches_expected_build"]:
            raise RuntimeError("running MuJoCo executable differs from expected build2 binary")
        write_json(out / "run_context.json", context)
        lifecycle_events.append({"phase": "startup_preparation", "event": "mujoco_started", "monotonic_ns": now_ns()})
        # Require a correctly bound LIVE collision authority before controller/motion startup.
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            status, snap = classify_snapshot(read_collision_snapshot(), now_ns(),
                                             expected_capture_id=capture_id,
                                             expected_fingerprint=scene_spec.model_fingerprint,
                                             expected_scene_binding=scene_spec.binding())
            if status is CollisionStatus.LIVE and snap is not None:
                collision_live_seen = True
                break
            time.sleep(0.02)
        if not collision_live_seen:
            raise RuntimeError("collision authority did not become LIVE for this capture/model; no motion started")
        context["preflight"]["loaded_mujoco_scene_authority"] = verify_scene_authority_log(
            mujoco_log_path.read_text(encoding="utf-8", errors="replace"), scene_spec)
        context["preflight"]["collision_reader_scene_binding"] = {
            "status": "LIVE", "scene_id": snap.scenario_id,
            "root_sha256": snap.scene_root_sha256,
            "model_closure_sha256": snap.model_closure_sha256,
            "runtime_model_fingerprint": snap.runtime_model_fingerprint,
            "obstacle_count": scene_spec.obstacle_count}
        write_json(out / "run_context.json", context)
        lifecycle_events.append({"phase": "startup_preparation", "event": "collision_source_live", "monotonic_ns": now_ns()})
        launch_shell = (f"source /opt/ros/humble/setup.bash && source {ROS_WS}/install/setup.bash && "
                        f"export AMENT_PREFIX_PATH={shlex.quote(env['S1_04_OVERLAY'])}:$AMENT_PREFIX_PATH && "
                        "ros2 launch rl_quadruped_controller mujoco.launch.py simulation_test:=false")
        ros = subprocess.Popen(["bash", "-lc", launch_shell],
                               cwd=ROS_WS, env=env, stdout=ros_log, stderr=subprocess.STDOUT,
                               start_new_session=True)
        time.sleep(8.0)
        if ros.poll() is not None:
            raise RuntimeError(f"ROS launch exited at startup with code {ros.returncode}")
        ready = False
        ready_queries: list[dict] = []
        ready_deadline = time.monotonic() + 15.0
        while time.monotonic() < ready_deadline:
            check = ros_query_observed("ros2 control list_controllers", env, timeout=6.0)
            ready_queries.append(check)
            clean_controller_output = re.sub(r"\x1b\[[0-9;]*m", "", check.get("output", ""))
            controller_lines = [line.strip() for line in clean_controller_output.splitlines()
                                if re.match(r"^rl_quadruped_controller\b", line.strip())]
            if (check.get("status") == "completed" and len(controller_lines) == 1 and
                    re.search(r"\bactive\b", controller_lines[0])):
                ready = True
                break
            if ros.poll() is not None:
                break
            time.sleep(1.0)
        context["preflight"]["controller_ready_queries"] = ready_queries
        if not ready:
            write_json(out / "run_context.json", context)
            raise RuntimeError("controller did not become active within 15 seconds")
        lifecycle_events.append({"phase": "startup_preparation", "event": "controller_active", "monotonic_ns": now_ns()})
        # The daemon view is diagnostic only. Gate startup on a fresh direct
        # view, retrying missing discovery for a bounded interval while still
        # rejecting any duplicate names or command failures immediately.
        graph_queries = bounded_controller_graph_queries(env)
        context["preflight"]["controller_node_graph_queries"] = graph_queries
        accepted_direct = graph_queries.get("accepted_direct_attempt")
        node_inventory = accepted_direct.get("inventory", {}) if accepted_direct else {}
        controller_nodes = node_inventory.get("observed_expected_names", [])
        context["preflight"]["controller_environment"] = None
        launch_pgid = os.getpgid(ros.pid) if ros.pid is not None else None
        launched_cm = [proc for proc in matching_processes({"ros2_control_node", "ros2_control_no", "controller_manager"})
                       if proc["pgid"] == launch_pgid]
        if len(launched_cm) != 1:
            write_json(out / "run_context.json", context)
            raise RuntimeError(f"expected one controller-manager writer process in this launch group, found {launched_cm!r}")
        controller_pid = int(launched_cm[0]["pid"])
        controller_identity = proc_identity(controller_pid)
        context["preflight"]["controller_environment"] = controller_identity
        expected_controller_library = str((ROS_WS / "install/rl_quadruped_controller/lib/rl_quadruped_controller/librl_quadruped_controller.so").resolve())
        mapped_controller_libraries = mapped_library_paths(controller_pid, "librl_quadruped_controller.so")
        if mapped_controller_libraries != [expected_controller_library]:
            raise RuntimeError(
                "controller loaded library path mismatch or ambiguity: "
                f"observed={mapped_controller_libraries!r}, expected={[expected_controller_library]!r}"
            )
        context["preflight"]["controller_library_mapping"] = {
            "pid": controller_pid,
            "expected_path": expected_controller_library,
            "observed_paths": mapped_controller_libraries,
            "status": "confirmed",
        }
        write_json(out / "run_context.json", context)
        controller_list = ros_query_observed("ros2 control list_controllers", env)
        clean_controller_list = re.sub(r"\x1b\[[0-9;]*m", "", controller_list.get("output", ""))
        rl_controller_lines = [line.strip() for line in clean_controller_list.splitlines()
                               if re.match(r"^rl_quadruped_controller\b", line.strip())]
        if (graph_queries.get("status") != "confirmed_unique" or
                controller_list.get("status") != "completed" or len(rl_controller_lines) != 1 or
                not re.search(r"\bactive\b", rl_controller_lines[0])):
            context["preflight"]["controller_list_after_graph"] = controller_list
            write_json(out / "run_context.json", context)
            raise RuntimeError(
                "bounded direct graph/controller checks did not confirm one active manager and controller: "
                f"graph_status={graph_queries.get('status')!r}; controller_list={controller_list.get('output')!r}"
            )
        context["preflight"].update({"controller_nodes": controller_nodes,
                                      "active_controller_list": controller_list.get("output", "").strip(),
                                      "controller_list_after_graph": controller_list,
                                      "launched_manager_process_evidence": launched_cm})
        stop_publisher, publisher_ready = start_prepared_stop_publisher(env)
        context["preflight"]["prepared_passive_publisher"] = {
            "status": "ready_before_rl", "helper_pid": stop_publisher.pid,
            "ready_monotonic_ns": publisher_ready.get("monotonic_ns"),
            "subscriber_count": publisher_ready.get("subscribers"),
            "topic": "/control_input", "command": 1,
        }
        lifecycle_events.append({"phase": "startup_preparation", "event": "passive_publisher_ready",
                                 "monotonic_ns": publisher_ready.get("monotonic_ns"),
                                 "helper_pid": stop_publisher.pid,
                                 "subscriber_count": publisher_ready.get("subscribers")})
        write_json(out / "run_context.json", context)
        context["lifecycle"].update({"mujoco_started_monotonic_ns": now_ns(),
                                      "ros_launch_started_monotonic_ns": now_ns(),
                                      "controller_ready_monotonic_ns": now_ns()})
        if args.config_only:
            managers = context["preflight"].get("launched_manager_process_evidence", [])
            expected_pid = int(managers[0]["pid"]) if len(managers) == 1 else None
            source_expected = {
                "package_share": str(overlay / "share/go2_description"),
                "config": context["settings"]["controller_config"],
                "launch_config": str(overlay / "share/go2_description/config/robot_control.yaml"),
                "controller_cmdline": managers[0].get("cmdline", "") if len(managers) == 1 else "",
                "agile_model": context["settings"]["model_paths"][0],
                "ra_model": context["settings"]["model_paths"][1],
                "recovery_model": context["settings"]["model_paths"][2],
                "candidate": candidate, "group": group,
                "entry_threshold": entry_threshold, "exit_threshold": exit_threshold,
                "hysteresis": hysteresis, "hold": hold, "hold_s": hold_duration_s,
            }
            wait_started = time.monotonic()
            while True:
                ros_log.flush()
                binding = verify_config_consumption(log_path.read_text(encoding="utf-8", errors="replace"),
                                                    source_expected, expected_pid)
                if binding["status"] != "pending" or time.monotonic() - wait_started >= SOURCE_EVIDENCE_WAIT_S:
                    break
                time.sleep(0.1)
            context["runtime_config_observed"] = {
                "status": binding["status"], "reason": binding["reason"],
                "evidence": binding.get("evidence"), "no_rl_requested": True,
            }
            write_json(out / "run_context.json", context)
            if binding["status"] != "confirmed":
                terminal_cause = "INVALID_RUNTIME_SOURCE"
                source_mismatch = f"initialization config evidence {binding['status']}: {binding['reason']}"
                raise RuntimeError(source_mismatch)
            terminal_cause = "CONFIG_PREFLIGHT_ONLY"
            finish_event_ns = now_ns()
            context["lifecycle"]["run_terminal_event"] = terminal_cause
            context["lifecycle"]["movement_end_monotonic_ns"] = finish_event_ns
            context["lifecycle"]["movement_phase_started"] = False
            lifecycle_events.append({"phase": "startup_preparation", "event": "A_config_consumption_confirmed",
                                     "monotonic_ns": finish_event_ns, "evidence": binding["evidence"]})
            raise ConfigOnlyComplete()
        controls.append(publish(env, 2))
        wait_controller_transition(ros_log, ros, "Switched from passive to fixed down", 20.0)
        controls.append(publish(env, 2))
        wait_controller_transition(ros_log, ros, "Switched from fixed down to fixed stand", 20.0)
        # The switch log is emitted when the old state's checkChange chooses
        # the next state, one controller cycle before FIXEDSTAND.enter() clears
        # the input command. Wait for an actual FIXEDSTAND run log so command 3
        # cannot be overwritten by that enter() reset.
        wait_controller_transition(ros_log, ros, "[BaseFixedStand]: [STAND-SYMM]", 20.0)
        ros_log.flush()
        loan_log_text = log_path.read_text(encoding="utf-8", errors="replace")
        loan_matches = [(int(index), name) for index, name in
                        re.findall(r"\[FOOT-LOANED\] index=(\d+) name=([^\s]+)", loan_log_text)]
        expected_loan_names = ["foot_force/FR", "foot_force/FL", "foot_force/RR", "foot_force/RL"]
        ordered_loan_names = [name for index, name in sorted(loan_matches)]
        if (len(loan_matches) != 4 or [index for index, _ in sorted(loan_matches)] != [0, 1, 2, 3]
                or ordered_loan_names != expected_loan_names):
            context["preflight"]["foot_force_loaned_interfaces"] = {
                "status": "mismatch_or_missing", "observed": loan_matches,
                "expected": expected_loan_names,
                "rl_entry_blocked": True,
            }
            write_json(out / "run_context.json", context)
            raise RuntimeError(f"loaned foot-force interface order mismatch; RL entry blocked: {loan_matches!r}")
        context["preflight"]["foot_force_loaned_interfaces"] = {
            "status": "confirmed", "observed": loan_matches,
            "ordered_names": ordered_loan_names,
            "expected": expected_loan_names,
            "monotonic_ns": now_ns(), "rl_entry_blocked_on_failure": True,
        }
        lifecycle_events.append({"phase": "startup_preparation",
                                 "event": "foot_force_loan_order_confirmed",
                                 "monotonic_ns": context["preflight"]["foot_force_loaned_interfaces"]["monotonic_ns"],
                                 "ordered_names": ordered_loan_names})
        reset_probe = subprocess.run(
            [str(ROOT / "unitree_mujoco/simulate/build2/p1_10_initial_state_probe"),
             str(scene_spec.path)], cwd=ROOT, capture_output=True, text=True, timeout=30)
        if reset_probe.returncode == 0:
            reset_fields = {}
            for line in reset_probe.stdout.splitlines():
                if "=" not in line:
                    continue
                key, value = line.split("=", 1)
                try:
                    reset_fields[key] = json.loads(value) if value.startswith("[") else value
                except json.JSONDecodeError:
                    reset_fields[key] = value
            context["initial_state"]["reset_state"].update({
                "status": "captured", "probe_exit_code": reset_probe.returncode,
                "probe_source": "unitree_mujoco/simulate/test/p1_10_initial_state_probe.cpp",
                "probe_executable": "unitree_mujoco/simulate/build2/p1_10_initial_state_probe",
                "startup_path": reset_fields.get("startup_path"),
                "qpos0": reset_fields.get("qpos0"),
                "base_pose_world_m": reset_fields.get("base_pose_world_m"),
                "base_quat_wxyz": reset_fields.get("base_quat_wxyz"),
                "yaw_rad": reset_fields.get("base_yaw_rad"),
                "reset_joint_velocity": "UNKNOWN; probe emits qpos only"})
        else:
            context["initial_state"]["reset_state"].update({
                "status": "unavailable", "probe_exit_code": reset_probe.returncode,
                "reason": (reset_probe.stderr or reset_probe.stdout)[-1000:]})
        standing_preparation = confirm_pre_rl_standing(
            capture_id, scene_spec,
            wall_cap_s=(args.prepare_observation_wall_cap_s if args.prepare_only else 20.0),
            stable_sim_s=0.50, ros_log_path=log_path, last_command=2)
        context["initial_state"]["pre_rl_standing_preparation"] = standing_preparation
        lifecycle_events.append({"phase": "startup_preparation", "event": "pre_rl_standing_confirmed",
                                 "status": standing_preparation.get("status"),
                                 "sim_time_interval_s": standing_preparation.get("stable_sim_time_s"),
                                 "wall_cap_s": standing_preparation.get("wall_cap_s"),
                                 "monotonic_ns": standing_preparation.get("finished_monotonic_ns")})
        if args.prepare_only:
            terminal_cause = ("PREPARATION_ONLY_COMPLETE"
                              if standing_preparation.get("status") == "STABLE"
                              else "PREPARATION_FAILURE")
            context["lifecycle"]["run_terminal_event"] = terminal_cause
            context["lifecycle"]["movement_phase_started"] = False
            lifecycle_events.append({"phase": "startup_preparation", "event": "prepare_only_terminal",
                                     "status": terminal_cause,
                                     "command_3_sent": False,
                                     "observation_wall_cap_s": args.prepare_observation_wall_cap_s,
                                     "monotonic_ns": now_ns()})
            raise PreparationOnlyComplete()
        if standing_preparation.get("status") != "STABLE":
            terminal_cause = "PREPARATION_FAILURE"
            raise RuntimeError("pre-RL standing preparation failed: " + str(standing_preparation.get("reason")))
        pre_rl_capture = capture_one_joint_state(env)
        pre_rl_capture["capture_phase"] = "after confirmed PASSIVE/FIXEDSTAND stable interval, before command 3"
        pre_rl_capture["rl_entry_command_monotonic_ns"] = None
        pre_rl_capture["relation_to_rl_entry"] = "named asynchronous ROS joint-state sample taken after the stable interval; exact same-cycle alignment is not assumed"
        sample = pre_rl_capture.get("sample") or {}
        joint_velocity = sample.get("velocity_rad_s", [])
        max_abs_joint_velocity = (max((abs(float(value)) for value in joint_velocity), default=math.inf)
                                  if pre_rl_capture.get("status") == "captured" else math.inf)
        pre_rl_capture["maximum_absolute_joint_velocity_rad_s"] = max_abs_joint_velocity if math.isfinite(max_abs_joint_velocity) else None
        pre_rl_capture["standing_velocity_gate"] = {"maximum_abs_joint_velocity_rad_s": max_abs_joint_velocity,
                                                      "limit_rad_s": 0.50,
                                                      "passed": max_abs_joint_velocity <= 0.50}
        context["initial_state"]["pre_rl_joint_state"] = pre_rl_capture
        lifecycle_events.append({"phase": "startup_preparation", "event": "pre_rl_joint_state_sampled",
                                 "monotonic_ns": (sample or {}).get("capture_monotonic_ns"),
                                 "status": pre_rl_capture.get("status"),
                                 "source": "/joint_states",
                                 "max_abs_velocity_rad_s": pre_rl_capture["maximum_absolute_joint_velocity_rad_s"]})
        if pre_rl_capture.get("status") != "captured":
            terminal_cause = "PREPARATION_FAILURE"
            raise RuntimeError("required pre-RL named joint position/velocity sample unavailable; RL entry refused: " +
                               str(pre_rl_capture.get("reason")))
        if max_abs_joint_velocity > 0.50:
            terminal_cause = "PREPARATION_FAILURE"
            raise RuntimeError(f"pre-RL joint velocity {max_abs_joint_velocity:.4f} rad/s exceeds fixed preparation limit 0.50; RL entry refused")
        pre_task_raw, pre_task_diagnostic = read_collision_snapshot_diagnostic()
        pre_task_validation_ns = now_ns()
        pre_task_status, pre_task_snapshot = classify_snapshot(
            pre_task_raw, pre_task_validation_ns, expected_capture_id=capture_id,
            expected_fingerprint=scene_spec.model_fingerprint,
            expected_scene_binding=scene_spec.binding())
        if pre_task_status is not CollisionStatus.LIVE or pre_task_snapshot is None:
            terminal_cause = "PREPARATION_FAILURE"
            context["initial_state"]["pre_task_safety_boundary"] = {
                "status": pre_task_status.value, "reader": pre_task_diagnostic,
                "observed_monotonic_ns": pre_task_validation_ns}
            raise RuntimeError("pre-RL task-window safety boundary is not LIVE; RL entry refused")
        if args.startup_recovery_observation_s > 0:
            if (pre_task_snapshot.version != 5 or pre_task_snapshot.physics_coverage_complete != 1 or
                    pre_task_snapshot.unknown_contacts != 0 or pre_task_snapshot.invalid_posture_steps != 0 or
                    pre_task_snapshot.fall_candidate != 0 or pre_task_snapshot.fall_confirmed != 0):
                terminal_cause = "PREPARATION_FAILURE"
                raise RuntimeError("S6-04 startup observation requires fresh v5 coverage, known contacts, valid posture and no active pre-entry fall")
            startup_clock_anchor = {
                "source": "last authoritative v5 PhysicsLoop snapshot before command 3",
                "sequence": pre_task_snapshot.sequence,
                "physics_step": pre_task_snapshot.physics_step,
                "sim_time_s": pre_task_snapshot.sim_time,
                "monotonic_ns": pre_task_snapshot.monotonic_ns,
                "unknown_contact_steps": pre_task_snapshot.unknown_contact_steps,
                "invalid_posture_steps": pre_task_snapshot.invalid_posture_steps,
                "first_invalid_posture_sim_time": pre_task_snapshot.first_invalid_posture_sim_time,
                "last_invalid_posture_sim_time": pre_task_snapshot.last_invalid_posture_sim_time,
                "fall_history_count": pre_task_snapshot.fall_history_count,
                "first_unknown_contact_sim_time": pre_task_snapshot.first_unknown_contact_sim_time,
                "last_unknown_contact_sim_time": pre_task_snapshot.last_unknown_contact_sim_time,
                "first_unknown_contact_step": pre_task_snapshot.first_unknown_contact_step,
                "last_unknown_contact_step": pre_task_snapshot.last_unknown_contact_step,
                "capture_id": pre_task_snapshot.capture_id,
                "model_fingerprint": pre_task_snapshot.runtime_model_fingerprint,
                "scenario_id": pre_task_snapshot.scenario_id,
                "conservative_lower_bound": True,
            }
        context["initial_state"]["pre_task_safety_boundary"] = {
            "status": "LIVE", "snapshot": collision_snapshot_payload(pre_task_status, pre_task_snapshot),
            "reader": pre_task_diagnostic, "observed_monotonic_ns": pre_task_validation_ns,
            "scope": "last authoritative physics snapshot before RL command; retained as the task-start coverage boundary"}
        write_json(out / "run_context.json", context)
        entry_readiness = {
            "recorder_ready": context["preflight"].get("recorder_readiness", {}).get("status") == "ready_readable_and_writable",
            "standing_confirmed": standing_preparation.get("status") == "STABLE",
            "joint_sample_valid": pre_rl_capture.get("status") == "captured",
            "joint_velocity_within_limit": max_abs_joint_velocity <= 0.50,
            "pre_task_safety_boundary_live": context["initial_state"].get("pre_task_safety_boundary", {}).get("status") == "LIVE",
        }
        context["preflight"]["rl_entry_readiness"] = {
            **entry_readiness, "failure": rl_entry_precondition_failure(entry_readiness),
            "checked_monotonic_ns": now_ns()}
        write_json(out / "run_context.json", context)
        entry_failure = rl_entry_precondition_failure(entry_readiness)
        if entry_failure is not None:
            terminal_cause = "PREPARATION_FAILURE"
            raise RuntimeError(entry_failure)
        rl_entry_publish_start_ns = now_ns()
        lifecycle_events.append({"phase": "motion", "event": "rl_policy_request_started",
                                 "monotonic_ns": rl_entry_publish_start_ns, "command": 3})
        controls.append(publish_prepared_command(stop_publisher, "START_RL"))
        rl_entry_publish_end_ns = now_ns()
        lifecycle_events.append({"phase": "motion", "event": "rl_policy_requested",
                                 "monotonic_ns": rl_entry_publish_end_ns, "command": 3})
        context["lifecycle"]["rl_entry_command_monotonic_ns"] = rl_entry_publish_end_ns
        context["lifecycle"]["rl_entry_command_invoked_monotonic_ns"] = rl_entry_publish_start_ns
        context["lifecycle"]["rl_entry_command_publish_interval_ns"] = [rl_entry_publish_start_ns,
                                                                              rl_entry_publish_end_ns]
        context["initial_state"]["pre_rl_joint_state"]["rl_entry_command_monotonic_ns"] = rl_entry_publish_end_ns
        sample_ns = (pre_rl_capture.get("sample") or {}).get("capture_monotonic_ns")
        context["initial_state"]["pre_rl_joint_state"].update({
            "sample_to_rl_request_invocation_ms": (
                (rl_entry_publish_start_ns - sample_ns) / 1e6 if isinstance(sample_ns, int) else None),
            "sample_to_rl_command_cli_return_ms": (
                (rl_entry_publish_end_ns - sample_ns) / 1e6 if isinstance(sample_ns, int) else None),
            "timing_note": "command CLI invocation/return bound supervisor call; actual ROS topic publication and controller consumption are not timestamped by this boundary",
        })
        context["commands"] = controls
        write_json(out / "run_context.json", context)
        terminal_cause = "INCOMPLETE"
        run_deadline = time.monotonic() + args.wall_cap_s
        while time.monotonic() < run_deadline:
            ros_log.flush()
            new_text = log_path.read_text(encoding="utf-8", errors="replace")
            lines = new_text[safety_line_offset:].splitlines()
            safety_line_offset = len(new_text)
            if args.startup_recovery_observation_s > 0 and startup_entry_marker is None:
                entry_match = next((S6_ENTRY_RE.search(line) for line in lines
                                    if S6_ENTRY_RE.search(line)), None)
                if entry_match is not None:
                    startup_entry_marker = {
                        "session_id": int(entry_match.group("session")),
                        "steady_ns": int(entry_match.group("steady_ns")),
                        "source": "controller-authored StateRL::enter log",
                    }
                    anchor_ns = startup_clock_anchor.get("monotonic_ns") if startup_clock_anchor else None
                    anchor_sim = startup_clock_anchor.get("sim_time_s") if startup_clock_anchor else None
                    age_ns = startup_entry_marker["steady_ns"] - anchor_ns if isinstance(anchor_ns, int) else None
                    if (not isinstance(anchor_sim, (int, float)) or not math.isfinite(float(anchor_sim)) or
                            not isinstance(age_ns, int) or age_ns < 0 or age_ns > 100_000_000):
                        startup_observation_event = {"status": "INVALID", "reason": "RL entry could not be bound to a fresh pre-entry physics clock sample",
                                                     "entry": startup_entry_marker, "clock_anchor": startup_clock_anchor,
                                                     "anchor_age_ns": age_ns}
                    else:
                        startup_clock_anchor["rl_entry_steady_ns"] = startup_entry_marker["steady_ns"]
                        startup_clock_anchor["rl_entry_sample_bracket_ns"] = [anchor_ns,
                            startup_entry_marker["steady_ns"]]
                        startup_clock_anchor["anchor_age_ms"] = age_ns / 1e6
                        startup_clock_anchor["window_deadline_sim_time_s"] = (
                            float(anchor_sim) + args.startup_recovery_observation_s)
                        context["lifecycle"]["startup_recovery_observation"] = {
                            "status": "ARMED", "entry": startup_entry_marker,
                            "clock_anchor": startup_clock_anchor,
                            "anchor_interpretation": "conservative pre-entry sim_time lower bound; window cannot exceed requested duration after RL entry",
                            "maximum_sim_time_s": args.startup_recovery_observation_s,
                            "tracker_modified": False,
                        }
                        startup_monitor_state = new_startup_observation_state(
                            startup_clock_anchor, args.startup_recovery_observation_s,
                            capture_id=capture_id,
                            model_fingerprint=scene_spec.model_fingerprint,
                            scenario_id=scene_spec.scene_id,
                            session_id=startup_entry_marker["session_id"])
                        lifecycle_events.append({"phase": "startup_observation", "event": "rl_entry_anchor_bound",
                            "monotonic_ns": startup_entry_marker["steady_ns"],
                            "session_id": startup_entry_marker["session_id"],
                            "clock_anchor": startup_clock_anchor})
                        write_json(out / "run_context.json", context)
            if (args.startup_recovery_observation_s > 0 and startup_observation_event is not None and
                    startup_observation_event.get("status") == "INVALID"):
                terminal_cause = "STARTUP_OBSERVATION_INVALID"
                terminal_event_ns = now_ns()
                break
            safety = next((line for line in lines if EMERGENCY_RE.search(line) or HARDSTOP_RE.search(line) or VETO_RE.search(line)), None)
            if safety:
                kind = "FROZEN_POSITION_PASSIVE" if EMERGENCY_RE.search(safety) else ("GLOBAL_HARD_STOP" if HARDSTOP_RE.search(safety) else "POLICY_SAFETY_VETO")
                observer_ns = now_ns()
                detection_match = re.search(r"detection_ns=(\d+)", safety)
                terminal_event_ns = int(detection_match.group(1)) if detection_match else observer_ns
                process_facts["external_safety_events"].append({
                    "event": kind, "source": "RlQuadrupedController/StateRL ROS log",
                    "message": safety.strip()[:1000], "observed_monotonic_ns": terminal_event_ns,
                    "observer_received_monotonic_ns": observer_ns})
                collision_status, collision_snapshot = classify_snapshot(
                    read_collision_snapshot(), observer_ns,
                    expected_capture_id=capture_id,
                    expected_fingerprint=scene_spec.model_fingerprint,
                    expected_scene_binding=scene_spec.binding())
                process_facts["external_safety_events"][-1]["collision_snapshot"] = (
                    collision_snapshot_payload(collision_status, collision_snapshot))
                terminal_cause = "SYSTEM_SAFETY_ABORT"
                break
            if ros.poll() is not None or mujoco.poll() is not None:
                terminal_cause = "PROCESS_EXIT_UNEXPECTED"
                break

            # The manual startup viewer uses only fresh, same-run MuJoCo
            # PhysicsLoop time. Policy frames are sampled independently below
            # and can be incomplete without ending this bounded observation.
            if (startup_monitor_state is not None and
                    startup_monitor_state.get("terminal") is None):
                physics_raw, physics_reader = read_collision_snapshot_diagnostic()
                physics_observed_ns = now_ns()
                physics_status, physics_snapshot = classify_snapshot(
                    physics_raw, physics_observed_ns,
                    expected_capture_id=capture_id,
                    expected_fingerprint=scene_spec.model_fingerprint,
                    expected_scene_binding=scene_spec.binding())
                physics_payload = (collision_snapshot_payload(physics_status, physics_snapshot)
                                   if physics_snapshot is not None else None)
                startup_decision = advance_startup_observation(
                    startup_monitor_state, physics_status.value, physics_payload,
                    physics_observed_ns)
                if startup_decision.get("status") == "STOP":
                    startup_observation_event = {
                        **startup_decision,
                        "source_snapshot": physics_payload,
                        "reader": physics_reader,
                    }
                    terminal_cause = startup_decision["terminal"]
                    terminal_event_ns = physics_observed_ns
                    lifecycle_events.append({"phase": "startup_observation",
                                             "event": "physics_clock_terminal",
                                             **startup_observation_event})
                    break

            raw = read_shm_frame()
            frame_sampled_ns = now_ns()
            status, frame = classify_frame(raw, frame_sampled_ns)
            if (startup_monitor_state is not None and
                    startup_monitor_state.get("terminal") is None):
                note_startup_policy_read_status(
                    startup_monitor_state, status.value,
                    frame if frame is not None and frame.rl_entered == 1 and frame.rl_active == 1 else None)
            if status is FrameStatus.LIVE and frame is not None:
                capture_result = recorder.observe(raw, status, frame, frame_sampled_ns)
                if not capture_result.get("persisted"):
                    terminal_cause = "RECORDING_FAILURE"
                    terminal_frame_ref = capture_result.get("frame_ref")
                    context["lifecycle"]["terminal_frame_capture"] = {
                        "status": "failed", "complete": False,
                        "reason": capture_result.get("reason", "frame snapshot was not persisted"),
                    }
                    terminal_event_ns = frame.monotonic_ns
                    break
                key = (frame.session_id, frame.sequence)
                if key != last_frame_key:
                    frame_rows += 1
                    last_frame_key = key
                    if recorder.source_mismatch is not None:
                        source_mismatch = recorder.source_mismatch
                        terminal_cause = "INVALID_RUNTIME_SOURCE"
                        terminal_frame_ref = capture_result["frame_ref"]
                        terminal_event_ns = frame.monotonic_ns
                        break
                    if bound_session_id is None:
                        ros_text = log_path.read_text(encoding="utf-8", errors="replace")
                        managers = context["preflight"].get("launched_manager_process_evidence", [])
                        expected_pid = int(managers[0]["pid"]) if len(managers) == 1 else None
                        source_expected = {
                            "package_share": str(overlay / "share/go2_description"),
                            "config": context["settings"]["controller_config"],
                            "launch_config": str(overlay / "share/go2_description/config/robot_control.yaml"),
                            "controller_cmdline": managers[0].get("cmdline", "") if len(managers) == 1 else "",
                            "agile_model": context["settings"]["model_paths"][0],
                            "ra_model": context["settings"]["model_paths"][1],
                            "recovery_model": context["settings"]["model_paths"][2],
                            "candidate": candidate, "candidate_mode": candidate_mode, "group": group,
                            "entry_threshold": entry_threshold, "exit_threshold": exit_threshold,
                            "hysteresis": hysteresis, "hold": hold, "hold_s": hold_duration_s,
                            "goal_x": args.goal_x, "goal_y": 0.0,
                        }
                        frame_binding = {"session_id": frame.session_id, "switching_mode": frame.switching_mode,
                                         "entry_threshold": frame.entry_threshold,
                                         "exit_threshold": frame.exit_threshold,
                                         "hold_enabled": frame.hold_enabled,
                                         "recovery_min_hold_s": frame.recovery_min_hold_s}
                        binding = verify_source_binding(ros_text, source_expected, frame_binding, expected_pid)
                        context["runtime_source_validation"] = {
                            "status": binding["status"], "reason": binding["reason"],
                            "wait_limit_s": SOURCE_EVIDENCE_WAIT_S,
                            "frame_session_id": frame.session_id,
                        }
                        if binding["status"] == "pending":
                            if source_wait_started is None:
                                source_wait_started = time.monotonic()
                            write_json(out / "run_context.json", context)
                            if time.monotonic() - source_wait_started < SOURCE_EVIDENCE_WAIT_S:
                                continue
                            source_mismatch = f"source evidence timeout ({binding['reason']})"
                        elif binding["status"] == "conflict":
                            source_mismatch = f"source evidence conflict ({binding['reason']}): {binding['evidence']}"
                        else:
                            bound_session_id = frame.session_id
                            context["session_id"] = bound_session_id
                            context["runtime_source_validation"] = {
                                "status": "confirmed", "reason": binding["reason"],
                                "wait_limit_s": SOURCE_EVIDENCE_WAIT_S,
                                "evidence": binding["evidence"],
                            }
                            context["runtime_config_observed"] = {
                                "session_id": bound_session_id,
                                "switching_mode": frame.switching_mode,
                                "group": group,
                                "entry_threshold": frame.entry_threshold,
                                "exit_threshold": frame.exit_threshold,
                                "hysteresis": group in ("B", "D"),
                                "hold": group in ("C", "D"),
                                "recovery_min_hold_s": hold_duration_s,
                                "goal_log_matches_expected": True,
                                "source_binding": binding["evidence"],
                            }
                            lifecycle_events.append({"phase": "motion", "event": "runtime_source_bound",
                                "monotonic_ns": now_ns(), "session_id": bound_session_id,
                                "candidate": candidate, "group": group,
                                "entry_threshold": frame.entry_threshold,
                                "exit_threshold": frame.exit_threshold, "goal_world_m": [args.goal_x, 0.0],
                                "source_evidence_wait_s": time.monotonic() - source_wait_started
                                    if source_wait_started is not None else 0.0})
                            write_json(out / "run_context.json", context)
                        if source_mismatch is not None:
                            terminal_cause = "INVALID_RUNTIME_SOURCE"
                            terminal_frame_ref = capture_result["frame_ref"]
                            terminal_event_ns = frame.monotonic_ns
                            break
                    elif frame.session_id != bound_session_id:
                        source_mismatch = f"runtime session changed {bound_session_id}->{frame.session_id}"
                        terminal_cause = "INVALID_RUNTIME_SOURCE"
                        terminal_frame_ref = capture_result["frame_ref"]
                        break
                    if (frame.switching_mode != candidate_mode or
                            abs(frame.entry_threshold - entry_threshold) >= 1e-5 or
                            abs(frame.exit_threshold - exit_threshold) >= 1e-5 or
                            abs(frame.recovery_min_hold_s - hold_duration_s) >= 1e-5 or
                            bool(frame.hold_enabled) != (group in ("C", "D"))):
                        source_mismatch = "runtime candidate or threshold changed after initial binding"
                        terminal_cause = "INVALID_RUNTIME_SOURCE"
                        terminal_frame_ref = capture_result["frame_ref"]
                        break
                    if frame.rl_entered == 1 and frame.rl_active == 1 and not context["lifecycle"].get("policy_active_start"):
                        active_start = {"session_id": frame.session_id, "rl_step": frame.rl_step,
                                        "source_sequence": frame.sequence,
                                        "sim_time_s": frame.sim_time_s if frame.sim_clock_valid else None,
                                        "sim_clock_valid": bool(frame.sim_clock_valid),
                                        "monotonic_ns": frame.monotonic_ns,
                                        "boundary": "first persisted policy frame with rl_entered=1 and rl_active=1"}
                        context["lifecycle"]["policy_active_start"] = active_start
                        context["lifecycle"]["task_start"] = {
                            "session_id": frame.session_id, "rl_step": frame.rl_step,
                            "source_sequence": frame.sequence,
                            "sim_time_s": frame.sim_time_s if frame.sim_clock_valid else None,
                            "sim_clock_valid": bool(frame.sim_clock_valid),
                            "monotonic_ns": frame.monotonic_ns,
                            "boundary": "first active policy decision frame; task metrics and safety window begin here"}
                        lifecycle_events.append({"phase": "motion", "event": "policy_active_start", **active_start})
                        task_start_sim_s = frame.sim_time_s if frame.sim_clock_valid else None
                        first_pose = (frame.world_pose[0], frame.world_pose[1])
                        boundary = context.get("initial_state", {}).get("pre_task_safety_boundary", {})
                        boundary_snapshot = boundary.get("snapshot") if isinstance(boundary, dict) else None
                        if isinstance(boundary_snapshot, dict):
                            step = boundary_snapshot.get("physics_step")
                            task_start_physics_step = step if isinstance(step, int) else None
                        pre_state = context["initial_state"].get("pre_rl_joint_state", {})
                        sampled_ns = (pre_state.get("sample") or {}).get("capture_monotonic_ns")
                        pre_state["sample_to_first_active_policy_frame_ms"] = (
                            (frame.monotonic_ns - sampled_ns) / 1e6 if isinstance(sampled_ns, int) else None)
                        context["initial_state"]["pre_rl_joint_state"] = pre_state
                        write_json(out / "run_context.json", context)
                    if sim_start_s is None and frame.sim_clock_valid:
                        sim_start_s = frame.sim_time_s
                        first_frame_ns = frame.monotonic_ns
                        if first_pose is None:
                            first_pose = (frame.world_pose[0], frame.world_pose[1])
                    safety_now = frame.safety_faulted or frame.policy_state == 2
                    motion_collision_events = []
                    confirmed_fall = False
                    arrived = False
                    sim_timeout = False
                    distance = None
                    collision_payload = capture_result.get("collision_snapshot")
                    startup_fall_delay_allowed = False
                    startup_terminal_override: str | None = None
                    if args.startup_recovery_observation_s > 0:
                        if (startup_entry_marker is None or startup_monitor_state is None or
                                frame.session_id != startup_entry_marker.get("session_id")):
                            startup_observation_event = {
                                "status": "INVALID",
                                "reason": "policy frame cannot be bound to the controller RL-entry session",
                                "session_id": frame.session_id,
                                "policy_cycle": frame.rl_step,
                            }
                            startup_terminal_override = "INVALID_RUNTIME_SOURCE"
                        elif not frame.ray_valid:
                            startup_observation_event = {
                                "status": "INVALID", "reason": "policy ray source is invalid",
                                "session_id": frame.session_id, "policy_cycle": frame.rl_step,
                            }
                            startup_terminal_override = "SYSTEM_SAFETY_ABORT"
                        elif startup_monitor_state["last_policy_cycle"] != frame.rl_step:
                            policy_note = note_startup_policy_frame(startup_monitor_state, frame)
                            if policy_note["status"] != "OK":
                                startup_observation_event = {**policy_note,
                                                             "session_id": frame.session_id,
                                                             "policy_cycle": frame.rl_step}
                                startup_terminal_override = "INVALID_RUNTIME_SOURCE"
                            elif startup_monitor_state["policy_gaps"] and len(startup_monitor_state["policy_gaps"]) == 1 and startup_monitor_state["policy_gaps"][-1]["current_policy_cycle"] == frame.rl_step:
                                print("[S6-04] 策略记录有缺帧，继续按 MuJoCo 物理时间观看；策略数据标记不完整。", flush=True)
                        startup_fall_delay_allowed = bool(startup_monitor_state and
                                                          startup_monitor_state.get("delay_low_height_fall"))
                    if frame.sim_clock_valid and sim_start_s is not None and first_pose is not None:
                        displacement = math.dist((frame.world_pose[0], frame.world_pose[1]), first_pose)
                        if movement_start_sim_s is None and displacement >= 0.10:
                            movement_start_sim_s = frame.sim_time_s
                            motion_start_state_capture.update({
                                "status": "capturing_joint_state",
                                "policy_frame_anchor": {
                                    "session_id": frame.session_id, "rl_step": frame.rl_step,
                                    "source_sequence": frame.sequence,
                                    "sim_time_s": frame.sim_time_s,
                                    "monotonic_ns": frame.monotonic_ns,
                                    "world_pose_xy_yaw": list(frame.world_pose),
                                    "displacement_from_first_policy_pose_m": displacement,
                                },
                                "joint_position_rad": "pending",
                                "joint_velocity_rad_s": "pending",
                                "joint_sample_relation": "asynchronous ROS state sample; exact same-cycle alignment is not assumed",
                            })
                            if isinstance(collision_payload, dict) and collision_payload.get("status") == CollisionStatus.LIVE.value:
                                motion_start_state_capture["physics_posture_sample"] = {
                                    "physics_step": collision_payload.get("physics_step"),
                                    "sim_time_s": collision_payload.get("sim_time"),
                                    "base_height_m": collision_payload.get("base_height_m"),
                                    "base_roll_rad": collision_payload.get("base_roll_rad"),
                                    "base_pitch_rad": collision_payload.get("base_pitch_rad"),
                                    "source": "versioned MuJoCo collision authority snapshot; independently timed from policy frame",
                                }
                            def capture_motion_joint_state():
                                motion_start_state_capture["joint_state_capture"] = capture_one_joint_state(env)
                            motion_start_capture_thread = threading.Thread(
                                target=capture_motion_joint_state, name="s6-01-joint-state-capture")
                            motion_start_capture_thread.start()
                            context["lifecycle"]["motion_start_event"] = "first valid pose >=0.10m from first valid policy pose"
                            context["lifecycle"]["motion_start_observed_sim_time_s"] = movement_start_sim_s
                            context["lifecycle"]["motion_start_interval_s"] = [previous_pose_time_s, frame.sim_time_s]
                            lifecycle_events.append({"phase": "motion", "event": "motion_start",
                                                     "monotonic_ns": frame.monotonic_ns,
                                                     "session_id": frame.session_id, "rl_step": frame.rl_step,
                                                     "sim_time_s": frame.sim_time_s,
                                                     "displacement_from_initial_m": displacement})
                        # Runtime task safety starts at the first active strategy decision, not
                        # after the historical 0.10 m displacement marker. If its clock was
                        # invalid, use the pre-RL physics-step boundary conservatively; reports
                        # still retain UNKNOWN for time-based coverage.
                        task_window_started = context["lifecycle"].get("task_start") is not None
                        if (task_window_started and isinstance(collision_payload, dict)
                                and collision_payload.get("status") == CollisionStatus.LIVE.value):
                            motion_collision_events, motion_fall_events = task_window_history_events(
                                collision_payload, task_start_sim_s,
                                frame.sim_time_s if frame.sim_clock_valid else None,
                                task_start_physics_step)
                            confirmed_fall_events = [event for event in motion_fall_events
                                                     if event.get("confirmed") is True]
                            confirmed_fall = bool(confirmed_fall_events)
                        if motion_collision_events:
                            for event in motion_collision_events:
                                lifecycle_events.append({"phase": "motion", "event": "nonfoot_obstacle_collision",
                                    "source": "PhysicsLoop cumulative collision episode",
                                    "session_id": frame.session_id, "policy_cycle": frame.rl_step,
                                    "physics_start_step": event.get("start_physics_step"),
                                    "physics_end_step": event.get("end_physics_step"),
                                    "start_sim_time_s": event.get("start_sim_time"),
                                    "end_sim_time_s": event.get("end_sim_time"),
                                    "robot_geom_id": event.get("robot_geom_id"),
                                    "obstacle_geom_id": event.get("obstacle_geom_id"),
                                    "robot_geom": event.get("robot_geom_name"),
                                    "robot_body_id": event.get("robot_body_id"),
                                    "robot_body": event.get("robot_body_name"),
                                    "obstacle_geom": event.get("obstacle_geom_name")})
                        if confirmed_fall:
                            for fall_event in confirmed_fall_events:
                                lifecycle_events.append({"phase": "motion", "event": "fall_confirmed",
                                    "source": "PhysicsLoop base_link posture rule cumulative episode",
                                    "session_id": frame.session_id, "policy_cycle": frame.rl_step,
                                    "physics_step": fall_event.get("confirmed_physics_step"),
                                    "sim_time_s": fall_event.get("confirmed_sim_time"),
                                    "start_physics_step": fall_event.get("start_physics_step"),
                                    "start_sim_time_s": fall_event.get("start_sim_time")})
                        distance = math.dist((frame.world_pose[0], frame.world_pose[1]), (args.goal_x, 0.0))
                        arrived = task_window_started and distance <= 0.5
                        sim_timeout = (task_start_sim_s is not None and
                                       frame.sim_clock_valid and
                                       frame.sim_time_s - task_start_sim_s >= args.sim_limit_s)
                    # During the explicit manual window, the independent live
                    # PhysicsLoop monitor owns fall terminal decisions. Its
                    # current episode state avoids treating a closed startup
                    # fall-history entry as a new active fall on every frame.
                    fall_for_terminal = (confirmed_fall if args.startup_recovery_observation_s <= 0.0
                                         else False)
                    arrival_for_terminal = (arrived if args.startup_recovery_observation_s <= 0.0
                                            else False)
                    timeout_for_terminal = (sim_timeout if args.startup_recovery_observation_s <= 0.0
                                             else False)
                    selected_terminal = select_strategy_terminal(
                        safety_now, bool(motion_collision_events),
                        fall_for_terminal and not startup_fall_delay_allowed,
                        arrival_for_terminal, timeout_for_terminal)
                    if startup_terminal_override is not None:
                        if selected_terminal in (None, "FALL_TERMINATION", "ARRIVED", "DIAGNOSTIC_SIM_TIME_LIMIT"):
                            selected_terminal = startup_terminal_override
                    if selected_terminal is not None:
                        terminal_cause = selected_terminal
                        terminal_frame_ref = capture_result["frame_ref"]
                        terminal_event_ns = frame.monotonic_ns
                        closeout_snapshot = collision_payload if isinstance(collision_payload, dict) else {}
                        normal_closeout_eligible, closeout_gate_reason = normal_closeout_gate(
                            terminal_cause, safety_now, motion_collision_events, confirmed_fall,
                            collision_payload if isinstance(collision_payload, dict) else None)
                        terminal_summary = {
                            "terminal_frame_session_id": frame.session_id,
                            "terminal_frame_rl_step": frame.rl_step,
                            "terminal_frame_sequence": frame.sequence,
                            "terminal_frame_sim_time_s": frame.sim_time_s if frame.sim_clock_valid else None,
                            "terminal_world_xy_m": list(frame.world_pose[:2]),
                            "goal_world_xy_m": [args.goal_x, 0.0],
                            "distance_to_goal_m": distance,
                            "normal_closeout_eligible": normal_closeout_eligible,
                            "normal_closeout_ineligible_reason": closeout_gate_reason,
                            "terminal_contact_objects": [
                                {"robot_geom_id": event.get("robot_geom_id"),
                                 "robot_geom_name": event.get("robot_geom_name"),
                                 "obstacle_geom_id": event.get("obstacle_geom_id"),
                                 "obstacle_geom_name": event.get("obstacle_geom_name"),
                                 "start_physics_step": event.get("start_physics_step"),
                                 "end_physics_step": event.get("end_physics_step"),
                                 "start_sim_time_s": event.get("start_sim_time"),
                                 "end_sim_time_s": event.get("end_sim_time")}
                                for event in motion_collision_events
                            ],
                        }
                        if safety_now:
                            lifecycle_events.append({"phase": "motion", "event": "SYSTEM_SAFETY_ABORT",
                                "source": "strategy frame safety_faulted/policy_state",
                                "session_id": frame.session_id, "policy_cycle": frame.rl_step})
                        if motion_collision_events:
                            context["lifecycle"]["collision_observed"] = motion_collision_events
                        if confirmed_fall and isinstance(collision_payload, dict):
                            context["lifecycle"]["fall_observed"] = confirmed_fall_events
                        if arrived:
                            context["lifecycle"]["arrival_observed"] = {"session_id": frame.session_id,
                                "rl_step": frame.rl_step, "sim_time_s": frame.sim_time_s,
                                "distance_m": distance, "threshold_m": 0.5}
                        context["lifecycle"]["normal_closeout_gate"] = {
                            "eligible": normal_closeout_eligible,
                            "terminal": terminal_cause,
                            "physics_status": closeout_snapshot.get("status"),
                            "unknown_contacts": closeout_snapshot.get("unknown_contacts"),
                            "fall_candidate": closeout_snapshot.get("fall_candidate"),
                            "fall_confirmed": closeout_snapshot.get("fall_confirmed"),
                            "robot_obstacle_contacts": closeout_snapshot.get("robot_obstacle_contacts"),
                            "reason": closeout_gate_reason}
                        break
                    if frame.sim_clock_valid and distance is not None:
                        previous_pose_time_s = frame.sim_time_s
                    context["session_id"] = frame.session_id
            time.sleep(0.002)
        else:
            if args.startup_recovery_observation_s > 0.0:
                terminal_cause = "STARTUP_OBSERVATION_WALL_CAP_INCOMPLETE"
                startup_observation_event = {
                    "status": "WALL_CAP_INCOMPLETE",
                    "reason": "wall-clock guard expired before the requested MuJoCo simulation-time window completed",
                }
            else:
                terminal_cause = "WALL_CLOCK_GUARD_INCOMPLETE"
        finish_event_ns = terminal_event_ns or now_ns()
        if args.startup_recovery_observation_s > 0.0:
            observation = context["lifecycle"].setdefault("startup_recovery_observation", {})
            monitor = startup_monitor_state or {}
            policy_gaps = monitor.get("policy_gaps", [])
            observation.update({
                "clock_source": "authoritative same-run MuJoCo PhysicsLoop sim_time",
                "requested_observation_sim_s": args.startup_recovery_observation_s,
                "deadline_sim_time_s": monitor.get("deadline_sim_time_s"),
                "last_physics_sim_time_s": monitor.get("last_sim_time_s"),
                "actual_observation_physics_s": (
                    max(0.0, float(monitor["last_sim_time_s"]) - float(monitor["anchor_sim_time_s"]))
                    if isinstance(monitor.get("last_sim_time_s"), (int, float)) and
                    isinstance(monitor.get("anchor_sim_time_s"), (int, float)) else None),
                "physics_source_status_at_stop": monitor.get("last_physics_status", "NOT_ARMED"),
                "physics_sample_count": monitor.get("physics_samples", 0),
                "physical_sample_gap_count": monitor.get("physics_sample_gaps", 0),
                "low_height_event_observed": monitor.get("low_height_seen", False),
                "low_height_tracker_confirmed": monitor.get("low_height_tracker_confirmed", False),
                "low_height_terminal_delayed": monitor.get("low_height_delay_logged", False),
                "recovered": monitor.get("recovered", False),
                "recovery_evidence_status": monitor.get("recovery_evidence_status", "UNKNOWN"),
                "recovery_evidence_reason": monitor.get("recovery_evidence_reason"),
                "recovery_dwell_start_sim_time_s": monitor.get("normal_dwell_start_sim_s"),
                "recovery_sim_time_s": monitor.get("recovery_sim_time_s"),
                "strategy_policy_frame_count": monitor.get("policy_frames", 0),
                "strategy_policy_clock_invalid_frame_count": monitor.get("policy_clock_invalid_frames", 0),
                "strategy_policy_sim_clock_stale_frame_count": monitor.get("policy_stale_clock_frames", 0),
                "strategy_policy_stale_sequence_count": len(monitor.get("policy_stale_sequences", [])),
                "strategy_policy_stale_sequences": monitor.get("policy_stale_sequences", []),
                "strategy_policy_read_status_observations": monitor.get("policy_read_status_counts", {}),
                "policy_recording_coverage": "incomplete" if (policy_gaps or
                    monitor.get("policy_clock_invalid_frames", 0) or
                    monitor.get("policy_nonlive_observations", 0) or not monitor.get("policy_frames"))
                    else "no_observed_policy_gaps_or_invalid_clocks",
                "policy_recording_gaps": policy_gaps,
                "missing_policy_cycles": sum(gap["missing_policy_cycles"] for gap in policy_gaps),
                "monitor_events": monitor.get("events", []),
                "cleanup_result": "pending process cleanup",
            })
            observation["interpretation"] = (
                "Only the authoritative same-run MuJoCo sim_time advances the viewing deadline and recovery dwell. "
                "Invalid policy clocks and policy-cycle gaps remain data-quality limitations and are never filled "
                "from physics time. They do not alone end viewing. Physics authority failures and existing safety "
                "stops remain terminal. Strategy-time metrics retain original validity and are not interpolated.")
            observation["final_event"] = startup_observation_event or {
                "status": "INTERRUPTED_BY_EXISTING_TERMINAL",
                "terminal": terminal_cause,
                "note": "existing hard-stop, collision, or supervisor failure ended the observation",
            }
            observation["terminal_classification"] = terminal_cause
        context["lifecycle"]["run_terminal_event"] = terminal_cause
        context["lifecycle"]["movement_end_monotonic_ns"] = finish_event_ns
        context["lifecycle"]["terminal_frame_ref"] = terminal_frame_ref
        context["lifecycle"]["movement_end_sim_time_s"] = terminal_frame_ref.get("sim_time_s") if terminal_frame_ref else None
        if terminal_frame_ref is not None and terminal_cause != "RECORDING_FAILURE":
            context["lifecycle"]["terminal_frame_capture"] = {
                "status": "persisted_same_snapshot", "complete": True,
                "identity": {key: terminal_frame_ref.get(key) for key in
                             ("session_id", "rl_step", "source_sequence", "monotonic_ns")},
            }
        if terminal_cause == "INVALID_RUNTIME_SOURCE":
            context["lifecycle"]["source_mismatch"] = source_mismatch
    except ConfigOnlyComplete:
        context["lifecycle"]["run_terminal_event"] = terminal_cause
        context["lifecycle"]["movement_end_monotonic_ns"] = finish_event_ns
    except PreparationOnlyComplete:
        finish_event_ns = now_ns()
        context["lifecycle"]["run_terminal_event"] = terminal_cause
        context["lifecycle"]["movement_phase_started"] = False
        context["lifecycle"]["movement_end_monotonic_ns"] = finish_event_ns
    except Exception as exc:
        startup_error = f"{type(exc).__name__}: {exc}"
        if terminal_cause not in {"INVALID_RUNTIME_SOURCE", "PREPARATION_FAILURE"}:
            terminal_cause = "STARTUP_FAILURE"
        finish_event_ns = now_ns()
        context["lifecycle"]["run_terminal_event"] = terminal_cause
        context["lifecycle"]["startup_error"] = startup_error
    except KeyboardInterrupt:
        terminal_cause = "OPERATOR_ABORT"
        finish_event_ns = now_ns()
        context["lifecycle"]["run_terminal_event"] = terminal_cause
        context["lifecycle"]["movement_end_monotonic_ns"] = finish_event_ns
    finally:
        # Close the motion-frame window before any cleanup request or process teardown.
        if motion_start_capture_thread is not None:
            motion_start_capture_thread.join(timeout=6.0)
            if motion_start_capture_thread.is_alive():
                motion_start_state_capture["joint_state_capture"] = {
                    "status": "unavailable", "reason": "bounded joint-state sampler did not finish before cleanup"}
            capture = motion_start_state_capture.get("joint_state_capture", {})
            sample = capture.get("sample") if isinstance(capture, dict) else None
            if isinstance(sample, dict):
                motion_start_state_capture["status"] = "captured_with_async_joint_sample"
                motion_start_state_capture["joint_position_rad"] = {
                    name: sample["position_rad"][index]
                    for index, name in enumerate(sample.get("name", []))
                    if index < len(sample.get("position_rad", []))}
                motion_start_state_capture["joint_velocity_rad_s"] = {
                    name: sample["velocity_rad_s"][index]
                    for index, name in enumerate(sample.get("name", []))
                    if index < len(sample.get("velocity_rad_s", []))}
                motion_start_state_capture["joint_sample_relation"] = (
                    "measured /joint_states sample captured after the strategy frame anchor; "
                    "ROS header and monotonic sample time retained; exact same-cycle alignment not claimed")
            elif capture:
                motion_start_state_capture["status"] = "joint_sample_unavailable"
                motion_start_state_capture["joint_position_rad"] = "UNKNOWN"
                motion_start_state_capture["joint_velocity_rad_s"] = "UNKNOWN"
        context["initial_state"]["motion_start_measured_state"] = motion_start_state_capture
        if capture_started and not recorder.recorder.stopped:
            recorder.stop.set()
            recorder.recorder.stop_sampling()
        if context.get("session_id") is None and recorder.bound_session_id is not None:
            context["session_id"] = recorder.bound_session_id
        if terminal_frame_ref is None and terminal_cause == "SYSTEM_SAFETY_ABORT" and recorder.last_frame_metadata is not None:
            context["lifecycle"]["last_frame_before_external_safety_event"] = recorder.last_frame_metadata
            context["lifecycle"]["terminal_frame_capture"] = {
                "status": "no_strategy_snapshot_for_external_event", "complete": False,
                "reason": "external safety event was observed from ROS log, not from the persisted strategy snapshot",
            }
        lifecycle_events.append({"phase": "motion_end", "event": terminal_cause,
                                 "monotonic_ns": finish_event_ns or now_ns(),
                                 "frame_ref": terminal_frame_ref})
        # PASSIVE uses the publisher that was already ready before RL entry. The
        # movement recorder is closed above; cleanup observations stay separate.
        post_terminal_stop = None
        cleanup_frame_observations: list[dict] = []
        if ros is not None and ros.poll() is None and stop_publisher is not None and stop_publisher.poll() is None:
            try:
                if context.get("lifecycle", {}).get("normal_closeout_gate", {}).get("eligible") is True:
                    post_terminal_shutdown = perform_normal_shutdown(
                        stop_publisher, log_path, capture_id, scene_spec, ros,
                        out / "standing_prompt.marker",
                        args.standing_observation_sim_s,
                        args.standing_observation_wall_cap_s, mujoco)
                    context["lifecycle"]["normal_shutdown"] = post_terminal_shutdown
                    standing_hold_complete = post_terminal_shutdown.get("standing_hold_complete") is True
                    context["lifecycle"]["standing_observation"] = {
                        "not_part_of_motion_capture_or_metrics": True,
                        "requested_sim_time_s": args.standing_observation_sim_s,
                        "wall_cap_s": args.standing_observation_wall_cap_s,
                        "observed_sim_time_s": post_terminal_shutdown.get("standing_observation_sim_time_s"),
                        "observed_elapsed_sim_s": post_terminal_shutdown.get("standing_observation_elapsed_s"),
                        "sample_count": len(post_terminal_shutdown.get("standing_observation_samples", [])),
                        "samples": post_terminal_shutdown.get("standing_observation_samples", []),
                        "complete": standing_hold_complete,
                    }
                    lifecycle_events.append({"phase": "cleanup", "event": "NORMAL_SHUTDOWN_REQUESTED",
                        "monotonic_ns": post_terminal_shutdown["request"]["publish_start_monotonic_ns"],
                        "command": 6, "confirmed": post_terminal_shutdown.get("standing_hold_complete", False)})
                    lifecycle_events.extend({"phase": "cleanup", **event}
                        for event in post_terminal_shutdown["controller_stages"])
                    context["lifecycle"]["normal_shutdown_physics_observation"] = {
                        "not_part_of_motion_capture_or_metrics": True,
                        "samples": post_terminal_shutdown["physics_observations"],
                        "fall_or_robot_obstacle_contact_vetoed": any(
                            token in (post_terminal_shutdown.get("failure_reason") or "")
                            for token in ("fall", "robot-obstacle contact")),
                        "standing_hold_failure_reason": post_terminal_shutdown.get("failure_reason"),
                        "failure_physics_observation": post_terminal_shutdown.get("failure_physics_observation")}
                    if standing_hold_complete:
                        post_terminal_stop = {"not_requested": True,
                            "controller_state_before_simulator_stop": "FIXEDSTAND",
                            "reason": "bounded standing hold completed; no normal PASSIVE, lower, or unload command"}
                    else:
                        post_terminal_stop = publish_prepared_command(stop_publisher, "HARD_STOP")
                else:
                    if terminal_cause in {"ARRIVED", "DIAGNOSTIC_SIM_TIME_LIMIT"}:
                        context["lifecycle"]["normal_shutdown_not_attempted"] = {
                            "reason": context.get("lifecycle", {}).get("normal_closeout_gate", {}).get("reason",
                                "no healthy terminal-frame gate evidence")}
                    post_terminal_stop = publish_prepared_stop(stop_publisher)
                request_ns = int(post_terminal_shutdown["request"]["publish_start_monotonic_ns"]
                                 if post_terminal_stop.get("not_requested") else
                                 post_terminal_stop["publish_start_monotonic_ns"])
                if not post_terminal_stop.get("not_requested"):
                    stop_command = int(post_terminal_stop["command"])
                    lifecycle_events.append({"phase": "cleanup",
                        "event": "HARD_STOP_FALLBACK_REQUESTED" if stop_command == 9 else "PASSIVE_STOP_REQUESTED",
                        "monotonic_ns": request_ns, "command": stop_command,
                        "publisher_invoked_monotonic_ns": post_terminal_stop["supervisor_invoked_monotonic_ns"],
                        "publish_return_monotonic_ns": post_terminal_stop["publish_return_monotonic_ns"]})
                post_terminal_stop["confirmation_scope"] = (
                    "no controller stop command was sent; FIXEDSTAND remained active until MuJoCo shutdown"
                    if post_terminal_stop.get("not_requested") else
                    "controller hard-stop branch logged completion after PASSIVE enter; "
                    "not a MuJoCo command-consumption acknowledgement")
                observed_keys: set[tuple[int, int]] = set()
                cleanup_start_ns = now_ns()
                confirmation = None
                deadline = time.monotonic() + (0.0 if post_terminal_stop.get("not_requested") else 2.0)
                while not post_terminal_stop.get("not_requested") and time.monotonic() < deadline:
                    ros_log.flush()
                    log_text = log_path.read_text(encoding="utf-8", errors="replace")
                    confirmation = hard_stop_confirmation(log_text, request_ns)
                    if confirmation is not None:
                        break
                    sampled_ns = now_ns()
                    raw_cleanup = read_shm_frame()
                    cleanup_status, cleanup_frame = classify_frame(raw_cleanup, sampled_ns)
                    if cleanup_status is FrameStatus.LIVE and cleanup_frame is not None:
                        key = (cleanup_frame.session_id, cleanup_frame.sequence)
                        if key not in observed_keys:
                            observed_keys.add(key)
                            cleanup_frame_observations.append({
                                "session_id": cleanup_frame.session_id,
                                "sequence": cleanup_frame.sequence,
                                "rl_step": cleanup_frame.rl_step,
                                "frame_monotonic_ns": cleanup_frame.monotonic_ns,
                                "observer_monotonic_ns": sampled_ns,
                                "policy_state": {0: "AGILE", 1: "RECOVERY", 2: "FAULTED"}.get(
                                    cleanup_frame.policy_state, "UNKNOWN"),
                                "sim_time_s": cleanup_frame.sim_time_s if cleanup_frame.sim_clock_valid else None,
                            })
                    time.sleep(0.005)
                cleanup_end_ns = now_ns()
                if post_terminal_stop.get("not_requested"):
                    post_terminal_stop.update({
                        "controller_stop_requested": False,
                        "controller_state_before_simulator_stop": "FIXEDSTAND",
                        "simulator_shutdown_started_monotonic_ns": cleanup_start_ns,
                        "no_passive_confirmation_claimed": True,
                    })
                    lifecycle_events.append({"phase": "cleanup", "event": "STANDING_HOLD_OBSERVATION_COMPLETE",
                        "monotonic_ns": cleanup_start_ns, "controller_state": "FIXEDSTAND",
                        "controller_stop_requested": False})
                else:
                    stop_confirmed = confirmation is not None
                    post_terminal_stop.update({
                        "controller_transition_confirmed": stop_confirmed,
                        "controller_confirmation": confirmation,
                        "controller_pid": (context.get("preflight", {}).get("launched_manager_process_evidence") or [{}])[0].get("pid"),
                        "controller_transition_delay_from_publish_start_ns": (
                            confirmation["controller_steady_monotonic_ns"] - request_ns
                            if confirmation and "controller_steady_monotonic_ns" in confirmation else None),
                        "confirmation_observation_window_monotonic_ns": [cleanup_start_ns, cleanup_end_ns],
                        "confirmation_not_observed_by_deadline": not stop_confirmed,
                    })
                    lifecycle_events.append({"phase": "cleanup", "event": "STRATEGY_STOP_CONFIRMATION",
                        "monotonic_ns": confirmation.get("controller_steady_monotonic_ns", cleanup_end_ns) if confirmation else cleanup_end_ns,
                        "confirmed": stop_confirmed, "evidence": confirmation,
                        "observation_window_monotonic_ns": [cleanup_start_ns, cleanup_end_ns]})
                context["lifecycle"]["cleanup_observation"] = {
                    "not_part_of_motion_capture_or_metrics": True,
                    "start_monotonic_ns": cleanup_start_ns,
                    "end_monotonic_ns": cleanup_end_ns,
                    "strategy_frames": cleanup_frame_observations,
                    "strategy_output_after_request_observed": bool(cleanup_frame_observations),
                    "interpretation": "only frames observed in this bounded cleanup window; absence is not proof of zero output",
                }
                post_terminal_stop["cleanup_policy_frame_count_observed"] = len(cleanup_frame_observations)
                if not post_terminal_stop.get("not_requested"):
                    post_terminal_stop["stop_request_to_confirmation"] = (
                        "measured from publisher publish_start to controller post-transition steady_ns"
                        if post_terminal_stop.get("controller_transition_confirmed") else
                        "UNKNOWN within recorded bounded observation window")
            except Exception as exc:
                post_terminal_stop = {"command": 1, "failed": f"{type(exc).__name__}: {exc}",
                                      "monotonic_ns": now_ns(),
                                      "confirmation": "UNKNOWN; request or acknowledgement failed"}
                context["lifecycle"]["normal_shutdown_error"] = post_terminal_stop["failed"]
        context["lifecycle"]["normal_shutdown"] = post_terminal_shutdown
        context["lifecycle"]["post_terminal_strategy_stop"] = post_terminal_stop
        def cleanup_owned_processes() -> None:
            # Preserve the requested order: close this run's MuJoCo first,
            # then its controller launch, then the prepared stop publisher.
            stop_results = []
            owned_processes = (("mujoco", mujoco), ("ros2_launch", ros),
                               ("prepared_stop_publisher", stop_publisher))
            for name, proc in owned_processes:
                if proc is None:
                    stop_results.append({"name": name, "started": False, "exit_code": None, "forced": False})
                    continue
                forced = False
                try:
                    if proc.poll() is None:
                        try:
                            os.killpg(proc.pid, signal.SIGINT)
                            proc.wait(timeout=6.0)
                        except subprocess.TimeoutExpired:
                            forced = True
                            os.killpg(proc.pid, signal.SIGTERM)
                            try:
                                proc.wait(timeout=3.0)
                            except subprocess.TimeoutExpired:
                                os.killpg(proc.pid, signal.SIGKILL)
                                proc.wait(timeout=3.0)
                        except ProcessLookupError:
                            pass
                    stop_results.append({"name": name, "started": True, "exit_code": proc.poll(), "forced": forced})
                    if name == "mujoco" and proc.poll() is not None:
                        stop_evidence = context.get("lifecycle", {}).get("post_terminal_strategy_stop", {})
                        confirmation = stop_evidence.get("controller_confirmation", {})
                        controller_state = confirmation.get("state") or (
                            "FIXEDSTAND" if context.get("lifecycle", {}).get("standing_observation", {}).get("complete")
                            else "UNKNOWN")
                        lifecycle_events.append({"phase": "process_cleanup", "event": "MUJOCO_STOPPED_BEFORE_CONTROLLER",
                                                 "monotonic_ns": now_ns(), "exit_code": proc.returncode,
                                                 "controller_state_before_stop": controller_state})
                except Exception as exc:
                    stop_results.append({"name": name, "started": True, "exit_code": proc.poll(),
                                         "forced": forced, "cleanup_error": f"{type(exc).__name__}: {exc}"})
            try:
                ros_log.flush()
                final_ros_text = log_path.read_text(encoding="utf-8", errors="replace")
                gain_trace = parse_startup_gain_trace_lines(final_ros_text)
                gain_trace_path = out / "startup_gain_transition.jsonl"
                if gain_trace:
                    gain_trace_path.write_text(
                        "".join(json.dumps(item, separators=(",", ":"), allow_nan=False) + "\n"
                                for item in gain_trace), encoding="utf-8")
                context.setdefault("lifecycle", {})["startup_gain_transition_trace"] = {
                    "status": "captured" if gain_trace else ("requested_missing" if args.startup_gain_trace else "not_requested"),
                    "path": str(gain_trace_path) if gain_trace else None,
                    "record_count": len(gain_trace),
                    "session_ids": sorted({item["session_id"] for item in gain_trace}),
                    "source": "controller-authored JSON records of last entry interface gains and actual written gains",
                    "scope": "ros2_control command-interface writes; not MuJoCo consumption",
                }
                known_safety_messages = {item.get("message") for item in process_facts["external_safety_events"]}
                known_safety_messages.update(item.get("message") for item in lifecycle_events
                                             if item.get("phase") == "cleanup" and item.get("source"))
                for line in final_ros_text.splitlines():
                    if not (EMERGENCY_RE.search(line) or HARDSTOP_RE.search(line) or VETO_RE.search(line)):
                        continue
                    message = line.strip()[:1000]
                    if message in known_safety_messages:
                        continue
                    kind = "FROZEN_POSITION_PASSIVE" if EMERGENCY_RE.search(line) else ("GLOBAL_HARD_STOP" if HARDSTOP_RE.search(line) else "POLICY_SAFETY_VETO")
                    event_match = re.search(r"detection_ns=(\d+)", line)
                    hard_stop_interval = None
                    if HARDSTOP_RE.search(line) and isinstance(post_terminal_stop, dict):
                        confirm_event = post_terminal_stop.get("controller_confirmation")
                        if isinstance(confirm_event, dict):
                            hard_stop_interval = [post_terminal_stop.get("publish_start_monotonic_ns"),
                                                  confirm_event.get("controller_steady_monotonic_ns")]
                    lifecycle_events.append({"phase": "cleanup", "event": kind,
                        "source": "RlQuadrupedController/StateRL ROS log", "message": message,
                        "event_monotonic_ns": int(event_match.group(1)) if event_match else None,
                        "event_time_source": "controller_detection_ns" if event_match else "not embedded in source line",
                        "event_time_interval_monotonic_ns": hard_stop_interval,
                        "observer_received_monotonic_ns": now_ns()})
                if args.startup_recovery_observation_s > 0.0:
                    trace_records = parse_startup_trace_lines(final_ros_text)
                    trace_path = out / "controller_startup_trace.jsonl"
                    with trace_path.open("w", encoding="utf-8") as trace_file:
                        for record in trace_records:
                            trace_file.write(json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n")
                    context["lifecycle"]["controller_startup_trace"] = {
                        "status": "captured" if trace_records else "missing",
                        "path": str(trace_path), "record_count": len(trace_records),
                        "first_policy_cycle": next((r.get("policy_cycle") for r in trace_records
                                                     if r.get("policy_cycle") is not None), None),
                        "last_policy_cycle": next((r.get("policy_cycle") for r in reversed(trace_records)
                                                    if r.get("policy_cycle") is not None), None),
                        "includes_actual_command_interface_values": True,
                        "scope": "controller-authored values written to ros2_control command interfaces; not MuJoCo consumption",
                    }
                    if not trace_records:
                        context["lifecycle"]["controller_startup_trace"]["reason"] = (
                            "no S6-04 trace rows were present in the controller ROS log")
            except Exception as exc:
                process_facts.setdefault("cleanup_errors", []).append(
                    {"stage": "final_ros_safety_scan", "error": f"{type(exc).__name__}: {exc}"})
            for name, log in (("ros_launch_log", ros_log), ("mujoco_log", mujoco_log)):
                try:
                    log.flush()
                    log.close()
                except Exception as exc:
                    process_facts.setdefault("cleanup_errors", []).append(
                        {"stage": name, "error": f"{type(exc).__name__}: {exc}"})
            cleanup_ns = now_ns()
            context["lifecycle"]["cleanup_complete_monotonic_ns"] = cleanup_ns
            lifecycle_events.append({"phase": "process_cleanup", "event": "process_groups_stopped",
                                     "monotonic_ns": cleanup_ns, "processes": stop_results})
            context["lifecycle"]["cleanup_processes"] = stop_results
            context["capture"] = {"frames_written": recorder.unique_frames,
                                   "capture_error": recorder.error,
                                   "frame_capture_writer": "supervisor_synchronous_single_writer",
                                   "authoritative_collision_live_before_motion": collision_live_seen,
                                   "pre_run_shared_frames_ignored_by_session_timestamp": recorder.pre_run_frames_ignored}
            process_facts.update({"exit_code": ros.returncode if ros is not None else None,
                                  "forced_termination": any(item["forced"] for item in stop_results),
                                  "shutdown_complete": all(proc is None or proc.poll() is not None
                                                           for proc in (ros, mujoco, stop_publisher)),
                                  "shutdown_request_source": "S1-04 bounded supervisor SIGINT with TERM/KILL fallback",
                                  "run_terminal_result": terminal_cause,
                                  "run_terminal_monotonic_ns": finish_event_ns,
                                  "cleanup_events": [e for e in lifecycle_events if e.get("phase") == "cleanup"]})
            if args.startup_recovery_observation_s > 0.0:
                startup_record = context["lifecycle"].get("startup_recovery_observation", {})
                startup_record["cleanup_result"] = (
                    "all owned run processes exited" if process_facts["shutdown_complete"]
                    else "one or more owned run processes remain or exit was not confirmed")
                startup_record["closeout_errors"] = list(process_facts.get("cleanup_errors", []))
                process_facts["startup_recovery_observation"] = dict(startup_record)
            context["result_contract"] = {"classification": terminal_cause,
                "startup_failure": startup_error,
                "fall_detection": "PhysicsLoop base_link posture rule; continuously >=0.30 valid sim seconds",
                "operator_abort": "not issued", "policy_frames_observed": recorder.unique_frames,
                "supervisor_loop_frames_observed": frame_rows,
                "first_valid_sim_time_s": sim_start_s, "first_valid_frame_monotonic_ns": first_frame_ns,
                "task_start": context.get("lifecycle", {}).get("task_start"),
                "task_window_safety_boundary": context.get("initial_state", {}).get("pre_task_safety_boundary"),
                "motion_start_sim_time_s": movement_start_sim_s,
                "motion_start_unknown_reason": None if movement_start_sim_s is not None else "no >=0.10m pose displacement observed",
                "valid_collision_source_seen": collision_live_seen,
                "movement_terminal_summary": terminal_summary,
                "cleanup_observation_is_excluded_from_motion_metrics": True,
                "command_consumption_by_mujoco": "UNKNOWN; no direct consumption acknowledgement"}
            if args.startup_recovery_observation_s > 0.0:
                context["result_contract"]["startup_recovery_observation"] = context.get("lifecycle", {}).get(
                    "startup_recovery_observation", {})
            context["commands"] = controls

        terminal, closeout_errors = finalize_run_closeout(
            out, context, process_facts, recorder, capture_started, cleanup_owned_processes)

    if args.startup_recovery_observation_s > 0.0:
        startup_record = context.get("lifecycle", {}).get("startup_recovery_observation", {})
        startup_record["closeout_errors"] = closeout_errors
        startup_record["closeout_complete"] = process_facts.get("closeout_complete")
        if terminal is not None:
            terminal["startup_recovery_observation"] = dict(startup_record)
            try:
                write_json(out / "terminal.json", terminal)
            except Exception as exc:
                context.setdefault("lifecycle", {}).setdefault("post_closeout_artifact_errors", []).append(
                    {"stage": "startup_terminal_context_update",
                     "error": f"{type(exc).__name__}: {exc}"})

    context["initial_state"]["first_active_policy_cycle"] = (
        extract_first_active_policy_state(log_path, out / "runtime_record.jsonl") if capture_started else
        {"status": "UNKNOWN", "reason": "strategy_capture_not_started"})
    context["initial_state"]["first_active_policy_cycle_capture_scope"] = (
        "S6-03 requires an exact same-session/cycle/monotonic timestamp match between the cycle-0 "
        "controller companion and the first active runtime JSONL payload; any gap is UNKNOWN and "
        "blocks preparation-readiness acceptance.")
    try:
        write_json(out / "run_context.json", context)
    except Exception as exc:
        context.setdefault("lifecycle", {}).setdefault("post_closeout_artifact_errors", []).append(
            {"stage": "initial_policy_state_context_update",
             "error": f"{type(exc).__name__}: {exc}"})

    if process_facts.get("shutdown_complete") is False:
        cleanup_summary = "未确认全部本次进程退出"
    else:
        cleanup_summary = "本次 ROS、MuJoCo 与停止发布器进程均已退出"
    lifecycle = context.get("lifecycle") or {}
    normal_shutdown = lifecycle.get("normal_shutdown") or {}
    strategy_stop = lifecycle.get("post_terminal_strategy_stop") or {}
    summary = {
        "end_reason": terminal_cause,
        "distance_to_goal_m": terminal_summary.get("distance_to_goal_m") if terminal_summary else None,
        "contact_objects": terminal_summary.get("terminal_contact_objects") if terminal_summary else [],
        "standing_hold_complete": bool(normal_shutdown.get("standing_hold_complete")),
        "normal_passive_requested": bool(normal_shutdown.get("passive_requested")),
        "passive_controller_transition_confirmed": bool(strategy_stop.get("controller_transition_confirmed")),
        "cleanup": cleanup_summary,
    }
    print(json.dumps({"run_id": run_id, "directory": str(out), "result": terminal_cause,
                      "summary": summary, "startup_error": startup_error, "frames": recorder.unique_frames,
                      "process_facts": process_facts}, ensure_ascii=False))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--overlay", required=True)
    parser.add_argument("--group", choices=("A", "B", "C", "D"), default="A",
                        help="research group; C/D use H=0.30 simulated seconds for this diagnostic")
    parser.add_argument("--entry-threshold", type=float, default=-0.05,
                        help="A/B/C/D entry threshold E")
    parser.add_argument("--exit-threshold", type=float, default=-0.08,
                        help="B/D exit threshold X; A/C derive exit from E")
    parser.add_argument("--hold-s", type=float, default=0.30,
                        help="C/D Recovery minimum hold H in simulation seconds; A/B disable hold")
    parser.add_argument("--goal-x", type=float, default=1.0)
    parser.add_argument("--scene", default="scene_obstacle.xml",
                        help="supported MuJoCo scene XML filename (default: scene_obstacle.xml)")
    parser.add_argument("--sim-limit-s", type=float, default=6.0)
    parser.add_argument("--wall-cap-s", type=float, default=60.0)
    parser.add_argument("--standing-observation-sim-s", type=float, default=5.0,
                        help="post-stand observation in simulation seconds (0..30)")
    parser.add_argument("--standing-observation-wall-cap-s", type=float, default=30.0,
                        help="independent wall-clock cap for post-stand observation (1..60)")
    parser.add_argument("--classification", default="bounded nonformal single-run diagnostic; not a formal experiment",
                        help="human-readable nonformal run purpose saved in run_context.json")
    parser.add_argument("--config-only", action="store_true",
                        help="stop after controller initialization proves selected group config consumption; never request RL")
    parser.add_argument("--prepare-only", action="store_true",
                        help="bounded PASSIVE/FIXEDSTAND observation; stop before joint capture and never send command 3")
    parser.add_argument("--prepare-observation-wall-cap-s", type=float, default=12.0,
                        help="wall-clock bound for --prepare-only fixed-stand observation (1..20 seconds)")
    parser.add_argument("--startup-recovery-observation-s", type=float, default=0.0,
                        help="S6-04 one-off simulation-only low-height recovery observation (0..4.0 s; default disabled)")
    parser.add_argument("--startup-gain-trace", action="store_true",
                        help="record actual S6-05 startup gain-interface writes; observational only")
    parser.add_argument("--display", default=":0")
    default_xauthority = os.environ.get("XAUTHORITY") or os.path.join(
        os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"), "gdm", "Xauthority")
    parser.add_argument("--xauthority", default=default_xauthority)
    args = parser.parse_args()
    if (not math.isfinite(args.entry_threshold) or not math.isfinite(args.exit_threshold) or
            not math.isfinite(args.hold_s) or args.exit_threshold >= args.entry_threshold or args.hold_s <= 0):
        parser.error("thresholds must be finite, X must be below E, and H must be positive")
    if (not math.isfinite(args.standing_observation_sim_s) or
            not 0.0 <= args.standing_observation_sim_s <= 30.0):
        parser.error("--standing-observation-sim-s must be finite and in [0, 30]")
    if (not math.isfinite(args.standing_observation_wall_cap_s) or
            not 1.0 <= args.standing_observation_wall_cap_s <= 60.0):
        parser.error("--standing-observation-wall-cap-s must be finite and in [1, 60]")
    if not math.isfinite(args.prepare_observation_wall_cap_s) or not 1.0 <= args.prepare_observation_wall_cap_s <= 20.0:
        parser.error("--prepare-observation-wall-cap-s must be finite and in [1, 20]")
    if (not math.isfinite(args.startup_recovery_observation_s) or
            not 0.0 <= args.startup_recovery_observation_s <= 4.0):
        parser.error("--startup-recovery-observation-s must be finite and in [0, 4.0]")
    if args.startup_recovery_observation_s > 0.0:
        if (args.group != "A" or args.scene != "scene_random_04.xml" or
                abs(args.entry_threshold - (-0.05)) > 1e-9 or abs(args.goal_x - 7.0) > 1e-9 or
                args.config_only or args.prepare_only):
            parser.error("the S6-04 startup observation is restricted to one A/-0.05 run on "
                         "scene_random_04.xml with goal-x=7.0; config/prepare-only modes are forbidden")
    if args.config_only and args.prepare_only:
        parser.error("--config-only and --prepare-only are mutually exclusive")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
