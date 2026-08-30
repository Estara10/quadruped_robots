#!/usr/bin/env python3
"""P1-09 — authoritative runtime record → P1-02 formal run binding.

Consumes ONE FINALIZED two-phase runtime record (``record.jsonl``) plus the run
orchestrator's real process-facts sidecar, derives the structured safety /
terminal events from authoritative sources ONLY (the runtime frame payloads and
the orchestrator's actual wait facts), writes the P1-02 formal artifact set
through ``FormalRunWriter``, and returns the P1-02 ``validate_run`` verdict
(VALID or INVALID).

Fail-closed boundaries (no mock / synthetic / legacy / text-log / default fill):

- The runtime record must be VALID in its own chain
  (``run_record`` record-validity), otherwise no formal write.
- Identity binding: FormalRunWriter-allocated ``run_id`` ↔ runtime-record
  ``run_id`` ↔ single ``session_id`` ↔ frame ``source_sequence`` /
  ``rl_step`` / ``monotonic_ns`` ↔ process facts. Any session, time, sequence,
  origin or terminal-boundary inconsistency refuses the formal write.
- ``safety_faulted`` / ``policy_state==FAULTED`` come only from authoritative
  frames; process exit / forced termination / shutdown completion come only from
  the orchestrator's real wait facts. Events are never parsed from ROS/MuJoCo
  text logs.
- ``collision`` / ``fall`` / goal / ``timeout`` / ``simulation_time_s`` have no
  verified authoritative runtime producer in the frame today; they are left
  absent and the P1-02 validator classifies the episode INVALID (never a
  fabricated SUCCESS / VALID).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

from formal_experiment_contract import (
    FormalRunWriter,
    REQUIRED_TELEMETRY_FIELDS,
    derive_seed,
    validate_run,
)
from run_record import RunRecordData, load_record, _validate_record_trust

# ---------------------------------------------------------------- schema facts
# The 6 event types the P1-02 contract requires (x-abs-events.required_types).
_REQUIRED_EVENT_TYPES = ("episode_start", "controller_active", "rl_entered", "valid_ready", "terminal", "shutdown")

# Frame payload keys that map to telemetry columns.
_RAY2D = "ray2d"  # 11
_CMD_CHAIN = ("action_raw", "action_clipped", "joint_target_rad", "torque_nm", "torque_saturated")  # 5x12


class BindingError(ValueError):
    """Raised when the runtime record / facts / context fail closed before any formal write."""


def _load_facts_file(facts_path: Optional[str]) -> Dict[str, Any]:
    """Load the orchestrator facts sidecar (best effort; missing → {})."""
    if not facts_path:
        return {}
    path = Path(facts_path)
    if not path.exists():
        return {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return payload if isinstance(payload, dict) else {}


def _first_frame(frames: Sequence[Mapping[str, Any]], predicate=None) -> Optional[Mapping[str, Any]]:
    for frame in frames:
        payload = frame.get("payload") or {}
        if predicate is None or predicate(payload):
            return frame
    return None


def load_and_check_record(record_path: str, facts_path: Optional[str] = None) -> tuple:
    """Load the finalized record; refuse on any trust/identity failure.

    Returns ``(data, facts, terminal, valid_live_payloads)``.
    """
    data = load_record(record_path)
    facts = _load_facts_file(facts_path)
    valid, trust_reasons, valid_live_payloads = _validate_record_trust(data)
    if not valid:
        raise BindingError("runtime record is not VALID in its own chain: " + "; ".join(trust_reasons))
    if data.terminal is None:
        raise BindingError("runtime record has no terminal line")
    if not valid_live_payloads:
        raise BindingError("runtime record has no LIVE authoritative payloads")
    # Single session across all LIVE payloads (the record chain already enforces it;
    # re-check here so the formal binding is independently safe).
    sessions = {p["session_id"] for p in valid_live_payloads}
    if len(sessions) != 1:
        raise BindingError("runtime record spans multiple sessions: " + ",".join(str(s) for s in sorted(sessions)))
    terminal = data.terminal
    # Authoritative process facts: the record terminal must carry them, and the
    # orchestrator facts sidecar must not contradict the terminal.
    if terminal.get("process_exit_code") is None:
        raise BindingError("runtime record has no authoritative process facts (missing_facts)")
    _FACT_KEYS = (
        ("exit_code", "process_exit_code"),
        ("forced_termination", "forced_termination"),
        ("shutdown_complete", "shutdown_complete"),
        ("shutdown_request_source", "shutdown_request_source"),
    )
    for file_key, terminal_key in _FACT_KEYS:
        file_value = facts.get(file_key)
        terminal_value = terminal.get(terminal_key)
        if file_value is not None and file_value != terminal_value:
            raise BindingError(
                f"orchestrator facts contradict record terminal: {file_key}={file_value!r} vs {terminal_key}={terminal_value!r}"
            )
    return data, facts, terminal, valid_live_payloads


# ------------------------------------------------------------------ manifest
_REQUIRED_CONTEXT_SECTIONS = (
    "created_at", "variant", "git", "models", "effective_config", "environment",
    "scenario", "seeds", "perception", "rates_hz", "thresholds",
)


def validate_context(context: Mapping[str, Any]) -> None:
    """Fail closed before any write when the manifest context is incomplete."""
    if not isinstance(context, Mapping):
        raise BindingError("manifest context must be a mapping")
    missing = [key for key in _REQUIRED_CONTEXT_SECTIONS if key not in context]
    if missing:
        raise BindingError("manifest context missing required section(s): " + ", ".join(missing))


# ------------------------------------------------------------------ events
# SINGLE authoritative event reducer: every event derives only from the runtime
# frames and the orchestrator's real wait facts. Text logs are never consulted.
def derive_events(
    data: RunRecordData,
    terminal: Mapping[str, Any],
    facts: Mapping[str, Any],
    run_id: str,
) -> List[Dict[str, Any]]:
    """Reduce the finalized record into the P1-02 structured events.

    Authoritative sources:
    - episode_start:  record meta ``created_at_ns`` (monotonic domain).
    - controller_active / rl_entered: first LIVE frame where the flag is set.
    - valid_ready:    the record is VALID in its own chain (last LIVE frame).
    - terminal:       record terminal (outcome UNKNOWN — no authoritative
      goal/collision/fall/timeout source).
    - shutdown:       orchestrator real wait facts (``shutdown_monotonic_ns``)
      and process-exit facts.
    ``simulation_time_s`` has no authoritative source → absent (validator → INVALID).
    """
    frames = [f for f in data.frames if f.get("status") == "LIVE"]
    first_active = _first_frame(frames, lambda p: bool(p.get("controller_active")))
    first_rl = _first_frame(frames, lambda p: bool(p.get("rl_entered")))
    last_live = frames[-1] if frames else None
    last_payload = (last_live or {}).get("payload") or {}

    def event(etype: str, sequence: int, monotonic_ns: Optional[int]) -> Dict[str, Any]:
        return {
            "run_id": run_id,
            "sequence": sequence,
            "monotonic_time_ns": monotonic_ns,
            "simulation_time_s": None,  # no authoritative source → validator INVALID
            "type": etype,
        }

    events = [event("episode_start", 0, data.meta.get("created_at_ns"))]

    seq = 1
    for etype, frame in (
        ("controller_active", first_active),
        ("rl_entered", first_rl),
    ):
        if frame is not None:
            monotonic = (frame.get("payload") or {}).get("monotonic_ns")
            if monotonic is not None:
                events.append(event(etype, seq, monotonic))
                seq += 1

    last_monotonic = last_payload.get("monotonic_ns")
    if last_monotonic is not None:
        events.append(event("valid_ready", seq, last_monotonic))
        seq += 1

    terminal_monotonic = terminal.get("last_frame_time_ns")
    events.append(event("terminal", seq, terminal_monotonic))
    events[-1]["outcome"] = "UNKNOWN"  # no authoritative goal/collision/fall/timeout source
    seq += 1

    # shutdown: orchestrator real wait facts only.
    shutdown_monotonic = facts.get("shutdown_monotonic_ns")
    shutdown = event("shutdown", seq, shutdown_monotonic)
    shutdown["process_exit_code"] = terminal.get("process_exit_code")
    shutdown["forced_termination"] = terminal.get("forced_termination")
    shutdown["shutdown_complete"] = terminal.get("shutdown_complete")
    shutdown["shutdown_request_source"] = terminal.get("shutdown_request_source")
    events.append(shutdown)
    return events


# ------------------------------------------------------------------ telemetry
def _fmt(value: Any) -> Any:
    """Pass finite numbers through; mark unavailable values explicitly as None."""
    if value is None:
        return None
    try:
        num = float(value)
    except (TypeError, ValueError):
        return None
    return num


def map_telemetry_row(payload: Mapping[str, Any], run_id: str, context: Mapping[str, Any]) -> Dict[str, Any]:
    """Map one authoritative frame payload into one P1-02 telemetry row.

    Fields with no authoritative runtime source stay ``None`` (absent), so the
    P1-02 validator flags them and classifies the episode INVALID — never a
    fabricated number.
    """
    world = payload.get("world_pose") or [None, None, None]
    lin = payload.get("lin_vel") or [None, None, None]
    cmd = payload.get("command") or [None, None, None]
    thresholds = context.get("thresholds", {})

    row: Dict[str, Any] = {
        "run_id": run_id,
        "sequence": payload.get("source_sequence"),
        "simulation_time_s": None,  # no authoritative source
        "monotonic_time_ns": payload.get("monotonic_ns"),
        "telemetry_fresh": "true",
        "base_x_m": _fmt(world[0]),
        "base_y_m": _fmt(world[1]),
        "base_z_m": None,            # no source
        "roll_rad": None,            # no source
        "pitch_rad": None,           # no source
        "yaw_rad": _fmt(world[2]),
        "policy_state": payload.get("policy_state"),
        "ra_value": _fmt(payload.get("ra_value")),
        "ra_entry_threshold": _fmt(thresholds.get("ra_entry_threshold")),
        "ra_exit_threshold": _fmt(thresholds.get("ra_exit_threshold")),
        "command_vx_mps": _fmt(cmd[0]),
        "command_vy_mps": _fmt(cmd[1]),
        "command_wz_rps": _fmt(cmd[2]),
        "actual_vx_mps": _fmt(lin[0]),
        "actual_vy_mps": _fmt(lin[1]),
        "actual_wz_rps": None,       # no yaw-rate source
        "recovery_vx_mps": None,     # no source
        "recovery_vy_mps": None,
        "recovery_wz_rps": None,
        "recovery_constraint_margin": None,
        "ray_valid": "true" if payload.get("ray_valid") else "false",
        "ray_age_ns": payload.get("ray_age_ns"),
        "controller_active": "true" if payload.get("controller_active") else "false",
        "rl_active": "true" if payload.get("rl_active") else "false",
        "collision_available": "false",  # collision_origin always UNAVAILABLE today
        "collision": None,               # no authoritative source
        "fall": None,                    # no authoritative source
        "arrival_candidate": None,       # no source
    }
    ray2d = payload.get(_RAY2D) or []
    for index in range(11):
        row[f"ray_log2_{index:02d}"] = _fmt(ray2d[index] if index < len(ray2d) else None)
    for prefix in _CMD_CHAIN:
        values = payload.get(prefix) or []
        for index in range(12):
            row[f"{prefix}_{index:02d}"] = _fmt(values[index] if index < len(values) else None)
    return row


def map_telemetry(valid_live_payloads: Sequence[Mapping[str, Any]], run_id: str, context: Mapping[str, Any]) -> List[Dict[str, Any]]:
    return [map_telemetry_row(p, run_id, context) for p in valid_live_payloads]


# ------------------------------------------------------------------ summary
def build_summary(
    terminal: Mapping[str, Any],
    live_payloads: Sequence[Mapping[str, Any]],
    invalid_reasons: Sequence[str],
) -> Dict[str, Any]:
    """Post-run formal summary from the record only; honest INVALID state."""
    ra_values = [_fmt(p.get("ra_value")) for p in live_payloads]
    speeds = []
    for p in live_payloads:
        lin = p.get("lin_vel") or [0.0, 0.0, 0.0]
        speeds.append(((lin[0] or 0.0) ** 2 + (lin[1] or 0.0) ** 2) ** 0.5)
    recovery_steps = sum(1 for p in live_payloads if p.get("policy_state") == 1)
    faulted = sum(1 for p in live_payloads if bool(p.get("safety_faulted")) or p.get("policy_state") == 2)
    metrics = {
        "duration_ns": terminal.get("duration_ns"),
        "velocity_avg_m_s": (sum(speeds) / len(speeds)) if speeds else None,
        "velocity_peak_m_s": max(speeds) if speeds else None,
        "ra_mean": (sum(ra_values) / len(ra_values)) if ra_values else None,
        "ra_min": min(ra_values) if ra_values else None,
        "ra_max": max(ra_values) if ra_values else None,
        "recovery_live_steps": recovery_steps,
        "safety_fault_live_steps": faulted,
        "live_frame_count": len(live_payloads),
    }
    return {
        "validity": "INVALID",  # honest: authoritative sources are missing today
        "terminal_outcome": "UNKNOWN",  # never a fabricated SUCCESS
        "invalid_reasons": list(invalid_reasons),
        "metrics": metrics,
        "terminal_facts": {
            "process_exit_code": terminal.get("process_exit_code"),
            "forced_termination": terminal.get("forced_termination"),
            "shutdown_complete": terminal.get("shutdown_complete"),
            "shutdown_request_source": terminal.get("shutdown_request_source"),
            "normal_shutdown": terminal.get("normal_shutdown"),
        },
    }


# ------------------------------------------------------------------ main bind
def bind_runtime_record(
    record_path: str,
    run_dir: Path,
    context: Mapping[str, Any],
    facts_path: Optional[str] = None,
) -> Dict[str, Any]:
    """Bind the finalized runtime record into a P1-02 formal run; return the verdict."""
    validate_context(context)
    data, facts, terminal, valid_live_payloads = load_and_check_record(record_path, facts_path)

    writer = FormalRunWriter(run_dir)
    run_id = writer.run_id

    # --- manifest (writer allocates run_id; schema-valid or fail closed) ---
    manifest = dict(context)
    try:
        writer.write_manifest(manifest)
    except ValueError as exc:
        raise BindingError(f"formal manifest refused by writer: {exc}") from exc

    # --- telemetry (all rows bound to this run_id) ---
    rows = map_telemetry(valid_live_payloads, run_id, context)
    if not rows:
        raise BindingError("no telemetry rows derived from the runtime record")
    writer.write_telemetry(rows)

    # --- structured events (single authoritative reducer) ---
    events = derive_events(data, terminal, facts, run_id)
    for event in events:
        writer.emit_event(event)

    # --- fixed plots (data-driven, carry source hashes) ---
    for plot in ("ra_switching.svg", "trajectory_obstacles.svg", "command_tracking.svg", "stability.svg", "recovery_markers.svg"):
        writer.write_data_plot(plot, data_points=len(rows))

    # --- summary (honest INVALID state; artifacts/hashes writer-owned) ---
    invalid_reasons = [
        "simulation_time_s:no_authoritative_runtime_source",
        "collision:no_authoritative_runtime_source",
        "fall:no_authoritative_runtime_source",
        "reached_goal:no_authoritative_runtime_source",
        "timeout:no_authoritative_runtime_source",
    ]
    summary = build_summary(terminal, valid_live_payloads, invalid_reasons)
    writer.write_summary(summary)

    # --- P1-02 validator verdict ---
    result = validate_run(run_dir)
    verdict = result.to_dict()
    verdict["runtime_record_run_id"] = data.meta.get("run_id")
    verdict["runtime_session_id"] = valid_live_payloads[0]["session_id"]
    verdict["formal_run_id"] = run_id
    verdict["runtime_live_frames"] = len(valid_live_payloads)
    return verdict


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("record", help="finalized runtime record (.jsonl)")
    parser.add_argument("--facts", default=None, help="orchestrator process-facts sidecar (JSON)")
    parser.add_argument("--context", required=True, help="manifest context JSON (authoritative run facts)")
    parser.add_argument("--run-dir", required=True, help="formal run artifact directory")
    args = parser.parse_args(argv)

    try:
        context = json.loads(Path(args.context).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        print(json.dumps({"binding_error": f"context unreadable: {exc}"}, indent=2, sort_keys=True))
        return 2
    if not isinstance(context, dict):
        print(json.dumps({"binding_error": "context is not a JSON object"}, indent=2, sort_keys=True))
        return 2

    try:
        verdict = bind_runtime_record(args.record, Path(args.run_dir), context, facts_path=args.facts)
    except BindingError as exc:
        print(json.dumps({"binding_error": str(exc)}, indent=2, sort_keys=True))
        return 2
    print(json.dumps(verdict, indent=2, sort_keys=True))
    return 0 if verdict.get("episode_state") == "VALID" else 1


if __name__ == "__main__":
    raise SystemExit(main())
