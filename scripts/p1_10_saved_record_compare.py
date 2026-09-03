#!/usr/bin/env python3
"""Fail-closed, deterministic comparison for one frozen P1-10 pair.

The CLI accepts only ``--pair-dir``.  It derives the pair manifest, Run A and
Run B directories, process facts, context files, and saved runtime records
from that directory.  It never opens shared memory, starts a process, or
accepts arbitrary external record paths.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import stat
import sys
from pathlib import Path
from typing import Any, Dict, List, Mapping, Sequence, Tuple

REPO = Path(__file__).resolve().parents[1]
SCRIPTS = REPO / "scripts"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))

from run_record import load_record, summarize_record  # noqa: E402


SCHEMA = "abs-go2-p1-10-saved-record-comparison/v2"
PAIR_SCHEMA = "abs-go2-p1-10-same-seed-replay-pair/v3"
COMPARISON_RULES_VERSION = "p1-10-saved-record-canonical-projection/v2"
EXPECTED_PAIR_ID = "P1-10-REPLAY-20260903-saved-record-closure-flat_goal_forward-stabilized"
EXPECTED_EVIDENCE_DIR = "docs/evidence/P1-10/replay_pair_20260903_saved_record_closure"
EXPECTED_PAIR_DIR = (REPO / EXPECTED_EVIDENCE_DIR).resolve()

EXPECTED_BINDING = {
    "scenario_id": "flat_goal_forward",
    "scenario_sha256": "beba99ed4e6f6c8f84eb1ac514f2da4b6e910c1587fdf91f5e95ac6bc639e092",
    "suite_manifest_sha256": "eb81d60742864fe9c870e957ba3ab601e80da3e64bc48a42c26f849570f3152d",
    "scene_root_sha256": "9ce83b3e61c722a523d0359536cee803f17610f95d2275fc32e96801ec3c1908",
    "model_closure_sha256": "8d9218de0dc02978fc0ef4ba1c790fa3b968fbdbfdb945e14522436a2574ea07",
    "root_seed": 20260902,
    "variant": "stabilized",
    "switching_mode": "stabilized_switch",
    "variant_binding_sha256": "2f0dfc4e8bf5237a578d99030facc38459fd5f899af49b508e48e29b7e8a4e1c",
    "baseline_manifest_sha256": "2667ed37a854f85e5a7c493e7d4a8b1871a84ce95d3e3b0742801d383f8dc915",
    "baseline_identity_document_sha256": "6c3563c25d45cc275db6b083f9f0fc0cc2067b48bc8f4a93dcace9f6d42817ea",
    "canonical_baseline_identity": "59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0",
    "initial_state_source": "scene_default",
    "initial_state_reset_source": "mj_makeData:qpos0; no keyframe reset",
    "initial_state_qpos_sha256": "a604dd11dc57ea655bf6d746dcf068a91e80a0a1eddc73d20c1a3800468f59d8",
    "initial_state_binding_sha256": "f7907a927c31d3d6a5d497ab274b3d913bf4fc8ccb0e9713a9dbd1e182d0a9a0",
    "run_window_s": 25.0,
}

EXACT_FRAME_FIELDS = (
    "source",
    "controller_active",
    "rl_entered",
    "rl_active",
    "safety_faulted",
    "policy_state",
    "ray_origin",
    "ray_valid",
    "collision_origin",
    "torque_saturated_computed",
)
NUMERIC_FRAME_FIELDS = (
    "world_pose",
    "command",
    "ra_value",
    "action_raw",
    "action_clipped",
    "joint_target_rad",
    "torque_nm",
    "ray2d",
)
EXCLUDED_FIELDS = (
    "run_id",
    "session_id",
    "source_sequence",
    "monotonic_ns",
    "recorded_at_ns",
    "wall_clock_timestamps",
    "PID/PGID",
    "reader_polling_cadence",
    "ray_age_ns",
    "lin_vel",
    "torque_saturated",
    "duration_ns",
    "first_frame_time_ns",
    "last_frame_time_ns",
)
EXACT_TERMINAL_FIELDS = (
    "termination_reason",
    "normal_shutdown",
    "process_exit_code",
    "forced_termination",
    "shutdown_complete",
    "shutdown_request_source",
    "safety_fault_last",
    "safety_fault_seen",
    "reached_goal",
    "timeout",
    "collision_events",
    "fall_events",
)
TERMINAL_UNKNOWN_ALLOWED = frozenset({
    "reached_goal", "timeout", "collision_events", "fall_events",
})
TERMINATION_REASONS = frozenset({
    "SAFETY_FAULT", "FORCED_TERMINATION", "NONZERO_EXIT",
    "FRAMES_ENDED_RC0", "UNKNOWN",
})
TERMINAL_EVENT_FIELDS = (
    "reached_goal", "timeout", "collision_events", "fall_events",
)
TERMINAL_VALUE_DOMAINS = {
    "termination_reason": ["SAFETY_FAULT", "FORCED_TERMINATION", "NONZERO_EXIT", "FRAMES_ENDED_RC0", "UNKNOWN"],
    "normal_shutdown": "bool",
    "process_exit_code": "int",
    "forced_termination": "bool",
    "shutdown_complete": "bool",
    "shutdown_request_source": ["SIGINT", "UNKNOWN"],
    "safety_fault_last": "bool",
    "safety_fault_seen": "bool",
    "reached_goal": ["UNKNOWN"],
    "timeout": ["UNKNOWN"],
    "collision_events": ["UNKNOWN"],
    "fall_events": ["UNKNOWN"],
}
REQUIRED_RUN_FILES = (
    "process_facts.json",
    "runtime_record.jsonl",
    "scenario_resolved_manifest.json",
    "p1_10_context.json",
)
SUCCESS_CHILDREN = ("child.mujoco", "child.ros2_launch")


class ContractError(ValueError):
    """A frozen-pair or saved-record contract violation."""


def canonical_json(value: Any) -> str:
    return json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    )


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path) -> Dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise ContractError(f"cannot parse JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise ContractError(f"expected JSON object: {path}")
    return value


def _known(value: Any, field: str) -> Any:
    if value is None or value == "" or value == "UNKNOWN":
        raise ContractError(f"{field}: missing/None/UNKNOWN")
    if isinstance(value, float) and not math.isfinite(value):
        raise ContractError(f"{field}: non-finite")
    return value


def _bool(value: Any, field: str) -> bool:
    if type(value) is not bool:
        raise ContractError(f"{field}: expected bool")
    return value


def _int(value: Any, field: str) -> int:
    if type(value) is not int:
        raise ContractError(f"{field}: expected int")
    return value


def _binding_from_pair(pair: Mapping[str, Any]) -> Dict[str, Any]:
    scenario = pair.get("scenario", {})
    contract = pair.get("scenario_contract", {})
    baseline = pair.get("baseline", {})
    variant = pair.get("variant", {})
    return {
        "scenario_id": scenario.get("scenario_id"),
        "scenario_sha256": scenario.get("scenario_sha256"),
        "suite_manifest_sha256": scenario.get("suite_manifest_sha256"),
        "scene_root_sha256": contract.get("scene_root_sha256"),
        "model_closure_sha256": contract.get("model_closure_sha256"),
        "root_seed": pair.get("seed", {}).get("root_seed"),
        "variant": variant.get("label"),
        "switching_mode": variant.get("switching_mode"),
        "variant_binding_sha256": variant.get("binding_sha256"),
        "baseline_manifest_sha256": baseline.get("manifest_sha256"),
        "baseline_identity_document_sha256": baseline.get("identity_file_sha256"),
        "canonical_baseline_identity": baseline.get("identity_sha256"),
        "initial_state_source": contract.get("initial_state_source"),
        "initial_state_reset_source": contract.get("initial_state_reset_source"),
        "initial_state_qpos_sha256": contract.get("initial_state_qpos_sha256"),
        "initial_state_binding_sha256": contract.get("initial_state_binding_sha256"),
        "run_window_s": contract.get("run_window_s"),
    }


def _binding_from_context(context: Mapping[str, Any]) -> Dict[str, Any]:
    scenario_id = context.get("scenario_id")
    scenario_sha = context.get("scenario_sha256")
    suite_sha = context.get("suite_sha256")
    baseline = context.get("baseline", {})
    initial_source = context.get("initial_state_source", {})
    initial = context.get("initial_state", {})
    variant = context.get("variant_binding", {})
    pairing = context.get("pairing", {})
    return {
        "scenario_id": scenario_id,
        "scenario_sha256": scenario_sha,
        "suite_manifest_sha256": suite_sha,
        "scene_root_sha256": context.get("scene", {}).get("root_xml_sha256"),
        "model_closure_sha256": context.get("scene", {}).get("model_closure_sha256"),
        "root_seed": context.get("seeds", {}).get("root_seed"),
        "variant": pairing.get("variant"),
        "switching_mode": variant.get("runtime_configuration", {}).get("switching_mode"),
        "variant_binding_sha256": variant.get("binding_sha256"),
        "baseline_manifest_sha256": baseline.get("manifest_sha256"),
        "baseline_identity_document_sha256": baseline.get("identity_file_sha256"),
        "canonical_baseline_identity": baseline.get("identity_sha256"),
        "initial_state_source": initial_source.get("kind"),
        "initial_state_reset_source": initial_source.get("reset_source"),
        "initial_state_qpos_sha256": initial.get("qpos_sha256"),
        "initial_state_binding_sha256": initial.get("binding_sha256"),
        "run_window_s": context.get("run_window_s", context.get("launch_contract", {}).get("window_s")),
    }


def _context_path_checks(context: Mapping[str, Any], label: str) -> None:
    if "scenario_path" in context and context.get("scenario_path") != "scenarios/p1_10/flat_goal_forward.json":
        raise ContractError(f"{label}.scenario_path: drift")
    if "suite_path" in context and context.get("suite_path") != "scenarios/p1_10/scenario_suite_manifest.json":
        raise ContractError(f"{label}.suite_path: drift")
    if "switching_mode" in context and context.get("switching_mode") != "stabilized_switch":
        raise ContractError(f"{label}.switching_mode: drift")
    launch = context.get("launch_contract")
    if not isinstance(launch, Mapping):
        raise ContractError(f"{label}.launch_contract: missing")
    expected = {
        "scenario": "flat_goal_forward",
        "scene": "scene_flat.xml",
        "initial_state_source": "scene_default",
        "root_seed": 20260902,
        "variant": "stabilized",
        "window_s": 25.0,
        "baseline_manifest": "docs/evidence/P1-08/P1-08_baseline_manifest.json",
    }
    for key, expected_value in expected.items():
        if type(launch.get(key)) is not type(expected_value) or launch.get(key) != expected_value:
            raise ContractError(f"{label}.launch_contract.{key}: drift")
    scene = context.get("scene")
    if scene is not None:
        if not isinstance(scene, Mapping):
            raise ContractError(f"{label}.scene: malformed")
        if scene.get("launch_arg") != "scene_flat.xml":
            raise ContractError(f"{label}.scene.launch_arg: drift")
        if scene.get("root_xml_sha256") != EXPECTED_BINDING["scene_root_sha256"]:
            raise ContractError(f"{label}.scene.root_xml_sha256: drift")
        if scene.get("model_closure_sha256") != EXPECTED_BINDING["model_closure_sha256"]:
            raise ContractError(f"{label}.scene.model_closure_sha256: drift")
    variant = context.get("variant_binding")
    if (
        not isinstance(variant, Mapping)
        or variant.get("status") != "SUPPORTED"
        or variant.get("label") != "stabilized"
    ):
        raise ContractError(f"{label}.variant_binding: unsupported/missing")
    consumed = variant.get("consumed_behavior")
    if not isinstance(consumed, Mapping) or consumed.get("controller_plugin_path") != (
        "quadruped_ros2_control_humble/install/rl_quadruped_controller/lib/"
        "rl_quadruped_controller/librl_quadruped_controller.so"
    ):
        raise ContractError(f"{label}.variant_binding.controller_plugin_path: drift")


def validate_context(
    context: Mapping[str, Any],
    pair_binding: Mapping[str, Any],
    label: str,
    *,
    require_full_binding: bool = False,
) -> Dict[str, Any]:
    _context_path_checks(context, label)
    actual = _binding_from_context(context)
    if require_full_binding is not True:
        raise ContractError(f"{label}: full binding validation is required")
    for key in EXPECTED_BINDING:
        value = _known(actual.get(key), f"{label}.{key}")
        if type(value) is not type(pair_binding.get(key)):
            raise ContractError(f"{label}.{key}: wrong type")
        if actual.get(key) != pair_binding.get(key):
            raise ContractError(f"{label}.{key}: does not match frozen pair")
    run_window = context.get("run_window_s", context.get("launch_contract", {}).get("window_s"))
    if type(run_window) is not float or run_window != 25.0:
        raise ContractError(f"{label}.run_window_s: drift")
    return actual


def validate_pair_manifest(pair: Mapping[str, Any]) -> List[str]:
    errors: List[str] = []
    if pair.get("schema") != PAIR_SCHEMA:
        errors.append(f"schema must be {PAIR_SCHEMA}")
    if pair.get("pair_id") != EXPECTED_PAIR_ID:
        errors.append("pair_id is not this frozen pair")
    if pair.get("evidence_dir") != EXPECTED_EVIDENCE_DIR:
        errors.append("evidence_dir is not this frozen pair")
    if pair.get("status_at_freeze") != "FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW":
        errors.append("pair is not frozen pending independent review")
    actual = _binding_from_pair(pair)
    for key, expected in EXPECTED_BINDING.items():
        if type(actual.get(key)) is not type(expected) or actual.get(key) != expected:
            errors.append(f"binding.{key} mismatch")
    planned = pair.get("planned_runs")
    if not isinstance(planned, Mapping):
        errors.append("planned_runs missing")
    else:
        for label in ("run_a", "run_b"):
            spec = planned.get(label)
            if not isinstance(spec, Mapping) or spec.get("directory") != ("run_A" if label == "run_a" else "run_B"):
                errors.append(f"planned_runs.{label}.directory mismatch")
            elif spec.get("required_files") != list(REQUIRED_RUN_FILES):
                errors.append(f"planned_runs.{label}.required_files mismatch")
    comparison = pair.get("comparison", {})
    if comparison.get("schema") != COMPARISON_RULES_VERSION:
        errors.append("comparison schema mismatch")
    if comparison.get("record_source") != "saved runtime_record.jsonl only; never live shared memory":
        errors.append("comparison record source mismatch")
    if comparison.get("context_binding_policy") != "scenario_resolved_manifest.json and p1_10_context.json in every run must each explicitly contain every pair-required binding field; no fallback from pair manifest, the other context, process facts, defaults, or None":
        errors.append("context binding policy mismatch")
    if comparison.get("exact_frame_fields") != list(EXACT_FRAME_FIELDS):
        errors.append("exact frame rule mismatch")
    if comparison.get("numeric_frame_fields") != list(NUMERIC_FRAME_FIELDS):
        errors.append("numeric frame rule mismatch")
    if comparison.get("exact_terminal_fields") != list(EXACT_TERMINAL_FIELDS):
        errors.append("exact terminal rule mismatch")
    if comparison.get("terminal_value_domains") != TERMINAL_VALUE_DOMAINS:
        errors.append("terminal value-domain rule mismatch")
    if comparison.get("excluded_fields") != list(EXCLUDED_FIELDS):
        errors.append("excluded field rule mismatch")
    if comparison.get("numeric_rule") != "exact canonical JSON equality; any nonzero delta fails; report max/mean delta diagnostically":
        errors.append("numeric equality rule mismatch")
    if comparison.get("exact_sequence_rules") != [
        "frame count must match",
        "strict rl_step sequence must match",
        "policy_state sequence and derived Recovery entry/exit sequence must match",
        "forced_termination must be false in both terminals",
    ]:
        errors.append("exact sequence rule mismatch")
    if comparison.get("canonical_encoding") != "json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False); sha256":
        errors.append("canonical encoding rule mismatch")
    return errors


def _bindings_match(left: Mapping[str, Any], right: Mapping[str, Any], label: str) -> None:
    for key, expected in EXPECTED_BINDING.items():
        left_value = _known(left.get(key), f"{label}.left.{key}")
        right_value = _known(right.get(key), f"{label}.right.{key}")
        if type(left_value) is not type(expected) or type(right_value) is not type(expected):
            raise ContractError(f"{label}.{key}: wrong type")
        if left_value != right_value:
            raise ContractError(f"{label}.{key}: binding mismatch")


def _run_dirs(pair_dir: Path, pair: Mapping[str, Any]) -> Dict[str, Path]:
    planned = pair["planned_runs"]
    result: Dict[str, Path] = {}
    for label, expected_dir in (("run_a", "run_A"), ("run_b", "run_B")):
        relative = Path(planned[label]["directory"])
        if relative.is_absolute() or relative.parts != (expected_dir,):
            raise ContractError(f"{label}: run directory path substitution rejected")
        candidate = pair_dir / relative
        try:
            candidate_stat = candidate.lstat()
        except OSError as exc:
            raise ContractError(f"{label}: cannot lstat run directory: {exc}") from exc
        if stat.S_ISLNK(candidate_stat.st_mode) or not stat.S_ISDIR(candidate_stat.st_mode):
            raise ContractError(f"{label}: run directory is not a real directory")
        try:
            resolved = candidate.resolve(strict=True)
        except OSError as exc:
            raise ContractError(f"{label}: cannot resolve run directory: {exc}") from exc
        if resolved.parent != pair_dir.resolve() or resolved.name != expected_dir:
            raise ContractError(f"{label}: run directory escapes frozen pair")
        result[label] = resolved
    return result


def _require_regular_file(path: Path, label: str, parent: Path) -> Path:
    """Require an ordinary, non-symlink file directly under ``parent``."""
    if path.parent != parent:
        raise ContractError(f"{label}: artifact is not a direct child of its run directory")
    try:
        file_stat = path.lstat()
    except OSError as exc:
        raise ContractError(f"{label}: cannot lstat artifact: {exc}") from exc
    if stat.S_ISLNK(file_stat.st_mode) or not stat.S_ISREG(file_stat.st_mode):
        raise ContractError(f"{label}: artifact is not a regular non-symlink file")
    try:
        resolved = path.resolve(strict=True)
    except OSError as exc:
        raise ContractError(f"{label}: cannot resolve artifact: {exc}") from exc
    if resolved.parent != parent.resolve():
        raise ContractError(f"{label}: artifact resolves outside its run directory")
    return resolved


def _validate_process_facts(facts: Mapping[str, Any], context_binding: Mapping[str, Any], record_data: Any, label: str) -> Dict[str, Any]:
    for key in ("exit_code", "shutdown_complete", "forced_termination", "shutdown_request_source", "run_id", "scene"):
        _known(facts.get(key), f"{label}.process_facts.{key}")
    if _int(facts["exit_code"], f"{label}.process_facts.exit_code") != 0:
        raise ContractError(f"{label}: coordinator exit_code is not zero")
    if _bool(facts["shutdown_complete"], f"{label}.process_facts.shutdown_complete") is not True:
        raise ContractError(f"{label}: shutdown_complete is not true")
    if _bool(facts["forced_termination"], f"{label}.process_facts.forced_termination") is not False:
        raise ContractError(f"{label}: forced_termination is not false")
    if facts["shutdown_request_source"] != "SIGINT":
        raise ContractError(f"{label}: shutdown source is not SIGINT")
    if facts["scene"] != "scene_flat.xml":
        raise ContractError(f"{label}: scene drift")
    if facts.get("cleanup_error_count") != 0:
        raise ContractError(f"{label}: cleanup errors are not zero")
    if facts.get("run_id") != record_data.meta.get("run_id"):
        raise ContractError(f"{label}: process facts/run record run_id mismatch")
    process_context = facts.get("p1_10_context")
    if not isinstance(process_context, Mapping):
        raise ContractError(f"{label}: process facts binding context missing")
    validate_context(
        process_context,
        context_binding,
        f"{label}.process_facts.p1_10_context",
        require_full_binding=True,
    )
    for child_name in SUCCESS_CHILDREN:
        child = facts.get(child_name)
        if not isinstance(child, Mapping):
            raise ContractError(f"{label}.{child_name}: missing")
        for key in ("exit_code", "not_launched", "escalated", "pid", "pgid", "signals", "cleanup_errors"):
            _known(child.get(key), f"{label}.{child_name}.{key}")
        if _int(child["exit_code"], f"{label}.{child_name}.exit_code") != 0:
            raise ContractError(f"{label}.{child_name}: wait rc is not zero")
        if _bool(child["not_launched"], f"{label}.{child_name}.not_launched") is not False:
            raise ContractError(f"{label}.{child_name}: child was not launched")
        if _bool(child["escalated"], f"{label}.{child_name}.escalated") is not False:
            raise ContractError(f"{label}.{child_name}: escalation present")
        if _int(child["pid"], f"{label}.{child_name}.pid") <= 0 or _int(child["pgid"], f"{label}.{child_name}.pgid") <= 0:
            raise ContractError(f"{label}.{child_name}: invalid pid/pgid")
        if child["cleanup_errors"] != []:
            raise ContractError(f"{label}.{child_name}: cleanup errors present")
        signals = child["signals"]
        if not isinstance(signals, list) or not any(
            isinstance(item, Mapping) and item.get("signal") == "SIGINT" and item.get("delivered") is True
            for item in signals
        ):
            raise ContractError(f"{label}.{child_name}: delivered SIGINT fact missing")
        if any(
            isinstance(item, Mapping) and item.get("signal") in ("SIGTERM", "SIGKILL") and item.get("delivered") is True
            for item in signals
        ):
            raise ContractError(f"{label}.{child_name}: delivered forced signal present")
    return {
        "exit_code": facts["exit_code"],
        "shutdown_complete": facts["shutdown_complete"],
        "forced_termination": facts["forced_termination"],
        "shutdown_request_source": facts["shutdown_request_source"],
        "run_id": facts["run_id"],
        "children": {name: {"exit_code": facts[name]["exit_code"], "pid": facts[name]["pid"], "pgid": facts[name]["pgid"]} for name in SUCCESS_CHILDREN},
    }


def _validate_terminal(record_data: Any, summary: Mapping[str, Any], process_facts: Mapping[str, Any], label: str) -> Dict[str, Any]:
    if summary.get("record_validity") != "VALID" or summary.get("authoritative_runtime_source") is not True:
        raise ContractError(f"{label}: runtime record is not VALID authoritative data")
    if record_data.terminal_count != 1 or record_data.terminal_is_last is not True:
        raise ContractError(f"{label}: terminal is not unique final line")
    terminal = record_data.terminal
    if not isinstance(terminal, Mapping):
        raise ContractError(f"{label}: terminal missing")
    for key in ("frames_observed", "first_frame_time_ns", "last_frame_time_ns", "duration_ns", "last_session_id"):
        if key not in terminal or terminal[key] is None:
            raise ContractError(f"{label}.terminal.{key}: missing/None")
    if _int(terminal["frames_observed"], f"{label}.terminal.frames_observed") != len(record_data.frames):
        raise ContractError(f"{label}: terminal frame count mismatch")
    for key in ("first_frame_time_ns", "last_frame_time_ns", "duration_ns", "last_session_id"):
        if _int(terminal[key], f"{label}.terminal.{key}") < 0:
            raise ContractError(f"{label}.terminal.{key}: negative")
    if _int(terminal["duration_ns"], f"{label}.terminal.duration_ns") != (
        terminal["last_frame_time_ns"] - terminal["first_frame_time_ns"]
    ):
        raise ContractError(f"{label}: terminal duration mismatch")
    for key in EXACT_TERMINAL_FIELDS:
        if key not in terminal or terminal[key] is None:
            raise ContractError(f"{label}.terminal.{key}: missing/None")
    if not isinstance(terminal["termination_reason"], str) or terminal["termination_reason"] not in TERMINATION_REASONS:
        raise ContractError(f"{label}.terminal.termination_reason: invalid value")
    for key in ("normal_shutdown", "forced_termination", "shutdown_complete", "safety_fault_last", "safety_fault_seen"):
        _bool(terminal[key], f"{label}.terminal.{key}")
    if not isinstance(terminal["process_exit_code"], int) or isinstance(terminal["process_exit_code"], bool):
        raise ContractError(f"{label}.terminal.process_exit_code: expected int")
    if not isinstance(terminal["shutdown_request_source"], str) or terminal["shutdown_request_source"] != "SIGINT":
        raise ContractError(f"{label}.terminal.shutdown_request_source: invalid value")
    for key in TERMINAL_EVENT_FIELDS:
        if type(terminal[key]) is not str or terminal[key] != "UNKNOWN":
            raise ContractError(f"{label}.terminal.{key}: expected exact UNKNOWN string")
    if terminal["normal_shutdown"] is not True:
        raise ContractError(f"{label}: normal_shutdown is not true")
    if terminal["process_exit_code"] != 0:
        raise ContractError(f"{label}: terminal process_exit_code is not zero")
    if terminal["forced_termination"] is not False:
        raise ContractError(f"{label}: terminal forced_termination is not false")
    if terminal["shutdown_complete"] is not True:
        raise ContractError(f"{label}: terminal shutdown incomplete")
    if terminal.get("fact_validation_errors") != []:
        raise ContractError(f"{label}: terminal fact validation errors present")
    if terminal.get("run_id") != record_data.meta.get("run_id") or terminal.get("run_id") != process_facts.get("run_id"):
        raise ContractError(f"{label}: terminal/process facts/run record run_id mismatch")
    live_sessions = {
        frame.get("payload", {}).get("session_id")
        for frame in record_data.frames
        if isinstance(frame.get("payload"), Mapping) and frame.get("status") == "LIVE"
    }
    if len(live_sessions) != 1 or None in live_sessions:
        raise ContractError(f"{label}: runtime session identity is missing/inconsistent")
    session_id = next(iter(live_sessions))
    if terminal.get("last_session_id") != session_id:
        raise ContractError(f"{label}: terminal/session identity mismatch")
    return {key: terminal[key] for key in EXACT_TERMINAL_FIELDS}


def _load_run(pair_dir: Path, run_dir: Path, pair_binding: Mapping[str, Any], label: str) -> Dict[str, Any]:
    paths = {
        name: _require_regular_file(run_dir / name, f"{label}.{name}", run_dir)
        for name in REQUIRED_RUN_FILES
    }
    resolved_context = _load_json(paths["scenario_resolved_manifest.json"])
    p1_context = _load_json(paths["p1_10_context.json"])
    binding_a = validate_context(
        resolved_context,
        pair_binding,
        f"{label}.scenario_resolved_manifest",
        require_full_binding=True,
    )
    binding_b = validate_context(
        p1_context,
        pair_binding,
        f"{label}.p1_10_context",
        require_full_binding=True,
    )
    _bindings_match(binding_a, binding_b, f"{label}: resolved manifest/context")
    record_data = load_record(str(paths["runtime_record.jsonl"]))
    summary = summarize_record(str(paths["runtime_record.jsonl"]))
    process_facts = _load_json(paths["process_facts.json"])
    process_summary = _validate_process_facts(process_facts, binding_a, record_data, label)
    terminal = _validate_terminal(record_data, summary, process_facts, label)
    return {
        "directory": str(run_dir.relative_to(pair_dir)),
        "paths": paths,
        "binding": binding_a,
        "record_data": record_data,
        "summary": summary,
        "process_facts": process_facts,
        "process_summary": process_summary,
        "terminal": terminal,
    }


def _policy_name(value: Any) -> str:
    return {0: "AGILE", 1: "RECOVERY", 2: "FAULTED"}.get(value, "UNKNOWN")


def _recovery_transitions(frames: Sequence[Mapping[str, Any]]) -> List[Dict[str, Any]]:
    transitions: List[Dict[str, Any]] = []
    previous = None
    for frame in frames:
        payload = frame.get("payload")
        if not isinstance(payload, Mapping):
            continue
        current = payload.get("policy_state")
        if current != previous:
            transitions.append({"rl_step": payload.get("rl_step"), "from": _policy_name(previous), "to": _policy_name(current)})
        previous = current
    return transitions


def _project_record(data: Any) -> Dict[str, Any]:
    frames: List[Dict[str, Any]] = []
    rl_steps: List[Any] = []
    for frame in data.frames:
        payload = frame.get("payload")
        projected: Dict[str, Any] = {"status": frame.get("status"), "availability": frame.get("availability")}
        if isinstance(payload, Mapping):
            projected["exact"] = {key: payload.get(key) for key in EXACT_FRAME_FIELDS}
            projected["numeric"] = {key: payload.get(key) for key in NUMERIC_FRAME_FIELDS}
            if "rl_step" in payload:
                rl_steps.append(payload["rl_step"])
        else:
            projected["exact"] = None
            projected["numeric"] = None
        frames.append(projected)
    terminal = data.terminal
    return {
        "meta": {"record_format_version": data.meta.get("record_format_version"), "source": data.meta.get("source")},
        "frame_count": len(data.frames),
        "rl_step_sequence": rl_steps,
        "frames": frames,
        "recovery_entry_exit_sequence": _recovery_transitions(data.frames),
        "terminal": {key: terminal[key] for key in EXACT_TERMINAL_FIELDS},
    }


def _diff_values(left: Any, right: Any, path: str, differences: List[Dict[str, Any]]) -> None:
    if type(left) is not type(right):
        differences.append({"path": path, "kind": "type", "a": left, "b": right})
        return
    if isinstance(left, dict):
        for key in sorted(set(left) | set(right)):
            if key not in left or key not in right:
                differences.append({"path": f"{path}.{key}", "kind": "missing", "a": left.get(key), "b": right.get(key)})
            else:
                _diff_values(left[key], right[key], f"{path}.{key}", differences)
    elif isinstance(left, list):
        if len(left) != len(right):
            differences.append({"path": path, "kind": "length", "a": len(left), "b": len(right)})
        else:
            for index, (a, b) in enumerate(zip(left, right)):
                _diff_values(a, b, f"{path}[{index}]", differences)
    elif left != right:
        differences.append({"path": path, "kind": "value", "a": left, "b": right})


def _numeric_stats(left: Any, right: Any, path: str) -> Dict[str, Any]:
    deltas: List[float] = []

    def walk(a: Any, b: Any) -> None:
        if isinstance(a, list) and isinstance(b, list) and len(a) == len(b):
            for item_a, item_b in zip(a, b):
                walk(item_a, item_b)
        elif isinstance(a, (int, float)) and isinstance(b, (int, float)):
            deltas.append(abs(float(a) - float(b)))

    walk(left, right)
    nonzero = [value for value in deltas if value != 0.0]
    return {
        "path": path,
        "sample_count": len(deltas),
        "nonzero_count": len(nonzero),
        "max_abs_delta": max(deltas, default=0.0),
        "mean_abs_delta": sum(deltas) / len(deltas) if deltas else 0.0,
    }


def _numeric_diagnostics(left: Mapping[str, Any], right: Mapping[str, Any]) -> List[Dict[str, Any]]:
    diagnostics: List[Dict[str, Any]] = []
    for index, (left_frame, right_frame) in enumerate(zip(left["frames"], right["frames"])):
        left_numeric = left_frame.get("numeric")
        right_numeric = right_frame.get("numeric")
        if not isinstance(left_numeric, Mapping) or not isinstance(right_numeric, Mapping):
            continue
        for field in NUMERIC_FRAME_FIELDS:
            diagnostics.append(_numeric_stats(left_numeric.get(field), right_numeric.get(field), f"frames[{index}].numeric.{field}"))
    return diagnostics


def compare_pair_dir(pair_dir: Path) -> Dict[str, Any]:
    pair_dir = Path(pair_dir)
    try:
        pair_stat = pair_dir.lstat()
    except OSError as exc:
        raise ContractError(f"pair-dir cannot be lstat'ed: {exc}") from exc
    if stat.S_ISLNK(pair_stat.st_mode) or not stat.S_ISDIR(pair_stat.st_mode):
        raise ContractError("pair-dir must be a real directory, not a symlink")
    try:
        pair_dir = pair_dir.resolve(strict=True)
    except OSError as exc:
        raise ContractError(f"pair-dir cannot be resolved: {exc}") from exc
    expected_pair_dir = (REPO / EXPECTED_EVIDENCE_DIR).resolve()
    if pair_dir != expected_pair_dir:
        raise ContractError("pair-dir is not the frozen pair directory")
    pair_path = pair_dir / "pair_manifest.json"
    pair_path = _require_regular_file(pair_path, "pair_manifest.json", pair_dir)
    pair = _load_json(pair_path)
    errors = validate_pair_manifest(pair)
    if errors:
        raise ContractError("pair manifest rejected: " + "; ".join(errors))
    pair_binding = _binding_from_pair(pair)
    run_dirs = _run_dirs(pair_dir, pair)
    run_a = _load_run(pair_dir, run_dirs["run_a"], pair_binding, "run_a")
    run_b = _load_run(pair_dir, run_dirs["run_b"], pair_binding, "run_b")
    if run_a["binding"] != run_b["binding"]:
        raise ContractError("run_A/run_B frozen binding mismatch")
    projection_a = _project_record(run_a["record_data"])
    projection_b = _project_record(run_b["record_data"])
    differences: List[Dict[str, Any]] = []
    _diff_values(projection_a, projection_b, "$", differences)
    canonical_projection = {"binding": pair_binding, "run_a": projection_a, "run_b": projection_b}
    canonical_hash = sha256_bytes(canonical_json(canonical_projection).encode("utf-8"))
    return {
        "schema": SCHEMA,
        "status": "PASS" if not differences else "FAIL",
        "pair_id": pair["pair_id"],
        "pair_manifest_sha256": sha256_file(pair_path),
        "binding": pair_binding,
        "comparison_rules_schema": COMPARISON_RULES_VERSION,
        "record_source": "saved runtime_record.jsonl only; never live shared memory",
        "canonical_projection_sha256": canonical_hash,
        "exact_match": not differences,
        "difference_count": len(differences),
        "differences": differences,
        "numeric_rule": "exact canonical JSON equality; any nonzero delta fails",
        "numeric_diagnostics": _numeric_diagnostics(projection_a, projection_b),
        "run_a": {
            "directory": run_a["directory"],
            "record_sha256": sha256_file(run_a["paths"]["runtime_record.jsonl"]),
            "process_facts_sha256": sha256_file(run_a["paths"]["process_facts.json"]),
            "scenario_resolved_manifest_sha256": sha256_file(run_a["paths"]["scenario_resolved_manifest.json"]),
            "p1_10_context_sha256": sha256_file(run_a["paths"]["p1_10_context.json"]),
            "process_success": run_a["process_summary"],
            "terminal": run_a["terminal"],
        },
        "run_b": {
            "directory": run_b["directory"],
            "record_sha256": sha256_file(run_b["paths"]["runtime_record.jsonl"]),
            "process_facts_sha256": sha256_file(run_b["paths"]["process_facts.json"]),
            "scenario_resolved_manifest_sha256": sha256_file(run_b["paths"]["scenario_resolved_manifest.json"]),
            "p1_10_context_sha256": sha256_file(run_b["paths"]["p1_10_context.json"]),
            "process_success": run_b["process_summary"],
            "terminal": run_b["terminal"],
        },
        "canonical_input": {
            "schema": SCHEMA,
            "comparison_rules_schema": COMPARISON_RULES_VERSION,
            "binding": pair_binding,
            "run_a_projection": projection_a,
            "run_b_projection": projection_b,
        },
    }


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")


def _markdown_report(result: Mapping[str, Any]) -> str:
    lines = [
        "# P1-10 Saved-Record Comparison Report",
        "",
        f"- Pair: `{result['pair_id']}`",
        f"- Status: **{result['status']}**",
        f"- Pair manifest SHA-256: `{result['pair_manifest_sha256']}`",
        f"- Canonical projection SHA-256: `{result['canonical_projection_sha256']}`",
        f"- Difference count: `{result['difference_count']}`",
        "- Source: saved `run_A/runtime_record.jsonl` and `run_B/runtime_record.jsonl` only",
        "",
        "## Binding",
        "",
    ]
    for key in sorted(result["binding"]):
        lines.append(f"- `{key}`: `{result['binding'][key]}`")
    lines.extend(["", "## Differences", ""])
    if result["differences"]:
        for item in result["differences"]:
            lines.append("- " + json.dumps(item, ensure_ascii=False, sort_keys=True))
    else:
        lines.append("- None")
    lines.extend(["", "## Numeric diagnostics", ""])
    for item in result["numeric_diagnostics"]:
        lines.append("- " + json.dumps(item, ensure_ascii=False, sort_keys=True))
    lines.append("")
    return "\n".join(lines)


def write_comparison_outputs(result: Mapping[str, Any], pair_dir: Path) -> None:
    targets = [
        pair_dir / "canonical_identity_input.json",
        pair_dir / "canonical_identity_output.json",
        pair_dir / "diff_report.json",
        pair_dir / "saved_record_comparison_report.md",
    ]
    existing = [str(path) for path in targets if path.exists()]
    if existing:
        raise FileExistsError("refusing to overwrite comparison output(s): " + ", ".join(existing))
    output = {key: value for key, value in result.items() if key != "canonical_input"}
    _write_json(targets[0], result["canonical_input"])
    _write_json(targets[1], output)
    _write_json(targets[2], {
        "schema": SCHEMA,
        "pair_id": result["pair_id"],
        "status": result["status"],
        "canonical_projection_sha256": result["canonical_projection_sha256"],
        "difference_count": result["difference_count"],
        "differences": result["differences"],
        "numeric_diagnostics": result["numeric_diagnostics"],
        "record_source": result["record_source"],
    })
    targets[3].write_text(_markdown_report(result), encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pair-dir", required=True, help="the already-frozen P1-10 pair directory")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    pair_dir = Path(args.pair_dir)
    try:
        result = compare_pair_dir(pair_dir)
        write_comparison_outputs(result, pair_dir)
    except Exception as exc:  # fail closed; no traceback is a comparison result
        print(f"REJECT: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({
        "status": result["status"],
        "pair_id": result["pair_id"],
        "canonical_projection_sha256": result["canonical_projection_sha256"],
        "difference_count": result["difference_count"],
    }, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
