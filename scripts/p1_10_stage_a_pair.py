#!/usr/bin/env python3
"""Create and validate the fresh current-instrumented Stage-A flat pair."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

REPO = Path(__file__).resolve().parents[1]
EVIDENCE_DIR = REPO / "docs/evidence/P1-10"
PAIR_DIR_REL = "docs/evidence/P1-10/replay_pair_20260907_stage_a_current_instrumented"
PAIR_DIR = REPO / PAIR_DIR_REL
PAIR_ID = "P1-10-STAGE-A-REPLAY-20260907-flat_goal_forward-stabilized-current-instrumented"
STAGE_MANIFEST = EVIDENCE_DIR / "stage_a_execution_manifest_20260905.json"
STAGE_MANIFEST_REL = "docs/evidence/P1-10/stage_a_execution_manifest_20260905.json"
STATUS = "FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW"
REQUIRED_RUN_FILES = (
    "preflight_evidence.json",
    "process_facts.json",
    "runtime_record.jsonl",
    "scenario_resolved_manifest.json",
    "p1_10_context.json",
)


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False, allow_nan=False).encode("utf-8")


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _load(path: Path) -> Dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected JSON object: {path}")
    return value


def _pair_locator(pair_dir_rel: str, pair_id: str) -> tuple[Path, str, str]:
    """Validate a new pair locator before deriving any manifest content."""
    candidate = Path(pair_dir_rel)
    if candidate.is_absolute() or candidate.parts[:3] != ("docs", "evidence", "P1-10"):
        raise ValueError("pair-dir must be a repository-relative child of docs/evidence/P1-10")
    if len(candidate.parts) != 4 or candidate.parts[-1] in {".", ".."}:
        raise ValueError("pair-dir must be one direct child under docs/evidence/P1-10")
    if not isinstance(pair_id, str) or not pair_id or pair_id in {"UNKNOWN", ".", ".."}:
        raise ValueError("pair-id must be a non-empty known string")
    pair_dir = (REPO / candidate).resolve()
    if pair_dir.parent != EVIDENCE_DIR.resolve():
        raise ValueError("pair-dir must remain directly under docs/evidence/P1-10")
    return pair_dir, candidate.as_posix(), pair_id


def build_pair_manifest(
    *, pair_dir_rel: str = PAIR_DIR_REL, pair_id: str = PAIR_ID,
    operator_runbook: str = "docs/evidence/P1-10/stage_a_operator_runbook_20260907.md",
) -> Dict[str, Any]:
    pair_dir, pair_dir_rel, pair_id = _pair_locator(pair_dir_rel, pair_id)
    import sys
    sys.path.insert(0, str(REPO / "scripts"))
    import p1_10_saved_record_compare as comparator  # noqa: E402

    stage = _load(STAGE_MANIFEST)
    identity = stage["identity_input"]
    fixed = identity["fixed_binding"]
    scenario = identity["scenario"]
    scene = identity["scene"]
    initial = identity["initial_state"]
    goal = identity["goal"]
    artifacts = identity["artifacts"]
    by_role = {item["role"]: item for item in artifacts}
    resolved = {
        "scenario_id": "flat_goal_forward",
        "scenario_sha256": scenario["sha256"],
        "suite_manifest_sha256": scenario["suite_sha256"],
    }
    # The pairing keys are taken from the production resolver, never guessed.
    from p1_10_scenario_suite import resolve_scenario  # noqa: E402
    resolved_context = resolve_scenario("flat_goal_forward", 20260902, "stabilized")
    return {
        "schema": "abs-go2-p1-10-same-seed-replay-pair/v3",
        "pair_id": pair_id,
        "status_at_freeze": STATUS,
        "evidence_dir": pair_dir_rel,
        "stage_a_execution_manifest": {
            "path": STAGE_MANIFEST_REL,
            "sha256": sha256_file(STAGE_MANIFEST),
            "identity_sha256": stage["identity_sha256"],
            "status": stage["status"],
        },
        "baseline": {
            "manifest_path": identity["baseline_parent"]["manifest_path"],
            "manifest_sha256": identity["baseline_parent"]["manifest_sha256"],
            "identity_path": identity["baseline_parent"]["identity_path"],
            "identity_file_sha256": identity["baseline_parent"]["identity_file_sha256"],
            "identity_sha256": identity["baseline_parent"]["canonical_identity_sha256"],
        },
        "scenario": {
            "scenario_id": scenario["id"], "scenario_path": scenario["path"],
            "scenario_sha256": scenario["sha256"], "suite_path": scenario["suite_path"],
            "suite_manifest_sha256": scenario["suite_sha256"],
        },
        "seed": {"root_seed": fixed["root_seed"], "role": "pairing/provenance_only",
                 "scenario_root_seed_key": resolved_context["pairing"]["scenario_root_seed_key"],
                 "p1_02_pairing_key": resolved_context["pairing"]["p1_02_pairing_key"],
                 "derived_seed_registry": {},
                 "random_producers": "none used in this fixed scene/default-initial-state/fixed-goal path"},
        "variant": {"label": "stabilized", "status": "SUPPORTED", "switching_mode": fixed["switching_mode"],
                    "binding_sha256": identity["variant_binding"]["binding_sha256"]},
        "scenario_contract": {
            "scene": scene["launch_arg"], "scene_root_sha256": scene["root_xml_sha256"],
            "model_closure_sha256": scene["model_closure_sha256"],
            "goal_world_xy_m": goal["goal"]["world_xy_m"], "resample_goal_on_arrival": False,
            "obstacle_layout": {"status": "EMPTY", "objects": []}, "config_overrides": {},
            "run_window_s": fixed["window_s"], "initial_state_source": fixed["initial_state_source"],
            "initial_state_reset_source": fixed["initial_state_reset_source"],
            "initial_state_startup_path": initial["source"]["startup_path"],
            "initial_state_qpos_sha256": initial["qpos_sha256"],
            "initial_state_binding_sha256": initial["binding_sha256"],
        },
        "runtime_identity": {
            "stage_a_manifest_sha256": sha256_file(STAGE_MANIFEST),
            "stage_a_identity_sha256": stage["identity_sha256"],
            "mujoco_executable": {"path": by_role["stage_a_mujoco_executable"]["path"], "sha256": by_role["stage_a_mujoco_executable"]["sha256"], "bytes": by_role["stage_a_mujoco_executable"]["bytes"]},
            "libmujoco": {"path": by_role["libmujoco_shared"]["path"], "sha256": by_role["libmujoco_shared"]["sha256"], "bytes": by_role["libmujoco_shared"]["bytes"]},
            "simulate_config_sha256": by_role["mujoco_simulate_config"]["sha256"],
            "robot_control_config_sha256": by_role["robot_control_config"]["sha256"],
            "abs_controller_config_sha256": by_role["abs_controller_config"]["sha256"],
            "launch_file_sha256": by_role["mujoco_launch"]["sha256"],
            "controller_plugin": {"path": by_role["controller_plugin"]["path"], "sha256": by_role["controller_plugin"]["sha256"], "bytes": by_role["controller_plugin"]["bytes"]},
            "hardware_plugin": {"path": by_role["hardware_plugin"]["path"], "sha256": by_role["hardware_plugin"]["sha256"], "bytes": by_role["hardware_plugin"]["bytes"]},
            "config_plugin_records": [item for item in artifacts if item["role"] in {"mujoco_simulate_config", "robot_control_config", "abs_controller_config", "mujoco_launch", "controller_plugin", "hardware_plugin"}],
        },
        "capture_identity_contract": identity["capture_identity_contract"],
        "comparison": {
            "schema": comparator.COMPARISON_RULES_VERSION,
            "record_source": "saved runtime_record.jsonl only; never live shared memory",
            "context_binding_policy": "scenario_resolved_manifest.json and p1_10_context.json in every run must each explicitly contain every pair-required binding field; no fallback from pair manifest, the other context, process facts, defaults, or None",
            "preflight_binding_policy": "preflight_evidence.json is required in every run and must explicitly bind the Stage-A manifest, pair identity, executable, scene closure, config/plugin, scenario, variant, seed, window, and initial qpos",
            "exact_frame_fields": list(comparator.EXACT_FRAME_FIELDS), "numeric_frame_fields": list(comparator.NUMERIC_FRAME_FIELDS),
            "exact_terminal_fields": list(comparator.EXACT_TERMINAL_FIELDS), "terminal_value_domains": comparator.TERMINAL_VALUE_DOMAINS,
            "excluded_fields": list(comparator.EXCLUDED_FIELDS), "numeric_rule": "exact canonical JSON equality; any nonzero delta fails; report max/mean delta diagnostically",
            "exact_sequence_rules": ["frame count must match", "strict rl_step sequence must match", "policy_state sequence and derived Recovery entry/exit sequence must match", "forced_termination must be false in both terminals"],
            "canonical_encoding": "json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False); sha256",
            "require_preflight_evidence": True,
        },
        "execution_policy": {"operator_launch_status": "PROHIBITED_UNTIL_INDEPENDENT_REVIEW_ACCEPTS_OFFLINE_CLOSURE",
                             "ordering": "Run A exactly once; Run B exactly once only after clean Run A; any failure is terminal and not retried",
                             "not_authorized": ["benchmark", "pilot", "multi-seed", "FormalRun acceptance", "obstacle scenario", "P1-11", "P1-12", "P1-13"],
                             "historical_pairs_not_reusable": ["docs/evidence/P1-10/replay_pair_20260902", "docs/evidence/P1-10/replay_pair_20260903", "docs/evidence/P1-10/replay_pair_20260905_stage_a_current_instrumented"]},
        "planned_runs": {"run_a": {"directory": "run_A", "required_files": list(REQUIRED_RUN_FILES)},
                         "run_b": {"directory": "run_B", "required_files": list(REQUIRED_RUN_FILES)},
                         "both_must_be_successful_before_comparison": True,
                         "comparison_input": ["run_A/runtime_record.jsonl", "run_B/runtime_record.jsonl"]},
        "required_run_artifacts": ["scenario_resolved_manifest.json", "p1_10_context.json", "preflight_evidence.json", "runtime_record.jsonl", "sim_clock_timing.jsonl", "rt_frame_timing.jsonl", "reader_stats.json", "process_facts.json", "orphan_inventory.json", "mujoco_raw.log", "ros2_launch_raw.log", "orchestrator_raw.log"],
        "comparison_artifacts": ["canonical_identity_input.json", "canonical_identity_output.json", "diff_report.json", "saved_record_comparison_report.md"],
        "tooling": {"comparison_cli": "scripts/p1_10_stage_a_saved_record_compare.py", "stage_a_manifest": STAGE_MANIFEST_REL,
                    "operator_runbook": operator_runbook},
    }


def write_pair_manifest(
    path: Optional[Path] = None, *, pair_dir_rel: str = PAIR_DIR_REL,
    pair_id: str = PAIR_ID, operator_runbook: str = "docs/evidence/P1-10/stage_a_operator_runbook_20260907.md",
) -> Dict[str, Any]:
    pair_dir, pair_dir_rel, pair_id = _pair_locator(pair_dir_rel, pair_id)
    path = Path(path) if path is not None else pair_dir / "pair_manifest.json"
    if path.resolve().parent != pair_dir:
        raise ValueError("pair manifest path must be the direct child of the selected pair directory")
    if path.exists() or path.is_symlink():
        raise FileExistsError(f"refusing to overwrite existing pair manifest: {path}")
    value = build_pair_manifest(
        pair_dir_rel=pair_dir_rel, pair_id=pair_id, operator_runbook=operator_runbook)
    path.parent.mkdir(parents=True, exist_ok=False)
    path.write_bytes(canonical_bytes(value) + b"\n")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true")
    parser.add_argument("--pair-dir", default=PAIR_DIR_REL,
                        help="new repository-relative pair directory")
    parser.add_argument("--pair-id", default=PAIR_ID)
    parser.add_argument("--operator-runbook",
                        default="docs/evidence/P1-10/stage_a_operator_runbook_20260907.md")
    args = parser.parse_args()
    if not args.write:
        parser.error("--write is required")
    pair_dir, pair_dir_rel, pair_id = _pair_locator(args.pair_dir, args.pair_id)
    value = write_pair_manifest(
        pair_dir_rel=pair_dir_rel, pair_id=pair_id,
        operator_runbook=args.operator_runbook)
    path = pair_dir / "pair_manifest.json"
    print(json.dumps({"pair_dir": str(pair_dir), "pair_id": value["pair_id"],
                      "manifest_sha256": sha256_file(path), "status": value["status_at_freeze"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
