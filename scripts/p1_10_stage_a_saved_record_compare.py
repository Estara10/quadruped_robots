#!/usr/bin/env python3
"""Stage-A saved-record comparator with mandatory preflight provenance.

The legacy comparator remains available for the historical flat closure. This
profile is bound to the fresh current-instrumented Stage-A pair and requires
preflight_evidence.json in both runs in addition to the four existing files.
It delegates projection/diff semantics to the reviewed comparator and never
opens live shared memory.
"""

from __future__ import annotations

import argparse
import json
import stat
import sys
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))

import p1_10_saved_record_compare as base  # noqa: E402
import p1_10_stage_a_execution as stage  # noqa: E402

PAIR_DIR_REL = "docs/evidence/P1-10/replay_pair_20260907_stage_a_current_instrumented"
PAIR_ID = "P1-10-STAGE-A-REPLAY-20260907-flat_goal_forward-stabilized-current-instrumented"
STAGE_MANIFEST_REL = "docs/evidence/P1-10/stage_a_execution_manifest_20260905.json"
REQUIRED_RUN_FILES = ("preflight_evidence.json",) + tuple(base.REQUIRED_RUN_FILES)


def _configure_base(pair_id: str, pair_dir_rel: str) -> None:
    # This module is a separate CLI process/profile; changing these module
    # globals cannot weaken the legacy comparator when it is invoked directly.
    base.REPO = REPO
    base.EXPECTED_PAIR_ID = pair_id
    base.EXPECTED_EVIDENCE_DIR = pair_dir_rel
    base.EXPECTED_PAIR_DIR = (REPO / pair_dir_rel).resolve()
    base.REQUIRED_RUN_FILES = REQUIRED_RUN_FILES


def _require(value: Any, label: str) -> Any:
    if value is None or value == "" or value == "UNKNOWN":
        raise base.ContractError(f"{label}: missing/None/UNKNOWN")
    return value


def _validate_preflight(value: Mapping[str, Any], pair: Mapping[str, Any],
                        pair_binding: Mapping[str, Any], label: str) -> None:
    checks = value.get("checks")
    if not isinstance(checks, Mapping):
        raise base.ContractError(f"{label}.preflight_evidence.checks: missing")
    manifest_check = checks.get("manifest")
    if not isinstance(manifest_check, Mapping) or manifest_check.get("stage_a") is not True:
        raise base.ContractError(f"{label}.preflight_evidence: Stage-A manifest check missing")
    frozen = pair.get("stage_a_execution_manifest")
    if not isinstance(frozen, Mapping):
        raise base.ContractError("pair manifest: Stage-A execution manifest binding missing")
    if manifest_check.get("pair_id") != pair["pair_id"]:
        raise base.ContractError(f"{label}.preflight_evidence: pair identity mismatch")
    expected_pair_path = (REPO / pair["evidence_dir"] / "pair_manifest.json").resolve()
    if manifest_check.get("pair_manifest_path") != str(expected_pair_path):
        raise base.ContractError(f"{label}.preflight_evidence: pair manifest path mismatch")
    if manifest_check.get("pair_manifest_sha256") != base.sha256_file(expected_pair_path):
        raise base.ContractError(f"{label}.preflight_evidence: pair manifest hash mismatch")
    if manifest_check.get("manifest_path") != str((REPO / frozen["path"]).resolve()):
        raise base.ContractError(f"{label}.preflight_evidence: manifest path mismatch")
    if manifest_check.get("manifest_sha256") != frozen.get("sha256"):
        raise base.ContractError(f"{label}.preflight_evidence: manifest hash mismatch")
    if manifest_check.get("identity_sha256") != frozen.get("identity_sha256"):
        raise base.ContractError(f"{label}.preflight_evidence: identity hash mismatch")
    if manifest_check.get("binary_sha256") != pair["runtime_identity"]["mujoco_executable"]["sha256"]:
        raise base.ContractError(f"{label}.preflight_evidence: executable hash mismatch")
    if type(manifest_check.get("binary_bytes")) is not int or manifest_check.get("binary_bytes") != pair["runtime_identity"]["mujoco_executable"]["bytes"]:
        raise base.ContractError(f"{label}.preflight_evidence: executable size mismatch")
    if manifest_check.get("scene_root_sha256") != pair["scenario_contract"]["scene_root_sha256"]:
        raise base.ContractError(f"{label}.preflight_evidence: scene root mismatch")
    if manifest_check.get("closure_sha256") != pair["scenario_contract"]["model_closure_sha256"]:
        raise base.ContractError(f"{label}.preflight_evidence: closure mismatch")
    if manifest_check.get("config_plugin") != pair["runtime_identity"]["config_plugin_records"]:
        raise base.ContractError(f"{label}.preflight_evidence: config/plugin mismatch")
    contract = pair["scenario_contract"]
    expected_fixed_binding = {
        "variant": pair["variant"]["label"],
        "switching_mode": pair["variant"]["switching_mode"],
        "root_seed": pair["seed"]["root_seed"],
        "window_s": contract["run_window_s"],
        "initial_state_source": contract["initial_state_source"],
        "initial_state_reset_source": contract["initial_state_reset_source"],
        "goal_world_xy_m": contract["goal_world_xy_m"],
    }
    if manifest_check.get("fixed_binding") != expected_fixed_binding:
        raise base.ContractError(f"{label}.preflight_evidence: fixed binding mismatch")
    if manifest_check.get("initial_state_qpos_sha256") != pair["scenario_contract"]["initial_state_qpos_sha256"]:
        raise base.ContractError(f"{label}.preflight_evidence: qpos binding mismatch")
    context = value.get("scenario_context")
    if not isinstance(context, Mapping):
        raise base.ContractError(f"{label}.preflight_evidence.scenario_context: missing")
    base.validate_context(context, pair_binding, f"{label}.preflight_evidence.scenario_context", require_full_binding=True)


_BASE_LOAD_RUN = base._load_run


def _load_run(pair_dir: Path, run_dir: Path, pair_binding: Mapping[str, Any], label: str,
              pair: Mapping[str, Any] | None = None) -> Dict[str, Any]:
    loaded = _BASE_LOAD_RUN(pair_dir, run_dir, pair_binding, label)
    if pair is None:
        raise base.ContractError("Stage-A pair manifest unavailable for preflight validation")
    preflight = base._load_json(loaded["paths"]["preflight_evidence.json"])
    _validate_preflight(preflight, pair, pair_binding, label)
    loaded["preflight_evidence"] = preflight
    return loaded


def _stage_binding(stage_manifest: Mapping[str, Any]) -> Dict[str, Any]:
    identity = stage_manifest["identity_input"]
    scenario = identity["scenario"]
    scene = identity["scene"]
    fixed = identity["fixed_binding"]
    initial = identity["initial_state"]
    baseline = identity["baseline_parent"]
    variant = identity["variant_binding"]
    return {
        "scenario_id": scenario["id"],
        "scenario_sha256": scenario["sha256"],
        "suite_manifest_sha256": scenario["suite_sha256"],
        "scene_root_sha256": scene["root_xml_sha256"],
        "model_closure_sha256": scene["model_closure_sha256"],
        "root_seed": fixed["root_seed"],
        "variant": fixed["variant"],
        "switching_mode": fixed["switching_mode"],
        "variant_binding_sha256": variant["binding_sha256"],
        "baseline_manifest_sha256": baseline["manifest_sha256"],
        "baseline_identity_document_sha256": baseline["identity_file_sha256"],
        "canonical_baseline_identity": baseline["canonical_identity_sha256"],
        "initial_state_source": fixed["initial_state_source"],
        "initial_state_reset_source": fixed["initial_state_reset_source"],
        "initial_state_qpos_sha256": initial["qpos_sha256"],
        "initial_state_binding_sha256": initial["binding_sha256"],
        "run_window_s": fixed["window_s"],
    }


def _validate_stage_pair(pair: Mapping[str, Any], pair_path: Path) -> None:
    if pair.get("pair_id") != PAIR_ID or pair.get("evidence_dir") != PAIR_DIR_REL:
        raise base.ContractError("Stage-A pair is not the current fresh pair")
    frozen = pair.get("stage_a_execution_manifest")
    if not isinstance(frozen, Mapping) or frozen.get("path") != STAGE_MANIFEST_REL:
        raise base.ContractError("Stage-A manifest binding malformed")
    stage_path = REPO / STAGE_MANIFEST_REL
    if not stage_path.is_file() or stage.sha256_file(stage_path) != frozen["sha256"]:
        raise base.ContractError("Stage-A manifest file hash mismatch")
    if frozen.get("status") != "FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW":
        raise base.ContractError("Stage-A manifest status is not pending independent review")
    stage_validation = stage.validate_manifest(stage_path, expected_executable=str(stage.EXECUTABLE))
    expected_binding = _stage_binding(stage_validation["manifest"])
    stage_identity = stage_validation["manifest"]["identity_input"]
    if pair.get("capture_identity_contract") != stage_identity.get("capture_identity_contract"):
        raise base.ContractError("pair capture-ID contract does not match Stage-A identity")
    actual_binding = base._binding_from_pair(pair)
    if actual_binding != expected_binding:
        raise base.ContractError("pair manifest binding does not exactly match Stage-A identity")
    # The legacy comparator has a reviewed flat binding table.  This separate
    # profile replaces that table with the frozen Stage-A manifest binding,
    # after proving the pair itself matches the manifest.  The legacy CLI is
    # a separate process and is unaffected.
    base.EXPECTED_BINDING = expected_binding
    errors = base.validate_pair_manifest(pair)
    if errors:
        raise base.ContractError("pair manifest rejected: " + "; ".join(errors))
    if pair_path.resolve() != (REPO / PAIR_DIR_REL / "pair_manifest.json").resolve():
        raise base.ContractError("pair manifest path substitution")
    if pair.get("status_at_freeze") != "FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW":
        raise base.ContractError("Stage-A pair is not pending independent review")


def compare_pair_dir(pair_dir: Path) -> Dict[str, Any]:
    pair_dir = Path(pair_dir)
    try:
        info = pair_dir.lstat()
    except OSError as exc:
        raise base.ContractError(f"pair-dir cannot be lstat'ed: {exc}") from exc
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
        raise base.ContractError("pair-dir must be a real directory, not a symlink")
    pair_dir_resolved = pair_dir.resolve(strict=True)
    if pair_dir_resolved != (REPO / PAIR_DIR_REL).resolve():
        raise base.ContractError("pair-dir is not the new frozen Stage-A pair directory")
    pair_path = pair_dir_resolved / "pair_manifest.json"
    pair = base._load_json(pair_path)
    if pair.get("pair_id") != PAIR_ID or pair.get("evidence_dir") != PAIR_DIR_REL:
        raise base.ContractError("pair manifest locator does not match the new frozen pair")
    _configure_base(pair["pair_id"], pair["evidence_dir"])
    _validate_stage_pair(pair, pair_path)
    binding = base._binding_from_pair(pair)
    run_dirs = base._run_dirs(pair_dir_resolved, pair)
    run_a = _load_run(pair_dir_resolved, run_dirs["run_a"], binding, "run_a", pair)
    run_b = _load_run(pair_dir_resolved, run_dirs["run_b"], binding, "run_b", pair)
    if run_a["binding"] != run_b["binding"]:
        raise base.ContractError("run_A/run_B frozen binding mismatch")
    # Reuse the reviewed deterministic projection and report implementation.
    projection_a = base._project_record(run_a["record_data"])
    projection_b = base._project_record(run_b["record_data"])
    differences = []
    base._diff_values(projection_a, projection_b, "$", differences)
    canonical_projection = {"binding": binding, "run_a": projection_a, "run_b": projection_b}
    canonical_hash = base.sha256_bytes(base.canonical_json(canonical_projection).encode("utf-8"))
    result = {
        "schema": base.SCHEMA, "status": "PASS" if not differences else "FAIL", "pair_id": pair["pair_id"],
        "pair_manifest_sha256": base.sha256_file(pair_path), "binding": binding,
        "comparison_rules_schema": base.COMPARISON_RULES_VERSION,
        "record_source": "saved runtime_record.jsonl only; never live shared memory",
        "canonical_projection_sha256": canonical_hash, "exact_match": not differences,
        "difference_count": len(differences), "differences": differences,
        "numeric_rule": "exact canonical JSON equality; any nonzero delta fails",
        "numeric_diagnostics": base._numeric_diagnostics(projection_a, projection_b),
        "run_a": {"directory": run_a["directory"], "record_sha256": base.sha256_file(run_a["paths"]["runtime_record.jsonl"])},
        "run_b": {"directory": run_b["directory"], "record_sha256": base.sha256_file(run_b["paths"]["runtime_record.jsonl"])},
        "canonical_input": {"schema": base.SCHEMA, "comparison_rules_schema": base.COMPARISON_RULES_VERSION,
                            "binding": binding, "run_a_projection": projection_a, "run_b_projection": projection_b},
    }
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pair-dir", required=True)
    args = parser.parse_args(argv)
    try:
        result = compare_pair_dir(Path(args.pair_dir))
        base.write_comparison_outputs(result, Path(args.pair_dir))
    except Exception as exc:  # fail closed
        print(f"REJECT: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"status": result["status"], "pair_id": result["pair_id"],
                      "canonical_projection_sha256": result["canonical_projection_sha256"],
                      "difference_count": result["difference_count"]}, sort_keys=True))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
