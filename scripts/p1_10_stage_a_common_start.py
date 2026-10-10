#!/usr/bin/env python3
"""Offline Stage-A common-start execution identity.

This manifest is intentionally separate from both the accepted P1-08 baseline
and the earlier ungated Stage-A identity.  It is a pre-run contract only; it
does not create a pair or launch a process.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import stat
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

REPO = Path(__file__).resolve().parents[1]
EVIDENCE_DIR = REPO / "docs/evidence/P1-10"
MANIFEST_PATH = EVIDENCE_DIR / "stage_a_common_start_execution_manifest_20260907.json"
EXECUTABLE = REPO / "unitree_mujoco/simulate/build2/unitree_mujoco"
STAGE_SCHEMA = "abs-go2-p1-10-stage-a-common-start-execution-manifest/v1"
IDENTITY_SCHEMA = "abs-go2-p1-10-stage-a-common-start-execution/v1"
EXECUTION_KIND = "P1-10_STAGE_A_FLAT_REPLAY_COMMON_START"
EXECUTION_KEY = "P1-10-STAGE-A-FLAT-COMMON-START"
MANIFEST_ID = "P1-10-STAGE-A-COMMON-START-flat_goal_forward-stabilized"
STATUS = "FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW"
CANONICAL_ENCODING = "json.dumps(value, sort_keys=True, separators=(',', ':')).encode('utf-8'); sha256"


class CommonStartManifestError(ValueError):
    pass


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False, allow_nan=False).encode("utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _repo_rel(path: Path) -> str:
    return str(Path(path).resolve().relative_to(REPO.resolve()))


def _regular(path: Path, label: str) -> None:
    try:
        info = path.lstat()
    except OSError as exc:
        raise CommonStartManifestError(f"{label}: lstat failed: {exc}") from exc
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
        raise CommonStartManifestError(f"{label}: regular non-symlink file required")
    if not os.access(path, os.R_OK):
        raise CommonStartManifestError(f"{label}: unreadable")


def _artifact(path: Path, logical_path: str, role: str) -> Dict[str, Any]:
    _regular(path, role)
    return {"path": logical_path, "role": role,
            "sha256": sha256_file(path), "bytes": path.stat().st_size}


def _common_source_identity(base: Mapping[str, Any]) -> Dict[str, Any]:
    sources = list(base["source_build_identity"]["sources"])
    present = {item["path"] for item in sources}
    additional = (
        ("common/abs_stage_a_common_start_contract.h", "common_start_contract"),
        ("scripts/p1_10_common_start.py", "common_start_python_contract"),
        ("scripts/p1_08_baseline_capture.py", "common_start_harness"),
        ("scripts/run_record.py", "common_start_record_contract"),
        ("scripts/p1_10_scenario_suite.py", "common_start_scenario_resolver"),
        ("scripts/p1_10_stage_a_execution.py", "common_start_parent_identity"),
        ("quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp", "common_start_rl_producer"),
        ("quadruped_ros2_control_humble/controllers/rl_quadruped_controller/include/rl_quadruped_controller/FSM/StateRL.h", "common_start_rl_header"),
        ("quadruped_ros2_control_humble/controllers/rl_quadruped_controller/CMakeLists.txt", "controller_build_contract"),
    )
    for logical_path, role in additional:
        if logical_path not in present:
            sources.append(_artifact(REPO / logical_path, logical_path, role))
    sources.sort(key=lambda item: item["path"])
    result = copy.deepcopy(base["source_build_identity"])
    result["sources"] = sources
    result["common_start_rebuild_required"] = True
    result["common_start_build_scope"] = {
        "simulator_target": "unitree_mujoco",
        "controller_target": "rl_quadruped_controller",
        "no_runtime_execution": True,
    }
    return result


def _common_contract() -> Dict[str, Any]:
    return {
        "schema": "abs-go2-p1-10-stage-a-common-start/v1",
        "version": 1,
        "shared_memory_name": "/mujoco_p1_10_stage_a_common_start",
        "layout_bytes": 264,
        "gate_state_domain": {
            "WAITING_FOR_RELEASE": 1,
            "RELEASED": 2,
            "FAILED": 3,
            "INVALID": 0,
        },
        "release": {
            "writer": "scripts/p1_10_common_start.py::GateClient",
            "one_shot": True,
            "capture_id": "harness-generated only; no CLI input or reuse",
            "preconditions": ["initial_ready", "controller_active", "rl_active", "declared_goal_command"],
            "timeout_or_missing_release": "fail closed",
        },
        "physics_scope": {
            "writer": "unitree_mujoco/simulate/src/main.cc::PhysicsLoop",
            "before_release": "mj_forward only; zero mj_step",
            "after_release": "the first PhysicsLoop mj_step records first_physics_step",
            "ui_step_forward": "excluded from formal capture authority",
        },
    }


def _anchor_contract() -> Dict[str, Any]:
    return {
        "schema": "abs-go2-p1-10-stage-a-common-start/v1",
        "version": 1,
        "producer": [
            "quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp::StateRL::enter",
            "quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp::StateRL::writeRtFrame",
            "unitree_mujoco/simulate/src/main.cc::PhysicsLoop",
        ],
        "required_fields": [
            "capture_id", "initial_qpos_sha256", "gate_state",
            "release_monotonic_ns", "rl_session_id", "rl_step",
            "rl_frame_sequence", "rl_monotonic_ns",
            "first_runtime_frame_sequence", "first_runtime_frame_rl_step",
            "first_runtime_frame_monotonic_ns", "first_physics_step",
            "first_physics_monotonic_ns", "first_sim_time", "flags",
        ],
        "record_projection": {
            "terminal": "start_anchor",
            "first_live_frame": "start_anchor_ref",
            "process_facts": "common_start_anchor",
        },
    }


def build_identity_input() -> Dict[str, Any]:
    import sys
    sys.path.insert(0, str(REPO / "scripts"))
    from p1_10_stage_a_execution import build_identity_input as build_ungated_identity  # noqa: E402

    plugin_path = REPO / "quadruped_ros2_control_humble/install/rl_quadruped_controller/lib/rl_quadruped_controller/librl_quadruped_controller.so"
    base = build_ungated_identity({
        "path": "quadruped_ros2_control_humble/install/rl_quadruped_controller/lib/rl_quadruped_controller/librl_quadruped_controller.so",
        "sha256": sha256_file(plugin_path),
    })
    identity = copy.deepcopy(base)
    identity["execution_kind"] = EXECUTION_KIND
    identity["execution_identity_key"] = EXECUTION_KEY
    identity["source_build_identity"] = _common_source_identity(base)
    identity["common_start_contract"] = _common_contract()
    identity["anchor_contract"] = _anchor_contract()
    identity["fixed_binding"]["common_start_enabled"] = True
    identity["fixed_binding"]["physics_steps_before_release"] = 0
    identity["runtime_record_contract"] = {
        "record_format_version": 2,
        "source": "scripts/run_record.py::RunRecordRecorder",
        "common_start_required": True,
        "anchor_schema": "abs-go2-p1-10-stage-a-common-start/v1",
        "legacy_records": "readable when common_start_required is absent; never relabeled as gated",
        "collision_instrumentation": "present in binary but flat pair has no obstacle collision evidence",
    }
    identity["authority_scope"] = {
        "launch": "p1_08_baseline_capture.py controlled launch with common-start manifest",
        "physics": "main.cc PhysicsLoop only",
        "ui": "reset/keyframe/step-forward/teleop prohibited",
        "collision": "collision v2 remains non-obstacle evidence for flat Stage-A",
        "anchor": "producer-side RL-enter plus first PhysicsLoop step and first runtime frame",
    }
    identity["canonical_encoding"] = CANONICAL_ENCODING
    return identity


def build_manifest() -> Dict[str, Any]:
    identity = build_identity_input()
    return {
        "schema_version": STAGE_SCHEMA,
        "manifest_id": MANIFEST_ID,
        "execution_kind": EXECUTION_KIND,
        "status": STATUS,
        "generated_by": {
            "script": "scripts/p1_10_stage_a_common_start.py",
            "script_sha256": sha256_file(Path(__file__)),
        },
        "runtime_locator": {
            "executable": str(EXECUTABLE.resolve()),
            "executable_path_is_locator_only": True,
        },
        "identity_schema": IDENTITY_SCHEMA,
        "identity_input": identity,
        "identity_sha256": hashlib.sha256(canonical_bytes(identity)).hexdigest(),
    }


def validate_manifest(path: Path, *, expected_executable: Optional[str] = None) -> Dict[str, Any]:
    path = Path(path)
    _regular(path, "common-start manifest")
    try:
        path.resolve().relative_to(EVIDENCE_DIR.resolve())
    except ValueError as exc:
        raise CommonStartManifestError("manifest must be under docs/evidence/P1-10") from exc
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise CommonStartManifestError(f"manifest parse failed: {exc}") from exc
    if not isinstance(manifest, dict):
        raise CommonStartManifestError("manifest root must be an object")
    required = ("schema_version", "manifest_id", "execution_kind", "status", "generated_by",
                "runtime_locator", "identity_schema", "identity_input", "identity_sha256")
    if any(key not in manifest for key in required):
        raise CommonStartManifestError("common-start manifest missing required field")
    if (manifest["schema_version"] != STAGE_SCHEMA or manifest["identity_schema"] != IDENTITY_SCHEMA or
            manifest["manifest_id"] != MANIFEST_ID or manifest["execution_kind"] != EXECUTION_KIND or
            manifest["status"] != STATUS):
        raise CommonStartManifestError("common-start manifest identity/status mismatch")
    generated = manifest["generated_by"]
    if (not isinstance(generated, dict) or generated.get("script") != "scripts/p1_10_stage_a_common_start.py" or
            generated.get("script_sha256") != sha256_file(Path(__file__))):
        raise CommonStartManifestError("common-start generator identity drift")
    if manifest["identity_sha256"] != hashlib.sha256(canonical_bytes(manifest["identity_input"])).hexdigest():
        raise CommonStartManifestError("common-start canonical identity hash mismatch")
    locator = manifest["runtime_locator"].get("executable") if isinstance(manifest["runtime_locator"], dict) else None
    if not isinstance(locator, str) or not Path(locator).is_absolute() or Path(locator).resolve() != EXECUTABLE.resolve():
        raise CommonStartManifestError("common-start executable locator substitution")
    if expected_executable is not None and Path(expected_executable).resolve() != Path(locator).resolve():
        raise CommonStartManifestError("actual executable does not match common-start identity")
    identity = manifest["identity_input"]
    if not isinstance(identity, Mapping) or identity.get("execution_kind") != EXECUTION_KIND or identity.get("execution_identity_key") != EXECUTION_KEY:
        raise CommonStartManifestError("wrong common-start identity input")
    for key in ("common_start_contract", "anchor_contract"):
        if key not in identity:
            raise CommonStartManifestError(f"identity missing {key}")
    fresh = build_identity_input()
    if identity != fresh:
        raise CommonStartManifestError("common-start identity input drift")
    return {
        "manifest": manifest,
        "manifest_path": _repo_rel(path),
        "manifest_sha256": sha256_file(path),
        "identity_sha256": manifest["identity_sha256"],
        "identity_input": identity,
        "executable": locator,
    }


def validate_resolved_context(context: Mapping[str, Any], validation: Mapping[str, Any]) -> None:
    """Require the harness-resolved context to equal this execution identity."""
    identity = validation["identity_input"]
    if context.get("scenario_id") != identity["scenario"]["id"] or context.get("scenario_sha256") != identity["scenario"]["sha256"]:
        raise CommonStartManifestError("resolved scenario binding does not match common-start identity")
    if context.get("suite_path") != identity["scenario"]["suite_path"] or context.get("suite_sha256") != identity["scenario"]["suite_sha256"]:
        raise CommonStartManifestError("resolved suite binding does not match common-start identity")
    scene = context.get("scene", {})
    frozen_scene = identity["scene"]
    for key in ("root_xml", "root_xml_sha256", "model_closure_sha256", "runtime_model_fingerprint"):
        if scene.get(key) != frozen_scene.get(key):
            raise CommonStartManifestError(f"resolved scene binding drift: {key}")
    initial = context.get("initial_state", {})
    frozen_initial = identity["initial_state"]
    for key in ("qpos", "qpos_sha256", "binding_sha256"):
        if initial.get(key) != frozen_initial.get(key):
            raise CommonStartManifestError(f"resolved initial-state binding drift: {key}")
    if context.get("goal") != identity["goal"]["goal"] or context.get("goal_injection") != identity["goal"]["injection"]:
        raise CommonStartManifestError("resolved goal binding drift")
    if context.get("pairing", {}).get("variant") != "stabilized" or context.get("run_window_s") != 25.0:
        raise CommonStartManifestError("resolved fixed variant/window drift")
    if context.get("variant_binding") != identity["variant_binding"]:
        raise CommonStartManifestError("resolved consumed controller binding drift")
    if context.get("initial_state_source", {}).get("kind") != "scene_default":
        raise CommonStartManifestError("resolved initial-state source drift")
    if context.get("initial_state_source", {}).get("reset_source") != "mj_makeData:qpos0; no keyframe reset":
        raise CommonStartManifestError("resolved initial-state reset source drift")


def write_manifest(path: Path = MANIFEST_PATH) -> Dict[str, Any]:
    manifest = build_manifest()
    path = Path(path)
    if path.exists():
        raise CommonStartManifestError(f"refusing to overwrite existing manifest: {path}")
    path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False, sort_keys=True) + "\n",
                    encoding="utf-8")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", nargs="?", const=str(MANIFEST_PATH))
    parser.add_argument("--validate")
    args = parser.parse_args()
    if args.write and args.validate:
        parser.error("--write and --validate are mutually exclusive")
    if args.write:
        manifest = write_manifest(Path(args.write))
        print(json.dumps({"manifest": args.write, "identity_sha256": manifest["identity_sha256"]}, indent=2))
        return 0
    if args.validate:
        result = validate_manifest(Path(args.validate), expected_executable=str(EXECUTABLE))
        print(json.dumps({key: result[key] for key in ("manifest_path", "manifest_sha256", "identity_sha256")}, indent=2))
        return 0
    parser.error("one of --write or --validate is required")


if __name__ == "__main__":
    raise SystemExit(main())
