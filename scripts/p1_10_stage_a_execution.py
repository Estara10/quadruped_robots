#!/usr/bin/env python3
"""Independent current-instrumented Stage-A flat replay identity.

This is an offline identity/preflight contract.  It deliberately reuses the
accepted flat scenario resolution while binding the *current* instrumented
executable as a separate execution artifact.  No capture ID is part of the
canonical identity.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import stat
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional

REPO = Path(__file__).resolve().parents[1]
EVIDENCE_DIR = REPO / "docs/evidence/P1-10"
MANIFEST_PATH = EVIDENCE_DIR / "stage_a_execution_manifest_20260905.json"
PAIR_DIR_REL = "replay_pair_20260905_stage_a_current_instrumented"
PAIR_ID = "P1-10-STAGE-A-REPLAY-20260905-flat_goal_forward-stabilized-current-instrumented"
STAGE_SCHEMA = "abs-go2-p1-10-stage-a-execution-manifest/v1"
IDENTITY_SCHEMA = "abs-go2-p1-10-stage-a-execution/v1"
STATUS = "FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW"
EXECUTABLE = REPO / "unitree_mujoco/simulate/build2/unitree_mujoco"
SCENARIO = REPO / "scenarios/p1_10/flat_goal_forward.json"
SUITE = REPO / "scenarios/p1_10/scenario_suite_manifest.json"
BASELINE = REPO / "docs/evidence/P1-08/P1-08_baseline_manifest.json"
BASELINE_IDENTITY = REPO / "docs/evidence/P1-08/P1-08_simulation_baseline_identity.json"
SCENE = REPO / "unitree_mujoco/unitree_robots/go2/scene_flat.xml"
LIB_MUJOCO = Path("/home/lidio/Libraries/mujoco-3.3.3/lib/libmujoco.so.3.3.3")
FLAGS = REPO / "unitree_mujoco/simulate/build2/CMakeFiles/unitree_mujoco.dir/flags.make"


class StageAIdentityError(ValueError):
    pass


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False, allow_nan=False).encode("utf-8")


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _repo_rel(path: Path) -> str:
    return str(path.resolve().relative_to(REPO.resolve()))


def _load(path: Path) -> Dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise StageAIdentityError(f"cannot parse {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise StageAIdentityError(f"JSON root is not object: {path}")
    return value


def _regular(path: Path, label: str) -> None:
    try:
        info = path.lstat()
    except OSError as exc:
        raise StageAIdentityError(f"{label}: lstat failed: {exc}") from exc
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
        raise StageAIdentityError(f"{label}: non-symlink regular file required")
    if not os.access(path, os.R_OK):
        raise StageAIdentityError(f"{label}: unreadable")


def _artifact(path: Path, logical: str, role: str) -> Dict[str, Any]:
    _regular(path, role)
    return {"path": logical, "role": role, "sha256": sha256_file(path),
            "bytes": path.stat().st_size}


def _closure() -> Dict[str, Any]:
    import sys
    sys.path.insert(0, str(REPO / "scripts"))
    from build_p1_08_manifest import resolve_closure  # noqa: E402
    _regular(SCENE, "flat scene")
    result = resolve_closure(SCENE)
    if result.get("failures"):
        raise StageAIdentityError("flat closure invalid: " + "; ".join(result["failures"]))
    files = []
    for item in sorted(result["xml_files"] + result["asset_files"], key=lambda x: x["path"]):
        path = Path(item["path"])
        _regular(path, "flat closure file")
        files.append({"path": _repo_rel(path), "role": item["role"],
                      "sha256": sha256_file(path), "bytes": path.stat().st_size})
    return {"root_xml": _repo_rel(SCENE), "root_xml_sha256": sha256_file(SCENE),
            "model_closure_sha256": result["closure_sha256"], "files": files}


def _runtime_artifacts() -> list:
    files = [
        ("unitree_mujoco/simulate/build2/unitree_mujoco", "stage_a_mujoco_executable", EXECUTABLE),
        ("unitree_mujoco/simulate/config.yaml", "mujoco_simulate_config", REPO / "unitree_mujoco/simulate/config.yaml"),
        ("quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml", "robot_control_config", REPO / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml"),
        ("quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml", "abs_controller_config", REPO / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml"),
        ("quadruped_ros2_control_humble/controllers/rl_quadruped_controller/launch/mujoco.launch.py", "mujoco_launch", REPO / "quadruped_ros2_control_humble/controllers/rl_quadruped_controller/launch/mujoco.launch.py"),
        ("quadruped_ros2_control_humble/install/rl_quadruped_controller/lib/rl_quadruped_controller/librl_quadruped_controller.so", "controller_plugin", REPO / "quadruped_ros2_control_humble/install/rl_quadruped_controller/lib/rl_quadruped_controller/librl_quadruped_controller.so"),
        ("quadruped_ros2_control_humble/install/hardware_unitree_mujoco/lib/libhardware_unitree_mujoco.so", "hardware_plugin", REPO / "quadruped_ros2_control_humble/install/hardware_unitree_mujoco/lib/libhardware_unitree_mujoco.so"),
        ("external/mujoco-3.3.3/lib/libmujoco.so.3.3.3", "libmujoco_shared", LIB_MUJOCO),
    ]
    return [_artifact(path, logical, role) for logical, role, path in files]


def _source_build_identity() -> Dict[str, Any]:
    source_paths = [
        "unitree_mujoco/simulate/src/main.cc",
        "unitree_mujoco/simulate/src/obstacle_collision_authority.h",
        "common/abs_collision_contract.h",
        "common/abs_collision_model_fingerprint.h",
        "unitree_mujoco/simulate/CMakeLists.txt",
    ]
    sources = [_artifact(REPO / path, path, "stage_a_source") for path in source_paths]
    flags = _artifact(FLAGS, "unitree_mujoco/simulate/build2/CMakeFiles/unitree_mujoco.dir/flags.make", "stage_a_build_flags")
    return {"sources": sources, "build": {"system": "CMake", "target": "unitree_mujoco", "build_type": "Release", "flags": flags}}


def build_identity_input(controller_plugin_override: Optional[Mapping[str, Any]] = None) -> Dict[str, Any]:
    import sys
    sys.path.insert(0, str(REPO / "scripts"))
    from p1_10_scenario_suite import extract_runtime_initial_state  # noqa: E402
    from p1_10_scenario_suite import resolve_scenario  # noqa: E402

    resolved = resolve_scenario(
        "flat_goal_forward", 20260902, "stabilized",
        controller_plugin_override=controller_plugin_override)
    closure = _closure()
    actual = extract_runtime_initial_state(SCENE)
    baseline = _load(BASELINE)
    baseline_identity = _load(BASELINE_IDENTITY)
    artifacts = _runtime_artifacts()
    accepted = next(item["sha256"] for item in baseline["binaries"] if item["role"] == "mujoco_executable")
    if artifacts[0]["sha256"] == accepted:
        raise StageAIdentityError("current executable unexpectedly equals accepted P1-08 executable")
    return {
        "execution_kind": "P1-10_STAGE_A_FLAT_REPLAY_CURRENT_INSTRUMENTED",
        "execution_identity_key": "P1-10-STAGE-A-FLAT-CURRENT-INSTRUMENTED",
        "scenario": {"id": resolved["scenario_id"], "path": _repo_rel(SCENARIO), "sha256": sha256_file(SCENARIO),
                      "suite_path": _repo_rel(SUITE), "suite_sha256": sha256_file(SUITE)},
        "scene": {**closure, "launch_arg": "scene_flat.xml", "runtime_model_fingerprint": actual["collision_model_fingerprint"],
                  "runtime_model_fingerprint_schema": actual["collision_model_fingerprint_schema"]},
        "initial_state": {"source": resolved["initial_state_source"], "qpos": actual["qpos"],
                          "qpos_sha256": actual["qpos_sha256"], "binding_sha256": resolved["initial_state"]["binding_sha256"]},
        "goal": {"goal": resolved["goal"], "injection": resolved["goal_injection"]},
        "variant_binding": resolved["variant_binding"],
        "fixed_binding": {"variant": "stabilized", "switching_mode": "stabilized_switch", "root_seed": 20260902,
                           "window_s": 25.0, "initial_state_source": "scene_default",
                           "initial_state_reset_source": "mj_makeData:qpos0; no keyframe reset", "goal_world_xy_m": [7.0, 0.0]},
        "baseline_parent": {"manifest_path": _repo_rel(BASELINE), "manifest_sha256": sha256_file(BASELINE),
                             "identity_path": _repo_rel(BASELINE_IDENTITY), "identity_file_sha256": sha256_file(BASELINE_IDENTITY),
                             "canonical_identity_sha256": baseline_identity["baseline_identity_sha256"],
                             "accepted_executable_sha256": accepted},
        "artifacts": artifacts,
        "source_build_identity": _source_build_identity(),
        "runtime_record_contract": {"record_format_version": 2, "source": "scripts/run_record.py::RunRecordRecorder",
                                     "collision_instrumentation": "present in binary but flat pair has no obstacle collision evidence"},
        "capture_identity_contract": {
            "schema": "abs-go2-capture-identity/v1",
            "format": "p1-10-capture-[32 lowercase hex characters]",
            "producer": "scripts/p1_08_baseline_capture.py::create_capture_id",
            "generation": "harness-generated per launch; no CLI input and no reuse",
            "canonical_identity_inclusion": "excluded from Stage-A canonical identity because it is per-run provenance",
        },
        "authority_scope": {"launch": "p1_08_baseline_capture.py controlled launch", "ui": "reset/keyframe/step-forward/teleop prohibited",
                            "collision": "collision v2 may be present but is not interpreted as obstacle evidence for flat pair"},
        "canonical_encoding": "json.dumps(value, sort_keys=True, separators=(',', ':')).encode('utf-8'); sha256",
    }


def build_manifest() -> Dict[str, Any]:
    identity = build_identity_input()
    return {"schema_version": STAGE_SCHEMA, "manifest_id": "P1-10-STAGE-A-EXECUTION-flat_goal_forward-stabilized-current-instrumented",
            "pair_id": PAIR_ID, "pair_dir": PAIR_DIR_REL,
            "status": STATUS, "generated_by": {"script": "scripts/p1_10_stage_a_execution.py",
            "script_sha256": sha256_file(REPO / "scripts/p1_10_stage_a_execution.py")},
            "runtime_locator": {"executable": str(EXECUTABLE.resolve()), "executable_path_is_locator_only": True},
            "identity_schema": IDENTITY_SCHEMA, "identity_input": identity,
            "identity_sha256": hashlib.sha256(canonical_bytes(identity)).hexdigest()}


def validate_manifest(path: Path, *, expected_executable: Optional[str] = None) -> Dict[str, Any]:
    path = Path(path)
    _regular(path, "Stage-A manifest")
    try:
        path.resolve().relative_to(EVIDENCE_DIR.resolve())
    except ValueError as exc:
        raise StageAIdentityError("manifest must be under docs/evidence/P1-10") from exc
    manifest = _load(path)
    required = ("schema_version", "manifest_id", "pair_id", "pair_dir", "status", "generated_by", "runtime_locator", "identity_schema", "identity_input", "identity_sha256")
    if any(key not in manifest for key in required):
        raise StageAIdentityError("manifest missing required field")
    if (manifest["schema_version"] != STAGE_SCHEMA or manifest["identity_schema"] != IDENTITY_SCHEMA
            or manifest["status"] != STATUS or manifest["pair_id"] != PAIR_ID
            or manifest["pair_dir"] != PAIR_DIR_REL):
        raise StageAIdentityError("manifest schema/status mismatch")
    if manifest["generated_by"].get("script") != "scripts/p1_10_stage_a_execution.py" or manifest["generated_by"].get("script_sha256") != sha256_file(REPO / "scripts/p1_10_stage_a_execution.py"):
        raise StageAIdentityError("generator identity drift")
    if manifest["identity_sha256"] != hashlib.sha256(canonical_bytes(manifest["identity_input"])).hexdigest():
        raise StageAIdentityError("canonical identity mismatch")
    locator = manifest["runtime_locator"].get("executable")
    if not isinstance(locator, str) or not Path(locator).is_absolute() or Path(locator).resolve() != EXECUTABLE.resolve():
        raise StageAIdentityError("runtime executable locator substitution")
    if expected_executable is not None and Path(expected_executable).resolve() != Path(locator).resolve():
        raise StageAIdentityError("actual executable does not match Stage-A locator")
    identity = manifest["identity_input"]
    required_identity = (
        "execution_kind", "execution_identity_key", "scenario", "scene",
        "initial_state", "goal", "variant_binding", "fixed_binding",
        "baseline_parent", "artifacts", "source_build_identity",
        "runtime_record_contract", "capture_identity_contract", "authority_scope",
    )
    if not isinstance(identity, Mapping) or any(key not in identity for key in required_identity):
        raise StageAIdentityError("identity input missing required field")
    if (identity.get("execution_kind") != "P1-10_STAGE_A_FLAT_REPLAY_CURRENT_INSTRUMENTED"
            or identity.get("execution_identity_key") != "P1-10-STAGE-A-FLAT-CURRENT-INSTRUMENTED"):
        raise StageAIdentityError("wrong Stage-A execution identity")
    if identity["scenario"] != {"id": "flat_goal_forward", "path": _repo_rel(SCENARIO), "sha256": sha256_file(SCENARIO),
                                "suite_path": _repo_rel(SUITE), "suite_sha256": sha256_file(SUITE)}:
        raise StageAIdentityError("flat scenario/suite binding drift")
    fresh = build_identity_input()
    # Exact comparison forces every derived binding, artifact hash, source hash,
    # qpos, goal, and closure field to remain current and explicit.
    if identity != fresh:
        raise StageAIdentityError("Stage-A identity input drift")
    return {"manifest": manifest, "manifest_sha256": sha256_file(path), "identity_sha256": manifest["identity_sha256"],
            "identity_input": identity, "executable": locator}


def write_manifest(path: Path) -> Dict[str, Any]:
    value = build_manifest()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(canonical_bytes(value) + b"\n")
    return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write")
    parser.add_argument("--validate")
    parser.add_argument("--executable")
    args = parser.parse_args()
    try:
        if args.write:
            value = write_manifest(Path(args.write))
            print(json.dumps({"manifest_sha256": sha256_file(Path(args.write)), "identity_sha256": value["identity_sha256"], "status": STATUS}, sort_keys=True))
        elif args.validate:
            value = validate_manifest(Path(args.validate), expected_executable=args.executable)
            print(json.dumps({"valid": True, "manifest_sha256": value["manifest_sha256"], "identity_sha256": value["identity_sha256"], "status": STATUS}, sort_keys=True))
        else:
            parser.error("--write or --validate required")
    except (StageAIdentityError, OSError, StopIteration) as exc:
        print(f"STAGE-A IDENTITY REJECTED: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
