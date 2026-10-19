#!/usr/bin/env python3
"""P1-10 Stage-B execution identity and offline preflight contract.

This module binds the first obstacle observational capture to the actual
instrumented executable and to the already reviewed obstacle_test1 scenario.
It is deliberately offline: manifest generation/validation may use the
construction-path initial-state probe, but never starts the simulator, ROS2,
or a controller.

The canonical identity is the SHA-256 of ``identity_input`` serialized as
UTF-8 JSON with sorted keys and compact separators.  Runtime locator paths
are kept outside that input so an absolute filesystem path cannot become part
of the identity.  All file checks use lstat and reject symlinks.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import re
import stat
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple

REPO = Path(__file__).resolve().parents[1]
P1_10_EVIDENCE = REPO / "docs" / "evidence" / "P1-10"
SCENARIO_PATH = REPO / "scenarios" / "p1_10" / "obstacle_test1.json"
CANDIDATE_SUITE_PATH = REPO / "scenarios" / "p1_10" / "obstacle_candidate_suite_manifest.json"
FLAT_SUITE_PATH = REPO / "scenarios" / "p1_10" / "scenario_suite_manifest.json"
BASELINE_MANIFEST = REPO / "docs" / "evidence" / "P1-08" / "P1-08_baseline_manifest.json"
BASELINE_IDENTITY = REPO / "docs" / "evidence" / "P1-08" / "P1-08_simulation_baseline_identity.json"
ROOT_XML = REPO / "unitree_mujoco" / "unitree_robots" / "go2" / "scene_test1.xml"
STAGE_B_EXECUTABLE = REPO / "unitree_mujoco" / "simulate" / "build2" / "unitree_mujoco"
LIB_MUJOCO = Path("/home/lidio/Libraries/mujoco-3.3.3/lib/libmujoco.so.3.3.3")
PROBE_SOURCE = REPO / "unitree_mujoco" / "simulate" / "test" / "p1_10_initial_state_probe.cpp"
PROBE_EXECUTABLE = REPO / "unitree_mujoco" / "simulate" / "build2" / "p1_10_initial_state_probe"
BUILD_FLAGS = REPO / "unitree_mujoco" / "simulate" / "build2" / "CMakeFiles" / "unitree_mujoco.dir" / "flags.make"
STAGE_SCHEMA = "abs-go2-p1-10-stage-b-execution-manifest/v1"
IDENTITY_SCHEMA = "abs-go2-p1-10-stage-b-execution/v1"
CAPTURE_SCHEMA = "abs-go2-capture-identity/v1"
MODEL_FINGERPRINT_SCHEMA = "abs-go2-collision-model-fingerprint/v1"
COLLISION_SCHEMA = "abs-go2-collision-snapshot/v2"
COLLISION_VERSION = 2
COLLISION_BYTES = 392
CAPTURE_ID_FORMAT = r"^p1-10-capture-[0-9a-f]{32}$"
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
CANONICAL_ENCODING = "json.dumps(value, sort_keys=True, separators=(',', ':')).encode('utf-8')"
STATUS = "FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW"

SOURCE_FILES = (
    "unitree_mujoco/simulate/src/main.cc",
    "unitree_mujoco/simulate/src/obstacle_collision_authority.h",
    "common/abs_collision_contract.h",
    "common/abs_collision_model_fingerprint.h",
    "unitree_mujoco/simulate/CMakeLists.txt",
)
CONFIG_FILES = (
    ("unitree_mujoco/simulate/config.yaml", "mujoco_simulate_config"),
    ("quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml", "robot_control_config"),
    ("quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml", "abs_controller_config"),
    ("quadruped_ros2_control_humble/controllers/rl_quadruped_controller/launch/mujoco.launch.py", "mujoco_launch"),
    ("quadruped_ros2_control_humble/install/rl_quadruped_controller/lib/rl_quadruped_controller/librl_quadruped_controller.so", "controller_plugin"),
    ("quadruped_ros2_control_humble/install/hardware_unitree_mujoco/lib/libhardware_unitree_mujoco.so", "hardware_plugin"),
)


class StageBManifestError(ValueError):
    """A missing, malformed, stale, or drifted Stage-B identity."""


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=False, allow_nan=False).encode("utf-8")


def canonical_json(value: Any) -> str:
    return canonical_bytes(value).decode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _repo_relative(path: Path) -> str:
    return str(path.resolve().relative_to(REPO.resolve()))


def _load_json(path: Path) -> Dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise StageBManifestError(f"unreadable JSON: {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise StageBManifestError(f"JSON root is not an object: {path}")
    return payload


def _lstat_regular(path: Path, label: str) -> None:
    try:
        info = path.lstat()
    except OSError as exc:
        raise StageBManifestError(f"{label}: lstat failed: {path}: {exc}") from exc
    if stat.S_ISLNK(info.st_mode):
        raise StageBManifestError(f"{label}: symlink rejected: {path}")
    if not stat.S_ISREG(info.st_mode):
        raise StageBManifestError(f"{label}: not a regular file: {path}")
    if not os.access(path, os.R_OK):
        raise StageBManifestError(f"{label}: not readable: {path}")


def _repo_path(value: str, label: str) -> Path:
    if not isinstance(value, str) or not value or Path(value).is_absolute():
        raise StageBManifestError(f"{label}: expected repository-relative path")
    candidate = (REPO / value).resolve()
    try:
        candidate.relative_to(REPO.resolve())
    except ValueError as exc:
        raise StageBManifestError(f"{label}: path escapes repository: {value}") from exc
    return candidate


def _hash_record(path: Path, logical_path: str, role: str) -> Dict[str, Any]:
    _lstat_regular(path, role)
    return {"path": logical_path, "role": role, "sha256": sha256_file(path),
            "bytes": path.stat().st_size}


def _require(mapping: Mapping[str, Any], keys: Iterable[str], label: str) -> None:
    missing = [key for key in keys if key not in mapping]
    if missing:
        raise StageBManifestError(f"{label}: missing {','.join(missing)}")


def _hash_field(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA256_RE.fullmatch(value) is None:
        raise StageBManifestError(f"{label}: invalid SHA-256")
    return value


def _runtime_artifact_files() -> List[Tuple[str, str, Path]]:
    files: List[Tuple[str, str, Path]] = [("unitree_mujoco/simulate/build2/unitree_mujoco",
                                            "stage_b_mujoco_executable", STAGE_B_EXECUTABLE)]
    files.extend((path, role, REPO / path) for path, role in CONFIG_FILES)
    files.append(("external/mujoco-3.3.3/lib/libmujoco.so.3.3.3",
                  "libmujoco_shared", LIB_MUJOCO))
    return files


def _closure_identity(root: Path) -> Dict[str, Any]:
    # Import lazily so this module remains a small offline identity tool.
    import sys
    sys.path.insert(0, str(REPO / "scripts"))
    from build_p1_08_manifest import resolve_closure  # noqa: E402

    root = root.resolve()
    _lstat_regular(root, "scene root XML")
    closure = resolve_closure(root)
    if closure.get("failures"):
        raise StageBManifestError("scene closure invalid: " + "; ".join(closure["failures"]))
    files = []
    for item in sorted(closure.get("xml_files", []) + closure.get("asset_files", []),
                       key=lambda value: value["path"]):
        path = Path(item["path"])
        _lstat_regular(path, "scene closure file")
        files.append({"path": _repo_relative(path), "role": item["role"],
                      "sha256": sha256_file(path), "bytes": path.stat().st_size})
    return {
        "root_xml": _repo_relative(root),
        "root_xml_sha256": sha256_file(root),
        "model_closure_sha256": closure["closure_sha256"],
        "files": files,
    }


def _probe_initial_state(root: Path) -> Dict[str, Any]:
    import sys
    sys.path.insert(0, str(REPO / "scripts"))
    from p1_10_scenario_suite import extract_runtime_initial_state  # noqa: E402
    try:
        return extract_runtime_initial_state(root)
    except (OSError, ValueError) as exc:
        raise StageBManifestError(f"offline initial-state probe failed: {exc}") from exc


def _candidate() -> Tuple[Dict[str, Any], Dict[str, Any]]:
    candidate = _load_json(SCENARIO_PATH)
    suite = _load_json(CANDIDATE_SUITE_PATH)
    if candidate.get("scenario_id") != "obstacle_test1":
        raise StageBManifestError("candidate scenario identity is not obstacle_test1")
    entries = [entry for entry in suite.get("formal_scenarios", [])
               if isinstance(entry, dict) and entry.get("scenario_id") == "obstacle_test1"]
    if len(entries) != 1:
        raise StageBManifestError("candidate suite has no unique obstacle_test1 entry")
    if entries[0].get("sha256") != sha256_file(SCENARIO_PATH):
        raise StageBManifestError("candidate suite/scenario hash mismatch")
    return candidate, suite


def _source_identity() -> List[Dict[str, Any]]:
    records = []
    for value in SOURCE_FILES:
        records.append(_hash_record(REPO / value, value, "stage_b_source"))
    records.append(_hash_record(PROBE_SOURCE, _repo_relative(PROBE_SOURCE),
                                "initial_state_probe_source"))
    records.append(_hash_record(PROBE_EXECUTABLE, _repo_relative(PROBE_EXECUTABLE),
                                "initial_state_probe_executable"))
    return records


def _build_identity() -> Dict[str, Any]:
    record = _hash_record(
        BUILD_FLAGS,
        "unitree_mujoco/simulate/build2/CMakeFiles/unitree_mujoco.dir/flags.make",
        "stage_b_build_flags",
    )
    return {
        "system": "CMake",
        "target": "unitree_mujoco",
        "build_type": "Release",
        "flags": record,
    }


def _artifact_identity() -> List[Dict[str, Any]]:
    return [_hash_record(path, logical, role)
            for logical, role, path in _runtime_artifact_files()]


def _baseline_parent() -> Dict[str, Any]:
    baseline = _load_json(BASELINE_MANIFEST)
    identity = _load_json(BASELINE_IDENTITY)
    if identity.get("baseline_identity_sha256") != "59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0":
        raise StageBManifestError("accepted P1-08 canonical identity mismatch")
    return {
        "manifest_path": _repo_relative(BASELINE_MANIFEST),
        "manifest_sha256": sha256_file(BASELINE_MANIFEST),
        "identity_path": _repo_relative(BASELINE_IDENTITY),
        "identity_file_sha256": sha256_file(BASELINE_IDENTITY),
        "canonical_identity_sha256": identity["baseline_identity_sha256"],
        "accepted_executable_sha256": next(
            item["sha256"] for item in baseline.get("binaries", [])
            if item.get("role") == "mujoco_executable"),
    }


def build_identity_input() -> Dict[str, Any]:
    candidate, candidate_suite = _candidate()
    closure = _closure_identity(ROOT_XML)
    runtime_initial = _probe_initial_state(ROOT_XML)
    if runtime_initial["collision_model_fingerprint"] != candidate["scene"]["runtime_model_fingerprint"]:
        raise StageBManifestError("candidate runtime model fingerprint differs from current probe")
    if runtime_initial["qpos_sha256"] != candidate["initial_state"]["qpos_sha256"]:
        raise StageBManifestError("candidate initial qpos differs from current probe")
    scenario_sha = sha256_file(SCENARIO_PATH)
    suite_sha = sha256_file(CANDIDATE_SUITE_PATH)
    artifacts = _artifact_identity()
    source_identity = _source_identity()
    baseline = _baseline_parent()
    if artifacts[0]["sha256"] == baseline["accepted_executable_sha256"]:
        raise StageBManifestError("Stage-B executable unexpectedly equals accepted P1-08 executable")
    objects = candidate.get("obstacle_layout", {}).get("objects")
    if not isinstance(objects, list) or not objects:
        raise StageBManifestError("obstacle metadata is missing")

    flat_suite = _load_json(FLAT_SUITE_PATH)
    stabilized = flat_suite.get("variants", {}).get("stabilized", {})
    if not isinstance(stabilized, dict) or stabilized.get("status") != "SUPPORTED":
        raise StageBManifestError("stabilized consumed binding is unavailable")
    variant_binding = copy.deepcopy(stabilized)

    return {
        "execution_kind": "P1-10_STAGE_B_OBSERVATIONAL_RUNTIME",
        "scenario": {
            "scenario_id": "obstacle_test1",
            "scenario_path": _repo_relative(SCENARIO_PATH),
            "scenario_sha256": scenario_sha,
            "candidate_suite_path": _repo_relative(CANDIDATE_SUITE_PATH),
            "candidate_suite_sha256": suite_sha,
            "candidate_suite_schema": candidate_suite.get("schema_version"),
            "candidate_status": candidate.get("status"),
        },
        "scene": {
            "launch_arg": "scene_test1.xml",
            "root_xml": closure["root_xml"],
            "root_xml_sha256": closure["root_xml_sha256"],
            "model_closure_sha256": closure["model_closure_sha256"],
            "closure_files": closure["files"],
            "runtime_model_fingerprint_schema": MODEL_FINGERPRINT_SCHEMA,
            "runtime_model_fingerprint": runtime_initial["collision_model_fingerprint"],
        },
        "obstacle_metadata": {
            "sha256": sha256_bytes(canonical_bytes(objects)),
            "source": "scenario.obstacle_layout.objects",
            "objects": objects,
        },
        "initial_state": {
            "source": candidate["initial_state_source"],
            "qpos": runtime_initial["qpos"],
            "qpos_sha256": runtime_initial["qpos_sha256"],
            "probe_source_sha256": sha256_file(PROBE_SOURCE),
            "probe_executable_sha256": sha256_file(PROBE_EXECUTABLE),
        },
        "goal": {
            "goal": candidate["goal"],
            "injection": candidate["goal_injection"],
        },
        "variant_binding": variant_binding,
        "fixed_binding": {
            "variant": "stabilized",
            "switching_mode": "stabilized_switch",
            "root_seed": 20260902,
            "window_s": 25.0,
            "initial_state_source": "scene_default",
            "initial_state_reset_source": "mj_makeData:qpos0; no keyframe reset",
            "goal_world_xy_m": [7.0, 0.0],
        },
        "baseline_parent": baseline,
        "artifacts": artifacts,
        "source_identity": source_identity,
        "build_identity": _build_identity(),
        "collision_contract": {
            "schema": COLLISION_SCHEMA,
            "version": COLLISION_VERSION,
            "layout_bytes": COLLISION_BYTES,
            "shm_name": "/mujoco_collision_v2",
            "capture_identity_schema": CAPTURE_SCHEMA,
            "model_fingerprint_schema": MODEL_FINGERPRINT_SCHEMA,
        },
        "capture_identity_contract": {
            "schema": CAPTURE_SCHEMA,
            "format": "p1-10-capture-<32 lowercase hex characters>",
            "generated_by": "scripts/p1_08_baseline_capture.py:create_capture_id",
            "source": "harness-generated per capture; no CLI override",
            "required_bindings": ["scenario", "scene", "closure", "runtime_model_fingerprint"],
        },
        "authority_scope": {
            "scope": "main.cc PhysicsLoop two harness-controlled mj_step paths only",
            "publish": "one collision snapshot after each in-scope PhysicsLoop mj_step",
            "ui_step_forward": "excluded from formal capture authority",
        },
        "operator_prohibitions": [
            "no UI reset",
            "no UI keyframe",
            "no UI step-forward",
            "no teleop",
        ],
        "canonical_encoding": CANONICAL_ENCODING,
    }


def build_manifest() -> Dict[str, Any]:
    identity_input = build_identity_input()
    script_path = REPO / "scripts" / "p1_10_stage_b_execution.py"
    return {
        "schema_version": STAGE_SCHEMA,
        "manifest_id": "P1-10-STAGE-B-EXECUTION-obstacle_test1-stabilized",
        "status": STATUS,
        "generated_by": {
            "script": _repo_relative(script_path),
            "script_sha256": sha256_file(script_path),
        },
        "runtime_locator": {
            "executable": str(STAGE_B_EXECUTABLE.resolve()),
            "executable_path_is_locator_only": True,
        },
        "identity_schema": IDENTITY_SCHEMA,
        "identity_input": identity_input,
        "identity_sha256": sha256_bytes(canonical_bytes(identity_input)),
    }


def _actual_path_for_logical(logical: str, runtime_locator: Optional[str] = None) -> Path:
    if logical == "unitree_mujoco/simulate/build2/unitree_mujoco":
        if runtime_locator is None:
            raise StageBManifestError("stage executable locator missing")
        path = Path(runtime_locator)
        if not path.is_absolute():
            raise StageBManifestError("runtime executable locator must be absolute")
        return path
    if logical.startswith("external/"):
        if logical != "external/mujoco-3.3.3/lib/libmujoco.so.3.3.3":
            raise StageBManifestError(f"unknown external artifact: {logical}")
        return LIB_MUJOCO
    return _repo_path(logical, "artifact path")


def validate_manifest(path: Path, *, expected_executable: Optional[str] = None) -> Dict[str, Any]:
    """Validate current files against a Stage-B manifest, fail-closed."""
    path = Path(path)
    _lstat_regular(path, "stage manifest")
    try:
        path.resolve().relative_to(P1_10_EVIDENCE.resolve())
    except ValueError as exc:
        raise StageBManifestError("stage manifest must be under docs/evidence/P1-10") from exc
    manifest = _load_json(path)
    _require(manifest, ("schema_version", "manifest_id", "status", "generated_by",
                        "runtime_locator", "identity_schema", "identity_input",
                        "identity_sha256"), "manifest")
    if manifest["schema_version"] != STAGE_SCHEMA or manifest["identity_schema"] != IDENTITY_SCHEMA:
        raise StageBManifestError("unsupported Stage-B manifest schema")
    if manifest["manifest_id"] != "P1-10-STAGE-B-EXECUTION-obstacle_test1-stabilized":
        raise StageBManifestError("wrong Stage-B manifest identity")
    if manifest["status"] != STATUS:
        raise StageBManifestError("Stage-B manifest status is not frozen pending review")
    generator = manifest["generated_by"]
    if not isinstance(generator, dict) or generator.get("script") != "scripts/p1_10_stage_b_execution.py":
        raise StageBManifestError("generator identity path mismatch")
    if generator.get("script_sha256") != sha256_file(REPO / "scripts/p1_10_stage_b_execution.py"):
        raise StageBManifestError("generator identity hash drift")
    if manifest["identity_input"].get("canonical_encoding") != CANONICAL_ENCODING:
        raise StageBManifestError("canonical encoding mismatch")
    if manifest["identity_sha256"] != sha256_bytes(canonical_bytes(manifest["identity_input"])):
        raise StageBManifestError("identity hash mismatch")
    locator = manifest["runtime_locator"]
    if not isinstance(locator, dict) or locator.get("executable_path_is_locator_only") is not True:
        raise StageBManifestError("runtime locator malformed")
    executable = locator.get("executable")
    if not isinstance(executable, str) or not Path(executable).is_absolute():
        raise StageBManifestError("runtime executable locator malformed")
    if Path(executable).resolve() != STAGE_B_EXECUTABLE.resolve():
        raise StageBManifestError("runtime executable locator substitution")
    if expected_executable is not None and Path(expected_executable).resolve() != Path(executable).resolve():
        raise StageBManifestError("actual --mujoco-bin is not the frozen Stage-B locator")

    identity = manifest["identity_input"]
    _require(identity, ("execution_kind", "scenario", "scene", "obstacle_metadata",
                        "initial_state", "goal", "variant_binding", "fixed_binding",
                        "baseline_parent", "artifacts", "source_identity",
                        "build_identity",
                        "collision_contract", "capture_identity_contract",
                        "authority_scope", "operator_prohibitions", "canonical_encoding"),
             "identity_input")
    if identity["execution_kind"] != "P1-10_STAGE_B_OBSERVATIONAL_RUNTIME":
        raise StageBManifestError("not a Stage-B observational identity")
    scenario = identity["scenario"]
    if scenario.get("scenario_id") != "obstacle_test1":
        raise StageBManifestError("Stage-B identity is not obstacle_test1")
    if scenario.get("scenario_path") != _repo_relative(SCENARIO_PATH):
        raise StageBManifestError("scenario path drift")
    if scenario.get("scenario_sha256") != sha256_file(SCENARIO_PATH):
        raise StageBManifestError("scenario hash drift")
    if scenario.get("candidate_suite_path") != _repo_relative(CANDIDATE_SUITE_PATH):
        raise StageBManifestError("candidate suite path drift")
    if scenario.get("candidate_suite_sha256") != sha256_file(CANDIDATE_SUITE_PATH):
        raise StageBManifestError("candidate suite hash drift")
    if scenario.get("candidate_status") != "UNSUPPORTED":
        raise StageBManifestError("candidate status changed without review")

    scene = identity["scene"]
    closure = _closure_identity(ROOT_XML)
    for key in ("root_xml_sha256", "model_closure_sha256"):
        _hash_field(scene.get(key), f"scene.{key}")
    if scene.get("launch_arg") != "scene_test1.xml" or scene.get("root_xml") != closure["root_xml"]:
        raise StageBManifestError("scene launch/root binding drift")
    if scene.get("root_xml_sha256") != closure["root_xml_sha256"] or scene.get("model_closure_sha256") != closure["model_closure_sha256"]:
        raise StageBManifestError("scene root/closure hash drift")
    if scene.get("closure_files") != closure["files"]:
        raise StageBManifestError("scene closure file inventory drift")
    _hash_field(scene.get("runtime_model_fingerprint"), "scene.runtime_model_fingerprint")
    if scene.get("runtime_model_fingerprint_schema") != MODEL_FINGERPRINT_SCHEMA:
        raise StageBManifestError("runtime model fingerprint schema drift")
    actual_initial = _probe_initial_state(ROOT_XML)
    if scene["runtime_model_fingerprint"] != actual_initial["collision_model_fingerprint"]:
        raise StageBManifestError("runtime model fingerprint drift")

    candidate, candidate_suite = _candidate()
    metadata = identity["obstacle_metadata"]
    objects = candidate["obstacle_layout"].get("objects")
    if metadata.get("objects") != objects or metadata.get("sha256") != sha256_bytes(canonical_bytes(objects)):
        raise StageBManifestError("obstacle metadata drift")
    initial = identity["initial_state"]
    if initial.get("qpos") != actual_initial["qpos"] or initial.get("qpos_sha256") != actual_initial["qpos_sha256"]:
        raise StageBManifestError("initial qpos drift")
    if initial.get("source") != candidate.get("initial_state_source"):
        raise StageBManifestError("initial-state source binding drift")
    if initial.get("probe_source_sha256") != sha256_file(PROBE_SOURCE) or initial.get("probe_executable_sha256") != sha256_file(PROBE_EXECUTABLE):
        raise StageBManifestError("initial-state probe identity drift")
    fixed = identity["fixed_binding"]
    if fixed != {"variant": "stabilized", "switching_mode": "stabilized_switch", "root_seed": 20260902,
                 "window_s": 25.0, "initial_state_source": "scene_default",
                 "initial_state_reset_source": "mj_makeData:qpos0; no keyframe reset",
                 "goal_world_xy_m": [7.0, 0.0]}:
        raise StageBManifestError("fixed binding drift")
    if identity["goal"].get("goal") != candidate.get("goal") or identity["goal"].get("injection") != candidate.get("goal_injection"):
        raise StageBManifestError("goal/injection drift")

    baseline = identity["baseline_parent"]
    _require(baseline, ("manifest_path", "manifest_sha256", "identity_path",
                        "identity_file_sha256", "canonical_identity_sha256",
                        "accepted_executable_sha256"), "baseline_parent")
    if baseline["manifest_path"] != _repo_relative(BASELINE_MANIFEST) or baseline["identity_path"] != _repo_relative(BASELINE_IDENTITY):
        raise StageBManifestError("P1-08 parent path drift")
    if baseline["manifest_sha256"] != sha256_file(BASELINE_MANIFEST) or baseline["identity_file_sha256"] != sha256_file(BASELINE_IDENTITY):
        raise StageBManifestError("P1-08 parent document drift")
    if baseline["canonical_identity_sha256"] != "59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0":
        raise StageBManifestError("P1-08 canonical identity drift")

    artifacts = identity["artifacts"]
    if not isinstance(artifacts, list) or len(artifacts) != len(_runtime_artifact_files()):
        raise StageBManifestError("artifact inventory incomplete")
    seen = set()
    for record in artifacts:
        if not isinstance(record, dict):
            raise StageBManifestError("artifact record malformed")
        _require(record, ("path", "role", "sha256", "bytes"), "artifact")
        if record["path"] in seen:
            raise StageBManifestError("duplicate artifact path")
        seen.add(record["path"])
        actual = _actual_path_for_logical(record["path"], executable)
        _lstat_regular(actual, "artifact")
        if record["sha256"] != sha256_file(actual) or record["bytes"] != actual.stat().st_size:
            raise StageBManifestError(f"artifact drift: {record['path']}")
    if artifacts[0]["sha256"] == baseline["accepted_executable_sha256"]:
        raise StageBManifestError("Stage-B executable is accepted P1-08 executable")
    build_identity = identity["build_identity"]
    if build_identity.get("system") != "CMake" or build_identity.get("target") != "unitree_mujoco" or build_identity.get("build_type") != "Release":
        raise StageBManifestError("Stage-B build identity metadata drift")
    flags = build_identity.get("flags")
    if not isinstance(flags, dict) or flags.get("path") != "unitree_mujoco/simulate/build2/CMakeFiles/unitree_mujoco.dir/flags.make":
        raise StageBManifestError("Stage-B build flags identity missing")
    _lstat_regular(BUILD_FLAGS, "Stage-B build flags")
    if flags.get("sha256") != sha256_file(BUILD_FLAGS) or flags.get("bytes") != BUILD_FLAGS.stat().st_size:
        raise StageBManifestError("Stage-B build flags drift")

    contract = identity["collision_contract"]
    if contract != {"schema": COLLISION_SCHEMA, "version": COLLISION_VERSION,
                    "layout_bytes": COLLISION_BYTES, "shm_name": "/mujoco_collision_v2",
                    "capture_identity_schema": CAPTURE_SCHEMA,
                    "model_fingerprint_schema": MODEL_FINGERPRINT_SCHEMA}:
        raise StageBManifestError("collision contract drift")
    capture = identity["capture_identity_contract"]
    if capture.get("schema") != CAPTURE_SCHEMA or capture.get("format") != "p1-10-capture-<32 lowercase hex characters>" or capture.get("source") != "harness-generated per capture; no CLI override":
        raise StageBManifestError("capture identity contract drift")
    if identity["authority_scope"] != {"scope": "main.cc PhysicsLoop two harness-controlled mj_step paths only",
                                        "publish": "one collision snapshot after each in-scope PhysicsLoop mj_step",
                                        "ui_step_forward": "excluded from formal capture authority"}:
        raise StageBManifestError("authority scope drift")
    flat_suite = _load_json(FLAT_SUITE_PATH)
    expected_variant = flat_suite.get("variants", {}).get("stabilized")
    if identity["variant_binding"] != expected_variant:
        raise StageBManifestError("stabilized consumed binding drift")
    if identity["variant_binding"].get("label") != "stabilized" or identity["variant_binding"].get("status") != "SUPPORTED":
        raise StageBManifestError("stabilized consumed binding missing")
    return {"manifest": manifest, "manifest_sha256": sha256_file(path),
            "identity_sha256": manifest["identity_sha256"], "executable": executable,
            "identity_input": identity, "candidate": candidate, "candidate_suite": candidate_suite}


def build_stage_b_context(validated: Mapping[str, Any]) -> Dict[str, Any]:
    """Create the production-shaped resolved context from the validated identity."""
    import sys
    sys.path.insert(0, str(REPO / "scripts"))
    from formal_experiment_contract import pairing_key as p1_02_pairing_key  # noqa: E402
    from p1_10_scenario_suite import variant_binding_hash  # noqa: E402

    identity = validated["identity_input"]
    scenario = validated["candidate"]
    suite = validated["candidate_suite"]
    fixed = identity["fixed_binding"]
    source = identity["initial_state"]["source"]
    qpos = identity["initial_state"]["qpos"]
    qpos_hash = identity["initial_state"]["qpos_sha256"]
    initial_binding = sha256_bytes(canonical_bytes({
        "source": {
            "kind": source["kind"], "startup_path": source["startup_path"],
            "reset_source": source["reset_source"], "probe_source": source["probe_source"],
            "probe_source_sha256": source["probe_source_sha256"],
        },
        "qpos_sha256": qpos_hash,
        "scene_model_closure_sha256": identity["scene"]["model_closure_sha256"],
    }))
    models = {item["role"]: item["sha256"] for item in identity["artifacts"]
              if item["role"] in {"controller_plugin"}}
    variant = copy.deepcopy(identity["variant_binding"])
    if variant_binding_hash(variant) != variant.get("binding_sha256"):
        raise StageBManifestError("stabilized variant binding hash mismatch")
    scenario_sha = identity["scenario"]["scenario_sha256"]
    seed_sources = {
        "random_producers": scenario.get("seed_registry", {}).get(
            "random_producers", "UNKNOWN"),
        "role": scenario.get("seed_registry", {}).get(
            "role", "pairing/provenance_only"),
    }
    effective_config_hash = sha256_bytes(canonical_bytes({
        "baseline_abs_controller_config_sha256": next(
            item["sha256"] for item in identity["artifacts"] if item["role"] == "abs_controller_config"),
        "scenario_config_overrides": scenario.get("config_overrides", {}),
    }))
    p1_key = p1_02_pairing_key({
        "scenario": {"id": "obstacle_test1", "sha256": scenario_sha},
        "seeds": {"root_seed": fixed["root_seed"]},
        "effective_config": {"sha256": effective_config_hash},
        "models": {"agile_policy": {"sha256": "5a87d692c3a04d1e286913fc392b1ad410bbaf115565b58db69e655c21e0b7cf"},
                   "ra_value": {"sha256": "05c40ff787b4143b15b20234005320d4477fb920ea63f50762828a908d1a90b7"},
                   "recovery_policy": {"sha256": "e3047a21c4391b29112028f0abd1a7ed04b72e52e923e4e388035e20b53b0171"}},
    })
    return {
        "schema_version": "abs-go2-deterministic-scenario/v2",
        "scenario_id": "obstacle_test1", "scenario_status": scenario["status"],
        "stage_b_capture_status": "IMPLEMENTED / AWAITING RUNTIME VALIDATION",
        "scenario_path": identity["scenario"]["scenario_path"],
        "scenario_sha256": scenario_sha,
        "suite_path": identity["scenario"]["candidate_suite_path"],
        "suite_sha256": identity["scenario"]["candidate_suite_sha256"],
        "baseline": scenario["baseline"],
        "scene": {
            "launch_arg": "scene_test1.xml", "root_xml": identity["scene"]["root_xml"],
            "root_xml_sha256": identity["scene"]["root_xml_sha256"],
            "model_closure_sha256": identity["scene"]["model_closure_sha256"],
            "closure_file_count": len(identity["scene"]["closure_files"]),
            "runtime_model_fingerprint": identity["scene"]["runtime_model_fingerprint"],
            "runtime_model_fingerprint_schema": MODEL_FINGERPRINT_SCHEMA,
        },
        "initial_state_source": source,
        "initial_state": {"qpos": qpos, "qpos_sha256": qpos_hash,
                           "base_pose_world_m": qpos[:3], "base_quat_wxyz": qpos[3:7],
                           "yaw_rad": 0.0, "binding_sha256": initial_binding},
        "variant_binding": variant, "goal": scenario["goal"],
        "goal_injection": scenario["goal_injection"],
        "switching_mode": scenario["switching_mode"], "run_window_s": 25.0,
        "obstacle_layout": scenario["obstacle_layout"],
        "config_overrides": scenario.get("config_overrides", {}),
        "seeds": {"root_seed": fixed["root_seed"], "sources": seed_sources,
                  "derived_seeds": {}},
        "pairing": {"scenario_root_seed_key": sha256_bytes(canonical_bytes({
            "scenario_id": "obstacle_test1", "scenario_sha256": scenario_sha,
            "root_seed": fixed["root_seed"]})), "p1_02_pairing_key": p1_key,
                    "variant": "stabilized"},
        "formal_context": {"scenario_id": "obstacle_test1", "scenario_sha256": scenario_sha,
                           "suite_sha256": identity["scenario"]["candidate_suite_sha256"],
                           "root_seed": fixed["root_seed"], "derived_seed_registry": seed_sources,
                           "baseline_identity_sha256": scenario["baseline"]["identity_sha256"],
                           "variant": "stabilized", "p1_02_pairing_key": p1_key,
                           "variant_binding_sha256": variant["binding_sha256"],
                           "initial_state_binding_sha256": initial_binding},
        "launch_contract": {"scenario": "obstacle_test1", "scene": "scene_test1.xml",
                             "initial_state_source": "scene_default", "root_seed": fixed["root_seed"],
                             "variant": "stabilized", "window_s": 25.0,
                             "baseline_manifest": scenario["baseline"]["manifest_path"],
                             "stage_b_execution_manifest_sha256": validated["manifest_sha256"]},
        "process_context": {"environment": {
            "ABS_P1_10_SCENARIO_ID": "obstacle_test1", "ABS_P1_10_SCENARIO_SHA256": scenario_sha,
            "ABS_P1_10_SUITE_SHA256": identity["scenario"]["candidate_suite_sha256"],
            "ABS_P1_10_ROOT_SEED": "20260902", "ABS_P1_10_VARIANT": "stabilized",
            "ABS_P1_10_BASELINE_IDENTITY_SHA256": scenario["baseline"]["identity_sha256"],
            "ABS_P1_10_DERIVED_SEED_REGISTRY": canonical_json(seed_sources),
            "ABS_P1_10_INITIAL_STATE_BINDING_SHA256": initial_binding,
            "ABS_P1_10_VARIANT_BINDING_SHA256": variant["binding_sha256"],
            "ABS_P1_10_ROOT_XML_SHA256": identity["scene"]["root_xml_sha256"],
            "ABS_P1_10_MODEL_CLOSURE_SHA256": identity["scene"]["model_closure_sha256"],
            "ABS_P1_10_EXPECTED_MODEL_FINGERPRINT": identity["scene"]["runtime_model_fingerprint"],
        }, "root_seed_role": "pairing/provenance_only", "consumer": "Stage-B collision authority"},
        "stage_b_execution_manifest": {"path": "docs/evidence/P1-10/stage_b_execution_manifest_20260905.json",
                                        "sha256": validated["manifest_sha256"],
                                        "identity_sha256": validated["identity_sha256"],
                                        "status": STATUS},
    }


def write_manifest(path: Path) -> Dict[str, Any]:
    manifest = build_manifest()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(canonical_bytes(manifest) + b"\n")
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", metavar="JSON", help="write a new Stage-B manifest")
    parser.add_argument("--validate", metavar="JSON", help="validate a Stage-B manifest")
    parser.add_argument("--executable", help="validation-only actual executable locator")
    args = parser.parse_args()
    try:
        if args.write:
            manifest = write_manifest(Path(args.write))
            print(json.dumps({"manifest": str(Path(args.write)),
                              "manifest_sha256": sha256_file(Path(args.write)),
                              "identity_sha256": manifest["identity_sha256"],
                              "status": manifest["status"]}, sort_keys=True))
        elif args.validate:
            result = validate_manifest(Path(args.validate), expected_executable=args.executable)
            print(json.dumps({"valid": True, "manifest_sha256": result["manifest_sha256"],
                              "identity_sha256": result["identity_sha256"],
                              "status": result["manifest"]["status"]}, sort_keys=True))
        else:
            parser.error("one of --write or --validate is required")
    except (StageBManifestError, OSError, StopIteration) as exc:
        print(f"STAGE-B IDENTITY REJECTED: {exc}", flush=True)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
