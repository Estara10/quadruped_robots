#!/usr/bin/env python3
"""P1-10 deterministic scenario and root-seed contract.

This module is deliberately offline.  It resolves a small, hash-bound
scenario suite and validates the arguments a future capture must use.  It
does not launch MuJoCo or ROS2 and it does not claim same-seed runtime replay.

The seed derivation is imported from the accepted P1-02 contract.  Do not
replace it with Python's process-randomized ``hash()`` or a second algorithm.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

REPO = Path(__file__).resolve().parents[1]
SCENARIO_DIR = REPO / "scenarios" / "p1_10"
SUITE_PATH = SCENARIO_DIR / "scenario_suite_manifest.json"
BASELINE_MANIFEST = REPO / "docs" / "evidence" / "P1-08" / "P1-08_baseline_manifest.json"
BASELINE_IDENTITY = REPO / "docs" / "evidence" / "P1-08" / "P1-08_simulation_baseline_identity.json"
SCENARIO_SCHEMA = "abs-go2-deterministic-scenario/v2"
SUITE_SCHEMA = "abs-go2-deterministic-suite/v2"
SUPPORTED = "SUPPORTED"
UNSUPPORTED = "UNSUPPORTED"
ALL_VARIANTS = ("paper-faithful", "stabilized", "agile-only")
SUPPORTED_VARIANTS = ("stabilized",)
VALID_VARIANTS = ALL_VARIANTS
P1_08_IDENTITY_SCHEMA = "abs-go2-p1-08-baseline-identity/v2"
INITIAL_STATE_PROBE = REPO / "unitree_mujoco" / "simulate" / "build2" / "p1_10_initial_state_probe"
INITIAL_STATE_PROBE_SOURCE = REPO / "unitree_mujoco" / "simulate" / "test" / "p1_10_initial_state_probe.cpp"
INITIAL_STATE_STARTUP_PATH = "main.cc:PhysicsThread: mj_loadXML -> mj_makeData -> sim.Load -> mj_forward"
LEGACY_INITIAL_STATE_PROBE_SHA256 = "71d148721fa9a14d88cf2eb32577ec0ca2942a3e36a956c14a5ceaf1578890f3"
CAPTURE_ID_PREFIX = "p1-10-capture-"
CAPTURE_ID_RE = re.compile(r"^p1-10-capture-[0-9a-f]{32}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")

sys.path.insert(0, str(Path(__file__).resolve().parent))
from formal_experiment_contract import derive_seed, pairing_key as p1_02_pairing_key  # noqa: E402
from build_p1_08_manifest import resolve_closure  # noqa: E402


def canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False)


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_json(value: Any) -> str:
    return sha256_bytes(canonical_json(value).encode("utf-8"))


def _load(path: Path) -> Dict[str, Any]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"expected JSON object: {path}")
    return payload


def _repo_relative(path: Path) -> str:
    return str(Path(path).resolve().relative_to(REPO.resolve()))


def _resolve_repo_path(value: str, *, base: Path = REPO) -> Path:
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = base / candidate
    resolved = candidate.resolve()
    try:
        resolved.relative_to(REPO.resolve())
    except ValueError as exc:
        raise ValueError(f"path escapes repository: {value}") from exc
    return resolved


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def extract_runtime_initial_state(scene_root: Path) -> Dict[str, Any]:
    """Read the actual MuJoCo construction path without stepping it.

    The probe is deliberately separate from the capture harness.  It mirrors
    ``PhysicsThread`` through ``mj_forward`` and proves that the scenario's
    declared qpos is the scene-default ``mj_makeData`` qpos0, not a keyframe
    that the current launcher never loads.
    """
    if not INITIAL_STATE_PROBE.is_file():
        raise ValueError(f"initial-state probe missing: {INITIAL_STATE_PROBE}")
    if not INITIAL_STATE_PROBE_SOURCE.is_file():
        raise ValueError(f"initial-state probe source missing: {INITIAL_STATE_PROBE_SOURCE}")
    try:
        result = subprocess.run(
            [str(INITIAL_STATE_PROBE), str(scene_root)],
            cwd=str(REPO), capture_output=True, text=True, timeout=30,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ValueError(f"initial-state probe failed to execute: {type(exc).__name__}: {exc}") from exc
    if result.returncode != 0:
        raise ValueError(f"initial-state probe failed rc={result.returncode}: {(result.stderr or result.stdout).strip()}")

    fields: Dict[str, Any] = {}
    for line in result.stdout.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        if value.startswith("["):
            try:
                fields[key] = json.loads(value)
            except json.JSONDecodeError as exc:
                raise ValueError(f"initial-state probe malformed vector: {key}") from exc
        else:
            fields[key] = value
    if fields.get("startup_path") != "mj_loadXML->mj_makeData->mj_forward":
        raise ValueError("initial-state probe startup path mismatch")
    for key in ("qpos_before_forward", "qpos0", "qpos_after_forward"):
        qpos = fields.get(key)
        if not isinstance(qpos, list) or not qpos or not all(_finite_number(x) for x in qpos):
            raise ValueError(f"initial-state probe missing/invalid {key}")
    if fields["qpos_before_forward"] != fields["qpos0"] or fields["qpos_after_forward"] != fields["qpos0"]:
        raise ValueError("initial-state probe qpos changed across construction/forward")
    fingerprint = fields.get("collision_model_fingerprint")
    if not isinstance(fingerprint, str) or SHA256_RE.fullmatch(fingerprint) is None:
        raise ValueError("initial-state probe missing/invalid collision model fingerprint")
    if fields.get("collision_model_fingerprint_schema") != "abs-go2-collision-model-fingerprint/v1":
        raise ValueError("initial-state probe fingerprint schema mismatch")
    qpos = fields["qpos0"]
    if len(qpos) < 7:
        raise ValueError("initial-state probe qpos is shorter than free-joint pose")
    quat = qpos[3:7]
    yaw = math.atan2(
        2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
        1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]),
    )
    return {
        "kind": "scene_default",
        "startup_path": INITIAL_STATE_STARTUP_PATH,
        "reset_source": "mj_makeData:qpos0; no keyframe reset",
        "probe_source": _repo_relative(INITIAL_STATE_PROBE_SOURCE),
        "probe_source_sha256": sha256_file(INITIAL_STATE_PROBE_SOURCE),
        "probe_executable": _repo_relative(INITIAL_STATE_PROBE),
        "probe_executable_sha256": sha256_file(INITIAL_STATE_PROBE),
        "qpos": qpos,
        "qpos_sha256": sha256_json(qpos),
        "base_pose_world_m": qpos[:3],
        "base_quat_wxyz": quat,
        "yaw_rad": yaw,
        "keyframe0_name": fields.get("keyframe0_name"),
        "keyframe0_is_not_loaded": fields.get("keyframe0_qpos") != qpos,
        "collision_model_fingerprint": fingerprint,
        "collision_model_fingerprint_schema": fields["collision_model_fingerprint_schema"],
    }


def _check_exact(mapping: Mapping[str, Any], required: Sequence[str], label: str, errors: List[str]) -> None:
    for key in required:
        if key not in mapping:
            errors.append(f"{label}.{key}:missing")


def variant_binding_hash(binding: Mapping[str, Any]) -> str:
    payload = {key: value for key, value in binding.items() if key != "binding_sha256"}
    return sha256_json(payload)


def validate_scenario_document(scenario: Mapping[str, Any], scenario_path: Optional[Path] = None) -> List[str]:
    """Validate a scenario without filling any omitted value from config."""
    errors: List[str] = []
    required = (
        "schema_version", "scenario_id", "status", "description", "baseline",
        "scene", "initial_state_source", "initial_state", "goal", "goal_injection", "switching_mode", "run_window_s",
        "obstacle_layout", "config_overrides", "random_sources",
    )
    _check_exact(scenario, required, "scenario", errors)
    allowed = set(required)
    errors.extend(f"scenario.{key}:undeclared" for key in scenario if key not in allowed)
    if scenario.get("schema_version") != SCENARIO_SCHEMA:
        errors.append("scenario.schema_version:unsupported")
    scenario_id = scenario.get("scenario_id")
    if not isinstance(scenario_id, str) or not scenario_id or "/" in scenario_id or "\\" in scenario_id:
        errors.append("scenario.scenario_id:invalid")
    status = scenario.get("status")
    if status not in (SUPPORTED, UNSUPPORTED):
        errors.append("scenario.status:invalid")
    if not isinstance(scenario.get("description"), str) or not scenario.get("description"):
        errors.append("scenario.description:missing")

    baseline = scenario.get("baseline", {})
    if not isinstance(baseline, Mapping):
        errors.append("scenario.baseline:type")
    else:
        _check_exact(baseline, ("manifest_path", "manifest_sha256", "identity_path", "identity_file_sha256", "identity_sha256"), "baseline", errors)
        if baseline.get("manifest_path") != _repo_relative(BASELINE_MANIFEST):
            errors.append("baseline.manifest_path:not_accepted_p1_08_manifest")
        if baseline.get("identity_path") != _repo_relative(BASELINE_IDENTITY):
            errors.append("baseline.identity_path:not_accepted_p1_08_identity")
        for key in ("manifest_sha256", "identity_file_sha256", "identity_sha256"):
            if not isinstance(baseline.get(key), str) or len(baseline.get(key, "")) != 64:
                errors.append(f"baseline.{key}:invalid_hash")

    scene = scenario.get("scene", {})
    if not isinstance(scene, Mapping):
        errors.append("scenario.scene:type")
    else:
        _check_exact(scene, ("launch_arg", "root_xml", "root_xml_sha256", "model_closure_sha256"), "scene", errors)
        launch_arg = scene.get("launch_arg")
        if not isinstance(launch_arg, str) or not launch_arg or Path(launch_arg).name != launch_arg:
            errors.append("scene.launch_arg:path_escape_or_missing")
        for key in ("root_xml_sha256", "model_closure_sha256"):
            if not isinstance(scene.get(key), str) or len(scene.get(key, "")) != 64:
                errors.append(f"scene.{key}:invalid_hash")
        if isinstance(scene.get("root_xml"), str):
            try:
                root = _resolve_repo_path(scene["root_xml"])
                if not root.is_file():
                    errors.append("scene.root_xml:missing")
            except ValueError:
                errors.append("scene.root_xml:path_escape")

    initial_source = scenario.get("initial_state_source", {})
    if not isinstance(initial_source, Mapping):
        errors.append("scenario.initial_state_source:type")
    else:
        _check_exact(initial_source, ("kind", "startup_path", "reset_source", "probe_source", "probe_source_sha256"),
                     "initial_state_source", errors)
        if initial_source.get("kind") != "scene_default":
            errors.append("initial_state_source.kind:unsupported")
        if initial_source.get("startup_path") != INITIAL_STATE_STARTUP_PATH:
            errors.append("initial_state_source.startup_path:mismatch")
        if initial_source.get("reset_source") != "mj_makeData:qpos0; no keyframe reset":
            errors.append("initial_state_source.reset_source:mismatch")
        if initial_source.get("probe_source") != _repo_relative(INITIAL_STATE_PROBE_SOURCE):
            errors.append("initial_state_source.probe_source:mismatch")
        if not isinstance(initial_source.get("probe_source_sha256"), str) or len(initial_source.get("probe_source_sha256", "")) != 64:
            errors.append("initial_state_source.probe_source_sha256:invalid")

    initial = scenario.get("initial_state", {})
    if not isinstance(initial, Mapping):
        errors.append("scenario.initial_state:type")
    else:
        _check_exact(initial, ("qpos", "qpos_sha256"), "initial_state", errors)
        qpos = initial.get("qpos")
        if not isinstance(qpos, list) or len(qpos) < 7 or not all(_finite_number(x) for x in qpos):
            errors.append("initial_state.qpos:invalid")
        if not isinstance(initial.get("qpos_sha256"), str) or len(initial.get("qpos_sha256", "")) != 64:
            errors.append("initial_state.qpos_sha256:invalid")
        elif isinstance(qpos, list) and sha256_json(qpos) != initial["qpos_sha256"]:
            errors.append("initial_state.qpos_sha256:does_not_match_qpos")

    goal = scenario.get("goal", {})
    if not isinstance(goal, Mapping):
        errors.append("scenario.goal:type")
    else:
        _check_exact(goal, ("world_xy_m", "resample_goal_on_arrival"), "goal", errors)
        xy = goal.get("world_xy_m")
        if not isinstance(xy, list) or len(xy) != 2 or not all(_finite_number(x) for x in xy):
            errors.append("goal.world_xy_m:invalid")
        if not isinstance(goal.get("resample_goal_on_arrival"), bool):
            errors.append("goal.resample_goal_on_arrival:invalid")
        if status == SUPPORTED and goal.get("resample_goal_on_arrival") is not False:
            errors.append("goal.resample_goal_on_arrival:formal_scenario_must_be_false")

    goal_injection = scenario.get("goal_injection", {})
    if not isinstance(goal_injection, Mapping):
        errors.append("goal_injection:type")
    else:
        _check_exact(goal_injection, ("source", "baseline_config_world_xy_m", "control_input", "effective_world_xy_m"), "goal_injection", errors)
        if not isinstance(goal_injection.get("source"), str) or not goal_injection.get("source"):
            errors.append("goal_injection.source:missing")
        base = goal_injection.get("baseline_config_world_xy_m")
        effective = goal_injection.get("effective_world_xy_m")
        control = goal_injection.get("control_input")
        if not isinstance(base, list) or len(base) != 2 or not all(_finite_number(x) for x in base):
            errors.append("goal_injection.baseline_config_world_xy_m:invalid")
        if not isinstance(effective, list) or len(effective) != 2 or not all(_finite_number(x) for x in effective):
            errors.append("goal_injection.effective_world_xy_m:invalid")
        if not isinstance(control, Mapping):
            errors.append("goal_injection.control_input:type")
        else:
            _check_exact(control, ("lx", "ly", "rx"), "goal_injection.control_input", errors)
            if not all(_finite_number(control.get(key)) for key in ("lx", "ly", "rx")):
                errors.append("goal_injection.control_input:invalid")
            elif isinstance(base, list) and len(base) == 2 and isinstance(effective, list) and len(effective) == 2:
                expected = [base[0] + 2.0 * control["ly"], base[1] - 2.0 * control["lx"]]
                if any(abs(expected[i] - effective[i]) > 1e-12 for i in range(2)):
                    errors.append("goal_injection:effective_goal_mismatch")
                if isinstance(goal, Mapping) and goal.get("world_xy_m") != effective:
                    errors.append("goal_injection:does_not_match_goal")

    if scenario.get("switching_mode") not in ("stabilized_switch", "paper_faithful_switch"):
        errors.append("switching_mode:invalid")
    if not _finite_number(scenario.get("run_window_s")) or float(scenario.get("run_window_s", 0)) <= 0:
        errors.append("run_window_s:invalid")
    if not isinstance(scenario.get("obstacle_layout"), Mapping):
        errors.append("obstacle_layout:type")
    if not isinstance(scenario.get("config_overrides"), Mapping):
        errors.append("config_overrides:type")
    elif status == SUPPORTED and scenario.get("config_overrides") != {}:
        # This increment has no registered override schema.  An override is
        # therefore rejected until a future task declares its exact field and
        # producer; it is never silently merged with the default config.
        errors.append("config_overrides:nonempty_without_registered_override")

    sources = scenario.get("random_sources", {})
    source_names = (
        "scenario_initial_condition_sampling", "mujoco_model_randomization",
        "controller_goal_resampling", "python_orchestrator", "cpp_rand",
        "torch_python_numpy", "obstacle_placement",
    )
    if not isinstance(sources, Mapping):
        errors.append("random_sources:type")
    else:
        if set(sources) != set(source_names):
            errors.append("random_sources:must_list_each_registered_source")
        for name in source_names:
            item = sources.get(name)
            if not isinstance(item, Mapping):
                errors.append(f"random_sources.{name}:type")
                continue
            _check_exact(item, ("status", "actual_injection_point", "evidence"), f"random_sources.{name}", errors)
            if item.get("status") not in ("DERIVED", "NOT_USED_IN_THIS_SCENARIO", "DECLARED_NOT_CONSUMED", "UNSUPPORTED"):
                errors.append(f"random_sources.{name}.status:invalid")
            if not isinstance(item.get("actual_injection_point"), str) or not item.get("actual_injection_point"):
                errors.append(f"random_sources.{name}.actual_injection_point:missing")
            if not isinstance(item.get("evidence"), str) or not item.get("evidence"):
                errors.append(f"random_sources.{name}.evidence:missing")
            if item.get("status") == "DERIVED" and not isinstance(item.get("derived_label"), str):
                errors.append(f"random_sources.{name}.derived_label:missing")
        if status == SUPPORTED and any(item.get("status") == "UNSUPPORTED" for item in sources.values() if isinstance(item, Mapping)):
            errors.append("random_sources:unsupported_source_in_supported_scenario")

    if status == UNSUPPORTED:
        obstacle = scenario.get("obstacle_layout", {})
        if isinstance(obstacle, Mapping) and obstacle.get("status") != UNSUPPORTED:
            errors.append("unsupported_scenario.obstacle_layout:must_be_explicitly_unsupported")
    return sorted(set(errors))


def validate_suite_manifest(suite_path: Path = SUITE_PATH) -> Dict[str, Any]:
    suite = _load(suite_path)
    errors: List[str] = []
    required = ("schema_version", "suite_id", "baseline", "variants", "scenarios", "canonical_encoding")
    _check_exact(suite, required, "suite", errors)
    if suite.get("schema_version") != SUITE_SCHEMA:
        errors.append("suite.schema_version:unsupported")
    if suite.get("canonical_encoding") != "json.dumps(value, sort_keys=True, separators=(',', ':')).encode('utf-8'); sha256":
        errors.append("suite.canonical_encoding:unexpected")
    baseline = suite.get("baseline", {})
    if not isinstance(baseline, Mapping):
        errors.append("suite.baseline:type")
    else:
        for key in ("manifest_path", "manifest_sha256", "identity_path", "identity_file_sha256", "identity_sha256"):
            if key not in baseline:
                errors.append(f"suite.baseline.{key}:missing")
        if baseline.get("manifest_path") != _repo_relative(BASELINE_MANIFEST):
            errors.append("suite.baseline:wrong_p1_08_manifest")
    variants = suite.get("variants", {})
    if not isinstance(variants, Mapping) or set(variants) != set(ALL_VARIANTS):
        errors.append("suite.variants:must_define_all_variant_labels")
        variants = {}
    for label in ALL_VARIANTS:
        item = variants.get(label)
        if not isinstance(item, Mapping):
            errors.append(f"suite.variants.{label}:type")
            continue
        _check_exact(item, ("label", "status"), f"suite.variants.{label}", errors)
        if item.get("label") != label:
            errors.append(f"suite.variants.{label}.label:mismatch")
        if item.get("status") not in (SUPPORTED, UNSUPPORTED):
            errors.append(f"suite.variants.{label}.status:invalid")
        if item.get("status") == SUPPORTED:
            _check_exact(item, ("runtime_configuration", "consumed_behavior", "binding_sha256"),
                         f"suite.variants.{label}", errors)
            if label not in SUPPORTED_VARIANTS:
                errors.append(f"suite.variants.{label}:unexpected_supported_variant")
            if not isinstance(item.get("runtime_configuration"), Mapping):
                errors.append(f"suite.variants.{label}.runtime_configuration:type")
            if not isinstance(item.get("consumed_behavior"), Mapping):
                errors.append(f"suite.variants.{label}.consumed_behavior:type")
            if isinstance(item.get("binding_sha256"), str) and len(item["binding_sha256"]) == 64:
                if variant_binding_hash(item) != item["binding_sha256"]:
                    errors.append(f"suite.variants.{label}.binding_sha256:mismatch")
        elif not isinstance(item.get("reason"), str) or not item.get("reason"):
            errors.append(f"suite.variants.{label}.reason:missing")
    entries = suite.get("scenarios", [])
    if not isinstance(entries, list) or not entries:
        errors.append("suite.scenarios:missing")
        entries = []
    seen = set()
    for index, entry in enumerate(entries):
        if not isinstance(entry, Mapping):
            errors.append(f"suite.scenarios[{index}]:type")
            continue
        _check_exact(entry, ("scenario_id", "path", "sha256", "status"), f"suite.scenarios[{index}]", errors)
        sid = entry.get("scenario_id")
        if sid in seen:
            errors.append(f"suite.scenarios[{index}]:duplicate_id")
        seen.add(sid)
        try:
            path = _resolve_repo_path(str(entry.get("path", "")))
            if not path.is_file():
                errors.append(f"suite.scenarios[{index}]:missing_file")
            elif sha256_file(path) != entry.get("sha256"):
                errors.append(f"suite.scenarios[{index}]:scenario_hash_mismatch")
            scenario = _load(path)
            errors.extend(f"{sid}:{error}" for error in validate_scenario_document(scenario, path))
            if scenario.get("scenario_id") != sid or scenario.get("status") != entry.get("status"):
                errors.append(f"suite.scenarios[{index}]:index_document_mismatch")
        except (ValueError, OSError, json.JSONDecodeError) as exc:
            errors.append(f"suite.scenarios[{index}]:{type(exc).__name__}")
    return {"valid": not errors, "errors": sorted(set(errors)), "suite": suite, "suite_sha256": sha256_file(suite_path)}


def _find_scenario(ref: str, suite: Mapping[str, Any]) -> Tuple[Path, Dict[str, Any]]:
    for entry in suite.get("scenarios", []):
        if ref == entry.get("scenario_id") or ref == entry.get("path"):
            path = _resolve_repo_path(str(entry["path"]))
            return path, dict(entry)
    raise ValueError(f"scenario is not registered in suite: {ref}")


def _validate_baseline(scenario: Mapping[str, Any], suite: Mapping[str, Any], suite_path: Path) -> Dict[str, Any]:
    baseline = scenario["baseline"]
    errors: List[str] = []
    manifest_path = _resolve_repo_path(baseline["manifest_path"])
    identity_path = _resolve_repo_path(baseline["identity_path"])
    for path, expected, label in ((manifest_path, baseline["manifest_sha256"], "manifest"),
                                  (identity_path, baseline["identity_file_sha256"], "identity_file")):
        if not path.is_file() or sha256_file(path) != expected:
            errors.append(f"baseline_{label}_hash_mismatch")
    manifest = _load(manifest_path)
    identity = _load(identity_path)
    if identity.get("schema") != P1_08_IDENTITY_SCHEMA or identity.get("baseline_identity_sha256") != baseline["identity_sha256"]:
        errors.append("baseline_identity_value_mismatch")
    canonical = identity.get("canonical_input", {})
    if canonical.get("manifest_sha256") != baseline["manifest_sha256"]:
        errors.append("baseline_identity_manifest_binding_mismatch")
    if canonical.get("model_closure_sha256") != scenario["scene"]["model_closure_sha256"]:
        errors.append("baseline_identity_closure_mismatch")
    abs_configs = [item for item in manifest.get("config_files", [])
                   if item.get("role") == "abs_controller_config"]
    if len(abs_configs) != 1:
        errors.append("baseline_abs_controller_config:missing_or_ambiguous")
    else:
        try:
            config_text = _resolve_repo_path(abs_configs[0]["path"]).read_text(encoding="utf-8")
            if not re.search(r"(?m)^\s*resample_goal_on_arrival\s*:\s*false\s*(?:#.*)?$", config_text):
                errors.append("baseline_abs_controller_config:resample_goal_not_explicit_false")
        except (KeyError, OSError, ValueError):
            errors.append("baseline_abs_controller_config:unreadable")
    return {
        "errors": sorted(set(errors)),
        "manifest": manifest,
        "identity": identity,
        "manifest_path": _repo_relative(manifest_path),
        "identity_path": _repo_relative(identity_path),
        "manifest_sha256": sha256_file(manifest_path),
        "identity_file_sha256": sha256_file(identity_path),
        "identity_sha256": identity.get("baseline_identity_sha256"),
    }


def _validate_scene(scenario: Mapping[str, Any], baseline_manifest: Mapping[str, Any]) -> Dict[str, Any]:
    scene = scenario["scene"]
    root = _resolve_repo_path(scene["root_xml"])
    closure = resolve_closure(root)
    errors = list(closure["failures"])
    if sha256_file(root) != scene["root_xml_sha256"]:
        errors.append("scene_root_xml_hash_mismatch")
    if closure["closure_sha256"] != scene["model_closure_sha256"]:
        errors.append("scene_model_closure_hash_mismatch")
    recorded = baseline_manifest.get("model_closure", {})
    if recorded.get("root_xml") != str(root) or recorded.get("closure_sha256") != scene["model_closure_sha256"]:
        errors.append("scene_not_same_as_accepted_p1_08_closure")
    try:
        model_xml = (root.parent / "go2.xml").read_text(encoding="utf-8")
        if 'name="home" qpos="0 0 0.445 1 0 0 0' not in model_xml:
            errors.append("initialization_home_keyframe_not_found")
    except OSError:
        errors.append("initialization_model_xml_unreadable")
    return {"errors": sorted(set(errors)), "root_xml": str(root), "root_xml_sha256": sha256_file(root), "closure": closure}


def scenario_pairing_key(scenario_id: str, scenario_sha256: str, root_seed: int) -> str:
    """The P1-10 pairing identity required to be identical across labels."""
    return sha256_json({"scenario_id": scenario_id, "scenario_sha256": scenario_sha256, "root_seed": root_seed})


def resolve_variant_binding(variant: str, suite: Mapping[str, Any], baseline_manifest: Mapping[str, Any],
                            switching_mode: str) -> Dict[str, Any]:
    """Return a binding only when the label has a real consumed path."""
    if variant not in ALL_VARIANTS:
        raise ValueError(f"invalid variant: {variant}")
    binding = suite.get("variants", {}).get(variant)
    if not isinstance(binding, Mapping) or binding.get("status") != SUPPORTED:
        reason = binding.get("reason", "variant has no consumed runtime binding") if isinstance(binding, Mapping) else "variant is undeclared"
        raise ValueError(f"variant_not_supported:{variant}:{reason}")
    runtime = binding.get("runtime_configuration", {})
    behavior = binding.get("consumed_behavior", {})
    abs_config = next((item for item in baseline_manifest.get("config_files", [])
                       if item.get("role") == "abs_controller_config"), None)
    plugin = next((item for item in baseline_manifest.get("binaries", [])
                   if item.get("role") == "controller_plugin"), None)
    if not isinstance(abs_config, Mapping) or not isinstance(plugin, Mapping):
        raise ValueError("variant_binding_missing_actual_baseline_consumer")
    if runtime.get("switching_mode") != switching_mode:
        raise ValueError("variant_binding_switching_mode_mismatch")
    if runtime.get("abs_controller_config_sha256") != abs_config.get("sha256"):
        raise ValueError("variant_binding_config_hash_mismatch")
    if behavior.get("controller_plugin_sha256") != plugin.get("sha256"):
        raise ValueError("variant_binding_controller_hash_mismatch")
    try:
        if sha256_file(_resolve_repo_path(abs_config["path"])) != runtime["abs_controller_config_sha256"]:
            raise ValueError("variant_binding_actual_config_hash_mismatch")
        if sha256_file(_resolve_repo_path(plugin["path"])) != behavior["controller_plugin_sha256"]:
            raise ValueError("variant_binding_actual_controller_hash_mismatch")
    except (KeyError, OSError, ValueError) as exc:
        if isinstance(exc, ValueError) and str(exc).startswith("variant_binding_actual_"):
            raise
        raise ValueError("variant_binding_actual_consumer_unreadable") from exc
    if variant_binding_hash(binding) != binding.get("binding_sha256"):
        raise ValueError("variant_binding_hash_mismatch")
    return dict(binding)


def validate_resolved_variant_context(resolved: Mapping[str, Any]) -> None:
    """Reject a context whose label was changed without changing its consumer."""
    variant = resolved.get("pairing", {}).get("variant")
    binding = resolved.get("variant_binding")
    if variant not in SUPPORTED_VARIANTS or not isinstance(binding, Mapping):
        raise ValueError("resolved variant has no supported runtime consumer")
    if binding.get("label") != variant or binding.get("status") != SUPPORTED:
        raise ValueError("variant label does not match consumed runtime binding")
    if variant_binding_hash(binding) != binding.get("binding_sha256"):
        raise ValueError("resolved variant binding hash mismatch")
    if binding.get("runtime_configuration", {}).get("switching_mode") != resolved.get("switching_mode"):
        raise ValueError("resolved variant binding does not match consumed switching mode")
    if resolved.get("formal_context", {}).get("variant_binding_sha256") != binding.get("binding_sha256"):
        raise ValueError("formal variant binding hash mismatch")
    initial = resolved.get("initial_state", {})
    if resolved.get("formal_context", {}).get("initial_state_binding_sha256") != initial.get("binding_sha256"):
        raise ValueError("formal initial-state binding hash mismatch")
    if resolved.get("process_context", {}).get("environment", {}).get("ABS_P1_10_VARIANT") != variant:
        raise ValueError("process variant label disagrees with resolved variant")
    if resolved.get("process_context", {}).get("environment", {}).get("ABS_P1_10_VARIANT_BINDING_SHA256") != binding.get("binding_sha256"):
        raise ValueError("process variant binding hash disagrees with resolved variant")
    if resolved.get("process_context", {}).get("environment", {}).get("ABS_P1_10_INITIAL_STATE_BINDING_SHA256") != initial.get("binding_sha256"):
        raise ValueError("process initial-state binding hash disagrees with resolved state")


def resolve_scenario(scenario_ref: str, root_seed: int, variant: str = "stabilized",
                    suite_path: Path = SUITE_PATH) -> Dict[str, Any]:
    if isinstance(root_seed, bool) or not isinstance(root_seed, int) or root_seed < 0:
        raise ValueError("root_seed must be a non-negative integer")
    if variant not in ALL_VARIANTS:
        raise ValueError(f"invalid variant: {variant}")
    suite_result = validate_suite_manifest(Path(suite_path))
    if not suite_result["valid"]:
        raise ValueError("scenario suite invalid: " + "; ".join(suite_result["errors"]))
    suite = suite_result["suite"]
    scenario_path, entry = _find_scenario(scenario_ref, suite)
    scenario = _load(scenario_path)
    errors = validate_scenario_document(scenario, scenario_path)
    if errors:
        raise ValueError("scenario invalid: " + "; ".join(errors))
    baseline_result = _validate_baseline(scenario, suite, Path(suite_path))
    variant_binding = resolve_variant_binding(variant, suite, baseline_result["manifest"], scenario["switching_mode"])
    scene_result = _validate_scene(scenario, baseline_result["manifest"])
    runtime_initial = extract_runtime_initial_state(Path(scene_result["root_xml"]))
    errors = baseline_result["errors"] + scene_result["errors"]
    if suite.get("baseline") != scenario.get("baseline"):
        errors.append("suite_scenario_baseline_binding_mismatch")
    facts = baseline_result["manifest"].get("effective_controller_static_facts", {})
    baseline_goal = scenario["goal_injection"]["baseline_config_world_xy_m"]
    if facts.get("abs.goal_x", {}).get("value") != baseline_goal[0] or facts.get("abs.goal_y", {}).get("value") != baseline_goal[1]:
        errors.append("scenario_goal_injection_baseline_config_mismatch")
    if facts.get("abs.switching_mode", {}).get("value") != scenario["switching_mode"]:
        errors.append("scenario_switching_mode_baseline_config_mismatch")
    declared_initial = scenario["initial_state"]
    declared_source = scenario["initial_state_source"]
    if declared_initial["qpos"] != runtime_initial["qpos"]:
        errors.append("initial_state.qpos_not_equal_to_actual_mj_makeData_qpos0")
    if declared_initial["qpos_sha256"] != runtime_initial["qpos_sha256"]:
        errors.append("initial_state.qpos_sha256_mismatch")
    if (declared_source["probe_source_sha256"] != runtime_initial["probe_source_sha256"] and
            declared_source["probe_source_sha256"] != LEGACY_INITIAL_STATE_PROBE_SHA256):
        errors.append("initial_state_source.probe_source_sha256_mismatch")
    if runtime_initial["keyframe0_is_not_loaded"] is not True:
        errors.append("initial_state_source:unexpected_keyframe_load")
    if scenario["status"] != SUPPORTED:
        errors.append("scenario_status_unsupported")
    if errors:
        raise ValueError("scenario preflight invalid: " + "; ".join(sorted(set(errors))))

    scenario_sha = sha256_file(scenario_path)
    closure = scene_result["closure"]
    root_seed_sources: Dict[str, Any] = {}
    for source_name, item in scenario["random_sources"].items():
        resolved = dict(item)
        if item["status"] == "DERIVED":
            label = item["derived_label"]
            resolved["derived_seed"] = derive_seed(root_seed, label)
        root_seed_sources[source_name] = resolved
    derived = {name: item["derived_seed"] for name, item in root_seed_sources.items() if "derived_seed" in item}

    initial_state_binding_sha = sha256_json({
        "source": {
            "kind": runtime_initial["kind"],
            "startup_path": runtime_initial["startup_path"],
            "reset_source": runtime_initial["reset_source"],
            "probe_source": runtime_initial["probe_source"],
            "probe_source_sha256": runtime_initial["probe_source_sha256"],
        },
        "qpos_sha256": runtime_initial["qpos_sha256"],
        "scene_model_closure_sha256": scenario["scene"]["model_closure_sha256"],
    })

    config_hash = sha256_json({
        "baseline_abs_controller_config_sha256": next(
            item["sha256"] for item in baseline_result["manifest"]["config_files"]
            if item["role"] == "abs_controller_config"),
        "scenario_config_overrides": scenario["config_overrides"],
    })
    models = {
        item["role"]: item["sha256"]
        for item in baseline_result["manifest"]["deployed_policy_artifacts"]
    }
    formal_key = p1_02_pairing_key({
        "scenario": {"id": scenario["scenario_id"], "sha256": scenario_sha},
        "seeds": {"root_seed": root_seed},
        "effective_config": {"sha256": config_hash},
        "models": {
            "agile_policy": {"sha256": models["agile_policy"]},
            "ra_value": {"sha256": models["ra_model"]},
            "recovery_policy": {"sha256": models["recovery_policy"]},
        },
    })
    return {
        "schema_version": SCENARIO_SCHEMA,
        "scenario_id": scenario["scenario_id"],
        "scenario_status": scenario["status"],
        "scenario_path": _repo_relative(scenario_path),
        "scenario_sha256": scenario_sha,
        "suite_path": _repo_relative(Path(suite_path).resolve()),
        "suite_sha256": suite_result["suite_sha256"],
        "baseline": {
            "manifest_path": baseline_result["manifest_path"],
            "manifest_sha256": baseline_result["manifest_sha256"],
            "identity_path": baseline_result["identity_path"],
            "identity_file_sha256": baseline_result["identity_file_sha256"],
            "identity_sha256": baseline_result["identity_sha256"],
        },
        "scene": {
            "launch_arg": scenario["scene"]["launch_arg"],
            "root_xml": _repo_relative(Path(scene_result["root_xml"])),
            "root_xml_sha256": scene_result["root_xml_sha256"],
            "model_closure_sha256": scenario["scene"]["model_closure_sha256"],
            "closure_file_count": closure["present_file_count"],
            "runtime_model_fingerprint": runtime_initial["collision_model_fingerprint"],
            "runtime_model_fingerprint_schema": runtime_initial["collision_model_fingerprint_schema"],
        },
        "initial_state_source": runtime_initial,
        "initial_state": {
            "qpos": runtime_initial["qpos"],
            "qpos_sha256": runtime_initial["qpos_sha256"],
            "base_pose_world_m": runtime_initial["base_pose_world_m"],
            "base_quat_wxyz": runtime_initial["base_quat_wxyz"],
            "yaw_rad": runtime_initial["yaw_rad"],
            "binding_sha256": initial_state_binding_sha,
        },
        "variant_binding": variant_binding,
        "goal": scenario["goal"],
        "goal_injection": scenario["goal_injection"],
        "switching_mode": scenario["switching_mode"],
        "run_window_s": scenario["run_window_s"],
        "obstacle_layout": scenario["obstacle_layout"],
        "config_overrides": scenario["config_overrides"],
        "seeds": {
            "root_seed": root_seed,
            "sources": root_seed_sources,
            "derived_seeds": derived,
        },
        "pairing": {
            "scenario_root_seed_key": scenario_pairing_key(scenario["scenario_id"], scenario_sha, root_seed),
            "p1_02_pairing_key": formal_key,
            "variant": variant,
        },
        "formal_context": {
            "scenario_id": scenario["scenario_id"],
            "scenario_sha256": scenario_sha,
            "suite_sha256": suite_result["suite_sha256"],
            "root_seed": root_seed,
            "derived_seed_registry": root_seed_sources,
            "baseline_identity_sha256": baseline_result["identity_sha256"],
            "variant": variant,
            "p1_02_pairing_key": formal_key,
            "variant_binding_sha256": variant_binding["binding_sha256"],
            "initial_state_binding_sha256": initial_state_binding_sha,
        },
        "launch_contract": {
            "scenario": scenario_ref,
            "scene": scenario["scene"]["launch_arg"],
            "initial_state_source": scenario["initial_state_source"]["kind"],
            "root_seed": root_seed,
            "variant": variant,
            "window_s": scenario["run_window_s"],
            "baseline_manifest": baseline_result["manifest_path"],
        },
        "process_context": {
            "environment": {
                "ABS_P1_10_SCENARIO_ID": scenario["scenario_id"],
                "ABS_P1_10_SCENARIO_SHA256": scenario_sha,
                "ABS_P1_10_SUITE_SHA256": suite_result["suite_sha256"],
                "ABS_P1_10_ROOT_SEED": str(root_seed),
                "ABS_P1_10_VARIANT": variant,
                "ABS_P1_10_BASELINE_IDENTITY_SHA256": baseline_result["identity_sha256"],
                "ABS_P1_10_DERIVED_SEED_REGISTRY": canonical_json(root_seed_sources),
                "ABS_P1_10_INITIAL_STATE_BINDING_SHA256": initial_state_binding_sha,
                "ABS_P1_10_VARIANT_BINDING_SHA256": variant_binding["binding_sha256"],
                "ABS_P1_10_ROOT_XML_SHA256": scenario["scene"]["root_xml_sha256"],
                "ABS_P1_10_MODEL_CLOSURE_SHA256": scenario["scene"]["model_closure_sha256"],
                "ABS_P1_10_EXPECTED_MODEL_FINGERPRINT": runtime_initial["collision_model_fingerprint"],
            },
            "root_seed_role": "pairing/provenance_only",
            "consumer": "none in current flat capture path",
        },
    }


def validate_capture_id(value: Any) -> str:
    if not isinstance(value, str) or CAPTURE_ID_RE.fullmatch(value) is None:
        raise ValueError("capture_id must match p1-10-capture-[32 lowercase hex characters]")
    return value


def bind_capture_identity(resolved_context: Mapping[str, Any], capture_id: str) -> Dict[str, Any]:
    """Bind a harness-generated identity without changing deterministic resolution."""
    validate_capture_id(capture_id)
    bound = json.loads(canonical_json(resolved_context))
    bound["capture_identity"] = {
        "schema": "abs-go2-capture-identity/v1",
        "capture_id": capture_id,
        "generated_by": "scripts/p1_08_baseline_capture.py",
    }
    bound["capture_identity_input"] = {
        "capture_id": capture_id,
        "scenario_id": bound["scenario_id"],
        "scenario_sha256": bound["scenario_sha256"],
        "suite_sha256": bound["suite_sha256"],
        "scene_root_sha256": bound["scene"]["root_xml_sha256"],
        "model_closure_sha256": bound["scene"]["model_closure_sha256"],
        "runtime_model_fingerprint": bound["scene"]["runtime_model_fingerprint"],
    }
    bound["launch_contract"]["capture_id"] = capture_id
    bound["process_context"]["capture_id"] = capture_id
    bound["process_context"]["environment"]["ABS_P1_10_CAPTURE_ID"] = capture_id
    return bound


def prepare_capture_context(args: Any, suite_path: Path = SUITE_PATH) -> Dict[str, Any]:
    """Strictly compare capture CLI arguments with a resolved scenario."""
    missing = [name for name in ("scenario", "root_seed", "variant", "scene", "manifest", "window_s", "initial_state_source")
               if not hasattr(args, name) or getattr(args, name) in (None, "")]
    if missing:
        raise ValueError("missing required P1-10 capture argument(s): " + ", ".join(missing))
    suite_result = validate_suite_manifest(Path(suite_path))
    if not suite_result["valid"]:
        raise ValueError("scenario suite invalid: " + "; ".join(suite_result["errors"]))
    suite = suite_result["suite"]
    scenario_path, _ = _find_scenario(str(args.scenario), suite)
    resolved = resolve_scenario(str(args.scenario), int(args.root_seed), str(args.variant), Path(suite_path))
    validate_resolved_variant_context(resolved)
    validate_launch_arguments(args, resolved)
    resolved["launch_contract"]["scenario"] = str(args.scenario)
    resolved["launch_contract"]["scenario_path"] = _repo_relative(scenario_path)
    resolved["launch_contract"]["manifest"] = _repo_relative(Path(str(args.manifest)).resolve())
    return resolved


def validate_launch_arguments(args: Any, resolved: Mapping[str, Any]) -> None:
    """Fail closed when actual launch arguments do not match resolved data."""
    contract = resolved.get("launch_contract", {})
    if str(args.scenario) != str(contract.get("scenario")):
        raise ValueError("actual harness --scenario disagrees with resolved scenario manifest")
    if int(args.root_seed) != int(contract.get("root_seed")):
        raise ValueError("actual harness --root-seed disagrees with resolved scenario manifest")
    if str(args.variant) != str(contract.get("variant")):
        raise ValueError("actual harness --variant disagrees with resolved scenario manifest")
    if str(args.initial_state_source) != str(resolved["initial_state_source"]["kind"]):
        raise ValueError("actual harness initial-state source disagrees with resolved startup source")
    if str(args.scene) != resolved["scene"]["launch_arg"]:
        raise ValueError("actual harness --scene disagrees with resolved scenario manifest")
    manifest_path = Path(str(args.manifest)).resolve()
    expected_manifest = _resolve_repo_path(resolved["baseline"]["manifest_path"])
    if manifest_path != expected_manifest:
        raise ValueError("actual harness --manifest disagrees with accepted baseline manifest")
    if float(args.window_s) != float(resolved["run_window_s"]):
        raise ValueError("actual harness --window-s disagrees with scenario run window")


def write_resolved_manifest(out_path: Path, resolved: Mapping[str, Any]) -> None:
    Path(out_path).write_text(json.dumps(resolved, indent=2, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")


def validate_paired_contexts(contexts: Sequence[Mapping[str, Any]]) -> List[str]:
    errors: List[str] = []
    if len(contexts) != 3:
        errors.append("comparison_requires_exactly_three_variant_contexts")
        return errors
    labels = [context.get("pairing", {}).get("variant") for context in contexts]
    if set(labels) != set(VALID_VARIANTS):
        errors.append("missing_required_variant_context")
    if len(labels) != len(set(labels)):
        errors.append("duplicate_variant_context")
    bindings = [context.get("variant_binding", {}) for context in contexts]
    if any(binding.get("status") != SUPPORTED for binding in bindings if isinstance(binding, Mapping)):
        errors.append("paired_variant_not_supported_by_runtime_binding")
    keys = [context.get("pairing", {}).get("scenario_root_seed_key") for context in contexts]
    if any(not key for key in keys) or len(set(keys)) != 1:
        errors.append("paired_scenario_root_seed_key_mismatch")
    return sorted(set(errors))


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--validate-suite", action="store_true")
    parser.add_argument("--suite", default=str(SUITE_PATH))
    parser.add_argument("--scenario")
    parser.add_argument("--root-seed", type=int)
    parser.add_argument("--variant", choices=VALID_VARIANTS, default="stabilized")
    parser.add_argument("--out")
    args = parser.parse_args(argv)
    if args.validate_suite:
        result = validate_suite_manifest(Path(args.suite))
        print(json.dumps({k: v for k, v in result.items() if k != "suite"}, indent=2, sort_keys=True))
        return 0 if result["valid"] else 2
    if args.scenario is None or args.root_seed is None:
        parser.error("--scenario and --root-seed are required unless --validate-suite is used")
    resolved = resolve_scenario(args.scenario, args.root_seed, args.variant, Path(args.suite))
    if args.out:
        write_resolved_manifest(Path(args.out), resolved)
    print(json.dumps(resolved, indent=2, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
