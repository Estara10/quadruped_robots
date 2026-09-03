#!/usr/bin/env python3
"""Offline P1-10 historical obstacle-map inventory and candidate builder.

This module deliberately does not start a MuJoCo runtime, start ROS, or inspect
runtime shared memory.  Its offline construction probe loads each XML with
MuJoCo but performs no mj_step. It resolves XML/asset provenance and emits UNSUPPORTED
candidate scenario documents until the missing behavioral authorities are
closed by a separately approved runtime change.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "scripts"))
from p1_10_scenario_suite import extract_runtime_initial_state  # noqa: E402
GO2_DIR = REPO / "unitree_mujoco" / "unitree_robots" / "go2"
MAPS = tuple(f"scene_test{i}.xml" for i in range(1, 6))
EXTRA_CANDIDATE = "scene_terrain.xml"
BASELINE = {
    "manifest_path": "docs/evidence/P1-08/P1-08_baseline_manifest.json",
    "manifest_sha256": "2667ed37a854f85e5a7c493e7d4a8b1871a84ce95d3e3b0742801d383f8dc915",
    "identity_path": "docs/evidence/P1-08/P1-08_simulation_baseline_identity.json",
    "identity_file_sha256": "6c3563c25d45cc275db6b083f9f0fc0cc2067b48bc8f4a93dcace9f6d42817ea",
    "identity_sha256": "59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0",
}
ROOT_SEED = 20260902
GOAL = [7.0, 0.0]
INITIAL_STATE_EVIDENCE = REPO / "docs/evidence/P1-10/initial_state_probe.json"
PROBE_SOURCE = "unitree_mujoco/simulate/test/p1_10_initial_state_probe.cpp"
REQUIRED_COLLISION_ATTRS = ("contype", "conaffinity", "group", "condim", "margin", "gap", "priority")
FILE_ATTR_TAGS = ("mesh", "hfield", "texture", "skin")


def canonical(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=False).encode()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> Tuple[str, int]:
    h = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest(), path.stat().st_size


def load_verified_initial_state() -> Tuple[List[Any], str, str]:
    document = json.loads(INITIAL_STATE_EVIDENCE.read_text(encoding="utf-8"))
    qpos = document.get("qpos0")
    qpos_sha = document.get("qpos_sha256")
    if not isinstance(qpos, list) or sha256_bytes(canonical(qpos)) != qpos_sha:
        raise ValueError("initial_state_probe qpos/hash is invalid")
    if document.get("qpos_before_forward") != qpos or document.get("qpos_after_forward") != qpos:
        raise ValueError("initial_state_probe qpos is not stable across mj_forward")
    if document.get("runtime_replay") is not False or document.get("mj_step_called") is not False or document.get("ros2_started") is not False:
        raise ValueError("initial_state_probe is not an offline construction-path probe")
    source_path = REPO / PROBE_SOURCE
    if sha256_file(source_path)[0] != document.get("probe_source_sha256"):
        raise ValueError("initial_state_probe source hash drift")
    return qpos, qpos_sha, document.get("probe_source_sha256")


QPOS, QPOS_SHA256, PROBE_SHA256 = load_verified_initial_state()


def rel(path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(REPO.resolve()).as_posix()
    except ValueError:
        return resolved.as_posix()


def contained(path: Path, root: Path) -> bool:
    try:
        path.resolve().relative_to(root.resolve())
        return True
    except ValueError:
        return False


def parse_vector(raw: Optional[str]) -> Optional[List[float]]:
    if raw is None:
        return None
    return [float(token) for token in raw.split()]


def attr_fact(element: ET.Element, name: str) -> Dict[str, Any]:
    if name not in element.attrib:
        return {"xml_status": "OMITTED", "runtime_resolution": "UNKNOWN"}
    raw = element.attrib[name]
    return {"xml_status": "EXPLICIT", "raw": raw}


def resolve_asset(name: str, source_xml: Path) -> Path:
    # Current Go2 XML uses compiler meshdir=assets and the historical scene
    # hfields also live below assets.  Keep both exact file-system candidates
    # and fail closed if neither exists.
    candidates = [source_xml.parent / "assets" / name, source_xml.parent / name]
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    return candidates[0].resolve()


def resolve_closure(root_xml: Path) -> Dict[str, Any]:
    root_xml = root_xml.resolve()
    closure_root = root_xml.parent.resolve()
    failures: List[str] = []
    records: Dict[str, Dict[str, Any]] = {}
    visiting: set[Path] = set()

    def add_file(path: Path, role: str, source: Path, ref: Optional[str] = None) -> None:
        resolved = path.resolve()
        if not contained(resolved, closure_root):
            failures.append(f"path escapes closure root: {resolved}")
            return
        key = str(resolved)
        if key in records and records[key]["role"] == "included_xml":
            return
        record: Dict[str, Any] = {
            "path": rel(resolved), "role": role, "source_xml": rel(source),
            "reference": ref, "present": resolved.is_file(),
        }
        if resolved.is_file():
            try:
                digest, size = sha256_file(resolved)
                record.update({"sha256": digest, "bytes": size})
            except (OSError, ValueError) as exc:
                failures.append(f"read failed: {resolved}: {exc}")
                record.update({"sha256": None, "bytes": None})
        else:
            failures.append(f"missing file: {resolved}")
            record.update({"sha256": None, "bytes": None})
        records[key] = record

    def visit(xml_path: Path) -> None:
        xml_path = xml_path.resolve()
        if not contained(xml_path, closure_root):
            failures.append(f"include escapes closure root: {xml_path}")
            return
        if xml_path in visiting:
            failures.append(f"include cycle: {xml_path}")
            return
        if str(xml_path) in records:
            return
        add_file(xml_path, "root_xml" if xml_path == root_xml else "included_xml", xml_path)
        if not xml_path.is_file():
            return
        visiting.add(xml_path)
        try:
            text = xml_path.read_text(encoding="utf-8")
            document = ET.fromstring(text)
        except (OSError, UnicodeError, ET.ParseError) as exc:
            failures.append(f"XML parse/read failed: {xml_path}: {exc}")
            visiting.remove(xml_path)
            return
        for element in document.iter():
            if element.tag == "include" and element.get("file"):
                include = (xml_path.parent / element.get("file", "")).resolve()
                visit(include)
            if element.tag in FILE_ATTR_TAGS and element.get("file"):
                asset = resolve_asset(element.get("file", ""), xml_path)
                add_file(asset, f"{element.tag}_asset", xml_path, element.get("file"))
        visiting.remove(xml_path)

    visit(root_xml)
    files = sorted(records.values(), key=lambda item: item["path"])
    digest_input = [{"path": item["path"], "sha256": item["sha256"]} for item in files]
    closure_sha = sha256_bytes(canonical(digest_input))
    return {
        "root_xml": rel(root_xml),
        "closure_root": rel(closure_root),
        "files": files,
        "closure_sha256": closure_sha,
        "present_file_count": sum(1 for item in files if item["present"]),
        "failures": sorted(set(failures)),
    }


def geom_fact(geom: ET.Element, ordinal: int, root_xml: Path) -> Dict[str, Any]:
    type_name = geom.get("type")
    if type_name is None:
        raise ValueError("obstacle geom without explicit type")
    fact = {
        "stable_id": f"{rel(root_xml)}:worldbody/geom[{ordinal}]",
        "element_order": ordinal,
        "source_xml": rel(root_xml),
        "type": type_name,
        "pos": {"raw": geom.get("pos"), "values": parse_vector(geom.get("pos"))},
        "size": {"raw": geom.get("size"), "values": parse_vector(geom.get("size"))},
        "quat": {"raw": geom.get("quat"), "values": parse_vector(geom.get("quat"))}
        if geom.get("quat") is not None else {"xml_status": "OMITTED"},
        "collision_attributes": {name: attr_fact(geom, name) for name in REQUIRED_COLLISION_ATTRS},
    }
    fact["metadata_sha256"] = sha256_bytes(canonical(fact))
    return fact


def inventory_map(filename: str) -> Dict[str, Any]:
    root = GO2_DIR / filename
    closure = resolve_closure(root)
    runtime_initial = extract_runtime_initial_state(root)
    try:
        document = ET.parse(root).getroot()
    except (OSError, ET.ParseError) as exc:
        return {"map_file": filename, "status": "BLOCKED", "errors": [str(exc)], "closure": closure}
    worldbody = document.find("worldbody")
    if worldbody is None:
        raise ValueError(f"{filename}: missing root worldbody")
    root_geoms = list(worldbody.findall("geom"))
    obstacles: List[Dict[str, Any]] = []
    non_obstacle: List[Dict[str, Any]] = []
    for ordinal, geom in enumerate(root_geoms, start=1):
        type_name = geom.get("type")
        is_floor = geom.get("name") == "floor" or type_name == "plane"
        fact = {
            "stable_id": f"{rel(root)}:worldbody/geom[{ordinal}]",
            "element_order": ordinal,
            "source_xml": rel(root),
            "name": geom.get("name"),
            "type": type_name,
        }
        if is_floor:
            fact["classification"] = "floor_or_non_obstacle"
            non_obstacle.append(fact)
        else:
            obstacles.append(geom_fact(geom, ordinal, root))
    semantic = [{key: item[key] for key in ("type", "pos", "size", "quat", "collision_attributes")} for item in obstacles]
    return {
        "map_file": filename,
        "root_xml": rel(root),
        "root_xml_sha256": sha256_file(root)[0],
        "closure": closure,
        "model_closure_sha256": closure["closure_sha256"],
        "runtime_model_fingerprint": runtime_initial["collision_model_fingerprint"],
        "static_obstacle_count": len(obstacles),
        "static_geom_count_complexity": "count_only; not behavioral difficulty",
        "obstacles": obstacles,
        "non_obstacle_root_geoms": non_obstacle,
        "semantic_fingerprint": sha256_bytes(canonical(semantic)),
        "status": "INVENTORIED" if not closure["failures"] else "BLOCKED",
        "errors": closure["failures"],
    }


def qpos_binding(scene: Dict[str, Any]) -> str:
    return sha256_bytes(canonical({
        "scene_root_sha256": scene["root_xml_sha256"],
        "model_closure_sha256": scene["model_closure_sha256"],
        "initial_state_source": "scene_default",
        "reset_source": "mj_makeData:qpos0; no keyframe reset",
        "qpos_sha256": QPOS_SHA256,
    }))


def validate_accepted_baseline_binding() -> None:
    manifest = REPO / BASELINE["manifest_path"]
    identity = REPO / BASELINE["identity_path"]
    if sha256_file(manifest)[0] != BASELINE["manifest_sha256"]:
        raise ValueError("accepted P1-08 baseline manifest hash drift")
    if sha256_file(identity)[0] != BASELINE["identity_file_sha256"]:
        raise ValueError("accepted P1-08 baseline identity document hash drift")
    identity_document = json.loads(identity.read_text(encoding="utf-8"))
    if identity_document.get("baseline_identity_sha256") != BASELINE["identity_sha256"]:
        raise ValueError("accepted P1-08 canonical baseline identity drift")


def candidate(entry: Dict[str, Any], index: int) -> Dict[str, Any]:
    scene_file = entry["map_file"]
    scene = {
        "launch_arg": scene_file,
        "root_xml": entry["root_xml"],
        "root_xml_sha256": entry["root_xml_sha256"],
        "model_closure_sha256": entry["model_closure_sha256"],
        "runtime_model_fingerprint": entry["runtime_model_fingerprint"],
        "runtime_model_fingerprint_schema": "abs-go2-collision-model-fingerprint/v1",
        "closure_files": entry["closure"]["files"],
    }
    scenario_id = f"obstacle_test{index}"
    return {
        "schema_version": "abs-go2-deterministic-obstacle-candidate/v1",
        "extends_schema": "abs-go2-deterministic-scenario/v2",
        "scenario_id": scenario_id,
        "status": "UNSUPPORTED",
        "description": f"Historical Go2 obstacle candidate from {scene_file}; offline formalization only.",
        "baseline": BASELINE,
        "scene": scene,
        "initial_state_source": {
            "kind": "scene_default",
            "startup_path": "main.cc:PhysicsThread: mj_loadXML -> mj_makeData -> sim.Load -> mj_forward",
            "reset_source": "mj_makeData:qpos0; no keyframe reset",
            "probe_source": PROBE_SOURCE,
            "probe_source_sha256": PROBE_SHA256,
        },
        "initial_state": {"qpos": QPOS, "qpos_sha256": QPOS_SHA256, "binding_sha256": qpos_binding(scene)},
        "goal": {"world_xy_m": GOAL, "resample_goal_on_arrival": False},
        "goal_injection": {
            "source": "scripts/p1_08_baseline_capture.py:pub_input before RL",
            "baseline_config_world_xy_m": GOAL,
            "control_input": {"lx": 0.0, "ly": 0.0, "rx": 0.0},
            "effective_world_xy_m": GOAL,
        },
        "switching_mode": "stabilized_switch",
        "run_window_s": 25.0,
        "obstacle_layout": {
            "status": "DECLARED_FROM_XML_OFFLINE_ONLY",
            "source": "root XML worldbody geom declarations",
            "objects": entry["obstacles"],
            "collision_attribute_semantics": "XML omission is recorded as UNKNOWN; no runtime defaults are asserted",
        },
        "ray_authority": {
            "status": "IMPLEMENTED_SOURCE_TRACE",
            "source": "unitree_mujoco/simulate/src/unitree_sdk2_bridge.h:computeRay2d",
            "frame": "body-yaw frame; origin body x offset -0.05 m",
            "rays": {"count": 11, "angles_deg": [-45, -36, -27, -18, -9, 0, 9, 18, 27, 36, 45], "range_m": [0.1, 6.0]},
            "stored_units": "log2(distance_m)",
            "no_hit": "max range 6.0 m before log2 encoding",
            "invalid": "missing/stale/incoherent/nonfinite shared-memory frame fails closed",
            "geom_filter": "static box/cylinder/sphere/capsule/ellipsoid candidates; floor, plane/hfield/mesh, robot groups 2/3, and dynamic bodies excluded",
            "effective_runtime_status": "UNKNOWN; no obstacle capture has established source/frame validity",
        },
        "collision_authority": ({
            "status": "IMPLEMENTED / AWAITING RUNTIME VALIDATION",
            "candidate_source": "unitree_mujoco/simulate/src/obstacle_collision_authority.h:ObstacleCollisionAuthority::publish -> /mujoco_collision_v2",
            "reason": "versioned MuJoCo contact source is implemented for obstacle_test1, but no obstacle runtime has validated the binding or saved-record coverage",
        } if index == 1 else {
            "status": "UNSUPPORTED",
            "candidate_source": "unitree_sdk2_bridge.h:updateCollisionTelemetry -> /mujoco_collision",
            "reason": "this candidate has no implemented formal collision authority",
        }),
        "terminal_outcome_authority": {
            "status": "UNSUPPORTED",
            "reason": "formal runtime binding has no verified goal/fall/timeout/collision producer; outcome remains UNKNOWN",
        },
        "seed_registry": {
            "root_seed": ROOT_SEED,
            "role": "pairing/provenance_only",
            "derived_seed_registry": {},
            "random_producers": "none asserted for fixed scene/default-initial-state/fixed-goal path",
        },
        "capture_eligibility": {
            "status": "UNSUPPORTED",
            "reasons": [
                "candidate suite is separate from the accepted flat capture suite",
                "current capture preflight is not registered for this obstacle candidate",
                "collision authority runtime validation is pending" if index == 1 else "collision authority is not implemented for this candidate",
                "goal/fall/timeout terminal authority is unavailable and must remain UNKNOWN",
                "no obstacle runtime record or repeatability evidence exists",
            ],
        },
    }


def validate_ray_contract(ray: Dict[str, Any]) -> List[str]:
    """Validate the offline declaration of the existing geometric ray source."""
    failures: List[str] = []
    if ray.get("status") != "IMPLEMENTED_SOURCE_TRACE":
        failures.append("ray.status")
    if ray.get("frame") != "body-yaw frame; origin body x offset -0.05 m":
        failures.append("ray.frame")
    rays = ray.get("rays", {})
    if rays.get("count") != 11 or rays.get("range_m") != [0.1, 6.0]:
        failures.append("ray.geometry")
    if rays.get("angles_deg") != [-45, -36, -27, -18, -9, 0, 9, 18, 27, 36, 45]:
        failures.append("ray.angles")
    if ray.get("stored_units") != "log2(distance_m)":
        failures.append("ray.units")
    for key in ("no_hit", "invalid", "geom_filter", "effective_runtime_status"):
        if not isinstance(ray.get(key), str) or not ray[key]:
            failures.append(f"ray.{key}")
    return failures


def validate_candidate(document: Dict[str, Any], entry: Dict[str, Any]) -> List[str]:
    """Strictly validate one candidate against current XML inventory facts."""
    failures: List[str] = []
    if document.get("status") != "UNSUPPORTED":
        failures.append("status")
    scene = document.get("scene", {})
    for key in ("root_xml", "root_xml_sha256", "model_closure_sha256", "runtime_model_fingerprint"):
        if scene.get(key) != entry.get(key):
            failures.append(f"scene.{key}")
    if document.get("obstacle_layout", {}).get("objects") != entry.get("obstacles"):
        failures.append("obstacle_layout.objects")
    if document.get("initial_state", {}).get("qpos") != QPOS:
        failures.append("initial_state.qpos")
    if document.get("initial_state", {}).get("qpos_sha256") != QPOS_SHA256:
        failures.append("initial_state.qpos_sha256")
    if document.get("goal", {}).get("world_xy_m") != GOAL:
        failures.append("goal.world_xy_m")
    if document.get("goal_injection", {}).get("effective_world_xy_m") != GOAL:
        failures.append("goal_injection.effective_world_xy_m")
    if document.get("switching_mode") != "stabilized_switch":
        failures.append("switching_mode")
    if document.get("run_window_s") != 25.0:
        failures.append("run_window_s")
    if document.get("initial_state_source", {}).get("kind") != "scene_default":
        failures.append("initial_state_source.kind")
    if document.get("seed_registry", {}).get("root_seed") != ROOT_SEED:
        failures.append("seed_registry.root_seed")
    expected_collision_status = (
        "IMPLEMENTED / AWAITING RUNTIME VALIDATION"
        if document.get("scenario_id") == "obstacle_test1" else "UNSUPPORTED"
    )
    if document.get("collision_authority", {}).get("status") != expected_collision_status:
        failures.append("collision_authority.status")
    if document.get("terminal_outcome_authority", {}).get("status") != "UNSUPPORTED":
        failures.append("terminal_outcome_authority.status")
    failures.extend(validate_ray_contract(document.get("ray_authority", {})))
    return failures


def build_document() -> Tuple[Dict[str, Any], Dict[str, Dict[str, Any]]]:
    validate_accepted_baseline_binding()
    maps = {name: inventory_map(name) for name in MAPS}
    extra = inventory_map(EXTRA_CANDIDATE)
    alias = {
        "alias_file": "scene_obstacle.xml",
        "alias_sha256": sha256_file(GO2_DIR / "scene_obstacle.xml")[0],
        "formal_scenario_id": "obstacle_test1",
        "deduplication": "BYTE_IDENTICAL_ROOT_AND_SEMANTIC_ALIAS",
        "canonical_file": "scene_test1.xml",
    }
    scenarios = {f"obstacle_test{i}": candidate(maps[f"scene_test{i}.xml"], i) for i in range(1, 6)}
    scenario_paths = []
    for scenario_id, document in scenarios.items():
        path = f"scenarios/p1_10/{scenario_id}.json"
        scenario_paths.append({"scenario_id": scenario_id, "path": path, "sha256": sha256_bytes(canonical(document)), "status": document["status"]})
    suite = {
        "schema_version": "abs-go2-obstacle-candidate-suite/v1",
        "suite_id": "p1-10-historical-five-obstacle-candidates",
        "status": "FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW",
        "formal_scenarios": scenario_paths,
        "aliases": [alias],
        "extra_candidates": [{"scene": EXTRA_CANDIDATE, "status": "FUTURE_GENERALIZATION_CANDIDATE", "root_xml_sha256": extra["root_xml_sha256"], "model_closure_sha256": extra["model_closure_sha256"]}],
        "fixed_binding": {"variant": "stabilized", "switching_mode": "stabilized_switch", "root_seed": ROOT_SEED, "run_window_s": 25.0, "initial_state_source": "scene_default", "initial_state_reset_source": "mj_makeData:qpos0; no keyframe reset", "goal_world_xy_m": GOAL, "baseline": BASELINE},
        "contract": "Every candidate is independently XML/closure/hash bound. Missing runtime authority is UNSUPPORTED; no candidate is capture eligible.",
    }
    inventory = {
        "schema_version": "abs-go2-p1-10-historical-map-inventory/v1",
        "generated_by": "scripts/p1_10_obstacle_inventory.py",
        "offline_only": True,
        "maps": list(maps.values()),
        "deduplication": {"scene_obstacle.xml": alias},
        "extra_candidate": extra,
        "scenario_suite_path": "scenarios/p1_10/obstacle_candidate_suite_manifest.json",
        "scenario_suite": suite,
        "source_trace": {
            "obstacle_test1": {
                "geom_to_ray": "MATCH / IMPLEMENTED SOURCE TRACE; bridge geometric producer filters static supported geom types",
                "ray_to_state_rl": "MATCH / IMPLEMENTED SOURCE TRACE; versioned ray shared memory is validated and fail-closed",
                "ray_to_ra_agile": "MATCH / IMPLEMENTED SOURCE TRACE; StateRL consumes ray in 19-D RA and 61-D Agile observations",
                "ra_to_switching": "MATCH / IMPLEMENTED SOURCE TRACE; stabilized_switch applies RASwitchingLogic thresholds/hold",
                "switching_to_recovery": "MATCH / IMPLEMENTED SOURCE TRACE; enter edge invokes Recovery twist/observation path",
                "contact_to_runtime_record": "IMPLEMENTED SOURCE TRACE for obstacle_test1; versioned collision snapshot is consumed by RunRecordRecorder; no runtime validation",
                "goal_fall_timeout_to_terminal": "MISSING IMPLEMENTATION / UNSUPPORTED; current reducer preserves UNKNOWN",
                "behavioral_effectiveness": "UNKNOWN; offline trace is not obstacle runtime evidence",
            }
        },
    }
    return inventory, scenarios


def write_outputs(out_evidence: Path) -> None:
    inventory, scenarios = build_document()
    scenario_dir = REPO / "scenarios" / "p1_10"
    scenario_dir.mkdir(parents=True, exist_ok=True)
    for scenario_id, document in scenarios.items():
        (scenario_dir / f"{scenario_id}.json").write_bytes(canonical(document) + b"\n")
    suite = inventory["scenario_suite"]
    # Bind file hashes after the canonical scenario bytes are on disk.
    suite["formal_scenarios"] = [
        {"scenario_id": scenario_id, "path": f"scenarios/p1_10/{scenario_id}.json", "sha256": sha256_file(scenario_dir / f"{scenario_id}.json")[0], "status": scenarios[scenario_id]["status"]}
        for scenario_id in scenarios
    ]
    suite_path = scenario_dir / "obstacle_candidate_suite_manifest.json"
    suite_path.write_bytes(canonical(suite) + b"\n")
    inventory["scenario_suite"] = suite
    inventory["scenario_suite_sha256"] = sha256_file(suite_path)[0]
    evidence_path = out_evidence.resolve()
    evidence_path.parent.mkdir(parents=True, exist_ok=True)
    evidence_path.write_bytes(canonical(inventory) + b"\n")
    print(json.dumps({"evidence": rel(evidence_path), "suite": rel(suite_path), "scenario_count": len(scenarios)}, indent=2))


def validate_inventory_document(path: Path) -> List[str]:
    document = json.loads(path.read_text(encoding="utf-8"))
    failures: List[str] = []
    for entry in document["maps"]:
        current = inventory_map(entry["map_file"])
        for key in ("root_xml_sha256", "model_closure_sha256", "semantic_fingerprint", "static_obstacle_count", "runtime_model_fingerprint"):
            if current.get(key) != entry.get(key):
                failures.append(f"{entry['map_file']}:{key}")
        if current.get("obstacles") != entry.get("obstacles"):
            failures.append(f"{entry['map_file']}:obstacles")
        if current.get("errors"):
            failures.append(f"{entry['map_file']}:closure_errors")
    return failures


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true", help="write candidate scenarios, suite, and evidence")
    parser.add_argument("--evidence", default="docs/evidence/P1-10/historical_five_map_inventory_20260903.json")
    parser.add_argument("--validate", metavar="JSON", help="validate a previously generated inventory against current files")
    args = parser.parse_args(argv)
    if args.validate:
        failures = validate_inventory_document(Path(args.validate))
        if failures:
            print("VALIDATION_FAILED", *failures, sep="\n")
            return 1
        print("HISTORICAL_MAP_INVENTORY_VALID")
        return 0
    if args.write:
        write_outputs(REPO / args.evidence)
        return 0
    inventory, _ = build_document()
    print(json.dumps(inventory, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
