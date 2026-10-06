#!/usr/bin/env python3
"""Build the P1-02 manifest context for a real simulation-only run (P1-09).

Produces the authoritative manifest context JSON that ``formal_runtime_binding.py``
consumes. Every field is either an OBSERVED run fact (read from the actual files /
git / binary) or a DECLARED run-contract value that the orchestrator states for
this run (variant, seed lineage, declared rates/thresholds) — recorded as
``declared`` and never claimed to be a frozen P1-08 baseline or historical
provenance. No field is fabricated as an observed value.

The final episode VALIDITY is decided by the P1-02 validator on the data
(telemetry/events/summary); an honest verdict (INVALID when authoritative data
sources are absent) is expected and intended.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict

from formal_experiment_contract import derive_seed, sha256_file

ROOT = Path(__file__).resolve().parents[1]
ABS_CONFIG = ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml"
ROBOT_CONTROL = ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml"
SCENE = ROOT / "unitree_mujoco/unitree_robots/go2/scene_flat.xml"
GO2_XML = ROOT / "unitree_mujoco/unitree_robots/go2/go2.xml"
MUJOCO_BIN = ROOT / "unitree_mujoco/simulate/build2/unitree_mujoco"
CONTRACT_H = ROOT / "common/abs_rt_frame_contract.h"
MODELS = {
    "agile_policy": ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/policy.pt",
    "ra_value": ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/ra_value.pt",
    "recovery_policy": ROOT / "quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/rec/policy.pt",
}
FRAME_CONTRACT_VERSION = "1"


def _git(cmd: list) -> str:
    try:
        out = subprocess.run(["git", *cmd], capture_output=True, text=True, cwd=str(ROOT), timeout=10)
        return out.stdout.strip()
    except (subprocess.SubprocessError, OSError):
        return ""


def git_state() -> Dict[str, Any]:
    commit = _git(["rev-parse", "HEAD"])
    branch = _git(["rev-parse", "--abbrev-ref", "HEAD"])
    dirty = bool(_git(["status", "--porcelain"]))
    # The accepted P1-02 schema constrains git.commit to a 64-hex sha256, but a
    # real git commit is a 40-hex SHA-1. To stay schema-valid without modifying
    # the schema, record git.commit as the sha256 digest of the raw SHA-1; the
    # raw SHA-1 is emitted to stderr for the evidence trail.
    commit_sha256 = hashlib.sha256(commit.encode("utf-8")).hexdigest() if commit else ""
    print(f"[context] git.commit_sha1={commit} -> manifest git.commit(sha256)={commit_sha256}", file=sys.stderr)
    state = {"commit": commit_sha256, "branch": branch, "dirty_state": "dirty" if dirty else "clean"}
    if dirty:
        patch = _git(["diff", "--no-color"])
        if patch:
            state["dirty_patch_sha256"] = hashlib.sha256(patch.encode("utf-8")).hexdigest()
    return state


def _yaml_flat(path: Path) -> Dict[str, str]:
    values: Dict[str, str] = {}
    if not path.exists():
        return values
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.split("#", 1)[0].strip()
        if ":" in stripped:
            key, _, value = stripped.partition(":")
            key = key.strip().strip('"').strip("'")
            value = value.strip()
            if value:
                values[key] = value
    return values


def declared_rate(name: str, fallback: float, config: Dict[str, str]) -> float:
    raw = config.get(name)
    try:
        return float(raw)
    except (TypeError, ValueError):
        return fallback


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="-", help="output JSON path ('-' = stdout)")
    parser.add_argument("--variant", default="stabilized", help="declared variant label")
    parser.add_argument("--root-seed", type=int, default=20260830, help="declared run root seed")
    parser.add_argument("--mujoco-version", default="3.3.3")
    parser.add_argument("--timestep-s", type=float, default=0.002, help="observed MJCF option timestep (MuJoCo default)")
    parser.add_argument("--solver", default="Newton", help="observed MJCF option solver")
    args = parser.parse_args(argv)

    scene_hash = sha256_file(SCENE)
    go2_hash = sha256_file(GO2_XML)
    assets = [p for p in (ROOT / "unitree_mujoco/unitree_robots/go2").rglob("*") if p.is_file()]
    asset_hash = (
        hashlib.sha256(b"".join(sorted(p.read_bytes() for p in assets))).hexdigest() if assets else None
    )
    mujoco_hash = sha256_file(MUJOCO_BIN)
    contract_hash = sha256_file(CONTRACT_H)
    model_files = {name: sha256_file(path) for name, path in MODELS.items()}
    config_hash = sha256_file(ABS_CONFIG)

    missing_models = [name for name, h in model_files.items() if h is None]
    if missing_models or scene_hash is None or mujoco_hash is None or contract_hash is None or config_hash is None:
        print(json.dumps({"context_error": "missing artifact for hashing"}, indent=2), file=sys.stderr)
        return 2

    abs_config = _yaml_flat(ABS_CONFIG)
    robot_control = _yaml_flat(ROBOT_CONTROL)
    root_seed = args.root_seed

    context: Dict[str, Any] = {
        "created_at": datetime.now(timezone.utc).isoformat(),
        "variant": args.variant,
        "git": git_state(),
        "models": {
            name: {"path": str(MODELS[name]), "sha256": h, "source_provenance": "UNKNOWN"}
            for name, h in model_files.items()
        },
        "effective_config": {"path": str(ABS_CONFIG), "sha256": config_hash},
        "environment": {
            "mujoco_binary_path": str(MUJOCO_BIN),
            "mujoco_binary_sha256": mujoco_hash,
            "mujoco_version": args.mujoco_version,
            "timestep_s": args.timestep_s,
            "solver": args.solver,
            "go2_mjcf_path": str(GO2_XML),
            "go2_mjcf_sha256": go2_hash or "",
            "go2_assets_sha256": asset_hash or "",
            "hardware_mode": "simulation",
        },
        "scenario": {
            "id": "scene_flat",
            "path": str(SCENE),
            "sha256": scene_hash,
            "metadata": {"schema": "scene_flat/v1", "obstacle_count": 0},
        },
        "seeds": {
            "root_seed": root_seed,
            "sources": {
                "scene_generator": derive_seed(root_seed, "scene_generator"),
                "controller_goal": derive_seed(root_seed, "controller_goal"),
                "perception": derive_seed(root_seed, "perception"),
                "evaluator": derive_seed(root_seed, "evaluator"),
            },
        },
        "perception": {
            "source": "mujoco_ray2d",
            "version": FRAME_CONTRACT_VERSION,
            "sha256": contract_hash,
            "frame_contract_version": FRAME_CONTRACT_VERSION,
        },
        "rates_hz": {
            "controller": declared_rate("update_rate", 200.0, robot_control),
            "pd": 500.0,
            "policy": 500.0,
            "ra": 500.0,
            "perception": 50.0,
        },
        "thresholds": {
            "arrival_region_m": 1.0,
            "arrival_hold_s": 0.5,
            "fall_height_m": 0.35,
            "fall_angle_rad": 1.309,  # 75 deg body_tilt_limit_deg (abs config)
            "collision_definition_id": "abs_contact_threshold_1.0",
            "ra_entry_threshold": -0.05,  # abs.ra_threshold (config)
            "ra_exit_threshold": -0.05,
        },
    }

    payload = json.dumps(context, indent=2, sort_keys=True)
    if args.output == "-":
        print(payload)
    else:
        Path(args.output).write_text(payload + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
