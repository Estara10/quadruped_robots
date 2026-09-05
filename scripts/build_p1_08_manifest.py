#!/usr/bin/env python3
"""P1-08 — machine-readable simulation-baseline manifest builder.

Records, with SHA-256, the exact assets, binaries, deployed policy artifacts,
effective static config keys, and source git state that constitute the current
Go2 MuJoCo + controller stack. Also computes the launched-scene model closure
hash (scene XML + recursive <include> XMLs + referenced asset files).

This is a pure offline inventory. It does not run MuJoCo, ROS, or any runtime.

Usage:
    python3 scripts/build_p1_08_manifest.py [--out baseline_manifest.json]
                                            [--probe-output model_probe.txt]

Static facts that are not derivable from a file (e.g. mjModel.opt values) come
from the optional `--probe-output` text written by the p1_08_model_probe
executable; without it those sections are recorded UNKNOWN.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

REPO = Path(__file__).resolve().parent.parent

# --- SHA-256 helpers --------------------------------------------------------

def sha256_of(path: Path) -> Tuple[str, int]:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest(), os.path.getsize(path)


def record(path: Path, role: str) -> Dict[str, Any]:
    p = path.resolve()
    if not p.exists():
        return {"path": str(p), "role": role, "sha256": None, "bytes": None,
                "present": False}
    digest, size = sha256_of(p)
    return {"path": str(p), "role": role, "sha256": digest, "bytes": size,
            "present": True}


# --- model closure resolution ----------------------------------------------

_INCLUDE_RE = re.compile(r'<include\s+file="([^"]+)"')
_MESH_RE = re.compile(r'<mesh\s+file="([^"]+)"')
_HFIELD_RE = re.compile(r'<hfield\s+file="([^"]+)"')


def is_within(path: Path, root: Path) -> bool:
    """True iff `path` (resolved) is inside `root` (resolved). Rejects escapes
    (../, absolute paths outside root, symlinks pointing outside)."""
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def resolve_closure(root_xml: Path, closure_root: Optional[Path] = None) -> Dict[str, Any]:
    """Recursively discover and hash the full MuJoCo model closure.

    - resolves <include> XMLs relative to the CURRENT XML's directory;
    - rejects any include/asset that escapes the closure root (default: the
      root XML's directory);
    - detects include CYCLES (visiting set) and MISSING files — both recorded
      as failures (fail-closed);
    - records every included XML + referenced mesh/hfield asset with its SHA-256;
    - returns a `failures` list; an empty list means the closure is complete.
    """
    root_xml = root_xml.resolve()
    if closure_root is None:
        closure_root = root_xml.parent
    closure_root = closure_root.resolve()

    failures: List[str] = []
    assets: List[Dict[str, Any]] = []
    xmls: List[Dict[str, Any]] = []
    visited: set = set()
    visiting: set = set()

    def visit(xml: Path) -> None:
        xml = xml.resolve()
        if xml in visiting:
            failures.append(f"include cycle: {xml}")
            return
        if xml in visited:
            return  # already fully processed (duplicate include is benign)
        visiting.add(xml)
        try:
            text = xml.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            failures.append(f"read failed: {xml}: {exc}")
            visiting.discard(xml)
            return
        role = "launched_scene_xml" if xml == root_xml else "included_scene_xml"
        xmls.append(record(xml, role))
        base_dir = xml.parent

        # mesh/hfield assets: meshdir="assets" resolves relative to the model dir.
        for pattern in (_MESH_RE, _HFIELD_RE):
            for name in pattern.findall(text):
                cand = (base_dir / "assets" / name)
                if not cand.exists():
                    cand = base_dir / name
                cand = cand.resolve()
                if not is_within(cand, closure_root):
                    failures.append(f"asset escape outside closure root: {cand}")
                    continue
                if cand.exists():
                    if not any(a["path"] == str(cand) for a in assets):
                        assets.append(record(cand, "mesh_asset"))
                else:
                    failures.append(f"asset missing: {cand}")
                    assets.append({"path": str(cand), "role": "mesh_asset",
                                   "sha256": None, "bytes": None, "present": False})

        # includes resolve relative to the current XML directory.
        for name in _INCLUDE_RE.findall(text):
            cand = (base_dir / name).resolve()
            if not is_within(cand, closure_root):
                failures.append(f"include escape outside closure root: {cand}")
                continue
            if not cand.exists():
                failures.append(f"include missing: {cand}")
                xmls.append({"path": str(cand), "role": "included_scene_xml",
                             "sha256": None, "bytes": None, "present": False})
                continue
            visit(cand)

        visiting.discard(xml)
        visited.add(xml)

    visit(root_xml)

    present = [a for a in assets if a["present"]] + [x for x in xmls if x["present"]]
    h = hashlib.sha256()
    for item in sorted(present, key=lambda d: d["path"]):
        h.update(item["path"].encode())
        h.update(b"\0")
        h.update(item["sha256"].encode())
    return {
        "root_xml": str(root_xml),
        "closure_root": str(closure_root),
        "xml_files": xmls,
        "asset_files": assets,
        "included_xml_files": [x for x in xmls if x["role"] == "included_scene_xml"],
        "closure_sha256": h.hexdigest(),
        "present_file_count": len(present),
        "failures": failures,
    }


# --- probe facts ------------------------------------------------------------

def parse_probe(text: str) -> Dict[str, Any]:
    facts: Dict[str, Any] = {}
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        key, _, value = line.partition("=")
        if not key:
            continue
        key = key.strip()
        value = value.strip()
        if key.startswith(("scene=", "mujoco_version=")):
            facts[key[:-1]] = value
        elif key.startswith("opt.") or key.startswith("dims.") or key.startswith("actuator."):
            top, _, rest = key.partition(".")
            facts.setdefault(top, {})[rest] = value
        elif key.startswith("step."):
            facts.setdefault("step", {})[key[5:]] = value
    return facts


# --- static config facts ----------------------------------------------------

def static_config_facts() -> Dict[str, Any]:
    g2 = REPO / "quadruped_ros2_control_humble" / "descriptions" / "unitree" / "go2_description"
    rl_control = g2 / "config" / "robot_control.yaml"
    abs_cfg = g2 / "config" / "abs" / "config.yaml"
    rec_cfg = g2 / "config" / "rec" / "config.yaml"
    mujoco_cfg = REPO / "unitree_mujoco" / "simulate" / "config.yaml"

    # Minimal YAML reads for the facts P1-08 needs; unknown keys are UNKNOWN.
    def read_yaml(path: Path) -> Dict[str, Any]:
        import yaml
        try:
            with open(path) as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            return {"__error__": str(e)}

    abs_ = read_yaml(abs_cfg)
    rec_ = read_yaml(rec_cfg)
    rl_ = read_yaml(rl_control)
    mujoco_ = read_yaml(mujoco_cfg)

    return {
        "controller_manager.update_rate_hz": {
            "value": rl_.get("controller_manager", {}).get("ros__parameters", {}).get("update_rate"),
            "source": str(rl_control),
        },
        "rl_quadruped_controller.update_rate_hz": {
            "value": rl_.get("rl_quadruped_controller", {}).get("ros__parameters", {}).get("update_rate"),
            "source": str(rl_control),
        },
        "rl_quadruped_controller.use_rl_thread": {
            "value": rl_.get("rl_quadruped_controller", {}).get("ros__parameters", {}).get("use_rl_thread"),
            "source": str(rl_control),
        },
        "rl_quadruped_controller.model_folder": {
            "value": rl_.get("rl_quadruped_controller", {}).get("ros__parameters", {}).get("model_folder"),
            "source": str(rl_control),
        },
        "abs.decimation": {"value": abs_.get("decimation"), "source": str(abs_cfg)},
        "abs.num_observations": {"value": abs_.get("num_observations"), "source": str(abs_cfg)},
        "abs.switching_mode": {"value": abs_.get("abs", {}).get("switching_mode"), "source": str(abs_cfg)},
        "abs.ra_threshold": {"value": abs_.get("abs", {}).get("ra_threshold"), "source": str(abs_cfg)},
        "abs.recovery_hold_steps": {"value": abs_.get("abs", {}).get("recovery_hold_steps"), "source": str(abs_cfg)},
        "abs.twist_*": {"value": {k: v for k, v in (abs_.get("abs", {}) or {}).items() if k.startswith("twist_")},
                        "source": str(abs_cfg)},
        "abs.goal_x": {"value": abs_.get("abs", {}).get("goal_x"), "source": str(abs_cfg)},
        "abs.goal_y": {"value": abs_.get("abs", {}).get("goal_y"), "source": str(abs_cfg)},
        "mujoco_simulate.robot_scene": {"value": mujoco_.get("robot_scene"), "source": str(mujoco_cfg)},
        "mujoco_simulate.domain_id": {"value": "duplicate_key_static_ambiguity", "source": str(mujoco_cfg)},
        "mujoco_simulate.interface": {"value": "duplicate_key_static_ambiguity", "source": str(mujoco_cfg)},
        "rec.config_model_folder": {"value": "rec", "source": str(rec_cfg)},
    }


# --- git facts --------------------------------------------------------------

def git_facts() -> Dict[str, Any]:
    def git(*args: str) -> str:
        r = subprocess.run(["git", "-C", str(REPO), *args], capture_output=True, text=True)
        return r.stdout.strip()

    commit = git("rev-parse", "HEAD")
    branch = git("rev-parse", "--abbrev-ref", "HEAD")
    dirty_paths = git("status", "--porcelain").splitlines()
    dirty_hash = None
    if dirty_paths:
        patch = subprocess.run(["git", "-C", str(REPO), "diff"], capture_output=True, text=True)
        dirty_hash = hashlib.sha256(patch.stdout.encode()).hexdigest()
    return {
        "commit": commit,
        "branch": branch,
        "dirty": bool(dirty_paths),
        "dirty_paths": dirty_paths,
        "dirty_patch_sha256": dirty_hash,
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(REPO / "docs" / "evidence" / "P1-08" / "P1-08_baseline_manifest.json"))
    ap.add_argument("--probe-output", default=None)
    args = ap.parse_args()

    g2 = REPO / "quadruped_ros2_control_humble" / "descriptions" / "unitree" / "go2_description"
    scene = REPO / "unitree_mujoco" / "unitree_robots" / "go2" / "scene_flat.xml"

    manifest: Dict[str, Any] = {
        "schema": "abs-go2-p1-08-baseline-manifest/v1",
        "task": "P1-08",
        "generated_by": "scripts/build_p1_08_manifest.py",
        "model_closure": resolve_closure(scene),
        "binaries": [
            record(REPO / "unitree_mujoco" / "simulate" / "build2" / "unitree_mujoco", "mujoco_executable"),
            record(Path("/home/lidio/Libraries/mujoco-3.3.3/lib/libmujoco.so"), "libmujoco_shared"),
            record(REPO / "quadruped_ros2_control_humble" / "install" / "rl_quadruped_controller" / "lib" / "rl_quadruped_controller" / "librl_quadruped_controller.so", "controller_plugin"),
            record(REPO / "quadruped_ros2_control_humble" / "install" / "hardware_unitree_mujoco" / "lib" / "libhardware_unitree_mujoco.so", "hardware_plugin"),
        ],
        "deployed_policy_artifacts": [
            record(g2 / "config" / "abs" / "policy.pt", "agile_policy"),
            record(g2 / "config" / "abs" / "ra_value.pt", "ra_model"),
            record(g2 / "config" / "rec" / "policy.pt", "recovery_policy"),
        ],
        "config_files": [
            record(REPO / "unitree_mujoco" / "simulate" / "config.yaml", "mujoco_simulate_config"),
            record(REPO / "quadruped_ros2_control_humble" / "descriptions" / "unitree" / "go2_description" / "config" / "robot_control.yaml", "robot_control_config"),
            record(g2 / "config" / "abs" / "config.yaml", "abs_controller_config"),
            record(g2 / "config" / "rec" / "config.yaml", "rec_controller_config"),
            record(REPO / "quadruped_ros2_control_humble" / "controllers" / "rl_quadruped_controller" / "launch" / "mujoco.launch.py", "mujoco_launch"),
            record(REPO / "scripts" / "launch_abs_sim.sh", "launch_script"),
        ],
        "effective_static_model_facts": None,
        "effective_controller_static_facts": static_config_facts(),
        "git": git_facts(),
    }

    if args.probe_output:
        text = Path(args.probe_output).read_text()
        manifest["effective_static_model_facts"] = parse_probe(text)
        manifest["model_probe_output"] = text
    else:
        manifest["effective_static_model_facts"] = {"_note": "UNKNOWN (no --probe-output given)"}

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(f"wrote {out}")
    print(f"model closure sha256: {manifest['model_closure']['closure_sha256']}")
    print(f"git commit: {manifest['git']['commit']} dirty={manifest['git']['dirty']}")


if __name__ == "__main__":
    main()
