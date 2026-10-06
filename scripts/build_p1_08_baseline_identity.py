#!/usr/bin/env python3
"""P1-08 — canonical baseline identity v2 (versioned, deterministic, fail-closed).

The baseline identity is the SHA-256 of a canonical byte string:

    json.dumps(IDENTITY_INPUT, sort_keys=True, separators=(",", ":")).encode("utf-8")

IDENTITY_INPUT (schema abs-go2-p1-08-baseline-identity/v2) binds, REQUIRED:

  - raw rt_frame_timing.jsonl          - finalized runtime_record.jsonl
  - raw sim_clock_timing.jsonl         - process_facts.json
  - orchestrator_raw.log               - reader_stats.json
  - mujoco_raw.log / ros2_launch_raw.log
  - the P1-08 manifest (and its recorded model/binary/plugin/policy/config
    hashes), git commit + dirty fact, capture dir, generator version/hash.

Missing ANY required input raises FileNotFoundError — a v2 identity is never
generated from an incomplete capture. This rejects the old v1 capture (which
has no runtime_record.jsonl / reader_stats.json) with an explicit reason; the
v1 identity schema stays superseded / non-acceptance.

Everything is read from the archived inputs (manifest + capture dir), so the
identity is independently reproducible from those inputs alone.

Usage:
    python3 scripts/build_p1_08_baseline_identity.py \
        --capture-dir <v2 capture dir> \
        --manifest <manifest.json> \
        [--out identity.json]
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

GENERATOR_VERSION = "2.0"
GENERATOR = "scripts/build_p1_08_baseline_identity.py"
SCHEMA = "abs-go2-p1-08-baseline-identity/v2"

# Every one of these is REQUIRED to generate a v2 identity.
REQUIRED_CAPTURE_FILES = [
    "rt_frame_timing.jsonl",
    "sim_clock_timing.jsonl",
    "runtime_record.jsonl",
    "process_facts.json",
    "orchestrator_raw.log",
    "mujoco_raw.log",
    "ros2_launch_raw.log",
    "reader_stats.json",
]


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def canonical_identity_input(manifest: dict, capture_dir: Path) -> dict:
    """Assemble the exact canonical input dict. Raises FileNotFoundError if any
    required input is missing (fail-closed; never generates an incomplete v2
    identity)."""
    missing = [name for name in REQUIRED_CAPTURE_FILES
               if not (capture_dir / name).exists()]
    if missing:
        raise FileNotFoundError(
            f"v2 identity requires ALL inputs; missing: {missing}")

    bin_sha = {b["role"]: b["sha256"] for b in manifest["binaries"] if b["sha256"]}
    pol_sha = {a["role"]: a["sha256"] for a in manifest["deployed_policy_artifacts"] if a["sha256"]}
    cfg_sha = {c["role"]: c["sha256"] for c in manifest["config_files"] if c["sha256"]}

    out = {
        "schema": SCHEMA,
        "generator": GENERATOR,
        "generator_version": GENERATOR_VERSION,
        "generator_sha256": sha256_file(Path(__file__).resolve()),
        "manifest_sha256": sha256_file(Path(manifest.get("_manifest_path", "")).resolve())
            if manifest.get("_manifest_path") else None,
        "rt_frame_timing_sha256": sha256_file(capture_dir / "rt_frame_timing.jsonl"),
        "sim_clock_timing_sha256": sha256_file(capture_dir / "sim_clock_timing.jsonl"),
        "runtime_record_sha256": sha256_file(capture_dir / "runtime_record.jsonl"),
        "process_facts_sha256": sha256_file(capture_dir / "process_facts.json"),
        "orchestrator_log_sha256": sha256_file(capture_dir / "orchestrator_raw.log"),
        "mujoco_log_sha256": sha256_file(capture_dir / "mujoco_raw.log"),
        "ros2_launch_log_sha256": sha256_file(capture_dir / "ros2_launch_raw.log"),
        "reader_stats_sha256": sha256_file(capture_dir / "reader_stats.json"),
        "capture_dir": capture_dir.name,
        "git_commit": manifest["git"]["commit"],
        "git_dirty": bool(manifest["git"]["dirty"]),
        "model_closure_sha256": manifest["model_closure"]["closure_sha256"],
        "mujoco_binary_sha256": bin_sha.get("mujoco_executable"),
        "controller_plugin_sha256": bin_sha.get("controller_plugin"),
        "hardware_plugin_sha256": bin_sha.get("hardware_plugin"),
        "agile_policy_sha256": pol_sha.get("agile_policy"),
        "ra_model_sha256": pol_sha.get("ra_model"),
        "recovery_policy_sha256": pol_sha.get("recovery_policy"),
        "robot_control_config_sha256": cfg_sha.get("robot_control_config"),
        "abs_controller_config_sha256": cfg_sha.get("abs_controller_config"),
        "mujoco_simulate_config_sha256": cfg_sha.get("mujoco_simulate_config"),
    }
    return out


def canonical_bytes(input_dict: dict) -> bytes:
    return json.dumps(input_dict, sort_keys=True, separators=(",", ":")).encode("utf-8")


def compute_identity(manifest: dict, capture_dir: Path) -> tuple:
    """Return (identity_hex, canonical_input_dict). Raises on missing inputs."""
    input_dict = canonical_identity_input(manifest, capture_dir)
    identity = hashlib.sha256(canonical_bytes(input_dict)).hexdigest()
    return identity, input_dict


def load_manifest(manifest_path: Path) -> dict:
    m = json.loads(manifest_path.read_text())
    m["_manifest_path"] = str(manifest_path)
    return m


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--capture-dir", required=True)
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    manifest = load_manifest(Path(args.manifest))
    capture_dir = Path(args.capture_dir)
    identity, input_dict = compute_identity(manifest, capture_dir)

    result = {
        "schema": SCHEMA,
        "generator_version": GENERATOR_VERSION,
        "baseline_identity_sha256": identity,
        "canonical_encoding": "json.dumps(input, sort_keys=True, separators=(',', ':')).encode('utf-8'); sha256",
        "canonical_input": input_dict,
    }
    if args.out:
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        print(f"wrote {out}")
    print(f"baseline_identity_sha256: {identity}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
