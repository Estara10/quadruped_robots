#!/usr/bin/env python3
"""Bounded production C++ publisher -> production Python reader checks for S6-11."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import select
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import abs_collision  # noqa: E402
import abs_scene  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--probe", type=Path, required=True)
    parser.add_argument("--layouts", type=Path, default=ROOT / "docs/thesis_project/designs/S6-11/layouts_s6_11.json")
    parser.add_argument("--scene-id-prefix", default="s611_",
                        help="prefix selecting catalog scenes from the supplied layouts")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    doc = json.loads(args.layouts.read_text(encoding="utf-8"))
    scenes = [s for s in doc["scenes"] if s["scene_id"].startswith(args.scene_id_prefix)]
    if not scenes:
        raise RuntimeError(f"no scenes selected by prefix {args.scene_id_prefix!r}")
    results = []
    for index, scene in enumerate(scenes):
        spec = abs_scene.resolve_scene(scene["filename"])
        if spec.scene_id != scene["scene_id"] or spec.obstacle_count != scene["obstacle_count"]:
            raise RuntimeError(f"catalog mismatch before publish: {scene['scene_id']}")
        capture = "p1-10-capture-" + uuid.uuid4().hex
        shm_name = f"/abs_s611_{os.getpid()}_{index}_{uuid.uuid4().hex[:8]}"
        env = os.environ.copy()
        env.update({
            "ABS_P1_10_SCENARIO_ID": spec.scene_id,
            "ABS_P1_10_ROOT_XML_SHA256": spec.root_sha256,
            "ABS_P1_10_MODEL_CLOSURE_SHA256": spec.closure_sha256,
            "ABS_P1_10_CAPTURE_ID": capture,
            "ABS_P1_10_EXPECTED_MODEL_FINGERPRINT": spec.model_fingerprint,
        })
        proc = subprocess.Popen(
            [str(args.probe), "--publish", str(spec.path.parent), spec.filename, shm_name],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=False, bufsize=0, env=env)
        ready = None
        try:
            deadline = time.monotonic() + 8.0
            startup_lines = []
            pending = b""
            while time.monotonic() < deadline and ready is None:
                readable, _, _ = select.select([proc.stdout], [], [], max(0.0, deadline - time.monotonic()))
                if not readable:
                    break
                pending += os.read(proc.stdout.fileno(), 65536)
                while b"\n" in pending:
                    raw_line, pending = pending.split(b"\n", 1)
                    line = raw_line.decode("utf-8", errors="replace").strip()
                    startup_lines.append(line)
                    if line.startswith("READY "):
                        ready = line
                        break
                if proc.poll() is not None:
                    break
            if ready is None:
                err = proc.stderr.read(4096).decode("utf-8", errors="replace") if proc.poll() is not None else "publisher did not report READY"
                raise RuntimeError(f"publisher readiness failed: {scene['scene_id']} lines={startup_lines!r} stderr={err!r}")
            match = dict(token.split("=", 1) for token in ready.split()[1:] if "=" in token)
            if match.get("id") != spec.scene_id or int(match.get("obstacle_count", "-1")) != spec.obstacle_count:
                raise RuntimeError(f"publisher binding mismatch: {scene['scene_id']} {ready}")
            path = f"/dev/shm/{shm_name.lstrip('/')}"
            raw, read_info = abs_collision.read_collision_snapshot_diagnostic(path, max_attempts=80, retry_interval_s=0.0005)
            now_ns = time.monotonic_ns()
            status, snapshot, rejection = abs_collision.classify_snapshot_diagnostic(
                raw, now_ns, expected_capture_id=capture,
                expected_fingerprint=spec.model_fingerprint,
                expected_scene_binding=spec.binding())
            if status is not abs_collision.CollisionStatus.LIVE or snapshot is None:
                detail = None if snapshot is None else {
                    "authoritative": snapshot.authoritative, "invalid_reason": snapshot.invalid_reason,
                    "scenario_id": snapshot.scenario_id,
                    "runtime_fingerprint": snapshot.runtime_model_fingerprint,
                    "foot_identity_valid": snapshot.foot_identity_valid,
                }
                raise RuntimeError(f"reader rejected {scene['scene_id']}: status={status.value} read={read_info} rejection={rejection} snapshot={detail}")
            if (snapshot.scenario_id != spec.scene_id or snapshot.physics_step != 1 or
                    snapshot.authoritative != 1 or snapshot.runtime_model_fingerprint != spec.model_fingerprint):
                raise RuntimeError(f"snapshot fields mismatch: {scene['scene_id']} status={status.value}")
            proc.stdin.write(b"\n")
            proc.stdin.flush()
            out, err = proc.communicate(timeout=4.0)
            out_text = out.decode("utf-8", errors="replace")
            err_text = err.decode("utf-8", errors="replace")
            if proc.returncode != 0 or f"CLEANED shm={shm_name}" not in out_text:
                raise RuntimeError(f"publisher cleanup failed: {scene['scene_id']} rc={proc.returncode} out={out_text!r} err={err_text!r}")
            if os.path.exists(path):
                raise RuntimeError(f"temporary shm remains: {path}")
            results.append({"scene_id": spec.scene_id, "filename": spec.filename,
                            "obstacle_count": spec.obstacle_count, "reader_status": status.value,
                            "reader_sequence": snapshot.sequence, "physics_step": snapshot.physics_step,
                            "runtime_model_fingerprint": snapshot.runtime_model_fingerprint,
                            "mujoco_version": match.get("mujoco_version"), "cleanup_exit_code": proc.returncode,
                            "shm_removed": True})
            print(f"LIVE {scene['scene_id']} geoms={spec.obstacle_count} seq={snapshot.sequence}", flush=True)
        except Exception:
            if proc.poll() is None:
                try:
                    if proc.stdin and not proc.stdin.closed:
                        proc.stdin.write(b"\n"); proc.stdin.flush()
                    proc.wait(timeout=2.0)
                except Exception:
                    proc.kill(); proc.wait(timeout=2.0)
            try:
                os.unlink(f"/dev/shm/{shm_name.lstrip('/')}")
            except FileNotFoundError:
                pass
            raise
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"publisher": str(args.probe), "reader": "scripts/abs_collision.py",
                                      "scene_count": len(results), "results": results},
                                     indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
