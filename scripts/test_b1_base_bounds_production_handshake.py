#!/usr/bin/env python3
"""Bounded real MuJoCo production publisher -> production Python reader check."""
from __future__ import annotations

import os
import subprocess
import sys
import time
import uuid
from pathlib import Path

from abs_base_bounds import read_snapshot
from abs_scene import resolve_scene

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / "unitree_mujoco/simulate/build2/b1_base_bounds_probe"


def run() -> None:
    scene = resolve_scene("dev25.xml")
    token = f"abs_b1_{os.getpid()}_{uuid.uuid4().hex[:10]}"
    collision_name = "/" + token + "_collision"
    bounds_name = "/" + token + "_bounds"
    shm_path = Path("/dev/shm") / bounds_name.lstrip("/")
    env = os.environ.copy()
    env.update({
        "ABS_BASE_BOUNDS_SHM_NAME": bounds_name,
        "ABS_P1_10_SCENARIO_ID": scene.scene_id,
        "ABS_P1_10_ROOT_XML_SHA256": scene.root_sha256,
        "ABS_P1_10_MODEL_CLOSURE_SHA256": scene.closure_sha256,
        "ABS_P1_10_CAPTURE_ID": "p1-10-capture-" + uuid.uuid4().hex,
        "ABS_P1_10_EXPECTED_MODEL_FINGERPRINT": scene.model_fingerprint,
        "ABS_FIELD_BOUNDS_X_MIN": "-0.5", "ABS_FIELD_BOUNDS_X_MAX": "8.0",
        "ABS_FIELD_BOUNDS_Y_MIN": "-3.0", "ABS_FIELD_BOUNDS_Y_MAX": "3.0",
    })
    capture = env["ABS_P1_10_CAPTURE_ID"]
    proc = subprocess.Popen([str(SIM), str(ROOT / "unitree_mujoco/unitree_robots/go2"),
                             scene.filename, collision_name], stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True, env=env)
    try:
        stdout, stderr = proc.communicate(timeout=10)
        if proc.returncode != 0 or "PUBLISHED " not in stdout:
            raise RuntimeError(f"production publisher failed: rc={proc.returncode} stdout={stdout!r} stderr={stderr!r}")
        result = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            result = read_snapshot(shm_path, expected_scene_id=scene.scene_id,
                                   expected_capture_id=capture,
                                   expected_fingerprint=scene.model_fingerprint)
            if result.get("status") == "LIVE":
                break
            time.sleep(0.01)
        assert result and result.get("status") == "LIVE", result
        snap = result["snapshot"]
        assert snap["bounds_valid"] == 1 and snap["physics_coverage_complete"] == 1, snap
        assert snap["physics_step"] == 3 and snap["scenario_id"] == scene.scene_id, snap
        assert snap["currently_outside"] == 0 and snap["outside_physics_steps"] == 1, snap
        assert snap["first_outside_step"] == 2 and snap["last_outside_step"] == 2, snap
        assert snap["capture_id"] == capture and snap["runtime_model_fingerprint"] == scene.model_fingerprint
        print("B1 production PhysicsLoop publisher -> Python reader: PASS")
        print(f"scene={scene.scene_id} final_step={snap['physics_step']} latched_between_reads={snap['outside_physics_steps']}")
        print(f"producer_exit={proc.returncode}; test SHM cleanup is parent-owned")
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait(timeout=3)
        try:
            shm_path.unlink()
        except FileNotFoundError:
            pass
        collision_path = Path("/dev/shm") / collision_name.lstrip("/")
        try:
            collision_path.unlink()
        except FileNotFoundError:
            pass


if __name__ == "__main__":
    run()
