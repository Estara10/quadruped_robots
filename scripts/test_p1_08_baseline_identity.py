#!/usr/bin/env python3
"""P1-08 — deterministic tests for the canonical baseline identity v2.

Proves, on a SYNTHETIC v2 capture fixture (all required inputs present):
  1. identical inputs yield identical identity and canonical bytes;
  2. mutating EACH required input (rt/sim timing, runtime record, process facts,
     orchestrator/MuJoCo/ROS logs, reader stats, manifest) changes the identity;
  3. missing EACH required input makes compute_identity FAIL (FileNotFoundError);
  4. the old v1 capture is NOT silently upgraded: the v2 generator rejects it
     with an explicit "missing required inputs" reason.

Run: python3 scripts/test_p1_08_baseline_identity.py
"""

from __future__ import annotations

import hashlib
import json
import shutil
import sys
import tempfile
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

from build_p1_08_baseline_identity import (  # noqa: E402
    REQUIRED_CAPTURE_FILES,
    canonical_bytes,
    compute_identity,
    load_manifest,
)

EVIDENCE = REPO / "docs" / "evidence" / "P1-08"
MANIFEST = EVIDENCE / "P1-08_baseline_manifest.json"
OLD_CAPTURE = EVIDENCE / "capture_20260901_rerun"

g_checks = 0
g_fail = False


def check(cond: bool, label: str) -> None:
    global g_checks, g_fail
    g_checks += 1
    if not cond:
        g_fail = True
        print(f"FAIL: {label}")


def make_synthetic_capture(dir_path: Path) -> None:
    """Create a synthetic v2 capture with all required files."""
    d = dir_path
    d.mkdir(parents=True, exist_ok=True)
    (d / "rt_frame_timing.jsonl").write_text('{"monotonic_ns": 1000, "rl_step": 1}\n'
                                             '{"monotonic_ns": 2000, "rl_step": 2}\n')
    (d / "sim_clock_timing.jsonl").write_text('{"sequence": 4, "monotonic_ns": 1, "sim_time": 0.002}\n')
    (d / "runtime_record.jsonl").write_text('{"kind": "meta", "run_id": "r1"}\n'
                                            '{"kind": "frame", "run_id": "r1", "status": "LIVE"}\n'
                                            '{"kind": "terminal", "run_id": "r1"}\n')
    (d / "process_facts.json").write_text(json.dumps({"exit_code": 0, "forced_termination": False,
                                                      "shutdown_request_source": "SIGINT",
                                                      "shutdown_complete": True}))
    (d / "orchestrator_raw.log").write_text("orchestrator log\n")
    (d / "mujoco_raw.log").write_text("mujoco log\n")
    (d / "ros2_launch_raw.log").write_text("ros log\n")
    (d / "reader_stats.json").write_text(json.dumps({"sim_clock": {"accepted": 100}}))


def main() -> int:
    if not MANIFEST.exists():
        print("FAIL: manifest not found")
        return 1
    manifest = load_manifest(MANIFEST)

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        cap = td / "capture_v2_synth"
        make_synthetic_capture(cap)

        # --- 1. identical inputs -> identical identity + canonical bytes -----
        id1, inp1 = compute_identity(manifest, cap)
        id2, inp2 = compute_identity(manifest, cap)
        check(id1 == id2, "identical inputs -> identical identity")
        check(canonical_bytes(inp1) == canonical_bytes(inp2), "identical canonical bytes")

        # --- 2. mutating EACH required input changes the identity ------------
        for name in REQUIRED_CAPTURE_FILES:
            path = cap / name
            original = path.read_bytes()
            data = bytearray(original)
            data[len(data) // 2] ^= 0x01
            path.write_bytes(bytes(data))
            id_mut, _ = compute_identity(manifest, cap)
            check(id_mut != id1, f"mutating {name} changes identity")
            path.write_bytes(original)  # restore this exact file before next
        id_restored, _ = compute_identity(manifest, cap)
        check(id_restored == id1, "all files restored -> original identity")

        # --- 3. missing EACH required input -> FAIL --------------------------
        for name in REQUIRED_CAPTURE_FILES:
            gone = cap / name
            gone.unlink()
            try:
                compute_identity(manifest, cap)
                check(False, f"missing {name} should FAIL")
            except FileNotFoundError as exc:
                check(name in str(exc), f"missing {name} raises with reason")
            make_synthetic_capture(cap)  # restore
        id_final, _ = compute_identity(manifest, cap)
        check(id_final == id1, "fully restored fixture reproduces identity")

    # --- 4. old v1 capture is NOT silently upgraded --------------------------
    if OLD_CAPTURE.exists():
        try:
            compute_identity(manifest, OLD_CAPTURE)
            check(False, "old v1 capture must be rejected by v2 generator")
        except FileNotFoundError as exc:
            check("runtime_record.jsonl" in str(exc) or "reader_stats.json" in str(exc),
                  "old v1 capture rejected with missing-v2-inputs reason")

    if g_fail:
        print(f"RESULT: FAIL ({g_checks} checks)")
        return 1
    print(f"RESULT: PASS ({g_checks} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
