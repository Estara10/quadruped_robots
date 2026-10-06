#!/usr/bin/env python3
"""P1-08 — Python tests for the v2 sim-clock seqlock reader.

Two layers:

1) Decode/rejection unit tests (synthetic bytes via struct.pack). These only
   exercise `decode_sim_clock_snapshot` / `read_sim_clock_from` and are labelled
   DECODE/REJECTION-ONLY: they do NOT prove the C++ writer's output format.

2) Real C++ -> Python integration over actual shared memory:
   - the C++ helper `p1_08_sim_clock_bridge` (built by CMake, no MuJoCo)
     creates a REAL `SimClockWriter` on a unique temporary shm and publishes
     known {monotonic_ns, sim_time};
   - `read_sim_clock(shm_path=...)` opens that shm and reads back the same
     valid v2 snapshot (forward integration, no Python fabrication);
   - negative fail-closed cases (odd / wrong version / NaN / zero monotonic)
     are applied to the real C++-created shm and must be rejected.

Run: python3 scripts/test_p1_08_sim_clock.py
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

from p1_08_baseline_capture import (  # noqa: E402
    SIM_CLOCK_MAGIC,
    SIM_CLOCK_SHM,
    SIM_CLOCK_SIZE,
    SIM_CLOCK_STRUCT,
    SIM_CLOCK_VERSION,
    decode_sim_clock_snapshot,
    read_sim_clock,
    read_sim_clock_from,
)

g_checks = 0
g_fail = False


def check(cond: bool, label: str) -> None:
    global g_checks, g_fail
    g_checks += 1
    if not cond:
        g_fail = True
        print(f"FAIL: {label}")


def snap(seq, version=SIM_CLOCK_VERSION, mono=1000, sim=0.002, magic=SIM_CLOCK_MAGIC):
    return SIM_CLOCK_STRUCT.pack(magic, version, seq, mono, sim)


def write_shm_at(path: str, data: bytes) -> None:
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
    os.ftruncate(fd, SIM_CLOCK_SIZE)
    os.write(fd, data)
    os.close(fd)


def find_bridge() -> Path:
    cands = [
        REPO / "unitree_mujoco" / "simulate" / "build2" / "p1_08_sim_clock_bridge",
    ]
    for c in cands:
        if c.exists():
            return c
    raise RuntimeError("p1_08_sim_clock_bridge not built (run the simulator build)")


def main() -> int:
    # ================= DECODE / REJECTION-ONLY (synthetic bytes) ============
    check(decode_sim_clock_snapshot(snap(2)) == (2, 1000, 0.002), "decode: valid even accepted")
    check(decode_sim_clock_snapshot(snap(0)) is None, "decode: sequence 0 rejected")
    check(decode_sim_clock_snapshot(snap(1)) is None, "decode: odd (in-progress) rejected")
    check(decode_sim_clock_snapshot(snap(4, version=SIM_CLOCK_VERSION - 1)) is None,
          "decode: old version rejected (no silent fallback)")
    check(decode_sim_clock_snapshot(snap(4, version=99)) is None, "decode: unknown version rejected")
    check(decode_sim_clock_snapshot(snap(4, mono=0)) is None, "decode: zero monotonic rejected")
    check(decode_sim_clock_snapshot(snap(4, sim=float("nan"))) is None, "decode: NaN sim_time rejected")
    check(decode_sim_clock_snapshot(snap(4, sim=float("inf"))) is None, "decode: +Inf sim_time rejected")
    check(decode_sim_clock_snapshot(snap(4, sim=float("-inf"))) is None, "decode: -Inf sim_time rejected")
    check(decode_sim_clock_snapshot(snap(4, magic=SIM_CLOCK_MAGIC ^ 0xFF)) is None,
          "decode: wrong magic rejected")
    check(decode_sim_clock_snapshot(snap(4)[:SIM_CLOCK_SIZE - 1]) is None, "decode: wrong size rejected")

    # ---- read_sim_clock_from: seqlock loop (DECODE/REJECTION-ONLY) ----------
    stable = iter([snap(2), snap(2), snap(2)])
    check(read_sim_clock_from(lambda: next(stable)) == (2, 1000, 0.002),
          "loop: stable even snapshot accepted")
    flaky = iter([snap(1), snap(2), snap(2)])
    check(read_sim_clock_from(lambda: next(flaky)) == (2, 1000, 0.002),
          "loop: odd-then-valid retried and accepted")
    torn = iter([snap(2), snap(4)] * 6)
    check(read_sim_clock_from(lambda: next(torn), max_attempts=5) is None,
          "loop: sequence changed during copy rejected (retry exhausts)")
    odd_all = iter([snap(1)] * 12)
    check(read_sim_clock_from(lambda: next(odd_all), max_attempts=5) is None,
          "loop: odd-only snapshot never accepted")
    nan_all = iter([snap(4, sim=float("nan"))] * 12)
    check(read_sim_clock_from(lambda: next(nan_all), max_attempts=5) is None,
          "loop: NaN snapshot never accepted")

    # ================= REAL C++ -> PYTHON shared-memory integration =========
    bridge = find_bridge()
    # The C++ bridge uses the shm_open NAME (no /dev/shm prefix); the Python
    # reader opens the FILE PATH (/dev/shm/<name>).
    shm_open_name = f"mujoco_sim_clock_itest_{os.getpid()}"
    shm_path = f"/dev/shm/{shm_open_name}"
    try:
        os.unlink(shm_path)
    except OSError:
        pass

    # --- positive: real C++ SimClockWriter -> Python read_sim_clock ---------
    r = subprocess.run([str(bridge), "--shm", shm_open_name, "--n", "3"],
                       capture_output=True, text=True, timeout=30)
    check(r.returncode == 0, "bridge: exits 0")
    last_line = next((ln for ln in r.stdout.splitlines() if ln.startswith("LAST ")), None)
    check(last_line is not None, "bridge: prints LAST line")
    _, mono_s, sim_s = last_line.split()
    expected_mono = int(mono_s)
    expected_sim = float(sim_s)
    check(expected_mono == 3000 and expected_sim == 0.006,
          f"bridge: published expected LAST 3000 0.006 (got {expected_mono} {expected_sim})")

    got = read_sim_clock(shm_path=shm_path)
    check(got is not None, "real C++->Python: valid v2 snapshot read back (not None)")
    if got is not None:
        _, mono, sim = got
        check(mono == expected_mono, "real C++->Python: monotonic_ns matches C++ writer")
        check(abs(sim - expected_sim) < 1e-12, "real C++->Python: sim_time matches C++ writer")

    # --- negative: fail-closed on the REAL C++-created shm ------------------
    # odd sequence
    write_shm_at(shm_path, snap(1))
    check(read_sim_clock(shm_path=shm_path) is None, "real shm: odd sequence rejected")
    # wrong / old version
    write_shm_at(shm_path, snap(4, version=SIM_CLOCK_VERSION - 1))
    check(read_sim_clock(shm_path=shm_path) is None, "real shm: old version rejected")
    write_shm_at(shm_path, snap(4, version=99))
    check(read_sim_clock(shm_path=shm_path) is None, "real shm: unknown version rejected")
    # NaN / Inf
    write_shm_at(shm_path, snap(4, sim=float("nan")))
    check(read_sim_clock(shm_path=shm_path) is None, "real shm: NaN sim_time rejected")
    write_shm_at(shm_path, snap(4, sim=float("inf")))
    check(read_sim_clock(shm_path=shm_path) is None, "real shm: +Inf sim_time rejected")
    write_shm_at(shm_path, snap(4, sim=float("-inf")))
    check(read_sim_clock(shm_path=shm_path) is None, "real shm: -Inf sim_time rejected")
    # zero monotonic
    write_shm_at(shm_path, snap(4, mono=0))
    check(read_sim_clock(shm_path=shm_path) is None, "real shm: zero monotonic rejected")
    # construction (init) state: odd marker + NaN payload + zero mono
    write_shm_at(shm_path, SIM_CLOCK_STRUCT.pack(SIM_CLOCK_MAGIC, SIM_CLOCK_VERSION, 1, 0, float("nan")))
    check(read_sim_clock(shm_path=shm_path) is None, "real shm: construction (init) state rejected")
    # re-valid snapshot on the same C++ shm name
    write_shm_at(shm_path, snap(4))
    check(read_sim_clock(shm_path=shm_path) == (4, 1000, 0.002),
          "real shm: re-valid snapshot accepted")

    try:
        os.unlink(shm_path)
    except OSError:
        pass

    # production shm must remain untouched (never created by this test)
    check(not os.path.exists(SIM_CLOCK_SHM), "production /mujoco_sim_clock not touched")

    if g_fail:
        print(f"RESULT: FAIL ({g_checks} checks)")
        return 1
    print(f"RESULT: PASS ({g_checks} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
