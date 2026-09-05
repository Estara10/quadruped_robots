#!/usr/bin/env python3
"""Mechanical checks for the P1-09R source-level m/d contract."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "unitree_mujoco/simulate/src/main.cc").read_text()
BRIDGE = (ROOT / "unitree_mujoco/simulate/src/unitree_sdk2_bridge.h").read_text()

assert MAIN.count("bridge_lifecycle->reloadAllowed()") >= 2
assert MAIN.count("if (mnew) mj_deleteModel(mnew);") >= 2
assert "bridge_stop_request.setMdMutex(&sim->mtx);" in MAIN
assert "std::lock_guard<std::recursive_mutex> md_lock(*md_mutex_);" in BRIDGE

run_start = BRIDGE.index("  void run() {")
run_end = BRIDGE.index("  std::unique_ptr<HighState_t>", run_start)
run = BRIDGE[run_start:run_end]
assert run.index("lowcmd->mutex_") < run.index("md_lock")
assert run.index("md_lock") < run.index("unlockAndPublish")
assert run.count("std::lock_guard<std::recursive_mutex>") == 1
guard_end = run.index("    emitRayDiagnostics();")
guard_text = run[run.index("md_lock"):guard_end]
assert "std::cout" not in guard_text
assert "unlockAndPublish" not in guard_text
print("P1-09R static contract: PASS")
