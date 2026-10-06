#!/usr/bin/env python3
"""Mechanical checks for P1-09S lock, publication, and fault ordering."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "unitree_mujoco/simulate/src/main.cc").read_text()
BRIDGE = (ROOT / "unitree_mujoco/simulate/src/unitree_sdk2_bridge.h").read_text()

assert MAIN.count("std::lock_guard<std::recursive_mutex> md_lock(sim.mtx)") >= 2
assert MAIN.count("if (mnew) mj_deleteModel(mnew);") >= 2
physics = MAIN[MAIN.index("void PhysicsThread"):MAIN.index("void *UnitreeSdk2BridgeThread")]
assert physics.index("mj_forward(m, d);") < physics.index("markInitialReady()")
bridge = MAIN[MAIN.index("void *UnitreeSdk2BridgeThread"):MAIN.index("//------------------------------------------ main")]
assert bridge.index("initialReady()") < bridge.index("ChannelFactory")

ctor = BRIDGE[BRIDGE.index("UnitreeSDK2BridgeBase(mjModel*"):BRIDGE.index("  virtual ~UnitreeSDK2BridgeBase")]
assert ctor.index("md_lock") < ctor.index("_check_sensor")
guard_end = ctor.index("    }")
assert "std::cout" not in ctor[:guard_end]
assert "shm_open" not in ctor[:guard_end]
assert "make_shared" not in ctor[:guard_end]

run = BRIDGE[BRIDGE.index("  void run() {"):BRIDGE.index("  std::unique_ptr<HighState_t>")]
assert run.index("md_lock") < run.index("computeRay2d")
assert run.index("emitRayDiagnostics") < run.index("unlockAndPublish")
fault = BRIDGE[BRIDGE.index("if (ray_test_fault_ == \"exit\")"):BRIDGE.index("if (ray_test_active_ && ray_test_fault_ == \"freeze\")")]
assert "std::_Exit(EXIT_SUCCESS)" in fault
assert "storeRelease(&ray2d_stamp_shm_ptr_->sequence, sequence + 2U)" not in fault
print("P1-09S static contract: PASS")
