#!/usr/bin/env python3
"""Consolidated P1-09W source-level lifecycle self-check."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "unitree_mujoco/simulate/src/main.cc").read_text()
BRIDGE = (ROOT / "unitree_mujoco/simulate/src/unitree_sdk2_bridge.h").read_text()
LIFECYCLE = (ROOT / "unitree_mujoco/simulate/src/bridge_lifecycle.h").read_text()
JOINABLE = (ROOT / "unitree_mujoco/simulate/src/joinable_thread.h").read_text()
U_TEST = (ROOT / "unitree_mujoco/simulate/test/p1_09u_bridge_lifecycle_state.cpp").read_text()

# One state variable and guarded transitions; no legacy bypass API.
assert "enum class State { INITIAL, RESERVED, ACTIVE, STOPPING, TERMINAL };" in LIFECYCLE
assert "mutable std::mutex state_mutex_;" in LIFECYCLE
assert "std::atomic<bool> bridge_active_" not in LIFECYCLE
assert "std::atomic<bool> terminal_" not in LIFECYCLE
assert "if (state_ != State::STOPPING) return false;" in LIFECYCLE
assert "markBridgeTerminal" not in LIFECYCLE
assert "if (state_ == State::RESERVED || state_ == State::ACTIVE)" in LIFECYCLE
assert "state_ == State::INITIAL" not in LIFECYCLE[LIFECYCLE.index("bool beginStop"):
                                                   LIFECYCLE.index("bool completeTerminal")]

# Main owns terminal completion and releases m/d only after both joins.
assert MAIN.index("bridge_stop_request.reserveBridge()") < MAIN.index("std::thread unitree_thread")
shutdown = MAIN[MAIN.index("bridge_stop_request.beginStop();"):MAIN.rindex("  return 0;\n}")]
assert shutdown.index("unitree_thread.join();") < shutdown.index("completeTerminal()")
assert shutdown.index("completeTerminal()") < shutdown.index("physicsthreadhandle.join();")
assert shutdown.index("physicsthreadhandle.join();") < shutdown.index("mj_deleteData(d);")
assert "markBridgeTerminal" not in MAIN

# Constructor-time model reads are guarded; scene metadata is snapshotted before output.
worker = MAIN[MAIN.index("void *UnitreeSdk2BridgeThread"):
              MAIN.index("//------------------------------------------ main")]
guard_start = worker.index("std::lock_guard<std::recursive_mutex> md_lock")
guard_end = worker.index("\n  }", guard_start)
guard = worker[guard_start:guard_end]
assert "mj_name2id(m" in guard and "model_nu = m->nu" in guard
assert "std::cerr" not in guard and "ChannelFactory" not in guard
scene = BRIDGE[BRIDGE.index("void printSceneInformation()"):BRIDGE.index("protected:")]
assert "SceneInfoSnapshot" in scene
assert "std::lock_guard<std::recursive_mutex> md_lock" in scene
assert scene.index("md_lock") < scene.index("std::cout")
snapshot_unlock = scene.index("    }\n\n    auto printObjects")
assert scene.index("std::cout") > snapshot_unlock
assert "std::cout" not in scene[scene.index("md_lock"):snapshot_unlock]

# Existing safety invariants and worker lifecycle remain present.
assert "std::_Exit(EXIT_SUCCESS)" in BRIDGE
assert "detach(" not in JOINABLE
assert "Deterministic linearization A" in U_TEST
assert "Deterministic linearization B" in U_TEST
assert "assert(activate_succeeded.load(std::memory_order_acquire));" in U_TEST
assert "assert(!activate_succeeded.load(std::memory_order_acquire));" in U_TEST

# Reload and fault-path regressions from R/S/T remain mechanically present.
assert MAIN.count("if (dnew && bridge_lifecycle->reloadAllowed())") == 2
assert MAIN.count("mj_deleteData(dnew);") >= 2
assert MAIN.count("mj_deleteModel(mnew);") >= 2
for start in [i for i in range(len(MAIN))
              if MAIN.startswith("std::lock_guard<std::recursive_mutex> md_lock(sim.mtx)", i)]:
    end = MAIN.find("\n          }", start)
    assert end >= 0
    assert "std::cerr" not in MAIN[start:end]
run = BRIDGE[BRIDGE.index("  void run() {"):BRIDGE.index("  std::unique_ptr<HighState_t>")]
assert run.count("std::lock_guard<std::recursive_mutex>") == 1
fault = BRIDGE[BRIDGE.index('if (ray_test_fault_ == "exit")'):
               BRIDGE.index('if (ray_test_active_ && ray_test_fault_ == "freeze")')]
assert "std::_Exit(EXIT_SUCCESS)" in fault

for path in (ROOT / "unitree_mujoco/simulate/src/main.cc",
             ROOT / "unitree_mujoco/simulate/src/bridge_lifecycle.h",
             ROOT / "unitree_mujoco/simulate/src/unitree_sdk2_bridge.h",
             ROOT / "unitree_mujoco/simulate/test/p1_09u_bridge_lifecycle_state.cpp",
             ROOT / "unitree_mujoco/simulate/test/p1_09v_terminal_invariant.cpp",
             ROOT / "scripts/test_p1_09w_static_contract.py"):
    for line_no, line in enumerate(path.read_text().splitlines(), 1):
        assert line == line.rstrip(" \t"), f"trailing whitespace: {path}:{line_no}"

print("P1-09W consolidated static contract: PASS")
