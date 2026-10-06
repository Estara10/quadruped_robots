#!/usr/bin/env python3
"""Mechanical checks for P1-09T lock-I/O and terminal bridge lifecycle."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "unitree_mujoco/simulate/src/main.cc").read_text()
LIFECYCLE = (ROOT / "unitree_mujoco/simulate/src/bridge_lifecycle.h").read_text()
THREAD = (ROOT / "unitree_mujoco/simulate/src/joinable_thread.h").read_text()

# Each reload critical section may only perform m/d and lifecycle operations.
cursor = 0
sections = []
while True:
    start = MAIN.find("std::lock_guard<std::recursive_mutex> md_lock(sim.mtx)", cursor)
    if start < 0:
        break
    end = MAIN.find("\n          }", start)
    assert end >= 0, "reload lock section must have a bounded closing brace"
    sections.append(MAIN[start:end])
    cursor = end + 1
assert len(sections) == 2
for section in sections:
    assert not any(token in section for token in
                   ("std::cerr", "std::cout", "printf(", "fprintf(",
                    "DDS", "shm_open", "mmap(", "sleep(", "wait("))

# Diagnostics are emitted only after the lock scope closes.
assert MAIN.count("reload rejected before m/d replace") == 2
for marker in ["bool reload_rejected = false;"]:
    assert MAIN.count(marker) == 2

# Initial m/d readiness precedes bridge observation/startup.
physics = MAIN[MAIN.index("void PhysicsThread"):MAIN.index("void *UnitreeSdk2BridgeThread")]
bridge = MAIN[MAIN.index("void *UnitreeSdk2BridgeThread"):MAIN.index("//------------------------------------------ main")]
assert physics.index("mj_forward(m, d);") < physics.index("markInitialReady()")
assert bridge.index("initialReady()") < bridge.index("ChannelFactory")

# One-way lifecycle and one-way joinable worker are explicit and mechanical.
assert "enum class State { INITIAL, RESERVED, ACTIVE, STOPPING, TERMINAL };" in LIFECYCLE
assert "std::atomic<bool> bridge_active_" not in LIFECYCLE
assert "std::atomic<bool> terminal_" not in LIFECYCLE
assert "state_ == State::INITIAL || state_ == State::TERMINAL" in LIFECYCLE
assert "terminal_" in THREAD
assert "join_in_progress_ || terminal_" in THREAD
assert "terminal_ = true;" in THREAD
assert "detach(" not in THREAD

# Both reload branches retain candidate cleanup and active-bridge rejection.
assert MAIN.count("if (dnew && bridge_lifecycle->reloadAllowed())") == 2
assert MAIN.count("mj_deleteData(dnew);") >= 2
assert MAIN.count("mj_deleteModel(mnew);") >= 2

print("P1-09T static contract: PASS")
