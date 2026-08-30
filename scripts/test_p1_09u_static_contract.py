#!/usr/bin/env python3
"""Mechanical source checks for the P1-09U lifecycle-state closure."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LIFECYCLE_PATH = ROOT / "unitree_mujoco/simulate/src/bridge_lifecycle.h"
MAIN_PATH = ROOT / "unitree_mujoco/simulate/src/main.cc"
BRIDGE_PATH = ROOT / "unitree_mujoco/simulate/src/unitree_sdk2_bridge.h"
TEST_PATH = ROOT / "unitree_mujoco/simulate/test/p1_09u_bridge_lifecycle_state.cpp"

lifecycle = LIFECYCLE_PATH.read_text()
main = MAIN_PATH.read_text()
test = TEST_PATH.read_text()

assert "enum class State { INITIAL, RESERVED, ACTIVE, STOPPING, TERMINAL };" in lifecycle
assert "std::atomic" not in lifecycle
assert "reserveBridge()" in lifecycle
assert "completeTerminal()" in lifecycle
assert "state_ == State::INITIAL || state_ == State::TERMINAL" in lifecycle
assert "bridge_stop_request.reserveBridge()" in main
assert "markBridgeInactive" not in main
assert "std::thread stopper" in test
assert "std::thread activator" in test
assert "for (int iteration = 0; iteration < 64; ++iteration)" in test
assert "Deterministic linearization A" in test
assert "Deterministic linearization B" in test
assert "assert(activate_succeeded.load(std::memory_order_acquire));" in test
assert "assert(!activate_succeeded.load(std::memory_order_acquire));" in test

for path in (LIFECYCLE_PATH, MAIN_PATH, BRIDGE_PATH, TEST_PATH):
    for number, line in enumerate(path.read_text().splitlines(), start=1):
        assert line == line.rstrip(" \t"), f"trailing whitespace: {path}:{number}"

print("P1-09U static contract: PASS")
