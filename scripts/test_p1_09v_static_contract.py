#!/usr/bin/env python3
"""Mechanical checks for P1-09V terminal and constructor m/d contracts."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "unitree_mujoco/simulate/src/main.cc").read_text()
LIFECYCLE = (ROOT / "unitree_mujoco/simulate/src/bridge_lifecycle.h").read_text()

assert "enum class State { INITIAL, RESERVED, ACTIVE, STOPPING, TERMINAL };" in LIFECYCLE
assert "bool completeTerminal()" in LIFECYCLE
assert "if (state_ != State::STOPPING) return false;" in LIFECYCLE
assert "state_ = State::TERMINAL;" in LIFECYCLE
assert "markBridgeTerminal" not in LIFECYCLE

worker = MAIN[MAIN.index("void *UnitreeSdk2BridgeThread"):
              MAIN.index("//------------------------------------------ main")]
guard_start = worker.index("std::lock_guard<std::recursive_mutex> md_lock")
guard_end = worker.index("\n  }", guard_start)
guard = worker[guard_start:guard_end]
assert "mj_name2id(m" in guard
assert "model_nu = m->nu" in guard
assert "std::cerr" not in guard
assert worker.index("ChannelFactory") < guard_start

main_start = MAIN.index("bridge_stop_request.beginStop();")
main_tail = MAIN[main_start:MAIN.rindex("  return 0;\n}")]
assert main_tail.index("unitree_thread.join();") < main_tail.index("completeTerminal()")
assert main_tail.index("completeTerminal()") < main_tail.index("physicsthreadhandle.join();")

print("P1-09V static contract: PASS")
