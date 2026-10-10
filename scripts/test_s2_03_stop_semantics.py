#!/usr/bin/env python3
"""Small S2-03 checks for completed hard-stop evidence and terminal priority."""
import importlib.util
from pathlib import Path

script = Path(__file__).with_name("s1_run_diagnostic.py")
spec = importlib.util.spec_from_file_location("s1_run_diagnostic", script)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(module)


def test_hard_stop_confirmation_requires_post_transition_marker_and_time():
    valid = "[INFO] [controller]: [HARD-STOP-CONFIRMED] command=1 state=PASSIVE steady_ns=1200 changed=1"
    assert module.hard_stop_confirmation(valid, 1100)["controller_steady_monotonic_ns"] == 1200
    assert module.hard_stop_confirmation(valid, 1201) is None
    assert module.hard_stop_confirmation("[ERROR] [HARD-STOP] command=1 -> forcing PASSIVE", 1) is None
    assert module.hard_stop_confirmation("Switched from RL to passive", 1) is None


def test_movement_terminal_priority_remains_unchanged():
    assert module.select_strategy_terminal(False, True, False, True, False) == "COLLISION_TERMINATION"
    assert module.select_strategy_terminal(True, True, False, False, False) == "SYSTEM_SAFETY_ABORT"
    assert module.select_strategy_terminal(False, False, False, False, True) == "DIAGNOSTIC_SIM_TIME_LIMIT"
