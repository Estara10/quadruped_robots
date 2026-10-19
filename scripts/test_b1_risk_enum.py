#!/usr/bin/env python3
"""Regression check for B1 tri-state risk-edge counting."""
from s1_analyze_run import risk_edge_counts


def test_unknown_risk_edges_are_not_true() -> None:
    counts = risk_edge_counts([
        {"risk_condition_entered": 2},
        {"risk_condition_entered": 0},
        {"risk_condition_entered": 1},
        {"risk_condition_entered": True},  # bool is not an enum integer here
        {},
    ])
    assert counts == {"true": 1, "false": 1, "unknown": 1, "invalid": 2}


if __name__ == "__main__":
    test_unknown_risk_edges_are_not_true()
    print("B1 risk enum check: PASS")
