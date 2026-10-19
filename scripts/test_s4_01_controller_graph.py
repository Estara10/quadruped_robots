#!/usr/bin/env python3
"""Targeted S4-01 regression for fresh controller-node uniqueness checks."""

from s1_run_diagnostic import controller_node_inventory


def test_one_manager_and_controller_is_unique():
    result = controller_node_inventory(
        "/robot_state_publisher\n/controller_manager\n/rl_quadruped_controller\n")
    assert result["unique"]
    assert result["counts"] == {
        "/controller_manager": 1,
        "/rl_quadruped_controller": 1,
    }


def test_duplicate_manager_or_controller_is_rejected():
    duplicate_manager = controller_node_inventory(
        "/controller_manager\n/controller_manager\n/rl_quadruped_controller\n")
    duplicate_controller = controller_node_inventory(
        "/controller_manager\n/rl_quadruped_controller\n/rl_quadruped_controller\n")
    assert not duplicate_manager["unique"]
    assert not duplicate_controller["unique"]


def test_unexpected_or_empty_graph_does_not_pass():
    assert not controller_node_inventory("/controller_manager\n")["unique"]
    assert not controller_node_inventory("")["unique"]


if __name__ == "__main__":
    test_one_manager_and_controller_is_unique()
    test_duplicate_manager_or_controller_is_rejected()
    test_unexpected_or_empty_graph_does_not_pass()
    print("S4-01 controller graph uniqueness checks PASS")
