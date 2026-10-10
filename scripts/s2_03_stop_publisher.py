#!/usr/bin/env python3
"""Persistent, pre-started control publisher used by bounded diagnostics."""
from __future__ import annotations

import json
import select
import sys
import time

import rclpy
from control_input_msgs.msg import Inputs


def emit(**value: object) -> None:
    print(json.dumps(value, sort_keys=True), flush=True)


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("s2_03_prepared_stop_publisher")
    publisher = node.create_publisher(Inputs, "/control_input", 10)
    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if publisher.get_subscription_count() > 0:
                emit(event="READY", monotonic_ns=time.monotonic_ns(), subscribers=publisher.get_subscription_count())
                break
        else:
            emit(event="NOT_READY", monotonic_ns=time.monotonic_ns(), reason="no /control_input subscriber")
            return 2

        command_map = {"STOP": 1, "HARD_STOP": 9, "NORMAL_SHUTDOWN": 6}
        for line in sys.stdin:
            token = line.strip()
            if token not in command_map:
                emit(event="REJECTED", token=token, reason="unknown command token")
                continue
            command = command_map[token]
            msg = Inputs()
            msg.command = command
            msg.lx = msg.ly = msg.rx = msg.ry = 0.0
            start_ns = time.monotonic_ns()
            publisher.publish(msg)
            return_ns = time.monotonic_ns()
            emit(event="PUBLISHED", token=token, command=command,
                 publish_start_monotonic_ns=start_ns, publish_return_monotonic_ns=return_ns,
                 subscribers=publisher.get_subscription_count())
        emit(event="EOF", monotonic_ns=time.monotonic_ns(), reason="publisher input closed")
        return 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
