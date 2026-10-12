#!/usr/bin/env python3
"""Print one measured ROS joint-state sample as JSON for bounded diagnostics."""

import argparse
import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-s", type=float, default=2.0)
    args = parser.parse_args()
    result = {"sample": None}

    class OneJointState(Node):
        def __init__(self):
            super().__init__("s6_01_joint_state_sample")
            self.subscription = self.create_subscription(
                JointState, "/joint_states", self.on_message, 10)

        def on_message(self, message: JointState) -> None:
            if result["sample"] is not None:
                return
            result["sample"] = {
                "source": "/joint_states sensor_msgs/msg/JointState",
                "capture_monotonic_ns": time.monotonic_ns(),
                "python_executable": sys.executable,
                "rclpy_module": rclpy.__file__,
                "ros_environment": {
                    key: os.environ.get(key, "<unset>")
                    for key in ("ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_DISTRO",
                                "AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "PYTHONPATH")
                },
                "header_stamp": {"sec": int(message.header.stamp.sec),
                                 "nanosec": int(message.header.stamp.nanosec)},
                "name": list(message.name),
                "position_rad": list(message.position),
                "velocity_rad_s": list(message.velocity),
                "effort_nm": list(message.effort),
            }

    rclpy.init(args=None)
    node = OneJointState()
    deadline = time.monotonic() + args.timeout_s
    try:
        while rclpy.ok() and result["sample"] is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result["sample"], separators=(",", ":")))
    return 0 if result["sample"] is not None else 2


if __name__ == "__main__":
    raise SystemExit(main())
