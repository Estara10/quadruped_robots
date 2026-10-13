#!/usr/bin/env python3
"""Persistent ROS query/control session used by bounded diagnostics."""
from __future__ import annotations

import json
import argparse
import select
import os
import sys
import time
from collections import Counter

import rclpy
from control_input_msgs.msg import Inputs
from controller_manager_msgs.srv import ListControllers
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from s6_07_startup_protocol import command_for_publish, publish_outcome


def emit(**value: object) -> None:
    print(json.dumps(value, sort_keys=True), flush=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--node-name", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node(args.node_name)
    publisher = node.create_publisher(Inputs, "/control_input", 10)
    list_controllers = node.create_client(
        ListControllers, "/controller_manager/list_controllers")
    latest_joint_state: dict | None = None

    def on_joint_state(message: JointState) -> None:
        nonlocal latest_joint_state
        latest_joint_state = {
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

    def endpoint_qos(endpoint) -> dict:
        qos = endpoint.qos_profile
        return {
            "history": str(qos.history), "depth": int(qos.depth),
            "reliability": str(qos.reliability), "durability": str(qos.durability),
            "liveliness": str(qos.liveliness),
        }

    def joint_state_publishers() -> list[dict]:
        publishers = []
        for endpoint in node.get_publishers_info_by_topic("/joint_states"):
            publishers.append({
                "node_name": endpoint.node_name,
                "node_namespace": endpoint.node_namespace,
                "topic_type": endpoint.topic_type,
                "qos": endpoint_qos(endpoint),
            })
        return publishers

    joint_state_subscription = node.create_subscription(
        JointState, "/joint_states", on_joint_state, qos_profile_sensor_data)
    try:
        emit(event="READY", monotonic_ns=time.monotonic_ns(), node_name=node.get_name(),
             node_namespace=node.get_namespace(), topic="/control_input",
             service="/controller_manager/list_controllers",
             joint_state_topic="/joint_states",
             joint_state_subscription_qos={"reliability": "BEST_EFFORT",
                                           "durability": "VOLATILE",
                                           "history": "KEEP_LAST", "depth": 5})

        for line in sys.stdin:
            token = line.strip()
            if token.startswith("STATUS "):
                try:
                    requested_timeout = float(token.split(None, 1)[1])
                except (IndexError, ValueError):
                    emit(event="STATUS", status="invalid_request", monotonic_ns=time.monotonic_ns())
                    continue
                timeout_s = max(0.02, min(1.0, requested_timeout))
                rclpy.spin_once(node, timeout_sec=min(0.01, timeout_s))
                request_ns = time.monotonic_ns()
                nodes = node.get_node_names_and_namespaces()
                names = [f"{namespace.rstrip('/')}/{name}" or "/" for name, namespace in nodes]
                expected = ("/controller_manager", "/rl_quadruped_controller")
                counts = Counter(name for name in names if name in expected)
                service_status = "unavailable"
                controllers = []
                if list_controllers.service_is_ready():
                    future = list_controllers.call_async(ListControllers.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
                    if future.done() and future.exception() is None:
                        service_status = "completed"
                        controllers = [{"name": item.name, "state": item.state,
                                        "type": item.type}
                                       for item in future.result().controller]
                    elif future.done():
                        service_status = "response_failed"
                    else:
                        service_status = "timeout"
                emit(event="STATUS", status="completed", request_monotonic_ns=request_ns,
                     response_monotonic_ns=time.monotonic_ns(), node_name=node.get_name(),
                     node_namespace=node.get_namespace(), graph_nodes=names,
                     expected_node_counts={name: counts.get(name, 0) for name in expected},
                     list_controllers_status=service_status, controllers=controllers,
                     control_input_subscribers=publisher.get_subscription_count(),
                     joint_state_publishers=joint_state_publishers(),
                     latest_joint_state=latest_joint_state)
                continue
            rclpy.spin_once(node, timeout_sec=0.01)
            subscribers = publisher.get_subscription_count()
            command, reject_reason = command_for_publish(token, subscribers)
            if reject_reason is not None:
                emit(event="FAILED", token=token, reason=reject_reason,
                     monotonic_ns=time.monotonic_ns())
                continue
            msg = Inputs()
            msg.command = command
            msg.lx = msg.ly = msg.rx = msg.ry = 0.0
            start_ns = time.monotonic_ns()
            try:
                publisher.publish(msg)
            except Exception as exc:
                outcome = publish_outcome(token, command, exc)
                outcome.update(publish_start_monotonic_ns=start_ns)
                emit(**outcome)
                continue
            return_ns = time.monotonic_ns()
            outcome = publish_outcome(token, command, None)
            outcome.update(publish_start_monotonic_ns=start_ns,
                           publish_return_monotonic_ns=return_ns, subscribers=subscribers)
            emit(**outcome)
        emit(event="EOF", monotonic_ns=time.monotonic_ns(), reason="startup session input closed")
        return 0
    finally:
        node.destroy_subscription(joint_state_subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
