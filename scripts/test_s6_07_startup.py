#!/usr/bin/env python3
"""Focused offline checks for production S6-07 startup gates and command protocol."""

import tempfile
import threading
import time
import unittest
from unittest import mock
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from manual_ab_compare import enrich_run_metrics, save_comparison
from s1_run_diagnostic import (
    finalize_run_closeout,
    capture_joint_state_from_startup_session,
    query_startup_session,
    rl_entry_precondition_failure,
    select_strategy_terminal,
    startup_controller_gate,
    startup_deadline_remaining,
    startup_physics_snapshot_is_current,
    startup_ros_session_command,
    startup_ros_session_node_name,
    stop_prepared_ros_session,
    start_prepared_stop_publisher,
    wait_controller_transition,
)
from s6_07_startup_protocol import command_for_publish, command_priority, publish_outcome


class S607StartupChecks(unittest.TestCase):
    def setUp(self):
        self.deadline = time.monotonic() + 5.0
        self.query = {
            "event": "STATUS",
            "expected_node_counts": {"/controller_manager": 1,
                                     "/rl_quadruped_controller": 1},
            "list_controllers_status": "completed",
            "controllers": [{"name": "rl_quadruped_controller", "state": "active"}],
            "control_input_subscribers": 1,
            "joint_state_publishers": [{"node_name": "joint_state_broadcaster",
                                         "node_namespace": "/",
                                         "topic_type": "sensor_msgs/msg/JointState",
                                         "qos": {"reliability": "ReliabilityPolicy.RELIABLE"}}],
        }
        self.ready_kwargs = dict(
            launch_alive=True, manager_pids=[100], launch_pgid=50,
            config_status="confirmed", mapped_libraries=["/install/librl_quadruped_controller.so"],
            expected_library="/install/librl_quadruped_controller.so",
            environment_matches=True,
        )

    def test_deadline_uses_one_absolute_end_for_fast_and_slow_readiness(self):
        now = 100.0
        deadline = now + 15.0
        self.assertEqual(startup_deadline_remaining(deadline, now), 15.0)
        self.assertEqual(startup_deadline_remaining(deadline, now + 4.0), 11.0)
        self.assertEqual(startup_deadline_remaining(deadline, now + 15.0), 0.0)

    def test_current_physics_authority_accepts_current_advancing_sample(self):
        ok, _ = startup_physics_snapshot_is_current(
            {"capture_id": "run1", "runtime_model_fingerprint": "model",
             "scenario_id": "flat", "monotonic_ns": 110, "sim_time": 0.02,
             "sequence": 4, "physics_step": 10},
            status="LIVE", capture_id="run1", model_fingerprint="model",
            scenario_id="flat", process_started_ns=100, now_monotonic_ns=120)
        self.assertTrue(ok)

    def test_old_future_stale_or_invalid_physics_authority_is_rejected(self):
        base = {"capture_id": "run1", "runtime_model_fingerprint": "model",
                "scenario_id": "flat", "monotonic_ns": 110, "sim_time": 0.02,
                "sequence": 4, "physics_step": 10}
        for changes, status, reason in (
            ({"capture_id": "old"}, "LIVE", "old run"),
            ({"monotonic_ns": 90}, "LIVE", "pre-run"),
            ({"monotonic_ns": 1_000}, "LIVE", "future"),
            ({"monotonic_ns": 1}, "STALE", "non-live"),
            ({"sim_time": float("nan")}, "LIVE", "invalid time"),
            ({"scenario_id": "wrong"}, "LIVE", "wrong scene"),
        ):
            with self.subTest(reason=reason):
                ok, _ = startup_physics_snapshot_is_current(
                    {**base, **changes}, status=status, capture_id="run1",
                    model_fingerprint="model", scenario_id="flat",
                    process_started_ns=100, now_monotonic_ns=120)
                self.assertFalse(ok)

    def test_launch_exit_is_immediate_failure(self):
        result = startup_controller_gate(self.query, **{**self.ready_kwargs, "launch_alive": False})
        self.assertEqual(result["status"], "failed")

    def test_service_available_but_controller_inactive_does_not_pass(self):
        inactive = {**self.query, "controllers": [{"name": "rl_quadruped_controller", "state": "inactive"}]}
        self.assertEqual(startup_controller_gate(inactive, **self.ready_kwargs)["status"], "pending")

    def test_delayed_graph_or_service_remains_pending(self):
        delayed_graph = {**self.query, "expected_node_counts": {"/controller_manager": 0,
                                                                  "/rl_quadruped_controller": 0}}
        delayed_service = {**self.query, "list_controllers_status": "unavailable"}
        self.assertEqual(startup_controller_gate(delayed_graph, **self.ready_kwargs)["status"], "pending")
        self.assertEqual(startup_controller_gate(delayed_service, **self.ready_kwargs)["status"], "pending")

    def test_duplicate_nodes_multiple_writers_and_source_mismatch_fail_closed(self):
        duplicate = {**self.query, "expected_node_counts": {"/controller_manager": 1,
                                                              "/rl_quadruped_controller": 2}}
        mismatch = {**self.ready_kwargs, "mapped_libraries": ["/wrong/librl_quadruped_controller.so"]}
        multiwriter = {**self.ready_kwargs, "manager_pids": [100, 101]}
        self.assertEqual(startup_controller_gate(duplicate, **self.ready_kwargs)["status"], "failed")
        self.assertEqual(startup_controller_gate(self.query, **mismatch)["status"], "failed")
        self.assertEqual(startup_controller_gate(self.query, **multiwriter)["status"], "failed")

    def test_joint_state_publisher_missing_duplicate_or_wrong_type_fails_closed(self):
        for publishers, expected in ((None, "pending"), ([], "pending"),
                                     ([self.query["joint_state_publishers"][0]] * 2, "failed"),
                                     ([{"node_name": "other", "topic_type": "sensor_msgs/msg/JointState"}], "failed"),
                                     ([{"node_name": "joint_state_broadcaster", "topic_type": "other/msg/Type"}], "failed")):
            changed = {**self.query, "joint_state_publishers": publishers}
            self.assertEqual(startup_controller_gate(changed, **self.ready_kwargs)["status"], expected)

    def test_query_timeout_does_not_extend_total_deadline(self):
        deadline = time.monotonic() + 2.0
        before = startup_deadline_remaining(deadline)
        time.sleep(0.03)
        after = startup_deadline_remaining(deadline)
        self.assertLess(after, before)
        self.assertEqual(startup_deadline_remaining(deadline, deadline + 1.0), 0.0)

    def test_production_ros_startup_entry_generates_unique_name_and_exact_command_environment(self):
        env = {"S1_04_OVERLAY": "/tmp/s6 overlay", "ROS_DOMAIN_ID": "17", "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp"}
        first, second = startup_ros_session_node_name(), startup_ros_session_node_name()
        self.assertNotEqual(first, second)
        self.assertRegex(first, r"^s6_07_startup_session_[0-9a-f]{10}$")
        command = startup_ros_session_command(env, first)
        self.assertEqual(command[:2], ["bash", "-lc"])
        self.assertIn("source /opt/ros/humble/setup.bash", command[2])
        self.assertIn("source " + str(Path(__file__).resolve().parents[1] / "quadruped_ros2_control_humble") + "/install/setup.bash", command[2])
        self.assertIn("export AMENT_PREFIX_PATH='/tmp/s6 overlay':$AMENT_PREFIX_PATH", command[2])
        self.assertIn("exec /usr/bin/python3 -u", command[2])
        self.assertIn("scripts/s2_03_stop_publisher.py --node-name " + first, command[2])
        with self.assertRaises(ValueError):
            startup_ros_session_command({}, first)
        with self.assertRaises(ValueError):
            startup_ros_session_command(env, "bad/name")

    def test_production_ros_startup_entry_passes_environment_and_checks_ready_handshake(self):
        node_name = startup_ros_session_node_name()
        env = {"S1_04_OVERLAY": "/tmp/overlay", "ROS_DOMAIN_ID": "18"}

        class Input:
            def write(self, value):
                self.value = value
            def flush(self):
                pass

        class Output:
            def readline(self):
                    return ('{"event":"READY","node_name":"' + node_name +
                        '","node_namespace":"/","topic":"/control_input",'
                        '"service":"/controller_manager/list_controllers",'
                        '"joint_state_topic":"/joint_states","joint_state_subscription_qos":{},"monotonic_ns":123}\n')

        class Proc:
            pid = 12345
            stdin = Input()
            stdout = Output()
            returncode = None
            def poll(self):
                return None

        proc = Proc()
        with mock.patch("s1_run_diagnostic.subprocess.Popen", return_value=proc) as popen, \
             mock.patch("s1_run_diagnostic.select.select", return_value=([proc.stdout], [], [])):
            actual_proc, ready = start_prepared_stop_publisher(env, node_name, timeout_s=0.2)
        self.assertIs(actual_proc, proc)
        self.assertEqual(ready["node_name"], node_name)
        kwargs = popen.call_args.kwargs
        self.assertIs(kwargs["env"], env)
        self.assertEqual(kwargs["cwd"], __import__("s1_run_diagnostic").ROS_WS)
        self.assertTrue(kwargs["start_new_session"])
        self.assertIn("ROS_DOMAIN_ID", kwargs["env"])
        self.assertIn(node_name, popen.call_args.args[0][2])

    def test_production_ros_startup_handshake_failure_cleans_owned_process(self):
        node_name = startup_ros_session_node_name()
        env = {"S1_04_OVERLAY": "/tmp/overlay"}
        class Input:
            def write(self, value): pass
            def flush(self): pass
        class Output:
            def readline(self): return '{"event":"NOT_READY","reason":"service missing"}\n'
        class Proc:
            pid = 4567
            stdin = Input()
            stdout = Output()
            returncode = None
            def __init__(self): self.waited = False
            def poll(self): return None if not self.waited else 0
            def wait(self, timeout=None): self.waited = True; self.returncode = 0; return 0
            def kill(self): self.waited = True; self.returncode = -9
        proc = Proc()
        with mock.patch("s1_run_diagnostic.subprocess.Popen", return_value=proc), \
             mock.patch("s1_run_diagnostic.select.select", return_value=([proc.stdout], [], [])), \
             mock.patch("s1_run_diagnostic.os.killpg") as killpg:
            with self.assertRaisesRegex(RuntimeError, "did not become ready"):
                start_prepared_stop_publisher(env, node_name, timeout_s=0.2)
        killpg.assert_called_once_with(proc.pid, __import__("s1_run_diagnostic").signal.SIGINT)
        self.assertTrue(proc.waited)

    def test_empty_status_response_is_returned_as_pending_not_treated_as_ready(self):
        class Input:
            def write(self, value): self.value = value
            def flush(self): pass
        class Output:
            def readline(self): return '{"event":"STATUS"}\n'
        class Proc:
            stdin = Input()
            stdout = Output()
            def poll(self): return None
        proc = Proc()
        with mock.patch("s1_run_diagnostic.select.select", return_value=([proc.stdout], [], [])):
            status = query_startup_session(proc, 0.1)
        self.assertEqual(status["event"], "STATUS")
        self.assertEqual(startup_controller_gate(status, **self.ready_kwargs)["status"], "pending")

    def test_pre_rl_joint_sample_uses_same_session_and_requires_fresh_named_finite_qdq(self):
        class Alive:
            stdin = object()
            stdout = object()
            def poll(self): return None
        proc = Alive()
        names = [f"{leg}_{joint}_joint" for leg in ("FR", "FL", "RR", "RL")
                 for joint in ("hip", "thigh", "calf")]
        responses = {"count": 0}
        def status_response(*_):
            responses["count"] += 1
            if responses["count"] == 1:
                return {"event": "STATUS", "response_monotonic_ns": time.monotonic_ns(),
                        "joint_state_publishers": [{"node_name": "joint_state_broadcaster",
                                                     "topic_type": "sensor_msgs/msg/JointState"}],
                        "latest_joint_state": {"capture_monotonic_ns": 1}}
            sample = {"capture_monotonic_ns": time.monotonic_ns(),
                      "source": "/joint_states sensor_msgs/msg/JointState",
                      "name": names, "position_rad": [float(i) for i in range(12)],
                      "velocity_rad_s": [0.0] * 12}
            return {"event": "STATUS", "response_monotonic_ns": time.monotonic_ns(),
                    "joint_state_publishers": [{"node_name": "joint_state_broadcaster",
                                                 "topic_type": "sensor_msgs/msg/JointState"}],
                    "node_name": "s6_07_startup_session_fixture", "latest_joint_state": sample}
        with mock.patch("s1_run_diagnostic.query_startup_session", side_effect=status_response) as query:
            result = capture_joint_state_from_startup_session(proc, timeout_s=0.5)
        self.assertEqual(result["status"], "captured")
        self.assertEqual(result["capture_session_node"], "s6_07_startup_session_fixture")
        self.assertEqual(len(result["sample"]["name"]), 12)
        self.assertEqual(len(result["rejected_samples"]), 1)
        self.assertIn("predates", result["rejected_samples"][0]["reason"])
        self.assertEqual(query.call_count, 2)

    def test_pre_rl_joint_sample_rejects_malformed_fresh_payload(self):
        class Alive:
            stdin = object()
            stdout = object()
            def poll(self): return None
        def malformed_response(*_):
            sample = {"capture_monotonic_ns": time.monotonic_ns(), "name": ["joint"] * 12,
                      "position_rad": [0.0] * 12, "velocity_rad_s": [0.0] * 12}
            return {"event": "STATUS", "response_monotonic_ns": time.monotonic_ns(),
                    "joint_state_publishers": [{"node_name": "joint_state_broadcaster",
                                                 "topic_type": "sensor_msgs/msg/JointState"}],
                    "latest_joint_state": sample}
        with mock.patch("s1_run_diagnostic.query_startup_session", side_effect=malformed_response):
            result = capture_joint_state_from_startup_session(Alive(), timeout_s=0.5)
        self.assertEqual(result["status"], "invalid")
        self.assertIn("exactly", result["rejected_samples"][0]["reason"])

    def test_pre_rl_joint_sample_rejects_incomplete_or_nonfinite_values(self):
        class Alive:
            stdin = object()
            stdout = object()
            def poll(self): return None
        names = [f"{leg}_{joint}_joint" for leg in ("FR", "FL", "RR", "RL")
                 for joint in ("hip", "thigh", "calf")]
        for mutation, reason in (({"position_rad": [0.0] * 11}, "length 12"),
                                 ({"velocity_rad_s": [float("nan")] + [0.0] * 11}, "non-finite")):
            payload = {"capture_monotonic_ns": time.monotonic_ns(), "name": names,
                       "position_rad": [0.0] * 12, "velocity_rad_s": [0.0] * 12, **mutation}
            def response(*_):
                payload["capture_monotonic_ns"] = time.monotonic_ns()
                return {"event": "STATUS", "response_monotonic_ns": time.monotonic_ns(),
                        "joint_state_publishers": [{"node_name": "joint_state_broadcaster",
                                                     "topic_type": "sensor_msgs/msg/JointState"}],
                        "latest_joint_state": payload}
            with mock.patch("s1_run_diagnostic.query_startup_session", side_effect=response):
                result = capture_joint_state_from_startup_session(Alive(), timeout_s=0.03)
            self.assertEqual(result["status"], "invalid")
            self.assertIn(reason, result["rejected_samples"][0]["reason"])

    def test_prepared_ros_session_eof_close_is_normal_zero_exit(self):
        class Input:
            closed = False
            def close(self): self.closed = True
        class Proc:
            stdin = Input()
            returncode = None
            def poll(self): return self.returncode
            def wait(self, timeout=None): self.returncode = 0; return 0
        result = stop_prepared_ros_session(Proc())
        self.assertEqual(result["status"], "closed")
        self.assertEqual(result["exit_code"], 0)
        self.assertTrue(result["request"].startswith("stdin EOF"))

    def test_readiness_requires_runtime_config_environment_and_publisher(self):
        for changes in (
            {"config_status": "pending"}, {"config_status": "conflict"},
            {"environment_matches": False}, {"control_input_subscribers": 0},
        ):
            changed_query = {**self.query, **{k: v for k, v in changes.items()
                                               if k == "control_input_subscribers"}}
            changed_kwargs = {**self.ready_kwargs, **{k: v for k, v in changes.items()
                                                       if k != "control_input_subscribers"}}
            result = startup_controller_gate(changed_query, **changed_kwargs)
            self.assertNotEqual(result["status"], "ready")

    def test_no_subscriber_or_unknown_command_is_rejected_and_publish_errors_are_explicit(self):
        self.assertEqual(command_for_publish("STAND_STEP", 0), (None, "no /control_input subscriber"))
        self.assertEqual(command_for_publish("SURPRISE", 1), (None, "unknown command token"))
        self.assertEqual(publish_outcome("START_RL", 3, OSError("writer unavailable"))["event"], "FAILED")
        successful = publish_outcome("START_RL", 3, None)
        self.assertIn("controller acceptance and MuJoCo consumption are separate", successful["meaning"])

    def test_start_command_mapping_and_stop_priority_are_preserved(self):
        self.assertEqual(command_for_publish("STAND_STEP", 1)[0], 2)
        self.assertEqual(command_for_publish("START_RL", 1)[0], 3)
        self.assertEqual(command_for_publish("STOP", 1)[0], 1)
        self.assertEqual(command_for_publish("HARD_STOP", 1)[0], 9)
        self.assertLess(command_priority("HARD_STOP"), command_priority("START_RL"))

    def test_standing_recorder_initial_state_and_controller_freshness_gate_rl_entry(self):
        ready = {"controller_readiness_fresh": True, "recorder_ready": True,
                 "standing_confirmed": True, "joint_sample_valid": True,
                 "joint_velocity_within_limit": True,
                 "pre_task_safety_boundary_live": True}
        self.assertIsNone(rl_entry_precondition_failure(ready))
        for key in ready:
            with self.subTest(missing=key):
                self.assertIsNotNone(rl_entry_precondition_failure({**ready, key: False}))

    def test_missing_controller_transition_never_confirms_command_progression(self):
        class Alive:
            def poll(self):
                return None
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "launch.log"
            path.write_text("", encoding="utf-8")
            with path.open("r", encoding="utf-8") as log:
                with self.assertRaises(TimeoutError):
                    wait_controller_transition(log, Alive(), "FIXEDSTAND", 0.01)

    def test_controller_transition_requires_actual_log_confirmation(self):
        class Alive:
            def poll(self):
                return None
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "launch.log"
            path.write_text("", encoding="utf-8")
            def write_transition():
                time.sleep(0.04)
                path.write_text("Switched from passive to fixed down\n", encoding="utf-8")
            writer = threading.Thread(target=write_transition)
            writer.start()
            with path.open("r", encoding="utf-8") as log:
                self.assertEqual(wait_controller_transition(log, Alive(),
                                                            "Switched from passive to fixed down", 0.5),
                                 "Switched from passive to fixed down")
            writer.join()

    def test_safety_terminal_priority_remains_above_arrival_and_normal_closeout(self):
        self.assertEqual(select_strategy_terminal(True, True, True, True, True),
                         "SYSTEM_SAFETY_ABORT")

    def test_production_closeout_attempts_owned_cleanup_before_record_finalization(self):
        order = []
        class Recorder:
            class Writer:
                def close(self):
                    order.append("recorder_close")
            recorder = Writer()
        with tempfile.TemporaryDirectory() as temp:
            process_facts = {"shutdown_complete": True}
            terminal, errors = finalize_run_closeout(
                Path(temp), {"lifecycle": {}}, process_facts, Recorder(), False,
                lambda: order.append("owned_cleanup"))
            self.assertEqual(terminal, None)
            self.assertEqual(errors, [])
            self.assertEqual(order, ["owned_cleanup", "recorder_close"])
            self.assertTrue(process_facts["closeout_complete"])

    def test_comparison_preserves_invalid_startup_run_with_missing_readiness(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "batch_plan.json").write_text(
                '{"repeats_per_group":1,"run_statuses":[{"group":"A","status":"FINISHED"}]}',
                encoding="utf-8")
            invalid = {"run_id": "startup-failed", "record_validity": "INVALID",
                       "navigation_terminal_result": "STARTUP_FAILURE",
                       "s6_preparation_readiness": None,
                       "task_window_safety": {"collision": "UNKNOWN", "fall": "UNKNOWN"}}
            save_comparison(root, {"A": [invalid]}, ("A",), {}, "test")
            summary = __import__("json").loads((root / "batch_summary.json").read_text())
            self.assertEqual(summary["groups"]["A"]["invalid"], 1)
            self.assertEqual(summary["groups"]["A"]["runs"][0]["run_id"], "startup-failed")
            self.assertIn("STARTUP_FAILURE", (root / "comparison.md").read_text())

    def test_invalid_startup_metrics_with_null_lifecycle_fields_are_enrichable(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (root / "run_context.json").write_text(
                '{"lifecycle":{"standing_observation":null,"post_terminal_strategy_stop":null}}',
                encoding="utf-8")
            result = enrich_run_metrics({"run_id": "startup-failed", "recovery_hold_audit": None},
                                        root, {"shutdown_complete": True,
                                               "closeout_complete": True})
            self.assertEqual(result["standing_observation_status"], "NOT_REACHED_OR_UNKNOWN")
            self.assertIsNone(result["passive_controller_transition_confirmed"])


if __name__ == "__main__":
    unittest.main()
