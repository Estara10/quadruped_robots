"""Bind one S1-04 policy frame to source/config events from its own ROS launch."""
from __future__ import annotations

import re
from typing import Any


SOURCE_RE = re.compile(
    r"\[S1-RUN-SOURCE\] pid=(\d+) package_share=(\S+) config=(\S+) "
    r"agile_model=(\S+) ra_model=(\S+) candidate=(\S+) "
    r"group=(\S+) entry_threshold=([-+0-9.eE]+) effective_exit_threshold=([-+0-9.eE]+) "
    r"hysteresis=(\S+) hold=(\S+) goal=\(([-+0-9.eE]+),([-+0-9.eE]+)\)"
)
SWITCH_CONFIG_RE = re.compile(
    r"\[S2-SWITCH-CONFIG\] group=(\S+) entry_threshold=([-+0-9.eE]+) "
    r"effective_exit_threshold=([-+0-9.eE]+) hysteresis=(\S+) hold=(\S+)"
)
NODE_RE = re.compile(r"^\[([^\]]+)\]")
WRITER_RE = re.compile(
    r"\[RtFrame\] Shared memory initialized: /mujoco_rt_frame "
    r"session_id=(\d+) writer_pid=(\d+)"
)
PATH_RE = re.compile(r"\[PATH\].*?\bgoal=\(([-+0-9.eE]+),([-+0-9.eE]+)\)")
REC_RE = re.compile(r"\[REC\] Recovery policy loaded: (\S+)")
REC_MISSING_RE = re.compile(r"\[REC\] Recovery policy not found")


def _node(line: str) -> str | None:
    match = NODE_RE.search(line)
    return match.group(1) if match else None


def _conflict(reason: str, **evidence: Any) -> dict[str, Any]:
    return {"status": "conflict", "reason": reason, "evidence": evidence}


def _pending(reason: str, **evidence: Any) -> dict[str, Any]:
    return {"status": "pending", "reason": reason, "evidence": evidence}


def verify_source_binding(
    log_text: str,
    expected: dict[str, Any],
    frame: dict[str, Any],
    expected_controller_pid: int | None,
) -> dict[str, Any]:
    """Return confirmed, pending (evidence absent), or conflict (evidence differs).

    `log_text` must be the current run's ros_launch.log. All required log events
    are joined by the ROS launch process tag; the source PID and writer PID must
    also agree with the preflight controller PID and frame session.
    """
    if expected_controller_pid is None:
        return _pending("expected_controller_pid_unavailable")
    controller_cmdline = expected.get("controller_cmdline")
    if not controller_cmdline:
        return _pending("controller_launch_command_missing")
    launched_configs = re.findall(r"--params-file\s+(\S+)", controller_cmdline)
    if launched_configs != [expected["launch_config"]]:
        return _conflict("controller_launch_config_mismatch", observed=controller_cmdline,
                         expected=[expected["launch_config"]])

    lines = log_text.splitlines()
    source_lines = [line for line in lines if "[S1-RUN-SOURCE]" in line]
    if not source_lines:
        return _pending("source_event_missing")

    parsed_sources: list[tuple[str | None, dict[str, Any]]] = []
    for line in source_lines:
        match = SOURCE_RE.search(line)
        if not match:
            return _pending("source_event_incomplete", line=line.strip())
        node = _node(line)
        (pid, package_share, config, agile_model, ra_model, candidate, group, entry,
         exit_threshold, hysteresis, hold, goal_x, goal_y) = match.groups()
        parsed_sources.append((node, {
            "pid": int(pid), "package_share": package_share, "config": config,
            "agile_model": agile_model, "ra_model": ra_model, "candidate": candidate,
            "group": group, "entry_threshold": float(entry), "exit_threshold": float(exit_threshold),
            "hysteresis": hysteresis, "hold": hold,
            "goal_x": float(goal_x), "goal_y": float(goal_y),
        }))

    if len(parsed_sources) != 1:
        return _conflict("multiple_controller_source_events", count=len(parsed_sources))
    source_node, source = parsed_sources[0]
    if source_node is None:
        return _pending("source_launch_node_missing")
    if source["pid"] != expected_controller_pid:
        return _conflict("controller_pid_mismatch", observed=source["pid"], expected=expected_controller_pid)

    fields = {
        "package_share": expected["package_share"],
        "config": expected["config"],
        "agile_model": expected["agile_model"],
        "ra_model": expected["ra_model"],
        "candidate": expected["candidate"],
        "group": expected["group"],
        "hysteresis": "false",
        "hold": "false",
    }
    for field, wanted in fields.items():
        if source[field] != wanted:
            return _conflict(f"{field}_mismatch", observed=source[field], expected=wanted)
    if abs(source["entry_threshold"] - float(expected["entry_threshold"])) > 1e-5:
        return _conflict("entry_threshold_mismatch", observed=source["entry_threshold"], expected=expected["entry_threshold"])
    if abs(source["exit_threshold"] - float(expected["exit_threshold"])) > 1e-5:
        return _conflict("exit_threshold_mismatch", observed=source["exit_threshold"], expected=expected["exit_threshold"])
    if abs(source["goal_x"] - float(expected["goal_x"])) > 1e-5 or abs(source["goal_y"] - float(expected["goal_y"])) > 1e-5:
        return _conflict("goal_mismatch", observed=[source["goal_x"], source["goal_y"]],
                         expected=[expected["goal_x"], expected["goal_y"]])

    path_lines = [line for line in lines if "[PATH]" in line]
    if not path_lines:
        return _pending("runtime_path_event_missing")
    if len(path_lines) != 1:
        return _conflict("multiple_runtime_path_events", count=len(path_lines))
    path_line = path_lines[0]
    path_match = PATH_RE.search(path_line)
    if path_match is None:
        return _pending("runtime_path_event_incomplete", line=path_line.strip())
    path_node = _node(path_line)
    path_goal = [float(path_match.group(1)), float(path_match.group(2))]
    if path_node != source_node:
        return _conflict("runtime_path_from_different_controller", path_node=path_node,
                         source_node=source_node)
    if abs(path_goal[0] - float(expected["goal_x"])) > 1e-5 or abs(path_goal[1] - float(expected["goal_y"])) > 1e-5:
        return _conflict("runtime_path_goal_mismatch", observed=path_goal,
                         expected=[expected["goal_x"], expected["goal_y"]])

    frame_mode = int(frame["switching_mode"])
    if frame_mode != int(expected["candidate_mode"]):
        return _conflict("frame_candidate_mismatch", observed=frame_mode, expected=expected["candidate_mode"])
    for field in ("entry_threshold", "exit_threshold"):
        if abs(float(frame[field]) - float(expected[field])) > 1e-5:
            return _conflict(f"frame_{field}_mismatch", observed=frame[field], expected=expected[field])

    # Any source/writer record in a second controller launch process is a
    # conflict. We never combine evidence across controller instances.
    related_writers: list[tuple[str | None, int, int]] = []
    for line in lines:
        if "[RtFrame] Shared memory initialized:" not in line:
            continue
        match = WRITER_RE.search(line)
        if not match:
            return _pending("writer_event_incomplete", line=line.strip())
        related_writers.append((_node(line), int(match.group(1)), int(match.group(2))))
    if not related_writers:
        return _pending("writer_session_event_missing")
    if len(related_writers) != 1:
        return _conflict("multiple_writer_session_events", count=len(related_writers))
    writer_node, writer_session, writer_pid = related_writers[0]
    if writer_node != source_node or writer_pid != expected_controller_pid or writer_pid != source["pid"]:
        return _conflict("writer_controller_mismatch", writer_node=writer_node, source_node=source_node,
                         writer_pid=writer_pid, controller_pid=source["pid"])
    if writer_session != int(frame["session_id"]):
        return _conflict("writer_session_mismatch", writer_session=writer_session,
                         frame_session=int(frame["session_id"]))

    recovery_loaded: list[tuple[str | None, str]] = []
    recovery_missing_nodes: list[str | None] = []
    for line in lines:
        if "[REC]" not in line:
            continue
        loaded = REC_RE.search(line)
        if loaded:
            recovery_loaded.append((_node(line), loaded.group(1)))
        elif REC_MISSING_RE.search(line):
            recovery_missing_nodes.append(_node(line))
    if recovery_missing_nodes:
        if any(node != source_node for node in recovery_missing_nodes):
            return _conflict("recovery_event_from_different_controller", nodes=recovery_missing_nodes,
                             source_node=source_node)
        return _conflict("recovery_model_not_loaded", nodes=recovery_missing_nodes)
    if not recovery_loaded:
        return _pending("recovery_model_event_missing")
    if len(recovery_loaded) != 1:
        return _conflict("multiple_recovery_model_events", count=len(recovery_loaded))
    recovery_node, recovery_path = recovery_loaded[0]
    if recovery_node != source_node:
        return _conflict("recovery_event_from_different_controller", recovery_node=recovery_node,
                         source_node=source_node)
    if recovery_path != expected["recovery_model"]:
        return _conflict("recovery_model_path_mismatch", observed=recovery_path,
                         expected=expected["recovery_model"])

    return {"status": "confirmed", "reason": "all_source_events_match", "evidence": {
        "controller_pid": source["pid"], "writer_pid": writer_pid,
        "session_id": writer_session, "launch_node": source_node,
        "candidate": source["candidate"], "package_share": source["package_share"],
        "group": source["group"], "hysteresis": source["hysteresis"], "hold": source["hold"],
        "config": source["config"], "agile_model": source["agile_model"],
        "ra_model": source["ra_model"], "recovery_model": recovery_path,
        "entry_threshold": source["entry_threshold"], "exit_threshold": float(frame["exit_threshold"]),
        "goal_world_m": [source["goal_x"], source["goal_y"]],
        "controller_launch_config": expected["launch_config"],
        "runtime_path_goal_world_m": path_goal,
    }}


def verify_config_consumption(
    log_text: str,
    expected: dict[str, Any],
    expected_controller_pid: int | None,
) -> dict[str, Any]:
    """Confirm A config was consumed by this launch before entering RL."""
    if expected_controller_pid is None:
        return _pending("expected_controller_pid_unavailable")
    controller_cmdline = expected.get("controller_cmdline")
    if not controller_cmdline:
        return _pending("controller_launch_command_missing")
    launched_configs = re.findall(r"--params-file\s+(\S+)", controller_cmdline)
    if launched_configs != [expected["launch_config"]]:
        return _conflict("controller_launch_config_mismatch", observed=controller_cmdline,
                         expected=[expected["launch_config"]])

    lines = log_text.splitlines()
    source_lines = [line for line in lines if "[S1-RUN-SOURCE]" in line]
    switch_lines = [line for line in lines if "[S2-SWITCH-CONFIG]" in line]
    if not source_lines or not switch_lines:
        return _pending("configuration_consumption_log_missing",
                        source_count=len(source_lines), switch_count=len(switch_lines))
    if len(source_lines) != 1 or len(switch_lines) != 1:
        return _conflict("multiple_configuration_consumption_events",
                         source_count=len(source_lines), switch_count=len(switch_lines))
    source_match = SOURCE_RE.search(source_lines[0])
    switch_match = SWITCH_CONFIG_RE.search(switch_lines[0])
    if source_match is None or switch_match is None:
        return _pending("configuration_consumption_event_incomplete")
    source_node, switch_node = _node(source_lines[0]), _node(switch_lines[0])
    if source_node is None or switch_node is None:
        return _pending("configuration_launch_node_missing")
    if source_node != switch_node:
        return _conflict("configuration_events_from_different_nodes",
                         source_node=source_node, switch_node=switch_node)

    source_values = source_match.groups()
    source = {
        "pid": int(source_values[0]), "package_share": source_values[1], "config": source_values[2],
        "agile_model": source_values[3], "ra_model": source_values[4], "candidate": source_values[5],
        "group": source_values[6], "entry_threshold": float(source_values[7]),
        "exit_threshold": float(source_values[8]), "hysteresis": source_values[9], "hold": source_values[10],
        "goal_x": float(source_values[11]), "goal_y": float(source_values[12]),
    }
    group, entry, exit_threshold, hysteresis, hold = switch_match.groups()
    switch = {"group": group, "entry_threshold": float(entry), "exit_threshold": float(exit_threshold),
              "hysteresis": hysteresis, "hold": hold}
    if source["pid"] != expected_controller_pid:
        return _conflict("controller_pid_mismatch", observed=source["pid"], expected=expected_controller_pid)
    for key in ("package_share", "config", "agile_model", "ra_model", "candidate", "group", "hysteresis", "hold"):
        wanted = expected[key]
        if source[key] != wanted:
            return _conflict(f"{key}_mismatch", observed=source[key], expected=wanted)
    for key in ("entry_threshold", "exit_threshold"):
        if abs(source[key] - float(expected[key])) > 1e-5 or abs(switch[key] - float(expected[key])) > 1e-5:
            return _conflict(f"{key}_mismatch", source=source[key], switch=switch[key], expected=expected[key])
    for key in ("goal_x", "goal_y"):
        if abs(source[key] - float(expected[key])) > 1e-5:
            return _conflict(f"{key}_mismatch", observed=source[key], expected=expected[key])
    if switch["group"] != expected["group"] or switch["hysteresis"] != expected["hysteresis"] or switch["hold"] != expected["hold"]:
        return _conflict("switch_config_event_mismatch", observed=switch, expected=expected)

    recovery_loaded: list[tuple[str | None, str]] = []
    for line in lines:
        loaded = REC_RE.search(line)
        if loaded:
            recovery_loaded.append((_node(line), loaded.group(1)))
    if not recovery_loaded:
        return _pending("recovery_model_event_missing")
    if len(recovery_loaded) != 1:
        return _conflict("multiple_recovery_model_events", count=len(recovery_loaded))
    rec_node, recovery_path = recovery_loaded[0]
    if rec_node != source_node or recovery_path != expected["recovery_model"]:
        return _conflict("recovery_model_source_mismatch", node=rec_node, source_node=source_node,
                         observed=recovery_path, expected=expected["recovery_model"])

    return {"status": "confirmed", "reason": "controller_initialization_consumed_A_config", "evidence": {
        **source, "effective_exit_threshold": switch["exit_threshold"],
        "goal_world_m": [source["goal_x"], source["goal_y"]],
        "switch_config_event": switch, "launch_node": source_node,
        "recovery_model": recovery_path, "controller_launch_config": expected["launch_config"],
    }}
