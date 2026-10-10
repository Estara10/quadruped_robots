#!/usr/bin/env python3
"""Bounded, self-cleaning S1 nonformal safety-recording diagnostic."""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import select
import signal
import shlex
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from abs_rt_frame import FrameStatus, classify_frame, read_shm_frame  # noqa: E402
from abs_collision import (  # noqa: E402
    CollisionStatus, classify_snapshot, read_collision_snapshot,
)
from abs_scene import resolve_scene  # noqa: E402
from run_record import RunRecordRecorder, collision_snapshot_payload  # noqa: E402
from s1_runtime_source import verify_config_consumption, verify_source_binding  # noqa: E402

MUJOCO_DIR = ROOT / "unitree_mujoco"
MUJOCO_BIN = MUJOCO_DIR / "simulate/build2/unitree_mujoco"
ROS_WS = ROOT / "quadruped_ros2_control_humble"
LIBTORCH = Path.home() / "Libraries/libtorch-cpu-2.0.1/lib"
SDKLIB = Path.home() / "Libraries/unitree_sdk2/lib"
EMERGENCY_RE = re.compile(r"\[EMERGENCY\].*Forcing PASSIVE")
HARDSTOP_RE = re.compile(r"\[HARD-STOP\].*forcing PASSIVE")
HARDSTOP_CONFIRMED_RE = re.compile(
    r"\[HARD-STOP-CONFIRMED\].*command=(?P<command>\d+).*"
    r"state=PASSIVE.*steady_ns=(?P<steady_ns>\d+)")
VETO_RE = re.compile(r"\[ABS-CONTRACT\].*PASSIVE")
SOURCE_EVIDENCE_WAIT_S = 3.0


class ConfigOnlyComplete(Exception):
    """Internal bounded stop after controller initialization consumed config."""


def now_ns() -> int:
    return time.monotonic_ns()


def select_strategy_terminal(safety_abort: bool, collision: bool, fall: bool,
                             arrived: bool, sim_timeout: bool) -> str | None:
    """Resolve same-observation terminal facts using the run contract order."""
    for active, result in (
        (safety_abort, "SYSTEM_SAFETY_ABORT"),
        (collision, "COLLISION_TERMINATION"),
        (fall, "FALL_TERMINATION"),
        (arrived, "ARRIVED"),
        (sim_timeout, "DIAGNOSTIC_SIM_TIME_LIMIT"),
    ):
        if active:
            return result
    return None


def normal_closeout_gate(terminal: str, safety_abort: bool, collision_events: list[dict],
                         fall_confirmed: bool, snapshot: dict | None) -> tuple[bool, str | None]:
    """Require healthy, authoritative terminal evidence before standing hold."""
    if terminal not in {"ARRIVED", "DIAGNOSTIC_SIM_TIME_LIMIT"}:
        return False, "terminal is not ARRIVED or controlled sim timeout"
    if safety_abort or collision_events or fall_confirmed:
        return False, "safety, collision, or confirmed fall veto"
    if not isinstance(snapshot, dict) or snapshot.get("status") != CollisionStatus.LIVE.value:
        return False, "physics snapshot is not LIVE"
    if snapshot.get("unknown_contacts") != 0:
        return False, "unknown contact coverage"
    if snapshot.get("fall_candidate") is not False or snapshot.get("fall_confirmed") is not False:
        return False, "fall state is uncertain or confirmed"
    if snapshot.get("robot_obstacle_contacts") != 0:
        return False, "robot-obstacle contact is present"
    return True, None


def standing_physics_sample_failure(snapshot: object) -> str | None:
    """Validate one live physics-authority sample during the standing hold."""
    if (not math.isfinite(snapshot.sim_time) or snapshot.physics_coverage_complete != 1 or
            snapshot.unknown_contacts != 0):
        return "invalid physics time, incomplete coverage, or unknown contact"
    if snapshot.fall_candidate or snapshot.fall_confirmed:
        return "fall candidate/confirmation during standing hold"
    if snapshot.robot_obstacle_contacts != 0:
        return "robot-obstacle contact during standing hold"
    if (snapshot.base_height_m < 0.25 or abs(snapshot.base_roll_rad) > 0.30 or
            abs(snapshot.base_pitch_rad) > 0.30 or snapshot.foot_ground_contacts < 2):
        return "standing posture/support condition lost"
    return None


def standing_hold_reached(start_sim_s: float, current_sim_s: float,
                           requested_sim_s: float) -> bool:
    return (math.isfinite(start_sim_s) and math.isfinite(current_sim_s) and
            math.isfinite(requested_sim_s) and current_sim_s >= start_sim_s and
            current_sim_s - start_sim_s >= requested_sim_s)


def matching_processes(names: set[str]) -> list[dict]:
    """Find relevant processes by kernel comm name, avoiding cmdline self-matches."""
    found: list[dict] = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            comm = (entry / "comm").read_text(encoding="utf-8").strip()
            if comm not in names:
                continue
            pid = int(entry.name)
            cmdline = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode("utf-8", errors="replace").strip()
            found.append({"pid": pid, "pgid": os.getpgid(pid), "comm": comm, "cmdline": cmdline})
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    return found


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, sort_keys=True) + "\n")


def build_env(capture_id: str, overlay: Path, display: str, xauthority: str,
              scene: object, run_dir: Path) -> dict[str, str]:
    env = os.environ.copy()
    env["DISPLAY"] = display
    env["XAUTHORITY"] = xauthority
    env["S1_04_OVERLAY"] = str(overlay)
    env["LD_LIBRARY_PATH"] = f"{SDKLIB}:{LIBTORCH}:/usr/lib/x86_64-linux-gnu:{env.get('LD_LIBRARY_PATH', '')}"
    env["MUJOCO_GL"] = "glfw"
    env["ABS_P1_10_SCENARIO_ID"] = scene.scene_id
    env["ABS_P1_10_ROOT_XML_SHA256"] = scene.root_sha256
    env["ABS_P1_10_MODEL_CLOSURE_SHA256"] = scene.closure_sha256
    env["ABS_P1_10_CAPTURE_ID"] = capture_id
    env["ABS_P1_10_EXPECTED_MODEL_FINGERPRINT"] = scene.model_fingerprint
    resolved_run_dir = run_dir.resolve()
    env["ABS_MUJOCO_STANDING_PROMPT_MARKER"] = str(resolved_run_dir / "standing_prompt.marker")
    env["ABS_MUJOCO_STANDING_FRAMEBUFFER_CAPTURE"] = str(resolved_run_dir / "standing_hold.ppm")
    return env


def ros_command(command: str, env: dict[str, str], timeout: float = 12.0) -> subprocess.CompletedProcess[str]:
    shell = (f"source /opt/ros/humble/setup.bash && source {ROS_WS}/install/setup.bash && "
             f"export AMENT_PREFIX_PATH={shlex.quote(env['S1_04_OVERLAY'])}:$AMENT_PREFIX_PATH && {command}")
    return subprocess.run(["bash", "-lc", shell], cwd=ROS_WS, env=env,
                          text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                          timeout=timeout, check=False)


def publish(env: dict[str, str], command: int, lx: float = 0.0, ly: float = 0.0) -> dict:
    msg = f"{{command: {command}, lx: {lx}, ly: {ly}, rx: 0.0, ry: 0.0}}"
    result = ros_command(f"ros2 topic pub --once /control_input control_input_msgs/msg/Inputs '{msg}'", env)
    return {"command": command, "monotonic_ns": now_ns(), "returncode": result.returncode,
            "output": result.stdout[-1200:]}


def start_prepared_stop_publisher(env: dict[str, str]) -> tuple[subprocess.Popen[str], dict]:
    helper = ROOT / "scripts/s2_03_stop_publisher.py"
    shell = (f"source /opt/ros/humble/setup.bash && source {ROS_WS}/install/setup.bash && "
             f"export AMENT_PREFIX_PATH={shlex.quote(env['S1_04_OVERLAY'])}:$AMENT_PREFIX_PATH && "
             f"/usr/bin/python3 -u {shlex.quote(str(helper))}")
    proc = subprocess.Popen(["bash", "-lc", shell], cwd=ROS_WS, env=env, text=True,
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            if proc.stdout is not None and select.select([proc.stdout], [], [], 0.1)[0]:
                line = proc.stdout.readline()
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if event.get("event") == "READY":
                    return proc, event
                if event.get("event") == "NOT_READY":
                    raise RuntimeError(f"prepared PASSIVE publisher did not become ready: {event}")
            if proc.poll() is not None:
                raise RuntimeError(f"prepared PASSIVE publisher exited before READY: rc={proc.returncode}")
        raise TimeoutError("prepared PASSIVE publisher READY timeout")
    except Exception:
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGINT)
                proc.wait(timeout=2.0)
            except Exception:
                proc.kill()
                proc.wait(timeout=2.0)
        raise


def publish_prepared_command(proc: subprocess.Popen[str], token: str) -> dict:
    if proc.poll() is not None or proc.stdin is None or proc.stdout is None:
        raise RuntimeError("prepared control publisher is not alive")
    invoked_ns = now_ns()
    proc.stdin.write(f"{token}\n")
    proc.stdin.flush()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if select.select([proc.stdout], [], [], 0.1)[0]:
            line = proc.stdout.readline()
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if event.get("event") == "PUBLISHED" and event.get("token") == token:
                event["supervisor_invoked_monotonic_ns"] = invoked_ns
                event["supervisor_ack_received_monotonic_ns"] = now_ns()
                return event
            if event.get("event") in {"EOF", "NOT_READY"}:
                raise RuntimeError(f"prepared PASSIVE publisher failed: {event}")
    raise TimeoutError(f"prepared {token} publish acknowledgement timeout")


def publish_prepared_stop(proc: subprocess.Popen[str]) -> dict:
    """Backward-compatible command-1 request used by abnormal closeout."""
    return publish_prepared_command(proc, "STOP")


NORMAL_SHUTDOWN_RE = re.compile(r"\[NORMAL-SHUTDOWN\]\s+(.*)$")


def normal_shutdown_events(log_text: str, after_monotonic_ns: int) -> list[dict]:
    events: list[dict] = []
    for line in log_text.splitlines():
        match = NORMAL_SHUTDOWN_RE.search(line)
        if not match:
            continue
        fields = dict(re.findall(r"([a-z0-9_]+)=([^\s]+)", match.group(1)))
        try:
            steady_ns = int(fields["steady_ns"])
        except (KeyError, ValueError):
            continue
        if steady_ns < after_monotonic_ns:
            continue
        events.append({"event": "NORMAL_SHUTDOWN_STAGE", "steady_ns": steady_ns,
                       "phase": fields.get("phase", "UNKNOWN"), "fields": fields,
                       "log_line": line.strip()[-1600:]})
    events.sort(key=lambda event: event["steady_ns"])
    return events


def wait_controller_transition(log_file, ros_process: subprocess.Popen | None,
                               transition: str, timeout_s: float) -> str:
    """Wait for a specific startup FSM transition before sending the next command."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        log_file.flush()
        if ros_process is None or ros_process.poll() is not None:
            raise RuntimeError(f"ROS launch exited before controller transition {transition!r}")
        text = log_file.name and Path(log_file.name).read_text(encoding="utf-8", errors="replace")
        if transition in text:
            return transition
        time.sleep(0.05)
    raise TimeoutError(f"controller did not report startup transition {transition!r} within {timeout_s:g}s")


def perform_normal_shutdown(proc: subprocess.Popen[str], log_path: Path,
                            capture_id: str, scene: object,
                            ros_process: subprocess.Popen | None,
                            standing_prompt_marker: Path,
                            standing_observation_s: float,
                            wall_cap_s: float) -> dict:
    """Decelerate, confirm standing, and observe a bounded standing hold."""
    request = publish_prepared_command(proc, "NORMAL_SHUTDOWN")
    request_ns = int(request["publish_start_monotonic_ns"])
    observed_physics: list[dict] = []
    standing_hold_physics: list[dict] = []
    seen_sequences: set[int] = set()
    deadline = time.monotonic() + wall_cap_s
    final_events: list[dict] = []
    failure_reason = None
    hold_started_sim_s: float | None = None
    previous_sim_s: float | None = None
    last_sequence = -1
    while time.monotonic() < deadline:
        if ros_process is None or ros_process.poll() is not None:
            failure_reason = "ROS/controller process exited during normal closeout"
            break
        log_text = log_path.read_text(encoding="utf-8", errors="replace")
        final_events = normal_shutdown_events(log_text, request_ns)
        phases = [event["phase"] for event in final_events]
        if "FAILED" in phases or "ABORTED" in phases:
            failure_reason = f"controller reported normal closeout {phases[-1]}"
            break
        safety_line = next((line for line in log_text.splitlines()
                            if EMERGENCY_RE.search(line) or HARDSTOP_RE.search(line) or VETO_RE.search(line)), None)
        if safety_line is not None:
            failure_reason = "controller safety/hard-stop event during normal closeout: " + safety_line.strip()[-500:]
            break

        collision_status, snap = classify_snapshot(
            read_collision_snapshot(), now_ns(), expected_capture_id=capture_id,
            expected_fingerprint=scene.model_fingerprint,
            expected_scene_binding=scene.binding())
        if collision_status is not CollisionStatus.LIVE or snap is None:
            failure_reason = f"collision/physics state became {collision_status.value} during closeout"
            break
        if snap.sequence not in seen_sequences:
            if snap.sequence <= last_sequence:
                failure_reason = "physics snapshot sequence did not advance monotonically"
                break
            seen_sequences.add(snap.sequence)
            last_sequence = snap.sequence
            observed_physics.append({"sequence": snap.sequence, "physics_step": snap.physics_step,
                                     "sim_time_s": snap.sim_time, "base_height_m": snap.base_height_m,
                                     "base_roll_rad": snap.base_roll_rad,
                                     "base_pitch_rad": snap.base_pitch_rad,
                                     "ground_contacts": snap.ground_contacts,
                                     "foot_ground_contacts": snap.foot_ground_contacts,
                                     "physics_coverage_complete": bool(snap.physics_coverage_complete),
                                     "robot_obstacle_contacts": snap.robot_obstacle_contacts,
                                     "fall_candidate": bool(snap.fall_candidate),
                                     "fall_confirmed": bool(snap.fall_confirmed)})
            if "STANDING_HOLD" in phases:
                failure_reason = standing_physics_sample_failure(snap)
                if failure_reason is not None:
                    break
                if previous_sim_s is not None and snap.sim_time <= previous_sim_s:
                    failure_reason = "simulation time paused or moved backward during standing observation"
                    break
                if hold_started_sim_s is None:
                    hold_started_sim_s = snap.sim_time
                    standing_prompt_marker.touch()
                    print("[S2-07] 已到达，保持站立；正在观察仿真状态。", flush=True)
                previous_sim_s = snap.sim_time
                standing_hold_physics.append(observed_physics[-1])
        if snap.fall_confirmed:
            failure_reason = "fall confirmed by the live physics authority during closeout"
            break
        if snap.robot_obstacle_contacts > 0:
            failure_reason = "robot-obstacle contact observed by the live physics authority during closeout"
            break
        if (hold_started_sim_s is not None and previous_sim_s is not None and
                standing_hold_reached(hold_started_sim_s, previous_sim_s, standing_observation_s)):
            return {"request": request, "controller_stages": final_events,
                    "physics_observations": observed_physics,
                    "standing_hold_complete": True,
                    "standing_state": "FIXEDSTAND",
                    "standing_observation_sim_time_s": [hold_started_sim_s, previous_sim_s],
                    "standing_observation_requested_s": standing_observation_s,
                    "standing_observation_elapsed_s": previous_sim_s - hold_started_sim_s,
                    "standing_observation_samples": standing_hold_physics,
                    "passive_requested": False,
                    "lowering_requested": False,
                    "confirmation_scope": "controller continuously checked standing feedback; physics authority supplied sampled posture/contact/coverage; no PASSIVE or lower command sent"}
        time.sleep(0.01)

    return {"request": request, "controller_stages": final_events,
            "physics_observations": observed_physics, "standing_hold_complete": False,
            "standing_state": "UNKNOWN",
            "standing_observation_sim_time_s": ([hold_started_sim_s, previous_sim_s]
                                                  if hold_started_sim_s is not None else None),
            "standing_observation_requested_s": standing_observation_s,
            "standing_observation_elapsed_s": (previous_sim_s - hold_started_sim_s
                                                 if hold_started_sim_s is not None and previous_sim_s is not None else 0.0),
            "standing_observation_samples": standing_hold_physics,
            "failure_reason": failure_reason or "bounded normal closeout observation expired",
            "passive_requested": False, "lowering_requested": False,
            "confirmation_scope": "incomplete; standing hold was not fully observed"}


def hard_stop_confirmation(log_text: str, after_monotonic_ns: int) -> dict | None:
    """Return only a completed PASSIVE transition logged after this stop request."""
    for line in log_text.splitlines():
        match = HARDSTOP_CONFIRMED_RE.search(line)
        if not match or int(match.group("command")) not in (1, 9):
            continue
        steady_ns = int(match.group("steady_ns"))
        if steady_ns >= after_monotonic_ns:
            return {"event": "hard_stop_transition_complete", "command": int(match.group("command")),
                    "state": "PASSIVE", "controller_steady_monotonic_ns": steady_ns,
                    "message": line.strip()[-1000:]}
    return None


def verify_scene_authority_log(log_text: str, scene: object) -> dict:
    events = [line for line in log_text.splitlines() if "[ABS-SCENE-AUTHORITY]" in line]
    if len(events) != 1:
        raise RuntimeError(f"expected one scene authority log, found {len(events)}")
    fields = dict(re.findall(r"([a-z0-9_]+)=([^\s]+)", events[0]))
    expected = {"scene_id": scene.scene_id, "root_sha256": scene.root_sha256,
                "model_closure_sha256": scene.closure_sha256,
                "runtime_model_fingerprint": scene.model_fingerprint,
                "obstacle_count": str(scene.obstacle_count)}
    mismatch = {key: {"observed": fields.get(key), "expected": value}
                for key, value in expected.items() if fields.get(key) != value}
    if mismatch:
        raise RuntimeError(f"loaded MuJoCo scene authority mismatch: {mismatch}")
    return {"status": "confirmed", "fields": expected, "log_line": events[0].strip()}


class FrameCapture:
    def __init__(self, path: Path, run_id: str, capture_id: str, run_start_ns: int,
                 scene: object):
        self.recorder = RunRecordRecorder(str(path), run_id=run_id, capture_id=capture_id,
                                          expected_fingerprint=scene.model_fingerprint,
                                          expected_scene_binding=scene.binding())
        self.stop = threading.Event()
        self.thread: threading.Thread | None = None  # Kept as an explicit no-worker state for closeout facts.
        self._lock = threading.Lock()
        self.error: str | None = None
        self.last_frame_key: tuple[int, int] | None = None
        self.unique_frames = 0
        self.bound_session_id: int | None = None
        self.source_mismatch: str | None = None
        self.last_frame_metadata: dict | None = None
        self.run_start_ns = run_start_ns
        self.pre_run_frames_ignored = 0

    def start(self) -> None:
        self.recorder.start()
        # The supervisor is the sole frame reader/writer. It records each exact
        # snapshot before evaluating it for a terminal event, so terminal capture
        # cannot race an asynchronous sampler or be overtaken by a later cycle.

    def observe(self, raw: bytes, status: FrameStatus, frame: object, sampled_ns: int) -> dict:
        """Persist the exact supervisor snapshot once, before terminal evaluation."""
        if status is not FrameStatus.LIVE or frame is None:
            return {"status": "not_live", "persisted": False}
        with self._lock:
            if self.stop.is_set() or self.recorder.stopped:
                self.error = "capture was stopped before the observed policy frame was persisted"
                return {"status": "failed", "persisted": False, "reason": self.error}
            if frame.session_id < self.run_start_ns:
                self.pre_run_frames_ignored += 1
                return {"status": "pre_run_ignored", "persisted": False}
            key = (frame.session_id, frame.sequence)
            if key == self.last_frame_key:
                return {"status": "already_persisted", "persisted": True,
                        "frame_ref": self.last_frame_metadata}
            if self.bound_session_id is None:
                self.bound_session_id = frame.session_id
            elif frame.session_id != self.bound_session_id:
                self.source_mismatch = f"runtime session changed {self.bound_session_id}->{frame.session_id}"
            if frame.switching_mode != 1 or abs(frame.entry_threshold + 0.05) >= 1e-5 or abs(frame.exit_threshold + 0.05) >= 1e-5:
                self.source_mismatch = (f"unexpected candidate/threshold: mode={frame.switching_mode}, "
                                        f"entry={frame.entry_threshold}, exit={frame.exit_threshold}")
            ref = {"session_id": frame.session_id, "rl_step": frame.rl_step,
                   "source_sequence": frame.sequence,
                   "sim_time_s": frame.sim_time_s if frame.sim_clock_valid else None,
                   "monotonic_ns": frame.monotonic_ns,
                   "same_snapshot_persisted": True,
                   "policy_state": {0: "AGILE", 1: "RECOVERY", 2: "FAULTED"}.get(frame.policy_state, "UNKNOWN")}
            try:
                line = self.recorder.record_snapshot(raw, sampled_ns)
                payload = line.get("payload") or {}
                expected = (frame.session_id, frame.sequence, frame.rl_step, frame.monotonic_ns)
                stored = (payload.get("session_id"), payload.get("source_sequence"),
                          payload.get("rl_step"), payload.get("monotonic_ns"))
                required = ("ra_value", "mode_before", "mode_after", "action_source",
                            "action_raw", "action_clipped", "joint_target_rad", "world_pose",
                            "sim_time_s", "sim_clock_valid", "monotonic_ns")
                if line.get("status") != FrameStatus.LIVE.value or stored != expected or any(k not in payload for k in required):
                    self.error = f"persisted frame does not match observed snapshot: expected={expected}, stored={stored}"
                    return {"status": "failed", "persisted": False, "reason": self.error, "frame_ref": ref}
            except Exception as exc:
                self.error = f"{type(exc).__name__}: {exc}"
                return {"status": "failed", "persisted": False, "reason": self.error, "frame_ref": ref}
            self.last_frame_key = key
            self.last_frame_metadata = {**ref, "persisted": True}
            self.unique_frames += 1
            return {"status": "persisted", "persisted": True, "frame_ref": self.last_frame_metadata,
                    "collision_snapshot": payload.get("collision_snapshot")}

    def finish(self, facts: dict) -> dict:
        self.stop.set()
        if not self.recorder.stopped:
            self.recorder.stop_sampling()
        terminal = self.recorder.finalize(facts)
        return terminal


def finalize_run_closeout(out: Path, context: dict, process_facts: dict,
                          recorder: FrameCapture, capture_started: bool,
                          cleanup_processes) -> tuple[dict | None, list[dict]]:
    """Production closeout path: stop owned groups, finalize capture, persist artifacts.

    Artifact failures are collected and retried into run_context/process_facts. They
    cannot prevent the process cleanup callback from running because cleanup is the
    first closeout action.
    """
    errors: list[dict] = []
    terminal: dict | None = None
    try:
        cleanup_processes()
    except Exception as exc:
        errors.append({"stage": "process_cleanup", "error": f"{type(exc).__name__}: {exc}"})
    for item in process_facts.get("cleanup_errors", []):
        errors.append({"stage": f"process_cleanup:{item.get('stage', 'unknown')}",
                       "error": item.get("error", "cleanup error without detail")})
    if process_facts.get("shutdown_complete") is False:
        errors.append({"stage": "process_cleanup", "error": "owned process exit was not confirmed"})

    context.setdefault("lifecycle", {})
    try:
        write_json(out / "run_context.json", context)
    except Exception as exc:
        errors.append({"stage": "run_context_pre_finalize", "error": f"{type(exc).__name__}: {exc}"})

    if capture_started:
        try:
            terminal = recorder.finish(process_facts)
            context["lifecycle"]["terminal_record_finalized"] = True
        except Exception as exc:
            errors.append({"stage": "recorder_finalize", "error": f"{type(exc).__name__}: {exc}"})
            try:
                recorder.recorder.close()
            except Exception as close_exc:
                errors.append({"stage": "recorder_close", "error": f"{type(close_exc).__name__}: {close_exc}"})
        if terminal is not None:
            try:
                write_json(out / "terminal.json", terminal)
                context["lifecycle"]["terminal_json_saved"] = True
            except Exception as exc:
                errors.append({"stage": "terminal_json", "error": f"{type(exc).__name__}: {exc}"})
            if context.get("session_id") is None and terminal.get("last_session_id") is not None:
                context["session_id"] = terminal["last_session_id"]
    else:
        try:
            recorder.recorder.close()
        except Exception as exc:
            errors.append({"stage": "recorder_close", "error": f"{type(exc).__name__}: {exc}"})

    context["lifecycle"]["closeout_errors"] = errors
    process_facts["closeout_errors"] = errors
    process_facts["closeout_complete"] = not errors and (not capture_started or terminal is not None)
    # Persist process facts and context independently so one failed artifact does
    # not suppress the other. Retry both once with the accumulated error list.
    for path, value, stage in (
        (out / "process_facts.json", process_facts, "process_facts"),
        (out / "run_context.json", context, "run_context_final"),
    ):
        try:
            write_json(path, value)
        except Exception as exc:
            errors.append({"stage": stage, "error": f"{type(exc).__name__}: {exc}"})
    context["lifecycle"]["closeout_errors"] = errors
    process_facts["closeout_errors"] = errors
    process_facts["closeout_complete"] = not errors and (not capture_started or terminal is not None)
    for path, value in ((out / "process_facts.json", process_facts), (out / "run_context.json", context)):
        try:
            write_json(path, value)
        except Exception:
            # Last-resort persistence failures are returned to the caller/stdout;
            # do not raise past the already completed process cleanup.
            pass
    return terminal, errors


def run(args: argparse.Namespace) -> int:
    try:
        scene_spec = resolve_scene(args.scene)
    except (OSError, ValueError) as exc:
        print(json.dumps({"result": "SCENE_PREFLIGHT_REJECTED", "scene": args.scene,
                          "reason": str(exc)}, ensure_ascii=False))
        return 2
    run_id = uuid.uuid4().hex
    capture_id = "p1-10-capture-" + uuid.uuid4().hex
    out = args.output_root / run_id
    out.mkdir(parents=True, exist_ok=False)
    overlay = Path(args.overlay).resolve()
    env = build_env(capture_id, overlay, args.display, args.xauthority, scene_spec, out)
    start_wall_ns = now_ns()
    recorder = FrameCapture(out / "runtime_record.jsonl", run_id, capture_id,
                            start_wall_ns, scene_spec)
    log_path = out / "ros_launch.log"
    mujoco_log_path = out / "mujoco.log"
    ros_log = log_path.open("w", encoding="utf-8")
    mujoco_log = mujoco_log_path.open("w", encoding="utf-8")
    mujoco: subprocess.Popen | None = None
    ros: subprocess.Popen | None = None
    stop_publisher: subprocess.Popen[str] | None = None
    post_terminal_shutdown: dict | None = None
    process_facts = {"exit_code": None, "forced_termination": False,
                     "shutdown_complete": False, "shutdown_request_source": "bounded_diagnostic_supervisor",
                     "external_safety_events": []}
    controls: list[dict] = []
    terminal_cause = "STARTUP_FAILURE"
    movement_start_sim_s: float | None = None
    sim_start_s: float | None = None
    first_pose: tuple[float, float] | None = None
    previous_pose_time_s: float | None = None
    first_frame_ns: int | None = None
    last_frame_key: tuple[int, int] | None = None
    frame_gap_count = 0
    safety_line_offset = 0
    start_event_ns: int | None = None
    finish_event_ns: int | None = None
    terminal_event_ns: int | None = None
    frame_rows = 0
    collision_live_seen = False
    startup_error: str | None = None
    capture_started = False
    bound_session_id: int | None = None
    source_mismatch: str | None = None
    source_wait_started: float | None = None
    terminal_frame_ref: dict | None = None
    terminal_summary: dict | None = None
    lifecycle_events: list[dict] = []

    context = {
        "schema": "s1-run-context-v1", "run_id": run_id, "capture_id": capture_id,
        "classification": ("controller initialization config-consumption preflight; no RL requested"
                            if args.config_only else args.classification),
        "created_monotonic_ns": start_wall_ns,
        "settings": {"scene": str(scene_spec.path), "scene_filename": scene_spec.filename,
                     "scene_id": scene_spec.scene_id,
                     "scene_obstacle_count": scene_spec.obstacle_count,
                     "scene_root_sha256": scene_spec.root_sha256,
                     "scene_model_closure_sha256": scene_spec.closure_sha256,
                     "runtime_model_fingerprint": scene_spec.model_fingerprint,
                     "candidate": "paper_faithful_switch",
                     "goal_world_m": [args.goal_x, 0.0], "arrival_threshold_m": 0.5,
                     "diagnostic_sim_limit_after_motion_s": args.sim_limit_s,
                     "standing_observation_sim_s": args.standing_observation_sim_s,
                     "standing_observation_wall_cap_s": args.standing_observation_wall_cap_s,
                     "controller_config": str(overlay / "share/go2_description/config/abs/config.yaml"),
                     "model_paths": [str(overlay / "share/go2_description/config/abs/policy.pt"),
                                     str(overlay / "share/go2_description/config/abs/ra_value.pt"),
                                     str(overlay / "share/go2_description/config/rec/policy.pt")],
                     "display": env.get("DISPLAY"),
                     "xauthority": env.get("XAUTHORITY"), "overlay": str(overlay),
                     "fall_rule": {"base_body": "base_link", "height_threshold_m": 0.22,
                                   "tilt_threshold_deg": 60.0, "duration_sim_s": 0.30,
                                   "combination": "height OR absolute roll OR absolute pitch",
                                   "duration_boundary": "candidate must hold continuously for >=0.30 valid sim seconds"},
                     "collision_scope": "PhysicsLoop mj_step; bound robot-obstacle contact episodes; ground, self, other, and unknown contacts retained separately"},
        "safety_result_contract": {
            "motion_start": "first valid policy pose displacement >=0.10m from initial pose",
            "collision_terminal": "immediate once an accumulated robot-obstacle episode intersects inclusive motion interval",
            "terminal_priority_same_observation": ["SYSTEM_SAFETY_ABORT", "COLLISION_TERMINATION",
                                                    "FALL_TERMINATION", "ARRIVED",
                                                    "DIAGNOSTIC_SIM_TIME_LIMIT", "WALL_CLOCK_GUARD_INCOMPLETE"],
            "cleanup_is_not_motion": True,
            "policy_stop_request_is_not_stop_confirmation": True,
            "mujoco_command_consumption": "UNKNOWN",
        },
        "lifecycle": {"motion_start_event": "first observed base displacement >=0.10m from first valid strategy-frame pose",
                      "motion_start_sim_time_s": None,
                      "motion_start_interval_s": None,
                      "run_terminal_event": None,
                      "movement_end_monotonic_ns": None,
                      "cleanup_complete_monotonic_ns": None,
                      "terminal_frame_capture": {"status": "not_terminal", "complete": None},
                      "motion_and_cleanup_are_separate": True},
        "commands": controls,
        "preflight": {"scene_binding": scene_spec.binding(),
                      "mujoco_launch_command": [str(MUJOCO_BIN), "-s", scene_spec.filename],
                      "ray_collision_geometry_selector_shared": True},
        "events": lifecycle_events,
    }
    write_json(out / "run_context.json", context)

    try:
        existing_processes = matching_processes({"ros2_control_node", "ros2_control_no", "controller_manager"})
        controller_process_lines = existing_processes
        if controller_process_lines:
            raise RuntimeError("pre-existing controller-manager process found; left untouched: " + " | ".join(str(item) for item in controller_process_lines))
        package_prefix = ros_command("ros2 pkg prefix go2_description", env)
        if package_prefix.returncode != 0 or Path(package_prefix.stdout.strip()).resolve() != overlay.resolve():
            raise RuntimeError(f"go2_description overlay was not selected: rc={package_prefix.returncode}, prefix={package_prefix.stdout.strip()!r}")
        controller_prefix = ros_command("ros2 pkg prefix rl_quadruped_controller", env)
        expected_controller_prefix = (ROS_WS / "install/rl_quadruped_controller").resolve()
        if controller_prefix.returncode != 0 or Path(controller_prefix.stdout.strip()).resolve() != expected_controller_prefix:
            raise RuntimeError(f"controller package prefix mismatch in launch shell: rc={controller_prefix.returncode}, prefix={controller_prefix.stdout.strip()!r}")
        share_check = ros_command(
            "python3 -c 'from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory(\"go2_description\"))'", env)
        if share_check.returncode != 0 or Path(share_check.stdout.strip()).resolve() != (overlay / "share/go2_description").resolve():
            raise RuntimeError(f"launch-equivalent ament share lookup differs from overlay: {share_check.stdout.strip()!r}")
        config_path = overlay / "share/go2_description/config/abs/config.yaml"
        config_text = config_path.read_text(encoding="utf-8")
        expected_goal = f"{args.goal_x:g}"
        goal_pattern = rf"(?m)^\s*goal_x:\s*{re.escape(expected_goal)}(?:\.0)?\s*(?:#.*)?$"
        switching_block = re.search(r"(?ms)^  switching:\s*\n((?:^    .*\n|^\s*#.*\n)*)", config_text)
        switching_text = switching_block.group(1) if switching_block else ""
        legacy_keys = re.findall(r"(?m)^  (?:switching_mode|ra_threshold|recovery_hold_steps):", config_text)
        if (not re.search(r'(?m)^    group:\s*["\']?A["\']?\s*(?:#.*)?$', switching_text)
                or not re.search(r"(?m)^    entry_threshold:\s*-0\.05\s*(?:#.*)?$", switching_text)
                or legacy_keys or not re.search(goal_pattern, config_text)):
            raise RuntimeError("overlay config does not satisfy abs.switching A/E contract or expected goal")
        missing_models = [path for path in context["settings"]["model_paths"] if not Path(path).is_file()]
        if missing_models:
            raise RuntimeError("expected deployment model path missing: " + ", ".join(missing_models))
        context["preflight"] = {"controller_processes_before_launch": controller_process_lines,
                                "launch_shell_go2_description_prefix": package_prefix.stdout.strip(),
                                "launch_shell_controller_prefix": controller_prefix.stdout.strip(),
                                "ament_share_lookup_same_shell": share_check.stdout.strip(),
                                "candidate_config_path": str(config_path),
                                "candidate_config_group_A_verified": True,
                                "candidate_config_entry_E_verified": True,
                                "candidate_config_legacy_fields_absent": True,
                                "candidate_config_goal_x_m_verified": True,
                                "launch_command": "source /opt/ros/humble/setup.bash; source workspace/install/setup.bash; export the same AMENT_PREFIX_PATH; ros2 launch ..."}
        write_json(out / "run_context.json", context)
        recorder.start()
        capture_started = True
        lifecycle_events.append({"phase": "startup_preparation", "event": "recorder_ready", "monotonic_ns": now_ns()})
        mujoco = subprocess.Popen([str(MUJOCO_BIN), "-s", scene_spec.filename], cwd=MUJOCO_DIR,
                                  env=env, stdout=mujoco_log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        time.sleep(3.0)
        if mujoco.poll() is not None:
            raise RuntimeError(f"MuJoCo exited at startup with code {mujoco.returncode}")
        lifecycle_events.append({"phase": "startup_preparation", "event": "mujoco_started", "monotonic_ns": now_ns()})
        # Require a correctly bound LIVE collision authority before controller/motion startup.
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            status, snap = classify_snapshot(read_collision_snapshot(), now_ns(),
                                             expected_capture_id=capture_id,
                                             expected_fingerprint=scene_spec.model_fingerprint,
                                             expected_scene_binding=scene_spec.binding())
            if status is CollisionStatus.LIVE and snap is not None:
                collision_live_seen = True
                break
            time.sleep(0.02)
        if not collision_live_seen:
            raise RuntimeError("collision authority did not become LIVE for this capture/model; no motion started")
        context["preflight"]["loaded_mujoco_scene_authority"] = verify_scene_authority_log(
            mujoco_log_path.read_text(encoding="utf-8", errors="replace"), scene_spec)
        context["preflight"]["collision_reader_scene_binding"] = {
            "status": "LIVE", "scene_id": snap.scenario_id,
            "root_sha256": snap.scene_root_sha256,
            "model_closure_sha256": snap.model_closure_sha256,
            "runtime_model_fingerprint": snap.runtime_model_fingerprint,
            "obstacle_count": scene_spec.obstacle_count}
        write_json(out / "run_context.json", context)
        lifecycle_events.append({"phase": "startup_preparation", "event": "collision_source_live", "monotonic_ns": now_ns()})
        launch_shell = (f"source /opt/ros/humble/setup.bash && source {ROS_WS}/install/setup.bash && "
                        f"export AMENT_PREFIX_PATH={shlex.quote(env['S1_04_OVERLAY'])}:$AMENT_PREFIX_PATH && "
                        "ros2 launch rl_quadruped_controller mujoco.launch.py simulation_test:=false")
        ros = subprocess.Popen(["bash", "-lc", launch_shell],
                               cwd=ROS_WS, env=env, stdout=ros_log, stderr=subprocess.STDOUT,
                               start_new_session=True)
        time.sleep(8.0)
        if ros.poll() is not None:
            raise RuntimeError(f"ROS launch exited at startup with code {ros.returncode}")
        ready = False
        ready_deadline = time.monotonic() + 15.0
        while time.monotonic() < ready_deadline:
            check = ros_command("ros2 control list_controllers", env, timeout=6.0)
            if "rl_quadruped_controller" in check.stdout and "active" in check.stdout:
                ready = True
                break
            if ros.poll() is not None:
                break
            time.sleep(1.0)
        if not ready:
            raise RuntimeError("controller did not become active within 15 seconds")
        lifecycle_events.append({"phase": "startup_preparation", "event": "controller_active", "monotonic_ns": now_ns()})
        node_check = ros_command("ros2 node list", env)
        controller_nodes = [line.strip() for line in node_check.stdout.splitlines()
                            if "controller_manager" in line or "rl_quadruped_controller" in line]
        launch_pgid = os.getpgid(ros.pid) if ros.pid is not None else None
        launched_cm = [proc for proc in matching_processes({"ros2_control_node", "ros2_control_no", "controller_manager"})
                       if proc["pgid"] == launch_pgid]
        if len(launched_cm) != 1:
            raise RuntimeError(f"expected one controller-manager writer process in this launch group, found {launched_cm!r}")
        controller_list = ros_command("ros2 control list_controllers", env)
        if node_check.returncode != 0 or controller_list.returncode != 0 or controller_nodes.count("/controller_manager") != 1:
            raise RuntimeError(f"controller manager not unique: nodes={controller_nodes!r}; controllers={controller_list.stdout!r}")
        context["preflight"].update({"controller_nodes": controller_nodes,
                                      "active_controller_list": controller_list.stdout.strip(),
                                      "launched_manager_process_evidence": launched_cm})
        stop_publisher, publisher_ready = start_prepared_stop_publisher(env)
        context["preflight"]["prepared_passive_publisher"] = {
            "status": "ready_before_rl", "helper_pid": stop_publisher.pid,
            "ready_monotonic_ns": publisher_ready.get("monotonic_ns"),
            "subscriber_count": publisher_ready.get("subscribers"),
            "topic": "/control_input", "command": 1,
        }
        lifecycle_events.append({"phase": "startup_preparation", "event": "passive_publisher_ready",
                                 "monotonic_ns": publisher_ready.get("monotonic_ns"),
                                 "helper_pid": stop_publisher.pid,
                                 "subscriber_count": publisher_ready.get("subscribers")})
        write_json(out / "run_context.json", context)
        context["lifecycle"].update({"mujoco_started_monotonic_ns": now_ns(),
                                      "ros_launch_started_monotonic_ns": now_ns(),
                                      "controller_ready_monotonic_ns": now_ns()})
        if args.config_only:
            managers = context["preflight"].get("launched_manager_process_evidence", [])
            expected_pid = int(managers[0]["pid"]) if len(managers) == 1 else None
            source_expected = {
                "package_share": str(overlay / "share/go2_description"),
                "config": context["settings"]["controller_config"],
                "launch_config": str(overlay / "share/go2_description/config/robot_control.yaml"),
                "controller_cmdline": managers[0].get("cmdline", "") if len(managers) == 1 else "",
                "agile_model": context["settings"]["model_paths"][0],
                "ra_model": context["settings"]["model_paths"][1],
                "recovery_model": context["settings"]["model_paths"][2],
                "candidate": "paper_faithful_switch", "group": "A",
                "entry_threshold": -0.05, "exit_threshold": -0.05,
                "hysteresis": "false", "hold": "false",
            }
            wait_started = time.monotonic()
            while True:
                ros_log.flush()
                binding = verify_config_consumption(log_path.read_text(encoding="utf-8", errors="replace"),
                                                    source_expected, expected_pid)
                if binding["status"] != "pending" or time.monotonic() - wait_started >= SOURCE_EVIDENCE_WAIT_S:
                    break
                time.sleep(0.1)
            context["runtime_config_observed"] = {
                "status": binding["status"], "reason": binding["reason"],
                "evidence": binding.get("evidence"), "no_rl_requested": True,
            }
            write_json(out / "run_context.json", context)
            if binding["status"] != "confirmed":
                terminal_cause = "INVALID_RUNTIME_SOURCE"
                source_mismatch = f"initialization config evidence {binding['status']}: {binding['reason']}"
                raise RuntimeError(source_mismatch)
            terminal_cause = "CONFIG_PREFLIGHT_ONLY"
            finish_event_ns = now_ns()
            context["lifecycle"]["run_terminal_event"] = terminal_cause
            context["lifecycle"]["movement_end_monotonic_ns"] = finish_event_ns
            context["lifecycle"]["movement_phase_started"] = False
            lifecycle_events.append({"phase": "startup_preparation", "event": "A_config_consumption_confirmed",
                                     "monotonic_ns": finish_event_ns, "evidence": binding["evidence"]})
            raise ConfigOnlyComplete()
        controls.append(publish(env, 2))
        wait_controller_transition(ros_log, ros, "Switched from passive to fixed down", 20.0)
        controls.append(publish(env, 2))
        wait_controller_transition(ros_log, ros, "Switched from fixed down to fixed stand", 20.0)
        # The switch log is emitted when the old state's checkChange chooses
        # the next state, one controller cycle before FIXEDSTAND.enter() clears
        # the input command. Wait for an actual FIXEDSTAND run log so command 3
        # cannot be overwritten by that enter() reset.
        wait_controller_transition(ros_log, ros, "[BaseFixedStand]: [STAND-SYMM]", 20.0)
        ros_log.flush()
        loan_log_text = log_path.read_text(encoding="utf-8", errors="replace")
        loan_matches = [(int(index), name) for index, name in
                        re.findall(r"\[FOOT-LOANED\] index=(\d+) name=([^\s]+)", loan_log_text)]
        expected_loan_names = ["foot_force/FR", "foot_force/FL", "foot_force/RR", "foot_force/RL"]
        ordered_loan_names = [name for index, name in sorted(loan_matches)]
        if (len(loan_matches) != 4 or [index for index, _ in sorted(loan_matches)] != [0, 1, 2, 3]
                or ordered_loan_names != expected_loan_names):
            context["preflight"]["foot_force_loaned_interfaces"] = {
                "status": "mismatch_or_missing", "observed": loan_matches,
                "expected": expected_loan_names,
                "rl_entry_blocked": True,
            }
            write_json(out / "run_context.json", context)
            raise RuntimeError(f"loaned foot-force interface order mismatch; RL entry blocked: {loan_matches!r}")
        context["preflight"]["foot_force_loaned_interfaces"] = {
            "status": "confirmed", "observed": loan_matches,
            "ordered_names": ordered_loan_names,
            "expected": expected_loan_names,
            "monotonic_ns": now_ns(), "rl_entry_blocked_on_failure": True,
        }
        lifecycle_events.append({"phase": "startup_preparation",
                                 "event": "foot_force_loan_order_confirmed",
                                 "monotonic_ns": context["preflight"]["foot_force_loaned_interfaces"]["monotonic_ns"],
                                 "ordered_names": ordered_loan_names})
        write_json(out / "run_context.json", context)
        controls.append(publish(env, 3))
        lifecycle_events.append({"phase": "motion", "event": "rl_policy_requested", "monotonic_ns": now_ns(), "command": 3})
        start_event_ns = now_ns()
        context["lifecycle"]["rl_entry_command_monotonic_ns"] = start_event_ns
        context["commands"] = controls
        write_json(out / "run_context.json", context)
        terminal_cause = "INCOMPLETE"
        run_deadline = time.monotonic() + args.wall_cap_s
        while time.monotonic() < run_deadline:
            ros_log.flush()
            new_text = log_path.read_text(encoding="utf-8", errors="replace")
            lines = new_text[safety_line_offset:].splitlines()
            safety_line_offset = len(new_text)
            safety = next((line for line in lines if EMERGENCY_RE.search(line) or HARDSTOP_RE.search(line) or VETO_RE.search(line)), None)
            if safety:
                kind = "FROZEN_POSITION_PASSIVE" if EMERGENCY_RE.search(safety) else ("GLOBAL_HARD_STOP" if HARDSTOP_RE.search(safety) else "POLICY_SAFETY_VETO")
                observer_ns = now_ns()
                detection_match = re.search(r"detection_ns=(\d+)", safety)
                terminal_event_ns = int(detection_match.group(1)) if detection_match else observer_ns
                process_facts["external_safety_events"].append({
                    "event": kind, "source": "RlQuadrupedController/StateRL ROS log",
                    "message": safety.strip()[:1000], "observed_monotonic_ns": terminal_event_ns,
                    "observer_received_monotonic_ns": observer_ns})
                collision_status, collision_snapshot = classify_snapshot(
                    read_collision_snapshot(), observer_ns,
                    expected_capture_id=capture_id,
                    expected_fingerprint=scene_spec.model_fingerprint,
                    expected_scene_binding=scene_spec.binding())
                process_facts["external_safety_events"][-1]["collision_snapshot"] = (
                    collision_snapshot_payload(collision_status, collision_snapshot))
                terminal_cause = "SYSTEM_SAFETY_ABORT"
                break
            if ros.poll() is not None or mujoco.poll() is not None:
                terminal_cause = "PROCESS_EXIT_UNEXPECTED"
                break

            raw = read_shm_frame()
            frame_sampled_ns = now_ns()
            status, frame = classify_frame(raw, frame_sampled_ns)
            if status is FrameStatus.LIVE and frame is not None:
                capture_result = recorder.observe(raw, status, frame, frame_sampled_ns)
                if not capture_result.get("persisted"):
                    terminal_cause = "RECORDING_FAILURE"
                    terminal_frame_ref = capture_result.get("frame_ref")
                    context["lifecycle"]["terminal_frame_capture"] = {
                        "status": "failed", "complete": False,
                        "reason": capture_result.get("reason", "frame snapshot was not persisted"),
                    }
                    terminal_event_ns = frame.monotonic_ns
                    break
                key = (frame.session_id, frame.sequence)
                if key != last_frame_key:
                    frame_rows += 1
                    last_frame_key = key
                    if recorder.source_mismatch is not None:
                        source_mismatch = recorder.source_mismatch
                        terminal_cause = "INVALID_RUNTIME_SOURCE"
                        terminal_frame_ref = capture_result["frame_ref"]
                        terminal_event_ns = frame.monotonic_ns
                        break
                    if bound_session_id is None:
                        ros_text = log_path.read_text(encoding="utf-8", errors="replace")
                        managers = context["preflight"].get("launched_manager_process_evidence", [])
                        expected_pid = int(managers[0]["pid"]) if len(managers) == 1 else None
                        source_expected = {
                            "package_share": str(overlay / "share/go2_description"),
                            "config": context["settings"]["controller_config"],
                            "launch_config": str(overlay / "share/go2_description/config/robot_control.yaml"),
                            "controller_cmdline": managers[0].get("cmdline", "") if len(managers) == 1 else "",
                            "agile_model": context["settings"]["model_paths"][0],
                            "ra_model": context["settings"]["model_paths"][1],
                            "recovery_model": context["settings"]["model_paths"][2],
                            "candidate": "paper_faithful_switch", "candidate_mode": 1, "group": "A",
                            "entry_threshold": -0.05, "exit_threshold": -0.05,
                            "hysteresis": "false", "hold": "false",
                            "goal_x": args.goal_x, "goal_y": 0.0,
                        }
                        frame_binding = {"session_id": frame.session_id, "switching_mode": frame.switching_mode,
                                         "entry_threshold": frame.entry_threshold,
                                         "exit_threshold": frame.exit_threshold}
                        binding = verify_source_binding(ros_text, source_expected, frame_binding, expected_pid)
                        context["runtime_source_validation"] = {
                            "status": binding["status"], "reason": binding["reason"],
                            "wait_limit_s": SOURCE_EVIDENCE_WAIT_S,
                            "frame_session_id": frame.session_id,
                        }
                        if binding["status"] == "pending":
                            if source_wait_started is None:
                                source_wait_started = time.monotonic()
                            write_json(out / "run_context.json", context)
                            if time.monotonic() - source_wait_started < SOURCE_EVIDENCE_WAIT_S:
                                continue
                            source_mismatch = f"source evidence timeout ({binding['reason']})"
                        elif binding["status"] == "conflict":
                            source_mismatch = f"source evidence conflict ({binding['reason']}): {binding['evidence']}"
                        else:
                            bound_session_id = frame.session_id
                            context["session_id"] = bound_session_id
                            context["runtime_source_validation"] = {
                                "status": "confirmed", "reason": binding["reason"],
                                "wait_limit_s": SOURCE_EVIDENCE_WAIT_S,
                                "evidence": binding["evidence"],
                            }
                            context["runtime_config_observed"] = {
                                "session_id": bound_session_id,
                                "switching_mode": frame.switching_mode,
                                "group": "A",
                                "entry_threshold": frame.entry_threshold,
                                "exit_threshold": frame.exit_threshold,
                                "hysteresis": False,
                                "hold": False,
                                "goal_log_matches_expected": True,
                                "source_binding": binding["evidence"],
                            }
                            lifecycle_events.append({"phase": "motion", "event": "runtime_source_bound",
                                "monotonic_ns": now_ns(), "session_id": bound_session_id,
                                "candidate": "paper_faithful_switch", "entry_threshold": frame.entry_threshold,
                                "exit_threshold": frame.exit_threshold, "goal_world_m": [args.goal_x, 0.0],
                                "source_evidence_wait_s": time.monotonic() - source_wait_started
                                    if source_wait_started is not None else 0.0})
                            write_json(out / "run_context.json", context)
                        if source_mismatch is not None:
                            terminal_cause = "INVALID_RUNTIME_SOURCE"
                            terminal_frame_ref = capture_result["frame_ref"]
                            terminal_event_ns = frame.monotonic_ns
                            break
                    elif frame.session_id != bound_session_id:
                        source_mismatch = f"runtime session changed {bound_session_id}->{frame.session_id}"
                        terminal_cause = "INVALID_RUNTIME_SOURCE"
                        terminal_frame_ref = capture_result["frame_ref"]
                        break
                    if frame.switching_mode != 1 or abs(frame.entry_threshold + 0.05) >= 1e-5 or abs(frame.exit_threshold + 0.05) >= 1e-5:
                        source_mismatch = "runtime candidate or threshold changed after initial binding"
                        terminal_cause = "INVALID_RUNTIME_SOURCE"
                        terminal_frame_ref = capture_result["frame_ref"]
                        break
                    if sim_start_s is None and frame.sim_clock_valid:
                        sim_start_s = frame.sim_time_s
                        first_frame_ns = frame.monotonic_ns
                        first_pose = (frame.world_pose[0], frame.world_pose[1])
                    safety_now = frame.safety_faulted or frame.policy_state == 2
                    motion_collision_events = []
                    confirmed_fall = False
                    arrived = False
                    sim_timeout = False
                    distance = None
                    collision_payload = capture_result.get("collision_snapshot")
                    if frame.sim_clock_valid and sim_start_s is not None and first_pose is not None:
                        displacement = math.dist((frame.world_pose[0], frame.world_pose[1]), first_pose)
                        if movement_start_sim_s is None and displacement >= 0.10:
                            movement_start_sim_s = frame.sim_time_s
                            context["lifecycle"]["motion_start_event"] = "first valid pose >=0.10m from first valid policy pose"
                            context["lifecycle"]["motion_start_observed_sim_time_s"] = movement_start_sim_s
                            context["lifecycle"]["motion_start_interval_s"] = [previous_pose_time_s, frame.sim_time_s]
                            lifecycle_events.append({"phase": "motion", "event": "motion_start",
                                                     "monotonic_ns": frame.monotonic_ns,
                                                     "session_id": frame.session_id, "rl_step": frame.rl_step,
                                                     "sim_time_s": frame.sim_time_s,
                                                     "displacement_from_initial_m": displacement})
                        if (movement_start_sim_s is not None and isinstance(collision_payload, dict)
                                and collision_payload.get("status") == CollisionStatus.LIVE.value):
                            motion_collision_events = [
                                event for event in collision_payload.get("collision_history", [])
                                if isinstance(event, dict)
                                and event.get("end_sim_time", -math.inf) >= movement_start_sim_s
                                and event.get("start_sim_time", math.inf) <= frame.sim_time_s
                            ]
                            motion_fall_events = [
                                event for event in collision_payload.get("fall_history", [])
                                if isinstance(event, dict)
                                and event.get("end_sim_time", -math.inf) >= movement_start_sim_s
                                and event.get("start_sim_time", math.inf) <= frame.sim_time_s
                            ]
                            confirmed_fall_events = [event for event in motion_fall_events
                                                     if event.get("confirmed") is True]
                            confirmed_fall = bool(confirmed_fall_events)
                        if motion_collision_events:
                            for event in motion_collision_events:
                                lifecycle_events.append({"phase": "motion", "event": "nonfoot_obstacle_collision",
                                    "source": "PhysicsLoop cumulative collision episode",
                                    "session_id": frame.session_id, "policy_cycle": frame.rl_step,
                                    "physics_start_step": event.get("start_physics_step"),
                                    "physics_end_step": event.get("end_physics_step"),
                                    "start_sim_time_s": event.get("start_sim_time"),
                                    "end_sim_time_s": event.get("end_sim_time"),
                                    "robot_geom_id": event.get("robot_geom_id"),
                                    "obstacle_geom_id": event.get("obstacle_geom_id"),
                                    "robot_geom": event.get("robot_geom_name"),
                                    "robot_body_id": event.get("robot_body_id"),
                                    "robot_body": event.get("robot_body_name"),
                                    "obstacle_geom": event.get("obstacle_geom_name")})
                        if confirmed_fall:
                            for fall_event in confirmed_fall_events:
                                lifecycle_events.append({"phase": "motion", "event": "fall_confirmed",
                                    "source": "PhysicsLoop base_link posture rule cumulative episode",
                                    "session_id": frame.session_id, "policy_cycle": frame.rl_step,
                                    "physics_step": fall_event.get("confirmed_physics_step"),
                                    "sim_time_s": fall_event.get("confirmed_sim_time"),
                                    "start_physics_step": fall_event.get("start_physics_step"),
                                    "start_sim_time_s": fall_event.get("start_sim_time")})
                        distance = math.dist((frame.world_pose[0], frame.world_pose[1]), (args.goal_x, 0.0))
                        arrived = distance <= 0.5
                        sim_timeout = (movement_start_sim_s is not None and
                                       frame.sim_time_s - movement_start_sim_s >= args.sim_limit_s)
                    selected_terminal = select_strategy_terminal(
                        safety_now, bool(motion_collision_events), confirmed_fall, arrived, sim_timeout)
                    if selected_terminal is not None:
                        terminal_cause = selected_terminal
                        terminal_frame_ref = capture_result["frame_ref"]
                        terminal_event_ns = frame.monotonic_ns
                        closeout_snapshot = collision_payload if isinstance(collision_payload, dict) else {}
                        normal_closeout_eligible, closeout_gate_reason = normal_closeout_gate(
                            terminal_cause, safety_now, motion_collision_events, confirmed_fall,
                            collision_payload if isinstance(collision_payload, dict) else None)
                        terminal_summary = {
                            "terminal_frame_session_id": frame.session_id,
                            "terminal_frame_rl_step": frame.rl_step,
                            "terminal_frame_sequence": frame.sequence,
                            "terminal_frame_sim_time_s": frame.sim_time_s if frame.sim_clock_valid else None,
                            "terminal_world_xy_m": list(frame.world_pose[:2]),
                            "goal_world_xy_m": [args.goal_x, 0.0],
                            "distance_to_goal_m": distance,
                            "normal_closeout_eligible": normal_closeout_eligible,
                            "normal_closeout_ineligible_reason": closeout_gate_reason,
                            "terminal_contact_objects": [
                                {"robot_geom_id": event.get("robot_geom_id"),
                                 "robot_geom_name": event.get("robot_geom_name"),
                                 "obstacle_geom_id": event.get("obstacle_geom_id"),
                                 "obstacle_geom_name": event.get("obstacle_geom_name"),
                                 "start_physics_step": event.get("start_physics_step"),
                                 "end_physics_step": event.get("end_physics_step"),
                                 "start_sim_time_s": event.get("start_sim_time"),
                                 "end_sim_time_s": event.get("end_sim_time")}
                                for event in motion_collision_events
                            ],
                        }
                        if safety_now:
                            lifecycle_events.append({"phase": "motion", "event": "SYSTEM_SAFETY_ABORT",
                                "source": "strategy frame safety_faulted/policy_state",
                                "session_id": frame.session_id, "policy_cycle": frame.rl_step})
                        if motion_collision_events:
                            context["lifecycle"]["collision_observed"] = motion_collision_events
                        if confirmed_fall and isinstance(collision_payload, dict):
                            context["lifecycle"]["fall_observed"] = confirmed_fall_events
                        if arrived:
                            context["lifecycle"]["arrival_observed"] = {"session_id": frame.session_id,
                                "rl_step": frame.rl_step, "sim_time_s": frame.sim_time_s,
                                "distance_m": distance, "threshold_m": 0.5}
                        context["lifecycle"]["normal_closeout_gate"] = {
                            "eligible": normal_closeout_eligible,
                            "terminal": terminal_cause,
                            "physics_status": closeout_snapshot.get("status"),
                            "unknown_contacts": closeout_snapshot.get("unknown_contacts"),
                            "fall_candidate": closeout_snapshot.get("fall_candidate"),
                            "fall_confirmed": closeout_snapshot.get("fall_confirmed"),
                            "robot_obstacle_contacts": closeout_snapshot.get("robot_obstacle_contacts"),
                            "reason": closeout_gate_reason}
                        break
                    if frame.sim_clock_valid and distance is not None:
                        previous_pose_time_s = frame.sim_time_s
                    context["session_id"] = frame.session_id
            time.sleep(0.002)
        else:
            terminal_cause = "WALL_CLOCK_GUARD_INCOMPLETE"
        finish_event_ns = terminal_event_ns or now_ns()
        context["lifecycle"]["run_terminal_event"] = terminal_cause
        context["lifecycle"]["movement_end_monotonic_ns"] = finish_event_ns
        context["lifecycle"]["terminal_frame_ref"] = terminal_frame_ref
        context["lifecycle"]["movement_end_sim_time_s"] = terminal_frame_ref.get("sim_time_s") if terminal_frame_ref else None
        if terminal_frame_ref is not None and terminal_cause != "RECORDING_FAILURE":
            context["lifecycle"]["terminal_frame_capture"] = {
                "status": "persisted_same_snapshot", "complete": True,
                "identity": {key: terminal_frame_ref.get(key) for key in
                             ("session_id", "rl_step", "source_sequence", "monotonic_ns")},
            }
        if terminal_cause == "INVALID_RUNTIME_SOURCE":
            context["lifecycle"]["source_mismatch"] = source_mismatch
    except ConfigOnlyComplete:
        context["lifecycle"]["run_terminal_event"] = terminal_cause
        context["lifecycle"]["movement_end_monotonic_ns"] = finish_event_ns
    except Exception as exc:
        startup_error = f"{type(exc).__name__}: {exc}"
        if terminal_cause != "INVALID_RUNTIME_SOURCE":
            terminal_cause = "STARTUP_FAILURE"
        finish_event_ns = now_ns()
        context["lifecycle"]["run_terminal_event"] = terminal_cause
        context["lifecycle"]["startup_error"] = startup_error
    except KeyboardInterrupt:
        terminal_cause = "OPERATOR_ABORT"
        finish_event_ns = now_ns()
        context["lifecycle"]["run_terminal_event"] = terminal_cause
        context["lifecycle"]["movement_end_monotonic_ns"] = finish_event_ns
    finally:
        # Close the motion-frame window before any cleanup request or process teardown.
        if capture_started and not recorder.recorder.stopped:
            recorder.stop.set()
            recorder.recorder.stop_sampling()
        if context.get("session_id") is None and recorder.bound_session_id is not None:
            context["session_id"] = recorder.bound_session_id
        if terminal_frame_ref is None and terminal_cause == "SYSTEM_SAFETY_ABORT" and recorder.last_frame_metadata is not None:
            context["lifecycle"]["last_frame_before_external_safety_event"] = recorder.last_frame_metadata
            context["lifecycle"]["terminal_frame_capture"] = {
                "status": "no_strategy_snapshot_for_external_event", "complete": False,
                "reason": "external safety event was observed from ROS log, not from the persisted strategy snapshot",
            }
        lifecycle_events.append({"phase": "motion_end", "event": terminal_cause,
                                 "monotonic_ns": finish_event_ns or now_ns(),
                                 "frame_ref": terminal_frame_ref})
        # PASSIVE uses the publisher that was already ready before RL entry. The
        # movement recorder is closed above; cleanup observations stay separate.
        post_terminal_stop = None
        cleanup_frame_observations: list[dict] = []
        if ros is not None and ros.poll() is None and stop_publisher is not None and stop_publisher.poll() is None:
            try:
                if context.get("lifecycle", {}).get("normal_closeout_gate", {}).get("eligible") is True:
                    post_terminal_shutdown = perform_normal_shutdown(
                        stop_publisher, log_path, capture_id, scene_spec, ros,
                        out / "standing_prompt.marker",
                        args.standing_observation_sim_s,
                        args.standing_observation_wall_cap_s)
                    context["lifecycle"]["normal_shutdown"] = post_terminal_shutdown
                    standing_hold_complete = post_terminal_shutdown.get("standing_hold_complete") is True
                    context["lifecycle"]["standing_observation"] = {
                        "not_part_of_motion_capture_or_metrics": True,
                        "requested_sim_time_s": args.standing_observation_sim_s,
                        "wall_cap_s": args.standing_observation_wall_cap_s,
                        "observed_sim_time_s": post_terminal_shutdown.get("standing_observation_sim_time_s"),
                        "observed_elapsed_sim_s": post_terminal_shutdown.get("standing_observation_elapsed_s"),
                        "sample_count": len(post_terminal_shutdown.get("standing_observation_samples", [])),
                        "samples": post_terminal_shutdown.get("standing_observation_samples", []),
                        "complete": standing_hold_complete,
                    }
                    lifecycle_events.append({"phase": "cleanup", "event": "NORMAL_SHUTDOWN_REQUESTED",
                        "monotonic_ns": post_terminal_shutdown["request"]["publish_start_monotonic_ns"],
                        "command": 6, "confirmed": post_terminal_shutdown.get("standing_hold_complete", False)})
                    lifecycle_events.extend({"phase": "cleanup", **event}
                        for event in post_terminal_shutdown["controller_stages"])
                    context["lifecycle"]["normal_shutdown_physics_observation"] = {
                        "not_part_of_motion_capture_or_metrics": True,
                        "samples": post_terminal_shutdown["physics_observations"],
                        "fall_or_robot_obstacle_contact_vetoed": any(
                            token in (post_terminal_shutdown.get("failure_reason") or "")
                            for token in ("fall", "robot-obstacle contact")),
                        "standing_hold_failure_reason": post_terminal_shutdown.get("failure_reason")}
                    if standing_hold_complete:
                        post_terminal_stop = {"not_requested": True,
                            "controller_state_before_simulator_stop": "FIXEDSTAND",
                            "reason": "bounded standing hold completed; no normal PASSIVE, lower, or unload command"}
                    else:
                        post_terminal_stop = publish_prepared_command(stop_publisher, "HARD_STOP")
                else:
                    if terminal_cause in {"ARRIVED", "DIAGNOSTIC_SIM_TIME_LIMIT"}:
                        context["lifecycle"]["normal_shutdown_not_attempted"] = {
                            "reason": context.get("lifecycle", {}).get("normal_closeout_gate", {}).get("reason",
                                "no healthy terminal-frame gate evidence")}
                    post_terminal_stop = publish_prepared_stop(stop_publisher)
                request_ns = int(post_terminal_shutdown["request"]["publish_start_monotonic_ns"]
                                 if post_terminal_stop.get("not_requested") else
                                 post_terminal_stop["publish_start_monotonic_ns"])
                if not post_terminal_stop.get("not_requested"):
                    stop_command = int(post_terminal_stop["command"])
                    lifecycle_events.append({"phase": "cleanup",
                        "event": "HARD_STOP_FALLBACK_REQUESTED" if stop_command == 9 else "PASSIVE_STOP_REQUESTED",
                        "monotonic_ns": request_ns, "command": stop_command,
                        "publisher_invoked_monotonic_ns": post_terminal_stop["supervisor_invoked_monotonic_ns"],
                        "publish_return_monotonic_ns": post_terminal_stop["publish_return_monotonic_ns"]})
                post_terminal_stop["confirmation_scope"] = (
                    "no controller stop command was sent; FIXEDSTAND remained active until MuJoCo shutdown"
                    if post_terminal_stop.get("not_requested") else
                    "controller hard-stop branch logged completion after PASSIVE enter; "
                    "not a MuJoCo command-consumption acknowledgement")
                observed_keys: set[tuple[int, int]] = set()
                cleanup_start_ns = now_ns()
                confirmation = None
                deadline = time.monotonic() + (0.0 if post_terminal_stop.get("not_requested") else 2.0)
                while not post_terminal_stop.get("not_requested") and time.monotonic() < deadline:
                    ros_log.flush()
                    log_text = log_path.read_text(encoding="utf-8", errors="replace")
                    confirmation = hard_stop_confirmation(log_text, request_ns)
                    if confirmation is not None:
                        break
                    sampled_ns = now_ns()
                    raw_cleanup = read_shm_frame()
                    cleanup_status, cleanup_frame = classify_frame(raw_cleanup, sampled_ns)
                    if cleanup_status is FrameStatus.LIVE and cleanup_frame is not None:
                        key = (cleanup_frame.session_id, cleanup_frame.sequence)
                        if key not in observed_keys:
                            observed_keys.add(key)
                            cleanup_frame_observations.append({
                                "session_id": cleanup_frame.session_id,
                                "sequence": cleanup_frame.sequence,
                                "rl_step": cleanup_frame.rl_step,
                                "frame_monotonic_ns": cleanup_frame.monotonic_ns,
                                "observer_monotonic_ns": sampled_ns,
                                "policy_state": {0: "AGILE", 1: "RECOVERY", 2: "FAULTED"}.get(
                                    cleanup_frame.policy_state, "UNKNOWN"),
                                "sim_time_s": cleanup_frame.sim_time_s if cleanup_frame.sim_clock_valid else None,
                            })
                    time.sleep(0.005)
                cleanup_end_ns = now_ns()
                if post_terminal_stop.get("not_requested"):
                    post_terminal_stop.update({
                        "controller_stop_requested": False,
                        "controller_state_before_simulator_stop": "FIXEDSTAND",
                        "simulator_shutdown_started_monotonic_ns": cleanup_start_ns,
                        "no_passive_confirmation_claimed": True,
                    })
                    lifecycle_events.append({"phase": "cleanup", "event": "STANDING_HOLD_OBSERVATION_COMPLETE",
                        "monotonic_ns": cleanup_start_ns, "controller_state": "FIXEDSTAND",
                        "controller_stop_requested": False})
                else:
                    stop_confirmed = confirmation is not None
                    post_terminal_stop.update({
                        "controller_transition_confirmed": stop_confirmed,
                        "controller_confirmation": confirmation,
                        "controller_pid": (context.get("preflight", {}).get("launched_manager_process_evidence") or [{}])[0].get("pid"),
                        "controller_transition_delay_from_publish_start_ns": (
                            confirmation["controller_steady_monotonic_ns"] - request_ns
                            if confirmation and "controller_steady_monotonic_ns" in confirmation else None),
                        "confirmation_observation_window_monotonic_ns": [cleanup_start_ns, cleanup_end_ns],
                        "confirmation_not_observed_by_deadline": not stop_confirmed,
                    })
                    lifecycle_events.append({"phase": "cleanup", "event": "STRATEGY_STOP_CONFIRMATION",
                        "monotonic_ns": confirmation.get("controller_steady_monotonic_ns", cleanup_end_ns) if confirmation else cleanup_end_ns,
                        "confirmed": stop_confirmed, "evidence": confirmation,
                        "observation_window_monotonic_ns": [cleanup_start_ns, cleanup_end_ns]})
                context["lifecycle"]["cleanup_observation"] = {
                    "not_part_of_motion_capture_or_metrics": True,
                    "start_monotonic_ns": cleanup_start_ns,
                    "end_monotonic_ns": cleanup_end_ns,
                    "strategy_frames": cleanup_frame_observations,
                    "strategy_output_after_request_observed": bool(cleanup_frame_observations),
                    "interpretation": "only frames observed in this bounded cleanup window; absence is not proof of zero output",
                }
                post_terminal_stop["cleanup_policy_frame_count_observed"] = len(cleanup_frame_observations)
                if not post_terminal_stop.get("not_requested"):
                    post_terminal_stop["stop_request_to_confirmation"] = (
                        "measured from publisher publish_start to controller post-transition steady_ns"
                        if post_terminal_stop.get("controller_transition_confirmed") else
                        "UNKNOWN within recorded bounded observation window")
            except Exception as exc:
                post_terminal_stop = {"command": 1, "failed": f"{type(exc).__name__}: {exc}",
                                      "monotonic_ns": now_ns(),
                                      "confirmation": "UNKNOWN; request or acknowledgement failed"}
                context["lifecycle"]["normal_shutdown_error"] = post_terminal_stop["failed"]
        context["lifecycle"]["normal_shutdown"] = post_terminal_shutdown
        context["lifecycle"]["post_terminal_strategy_stop"] = post_terminal_stop
        def cleanup_owned_processes() -> None:
            # On a healthy completed standing hold, stop MuJoCo first while the
            # controller still commands FIXEDSTAND; then close ROS/controller.
            stop_results = []
            standing_complete = (context.get("lifecycle", {}).get("standing_observation", {})
                                 .get("complete") is True)
            owned_processes = (("mujoco", mujoco), ("ros2_launch", ros),
                               ("prepared_stop_publisher", stop_publisher)) if standing_complete else (
                               ("ros2_launch", ros), ("mujoco", mujoco),
                               ("prepared_stop_publisher", stop_publisher))
            for name, proc in owned_processes:
                if proc is None:
                    stop_results.append({"name": name, "started": False, "exit_code": None, "forced": False})
                    continue
                forced = False
                try:
                    if proc.poll() is None:
                        try:
                            os.killpg(proc.pid, signal.SIGINT)
                            proc.wait(timeout=6.0)
                        except subprocess.TimeoutExpired:
                            forced = True
                            os.killpg(proc.pid, signal.SIGTERM)
                            try:
                                proc.wait(timeout=3.0)
                            except subprocess.TimeoutExpired:
                                os.killpg(proc.pid, signal.SIGKILL)
                                proc.wait(timeout=3.0)
                        except ProcessLookupError:
                            pass
                    stop_results.append({"name": name, "started": True, "exit_code": proc.poll(), "forced": forced})
                    if standing_complete and name == "mujoco" and proc.poll() is not None:
                        lifecycle_events.append({"phase": "process_cleanup", "event": "MUJOCO_STOPPED_BEFORE_CONTROLLER",
                                                 "monotonic_ns": now_ns(), "exit_code": proc.returncode,
                                                 "controller_state_before_stop": "FIXEDSTAND"})
                except Exception as exc:
                    stop_results.append({"name": name, "started": True, "exit_code": proc.poll(),
                                         "forced": forced, "cleanup_error": f"{type(exc).__name__}: {exc}"})
            try:
                ros_log.flush()
                final_ros_text = log_path.read_text(encoding="utf-8", errors="replace")
                known_safety_messages = {item.get("message") for item in process_facts["external_safety_events"]}
                known_safety_messages.update(item.get("message") for item in lifecycle_events
                                             if item.get("phase") == "cleanup" and item.get("source"))
                for line in final_ros_text.splitlines():
                    if not (EMERGENCY_RE.search(line) or HARDSTOP_RE.search(line) or VETO_RE.search(line)):
                        continue
                    message = line.strip()[:1000]
                    if message in known_safety_messages:
                        continue
                    kind = "FROZEN_POSITION_PASSIVE" if EMERGENCY_RE.search(line) else ("GLOBAL_HARD_STOP" if HARDSTOP_RE.search(line) else "POLICY_SAFETY_VETO")
                    event_match = re.search(r"detection_ns=(\d+)", line)
                    hard_stop_interval = None
                    if HARDSTOP_RE.search(line) and isinstance(post_terminal_stop, dict):
                        confirm_event = post_terminal_stop.get("controller_confirmation")
                        if isinstance(confirm_event, dict):
                            hard_stop_interval = [post_terminal_stop.get("publish_start_monotonic_ns"),
                                                  confirm_event.get("controller_steady_monotonic_ns")]
                    lifecycle_events.append({"phase": "cleanup", "event": kind,
                        "source": "RlQuadrupedController/StateRL ROS log", "message": message,
                        "event_monotonic_ns": int(event_match.group(1)) if event_match else None,
                        "event_time_source": "controller_detection_ns" if event_match else "not embedded in source line",
                        "event_time_interval_monotonic_ns": hard_stop_interval,
                        "observer_received_monotonic_ns": now_ns()})
            except Exception as exc:
                process_facts.setdefault("cleanup_errors", []).append(
                    {"stage": "final_ros_safety_scan", "error": f"{type(exc).__name__}: {exc}"})
            for name, log in (("ros_launch_log", ros_log), ("mujoco_log", mujoco_log)):
                try:
                    log.flush()
                    log.close()
                except Exception as exc:
                    process_facts.setdefault("cleanup_errors", []).append(
                        {"stage": name, "error": f"{type(exc).__name__}: {exc}"})
            cleanup_ns = now_ns()
            context["lifecycle"]["cleanup_complete_monotonic_ns"] = cleanup_ns
            lifecycle_events.append({"phase": "process_cleanup", "event": "process_groups_stopped",
                                     "monotonic_ns": cleanup_ns, "processes": stop_results})
            context["lifecycle"]["cleanup_processes"] = stop_results
            context["capture"] = {"frames_written": recorder.unique_frames,
                                   "capture_error": recorder.error,
                                   "frame_capture_writer": "supervisor_synchronous_single_writer",
                                   "authoritative_collision_live_before_motion": collision_live_seen,
                                   "pre_run_shared_frames_ignored_by_session_timestamp": recorder.pre_run_frames_ignored}
            process_facts.update({"exit_code": ros.returncode if ros is not None else None,
                                  "forced_termination": any(item["forced"] for item in stop_results),
                                  "shutdown_complete": all(proc is None or proc.poll() is not None
                                                           for proc in (ros, mujoco, stop_publisher)),
                                  "shutdown_request_source": "S1-04 bounded supervisor SIGINT with TERM/KILL fallback",
                                  "run_terminal_result": terminal_cause,
                                  "run_terminal_monotonic_ns": finish_event_ns,
                                  "cleanup_events": [e for e in lifecycle_events if e.get("phase") == "cleanup"]})
            context["result_contract"] = {"classification": terminal_cause,
                "startup_failure": startup_error,
                "fall_detection": "PhysicsLoop base_link posture rule; continuously >=0.30 valid sim seconds",
                "operator_abort": "not issued", "policy_frames_observed": recorder.unique_frames,
                "supervisor_loop_frames_observed": frame_rows,
                "first_valid_sim_time_s": sim_start_s, "first_valid_frame_monotonic_ns": first_frame_ns,
                "motion_start_sim_time_s": movement_start_sim_s,
                "motion_start_unknown_reason": None if movement_start_sim_s is not None else "no >=0.10m pose displacement observed",
                "valid_collision_source_seen": collision_live_seen,
                "movement_terminal_summary": terminal_summary,
                "cleanup_observation_is_excluded_from_motion_metrics": True,
                "command_consumption_by_mujoco": "UNKNOWN; no direct consumption acknowledgement"}
            context["commands"] = controls

        terminal, closeout_errors = finalize_run_closeout(
            out, context, process_facts, recorder, capture_started, cleanup_owned_processes)

    if process_facts.get("shutdown_complete") is False:
        cleanup_summary = "未确认全部本次进程退出"
    else:
        cleanup_summary = "本次 ROS、MuJoCo 与停止发布器进程均已退出"
    summary = {
        "end_reason": terminal_cause,
        "distance_to_goal_m": terminal_summary.get("distance_to_goal_m") if terminal_summary else None,
        "contact_objects": terminal_summary.get("terminal_contact_objects") if terminal_summary else [],
        "standing_hold_complete": bool(
            (context.get("lifecycle", {}).get("normal_shutdown") or {}).get("standing_hold_complete")),
        "normal_passive_requested": bool(
            (context.get("lifecycle", {}).get("normal_shutdown") or {}).get("passive_requested")),
        "passive_controller_transition_confirmed": bool(
            context.get("lifecycle", {}).get("post_terminal_strategy_stop", {}).get("controller_transition_confirmed")),
        "cleanup": cleanup_summary,
    }
    print(json.dumps({"run_id": run_id, "directory": str(out), "result": terminal_cause,
                      "summary": summary, "startup_error": startup_error, "frames": recorder.unique_frames,
                      "process_facts": process_facts}, ensure_ascii=False))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--overlay", required=True)
    parser.add_argument("--goal-x", type=float, default=1.0)
    parser.add_argument("--scene", default="scene_obstacle.xml",
                        help="supported MuJoCo scene XML filename (default: scene_obstacle.xml)")
    parser.add_argument("--sim-limit-s", type=float, default=6.0)
    parser.add_argument("--wall-cap-s", type=float, default=60.0)
    parser.add_argument("--standing-observation-sim-s", type=float, default=5.0,
                        help="post-stand observation in simulation seconds (0..30)")
    parser.add_argument("--standing-observation-wall-cap-s", type=float, default=30.0,
                        help="independent wall-clock cap for post-stand observation (1..60)")
    parser.add_argument("--classification", default="bounded nonformal single-run diagnostic; not a formal experiment",
                        help="human-readable nonformal run purpose saved in run_context.json")
    parser.add_argument("--config-only", action="store_true",
                        help="stop after controller initialization proves A config consumption; never request RL")
    parser.add_argument("--display", default=":0")
    parser.add_argument("--xauthority", default="/run/user/1000/gdm/Xauthority")
    args = parser.parse_args()
    if (not math.isfinite(args.standing_observation_sim_s) or
            not 0.0 <= args.standing_observation_sim_s <= 30.0):
        parser.error("--standing-observation-sim-s must be finite and in [0, 30]")
    if (not math.isfinite(args.standing_observation_wall_cap_s) or
            not 1.0 <= args.standing_observation_wall_cap_s <= 60.0):
        parser.error("--standing-observation-wall-cap-s must be finite and in [1, 60]")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
