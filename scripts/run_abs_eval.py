#!/usr/bin/env python3
"""Run automated ABS MuJoCo evaluation episodes.

This script is intentionally simulation-focused. It launches MuJoCo + the ROS2
controller, enters RL mode through /control_input, samples simulator shared
memory for success/fall/proximity metrics, and parses StateRL [EVAL] logs into
per-run summary.json + telemetry.csv files.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import shlex
import signal
import struct
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

ROOT = Path.home() / "quadruped_robots"
MUJOCO_DIR = ROOT / "unitree_mujoco"
MUJOCO_BIN = MUJOCO_DIR / "simulate" / "build2" / "unitree_mujoco"
ROS2_WS = ROOT / "quadruped_ros2_control_humble"
DEFAULT_OUTPUT_ROOT = ROOT / "logs" / "abs_eval"
UNITREE_SDK2_LIB = Path.home() / "Libraries" / "unitree_sdk2" / "lib"
LIBTORCH_LIB = Path.home() / "Libraries" / "libtorch-cpu-2.0.1" / "lib"

QPOS_PATH = Path("/dev/shm/mujoco_qpos")
RAY2D_PATH = Path("/dev/shm/mujoco_ray2d")
QPOS_FORMAT = "19d"
RAY2D_FORMAT = "11f"
QPOS_SIZE = struct.calcsize(QPOS_FORMAT)
RAY2D_SIZE = struct.calcsize(RAY2D_FORMAT)

FLOAT = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?|nan|inf|-inf"
EVAL_RE = re.compile(
    rf"\[EVAL\] step=(?P<step>\d+) t=(?P<t>{FLOAT}) "
    rf"robot=\((?P<robot_x>{FLOAT}),(?P<robot_y>{FLOAT})\) yaw=(?P<yaw>{FLOAT}) "
    rf"goal=\((?P<goal_x>{FLOAT}),(?P<goal_y>{FLOAT})\) dist=(?P<dist>{FLOAT}) "
    rf"body=\((?P<body_x>{FLOAT}),(?P<body_y>{FLOAT})\) heading=(?P<heading>{FLOAT}) "
    rf"arrived=(?P<arrived>\d+) ra=(?P<ra>{FLOAT}) entry=(?P<entry>{FLOAT}) "
    rf"recovery=(?P<recovery>\d+) hold=(?P<hold>-?\d+) "
    rf"twist=\((?P<twist_vx>{FLOAT}),(?P<twist_vy>{FLOAT}),(?P<twist_wz>{FLOAT})\) "
    rf"min_ray_log=(?P<min_ray_log>{FLOAT}) max_ray_log=(?P<max_ray_log>{FLOAT}) "
    rf"min_ray_m=(?P<min_ray_m>{FLOAT}) "
    rf"lin_vel=\((?P<lin_vel_x>{FLOAT}),(?P<lin_vel_y>{FLOAT}),(?P<lin_vel_z>{FLOAT})\) "
    rf"ang_vel=\((?P<ang_vel_x>{FLOAT}),(?P<ang_vel_y>{FLOAT}),(?P<ang_vel_z>{FLOAT})\) "
    rf"action_range=\((?P<action_min>{FLOAT}),(?P<action_max>{FLOAT})\) "
    rf"contact=\((?P<contact_fr>{FLOAT}),(?P<contact_fl>{FLOAT}),(?P<contact_rr>{FLOAT}),(?P<contact_rl>{FLOAT})\)"
)

TELEMETRY_FIELDS = [
    "step", "t", "robot_x", "robot_y", "yaw", "goal_x", "goal_y", "dist",
    "body_x", "body_y", "heading", "arrived", "ra", "entry", "recovery",
    "hold", "twist_vx", "twist_vy", "twist_wz", "min_ray_log", "max_ray_log",
    "min_ray_m", "lin_vel_x", "lin_vel_y", "lin_vel_z", "ang_vel_x",
    "ang_vel_y", "ang_vel_z", "action_min", "action_max", "contact_fr",
    "contact_fl", "contact_rr", "contact_rl",
]


@dataclass
class SceneClearance:
    status: str = "OK"
    min_spawn_clearance_m: Optional[float] = None
    spawn_violation: bool = False
    corridor_warning: bool = False
    notes: List[str] = None

    def __post_init__(self) -> None:
        if self.notes is None:
            self.notes = []


@dataclass
class MonitorResult:
    success: bool = False
    time_to_goal_s: Optional[float] = None
    fall: bool = False
    fall_reason: str = ""
    collision_proxy: bool = False
    collision_proxy_reason: str = ""
    min_goal_error_m: Optional[float] = None
    final_goal_error_m: Optional[float] = None
    min_ray_distance_m: Optional[float] = None
    samples: int = 0
    qpos_available: bool = False
    ray2d_available: bool = False


def parse_float(value: str) -> float:
    return float(value)


def read_struct(path: Path, fmt: str, size: int) -> Optional[Tuple[float, ...]]:
    try:
        with path.open("rb") as f:
            data = f.read(size)
        if len(data) != size:
            return None
        return struct.unpack(fmt, data)
    except FileNotFoundError:
        return None
    except OSError:
        return None


def quat_wxyz_to_rpy(w: float, x: float, y: float, z: float) -> Tuple[float, float, float]:
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def shell_source_prefix() -> str:
    return (
        "source /opt/ros/humble/setup.bash && "
        f"source {shlex.quote(str(ROS2_WS / 'install' / 'setup.bash'))} && "
    )


def ros2_command(command: str) -> List[str]:
    return ["bash", "-lc", shell_source_prefix() + command]


def make_env() -> Dict[str, str]:
    env = os.environ.copy()
    existing = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = f"{UNITREE_SDK2_LIB}:{LIBTORCH_LIB}:{existing}"
    return env


def start_process(cmd: List[str], cwd: Path, log_file, env: Dict[str, str]) -> subprocess.Popen:
    return subprocess.Popen(
        cmd,
        cwd=str(cwd),
        stdout=log_file,
        stderr=subprocess.STDOUT,
        env=env,
        preexec_fn=os.setsid,
    )


def stop_process(proc: Optional[subprocess.Popen], name: str, timeout_s: float = 5.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        print(f"[cleanup] {name} did not stop after SIGTERM, sending SIGKILL")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait(timeout=timeout_s)
    except ProcessLookupError:
        pass


def run_ros2_once(command: str, timeout_s: float = 10.0, quiet: bool = True) -> subprocess.CompletedProcess:
    stdout = subprocess.DEVNULL if quiet else subprocess.PIPE
    stderr = subprocess.DEVNULL if quiet else subprocess.PIPE
    return subprocess.run(
        ros2_command(command),
        cwd=str(ROS2_WS),
        env=make_env(),
        stdout=stdout,
        stderr=stderr,
        timeout=timeout_s,
        text=True,
        check=False,
    )


def publish_control(command: int, lx: float, ly: float, rx: float, ry: float) -> None:
    msg = f"{{command: {command}, lx: {lx}, ly: {ly}, rx: {rx}, ry: {ry}}}"
    run_ros2_once(
        f"ros2 topic pub --once /control_input control_input_msgs/msg/Inputs {shlex.quote(msg)}",
        timeout_s=10.0,
        quiet=True,
    )


def wait_for_controller(timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        result = run_ros2_once("ros2 control list_controllers", timeout_s=5.0, quiet=False)
        output = (result.stdout or "") + (result.stderr or "")
        if "rl_quadruped_controller" in output and "active" in output:
            return True
        time.sleep(1.0)
    return False


def auto_enter_rl(args: argparse.Namespace) -> None:
    publish_control(2, 0.0, 0.0, 0.0, 0.0)
    time.sleep(0.5)
    publish_control(2, 0.0, 0.0, 0.0, 0.0)
    time.sleep(args.fixedstand_wait_s)
    publish_control(3, args.command_lx, args.command_ly, args.command_rx, args.command_ry)
    time.sleep(0.5)
    publish_control(3, args.command_lx, args.command_ly, args.command_rx, args.command_ry)
    time.sleep(args.rl_settle_s)


def monitor_episode(args: argparse.Namespace, eval_goal: Tuple[float, float]) -> MonitorResult:
    result = MonitorResult()
    start = time.monotonic()
    deadline = start + args.duration
    near_obstacle_since: Optional[float] = None

    while time.monotonic() < deadline:
        now = time.monotonic()
        qpos = read_struct(QPOS_PATH, QPOS_FORMAT, QPOS_SIZE)
        ray2d = read_struct(RAY2D_PATH, RAY2D_FORMAT, RAY2D_SIZE)
        result.samples += 1

        if qpos is not None:
            result.qpos_available = True
            x, y, z = qpos[0], qpos[1], qpos[2]
            goal_error = math.hypot(eval_goal[0] - x, eval_goal[1] - y)
            result.final_goal_error_m = goal_error
            if result.min_goal_error_m is None or goal_error < result.min_goal_error_m:
                result.min_goal_error_m = goal_error

            if goal_error <= args.arrival_threshold:
                result.success = True
                result.time_to_goal_s = now - start
                if not args.continue_after_success:
                    break

            qw, qx, qy, qz = qpos[3], qpos[4], qpos[5], qpos[6]
            roll, pitch, _ = quat_wxyz_to_rpy(qw, qx, qy, qz)
            if z < args.fall_height:
                result.fall = True
                result.fall_reason = f"base_height {z:.3f} < {args.fall_height:.3f}"
            elif abs(roll) > args.fall_angle or abs(pitch) > args.fall_angle:
                result.fall = True
                result.fall_reason = (
                    f"abs(roll/pitch)=({abs(roll):.3f},{abs(pitch):.3f}) "
                    f"> {args.fall_angle:.3f}"
                )

            if result.fall and args.stop_on_failure:
                break

        if ray2d is not None:
            result.ray2d_available = True
            distances = [2.0 ** v for v in ray2d if math.isfinite(v)]
            if distances:
                min_ray = min(distances)
                if result.min_ray_distance_m is None or min_ray < result.min_ray_distance_m:
                    result.min_ray_distance_m = min_ray

                if min_ray <= args.collision_proxy_distance:
                    if near_obstacle_since is None:
                        near_obstacle_since = now
                    elif now - near_obstacle_since >= args.collision_proxy_hold_s:
                        result.collision_proxy = True
                        result.collision_proxy_reason = (
                            f"min_ray {min_ray:.3f} <= {args.collision_proxy_distance:.3f} "
                            f"for {args.collision_proxy_hold_s:.2f}s"
                        )
                        if args.stop_on_failure:
                            break
                else:
                    near_obstacle_since = None

        time.sleep(args.sample_period)

    return result


def _parse_vec(text: str, expected: int) -> Optional[List[float]]:
    try:
        values = [float(v) for v in text.split()]
    except ValueError:
        return None
    if len(values) < expected:
        return None
    return values


def _footprint_clearance(geom_type: str, pos: List[float], size: List[float]) -> Optional[float]:
    x, y = pos[0], pos[1]
    if geom_type == "box":
        sx, sy = size[0], size[1]
        dx = max(abs(x) - sx, 0.0)
        dy = max(abs(y) - sy, 0.0)
        return math.hypot(dx, dy)
    if geom_type in {"cylinder", "sphere", "capsule", "ellipsoid"}:
        radius = size[0]
        return max(0.0, math.hypot(x, y) - radius)
    return None


def _intersects_initial_corridor(
    geom_type: str,
    pos: List[float],
    size: List[float],
    corridor_x_m: float,
    corridor_half_width_m: float,
) -> bool:
    x, y = pos[0], pos[1]
    if geom_type == "box":
        front = x - size[0]
        rear = x + size[0]
        lateral_clearance = abs(y) - size[1]
    elif geom_type in {"cylinder", "sphere", "capsule", "ellipsoid"}:
        radius = size[0]
        front = x - radius
        rear = x + radius
        lateral_clearance = abs(y) - radius
    else:
        return False

    overlaps_x = rear >= 0.0 and front <= corridor_x_m
    overlaps_y = lateral_clearance <= corridor_half_width_m
    return overlaps_x and overlaps_y


def analyze_scene_clearance(scene: str, args: argparse.Namespace) -> SceneClearance:
    scene_path = MUJOCO_DIR / "unitree_robots" / "go2" / scene
    result = SceneClearance()
    if not scene_path.exists():
        result.status = "UNKNOWN"
        result.notes.append(f"scene file not found: {scene_path}")
        return result

    try:
        root = ET.parse(scene_path).getroot()
    except ET.ParseError as exc:
        result.status = "UNKNOWN"
        result.notes.append(f"XML parse error: {exc}")
        return result

    obstacle_types = {"box", "cylinder", "sphere", "capsule", "ellipsoid"}
    for geom in root.findall(".//geom"):
        geom_type = geom.attrib.get("type", "")
        if geom_type not in obstacle_types:
            continue
        pos = _parse_vec(geom.attrib.get("pos", ""), 3)
        size = _parse_vec(geom.attrib.get("size", ""), 1)
        if pos is None or size is None:
            continue
        if geom_type == "box" and len(size) < 2:
            continue

        clearance = _footprint_clearance(geom_type, pos, size)
        if clearance is not None:
            if result.min_spawn_clearance_m is None or clearance < result.min_spawn_clearance_m:
                result.min_spawn_clearance_m = clearance
            if clearance < args.spawn_clearance_radius:
                result.spawn_violation = True
                result.notes.append(
                    f"spawn clearance {clearance:.3f}m < {args.spawn_clearance_radius:.3f}m "
                    f"for {geom_type} pos={geom.attrib.get('pos')} size={geom.attrib.get('size')}"
                )

        if _intersects_initial_corridor(
            geom_type, pos, size, args.corridor_clear_x, args.corridor_half_width
        ):
            result.corridor_warning = True
            result.notes.append(
                f"initial corridor obstacle within x<= {args.corridor_clear_x:.2f}m, "
                f"|y|<= {args.corridor_half_width:.2f}m: {geom_type} "
                f"pos={geom.attrib.get('pos')} size={geom.attrib.get('size')}"
            )

    if result.spawn_violation:
        result.status = "FAIL_SPAWN_CLEARANCE"
    elif result.corridor_warning:
        result.status = "WARN_CORRIDOR"
    else:
        result.status = "OK"
    return result


def parse_eval_log(log_path: Path, telemetry_csv: Path) -> Dict[str, object]:
    rows: List[Dict[str, object]] = []
    counters = {
        "recovery_entries": 0,
        "recovery_exits": 0,
        "goal_arrived_logs": 0,
        "goal_resamples": 0,
        "twist_gd_count": 0,
        "dds_timeout_count": 0,
        "rec_nan_count": 0,
    }

    if not log_path.exists():
        return {"rows": rows, **counters}

    with log_path.open("r", errors="replace") as f:
        for line in f:
            if "ENTER recovery" in line:
                counters["recovery_entries"] += 1
            if "EXIT recovery" in line:
                counters["recovery_exits"] += 1
            if "[ARRIVED]" in line:
                counters["goal_arrived_logs"] += 1
            if "GOAL-RESAMPLE" in line:
                counters["goal_resamples"] += 1
            if "[TWIST-GD]" in line:
                counters["twist_gd_count"] += 1
            if "DDS timeout" in line or "[EMERGENCY]" in line:
                counters["dds_timeout_count"] += 1
            if "[REC-NAN]" in line:
                counters["rec_nan_count"] += 1

            match = EVAL_RE.search(line)
            if not match:
                continue
            row: Dict[str, object] = {}
            for key in TELEMETRY_FIELDS:
                raw = match.group(key)
                if key in {"step", "arrived", "recovery", "hold"}:
                    row[key] = int(raw)
                else:
                    row[key] = parse_float(raw)
            rows.append(row)

    telemetry_csv.parent.mkdir(parents=True, exist_ok=True)
    with telemetry_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=TELEMETRY_FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    return {"rows": rows, **counters}


def summarize_from_rows(rows: List[Dict[str, object]]) -> Dict[str, Optional[float]]:
    if not rows:
        return {
            "eval_samples": 0,
            "ra_min": None,
            "ra_max": None,
            "ra_mean": None,
            "recovery_ratio": None,
            "first_arrival_t_log_s": None,
            "min_dist_log_m": None,
            "final_dist_log_m": None,
            "min_ray_log_m": None,
        }

    ra_values = [float(r["ra"]) for r in rows if math.isfinite(float(r["ra"]))]
    recovery_values = [int(r["recovery"]) for r in rows]
    arrived_rows = [r for r in rows if int(r["arrived"]) == 1]
    dists = [float(r["dist"]) for r in rows]
    rays = [float(r["min_ray_m"]) for r in rows if math.isfinite(float(r["min_ray_m"]))]

    return {
        "eval_samples": len(rows),
        "ra_min": min(ra_values) if ra_values else None,
        "ra_max": max(ra_values) if ra_values else None,
        "ra_mean": sum(ra_values) / len(ra_values) if ra_values else None,
        "recovery_ratio": sum(recovery_values) / len(recovery_values) if recovery_values else None,
        "first_arrival_t_log_s": float(arrived_rows[0]["t"]) if arrived_rows else None,
        "min_dist_log_m": min(dists) if dists else None,
        "final_dist_log_m": dists[-1] if dists else None,
        "min_ray_log_m": min(rays) if rays else None,
    }


def is_stuck_recovery_loop(summary: Dict[str, object]) -> bool:
    if summary.get("success") is True:
        return False
    if summary.get("fall") is True or summary.get("collision_proxy") is True:
        return False
    if int(summary.get("dds_timeout_count") or 0) > 0:
        return False

    recovery_ratio = summary.get("recovery_ratio")
    recovery_entries = int(summary.get("recovery_entries") or 0)
    min_goal_error = summary.get("min_goal_error_m")
    final_goal_error = summary.get("final_goal_error_m")
    if recovery_ratio is None or min_goal_error is None or final_goal_error is None:
        return False

    return (
        float(recovery_ratio) >= 0.45
        and recovery_entries >= 20
        and float(min_goal_error) >= 3.0
        and float(final_goal_error) >= 3.0
    )


def determine_result(monitor: MonitorResult, parsed: Dict[str, object], process_failed: bool) -> str:
    if process_failed:
        return "STARTUP_FAILED"
    if monitor.success:
        return "SUCCESS"
    if int(parsed.get("dds_timeout_count", 0)) > 0:
        return "DDS_TIMEOUT"
    if monitor.fall:
        return "FALL"
    if monitor.collision_proxy:
        return "COLLISION_PROXY"
    return "TIMEOUT"


def run_episode(scene: str, run_index: int, args: argparse.Namespace, session_dir: Path) -> Dict[str, object]:
    scene_name = Path(scene).stem
    run_dir = session_dir / scene_name / f"run_{run_index:03d}"
    run_dir.mkdir(parents=True, exist_ok=True)
    log_path = run_dir / "runtime.log"
    telemetry_csv = run_dir / "telemetry.csv"
    summary_path = run_dir / "summary.json"

    env = make_env()
    mujoco_proc: Optional[subprocess.Popen] = None
    ros_proc: Optional[subprocess.Popen] = None
    process_failed = False
    monitor = MonitorResult()

    eval_goal = (
        args.goal_x + max(-1.0, min(1.0, args.command_ly)) * args.goal_trim_scale,
        args.goal_y + max(-1.0, min(1.0, -args.command_lx)) * args.goal_trim_scale,
    )

    print(f"\n=== {scene} run {run_index} ===")
    print(f"run_dir: {run_dir}")
    print(f"eval_goal: ({eval_goal[0]:.2f}, {eval_goal[1]:.2f})")

    scene_clearance = analyze_scene_clearance(scene, args)
    print(
        f"scene_clearance: {scene_clearance.status} "
        f"min_spawn_clearance={scene_clearance.min_spawn_clearance_m}"
    )
    if scene_clearance.status != "OK":
        for note in scene_clearance.notes[:3]:
            print(f"  - {note}")

    if args.skip_invalid_spawn and scene_clearance.spawn_violation:
        log_path.write_text("Skipped: invalid scene spawn clearance.\n" + "\n".join(scene_clearance.notes) + "\n")
        with telemetry_csv.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=TELEMETRY_FIELDS)
            writer.writeheader()
        summary: Dict[str, object] = {
            "scene": scene,
            "run_index": run_index,
            "result": "INVALID_SCENE_SPAWN",
            "success": False,
            "skipped": True,
            "skip_reason": "spawn clearance violation",
            "time_to_goal_s": None,
            "duration_limit_s": args.duration,
            "goal": {"x": eval_goal[0], "y": eval_goal[1]},
            "configured_goal": {"x": args.goal_x, "y": args.goal_y},
            "scene_clearance_status": scene_clearance.status,
            "scene_min_spawn_clearance_m": scene_clearance.min_spawn_clearance_m,
            "scene_spawn_violation": scene_clearance.spawn_violation,
            "scene_corridor_warning": scene_clearance.corridor_warning,
            "scene_clearance_notes": scene_clearance.notes,
            "runtime_log": str(log_path),
            "telemetry_csv": str(telemetry_csv),
            "summary_json": str(summary_path),
            "recovery_entries": 0,
            "recovery_exits": 0,
            "goal_arrived_logs": 0,
            "goal_resamples": 0,
            "twist_gd_count": 0,
            "dds_timeout_count": 0,
            "rec_nan_count": 0,
            "eval_samples": 0,
        }
        with summary_path.open("w") as f:
            json.dump(summary, f, indent=2, ensure_ascii=False)
        print(f"[result] INVALID_SCENE_SPAWN summary={summary_path}")
        return summary

    with log_path.open("wb") as log_file:
        try:
            mujoco_cmd = [str(MUJOCO_BIN), "-s", scene]
            print("[1/4] starting MuJoCo")
            mujoco_proc = start_process(mujoco_cmd, MUJOCO_DIR, log_file, env)
            time.sleep(args.mujoco_startup_s)
            if mujoco_proc.poll() is not None:
                print("[error] MuJoCo exited during startup")
                process_failed = True
                raise RuntimeError("MuJoCo startup failed")

            print("[2/4] starting ROS2 controller")
            ros_proc = start_process(
                ros2_command("ros2 launch rl_quadruped_controller mujoco.launch.py"),
                ROS2_WS,
                log_file,
                env,
            )
            time.sleep(args.ros_startup_s)
            if ros_proc.poll() is not None:
                print("[error] ROS2 launch exited during startup")
                process_failed = True
                raise RuntimeError("ROS2 startup failed")

            if args.wait_controller:
                print("[3/4] waiting for controller active")
                ready = wait_for_controller(args.controller_timeout_s)
                print(f"controller_ready={ready}")

            if args.auto_rl:
                print("[4/4] entering RL")
                auto_enter_rl(args)
            else:
                print("[4/4] auto RL disabled")

            print(f"[monitor] running up to {args.duration:.1f}s")
            monitor = monitor_episode(args, eval_goal)
            print(
                f"[monitor] success={monitor.success} fall={monitor.fall} "
                f"collision_proxy={monitor.collision_proxy} "
                f"final_goal_error={monitor.final_goal_error_m}"
            )

        except KeyboardInterrupt:
            raise
        except Exception as exc:
            print(f"[error] {exc}")
        finally:
            stop_process(ros_proc, "ros2")
            stop_process(mujoco_proc, "mujoco")
            log_file.flush()

    parsed = parse_eval_log(log_path, telemetry_csv)
    row_summary = summarize_from_rows(parsed["rows"])

    # If shared-memory success was unavailable, fall back to StateRL [EVAL] arrival.
    if not monitor.success and row_summary["first_arrival_t_log_s"] is not None:
        monitor.success = True
        monitor.time_to_goal_s = row_summary["first_arrival_t_log_s"]

    if monitor.min_ray_distance_m is None:
        monitor.min_ray_distance_m = row_summary["min_ray_log_m"]
    if monitor.min_goal_error_m is None:
        monitor.min_goal_error_m = row_summary["min_dist_log_m"]
    if monitor.final_goal_error_m is None:
        monitor.final_goal_error_m = row_summary["final_dist_log_m"]

    result = determine_result(monitor, parsed, process_failed)
    summary: Dict[str, object] = {
        "scene": scene,
        "run_index": run_index,
        "result": result,
        "success": monitor.success,
        "time_to_goal_s": monitor.time_to_goal_s,
        "duration_limit_s": args.duration,
        "goal": {"x": eval_goal[0], "y": eval_goal[1]},
        "configured_goal": {"x": args.goal_x, "y": args.goal_y},
        "command": {
            "lx": args.command_lx,
            "ly": args.command_ly,
            "rx": args.command_rx,
            "ry": args.command_ry,
        },
        "arrival_threshold_m": args.arrival_threshold,
        "min_goal_error_m": monitor.min_goal_error_m,
        "final_goal_error_m": monitor.final_goal_error_m,
        "fall": monitor.fall,
        "fall_reason": monitor.fall_reason,
        "fall_height_m": args.fall_height,
        "fall_angle_rad": args.fall_angle,
        "collision_proxy": monitor.collision_proxy,
        "collision_proxy_reason": monitor.collision_proxy_reason,
        "collision_proxy_distance_m": args.collision_proxy_distance,
        "scene_clearance_status": scene_clearance.status,
        "scene_min_spawn_clearance_m": scene_clearance.min_spawn_clearance_m,
        "scene_spawn_violation": scene_clearance.spawn_violation,
        "scene_corridor_warning": scene_clearance.corridor_warning,
        "scene_clearance_notes": scene_clearance.notes,
        "invalid_spawn_failure": (
            result == "FALL"
            and monitor.samples <= args.invalid_spawn_max_samples
            and scene_clearance.spawn_violation
        ),
        "min_ray_distance_m": monitor.min_ray_distance_m,
        "qpos_available": monitor.qpos_available,
        "ray2d_available": monitor.ray2d_available,
        "monitor_samples": monitor.samples,
        "runtime_log": str(log_path),
        "telemetry_csv": str(telemetry_csv),
        "summary_json": str(summary_path),
        "recovery_entries": parsed["recovery_entries"],
        "recovery_exits": parsed["recovery_exits"],
        "goal_arrived_logs": parsed["goal_arrived_logs"],
        "goal_resamples": parsed["goal_resamples"],
        "twist_gd_count": parsed["twist_gd_count"],
        "dds_timeout_count": parsed["dds_timeout_count"],
        "rec_nan_count": parsed["rec_nan_count"],
        **row_summary,
    }
    if result == "TIMEOUT" and is_stuck_recovery_loop(summary):
        result = "STUCK_RECOVERY_LOOP"
        summary["result"] = result
        summary["stuck_recovery_loop"] = True
    else:
        summary["stuck_recovery_loop"] = False

    with summary_path.open("w") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print(f"[result] {result} summary={summary_path}")
    return summary


def write_session_index(session_dir: Path, summaries: Iterable[Dict[str, object]]) -> None:
    summaries = list(summaries)
    with (session_dir / "session_summary.json").open("w") as f:
        json.dump(summaries, f, indent=2, ensure_ascii=False)

    csv_path = session_dir / "session_summary.csv"
    if not summaries:
        return
    fieldnames = [
        "scene", "run_index", "result", "success", "time_to_goal_s",
        "min_goal_error_m", "final_goal_error_m", "min_ray_distance_m",
        "recovery_entries", "recovery_exits", "recovery_ratio", "ra_max",
        "ra_mean", "dds_timeout_count", "fall", "collision_proxy",
        "stuck_recovery_loop", "scene_clearance_status", "scene_min_spawn_clearance_m", "invalid_spawn_failure", "runtime_log",
    ]
    with csv_path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(summaries)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenes",
        nargs="+",
        default=["scene.xml", "scene_terrain.xml", "scene_test1.xml", "scene_test2.xml", "scene_test3.xml", "scene_test4.xml", "scene_test5.xml"],
        help="MuJoCo scene XML names under unitree_robots/go2/",
    )
    parser.add_argument("--runs-per-scene", type=int, default=1)
    parser.add_argument("--duration", type=float, default=30.0, help="Max seconds after RL entry per episode")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--session-name", default=datetime.now().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--dry-run", action="store_true")

    parser.add_argument("--goal-x", type=float, default=7.0)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--goal-trim-scale", type=float, default=2.0, help="Matches StateRL joystick goal trim scale")
    parser.add_argument("--arrival-threshold", type=float, default=0.5)
    parser.add_argument("--command-lx", type=float, default=0.0)
    parser.add_argument("--command-ly", type=float, default=0.0, help="Joystick goal trim in x; default 0 keeps eval goal at YAML goal_x")
    parser.add_argument("--command-rx", type=float, default=0.0)
    parser.add_argument("--command-ry", type=float, default=0.0)

    parser.add_argument("--mujoco-startup-s", type=float, default=3.0)
    parser.add_argument("--ros-startup-s", type=float, default=8.0)
    parser.add_argument("--fixedstand-wait-s", type=float, default=4.0)
    parser.add_argument("--rl-settle-s", type=float, default=0.0, help="Optional wait after sending RL command; keep 0 for accurate time-to-goal")
    parser.add_argument("--wait-controller", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--controller-timeout-s", type=float, default=20.0)
    parser.add_argument("--auto-rl", action=argparse.BooleanOptionalAction, default=True)

    parser.add_argument("--sample-period", type=float, default=0.2)
    parser.add_argument("--continue-after-success", action="store_true")
    parser.add_argument("--stop-on-failure", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--fall-height", type=float, default=0.20)
    parser.add_argument("--fall-angle", type=float, default=0.80)
    parser.add_argument("--collision-proxy-distance", type=float, default=0.18)
    parser.add_argument("--collision-proxy-hold-s", type=float, default=0.20)
    parser.add_argument("--spawn-clearance-radius", type=float, default=0.75)
    parser.add_argument("--corridor-clear-x", type=float, default=1.5)
    parser.add_argument("--corridor-half-width", type=float, default=0.45)
    parser.add_argument("--skip-invalid-spawn", action="store_true", help="Skip scenes whose obstacle footprint violates spawn clearance")
    parser.add_argument("--invalid-spawn-max-samples", type=int, default=3, help="Mark very early falls in invalid-clearance scenes for analysis")
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    session_dir = args.output_root / args.session_name

    scene_dir = MUJOCO_DIR / "unitree_robots" / "go2"
    missing = [scene for scene in args.scenes if not (scene_dir / scene).exists()]
    if missing:
        print(f"[error] missing scene files under {scene_dir}: {missing}", file=sys.stderr)
        return 2
    if not MUJOCO_BIN.exists():
        print(f"[error] MuJoCo binary not found: {MUJOCO_BIN}", file=sys.stderr)
        return 2

    print("ABS automated simulation evaluation")
    print(f"session_dir: {session_dir}")
    print(f"scenes: {args.scenes}")
    print(f"runs_per_scene: {args.runs_per_scene}")
    if args.dry_run:
        print("dry run: no processes launched")
        return 0

    session_dir.mkdir(parents=True, exist_ok=True)
    summaries: List[Dict[str, object]] = []
    try:
        for scene in args.scenes:
            for run_index in range(1, args.runs_per_scene + 1):
                summary = run_episode(scene, run_index, args, session_dir)
                summaries.append(summary)
                write_session_index(session_dir, summaries)
                time.sleep(2.0)
    except KeyboardInterrupt:
        print("\nInterrupted by user. Writing partial session summary.")
        write_session_index(session_dir, summaries)
        return 130

    write_session_index(session_dir, summaries)
    print(f"\nAll episodes complete. Session summary: {session_dir / 'session_summary.csv'}")
    print("Next: python3 scripts/analyze_abs_eval.py --input", session_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
