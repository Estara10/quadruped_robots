#!/usr/bin/env python3
"""P1-09AE — single real Runtime Record Capture orchestrator (env-corrected).

Director-authorized change contract (2026-08-30):
- set temporary LD_LIBRARY_PATH / DISPLAY / XAUTHORITY for the MuJoCo, ROS2,
  HUD and record-recorder child processes only;
- launch ONE controlled simulation-only run;
- write raw logs/facts/record/summary under /tmp/p1_09ae_capture_v2/;
- archive evidence to docs/evidence/P1-09/ afterwards (done by the operator).

Correction vs the 2026-08-30 11:28 failed attempt: this orchestrator exports
LD_LIBRARY_PATH with unitree_sdk2/lib + libtorch in the PROCESS environment, so
EVERY child (MuJoCo, ros2 launch, control pub, HUD, recorder, summary) inherits
it — matching scripts/launch_abs_sim.sh line 24. The failed attempt only set it
on the MuJoCo child.

The task requires the recorder STOP (sampling) before the controller exits and
FINALIZE only after real process facts are known; the existing thin CLI
record_runtime_run.py couples stop_sampling()+finalize() in one signal path, so
this orchestrator drives the SAME existing recorder lifecycle through its module
API (run_record.RunRecordRecorder + abs_rt_frame.read_shm_frame).

No project code/config/schema/controller is modified; one run only; no retry.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path("/home/lidio/quadruped_robots")
SCRIPTS = ROOT / "scripts"
MUJOCO_DIR = ROOT / "unitree_mujoco"
MUJOCO_BIN = MUJOCO_DIR / "simulate" / "build2" / "unitree_mujoco"
ROS2_WS = ROOT / "quadruped_ros2_control_humble"
SDK_LIB = "/home/lidio/Libraries/unitree_sdk2/lib"
TORCH_LIB = "/home/lidio/Libraries/libtorch-cpu-2.0.1/lib"
CAP = Path("/tmp/p1_09ae_capture_v2")
RECORD_PATH = CAP / "record.jsonl"
FACTS_PATH = CAP / "process_facts.json"
MUJOCO_LOG = CAP / "mujoco_raw.log"
ROS_LOG = CAP / "ros2_launch_raw.log"
HUD_LOG = CAP / "hud_raw.txt"

# --- env correction: every child process inherits the runtime library paths ---
os.environ["LD_LIBRARY_PATH"] = f"{SDK_LIB}:{TORCH_LIB}:" + os.environ.get("LD_LIBRARY_PATH", "")

sys.path.insert(0, str(SCRIPTS))
from abs_rt_frame import FrameStatus, classify_frame, read_shm_frame  # noqa: E402
from run_record import RunRecordRecorder  # noqa: E402

ORCH = open(CAP / "orchestrator_raw.log", "w", encoding="utf-8", buffering=1)


def log(msg: str) -> None:
    line = f"{time.strftime('%Y-%m-%dT%H:%M:%S%z')} {msg}"
    print(line, flush=True)
    ORCH.write(line + "\n")
    ORCH.flush()


def wait_rc(p: subprocess.Popen, timeout: float, name: str):
    try:
        rc = p.wait(timeout=timeout)
    except subprocess.TimeoutExpired:
        log(f"WAIT_TIMEOUT {name} pid={p.pid} timeout_s={timeout}")
        return None
    log(f"PROCESS_EXIT {name} pid={p.pid} rc={rc}")
    return rc


def pgid_of(pid: int):
    try:
        return os.getpgid(pid)
    except (ProcessLookupError, PermissionError):
        return None


def sigdisp(pid: int) -> None:
    try:
        for line in Path(f"/proc/{pid}/status").read_text().splitlines():
            if line.startswith("SigIgn:") or line.startswith("SigCgt:"):
                log(f"SIGDISP pid={pid} {line.strip()}")
    except Exception as exc:  # noqa: BLE001 - diagnostic only
        log(f"SIGDISP pid={pid} error={exc}")


def bash_src() -> str:
    return (
        "source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
        f"source {ROS2_WS}/install/setup.bash >/dev/null 2>&1; "
    )


PGREP_PAT = r"unitree_mujoco|mujoco\.launch|ros2 launch|controller_manager"


def main() -> int:
    log("P1-09AE_BEGIN scene=scene_flat.xml interface=lo launch=mujoco.launch.py simulation_test=0")
    log(f"DISPLAY_ENV DISPLAY={os.environ.get('DISPLAY')} XAUTHORITY={os.environ.get('XAUTHORITY')}")

    log("PROCESS_PRECHECK")
    for path in (MUJOCO_BIN, MUJOCO_DIR / "unitree_robots" / "go2" / "scene_flat.xml"):
        if not path.exists():
            log(f"PRECHECK_FAIL missing={path}")
            return 2
    existing_shm = list(Path("/dev/shm").glob("mujoco_*"))
    log(f"PRECHECK_SHM existing={[p.name for p in existing_shm]}")
    out = subprocess.run(["pgrep", "-af", PGREP_PAT],
                         capture_output=True, text=True).stdout.strip()
    log(f"PRECHECK_PGREP [{out}]")

    # ------------------------------------------------------------- 1. MuJoCo
    log("PROCESS_START mujoco")
    mujoco = subprocess.Popen(
        [str(MUJOCO_BIN), "-s", "scene_flat.xml"],
        cwd=str(MUJOCO_DIR), env=os.environ, start_new_session=True,
        stdout=open(MUJOCO_LOG, "wb"), stderr=subprocess.STDOUT,
    )
    mujoco_pgid = pgid_of(mujoco.pid)
    log(f"PROCESS_START mujoco pid={mujoco.pid} pgid={mujoco_pgid}")
    time.sleep(3)
    if mujoco.poll() is not None:
        log(f"MUJOCO_EXITED_EARLY rc={mujoco.returncode} -> FAIL, no retry")
        return 3
    sigdisp(mujoco.pid)

    # ------------------------------------------------------------- 2. ROS2
    log("PROCESS_START ros2_launch")
    ros_cmd = (
        bash_src()
        + f"cd {ROS2_WS}; "
        + "exec ros2 launch rl_quadruped_controller mujoco.launch.py simulation_test:=0"
    )
    ros = subprocess.Popen(
        ["bash", "-lc", ros_cmd],
        start_new_session=True,
        stdout=open(ROS_LOG, "wb"), stderr=subprocess.STDOUT,
    )
    ros_pgid = pgid_of(ros.pid)
    log(f"PROCESS_START ros2_launch pid={ros.pid} pgid={ros_pgid}")

    # ------------------------------------------------------------- controller ready
    deadline = time.time() + 45
    ready = False
    list_cmd = bash_src() + "ros2 control list_controllers"
    last_stdout = ""
    while time.time() < deadline:
        try:
            r = subprocess.run(["bash", "-lc", list_cmd], capture_output=True, text=True, timeout=15)
            last_stdout = r.stdout
        except subprocess.TimeoutExpired:
            last_stdout = ""
        for line in last_stdout.splitlines():
            if "rl_quadruped_controller" in line and "active" in line and "inactive" not in line:
                ready = True
                break
        if ready:
            break
        time.sleep(1)
    log(f"CONTROLLER_READY={1 if ready else 0} last=[{last_stdout.strip()[:200]}]")
    if not ready:
        log("CONTROLLER_READY_TIMEOUT -> abort, no retry")
        mujoco.kill()
        return 4

    # ------------------------------------------------------------- 3. HUD
    log("PROCESS_START hud")
    hud = subprocess.Popen(
        ["bash", "-lc", f"cd {SCRIPTS}; exec python3 abs_live_hud.py --iters 60"],
        start_new_session=True,
        stdout=open(HUD_LOG, "wb"), stderr=subprocess.STDOUT,
    )
    log(f"PROCESS_START hud pid={hud.pid}")

    # ------------------------------------------------------------- 4. recorder (existing module API)
    recorder = RunRecordRecorder(str(RECORD_PATH))
    recorder.start()
    log(f"RECORDER_START run_id={recorder.run_id} -> {RECORD_PATH} state={recorder.state}")

    sample_stats = {"live": 0, "missing": 0, "other": 0}
    sample_stop = {"v": False}

    def sample_loop() -> None:
        while not sample_stop["v"]:
            raw = read_shm_frame()
            try:
                line = recorder.record_snapshot(raw)
            except RuntimeError:
                break
            st = line.get("status")
            if st == "LIVE":
                sample_stats["live"] += 1
            elif st == "MISSING":
                sample_stats["missing"] += 1
            else:
                sample_stats["other"] += 1
            time.sleep(0.05)

    sample_thread = threading.Thread(target=sample_loop, daemon=True)
    sample_thread.start()

    # ------------------------------------------------------------- 5. control 2 -> 2 -> 3
    def pub_cmd(cmd: int, lx: float = 0.0, ly: float = 0.0, rx: float = 0.0, ry: float = 0.0) -> int:
        msg = "{" + f"command: {cmd}, lx: {lx}, ly: {ly}, rx: {rx}, ry: {ry}" + "}"
        shell = bash_src() + (
            "ros2 topic pub --once --qos-reliability reliable "
            f"/control_input control_input_msgs/msg/Inputs '{msg}'"
        )
        log(f"CONTROL_COMMAND command={cmd} lx={lx} ly={ly} rx={rx} ry={ry}")
        try:
            r = subprocess.run(["bash", "-lc", shell], capture_output=True, text=True, timeout=30)
        except subprocess.TimeoutExpired:
            log("CONTROL_PUB_TIMEOUT")
            return -1
        log(f"CONTROL_PUB_RC rc={r.returncode}")
        return r.returncode

    pub_cmd(2)
    time.sleep(0.5)
    pub_cmd(2)
    log("WAIT_FIXEDDOWN_TO_FIXEDSTAND")
    time.sleep(4)
    pub_cmd(3, ly=1.0)
    time.sleep(0.5)
    pub_cmd(3, ly=1.0)
    log("WAIT_FIXEDSTAND_TO_RL")
    time.sleep(3)

    # ------------------------------------------------------------- 6. confirm RL LIVE
    rl_frame = None
    dl = time.time() + 20
    while time.time() < dl:
        raw = read_shm_frame()
        status, frame = classify_frame(raw, time.monotonic_ns())
        if status is FrameStatus.LIVE and frame.rl_active:
            rl_frame = frame
            break
        time.sleep(0.1)
    if rl_frame is None:
        log("RL_LIVE_CONFIRMED=0 (no LIVE rl_active frame within 20s)")
    else:
        log(f"RL_LIVE_CONFIRMED session_id={rl_frame.session_id} rl_step={rl_frame.rl_step} "
            f"policy_state={rl_frame.policy_state} ra_value={rl_frame.ra_value:.4f}")

    # ------------------------------------------------------------- 7. short observation window
    WINDOW_S = 10.0
    log(f"OBSERVE_WINDOW start window_s={WINDOW_S}")
    time.sleep(WINDOW_S)
    log(f"OBSERVE_WINDOW end stats_live={sample_stats['live']} missing={sample_stats['missing']} other={sample_stats['other']}")

    # ------------------------------------------------------------- 8. STOP recorder (no terminal yet)
    recorder.stop_sampling()
    sample_stop["v"] = True
    sample_thread.join(timeout=2)
    log(f"RECORDER_STOPPED state={recorder.state} finalized={recorder.finalized}")
    lines = RECORD_PATH.read_text().splitlines()
    last_kind = json.loads(lines[-1]).get("kind") if lines else None
    log(f"RECORDER_STOP_VERIFY last_line_kind={last_kind} terminal_present={last_kind == 'terminal'} line_count={len(lines)}")

    # ------------------------------------------------------------- 9. normal process exit
    def stop_group(p, name: str, saved_pid, saved_pgid, first_sig: int, timeout_s: float) -> tuple:
        sigdisp(p.pid)
        log(f"STOP_REQUEST {name} pid={p.pid} pgid={saved_pgid} signal={first_sig}")
        try:
            os.kill(saved_pid, first_sig)
            kill_rc = 0
        except (ProcessLookupError, PermissionError):
            kill_rc = -1
        log(f"STOP_KILL_RC {name} leader_pid={saved_pid} rc={kill_rc}")
        rc = wait_rc(p, timeout_s, name)
        forced = False
        if rc is None:
            log(f"STOP_TIMEOUT {name} -> SIGTERM escalation to group (cleanup only)")
            forced = True
            try:
                os.killpg(saved_pgid, signal.SIGTERM)
            except (ProcessLookupError, PermissionError):
                pass
            rc = wait_rc(p, 10, name)
        return rc, forced

    ros_rc, ros_forced = stop_group(ros, "ros2_launch", ros.pid, ros_pgid, signal.SIGINT, 15)
    mujoco_rc, mujoco_forced = stop_group(mujoco, "mujoco", mujoco.pid, mujoco_pgid, signal.SIGINT, 15)

    # Let the bounded HUD finish / clean up.
    try:
        hud_rc = hud.wait(timeout=10)
    except subprocess.TimeoutExpired:
        log("HUD_TIMEOUT kill")
        hud.kill()
        hud_rc = hud.wait(timeout=5)
    log(f"PROCESS_EXIT hud pid={hud.pid} rc={hud_rc}")

    # ------------------------------------------------------------- 10. residue check
    time.sleep(1)
    out = subprocess.run(["pgrep", "-af", PGREP_PAT],
                         capture_output=True, text=True).stdout.strip()
    residue = bool(out)
    log(f"PROCESS_POSTCHECK residue={'YES' if residue else 'NONE'} [{out}]")

    # ------------------------------------------------------------- 11. real process facts
    forced = bool(ros_forced or mujoco_forced)
    exit_code = 0 if (ros_rc == 0 and mujoco_rc == 0) else 1
    shutdown_complete = (ros_rc is not None and mujoco_rc is not None) and not residue
    src = "recorder_stop;SIGINT_ros_launch;SIGINT_mujoco"
    if ros_forced:
        src += ";SIGTERM_ros_launch_group"
    if mujoco_forced:
        src += ";SIGTERM_mujoco_group"
    facts = {
        "exit_code": exit_code,
        "forced_termination": forced,
        "shutdown_complete": shutdown_complete,
        "shutdown_request_source": src,
        "mujoco_rc": mujoco_rc,
        "ros_launch_rc": ros_rc,
    }
    FACTS_PATH.write_text(json.dumps(facts, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    log(f"FACTS_WRITTEN {json.dumps(facts, sort_keys=True)}")

    # ------------------------------------------------------------- 12. FINALIZE recorder
    terminal = recorder.finalize(facts)
    log(f"RECORDER_FINALIZE run_id={recorder.run_id} state={recorder.state} "
        f"normal_shutdown={terminal.get('normal_shutdown')} term={terminal.get('termination_reason')} "
        f"frames_observed={terminal.get('frames_observed')}")

    # ------------------------------------------------------------- 13. post-run summary
    r = subprocess.run(["python3", "post_run_summary.py", str(RECORD_PATH), "--json"],
                       cwd=str(SCRIPTS), capture_output=True, text=True, timeout=30)
    (CAP / "post_run_summary.json").write_text(r.stdout, encoding="utf-8")
    log(f"POST_RUN_SUMMARY rc={r.returncode}")
    rt = subprocess.run(["python3", "post_run_summary.py", str(RECORD_PATH)],
                        cwd=str(SCRIPTS), capture_output=True, text=True, timeout=30)
    (CAP / "post_run_summary.txt").write_text(rt.stdout, encoding="utf-8")

    # ------------------------------------------------------------- 14. error scan
    for path, tag in ((MUJOCO_LOG, "mujoco"), (ROS_LOG, "ros2"), (HUD_LOG, "hud")):
        txt = path.read_text(errors="replace")
        hits = [w for w in ("terminate", "SIGABRT", "abort", "Segmentation", "core dump", "GLFW")
                if w in txt]
        log(f"ERROR_SCAN {tag} hits={hits}")

    log(f"STATUS ros_rc={ros_rc} ros_forced={ros_forced} mujoco_rc={mujoco_rc} mujoco_forced={mujoco_forced} "
        f"exit_code={exit_code} forced={forced} shutdown_complete={shutdown_complete} "
        f"live_frames={sample_stats['live']} run_id={recorder.run_id}")
    log("P1-09AE_END")
    ORCH.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
