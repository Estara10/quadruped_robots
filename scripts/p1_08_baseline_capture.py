#!/usr/bin/env python3
"""P1-08 — one controlled v2 sim-clock baseline capture (single run, no retry).

Hardened recapture harness. Exactly one run; any preflight or run failure is
archived and the run stops with BLOCKED/FAILED FOR THIS RUN — no retry.

Fail-closed guarantees implemented here:
  - PREFLIGHT runs `ldd` under the EXACT `child_env()` used for the ROS launch
    and checks returncode/timeout/exception/not-found; the command, env
    summary, stdout/stderr and returncode are archived.
  - The actual launched binary and scene are bound to the v2 manifest: the
    binary hash must equal the manifest `mujoco_executable` hash; the scene must
    resolve (by the MuJoCo rule) to the manifest canonical `root_xml` and its
    hash must match the closure; path escape / missing / mismatch = PRECHECK
    FAIL.
  - Shm cleanup is narrow: only the task's exact named shms are handled, and
    only after confirming no live process using executable/argv identity
    (residual-process check); an unlink failure is PRECHECK FAIL.
  - The runtime record is fail-closed: EVERY distinct present frame (LIVE or
    malformed/non-authoritative/STALE/SYNTHETIC) is passed raw to
    `RunRecordRecorder.record_snapshot`; MISSING is only a legal gap; any
    present bad frame makes the whole record INVALID (never filtered to VALID).
  - Two-phase exit facts: the top-level coordinator `exit_code` is 0 ONLY when
    both required children actually wait()==0; per-process PID/PGID/signal
    timeline/wait-rc/escalation are preserved; `shutdown_request_source` is the
    real signal ("SIGINT").
  - All post-launch branches enter one try/finally cleanup (stop sampling, real
    SIGINT + wait per started child, TERM escalation only on timeout with each
    actual signal recorded, facts + raw logs saved, recorder finalized with real
    facts, no orphans).
  - The capture window is FIXED at 25 s; any other value fails preflight.

Run under a ROS-sourced shell, e.g.:
    bash -c 'source /opt/ros/humble/setup.bash \
             && source <ws>/install/setup.bash \
             && python3 scripts/p1_08_baseline_capture.py --out-dir <fresh-dir> \
                 --manifest <manifest.json>'
"""

from __future__ import annotations

import argparse
import fcntl
import json
import math
import mmap
import os
import signal
import secrets
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SCRIPTS = REPO / "scripts"
sys.path.insert(0, str(SCRIPTS))

from abs_rt_frame import RuntimeFrame, read_shm_frame  # noqa: E402
from build_p1_08_manifest import resolve_closure  # noqa: E402
from p1_10_scenario_suite import (  # noqa: E402
    bind_capture_identity,
    prepare_capture_context,
    write_resolved_manifest,
)
from run_record import RunRecordRecorder, summarize_record  # noqa: E402

RT_MAGIC = 0x414253525446524D  # "ABSRTFRM"
RT_SOURCE_AUTHORITATIVE = 1
SIM_CLOCK_MAGIC = 0x414253434C4F434B  # "ABSCLOCK"
SIM_CLOCK_VERSION = 2                 # contract v2
SIM_CLOCK_SHM = "/dev/shm/mujoco_sim_clock"
SIM_CLOCK_STRUCT = struct.Struct("<4Qd")
SIM_CLOCK_SIZE = SIM_CLOCK_STRUCT.size

# Task's exact shared-memory names (only these are handled, and only after the
# residual-process check confirms no live user).
TASK_SHM_CLEANUP = [
    "/dev/shm/mujoco_sim_clock",
    "/dev/shm/mujoco_rt_frame",
]

LOG_F = None

# Harness-owned exclusive capture lock. Prevents two instances of THIS harness
# from racing on preflight + shm cleanup. It does NOT (and cannot) control
# external non-cooperating processes; those are checked fail-closed separately.
CAPTURE_LOCK_PATH = Path(tempfile.gettempdir()) / "abs_p1_08_capture.lock"


class CaptureLock:
    """Non-blocking exclusive flock over a lock file."""

    def __init__(self, path: Path = CAPTURE_LOCK_PATH):
        self.path = path
        self._fh = None

    def acquire(self) -> bool:
        try:
            self._fh = open(self.path, "w")
            fcntl.flock(self._fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            return True
        except OSError:
            return False

    def release(self) -> None:
        if self._fh is not None:
            try:
                fcntl.flock(self._fh.fileno(), fcntl.LOCK_UN)
            finally:
                self._fh.close()
                self._fh = None


def log(msg: str) -> None:
    line = f"[{time.strftime('%H:%M:%S')}] {msg}"
    print(line, flush=True)
    if LOG_F:
        LOG_F.write(line + "\n")
        LOG_F.flush()


def sha256_file(path: str) -> str:
    import hashlib
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def run_cmd(cmd, timeout_s=10):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except subprocess.TimeoutExpired:
        return -1, "timeout"
    except Exception as e:
        return -1, f"exception:{e}"


# ---------------------------------------------------------------------------
# child environment (shared by ldd preflight AND all launches)
# ---------------------------------------------------------------------------
def child_env():
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = os.pathsep.join([
        "/home/lidio/Libraries/unitree_sdk2/lib",
        "/home/lidio/Libraries/libtorch-cpu-2.0.1/lib",
        env.get("LD_LIBRARY_PATH", ""),
    ])
    return env


# ---------------------------------------------------------------------------
# sim-clock reader (unchanged API; used by tests + orchestrator)
# ---------------------------------------------------------------------------
def _sim_clock_raw(shm_path: str):
    try:
        fd = os.open(shm_path, os.O_RDONLY)
    except OSError:
        return None
    try:
        if os.fstat(fd).st_size < SIM_CLOCK_SIZE:
            return None
        buf = mmap.mmap(fd, SIM_CLOCK_SIZE, access=mmap.ACCESS_READ)
        try:
            return bytes(buf[:SIM_CLOCK_SIZE])
        finally:
            buf.close()
    finally:
        os.close(fd)


def decode_sim_clock_snapshot(raw: bytes):
    if len(raw) != SIM_CLOCK_SIZE:
        return None
    m, v, seq, mono, sim = SIM_CLOCK_STRUCT.unpack(raw)
    if m != SIM_CLOCK_MAGIC:
        return None
    if v != SIM_CLOCK_VERSION:
        return None
    if seq == 0 or (seq & 1):
        return None
    if mono == 0:
        return None
    if not math.isfinite(sim):
        return None
    return (seq, mono, sim)


def read_sim_clock_from(raw_snapshot, max_attempts: int = 5):
    for _ in range(max_attempts):
        a = decode_sim_clock_snapshot(raw_snapshot())
        if a is None:
            continue
        b = decode_sim_clock_snapshot(raw_snapshot())
        if b is None:
            continue
        if b[0] != a[0]:
            continue
        return b
    return None


def read_sim_clock(shm_path: str = SIM_CLOCK_SHM, max_attempts: int = 5):
    def raw():
        return _sim_clock_raw(shm_path)
    return read_sim_clock_from(raw, max_attempts)


def read_sim_clock_detailed(shm_path: str = SIM_CLOCK_SHM, max_attempts: int = 5):
    reason = "unavailable"
    for _ in range(max_attempts):
        raw = _sim_clock_raw(shm_path)
        if raw is None:
            continue
        if len(raw) != SIM_CLOCK_SIZE:
            reason = "short"
            continue
        m, v, seq, mono, sim = SIM_CLOCK_STRUCT.unpack(raw)
        if m != SIM_CLOCK_MAGIC:
            reason = "magic"
            continue
        if v != SIM_CLOCK_VERSION:
            reason = "version"
            continue
        if seq == 0:
            reason = "seq0"
            continue
        if seq & 1:
            reason = "odd"
            continue
        if mono == 0:
            reason = "mono0"
            continue
        if not math.isfinite(sim):
            reason = "nonfinite"
            continue
        raw2 = _sim_clock_raw(shm_path)
        if raw2 is None:
            reason = "unavailable2"
            continue
        m2, v2, seq2, mono2, sim2 = SIM_CLOCK_STRUCT.unpack(raw2)
        if seq2 != seq or (seq2 & 1):
            reason = "changed"
            continue
        return ("accepted", (seq2, mono2, sim2))
    return ("rejected", reason)


# ---------------------------------------------------------------------------
# A. preflight pieces
# ---------------------------------------------------------------------------
def resolve_scene_path(mujoco_bin: str, scene_arg: str) -> Path:
    """Mirror the MuJoCo `-s` resolution: exe_dir.parent.parent /
    unitree_robots / go2 / <scene>."""
    exe_dir = Path(mujoco_bin).resolve().parent
    return (exe_dir.parent.parent / "unitree_robots" / "go2" / scene_arg).resolve()


def run_ldd(plugin_so: Path, env: dict, runner=None, timeout_s: int = 10) -> dict:
    """Run `ldd` under the EXACT child env. Returns an evidence dict."""
    runner = runner or subprocess.run
    base = {
        "command": ["ldd", str(plugin_so)],
        "env_summary": {"LD_LIBRARY_PATH": env.get("LD_LIBRARY_PATH", "")},
        "returncode": None, "stdout": "", "stderr": "", "exception": None,
    }
    try:
        p = runner(["ldd", str(plugin_so)], capture_output=True, text=True,
                   env=env, timeout=timeout_s)
        base.update({"returncode": p.returncode, "stdout": p.stdout or "",
                     "stderr": p.stderr or ""})
    except subprocess.TimeoutExpired as e:
        base.update({"returncode": None, "exception": "timeout",
                     "stdout": (e.stdout or b"" if isinstance(e.stdout, bytes) else e.stdout or ""),
                     "stderr": (e.stderr or b"" if isinstance(e.stderr, bytes) else e.stderr or "")})
    except Exception as e:  # noqa: BLE001
        base["exception"] = f"{type(e).__name__}: {e}"
    combined = (base["stdout"] if isinstance(base["stdout"], str) else str(base["stdout"])) + \
               (base["stderr"] if isinstance(base["stderr"], str) else str(base["stderr"]))
    base["not_found"] = [ln for ln in combined.splitlines() if "not found" in ln]
    base["ok"] = (base["returncode"] == 0 and base["exception"] is None
                  and "libddsc.so" in combined and not base["not_found"])
    return base


def check_ldd(plugin_so: Path, env: dict):
    ev = run_ldd(plugin_so, env)
    return ev["ok"], ev


def verify_manifest_hashes(manifest_path: Path, mujoco_bin: str, scene_arg: str):
    """Validate the ACTUAL launched binary + scene + FULL model closure +
    recorded artifacts/configs against the manifest. Returns (failures, evidence)."""
    m = json.loads(manifest_path.read_text())
    failures = []
    evidence = {}

    # actual launched binary must equal manifest mujoco_executable
    expected_bin = next((b["sha256"] for b in m["binaries"]
                         if b["role"] == "mujoco_executable" and b["sha256"]), None)
    actual_bin = sha256_file(str(Path(mujoco_bin).resolve()))
    evidence["mujoco_bin_actual"] = actual_bin
    evidence["mujoco_bin_manifest"] = expected_bin
    if expected_bin is None or actual_bin != expected_bin:
        failures.append(f"actual mujoco binary mismatch vs manifest ({mujoco_bin})")

    # scene must resolve to the canonical closure root_xml (no path escape).
    resolved = resolve_scene_path(mujoco_bin, scene_arg)
    root = Path(m["model_closure"]["root_xml"]).resolve()
    evidence["scene_resolved"] = str(resolved)
    evidence["scene_root_manifest"] = str(root)
    if "/" in scene_arg or ".." in scene_arg or scene_arg != Path(scene_arg).name:
        failures.append(f"scene path escape/reference: {scene_arg}")
    if resolved != root:
        failures.append(f"scene {scene_arg} resolves to {resolved}, manifest root is {root}")

    # FULL model closure: re-discover recursively and compare the whole closure
    # (all included XMLs + assets) against the manifest — not just the root XML.
    closure = m.get("model_closure", {})
    fresh = resolve_closure(root)
    evidence["fresh_closure_failures"] = fresh["failures"]
    if fresh["failures"]:
        failures.append(f"fresh closure failed: {fresh['failures']}")
    recorded_closure_sha = closure.get("closure_sha256")
    if recorded_closure_sha is None:
        failures.append("manifest missing model_closure.closure_sha256")
    elif fresh["closure_sha256"] != recorded_closure_sha:
        failures.append("fresh closure_sha256 != manifest closure_sha256")
    # every manifest-recorded XML must be present with matching hash
    manifest_xml = {str(Path(x["path"]).resolve()): x for x in closure.get("xml_files", [])}
    fresh_xml = {str(Path(x["path"]).resolve()): x for x in fresh["xml_files"]}
    for p, x in manifest_xml.items():
        if x.get("present"):
            fx = fresh_xml.get(p)
            if fx is None:
                failures.append(f"manifest XML missing from fresh closure: {p}")
            elif fx["sha256"] != x["sha256"]:
                failures.append(f"XML hash mismatch: {p}")
    evidence["manifest_included_xml_count"] = len(closure.get("included_xml_files", []))

    # recorded artifacts/configs/plugins (installed paths) must match manifest
    for kind, group in (("binary", m["binaries"]),
                        ("artifact", m["deployed_policy_artifacts"]),
                        ("config", m["config_files"])):
        for item in group:
            if not item.get("present"):
                continue
            actual = sha256_file(item["path"])
            if actual != item["sha256"]:
                failures.append(f"{kind} {item['role']} hash mismatch: {item['path']}")
    evidence["failures"] = failures
    return failures, evidence


def clean_task_shms():
    """Narrow cleanup: only the task's exact named shms, only after the caller
    has confirmed no live process. Records before/after state. Returns
    (ok, evidence); an unlink failure or uncertain state is fail-closed."""
    evidence = {"before": {}, "after": {}, "removed": []}
    for p in TASK_SHM_CLEANUP:
        evidence["before"][p] = "present" if os.path.exists(p) else "absent"
    for p in TASK_SHM_CLEANUP:
        if os.path.exists(p):
            try:
                os.unlink(p)
                evidence["removed"].append(p)
            except OSError as e:
                return False, {**evidence, "error": f"unlink_failed {p}: {e}"}
    for p in TASK_SHM_CLEANUP:
        evidence["after"][p] = "present" if os.path.exists(p) else "absent"
        if os.path.exists(p):
            return False, {**evidence, "error": f"still_present_after_unlink {p}"}
    return True, evidence


DEFAULT_MUJOCO_BIN = REPO / "unitree_mujoco" / "simulate" / "build2" / "unitree_mujoco"
DEFAULT_CONTROLLER_CONFIG = (
    REPO / "quadruped_ros2_control_humble" / "descriptions" / "unitree" /
    "go2_description" / "config" / "robot_control.yaml"
)


class ProcessInspectionError(RuntimeError):
    """Process identity could not be inspected without ambiguity."""


def _parse_proc_stat(stat_text: str) -> tuple:
    """Parse state and PPID from /proc/<pid>/stat."""
    close = stat_text.rfind(")")
    if close < 0:
        raise ProcessInspectionError("/proc stat has no closing comm delimiter")
    fields = stat_text[close + 2:].split()
    # After the final ')' fields[0] is state and fields[1] is ppid.
    if len(fields) < 2 or not fields[0]:
        raise ProcessInspectionError("/proc stat has no state/ppid fields")
    try:
        return fields[0], int(fields[1])
    except ValueError as exc:
        raise ProcessInspectionError("/proc stat ppid is not an integer") from exc


def _read_process_record(pid: int, proc_root: Path = Path("/proc")) -> dict:
    """Read one process identity atomically enough for preflight purposes.

    The executable symlink is authoritative for identity.  argv is retained
    only to attribute ROS launch/controller processes to this capture stack.
    Any read failure is an inspection error; callers must fail closed.
    """
    proc_dir = proc_root / str(pid)
    try:
        exe = os.path.realpath(os.readlink(proc_dir / "exe"))
        raw_cmdline = (proc_dir / "cmdline").read_bytes()
        stat_text = (proc_dir / "stat").read_text()
    except (OSError, UnicodeError) as exc:
        raise ProcessInspectionError(f"pid={pid}: unable to read identity: {exc}") from exc
    argv = [part.decode("utf-8", errors="strict") for part in raw_cmdline.split(b"\0") if part]
    state, ppid = _parse_proc_stat(stat_text)
    return {"pid": pid, "ppid": ppid, "state": state, "exe": exe, "argv": argv}


def _read_process_records(proc_root: Path = Path("/proc")) -> list:
    """Read all numeric /proc entries, failing closed on any inspection error."""
    try:
        pids = sorted(int(entry.name) for entry in proc_root.iterdir() if entry.name.isdigit())
    except (OSError, ValueError) as exc:
        raise ProcessInspectionError(f"unable to enumerate process table: {exc}") from exc
    return [_read_process_record(pid, proc_root) for pid in pids]


def _ancestor_pids(records: list, self_pid: int | None = None) -> set:
    """Return self and every ancestor PID from the same inspected snapshot."""
    self_pid = os.getpid() if self_pid is None else int(self_pid)
    by_pid = {int(record["pid"]): record for record in records}
    if self_pid not in by_pid:
        raise ProcessInspectionError(f"self pid={self_pid} absent from process snapshot")
    excluded = {self_pid}
    pid = self_pid
    while pid != 1:
        record = by_pid.get(pid)
        if record is None or "ppid" not in record:
            raise ProcessInspectionError(f"ancestor chain missing for pid={pid}")
        parent = int(record["ppid"])
        if parent <= 0 or parent == pid:
            raise ProcessInspectionError(f"invalid ancestor link pid={pid} ppid={parent}")
        excluded.add(parent)
        pid = parent
    return excluded


def _argv_has_ros2_launch(argv: list) -> bool:
    """Require the actual ros2 launcher argv, not a shell's text command."""
    has_ros2_executable = any(Path(token).name == "ros2" for token in argv)
    for index in range(len(argv) - 3):
        if (argv[index] == "launch" and argv[index + 1] == "rl_quadruped_controller"
                and argv[index + 2] == "mujoco.launch.py"):
            return has_ros2_executable
    return False


def _argv_has_controller_config(argv: list, expected_config: Path) -> bool:
    """Attribute ros2_control_node to this Go2 capture's controller config."""
    installed_config = (
        REPO / "quadruped_ros2_control_humble" / "install" / "go2_description" /
        "share" / "go2_description" / "config" / "robot_control.yaml"
    )
    # The launch substitution may use either the source path or the installed
    # package-share path.  Keep the allowlist exact; do not accept an arbitrary
    # command line that merely ends in a similarly named YAML file.
    allowed = {
        str(expected_config), str(expected_config.resolve()),
        str(installed_config), str(installed_config.resolve()),
    }
    return any(token in allowed for token in argv)


def _classify_runtime_process(record: dict, expected_mujoco: Path,
                              expected_controller_config: Path) -> tuple:
    """Return (kind, reason), (None, None), or raise on identity ambiguity."""
    try:
        pid = int(record["pid"])
        state = str(record["state"])
        exe = str(record["exe"])
        argv = list(record["argv"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ProcessInspectionError(f"malformed process identity record: {record!r}") from exc
    if len(state) != 1:
        raise ProcessInspectionError(f"pid={pid} has ambiguous process state={state!r}")
    expected = str(expected_mujoco.resolve())
    if exe == expected or exe == expected + " (deleted)":
        if state == "Z":
            raise ProcessInspectionError(f"pid={pid} is a zombie MuJoCo process")
        return "mujoco", f"pid={pid} exe={exe}"
    exe_name = Path(exe).name
    if exe_name == "ros2_control_node":
        if state == "Z":
            raise ProcessInspectionError(f"pid={pid} is a zombie controller process")
        if _argv_has_controller_config(argv, expected_controller_config):
            return "ros2_control_node", f"pid={pid} exe={exe} capture_config=matched"
        raise ProcessInspectionError(
            f"pid={pid} is ros2_control_node but capture-stack config identity is ambiguous")
    if exe_name == "ros2" or exe_name == "python" or exe_name.startswith("python3"):
        if _argv_has_ros2_launch(argv):
            if state == "Z":
                raise ProcessInspectionError(f"pid={pid} is a zombie ROS launch process")
            return "ros2_launch", f"pid={pid} exe={exe} argv=capture_launch"
    # A shell/Python command that merely mentions the simulator path is not a
    # runtime identity match; its executable was inspected and is unrelated.
    return None, None


def inspect_residual_processes(expected_mujoco: str | Path = DEFAULT_MUJOCO_BIN,
                               expected_controller_config: str | Path = DEFAULT_CONTROLLER_CONFIG,
                               *, process_records: list | None = None,
                               inspector=None, excluded_pids: set | None = None,
                               self_pid: int | None = None) -> tuple:
    """Identity-based residual check: (state, structured evidence).

    Only an exact MuJoCo executable or an attributable ROS launch/controller
    process is a match.  Inspection failures and ambiguous controller identity
    are ``uncertain`` and must be rejected by preflight.  ``process_records`` /
    ``inspector`` are deliberately injectable for offline tests.
    """
    evidence = {"method": "proc_identity_v1", "matches": [], "excluded_pids": []}
    try:
        if process_records is None:
            process_records = (inspector() if inspector is not None
                               else _read_process_records())
        if not isinstance(process_records, list):
            raise ProcessInspectionError("process inspector did not return a list")
        if excluded_pids is None:
            excluded_pids = _ancestor_pids(process_records, self_pid=self_pid)
        excluded_pids = {int(pid) for pid in excluded_pids}
        evidence["excluded_pids"] = sorted(excluded_pids)
        for record in process_records:
            pid = int(record["pid"])
            if pid in excluded_pids:
                continue
            kind, detail = _classify_runtime_process(
                record, Path(expected_mujoco), Path(expected_controller_config))
            if kind is not None:
                evidence["matches"].append({"pid": pid, "kind": kind, "detail": detail})
        if evidence["matches"]:
            return "found", evidence
        return "none", evidence
    except Exception as exc:  # noqa: BLE001 - process identity must fail closed
        evidence["inspection_error"] = f"{type(exc).__name__}: {exc}"
        return "uncertain", evidence


def create_capture_id() -> str:
    """Generate the per-launch identity; it is never accepted as CLI input."""
    return "p1-10-capture-" + secrets.token_hex(16)


def preflight(args, out: Path, capture_id: str):
    """Returns (ok, evidence, env). On FAIL (or any exception), the evidence is
    archived by the caller and the out-dir is never created. No bare-exception
    exit: the caller wraps this in try/except and records a structured reason."""
    evidence = {"checks": {}}
    env = child_env()

    # P1-10 scenario/seed binding is checked before any runtime prerequisite.
    # The resolved context is not inferred from the legacy P1-08 defaults.
    resolved_context = bind_capture_identity(prepare_capture_context(args), capture_id)
    env.update(resolved_context["process_context"]["environment"])
    evidence["checks"]["scenario_context"] = {
        "ok": True,
        "scenario_id": resolved_context["scenario_id"],
        "scenario_sha256": resolved_context["scenario_sha256"],
        "capture_identity": resolved_context["capture_identity"],
        "suite_sha256": resolved_context["suite_sha256"],
        "root_seed": resolved_context["seeds"]["root_seed"],
        "root_seed_role": "pairing/provenance_only",
        "random_consumer": "none in current flat capture path",
        "variant": resolved_context["pairing"]["variant"],
        "launch_contract": resolved_context["launch_contract"],
        "capture_id": capture_id,
    }
    evidence["scenario_context"] = resolved_context

    # 1. residual processes: inspect executable identity and attributable argv.
    # Never match the harness command line itself or an ancestor shell.
    state, detail = inspect_residual_processes(
        args.mujoco_bin, DEFAULT_CONTROLLER_CONFIG)
    if state != "none":
        evidence["checks"]["residual_processes"] = {"state": state, **detail}
        return False, evidence, env
    evidence["checks"]["residual_processes"] = {"state": "none", **detail}

    # 2. X11
    rc, _ = run_cmd(["xdpyinfo"])
    if rc != 0:
        evidence["checks"]["x11"] = f"xdpyinfo rc={rc}"
        return False, evidence, env
    evidence["checks"]["x11"] = "reachable"

    # 3. ldd under the EXACT child env (archived)
    plugin_so = REPO / "quadruped_ros2_control_humble" / "install" / "hardware_unitree_mujoco" / "lib" / "libhardware_unitree_mujoco.so"
    if not plugin_so.exists():
        evidence["checks"]["ldd"] = {"ok": False, "reason": "plugin missing"}
        return False, evidence, env
    ldd_ok, ldd_ev = check_ldd(plugin_so, env)
    evidence["checks"]["ldd"] = ldd_ev
    if not ldd_ok:
        return False, evidence, env

    # 4. manifest identity binding (actual binary + scene + FULL closure)
    if args.manifest:
        failures, man_ev = verify_manifest_hashes(Path(args.manifest), args.mujoco_bin, args.scene)
        evidence["checks"]["manifest"] = man_ev
        if failures:
            return False, evidence, env
    else:
        evidence["checks"]["manifest"] = {"failures": ["no --manifest given (PRECHECK FAIL)"]}
        return False, evidence, env

    # 5. fresh capture dir
    if out.exists():
        evidence["checks"]["fresh_out_dir"] = "exists"
        return False, evidence, env
    evidence["checks"]["fresh_out_dir"] = "fresh"

    # 6. fixed 25 s window
    if args.window_s != 25.0:
        evidence["checks"]["window_s"] = args.window_s
        return False, evidence, env
    evidence["checks"]["window_s"] = 25.0

    # 7. narrow shm cleanup (before/after/spawn checks)
    ok, clean_ev = clean_task_shms()
    evidence["checks"]["shm_cleanup"] = clean_ev
    if not ok:
        return False, evidence, env
    # re-confirm no process appeared between the first check and now (spawn guard)
    state2, detail2 = inspect_residual_processes(
        args.mujoco_bin, DEFAULT_CONTROLLER_CONFIG)
    if state2 != "none":
        evidence["checks"]["residual_processes_after_cleanup"] = {"state": state2, **detail2}
        return False, evidence, env
    evidence["checks"]["residual_processes_after_cleanup"] = {"state": "none", **detail2}

    return True, evidence, env


# ---------------------------------------------------------------------------
# B. sample loop: fail-closed record + timing + reader stats
# ---------------------------------------------------------------------------
def compute_stride2_gaps(distinct_evens):
    """Compute v2 sim-clock publish-gap statistics over a list of DISTINCT,
    strictly-increasing, EVEN sequence numbers (the v2 writer advances its even
    sequence by +2 per publish: odd in-progress marker then even).

    For each advancing pair: missing = (next - prev) / 2 - 1.
    A non-even, non-increasing (rollback), or duplicate value is fail-closed:
    it is recorded in `errors` and never fabricated into a gap.

    Returns {"distinct": n, "pairs": n-1, "total_missing": int,
             "max_single_gap": int, "sequence_stride": 2, "errors": [...]}.
    """
    errors = []
    total = 0
    mx = 0
    pairs = 0
    prev = None
    for seq in distinct_evens:
        if seq % 2 != 0:
            errors.append(f"non-even sequence {seq}")
            continue
        if prev is not None:
            if seq <= prev:
                errors.append(f"sequence not strictly increasing/rollback {prev} -> {seq}")
                continue
            pairs += 1
            missing = (seq - prev) // 2 - 1
            if missing < 0:
                errors.append(f"negative gap {prev} -> {seq}")
                continue
            total += missing
            mx = max(mx, missing)
        prev = seq
    return {"distinct": len(distinct_evens), "pairs": pairs,
            "total_missing": total, "max_single_gap": mx,
            "sequence_stride": 2, "errors": errors}


def sample_and_record(out: Path, recorder, window_s: float) -> dict:
    rt_fh = open(out / "rt_frame_timing.jsonl", "w")
    sim_fh = open(out / "sim_clock_timing.jsonl", "w")
    stats = {
        "sim_clock": {"attempts": 0, "accepted": 0, "rejected": 0,
                      "reasons": {}, "seq_gaps": 0, "seq_gap_max": 0},
        "rt_frame": {"attempts": 0, "accepted": 0, "rejected": 0,
                     "reasons": {}, "rl_step_gaps": 0, "rl_step_gap_max": 0},
        "record_statuses": {},
    }
    last_recorded_raw = None   # recorder dedup: every DISTINCT snapshot once
    last_seq = None
    last_rl_step = None
    distinct_evens = []        # distinct accepted sim-clock even sequences
    deadline = time.monotonic() + window_s
    while time.monotonic() < deadline:
        # ---- sim clock (reader stats + timing) ----
        stats["sim_clock"]["attempts"] += 1
        status, value = read_sim_clock_detailed()
        if status == "accepted":
            stats["sim_clock"]["accepted"] += 1
            seq, mono, sim = value
            if last_seq is None or seq != last_seq:
                # DISTINCT accepted snapshot -> one timing row; gaps computed
                # post-window over the distinct even (stride-2) sequence.
                distinct_evens.append(seq)
                sim_fh.write(json.dumps({"sequence": seq, "monotonic_ns": mono, "sim_time": sim}) + "\n")
                last_seq = seq
        else:
            stats["sim_clock"]["rejected"] += 1
            stats["sim_clock"]["reasons"][value] = stats["sim_clock"]["reasons"].get(value, 0) + 1

        # ---- rt frame: recorder (fail-closed) + timing + stats ----
        raw = read_shm_frame()
        stats["rt_frame"]["attempts"] += 1
        # RECORD: every DISTINCT present snapshot (or MISSING transition) reaches
        # the fail-closed recorder; duplicates are the same frame, not re-recorded.
        if raw != last_recorded_raw:
            line = recorder.record_snapshot(raw)
            stats["record_statuses"][line.get("status", "?")] = \
                stats["record_statuses"].get(line.get("status", "?"), 0) + 1
            last_recorded_raw = raw
        # TIMING + stats on the snapshot
        if raw:
            try:
                f = RuntimeFrame.from_bytes(raw)
            except Exception:
                stats["rt_frame"]["rejected"] += 1
                stats["rt_frame"]["reasons"]["malformed"] = stats["rt_frame"]["reasons"].get("malformed", 0) + 1
            else:
                if f.magic != RT_MAGIC or f.source != RT_SOURCE_AUTHORITATIVE or f.rl_active != 1:
                    stats["rt_frame"]["rejected"] += 1
                    reason = "not_live" if f.rl_active != 1 else "source_or_magic"
                    stats["rt_frame"]["reasons"][reason] = stats["rt_frame"]["reasons"].get(reason, 0) + 1
                else:
                    stats["rt_frame"]["accepted"] += 1
                    if f.rl_step > (last_rl_step if last_rl_step is not None else -1):
                        if last_rl_step is not None:
                            gap = f.rl_step - last_rl_step - 1  # rl_step stride-1
                            if gap > 0:
                                stats["rt_frame"]["rl_step_gaps"] += gap
                                stats["rt_frame"]["rl_step_gap_max"] = max(stats["rt_frame"]["rl_step_gap_max"], gap)
                        last_rl_step = f.rl_step
                        rt_fh.write(json.dumps({
                            "monotonic_ns": f.monotonic_ns, "rl_step": f.rl_step,
                            "policy_state": f.policy_state, "session_id": f.session_id,
                            "ra_value": f.ra_value}) + "\n")
        else:
            stats["rt_frame"]["rejected"] += 1
            stats["rt_frame"]["reasons"]["unavailable"] = stats["rt_frame"]["reasons"].get("unavailable", 0) + 1

    rt_fh.close()
    sim_fh.close()

    # --- post-window: stride-2 gap summary (sim_clock), fail-closed ---
    gap_stats = compute_stride2_gaps(distinct_evens)
    stats["sim_clock"]["distinct_accepted"] = gap_stats["distinct"]
    stats["sim_clock"]["seq_gaps"] = gap_stats["total_missing"]
    stats["sim_clock"]["seq_gap_max"] = gap_stats["max_single_gap"]
    stats["sim_clock"]["sequence_stride"] = gap_stats["sequence_stride"]
    stats["sim_clock"]["gap_errors"] = gap_stats["errors"]
    return stats


# ---------------------------------------------------------------------------
# C. two-phase shutdown / facts / cleanup
# ---------------------------------------------------------------------------
# Required children whose real wait() facts determine the coordinator outcome.
REQUIRED_CHILDREN = ["mujoco", "ros2_launch"]
# Shutdown order: ROS controller before MuJoCo (matches P1-09 shutdown order).
SHUTDOWN_ORDER = ["ros2_launch", "mujoco"]


def _delivered(signals, sig_names):
    return any(s.get("delivered") is True and s.get("signal") in sig_names
               for s in signals)


def build_process_facts(meta, run_id, start_wall, end_wall, scene, p1_10_context=None):
    """Derive the top-level coordinator facts from per-child state.

    Semantics (fail-closed, no fabrication):
      - forced_termination == True ONLY when a TERM/KILL was actually DELIVERED
        (a plain nonzero exit or a failed escalation is NOT forced).
      - shutdown_request_source == "SIGINT" only when SIGINT was actually
        DELIVERED to at least one target; a failed/absent SIGINT send leaves it
        "UNKNOWN" (never fabricated as SIGINT).
      - per-child `escalated` is recomputed from the delivered signal timeline so
        it is consistent with `signals` (delivered=True only).
      - exit_code == 0 only when every REQUIRED child actually waited with rc==0;
        a nonzero rc maps to a deterministic nonzero value; a missing/None wait
        maps to None (UNKNOWN), never fabricated as 0.
    """
    per = {}
    rcs = []
    for name in REQUIRED_CHILDREN:
        if name not in meta:
            per[name] = {"pid": None, "pgid": None, "exit_code": None,
                         "signals": [], "escalated": False, "not_launched": True,
                         "cleanup_errors": [], "poll_attempts": []}
            rcs.append(None)
            continue
        m = meta[name]
        signals = list(m.get("signals", []))
        per[name] = {"pid": m["proc"].pid, "pgid": m["proc"].pid,
                     "exit_code": m.get("wait_rc"),
                     "signals": signals,
                     "escalated": _delivered(signals, ("SIGTERM", "SIGKILL")),
                     "not_launched": False,
                     "cleanup_errors": list(m.get("cleanup_errors", [])),
                     "poll_attempts": list(m.get("poll_attempts", []))}
        rcs.append(m.get("wait_rc"))

    escalated = any(per[n].get("escalated") for n in REQUIRED_CHILDREN)
    any_sigint_delivered = any(_delivered(per[n]["signals"], ("SIGINT",))
                               for n in REQUIRED_CHILDREN)
    if all(rc == 0 for rc in rcs):
        top_exit = 0
    elif any(rc is not None and rc != 0 for rc in rcs):
        top_exit = next(rc for rc in rcs if rc is not None and rc != 0)
    else:
        top_exit = None  # some child missing / unwaited

    cleanup_error_count = sum(len(per[n].get("cleanup_errors", []))
                              for n in REQUIRED_CHILDREN)
    facts = {
        "run_id": run_id,
        "start_wall": start_wall,
        "end_wall": end_wall,
        "exit_code": top_exit,
        "forced_termination": escalated,
        "shutdown_request_source": "SIGINT" if any_sigint_delivered else "UNKNOWN",
        "shutdown_complete": all(rc is not None for rc in rcs),
        "cleanup_error_count": cleanup_error_count,
        "scene": scene,
        **{f"child.{name}": v for name, v in per.items()},
    }
    if p1_10_context is not None:
        facts["capture_id"] = p1_10_context["capture_identity"]["capture_id"]
        facts["p1_10_context"] = p1_10_context
    return facts


def wait_pid(proc, timeout_s: float) -> int:
    """Wait up to `timeout_s` for `proc` to exit. Poll exceptions are tolerated
    (retry with sleep); returns the real rc once poll succeeds, or None at the
    deadline when the exit cannot be confirmed (UNKNOWN — never fabricated)."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            rc = proc.poll()
        except Exception:  # noqa: BLE001  (poll failure -> cannot confirm, keep trying)
            time.sleep(0.2)
            continue
        if rc is not None:
            return rc
        time.sleep(0.2)
    return None


def _signal_pg(proc, sig, name, meta):
    """Send `sig` to `proc`'s process group and record the DELIVERY truth.

    Returns True only when the signal was actually delivered (killpg returned
    without error). ANY ordinary Exception (not BaseException/SystemExit/
    KeyboardInterrupt) is caught and recorded with delivered=False plus an
    exception_type/exception_message — never treated as delivered, never raised
    out to interrupt the cleanup loop. The signal, target PID/PGID and request
    time are always preserved.
    """
    t = time.monotonic()
    delivered = False
    result = None
    exc_type = None
    exc_msg = None
    pid = None
    try:
        pid = proc.pid
        os.killpg(pid, sig)
        delivered = True
        result = "delivered"
    except Exception as e:  # noqa: BLE001
        delivered = False
        result = f"failed:{e}"
        exc_type = type(e).__name__
        exc_msg = str(e)
    entry = {"signal": sig.name, "time_s": t, "delivered": delivered,
             "result": result, "target_pid": pid, "target_pgid": pid,
             "exception_type": exc_type, "exception_message": exc_msg}
    meta.setdefault(name, {"proc": proc, "signals": [], "wait_rc": None, "escalated": False})
    meta[name]["signals"].append(entry)
    return delivered


def _wait_or_escalate(proc, name, meta):
    """SIGINT already sent by caller; wait, then TERM then KILL only on timeout.
    `escalated` is set True ONLY when a TERM/KILL was actually delivered; a failed
    escalation is recorded in the signal timeline with delivered=False."""
    rc = wait_pid(proc, 60.0)
    if rc is not None:
        meta[name]["wait_rc"] = rc
        return rc
    # escalate: TERM (escalated only if actually delivered)
    if _signal_pg(proc, signal.SIGTERM, name, meta):
        meta[name]["escalated"] = True
    rc = wait_pid(proc, 10.0)
    if rc is not None:
        meta[name]["wait_rc"] = rc
        return rc
    # escalate: KILL (escalated only if actually delivered)
    if _signal_pg(proc, signal.SIGKILL, name, meta):
        meta[name]["escalated"] = True
    rc = wait_pid(proc, 5.0)
    meta[name]["wait_rc"] = rc
    return rc


def _finalize_capture(children, meta, recorder, run_id, start_wall, scene, stats, out,
                      p1_10_context=None, mujoco_bin=DEFAULT_MUJOCO_BIN):
    """Unified post-launch lifecycle (single path for success + all failures).

    Order: stop_sampling -> SIGINT + wait (TERM/KILL only on timeout) ->
    write process_facts.json -> finalize recorder with the same facts ->
    save reader stats. No early-return path bypasses this."""
    # 1. stop sampling (only if started and not yet finalized)
    if recorder is not None and not recorder.finalized:
        try:
            recorder.stop_sampling()
        except Exception as e:  # noqa: BLE001
            log(f"cleanup: stop_sampling: {e}")

    # 2-4. SIGINT + wait + escalate (ordered: ros2 first, then mujoco).
    # Each child's signal/wait is exception-guarded so one child's failure never
    # interrupts the cleanup of the remaining children; every exception is
    # recorded (cleanup_errors) and a wait is still attempted (UNKNOWN/failed,
    # never a fabricated success).
    def _handle_child(name, proc):
        m = meta.setdefault(name, {"proc": proc, "signals": [], "wait_rc": None,
                                   "escalated": False, "cleanup_errors": [],
                                   "poll_attempts": []})
        # poll attempt — a poll exception is recorded, NOT an early return: the
        # child is treated as state-unknown/running and still gets signal + wait.
        poll_exc = None
        poll_rc = None
        try:
            poll_rc = proc.poll()
        except Exception as e:  # noqa: BLE001
            poll_exc = e
        if poll_exc is not None:
            m["poll_attempts"].append({"stage": "poll", "result": "exception",
                                       "exception_type": type(poll_exc).__name__,
                                       "exception_message": str(poll_exc),
                                       "time_s": time.monotonic()})
            log(f"cleanup: {name} poll exception: {poll_exc}")
        elif poll_rc is not None:
            m["poll_attempts"].append({"stage": "poll", "result": "exited",
                                       "rc": poll_rc, "time_s": time.monotonic()})
            m["wait_rc"] = poll_rc
            log(f"cleanup: {name} already exited rc={poll_rc}")
            return
        else:
            m["poll_attempts"].append({"stage": "poll", "result": "running",
                                       "time_s": time.monotonic()})
        # state unknown or running: attempt signal + wait (recorded separately)
        try:
            _signal_pg(proc, signal.SIGINT, name, meta)
            _wait_or_escalate(proc, name, meta)
        except Exception as e:  # noqa: BLE001
            m["cleanup_errors"].append({"stage": "signal_or_wait",
                                        "exception_type": type(e).__name__,
                                        "exception_message": str(e),
                                        "time_s": time.monotonic()})
            log(f"cleanup: {name} signal/wait exception: {e}")
            try:
                m["wait_rc"] = wait_pid(proc, 5.0)
            except Exception as e2:  # noqa: BLE001
                m["cleanup_errors"].append({"stage": "wait",
                                            "exception_type": type(e2).__name__,
                                            "exception_message": str(e2),
                                            "time_s": time.monotonic()})
        log(f"cleanup: {name} wait_rc={m.get('wait_rc')} escalated={m.get('escalated')}")

    for name in SHUTDOWN_ORDER:
        if name in children:
            _handle_child(name, children[name])
    # any child not in SHUTDOWN_ORDER (defensive) — still SIGINT+wait
    for name, proc in children.items():
        if name not in SHUTDOWN_ORDER:
            _handle_child(name, proc)

    # 5. build + write process_facts.json BEFORE finalize (never bypassed)
    end_wall = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    facts = build_process_facts(meta, run_id, start_wall, end_wall, scene,
                                p1_10_context=p1_10_context)
    try:
        (out / "process_facts.json").write_text(json.dumps(facts, indent=2) + "\n")
    except Exception as e:  # noqa: BLE001
        log(f"cleanup: process_facts write failed: {e}")

    # 6. finalize recorder with the same facts
    if recorder is not None and not recorder.finalized:
        try:
            recorder.finalize(facts)
            recorder.close()
            log(f"cleanup: recorder finalized run_id={run_id}")
        except Exception as e:  # noqa: BLE001
            log(f"cleanup: recorder finalize failed: {e}")

    # 7. save reader stats + cleanup result
    if stats is not None:
        try:
            (out / "reader_stats.json").write_text(json.dumps(stats, indent=2, sort_keys=True) + "\n")
        except Exception as e:  # noqa: BLE001
            log(f"cleanup: reader_stats write failed: {e}")

    # 8. capture-end orphan/process inventory snapshot. This is an ARCHIVED
    # capture-end check (documented as such) — it is not a substitute for an
    # independent supervisor and must not be over-claimed as a formal proof.
    _write_orphan_inventory(out, end_wall, mujoco_bin=mujoco_bin)
    return facts


def _write_orphan_inventory(out: Path, end_wall: str,
                            mujoco_bin: str | Path = DEFAULT_MUJOCO_BIN) -> dict:
    """Persist a capture-end orphan/process inventory artifact for FUTURE runs.

    The 2026-09-02 v2 capture predates this artifact, so its post-run orphan
    inventory is UNKNOWN (no independent inventory was archived at capture end).
    This method ensures future captures archive one."""
    inv = {
        "artifact": "orphan_inventory",
        "end_wall": end_wall,
        "note": "capture-end process identity snapshot; documents post-cleanup "
                "process state, not an independent-supervisor proof",
    }
    try:
        state, detail = inspect_residual_processes(
            mujoco_bin, DEFAULT_CONTROLLER_CONFIG)
        inv["process_identity_state"] = state
        inv["process_identity_evidence"] = detail
    except Exception as e:  # noqa: BLE001
        inv["process_identity_state"] = "error"
        inv["process_identity_evidence"] = {
            "inspection_error": f"{type(e).__name__}: {e}"
        }
    try:
        (out / "orphan_inventory.json").write_text(
            json.dumps(inv, indent=2, sort_keys=True) + "\n")
    except Exception as e:  # noqa: BLE001
        log(f"cleanup: orphan_inventory write failed: {e}")
    return inv


def launch_and_run(env, out: Path, args) -> int:
    """Launch + run + unified two-phase cleanup. Never orphans."""
    global LOG_F
    children = {}
    meta = {}
    recorder = None
    run_id = None
    stats = None
    result = 7  # default: exception/unknown
    LOG_F = open(out / "orchestrator_raw.log", "w")
    start_wall = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    facts = None
    p1_10_context = getattr(args, "_p1_10_context", None)
    try:
        mujoco_dir = REPO / "unitree_mujoco"
        ros2_ws = REPO / "quadruped_ros2_control_humble"
        mujoco_log = open(out / "mujoco_raw.log", "w")
        log(f"P1-10 context: scenario={p1_10_context['scenario_id']} "
            f"scenario_sha256={p1_10_context['scenario_sha256']} "
            f"suite_sha256={p1_10_context['suite_sha256']} "
            f"root_seed={p1_10_context['seeds']['root_seed']} "
            f"variant={p1_10_context['pairing']['variant']} "
            f"variant_binding={p1_10_context['variant_binding']['binding_sha256']} "
            f"initial_state={p1_10_context['initial_state']['binding_sha256']} "
            f"baseline_identity={p1_10_context['baseline']['identity_sha256']} "
            f"capture_id={p1_10_context['capture_identity']['capture_id']} "
            f"runtime_model_fingerprint={p1_10_context['scene']['runtime_model_fingerprint']}")
        log(f"launch MuJoCo: {args.mujoco_bin} -s {args.scene}")
        mujoco = subprocess.Popen([args.mujoco_bin, "-s", args.scene], cwd=str(mujoco_dir),
                                  env=env, stdout=mujoco_log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        children["mujoco"] = mujoco
        log(f"MuJoCo pid={mujoco.pid} pgid={mujoco.pid}")
        if not wait_sim_clock_advance(20.0):
            log("FAIL: v2 sim clock never advanced")
            result = 3
            return result
        log("v2 sim clock advancing")

        ros_log = open(out / "ros2_launch_raw.log", "w")
        log("launch ros2: mujoco.launch.py simulation_test:=0")
        ros_proc = subprocess.Popen(
            ["ros2", "launch", "rl_quadruped_controller", "mujoco.launch.py", "simulation_test:=0"],
            cwd=str(ros2_ws), env=env, stdout=ros_log, stderr=subprocess.STDOUT,
            start_new_session=True)
        children["ros2_launch"] = ros_proc
        log(f"ros2 launch pid={ros_proc.pid} pgid={ros_proc.pid}")
        if not wait_controller_active(90.0):
            log("FAIL: controller never active")
            result = 4
            return result
        log("controller active")

        goal_input = p1_10_context["goal_injection"]["control_input"]
        log("auto-RL: pub 2 -> 2 -> 3 with scenario-declared goal trim "
            f"lx={goal_input['lx']} ly={goal_input['ly']} rx={goal_input['rx']}")
        pub_input(2, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.5)
        pub_input(2, 0.0, 0.0, 0.0, 0.0)
        time.sleep(4.0)
        pub_input(3, goal_input["lx"], goal_input["ly"], goal_input["rx"], 0.0)
        time.sleep(0.5)
        pub_input(3, goal_input["lx"], goal_input["ly"], goal_input["rx"], 0.0)
        if not wait_rl_active(30.0):
            log("FAIL: RL never active")
            result = 5
            return result
        log("RL active")

        record_path = out / "runtime_record.jsonl"
        capture_id = p1_10_context["capture_identity"]["capture_id"]
        expected_fingerprint = p1_10_context["scene"]["runtime_model_fingerprint"]
        recorder = RunRecordRecorder(str(record_path), capture_id=capture_id,
                                     expected_fingerprint=expected_fingerprint)
        run_id = recorder.start()
        log(f"runtime record started run_id={run_id}")

        log(f"sampling for {args.window_s:.1f}s (fixed)")
        stats = sample_and_record(out, recorder, args.window_s)
        log(f"captured sim_clock={stats['sim_clock']['accepted']} rt_frame={stats['rt_frame']['accepted']} "
            f"record_statuses={stats['record_statuses']}")

        result = 0
    except Exception as e:  # noqa: BLE001
        log(f"EXCEPTION during capture: {type(e).__name__}: {e}")
        result = 7
    finally:
        # Unified cleanup for success, early-failure, exception, and timeout.
        facts = _finalize_capture(children, meta, recorder, run_id, start_wall,
                                  args.scene, stats, out,
                                  p1_10_context=p1_10_context,
                                  mujoco_bin=args.mujoco_bin)
        if LOG_F:
            LOG_F.close()
            LOG_F = None

    # post-cleanup: determine final result (0 only on a complete valid run)
    if result != 0:
        log(f"== FAILED result={result} facts={facts} ==")
        return result
    record_valid = False
    try:
        summary = summarize_record(str(out / "runtime_record.jsonl"))
        record_valid = summary.get("record_validity") == "VALID"
    except Exception:  # noqa: BLE001
        record_valid = False
    ok = (facts is not None and facts.get("exit_code") == 0
          and not facts.get("forced_termination") and facts.get("shutdown_complete")
          and record_valid
          and stats is not None
          and stats["sim_clock"]["accepted"] >= 100 and stats["rt_frame"]["accepted"] >= 100)
    log(f"== DONE ok={ok} exit_code={facts.get('exit_code') if facts else None} "
        f"forced={facts.get('forced_termination') if facts else None} "
        f"record_validity={'VALID' if record_valid else 'INVALID'} ==")
    return 0 if ok else 6


def wait_sim_clock_advance(timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    last_seq = None
    while time.monotonic() < deadline:
        status, value = read_sim_clock_detailed()
        if status == "accepted":
            seq = value[0]
            if last_seq is not None and seq > last_seq:
                return True
            last_seq = seq if last_seq is None else max(last_seq, seq)
        time.sleep(0.005)
    return False


def wait_controller_active(timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rc, o = run_cmd(["ros2", "control", "list_controllers"])
        if rc == 0 and "rl_quadruped_controller" in o and "active" in o:
            return True
        time.sleep(1.0)
    return False


def pub_input(command: int, lx: float, ly: float, rx: float, ry: float) -> bool:
    cmd = ["ros2", "topic", "pub", "--once", "/control_input", "control_input_msgs/msg/Inputs",
           f"{{command: {command}, lx: {lx}, ly: {ly}, rx: {rx}, ry: {ry}}}"]
    rc, _ = run_cmd(cmd)
    return rc == 0


def wait_rl_active(timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        data = read_shm_frame()
        if data:
            try:
                f = RuntimeFrame.from_bytes(data)
                if f.magic == RT_MAGIC and f.source == RT_SOURCE_AUTHORITATIVE and f.rl_active == 1:
                    return True
            except Exception:
                pass
        time.sleep(0.2)
    return False


def _archive_preflight_fail(out: Path, evidence) -> None:
    p = Path(str(out) + "_preflight_fail.json")
    p.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")
    print(f"PRECHECK FAIL — not launching (archived {p})", flush=True)


def build_p1_10_context(resolved_context):
    """Build the complete P1-10 context directly from scenario resolution.

    Every value is copied by explicit key from the already-validated resolved
    scenario context. A missing resolution field raises before write; no pair
    manifest, comparator, default, or other-run fallback is involved.
    """
    context = {
        "scenario_id": resolved_context["scenario_id"],
        "scenario_path": resolved_context["scenario_path"],
        "scenario_sha256": resolved_context["scenario_sha256"],
        "suite_path": resolved_context["suite_path"],
        "suite_sha256": resolved_context["suite_sha256"],
        "scene": resolved_context["scene"],
        "baseline": resolved_context["baseline"],
        "initial_state_source": resolved_context["initial_state_source"],
        "initial_state": resolved_context["initial_state"],
        "variant_binding": resolved_context["variant_binding"],
        "switching_mode": resolved_context["switching_mode"],
        "seeds": resolved_context["seeds"],
        "pairing": resolved_context["pairing"],
        "formal_context": resolved_context["formal_context"],
        "process_context": resolved_context["process_context"],
        "launch_contract": resolved_context["launch_contract"],
        "run_window_s": resolved_context["run_window_s"],
    }
    if "capture_identity" in resolved_context:
        context["capture_identity"] = resolved_context["capture_identity"]
        context["capture_identity_input"] = resolved_context["capture_identity_input"]
    return context


def write_p1_10_context(path: Path, resolved_context) -> None:
    """Persist the full P1-10 context emitted by the production harness."""
    write_resolved_manifest(path, build_p1_10_context(resolved_context))


def capture(args) -> int:
    out = Path(args.out_dir).resolve()
    lock = CaptureLock()
    if not lock.acquire():
        _archive_preflight_fail(out, {"checks": {
            "capture_lock": "held (another harness instance running; refusing to start)"}})
        return 2
    try:
        # preflight is wrapped: any exception becomes a structured PRECHECK FAIL
        # (never a bare traceback with a lost reason).
        capture_id = create_capture_id()
        try:
            ok, evidence, env = preflight(args, out, capture_id)
        except Exception as e:  # noqa: BLE001
            _archive_preflight_fail(out, {"checks": {
                "preflight_exception": f"{type(e).__name__}: {e}"}})
            return 2
        if not ok:
            _archive_preflight_fail(out, evidence)
            return 2
        out.mkdir(parents=True)
        # New P1-10 context is written only into this new capture directory;
        # no existing P1-08 evidence is amended or reinterpreted.
        resolved_context = evidence["scenario_context"]
        write_resolved_manifest(out / "scenario_resolved_manifest.json", resolved_context)
        write_p1_10_context(out / "p1_10_context.json", resolved_context)
        args._p1_10_context = resolved_context
        (out / "preflight_evidence.json").write_text(
            json.dumps(evidence, indent=2, sort_keys=True) + "\n")
        return launch_and_run(env, out, args)
    finally:
        lock.release()


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--window-s", type=float, default=25.0, help="MUST be 25.0 (fixed window)")
    ap.add_argument("--scene", default="scene_flat.xml")
    ap.add_argument("--mujoco-bin", default=str(REPO / "unitree_mujoco" / "simulate" / "build2" / "unitree_mujoco"))
    ap.add_argument("--manifest", required=True)
    ap.add_argument("--scenario", required=True,
                     help="registered P1-10 scenario ID or repository-relative JSON path")
    ap.add_argument("--root-seed", required=True, type=int)
    ap.add_argument("--variant", required=True,
                     choices=("paper-faithful", "stabilized", "agile-only"))
    ap.add_argument("--initial-state-source", required=True,
                    choices=("scene_default",),
                    help="actual MuJoCo startup state source; no keyframe reset is used")
    args = ap.parse_args()
    return capture(args)


if __name__ == "__main__":
    sys.exit(main())
