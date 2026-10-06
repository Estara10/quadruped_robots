#!/usr/bin/env python3
"""P1-08 — offline harness tests (NO MuJoCo/ROS2 launch).

Covers:
  A. full model-closure: escape / cycle / missing / mutation.
  B. preflight: child-env ldd normal/nonzero/not-found/timeout/exception;
     binary/scene mismatch; preflight exception -> structured FAIL; capture lock.
  C. cleanup order + process-facts semantics: stop_sampling before signal; facts
     before finalize; natural/SIGINT/nonzero/TERM/KILL/missing-wait semantics;
     early-failure still waits + archives.
  D. record fail-closed (present bad frame -> INVALID); top-level facts read by
     RunRecordRecorder; fixed 25 s window.

Run: python3 scripts/test_p1_08_harness.py
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import subprocess
import sys
import tempfile
from pathlib import Path
from unittest import mock

SCRIPTS = Path(__file__).resolve().parent
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))

import p1_08_baseline_capture as capmod  # noqa: E402
from p1_08_baseline_capture import (  # noqa: E402
    REQUIRED_CHILDREN,
    build_process_facts,
    check_ldd,
    child_env,
    resolve_scene_path,
    run_ldd,
    verify_manifest_hashes,
)
from build_p1_08_manifest import resolve_closure  # noqa: E402
from abs_rt_frame import FRAME_MAGIC, FRAME_VERSION, SOURCE_AUTHORITATIVE_RUNTIME  # noqa: E402
from abs_rt_frame import _FRAME_STRUCT  # noqa: E402
from run_record import RunRecordRecorder, summarize_record  # noqa: E402

g_checks = 0
g_fail = False


def check(cond: bool, label: str) -> None:
    global g_checks, g_fail
    g_checks += 1
    if not cond:
        g_fail = True
        print(f"FAIL: {label}")


class FakeProc:
    def __init__(self, pid, rc=None):
        self.pid = pid
        self._rc = rc

    def poll(self):
        return self._rc


def make_frame(magic=FRAME_MAGIC, version=FRAME_VERSION, seq=2, mono=0,
               session=42, rl_step=1, source=SOURCE_AUTHORITATIVE_RUNTIME,
               rl_active=1, ra_value=0.0):
    import time as _t
    if mono == 0:
        mono = _t.monotonic_ns()
    vals = [magic, version, seq, mono, session, rl_step, 0]  # 7Q
    vals += [source, 1, 1, rl_active, 0, 0, 1, 1, 0, 0, 0]   # 11I
    floats = [ra_value] + [0.0] * 80
    return _FRAME_STRUCT.pack(*(vals + floats))


# ---------------------------------------------------------------------------
# A. full model closure
# ---------------------------------------------------------------------------
def test_stride2_gap_math():
    """compute_stride2_gaps: consecutive stride-2, single gap, multi gap,
    non-even / rollback fail-closed."""
    from p1_08_baseline_capture import compute_stride2_gaps as g
    # consecutive stride-2 (4,6,8,10) -> no gaps
    r = g([4, 6, 8, 10])
    check(r["total_missing"] == 0 and r["max_single_gap"] == 0 and r["errors"] == [],
          "consecutive stride-2 -> 0 gaps")
    # single gap: 4 -> 8 means one publish missing between them
    r = g([4, 8])
    check(r["total_missing"] == 1 and r["max_single_gap"] == 1,
          "single stride-2 gap -> total 1 max 1")
    # multi gap: 4 -> 24 -> 26 = (24-4)/2-1=9 missing then 0
    r = g([4, 24, 26])
    check(r["total_missing"] == 9 and r["max_single_gap"] == 9,
          "multi gap -> total 9 max 9")
    # Reviewer-fact scale: gap (next-prev)/2-1 == 10 for a 22-step difference
    r = g([4, 26])
    check(r["total_missing"] == 10 and r["max_single_gap"] == 10,
          "(26-4)/2-1 = 10 max gap")
    # non-even sequence -> fail-closed error, not fabricated gap
    r = g([4, 7])
    check(r["errors"] and "non-even" in r["errors"][0] and r["total_missing"] == 0,
          "non-even sequence -> fail-closed error")
    # rollback (not strictly increasing) -> fail-closed error
    r = g([10, 6, 12])
    check(r["errors"] and "rollback" in r["errors"][0], "rollback -> fail-closed error")
    # duplicates not counted (function expects distinct input; a repeated even is an error)
    r = g([4, 4])
    check(r["errors"], "duplicate even (non-increasing) -> error")


def test_closure_escape_cycle_missing_mutation():
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        (td / "root.xml").write_text('<mujoco><include file="child.xml"/></mujoco>')
        (td / "child.xml").write_text('<mujoco><mesh file="m.obj"/></mujoco>')
        (td / "assets").mkdir()
        (td / "assets" / "m.obj").write_text("mesh")

        # clean closure
        c = resolve_closure(td / "root.xml")
        check(c["failures"] == [], "clean closure has no failures")
        check(len(c["included_xml_files"]) == 1, "one included XML recorded")
        orig_sha = c["closure_sha256"]

        # mutation: change included XML -> closure_sha256 changes + hash differs
        (td / "child.xml").write_text('<mujoco><mesh file="m.obj"/>CHANGED</mujoco>')
        c2 = resolve_closure(td / "root.xml")
        check(c2["closure_sha256"] != orig_sha, "mutating included XML changes closure hash")

        # escape: include outside closure root
        (td / "outside.xml").write_text("<mujoco/>")
        (td / "root_escape.xml").write_text('<mujoco><include file="../outside.xml"/></mujoco>')
        c3 = resolve_closure(td / "root_escape.xml")
        check(any("escape" in f for f in c3["failures"]), "include escape detected")

        # cycle: a includes b includes a
        (td / "a.xml").write_text('<mujoco><include file="b.xml"/></mujoco>')
        (td / "b.xml").write_text('<mujoco><include file="a.xml"/></mujoco>')
        c4 = resolve_closure(td / "a.xml")
        check(any("cycle" in f for f in c4["failures"]), "include cycle detected")

        # missing include
        (td / "root_missing.xml").write_text('<mujoco><include file="nope.xml"/></mujoco>')
        c5 = resolve_closure(td / "root_missing.xml")
        check(any("missing" in f for f in c5["failures"]), "missing include detected")


def test_manifest_closure_positive():
    # positive: real manifest + real scene + real binary -> no closure failures
    man = REPO / "docs" / "evidence" / "P1-08" / "P1-08_baseline_manifest.json"
    binp = str(REPO / "unitree_mujoco" / "simulate" / "build2" / "unitree_mujoco")
    failures, _ = verify_manifest_hashes(man, binp, "scene_flat.xml")
    check(failures == [], f"real manifest closure+binary verifies clean (got {failures[:2]})")


# ---------------------------------------------------------------------------
# B. child-env ldd preflight
# ---------------------------------------------------------------------------
def test_ldd():
    plugin = REPO / "quadruped_ros2_control_humble" / "install" / "hardware_unitree_mujoco" / "lib" / "libhardware_unitree_mujoco.so"
    env = child_env()

    class R:
        def __init__(self, rc, out, err=""):
            self.returncode = rc
            self.stdout = out
            self.stderr = err

    ok, ev = check_ldd(plugin, env)
    check(ok, "ldd normal resolves under child env")
    check(ev["returncode"] == 0 and ev["exception"] is None, "ldd evidence rc=0")

    def runner_nonzero(cmd, **kw):
        return R(1, "", "")
    ev = run_ldd(plugin, env, runner=runner_nonzero)
    check(not ev["ok"] and ev["returncode"] == 1, "ldd nonzero -> FAIL")

    def runner_notfound(cmd, **kw):
        return R(0, "\tlibddsc.so.0 => not found\n", "")
    ev = run_ldd(plugin, env, runner=runner_notfound)
    check(not ev["ok"] and ev["not_found"], "ldd not-found -> FAIL")

    def runner_timeout(cmd, **kw):
        raise subprocess.TimeoutExpired(cmd, timeout=10)
    ev = run_ldd(plugin, env, runner=runner_timeout)
    check(not ev["ok"] and ev["exception"] == "timeout", "ldd timeout -> FAIL")

    def runner_exc(cmd, **kw):
        raise RuntimeError("boom")
    ev = run_ldd(plugin, env, runner=runner_exc)
    check(not ev["ok"] and "boom" in (ev["exception"] or ""), "ldd exception -> FAIL")


def test_residual_process_identity_fail_closed():
    """Residual detection uses executable identity, attribution and exclusions.

    These are injected process-table records: no process is started and no
    real runtime or shared memory is touched.
    """
    mujoco = str(capmod.DEFAULT_MUJOCO_BIN.resolve())
    controller_cfg = str(capmod.DEFAULT_CONTROLLER_CONFIG.resolve())

    def rec(pid, exe, argv, ppid=1, state="S"):
        return {"pid": pid, "ppid": ppid, "state": state, "exe": exe, "argv": argv}

    # Exact simulator identity remains a hard residual failure.
    state, ev = capmod.inspect_residual_processes(
        mujoco, controller_cfg,
        process_records=[rec(200, mujoco, [mujoco, "-s", "scene_flat.xml"])],
        excluded_pids={1})
    check(state == "found" and ev["matches"][0]["kind"] == "mujoco",
          "real MuJoCo identity -> residual FAIL")

    # A shell command that merely contains the simulator path is not a match.
    state, ev = capmod.inspect_residual_processes(
        mujoco, controller_cfg,
        process_records=[rec(201, "/usr/bin/bash", ["bash", "-lc", mujoco])],
        excluded_pids={1})
    check(state == "none" and ev["matches"] == [],
          "shell mentioning MuJoCo path -> not residual")

    # The same identity rule protects the controller, with capture-stack config.
    state, ev = capmod.inspect_residual_processes(
        mujoco, controller_cfg,
        process_records=[rec(202, "/opt/ros/humble/lib/controller_manager/ros2_control_node",
                             ["ros2_control_node", "--params-file", controller_cfg])],
        excluded_pids={1})
    check(state == "found" and ev["matches"][0]["kind"] == "ros2_control_node",
          "attributable controller identity -> residual FAIL")

    # Actual ros2 launch identity is also attributable; arbitrary ros2 text is not.
    launch_argv = ["/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch",
                   "rl_quadruped_controller", "mujoco.launch.py", "simulation_test:=0"]
    state, _ = capmod.inspect_residual_processes(
        mujoco, controller_cfg,
        process_records=[rec(203, "/usr/bin/python3", launch_argv)],
        excluded_pids={1})
    check(state == "found", "attributable ros2 launch identity -> residual FAIL")

    # Self and ancestor entries are excluded even if their command line has a
    # runtime-looking token; this is the false-positive regression.
    state, ev = capmod.inspect_residual_processes(
        mujoco, controller_cfg,
        process_records=[rec(204, mujoco, [mujoco, "-s", "scene_flat.xml"]),
                         rec(205, "/usr/bin/bash", ["bash", "-lc", "ros2 launch"])],
        excluded_pids={204, 205})
    check(state == "none" and ev["excluded_pids"] == [204, 205],
          "self/ancestor runtime-looking entries -> excluded")

    # An unreadable/ambiguous process table never becomes an allow decision.
    def inspector_failure():
        raise OSError("/proc denied")
    state, ev = capmod.inspect_residual_processes(
        mujoco, controller_cfg, inspector=inspector_failure, excluded_pids={1})
    check(state == "uncertain" and "inspection_error" in ev,
          "process inspection failure -> fail closed")

    state, ev = capmod.inspect_residual_processes(
        mujoco, controller_cfg,
        process_records=[rec(206, "/opt/ros/humble/lib/controller_manager/ros2_control_node",
                             ["ros2_control_node"])],
        excluded_pids={1})
    check(state == "uncertain" and "ambiguous" in ev["inspection_error"],
          "controller identity ambiguity -> fail closed")

    state, ev = capmod.inspect_residual_processes(
        mujoco, controller_cfg,
        process_records=[rec(207, "/usr/bin/python3", ["python3", "worker.py"]),
                         rec(208, "/usr/bin/bash", ["bash", "-lc", mujoco])],
        excluded_pids={1})
    check(state == "none" and ev["matches"] == [],
          "unrelated processes -> no residual accepted")

    state, ev = capmod.inspect_residual_processes(
        mujoco, controller_cfg,
        process_records=[rec(209, mujoco, [mujoco, "-s", "scene_flat.xml"], state="Z")],
        excluded_pids={1})
    check(state == "uncertain" and "zombie" in ev["inspection_error"],
          "relevant zombie identity -> fail closed")


def test_binary_scene_mismatch():
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        fake_bin = td / "fake_mujoco"
        fake_bin.write_bytes(b"not real")
        fake_scene = td / "fake_scene.xml"
        fake_scene.write_bytes(b"fake")
        manifest = {
            "binaries": [{"role": "mujoco_executable", "sha256": "0" * 64,
                          "present": True, "path": str(fake_bin)}],
            "deployed_policy_artifacts": [], "config_files": [],
            "model_closure": {"root_xml": str(fake_scene),
                              "xml_files": [{"path": str(fake_scene), "sha256": "0" * 64, "present": True}],
                              "asset_files": [], "closure_sha256": "0" * 64,
                              "included_xml_files": []},
            "git": {"commit": "x", "dirty": False},
        }
        mp = td / "m.json"
        mp.write_text(json.dumps(manifest))
        failures, _ = verify_manifest_hashes(mp, str(fake_bin), "../escape.xml")
        check(any("escape" in f or "path escape" in f for f in failures), "scene escape -> FAIL")
        failures, _ = verify_manifest_hashes(mp, str(fake_bin), "other.xml")
        check(any("resolves to" in f for f in failures), "scene mismatch -> FAIL")
        failures, _ = verify_manifest_hashes(mp, str(fake_bin), "fake_scene.xml")
        check(any("mujoco binary mismatch" in f for f in failures), "binary mismatch -> FAIL")


def test_preflight_exception_and_lock():
    args = argparse.Namespace(out_dir=str(REPO / "docs" / "evidence" / "P1-08" / "capture_PF_EXC"),
                              window_s=25.0, scene="scene_flat.xml",
                              mujoco_bin=str(REPO / "unitree_mujoco" / "simulate" / "build2" / "unitree_mujoco"),
                              manifest=str(REPO / "docs" / "evidence" / "P1-08" / "P1-08_baseline_manifest.json"))
    # preflight raises -> structured PRECHECK FAIL, no launch, no bare traceback
    with mock.patch.object(capmod, "preflight", side_effect=RuntimeError("boom")) as _pf, \
         mock.patch.object(capmod, "launch_and_run", return_value=0) as _launch:
        rc = capmod.capture(args)
        check(rc == 2, "preflight exception -> rc=2")
        _launch.assert_not_called()
    p = Path(str(args.out_dir) + "_preflight_fail.json")
    check(p.exists(), "preflight exception archived to _preflight_fail.json")
    if p.exists():
        data = json.loads(p.read_text())
        check("boom" in json.dumps(data), "exception reason recorded")
        p.unlink()

    # lock held -> PRECHECK FAIL (second concurrent harness refused)
    with tempfile.TemporaryDirectory() as td:
        lock = capmod.CaptureLock(Path(td) / "l.lock")
        check(lock.acquire(), "lock acquire succeeds")
        lock2 = capmod.CaptureLock(Path(td) / "l.lock")
        check(not lock2.acquire(), "second acquire fails while held")
        lock.release()
        check(lock2.acquire(), "lock re-acquirable after release")
        lock2.release()


# ---------------------------------------------------------------------------
# C. cleanup order + process-facts semantics
# ---------------------------------------------------------------------------
def test_build_process_facts_semantics():
    def sig(signal, delivered=True, result="delivered"):
        return {"signal": signal, "time_s": 1.0, "delivered": delivered, "result": result}

    def meta(signals=None, rc=0, escalated=False):
        m = {}
        for name in REQUIRED_CHILDREN:
            m[name] = {"proc": FakeProc(1), "signals": list(signals or []),
                       "wait_rc": rc, "escalated": escalated}
        return m

    # SIGINT success -> exit 0, source SIGINT, no forced
    f = build_process_facts(meta(signals=[sig("SIGINT")], rc=0), "r1", "s", "e", "scene")
    check(f["exit_code"] == 0 and f["forced_termination"] is False
          and f["shutdown_request_source"] == "SIGINT" and f["shutdown_complete"] is True,
          "SIGINT delivered -> exit 0, source SIGINT, no forced")

    # natural exit (no SIGINT) -> source UNKNOWN, not forced
    f = build_process_facts(meta(signals=[], rc=0), "r1", "s", "e", "scene")
    check(f["shutdown_request_source"] == "UNKNOWN", "natural exit -> source UNKNOWN")
    check(f["forced_termination"] is False, "natural exit -> not forced")

    # nonzero exit (SIGINT delivered, rc=1) -> exit 1, NOT forced
    f = build_process_facts(meta(signals=[sig("SIGINT")], rc=1), "r1", "s", "e", "scene")
    check(f["exit_code"] == 1 and f["forced_termination"] is False,
          "nonzero natural exit -> exit 1, NOT forced")

    # SIGINT SEND FAILED -> source != SIGINT, failure fact traceable
    f = build_process_facts(meta(signals=[sig("SIGINT", delivered=False, result="failed:ESRCH")], rc=0),
                            "r1", "s", "e", "scene")
    check(f["shutdown_request_source"] != "SIGINT", "SIGINT send failed -> source != SIGINT")
    check(any(s.get("delivered") is False for s in f["child.mujoco"]["signals"]),
          "SIGINT send failure fact preserved")

    # TERM delivered -> forced True, escalated True
    f = build_process_facts(meta(signals=[sig("SIGINT"), sig("SIGTERM")], rc=143, escalated=True),
                            "r1", "s", "e", "scene")
    check(f["forced_termination"] is True, "TERM delivered -> forced True")
    check(f["child.mujoco"]["escalated"] is True, "child escalated True")

    # TERM SEND FAILED -> forced != True
    f = build_process_facts(meta(signals=[sig("SIGINT"), sig("SIGTERM", delivered=False, result="failed:ESRCH")],
                                rc=143, escalated=True), "r1", "s", "e", "scene")
    check(f["forced_termination"] is not True, "TERM send failed -> forced != True")
    check(f["child.mujoco"]["escalated"] is False, "child escalated False on failed TERM")

    # KILL delivered -> forced True
    f = build_process_facts(meta(signals=[sig("SIGINT"), sig("SIGTERM"), sig("SIGKILL")],
                                rc=137, escalated=True), "r1", "s", "e", "scene")
    check(f["forced_termination"] is True, "KILL delivered -> forced True")

    # KILL SEND FAILED (and TERM also failed) -> forced != True
    f = build_process_facts(meta(signals=[sig("SIGINT"),
                                          sig("SIGTERM", delivered=False, result="failed:ESRCH"),
                                          sig("SIGKILL", delivered=False, result="failed:ESRCH")],
                                rc=137, escalated=True), "r1", "s", "e", "scene")
    check(f["forced_termination"] is not True, "KILL send failed -> forced != True")

    # missing wait -> exit None (not 0), incomplete
    f = build_process_facts(meta(signals=[sig("SIGINT")], rc=None), "r1", "s", "e", "scene")
    check(f["exit_code"] is None and f["shutdown_complete"] is False,
          "missing wait -> exit None (no fabricated 0), incomplete")

    # missing child (not launched) -> not_launched, exit None
    m = meta(signals=[sig("SIGINT")], rc=0)
    del m["ros2_launch"]
    f = build_process_facts(m, "r1", "s", "e", "scene")
    check(f["child.ros2_launch"]["not_launched"] is True, "missing child -> not_launched")
    check(f["exit_code"] is None, "missing child -> exit None")


def test_signal_delivery_truthfulness():
    """Inject a signal sender (mock os.killpg) to prove delivered/flags stay
    truthful — no real child, no real signal."""
    import signal as _sig

    class FakeProc3:
        pid = 42

    # success -> delivered True
    meta = {}
    with mock.patch.object(capmod.os, "killpg", return_value=None):
        ok = capmod._signal_pg(FakeProc3(), _sig.SIGINT, "mujoco", meta)
    check(ok is True, "signal send success -> delivered True")
    check(meta["mujoco"]["signals"][-1]["delivered"] is True, "delivered True recorded")

    # failure (raise OSError) -> delivered False, failure reason traceable
    meta2 = {}
    with mock.patch.object(capmod.os, "killpg", side_effect=OSError("ESRCH")):
        ok2 = capmod._signal_pg(FakeProc3(), _sig.SIGINT, "mujoco", meta2)
    check(ok2 is False, "signal send failure -> delivered False")
    check("failed" in meta2["mujoco"]["signals"][-1]["result"], "failure reason traceable")


def test_signal_exception_fail_closed():
    """Generic (non-OSError) signal exceptions are caught, recorded, and never
    misread as delivered / SIGINT source / forced."""
    import signal as _sig

    class FakeProc4:
        pid = 99

    # SIGINT sender raises RuntimeError -> delivered=False + type/message traceable
    meta = {}
    with mock.patch.object(capmod.os, "killpg", side_effect=RuntimeError("boom")):
        ok = capmod._signal_pg(FakeProc4(), _sig.SIGINT, "mujoco", meta)
    check(ok is False, "RuntimeError SIGINT -> delivered False")
    e = meta["mujoco"]["signals"][-1]
    check(e["delivered"] is False and e["exception_type"] == "RuntimeError"
          and e["exception_message"] == "boom", "exception type/message recorded")
    check(e["signal"] == "SIGINT" and e["target_pid"] == 99, "signal + target preserved")
    facts = build_process_facts(meta, "r", "s", "e", "scene")
    check(facts["shutdown_request_source"] != "SIGINT", "RuntimeError SIGINT -> source != SIGINT")

    # TERM sender raises ordinary exception -> forced != true
    meta2 = {n: {"proc": FakeProc4(),
                 "signals": [{"signal": "SIGINT", "time_s": 0, "delivered": True,
                              "result": "delivered", "exception_type": None,
                              "exception_message": None}],
                 "wait_rc": 143, "escalated": False} for n in REQUIRED_CHILDREN}
    with mock.patch.object(capmod.os, "killpg", side_effect=ValueError("nope")):
        ok2 = capmod._signal_pg(FakeProc4(), _sig.SIGTERM, "mujoco", meta2)
    check(ok2 is False, "TERM ordinary exception -> delivered False")
    facts2 = build_process_facts(meta2, "r", "s", "e", "scene")
    check(facts2["forced_termination"] is not True, "TERM exception -> forced != True")


def test_finalize_continuity_on_signal_exception():
    """One child's signal/wait exception does NOT block another child's cleanup;
    process_facts is still written and recorder.finalize is still called."""
    order = []

    class FakeRec:
        finalized = False
        def stop_sampling(self):
            order.append("stop")
        def finalize(self, facts):
            order.append("finalize")
            self.finalized = True
        def close(self):
            order.append("close")

    class Proc:
        def __init__(self, pid):
            self.pid = pid
        def poll(self):
            return None

    children = {"mujoco": Proc(1), "ros2_launch": Proc(2)}
    meta = {}
    rec = FakeRec()

    def fake_signal(p, s, n, m):
        m.setdefault(n, {"proc": p, "signals": [], "wait_rc": None, "escalated": False})
        m[n]["signals"].append({"signal": s.name, "time_s": 0, "delivered": True,
                                "result": "delivered", "exception_type": None,
                                "exception_message": None})
        order.append("sig:" + n)
        return True

    def fake_wait(p, n, m):
        if n == "mujoco":
            raise RuntimeError("boom")  # mujoco's signal/wait path raises
        m[n]["wait_rc"] = 0
        order.append("wait:" + n)
        return 0

    def fake_facts(*a, **k):
        order.append("facts")
        return {"exit_code": None, "forced_termination": False,
                "shutdown_complete": False, "shutdown_request_source": "UNKNOWN"}

    with mock.patch.object(capmod, "_signal_pg", side_effect=fake_signal), \
         mock.patch.object(capmod, "_wait_or_escalate", side_effect=fake_wait), \
         mock.patch.object(capmod, "wait_pid", side_effect=lambda p, t: 0), \
         mock.patch.object(capmod, "build_process_facts", side_effect=fake_facts):
        out = Path(tempfile.mkdtemp())
        facts = capmod._finalize_capture(children, meta, rec, "r1", "s", "scene", None, out)
    check("finalize" in order, "finalize called despite signal exception")
    check((out / "process_facts.json").exists(), "process_facts written despite exception")
    check(rec.finalized, "recorder finalized despite exception")
    check("sig:mujoco" in order and "sig:ros2_launch" in order,
          "both children signal-attempted (one exception did not block the other)")
    check(facts["shutdown_complete"] is False and facts["exit_code"] is None,
          "run_record facts do NOT fabricate normal shutdown on failure")


def test_cleanup_errors_persisted_in_real_facts():
    """signal/wait exception -> structured cleanup_errors appear in the REAL
    process_facts.json per-child facts (not mocked build_process_facts)."""
    import signal as _sig

    class Rec:
        finalized = False
        def stop_sampling(self):
            pass
        def finalize(self, facts):
            self.finalized = True
        def close(self):
            pass

    class PollRunning:
        pid = 1
        def poll(self):
            return None  # running

    class PollExited:
        pid = 2
        def poll(self):
            return 0  # already exited

    children = {"mujoco": PollRunning(), "ros2_launch": PollExited()}
    meta = {}
    rec = Rec()

    def fake_wait_or_escalate(p, n, m):
        raise RuntimeError("wait boom")

    def fake_wait_pid(p, t):
        return None  # cannot confirm exit

    with mock.patch.object(capmod.os, "killpg", return_value=None), \
         mock.patch.object(capmod, "_wait_or_escalate", side_effect=fake_wait_or_escalate), \
         mock.patch.object(capmod, "wait_pid", side_effect=fake_wait_pid):
        out = Path(tempfile.mkdtemp())
        capmod._finalize_capture(children, meta, rec, "r1", "s", "scene", None, out)

    # READ the real persisted process_facts.json (build_process_facts NOT mocked)
    pf = json.loads((out / "process_facts.json").read_text())
    mj = pf["child.mujoco"]
    check(any(e.get("stage") == "signal_or_wait" and e.get("exception_type") == "RuntimeError"
              and "wait boom" in e.get("exception_message", "")
              and "time_s" in e for e in mj["cleanup_errors"]),
          "cleanup_errors persisted with stage/type/message/time")
    check(any(a.get("stage") == "poll" and a.get("result") == "running"
              for a in mj["poll_attempts"]), "poll attempt persisted")
    check(mj["exit_code"] is None, "mujoco unconfirmed -> exit_code None (not 0)")
    check(pf["child.ros2_launch"]["exit_code"] == 0,
          "other child (already exited) cleaned up despite mujoco failure")
    check(rec.finalized and (out / "process_facts.json").exists(),
          "recorder finalized + process_facts written")


def test_poll_exception_no_fabricated_facts():
    """poll() raises RuntimeError -> still records poll/signal/wait attempts, and
    the REAL persisted facts never fabricate exit_code=0 / shutdown_complete=true
    / SIGINT / forced=true."""
    import signal as _sig

    class Rec:
        finalized = False
        def stop_sampling(self):
            pass
        def finalize(self, facts):
            self.finalized = True
        def close(self):
            pass

    class PollRaises:
        pid = 1
        def poll(self):
            raise RuntimeError("poll boom")

    class PollExited:
        pid = 2
        def poll(self):
            return 0

    children = {"mujoco": PollRaises(), "ros2_launch": PollExited()}
    meta = {}
    rec = Rec()

    def killpg_fail(*a, **k):
        raise OSError("ESRCH")  # every signal send fails -> nothing delivered

    def wait_unconfirmable(p, t):
        return None  # cannot confirm exit

    with mock.patch.object(capmod.os, "killpg", side_effect=killpg_fail), \
         mock.patch.object(capmod, "wait_pid", side_effect=wait_unconfirmable):
        out = Path(tempfile.mkdtemp())
        facts = capmod._finalize_capture(children, meta, rec, "r1", "s", "scene", None, out)

    pf = json.loads((out / "process_facts.json").read_text())
    mj = pf["child.mujoco"]
    # poll exception recorded
    check(any(a.get("result") == "exception" and a.get("exception_type") == "RuntimeError"
              and "poll boom" in a.get("exception_message", "") for a in mj["poll_attempts"]),
          "poll exception recorded with type/message")
    # signal + wait attempts still happened after the poll exception (SIGINT/TERM/KILL
    # all attempted, none delivered because killpg fails)
    sigs = mj["signals"]
    check([s.get("signal") for s in sigs] == ["SIGINT", "SIGTERM", "SIGKILL"],
          "signal attempts (SIGINT/TERM/KILL) recorded after poll exception")
    check(all(s.get("delivered") is False for s in sigs), "no signal delivered (killpg failed)")
    # wait rc UNKNOWN, not fabricated
    check(mj["exit_code"] is None, "mujoco wait rc UNKNOWN (not fabricated 0)")
    check(mj["escalated"] is False, "mujoco not escalated (TERM/KILL not delivered)")
    # other child cleaned up despite the poll exception
    check(pf["child.ros2_launch"]["exit_code"] == 0,
          "other child cleaned despite one child poll exception")
    # top-level facts do NOT fabricate success
    check(facts["exit_code"] is None and facts["shutdown_complete"] is False,
          "top-level does not fabricate exit_code=0 / shutdown_complete=true")
    check(facts["shutdown_request_source"] != "SIGINT", "top-level does not fabricate SIGINT source")
    check(facts["forced_termination"] is not True, "top-level does not fabricate forced=true")
    check(rec.finalized and (out / "process_facts.json").exists(),
          "recorder finalized + process_facts written despite poll exception")


def test_facts_match_recorder_top_level():
    """The final process facts passed to finalize() carry the top-level fields
    the recorder reads (exit_code/forced/source/complete)."""
    facts = build_process_facts(
        {n: {"proc": FakeProc(1), "signals": [{"signal": "SIGINT", "time_s": 1.0,
                                               "delivered": True, "result": "delivered"}],
             "wait_rc": 0, "escalated": False} for n in REQUIRED_CHILDREN},
        "r1", "s", "e", "scene")
    check(facts["exit_code"] == 0 and facts["forced_termination"] is False
          and facts["shutdown_request_source"] == "SIGINT" and facts["shutdown_complete"] is True,
          "top-level facts carry recorder-read fields")


def test_finalize_order_stop_before_signal():
    # stop_sampling happens before any signal; facts written before finalize.
    order = []

    class FakeRec:
        finalized = False
        def stop_sampling(self):
            order.append("stop")
        def finalize(self, facts):
            order.append("finalize")
            self.finalized = True
        def close(self):
            order.append("close")

    class FakeProc2:
        pid = 1
        _rc = None
        def poll(self):
            return self._rc

    children = {"mujoco": FakeProc2(), "ros2_launch": FakeProc2()}
    meta = {}
    recorder = FakeRec()

    def fake_signal(p, s, n, m):
        m.setdefault(n, {"proc": p, "signals": [], "wait_rc": None, "escalated": False})
        m[n]["signals"].append({"signal": s.name, "time_s": 0, "result": "sent"})
        order.append("sig:" + n)
        return True

    def fake_wait(p, n, m):
        m[n]["wait_rc"] = 0
        order.append("wait:" + n)
        return 0

    def fake_facts(*a, **k):
        order.append("facts")
        return {"exit_code": 0, "forced_termination": False,
                "shutdown_complete": True, "shutdown_request_source": "SIGINT"}

    with mock.patch.object(capmod, "_signal_pg", side_effect=fake_signal), \
         mock.patch.object(capmod, "_wait_or_escalate", side_effect=fake_wait), \
         mock.patch.object(capmod, "build_process_facts", side_effect=fake_facts):
        out = Path(tempfile.mkdtemp())
        capmod._finalize_capture(children, meta, recorder, "r1", "s", "scene", None, out)
    # stop must come before sig; facts before finalize
    stop_idx = order.index("stop")
    sig_idx = min(i for i, x in enumerate(order) if x.startswith("sig:"))
    facts_idx = order.index("facts")
    fin_idx = order.index("finalize")
    check(stop_idx < sig_idx, "stop_sampling before signal")
    check(facts_idx < fin_idx, "facts before finalize")
    check(recorder.finalized, "recorder finalized")


def test_record_fail_closed_and_top_level_facts():
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        facts = {"exit_code": 0, "forced_termination": False,
                 "shutdown_request_source": "SIGINT", "shutdown_complete": True}
        # LIVE only -> VALID
        rec = RunRecordRecorder(str(td / "live.jsonl"))
        rec.start()
        rec.record_snapshot(make_frame(seq=2, rl_step=1))
        rec.record_snapshot(make_frame(seq=4, rl_step=2))
        rec.stop_sampling()
        rec.finalize(facts)
        rec.close()
        check(summarize_record(str(td / "live.jsonl"))["record_validity"] == "VALID", "LIVE-only VALID")

        # + bad magic -> INVALID
        rec = RunRecordRecorder(str(td / "bad.jsonl"))
        rec.start()
        rec.record_snapshot(make_frame(seq=2, rl_step=1))
        rec.record_snapshot(make_frame(magic=0, seq=4, rl_step=2))
        rec.stop_sampling()
        rec.finalize(facts)
        rec.close()
        check(summarize_record(str(td / "bad.jsonl"))["record_validity"] == "INVALID",
              "present bad frame -> INVALID")

        # + non-authoritative -> INVALID
        rec = RunRecordRecorder(str(td / "na.jsonl"))
        rec.start()
        rec.record_snapshot(make_frame(seq=2, rl_step=1))
        rec.record_snapshot(make_frame(seq=4, source=0, rl_step=2))
        rec.stop_sampling()
        rec.finalize(facts)
        rec.close()
        check(summarize_record(str(td / "na.jsonl"))["record_validity"] == "INVALID",
              "non-authoritative -> INVALID")

        # top-level facts read by finalize
        rec = RunRecordRecorder(str(td / "f.jsonl"))
        rec.start()
        rec.record_snapshot(make_frame(seq=2, rl_step=1))
        rec.stop_sampling()
        rec.finalize(facts)
        rec.close()
        term = [json.loads(l) for l in (td / "f.jsonl").read_text().splitlines()][-1]
        check(term["process_exit_code"] == 0 and term["shutdown_request_source"] == "SIGINT"
              and term["shutdown_complete"] is True and term["normal_shutdown"] is True,
              "top-level facts read by RunRecordRecorder")

        # missing exit_code -> UNKNOWN (None), never fabricated 0
        rec = RunRecordRecorder(str(td / "g.jsonl"))
        rec.start()
        rec.record_snapshot(make_frame(seq=2, rl_step=1))
        rec.stop_sampling()
        rec.finalize({"forced_termination": False, "shutdown_request_source": "SIGINT", "shutdown_complete": True})
        rec.close()
        term = [json.loads(l) for l in (td / "g.jsonl").read_text().splitlines()][-1]
        check(term["process_exit_code"] is None, "missing exit_code -> UNKNOWN (None)")
        check(term["normal_shutdown"] is not True, "missing exit_code -> normal_shutdown not True")


def main() -> int:
    test_stride2_gap_math()
    test_closure_escape_cycle_missing_mutation()
    test_manifest_closure_positive()
    test_ldd()
    test_residual_process_identity_fail_closed()
    test_binary_scene_mismatch()
    test_preflight_exception_and_lock()
    test_build_process_facts_semantics()
    test_signal_delivery_truthfulness()
    test_signal_exception_fail_closed()
    test_finalize_continuity_on_signal_exception()
    test_facts_match_recorder_top_level()
    test_cleanup_errors_persisted_in_real_facts()
    test_poll_exception_no_fabricated_facts()
    test_finalize_order_stop_before_signal()
    test_record_fail_closed_and_top_level_facts()
    if g_fail:
        print(f"RESULT: FAIL ({g_checks} checks)")
        return 1
    print(f"RESULT: PASS ({g_checks} checks)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
