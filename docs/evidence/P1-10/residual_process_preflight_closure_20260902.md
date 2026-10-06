# P1-10 Residual-Process Preflight False-Positive Closure

Date: 2026-09-02

Status: **IMPLEMENTED / AWAITING INDEPENDENT REVIEW**. This is an offline
harness correction only. No MuJoCo/ROS2 replay was run, the failed pair was
not retried, Run B was not started, and no P1-11/P1-12/P1-13 work was started.

## Root cause

The failed authorized pair used:

```text
pgrep -af "unitree_mujoco|ros2 launch|ros2_control_node"
```

This searched the complete command-line text. The capture invocation itself
contained the MuJoCo executable path, so the active Codex sandbox command was
reported as a residual runtime process. The archived failure is
`docs/evidence/P1-10/replay_pair_20260902/run_A_preflight_fail.json`, SHA-256
`8e131fce2b1d25a598caec0b412713ee39e4ef67876c5c620d963211844bde0f`.

## Detection rule

Before: a broad substring match over all command lines; no executable identity,
capture attribution, or self/ancestor exclusion.

After: `scripts/p1_08_baseline_capture.py` reads a complete `/proc` snapshot
of executable, argv, state, and PPID identity. It excludes the harness PID and
the entire ancestor chain before classification. A process is a residual only
if it is non-zombie and:

- `/proc/<pid>/exe` exactly matches the expected MuJoCo executable, including a
  deleted-instance suffix; or
- the executable is the ROS launcher runtime and argv contains the exact
  `ros2 launch rl_quadruped_controller mujoco.launch.py` sequence; or
- the executable is `ros2_control_node` and argv contains the Go2
  `go2_description/config/robot_control.yaml` capture configuration.

A shell or unrelated process whose argv merely mentions the simulator path is
not a match because its executable identity is inspected. An unreadable,
malformed, missing, or ambiguous process identity returns `uncertain`; the
preflight rejects every state other than `none`.

## Offline proof

Focused injected process-table tests cover:

- exact MuJoCo residual: rejected;
- attributable controller residual: rejected;
- attributable ROS launch residual: rejected;
- shell/path-only mention: not rejected;
- self/ancestor runtime-looking entries: excluded;
- process-inspection exception: `uncertain`, rejected;
- controller without capture config attribution: `uncertain`, rejected;
- unrelated processes: no residual accepted.

Results:

- `python3 scripts/test_p1_08_harness.py`: **PASS (93 checks)**;
- `python3 -m py_compile scripts/p1_08_baseline_capture.py scripts/test_p1_08_harness.py`: **PASS**;
- `git diff --check`: **PASS**;
- direct offline identity scan of the current process table: `none`;
- no MuJoCo/ROS2 process was started by this task.

Machine-readable evidence: `residual_process_preflight_closure_20260902.json`.

The prior pair remains terminally `FAILED FOR THIS PAIR`. This correction must
pass independent Reviewer approval before a new Director authorization can be
requested for any future replay.
