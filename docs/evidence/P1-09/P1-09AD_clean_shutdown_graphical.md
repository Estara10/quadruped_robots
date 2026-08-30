# P1-09AD — Graphical MuJoCo-only Clean-Shutdown Validation

## Result

**PARTIAL PASS**. In the user-confirmed graphical session (`DISPLAY=:0`), the
X11 preflight succeeded and MuJoCo 3.3.3 started successfully; after `Ctrl+C`
the process exited `rc=0` and no TERM/KILL escalation was used. This is the
first post-P1-09W MuJoCo-only run to reach a clean `rc=0` exit. The archived
raw log proves startup/initialization only; it does **not** contain explicit
bridge stop/join, physics join, or m/d release output, so the internal
shutdown ordering that DEC-008/DEC-009 require remains unproven. This is a
**PARTIAL PASS** — not a full clean-shutdown proof, not P1-09 Acceptance, and
not Phase 1 Acceptance.

## Boundary

- Simulator only; no ROS2, controller-manager, StateRL/StateRLRec, HUD,
  benchmark, FormalRunWriter, pilot, or real robot.
- Loopback/simulation interface `"lo"` (non-multicast), matching the
  simulation-only boundary used in prior runs.
- No source/configuration changes were made for this run.
- Archive-only task: the source artifacts in `/tmp/p1_09ad/` were copied
  byte-identical into this directory; nothing was re-run.

## Prior attempt — preflight BLOCKED

The first P1-09AD attempt did not start MuJoCo because the mandatory X11
preflight in that execution context failed:

```text
DISPLAY=:0
XAUTHORITY=/run/user/1000/gdm/Xauthority
xdpyinfo:  unable to open display ":0".
XDPYINFO_RC=1
```

Raw: [`P1-09AD_preflight_raw.log`](P1-09AD_preflight_raw.log).

## Current run evidence

Run facts:

| Item | Observed | Evidence source |
|---|---|---|
| Session | Graphical terminal, `DISPLAY=:0` | operator account |
| Display preflight (`xdpyinfo`) | succeeded | operator account |
| Interface | `"lo"` (loopback, non-multicast) | `P1-09AD_mujoco_raw.log` line 1 |
| MuJoCo version | `3.3.3` | `P1-09AD_mujoco_raw.log` line 2 |
| Model compile | link / joint / actuator / sensor tables printed | `P1-09AD_mujoco_raw.log` |
| Shared memory | `/mujoco_ray2d`, `/mujoco_qpos`, `/mujoco_collision` initialized | `P1-09AD_mujoco_raw.log` |
| Ray source | `geometric` | `P1-09AD_mujoco_raw.log` |
| Terminate method | `Ctrl+C` (SIGINT) | operator account |
| MuJoCo exit code | `0` | `P1-09AD_exit.txt` (`MUJOCO_RC=0`) |
| TERM/KILL escalation | none | operator account |

First logged line timestamp: `1787988099.086377` (unix seconds ≈
`2026-08-29T15:21:39+08:00`).

## Required lifecycle evidence

| Evidence | Result |
|---|---|
| Bridge/SDK startup | present in log (shm + ray2d init) |
| Bridge stop/join | UNKNOWN — no log output |
| Physics join | UNKNOWN — no log output |
| m/d release | UNKNOWN — no log output |
| normal `main` return | exit `rc=0` recorded; internal return path not proven |
| TERM/KILL escalation | none |
| terminate / SIGABRT / abort / core dump | no such text in captured output |

The archived log contains no signal-handler, stop/join, or release output; the
last line is `[Ray2D] Source: geometric`. `exit.txt` records only
`MUJOCO_RC=0`. Therefore the observed clean `rc=0` exit is a **PARTIAL PASS**:
startup and the final exit code are evidenced, but the stop-and-join ordering
required by DEC-008/DEC-009 is not demonstrated by this log.

## Acceptance

This task is a lifecycle startup + clean-exit observation only. It is not a
benchmark, formal experiment, P1-09 Acceptance, or Phase 1 Acceptance. P1-09
remains **EXECUTING / NOT ACCEPTED** and Phase 1 remains **NOT ACCEPTED**.

## Raw evidence

- [`P1-09AD_mujoco_raw.log`](P1-09AD_mujoco_raw.log) — application output of
  the successful run (SHA256 `7940668b9832fd3f7bddf8ddeceb160a3b7af016491ef4c8c6b27b03dc88a805`)
- [`P1-09AD_exit.txt`](P1-09AD_exit.txt) — `MUJOCO_RC=0`
- [`P1-09AD_preflight_raw.log`](P1-09AD_preflight_raw.log) — earlier blocked preflight

## Remaining UNKNOWN

- Runtime bridge stop/join, physics join, and m/d release ordering after
  SIGINT (not present in this log).
- Whether the same `rc=0` result is reproducible under an explicit signal
  timeline and process-group capture (this run was interactive in a graphical
  terminal; no orchestrator sidecar recorded PID/PGID or the signal/wait
  timeline).
- DDS/external-thread teardown and the other existing P1-09 UNKNOWN items.
