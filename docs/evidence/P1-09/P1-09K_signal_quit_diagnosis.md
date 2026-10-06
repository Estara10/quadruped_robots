# P1-09K — MuJoCo Signal / Quit Runtime Diagnosis

Date: 2026-08-28  
Status: **PARTIAL — SIGINT disposition and result confirmed; UI-close path
trigger was unavailable and remains UNKNOWN.**

## Boundary

Two independent MuJoCo-only instances were used. Neither instance started
ROS2, controller-manager, RL, StateRL/StateRLRec, HUD, benchmark,
`FormalRunWriter`, or a real robot. Both used `scene_flat.xml`, the existing
MuJoCo executable, and loopback configuration. No source, configuration, or
launch script was modified.

An initial temporary harness attempt was invalid because of a log-path/child
ownership bug and did not start MuJoCo. It is not included as evidence. The
corrected run below is the only runtime evidence for each path, and no path was
repeated.

## Raw evidence

- [`P1-09K_diagnosis_raw.log`](P1-09K_diagnosis_raw.log)
- [`P1-09K_sigint_mujoco_raw.log`](P1-09K_sigint_mujoco_raw.log)
- [`P1-09K_ui_mujoco_raw.log`](P1-09K_ui_mujoco_raw.log)

## SIGINT disposition and result

Process metadata:

| Field | Value |
|---|---|
| PID | `128079` |
| PPID | `128074` |
| PGID / SID | `128079 / 128079` |
| Command | `/home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco -s scene_flat.xml` |
| `SigBlk` | `0000000000000000` |
| `SigIgn` | `0000000000000006` |
| `SigCgt` | `0000000100000000` |

On Linux, the low bit mask `0x2` corresponds to signal number 2, SIGINT; the
captured `SigIgn=0x6` therefore confirms that SIGINT was ignored by the running
MuJoCo process. This is runtime evidence, not an inference from source.

Timeline:

| Event | Timestamp |
|---|---|
| process start | `2026-08-28T11:35:14.171Z` |
| disposition snapshot | `2026-08-28T11:35:17.177Z` |
| SIGINT sent to PGID `128079` | `2026-08-28T11:35:17.230Z` |
| SIGINT wait timeout | `2026-08-28T11:35:22.446Z` (`5 s` bound) |
| post-timeout disposition snapshot | `2026-08-28T11:35:22.451Z` |
| SIGTERM cleanup sent | `2026-08-28T11:35:22.461Z` |
| final wait result | `2026-08-28T11:35:22.575Z`, `rc=143` |

Conclusion: SIGINT disposition is **CONFIRMED: ignored**; SIGINT self-exit is
**FAIL**; `rc=143` is a SIGTERM cleanup result and is not clean shutdown.
The static-audit hypothesis that the ignored disposition was inherited from an
asynchronous shell launch is **LIKELY**, but the exact origin of the disposition
is not proven by this test.

## UI Quit / window-close result

The independent second instance started successfully:

| Field | Value |
|---|---|
| PID | `128319` |
| PPID | `128074` |
| PGID / SID | `128319 / 128319` |
| `SigBlk` | `0000000000000000` |
| `SigIgn` | `0000000000000006` |
| `SigCgt` | `0000000100000000` |
| trigger requested | `WM_DELETE_WINDOW` at `2026-08-28T11:35:25.678Z` |
| trigger result | helper returned `UI_CLOSE_RESULT=NO_MUJOCO_WINDOW`, rc `2` |
| cleanup | no window event was delivered; after 5 s, SIGTERM cleanup at `11:35:30.962Z` |
| final wait result | `11:35:31.076Z`, `rc=143` |

Because the X11 helper did not identify a MuJoCo window, the existing
`WM_DELETE_WINDOW` / GLFW close path was not actually exercised. Consequently:

- UI Quit/window-close trigger: **UNKNOWN / NOT EXECUTED**;
- process exit after UI close: **UNKNOWN**;
- the `rc=143` result must not be attributed to UI close.

Static source evidence still confirms that the internal UI Quit action stores
`exitrequest=1` at `simulate.cc:1466-1468`, and the render loop checks that flag
at `simulate.cc:2791`. It does not prove that the whole process exits because
the bridge thread remains unbounded and unjoined.

## Main-thread versus whole-process exit

| Question | Result |
|---|---|
| Does the MuJoCo process ignore SIGINT in the tested launch context? | **CONFIRMED** |
| Does SIGINT cause the child to exit? | **FAIL** — timeout, then TERM cleanup |
| Was UI Quit/window-close actually delivered? | **UNKNOWN** — no window target found |
| Is a process-wide orderly shutdown proven? | **FAIL / NOT PROVEN** |
| Does source contain an unbounded bridge thread not joined by main? | **CONFIRMED** — `main.cc:535-568`, `628`, `634-636` |
| Does source provide a signal-to-`exitrequest` bridge? | **CONFIRMED: NO** |

## Minimal follow-up recommendation (not implemented)

The next code change should remain lifecycle-only:

1. establish an explicit SIGINT/SIGTERM stop request using signal-safe state;
2. make the Unitree bridge wait and loop observe that stop request;
3. remove worker-thread `exit(0)` and return to `main` for ordered joins;
4. join the bridge thread before returning from `main`.

Before implementation, one separate UI-only diagnostic may be considered to
resolve the X11 target issue. It must record the actual window ID and prove that
the `exitrequest` path causes the whole process to exit; it must not treat TERM
cleanup as success.

P1-09 and Phase 1 remain unaccepted.
