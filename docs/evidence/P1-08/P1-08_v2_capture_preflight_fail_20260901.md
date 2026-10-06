# P1-08 — v2 Recapture Run FAILED AT PREFLIGHT (2026-09-01, single run, no retry)

Status: **P1-08 — BLOCKED / FAILED FOR THIS RUN (v2 recapture preflight FAIL)**.
The single Director-authorized v2 recapture run was consumed by a **preflight
failure** and was **not retried** (task rule). Phase 1 remains **NOT ACCEPTED**.

## What happened

On 2026-09-01 20:03:16 the authorized one-shot v2 capture
(`p1_08_baseline_capture.py --out-dir docs/evidence/P1-08/capture_20260901_v2
--window-s 25 --manifest <v2 manifest>`) aborted at preflight **before any
launch**:

```
[20:03:16] == preflight ==
[20:03:16] preflight: X11 reachable (DISPLAY=:0)
[20:03:16] FAIL preflight: hardware plugin unresolved: ['\tlibddsc.so.0 => not found', '\tlibddscxx.so.0 => not found']
[20:03:16] PRECHECK FAIL — not launching
```

Preserved terminal output: `capture_20260901_v2_preflight_fail_terminal.log`.

## Exact root cause (measurement-script defect)

The orchestrator's preflight `ldd` check ran with the **current process env**
(which did not include `/home/lidio/Libraries/unitree_sdk2/lib`), so
`libddsc.so.0` / `libddscxx.so.0` appeared unresolved. The **actual child
environment** (unitree_sdk2/lib + libtorch/lib + inherited ROS) **does** resolve
them — verified independently:

- `LD_LIBRARY_PATH=unitree_sdk2/lib:libtorch/lib:<ROS>` + `ldd
  libhardware_unitree_mujoco.so` → `libddsc.so.0`/`libddscxx.so.0` resolved,
  **no `not found` entries**.
- After the fix, the same check run through the orchestrator's `child_env()`
  reports `libddsc resolved: True`, `not-found entries: NONE`.

The defect was in the P1-08 measurement script's preflight (it checked the wrong
environment), not in the environment, stack, hashes, or the v2 sim-clock
contract. The orchestrator's ldd preflight was corrected to run under
`child_env()`; **this is a script fix only, not a re-run**.

## Boundary facts

- **Nothing was launched**: no MuJoCo, no ROS, no controller; `capture_20260901_v2`
  out-dir was **not created** (preflight aborts before launch); no residual
  processes; no `/dev/shm/mujoco_sim_clock`.
- Per the task rule ("preflight failure → preserve raw evidence, write failure
  evidence, status P1-08 BLOCKED/FAILED FOR THIS RUN, no accepted baseline, **no
  retry**") this run is **FAILED FOR THIS RUN**.
- **No v2 capture data exists**; no accepted baseline produced.
- The old v1 capture (`capture_20260901_rerun`) and identities `bdd47a0d…` /
  `99b995b0…` remain **superseded / non-acceptance**.
- A new v2 recapture requires **separate Director re-authorization**; the
  corrected orchestrator preflight (child-env ldd check) passes all hash/env
  checks so the next authorized run can proceed.
- No MuJoCo/ROS2 runtime was run; no P1-10; no timestep/control/policy/solver/
  model/scene/gains change; no commit/push.

Phase 1 remains **NOT ACCEPTED**.
