# P1-09AE — MuJoCo-only controlled clean-shutdown run

Status: **RUNTIME PASS, REVIEW REQUIRED**. This is one bounded simulation-only
run, not a benchmark, formal experiment, pilot, or acceptance claim.

## What ran

On 2026-08-29 in the graphical X11 environment, the process was launched once:

```text
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
  ./build2/unitree_mujoco -s scene_flat.xml
```

No ROS2, controller, FormalRunWriter, benchmark, pilot, or real robot process
was started. The scene was `scene_flat.xml`; the simulator reported interface
`lo` and MuJoCo 3.3.3.

## Controlled result

- PID `103750`, PGID `103743` were recorded before the signal.
- Before delivery, `SigIgn=0x4`; SIGINT's bit is not set. `SigCgt=0x4002`
  includes SIGINT's bit.
- At `2026-08-29T17:27:34+08:00`, `kill -INT 103750` returned `0`.
- The supervisor waited up to 10 seconds. It did **not** log a timeout or send
  TERM/KILL.
- `wait` returned `MUJOCO_RC=0`.
- A post-run check found PID `103750` absent.

Therefore this run proves that the launched MuJoCo process accepted the
supervisor's SIGINT delivery and exited normally within the bounded wait window.

## Evidence integrity

| File | SHA-256 | Bytes / lines |
| --- | --- | --- |
| `P1-09AE_orchestrator_raw.log` | `0174bbf23b3260c873df69dc4d176af85b6b1fb5c0d73c56e0f5028ca0754f22` | 224 / 8 |
| `P1-09AE_mujoco_raw.log` | `3ad3fd2d473168dca943258b6a4e3ce403b5c8e973c2d0dbc223bb8712736c8a` | 3851 / 101 |

## Boundaries and remaining unknowns

The current raw simulator log has startup lines only: it does not emit explicit
bridge request-stop/join, physics join, `mjModel`/`mjData` release, or DDS
teardown events. This run therefore does not independently prove those internal
ordering facts, does not validate runtime reload behavior, and does not accept
P1-09, P1-02 runtime integration, or Phase 1.
