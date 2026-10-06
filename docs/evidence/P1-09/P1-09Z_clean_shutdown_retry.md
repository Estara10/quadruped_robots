# P1-09Z — XAUTHORITY MuJoCo-only Clean-Shutdown Retry

## Result

**BLOCKED** at the mandatory display preflight. The authorized retry did not
start MuJoCo because the required X11 connection test returned non-zero. No
second preflight and no MuJoCo launch were performed.

## Stage A preflight

Environment was restricted exactly as authorized:

```text
DISPLAY=:0
XAUTHORITY=/run/user/1000/gdm/Xauthority
```

Command:

```bash
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority xdpyinfo
```

Observed result:

```text
xdpyinfo:  unable to open display ":0".
XDPYINFO_RC=1
```

Classification: **BLOCKED / CONFIRMED immediate preflight failure**.

## Stage B

Not executed. Therefore there is no MuJoCo PID/PGID, `/proc` signal status,
bridge/physics startup evidence, SIGINT timeline, wait result, or MuJoCo exit
code for P1-09Z. No TERM/KILL was sent, and no runtime process was started.

## Acceptance status

P1-09Z does not provide clean-shutdown evidence and is not a formal experiment,
benchmark, P1-09 Acceptance, or Phase 1 Acceptance. P1-09 remains
**EXECUTING / NOT ACCEPTED** and Phase 1 remains **NOT ACCEPTED**.

## Raw preflight

The complete preflight output is preserved in
[`P1-09Z_preflight_raw.log`](P1-09Z_preflight_raw.log).

## Remaining UNKNOWN

- Whether the post-P1-09W MuJoCo child can self-exit rc=0 after SIGINT in a
  reachable X11 environment.
- Bridge stop/join, physics join, m/d release, and normal-main-return runtime
  evidence.
- Exact lower-level cause of the unreachable display despite the Xauthority
  path (authorization, stale socket, namespace, or display-server state).
