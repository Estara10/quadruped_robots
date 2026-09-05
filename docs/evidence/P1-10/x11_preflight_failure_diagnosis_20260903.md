# P1-10 — X11 Preflight Failure Minimal Diagnosis

Date: 2026-09-03  
Scope: diagnosis of the X11 preflight failure for
`P1-10-REPLAY-20260903-flat_goal_forward-stabilized` only.

## Classification

**ENVIRONMENT BLOCKED** — the declared graphical display is not usable from
the current harness/session namespace. This is not a new pair, replay retry,
or runtime evidence.

## Exact evidence

The Run A preflight archived `xdpyinfo rc=1` after the residual-process check
passed. The exact current environment and the exact `child_env()` environment
both contain:

```text
DISPLAY=:0
XAUTHORITY=/run/user/1000/gdm/Xauthority
```

The values are inherited from the harness invocation environment. The
harness `child_env()` copies both values unchanged. It only changes
`LD_LIBRARY_PATH` by prepending:

```text
/home/lidio/Libraries/unitree_sdk2/lib:/home/lidio/Libraries/libtorch-cpu-2.0.1/lib:
```

Running `xdpyinfo` with the exact child environment produced:

```text
returncode: 1
stdout: ""
stderr: 'xdpyinfo:  unable to open display ":0".\n'
```

The same result occurred in the parent/current environment. Therefore the
child-library-path construction does not explain the failure.

The declared X11 artifacts were:

| Check | Result |
|---|---|
| `/tmp/.X11-unix/X0` | exists, mode `0777` |
| Unix socket connect | `connect_ex=1` |
| `/run/user/1000/gdm/Xauthority` | exists, readable, 100 bytes |
| visible X server process | none in the current process namespace |
| `xdpyinfo` under child env | rc `1`, unable to open `:0` |

The socket node's existence is not proof of a live reachable X server. The
lower-level reason for the failed connection—stale listener, authorization,
namespace isolation, or a server outside this namespace—remains UNKNOWN.

## Harness assessment

The X11 preflight implementation calls `run_cmd(["xdpyinfo"])` without an
explicit `env` argument, while MuJoCo and ROS children use `child_env()`.
This is a latent exact-environment consistency weakness, but it is not causal
for this failure: `DISPLAY` and `XAUTHORITY` are byte-for-byte identical in
both environments and the direct child-environment probe fails identically.
No runtime code or harness change is justified by this diagnosis.

## Recovery prerequisite

Before any separately authorized future pair, an operator must provide a live,
reachable graphical X11 session in the same execution namespace and verify,
using the exact harness child environment, that:

```text
xdpyinfo
returncode=0
```

for the declared `DISPLAY` and matching readable `XAUTHORITY`. The X11 socket
must be a connected server endpoint, not merely an existing socket node. No
headless bypass or simulator launch-semantics change is authorized by this
diagnosis; a new Director authorization remains required.

## Boundaries

- No MuJoCo or ROS2 process was started.
- No new pair was created and neither failed pair was retried.
- No benchmark, pilot, FormalRun, multi-seed evaluation, or P1-11/P1-12/P1-13
  task was started.
- P1-10 remains `IMPLEMENTED / AWAITING INDEPENDENT REVIEW`.
