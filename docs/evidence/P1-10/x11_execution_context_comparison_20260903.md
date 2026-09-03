# P1-10 — X11 Execution-Context Readiness Comparison

Date: 2026-09-03  
Scope: read-only comparison of the operator-supplied known-good graphical
terminal context and the exact current Execution/harness context.

No pair was created and no MuJoCo/ROS2 process was started.

## Sources

Known-good context: the operator-confirmed graphical-terminal evidence in
`P1-09AE_record_capture_20260830.md` and its raw log, where
`DISPLAY=:0`, `XAUTHORITY=/run/user/1000/gdm/Xauthority`, and `xdpyinfo rc=0`
were recorded before a real graphical run.

Execution context: the current process environment and `child_env()` from
`scripts/p1_08_baseline_capture.py`, with read-only checks performed on
2026-09-03.

## Comparison

| Field | Known-good graphical terminal | Current Execution/harness |
|---|---|---|
| `DISPLAY` | `:0` | `:0` |
| `XAUTHORITY` | `/run/user/1000/gdm/Xauthority` | `/run/user/1000/gdm/Xauthority` |
| uid/gid | **UNKNOWN** — not captured in supplied evidence | uid `1000` (`lidio`), gid `1000` (`lidio`) |
| `xdpyinfo` | rc `0` in operator evidence; stderr not archived | rc `1`; stdout empty; stderr `xdpyinfo:  unable to open display ":0".\n` |
| `/proc/self/ns/pid` | **UNKNOWN** — not captured | `pid:[4026533154]` |
| `/proc/self/ns/mnt` | **UNKNOWN** — not captured | `mnt:[4026533152]` |
| `/proc/self/ns/user` | **UNKNOWN** — not captured | `user:[4026533445]` |
| `/tmp/.X11-unix/X0` | existence/metadata not captured; functional X11 access was demonstrated by `xdpyinfo rc=0` | exists; uid `1000`, gid `1000`, mode `0777`, size `0`; `connect_ex=1` (not reachable) |
| Xauthority file read | direct read not separately captured; functional authorization succeeded with `xdpyinfo rc=0` | exists; uid `1000`, gid `1000`, mode `0700`, size `100`; readable and open/read succeeds |

The current `child_env()` comparison shows `DISPLAY` and `XAUTHORITY` are
unchanged from the current session. The only changed environment key is
`LD_LIBRARY_PATH`, which is prepended with the required Unitree SDK2 and
LibTorch directories for runtime dependencies. The exact child-environment
`xdpyinfo` result is the same failure as the current environment.

## Classification

The immediate condition is **ENVIRONMENT BLOCKED**: the current Execution
context cannot connect to the declared display even though the Xauthority file
is readable and the X0 socket node exists.

The evidence rules out a DISPLAY/XAUTHORITY value-construction mismatch as the
cause. It does not distinguish among namespace isolation, a stale/unserved
socket, or X11 authorization/access policy, because the known-good context did
not archive its namespace links or uid/gid and the current socket failure is
reported only as `connect_ex=1` / `xdpyinfo rc=1`. Those lower-level causes
remain **UNKNOWN**.

The harness has a latent consistency weakness: its X11 call is
`run_cmd(["xdpyinfo"])` without an explicit `env`, while runtime children use
`child_env()`. This is not shown to cause the present failure because both
environments have identical DISPLAY/XAUTHORITY values and both fail
identically. No harness change is justified by this comparison.

## Minimal future requirement

Execution must be launched from the same reachable desktop namespace/session as
the known-good graphical terminal, with matching uid/gid, `DISPLAY`, and
readable matching `XAUTHORITY`. Before any new Director-authorized pair, the
exact harness child environment must demonstrate a connected X0 endpoint and
`xdpyinfo` return code `0`. No `xhost`, headless bypass, display-variable
override, or simulator launch change is authorized by this diagnosis.

P1-10 remains `IMPLEMENTED / AWAITING INDEPENDENT REVIEW`. Both failed pairs
remain unchanged; no replay retry or new pair was performed.
