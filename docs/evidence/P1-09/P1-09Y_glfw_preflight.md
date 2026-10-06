# P1-09Y — MuJoCo GLFW/Display Preflight

## Scope and result

This was a read-only environment diagnostic. No MuJoCo, ROS2, controller,
benchmark, formal run, or real robot was started. Result: **BLOCKED** for a
new P1-09X run because the configured X11 display is not reachable and no
existing headless wrapper was found.

## Evidence

| Check | Observation | Classification |
|---|---|---|
| `DISPLAY` | `:0` | CONFIRMED configured value |
| `WAYLAND_DISPLAY` | empty | CONFIRMED no Wayland display selected |
| `XDG_RUNTIME_DIR` | `/run/user/1000` | CONFIRMED |
| `XAUTHORITY` | `/run/user/1000/gdm/Xauthority` | CONFIRMED configured value |
| X11 socket | `/tmp/.X11-unix/X0` exists | CONFIRMED socket path exists |
| X server reachability | `xdpyinfo` and `xset q` both returned rc=1: `unable to open display ":0"` | CONFIRMED unavailable to this process |
| X/Wayland/Xvfb process | no usable Xorg/Xwayland/Wayland/Xvfb process was observed in the available process namespace | UNKNOWN whether a host display exists outside this namespace |
| `xvfb-run` / `Xvfb` | not installed/found | CONFIRMED unavailable |
| EGL/OSMesa libraries | system EGL and GL libraries exist; no OSMesa entry was found | CONFIRMED availability is not equivalent to simulator support |
| GLFW | `/lib/x86_64-linux-gnu/libglfw.so.3` exists and is linked by the binary | CONFIRMED |
| simulator graphics build | CMake target links `glfw`; no project headless/EGL/OSMesa mode is defined in the target | CONFIRMED |
| P1-09X application error | captured `ERROR: could not initialize GLFW` with MuJoCo rc=1 | CONFIRMED |

## Root-cause classification

The immediate blocker is **CONFIRMED**: the process has `DISPLAY=:0`, but the
X11 client checks cannot open that display. This is consistent with P1-09X's
GLFW initialization failure. The exact lower-level cause (stale socket,
authorization mismatch, namespace isolation, or unavailable X server behind
the socket) is **UNKNOWN**; this preflight did not alter or start a display
server.

The absence of a project-configured headless backend is **CONFIRMED** from the
CMake target: the executable links GLFW and X11 transitively, while no
`xvfb-run`, Xvfb, OSMesa, or project EGL-headless startup path is available.
EGL libraries being installed does not prove that this executable can use EGL
without a code/configuration change.

## Existing startup path

The existing command is:

```text
/home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco -s scene_flat.xml
```

`config.yaml` selects `scene_flat.xml` and loopback interface `lo`. The `-s`
argument resolves the scene, but does not establish a display server or
headless backend. Duplicate `domain_id`/`interface` keys were observed in the
existing configuration; they were not changed in this diagnostic.

## Candidate command — not executed

After a valid X11 session is made reachable and its matching authorization is
confirmed, the minimal candidate is:

```bash
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
  /home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/unitree_mujoco \
  -s scene_flat.xml
```

This command is only a candidate. It must not be treated as validated until a
separate Director-authorized P1-09X run succeeds in the preflight checks.
`xvfb-run` cannot currently be proposed as an executable local command because
it is not installed, and installing packages is outside this task.

## Authorization boundary

A new P1-09X retry requires explicit, separate Director authorization. This
task does not authorize installing a display server, changing simulator code,
changing configuration, or retrying the clean-shutdown run.
