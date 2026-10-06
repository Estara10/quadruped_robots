# Keyboard Input

Reads keyboard input and publishes `control_input_msgs/msg/Inputs` on `/control_input`.

---

## Build

```bash
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
colcon build --packages-select keyboard_input --symlink-install
```

## Run

```bash
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run keyboard_input keyboard_input
```

---

## ABS / RL Controller Keys

For `rl_quadruped_controller`:

| Key | command | Meaning |
|---|---:|---|
| `1` | 1 | PASSIVE / stop sentinel / unload control |
| `2` | 2 | FSM toggle: PASSIVE→FIXEDDOWN→FIXEDSTAND; from FIXEDSTAND returns FIXEDDOWN |
| `3` | 3 | Enter RL from FIXEDSTAND (simulation only until real ray2d exists) |
| `4` | 4 | Manual RL_REC test |
| `9` | 9 | HARD STOP: global force PASSIVE |
| `Space` | 0 | Clear velocity/joystick input |

Movement input:

| Key | Effect |
|---|---|
| `W/S` | forward/back trim (`ly`) |
| `A/D` | left/right trim (`lx`) |
| `J/L` | yaw trim (`rx`) |
| `I/K` | right stick y (`ry`, rarely used) |

`1` and `9` currently have the same low-level effect: both force PASSIVE, which writes `kp=0,kd=0,tau=0` and the hardware layer sends Unitree `PosStopF/VelStopF` stop sentinels. This is an unload/stop, not a high-stiffness brake.

---

## Real Robot Safety

On real Go2, do **not** press `3` until a real ray2d source is connected and verified.

Current real robot validation sequence:

```text
robot prone
launch controller
confirm sport_mode released
confirm [MOTOR-MAP] and [VERIFY] logs
press 9 first (no motion expected)
press 2 once for FIXEDDOWN
press 2 again only if FIXEDDOWN is smooth and quiet
```
