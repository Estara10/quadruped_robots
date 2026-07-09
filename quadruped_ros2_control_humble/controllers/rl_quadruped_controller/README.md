# RL Quadruped Controller — ABS Go2

ROS2 Humble controller for ABS reproduction on Unitree Go2.

Core implementation:

```text
src/FSM/StateRL.cpp
```

Features:

- Agile policy inference (61-dim obs)
- RA value network (19-dim obs)
- Inline recovery policy (49-dim obs)
- Paper-matched recovery twist gradient descent
- MuJoCo ray2d shared memory input
- World-frame goal navigation
- recovery-aware straight-line path tracking
- global hard stop command

---

## Build

```bash
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
colcon build --packages-select rl_quadruped_controller --symlink-install \
  --cmake-args -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch
```

Recommended full build:

```bash
colcon build --packages-select hardware_unitree_mujoco rl_quadruped_controller keyboard_input go2_description --symlink-install \
  --cmake-args -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch
```

---

## Launch

Simulation/real both currently use:

```bash
source /opt/ros/humble/setup.bash
source ~/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash
export LD_LIBRARY_PATH=/home/lidio/Libraries/unitree_sdk2/lib:/home/lidio/Libraries/libtorch-cpu-2.0.1/lib:$LD_LIBRARY_PATH
ros2 launch rl_quadruped_controller mujoco.launch.py
```

Mode is determined by `go2_description/xacro/ros2_control.xacro`:

- simulation: network params commented, default `lo/domain=1`
- real: `domain=0`, `network_interface=enp7s0`

---

## FSM

```text
PASSIVE --2--> FIXEDDOWN --2--> FIXEDSTAND --3--> RL
                                      └--4--> RL_REC manual test
```

Global hard stop:

```text
1 or 9 -> PASSIVE immediately
```

`1` and `9` both unload control via PASSIVE/stop sentinel. They are not high-stiffness brakes.

---

## Joint Order

Controller order:

```text
FR_hip, FR_thigh, FR_calf,
FL_hip, FL_thigh, FL_calf,
RR_hip, RR_thigh, RR_calf,
RL_hip, RL_thigh, RL_calf
```

At activation, all loaned command/state interfaces are explicitly sorted by YAML `joints` order. Startup should print:

```text
[VERIFY] joint interface order: FR_hip_joint FR_thigh_joint FR_calf_joint ... RL_calf_joint
```

Policy order is ROS1/FL-first and is handled by:

```yaml
policy_joint_order: ros1_fl_fr_rl_rr
```

---

## ABS Config

Main config:

```text
descriptions/unitree/go2_description/config/abs/config.yaml
```

Important fields:

```yaml
goal_x: 7.0
goal_y: 0.0
resample_goal_on_arrival: false
path_tracking_enabled: true
path_lateral_gain: 1.5
path_heading_gain: 1.5
recovery_hold_steps: 30
emergency_stop_enabled: true
```

Path tracking behavior:

```text
normal agile: path_on=1, follow start→goal line
recovery: path_on=0, allow detour for obstacle avoidance
exit recovery: path_on=1, return to path
```

Logs:

```text
[PATH]
[GOAL] ... path_err=... path_on=...
[EVAL]
[RA]
[RA-REC]
[TWIST-GD]
```

---

## Real Go2 Gate

Do not enter RL (`3`) on real Go2 until real ray2d perception is connected and verified. Current real robot testing is limited to:

```text
PASSIVE
FIXEDDOWN
FIXEDSTAND
HARD STOP
```
