# CLAUDE.md — quadruped_ros2_control_humble

**This is the main development repository** for the ABS paper reproduction.  
ROS2 Humble controller for Go2 quadruped robot.  
Parent project: `/home/lidio/quadruped_robots/CLAUDE.md`

## What we use here

| Component | Path | Role |
|-----------|------|------|
| RL Controller | `controllers/rl_quadruped_controller/` | **核心**: 策略推理, RA, recovery, goal 导航 |
| Hardware Interface | `hardwares/hardware_unitree_mujoco/` | MuJoCo DDS 桥接 → ros2_control |
| Go2 Description | `descriptions/unitree/go2_description/` | URDF, config YAML, policy .pt 文件 |
| Keyboard Input | `commands/keyboard_input/` | 键盘控制节点 |

**Router**: `rl_quadruped_controller` (not `unitree_guide_controller` — that's a different controller!)

## Core File: StateRL.cpp

`controllers/rl_quadruped_controller/src/FSM/StateRL.cpp` contains ALL control logic:

- `runModel()` — RL step: state → goal command → observation → RA → policy/recovery action
- `computeObservation()` — 61-dim agile policy observation
- `computeRAObservation()` — 19-dim RA model observation
- `computeRecoveryObservation()` — 49-dim recovery policy observation
- `computeRecoveryTwist()` — **paper's gradient descent** (3 iters, torch::autograd)
- `logEvalTelemetry()` / `logSymmetryDebug()` — simulation evaluation diagnostics only
- `forward()` — agile policy inference
- `loadYaml()` — read abs/config.yaml

## Key Config Files

| File | Purpose |
|------|---------|
| `descriptions/unitree/go2_description/config/abs/config.yaml` | Policy params, RA, twist, goal, telemetry |
| `descriptions/unitree/go2_description/config/abs/policy.pt` | TorchScript agile policy |
| `descriptions/unitree/go2_description/config/rec/policy.pt` | TorchScript recovery policy |
| `descriptions/unitree/go2_description/config/robot_control.yaml` | ros2_control setup |

Important ABS config flags:
- `goal_x`, `goal_y`: first world-frame target.
- `resample_goal_on_arrival: false`: stop at first target by default; set `true` for continuous random-goal evaluation.
- `eval_telemetry_enabled`, `symmetry_debug_enabled`: diagnostics only, no control effect.

## FSM States

PASSIVE → FIXEDDOWN → FIXEDSTAND → RL (agile + inline recovery) → RL_REC (manual)
Key 2: FIXEDDOWN, Key 3: RL, Key 4: manual RL_REC

## Joint Order

ALL joints in FR, FL, RR, RL order (matches MuJoCo actuators/sensors and DDS motor_state).  
Policy expects FL-first ROS1 deployment order — `policy_joint_order: ros1_fl_fr_rl_rr` handles dof/contact/action remap.  
Estimator fallback also uses FR, FL, RR, RL leg chain order; do not pass FL-first `feet_names` unless the controller joint order is changed too.  
Joint limits: hip ±1.0472, thigh [-1.5708, 3.4907], calf [-2.7227, -0.83776].

## Runtime Conventions

- Agile policy observation does not include linear velocity, but RA and recovery do.
- RA/recovery `lin_vel` must be body-frame velocity, matching training `quat_rotate_inverse(base_quat, root_states[:, 7:10])` and ROS1 `zed_linvelo`.
- MuJoCo runtime should use odometer `velocity.x/y/z` rotated into body frame; estimator velocity is only fallback.
- Contact is read in controller order FR,FL,RR,RL, then remapped to policy order FL,FR,RL,RR in observation construction.
- `launch_abs_sim.sh` auto-enters RL; by default the controller stops at the first configured goal.

## Building

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rl_quadruped_controller --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch
```

## Hardware Interface (MuJoCo)

`hardwares/hardware_unitree_mujoco/` bridges DDS ↔ ros2_control:
- Receives LowState (joints, IMU, foot_force) and SportModeState (odometer) via DDS
- Sends LowCmd (position, velocity, kp, kd, tau) via DDS
- Odometer sensor: position(x,y,z) + velocity(x,y,z) from MuJoCo framepos/framelinvel
