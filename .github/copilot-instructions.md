# Copilot Instructions for Go2 Quadruped Robots Project

This file provides coding conventions and technical context for GitHub Copilot when working in this repository.

## Project Context

Reproducing ABS (Agile But Safe) paper (RSS 2024) — RL-based collision-free quadruped locomotion.
- Paper robot: Unitree Go1. Lab robot: Unitree Go2
- Training: Isaac Gym Preview 4 + PyTorch. Deployment: ROS2 Humble + MuJoCo

## Critical Joint Order Convention

**ALL code must use joint order: FR, FL, RR, RL**

```
Index 0-2:   FR hip, thigh, calf   (Front Right)
Index 3-5:   FL hip, thigh, calf   (Front Left)
Index 6-8:   RR hip, thigh, calf   (Rear Right)
Index 9-11:  RL hip, thigh, calf   (Rear Left)
```

This matches: Isaac Gym URDF, MuJoCo XML (`go2.xml`), `RlQuadrupedController.h`, `robot_control.yaml`.

**Foot/contact force order**: FR, FL, RR, RL (same as joints).

## RL Controller Architecture

Two separate controllers — DO NOT confuse them:

- `rl_quadruped_controller` (plugin: `LeggedGymController`) — has RL state, for ABS
- `unitree_guide_controller` (plugin: `UnitreeGuideController`) — FSM only, NO RL states

## Agile Policy Observation (61-dim)

```
[0:4]   contact(4)      FR, FL, RR, RL  (±1 = contact/no-contact)
[4:7]   ang_vel(3)      IMU gyro x, y, z
[7:10]  gravity_vec(3)  gravity direction in body frame
[10:13] commands(3)     POSITION targets in body frame [x, y, yaw]
[13]    timer(1)        Countdown 1.0→0.0 (inverted from episode_timer_)
[14:26] dof_pos(12)     Joint positions (FR hip..RL calf)
[26:38] dof_vel(12)     Joint velocities
[38:50] actions(12)     Previous action output
[50:61] ray2d(11)       Ray distances, log2 transformed
```

Recovery Policy: 49-dim (same minus timer and ray2d).

## Key Parameters (Go2)

```yaml
action_scale: 0.25
rl_kp: 30          # PD proportional gain
rl_kd: 0.65        # PD derivative gain
decimation: 4       # RL runs at frequency/4
hip_scale_reduction_indices: [0, 3, 6, 9]
dof_vel_scale: 0.2
contact_threshold: 1.0  # Newtons
max_episode_length_s: 9.0
ray2d_count: 11
ray2d_max_range: 6.0
```

## Commands Format

ABS uses **POSITION** commands (distance to target in body frame), NOT velocity:
- Training ranges: pos_x=[1.5, 7.5], pos_y=[-2.0, 2.0], heading=[-0.3, 0.3]
- Keyboard maps [0,1] to training range: `cmd = {lx*6.0, ly*2.0, ryaw*0.3}`

## Timer Semantics

Training: `timer_left / episode_length_s` — counts DOWN from ~1.0 to 0.0
Deployment: `1.0 - min(episode_timer/max_ep_len, 1.0)` — inverts the up-counting timer

## Files Modified for ABS Integration

| File | What changed |
|------|-------------|
| `controllers/rl_quadruped_controller/src/FSM/StateRL.cpp` | Timer inversion, commands scaling, debug logging |
| `controllers/rl_quadruped_controller/src/FSM/StateRLRec.cpp` | New — recovery policy (49-dim) |
| `controllers/rl_quadruped_controller/src/RlQuadrupedController.h` | Fixed stand_pos_, added rlRec |
| `descriptions/unitree/go2_description/config/robot_control.yaml` | Joint order FR-first, use_rl_thread=false |
| `libraries/controller_common/include/.../enumClass.h` | Added RL, RL_REC states |
| `descriptions/unitree/go2_description/config/abs/config.yaml` | New — 61-dim agile config |
| `descriptions/unitree/go2_description/config/rec/config.yaml` | New — 49-dim recovery config |

## Code Patterns

### Controller update flow (StateRL)

```
run() → getState() → runModel() → setCommand()
         (reads       (policy      (publishes
          sensors,     inference,   motor
          control      sets obs)    commands)
          inputs)
```

### PD control formula

```cpp
// action_scaled = raw_action * action_scale
// target_q = action_scaled + default_dof_pos
// torque = kp * (target_q - current_q) - kd * current_dq
// Position control: set target_q, kp, kd; let hardware handle the rest
```

### Debug logging (first 10 RL steps)

```cpp
static int debug_step = 0;
if (debug_step < 10) {
    RCLCPP_INFO(..., "[DEBUG-%d] ...", debug_step, ...);
    debug_step++;
}
```

## Constraints

- Python 3.8 only (Isaac Gym + numpy <1.24 requirement)
- Isaac Gym Preview 4 — closed source, do not substitute
- Libtorch 2.0.1 CPU for ROS2 build
- RTX 4060 (8GB): reduce num_envs if OOM
- Server shared resource: use CUDA_VISIBLE_DEVICES, never kill others' processes
- Never delete files without explicit user approval and clear reason
