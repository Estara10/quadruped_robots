# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

**Last updated**: 2026-06-06 — Goal navigation (odometer), ray2d geom filter, paper-reproduction protocol

## Project Goal

Reproduce the **ABS (Agile But Safe)** paper (RSS 2024) — an RL-based dual-policy framework for collision-free high-speed quadruped locomotion (max 3.1 m/s). After simulation reproduction, adapt from Go1 (paper) to Go2 (lab robot) and deploy to real hardware.

Paper PDF: `agile but safe(1).pdf`
Paper code: https://github.com/LeCAR-Lab/ABS

## Repo Layout

This workspace contains five independent repositories, each cloned directly:

| Repo | Purpose | Framework |
|------|---------|-----------|
| `ABS/` | **Target paper** — agile policy + recovery policy + RA value + ray-prediction | Isaac Gym, PyTorch, ROS1 |
| `legged_gym/` | Base RL training framework (vanilla Legged Gym) | Isaac Gym, RSL-RL |
| `quadruped_ros2_control_humble/` | Go2 ROS2 control stack with MuJoCo/Gazebo sim, OCS2 MPC + RL controllers | ROS2 Humble, MuJoCo, Gazebo |
| `rl_sar/` | RL sim-and-real framework, multi-simulator (Isaac Gym/Sim, MuJoCo, Gazebo), multi-robot | Isaac Gym, MuJoCo, ROS1/ROS2 |
| `legged_control/` | NMPC+WBC legged robot control (OCS2-based, ROS1). **No longer maintained** | ROS1 Noetic, OCS2 |
| `unitree_mujoco/` | Standalone MuJoCo physics simulator for Go2 (uses unitree_sdk2 for DDS comm) | MuJoCo 3.3.3, DDS |

## Key Architecture: ABS

ABS has two parts — training (Isaac Gym simulation) and deployment (ROS1 + Unitree SDK on robot):

### Training (`ABS/training/`)

Built on top of Legged Gym + RSL-RL:

```
ABS/training/
  legged_gym/          # Isaac Gym envs (forked from leggedrobotics/legged_gym)
    legged_gym/envs/
      base/            # Base env classes
        legged_robot.py            # Base legged robot env
        legged_robot_pos.py        # Goal-reaching extension
        legged_robot_rec.py        # Recovery policy env
        legged_robot_config.py     # PPO config dataclasses
        legged_robot_pos_config.py # Goal-reaching config
      go1/             # Unitree Go1 configs
      go2/             # Unitree Go2 configs (lab robot)
    legged_gym/scripts/
      train.py         # Policy training entry point
      play.py          # Policy evaluation / export for RA training
      testbed.py       # RA value training + evaluation + end-to-end test
      camrec.py        # Ray-prediction dataset collection
      train_depth_resnet.py  # Ray-prediction network training
  rsl_rl/              # PPO implementation (forked from leggedrobotics/rsl_rl)
```

**Four trainable modules** (Figure 2a of the paper):
1. **Agile Policy** — goal-reaching RL, outputs joint targets, trained via PPO
2. **RA Value Network** — predicts reach-avoid values conditioned on agile policy, trained from rollout data using discounted RA Bellman equation
3. **Recovery Policy** — tracks twist commands to lower RA values, also PPO-trained
4. **Ray-Prediction Network** — depth image → sparse ray distances (serves as exteroception for policy + RA network)

### Deployment (`ABS/deployment/`)

ROS1 Noetic on Ubuntu 20.04, Unitree Go1 EDU + Orin NX + ZED mini:
- `publisher_depthimg_linvel.py` — publishes ray predictions + odometry
- `depth_obstacle_depth_goal_ros.py` — main control loop (agile/recovery policy switch based on RA values)
- `led_control_ros.py` — LED feedback for RA values
- `onnx_model_converter.py` — PyTorch .pt → ONNX for onboard inference

### Key Reference: ROS1 Deployment Observation Construction

In `ABS/deployment/src/abs_src/depth_obstacle_depth_goal_ros.py` `make_observation_from_lowhigh_state()`:
- Commands are POSITION-based (goal position in robot frame), NOT velocity-based
- Timer is always 0.5 in ROS1 deployment (constant, not time-varying)
- Contact order in ROS1: FL, FR, RL, RR (reordered from Unitree SDK)

## Key Architecture: quadruped_ros2_control_humble

ROS2 Humble control framework for Go2 (and other quadrupeds), cloned from `github.com/legubiao/quadruped_ros2_control` (humble branch):

```
quadruped_ros2_control_humble/
  controllers/
    ocs2_quadruped_controller/  # MPC-based (OCS2) controller
    rl_quadruped_controller/    # RL policy inference controller (target for ABS deployment)
    unitree_guide_controller/   # FSM-based guide controller (separate from RL controller!)
  hardwares/
    hardware_unitree_sdk2/      # Real Go2 hardware interface via unitree_sdk2
    hardware_unitree_mujoco/    # MuJoCo simulation hardware interface
    gz_quadruped_hardware/      # Gazebo simulation hardware interface
  descriptions/unitree/         # URDF models: go1, go2, a1, aliengo, b2
  libraries/
    controller_common/          # FSMState base class, CtrlInterfaces, enumClass
    gz_quadruped_playground/    # Gazebo sim with LiDAR/depth camera
  commands/
    keyboard_input/             # Keyboard teleop node (not control_input!)
```

### CRITICAL: Two Different Controllers

There are **two separate controllers** in this repo, and they have DIFFERENT FSM state mappings:

| Controller | Package/Plugin | Key 3 Action | Key 4 Action | Launch File |
|------------|---------------|-------------|-------------|-------------|
| `unitree_guide_controller` | `UnitreeGuideController` | FREESTAND | TROTTING | `ros2 launch unitree_guide_controller mujoco.launch.py` |
| `rl_quadruped_controller` | `LeggedGymController` | **RL** | **RL_REC** | `ros2 launch rl_quadruped_controller mujoco.launch.py` |

**Always use `rl_quadruped_controller` for ABS policy testing!** The guide controller does NOT have RL states.

### FSM State Flow (rl_quadruped_controller)

```
PASSIVE --(key 2)--> FIXEDDOWN --(key 2)--> FIXEDSTAND --(key 3)--> RL --(key 4)--> RL_REC
                                                              ^                    |
                                                              |----(done)----------|
```

All state transitions happen via keyboard `command` values. The keyboard publishes to `/control_input` topic.

### Joint Order Convention — CRITICAL

**The entire system MUST use the same joint order: FR, FL, RR, RL** (matches Isaac Gym training URDF and MuJoCo XML):

| Index | Joint | Leg |
|-------|-------|-----|
| 0-2 | hip, thigh, calf | FR (Front Right) |
| 3-5 | hip, thigh, calf | FL (Front Left) |
| 6-8 | hip, thigh, calf | RR (Rear Right) |
| 9-11 | hip, thigh, calf | RL (Rear Left) |

#### Why FR-first matters (2026-06-01 root cause analysis)

The full data flow: **MuJoCo → DDS → HardwareInterface → ros2_control.xacro → YAML → Controller → Policy**

Every layer in this chain uses FR-first order, verified on 2026-06-01:

| Layer | File | Order |
|-------|------|-------|
| MuJoCo actuators | `unitree_mujoco/unitree_robots/go2/go2.xml` L228-239 | FR, FL, RR, RL |
| DDS motor_state | `unitree_mujoco/simulate/src/unitree_sdk2_bridge.h` L148 | FR, FL, RR, RL |
| ros2_control.xacro | `descriptions/.../xacro/ros2_control.xacro` L13-155 | FR, FL, RR, RL |
| Training URDF | `ABS/.../resources/robots/go2/urdf/go2.urdf` | FR, FL, RR, RL |
| robot_control.yaml | `descriptions/.../config/robot_control.yaml` | **MUST be FR, FL, RR, RL** |

**MuJoCo touch sensor order**: FR_touch, FL_touch, RR_touch, RL_touch → DDS foot_force: FR, FL, RR, RL → training contact: FR, FL, RR, RL

**Earlier versions had FL-first ordering in `robot_control.yaml`** which caused FR↔FL joint data swap in observations AND actions. Due to left-right symmetry, the robot walked but veered left. This has been fixed on 2026-06-01.

**Key lesson**: Do NOT add index remapping (like `contact_map`) to fix order mismatches — fix the root cause (YAML order). See full analysis at `docs/joint-order-root-cause.md`.

**⚠️ config.yaml `policy_joint_order` must be `"ros1_fl_fr_rl_rr"`**: IsaacGym's `get_asset_dof_names()` returns dof names in **alphabetical order** (FL→FR→RL→RR), NOT URDF document order. The Go2 policy was trained FL-first. The remap converts our FR-first controller data to FL-first policy observations. Removing this remap causes the robot to flip (front hips swing backward instead of forward). Do NOT disable it. (Verified 2026-06-02 via empirical test)

### 61-Dim Agile Policy Observation Layout

Index in `computeObservation()` flat tensor `obs[0][i]`:

| Indices | Field | Dims | Description |
|---------|-------|------|-------------|
| 0-3 | contact | 4 | Foot contact: +1=contact, -1=no contact. Order: FR, FL, RR, RL |
| 4-6 | ang_vel | 3 | Angular velocity from IMU gyro [x, y, z] (body frame) |
| 7-9 | gravity_vec | 3 | Gravity direction in body frame (computed from quaternion) |
| 10-12 | commands | 3 | Position commands in body frame [x, y, yaw] |
| 13 | timer | 1 | Constant 0.5 (matches ROS1 ABS deployment) |
| 14-25 | dof_pos | 12 | Joint positions (FR_hip..RL_calf) |
| 26-37 | dof_vel | 12 | Joint velocities |
| 38-49 | actions | 12 | Previous action output (from policy) |
| 50-60 | ray2d | 11 | Ray distances (log2 transformed, currently constant log2(6.0)=2.585) |

### 49-Dim Recovery Policy Observation Layout

Subset of Agile Policy: contact(4) + ang_vel(3) + gravity_vec(3) + commands(3) + dof_pos(12) + dof_vel(12) + actions(12). **No timer, no ray2d.**

### Keyboard Input Details

Package: `keyboard_input` (not `control_input`!)
Executable: `keyboard_input`
Source: `commands/keyboard_input/src/KeyboardInput.cpp`

Key mapping:
- `1`-`4`: command values → FSM state transitions
- `W/S`: ly (forward/back) with sensitivity=0.05 per press
- `A/D`: lx (left/right)
- `I/K`: ry (pitch)
- `J/L`: rx (yaw)
- `Space`: reset all values + command=0

**Important timing detail**: When a command key (1-4) is pressed, ALL movement values (lx, ly, rx, ry) are zeroed. Press W **after** entering RL mode. Need ~20 presses of W to reach ly=1.0.

## Environment Setup

| Component | Status | Details |
|-----------|--------|---------|
| CUDA 11.8 | Done | `/usr/local/cuda-11.8` (nvcc 11.8.89) |
| Conda env `abs` | Done | Python 3.8.20, located at `/home/lidio/anaconda3/envs/abs/` |
| PyTorch 2.0.1+cu118 | Done | GPU: RTX 4060 Laptop, CUDA available |
| Isaac Gym Preview 4 | Done | Installed at `/home/lidio/isaacgym/isaacgym/` |
| rsl_rl (ABS fork) | Done | Installed from `ABS/training/rsl_rl/` |
| numpy==1.21, tensorboard, setuptools==59.5.0 | Done | |
| ABS legged_gym | Done | Installed on server `/data/sxq/ABS/training/legged_gym/` |
| libtorch 2.0.1 CPU | Done | `/home/lidio/Libraries/libtorch-cpu-2.0.1/` (for ros2 colcon build) |
| MuJoCo 3.3.3 | Done | `/home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/` |
| unitree_sdk2 | Done | `/home/lidio/Libraries/unitree_sdk2/` (DDS comm between MuJoCo and ROS2) |

**Activation**: Conda env auto-sets `PATH`, `CUDA_HOME`, `LD_LIBRARY_PATH` via `activate.d/env_vars.sh`.

**Isaac Gym tarball**: `/home/lidio/下载/1/IsaacGym_Preview_4_Package.tar.gz` (192MB, keep as backup).

**Server guide**: `/home/lidio/quadruped_robots/服务器训练指南.md` — SSH, tmux, TensorBoard 操作

## Current Status (2026-06-03)

### Training: All Complete ✅

| Module | Robot | Status | Details |
|--------|-------|--------|---------|
| Agile Policy | Go2 | Done | 4000 iters, exported |
| Recovery Policy | Go2 | Done | 6000 iters, exported |
| RA Value Network | Go2 | Done | 135k steps, TorchScript converted |
| Ray-Prediction | Go2 | Done | ResNet18, 250 epochs (not deployed) |

### ROS2 Deployment Status

| Milestone | Status | Details |
|-----------|--------|---------|
| M1: Basic colcon build | ✅ | libtorch CPU, Humble branch |
| M2: 61-dim observation | ✅ | contact/timer/ray2d |
| M3: MuJoCo sim pipeline | ✅ | unitree_mujoco + DDS |
| M4: Agile Policy running | ✅ | policy_joint_order: ros1_fl_fr_rl_rr (essential!) |
| M4: Agile Policy running | ✅ | 61-dim obs, goal navigation via odometer |
| M5: Recovery Policy inference | ✅ | 49-dim obs, inline recovery (matches ROS1 lines 532-536) |
| M6: Ray2d (MuJoCo) | ✅ | 2D ray-circle → shm, geom type filter |
| M6b: Ray-Prediction | ❌ | Depth-camera-based ray prediction not integrated |
| M7a: RA Value + Auto-switch | ✅ | ra_value inference, auto-switch at threshold -0.05 |
| M7b: Recovery twist = paper GD | ❌ | **DEVIATION**: using ray2d-driven twist, NOT paper's gradient descent. Must revert to paper method. |
| M7c: Goal navigation | ✅ | World-frame goal via odometer, arrival detection |
| M8: Real Go2 deployment | ❌ | Not started |
| D.3: Auto-launch | ✅ | launch_abs_sim.sh: auto FSM transitions to RL |
| E.1: DDS timeout | ✅ | 200ms frozen joints → force PASSIVE + FATAL log |
| E.2: Remote emergency | ❌ | `/rt/wirelesscontroller` not subscribed |
| E.3: Soft start | ✅ | Kp/Kd ramp 0→target over 0.5s |
| E.4: Temperature monitor | ❌ | Motor temp not exposed to controller |
| — Goal resampling | ❌ | Paper resets goal every 1600 timesteps |

### Ray2d Architecture (Phase B, 2026-06-03)

```
MuJoCo Bridge (unitree_sdk2_bridge.h)
  mj_ray() × 11 → log2 transform → /mujoco_ray2d (POSIX shm)
       ↓ shared memory (zero-copy)
StateRL::runModel()
  read shm → obs_.ray2d → policy + RA model
```

**Key parameters**: theta [-45°, +45°], step 9° → 11 rays, origin (-0.05, 0, 0.25) body-frame, max 6.0m, log2 transform.

**Critical finding**: `mj_ray()` `bodyexclude` only excludes bodies connected by weld constraints. Go2 legs are hinge joints → rays hit own legs. Fix: raised ray origin z to 0.25 (above hips).

**Fallback**: If shm not available (MuJoCo not started), auto-fallback to constant log2(6.0).

**Verification** (scene.xml, flat):
- ray2d: all 2.585 (max range, no obstacles)
- ra_value: -0.7372 (negative = safe, no auto-switch)

### Strategy/Model Files (Local Paths)

| File | Path | Status |
|------|------|--------|
| Agile Policy | `quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/policy.pt` | Deployed |
| Agile Config | `quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml` | Deployed |
| Recovery Policy | `quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/rec/policy.pt` | Deployed (TorchScript) |
| Recovery Config | `quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/rec/config.yaml` | Deployed |
| RA Value Network | `ABS/training/legged_gym/logs/go2_pos_rough/exported/RA/` | Not deployed to ROS2 |
| Ray-Prediction Model | `ABS/training/legged_gym/legged_gym/depth_logs/` (ResNet18, 43MB) | Not deployed to ROS2 |
| Go1/Go2 Training Policies | `ABS/training/legged_gym/logs/` | Training artifacts |

### How to Launch MuJoCo Simulation (3 Terminals)

**Terminal 1** — MuJoCo physics engine (start first):
```bash
cd ~/quadruped_robots/unitree_mujoco
export LD_LIBRARY_PATH=/home/lidio/Libraries/unitree_sdk2/lib:$LD_LIBRARY_PATH
./simulate/build2/unitree_mujoco
```

**Terminal 2** — ROS2 RL controller (CRITICAL: use rl_quadruped_controller, NOT unitree_guide_controller):
```bash
source /opt/ros/humble/setup.bash
source ~/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash
export LD_LIBRARY_PATH=/home/lidio/Libraries/unitree_sdk2/lib:/home/lidio/Libraries/libtorch-cpu-2.0.1/lib:$LD_LIBRARY_PATH
ros2 launch rl_quadruped_controller mujoco.launch.py
```

**Terminal 3** — Keyboard teleop:
```bash
source /opt/ros/humble/setup.bash
source ~/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash
ros2 run keyboard_input keyboard_input
```

**Terminal 4** (optional) — Debug keyboard input:
```bash
source /opt/ros/humble/setup.bash
source ~/quadruped_robots/quadruped_ros2_control_humble/install/setup.bash
ros2 topic echo /control_input
```

**Operation sequence**:
1. In terminal 3: Press W ~20 times to build up ly=1.0
2. Press `2` → wait for "fixed down" → press `2` → wait for "fixed stand" → press `3` → RL mode
3. Immediately after pressing `3`, keep pressing `W` repeatedly
4. Watch terminal 2 for `[DEBUG-0]` through `[DEBUG-9]` output
5. Check `cmd=X.XX` field — if non-zero, keyboard is working

## ROS2 Build Commands

```bash
# Full RL controller build (with libtorch CPU)
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
colcon build --packages-select rl_quadruped_controller --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH="/home/lidio/Libraries/libtorch-cpu-2.0.1:/opt/ros/humble" \
  -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch \
  -DCaffe2_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Caffe2

# Build only description/config changes (no C++ compilation needed)
colcon build --packages-select go2_description --symlink-install

# Build keyboard_input
colcon build --packages-select keyboard_input --symlink-install

# Build all packages
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH="/home/lidio/Libraries/libtorch-cpu-2.0.1:/opt/ros/humble" \
  -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch \
  -DCaffe2_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Caffe2
```

## Key Modified Files (2026-05-30/31)

| File | Change Summary |
|------|---------------|
| `controllers/rl_quadruped_controller/src/RlQuadrupedController.h` | Added `#include StateRLRec.h`, `rlRec` to FSMStateList, fixed `stand_pos_` to match training (0, 0.8, -1.5) |
| `controllers/rl_quadruped_controller/src/RlQuadrupedController.cpp` | Added StateRLRec instantiation in `on_activate()`, RL_REC case in `getNextState()` |
| `controllers/rl_quadruped_controller/src/FSM/StateRL.cpp` | ① Timer inverted (1.0→0.0) ② Commands scaled to training range ③ Debug logging (10 steps) ④ checkChange: key 4→RL_REC ⑤ `use_rl_thread` respected |
| `controllers/rl_quadruped_controller/src/FSM/StateFixedStand.cpp` | key 3→RL, key 4→RL_REC |
| `descriptions/unitree/go2_description/config/robot_control.yaml` | **Joint order FR,FL,RR,RL**; `use_rl_thread: false`; foot force order fixed; stand_pos/down_pos aligned |
| `libraries/controller_common/include/controller_common/common/enumClass.h` | Added `RL` and `RL_REC` to FSMStateName enum |
| `controllers/rl_quadruped_controller/include/.../FSM/StateRLRec.h` | **New file** — Recovery policy FSM state header |
| `controllers/rl_quadruped_controller/src/FSM/StateRLRec.cpp` | **New file** — Recovery policy implementation (49-dim obs) |
| `descriptions/unitree/go2_description/config/abs/config.yaml` | **New file** — Agile policy 61-dim config |
| `descriptions/unitree/go2_description/config/rec/config.yaml` | **New file** — Recovery policy 49-dim config |
| `descriptions/unitree/go2_description/config/abs/policy.pt` | **New file** — TorchScript agile policy (783K) |
| `descriptions/unitree/go2_description/config/rec/policy.pt` | **New file** — TorchScript recovery policy (759K) |
| `controllers/rl_quadruped_controller/CMakeLists.txt` | Added StateRLRec.cpp |

## Key Modified Files (2026-06-05 — Phase D.3 + E safety)

| File | Change Summary |
|------|---------------|
| `scripts/launch_abs_sim.sh` | Auto-enter RL: `ros2 topic pub` sends FSM commands (2→2→3), auto ly=1.0 |
| `controllers/rl_quadruped_controller/src/RlQuadrupedController.h` | DDS timeout: `last_joint_positions_`, `dds_timeout_counter_`, threshold=100 steps |
| `controllers/rl_quadruped_controller/src/RlQuadrupedController.cpp` | DDS timeout check in `update()`: frozen joints >200ms → force PASSIVE + FATAL log |
| `controllers/rl_quadruped_controller/include/.../FSM/StateRL.h` | Soft start: `mutable soft_start_step_`, `soft_start_steps_=250` |
| `controllers/rl_quadruped_controller/src/FSM/StateRL.cpp` | `setCommand()` scales Kp/Kd by ratio; `enter()` resets counter; YAML reads `soft_start_steps` |
| `descriptions/unitree/go2_description/config/abs/config.yaml` | Added `soft_start_steps: 250` (~0.5s ramp) |
| `controllers/rl_quadruped_controller/doc/real_go2_deployment.md` | Updated safety feature status (DDS timeout ✅, Soft start ✅) |

## Key Constraints

0. **⚠️ 复现论文方法是最高优先级 — 不允许自行设计替代方案**
   - 本项目目标是**复现 ABS 论文**，不是"让机器人能避障就行"。
   - 凡是论文中有明确算法的，**必须先忠实地实现论文方法**。只有经过验证确实无法工作（且究明了原因），才可以讨论替代方案。
   - 遇到困难时，先问"论文为什么能工作？我们的差异在哪？"——而不是直接换一个更简单的方案。走捷径 = 离目标越来越远。
   - 以下模块论文有明确实现，必须用论文方法：
     1. **Recovery Twist 优化** — 梯度下降通过 RA 模型 (C++ `torch::autograd`)，不是 ray2d 驱动
     2. **RA 模型推理** — 19→64→64→1 Tanh，触发阈值 `ra > -twist_eps` (= -0.05)
     3. **Recovery 策略推理** — 49-dim 观测，内联替代 agile action
     4. **Goal 导航** — 世界坐标目标 + 机器人定位 → 机体坐标系位置指令
     5. **Contact 检测** — 足力阈值（仿真=1N 匹配训练，实机=待确认 Go2 SDK 单位）
     6. **Ray2d 感知** — 仿真=几何射线，实机=深度相机+ResNet18
   - 只有 ROS1→ROS2 架构差异（见下方#1）和仿真/实机环境差异，才可以做适配性修改。

1. **先看 ROS1 源码再动手** — 遇到任何部署问题，第一步是查看 `ABS/deployment/src/abs_src/` 下 ROS1 的对应实现。ABS 是一个复现项目，ROS1 代码是唯一权威参考。只有在 ROS1 方案在 ROS2/LibTorch 中确实不可行时，才考虑替代方案。这条规则适用于所有模块：ray2d、recovery、RA 推理、observation 构造、命令处理等。

1. **Isaac Gym Preview 4 is required** — closed-source. Do not try to substitute without understanding the codebase.
2. **Python 3.8 only** — Legged Gym fork in ABS depends on numpy <1.24 and Isaac Gym Preview 4 bindings.
3. **Paper is Go1, lab is Go2** — URDF, PD gains, and mass distribution differ.
4. **RTX 4060 (8GB VRAM) is tight** for 1280 parallel envs. Reduce `num_envs` in config if OOM.
5. **legged_control is deprecated** per its own README. Use `quadruped_ros2_control` for ROS2-based control.
6. **服务器操作严禁影响他人** — 多人共用服务器（4×A800），所有操作必须限定在 `/data/sxq/` 和 `/home/zhaofangxu/` 下。用 `CUDA_VISIBLE_DEVICES` 限定只使用空闲 GPU。禁止修改 `/data/isaacgym/`。禁止 kill 他人进程。禁止修改系统级配置。
7. **禁止随意删除文件或目录** — 任何 `rm`、`rm -rf` 必须：①给出明确理由；②经用户批准后才能执行。删除提示必须用中文。

## Verification Commands

```bash
# Activate environment
conda activate abs

# CUDA + PyTorch
python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"

# Isaac Gym imports
python -c "from isaacgym import gymapi, gymutil, gymtorch; print('OK')"

# Go2 training (server)
CUDA_VISIBLE_DEVICES=1 python scripts/train.py --task=go2_pos_rough --num_envs=1280 --max_iterations=4000 --headless
CUDA_VISIBLE_DEVICES=1 python scripts/train.py --task=go2_rec_rough --num_envs=1280 --max_iterations=6000 --headless

# Go2 export
CUDA_VISIBLE_DEVICES=1 python scripts/play.py --task=go2_pos_rough --num_envs=1
CUDA_VISIBLE_DEVICES=1 python scripts/play.py --task=go2_rec_rough --num_envs=1

# Go2 RA + end-to-end
CUDA_VISIBLE_DEVICES=1 python scripts/testbed.py --task=go2_pos_rough --headless --trainRA --num_envs=1280
CUDA_VISIBLE_DEVICES=1 python scripts/testbed.py --task=go2_pos_rough --headless --num_envs=1000 --testRA
```

## Custom Commands

### 日报生成 (/daily 或 "写今日总结")

```bash
conda activate abs
python /home/lidio/quadruped_robots/scripts/daily_summary.py "2026-xx-xx" "## 今日完成
- item 1
- item 2

## 问题与解决
- problem and fix

## 明日计划
- plan 1"
```

Output: `~/quadruped_robots/日报/日报_YYYYMMDD.docx`

## License Notes

- ABS training code: CC BY-NC 4.0 (no commercial use)
- Legged Gym: BSD-3-Clause (NVIDIA + ETH Zurich)
- RSL-RL: BSD-3-Clause (ETH Zurich)
- quadruped_ros2_control: Apache 2.0
- rl_sar: Apache 2.0
