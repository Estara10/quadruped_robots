# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Goal

Reproduce the **ABS (Agile But Safe)** paper (RSS 2024) — an RL-based dual-policy framework for collision-free high-speed quadruped locomotion (max 3.1 m/s). After simulation reproduction, adapt from Go1 (paper) to Go2 (lab robot) and deploy to real hardware.

Paper PDF: `agile but safe(1).pdf`
Paper code: https://github.com/LeCAR-Lab/ABS

## Repo Layout

This workspace contains five independent repositories, each cloned directly:

| Repo | Purpose | Framework |
|------|---------|-----------|
| `ABS/` | **Target paper** — agile policy + recovery policy + RA value + ray-prediction | Isaac Gym, PyTorch, ROS1 |
| `legged_gym/` | Base RL training framework (vanilla Legged Gym) — use this to verify the environment before ABS | Isaac Gym, RSL-RL |
| `quadruped_ros2_control_humble/` | Go2 ROS2 control stack with MuJoCo/Gazebo sim, OCS2 MPC + RL controllers | ROS2 Humble, MuJoCo, Gazebo |
| `rl_sar/` | RL sim-and-real framework, multi-simulator (Isaac Gym/Sim, MuJoCo, Gazebo), multi-robot | Isaac Gym, MuJoCo, ROS1/ROS2 |
| `legged_control/` | NMPC+WBC legged robot control (OCS2-based, ROS1). **No longer maintained** — reference only | ROS1 Noetic, OCS2 |

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
        go1_config.py              # URDF, PD gains, domain rand
        go1_pos_config.py          # Agile policy training config
        go1_rec_config.py          # Recovery policy training config
      go2/             # Unitree Go2 configs (lab robot)
        go2_config.py              # URDF, PD gains, domain rand
        go2_pos_config.py          # Agile policy training config
        go2_rec_config.py          # Recovery policy training config
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

## Key Architecture: quadruped_ros2_control_humble

ROS2 Humble control framework for Go2 (and other quadrupeds), cloned from `github.com/legubiao/quadruped_ros2_control` (humble branch):

```
quadruped_ros2_control_humble/
  controllers/
    ocs2_quadruped_controller/  # MPC-based (OCS2) controller
    rl_quadruped_controller/    # RL policy inference controller (target for ABS deployment)
    unitree_guide_controller/   # FSM-based guide controller
  hardwares/
    hardware_unitree_sdk2/      # Real Go2 hardware interface via unitree_sdk2
    gz_quadruped_hardware/      # Gazebo simulation hardware interface
  descriptions/unitree/         # URDF models: go1, go2, a1, aliengo, b2
  libraries/
    gz_quadruped_playground/    # Gazebo sim with LiDAR/depth camera
```

The RL controller (`controllers/rl_quadruped_controller/`) is the integration target for deploying ABS-trained policies on Go2 via ROS2.

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

**Activation**: Conda env auto-sets `PATH`, `CUDA_HOME`, `LD_LIBRARY_PATH` via `activate.d/env_vars.sh`.

**Isaac Gym tarball**: `/home/lidio/下载/1/IsaacGym_Preview_4_Package.tar.gz` (192MB, keep as backup).

**Server guide**: `/home/lidio/quadruped_robots/服务器训练指南.md` — SSH, tmux, TensorBoard 操作

## Current Status (2026-05-28)

**Go2 仿真训练全部完成**，ROS2 Humble 编译通过，待 MuJoCo 仿真验证。

| Module | Robot | Status | Details |
|--------|-------|--------|---------|
| Agile Policy | Go1 | Done | 4000 iters, exported |
| RA Value Network | Go1 | Done | trained |
| Recovery Policy | Go1 | Done | 6000 iters, exported |
| Agile Policy | Go2 | Done | 4000 iters, exported, 端到端碰撞率 1.22% |
| Recovery Policy | Go2 | Done | 6000 iters, exported |
| RA Value Network | Go2 | Done | 135k steps |
| Ray-Prediction | Go2 | Done | ResNet18, 250 epochs |
| ROS2 61-dim 观测 | Go2 | Done | StateRL.cpp/h 修改完成, Humble 编译通过 |
| ROS2 MuJoCo 验证 | Go2 | Next | ros2 launch 仿真测试 |

Progress plan: `/home/lidio/quadruped_robots/ABS复现计划.md`

## Key Constraints

1. **Isaac Gym Preview 4 is required** — it's closed-source, may be discontinued by NVIDIA. Do not try to substitute with Isaac Sim/Isaac Lab without understanding the codebase first.
2. **Python 3.8 only** — the Legged Gym fork in ABS depends on numpy <1.24 and Isaac Gym Preview 4 bindings.
3. **Paper is Go1, lab is Go2** — URDF, PD gains, and mass distribution differ. The paper authors explicitly state they "won't tune it for your own robots."
4. **RTX 4060 (8GB VRAM) is tight** for 1280 parallel envs. Reduce `num_envs` in config if OOM.
5. **legged_control is deprecated** per its own README. Use `quadruped_ros2_control` for ROS2-based Go2 control instead.
6. **服务器操作严禁影响他人** — 多人共用服务器（4×A800），所有操作必须限定在 `/data/sxq/` 和 `/home/zhaofangxu/` 下。用 `CUDA_VISIBLE_DEVICES` 限定只使用空闲 GPU。禁止修改 `/data/isaacgym/`（已共享安装）。禁止 kill 他人进程。禁止修改系统级配置。
7. **禁止随意删除文件或目录** — 任何 `rm`、`rm -rf`、删除操作必须：①给出明确理由；②经用户批准后才能执行。删除提示必须用中文。

## Verification Commands

```bash
# Activate environment
conda activate abs

# CUDA + PyTorch (verified)
python -c "import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))"

# Isaac Gym imports (verified)
python -c "from isaacgym import gymapi, gymutil, gymtorch; print('OK')"

# ABS agile policy training
cd ABS/training/legged_gym/legged_gym
python scripts/train.py --task=go1_pos_rough --max_iterations=4000

# ABS play (export policy for RA training)
python scripts/play.py --task=go1_pos_rough

# ABS RA value training
python scripts/testbed.py --task=go1_pos_rough --num_envs=1000 --headless --trainRA

# ABS recovery policy
python scripts/train.py --task=go1_rec_rough --max_iterations=1000

# Go2 training (server)
CUDA_VISIBLE_DEVICES=1 python scripts/train.py --task=go2_pos_rough --num_envs=1280 --max_iterations=4000 --headless
CUDA_VISIBLE_DEVICES=1 python scripts/train.py --task=go2_rec_rough --num_envs=1280 --max_iterations=6000 --headless

# Go2 export
CUDA_VISIBLE_DEVICES=1 python scripts/play.py --task=go2_pos_rough --num_envs=1
CUDA_VISIBLE_DEVICES=1 python scripts/play.py --task=go2_rec_rough --num_envs=1

# Go2 RA + end-to-end
CUDA_VISIBLE_DEVICES=1 python scripts/testbed.py --task=go2_pos_rough --headless --trainRA --num_envs=1280
CUDA_VISIBLE_DEVICES=1 python scripts/testbed.py --task=go2_pos_rough --headless --num_envs=1000 --testRA

# Go2 ROS2 MuJoCo simulation (separate terminal)
ros2 launch unitree_guide_controller mujoco.launch.py

# Go2 ROS2 colcon build (with libtorch CPU)
cd quadruped_ros2_control_humble
export PATH="/usr/bin:$PATH"
source /opt/ros/humble/setup.bash
colcon build --packages-select rl_quadruped_controller --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH="/home/lidio/Libraries/libtorch-cpu-2.0.1:/opt/ros/humble" \
  -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch \
  -DCaffe2_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Caffe2
```

## License Notes

- ABS training code: CC BY-NC 4.0 (no commercial use)
- Legged Gym: BSD-3-Clause (NVIDIA + ETH Zurich)
- RSL-RL: BSD-3-Clause (ETH Zurich)
- quadruped_ros2_control: Apache 2.0
- rl_sar: Apache 2.0

## Custom Commands

### 日报生成 (/daily 或 "写今日总结")

When the user asks for a daily summary (e.g., "写今日总结", "生成日报", "/daily"):

1. Review the conversation to identify what was accomplished today
2. Summarize in Chinese: modules trained, problems solved, key decisions, and next steps
3. Generate the .docx file with:

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

4. Tell the user the output path: `~/quadruped_robots/日报/日报_YYYYMMDD.docx`
