# Go2 Quadruped Robot — ABS (Agile But Safe) Reproduction & Deployment

Reproduce the **ABS paper** (RSS 2024, CMU & ETH Zurich) — RL-based collision-free high-speed quadruped locomotion (max 3.1 m/s). Deploy to Unitree Go2 robot via ROS2 + MuJoCo.

Paper: https://agile-but-safe.github.com/ | Code: https://github.com/LeCAR-Lab/ABS

## Results (Go2)

| Metric | Go2 (Ours) | Paper (Go1) |
|--------|-----------|-------------|
| Collision Rate | 1.22% | ~1% |
| Goal-Reaching Rate | 87.97% | ~90% |
| Avg Speed | 1.45 m/s | ~1.5 m/s |
| Max Speed | 2.82 m/s | ~3.1 m/s |
| RA Collision Recall | 78.42% | ~80% |
| Recovery Success Rate | 97.75% | ~97% |

## Quick Start

### Training (Isaac Gym + Server GPUs)

```bash
conda activate abs
cd ABS/training/legged_gym/legged_gym
python scripts/train.py --task=go2_pos_rough --num_envs=1280 --max_iterations=4000 --headless
```

### MuJoCo Simulation (3 Terminals)

**Terminal 1** — MuJoCo physics engine (start first):
```bash
cd unitree_mujoco && export LD_LIBRARY_PATH=/home/lidio/Libraries/unitree_sdk2/lib:$LD_LIBRARY_PATH
./simulate/build2/unitree_mujoco
```

**Terminal 2** — ROS2 RL controller (**must use rl_quadruped_controller, NOT unitree_guide_controller**):
```bash
source /opt/ros/humble/setup.bash
source quadruped_ros2_control_humble/install/setup.bash
export LD_LIBRARY_PATH=/home/lidio/Libraries/unitree_sdk2/lib:/home/lidio/Libraries/libtorch-cpu-2.0.1/lib:$LD_LIBRARY_PATH
ros2 launch rl_quadruped_controller mujoco.launch.py
```

**Terminal 3** — Keyboard teleop:
```bash
source /opt/ros/humble/setup.bash
source quadruped_ros2_control_humble/install/setup.bash
ros2 run keyboard_input keyboard_input
```

**FSM flow**: Press W many times → `2` (fixed down) → `2` (fixed stand) → `3` (RL mode) → keep pressing W

### ROS2 Build

```bash
cd quadruped_ros2_control_humble && source /opt/ros/humble/setup.bash
colcon build --packages-select rl_quadruped_controller --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_PREFIX_PATH="/home/lidio/Libraries/libtorch-cpu-2.0.1:/opt/ros/humble" \
  -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch \
  -DCaffe2_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Caffe2
```

## Repositories

| Repo | Purpose | Framework |
|------|---------|-----------|
| `ABS/` | Target paper — agile + recovery + RA value + ray-prediction | Isaac Gym, PyTorch, ROS1 |
| `legged_gym/` | Base RL training framework (vanilla Legged Gym) | Isaac Gym, RSL-RL |
| `quadruped_ros2_control_humble/` | Go2 ROS2 control (MuJoCo/Gazebo, OCS2 MPC + RL) | ROS2 Humble, MuJoCo, Gazebo |
| `rl_sar/` | RL sim-and-real framework | Isaac Gym, MuJoCo, ROS1/ROS2 |
| `legged_control/` | NMPC+WBC (deprecated, reference only) | ROS1 Noetic, OCS2 |
| `unitree_mujoco/` | Standalone MuJoCo physics sim for Go2 | MuJoCo 3.3.3, DDS |

## Status (2026-05-31)

| Phase | Status |
|-------|--------|
| Go1 Training (agile, recovery, RA, ray) | ✅ Complete |
| Go2 Training (all 4 modules, end-to-end test) | ✅ Complete |
| ROS2 Compilation | ✅ Complete |
| MuJoCo Simulation Pipeline | ✅ Complete |
| Agile Policy MuJoCo Test | 🔴 **Walks in circles** — debugging |
| Recovery Policy Integration | ✅ Code written, compiling |
| Ray-Prediction Integration | ❌ Not started |
| RA Value + Auto-Switch | ❌ Not started |
| Real Go2 Deployment | ❌ Not started |

## Environment

| Component | Path/Value |
|-----------|-----------|
| CUDA | 11.8 (`/usr/local/cuda-11.8`) |
| Conda | `abs` (Python 3.8.20) |
| PyTorch | 2.0.1+cu118 |
| Isaac Gym | Preview 4 (`/home/lidio/isaacgym/isaacgym/`) |
| libtorch (CPU) | 2.0.1 (`/home/lidio/Libraries/libtorch-cpu-2.0.1/`) |
| ROS2 | Humble (`/opt/ros/humble/`) |
| MuJoCo | 3.3.3 (`unitree_mujoco/simulate/build2/`) |
| unitree_sdk2 | `/home/lidio/Libraries/unitree_sdk2/` |
| Server GPU | 4× A800 80GB (shared) |

## Documentation Index

| Document | Audience | Content |
|----------|----------|---------|
| `README.md` | **All AI tools + humans** | This file — overview and quick start |
| `CLAUDE.md` | **Claude Code** | Full technical manual: architecture, observation layout, debugging |
| `ABS复现计划.md` | Humans | High-level progress tracker |
| `服务器训练指南.md` | Humans | Server SSH, tmux, TensorBoard |
| `仿真部署手册.md` | Humans | MuJoCo simulation setup |
| `.github/copilot-instructions.md` | **GitHub Copilot** | Coding conventions + technical context |

## Key Conventions

- **Joint order** (must match everywhere): FR, FL, RR, RL
- **Foot/contact order**: FR, FL, RR, RL
- **Agile obs**: 61-dim (contact+ang_vel+gravity+commands+timer+dof_pos+dof_vel+actions+ray2d)
- **Recovery obs**: 49-dim (same minus timer and ray2d)
- **PD gains** (Go2): Kp=30, Kd=0.65
- **RL controller**: `rl_quadruped_controller` (NOT `unitree_guide_controller`)
- **Server safety**: never modify `/data/isaacgym/`, never kill others' processes

## License

- ABS training: CC BY-NC 4.0
- Legged Gym, RSL-RL: BSD-3-Clause
- quadruped_ros2_control, rl_sar: Apache 2.0
