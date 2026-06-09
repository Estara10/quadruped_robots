# CLAUDE.md

This file provides guidance to Claude Code when working with code in this repository.

**Project**: Reproduce ABS (Agile But Safe) paper — dual-policy collision-free quadruped locomotion.  
**Robot**: Go2 (paper uses Go1). **Simulator**: MuJoCo. **ROS**: Humble. **Inference**: LibTorch.  
**Last updated**: 2026-06-09 (final)

---

## Quick Orientation (read this first)

```
Goal: 机器人自主导航到目标点，遇到障碍自动切换 recovery 避障

仿真管线:
  MuJoCo (物理+射线) ──DDS──→ ros2_control ──→ StateRL.cpp (policy/RA/recovery)
                                    ↑
                          /control_input (键盘/脚本)

关键目录:
  quadruped_ros2_control_humble/  ← 主开发仓库 (RL controller)
  ABS/                            ← 论文源码 (训练+ROS1部署参考)
  unitree_mujoco/                 ← MuJoCo 仿真器
  scripts/                        ← 启动/测试脚本

核心文件:
  controllers/rl_quadruped_controller/src/FSM/StateRL.cpp  ← 一切控制逻辑
  unitree_mujoco/simulate/src/unitree_sdk2_bridge.h         ← DDS桥+射线

最简启动:
  ~/quadruped_robots/scripts/launch_abs_sim.sh     # 默认到达首目标后停止
  MUJOCO_SCENE=scene_obstacle.xml ./scripts/launch_abs_obstacle.sh  # 障碍物

当前状态: 仿真核心链路已端到端验证通过，4场景12次100%成功率。实机待部署。
```

---

## Repo Layout

```
quadruped_robots/                  (this workspace)
├── CLAUDE.md                      ← 你在这里
├── 仿真部署手册.md                 ← 详细部署指南
├── 服务器训练指南.md                ← 服务器 SSH/tmux/TensorBoard
├── agile but safe(1).pdf          ← 论文 PDF
├── scripts/
│   ├── launch_abs_sim.sh          ← 一键启动 (平地)
│   ├── launch_abs_terrain.sh      ← 一键启动 (障碍物场景)
│   ├── generate_test_scenes.py    ← 随机障碍物场景生成
│   ├── ray_predictor.py           ← 深度相机+ResNet18 (实验)
│   └── reproduce_report.py        ← docx 报告生成
├── ABS/                           ← 论文源码 (Isaac Gym, PyTorch, ROS1)
│   ├── training/legged_gym/       ← 训练代码 + 策略/RA/ResNet18 模型
│   └── deployment/src/            ← ROS1 部署 (唯一权威算法参考)
├── quadruped_ros2_control_humble/ ← 主开发仓库 (ROS2 Humble controller)
│   └── controllers/rl_quadruped_controller/  ← RL 控制器 (核心)
│   └── hardwares/hardware_unitree_mujoco/    ← MuJoCo 硬件接口
│   └── descriptions/unitree/go2_description/ ← Go2 URDF + config
└── unitree_mujoco/                ← MuJoCo 仿真器 (C++, DDS 桥)
    ├── simulate/src/unitree_sdk2_bridge.h    ← DDS 桥 + 射线
    └── unitree_robots/go2/        ← Go2 模型 + 场景 XML
```

## Paper Architecture (ABS)

```
Training (Isaac Gym)                     Deployment (ROS1/ROS2)
┌──────────────────────┐                 ┌──────────────────────┐
│ ① Agile Policy (PPO) │                 │ ① Agile Policy       │
│   61-dim → 12 joint   │                 │   61-dim → joint cmd │
│                       │                 │                      │
│ ② RA Value Network   │                 │ ② RA Value Network   │
│   19→64→64→1 (Tanh)  │                 │   评估安全性 → 阈值   │
│                       │                 │                      │
│ ③ Recovery Policy    │                 │ ③ Recovery Policy    │
│   49-dim → 12 joint   │                 │   twist→joint cmd    │
│                       │                 │                      │
│ ④ Ray-Pred (ResNet18)│                 │ ④ Ray-Pred (ResNet18) │
│   depth→11 rays       │                 │   仿真用几何射线替代  │
└──────────────────────┘                 └──────────────────────┘
```

## ⚠️ Key Constraint: Paper Reproduction First

本项目目标是**复现 ABS 论文**，不是"让机器人能避障就行"。
凡是论文中有明确算法的，**必须先忠实地实现论文方法**。
遇到困难时先问"论文为什么能工作？我们的差异在哪？"
只有 ROS1→ROS2 架构差异和仿真/实机环境差异，才可以做适配性修改。

**论文方法对照表**（不可自行替换）:

| 模块 | 论文方法 | 参考代码 |
|------|---------|---------|
| Recovery Twist 优化 | 梯度下降 ×3, loss=lam·max(ra+2eps,0)+0.02·pos_dev² | ROS1 L498-525 |
| RA 模型推理 | 19→64→64→1 Tanh, 触发阈值 ra>-twist_eps | ROS1 L475-488 |
| Recovery 策略推理 | 49-dim obs, 内联替代 agile action | ROS1 L532-536 |
| Goal 导航 | 世界坐标目标 → 机体坐标系位置指令 | ROS1 L145-175 |
| Contact 检测 | 足力阈值 >1N (仿真匹配训练) | training: contact_forces>1.0 |
| Ray2d 感知 | 仿真=几何射线, 实机=深度相机+ResNet18 | 各自独立 |

## Current Status (2026-06-09)

### Done ✅

敏捷策略推理 | 恢复策略推理 | RA 值网络 | Recovery Twist (论文梯度下降) | 目标导航 | 到达检测 | 射线感知 | FSM 自动启动 | DDS 超时 | 软启动 | RA/recovery 机体系速度修复 | Estimator 腿链顺序修复 | 首目标到达后停止配置 | 安全机制 (姿态/action/卸力) | 多场景评估基线 (12/12=100%)

### In Progress 🔄

避障行为精细化（recovery 参数调优）| heading 偏航改善（策略层偏置）

### Pending ❌

Ray-Prediction (ResNet18, M6b) — 需域适应 | 实机 Go2 部署 (M8) | 遥控器急停 (E.2) | 温度监控 (E.4) | 坡地场景

## Key Implementation Details

### Joint Order Convention — CRITICAL

全链路使用 **FR, FL, RR, RL** 顺序（匹配 MuJoCo 模型和 DDS 桥）。
策略训练时 Isaac Gym 按字母序导出 DOF 名（FL-first），因此观测/动作需通过 `policy_joint_order: "ros1_fl_fr_rl_rr"` 进行 remap。
切勿在 YAML 中更改关节顺序或在代码中添加 contact_map 等临时映射。

### FSM State Flow

```
PASSIVE --(key 2)--> FIXEDDOWN --(key 2)--> FIXEDSTAND --(key 3)--> RL
                                             key 4 → RL_REC (手动测试)
```
RL 状态内已有论文的内联 recovery（RA 触发自动切换），RL_REC 仅用于手动测试。

### Observation Layouts

- **Agile (61-dim)**: contact(4) + ang_vel(3) + gravity_vec(3) + commands(3) + timer(1) + dof_pos(12) + dof_vel(12) + actions(12) + ray2d(11)
- **Recovery (49-dim)**: 同上但无 timer 和 ray2d，commands 替换为 twist(3)
- **RA (19-dim)**: lin_vel(3) + ang_vel(3) + commands[0:2](2) + ray2d(11)

Timer 恒为 0.5（匹配 ROS1 部署）。Contact = +1(着地)/-1(离地)。RA/recovery 的 `lin_vel` 必须是机体系速度；MuJoCo 仿真优先使用 odometer world velocity 再旋到 body frame，fallback 才用 estimator。

### Goal Arrival Behavior

`abs/config.yaml` 中 `resample_goal_on_arrival: false` 为默认值：到达首个目标后 commands 置零并站住，便于复现实验和调试。若要连续随机目标评估，显式改为 `true`。

### Ray2d Architecture

```
MuJoCo Bridge (unitree_sdk2_bridge.h):
  2D 射线圆相交 → 11 条射线 (θ=-45°~+45°, step=9°)
  → log2 变换 → /mujoco_ray2d (POSIX shm)
  → /mujoco_qpos (19 doubles, 供 Python 深度渲染同步)

Geom 过滤: 跳过 plane/hfield/mesh, robot groups 2/3, 动态体
          保留 box/cylinder/sphere/capsule/ellipsoid
```

### Recovery Twist Optimization (Paper Method)

```
1. twist = [当前vx, 当前vy, 当前wz], requires_grad=true
2. 3 次迭代:
   a. 构造 19-dim RA 观测 (含当前 twist 替代 lin_vel[0:2] 和 ang_vel[2])
   b. ra_val = RA_model.forward(ra_obs)
   c. loss = 10·max(ra+0.1, 0) + 0.02·((vx·0.05-cmd_x)²+(vy·0.05-cmd_y)²)
   d. loss.backward() → clip_grad(±1.0) → lr=0.5 更新 → clip twist
3. 保持 30 RL 步 (≈240ms, 匹配 ROS1 3 步×80ms) → 允许退出
```

频率: ROS2 125Hz (8ms/步) vs ROS1 12.5Hz (80ms/步)。
30 步保持通过 `recovery_hold_steps` YAML 配置。

## Building

```bash
# RL Controller (most common)
cd ~/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash
colcon build --packages-select rl_quadruped_controller --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch

# Config-only changes
colcon build --packages-select go2_description --symlink-install

# MuJoCo simulator
cd ~/quadruped_robots/unitree_mujoco/simulate/build2 && make -j$(nproc)
```

## Environment

- Conda env `abs`: Python 3.8, PyTorch 2.0.1+cu118, `/home/lidio/anaconda3/envs/abs/`
- CUDA 11.8: `/usr/local/cuda-11.8`
- Isaac Gym Preview 4: `/home/lidio/isaacgym/isaacgym/`
- LibTorch CPU 2.0.1: `/home/lidio/Libraries/libtorch-cpu-2.0.1/`
- MuJoCo 3.3.3: `/home/lidio/quadruped_robots/unitree_mujoco/simulate/build2/`
- Unitree SDK2: `/home/lidio/Libraries/unitree_sdk2/`

## Related Documentation

- `仿真部署手册.md` — 完整部署指南 (启动/功能/配置/日志)
- `服务器训练指南.md` — 服务器 SSH/tmux/TensorBoard
- `ABS/CLAUDE.md` — 训练+ROS1 参考
- `quadruped_ros2_control_humble/CLAUDE.md` — Controller 详细
- `unitree_mujoco/CLAUDE.md` — 仿真器详细
- `reports/ABS复现进展报告.docx` — 论文对比报告

## Key Modified Files History

### 2026-06-09 — Runtime validation + behavior calibration

| File | Change |
|------|--------|
| `StateRL.cpp` | RA/recovery lin_vel uses body-frame velocity, configurable goal resampling, `[EVAL]`/`[SYMM]` telemetry |
| `StateRLRec.cpp` | Manual recovery uses same body-frame velocity convention |
| `QuadrupedRobot.cpp` | Estimator fallback leg chain order aligned with controller FR,FL,RR,RL |
| `abs/config.yaml` | `resample_goal_on_arrival: false`, evaluation telemetry toggles |
| `launch_abs_sim.sh` | Auto-RL still starts, but controller stops at first configured goal by default |
| `BaseFixedStand.cpp` | `[STAND-SYMM]` diagnostics for symmetry debugging |
| `StateRL.h` | Safety params (`body_tilt_limit_deg`, `action_output_clip`), `checkBodySafety()`, telemetry toggles |
| Runtime validation | Flat/scene_test1 ran through RL, recovery enter/exit observed |
| **Evaluation baseline** | 4 scenes × 3 runs, 12/12 = 100% success, 0 falls, 0 timeouts |
| Scene system | `scene_flat/obstacle/terrain/slope` with unified `MUJOCO_SCENE` env var |
| Log cleanup | Chinese annotations on [GOAL]/[RA-REC]/[TWIST-GD]; debug toggles default off |

### 2026-06-06/07 — Goal nav + Recovery GD + Cleanup

| File | Change |
|------|--------|
| `StateRL.cpp` | World-frame goal nav, goal resampling, paper GD recovery twist, 30-step hold, rm debug |
| `StateRL.h` | goal_x_, goal_y_, recovery_hold_steps_; rm accumulated_yaw_, dead members |
| `RlQuadrupedController.cpp/.h` | Odometer sensor claiming |
| `abs/config.yaml` | goal_x, goal_y, recovery_hold_steps; rm rl_cooldown_steps |
| `robot_control.yaml` | Odometer sensor config |
| `unitree_sdk2_bridge.h` | Geom type filter, qpos shm, rm debug diag |
| `scripts/` | launch_abs_terrain.sh, generate_test_scenes.py, ray_predictor.py, test_multiscene.sh |
| `CLAUDE.md` | Paper-reproduction protocol, updated status |

### 2026-06-05 — Safety

`launch_abs_sim.sh`, DDS timeout, soft start

### 2026-05-30/31 — Initial Setup

Joint order fix, StateRL, StateRLRec, FSM states, config files
