# ABS 论文复现 — 完整计划

## 论文信息

- **标题**: Agile But Safe: Learning Collision-Free High-Speed Legged Locomotion
- **作者**: Tairan He, Chong Zhang (CMU & ETH Zurich)
- **发表**: RSS 2024
- **代码**: https://github.com/LeCAR-Lab/ABS
- **项目页**: https://agile-but-safe.github.com/
- **PDF**: `/home/lidio/quadruped_robots/agile but safe(1).pdf`

### 方法概要

ABS 是一个基于 RL 的双策略框架：
1. **Agile Policy** — 端到端 RL 策略，从深度图+本体感知输出关节指令，实现高速避障（最高 3.1 m/s）
2. **Recovery Policy** — 安全恢复策略，当敏捷策略可能失败时接管控制
3. **Reach-Avoid Value Network** — reach-avoid 值函数，决定策略切换时机
4. **Ray-Prediction Network** — 深度图 → 稀疏射线距离，作为外感知输入

---

## 当前状态（截至 2026-05-27）

### 本地环境

| 项目 | 值 |
|------|-----|
| GPU | NVIDIA RTX 4060 Laptop (8GB VRAM) |
| CUDA | 11.8 (`/usr/local/cuda-11.8`) |
| Conda env | `abs` (Python 3.8.20) |
| PyTorch | 2.0.1+cu118 |
| Isaac Gym | Preview 4 (`/home/lidio/isaacgym/isaacgym/`) |

### 服务器环境

| 项目 | 值 |
|------|-----|
| SSH | `ssh ABS_training` (9.tcp.vip.cpolar.cn:13333) |
| User | zhaofangxu |
| GPU | 4× A800 80GB (当前用 GPU 0 和 GPU 1) |
| CPU | 32 核, 503GB 内存 |
| CUDA 驱动 | 12.4, toolkit 11.8 |
| Isaac Gym | `/data/isaacgym/` (不修改), 自装 `/data/sxq/isaacgym_python/` |
| ABS 代码 | `/data/sxq/ABS/training/` |
| conda env | `abs` (Python 3.8) |
| 数据盘 | `/data/sxq/` (6TB) |
| **严禁** | 修改 `/data/isaacgym/`，kill 他人进程，改系统配置 |

### Go1 训练状态 (已完成)

| # | 模块 | 命令 | 状态 |
|---|------|------|------|
| 1 | Agile Policy | `train.py --task=go1_pos_rough --num_envs=1280 --max_iterations=4000` | ✅ 完成 |
| 2 | RA Value Network | `testbed.py --task=go1_pos_rough --headless --trainRA --num_envs=1280` | ✅ 完成 |
| 3 | Recovery Policy | `train.py --task=go1_rec_rough --num_envs=1280 --max_iterations=6000` | ✅ 完成 |
| 4 | 端到端测试 | `testbed.py --task=go1_pos_rough --headless --testRA --num_envs=1000` | ✅ 完成 |

### Go1 端到端测试结果

| 指标 | Hard (d=2) | Medium (d=1) | 论文目标 |
|------|-----------|-------------|----------|
| 到达率 | 66.11% | 89.66% | >90% |
| 碰撞率 | 21.26% | 7.34% | <4.06% |
| Recovery 成功率 | 67.74% | 76.69% | - |
| RA 碰撞预警率 | 94.76% | 92.05% | - |
| 平均速度 | 1.53 m/s | 2.07 m/s | 3.1 m/s max |

### Go2 训练状态 (进行中)

| # | 模块 | 命令 | 状态 |
|---|------|------|------|
| 1 | Agile Policy | `train.py --task=go2_pos_rough --num_envs=1280 --max_iterations=4000` | 🔄 GPU 0, ~1500/4000 |
| 2 | Recovery Policy | `train.py --task=go2_rec_rough --num_envs=1280 --max_iterations=6000` | 🔄 GPU 1, 刚启动 |
| 3 | RA Value Network | `testbed.py --task=go2_pos_rough --headless --trainRA --num_envs=1280` | ⏳ 待 Agile 导出 |
| 4 | 端到端测试 | `testbed.py --task=go2_pos_rough --headless --testRA --num_envs=1000` | ⏳ 待全部完成 |

### Go2 已完成工作

- URDF: `resources/robots/go2/urdf/go2.urdf` (修复 `dont_collapse="true"`)
- Meshes: `resources/robots/go2/meshes/`
- 配置: `go2_pos_config.py`, `go2_rec_config.py`
- 注册: `envs/__init__.py` 中注册 `go2_pos_rough` 和 `go2_rec_rough`
- Bug 修复: URDF foot_fixed 缺 dont_collapse → 观测维度 57≠61; rec_config 缺 default_joint_angles → KeyError

### 服务器模型文件

```
/data/sxq/ABS/training/legged_gym/logs/
├── go1_pos_rough/                                    # Go1 Agile (已完成)
│   ├── 05_26_12-54-39_/model_4000.pt
│   └── exported/
├── go1_rec_rough/                                    # Go1 Recovery (已完成)
│   └── exported/policies/
├── go2_pos_rough/                                    # Go2 Agile (训练中)
└── go2_rec_rough/                                    # Go2 Recovery (训练中)
```

---

## 阶段 4: Go2 适配 ✅ 已完成

### 4.1 创建 Go2 Isaac Gym URDF ✅

Go2 URDF 已创建并修复:
- `ABS/training/legged_gym/resources/robots/go2/urdf/go2.urdf`
- `ABS/training/legged_gym/resources/robots/go2/meshes/`
- 关键修复: 4 个 foot_fixed joints 添加 `dont_collapse="true"` 保证 foot contact 检测

### 4.2 创建 Go2 训练配置 ✅

| 文件 | 用途 | 状态 |
|------|------|------|
| `go2_config.py` | 基础 URDF/PD/奖励 | ✅ |
| `go2_pos_config.py` | Agile Policy (61维观测, ray2d=11) | ✅ |
| `go2_rec_config.py` | Recovery Policy (49维观测, 无ray2d) | ✅ |

**Go1 vs Go2 关键差异**:

| 参数 | Go1 | Go2 |
|------|-----|-----|
| 站立高度 | 0.37 m | 0.35 m |
| PD Kp | 20 | 30 |
| PD Kd | 0.5 | 0.65 |
| 质量 rand | -1.0~1.0 | -2.0~2.0 |
| num_envs | 1280 | 1280 |

### 4.3 训练 ✅ → 进行中

1. ✅ 本地验证 Go2 URDF 可加载
2. ✅ 上传到服务器
3. 🔄 并行训练: Agile (GPU 0) + Recovery (GPU 1)
4. ⏳ RA Value Network + 端到端测试

---

## 阶段 5: ROS2 部署

### 5.1 模型导出与适配
- 导出 JIT .pt（proprioceptive 版本，无 ray2d）
- ABS 训练观测 61 维 → ROS2 RL 控制器 45 维 (`commands(3)+ang_vel(3)+gravity(3)+dof_pos(12)+dof_vel(12)+actions(12)`)
- 观测缩放因子映射: lin_vel×2, ang_vel×0.25, dof_vel×0.05

### 5.2 ROS2 控制器配置
- 配置文件: `quadruped_ros2_control/.../go2_description/config/legged_gym/config.yaml`
- 修改 model_folder、PD 参数、观测缩放

### 5.3 仿真验证 → 实机
- MuJoCo 仿真测试
- 实机部署

---

## 验证检查点

- [x] 本地环境: CUDA + PyTorch + Isaac Gym
- [x] 本地训练验证: Agile Policy 跑通
- [x] 服务器环境: nvidia-smi + conda + Isaac Gym
- [x] Go1 Agile Policy: 服务器 1280 envs × 4000 iters, 导出完成
- [x] Go1 Recovery Policy: 服务器 1280 envs × 6000 iters, 导出完成
- [x] Go1 RA Value Network: 训练完成
- [x] Go1 端到端测试: Medium 89.66% 到达率
- [x] Go2 URDF: 创建并修复 dont_collapse bug
- [x] Go2 配置: pos_config + rec_config 创建完成
- [ ] Go2 Agile Policy 训练完成
- [ ] Go2 Recovery Policy 训练完成
- [ ] Go2 RA Value Network 训练完成
- [ ] Go2 端到端测试
- [ ] ROS2 仿真验证
- [ ] Go2 实机部署
