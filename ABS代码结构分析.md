# ABS 代码结构分析

## 总体架构

```
ABS/
├── training/          # 训练（Isaac Gym + PPO）
│   ├── legged_gym/    # 仿真环境 + 训练脚本
│   └── rsl_rl/        # PPO 算法实现（fork 自 ETH Zurich）
└── deployment/        # 部署（ROS1 + Unitree SDK）
    └── src/abs_src/   # 四个 Python ROS 节点
```

---

## 训练侧 — 核心文件映射到论文四个模块

### 1. Agile Policy（目标导向避障策略）

| 文件 | 行数 | 作用 |
|------|------|------|
| `envs/base/legged_robot.py` | 1124 | 基础四足环境——物理仿真、域随机化、ray2d 传感器、观测/奖励计算 |
| `envs/base/legged_robot_pos.py` | 357 | 继承基类，加了**目标到达**逻辑（论文中 agile policy 的核心） |
| `envs/go1/go1_config.py` | 116 | Go1 机器人参数：URDF路径、PD刚度阻尼、关节名 |
| `envs/go1/go1_pos_config.py` | 242 | **Agile Policy 训练配置**：1280并行环境、61维观测、奖励权重、ray2d传感器参数 |
| `scripts/train.py` | 48 | 通用训练入口，`python train.py --task=go1_pos_rough` |

**观测空间 (61维)**：本体感知 (lin_vel×3 + ang_vel×3 + dof_pos×12 + dof_vel×12) + 命令 (位置目标×2 + 偏航目标×1) + ray2d (11维射线距离)

**动作空间 (12维)**：12个关节的目标角度偏移量

### 2. RA Value Network（Reach-Avoid 值函数，决定何时切换策略）

| 文件 | 行数 | 作用 |
|------|------|------|
| `scripts/testbed.py` | 582 | **核心文件**。三种模式：<br>① `--trainRA`：从 agile policy rollout 数据中训练 RA value network<br>② `--testRA`：加载 RA + recovery policy，端到端评估（agile ↔ recovery 切换）<br>③ 默认：仅运行 agile policy，收集数据 |

RA 值函数是一个简单 MLP `(19→64→64→1, Tanh)`，输出 ∈(-1, 1)：
- 正值 → 危险，切换到 Recovery Policy
- 负值 → 安全，继续用 Agile Policy

训练用的是 discounted RA Bellman 方程，从 rollout 缓冲区采样，在线训练。

### 3. Recovery Policy（安全恢复策略）

| 文件 | 行数 | 作用 |
|------|------|------|
| `envs/base/legged_robot_rec.py` | 193 | Recovery 环境：简单跟踪 twist 指令（无避障），不碰撞就行 |
| `envs/go1/go1_rec_config.py` | 177 | Recovery 训练配置 |
| `scripts/train.py` | 48 | 同 train.py，`--task=go1_rec_rough` |

论文中 recovery policy 在 testbed.py 的 `--testRA` 模式下被加载并调用（行 222-223）。

### 4. Ray-Prediction Network（深度图 → 稀疏射线）

| 文件 | 行数 | 作用 |
|------|------|------|
| `scripts/camrec.py` | 137 | 运行 agile policy + 开启深度相机，采集 (深度图, ray2d标签) 配对数据 |
| `scripts/train_depth_resnet.py` | 233 | 训练 ResNet 从深度图预测稀疏射线距离 |

---

## RSL-RL 修改（PPO 算法层）

| 文件 | 修改 |
|------|------|
| `modules/actor_critic_cost.py` | 新增：带 cost head 的 Actor-Critic（用于 Lagrangian PPO） |
| `runners/on_policy_runner_cost.py` | 新增：支持 cost 约束的 PPO runner |
| `storage/rollout_storage_extend.py` | 扩展：存储额外的 RA 观测值 |

---

## 部署侧（ROS1 + Unitree Go1）

| 文件 | 作用 |
|------|------|
| `publisher_depthimg_linvel.py` | ROS 节点：订阅深度图 → Ray-Prediction Network 推理 → 发布射线距离 + 里程计 |
| `depth_obstacle_depth_goal_ros.py` | **主控制循环**：接收射线+里程计 → RA Value Network 推理 → 切换 Agile/Recovery Policy → 发关节指令给机器人 |
| `led_control_ros.py` | LED 颜色反馈 RA 值（绿=安全，红=危险） |
| `onnx_model_converter.py` | PyTorch `.pt` → ONNX 转换（机载推理用） |

---

## 训练数据流总结

```
train.py ──► Agile Policy ──► play.py 导出 JIT ──┐
                                                   │
testbed.py (默认模式) ◄── 加载 Agile Policy ◄──────┘
    │                   收集 rollout 数据
    │
    ├─ --trainRA ──► 在线训练 RA Value Network
    │                    │
    │                    ▼
    ├─ --testRA  ──► 端到端：Agile ↔ Recovery 切换
    │                (加载 RA Value + Recovery Policy)
    │
camrec.py  ──► 采集深度图数据 ──► train_depth_resnet.py
```
