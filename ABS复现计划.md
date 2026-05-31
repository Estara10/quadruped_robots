# ABS 论文复现 — 完整计划

> 更新: 2026-05-31

## 论文信息

- **标题**: Agile But Safe: Learning Collision-Free High-Speed Legged Locomotion
- **作者**: Tairan He, Chong Zhang (CMU & ETH Zurich)
- **发表**: RSS 2024
- **代码**: https://github.com/LeCAR-Lab/ABS
- **PDF**: `agile but safe(1).pdf`

### 方法概要

ABS 基于 RL 的双策略框架：
1. **Agile Policy** — 61维观测→12维动作(关节目标)，高速避障
2. **Recovery Policy** — 49维观测→12维动作，跟踪 twist 指令
3. **RA Value Network** — 19维输入→标量，决定策略切换
4. **Ray-Prediction Network** — 深度图→11条射线距离

---

## 当前状态 (2026-05-31)

### 训练 — 全部完成 ✅

| # | 模块 | 机器人 | 详情 |
|---|------|--------|------|
| 1 | Agile Policy | Go1 | 4000 iters |
| 2 | Recovery Policy | Go1 | 6000 iters |
| 3 | RA Value | Go1 | 完成 |
| 4 | Agile Policy | Go2 | 4000 iters, 碰撞率 1.22% |
| 5 | Recovery Policy | Go2 | 6000 iters |
| 6 | RA Value | Go2 | 135k steps |
| 7 | Ray-Prediction | Go2 | ResNet18, 250 epochs |
| 8 | 端到端测试 | Go2 | 256k 集, 碰撞率 1.22% |

### ROS2 部署 — 进行中 🔴

| 里程碑 | 状态 |
|--------|------|
| M1: 基础编译 | ✅ |
| M2: 61维观测 | ✅ |
| M3: MuJoCo 管道 | ✅ |
| M4: Agile 推理 | 🔴 走圆圈 |
| M5: Recovery | ✅ |
| M6: Ray-Pred | ❌ |
| M7: RA+切换 | ❌ |
| M8: 真机 | ❌ |

### 🔴 当前阻塞: 机器人走圆圈

已修复 6 个问题，待验证。详见 `/home/lidio/.claude/plans/mpc-squishy-lollipop.md`

---

## 环境

| 项目 | 值 |
|------|-----|
| 本地 GPU | RTX 4060 Laptop (8GB) |
| CUDA | 11.8 |
| Conda | `abs` (Python 3.8.20) |
| PyTorch | 2.0.1+cu118 |
| Isaac Gym | Preview 4 |
| libtorch | 2.0.1 CPU |
| MuJoCo | 3.3.3 |
| ROS2 | Humble |
| 服务器 GPU | 4× A800 80GB |

## 参考文档

- **CLAUDE.md** — 完整技术文档（新会话第一读）
- **计划书** — `/home/lidio/.claude/plans/mpc-squishy-lollipop.md`
- 服务器训练 — `服务器训练指南.md`
- MuJoCo 仿真 — `仿真部署手册.md`
