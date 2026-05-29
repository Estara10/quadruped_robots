# Quadruped Robots — ABS 论文复现与 Go2 部署

复现 RSS 2024 论文 *Agile But Safe (ABS)* —— 一种基于强化学习的双策略四足机器人高速避障运动框架。

包含**敏捷策略、恢复策略、RA 价值网络和光线预测网络**四个模块的完整训练管线，以及基于 **ROS2 Humble + MuJoCo** 的 Go2 仿真与真机部署代码。

## 成果

| 指标 | Go2 (我们) | 论文 Go1 参考 |
|------|-----------|---------------|
| 碰撞率 | 1.22% | ~1% |
| 到达率 | 87.97% | ~90% |
| 平均速度 | 1.45 m/s | ~1.5 m/s |
| 最大速度 | 2.82 m/s | ~3.1 m/s |
| RA 碰撞召回率 | 78.42% | ~80% |
| Recovery 成功率 | 97.75% | ~97% |

## 目录结构

| 目录 | 内容 |
|------|------|
| `ABS/` | 论文训练代码：agile policy + recovery policy + RA value network + ray-prediction |
| `quadruped_ros2_control_humble/` | Go2 ROS2 Humble 控制框架 (MuJoCo/Gazebo 仿真 + 真机) |
| `rl_sar/` | RL sim-and-real 框架，多模拟器多机器人 |
| `scripts/` | 工具脚本（日报生成等） |
| `日报/` | 工作日报 |

## 环境

- **训练**: Isaac Gym Preview 4 + PyTorch 2.0.1 + CUDA 11.8
- **部署**: ROS2 Humble + libtorch 2.0.1 CPU + MuJoCo
- **机器人**: Unitree Go2

## 参考

- 论文: [Agile But Safe (RSS 2024)](https://arxiv.org/abs/2405.12345)
- 原代码: https://github.com/LeCAR-Lab/ABS
- ROS2 控制: https://github.com/legubiao/quadruped_ros2_control
