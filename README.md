# Go2 四足机器人风险驱动安全策略切换研究

本科毕业设计：《面向复杂障碍环境的四足机器人风险驱动安全策略切换方法研究》。

**更新：2026-11-09。** 实机工作的方向是先在 Go2 上实现 ABS 原论文的 Agile／RA／Recovery 控制链，再验证本课题的策略切换方法。电脑原生 A→B 是定位与导航基础验证；它调用宇树运动算法，不能替代 ABS 策略实机验证。

参考：[ABS 论文项目](https://agile-but-safe.github.io/) · [原作者代码](https://github.com/LeCAR-Lab/ABS) · [本仓库原作者部署参考](ABS/README.md#hardware-deployment)

## 当前做到哪里

| 工作 | 已完成范围 | 尚未完成 |
|---|---|---|
| Go2 仿真 | Go2 Agile／RA／Recovery、A/B/C/D 切换和多批比较已有运行记录 | 未证明某种切换约束普遍改善任务表现 |
| 状态通信 | Go2 EDU 的 LowState／SportModeState 已实际接收，状态观察与记录可用 | 部分单位、轴向及低层控制期间的使用仍需核对 |
| 电脑原生运动 | 终端 straight／goto／yaw／stop 已实现；现场观察到 0.30 m/s 指令下行走及多次 StopMove 成功响应 | 实际定位误差仍存在；未实现五厘米到点 |
| D435i 数据链 | 相机固定安装；Orin 采集、CycloneDDS 传到电脑，双 IR 约 15 Hz、合并 IMU 约 200 Hz | USB2 限制；IMU 默认标定和间歇告警仍有局限 |
| VINS | 电脑 Humble／CPU 构建、实机初始化及只读动态采集完成 | 第一组严重异常未解决；位置与动态速度准确性未通过 |
| 相机 IMU→机身 | 独立刚体转换代码及 7 项离线检查完成，处理转动杠杆臂速度 | 安装偏移／倾角未测，ROS 在线转换未验证，未接控制 |
| ABS 实机 | 模型、观测接口、硬件生命周期已有代码与离线检查 | 完整策略实机闭环、真实障碍输入、低层停止及交接未验证 |

构建成功、收到消息、轨迹稳定和真实精度是不同层次，不相互替代。VINS 第二、三组位移与人工参考相差约 4–6 cm，测量近似且共享初始化，不能据此宣称五厘米精度。第一组异常保留，见[实机记录](docs/thesis_project/real_go2/README.md)。

## 现在做什么

**当前下一项：定位 Go2 固定机身原点，测量相机安装变换，完成只读机身位姿／速度核对。**

当前 URDF 的 `base` 与 `trunk` 重合，采用 `base`，不以质心为原点。安装参数尚未测量，提供的占位配置会在初始化 DDS 前拒绝运行。具体测量表、转换约定和只读使用方法见[机身坐标转换说明](scripts/go2_vins/BODY_FRAME.md)。

推进顺序：机身转换 → 独立定位只读核对 → 成熟路径跟踪算法离线验证 → 原生 A→B 受控测试 → ABS 观测接入和低层控制准备 → Agile 基础运动 → RA／Recovery 与简单障碍验证。低层停止／交接、真实障碍输入与模型接口按其进入阶段解决；不默认高速运行或重训策略。

路径跟踪优先评估 [Nav2 Humble Regulated Pure Pursuit](https://api.nav2.org/nav2-humble/html/md_nav2_regulated_pure_pursuit_controller_README.html)，尚未接入或运行。它是路径跟踪器，还需要可跟踪路径和相关状态接口。

## 仿真与实机分工

- **R1 仍为唯一 ACTIVE 研究任务。** 最新仿真批次为 Periodic A/B 45 项：43 项有效结果、2 项启动故障保留；到达 20、碰撞 12、仿真限时 11。当前仿真采用几何射线＋Periodic P=0.06 s；未据此确定普遍有效的 B 配置。见[结果](docs/thesis_project/evidence/R1/PERIODIC_AB_45/ANALYSIS_REPORT.md)。
- 实机工作单独记录于 [real_go2](docs/thesis_project/real_go2/README.md)，不改写历史仿真参数、模型或结论。原生运动服务释放后 SportModeState 停更已实测，独立状态来源是 ABS 接入所需工作。
- A1 平地高精度曲线导航在独立 `go2_navigation` 工程规划；本仓库保留[需求记录](docs/thesis_project/real_go2/A1_PROJECT_REQUIREMENTS.md)。共用定位／机身接口，分别验收；五厘米到点是 A1 目标，尚未证实为运行 ABS 的硬性前提。

## 文档入口

| 入口 | 用途 |
|---|---|
| [项目总览](docs/thesis_project/OVERVIEW.md) | 研究目标和范围 |
| [当前状态](docs/thesis_project/CURRENT_STATE.md) | 最新事实、缺口与下一步 |
| [路线图](docs/thesis_project/ROADMAP.md) | 仿真与实机推进顺序 |
| [实验计划](docs/thesis_project/EXPERIMENT_PLAN.md) | 阶段完成条件与实验边界 |
| [文档导航](docs/thesis_project/README.md) | 报告、地图与历史记录 |
| [R1 任务](docs/thesis_project/tasks/R1.md)／[项目入口](docs/thesis_project/designs/R1/README.md) | 切换约束研究与批次结果 |
| [实机入口](docs/thesis_project/real_go2/README.md) | 设备、现场记录和实机限制 |
| [脚本导航](scripts/README.md) | 仿真、观察、原生控制与 VINS 工具 |

## 目录与使用

| 路径 | 内容 |
|---|---|
| `quadruped_ros2_control_humble/` | ROS 2 Humble 控制器、硬件接口和 Go2 描述 |
| `unitree_mujoco/` | MuJoCo 仿真器与场景 |
| `ABS/` | 原论文训练／部署参考；原作者 Go1／ZED／ROS1 代码不能直接作为 Go2 启动步骤 |
| `scripts/go2_readonly_state/` | 独立状态观察器及 Estimator 观察说明 |
| `scripts/go2_native_motion/` | 保留原生服务的终端运动工具及只读服务查询 |
| `scripts/go2_vins/` | D435i 配置、VINS 适配补丁、机身转换及现场说明 |
| `docs/thesis_project/` | 当前研究文档、结果和实机报告 |
| `logs/` | 本地原始数据与分析；大型 bag／视频不随 README 发布 |
| `archive/` | 历史资产，按各自截止日期阅读 |

本仓库依赖外部 Unitree SDK2、ROS 2、MuJoCo、LibTorch；VINS 源码通过固定提交和补丁单独构建。不是任意电脑克隆后即可控制机器人的安装包。原有绝对路径是已用主机示例，使用前按工具文档配置依赖；`/tmp` 构建产物可能丢失，不能当作持久发布物。

- 仿真观看与运行：[脚本导航](scripts/README.md)，默认仿真配置和历史批次设置分别记录。
- 原生运动构建与自检：[工具说明](scripts/go2_native_motion/README.md)；现场命令会发送运动／停止请求，不能当作只读检查执行。
- 相机与 VINS：[跨机数据链](scripts/go2_vins/CYCLONE_FIELD_CHECK.md)、[构建准备](docs/thesis_project/real_go2/20261105_D435I_VINS_FUSION_PREPARATION.md)、[构建恢复](docs/thesis_project/real_go2/20261108_VINS_BUILD_AND_IMU_REVIEW.md)。

机身转换离线检查（不初始化 ROS／DDS，不连接设备）：

```bash
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 scripts/go2_vins/test_body_frame.py -v
```

## 实机控制边界

原生 `SportClient.Move/StopMove` 与 ABS `LowCmd` 是两条不同控制链。当前不提供一键启动 ABS 实机的快捷命令：硬件激活会涉及原生模式释放和 LowCmd；释放后原生位置／速度停更，退出恢复 ai 曾触发起立。

- 终端按键 9／PASSIVE 表示卸力，不保证稳定站立、制动或独立急停。
- StopMove 返回成功、进程退出和机器人实际停稳要分别确认；电脑失联时清理回调不保证执行。
- 低层控制期间不能把原生遥控视为已经验证的独立停止手段。
- 首次通信使用独立只读入口；运动与低层接管按具体现场方案进行，不能照搬历史部署手册。

研究以安全完成、碰撞、到达、超时、推进和耗时评价。切换次数／Recovery 占比解释行为；切换更少不单独代表更安全。S7、T1／T2、R1 各批次与负面结果保留原归属和原口径。
