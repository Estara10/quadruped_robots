# S2-01 短诊断预先参数记录

记录时间：2026-10-10（运行前）

## 目的与场景选择

使用现有 `unitree_mujoco/unitree_robots/go2/scene_obstacle.xml`，目标世界坐标 `(7.0, 0.0) m`。该固定场景包含分布在机器人前进区域周边的静态障碍；7 m 目标避免复用此前 1 m 目标在约 0.5 m 到达半径下很快结束的短任务，可在既定 6 s 仿真运动上限内观察多个真实 RA 决策周期。选择仅为诊断可观察性，不声称该场景已验证能触发 RA 转换；不扫描场景、目标或参数。

## 运行前固定设置

- 候选：`paper_faithful_switch`。
- 进入阈值：`E = -0.05`，来源为本次部署 `abs/config.yaml` 的 `ra_threshold`。
- 退出条件：`RA < E`；记录阈值与进入阈值相同。无滞回、无保持。
- `recovery_hold_steps=30` 保留在 YAML 中但 paper 候选不消费保持计数；本任务不将其解释为 A 组保持。
- 使用同一部署 Agile、RA、Recovery 模型及观测/动作/Recovery 路径；仿真只用模型自然输出，不注入 RA。
- 目标到达半径：`0.50 m`。
- 运动阶段仿真时限：`6 s`；墙钟保护：`50 s`。
- 场景运行最多两次；若来源/配置/接触接口不符、安全 veto 或记录失效则停止，不补跑碰运气。
- 不暂停、不重置、不热更新、不关闭安全检查。
- 每次运行前采集器就绪；controller 在 PASSIVE/固定站立准备阶段必须直接记录四个实际 loaned foot-force interface，并与源码链一致后才发送 RL 命令。

## 运行来源

使用 `quadruped_ros2_control_humble/install/go2_description` 作为本次 overlay。该 prefix 中的 config 文件解析到工作区的 Go2 描述源码；launch shell 将此 overlay 放到 `AMENT_PREFIX_PATH` 首位，控制器包仍取自本次工作区 install prefix。运行记录必须再次确认 package share、两个 config 路径、三模型绝对路径、候选、阈值、目标、唯一 writer 和 session。

## 定向配置消费补充记录（2026-10-10）

原参数记录与 `a5b88cf10d4748d4b15ce1b9566e524e` 保持为 legacy 配置下的行为证据，不追溯改称新配置运行。为核验新 `abs.switching` 入口，另一次只启动记录器、MuJoCo 和 controller 初始化；沿用本计划相同 `scene_obstacle.xml`、模型路径、7.0 m 目标、`E=-0.05` 和 6 s/50 s 上限，但不发送 RL 请求，因此运行上限没有用于运动，也没有策略/运动数据。新配置显式使用 `group: A` 和 `entry_threshold: -0.05`，有效退出阈值从 E 推导，滞回/保持关闭。原始监督器终局与独立来源重验见执行报告，不更改本计划对首次运动诊断的历史描述。
