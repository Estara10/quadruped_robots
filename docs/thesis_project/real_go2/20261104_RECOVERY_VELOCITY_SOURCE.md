# Recovery 线速度来源显式选择

## 问题与修改

原 `StateRLRec::runModel()` 仅用 odom state interface 数量判断来源：达到 6 项就读取 `[3:6]`，否则在 Estimator 启用时读取其世界系速度。实机硬件描述仍导出 6 项接口，而 SportModeState 停更，因此“接口存在”不能代表其速度仍更新。

新增控制器参数 `recovery_velocity_source`，只作用于 Recovery：

- `odom`：严格读取已有 odom 速度 `[3:6]`；接口少于 6 项或速度非有限时阻止进入 Recovery。
- `estimator`：严格读取现有 Estimator 的世界系速度；Estimator 未启用/不存在或速度非有限时阻止进入 Recovery。即使 odom 仍有 6 项，也不会回退。

来源不可用时，控制器在状态转换前保持当前状态并限频报错；Recovery运行中来源变为不可用时，使用已有 `safetyVeto` 路径停止策略执行，不向模型补零或读取另一来源。来源选择后仍只经过原有一次 `quatRotateInverse(base_quat, lin_vel_world, framework)`，Recovery输入合同、顺序、单位缩放和接触阈值均未改。

Estimator的“可用”检查目前是已启用、对象存在且输出有限；Estimator没有对外提供更新时间/有效性标志。源码中模型质量为零时更新会直接返回并保留上次状态，因此有限性本身不能证明每次更新都成功或速度准确。后续若实际Go2模型配置无效，现有控制器模型创建/更新路径需另行检查；本轮不扩大为Estimator状态机改造。

控制器参数默认值为 `odom`，Go2 仿真配置文件未改，现有仿真行为保持不变。实机候选覆盖文件为 [RECOVERY_VELOCITY_SOURCE_CANDIDATE.yaml](RECOVERY_VELOCITY_SOURCE_CANDIDATE.yaml)，当前未被实机 launch 引用，也没有启用实机策略。

## 数据流与证据

配置在 [Go2 `robot_control.yaml`](../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml:150) 声明；参数解析与传入 Recovery 在 [RlQuadrupedController.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/RlQuadrupedController.cpp:477) 和该文件的 Recovery 构造处；严格来源选择位于 [RecoveryVelocitySource.h](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/include/rl_quadruped_controller/FSM/RecoveryVelocitySource.h)；接口读取及唯一坐标变换位于 [StateRLRec.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRLRec.cpp:428)。

原先：`odom接口数>=6 → 读odom；否则读Estimator（不可用时遗留零向量）→ world-to-body`。

现在：`明确参数 → 只读所选来源并检查数量/启用状态/有限值 → 缺失则阻止进入或触发现有Veto；有效则 world-to-body 一次`。

Agile/RA 仍有独立 odom 速度分支，位置导航仍读 odom `[0:2]`；本修改不替换这两条路径。故即使 Recovery 候选选择 Estimator，也不能据此认为 RA 使用了 Estimator 速度或完整导航获得了独立世界位置。

## 离线验证和限制

控制器目标在 `build_s6_05/rl_quadruped_controller` 离线构建通过。临时 C++ 定向 selector 检查覆盖：6项 odom 存在时 `odom` 原值通过；同样6项 odom 存在时 `estimator` 选择不同的 Estimator 向量；被选择来源缺失或含 NaN 时拒绝，不回退；未知参数值拒绝。YAML 检查确认默认仍是参数默认 `odom`，实机候选覆盖为 `estimator`。90°偏航数值检查与Recovery现有逆四元数公式一致，且源码确认该速度只经单次 `quatRotateInverse` 写入 Recovery 的 `lin_vel`；变换实现未修改。

既有30秒趴伏静止只读观察记录 [RESULT.md](../../../logs/real_go2/20261104_163622_estimator_static/RESULT.md) 显示 LowState 约500 Hz、Estimator 约200 Hz，输出有限且静态稳定。它没有动态参考真值，不能证明运动速度方向、幅值、延迟或足滑误差；本候选不启用 Recovery，也不证明低层运动条件具备。

下一步现场验证需单独安排：在原生服务保持运行、机器人静止且无人接管控制的前提下，先只读观察 Recovery 选用来源的连续更新与静态数值；任何启用 Estimator 进行 Recovery 的讨论都还需在有充分机械支撑和人工停止措施下另行评估动态精度与策略风险。
