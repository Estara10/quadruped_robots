# 2026-11-04 Estimator 能否作为 Recovery 速度源：离线专项审查

## 结论

**值得把仓库现有Estimator作为Recovery机身线速度的候选来源，但当前不应直接启用或宣称可用。** 算法输入来自LowState衍生的IMU、关节运动学和足接触，不依赖SportModeState；它的速度状态按实现处在世界系，Recovery随后用当前IMU姿态旋到机体系，和已记录的Recovery观测契约在坐标变换、单位缩放和字段位置上相符。

有两个具体阻断仍待处理：

1. 趴伏记录中的四路足力为14–16，而Estimator阈值固定为1；Estimator会把趴伏时四足都当作接触。这样的接触约束可能把趴伏误建模成支撑，速度偏差大小目前未知。
2. 硬件描述仍导出6个里程计接口。Recovery当前代码先按接口数量选择SportModeState速度；它不判断样本新鲜度。原生服务停更后，这条优先路径会继续读取旧接口/过期值，Estimator不会自然接管。

因此本轮不修改生产代码、不改变模型或仿真路径。下一步应先用只读、无LowCmd程序把现有估计器离线/实测链路在可靠支撑下观测，并先解决趴伏接触阈值的证据问题，再决定是否提供一个硬件专用速度源选择项。

## 调用链：构造、输入更新与速度消费

| 环节 | 实际代码行为 |
|---|---|
| 创建条件 | `LeggedGymController::on_init`在`foot_force_interface_types_.size()==4`时设置`enable_estimator_=true`并构造Estimator；硬件仿真控制配置含四路接口，因此并非“当前未启用”。[RlQuadrupedController.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/RlQuadrupedController.cpp:462)、[控制器状态接口配置](../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml:1) |
| 机器人模型 | `on_configure`收到robot_description后创建`QuadrupedRobot`；它按FR、FL、RR、RL四条腿建KDL链，并由URDF link inertia求质量。[RlQuadrupedController.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/RlQuadrupedController.cpp:486)、[QuadrupedRobot.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/robot/QuadrupedRobot.cpp:12) |
| 更新调用 | 控制器每次`update()`在FSM前先用关节接口更新运动学，再调用Estimator更新。控制器配置update_rate=1000Hz，模型dt由该rate设为1ms；策略配置decimation=4，因此Recovery推理约250Hz（这只是配置推算，实机控制周期尚未测）。[主循环](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/RlQuadrupedController.cpp:212)、[robot_control.yaml](../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml:1)、[Recovery config](../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/rec/config.yaml:1) |
| 算法输入 | 固定dt线性Kalman状态含位置3、速度3、四足世界位置12；加速度经姿态旋转并加重力作过程输入，足端相对机身位置/速度由Go2 URDF运动学及关节q/dq得到；四路足力改变各足过程/观测协方差，接触时融合“足端世界速度为零”约束。源码：[Estimator.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/control/Estimator.cpp:155)、[Estimator.h](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/include/rl_quadruped_controller/control/Estimator.h:106)、[QuadrupedRobot.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/robot/QuadrupedRobot.cpp:115)。 |
| IMU约定 | 驱动接口顺序为orientation.wxyz、gyro.xyz、linear_acceleration.xyz；Estimator直接用四元数构造旋转，再执行`R * acceleration + [0,0,-9.81]`。恢复后静止趴伏样本的加速度z约+9.5m/s²，与该重力补偿方向相容；这只是一段静态记录，不证明旋转矩阵四元数语义全姿态正确。[HardwareUnitree.cpp](../../../quadruped_ros2_control_humble/hardwares/hardware_unitree_mujoco/src/HardwareUnitree.cpp:282)、[mathTools.h](../../../quadruped_ros2_control_humble/libraries/controller_common/include/controller_common/common/mathTools.h:84) |
| 关节顺序 | 插件输出/controller接口配置是FR,FL,RR,RL，每腿hip/thigh/calf；Estimator的`QuadrupedRobot`按同一FR、FL、RR、RL创建链，并以连续3关节槽读取q/dq。实现有一致的顺序路径；真实硬件motor-index映射与URDF通道仍需要历史接口验证，当前仅读码可见。[robot_control.yaml](../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml:23)、[HardwareUnitree.cpp](../../../quadruped_ros2_control_humble/hardwares/hardware_unitree_mujoco/src/HardwareUnitree.cpp:287) |
| 输出 | `getVelocity()`返回`x_hat_[3:6]`；过程输入使用姿态旋转到世界系，静止接触约束也在世界系，故源码意图为世界系m/s。Estimator没有单位换算；依赖IMU加速度m/s²及KDL关节量rad/rad/s这些输入语义正确。[Estimator.h](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/include/rl_quadruped_controller/control/Estimator.h:35) |
| Recovery消费 | `StateRLRec::runModel`优先取6项里程计接口中的速度[3:6]；只有接口数量不足时才取Estimator世界系速度。随后`quatRotateInverse(base_quat, lin_vel_world, framework)`旋到机体系，`computeObservation`把该3维放在Recovery 49维输入起始处，scale=`lin_vel_scale=1.0`。`framework=isaacgym`四元数按策略xyzw处理。源码：[StateRLRec.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRLRec.cpp:430)、[观测组装](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRLRec.cpp:260)、[contract](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/AbsObservationContract.cpp:25)、[config](../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/rec/config.yaml:34)。 |

### 初始化、重置和失效

Estimator构造时`x_hat_`置零、协方差设大；没有首次姿态/足位初始化，也没有显式重置接口。每个控制周期更新，但没有输入时龄、非有限值、单调时钟间隔或创新量失效标记。若URDF模型尚未配置，控制器提前返回，不运行FSM；模型质量为零时Estimator本身直接return，保留上次状态。重新进入Recovery不会重置Estimator，也不意味着速度已重新校准。源码：[Estimator.cpp构造](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/control/Estimator.cpp:12)、[update](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/control/Estimator.cpp:155)、[控制器调用](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/RlQuadrupedController.cpp:212)。

另一个细节：虽创建条件只检查4个足力接口，但更新函数直接索引前4项；当前Go2配置满足该条件。StateRL/Rec将`enable_estimator_`保存在控制器通用组件中，但线速度选择实际被6项odom接口数量优先级决定，而不是启用开关。

## Recovery训练契约对照

项目已有S1-02模型接口报告将Recovery观测合同记为49维：`lin_vel(3), contact(4), ang_vel(3), gravity(3), command/twist(3), dof_pos(12), dof_vel(12), previous_action(12)`。其中线速度是**机体系m/s**、scale 1.0；它占Recovery输入第0–2项。该合同基于部署配置和训练/导出源复核记录，不是本轮新训练或外部推断。[S1-02执行报告](../evidence/S1-02/S1-02_EXECUTION_REPORT.md:15)

| 项 | Recovery需求 | Estimator路径 | 判断 |
|---|---|---|---|
| 坐标系 | body frame | world velocity经当前姿态逆旋转到body | 代码路径一致；仍需实机转向/运动数据验证姿态与速度方向。 |
| 单位 | m/s | 无显式缩放；加速度补重力、关节运动学、固定秒dt | 预期一致，但依赖LowState与URDF单位语义。 |
| 位置/排列 | Recovery张量第0–2项 | `abs_observation::recovery`首拼接`in.lin_vel` | 一致。 |
| 更新/采样 | 控制周期更新、策略decimation 4 | Estimator默认跟随controller update，随后Recovery每4周期取一次 | 设计上匹配；真实1kHz周期稳定性未知。 |
| 足接触 | 训练观测的足接触与实际阈值/顺序须一致 | Estimator用原始4个力值与固定阈值1比较；策略接触另使用配置阈值1和FR/FL/RR/RL→训练顺序映射 | 风险未闭合。趴伏时四足均高于1，说明趴伏下Estimator把四足判接触，未证明运动支撑时误判率。 |
| 速度准确度 | 移动时可信机身速度 | 估计器设计上以足端静止约束校正IMU积分 | 尚无本机Go2慢走对照数据；不能从静止或仿真精度推出实机准确。 |

可供继续核查的原始部署资料只覆盖Recovery观测顺序/缩放；没有找到绑定到本模型checkpoint的训练run记录证明其他速度坐标约定。无需另查官方服务：本候选基于仓库已有算法与已记录的模型合同。

## 已有趴伏记录对阈值的约束

2026-11-04恢复后人工遥控趴伏的5秒只读记录中，50帧LowState均保持趴伏姿态；样本足力示例为`[14,16,16,16]`（记录单位/标定没有独立证明）。Estimator阈值是硬编码默认1，判断条件为`force < threshold`即非接触，否则接触。因此这批趴伏样本下四腿都会被视作接触。它揭示接触约束可能不匹配卸力趴伏，但不能证明阈值应改为何值，也不能证明站立或慢走时阈值失效。[原始记录](../../../logs/real_go2/20261104_mode_handoff_execute/recovery_readonly_state.jsonl)、[Estimator.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/control/Estimator.cpp:169)。

阈值来源为构造默认值1；控制器目前以默认构造Estimator，没有将配置`feet_force_threshold`传给Estimator。控制器另有20.0N的支撑状态门槛，但那是shutdown支撑判定，不能据此推断Estimator门槛应为20。[Estimator.h](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/include/rl_quadruped_controller/control/Estimator.h:19)、[controller header](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/RlQuadrupedController.h:110)。

## 方案、边界和离线验证

### 建议的最小接入（本轮不写代码）

先为StateRL/StateRLRec加入一个**实机专用、默认关闭**的速度来源配置，例如明确选择`native_odom`或`estimator`；仿真仍强制沿用现有odom路径。选择estimator时要求Estimator已创建、robot_description模型可用、接口输入有限且持续更新；将world速度经现有`quatRotateInverse`变为body速度后再按Recovery现有输入顺序和scale=1送入，绝不做零值回退。该选项只替换Recovery所需速度源；Agile/RA原模型观测不因此增删字段。独立导航位置仍沿用当前定位接口，所以Estimator速度候选不能解决世界位置/目标到达判断。

实现前还需决定是否让Estimator速度仅用于Recovery，或同时用于RA的线速度诊断/twist优化；项目代码中RA构造确实包含`lin_vel`，虽然已部署RA网络输入与历史报告需再核对后才能确认其会否受影响。本轮不擅自扩展到RA。

### 不发控制命令的观察方式

后续如用户授权，只使用独立只读采集程序订阅LowState并加载本地Go2 URDF/现有Estimator，记录IMU、q/dq、各足力、接触判断、Estimator世界速度、姿态旋转后的body速度及单调时间间隔；程序不得创建MotionSwitcherClient/LowCmd publisher、不得启动实机controller或加载策略模型。先机械支撑、机器人静止趴伏观察初始化和漂移，仅能验证输入更新、算法不发散及静止输出大致稳定；趴伏接触失配必须单独标记。静止数据不能证明运动时幅值、符号、延迟和侧向分量准确，也不能代替低速受控走动对照。

### 本轮离线检查

- 手工追踪并核对构造→robot_description→每控制周期更新→Recovery输入路径；确认真实launch的接口契约中6项odom仍存在，Estimator因此被旧来源遮蔽。
- 从源码检查`quatRotateInverse`路径及Recovery concat、scale；从S1-02既有合同核对训练侧body m/s、scale 1、输入顺序。
- 以2026-11-04趴伏记录的足力值与阈值1做直接比较：50帧均高于1（示例14–16），故该姿态下Estimator四足接触判定为真。这个比较仅揭示状态语义风险，不是估计速度精度验证。
- 未改C++、模型、配置或仿真；没有启动ROS/DDS/观察程序，因此不需要构建。无设备连接、模式切换、LowCmd、策略推理或运动。

## 影响与下一步

不影响仿真工作线；没有改变共享控制器或仿真默认行为。Recovery线速度Estimator是**有条件候选**，并非当前可用实机能力；启动配置仍被SportModeState优先级遮蔽，足力阈值对趴伏状态明显不适配，动态误差未知。导航世界位置仍缺少独立来源；但这不阻止在支撑条件下单独评估速度估计，也不妨碍纯离线接口分析。

**唯一推荐下一项：**获批后制作并运行一次“只读Estimator静止观察”——只订阅LowState，使用现有Go2 URDF和Estimator记录原始输入、阈值接触标志、world/body估计速度及时间间隔，不加载策略、不启动项目硬件插件、不建立LowCmd/MotionSwitcher客户端。完成条件：确认接口映射与时间周期，观察是否出现非有限值/明显漂移，并明确趴伏接触造成的估计表现；只有通过该观察后，再单独讨论有支撑的低速运动精度验证。该静止观察不宣称验证动态精度或完整导航。
