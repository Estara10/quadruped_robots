# 2026-11-04 Go2 离线观测依赖与退出审查

## 结论

当前 Go2 EDU 的项目闭环不能在原生运动服务关闭后直接运行：实机已确认 `SportModeState` 停更，而 StateRL 的导航位姿及 Agile/Recovery 线速度路径依赖该接口或尚未启用/验证的估计器。LowState仍更新，足以提供IMU、12关节和四路足力，但不能单独提供已验证的世界位置和机身线速度。不能以停止更新的SportModeState、默认零值或IMU加速度积分补齐。

退出也不是“恢复 ai 即交还”。本次 ai 恢复后出现起立，故任何项目控制测试均保持NO-GO。离线代码已具备正常生命周期退出顺序（stop sentinel、关闭LowCmd发布器、恢复之前记录的原生模式并查询）；这仍不能保证末帧送达、原生恢复无动作、电脑崩溃时清理执行或机器人保持支撑。

## 实际观测依赖

代码位置均相对仓库根目录。Agile模型配置为61维；Recovery为49维。RA共享Agile观测构造，RA不是独立的传感器来源。当前Go2真实launch将`runtime_mode=hardware`传给控制器。

| 数据 | 实际用途及约定 | 来源/单位/更新 | 原生服务释放后的证据与状态 |
|---|---|---|---|
| 世界位置 x/y | StateRL用于起点、目标方向/路径跟踪、到达与导航指标，不进入Agile或Recovery策略张量；RA也不直接用位置。`StateRL.cpp`导航段读取里程计接口[StateRL.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp:2236)。 | 硬件插件取SportModeState.position[0:3]映射到6项里程计接口；预期米，但坐标系/原点语义由SDK服务定义，项目没有独立校准。[HardwareUnitree.cpp](../../../quadruped_ros2_control_humble/hardwares/hardware_unitree_mujoco/src/HardwareUnitree.cpp:313) | 实测SportModeState停更；恢复后位置归零，不能跨恢复相减。当前缺少释放期间可用且验证的世界定位。导航不可用。 |
| 机身线速度 | Agile张量本身不含线速度；StateRL用它将目标变换到机身坐标并用于运行记录/诊断。RA继承同一状态路径但模型观测不含线速度。Recovery的49维观测含机身坐标线速度。[StateRL.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp:2220)、[Recovery组装](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRLRec.cpp:433)。 | 优先取SportModeState.velocity的三分量，再用IMU四元数旋转到机身坐标；代码未注明SDK速度字段单位/坐标约定，需接口确认。替代路径为`Estimator::getVelocity()`，其输出经相同旋转按世界速度处理。 | SportModeState实测停更。StateRL在缺接口且估计器关闭时会保留初始化零速度；不能将其当真实观测。实机launch没有提供定位来源证明。Recovery闭环因此不准入。 |
| 角速度 | Agile/RA/Recovery策略观测均使用三轴陀螺仪；StateRL按配置原序读取。Agile配置scale=1，训练/SDK单位兼容性尚需接口文件或实测确认。[StateRL.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp:2217)、[abs config](../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml:9) | LowState IMU gyroscope[0:3]；硬件接口保留DDS分量顺序。通常按rad/s使用，但插件本身没有转换。 | 实机确认LowState释放期间持续更新；本次静态接口检查已读取。字段可用，训练语义/频率须在未来受支撑只读阶段补核。 |
| 姿态 | Agile/Recovery由IMU四元数生成重力方向观测；导航从其算yaw。不同framework分支决定接口四元数重排。[StateRL.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp:2122) | LowState IMU四元数；`isaacgym`分支将接口wxyz重排为策略xyzw，`isaacsim`保持接口顺序。归一化/坐标语义由设备SDK提供。 | LowState实测持续更新；字段可读，IMU与模型姿态约定仍须正式接口核对。 |
| 关节位置/速度 | Agile、RA、Recovery策略输入；12维。插件按配置中的joint遍历映射电机index，控制器顺序FR,FL,RR,RL；策略配置声明训练顺序`ros1_fl_fr_rl_rr`，观测适配器重排。位置、速度尺度分别1、0.2。[StateRL.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp:2139)、[观测契约](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/AbsObservationContract.cpp:1) | LowState motor q/dq；插件无单位换算；Go2接口通常为rad和rad/s，但需以SDK定义与当前硬件读取确认。 | LowState实测持续更新。本次无项目控制器或模型运行，因此只证实消息持续，不代表控制使用时的周期与训练接口匹配。 |
| 足力/接触 | Agile和Recovery策略用四足接触（力阈值1.0），Recovery接触做时间平滑；终止/支撑相关逻辑也读取足力。策略顺序从控制器FR,FL,RR,RL映射到训练策略顺序。[HardwareUnitree.cpp](../../../quadruped_ros2_control_humble/hardwares/hardware_unitree_mujoco/src/HardwareUnitree.cpp:300)、[观测契约](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/AbsObservationContract.cpp:22) | LowState.foot_force四值，无插件换算；阈值按配置称N，传感器标定/单位未在本次离线代码中证明。 | LowState保持更新，但本次没有验证实机控制中的接触阈值及支撑检测。可读不等于已校准。 |
| 其他策略量 | Agile额外含命令、滚动timer、前向11束射线、关节历史动作；Recovery含命令和历史动作，不含timer/射线。Recovery命令来自RA优化或遥控量；停止链遥控输入读LowState遥控字段。[配置](../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml:1)、[Recovery配置](../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/rec/config.yaml:1) | 射线由外部感知/预测进程共享内存输入；不是硬件插件状态。遥控映射L2+B→9已只读确认，但项目低层控制的实际停止效果未验证。 | 不受SportModeState单项停更影响；射线现场可靠性及遥控停止效果均未完成实机闭环验证。 |

### 状态估计器核查

仓库有`controllers/rl_quadruped_controller/src/control/Estimator.cpp`：基于IMU、关节运动学和足接触约束的浮动基座Kalman估计实现，不是单纯加速度积分。控制器在四个足力接口存在时自动创建并更新Estimator；但Recovery的`getState()`优先检查里程计接口数量是否至少6项，真实Go2描述导出了这6项，因此SportModeState即使停更也会遮蔽Estimator输出。仅当接口数量不足时才取Estimator速度。位置导航仍无条件读里程计，Estimator位置没有接入导航。已发现的具体差异与候选接入方案见[Estimator专项分析](20261104_ESTIMATOR_RECOVERY_VELOCITY_REVIEW.md)。

| 分类 | 结论 |
|---|---|
| 已有且实机验证 | LowState更新；其中IMU、关节、足力和遥控字段可被只读程序接收。 |
| 已有、仅代码可见 | 基于接触的Estimator速度估计；当前未证明Go2配置启用，也未实机验证兼容性/漂移/重置。 |
| 候选而非事实 | 外部视觉/激光定位或官方独立里程计服务；需先确认设备、接口、时间戳、坐标系和实测连续性。 |
| 当前缺失 | 原生ai释放期间持续、与训练/导航坐标兼容的位置及线速度来源。 |

## 退出与停止链

### 代码当前行为

硬件`on_init`初始化DDS并订阅状态；`on_activate`在非loopback模式下先查询并释放原生运动模式，成功后才初始化并创建LowCmd发布器。查询/释放失败则激活报错；发布器初始化失败会关闭发布器并尝试恢复原模式。[硬件源码](../../../quadruped_ros2_control_humble/hardwares/hardware_unitree_mujoco/src/HardwareUnitree.cpp:100)

正常deactivate/shutdown/析构进入`stopAndRestoreMotionMode()`：与`write()`互斥，先尝试发一帧所有电机stop sentinel，再关闭发布器，之后恢复进入前记录的模式并查询。sentinel是Unitree协议停止值，前提是LowCmd publisher仍可用且DDS发送成功；不等于制动、保持站立或物理急停。正常RL控制器停用本身会清零策略线程/输出，但硬件可能仍激活，须走硬件停用链。StateRL的PASSIVE会撤掉增益/力矩，项目记录已明确它是卸力，机器人可能屈膝/下落。

### 必须采用的现场退出方案（仅供后续单独审阅，不授权启动）

1. **进入前先落实物理支撑并安排人工接管验证**：地面清空，机械支撑只提供物理承托，不证明控制权已交还或遥控器此时有效。服务关闭或电脑故障时遥控器能否接管尚未验证；现场人员需持遥控器并能观察响应。不能以“空中自由站立”验证退出。
2. **正常结束/遥控停止/状态过期**：遥控L2+B映射只在项目输入层请求PASSIVE；状态过期或控制异常时当前软件路径也可能进入PASSIVE。操作员先确认机器人由支撑承托，再请求控制器停止，并通过生命周期停用硬件；代码最后尝试stop sentinel，随后停发LowCmd。不得把PASSIVE或停止发包叫作保持站立。
3. **再处理原生服务**：硬件退出会尝试恢复之前的ai模式，但实机已证明恢复ai可能延迟且触发起立。恢复动作可能有数秒级延迟（本次从release到查询确认约6.487秒）；低层编程时原生遥控是否能停止/趴伏没有本机验证，Go2用户手册还指出低层编程时遥控命令无效。操作者必须保持支撑，不能预设遥控能立即趴伏。恢复确认只说明服务状态，不是物理无动作交接成功。若不希望触发ai，必须先由官方/本机固件资料确认替代人工恢复步骤；不能简单删掉恢复调用后就称交接已安全。
4. **初始化/激活失败**：确认LowCmd publisher尚未开放或已关闭；支撑继续保持。检查原生模式查询结果并由操作员按现场流程接管，不启动策略重试。
5. **进程中断/电脑掉电/网络中断**：不能保证析构、stop sentinel或恢复调用运行。遥控器在服务关闭或电脑失联时能否实际接管并趴伏尚未验证；不可将“操作员立即按遥控”写成已有保证。机器人须一直由机械支撑承托，直到后续专项现场验证以可见物理响应确认人工控制有效。

无论正常退出还是故障退出，需分别确认：LowCmd是否停止发布、机器人是否仍由支撑稳定承托、原生服务是否恢复、遥控是否已实际接管。一个确认不能替代其他项。

## 最小修复和验证状态

本轮未把Estimator接入Recovery；原生服务关闭时其足接触判据存在趴伏假接触风险，且旧接口存在性会遮蔽其速度输出。控制器的零速度初始化和硬件未就绪时的兼容性行为不能作为实机数据来源。Recovery速度候选与导航位置缺口分开判断：可单独在支撑条件下观察/验证速度估计，不要求先解决世界定位；完整导航仍需有效位置。不得调整100ms阈值、观测维度或顺序。

本轮无DDS初始化、设备连接、ROS/实机launch、模型推理、LowCmd发布或运动。未运行任何新进程；机器人保持关机。代码构建依据为准备说明中已完成的离线构建记录，不将其称作实机安全验证。

## 下一项具体任务

**只读确认原生ai服务释放时仍可连续提供的独立定位/线速度来源（含坐标、时间戳、频率和重置语义），同时拟定带机械支撑的人工退出验证步骤；在两者审阅通过前不启动项目低层控制。**
