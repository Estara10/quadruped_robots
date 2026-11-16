# 2026-11-04 遥控停止、LowCmd中断与控制权交还审查

## 结论

在Go2低层控制期间，**目前没有证据证明原生遥控器可在电脑失效时独立停止机器人**。官方Go2用户手册写明：开发者低层编程时遥控器命令无效；官方SDK示例也将遥控按键读入正在运行的电脑控制程序，由该程序解释。项目读取到LowState里的遥控字段，只能说明电脑能读信号，不能说明机器人本体执行了按键。

官方公开SDK示例和本机SDK中没有找到适用于Go2 EDU、本机固件的LowCmd接收超时安全响应保证。Go2在命令中断后是保持最近目标、阻尼、卸力还是切换其他状态，当前标为UNKNOWN。项目最后发布的stop sentinel是低层命令字段的特殊停止值，不是独立急停，也不能替代机器人端通信丢失保护。

正常退出所需“先受控趴伏—确认姿态稳定—停止LowCmd—无动作交回原生遥控”尚无完整可用实现。恢复原生`ai`曾导致非预期起立，而官方资料又提示低层控制时遥控命令无效；所以当前不能宣称退出方案已闭环，也不应开始完整实机控制。机械支撑只能承托机器人，不能证明控制权已交还。

## 场景—停止执行主体—电脑依赖

| 场景 | 遥控信号接收与停止执行主体 | 是否依赖电脑 | 证据及仍未知 |
|---|---|---|---|
| 原生运动服务运行 | 原生服务/机器人控制栈接收遥控并执行按钮动作；本机已在趴伏时实际遥控回趴伏。 | 正常遥控不依赖项目电脑。 | 用户实机记录确认ai恢复后可以遥控趴伏；仅代表本次服务已恢复后的情形，不证明低层控制或服务关闭期间。固件版本未知。 |
| 原生服务释放、电脑正常运行 | LowState仍含遥控字段；项目遥控节点在电脑上解码，发`/control_input`；控制器在电脑上将command 9切换到PASSIVE；硬件插件继续发周期LowCmd。 | **依赖电脑、ROS控制器和DDS链。** | 项目代码：[遥控解码](../../../quadruped_ros2_control_humble/commands/unitree_joystick_input/src/JoystickInput.cpp:62)、[全局停止分支](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/RlQuadrupedController.cpp:76)、[PASSIVE零增益命令](../../../quadruped_ros2_control_humble/libraries/controller_common/src/FSM/StatePassive.cpp:12)。项目实机只读验证了L2+B映射command 9且不发布；未验证真实控制器下电机响应。官方Go2手册称低层编程时遥控命令无效，故不能假定机器人原生执行按键。 |
| 原生服务释放、程序卡死/退出 | 如果电脑控制程序不能解码和处理按键，就没有已知执行主体；退出回调可能不运行。 | 遥控字段曾能进入LowState不等于有执行主体。 | 本项目无独立机器人端按键停止程序。SDK接收数据超时工具仅供电脑程序判断，并非机器人停止机制。Go2端LowCmd timeout的动作未知。 |
| 网线断开/电脑掉电 | 电脑既不能读遥控，也不能发PASSIVE或后续LowCmd。可能的机器人端命令看门狗/原生遥控仲裁未获适用于本机的证据。 | 无电脑控制路径。 | 该EDU本机的LowCmd中断行为、遥控是否能抢占、模式是否自动恢复全部UNKNOWN。不要把本项目DDS超时检测视为机器人端保护。 |

官方Go2用户手册第15页对开发者模式直接写明：“Remote control commands are invalid during low level programming.” [手册PDF页面](https://fccid.io/2A5PE-YUSHU004/User-Manual/User-Manual-7349899.pdf#page=16)。该文件为Go2用户手册的FCC公开版本（© Unitree 2023）；适用范围为手册描述的Go2开发者低层编程，一般性文字不能替代本台EDU固件版本确认。官方[Unitree Go2产品/手册入口](https://www.unitree.com/app/go2/)列有Go2 User Manual。官方[unitree_mujoco说明](https://github.com/unitreerobotics/unitree_mujoco#supported-unitree-sdk2-messages)说明真实机器人关闭内置运动服务后SportModeState不可读，但没有规定LowCmd断流响应。

官方[Go2 low-level SDK例程](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/go2/go2_low_level.cpp)设置`PosStopF=2.146E+9`、`VelStopF=16000`，用20个电机槽初始化字段，并以2ms周期发布示例LowCmd；它演示如何持续发送命令，未说明命令断流后电机的故障策略。官方[Go2用户手册](https://fccid.io/2A5PE-YUSHU004/User-Manual/User-Manual-7349899.pdf#page=16)也未提供该超时动作。两者均不是本机故障测试结果。

## LowCmd中断与stop sentinel

本项目每帧先将20个电机槽初始化为`mode=0x01, q=2.146e9, dq=16000, kp=kd=tau=0`，再覆盖12个有效关节。正常硬件deactivate/shutdown/destructor时，在互斥锁保护下尝试发布一帧这样的sentinel，然后关闭publisher，最后尝试恢复原生模式并查询确认。[硬件源码](../../../quadruped_ros2_control_humble/hardwares/hardware_unitree_mujoco/src/HardwareUnitree.cpp:397)、[退出函数](../../../quadruped_ros2_control_humble/hardwares/hardware_unitree_mujoco/src/HardwareUnitree.cpp:477)

准确边界：

- 该帧包含特殊q/dq值和零PD增益/零前馈力矩，符合SDK例程的命令初始化写法；没有证据表明它在Go2固件中会保持姿态、主动趴伏或触发独立急停。
- sentinel只有在电脑仍运行、DDS publisher有效且`Write`成功时才可能发出。`Write`成功也不证明机器人已接收或电机按某动作响应。
- 关闭publisher只是电脑停止后续发包；对机器人端最后一条命令如何超时处理，未找到Go2特定官方保证。不要声称是阻尼或卸力。
- 官方SDK的`Subscription::isTimeout()`/G1 `lost_connection()`是接收侧软件超时判断；未找到它们为Go2硬件自动停止的实现。本机C++ SDK中未找到Go2 LowCmd断流策略。
- 固件型号/版本、低层电机控制板固件及本台Go2 EDU对应的命令看门狗行为都未确认；对其他Unitree型号、仿真器或第三方实现的机制不外推到本机。

## 当前项目停止路径

### 遥控L2+B和状态异常

`JoystickInput`从`LowState.wireless_remote`解码40字节按键；L2+B得到command 9。`real_go2.launch.py`将遥控节点设成`state_source=lowstate, stop_only=true`。控制器全局分支把command 1或9转到PASSIVE；PASSIVE把12个关节的command interface位置、速度、力矩、Kp、Kd置零。硬件插件仍会持续write这些零增益命令，直至退出硬件生命周期。因此这是一条**电脑端停止路径**，PASSIVE是卸力，不是锁住站立、受控趴伏或物理断电急停。

控制器的射线过期/策略安全veto也仅在控制器线程仍运行时切到PASSIVE。DDS冻结计数器观察关节位置连续不变后切到PASSIVE（阈值100次controller update；头文件注释按500Hz估约200ms，当前Go2配置update_rate=1000Hz时名义约100ms，实际时长取决于controller update是否持续执行）；这也是电脑端判断。若LowState和LowCmd同时因网线故障不可用，电脑即使作出PASSIVE决定也无法保证该命令到达机器人。进程崩溃、强杀、卡死或主机掉电时不能依赖生命周期回调。

### 目前可找到的受控下蹲/趴伏代码

- 有通用`StateFixedDown`：读取当前关节姿态，约1.2秒平滑移动至配置的`down_pos`，随后以配置增益维持；它不是本轮验证的安全动作，默认下蹲位置/增益没有在本台EDU验证。[StateFixedDown.cpp](../../../quadruped_ros2_control_humble/libraries/controller_common/src/FSM/StateFixedDown.cpp:14)、[Go2配置](../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml:25)。command 2可进入该状态，状态完成后仍驻留；command 1再转PASSIVE。当前实机launch的stop_only模式不会下发command 2。
- ABS正常关闭流程command 6先将Agile的机身命令在0.6秒内降为零，等待速度/角速度/姿态和支撑条件稳定后进入`FIXEDSTAND`，确认站立后保持支撑增益；源码中没有把该流程继续转换到`FIXEDDOWN`并确认趴伏的可达阶段。它是受控站立保持检查，不是满足本任务目标的正常退出流程。[StateRL状态机](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp:1760)、[controller closeout](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/RlQuadrupedController.cpp:326)。
- 即使未来以受控代码进入`FIXEDDOWN`，之后停止LowCmd并恢复`ai`仍可能触发原生起立。本次恢复`ai`后意外起立已实测；没有证据说明其他服务名或某个遥控组合能无动作接管。

因此本轮不修改动作代码，不把默认down_pos或增益写进建议现场动作。正常退出目前无已验证的完整路径。

## 操作方案

### 目标正常退出：当前是待实现/验证方案，不是现成功能

1. 前提为另外完成硬件接口/观测来源审核、可靠物理支撑、清空活动空间和可见遥控器。支撑只防止下落，不等于控制权交接。
2. 若由用户另行授权未来控制试验，应在LowCmd仍稳定发送时先用经过专项确认的安全动作减速并下蹲；用新鲜LowState中关节位置/速度、姿态与可见机身高度连续确认落地稳定。任何反馈过期、姿态异常、动作未完成或支撑异常，保持外部支撑，停止进一步控制尝试，不发送未经验证的关节目标。
3. 在确认物理趴伏后，当前代码可发送一帧stop sentinel并关闭LowCmd publisher；这不会形成已知的机器人端安全状态。需先向Unitree确认适用于该固件的“退出低层控制、不自动起立并允许遥控接管”的官方流程。恢复`ai`不能作为默认最后步骤，因为本机实测会起立且可延迟；单纯删去SelectMode恢复也没有证明遥控因此有效。
4. 只有按官方/本机固件明确流程完成交接后，才分别确认原生服务状态、遥控实际有可见响应、LowCmd确实停止及机器人姿态稳定。任何接口返回码或机械支撑本身都不满足物理交接确认。

当前不能执行第2–4步：down动作的真实安全性和服务恢复/遥控交接语义都未闭合。不得将PASSIVE当作趴伏，也不应现场试运行未经确认的`FIXEDDOWN`参数。

### 故障处置

| 故障 | 电脑仍可运行时的代码行为 | 还能依靠什么/限制 |
|---|---|---|
| 输入状态过期、射线过期、推理异常 | 相关软件分支请求PASSIVE；若控制循环及LowCmd通信正常，后续周期继续发零增益命令。 | 这是电脑端响应，不等同物理停止；若新鲜状态丢失，不能盲目发趴伏动作。保持现场支撑、停止新任务并记录状态。 |
| 控制进程崩溃/强杀/线程卡死 | 析构和停止回调不保证执行；线程卡死时也不能保证周期PASSIVE。若进程仅崩溃，DDS publisher销毁可能发生，但机器人对命令中断的响应未知。 | 无已证明的本体stop，也无已证明的遥控独立接管。只可依赖预先设置的机械承托防止跌落，不能宣称停止/交接完成。 |
| 网线断开/电脑掉电 | 无法解码按键、发PASSIVE、stop sentinel或恢复服务；DDS超时计数器也无法令远端接收命令。 | LowCmd超时反应和遥控仲裁未知；机械承托仅承托。现场避免在无支撑情况下进入低层控制，直到本机失联反应有依据。 |

发生故障时不要求先趴伏——只有电脑、状态和LowCmd发送链都可靠时，正常受控退出才可能尝试动作；否则不应追加运动命令。遥控在低层模式下独立停止的能力，根据官方手册不能假定有效。

## 下一次现场验证：唯一建议任务，尚不执行

**先向Unitree官方支持核对本机Go2 EDU固件下两个事实：LowCmd停止更新/断链后的电机端响应，以及低层编程状态下原生遥控能否中断控制并执行Down。** 提供设备序列号、EDU硬件版、Go2 App/机器人固件和电机控制固件版本，索取适用的LowCmd timeout、Stop sentinel、release低层控制及无需站立的遥控接管说明。目的在于决定是否存在安全的失联和正常交接程序；本次不操作设备。

完成条件：官方答复或本机固件对应的正式资料明确列出超时后的电机状态、触发条件/时间、遥控接管优先级以及退出低层控制而不触发起立的操作步骤。若官方无法确认，则状态保持UNKNOWN，不安排拔线/杀进程现场故障试验，继续NO-GO。只有获得明确流程并经用户另行审阅后，才设计有支撑的现场确认；本文件不授权连接或测试。

## 本轮范围

新增本离线审查和本轮必需的README/历史记录链接澄清；没有修改代码、控制配置、模型或仿真设置。未连接Go2、未初始化实机DDS、未调用模式接口、未启动实机控制器、未推理模型、未发LowCmd。R1唯一ACTIVE任务和所有仿真结果不变。离线进程未启动；本轮进程清理无需执行。
