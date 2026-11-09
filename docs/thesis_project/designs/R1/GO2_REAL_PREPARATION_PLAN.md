# Go2实机连接前准备计划

> **历史准备计划，按 2026-11-02／03 当时条件阅读。** 2026-11-09 已确认 Go2 EDU、Orin、D435i，完成首次通信、原生运动和 VINS 部分只读实测；原 12 项仿真射线排程已续跑完成。当前硬件模式释放移至激活阶段，退出恢复原模式，不能再按下文旧初始化／SelectMode 描述理解现行代码。本文件保留当时评估，不是现场启动手册；最新计划与事实见[实机入口](../../real_go2/README.md)、[机身安装转换](../../../../scripts/go2_vins/BODY_FRAME.md)和[当前状态](../../CURRENT_STATE.md)。完整 ABS 实机仍未通过。

状态：R1唯一ACTIVE。阶段1接口只读核对、阶段2最小计时/状态新鲜度适配和离线检查已完成。阶段3模型/共享内存接入及静态比较完成；后续CUDA诊断与一次独立scene_flat动态核验也完成，独立动态项ARRIVED且419个任务窗周期均消费有效预测射线、无200ms过期中止，但安全完成为UNKNOWN，收尾另记录DDS超时FATAL。原阶段4冻结12项仍停在两项几何ARRIVED、一项预测`SYSTEM_SAFETY_ABORT`，其余9项未启动；不把独立核验计入原批次。Go2只读观察入口仍等待实机验证。本文件不代表实机可用或普遍实时性已通过。

本轮状态、原因定位、独立运行/动态核验、静态误差及原12项停止位置见[核验报告](../../evidence/R1/GO2_RAY_PRED_12/STATUS_REPORT.md)；原始排程及启动/清理记录见[批次计划](../../../../logs/09_R1_switch_constraints/go2_ray_pred_12/20261102_r1_go2_ray_pred_12/run_plan.json)。

## 当时结论（历史）

仓库已有Go2 LowState/LowCmd的ROS 2硬件接口、ABS策略与恢复策略模型、关节/IMU/足端观测转换和MuJoCo几何射线读取。现有实机硬件插件在非`lo`网络上初始化时会释放原生运动模式；启用控制器后硬件`write()`会持续发布LowCmd。因此当前启动链不是只读状态入口，首次连接前必须先准备不激活控制权的状态读取方式。

独立的[只读状态观察器](../../../../scripts/go2_readonly_state/README.md)已通过SDK2构建及loopback无消息等待/退出检查；这仅验证程序可启动，不代表机器人消息可达或字段物理含义正确。实机型号/版本、计算设备及架构、实际深度相机和部署依赖版本仍待用户提供或设备确认。本轮已加入运行模式显式选择、硬件状态时龄标记及预测射线版本2共享内存写入。预测模型选择现存的2026-06-11 soft-safety微调权重，不使用缺失的默认路径、不自动fallback。原第3项CPU推理长尾导致安全中止；随后显式CUDA和子进程环境修正后，独立60秒检查及一次scene_flat预测动态核验均未观察到200ms帧间断，controller确实读取预测源。动态核验安全完成仍UNKNOWN，原12项剩余9项仍暂停。

后续新增仿真统一使用Periodic，暂用P=0.06仿真秒；这只是后续运行配置，不修改部署默认配置，也不声称与ABS原作者的更新调度完全相同。Frozen代码和历史A/B/C/D、E/X/H结果保持原状，几何射线历史结果不重新解释为预测射线结果。

## 实机接口准备表

| 项目 | 当前代码来源与行为 | 已有验证情况 | 实机缺口 | 下一步 |
|---|---|---|---|---|
| 设备/部署环境 | `hardware_unitree_mujoco`使用Unitree SDK2 DDS LowState/LowCmd；ROS 2控制器基于ros2_control。旧启动脚本引用ROS Humble、`unitree_sdk2`和CPU LibTorch 2.0.1路径。 | 只能确认仓库代码和脚本声明这些依赖；没有核对设备上实际安装情况。旧手册写有`enp7s0`、`domain=0`，不能证明当前设备一致。 | Go2具体型号/固件、外部电脑或机载运行、CPU/OS架构以及SDK/ROS/PyTorch与LibTorch的已安装版本均待用户提供或现场确认。 | 先确定设备与主机清单，再核对对应SDK及依赖。 |
| 关节位置/速度 | `HardwareUnitree::read()`从LowState motor `q/dq`读取；controller order为FR、FL、RR、RL，每腿hip/thigh/calf。StateRL按12维输入策略。 | MuJoCo接口和历史策略运行复用该路径；仓库有motor index映射日志。未验证用户手中实机的索引/符号/单位。 | 现场确认消息频率、关节顺序、弧度和停更行为。 | 只读状态采样时记录名称、值与时间戳，对照实体电机映射。 |
| IMU | LowState提供四元数、gyro、accelerometer；硬件层保存四元数为w,x,y,z。StateRL按`isaacgym`顺序使用x,y,z,w，角速度输入并从四元数旋转重力方向。 | 仿真/既有模型输入链复用；未核验实机坐标系、四元数方向与单位约定。 | 上电静止姿态、轴向/符号、实际刷新率待连接确认。加速度当前并非策略观察量。 | 只读采样时以静止姿态确认轴向和重力投影，不激活控制器。 |
| 足端接触 | LowState `foot_force[4]`映射FR、FL、RR、RL；StateRL用力值大于配置阈值1.0判接触，再做temporal contact编码；送入策略前重排为ROS1的FL、FR、RL、RR。 | 当前MuJoCo实验使用此链；低层硬件文档称支持足端力，但无实机值域记录。 | 实机`foot_force`含义、单位、足序与支撑时值待连接确认。 | 只读静止/支撑状态检查值域和顺序，决定是否需要观测转换。 |
| 机身线速度 | LowState不直接提供策略所用线速度；StateRL优先读odometer接口的世界系velocity，再按IMU旋转到机体系；仅在接口不足且估计器启用时使用估计器。 | MuJoCo odometer路径在历史仿真中使用。 | hardware插件的odometer velocity来自SportModeState；释放原生sport_mode后是否继续更新不能从离线代码确认，必须连接后实测。时间戳/坐标系新鲜度未校验。 | 只读连接时比较LowState与SportModeState更新；若后者不适用，再确定最小速度来源。 |
| 机器人位置/目标 | StateRL从odometer接口读取世界系x/y，默认目标由ABS配置给出(7,0)m，使用IMU四元数计算偏航并把目标变换到机体系。 | MuJoCo地面真值位置路径已用于仿真。 | 实机SportModeState位置来源、原点、朝向及释放sport_mode后是否更新待连接验证；无外部定位时世界目标没有稳定参考。 | 先确定可用定位及其world/map原点，再设置实机目标表达。 |
| 障碍射线 | 策略读取11个`log2(距离米)`值，最大6m；训练几何射线角度约−45°至+45°、每9°一条，射线x起点−0.05m。控制器要求与共享内存payload配套的stamp/header，并在200ms新鲜度内验证。 | MuJoCo几何射线参与既有仿真实验，读取检查和失效安全路径已有运行记录。 | 实机只能由真实深度输入产生射线；相机未知。现有ZED/RealSense预测脚本只写裸payload，没有stamp/header，与当前读取协议不兼容。 | 先选定设备后对齐图像/射线几何、预测标签语义、时间戳协议及更新率；不混淆历史几何结果。 |
| 深度模型与预处理 | MuJoCo预测器采用160×90、米制深度裁剪0.1–6m、log2、三通道复制、11维log2距离输出；前向视角名义高0.27m、HFOV 102°。ZED/RealSense入口仍未适配当前协议。 | 选用soft-safety微调checkpoint；已在4个同位姿点/地图加载并推理。静态误差、native teardown修正及动态安全中止见核验报告；本批动态链路已实际运行，但尚未完成比较。几何标签现按注册场景障碍选择。 | 相机型号/内参/安装外参、深度无效值与部署架构待定；静态CPU成本不能代表控制并发成本。 | 在审阅预测推理长尾与安全中止后，另行决定是否续跑动态比较；相机确定后再单独核对真机输入。 |
| policy/RA/Recovery权重 | Go2 `config/abs/config.yaml`绑定61维policy输入、`policy.pt`和`ra_value.pt`；StateRL按配置路径加载。`rec/policy.pt`另用于恢复策略。 | 三个权重文件在仓库配置目录存在，历史MuJoCo闭环运行验证过相应仿真流程；未证实目标实机架构上的加载及推理成本。 | 部署主机与Torch/LibTorch ABI和CPU指令集未确认。 | 阶段2在部署设备可用时核对依赖并只计算策略输出，不发命令；实际成本仅对实测设备有效。 |
| 动作与电机命令 | policy_joint_order为`ros1_fl_fr_rl_rr`；控制器内部FR、FL、RR、RL与策略顺序转换。配置action_scale=0.25、默认姿态在ABS配置/模型参数，PD Kp=30、Kd=0.65、action clip±4、torque limit 33.5并由StateRL监控。硬件write转成LowCmd并使用显式motor index映射。 | 仿真配置和接口顺序有历史运行及日志核对；实机电机映射未确认。 | 动作实际幅度、力矩行为、索引、低层控制频率与机型兼容待受控设备验证。 | 阶段2离线校验映射和值域；上机时先低风险状态确认，不能靠任意缩小关节动作代替安全准备。 |
| 控制权/只读入口 | 硬件插件`on_init()`启动LowState/HighState订阅和LowCmd发布器；网络接口不是`lo`时调用ReleaseMode。控制器周期`write()`持续发布LowCmd。独立观察器只创建两个`ChannelSubscriber`，退出时关闭订阅并释放本地DDS资源。 | 观察器在本机SDK2完成构建；loopback无消息时两路分别显示等待，定向SIGINT后观察器子进程退出码0。静态代码未发现LowCmd发布器或控制权API调用。 | 实际机器人消息、频率/连续性及字段意义待实机连接核验。旧`launch_abs_real.sh`与当前真机/仿真launch行为不一致，不能用作只读入口。 | 设备条件确认后，按观察器说明显式指定实际网卡和DDS domain运行；不启动硬件插件或controller。 |
| PASSIVE/停止 | StateRL故障路径进入PASSIVE请求，硬件层Kp/Kd/tau全零时改发Unitree stop sentinel；这表示卸力，不是保持姿态或制动。硬件析构会尝试SelectMode("normal")恢复原生sport_mode。 | 仿真状态与代码路径可读；没有实机接管、释放或落地结果。 | 卸力后机器人姿态、支撑和遥控接管行为只能实测确认；Ctrl+C/异常析构时实际接管也待验证。 | 先在支撑与人员监护条件下确认控制权、停止、人工接管，再考虑站立或运动。 |
| 时钟与任务计时 | MuJoCo模式继续使用仿真时钟；hardware模式使用单调时钟。H/Periodic基于当前明确运行模式起计，任务episode计时采用运行模式时长；计算成本仍用steady clock。 | 构建和定向schedule检查通过；仿真及实机时钟行为尚未在运行时验证。 | 实机任务时间会按墙钟推进，实际策略频率和延迟需设备验证。 | 连接前无剩余必要代码修改；在设备可用后观测单调计时与实际周期。 |
| 缺失/停更处理 | LowState与SportModeState分别记录最近接收时间。首次缺失或超过配置的100ms暂定时限时，硬件接口将对应状态设为非有限；StateRL在推理前发现不完整观测后复用现有safetyVeto/PASSIVE。射线继续使用原版本化时间戳协议及既有安全超时。 | 硬件插件和控制器构建通过；未实机验证消息更新率、超时触发或SportModeState可用性。 | 100ms需按真实DDS接收间隔复核；接收时龄不证明传感器采样时钟。 | 首次只读连接先量两类消息的接收率/时龄；任何控制验证另行受控安排。 |

## 五阶段路线

### 阶段1：接口只读核对——已完成

产出即本文件的接口表和缺口清单。实现、历史运行覆盖与设备待确认分开记录。无需清除全部UNKNOWN；完成条件是能确定下一项具体工作。

### 阶段2：最小接口适配与离线检查——已完成（未实机验证）

只处理阶段1确认会阻止接口运行的计时和输入新鲜度问题。构建及定向离线检查通过，没有启动ROS控制器或发送LowCmd。设备未具备时不声称实际推理实时性；100ms仅为待实测的接收超时值。

实机运行持续时间使用单调递增时钟；仿真仍用仿真时钟。H从Recovery入口开始，退出清除，重进重新计时；控制器重启开始新计时段。周期计算、策略周期、数据新鲜度分别计量。

### 阶段3：深度图预测射线验证——静态完成，CUDA诊断完成

已完成模型来源/接口核对、注册几何标签对齐和两图各4个位姿的静态预测比较。输出偏短，false-danger较多。原第3项长停顿定位为CPU同步推理长尾；显式CUDA、三次启动预热及预测器子进程环境修正后，固定姿态独立60秒输出29.84Hz，最大帧间隔76.22ms。随后一次scene_flat动态核验中控制器实际读取预测射线，未见超过200ms的间断或预测安全中止。单次动态任务的安全完成仍UNKNOWN，且收尾有DDS超时FATAL；这不证明一般并发实时性或导航收益。详细证据见[核验报告](../../evidence/R1/GO2_RAY_PRED_12/STATUS_REPORT.md)。

若实际相机已具备，可独立于机器人采集少量已知距离静态障碍数据。相机未知时不预设替代设备。发现严重错误先定位原因，再决定是否进入运动仿真；不自动重训。

### 阶段4：最小完整仿真比较——原冻结批次仍暂停

冻结计划为12项：`scene_flat.xml`开阔图及`challenge27.xml`可绕行障碍图；几何/预测来源；每格3次。使用A组E=−0.05、H=0，全部Periodic P=0.06秒，交错排程。原排程保持不变：两项几何到达，challenge27预测项以ray_age=218.217ms触发`SYSTEM_SAFETY_ABORT`，剩余9项未派发。原中止后另有一次独立scene_flat预测动态核验，ARRIVED但安全完成UNKNOWN；它不占用或替代冻结计划项。后续显式CUDA运行最大帧间隔57.24ms、controller最大读取时龄49.83ms，无几何回退或预测过期中止。收尾阶段DDS timeout/FATAL已记录，不能据此声称运行期间已全面通过。首次批次各启动及清理细节见[冻结批次计划](../../../../logs/09_R1_switch_constraints/go2_ray_pred_12/20261102_r1_go2_ray_pred_12/run_plan.json)和报告。

完成标准是路径接通、行为可解释、计算成本有测量，不要求预测模式成功率更高。12次只作流程与小规模比较，不作普遍性能结论。

### 阶段5：首次连接及受控实机验证——设备和条件具备后另行安排

1. 只读状态核对，不切换控制权、不发布LowCmd。
2. 在适当支撑与人员监护下，确认控制权、停止和人工接管。
3. 站立、短距离低速开阔地运动。
4. 感知和基础运动可靠后，再测试简单静态障碍任务。

离线检查不能替代真实状态刷新、通信和电机响应验证。实机只做有代表性的验证，不复制历史完整参数矩阵。

## 最少修改清单

**连接前可完成：**计时与状态新鲜度适配、构建、射线共享内存接入、标签对齐、静态预测比较及CUDA长尾修正已完成。独立60秒和一次scene_flat动态核验未观察到200ms预测停顿；原冻结12项仍暂停，剩余9项需审阅后决定是否续跑。设备型号/固件、运行主机与架构、实际相机和对应部署依赖需由用户提供或现场确认。

**必须连接后验证：**LowState关节/IMU/接触更新率和映射；SportModeState定位/速度在控制权释放前后的更新情况；相机内参、安装位姿、深度单位/无效值和实际预测频率；部署设备推理/控制周期耗时；释放控制权、LowCmd、PASSIVE卸力及人工接管的实际效果。

## 建议的下一项任务

CUDA与子进程环境修正后，一次动态核验没有复现预测停顿，但安全完成UNKNOWN且收尾记录DDS超时FATAL。建议先审阅该独立核验和原冻结批次日志，再决定是否恢复原剩余9项；如恢复，原第3项仍保留，不重跑，200ms保护不变。之后根据用户提供的Go2型号/固件、主机架构和相机资料安排首次只读连接；观察器使用现场确认的网卡和DDS domain，不改变控制权、不发机器人命令，并分别测LowState/SportModeState接收率及时龄。释放原生控制后的SportModeState更新仍需另行受控核实。

## 研究与状态原则

- R1仍为唯一ACTIVE；本路线不是R1整体完成或部署批准。
- 后续新增仿真统一Periodic P=0.06秒；部署默认不因此改变。
- 原几何射线实验、A/B/C/D和E/X/H结果按原配置保留，预测射线实验单独标注感知来源。
- 按安全完成、碰撞、超时、推进和耗时评价；切换次数只作解释。
- 不为了安全表象任意缩小动作，不因负面结果改图或扩大实验。
- 不把旧手册、文件存在、构建通过或离线检查表述成实机验证。

## 主要代码依据

- [StateRL.cpp](../../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp)、[StateRL.h](../../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/include/rl_quadruped_controller/FSM/StateRL.h)、[ABS配置](../../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml)
- [Unitree硬件实现](../../../../quadruped_ros2_control_humble/hardwares/hardware_unitree_mujoco/src/HardwareUnitree.cpp)、[实机launch](../../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/launch/real_go2.launch.py)、[旧实机启动脚本](../../../../scripts/launch_abs_real.sh)
- [MuJoCo射线预测器](../../../../scripts/ray_predictor.py)、[ZED预测器](../../../../scripts/zed_ray_predictor.py)、[RealSense预测器](../../../../scripts/realsense_ray_predictor.py)、[射线共享内存协议](../../../../common/abs_ray2d_shm_contract.h)
- [Go2射线训练配置](../../../../ABS/training/legged_gym/legged_gym/envs/go2/go2_pos_config.py)、[Go2硬件接口说明](../../../../quadruped_ros2_control_humble/hardwares/hardware_unitree_mujoco/README.md)。旧[实机部署手册](../../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/doc/real_go2_deployment.md)仅作历史线索；其启动命令与当前真机launch、仿真launch的实际行为不一致，不能照此作为只读入口。
