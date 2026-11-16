# Go2 只读 Estimator 观察程序准备

## 目的与边界

本轮为后续核查 LowState→Estimator→Recovery 速度坐标转换准备独立只读工具。程序不运行 Recovery/Agile/RA，不加载模型，不启动 ROS 控制器，不发布 LowCmd，也不创建 MotionSwitcher 客户端。本轮机器人保持关机；没有初始化实机 DDS 或执行现场观察。

## 调用链与观察周期

| 项目 | 代码证据 | 本观察器处理 |
|---|---|---|
| 关节槽序 | Go2配置和硬件插件按 FR、FL、RR、RL，每腿hip/thigh/calf；插件当前 `motor_index_map_={0…11}`，LowState回调读取DDS同索引。[插件映射](../../../quadruped_ros2_control_humble/hardwares/hardware_unitree_mujoco/include/hardware_unitree_mujoco/HardwareUnitree.h:61)、[状态读取](../../../quadruped_ros2_control_humble/hardwares/hardware_unitree_mujoco/src/HardwareUnitree.cpp:282) | 直接记录LowState索引0–11并按当前顺序列名；硬件映射仍需现场静态核对。 |
| q/dq | 插件从LowState的`motor_state[i].q/dq`写状态接口；算法假设分别为rad、rad/s。[Estimator输入](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/robot/QuadrupedRobot.cpp:113) | 每个Estimator周期完整记录12项q/dq。 |
| Go2运动学 | `QuadrupedRobot`从URDF构建base→FR/FL/RR/RL foot四条KDL链；各腿用连续3个关节计算足端位置和雅可比速度。[模型](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/robot/QuadrupedRobot.cpp:9) | 读取仓库Go2 `robot.urdf`，调用原有模型类，不复制运动学算法。 |
| IMU | 插件映射四元数wxyz、gyro xyz、accel xyz；Estimator计算`R*acceleration + [0,0,-9.81]`并用固定dt推进速度。[硬件映射](../../../quadruped_ros2_control_humble/hardwares/hardware_unitree_mujoco/src/HardwareUnitree.cpp:289)、[算法](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/control/Estimator.cpp:194) | 记录原始四元数wxyz、角速度和加速度、四元数范数；不重排或归一化输入。rad/s与m/s²是接口预期，实际硬件语义仍须现场确认。 |
| 足力/接触 | LowState四路按FR/FL/RR/RL写入；Estimator当前默认阈值1，`force < 1`为非接触，`force >= 1`参与足端静止约束。[Estimator判断](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/control/Estimator.cpp:169) | 同时记录四路原始足力和阈值布尔值；阈值不改。趴伏记录14–16只表明这些值会判接触，不能据此调阈值。 |
| 固定dt与调用频率 | Go2 controller manager为1000Hz；RL控制器自身`update_rate`配置为200Hz，`on_init`将该参数写入`CtrlInterfaces.frequency_`，Estimator用`dt=1/frequency`。ROS2 Control Humble允许按控制器配置update_rate；故当前控制器Estimator的固定dt为5ms。[项目配置](../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml:4)、[RL参数](../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml:153)、[初始化](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/RlQuadrupedController.cpp:468)、[dt](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/control/Estimator.cpp:18)、[ROS2 Control Humble控制器周期说明](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html) | 观察器按200Hz调用同一个Estimator，保持dt=5ms；每次使用最新LowState，即使样本重复也仍按Estimator配置周期更新，并记录重复标志和接收间隔。它不按回调每条消息更新。实际运行时序以CSV实测字段为准。 |
| 初始/失效 | Estimator构造时状态清零、协方差置大；无显式重置，原算法无输入时龄和非有限检查。[构造与更新](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/control/Estimator.cpp:12) | 每次进程新建估计器；首条LowState到达前不更新。输入过期或输入/输出非有限时写最后一行并退出。此为观察器局部行为，不改变生产控制器。 |
| 输出与Recovery | `getVelocity()`为世界系估计速度；Recovery先检查odom接口数量，接口≥6就选odom，否则在Estimator启用时取Estimator世界速度，再按IsaacGym xyzw四元数执行逆旋转到机体系。[Recovery选择与转换](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRLRec.cpp:433)、[Recovery姿态顺序](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp:2126) | 记录Estimator世界速度及`R.transpose()*v_world`机体系速度。90°偏航离线检查与Recovery逆四元数旋转公式一致；没有改变Recovery消费路径。 |

**既有记录边界：**历史只读记录是约10Hz快照，不能当作200Hz连续LowState输入，也不能用来评价Estimator固定dt下的漂移或动态速度。趴伏足力14–16不是接触误判的证据；它只说明阈值1下四路被标为接触。世界定位来源仍缺失，但本工具仅评估静止速度观测，不要求解决导航位置。

## 实现与离线结果

- 新增`go2_readonly_estimator`构建选项，默认OFF；仅显式设置`BUILD_GO2_READONLY_ESTIMATOR_OBSERVER=ON`时才查找SDK并生成目标。实时实现位于[go2_readonly_estimator.cpp](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/go2_readonly_estimator.cpp)，CMake定义见[CMakeLists.txt](../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/CMakeLists.txt:23)。仿真和普通RL包构建不包含此观察器。
- 目标直接编译包中已有的`Estimator.cpp`、`QuadrupedRobot.cpp`和`RobotLeg.cpp`，不链接RL控制器共享库、不链接Torch运行库。`ldd`确认无RL控制器或Torch动态库依赖；源码只创建`rt/lowstate` Subscriber。
- 实机launch使用`xacro/robot.xacro`生成robot_description。本轮离线渲染该xacro（只设置network_interface/domain文本参数，不启动DDS），并与观察命令使用的`urdf/robot.urdf`逐腿比较：FR/FL/RR/RL四条base→foot链的关节名、父子link、类型、关节原点和活动轴均一致；KDL构造质量也相同。额外相机/lidar固定链不参与四足运动学。
- CSV逐周期记录本机steady时间、Estimator调用间隔、LowState样本时龄/接收间隔/本地接收序号、原始tick、重复样本、输入有限性、姿态范数、q/dq、IMU、原始足力、阈值接触标记、world/body速度。最新输入超过100ms或出现非有限值时记录后终止，不向Estimator继续送旧值。
- 隔离`/tmp/go2_estimator_build/controller`构建通过。`--self-check`在不初始化DDS的情况下加载真实Go2 URDF，构造四条KDL链和现有Estimator；合成静态状态下dt=0.005s、质量15.098kg、四足阈值接触标记为真，速度输出约`[1.23e-6,-1.69e-7,5.76e-7] m/s`。另检查Estimator的wxyz旋转转置与Recovery IsaacGym xyzw逆旋转在90°偏航下相符。
- 自检输入是合成静止状态，只验证模型构造、接口通路、初始化后数值有限和坐标变换关系；不能证明Go2上的惯导偏差、接触语义、动态速度准确性。
- `rg`源码核对未发现LowCmd、MotionSwitcher、SelectMode/ReleaseMode或模型加载路径；执行文件动态依赖也不含RL控制器库或Torch。没有运行实时入口，没有启动DDS。离线构建与自检进程均已退出。

### 现场静止观察的准确命令

仅在用户安排观察后使用。保留原生运动服务运行，不释放控制权、不启动ROS控制器、不发送任何运动命令、不要求机器人运动。命令使用已验证连接的domain 0和`enp7s0`；现场先确认网卡仍连接Go2。

```bash
env LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu:/home/lidio/Libraries/unitree_sdk2/lib \
  /tmp/go2_estimator_build/controller/go2_readonly_estimator \
  --domain 0 --interface enp7s0 \
  --urdf /home/lidio/quadruped_robots/quadruped_ros2_control_humble/descriptions/unitree/go2_description/urdf/robot.urdf \
  --csv /tmp/go2_estimator_static_20261104.csv --duration-s 30
```

**观察记录项：**CSV中LowState接收间隔和时龄、重复标志、原始tick、四元数范数、FR/FL/RR/RL各关节q/dq、IMU及足力、阈值接触位、世界/机体系速度、输入和估计有限标记。

**完成条件：**完成30秒记录；各行输入与估计均有限、LowState样本时龄不超过100ms；姿态和q/dq与现场静止状态一致；无明显速度持续漂移；确认使用的URDF腿序、四元数顺序和阈值接触结果。若LowState中断，观察器在3秒无首帧或样本时龄超过100ms时退出；CSV明确记录最后状态。

**停止条件：**现场人员需要操作机器人、机器人姿态/支撑不再稳定、数据映射与预期不符，或观察器因过期/非有限输入退出，立即按Ctrl+C结束观察并保存CSV；若姿态本身变化，先由现场人员按既有现场流程处置，不依赖该程序控制机器人。观察器正常结束会关闭订阅并释放本机SDK通道。

静止速度接近零只构成基础静态检查；它不能证明运动时的方向、幅值、延迟和足滑影响，也不能作为Recovery实机启用依据。静止观察必须不接管控制、不中断原生服务。

## 未知与下一步

仍未知：本机Go2固件与LowState字段单位/标定；释放服务时真实LowState频率和时序；低层硬件的实际关节映射；趴伏状态四足接触约束对速度解的影响；脚滑、IMU偏置和动态速度误差。没有动态真值对照数据，因此本观察器只准备基础静止观察，未提供策略速度源接入。

**唯一推荐下一步：**用户安排一次原生服务保持运行、姿态稳定且机器人不动的30秒只读Estimator观察。完成条件是上述CSV完整记录并按静态完成条件审阅；若通过，只能得出静止输入/初始化表现可接受，动态速度精度仍需单独授权与验证。

本修改不改变模型观测、Recovery数据路径、硬件插件、配置或仿真默认行为；不改唯一ACTIVE任务，不训练模型，不影响R1仿真工作线。
