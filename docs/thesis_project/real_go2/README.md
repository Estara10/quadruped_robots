# Go2实机准备与接口修复

> **当前续接（2026-11-06 晚）：**此前相机-only视频参考平移约70秒、消息窗口净位移1.011 m，未复现秒级空档或0.436 m级相邻跳变；外部视频漏做同步，动态精度仍未验证。相机固定安装到Go2后，只读确认当前为USB2（Bus 01 480Mb/s，Bus 02 10000M未使用）。机载Orin已安装RealSense Foxy驱动和最小udev规则：480×270@15双IR+IMU在Orin本机稳定（15.15/200.06 Hz），640×480双IR因右路MIPI错误不可用；跨机到电脑FastDDS仅0.2–2.5 Hz，时钟对齐与ufw排除后仍无改善，根因未证明。480×270 VINS配置已建立但未初始化。电脑已装CycloneDDS，下一步两端统一切换后复测；Go2控制链未启动，无残留进程。见[机载驱动与跨机排查](20261106_ONBOARD_REALSENSE_BRINGUP.md)、[视频参考平移记录](20261106_VINS_VIDEO_REFERENCE_TRANSLATION.md)、[停止修复记录](20261106_VINS_SHUTDOWN_FIX.md)、[配对缺口诊断](20261106_VINS_STEREO_PAIRING_GAP_DIAGNOSIS.md)、[右图交付QoS离线诊断](20261106_VINS_RIGHT_IMAGE_DELIVERY_QOS_DIAGNOSIS.md)、[位置回摆审阅](20261106_VINS_POSITION_BOUNCE_REVIEW.md)和[当日续接记录](SESSION_20261105.md)。以下准备与历史记录按各自日期理解。

> 2026-11-04实机发现：恢复原生ai后机器人意外起立，趴伏交接未通过；用户已遥控趴伏，所有电脑检测进程已退出。交接诊断入口已冻结，完整实机控制/自动恢复路径继续NO-GO。释放原生服务时SportModeState确实停更；Recovery已有Estimator速度候选，但动态准确性未验证，独立世界位置来源仍未解决。见[实际结果](20261104_MODE_HANDOFF_RESULT.md)和[Recovery速度来源审查](20261104_RECOVERY_VELOCITY_SOURCE.md)。

> 2026-11-04离线停止链审查：Go2用户手册称低层编程时遥控命令无效；本项目L2+B停止依赖电脑控制链，不能证明电脑失联时遥控可独立停止。LowCmd中断后的Go2 EDU本机动作未知。见[遥控停止与LowCmd审查](20261104_REMOTE_CONTROL_AND_LOWCMD_FAILURE_REVIEW.md)。

本目录记录独立实机工作线，不改变R1唯一ACTIVE任务，不改写历史仿真结果。

- [D435i软件环境准备](20261105_D435I_SOFTWARE_PREPARATION.md)：历史安装准备；用户后续已安装并完成图像/IMU接收检查，现状见当日续接记录。
- [D435i VINS接入准备](20261105_D435I_VINS_FUSION_PREPARATION.md)：CPU/Humble隔离构建通过；无输入等待阶段及初始化后持续输出时的SIGINT退出码均为0，动态定位仍待验证。停止修复依据见[2026-11-06记录](20261106_VINS_SHUTDOWN_FIX.md)。
- [VINS双目配对缺口离线诊断](20261106_VINS_STEREO_PAIRING_GAP_DIAGNOSIS.md)：输入-only回放未复现2.6 s优化odom空档；新增默认关闭的接收/配对诊断日志，现场订阅QoS与数据交付仍待核实。
- [VINS右图交付QoS离线诊断](20261106_VINS_RIGHT_IMAGE_DELIVERY_QOS_DIAGNOSIS.md)：最新现场记录确认相机图像为可靠发布、VINS图像为best-effort订阅；单次同包候选回放收齐全部源图像，但未复现现场长间隔，根因和现场效果仍待验证。
- [VINS位置回摆现场数据审阅](20261106_VINS_POSITION_BOUNCE_REVIEW.md)：定位到优化状态校正先进入odom、再重锚IMU传播；无明确代码缺陷，真实运动仍缺独立记录。
- [VINS视频参考平移采集](20261106_VINS_VIDEO_REFERENCE_TRANSLATION.md)：相机-only约70秒，消息窗口净位移1.011 m，无秒级空档或0.436 m级跳变；外部视频漏做同步动作，未精确对齐，动态精度未验证。
- [机载D435i驱动与跨机排查](20261106_ONBOARD_REALSENSE_BRINGUP.md)：Foxy/arm64驱动与最小udev规则已装；480×270双IR+IMU本机15/200 Hz，640×480双IR右路MIPI失败；跨机FastDDS仅0.2–2.5 Hz；VINS未初始化；CycloneDDS待测。
- [Estimator只读观察程序准备](20261104_ESTIMATOR_OBSERVER_PREPARATION.md)：复用现有Estimator与Go2 URDF，独立LowState观察器已离线构建和静态自检；未初始化DDS或执行现场观察。
- [Recovery线速度来源显式选择](20261104_RECOVERY_VELOCITY_SOURCE.md)：Recovery现支持明确选 `odom` 或 `estimator`，静态仿真默认仍选 odom；Estimator 实机候选未接入 launch，动态速度精度未验证。
- [原生遥控慢走速度对照准备](20261105_DYNAMIC_VELOCITY_COMPARISON_PREPARATION.md)：历史现场方案；两次慢走已执行，观察器增加Sport逐条记录及共用时间基准，实际结果见当日续接记录。
- [Estimator慢走动态异常离线诊断](20261105_DYNAMIC_ESTIMATOR_ANOMALY_DIAGNOSIS.md)：对两次原生遥控慢走日志和Estimator足端约束链作离线诊断；只修复观察器CSV缺字段，不据此宣称动态速度准确。
- [Estimator动态回放与最小修复](20261105_ESTIMATOR_DYNAMIC_REPLAY_FIX.md)：两批原始记录已由原Estimator逐行复现，并比较原方程、仅旋转足端测量、再加角速度项；修正版保持显式选择，仿真默认行为未改变。
- [慢走逐腿固定足约束对照](20261105_LEGWISE_FIXED_FOOT_CONSTRAINT_REVIEW.md)：按Go2 URDF/KDL分别计算四腿固定足速度候选，并与足力、关节运动及Estimator输出对照；动态约束分歧明显，但尚不能据此标注支撑腿或确认因果。

## 已知设备与推进顺序

已确认Go2 EDU，使用本台外接电脑；enp7s0、电脑192.168.123.99/24、DDS domain=0已实际通信。双手遥控器照片和L2+B按下/松开数据已核对，原生前置彩色相机可预览；Go2固件仍待确认。外接深度设备现为D435i，图像/IMU已收到；相机已固定安装到Go2，用户报告USB传输速率由3.0降为2.0。只读核对确认相机当前在Bus 01、协商480Mb/s；换线绕过USB2 Hub后仍为480Mb/s，Bus 02的10000M根口未使用。限制在端口/上行链路或线缆，当前扩展坞无已确认的USB3路径，按USB2设计。机载Orin NX为Ubuntu 20.04/arm64/Foxy；已装RealSense 4.51.1/2.51.1驱动和仅针对8086:0b3a的udev规则，系统时钟已对齐电脑；480×270双IR+IMU本机15/200Hz，640×480双IR右路MIPI失败；跨机FastDDS图像仅0.2–2.5Hz，电脑已装CycloneDDS待测。VINS的480×270配置已建立但尚未初始化；动态位置/速度仍未验证。原生服务释放期间SportModeState停更已实测。Recovery可显式选Estimator，但两次慢走动态异常尚未解决，未启用实机策略。

只读状态连接 → 静止接口/遥控输入核对 → 有支撑的控制权与停止确认 → 受控站立 → 人工操控慢走 → 速度、姿态、关节状态与估计力矩正常后再慢跑 → 真实障碍感知 → 代表性简单障碍实验。每一步须按实际结果决定下一步，不把构建通过当作实机可用。

## 2026-11-04 控制权与停止修复

用户授权修复离线排查发现的代码问题；本轮未连接设备、未启动ROS节点、模型或运动。

- 硬件插件初始化仅建立DDS与状态订阅；控制权释放和LowCmd发布器创建移到硬件激活阶段。查询失败或释放未确认，激活返回ERROR，不创建命令发布器。
- 激活前记录原生模式；只在查询确认原生模式已释放后开始LowCmd通信。激活时清除旧增益和力矩，避免再次激活沿用旧命令。
- 硬件停用/关闭/析构时，发送最后一帧stop sentinel，关闭LowCmd发布器，然后恢复进入前的模式并查询确认。与write互斥，发布器关闭后不能再写命令。最后发送失败、恢复失败均明确报告；失败不包装成成功。
- 进入前无原生模式时不擅自选择normal；现场必须安排明确的人工恢复方式。恢复查询失败会保留原模式信息供后续关闭重试。
- 单独停用RL控制器会停止当前状态推理并清零命令，但硬件可能仍活动；PASSIVE/命令1或9仍是卸力，不等于保持站立或恢复原生遥控。交还使用硬件停用/关闭链。
- 实机launch要求显式提供network_interface与domain，并传入URDF硬件参数。仿真仍默认lo/domain=1；未改策略、权重、阈值、Periodic或保持时间。
- 只读观察器增加两路最大本机接收间隔，以及SportModeState原始stamp、stamp最近变化时间、mode和error_code。显示用于现场判断，不根据未知时间戳语义增设控制阈值，也不自动换用尚未验证的速度估计器。

## 离线验证与边界

硬件插件、RL控制器、只读观察器构建成功。离线展开xacro确认默认lo/domain=1与显式网络参数均进入硬件描述，实机launch语法通过。定向SDK替代测试在/tmp/go2_handoff_check中直接编译本次硬件源码，不建立DDS网络；覆盖查询失败、释放未确认、关闭发布器先于恢复、恢复原模式、重复关闭、重新激活清零、原生模式原先关闭、恢复失败、请求报错但状态查询已确认、末帧发送失败、发布器初始化异常、末帧发送异常及loopback不调用模式接口。10组定向检查全部通过。已将构建后的硬件插件和控制器安装到本机现有工作空间。

这些检查不能证明真实机器人收到了stop sentinel，不能证明恢复模式后遥控器已实际接管。电脑断电、SIGKILL或网络失联时，不保证主机关闭回调执行；现场必须确认可用的人工停止和恢复路径。停止卸力须有适当支撑，不能据此安排自由站立中接管测试。

观察器另通过合成消息回调检查：最大接收间隔、机器人stamp不变/变化、mode/error_code存储和显示均符合预期；该检查未初始化DDS。

## 下一项任务

下一项：两端统一 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（电脑已装 `ros-humble-rmw-cyclonedds-cpp 1.3.5`），在 domain 42、非 localhost 下重测图像/IMU 跨机速率。若恢复约 15/200 Hz，运行 480×270 VINS；若仍慢，再比较机载 VINS 或非 DDS 图像传输。不先改 VINS 算法或标定；640×480 双 IR 的 USB2 限制留待跨机链路结论后决定。Go2 控制链不启动。SportModeState 停更与恢复 ai 起立的历史结果保留，不重复趴伏交接流程。完整控制器及 RL 运动仍需有效状态来源和明确的支撑/停止/恢复方案。细节见[机载驱动与跨机排查](20261106_ONBOARD_REALSENSE_BRINGUP.md)。

## 现场记录

- [首次只读连接](20261104_FIRST_READONLY.md)：两路消息接收通过。
- [静止接口核对](20261104_STATIC_INTERFACES.md)：原生服务ai；已完成数值基础检查，用户确认趴伏且略有前后倾斜，固件信息待提供。
- [只读状态曲线与前置相机](20261104_LIVE_VIEW.md)：实际取图成功，窗口显示及关闭检查通过。
- [遥控停止链与只读按键核对](20261104_REMOTE_STOP.md)：L2+B实际映射9、松开回0，全程不发布；实机显式使用LowState内遥控字段，接管后的停止效果仍待验证。
- [趴伏交接检查准备（历史）](20261104_MODE_HANDOFF_PREP.md)：专用程序及8项离线检查完成；初次审批拒绝记录保留，后续用户已明确同意并完成一次实际检查。
- [实际交接结果](20261104_MODE_HANDOFF_RESULT.md)：服务恢复确认，但出现意外起立，未通过趴伏目标；人工已恢复趴伏，诊断入口冻结。
- [遥控停止与LowCmd故障审查](20261104_REMOTE_CONTROL_AND_LOWCMD_FAILURE_REVIEW.md)：结合官方Go2手册、SDK示例、项目按键处理和硬件生命周期，区分电脑端停止与未知的机器人端失联行为。
