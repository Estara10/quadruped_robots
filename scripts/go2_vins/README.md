# D435i／VINS 与 Go2 机身状态

更新：2026-11-09。为电脑原生 A→B 和后续 ABS 实机提供独立位置／速度来源。当前已完成部分实机只读采集；定位准确性、机身在线转换与控制接入均未通过。

## 当前链路

```text
D435i（固定在 Go2，USB2）
  → Orin：双 IR 480×270@15 Hz + 合并相机 IMU 约 200 Hz
  → CycloneDDS / ROS domain 42
  → 电脑：Humble CPU VINS-Fusion
  → 相机 IMU 参考点位姿／机体系线速度
  → [待测安装变换、待只读验证] Go2 base 位姿／速度
```

电脑 VINS 不是 Go2 原生运动服务。此目录没有 SportClient／LowCmd 运动发送入口；相机、定位和转换程序也不能停止机器人。原生运动入口在 [go2_native_motion](../go2_native_motion/README.md)，尚未接本目录输出。

## 文件与记录

| 入口 | 内容／状态 |
|---|---|
| [BODY_FRAME.md](BODY_FRAME.md) | 当前下一项：base 原点、安装测量、转换约定与只读核对 |
| [body_frame.py](body_frame.py) | 不依赖 ROS 的刚体位姿／速度转换，包含转动杠杆臂项 |
| [go2_body_odometry.py](go2_body_odometry.py) | 传播 odom＋同时间 IMU → `/go2/vins/base_odometry`；在线未验证 |
| [未测量配置](config/go2_mount.unmeasured.json) | 偏移／旋转为 null，初始化 DDS 前拒绝运行 |
| [离线检查](test_body_frame.py) | 7 项通过；不证明实机精度 |
| [480×270 配置](config/d435i_480x270/realsense_stereo_imu.yaml) | 当前 USB2 候选；安装外参是另一项参数，不能直接照抄此处相机内部外参 |
| [VINS 适配补丁](patches/0001-vins-humble-cpu-and-odometry.patch) | Humble／CPU／Ceres 兼容、速度消息约定、线程退出及可选诊断 |
| [跨机说明](CYCLONE_FIELD_CHECK.md) | 实测 CycloneDDS 相机数据链；按现场 IP 配置 |
| [构建准备](../../docs/thesis_project/real_go2/20261105_D435I_VINS_FUSION_PREPARATION.md) | 固定源码与外部构建依赖 |
| [构建恢复及 IMU 审阅](../../docs/thesis_project/real_go2/20261108_VINS_BUILD_AND_IMU_REVIEW.md) | `/tmp` 重建与告警边界 |
| [实机最新记录](../../docs/thesis_project/real_go2/README.md) | 动态结果、异常、设备与下一步 |

`config/d435i_640x480/` 保留相机直连电脑阶段的配置，不作为当前机载 USB2 的默认现场模式。udev 规则只针对已记录设备，不自动安装。

## 构建与数据合同

VINS 源码候选：[zinuok/VINS-Fusion-ROS2](https://github.com/zinuok/VINS-Fusion-ROS2)，固定提交 `72023bc5d2abb14faa4c02a5c040d84f996b587a`，按准备记录应用本目录补丁。外部源码／Ceres／OpenCV／Eigen／ROS 不包含在此目录中，临时 `/tmp` 二进制可能丢失。

补丁后的 odom pose 在 `vins_world`，linear twist 已在 `camera_imu_optical_frame`，不再逆旋转一次。现有 odom angular 未填写，机身转换使用同步相机原始 gyro，仍含零偏；不补零假装已知。输出保持 VINS 世界参考，不自动重置起点，不发布 TF，不修复 VINS 跳变。

逐次外参／td 诊断来自 A/B 工作的额外 diff，见[回放报告](../../docs/thesis_project/real_go2/20261109_EXTRINSIC_AB_REPLAY.md)。不要把默认关闭的诊断或临时二进制误当作基础补丁已包含的功能。

## 离线验证

从仓库根目录运行，使用系统 NumPy／SciPy；不初始化 ROS／DDS：

```bash
PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 scripts/go2_vins/test_body_frame.py -v
```

当前先测相机到固定机身的安装参数，再只读核对。转换通过后才进入成熟路径跟踪离线评估和受控原生运动；Nav2 RPP 尚未接入。完整 ABS 实机仍需真实障碍输入、RA／Recovery 状态接口及低层停止／交接验证。五厘米到点尚未实现。
