# D435i：Orin到电脑的CycloneDDS传输检查

日期：2026-11-08。只启动相机采集与电脑状态订阅，不运行VINS、goto、论文控制器或机器人运动。Go2保持稳定趴伏，D435i通过USB连接机载Orin，电脑通过网线连接Go2。

本次问题：此前Orin本机480×270双IR约15 Hz、合并IMU约200 Hz，但FastDDS跨机接收仅0.2–2.5 Hz。只比较统一CycloneDDS后的接收情况；不同时改相机profile、算法和标定。

## 1. 终端A：登录Orin并启动相机

电脑本地终端：

```bash
ssh unitree@192.168.123.18
```

若出现 `ros:foxy(1) noetic(2) ?`，输入1。进入 `unitree@ubuntu` 后执行：

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.123.18</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
ls -l /opt/ros/foxy/lib/librmw_cyclonedds_cpp.so
```

若库不存在，先停在此处并保留报错。库存在时启动：

```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/camera/camera -r __node:=camera \
  -p device_type:=d435i -p enable_depth:=false -p enable_color:=false \
  -p enable_infra1:=true -p enable_infra2:=true \
  -p depth_module.profile:=480,270,15 -p depth_module.emitter_enabled:=0 \
  -p enable_gyro:=true -p enable_accel:=true -p gyro_fps:=200 -p accel_fps:=100 \
  -p enable_sync:=true -p unite_imu_method:=2
```

相机profile沿用已验证的Foxy命令。保持此终端运行。若设备缺失、权限失败、右路错误或CycloneDDS配置报错，停止并保留输出，不继续修改系统或换profile。

## 2. 终端B：电脑接收检查

另开电脑本地终端，不进入SSH。执行：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.123.99</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
ros2 topic list --no-daemon
```

确认实际话题名称。按已有记录，预期包含 `/camera/camera/infra1/image_rect_raw`、`/camera/camera/infra2/image_rect_raw`、`/camera/camera/imu`。若名称不同，先保留实际列表，不照抄不存在的话题。

逐条执行，各测12秒后自动退出：

```bash
timeout --signal=INT --kill-after=2s 12s ros2 topic hz \
  /camera/camera/infra1/image_rect_raw --window 30 --wall-time
```

```bash
timeout --signal=INT --kill-after=2s 12s ros2 topic hz \
  /camera/camera/infra2/image_rect_raw --window 30 --wall-time
```

```bash
timeout --signal=INT --kill-after=2s 12s ros2 topic hz \
  /camera/camera/imu --window 200 --wall-time
```

保留每项的average rate和max。当前预期是左右各接近15 Hz、IMU接近200 Hz，没有数秒级空档。CLI速率会受订阅QoS和电脑负载影响，只用作本轮快速判断，不当作传感器采样频率或VIO精度证明。timeout返回124是预定限时结束。

## 3. 结束与判断

三项结束后，在终端A按Ctrl+C结束相机节点，等待返回SSH提示符，再检查：

```bash
pgrep -af '[r]ealsense2_camera_node'
```

电脑本地检查本次订阅命令是否结束：

```bash
pgrep -af '[r]os2 topic hz'
```

如有输出，确认进程来源后处理，不影响其他会话。不要关闭Unitree原生服务。把相机启动结果、实际话题名及三项频率发给主助手。

- 恢复正常：再恢复电脑VINS构建，验证安装状态下的独立位姿/速度。
- 仍出现此前量级的严重降频：停止本轮，下一步根据记录比较机载VINS方案，不反复扩大DDS调参。
- 当前不接任何控制闭环，不把相机数据正常等同于定位准确。

## 配置说明与本机检查

两端IP沿用直接记录：Orin192.168.123.18、电脑192.168.123.99；2026-11-08主助手已只读确认电脑网口UP、直连路由和机载电脑ping成功，电脑Humble CycloneDDS库存在，未发现相机/VINS/原生运动工具残留。没有在机载端重新核对当前库文件，由终端A命令完成确认。原 `/tmp/go2_vins_build/install/vins/lib/vins/vins_node` 当前不存在，不直接启动旧VINS命令。

使用 `CYCLONEDDS_URI` 临时指定网卡IP，不改系统默认配置。[CycloneDDS配置指南](https://cyclonedds.io/docs/cyclonedds/latest/config/index)列出该环境变量及NetworkInterfaceAddress配置；该标签在较新版本可能显示deprecated提示，单纯提示不等于失败。采用该标签是为兼顾现有Foxy/Humble环境，不与新版Interfaces配置混用。若实际解析失败，保留原始错误并结束，不在现场盲改。

## 4. 下一次 VINS 只读观察（2026-11-08 准备）

相机与三路接收确认后，按[VINS构建恢复与IMU告警审阅](../../docs/thesis_project/real_go2/20261108_VINS_BUILD_AND_IMU_REVIEW.md)执行：

- 电脑先按 best_effort 记录 `/camera/camera/imu`、`/vins/odometry`、`/vins/imu_propagate`，QoS 文件为[`field_record_qos_best_effort.yaml`](field_record_qos_best_effort.yaml)；
- 再启动恢复后的 `/tmp/go2_vins_build/install/vins/lib/vins/vins_node`，配置 `config/d435i_480x270/realsense_stereo_imu.yaml`，设置 `VINS_IMAGE_QOS=best_effort`、`VINS_SYNC_DIAGNOSTICS=1`；
- 先在机器人静止时观察 `SYNC_DIAG`、特征数、`Initialization finish!` 和 `/vins/odometry`/`/vins/imu_propagate`；未初始化则停止，不自行移动机器人；
- 停止顺序为 VINS → 记录器 → 相机；检查退出码和两端残留进程；全程不发送机器人运动、停止、恢复或模式切换指令。

时间对齐、QoS 核对、完成/停止条件见上述记录。
