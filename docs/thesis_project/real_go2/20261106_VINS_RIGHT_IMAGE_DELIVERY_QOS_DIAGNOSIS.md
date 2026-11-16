# D435i/VINS右图交付与QoS候选离线诊断

日期：2026-11-06。本轮只读代码、使用已保存rosbag做隔离回放；未打开相机、未连接Go2、未启动机器人链路。

## 结论

现场证据把缺口限定在**右图进入VINS图像队列之前或图像回调未完成记录之前**，但还不能区分DDS交付、executor调度和回调等待。没有发现3 ms FIFO配对器会抑制右图回调的实现错误。基于现场发布端为RELIABLE、VINS订阅端为BEST_EFFORT这一证据，加入了一个默认不启用的订阅可靠性候选。相同录包的一次对照中，候选比BEST_EFFORT多收到2张右图和1张左图，收齐全部源header；这是候选依据，不是现场根因已证实或故障已修复。

## 现场证据与代码路径

最新现场记录[`20261106_112612_vins_live_pairing_diagnostic`](../../../logs/real_go2/20261106_112612_vins_live_pairing_diagnostic/RESULT.md)显示：

- 原始采集器收到810张左图、706张右图；VINS回调收到810L/695R，配对和dispatch均为695，丢弃115张旧左图，没有右图drop记录。
- 两个订阅者共有638个右图header；采集器独有68个、VINS独有57个。不能把净计数差11解读为只有11张不同。
- VINS右图回调最大间隔1.333368 s；间隔期间采集器收到了4张中间header的右图。优化odom最大接收间隔1.497913 s、header间隔1.466789 s；IMU传播最大接收间隔9.89 ms；solver耗时最高14.03 ms。
- 采集器自己的右图最大间隔约0.800 s，说明输入中也有真实的大间隔；现场1.33 s回调间隔还包含VINS未收到的样本。

固定上游提交`72023bc5d2abb14faa4c02a5c040d84f996b587a`的`vins/src/rosNodeTest.cpp`路径如下：

- `img0_callback`/`img1_callback`（约44、58行）在回调入口先记steady时间和header，随后锁`m_buf`、将消息放入FIFO，解锁后才打印receive日志。未出现receive记录说明回调未完成入队/日志点；消息可能尚未交付，也可能已进入回调后被executor排队或等待互斥锁，日志本身不能区分它们。
- `sync_process`（约97–165行）每轮查看两FIFO队首；header差超过3 ms时丢弃较旧帧，匹配时转换图像并出队，之后调用`Estimator::inputImage()`。转换图像时持有`m_buf`，因此存在回调等待互斥锁的候选路径，但没有锁等待时长证据。`inputImage()`在解锁后调用。
- `main`（约332–363行）分别订阅左右图像，运行`MultiThreadedExecutor`及独立同步线程。回调没有指定独立callback group。队列配对只能处理已经进入FIFO的图像；本次所有进入VINS的右图都配对，没有证据表明FIFO丢弃解释了缺失的receive记录。

因此本轮没有改executor、互斥锁、FIFO深度、队列丢弃方式或3 ms容差。solver峰值约14 ms不足以单独解释秒级缺更；回放下优化odom仍有约0.93 s header间隔，也说明QoS候选不能消除相机输入本身的间歇。

## QoS来源与单一候选

现场`endpoint_qos.json`发现图像发布端为`RELIABLE`/`TRANSIENT_LOCAL`，VINS与采集器订阅端为`BEST_EFFORT`/`VOLATILE`。ROS 2 Humble采用requested/offered兼容规则：可靠发布端与best-effort订阅端兼容；transient-local发布端与volatile订阅端也兼容。因此现场端点差异本身不是QoS不兼容错误证明。[ROS 2 Humble QoS文档](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html)

Humble的`rmw_qos_profile_sensor_data`实际是KeepLast 5、BEST_EFFORT、VOLATILE；当前VINS左右图订阅使用`rclcpp::SensorDataQoS()`，依据为固定上游源码和本机`/opt/ros/humble/include/rmw/rmw/qos_profiles.h`。[Humble RMW源码](https://github.com/ros2/rmw/blob/humble/rmw/include/rmw/qos_profiles.h)

RealSense ROS 4.58.4官方源码将`IMAGE_QOS`常量设为`SYSTEM_DEFAULT`，并把该字符串映射到`rmw_qos_profile_system_default`；Humble的这个profile将history、depth、reliability和durability都留给RMW实现决定。[RealSense 4.58.4常量](https://raw.githubusercontent.com/realsenseai/realsense-ros/v4.58.4/realsense2_camera/include/constants.h)、[RealSense 4.58.4 QoS映射](https://raw.githubusercontent.com/realsenseai/realsense-ros/v4.58.4/realsense2_camera/src/ros_utils.cpp)、[Humble RMW默认profile](https://github.com/ros2/rmw/blob/humble/rmw/include/rmw/qos_profiles.h)。本次保存的相机参数文件只设置投射器，启动参数没有显式设置图像QoS；现场有效端点被发现为RELIABLE/TRANSIENT_LOCAL。现有记录不能说明系统/RMW配置中具体哪一项使durability解析为TRANSIENT_LOCAL，故不推断隐藏配置或驱动缺陷。RealSense ROS也允许按stream设置`<stream>_qos`，但本次启动命令没有覆盖它。[RealSense ROS官方说明](https://github.com/realsenseai/realsense-ros/blob/ros2-master/README.md)

候选只对图像订阅调用`image_qos.reliable()`，保持原KeepLast 5、VOLATILE、IMU QoS、Estimator、配对和输出不变。选择可靠订阅的理由是让接收端请求重传能力，并与实测的可靠发布端相配；代价是丢包时可能增加等待或数据时龄。它不保证及时交付，也不能弥补相机源流停顿。若现场图像发布端改为BEST_EFFORT，RELIABLE订阅会不兼容，必须先停止候选运行。

`VINS_IMAGE_QOS`未设置时保留原BEST_EFFORT默认；`VINS_IMAGE_QOS=best_effort`等价显式默认，`VINS_IMAGE_QOS=reliable`仅选择候选，其他值启动时报错。该开关只用于图像订阅。已同步到[可复现适配补丁](../../../scripts/go2_vins/patches/0001-vins-humble-cpu-and-odometry.patch)。

## 同一录包离线对照

输入为完整现场录包[`input_and_odometry`](../../../logs/real_go2/20261106_112612_vins_live_pairing_diagnostic/input_and_odometry)，仅播放左右红外和合并IMU三路；录包内原有`/vins/odometry`与`/vins/imu_propagate`未播放。两次使用同一个运行配置、同一速率1.0、同一图像发布QoS（RELIABLE/TRANSIENT_LOCAL，深度5；IMU BEST_EFFORT/VOLATILE，深度5）和ROS域42/localhost-only。深度5是回放设置，现场发布端真实history/depth仍未知。

| 指标 | 原BEST_EFFORT基线 | RELIABLE候选 |
|---|---:|---:|
| VINS收到左/右图，源录包810/706 | 809 / 704 | 810 / 706 |
| VINS收到header不在源CSV中的左/右图 | 0 / 0 | 0 / 0 |
| 配对 / 丢弃左 / 丢弃右 / dispatch | 704 / 105 / 0 / 704 | 706 / 104 / 0 / 706 |
| VINS右图回调最大到达间隔 | 0.799994 s | 0.799959 s |
| 最大配对header间隔 | 0.800066 s | 0.800066 s |
| 优化odom数量；最大接收间隔 / header间隔 | 342；0.944636 / 0.933411 s | 343；0.947134 / 0.933411 s |
| IMU传播数量；最大接收间隔 / header间隔 | 10480；0.110522 / 0.005002 s | 10508；0.010007 / 0.005002 s |
| VINS初始化完成；异常/失效日志 | 是；0 | 是；0 |
| VINS轨迹起止净位移模长 | 1.1424 m | 1.0503 m |

两次都完整启动并初始化；成功回放的播放器、输出录制器和VINS内层进程退出码均为0。接收间隔由输出录包的到达时间计算；基线传播输出header保持约5 ms，但录制到达时间出现110 ms间隔，说明接收间隔也受录制/本机调度影响。轨迹净位移受初始化与运行状态影响，不是距离真值或候选优劣指标。

候选在本次本机回放中收齐全部源header，且多到2张右图、1张左图；两种回放都只看到约0.8 s的右图最大源间隔，优化header最大间隔均约0.93 s，没有复现现场1.33–1.50 s或此前2.6 s空档。候选效果只支持“一次受限本机回放的交付完整度改善”。运行日志同时有FastDDS `getifaddrs`/UDP `Operation not permitted`，受限沙箱可能影响传输与调度，故不能代表现场实时链路。

第一次离线启动因沙箱内`/home/lidio/.ros/log`只读而失败，日志保留在`baseline_startup_failure/`（VINS SIGABRT，rosbag程序段错误，未处理传感器数据）。将`ROS_LOG_DIR`改到`/tmp`后基线与候选都正常结束；这属于离线环境写权限问题，不作为QoS结果。

结果和可复现文件均保存在[`logs/real_go2/20261106_vins_right_image_delivery_qos/`](../../../logs/real_go2/20261106_vins_right_image_delivery_qos/)：每次的`vins.log`、`bag_play.log`、`output_record.log`、`exit_codes.txt`和`summary.json`；共同回放QoS为`playback_field_qos.yaml`。补丁从固定提交重新应用，`git apply --check`、应用后`git diff --check`通过；Humble隔离构建的`camera_models`和`vins`通过，`ldd`没有未解析共享库。离线运行未访问相机。

## 下一次唯一推荐：一次相机-only候选验证

保持Go2关机、不接Go2 DDS、不启动任何机器人控制。沿用640×480@15 Hz双IR、gyro200 Hz/accel100 Hz、IMU合并方式2、深度和彩色关闭、投射器维持上次现场的关闭状态。已有1 m标记只作动作参考，不是精度门槛。建议记录约55秒：启动和初始化后静止10秒，沿标记人工平移相机约1 m、约10–15秒，停止后再静止约10秒；不运行机器人电机。若现场无法安全人工平移，则只做静止右图交付检查，并明确不评价动态输出。

先启动相机，再用`ros2 topic info -v`确认两个IR发布端当前均为RELIABLE；仅在此条件满足时运行候选。若不是RELIABLE，或图像话题有未知的多发布端，停止这次候选检查并保留现场QoS记录，不以候选不兼容掩盖数据。候选订阅应发现为RELIABLE/VOLATILE。

准备文件已放在`field_candidate/`：运行配置只把输出路径指向新目录，左右相机参数沿用当前候选；`camera_options.yaml`保持投射器关闭；[记录器QoS](../../../logs/real_go2/20261106_vins_right_image_delivery_qos/field_record_qos.yaml)按图像RELIABLE、IMU BEST_EFFORT、VINS输出RELIABLE建立订阅。

**终端A：相机。**

```bash
rtk mkdir -p /home/lidio/quadruped_robots/logs/real_go2/20261106_vins_right_image_delivery_qos/field_candidate/ros_logs
rtk env -i HOME=/home/lidio PATH=/usr/bin:/bin LANG=C.UTF-8 TERM=xterm \
  ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
  ROS_LOG_DIR=/home/lidio/quadruped_robots/logs/real_go2/20261106_vins_right_image_delivery_qos/field_candidate/ros_logs \
  bash --noprofile --norc -c '
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=camera device_type:=d435i \
  enable_depth:=false enable_color:=false \
  enable_infra1:=true enable_infra2:=true \
  depth_module.infra_profile:=640,480,15 \
  enable_gyro:=true enable_accel:=true gyro_fps:=200 accel_fps:=100 \
  enable_sync:=true unite_imu_method:=2 \
  config_file:=/home/lidio/quadruped_robots/logs/real_go2/20261106_vins_right_image_delivery_qos/field_candidate/camera_options.yaml
'
```

**终端B：确认图像端点QoS。** 两个话题都要确认只有预期相机发布端且Reliability为RELIABLE；否则停止候选流程。

```bash
rtk env -i HOME=/home/lidio PATH=/usr/bin:/bin LANG=C.UTF-8 TERM=xterm \
  ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
  ROS_LOG_DIR=/home/lidio/quadruped_robots/logs/real_go2/20261106_vins_right_image_delivery_qos/field_candidate/ros_logs \
  bash --noprofile --norc -c '
source /opt/ros/humble/setup.bash
ros2 topic info -v /camera/camera/infra1/image_rect_raw
ros2 topic info -v /camera/camera/infra2/image_rect_raw
'
```

**终端C：先开始记录，再启动VINS。** 录左右IR、IMU、优化odom及传播odom，不回放或混入旧VINS输出。

```bash
rtk env -i HOME=/home/lidio PATH=/usr/bin:/bin LANG=C.UTF-8 TERM=xterm \
  ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
  ROS_LOG_DIR=/home/lidio/quadruped_robots/logs/real_go2/20261106_vins_right_image_delivery_qos/field_candidate/ros_logs \
  bash --noprofile --norc -c '
source /opt/ros/humble/setup.bash
exec ros2 bag record \
  -o /home/lidio/quadruped_robots/logs/real_go2/20261106_vins_right_image_delivery_qos/field_candidate/input_and_vins \
  --qos-profile-overrides-path /home/lidio/quadruped_robots/logs/real_go2/20261106_vins_right_image_delivery_qos/field_record_qos.yaml \
  /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw \
  /camera/camera/imu /vins/odometry /vins/imu_propagate
'
```

**终端D：候选VINS。** 等日志出现`Initialization finish!`并稳定接收后，才开始上述人工平移。`tee`旁路保存完整同步诊断输出，Ctrl+C后shell记录VINS进程本身退出码。

```bash
rtk env -i HOME=/home/lidio PATH=/usr/bin:/bin LANG=C.UTF-8 TERM=xterm \
  ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 \
  ROS_LOG_DIR=/home/lidio/quadruped_robots/logs/real_go2/20261106_vins_right_image_delivery_qos/field_candidate/ros_logs \
  VINS_IMAGE_QOS=reliable VINS_SYNC_DIAGNOSTICS=1 \
  bash --noprofile --norc -c '
source /opt/ros/humble/setup.bash
source /tmp/go2_vins_build/install/setup.bash
set +e
stdbuf -oL /tmp/go2_vins_build/install/vins/lib/vins/vins_node \
  /home/lidio/quadruped_robots/logs/real_go2/20261106_vins_right_image_delivery_qos/field_candidate/runtime_config.yaml \
  --ros-args -r __ns:=/vins 2>&1 | tee /home/lidio/quadruped_robots/logs/real_go2/20261106_vins_right_image_delivery_qos/field_candidate/vins.log
pipe_status=( "${PIPESTATUS[@]}" )
printf "vins_exit=%s tee_exit=%s\n" "${pipe_status[0]}" "${pipe_status[1]}" > /home/lidio/quadruped_robots/logs/real_go2/20261106_vins_right_image_delivery_qos/field_candidate/vins_exit_codes.txt
'
```

先让操作者停止相机平移，再按顺序Ctrl+C停止VINS、录包器、相机。记录VINS内层退出码和录包器退出情况，最后核对本次`vins_node`、`realsense2_camera_node`和rosbag进程均已退出。

主要判断：将录包中的右图header集合与VINS `SYNC_DIAG receive`集合逐一比对；报告左右到达数、配对/drop/dispatch数量、回调与输出最大接收/header间隔和输入时龄。成功表现是源录包中可见的右图都进入VINS receive，配对/dispatch跟随实际输入，且不再出现无法由源图像间隔解释的长输出空档。若此候选仍有源包可见而回调缺失，记录端点QoS和同一header，不再调整队列、容差、executor或标定；若不复现，则只报告此场现场样本未复现。位移对1 m线只作行为检查，不用于判定可靠性修复。

出现VINS不初始化、右图数据中断或现场人员要求停止时，不开始平移；若正在平移，操作者先停止移动，再依上述顺序退出。采集器没有停止机器人的能力；本方案中Go2保持关机。

**后续（2026-11-06）：**该候选现场验证已于本日执行（[现场记录](../../../logs/real_go2/20261106_132345_vins_reliable_field_candidate/RESULT.md)），双目连续配对、退出码0；随后一次视频参考平移未复现秒级交付空档，但外部视频漏做同步动作，动态精度未验证。见[视频参考平移记录](20261106_VINS_VIDEO_REFERENCE_TRANSLATION.md)。

## 范围与影响

改动只在独立VINS适配补丁和隔离构建中。没有修改Go2共享控制器、模型、Recovery、导航或仿真配置；仿真工作线不受影响。候选配置没有接入实机launch，也未启用Recovery。离线构建/回放通过不等于相机实时跟踪或Go2速度精度通过。
