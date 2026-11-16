# D435i 无线跨机交付失败：离线定位

日期：2026-11-11。仅审阅既有记录、运行电脑宿主机只读核对，并查阅固定版本源码与配置文档。未SSH、未访问相机/机器人、未初始化DDS或启动节点。

## 结论

当前已确认是**电脑端无线跨机ROS图像与IMU交付失败**；不能归因于Orin本机UDP接收缓存，因为Orin已有独立、重复通过的8MiB `rmem_max`修复。电脑无线观察30秒收到Depth 2张、左右IR 16/12张、IMU 778条；深度最后样本时龄28.00秒，IR约21秒，IMU结束时龄0.40秒且中间最大间隔1.42秒。Depth两样本之间约80ms、算出的12.46Hz仅是两帧局部间隔，不是持续频率。CameraInfo 11条，最大间隔20.73秒。

相机窗口和电脑30秒观察确实重叠：观察器单调时刻为17827.812–17857.851；电脑单调时钟下首条CameraInfo约17829.350收到、首条Depth约17829.772收到。相机日志和ROS header使用另一时间基准：相机`Node Is Up`为1794379150.905，第一条Depth header为1794379151.055、第一条IR header为1794379151.132。只在相机header时间线上比较这些值；没有将电脑单调时间与相机时间相减。结束时龄字段将末尾停收单独保留，不能从已收样本的首末频率掩盖。

本次电脑UDP `RcvbufErrors`、`InErrors`、`MemErrors`增量均为0；两端`rmem_max`都是8MiB。电脑宿主机重新只读核对后仍为 `rmem_max=8388608`、`rmem_default=212992`，Humble/rclpy/消息类型/CycloneDDS RMW均可用，接口为无线`192.168.1.115`、有线DOWN。无相关相机/VINS进程。无需安装软件或继续增加接收缓存。

## `Out of frame resources!` 的证据边界

189条日志来自librealsense `synthetic-stream.cpp`帧处理回调，首次出现在相机`RealSense Node Is Up!`后约1秒。v2.51.1公开API将`rs2_allocate_synthetic_video_frame`定义为从processing block的frame source分配合成图像帧；返回帧需要释放，资源不足时实现可从free store分配。结合日志中的`Wrong Api Call Sequence] Out of frame resources!`，可确认SDK合成帧分配/处理路径报错；不能据此断言系统RAM耗尽或固定大小池已耗尽。profile打开记录只证明启动时配置被接受。

4.51.1 ROS驱动的`frame_callback`对视频帧调用`publishFrame`，图像publisher随后发布`Image`消息；这个串行回调和SDK帧生命周期使下游发布延迟成为一个可检验的背压路径，但现有日志没有回调耗时、SDK输入帧频或仍被引用帧数，不能证明DDS阻塞导致SDK错误。独立订阅器有少量样本，也不能单独区分相机回调停产与ROS/DDS交付停顿。端点QoS来自旧现场记录（RELIABLE/VOLATILE发布、BEST_EFFORT/VOLATILE订阅），本次没有新的端点快照；该QoS组合可通信，QoS差异本身不是不兼容证据。

## DDS配置与最小候选判断

本次两端已显式绑定当前Wi-Fi地址，CycloneDDS发现成功并收到少量消息，因此“完全绑定错接口”不符合现有证据；此前无线SSH及ping只证明IP连通，不证明DDS图像吞吐。现场为CycloneDDS 0.7.0，公开的0.8.2及更新文档不能代替0.7.0生效配置。当前没有保存该版本的运行配置日志或实际用户数据单播/组播路径，故不把Wi-Fi默认多播行为写成已确认事实。

`Discovery/Peers`候选主要指向参与者发现地址，现有证据不足以证明发现地址是主因，因此不建议把它作为本次第一个变量。版本源码依据：[RealSense ROS 4.51.1 `frame_callback`](https://github.com/realsenseai/realsense-ros/blob/4.51.1/realsense2_camera/src/base_realsense_node.cpp)、[同版图像publisher](https://github.com/realsenseai/realsense-ros/blob/4.51.1/realsense2_camera/src/image_publisher.cpp)、[librealsense 2.51.1合成帧分配API](https://github.com/realsenseai/librealsense/blob/v2.51.1/include/librealsense2/h/rs_frame.h)。后续CycloneDDS说明仅作背景，不用于断定本机0.7.0默认行为：[0.8.2配置说明](https://cyclonedds.io/docs/cyclonedds/0.8.2/config.html)。

目前没有足够依据直接改成显式peer/强制单播：绑定地址正确、已有跨机消息，且`Out of frame resources`在相机端快速出现；显式peer只针对发现地址，不能解释或确认消除帧池耗尽。没有代码缺陷已被本次记录证明，故不修改生产DDS配置、QoS、观察器、驱动或帧队列，也不做构建。

## 下一次唯一推荐：同profile有线跨机基线

使用已有电脑—Orin直连网段做**一次**30秒相机-only有线跨机观察。它只把传输路径从Wi-Fi改为有线；profile、QoS、缓存、DDS版本/域、观察器与时长保持一致，用来区分无线链路影响与跨机DDS/发布路径问题。若有线持续正常，问题范围偏向Wi-Fi链路/其多播与单播表现；若有线仍停收并同样报帧资源错误，优先查跨机DDS发布阻塞与SDK帧持有链。两种结果都不是无线最终修复证明。

Go2保持开机、静止（稳定趴伏或站立），确保机载Orin和相机供电；接上既有Orin—电脑有线连接，不发送机器人控制命令。只读确认电脑`enp7s0=192.168.123.99`、Orin有线口`192.168.123.18`、两端CycloneDDS RMW和domain42；若地址或接收缓存与记录不同，停止并保存现状，不改网络。电脑还需确认运行前没有本任务相机/观察器进程。Orin `rmem_max`沿用已安装8MiB配置，不改`rmem_default`。

主助手审阅修正：下面分终端片段仅作参数参考，现场应复用无线观察时的自动串联方式（电脑观察器后台启动后，立即经SSH启动限时相机），不在30秒窗口内再手工准备第二个终端。电脑采用已验证的干净系统Python/Humble环境，清除Anaconda的PYTHONHOME、PYTHONPATH、LD_LIBRARY_PATH后再加载Humble。设备进程匹配使用完整命令行，避免Linux进程短名称截断；机载旧版SCP不使用源路径结尾`/.`。

先在电脑确认有线地址、程序占用和缓存；若电脑`enp7s0`没有记录中的`192.168.123.99`、Orin有线口没有`192.168.123.18`，或接口/接收缓存与预期不同，则保存现状并停止，不修改网络配置。电脑生成一个唯一`RUN_ID`并记下它：

```bash
set -u
RUN_ID=$(date +%Y%m%d_%H%M%S)_wired_camera_check
RUN_DIR="$PWD/logs/real_go2/$RUN_ID"
mkdir -p "$RUN_DIR"
ip -brief address > "$RUN_DIR/pc_addresses.txt"
sysctl net.core.rmem_max net.core.rmem_default > "$RUN_DIR/pc_rmem.txt"
pgrep -f '[r]ealsense2_camera_node' > "$RUN_DIR/pc_camera_pids.txt" || true
pgrep -x vins_node > "$RUN_DIR/pc_vins_pids.txt" || true
cat "$RUN_DIR/pc_addresses.txt" "$RUN_DIR/pc_rmem.txt" "$RUN_DIR/pc_camera_pids.txt" "$RUN_DIR/pc_vins_pids.txt"

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.123.99</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
CAMERA_RUN="$RUN_DIR" /usr/bin/python3 logs/real_go2/20261111_pc_wireless_camera_pyqr8gos/observe_pc.py > "$RUN_DIR/observer.log" 2>&1 &
OBSERVER_PID=$!
echo "RUN_ID=$RUN_ID observer_pid=$OBSERVER_PID RUN_DIR=$RUN_DIR"
```

观察器固定运行30秒。输出包括各流电脑接收单调时间、header、结束时龄、CameraInfo、深度ROI和最后收到的图像。立即在Orin启动单一相机pipeline，使用同一`RUN_ID`保存到Orin独立临时目录；只在字段和profile预检相符时继续：

```bash
set -u
RUN_ID='<复制电脑打印的RUN_ID>'
RUN_DIR="/tmp/$RUN_ID"
mkdir -p "$RUN_DIR"
ip -brief address > "$RUN_DIR/orin_addresses.txt"
sysctl net.core.rmem_max net.core.rmem_default > "$RUN_DIR/orin_rmem.txt"
pgrep -f '[r]ealsense2_camera_node' > "$RUN_DIR/orin_camera_pids.txt" || true
pgrep -x vins_node > "$RUN_DIR/orin_vins_pids.txt" || true
cat "$RUN_DIR/orin_addresses.txt" "$RUN_DIR/orin_rmem.txt" "$RUN_DIR/orin_camera_pids.txt" "$RUN_DIR/orin_vins_pids.txt"

source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.123.18</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
timeout --signal=INT --kill-after=3s 35s ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/camera/camera -r __node:=camera \
  -p device_type:=d435i -p enable_depth:=true -p enable_color:=false \
  -p enable_infra1:=true -p enable_infra2:=true \
  -p depth_module.profile:=480,270,15 -p depth_module.emitter_enabled:=0 \
  -p enable_gyro:=true -p enable_accel:=true -p gyro_fps:=200 -p accel_fps:=100 \
  -p enable_sync:=true -p unite_imu_method:=2 -p diagnostics_period:=1.0 \
  > "$RUN_DIR/camera.log" 2>&1
CAMERA_EXIT=$?
echo "camera_timeout_exit=$CAMERA_EXIT" | tee "$RUN_DIR/exits.txt"
```

相机已运行后，在电脑记录各输入端点的实时QoS（单独保存，固定域/URI；只查询，不改QoS）：

```bash
for topic in \
  /camera/camera/depth/image_rect_raw \
  /camera/camera/depth/camera_info \
  /camera/camera/infra1/image_rect_raw \
  /camera/camera/infra2/image_rect_raw \
  /camera/camera/imu; do
  ros2 topic info --verbose "$topic"
done > "$RUN_DIR/endpoint_qos.txt" 2>&1

wait "$OBSERVER_PID"
OBSERVER_EXIT=$?
echo "observer_exit=$OBSERVER_EXIT" | tee "$RUN_DIR/observer_exit.txt"
mkdir -p "$RUN_DIR/orin"
scp -r "unitree@192.168.123.18:/tmp/$RUN_ID" "$RUN_DIR/orin/"
```

从相机日志核对序列号`238222071369`、USB类型、Depth与IR的480x270@15、gyro200/accel100、同步和emitter设置；不匹配就停止判读。电脑还需保留observer PID退出状态、`observation.json`、图像样本、端点QoS和相机超时退出状态。预检发现节点占用时不启动第二个pipeline；ROS进程只检查、不杀其他任务。

完成条件：30秒内Depth及双IR接近15Hz、IMU接近200Hz，无数秒停收；报告最大接收间隔和结束时龄。200ms只是后续射线链时龄参考，不由本次接收检查证明整个推理链满足。若任一图像停收/显著大间隔、相机出现连续帧资源错误、设备/profile不符或现场人员要求停止，保留记录并停止，不重启、不改QoS/profile/缓存、不追加分支。先结束相机并等待关闭日志，再核对相机、观察器、SSH及ROS辅助进程退出。机器人始终不运动。

## 离线处理与影响

本轮只增添本报告并更新`real_go2/README.md`索引；没有修改观察器或DDS候选配置，因此无需运行ROS/DDS离线测试或构建。没有初始化DDS、访问设备或启动进程。`Out of frame resources`相关帧资源/回调耗时仍待这一次有线同profile记录补齐。现有方案不改VINS双IR/IMU或ABS深度射线定义，也不影响仿真/A1工程。
