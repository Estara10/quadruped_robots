# D435i 联合流中深度图交付缺口

日期：2026-11-11。离线审阅本机已保存的 Orin 观察记录和 realsense-ros 4.51.1 / librealsense 2.51.1 对应源码。未连接 Go2/Orin 或相机，未初始化 DDS，未启动相机、VINS或控制器。

## 结论

本次记录能证明 Depth 帧到达了 ROS 驱动的深度帧处理路径，但不能证明 ROS Image 已持续发出或交付到观察器。证据把缺口缩小到 `CameraInfo` 发布之后的 ROS Image 转换/发布、DDS/RMW交付或订阅器接收阶段；目前不能在这几者之间定因。

这不支持“深度传感器完全没有产出”。也不能据此排除 USB2 联合流负载或 SDK/驱动问题：相机日志的 profile 打开成功只表示启动时打开了请求配置，不代表整个观察期都持续输出。

## 观察数据与统计口径

有效记录是 `logs/real_go2/20261111_d435i_joint_profile_TqAWuIYI/go2_camera_joint_TqAWuIYI/observation_timed.json`。空的 `observation.json` 对应观察器晚于相机退出才启动，是编排失误，不作为故障证据。

观察器在 Orin 本机运行15.03秒，以 `qos_profile_sensor_data` 订阅 Depth Image、左右 IR、合并 IMU 和 Depth CameraInfo。回调用单调时钟累计首末接收时刻、内部最大间隔和消息数；没有逐帧时间记录，也没有记录观察器结束的单调时刻。

| 话题 | 样本 | 记录结果 |
|---|---:|---|
| Depth Image | 4 | 接收时间跨度0.598秒，header跨度0.691秒；局部首末接收频率约5.01 Hz，内部最大间隔466.5 ms。这个频率不能代表持续频率。|
| Depth CameraInfo | 137 | 首末频率约15.01 Hz，内部最大间隔68.6 ms。|
| 左/右 IR | 各135 | 首末频率约15.02 Hz，持续到本次观察末段。|
| 合并 IMU | 1811 | 首末频率约199.33 Hz，1811条六轴值均有限。|

最后一张 Depth Image 到最后一条其他流接收记录相差8.515秒。该值不是独立测得的“到观察结束”间隔：JSON没有记录准确的观察器结束单调时刻。它显示深度停止接收远早于其他流的最后一条样本；`max_rx_gap` 只覆盖已收到的四帧，不包含其后的空档。CameraInfo持续到达也不等于图像已交付。

相机仅一个pipeline，Depth Z16与IR1/IR2 Y8均为480×270@15，gyro 200 Hz、accel 100 Hz、`unite_imu_method=2`、`enable_sync=true`、emitter关闭、color关闭。设备为D435i 238222071369，固件05.13.00.55，Orin USB树480M / SDK报告USB2.1；观察器与相机节点同在Orin，不涉及电脑跨机图像链路。

## 对应版本的数据路径

固定版本的源码显示：视频流回调在 `enable_sync` 时交给异步同步器，frameset回调再逐帧处理 Depth。Depth进入 `publishFrame` 后先构造深度Mat并按设备scale处理；随后发布CameraInfo，再分配/转换ROS Image并调用 `image_publisher->publish`，最后才发布该帧metadata。因观察器订阅了CameraInfo，驱动源码中的订阅条件满足；137个递增时间戳的Depth CameraInfo因此是深度帧持续进入驱动处理函数、通过深度scale处理并走到CameraInfo发布步骤的直接证据。

- `enable_sync` 与帧回调路径：[4.51.1 `rs_node_setup.cpp`](https://github.com/realsenseai/realsense-ros/blob/4.51.1/realsense2_camera/src/rs_node_setup.cpp#L1118-L1129)、[4.51.1 `base_realsense_node.cpp`](https://github.com/realsenseai/realsense-ros/blob/4.51.1/realsense2_camera/src/base_realsense_node.cpp#L524-L558)。
- CameraInfo先于Image转换和发布；metadata在Image发布调用之后，且该版本函数签名默认开启metadata：[4.51.1 `base_realsense_node.cpp`](https://github.com/realsenseai/realsense-ros/blob/4.51.1/realsense2_camera/src/base_realsense_node.cpp#L996-L1040)、[header定义](https://github.com/realsenseai/realsense-ros/blob/4.51.1/realsense2_camera/include/base_realsense_node.h#L176-L182)。metadata可作下一次检查的阶段标记，但收到metadata仍不等同于订阅端收到Image。
- 驱动传感器回调在帧回调处理后更新频率诊断；异常会记录 `An error has occurred during frame callback`：[4.51.1 `ros_sensor.cpp`](https://github.com/realsenseai/realsense-ros/blob/4.51.1/realsense2_camera/src/ros_sensor.cpp#L43-L59)。正的 `diagnostics_period` 可发布各启用流的频率信息：[4.51.1 README](https://github.com/realsenseai/realsense-ros/blob/4.51.1/README.md#L407-L412)。

本次相机日志中未发现上述frame callback异常或图像分配失败日志。CameraInfo持续而Image观察器仅收四帧，故同步器“完全没有收到深度帧”不能解释全段差异；但源码调用 `publish()` 也不能证明DDS读端实际收到消息。

尚存候选：Image消息分配/转换后的发布路径；ROS publisher或RMW/DDS传输；观察器自身的订阅/执行路径。观察器是单线程 `spin_once`，Image QoS队列深度5，统计不含逐帧日志；没有记录现场Image/CameraInfo/metadata端点QoS、第二个独立Image接收器或驱动频率诊断。该观察器的统计缺项降低定位能力，但现有证据没有表明它制造了“4帧”现象。第一次空观察文件则已明确是启动顺序错误。

Motion Module/HID/默认IMU标定告警，以及第二次的RGB hue控制超时都没有足够证据解释Depth Image缺口；不能归因为USB带宽、相机损坏或无线网卡干扰。USB2低带宽仍是待验证条件，不是已证根因。

## 深度单位边界

`depth_units_param.txt`记录的是 `Parameter not set.`；CLI退出码0和枚举的0.001选项默认值都不是设备实读scale。4.51.1驱动从SDK `depth_sensor.get_depth_scale()`读取设备scale，再在发布ROS `16UC1`前将像素换算到毫米单位：[scale读取位置](https://github.com/realsenseai/realsense-ros/blob/4.51.1/realsense2_camera/src/rs_node_setup.cpp#L307-L310)、[毫米换算实现](https://github.com/realsenseai/realsense-ros/blob/4.51.1/realsense2_camera/src/base_realsense_node.cpp#L170-L206)。因此 `depth_last.bin` 是ROS发布消息数据的副本，按该版本实现其单位为毫米；它不是SDK原始Z16直接转储，也没有提供设备scale的实读值。本轮不做米制深度预测。

## 修改与验证

没有发现足以修复的观察器或适配脚本逻辑错误，因此没有改代码、没有改驱动/profile/QoS，也不需要构建。已离线核对本次JSON、日志、观察器实现及固定版本源码调用顺序；历史原始记录未改。代码审阅只能缩小数据链路边界，不能代替下一次现场观测。

## 下一次唯一推荐检查：同一pipeline的阶段对照

当前相机仍接Go2机载Orin，测试时保持Go2开机、稳定趴伏，保留机载电脑和相机供电；不发送机器人控制命令，不开Viewer。只有另有已确认的独立供电时，才可关闭Go2本体而继续检查Orin相机链路。仅重复一次既有480×270@15的Depth+双IR+IMU profile；唯一新增项是启动时 `diagnostics_period:=1.0`。从同一个Orin本机观察器接收Image/CameraInfo，并同时记录驱动 `/diagnostics`、Depth metadata及Image端点QoS。不要改分辨率、频率、emitter或订阅QoS。

启动相机45秒计时前，先完成两个SSH终端的登录、终端B的ROS环境和输出目录准备，并备好订阅命令。相机日志出现 `RealSense Node Is Up!` 后立即在已准备的终端B启动订阅，确保完整15–20秒观察窗与相机运行重叠；不能等相机开始后才登录并逐段准备终端B。若准备超时导致窗口不重叠，记为编排失败，不作为相机故障证据。以下命令按用途分节，实际环境准备应先于相机启动。

先在电脑将已保存观察器复制到Orin（不依赖机载`/tmp`仍保留旧副本）：

```bash
scp logs/real_go2/20261111_d435i_joint_profile_TqAWuIYI/go2_camera_joint_TqAWuIYI/observe.py \
  unitree@192.168.123.18:/tmp/go2_depth_delivery_observe.py
```

在电脑终端先按既有现场路径登录Orin；若SSH菜单提示 `ros:foxy(1) noetic(2)`，选择1：

```bash
ssh unitree@192.168.123.18
```

在Orin终端A运行以下相机命令：

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.123.18</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
timeout --signal=INT --kill-after=3s 45s ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/camera/camera -r __node:=camera \
  -p device_type:=d435i -p enable_depth:=true -p enable_color:=false \
  -p enable_infra1:=true -p enable_infra2:=true \
  -p depth_module.profile:=480,270,15 -p depth_module.emitter_enabled:=0 \
  -p enable_gyro:=true -p enable_accel:=true -p gyro_fps:=200 -p accel_fps:=100 \
  -p enable_sync:=true -p unite_imu_method:=2 -p diagnostics_period:=1.0
```

确认启动日志中的序列号、USB类型和全部profile与本次一致后，再在Orin第二个终端（ROS菜单同样选Foxy）创建唯一输出目录，避免覆盖历史文件：

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.123.18</NetworkInterfaceAddress></General></CycloneDDS>'
RUN_DIR=/tmp/go2_depth_delivery_$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUN_DIR"
echo "$RUN_DIR"
```

终端B保持相同ROS环境和当前shell，创建好输出目录后并行运行诊断、metadata、独立 `topic hz` 和既有观察器；随后保存三个端点信息。这样只需两个终端，所有订阅器都在同一个15–20秒窗口内：

```bash
timeout --signal=INT --kill-after=2s 20s ros2 topic echo /diagnostics > "$RUN_DIR/diagnostics.yaml" 2>&1 &
timeout --signal=INT --kill-after=2s 20s ros2 topic echo /camera/camera/depth/metadata > "$RUN_DIR/depth_metadata.yaml" 2>&1 &
timeout --signal=INT --kill-after=2s 20s ros2 topic hz /camera/camera/depth/image_rect_raw --window 15 --wall-time > "$RUN_DIR/depth_hz.txt" 2>&1 &
CAMERA_RUN="$RUN_DIR" python3 /tmp/go2_depth_delivery_observe.py > "$RUN_DIR/observer.log" 2>&1 &
ros2 topic info -v /camera/camera/depth/image_rect_raw > "$RUN_DIR/depth_image_qos.txt"
ros2 topic info -v /camera/camera/depth/camera_info > "$RUN_DIR/depth_info_qos.txt"
ros2 topic info -v /camera/camera/depth/metadata > "$RUN_DIR/depth_metadata_qos.txt"
wait
```

结果判读：`/diagnostics` Depth频率反映驱动中SDK帧回调入口，CameraInfo反映帧进入发布函数；若metadata连续而两个Image接收器仍无消息，说明驱动走过Image发布调用，优先定位Image DDS/订阅端；若metadata无而CameraInfo持续，先确认metadata端点存在，再检查Image构造/发布调用之前的错误；若诊断继续而CameraInfo停更，缺口在同步器之后的frameset处理；若诊断与CameraInfo都停更，缺口更靠近设备/SDK回调。若原观察器缺帧而独立 `topic hz` 收到图像，才支持观察器侧问题。本次运行不改变profile、QoS或算法，不做扫描。

所有监视最长20秒，结束后在相机终端按Ctrl+C并等传感器关闭日志完成；timeout的预定SIGINT返回124属预期限时退出。核对 `/tmp` 新目录中的文件及相机节点、观察器、`ros2 topic echo`、`ros2 topic hz`均已退出。若出现设备/profile不符、驱动错误或现场人员要求停止，立即结束本轮并保留记录；若Depth持续缺图，达到既定观察时长后结束，不重启或换profile。成功条件仅是能用诊断、metadata、CameraInfo、Image接收和端点QoS定位交付边界；不是深度精度或ABS避障验收。

如果下次仍只有Depth Image缺失，需据阶段证据再选择是驱动发布路径问题还是DDS/读端问题；如果联合配置本身不能维持，当前ABS深度射线输入不能用该配置持续提供。保留双IR+IMU可继续定位验证但没有深度射线；保留Depth并削减其他流是否受设备支持、对VINS双目输入的影响都尚未验证，不在本轮预判或更改。
