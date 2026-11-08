# 2026-11-08 VINS 构建恢复与 IMU 告警审阅

范围：离线恢复电脑 VINS 构建，审阅 2026-11-08 相机启动的 IMU 告警，准备下一次相机只读定位观察。本轮未连接机器人或相机，未初始化实机 DDS，未运行 VINS 或现场回放，未修改系统包/内核/固件。未提交 Git。

## 1. 结论

- 电脑 VINS 已按原方案恢复：固定上游提交 `72023bc5d2abb14faa4c02a5c040d84f996b587a`，应用仓库原补丁，Humble/CPU 隔离构建成功，`vins_node` 可执行，183 个动态依赖无缺失。
- `Motion Module failure` 是设备固件经 RealSense 通知解码器上报的 **HW error report**（`mm_failure`），不是 ROS 节点的软件异常；本次日志只在启动阶段出现，IMU 之后仍持续输出。
- `HID set_power 1 failed` 在本版本代码中可能只是 **“目标值已经是 1”导致的返回值语义问题**：`write_fs_attribute` 在值未变化时直接返回默认 `false`，上层据此打印 failed。该告警不能单独证明 accel buffer 被禁用。
- `IMU Calibration is not available` 表示当前SDK未取得可用的设备IMU标定，运行路径使用默认内参和外参；这不等于原始数据全无效，也不能仅凭日志断言设备EEPROM内完全没有标定数据。
- 约 200 Hz 的合并 IMU 说明该话题在统计窗内持续发布；按 `unite_imu_method=2` 的代码路径，它依赖 accel 样本存在。但频率本身不证明数值有效、时间戳连续、图像/IMU 严格同步或 VIO 精度。
- 下一次现场只需判断“定位链是否工作”，不预设 5 cm 精度；先观察静态初始化和跟踪，需要运动激励时由主助手另行安排原生遥控低速运动。

## 2. VINS 构建恢复

### 2.1 源码与补丁

- 工作源码：`/tmp/go2_vins_fusion_ros2`（临时目录，重启后需重建）。
- 上游：`zinuok/VINS-Fusion-ROS2`，固定提交 `72023bc5d2abb14faa4c02a5c040d84f996b587a`。
- 补丁：`scripts/go2_vins/patches/0001-vins-humble-cpu-and-odometry.patch`，`git apply --check` 通过，应用后 `git diff --check` 通过；28 个文件变更。
- 已确认补丁中的关键修复仍在：
  - `vins/src/rosNodeTest.cpp`：`sync_thread.join()`、`estimator.shutdown()`、`clearPub()`；
  - `vins/src/estimator/estimator.cpp`：`Estimator::shutdown()`；
  - `vins/src/rosNodeTest.cpp`：`VINS_IMAGE_QOS` 的 best_effort/reliable 选择；
  - `vins/src/utility/visualization.cpp`：世界系速度转为 body 系、`child_frame_id` 设置。

恢复命令（`/tmp` 丢失后可直接复用）：

```bash
git clone https://github.com/zinuok/VINS-Fusion-ROS2.git /tmp/go2_vins_fusion_ros2
cd /tmp/go2_vins_fusion_ros2
git checkout 72023bc5d2abb14faa4c02a5c040d84f996b587a
git apply --check /home/lidio/quadruped_robots/scripts/go2_vins/patches/0001-vins-humble-cpu-and-odometry.patch
git apply /home/lidio/quadruped_robots/scripts/go2_vins/patches/0001-vins-humble-cpu-and-odometry.patch
git diff --check
```

### 2.2 构建

使用系统 CMake 3.22、系统 Ceres 2.0、OpenCV 4.5.4、Eigen 3.4，避免 Anaconda 的 CMake 4.x/Python 污染：

```bash
env -i HOME=/home/lidio PATH=/usr/bin:/bin LANG=C.UTF-8 TERM=xterm \
  CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 \
  bash --noprofile --norc -c '
source /opt/ros/humble/setup.bash
cd /tmp/go2_vins_fusion_ros2
colcon --log-base /tmp/go2_vins_build/log build --packages-up-to vins \
  --parallel-workers 1 \
  --event-handlers console_direct+ \
  --build-base /tmp/go2_vins_build/build \
  --install-base /tmp/go2_vins_build/install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-DEIGEN_DONT_PARALLELIZE
'
```

结果：

- `Summary: 2 packages finished [2min 39s]`，退出码 0；仅有上游弃用/变长数组等既有警告。
- 可执行入口：`/tmp/go2_vins_build/install/vins/lib/vins/vins_node`。
- `ros2 pkg prefix vins` → `/tmp/go2_vins_build/install/vins`。
- `ros2 pkg executables vins` → `vins vins_node`。
- `ldd` 共 183 项，未发现 `not found`；包括系统 `libceres.so.2`。
- 构建日志：`logs/real_go2/20261108_vins_build/build.log`（本机保留，不随 Git 上传）。

未安装/升级任何系统包，未修改系统依赖。`/tmp` 内容不是持久资产，电脑重启后需重新 clone、应用同一补丁并重跑上述构建。

## 3. IMU 告警审阅

### 3.1 本次原始记录

来自 `logs/real_go2/20261108_073639_cyclone_field/`（原始文件未修改）：

- 相机：D435I，FW `05.13.00.55`，USB 2.1，480×270@15 双 IR，`Sync Mode: On`，gyro 200/accel 100，`unite_imu_method=2`。
- `IMU Calibration is not available, default intrinsic and extrinsic will be used.`（`ds5-motion.cpp:473`）。
- `HID set_power 1 failed for .../HID-SENSOR-200073.2.auto/iio:device0/buffer/enable`（`backend-hid.cpp:715`）。
- `Motion Module failure,...,Error,Hardware Error`（`ds5-private.h` 的 `mm_failure`）。
- 电脑接收：左图 15.015 Hz、右图 15.014 Hz、合并 IMU 199.995 Hz；12 秒窗内未见数秒级空档。
- 本次没有记录 gyro/accel 数值、有限性、header 时间连续性或通知是否重复。

### 3.2 Motion Module failure

直接证据：

- `src/ds5/ds5-private.h` 的 `ds5_fw_error_report` 将 `mm_failure` 映射为字符串 `Motion Module failure`。
- `src/ds5/ds5-device.cpp` 的 `ds5_notification_decoder::decode()` 对表中的 code 返回 `RS2_NOTIFICATION_CATEGORY_HARDWARE_ERROR`、`RS2_LOG_SEVERITY_ERROR` 和该字符串。
- 因此它是**设备固件通过 HW error report 上报的错误码**，不是 ROS 节点的异常推断。日志中的数字是通知携带的时间/值字段，不是可公开解释的故障码明细。

能说明什么：

- 设备在启动阶段报告过一次 Motion Module failure；同一通知在日志中出现两行，不能当作两次独立事件。
- 本次日志中没有看到它在 12 秒接收窗内重复；但不能据此证明未来不重复或数据必然健康。

不能说明什么：

- 不能确定内部故障原因（固件未提供更细解释）；
- 不能仅凭该通知判定 IMU 数据全部无效；
- 也不能因 200 Hz 正常就忽略该通知。

### 3.3 HID set_power ... failed

直接证据：

- `src/linux/backend-hid.cpp` 的 `iio_hid_sensor::set_power()` 调用 `write_fs_attribute(path, on)`；返回 false 时打印该 warning。
- `src/linux/backend-hid.h` 的 `write_fs_attribute()`：`res` 初始为 `false`；只有当前值 `cval != val` 时才写入并确认，随后设置 `res = (cval == val)`。**如果当前值已经等于目标值，函数不写入，直接返回 `false`**。
- 因此当 `buffer/enable` 已经是 `1` 时，`set_power(true)` 也会被上层记录为 “failed”。

能说明什么：

- 该 warning 本身不是“写失败”的充分证据；它可能是“已经是目标状态”的返回值语义问题。
- 该路径对应 accel 的 IIO buffer（`HID-SENSOR-200073.2.auto`，accel_3d）。

不能说明什么：

- 不能仅凭 warning 认定 accel buffer 被禁用或 accel 数据缺失；
- 也不能证明它与 Motion Module failure 有因果关系；两者时间上相隔约 0.9 秒，只是相关线索。

旁证：

- `realsense2_camera` 4.51.1 的 `FillImuData_LinearInterpolation()` 只在收到 accel 且历史足够时生成合并 IMU 消息；`CreateUnitedMessage()` 才发布 `/camera/camera/imu`。
- 本次合并 IMU 持续约 200 Hz，说明该代码路径在统计窗内确实看到了 accel 样本；这比“话题频率正常”更强，但仍不证明数值质量。

### 3.4 默认 IMU 标定

直接证据：

- `src/ds5/ds5-motion.cpp` 的 `mm_calib_handler` 读取 EEPROM 标定表；异常或无效时打印 `IMU Calibration is not available...`，随后使用默认 intrinsic/extrinsic。
- 本次日志明确出现该警告。

影响：

- 当前SDK路径没有取得可用的设备级IMU标定；VINS配置中的噪声和外参是上游默认值/SDK枚举初值，不是本机实测标定。
- 原始 gyro/accel 仍可输出；但姿态、尺度、速度和长期漂移不能按“已校准 IMU”看待。
- VINS `estimate_extrinsic: 1` 可在有运动激励时优化外参初值，但不能替代 IMU 标定，也不能在没有激励的静态数据上证明精度。

### 3.5 200 Hz 能证明与不能证明

能证明：

- 合并 IMU 话题在 12 秒窗内以约 200 Hz 发布/接收；
- 按 `unite_imu_method=2` 的实现，合并消息依赖 accel 样本存在，说明该窗口内 accel 数据进入了合并路径；
- gyro 名义 200 Hz、accel 名义 100 Hz，合并消息按 gyro 时间戳发布，accel 为线性插值。

不能证明：

- gyro/accel 数值有限、单位/轴向正确、无零值或卡值；
- header 时间戳单调、无重复/跳变；`ros2 topic hz --wall-time` 用的是到达时间，不是 header；
- 图像与 IMU 严格硬件同步；
- 无丢帧、无内部缓冲延迟；
- VIO 初始化成功、姿态/位置/速度准确。

下一次现场最少要记录：合并 IMU 的 gyro/accel 数值与有限性、静止时的 gyro 零偏和 accel 重力模长、header 时间差与连续性、Motion Module failure/HID 警告是否重复，以及 VINS 初始化/跟踪/优化与传播输出。

## 4. 下一次最小只读定位观察

目标：判断“Orin 采集 → 有线 CycloneDDS → 电脑 VINS”这条链是否能工作；不预设 5 cm 精度，不接 goto/Recovery/策略。

### 4.1 时间对齐（现场执行，本轮不执行）

Orin 重启后可能回到 1970。VINS 的双目/IMU 配对使用同一 Orin 时钟的 header，绝对时钟偏差不直接破坏同机配对；但输出时间、bag 时间和跨机对比需要对齐。

现场在启动相机前：

```bash
# 电脑
date -u +%s
# Orin（把 <PC_EPOCH> 换成上一条输出）
sudo date -u -s @<PC_EPOCH>
date -u
```

记录两台机器 `date -u` 的结果。Orin 无 NTP/RTC，之后可能漂移；每轮现场重新对齐。若不允许改时间，则记录实际偏差，并明确 VINS 输出时间戳来自 Orin 时钟。

### 4.2 相机（Orin）

```bash
ssh unitree@192.168.123.18
# 若出现 ros:foxy(1) noetic(2) ?，输入 1
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.123.18</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
ls -l /opt/ros/foxy/lib/librmw_cyclonedds_cpp.so
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/camera/camera -r __node:=camera \
  -p device_type:=d435i -p enable_depth:=false -p enable_color:=false \
  -p enable_infra1:=true -p enable_infra2:=true \
  -p depth_module.profile:=480,270,15 -p depth_module.emitter_enabled:=0 \
  -p enable_gyro:=true -p enable_accel:=true -p gyro_fps:=200 -p accel_fps:=100 \
  -p enable_sync:=true -p unite_imu_method:=2
```

保存完整相机 stdout（包含 IMU 标定、HID、Motion Module 通知和是否有重复）。若设备缺失、权限失败、右路错误或 CycloneDDS 报错，停止并保留输出。

### 4.3 电脑：QoS 与记录

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.123.99</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
ros2 topic list --no-daemon
ros2 topic info -v /camera/camera/infra1/image_rect_raw
ros2 topic info -v /camera/camera/imu
```

预期话题：`/camera/camera/infra1/image_rect_raw`、`/camera/camera/infra2/image_rect_raw`、`/camera/camera/imu`。确认实际名称和发布端 reliability；VINS 图像/IMU 默认使用 `SensorDataQoS` best-effort，因此设置 `VINS_IMAGE_QOS=best_effort`。若实际发布端为 reliable，才考虑 reliable；不要盲目沿用之前电脑侧可靠订阅候选。

记录最小数据（在 VINS 启动前开始，避免漏掉初始化）：

```bash
RUN=/home/lidio/quadruped_robots/logs/real_go2/$(date +%Y%m%d_%H%M%S)_vins_observe
mkdir -p "$RUN"
ros2 bag record -o "$RUN/bag" \
  --qos-profile-overrides-path /home/lidio/quadruped_robots/scripts/go2_vins/field_record_qos_best_effort.yaml \
  /camera/camera/imu /vins/odometry /vins/imu_propagate
```

bag 记录 IMU 数值和 VINS 优化/传播输出；`/camera/camera/imu` 的 QoS 为 best_effort，VINS 输出按 reliable 记录。若实际话题不同，先按 `ros2 topic list` 修正再记录。

### 4.4 电脑：VINS

以下命令中的 `$RUN` 替换为 4.3 创建的目录；不同终端不共享变量。

基准配置的output_path是固定历史目录，不能直接复用输出路径。每次使用独立运行配置，保留左右相机文件的相对路径；不要把cam0_calib/cam1_calib改成绝对路径（当前移植实现会再拼接配置目录）。先在同一VINS终端设置本轮实际RUN，然后生成副本：

```bash
export RUN
python3 - <<'PY'
import os, shutil
from pathlib import Path
src = Path('/home/lidio/quadruped_robots/scripts/go2_vins/config/d435i_480x270')
run = Path(os.environ['RUN'])
dst = run / 'config'
dst.mkdir(parents=True, exist_ok=False)
for name in ['left.yaml', 'right.yaml']:
    shutil.copy2(src / name, dst / name)
lines = []
for line in (src / 'realsense_stereo_imu.yaml').read_text().splitlines():
    if line.startswith('output_path:'):
        line = 'output_path: "' + str(run) + '"'
    elif line.startswith('pose_graph_save_path:'):
        line = 'pose_graph_save_path: "' + str(run / 'pose_graph') + '"'
    lines.append(line)
(dst / 'realsense_stereo_imu.yaml').write_text('\n'.join(lines) + '\n')
PY
```

```bash
source /opt/ros/humble/setup.bash
source /tmp/go2_vins_build/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.123.99</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
export VINS_IMAGE_QOS=best_effort
export VINS_SYNC_DIAGNOSTICS=1
export ROS_LOG_DIR="$RUN/ros_logs"
mkdir -p "$ROS_LOG_DIR"
cd "$RUN/config"
/tmp/go2_vins_build/install/vins/lib/vins/vins_node \
  "$RUN/config/realsense_stereo_imu.yaml" \
  --ros-args -r __ns:=/vins 2>&1 | tee "$RUN/vins.log"
```

配置：`d435i_480x270`，`show_track: 0`；不改 640×480 历史配置，不改 VINS 算法、噪声或标定。

判断顺序：

1. 日志出现图像接收/配对（`SYNC_DIAG`），说明 topic/QoS/时间戳路径工作；
2. 观察 `feature tracking` 的 `n_pts`；静态场景特征过少时会出现 `feature tracking not enough`；
3. 观察 `Initialization finish!` 与 `/vins/odometry` 是否开始发布；
4. 若初始化，记录 `/vins/odometry`（图像优化后状态）与 `/vins/imu_propagate`（IMU 高频传播）分别的时间和数值；不能只看传播频率判断定位健康；
5. 输出参考点仍是相机 IMU；转成 Go2 机身反馈还需安装外参和参考点转换，本轮不做。

### 4.5 完成条件、停止条件与清理

完成条件：相机/VINS 进程稳定，图像与 IMU 有配对/跟踪记录，VINS 输出（若初始化）有可读的位姿/速度和时间；数据足以判断定位链是否工作，不要求精度达标。

首次静态窗口约60秒，未初始化也按时结束并保留数据；不能因缺少运动激励时未初始化，就直接判定相机或VINS不可用。需要激励时再安排受控原生遥控，不通过无限等待、放宽参数或自动移动制造初始化成功。

停止条件：相机缺失/权限/右路错误；VINS 无图像或无 IMU；持续数秒级中断；VINS 崩溃/失跟踪不可恢复；现场人员要求停止；任何与预期不符的机器人状态。

停止顺序：

1. VINS 终端 Ctrl+C，等待退出码 0，检查日志无 `terminate called`、FastDDS/Publisher 清理错误；
2. 记录器 Ctrl+C，等待 bag 正常关闭；
3. 相机终端 Ctrl+C，等待 SSH 提示符；
4. Orin 检查 `pgrep -af '[r]ealsense2_camera_node'`；电脑检查 `pgrep -af '[v]ins_node|[r]os2 bag record'`；
5. 如有残留，确认来源后仅处理本次任务进程；不关闭 Unitree 原生服务。

全程不发送机器人运动、停止、恢复或模式切换指令。下一次先观察静态初始化和跟踪；需要运动激励时由主助手另行安排用户原生遥控低速运动，本轮不自行安排或运行。

## 5. 未决与边界

- Motion Module failure 的固件内部原因未知；本次仅一次启动通知，未记录是否重复和完整 IMU 数值。
- HID set_power warning 的“已等于目标值”解释基于 v2.51.1 源码；现场仍应记录实际 buffer/数据，不把它当成已消除的硬件故障。
- 当前SDK未取得可用IMU标定而使用默认值；VIO精度/尺度/姿态/速度不能按已校准系统评价，不将该日志扩大解释为设备内没有任何标定数据。
- 200 Hz 合并 IMU 是 gyro 速率 + accel 线性插值；不是 200 Hz 独立 accel。
- 480×270 双 IR 与 640×480 的限制、安装外参、时间同步和动态精度仍未验证。
- 构建源码/安装树在 `/tmp`，重启后需按本文命令重建。

## 6. 证据位置

- 本次现场原始记录：`logs/real_go2/20261108_073639_cyclone_field/`（未修改）。
- 传输结果记录：[20261108_CYCLONE_CAMERA_TRANSPORT_RESULT.md](20261108_CYCLONE_CAMERA_TRANSPORT_RESULT.md)。
- 构建日志：`logs/real_go2/20261108_vins_build/build.log`（本机保留）。
- 补丁：[scripts/go2_vins/patches/0001-vins-humble-cpu-and-odometry.patch](../../../scripts/go2_vins/patches/0001-vins-humble-cpu-and-odometry.patch)。
- 配置：[scripts/go2_vins/config/d435i_480x270/](../../../scripts/go2_vins/config/d435i_480x270/)。
- 记录 QoS：[scripts/go2_vins/field_record_qos_best_effort.yaml](../../../scripts/go2_vins/field_record_qos_best_effort.yaml)。
- 现场检查：[scripts/go2_vins/CYCLONE_FIELD_CHECK.md](../../../scripts/go2_vins/CYCLONE_FIELD_CHECK.md)。
