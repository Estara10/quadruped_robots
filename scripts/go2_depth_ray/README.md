# Go2 D435i 深度→ABS射线只读观察

> 2026-11-16新增同帧双模型只读模式：相机当前已从机器人拆下。下文旧跨机示例是历史网络观察命令，不能照搬其IP/domain到下一次本机相机观察。新模式与测量方法见[双模型准备报告](../../docs/thesis_project/real_go2/20261116_D435I_DUAL_MODEL_READONLY_OBSERVER.md)。

入口 [`../go2_depth_ray_observer.py`](../go2_depth_ray_observer.py) 只订阅已有ROS深度话题，不打开SDK pipeline。默认观察30秒；现场60Hz检查可显式设42秒，以覆盖约36秒相机运行和末尾停收。它只写逐帧CSV与汇总JSON，不保存图像、不发布射线协议、不访问共享内存或控制器。

## 处理语义

- 显式要求 `--model`，CPU单线程加载现有TorchScript权重并做一次预热。
- 只接受本轮已测的 `480×270 16UC1` ROS深度（毫米），按0.001转米；模型输入为`160×90`、截断到`[0.1,6]m`、`log2`并复制为3通道。
- 复用既有中心采样坐标。候选有效掩码只看被采样中心像素是否非零；同时记录原严格适配器的3×3足迹全有效掩码比例。中心零值仍是未知，不填补深度。
- 对每个实际进入处理的原始深度帧，复用同一 `source_valid` 记录全图零值比例和原图中央ROI `x=[220,260), y=[115,155)` 的零值比例；被最新帧槽覆盖、未进入处理的帧仍只计覆盖，不另行扫描像素。
- 若未知中心存在且没有显式 `--unknown-fill-m`，该帧只记掩码并跳过推理。指定占位值后，未知格仅为诊断推理临时填入该数值，CSV/JSON保留未知比例及占位值，结果始终标记 `prediction_valid_for_control=false`。占位不代表障碍距离，也不保证网络输出保守。
- ROS回调只记录header和电脑单调接收时刻，将消息交给单个最新待处理槽。推理期间新帧覆盖旧待处理帧并增加计数；不会排队积累或重复推理旧帧。DDS/rclpy自身的回调缓存不由此槽控制。
- 订阅深度及CameraInfo（SensorData QoS）。不比较相机header与PC单调时钟；没有可靠对时，采集到电脑的时龄字段留空。
- 有限观察收到Ctrl+C时，程序接住`KeyboardInterrupt`或rclpy context关闭产生的`ExternalShutdownException`，进入同一清理路径并写出结果；后者本轮只做静态代码与语法核对，未启动ROS/DDS发信号验证。

## 离线自检

无需source ROS，不导入rclpy或初始化DDS：

```bash
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/abs-depth-ray-selfcheck-mpl /usr/bin/python3 \
  scripts/go2_depth_ray_observer.py \
  --model archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/ray_pred_mujoco_finetuned_best.pt \
  --offline-self-check \
  --self-check-input-dir logs/real_go2/20261111_075040_d435i_new_position_standing \
  --unknown-fill-m 0.1
```

自检把向量化中心样本和`realsense_depth_to_abs_rays.py`逐格比较，并用合成慢处理线程检查只留最新帧、覆盖计数及不重复处理。占位推理仅输出终端JSON。

## 现场只读检查

### 本机连接相机：medium / epoch3同帧对照（本轮准备，现场未运行）

使用时先核对D435i已装回、相机和ROS环境及本机进程占用。为相机和observer选择一个当前未占用的本机ROS domain，并使用`ROS_LOCALHOST_ONLY=1`；不要复用历史跨机网卡、IP或domain。只启动一个RealSense pipeline，observer不访问SDK、不创建第二个pipeline。

用同一块平整目标面依次摆在0°、左右±27°、较外侧±36°，每个摆位单独执行下方一次有限观察；换位时Go2保持站立静止、相机不动。记录目标面实测位置。若不能确定base_link射线起点地面垂足，只作定性方向比较，不报告精确距离。

电脑单终端命令（相机与observer自动重叠，40秒相机、30秒观察，先留时间读回设置）：

```bash
cd /home/lidio/quadruped_robots
set -euo pipefail
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
read -r -p '输入已只读核对为未占用的本机ROS domain整数: ' ROS_DOMAIN_ID
[[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || { echo 'ROS_DOMAIN_ID必须是整数' >&2; exit 2; }
export ROS_DOMAIN_ID
RUN_DIR="logs/real_go2/$(date +%Y%m%d_%H%M%S)_dual_ray_readonly"
mkdir "$RUN_DIR"
printf 'RUN_DIR=%s\n' "$RUN_DIR"

timeout --signal=INT --kill-after=4s 40s ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=camera device_type:=d435i \
  enable_depth:=true enable_color:=false enable_infra1:=true enable_infra2:=true \
  depth_module.depth_profile:=480,270,60 depth_module.infra_profile:=480,270,60 enable_sync:=true \
  hole_filling_filter.enable:=true \
  >"$RUN_DIR/camera.log" 2>&1 &
CAMERA_PID=$!
cleanup_camera() { kill -INT "$CAMERA_PID" 2>/dev/null || true; wait "$CAMERA_PID" 2>/dev/null || true; }
trap cleanup_camera EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
sleep 3
ros2 param set /camera/camera hole_filling_filter.holes_fill 1 | tee "$RUN_DIR/filter_mode.txt"
ros2 param set /camera/camera depth_module.emitter_enabled 1 | tee "$RUN_DIR/emitter.txt"
ros2 param get /camera/camera hole_filling_filter.enable | tee -a "$RUN_DIR/filter_mode.txt"
ros2 param get /camera/camera hole_filling_filter.holes_fill | tee -a "$RUN_DIR/filter_mode.txt"
ros2 param get /camera/camera depth_module.emitter_enabled | tee -a "$RUN_DIR/emitter.txt"

set +e
timeout --signal=INT --kill-after=4s 34s /usr/bin/python3 scripts/go2_depth_ray_observer.py \
  --model archive/ray_pred_finetune/mujoco_finetune_medium_20260611/ray_pred_mujoco_finetuned_best.pt \
  --compare-model logs/real_go2/20261115_d435i_ray_adaptation_data/training_runs/d435i_medium_layer4fc_20261115/ray_pred_mujoco_finetuned_best.pt \
  --depth-topic /camera/camera/depth/image_rect_raw \
  --camera-info-topic /camera/camera/depth/camera_info \
  --input-depth-stage sdk-hole-filled --unknown-fill-m 6.0 \
  --duration-s 30 --output-dir "$RUN_DIR/observer" \
  > >(tee "$RUN_DIR/observer.log") 2>&1
OBSERVER_RC=$?
if [ "$OBSERVER_RC" -ne 0 ]; then kill -INT "$CAMERA_PID" 2>/dev/null || true; fi
set +e
wait "$CAMERA_PID"
CAMERA_RC=$?
set -e
trap - EXIT INT TERM
printf 'observer=%s\ncamera_timeout=%s\n' "$OBSERVER_RC" "$CAMERA_RC" >"$RUN_DIR/exits.txt"
cat "$RUN_DIR/exits.txt"
```

过滤器enable由ROS launch参数设置；mode和投射器在节点启动后设置并读回。期望值分别为`true`、`1`、`1`。任一设置或读回失败时应停止观察、保留日志并退出，不把输入标成SDK mode1补洞；observer不执行第二次补洞。相机40秒超时通常返回124，此值本身不等于传感器关闭失败；核对`camera.log`、summary和本机进程已退出。若observer报错会向camera发送SIGINT；Ctrl+C时也需核对两个进程均退出。

第一组`ray_00_m..ray_10_m`对应`--model` medium，第二组`compare_ray_00_m..compare_ray_10_m`对应`--compare-model` epoch3。共同的header stamp与receive index表示同帧，耗时字段分开记录；接收到两模型完成的处理时龄计到第二模型结束。输入全图/中央ROI零值比例指**收到的图像**；若它已过SDK filter，单一话题无法还原补洞前的未知掩码，JSON会留空而不猜测。

相机装回后保持机器人静止。ABS束起点为base_link水平面`(-0.05,0,0)m`，−角在右、+角在左，每9°一束。卷尺从该起点的地面垂足沿束方向量到障碍交点，不能从镜头或红十字代替。当前相机安装仍只是近似外参；首次对照只作定性响应判断，详见[机身束测量示意与准备报告](../../docs/thesis_project/real_go2/20261116_D435I_DUAL_MODEL_READONLY_OBSERVER.md)。

历史网络参数（运行前由主助手只读确认）：PC `enp7s0 / 192.168.123.99`，Orin `eth0 / 192.168.123.18`，CycloneDDS、ROS domain 42。不要为此改网络或系统时钟。先确认profile、两端地址/RMW、Orin无其他相机节点占用，并保证有线链路仍连接。

1. PC终端：

```bash
cd /home/lidio/quadruped_robots
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.123.99</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
RUN_DIR="logs/real_go2/$(date +%Y%m%d_%H%M%S)_depth_ray60_readonly"
mkdir "$RUN_DIR"
printf 'RUN_DIR=%s\n' "$RUN_DIR"
set -o pipefail
/usr/bin/python3 scripts/go2_depth_ray_observer.py \
  --model archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/ray_pred_mujoco_finetuned_best.pt \
  --duration-s 42 --unknown-fill-m 0.1 \
  --output-dir "$RUN_DIR/observer" 2>&1 | tee "$RUN_DIR/observer.log"
OBSERVER_RC=${PIPESTATUS[0]}
printf 'observer=%s\n' "$OBSERVER_RC" >"$RUN_DIR/observer.exit"
cat "$RUN_DIR/observer.exit"
```

2. 在PC另一终端，先把上一步打印的实际`RUN_DIR`赋给变量，再启动**一个**Orin相机pipeline。启动观察器后尽快执行，确保时间窗口重叠：

```bash
cd /home/lidio/quadruped_robots
RUN_DIR='logs/real_go2/<上一步打印的目录名>'
set -o pipefail
ssh -tt -b 192.168.123.99 -o BatchMode=yes -o ConnectTimeout=5 \
  -o StrictHostKeyChecking=yes unitree@192.168.123.18 \
  'source /opt/ros/foxy/setup.bash && \
   export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
   export CYCLONEDDS_URI="<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.123.18</NetworkInterfaceAddress></General></Domain></CycloneDDS>" && \
   timeout --signal=INT --kill-after=4s 36s ros2 run realsense2_camera realsense2_camera_node --ros-args \
     -r __ns:=/camera/camera -r __node:=camera \
     -p device_type:=d435i -p enable_depth:=true -p enable_color:=false \
     -p enable_infra1:=true -p enable_infra2:=true \
     -p depth_module.profile:="480,270,60" -p depth_module.emitter_enabled:=1 \
     -p enable_gyro:=true -p enable_accel:=true -p gyro_fps:=200 -p accel_fps:=100 \
     -p enable_sync:=true -p unite_imu_method:=2 -p diagnostics_period:=1.0' \
  2>&1 | tee "$RUN_DIR/camera.log"
CAMERA_RC=${PIPESTATUS[0]}
printf 'camera_ssh=%s\n' "$CAMERA_RC" | tee "$RUN_DIR/camera.exit"
```

命令沿用已测profile：480×270 Depth＋双IR 60Hz、gyro200/accel100并合并，RGB关闭、投射器开启、同步开启。`timeout`到时通常返回124；同时要检查camera.log里的SIGINT/传感器关闭记录，不把124单独解释为清理失败。若需提前停止相机，在带TTY的SSH终端按Ctrl+C并检查传感器关闭日志；结束观察器则在PC观察终端按Ctrl+C。观察器退出不能停止相机或机器人。

输出：`observer/per_frame.csv`逐处理帧header、接收/处理单调时刻、两项原图零值比例、队列等待、预处理/推理/接收到完成时长、未知比例及11束距离；`observer/summary.json`记录接收/输出频率、覆盖数、最大间隔、两项零值比例分布、末帧时龄、CameraInfo和时钟限制。无跨机采集时龄。终端摘要每5秒一次。

现场只判断实际频率、射线输出节奏、未知比例、推理耗时和覆盖/排队。遇profile不符、持续无效输出、明显停收或主助手要求停止时结束采集并保留日志。这个只读节点没有机器人停止能力。视场约89°与当前微调数据102°不一致；输出及占位策略均未验证为物理射线精度，不用于控制。
