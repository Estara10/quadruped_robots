# 2026-11-09 第二、三组分段审阅与下次三组观察准备

范围：只读离线审阅第 2/3 轮（共享一次 VINS 运行的已关闭 bag 与已抽取 CSV）的静止前、运动、停止附近与停后分段。未连接 Go2/相机，未初始化实机 DDS，未启动 VINS/回放/记录器，未发送任何机器人命令，未修改原始记录或生产配置，未提交 Git。第 1 轮原始数据与已有结论保留，本轮不分析、不回放、不安排其往返复现。

## 1. 针对的问题

判断 D435i＋VINS 的定位偏差主要出现在运动期间位移估计、停止后估计校正，还是测量窗口/参考点/人工口径。第 1 轮（快速前冲＋后退＋再前进、含严重估计异常）暂不展开；本轮只对第 2/3 组做分段，为之后重新三组测量补足关键证据。

用户口径（沿用，不追改）：第 2 组人工净距离约 1.16 m，第 3 组约 1.15 m；计时不准、不作速度真值；横向位移未独立测量；两轮共享一次 VINS 初始化。聊天通知时间只是近似边界，不是准确物理起停。

## 2. 数据与口径

原始记录：`logs/real_go2/20261108_091600_vins_repeat_walk_02_03_retry/`（未修改）。分析复用 `20261108_repeat_walk_summary/review/round23_{odometry,propagation,imu}.csv`，不重新读 bag、不重跑 VINS。

窗口（`operator_events.jsonl`，聊天时间仅作近似边界）：

| 轮次 | walk_requested E | 停稳报告 S | 返回起点标记 | 备注 |
|---|---|---|---|---|
| R2 | 1794129468.215 | 1794129505.531 | 1794129529.529 | 返回起点单独标记，不并入测量 |
| R3 | 1794129675.274 | 1794129744.653 | — | 停后记录到 1794129767.968 |

- pre 窗口：E−8 到 E−2；post 沿用 S+2 到 S+10；停后分段另按 S→S+5、S+5→S+10、S+10→S+15。
- 位置中位数口径；世界系 X/Y 是相机 IMU 参考点，未转换为 Go2 机身参考点。
- 优化 odom 与传播 imu_propagate 来自同一估计器，不当作两个独立定位结果；相邻差分跳变不是真实速度。
- `n_pts` 是每帧新补充点数，不是总跟踪数；本轮不据此判断特征充足。

## 3. 分段结果

图：[round23_segmented.png](../../../logs/real_go2/20261109_round23_segmented/round23_segmented.png)

| 指标 | R2（odom/prop） | R3（odom/prop） |
|---|---:|---:|
| 人工距离 | 1.16 m | 1.15 m |
| pre→post 水平净位移 | 1.204 / 1.204 m | 1.207 / 1.207 m |
| 与人工距离差 | +0.044 / +0.044 m | +0.057 / +0.057 m |
| pre→post Z 变化 | +0.007 / +0.006 m | −0.014 / −0.014 m |
| 测量窗最大相邻步长 | 0.094 / 0.088 m | 0.080 / 0.083 m |
| 停后 0–5 s 漂移（相对 S 位） | 0.0003 m | 0.0005 m |
| 停后 5–10 s 漂移 | 0.0051 m | 0.0174 m |
| 停后 10–15 s 漂移 | 0.0064 m | 0.0180 m |
| 停后最大瞬时变化 | 约 1.5 cm 内 | 峰值约 5–7 cm、净约 1.8 cm |

### 3.1 运动区间（odom 大相邻步定位）

- R2：运动起始约 E+17.4 s，结束约 E+19.7 s，约 2.3 s，步长约 0.07–0.09 m（约 0.6 m/s 量级）。
- R3：运动起始约 E+49.4 s，主要约 E+49.9–51.2 s，约 1.3 s，步长约 0.07–0.08 m。
- 两轮物理运动都远早于聊天停稳报告（各约 18 s 后）；停稳报告前后位置已稳定，说明“停后”窗口确实在物理停止之后。

### 3.2 停止附近与停后分段

- R2：S±2 s 位置中位数 (0.081, 1.175) m；停后 0–5/5–10/10–15 s 相对该位漂移 0.3/5.1/6.4 mm，无重锚定瞬态；Z 变化约 +1.6 mm。停后稳定。
- R3：S±2 s 位置 (0.027, 1.215) m；停后 0–5 s 内先稳定，随后约 S+2.7–5.1 s 出现一次瞬态，水平峰值到约 (0.082, 1.265) m（瞬时约 5–7 cm），随后回落到约 (0.040, 1.227) m 并稳定；净校正约 1.8 cm，Z 变化约 +5 mm。
- R3 该瞬态有传播跟随：imu_propagate 在 S+2.857 s 出现一次 8.3 cm 相邻跳变，之后回到优化状态附近，符合优化校正重锚传播（与第 1 轮大跳变同机制，但量级为厘米级）。

### 3.3 运动与停后对净位移的贡献（odom）

- R2：到停稳报告的位移约 1.209 m，停后漂移约 +0.006 m；净位移几乎全部来自运动，停后变化可忽略。
- R3：到停稳报告的位移约 1.196 m，停后校正约 +0.018 m；停后校正贡献约 1.8 cm，是两组中唯一的明显停后项。

### 3.4 窗口敏感性

pre 起点取 E−10/−8/−6 s、post 起点 S+2/S+3 s、post 止点 S+8/S+10 s 组合下：R2 水平净位移 1.2039–1.2057 m（odom）；R3 1.2070–1.2074 m（odom）。两轮对窗口选择均稳定（毫米级），净位移主要不是选窗造成。

## 4. 结论

1. **是否存在第 1 组量级的异常？** 没有。两轮测量窗最大相邻步长 ≤0.094 m（R2）/0.083 m（R3，含一次 8.3 cm 传播重锚），Z 变化 ≤2 cm；无第 1 轮的约 1 m 状态跳变、0.9 m Z 变化或千米级发散。第 1 轮量级异常在 2/3 轮未重现。
2. **与卷尺距离的差异有多少与停后输出变化有关？** 很少。停后输出变化为 R2 约 0.6 cm、R3 约 1.8 cm（峰值 5–7 cm 的瞬态），远小于两轮 4.4/5.7 cm 的 VINS−人工差。差异主体不是停后校正，而是运动期间位移估计本身、人工距离近似、横向未测、参考点不一致等共同作用，现有数据不能进一步分解。
3. **现有数据能否支持稳定尺度补偿？** 不能。仅两次走动、共享一次初始化、不是独立重复；两轮同向偏大（+4.4/+5.7 cm）但样本太少、人工距离近似、横向未测、无独立真值；不拟合补偿系数，不调外参/td/噪声/配对阈值，不以逼近卷尺为目标调参。
4. **下一次三组测量最需要补足哪几项？** 每组独立起停与 ≥10 s 运动前、≥15 s 停后；用机器人上同一固定参考点记录前向与横向分量并说明与相机 IMU 参考点的区别；明确三组是否独立初始化；开启逐次外参/td 诊断（`VINS_DIAG_EXTRINSIC=1`）记录外参与 td 历史；就位站稳后再定起点、测量结束前保持站立不返回。详见第 6 节。

## 5. 证据局限

- 聊天标记不是物理起停；人工距离近似、横向未测；计时不可作速度真值。
- 优化与传播同源，不构成独立定位；相邻差分跳变不是真实速度；IMU 安静不能排除匀速平移。
- 无独立高度/姿态/位移真值；R3 停后瞬态是实际微动还是纯估计校正，现有数据不能区分（但传播 8.3 cm 跟随符合优化重锚）。
- 世界系 X/Y 是相机 IMU 参考点，未转换为 Go2 机身参考点。
- 两轮共享一次 VINS 初始化，不是独立重复；未做独立初始化下的重复。
- 本轮只读、未修改任何算法/配置/原始数据，未提交 Git。

## 6. 下一次三组现场观察（准备，本轮不执行）

目标：得到三组有独立起停、独立/明确初始化、含逐次外参/td 的位姿序列，判断偏差是否随运动过程稳定、停后校正是否可重复、是否存在可用尺度倾向。保持原生遥控、电脑只读，不接 goto/Recovery/策略。

### 6.1 通用要求

- 沿用已验证采集链与 480×270 配置（`scripts/go2_vins/config/d435i_480x270/`），不切算法参数、不改噪声/配对阈值/分辨率/频率/td/尺度。
- 每组：就位站稳后再定起点；运动前记录 ≥10 s；停稳后继续记录 ≥15 s；测量结束前保持站立，不趴伏、不返回起点。
- 用机器人上同一固定参考点测量起终点，记录前向与横向两个分量；说明它与相机 IMU 参考点的区别（相机 IMU 是 VINS body，尚未转换到 Go2 基座）。
- 卷尺/地面标记仅作评价，不输入定位；不使用动捕/定位标签；不追求精确同一距离，记录每组实际净位移即可。
- 明确三组是否独立初始化：优先相机保持连续运行（共享初始化），若重启 VINS 必须等 `Initialization finish!` 完成再安排运动；未完成不运动。在记录中标注每组的初始化归属。
- 逐次外参/td：诊断二进制已支持默认关闭的 `VINS_DIAG_EXTRINSIC=1`（输出 `VINS_DIAG_EXT <t> <td> <tic> <ric> <p> <v>`）；直接设该环境变量即可，不为此扩大开发。若该二进制因 `/tmp` 重启丢失，需先按固定提交 `72023bc5d2abb14faa4c02a5c040d84f996b587a`＋仓库补丁重建并重新加入诊断输出（见 `20261108_VINS_BUILD_AND_IMU_REVIEW.md` 与 `20261109_EXTRINSIC_AB_REPLAY.md`）。
- 方向/姿态异常、线缆风险、输入持续中断、初始化失败或明显位置跳变时，现场人员优先停止机器人再结束采集；采集器不承担停止机器人作用。
- 停止后检查本次进程退出，不能只凭 Ctrl+C 宣称清理完成。

### 6.2 时间对齐（现场执行）

Orin 重启后可能回到 1970；VINS 同机配对不依赖绝对钟，但跨机对比需对齐：

```bash
# 电脑
date -u +%s
# Orin（<PC_EPOCH> 换成上一条输出）
sudo date -u -s @<PC_EPOCH>
date -u
```

记录两台 `date -u` 结果；之后可能漂移，每轮重新对齐。

### 6.3 相机（Orin，终端 A）

```bash
ssh unitree@192.168.123.18
# 若出现 ros:foxy(1) noetic(2) ? 输入 1
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

保存完整相机 stdout（IMU 标定、HID、Motion Module 通知及是否重复）。设备缺失/权限/右路错误/CycloneDDS 报错则停止并保留输出。

### 6.4 电脑：记录器（终端 B，先于 VINS 启动，避免漏初始化）

```bash
RUN=/home/lidio/quadruped_robots/logs/real_go2/$(date +%Y%m%d_%H%M%S)_vins_3group
mkdir -p "$RUN"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.123.99</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
ros2 topic list --no-daemon
ros2 bag record -o "$RUN/bag" \
  --qos-profile-overrides-path /home/lidio/quadruped_robots/scripts/go2_vins/field_record_qos_best_effort.yaml \
  /camera/camera/imu /vins/odometry /vins/imu_propagate
```

记录 IMU 数值与 VINS 优化/传播输出；`/camera/camera/imu` best_effort、VINS 输出 reliable。若实际话题名不同，先按 `ros2 topic list` 修正。

### 6.5 电脑：VINS（终端 C）

```bash
export RUN
python3 - <<'PY'
import os, shutil
from pathlib import Path
src = Path('/home/lidio/quadruped_robots/scripts/go2_vins/config/d435i_480x270')
run = Path(os.environ['RUN'])
dst = run / 'config'; dst.mkdir(parents=True, exist_ok=False)
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

source /opt/ros/humble/setup.bash
source /tmp/go2_vins_build/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>192.168.123.99</NetworkInterfaceAddress></General></Domain></CycloneDDS>'
export VINS_IMAGE_QOS=best_effort
export VINS_SYNC_DIAGNOSTICS=1
export VINS_DIAG_EXTRINSIC=1
export ROS_LOG_DIR="$RUN/ros_logs"
mkdir -p "$ROS_LOG_DIR"
cd "$RUN/config"
/tmp/go2_vins_build/install/vins/lib/vins/vins_node \
  "$RUN/config/realsense_stereo_imu.yaml" \
  --ros-args -r __ns:=/vins 2>&1 | tee "$RUN/vins.log"
```

### 6.6 每轮操作员记录

每组在 `RUN/operator_events.jsonl` 追加聊天事件（注明“聊天时间非物理边界”），并单独记录：实际起停秒表/视频、卷尺前向与横向分量、机器人参考点位置、初始化归属（独立 or 共享）、任何异常（方向/姿态/线缆/断流/跳变）。

### 6.7 停止与清理

1. VINS 终端 Ctrl+C，等待退出码 0，检查日志无 `terminate called`/清理错误；
2. 记录器 Ctrl+C，等待 bag 正常关闭；
3. 相机终端 Ctrl+C，等待 SSH 提示符；
4. Orin：`pgrep -af '[r]ealsense2_camera_node'`；电脑：`pgrep -af '[v]ins_node|[r]os2 bag record'`；
5. 有残留则确认来源后仅处理本次任务进程，不关闭 Unitree 原生服务。

## 7. 文件与复现

新增（`logs/real_go2/20261109_round23_segmented/`，只读已关闭 CSV）：

- `result.json`：分段与敏感性数值。
- `round23_segmented.png`：位置与停后变化对照图。
- `plot_round23_segmented.py`：绘图脚本。

复现：

```bash
cd /home/lidio/quadruped_robots/logs/real_go2
python3 20261109_round23_segmented/plot_round23_segmented.py
```

数据与脚本复用 `20261108_repeat_walk_summary/`（`round23_*.csv`、`analyze_round23.py`）。本轮未读正在录制的 bag，未启动 VINS/ROS/设备，未提交 Git。
