# D435i 两次柜面约 2 m 记录核对与下一次约 1 m 测量

本次只离线读取两组已有记录，未访问相机、Orin 或 Go2，没有启动 ROS/DDS、训练或控制。

## 两组记录是否存在可解释差值的代码错误

没有发现读取、单位、像素索引、标记帧或补洞混用错误。两组文件都是同一序列号 `238322070866` 的 `480×270` 深度帧，元数据比例尺为 `0.001 m/unit`。原始文件按小端 `<u2` 读入后，`(x,y)=(240,135)` 的值与 `frames.csv` 的 `center_pixel_depth_m` 及 marker 的 Z 相符；保存序号 0/120/240 对应的帧号也相符。两组 marker 中深度与 IR 的设备时间戳相同。中心深度在原始帧与补洞帧中相同，测距统计使用的是原始深度。

采集源码 [`capture.cpp`](../../../logs/real_go2/20261113_remount_adaptation_capture/capture.cpp) 用 SDK 做深度像素反投影、深度到 IR1 的外参变换，再投影到 IR1；不是把两个图的中心默认视为同一点。两组保存 marker 的投影坐标都是 `(240,135)`。这个点离主点 `(245.186,134.22)` 很近。按各自内参算，Z 深度到光线直线距离仅多约 `0.048 cm`；不能解释两组约 1 cm 的中位数差。

| 统计量 | A：用户确认镜头附近到红十字约 1.96 m | B：用户报告镜头到柜面约 2 m |
|---|---:|---:|
| 全段单像素 `(240,135)` Z 中位数 | 2.088 m | 2.099 m |
| 保存的 0/120/240 帧单像素 Z | 2.099 / 2.099 / 2.110 m | 2.099 / 2.099 / 2.099 m |
| 全段单像素 P5–P95 | 2.055–2.122 m | 2.077–2.122 m |
| 同一统计范围 | 中央 40×40 的帧内有效深度中位数，其时间中位数也是 2.088 m | 中央 40×40 的帧内有效深度中位数，其时间中位数为 2.110 m |
| 自动曝光首帧记录 | 2296 μs | 4016 μs |

若只比较单像素中位数，B 比 A 大 `1.1 cm`，小于两组的帧间波动范围。两组中央 40×40 区域统计不同，说明不能把“中心距离”不加区分地用于区域中位数和单像素。两次曝光状态也不同。现有资料没有证明两次相机姿态、柜面上被红十字命中的具体物理点和卷尺端点完全相同。

因此，这个差值可由帧间波动、曝光/场景条件或测点/姿态细微差异解释；现有数据不能进一步分开这些因素。A 的 1.96 m 是用户确认的近似距离，B 只是“约 2 m”且未记录尺量起止点。它们都不是精密标定值，不能据此说相机尺度有误，也不能拟合修正系数。

完整既有核对见 [196 cm 单组审计](20261113_D435I_196CM_DISTANCE_AUDIT.md)。本次复算没有改原始 CSV、深度或 IR 文件。

## 下一次约 1 m 最小测量

### 固定对象与参考点

1. 让相机保持当前安装；选一个平整、哑光的柜面或墙面，避开接缝、把手、反光区和边缘。机器人保持静止，不启动运动控制。
2. 在相机外壳前部选一个易辨认的位置贴小胶带，标作卷尺起点 `H`，拍下近端照片。`H` 是外壳基准，不称为光学中心。
3. 让目标平面覆盖图像中心。采集后按下方 marker 生成 IR 红十字图，红十字来自 SDK 对深度测点 `(240,135)` 的实际投影。确认十字落在平整目标面内部；若落在边缘、缝隙或该深度像素为 0，废弃该次结果并重新摆放目标/重采，绝不取邻近像素代替。
4. 在十字对应的物理点做一个细小可移除标记，卷尺从 `H` 量到该点，记录实际读数和估计误差，并拍近端、远端照片。目标、相机和机器人在采集与测量之间不得移动。记录机器人/相机姿态；优先沿用 A 的静态趴伏摆位，A 是唯一有已确认近似卷尺参考的记录。
5. 如果先用一次短采集定位红十字，则该次只作对点预览；标好点、测完后再做下面的最终采集。最终分析只用最终采集的原始深度。一次运行本身约 6 秒，保存 0/120/240 三组深度和同步 IR，不连续存图。

### 最终采集命令

在电脑仓库根目录打开一个终端。先确认有线链路可达、Orin 没有其他相机程序占用。该命令在 Orin 编译并运行现有独立 SDK 采集程序：只开一个深度+IR1 pipeline，`480×270@60 Hz`，不启动 ROS/DDS、VINS 或机器人控制。输出目录用本地时间戳命名；远端墙钟不用于对时。

```bash
set -u
STAMP="$(date +%Y%m%d_%H%M%S)"
LOCAL="logs/real_go2/20261113_d435i_1m_distance_capture/${STAMP}"
REMOTE="/tmp/go2_d435i_1m_${STAMP}"
mkdir -p "$LOCAL"

ssh -o BatchMode=yes -o ConnectTimeout=8 unitree@192.168.123.18 "mkdir -p '$REMOTE'"
scp logs/real_go2/20261113_remount_adaptation_capture/capture.cpp \
  unitree@192.168.123.18:"$REMOTE/capture.cpp"

ssh -o BatchMode=yes -o ConnectTimeout=8 unitree@192.168.123.18 \
  "g++ -std=c++17 -I/opt/ros/foxy/include -L/opt/ros/foxy/lib/aarch64-linux-gnu -Wl,-rpath,/opt/ros/foxy/lib/aarch64-linux-gnu '$REMOTE/capture.cpp' -o '$REMOTE/d435i_capture' -lrealsense2 && mkdir -p '$REMOTE/data' && '$REMOTE/d435i_capture' 480 270 60 '$REMOTE/data'"
CAPTURE_EXIT=$?
printf '%s\n' "$CAPTURE_EXIT" > "$LOCAL/capture_exit.txt"
if [ "$CAPTURE_EXIT" -ne 0 ]; then echo "采集失败，退出码=$CAPTURE_EXIT；保留远端目录供检查"; exit "$CAPTURE_EXIT"; fi

scp -r unitree@192.168.123.18:"$REMOTE/data" "$LOCAL/"
ssh -o BatchMode=yes -o ConnectTimeout=8 unitree@192.168.123.18 \
  'source /opt/ros/foxy/setup.bash && rs-enumerate-devices --no-eth --option --calib_data' \
  > "$LOCAL/rs-enumerate-devices.txt" 2>&1 || echo "校准枚举命令失败；不要填造缺失的IR/外参记录"
ssh -o BatchMode=yes -o ConnectTimeout=8 unitree@192.168.123.18 \
  "if pgrep -x d435i_capture >/dev/null; then echo '采集进程仍在运行'; exit 1; fi; rm -rf '$REMOTE'"
```

该程序启动前会打印实际序列号和固件；继续前确认序列号仍为 `238322070866`，配置实际为 `480×270@60`。采集成功应报告约 360 帧，并恢复其记录的相机选项。保留 `capture_exit.txt`、`metadata.txt`、`frames.csv`、marker 和原始文件。不要把空洞填补文件用于距离统计。

### 从保存样本生成实际测点图

在电脑仓库根目录执行以下命令，生成第 120 帧的 IR 标记图。代码读取 marker 中 SDK 投影出的 IR 坐标，不硬编码“两个图中心相同”；也会检查对应原始深度是否有效。打开图片后确认红十字在目标面内部，再按现场步骤量距。

```bash
CAPTURE_DIR="$(find logs/real_go2/20261113_d435i_1m_distance_capture -mindepth 2 -maxdepth 2 -type d -name data | sort | tail -1)"
IMAGE_OUT="$(dirname "$CAPTURE_DIR")/measurement_point_ir.png"
CAPTURE_DIR="$CAPTURE_DIR" IMAGE_OUT="$IMAGE_OUT" /usr/bin/python3 - <<'PY'
import os
from pathlib import Path
import numpy as np
from PIL import Image, ImageDraw

d = Path(os.environ['CAPTURE_DIR'])
i = 120
meta = dict(line.split('=', 1) for line in (d / 'metadata.txt').read_text().splitlines() if '=' in line)
w, h, scale = int(meta['width']), int(meta['height']), float(meta['scale'])
marker = [float(v) for v in (d / f'{i}_marker.txt').read_text().split()]
x, y = round(marker[0]), round(marker[1])
raw = np.fromfile(d / f'{i}_depth.u16', dtype='<u2').reshape(h, w)
if not (0 <= x < w and 0 <= y < h) or raw[y, x] == 0:
    raise SystemExit('测点越界或原始深度无效：不要用邻近像素，调整目标后重采')
ir = np.fromfile(d / f'{i}_ir1.u8', dtype='u1').reshape(h, w)
im = Image.fromarray(ir).convert('RGB')
draw = ImageDraw.Draw(im)
draw.line((x-8, y, x+8, y), fill=(255, 0, 0), width=1)
draw.line((x, y-8, x, y+8), fill=(255, 0, 0), width=1)
im.save(os.environ['IMAGE_OUT'])
print(f'IR测点=({marker[0]:.3f},{marker[1]:.3f}) px; 原始Z={raw[y,x]*scale:.4f} m; 图像={os.environ["IMAGE_OUT"]}')
PY
xdg-open "$IMAGE_OUT"
```

如果红十字没有落在平整目标面内部，或中心原始深度为 0，就不接受这一组，不使用补洞值或邻近值。完成测量后，保持现场几何不变；记录卷尺读数、误差估计和两端照片。最终采集与卷尺测量前后顺序可以互换，关键是两者期间不移动相机、机器人或目标。

### 怎么与旧的约 2 m 结果比较

统一比较原始深度在 SDK 投影测点处的 Z 值，并分别保留卷尺的 `H` 外壳基准距离。中心 Z 到光线直线距离换算在 2.1 m 仅约 `0.05 cm`，在 1 m 也远小于厘米级卷尺参考误差。由于 `H` 到真实光学中心的结构偏移未知、A/B 起点和柜面点未完全统一，1 m 与旧 2 m 只能作趋势线索：固定绝对差偏向参考点偏移，差值随距离成比例增大偏向尺度项；但两条旧参考的精度不等，不能据此拟合或应用校准。若最终需要定量分开二者，必须有同一 `H`、同一测量方法的可复现距离对；本次不生成校正系数。

## 未补齐的记录项

两次旧采集的 `metadata.txt` 都没有持久化 IR 内参和深度→IR 外参矩阵；marker 保存了 SDK 算出的投影坐标和同一时刻戳，且当前代码路径已确认正确，但无法仅凭元数据独立重放投影。下一次应保留 SDK 枚举的校准输出（如现场 `rs-enumerate-devices -c` 可用则保存其完整输出），并保留 marker。若枚举输出不包含本次活动 profile 的外参，就把这一项标成未知；不要假造或用相机中心替代。
