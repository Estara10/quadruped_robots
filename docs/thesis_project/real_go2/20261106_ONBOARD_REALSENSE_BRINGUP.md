# 机载 D435i RealSense 驱动接入与跨机传输排查

日期：2026-11-06。范围：Go2 机载 Orin 上的 D435i 驱动安装、相机-only 流测试、与电脑 VINS 的跨机链路排查。未启动 Go2 控制链，未发布 LowCmd，未改变原生遥控或 Unitree 服务。

## 结论

1. 机载 Orin 已安装 Foxy/arm64 的 RealSense 驱动，设备可枚举并输出双 IR + IMU。
2. **480×270@15 双 IR + IMU 在 Orin 本机稳定**：左右各 15.15 Hz、IMU 200.06 Hz，最大间隔约 0.069/0.007 s。
3. **640×480 双 IR 在当前 USB2 路径不可用**：右路报 `Right MIPI error`，只有单路可工作；640×480@6 仍失败。该限制与总带宽关系不明确，不能据此直接归因 USB2 带宽。
4. **跨机到电脑的速率极低**：图像约 0.2–0.5 Hz、IMU 约 1–2.5 Hz，间隔数秒。Orin 时钟已对齐电脑、电脑 `ufw` 为 `ENABLED=no`，两者均未改善。当前证据指向 Foxy FastDDS ↔ Humble FastDDS 的跨机传输，但根因尚未证明。
5. 480×270 VINS 配置已建立但未初始化：日志为 `feature tracking not enough`，特征数 1–6，左右图像到达间隔数秒。该结果与跨机传输受限一致，尚不能作为 VINS 算法本身的结论。
6. 电脑已安装 CycloneDDS（7 个包，0 升级），尚未进行两端统一切换后的复测。

## 机载环境

| 项目 | 实测 |
|---|---|
| 系统 | Ubuntu 20.04.6 LTS / kernel 5.10.104-tegra / arm64 / 8 核 / 15 GiB / 根分区 206 GiB 可用 |
| ROS | Foxy（另有 Noetic）；`cv_bridge`、`image_transport` 已装 |
| RMW | `rmw_fastrtps_cpp`、`rmw_cyclonedds_cpp` 均已装 |
| 网络 | `eth0` 192.168.123.18/24；电脑 `enp7s0` 192.168.123.99/24 |
| 时钟 | 2026-11-06 已用 `sudo date -u -s` 对齐电脑 UTC；无 NTP，可能漂移 |

## 驱动安装

Orin 无外网（DNS、8.8.8.8、网关均不可达），`apt-get install` 在下载阶段失败。改为电脑下载、校验后传输：

- 索引中的版本：`ros-foxy-librealsense2 2.51.1`、`ros-foxy-realsense2-camera-msgs 4.51.1`、`ros-foxy-realsense2-camera 4.51.1`。
- 清华镜像的 pool 路径返回 404；改从 Aliyun ROS 2 镜像下载同一版本。
- 三个 `.deb` 的 SHA256 与清华 `Packages.gz` 索引逐项一致；传输到 Orin 后再次核对一致。
- `sudo dpkg -i /tmp/realsense_debs/*.deb` 成功，`DPKG_EXIT=0`；3 个包均为 `ii`，升级 0、卸载 0。
- 首次 apt 失败未安装任何包；未改内核、未刷固件、未升级系统。

## udev 规则

ROS 包未带 udev 规则；IMU 的 IIO sysfs 和 HID sensor 的 `enable_sensor` 对 `unitree` 不可写。安装了一条仅针对 `8086:0b3a` 的最小规则：

- Orin 路径：`/etc/udev/rules.d/99-realsense-d435i-local.rules`
- 仓库副本：[`scripts/go2_vins/udev/99-realsense-d435i-local.rules`](../../../scripts/go2_vins/udev/99-realsense-d435i-local.rules)
- 覆盖 USB 设备、`iio*`、`hid_sensor*` 和 `HID-SENSOR-*` 平台设备；只作用于该 VID/PID，不改其他设备。
- 安装后 `udevadm control --reload-rules` 并触发；IMU 在本机测试中达到 200 Hz。

## 设备与可用模式

- D435I，序列号 `238222071369`，固件 `05.13.00.55`（驱动建议 `05.13.00.50`，本轮不升级固件）。
- USB 2.1；`rs-enumerate-devices` 可无 sudo 运行。
- 双 IR/深度常见模式包括 1280×720@6、848×480@10、640×480@30/15/6、480×270@60/30/15。
- Motion Module：Accel 200/100 Hz、Gyro 400/200 Hz。
- 480×270 IR 内参：`fx=fy=244.104263305664`、`cx=240.514297485352`、`cy=132.638809204102`，畸变系数 0，FOV 约 89.03°×57.89°。

## 本机流测试

Foxy 的 `rs_launch.py` 在传非空 `config_file` 时有 Python 表达式错误，本轮改用 `ros2 run` 直接启动节点：

```bash
ros2 run realsense2_camera realsense2_camera_node --ros-args \
  -r __ns:=/camera/camera -r __node:=camera \
  -p device_type:=d435i -p enable_depth:=false -p enable_color:=false \
  -p enable_infra1:=true -p enable_infra2:=true \
  -p depth_module.profile:=480,270,15 -p depth_module.emitter_enabled:=0 \
  -p enable_gyro:=true -p enable_accel:=true -p gyro_fps:=200 -p accel_fps:=100 \
  -p enable_sync:=true -p unite_imu_method:=2
```

本机用 rclpy best-effort 订阅 12 秒：

| 话题 | 速率 | 最大到达间隔 | 最大 header 间隔 |
|---|---:|---:|---:|
| `infra1/image_rect_raw` | 15.15 Hz | 0.0687 s | 0.0671 s |
| `infra2/image_rect_raw` | 15.15 Hz | 0.0699 s | 0.0671 s |
| `imu` | 200.06 Hz | 0.0071 s | 0.0052 s |

640×480 双 IR 测试中右路报 `Right MIPI error`；infra1-only、infra2-only 分别可收到数据。640×480@6 双 IR 仍只有左路。该现象与 USB2 供电/模式限制的关系未确认。

## 跨机传输与 VINS

电脑用同一节点、domain 42、FastDDS、`ROS_LOCALHOST_ONLY=0` 订阅，12 秒统计：

| 话题 | 电脑速率 | 最大到达间隔 |
|---|---:|---:|
| infra1 | 0.22–0.49 Hz | 6.1–9.2 s |
| infra2 | 0.20–0.33 Hz | 6.1–9.2 s |
| imu | 1.27–2.46 Hz | 3.1–6.1 s |

已排除：
- Orin 时钟 1970 → 对齐电脑后速率无改善；
- 电脑 `ufw` 服务 active，但 `/etc/ufw/ufw.conf` 为 `ENABLED=no`，未启用防火墙；
- 相机本机速率正常，说明瓶颈不在相机或 USB2 采集。

当前判断：跨机 Foxy FastDDS 2.x ↔ Humble FastDDS 2.x 的传输是主要嫌疑；String 小消息测试可通，图像/高频 IMU 不可用。根因未证明。

VINS 配置：
- 新建 [`scripts/go2_vins/config/d435i_480x270/`](../../../scripts/go2_vins/config/d435i_480x270/)（`left.yaml`、`right.yaml`、`realsense_stereo_imu.yaml`），不改原 640 配置；`show_track: 0`。
- 电脑 VINS 使用 `VINS_IMAGE_QOS=best_effort`、`VINS_SYNC_DIAGNOSTICS=1`，在 40 秒内未初始化；日志出现 `feature tracking not enough`，`n_pts` 为 1–6，左右图像到达间隔数秒。
- VINS 结论暂不成立：先修跨机传输，再判断 480×270 特征数和动态精度。

## 系统变更与边界

Orin 上的持久变更：
- 安装 3 个 ROS Foxy RealSense 包；
- 新增 `/etc/udev/rules.d/99-realsense-d435i-local.rules`；
- 系统时钟已对齐到电脑 UTC（无 NTP）。

未做：内核补丁、固件升级、系统升级、Unitree 服务修改、Go2 控制链启动、LowCmd 发布、Recovery 接入。

结束状态：Orin 与电脑均无本次相机/VINS 残留进程；相机未推流；机器人原生控制未受影响。

## 下一步

1. 两端统一 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`（电脑已装 `ros-humble-rmw-cyclonedds-cpp 1.3.5`），domain 42、非 localhost，重测图像/IMU 跨机速率。
2. 若恢复约 15/200 Hz，运行 480×270 VINS，检查初始化、特征数和静态/低速运动表现。
3. 若仍慢，再比较机载 VINS（Orin 本机速率已满足）或换用非 DDS 图像传输；不先改 VINS 算法或标定。
4. 640×480 双 IR 在当前 USB2 路径仍不可用；是否改用 USB3/供电方案，等跨机链路结论后再决定。

## 证据位置

- 安装与本地日志：`logs/real_go2/20261106_onboard_realsense_stage1/`（本机保留，不随 Git 上传）。
- 480×270 VINS 运行日志：`logs/real_go2/vins_480x270_bringup/vins.log`。
- 相机配置：[`scripts/go2_vins/config/d435i_480x270/`](../../../scripts/go2_vins/config/d435i_480x270/)。
- udev 规则：[`scripts/go2_vins/udev/99-realsense-d435i-local.rules`](../../../scripts/go2_vins/udev/99-realsense-d435i-local.rules)。
- `/tmp` 下的隔离 VINS 构建与临时脚本不随 Git 保存，重启后需重建。
