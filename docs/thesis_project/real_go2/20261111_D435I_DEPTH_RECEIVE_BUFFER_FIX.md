# D435i深度停收：UDP接收缓存修复

日期：2026-11-11。用户明确优先修复深度图持续接收，手工44cm对照不作固定距离补偿。本轮连接Orin，进行了有限的静态相机前后对照和独立确认；未发送Go2运动、StopMove、LowCmd或模式切换，未运行VINS、策略或仿真。

## 结论与实际修改

本轮定位并修复了Orin本机的深度图停收：系统UDP接收缓存上限 `net.core.rmem_max=212992`（208KiB），相机运行期间出现接收缓存溢出。仅把该上限提高到 `8388608`（8MiB），同一个观察器即持续收到约15Hz深度；独立重启相机与观察器后再次通过，两个修正后窗口的UDP `RcvbufErrors` 增量均为0。

**没有改相机profile、图像订阅QoS、相机驱动或模型。**没有安装新软件、改固件、调整射线/安全阈值；原来的BEST_EFFORT图像订阅可以继续使用。本轮修复是接收缓存上限，不是宣布原发布/订阅QoS不兼容。

已在Orin安装并应用 `/etc/sysctl.d/60-go2-camera-dds.conf`，内容与仓库[配置文件](../../../scripts/go2_vins/config/60-go2-camera-dds.conf)一致。当前 `rmem_max=8388608`、`rmem_default=212992`，默认值、发送缓存和其他网络设置保持不变。8MiB是允许应用请求的上限，不是证明每个套接字实际分配了8MiB。未重启机器人验证开机加载；不为此追加重启测试。

## 直接证据

原始记录保存在：`logs/real_go2/20261111_d435i_depth_buffer_fix_zJwzaZON/go2_depth_buffer_zJwzaZON/`，汇总为其父目录 `summary.json`。

| 条件 | 观察时长 | 深度图 / CameraInfo | 深度接收表现 | 结束时最后深度时龄 | 窗口内UDP接收缓存溢出增量 |
|---|---:|---:|---|---:|---:|
| 原上限212992 | 20秒 | 3 / 291 | 仅启动后3帧，随后停收 | 19.218秒 | 1633 |
| 上限8388608 | 20秒 | 291 / 291 | 持续15.016Hz，最大接收间隔71.91ms | 42.82ms | 0 |
| 同一修正，独立启动确认 | 40秒 | 591 / 591 | 持续15.016Hz，最大接收间隔73.14ms | 24.64ms | 0 |

修正后两次双IR仍约15Hz、IMU约200Hz。深度逐帧CSV行数与回调计数一致、header严格递增，最大header间隔约78.4/78.2ms；没有用重复帧或刷新旧图时间戳填补数据。基线3帧内部频率约14.9Hz不代表持续频率，判断依据同时包括19.2秒末尾停收。

UDP计数来自机载 `/proc/net/snmp` 的前后差，是主机全局计数，不是逐数据包相机归属追踪。但相机/观察器参数不变、只提高接收上限后，深度交付恢复且溢出归零，并在独立启动中重复，已足以采用本项修复。没有为根因证明再回退缓存制造故障。

## 运行条件与复用

- D435i序列号238222071369，固件05.13.00.55，实际USB2/480M。
- Orin Ubuntu20.04/aarch64/Foxy，realsense-ros4.51.1、librealsense2.51.1；CycloneDDS0.7.0、rmw_cyclonedds_cpp0.7.11。
- ROS domain42，绑定192.168.123.18。
- 唯一pipeline：Depth Z16及IR1/IR2 Y8均480×270@15；gyro200、accel100、sync=true、unite_imu_method=2、emitter=0、color=false、diagnostics_period=1.0。
- 基线和第一次修正复用44cm观察器20秒；独立确认只将观察时长改为40秒，采集/回调/订阅QoS不变。44cm字段来自复用记录器，不用于本轮标定或精度判断。
- 每段均先启动观察器，再启动相机；相机分别限定25/25/45秒。没有并开Viewer或第二个pipeline。无额外Image接收器掩盖单观察器停收。

原源码未改，相机停收涉及的图像尺寸259200字节也未减小。修复与官方CycloneDDS关于大样本接收缓存的建议一致：[ROS2 RMW性能建议](https://github.com/ros2/rmw_cyclonedds#performance-recommendations)、[CycloneDDS FAQ](https://cyclonedds.io/content/faq#why-are-there-huge-latency-outliers-when-sending-huge-samples)。官方建议提高接收缓存；本轮实测只提高上限已足够，因此没有同时改变默认缓存或QoS。

## 配置安装与回退

本机Orin已经安装，不需重复执行。移植到其他接收主机时先查当前值与文件，不能无条件覆盖别人的网络配置。以下为本轮所用内容的复现方式：

电脑仓库根目录：

```bash
scp scripts/go2_vins/config/60-go2-camera-dds.conf \
  unitree@192.168.123.18:/tmp/60-go2-camera-dds.conf
```

Orin：

```bash
sysctl net.core.rmem_max net.core.rmem_default
sudo install -m 644 /tmp/60-go2-camera-dds.conf /etc/sysctl.d/60-go2-camera-dds.conf
sudo sysctl -p /etc/sysctl.d/60-go2-camera-dds.conf
```

本轮安装前已确认目标文件不存在，安装后读回文件及两个实际参数并保存。若要回退本次新增设置，先停止使用中的相机/订阅进程，再删除本次新增文件并恢复本机原值：

```bash
sudo rm /etc/sysctl.d/60-go2-camera-dds.conf
sudo sysctl -w net.core.rmem_max=212992
```

上述原值仅适用于本机本次记录，不作为其他电脑的回退默认值。配置复制品保存在本轮原始记录中，不含登录凭据。

## 范围与下一项

本项通过的是 **Orin本机、480×270@15联合流的持续接收修复**。没有验证30Hz、电脑跨机深度接收或相机与VINS/射线模型并行负载；本轮不把这些列为本项未完成。深度空洞、模型视场差异、机身安装外参及低层停止属于后续各自任务，不由网络缓存修复解决。

下一项只建议：沿用已修正的Orin配置，在电脑做一次深度/双IR/IMU只读接收核对，确认后续ABS在电脑运行时的数据入口。不重复测44cm、调scale、换USB硬件或扩展A1导航。

## 收尾

三段观察器均退出0；相机按预定timeout发SIGINT退出124，日志保留传感器关闭结果。最终机载进程核对无本轮相机、Viewer、VINS、bag、观察器或ROS daemon；SSH/SCP和临时sudo会话均已退出。原始数据已复制回电脑。保留机载/tmp原始记录，持久化网络修复按上文记录；机器人原生服务未改。

未修改模型、控制器、仿真、A1工程或唯一ACTIVE任务；未提交或推送Git。本轮只新增一项配置并更新相机实机记录入口，没有新增通用检查框架。
