# D435i 软件环境准备记录

日期：2026-11-05。仅做本机软件与APT包检查、包内容离线检查和命令准备；没有枚举或启动相机，没有连接Go2或初始化机器人DDS。

## 本机检查

- Ubuntu 22.04.5 LTS，amd64，内核 `6.8.0-138-generic`；当前ROS环境为 Humble / ROS 2，安装前缀 `/opt/ros/humble`。
- 已安装包、常用系统目录和 `~/Libraries` 中未发现 librealsense、RealSense ROS驱动、viewer或枚举器；系统Python 3找不到 `pyrealsense2`。本任务不需要Python绑定，后续以ROS驱动为主。
- APT已有ROS 2源，无RealSense上游Debian源。候选版本为 `ros-humble-librealsense2 2.58.4-1jammy.20260830.174155`、`ros-humble-realsense2-camera 4.58.4-1jammy.20260915.211150`、`ros-humble-realsense2-camera-msgs 4.58.4-1jammy.20260907.215954`。不需要额外安装 `realsense2_description` 才能启动相机节点。
- 对最小安装执行APT模拟：4个新包（SDK、camera、camera-msgs、diagnostic-updater），0个升级、0个移除。没有改系统包。
- 下载到 `/tmp` 的官方ROS SDK候选包清单中，确实包含 `/opt/ros/humble/bin/realsense-viewer`、`rs-enumerate-devices`及版本为2.58.4的SDK库；camera候选包包含 `realsense2_camera_node` 和 `rs_launch.py`。因此不需要叠加RealSense上游SDK包或源码构建。该直接包清单检查优先于上游README中“ROS源SDK无图形工具”的通用描述。
- SDK候选包没有携带 `99-realsense-libusb.rules`，本机常见udev规则目录也未找到RealSense规则。普通用户访问USB/HID设备的权限仍待安装规则并在现场验证。
- 当前 `CMAKE_PREFIX_PATH` 混有 `/home/lidio/catkin_ws/devel` 和MuJoCo/Franka路径。相机检查应在单独终端清掉旧ROS/自定义库环境后只source Humble，避免ROS1路径遮蔽ROS 2依赖。Unitree SDK2与RealSense SDK无关，本次未将二者混用。

## 用户手动安装

当前APT模拟的最小命令如下；不会安装DKMS、添加RealSense上游SDK源、全面升级系统或移除已有包：

```bash
sudo apt-get install --no-install-recommends \
  ros-humble-librealsense2 ros-humble-realsense2-camera
```

RealSense官方ROS文档说明Ubuntu 22.04支持Humble，并提供ROS仓库SDK与ROS驱动安装路线：[realsense-ros安装说明](https://github.com/realsenseai/realsense-ros/blob/ros2-master/README.md)。本机选择这一条路线，避免同时安装ROS仓库和Intel/RealSense上游的两套SDK。虽然官方ROS安装说明中的通用包描述未承诺viewer，本机已检查当前APT候选包，实际包含viewer和枚举器。

本机内核为6.8。RealSense发行包文档列出的DKMS HWE支持版本不包括6.8；另一份官方Linux说明指出多数已有型号由主线UVC驱动支持，内核补丁主要面向尚未被系统驱动识别的新型号。因而这里先选用户态ROS仓库包，不装DKMS、不换内核、不打补丁；D435i真实视频/IMU数据仍需后续现场验证。[官方发行包说明](https://github.com/realsenseai/librealsense/blob/master/doc/distribution_linux.md)、[Linux安装说明](https://github.com/realsenseai/librealsense/blob/master/doc/installation.md)。

若现场以普通用户运行时遇到USB权限错误，且确认没有已安装的RealSense udev规则，可手动安装与SDK版本对应的官方规则（来自[librealsense v2.58.4配置](https://github.com/realsenseai/librealsense/blob/v2.58.4/config/99-realsense-libusb.rules)）：

```bash
curl -fsSL \
  https://raw.githubusercontent.com/realsenseai/librealsense/v2.58.4/config/99-realsense-libusb.rules \
  -o /tmp/99-realsense-libusb.rules
sudo install -D -m 0644 /tmp/99-realsense-libusb.rules \
  /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

安装规则后需要重新插拔相机，再按普通用户运行枚举器。上游规则对匹配USB接口使用`MODE=0666`，允许本机所有用户访问；若现场权限策略不允许这种较宽权限，应先由主助手确认最小权限规则，不要用sudo启动整个ROS图像节点来掩盖权限问题。

## 现场软件检查命令

在一个新的终端中使用干净ROS 2环境；此设置也把相机ROS图与Go2已用的domain 0隔离：

```bash
unset ROS_DISTRO ROS_VERSION ROS_PACKAGE_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
```

安装后先核对版本、路径和动态库，不访问设备：

```bash
dpkg-query -W -f='${binary:Package}\t${Version}\n' \
  ros-humble-librealsense2 ros-humble-realsense2-camera \
  ros-humble-realsense2-camera-msgs ros-humble-diagnostic-updater
command -v rs-enumerate-devices
command -v realsense-viewer
command -v ros2
ros2 pkg prefix realsense2_camera
ldd "$(command -v rs-enumerate-devices)" | grep 'not found' || true
ldd "$(command -v realsense-viewer)" | grep 'not found' || true
```

离线检查已确认SDK包内枚举器 `--help`、`--version`可运行，版本报告 `2.58.4.0`；`realsense-viewer`与枚举器的SDK/系统共享库依赖均能解析。没有打开viewer窗口。

### 1. 查看设备、固件、USB模式和流配置

只枚举USB设备并列出控件、标定和可用模式；`--no-eth`禁止RealSense SDK探测以太网/DDS设备：

```bash
rs-enumerate-devices --no-eth --option --calib_data
```

该命令应显示型号、序列号、固件、USB类型和各传感器支持的流模式。单个流模式列表不保证所有模式能同时组合，具体组合仍以ROS节点实际打开结果为准。[枚举器参数说明](https://github.com/realsenseai/librealsense/blob/master/tools/enumerate-devices/readme.md)。

### 2. 打开官方查看器

```bash
realsense-viewer
```

此命令会访问相机并可能启动传感器；本轮未运行。后续检查时关闭viewer窗口，并确认 `pgrep -af realsense-viewer` 中本次进程已退出。

### 3. 启动初始ROS数据流（profile待枚举确认）

尚未枚举这台相机的profile，因此不填入未经确认的分辨率或帧率。APT候选的`rs_launch.py`对图像profile使用`0,0,0`默认值；首轮由SDK选择设备默认模式，显式开启左右红外、深度、RGB和两路IMU，并保持投射器、曝光及滤波器默认值：

```bash
ros2 launch realsense2_camera rs_launch.py \
  device_type:=d435i \
  enable_depth:=true enable_infra1:=true enable_infra2:=true \
  enable_color:=true enable_gyro:=true enable_accel:=true \
  unite_imu_method:=0
```

低负荷候选为深度/双红外`640x480x15`、RGB`640x480x15`，但**只有**枚举结果列出这些模式后才在命令中显式加入profile参数；枚举的单流模式仍不保证全部同时可用，须看节点打开日志及话题实测。若不支持就保留默认配置，不猜测模式。profile参数名为`depth_module.depth_profile`、`depth_module.infra_profile`（infra1/2共用）和`rgb_camera.color_profile`，不可用组合会有警告或回退默认值。[ROS参数说明](https://github.com/realsenseai/realsense-ros/blob/ros2-master/README.md)。

`unite_imu_method:=0`保留独立的`gyro/sample`与`accel/sample`两路原始消息；`1`把最新一条加速度样本复制到每条陀螺仪样本，`2`将加速度插值到陀螺仪时间戳。合并只改变消息组织/时间配对，不生成相机姿态或里程计。单独的gyro/accel `sensor_msgs/msg/Imu`只代表角速度和线加速度，不是姿态估计或位置/速度来源。初次观察优先保留两路时间戳，因此不发布合并的`imu`话题。

### 4. 核对话题、频率、时间戳和相机参数

默认节点名为`/camera/camera`。相应图像话题为`infra1/image_rect_raw`、`infra2/image_rect_raw`、`depth/image_rect_raw`和`color/image_raw`；IMU为`gyro/sample`、`accel/sample`。先查看完整话题及类型：

```bash
ros2 node list
ros2 topic list -t | grep -E '/camera/camera/(infra1|infra2|depth|color|gyro|accel)/'
ros2 topic info -v /camera/camera/depth/image_rect_raw
```

分别测接收频率（每条命令持续运行，按Ctrl+C结束该频率测量）：

```bash
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/infra2/image_rect_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/gyro/sample
ros2 topic hz /camera/camera/accel/sample
```

`ros2 topic hz`测的是本机接收间隔，不等同于设备header时间。查看独立消息时间戳和每路标定参数：

```bash
ros2 topic echo /camera/camera/gyro/sample --once --field header.stamp
ros2 topic echo /camera/camera/accel/sample --once --field header.stamp
ros2 topic echo /camera/camera/depth/camera_info --once
ros2 topic echo /camera/camera/color/camera_info --once
ros2 topic echo /camera/camera/infra1/camera_info --once
ros2 topic echo /camera/camera/infra2/camera_info --once
```

查看实际选中的profile和IMU频率：

```bash
ros2 param get /camera/camera depth_module.depth_profile
ros2 param get /camera/camera depth_module.infra_profile
ros2 param get /camera/camera rgb_camera.color_profile
ros2 param get /camera/camera gyro_fps
ros2 param get /camera/camera accel_fps
ros2 param get /camera/camera unite_imu_method
```

### 5. 正常停止并检查进程

ROS launch在前台运行，按Ctrl+C并等待launch报告节点关闭。关闭viewer窗口。然后在相同ROS环境中检查相机节点消失，并停止本domain的CLI daemon：

```bash
ros2 node list
ps -eo pid,ppid,args | grep -E '[r]s_launch.py|[r]ealsense2_camera_node|[c]omponent_container|[r]ealsense-viewer'
ros2 daemon stop
```

`component_container`可能属于别的ROS会话；只根据本次launch的PID/父子关系核对，不对匹配字符串执行批量kill。检查后确认本次launch、viewer和domain 42的CLI daemon均退出。

## 验证边界与下一步

本轮没有系统安装权限。尝试安装时遇到dpkg锁权限拒绝；容器启用了`no new privileges`，sudo不能提权。按用户要求，安装由用户手动执行，准确命令如上。APT包仅下载到`/tmp`做清单与二进制检查，已知软件版本/可执行路径来自候选包而非已安装系统。

完成的离线检查：系统/ROS/架构/内核及包状态；APT依赖模拟；SDK候选包工具、库和头文件路径；`rs-enumerate-devices --help`与`--version`；viewer/枚举器的`ldd`；ROS2 `topic echo`和`topic hz`帮助语法。未启动viewer、相机ROS节点、图像/IMU流或录制；未枚举相机、读取实际profile、测量帧率/时间戳或测试USB/udev权限。前述5000 Mb/s USB链路为主助手已有直接记录，不是本轮验证结果。

主助手手动安装后，下一项是执行本记录的USB枚举并核对普通用户权限与实际模式；确认可用配置后，再按默认profile命令做短时静止数据检查。软件安装/构建通过都不能代替相机流和标定数据的现场验证。
