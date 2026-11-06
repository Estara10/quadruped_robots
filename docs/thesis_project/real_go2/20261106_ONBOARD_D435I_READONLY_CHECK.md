# Go2扩展坞与D435i首次机载只读检查

日期：2026-11-06。用户将前向安装的D435i用C转C线连接扩展坞，电脑通过网线连接扩展坞，并手动验证SSH登录成功。主助手随后建立独立SSH会话，只读检查系统、USB和已有软件；未启动相机数据流、VINS、Go2 DDS或控制程序。

## 网络与登录

- 电脑enp7s0为192.168.123.99/24，carrier=1。
- 此前抓包直接出现的192.168.123.18本次ping成功，2包无丢失，时延约0.2–0.4ms；SSH服务开放。
- 用户已确认账号unitree可登录。主助手会话选择现有ROS 2 Foxy环境，只加载环境，未启动ROS节点。

## 机载系统实际记录

- Ubuntu 20.04.6 LTS、aarch64、内核5.10.104-tegra。
- /proc/device-tree/model：NVIDIA Orin NX Developer Kit。
- compatible：nvidia,p3768-0000+p3767-0000、nvidia,p3767-0000、nvidia,tegra234、nvidia,tegra23x。
- /etc/nv_tegra_release：R35 revision 3.1。
- 内存报告15Gi，根文件系统234G、可用约206G。以上来自系统自报，不用照片猜测模块型号。
- /opt/ros包含foxy与noetic；本会话ROS_DISTRO=foxy、RMW_IMPLEMENTATION=rmw_cyclonedds_cpp。

## D435i和当前USB路径

lsusb读到8086:0b3a，sysfs产品名为Intel(R) RealSense(TM) Depth Camera 435i。

```text
Bus 02 root: tegra-xusb, 10000M
Bus 01 root: tegra-xusb, 480M
  Port 3: VIA Labs 2109:2211 USB2.0 Hub, 480M
    Port 1: Intel 8086:0b3a, uvcvideo/usbhid, 480M
```

相机已被机载系统枚举，证明所接Type-C口至少可以传数据；当前实际协商为480Mb/s，不是USB 3。仅凭这条路径无法判定是线缆、连接状态、端口设计还是其他因素造成。此前电脑直连的5000Mb/s是另一条实际记录，不能当成本次机载连接的速率。

USB 2不等于相机完全不能使用；但当前尚未枚举该模式的可用双目/IMU组合，不能假定电脑USB 3下的配置原样可用，也不能仅靠接口外观认定带宽。

## 相机软件与时间

- PATH未找到rs-enumerate-devices、realsense-viewer或v4l2-ctl。
- ROS 2包索引找不到realsense2_camera、librealsense2。
- dpkg查询只发现ros-noetic-realsense2-description 2.3.2；描述包不等于可用相机驱动。
- /usr/local/lib及/usr/lib/aarch64-linux-gnu未发现librealsense相关库。检查范围内没有现成可直接运行的机载RealSense SDK/ROS 2驱动。
- 机载UTC读数为1970-01-01T00:15:36，电脑随后读数为2026-11-06T07:07:53Z。两次读取不严格同时，但已足以确认时钟未对齐；跨机时间戳和数据时龄判断前需处理。不修改时间服务或现有系统设置。

机载Ubuntu20.04/ARM64/Foxy与电脑Ubuntu22.04/amd64/Humble不同，不能直接复制电脑的二进制或APT包。当前拟保留电脑运行VINS，机载先只运行相机驱动并传输双目、IMU；不要求本轮移植VINS到机载。

## 判断与收尾

已完成：机载登录、系统识别、D435i USB枚举及当前速率核对。
尚未完成：机载相机驱动、实际流profile、跨机ROS数据传输、时间对齐、安装变换和Go2运动下观测验证。

唯一建议下一项：由执行器离线准备适配现有Ubuntu20.04/ARM64/Foxy的机载RealSense驱动与跨机接入方案，给出最小安装清单和准确操作步骤；把时钟对齐、USB协商速率/可用profile作为接入时检查，不升级机载系统、不刷固件、不修改原生控制服务。必要时再安排主助手操作现场安装及端口对照。

未安装包、修改时间或系统配置，未运行相机/策略。检查未发现机载realsense2_camera_node、realsense-viewer或vins_node进程。主助手建立的SSH会话已exit，工具返回码0；用户自己的SSH终端未操作。未改ACTIVE任务、仿真配置或代码，未提交Git。
