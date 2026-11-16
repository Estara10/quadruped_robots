# D435i首次枚举与数据流启动检查

2026-11-05。用户已安装ROS Humble librealsense2 2.58.4及realsense2_camera 4.58.4。

设备枚举成功：D435I，固件5.13.0.55，USB descriptor 3.2（sysfs速率5000Mb/s），IMU BMI085。可枚举左右红外/深度/RGB 640x480@15Hz，以及gyro200Hz、accel100Hz。完整设备模式与出厂标定输出保存在device_enumeration.txt；枚举信息不等于标定精度已验证。

在独立ROS域42启动一次短时相机检查，启用上述图像组合和独立gyro/accel，unite_imu_method=0。未启动VINS、机器人DDS或控制器。

启动日志确认图像profile能打开，但IMU IIO scan_elements/in_anglvel_x_en等路径报Permission denied，本机未发现RealSense udev规则；随后驱动报告设备断开/重连以及No such device。未取得足够证据确定断连原因，不能直接认定由权限导致，也未验证图像质量、频率或IMU实时数据。SDK另提示IMU Calibration is not available，使用默认参数；这是后续VIO需处理的独立问题，不能把默认IMU参数当作已标定。

发现故障后向本次launch发送SIGINT。日志确认相机节点cleanly退出，外层命令退出码0，随后按已知进程路径检查无残留。退出码0只证明收尾正常，不代表采集成功。

下一步：按已安装SDK对应的官方规则补齐RealSense设备权限，重载规则后重新插拔，再进行一次相同的短时数据检查。不刷固件、不打内核补丁、不用sudo运行整个相机节点掩盖用户权限问题。本轮未操作机器人、未更改仿真或ACTIVE任务。
