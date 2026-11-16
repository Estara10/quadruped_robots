# 电脑相机接收环境核对

日期：2026-11-11。执行器受限容器无法读取网络或导入ROS后，主助手使用宿主机执行环境完成只读核对。没有访问机器人/相机，没有调用rclpy.init，没有启动ROS/DDS节点或修改系统参数。

## 已确认

| 项目 | 宿主机实际结果 |
|---|---|
| 有线接口 | enp7s0，UP，192.168.123.99/24 |
| 无线接口 | wlp0s20f3，192.168.1.115/24；本轮继续使用有线 |
| net.core.rmem_max | 212992 |
| net.core.rmem_default | 212992 |
| Python | /usr/bin/python3，Humble对应Python3.10环境 |
| rclpy | /opt/ros/humble/local/lib/python3.10/dist-packages/rclpy/__init__.py，导入成功 |
| 消息 | sensor_msgs Image、Imu、CameraInfo导入成功 |
| NumPy | 系统版1.21.5，导入成功 |
| CycloneDDS RMW | /opt/ros/humble/lib/librmw_cyclonedds_cpp.so存在；没有初始化验证 |

宿主机pgrep核对未发现相机节点、vins_node、ros2 bag、go2_native_motion或ROS daemon匹配进程。本轮没有启动需要清理的后台进程。

## 执行器继续方式

前次失败属于容器/进程环境限制，不是电脑缺少ROS。不要安装另一套ROS或使用Anaconda3.13导入Humble。

主助手实际成功的调用方式如下。执行器应在允许访问宿主机的执行环境运行，按AGENTS使用rtk前缀；权限请求由工具的宿主机执行机制处理，不通过修改容器网络或系统包规避。

```bash
rtk proxy env -u PYTHONHOME -u PYTHONPATH -u LD_LIBRARY_PATH \
  PYTHONNOUSERSITE=1 PATH=/usr/bin:/bin:/usr/sbin:/sbin \
  /bin/bash --noprofile --norc -c '
    source /opt/ros/humble/setup.bash
    /usr/bin/python3 -c "import sys,rclpy,numpy; from sensor_msgs.msg import Image,Imu,CameraInfo; print(sys.executable,rclpy.__file__,numpy.__version__)"
  '
```

继续原任务：复用已有观察器准备电脑30秒接收、合成消息离线检查和现场单pipeline自动重叠命令。不要重复质疑已核对项目，不因容器默认Python缺包宣称宿主ROS不可用。网络状态应在宿主只读环境核对；离线自检不初始化DDS。

## 现场前尚需处理

电脑当前接收缓存上限也只有208KiB。Orin缓存修复已通过，电脑侧参数本轮尚未修改。现场运行前拟将电脑rmem_max提高到8MiB，沿用仓库 `scripts/go2_vins/config/60-go2-camera-dds.conf`，不修改rmem_default或降低现有更高值；具体应用由主助手现场安排，不是执行器本轮离线任务。

本次只核对导入和库文件，电脑跨机图像接收仍待现场验证。未提交Git。
