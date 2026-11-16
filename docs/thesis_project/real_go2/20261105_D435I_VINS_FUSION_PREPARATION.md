# D435i 双目加相机 IMU 的 VINS 接入准备

日期：2026-11-05。离线完成隔离构建与兼容修复。本轮没有启动相机、VINS节点、机器人 DDS 或控制器。

## 结论

选用 [`zinuok/VINS-Fusion-ROS2`](https://github.com/zinuok/VINS-Fusion-ROS2/tree/72023bc5d2abb14faa4c02a5c040d84f996b587a)，固定在提交 `72023bc5d2abb14faa4c02a5c040d84f996b587a`。它包含 D435i 双目 IMU 配置，ROS 2 节点可读取双目图像和 `sensor_msgs/Imu`。这是ABS原作者使用ZED SDK跟踪之外的同类视觉惯性定位替代路线，不是原作者完全相同的算法。仅准备构建 `camera_models` 和 `vins` 估计器，不构建 `loop_fusion`、`global_fusion`，也不安装RTAB-Map。上游许可证为GPLv3；本仓库只保存运行配置和窄适配补丁，没有复制整仓源代码。

该移植README写明Ubuntu 20.04、ROS 2 Foxy、OpenCV 3.4.1、Ceres 2.1.0和Eigen 3.3.9，没有承诺Humble。本机用户已安装APT版Ceres开发包；本轮使用系统Ceres 2.0.0、OpenCV 4.5.4、Eigen 3.4.0和Humble完成 `camera_models`、`vins` 隔离构建。构建过程发现并修复了本机 `/usr/local` 与APT Eigen头文件混用，以及上游使用Ceres 2.1 Manifold API的问题。构建成功证明本机生成了可执行文件和安装树，不证明相机输入、时间同步、初始化或跟踪已通过。

原ABS实现可见 [`publisher_depthimg_linvel.py`](../../../ABS/deployment/src/abs_src/publisher_depthimg_linvel.py)：依赖ZED SDK的位置跟踪，并从位置差分得到速度。这里的VINS替代路线仍须单独验证速度表现。

## 代码调用链和数据语义

在所选提交中，`vins/src/rosNodeTest.cpp` 从配置读取三个订阅话题。IMU回调读取header时间、`linear_acceleration.{x,y,z}`和`angular_velocity.{x,y,z}`后传给`Estimator::inputIMU()`；不消费IMU orientation字段。`sensor_msgs/Imu`消息本身不是姿态解算结果。ROS消息语义下加速度为m/s²、角速度为rad/s。代码不对两组三轴数据做轴变换，因此要求合并后的IMU消息坐标轴与VINS配置中的IMU body轴一致。

双目回调把左右图像分别放入队列，再按header时间戳配对，容差为3 ms；旧帧会被丢弃，不使用接收顺序直接拼帧。相机驱动的`enable_sync`与VINS自己的3 ms队列配对是两层机制，均不证明硬件级完全同步。

VINS内部状态位置和速度是配置中IMU body参考点在初始化VINS世界系下的量。上游`vins/src/utility/visualization.cpp`将世界系`Vs`直接写入`nav_msgs/Odometry.twist`，同时标注body `child_frame_id`，两者坐标不一致。随附补丁只改消息发布：`pose`仍是VINS世界系下相机IMU参考点；两类`twist`均用姿态转到`child_frame_id`所指的相机IMU body系。Estimator内部`Ps/Vs/Rs`和优化状态未改变，也没有变成Go2机身速度。

- `/vins/odometry`：每个图像优化完成后发布；仅在估计器进入`NON_LINEAR`状态后发布，包含优化后的状态。补丁后`twist`为IMU body系。
- `/vins/imu_propagate`：初始化后每条IMU消息触发快速传播输出，低延迟但在下一次图像优化前会积累传播误差。补丁后`twist`也为IMU body系。
- 代码没有单独的初始化状态或失跟踪状态话题。首次出现`odometry`和日志`Initialization finish!`可识别初始化成功；`image_track`用于观察特征跟踪。断流时没有独立lost状态通知。

配置把VINS body命名为`camera_imu_optical_frame`。它不是Go2机身坐标系。转成Go2机身速度还需要实测相机到Go2基座的安装变换，并在有角运动时处理相机IMU原点到Go2基座原点的杠杆臂速度差。该变换本轮不猜测，也不接入Recovery、Agile/RA或导航。

## 本机参数与候选假设

配置位于 [`scripts/go2_vins/config/d435i_640x480/`](../../../scripts/go2_vins/config/d435i_640x480/)。主配置使用已验证的640×480红外流和实际记录的话题命名：

- 左右：`/camera/camera/infra1/image_rect_raw`、`/camera/camera/infra2/image_rect_raw`。
- IMU：`/camera/camera/imu`。需要相机启动时设置`unite_imu_method:=2`才会生成该合并话题。
- 两个相机文件采用所选640×480流的本机内参`fx=fy=390.5668335`、`cx=320.82287598`、`cy=236.22209167`，畸变系数为0。已保存的ROS `CameraInfo`为`D=0, R=I`，其K与设备枚举中同分辨率红外内参一致；因此这组配置只用于当前`image_rect_raw`话题和640×480模式，不混用彩色相机内参或其他分辨率参数。
- 主配置不覆写上游`FOCAL_LENGTH=460`。代码审查发现该常量是视差/误差使用的虚拟像素尺度：`feature_manager.cpp`用它显示平均视差，`parameters.cpp`又以同一尺度将`keyframe_parallax`转为归一化阈值；`feature_tracker.cpp`把归一化点放到以该尺度构造的虚拟像平面上做`F_threshold`的RANSAC筛选与绘图；`estimator.cpp`用它缩放重投影残差权重和离群判断。上述用途内部保持一致，因此不能只因本机实际`fx=390.5668`就把460替换为物理焦距。相机模型文件仍用本机真实内参。
- 设备枚举给出的红外到Gyro旋转为单位阵；平移分别为`(-0.0055200, 0.0051000, 0.0117400)m`和`(0.04450496, 0.0051000, 0.0117400)m`。VINS的`body_T_cam`按`p_body = R_body_cam * p_cam + t_body_cam`解释。左右平移差为0.050024959m；与右相机`CameraInfo.P[3]`推得的0.050024959m基线一致。其基础来自本机`rs-enumerate-devices`记录；VINS接入时的轴语义和精度仍需现场核对。
- 相机ROS记录显示陀螺仪约200 Hz、加速度计约100 Hz。独立话题`unite_imu_method=0`此前已收到，但合并话题method 2本机尚未运行。method 2把加速度插值到陀螺仪时间戳并以陀螺仪速率发布；它不生成姿态，也不保证与图像硬件同步。官方参数说明见[RealSense ROS驱动文档](https://github.com/realsenseai/realsense-ros/blob/ros2-master/README.md)。
- 本机枚举报告“IMU Calibration is not available”，偏置和噪声方差显示为0，驱动使用默认内外参。这些0不能当成真实零噪声。配置中的`acc_n=0.1, gyr_n=0.01, acc_w=0.001, gyr_w=0.0001, g_norm=9.805`来自上游VINS D435i示例，仅是启动候选值，不是这台相机的实测噪声或标定结果。`estimate_extrinsic=1`允许在运动充分时围绕枚举初值优化相机到IMU外参；`estimate_td=1, td=0`允许VINS在有运动激励时估计初始未知时间偏差。
- GPU关闭；所选提交的`feature_tracker.h`中`GPU_MODE`宏本来就是注释状态，不需要为CPU构建改源代码。README的GPU操作说明与当前提交源码不完全同步。

本配置不融合Go2关节、足力或机身IMU。IMU插值、同步、噪声参数及外参假设均待实测。

## 时间戳和图像间隔证据

`logs/real_go2/20261105_144151_d435i_stream_check/`的独立时间戳记录中，左右图像约15 Hz；对298条左图，294条在VINS的3 ms范围内找到右图，另有1条右图没有对应左图。最大最近header时间差66.716 ms。该次相机日志为`Sync Mode: Off`、IMU分离模式，不能当作method 2或`enable_sync=true`时的行为。

`logs/real_go2/20261105_145010_d435i_scene_check/gap_summary.json`显示红外header最大间隔约340.7 ms；去掉开头2秒后，接收间隔仍出现约70.6 ms和132.4 ms的大间隔。因此大间隔不只发生在启动瞬间。该次平均速率约14.6 Hz，单看平均值会掩盖间隔。首个VINS候选仅开左右红外与IMU、关闭深度和彩色，以减少带宽和处理负担；这是否能改善帧间隔待实测。

当前投射器保持设备默认状态，因为本机已有图像样本只有该设置，且投射散斑是否妨碍VINS特征跟踪尚未测量。RTAB-Map的[D435i示例launch](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/realsense_d435i_stereo.launch.py)把投射器关闭作为其配置选择，本轮只借用话题与同步接线思路，不直接照搬该设置。VINS本身不消费深度；若以后同时启用深度，投射器选择还要重新评估深度质量。

没有左右图像与IMU的完整同步录包，不能进行VIO离线回放，也不以时间戳CSV伪造回放。

## 小型CPU/Humble适配补丁

补丁文件：[`scripts/go2_vins/patches/0001-vins-humble-cpu-and-odometry.patch`](../../../scripts/go2_vins/patches/0001-vins-humble-cpu-and-odometry.patch)。它只改独立VINS估计器所需内容：

- 在`clearState()`之前初始化预积分和边缘化指针，避免首次清理读取未初始化指针。
- 固定使用CPU `DENSE_SCHUR`，移除CPU构建时仍会解析的`ceres::CUDA`枚举引用。
- 将所需估计器的Ceres 2.1 `Manifold`接口适配到本机Ceres 2.0 `LocalParameterization`接口：保留原有`Plus`和Jacobian计算，将`PlusJacobian`映射为`ComputeJacobian`，按Ceres 2.0接口注册；SFM用四元数参数化对象改为Ceres 2.0对应类。没有修改状态、残差、协方差或求解器配置。
- 将本次构建包内的Eigen头文件改为`<Eigen/...>`，避免上游`<eigen3/Eigen/...>`先命中本机`/usr/local/include/eigen3`中的Eigen 3.3.9、再与APT Eigen 3.4.0/OpenCV混用。
- 将Humble下Marker寿命设为`Duration::from_seconds(0.0)`。
- 将两类Odometry的世界系速度旋为child/body系，并为`imu_propagate`设置child frame。
- 2026-11-06增加有序停止：SIGINT后停止并join图像同步线程，停止并joinEstimator处理线程，再于局部Node销毁前释放全局Publisher。具体根因和验证边界见[停止修复记录](20261106_VINS_SHUTDOWN_FIX.md)。

Humble补丁来源是[RTAB-Map示例引用的VINS补丁](https://gist.github.com/matlabbe/ebbb343cd744da9d6d6d6ded2e1557fd)；本项目仅取独立CPU估计器所需改动，不应用其中loop_fusion、共享库导出等无关部分。所选仓库源码已有GPU宏注释，因此没有改该文件。适配涉及`camera_models`的四元数局部参数化、`vins/src/factor/pose_local_parameterization.*`、`estimator.cpp`与`initial_sfm.cpp`；新旧API仅调整接口名与注册方式，保留原加法更新和Jacobian计算。

最终补丁从固定提交`72023bc5d2abb14faa4c02a5c040d84f996b587a`在干净临时worktree中重新应用，`git apply --check`与`git diff --check`通过。此前记录中的非零航向速度转换检查仍适用于此次未改变的旋转公式。本轮OpenCV `FileStorage`离线解析了主配置、左右相机文件和两个4×4外参矩阵，检查话题名、图像尺寸、相对标定文件路径与矩阵尺寸。完整`vins_node`已构建；未启动节点或测试真实话题。

## 隔离构建和依赖

本机依赖核对：

- Ubuntu 22.04.5、amd64、内核6.8、ROS 2 Humble。
- `libceres-dev` / `libceres2`：`2.0.0+dfsg1-5`；`libgoogle-glog-dev` / runtime：`0.5.0+really0.4.0-2`；`libgflags-dev`：`2.2.2-2`；`libsuitesparse-dev`：`1:5.10.1+dfsg-4build1`。
- Eigen `3.4.0-2ubuntu2`、OpenCV `4.5.4+dfsg-9ubuntu4`、Boost开发组件 `1.74.0.3ubuntu7`。另发现未由APT管理的`/usr/local/include/eigen3`版本3.3.9；构建补丁通过规范Eigen include路径选择APT版，未改系统安装。
- RealSense ROS 4.58.4 / librealsense 2.58.4（来自本机既有软件/现场记录）；此次构建不链接或启动RealSense节点。

第一次构建在Ceres已找到后暴露了Eigen 3.3.9/3.4.0头文件混用；修正include路径后又发现移植代码使用Ceres 2.1的`Manifold`和`SetManifold`，而本机Ceres 2.0提供`LocalParameterization`。适配后`camera_models`和`vins`均构建并安装成功。构建日志仅剩上游弃用、变长数组及其他既有编译警告；没有为这些非阻断警告扩大修改。

构建使用干净环境避免Anaconda/ROS1污染；`colcon`一次仅构建一个包，底层并行限制为2。构建及安装产物保留在`/tmp/go2_vins_build`，源码为固定提交上应用仓库补丁的`/tmp/go2_vins_fusion_ros2`：

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

本命令只选择`camera_models`与`vins`依赖链；不会构建`loop_fusion`或`global_fusion`，也不安装完整RTAB-Map。SDK配置和链接检查已通过：`ros2 pkg prefix vins`解析为`/tmp/go2_vins_build/install/vins`，可执行入口为`vins vins_node`；`ldd`解析了181个动态依赖条目，没有`not found`，包括系统`libceres.so.2`。2026-11-06在固定提交`72023bc5d2abb14faa4c02a5c040d84f996b587a`上更新同一补丁并增量重建`vins`成功；安装入口仍为`/tmp/go2_vins_build/install/vins/lib/vins/vins_node`，候选配置和启动命令未变。

**2026-11-08 恢复构建：**`/tmp`下源码和安装树均已丢失；已在同一固定提交上重新clone、应用同一补丁并完整构建`camera_models`/`vins`成功（2 packages finished，exit 0）。入口仍为`/tmp/go2_vins_build/install/vins/lib/vins/vins_node`；`ros2 pkg prefix vins`解析正确；`ldd`共183项，无`not found`。本轮未安装或升级系统包。构建与IMU告警审阅、下一次现场命令见[2026-11-08记录](20261108_VINS_BUILD_AND_IMU_REVIEW.md)。

## 后续相机现场命令

> 当前现场以 480×270、CycloneDDS 和 VINS 记录为准，命令见[2026-11-08记录](20261108_VINS_BUILD_AND_IMU_REVIEW.md)与[CYCLONE_FIELD_CHECK.md](../../../scripts/go2_vins/CYCLONE_FIELD_CHECK.md)。本节保留首次 640×480 相机-only 检查的历史命令，不直接用于下一次 480×270 VINS 观察。

以下是后续相机-only首次跟踪检查的准确命令，不启动Go2控制。本轮未执行。两个命令分别放在终端A、终端B；二者都使用ROS域42，避免和项目Go2域0混用。

**终端A：左右红外与IMU，保持相机默认投射器设置。** 本机此前验证过此红外profile与深度/RGB/IMU同时开启，但尚未验证`enable_sync=true`和合并IMU组合：

```bash
env -i HOME=/home/lidio PATH=/usr/bin:/bin LANG=C.UTF-8 TERM=xterm \
  bash --noprofile --norc -c '
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=camera device_type:=d435i \
  enable_depth:=false enable_color:=false \
  enable_infra1:=true enable_infra2:=true \
  depth_module.infra_profile:=640,480,15 \
  enable_gyro:=true enable_accel:=true gyro_fps:=200 accel_fps:=100 \
  enable_sync:=true unite_imu_method:=2
'
```

**终端B：启动VINS。**

普通运行默认关闭同步诊断。后续定位双目接收/配对问题时，可在终端B命令的环境中显式设置`VINS_SYNC_DIAGNOSTICS=1`，记录图像回调接收时刻、header时间、队列长度、配对/丢弃原因和派发时刻。具体字段和离线对照见[配对缺口诊断](20261106_VINS_STEREO_PAIRING_GAP_DIAGNOSIS.md)。

```bash
env -i HOME=/home/lidio PATH=/usr/bin:/bin LANG=C.UTF-8 TERM=xterm \
  bash --noprofile --norc -c '
source /opt/ros/humble/setup.bash
source /tmp/go2_vins_build/install/setup.bash
export ROS_DOMAIN_ID=42
mkdir -p /home/lidio/quadruped_robots/logs/real_go2/vins_first_bringup
ros2 run vins vins_node \
  /home/lidio/quadruped_robots/scripts/go2_vins/config/d435i_640x480/realsense_stereo_imu.yaml \
  --ros-args -r __ns:=/vins
'
```

核对输入话题、类型和实际频率：

```bash
ros2 topic list -t | grep -E '/camera/camera/(infra1|infra2|imu|gyro|accel)/'
ros2 topic info -v /camera/camera/imu
ros2 topic echo /camera/camera/imu --once
ros2 topic echo /camera/camera/infra1/camera_info --once
ros2 topic echo /camera/camera/infra2/camera_info --once
ros2 topic hz /camera/camera/infra1/image_rect_raw
ros2 topic hz /camera/camera/infra2/image_rect_raw
ros2 topic hz /camera/camera/imu
```

确认合并IMU消息中的`header.frame_id`、`linear_acceleration`、`angular_velocity`有限，角速度/加速度符号和静止重力方向合理；VINS不依赖IMU orientation字段。再核对VINS输出：

```bash
ros2 node list
ros2 topic list -t | grep -E '^/vins/(odometry|imu_propagate|image_track|path)'
ros2 topic hz /vins/image_track
ros2 topic hz /vins/odometry
ros2 topic echo /vins/odometry --once
ros2 topic echo /vins/imu_propagate --once
```

首次判断：日志出现`Initialization finish!`且`/vins/odometry`开始发布，表示内部初始化完成；`/vins/image_track`应持续发布并可视检查跟踪点。若无优化里程计、图像时间持续出现大间隔或跟踪明显丢失，先Ctrl+C停止VINS和相机进程，保存输出，不尝试接入Recovery。无初始化状态话题，也没有单次静止运行就证明动态速度准确的判据。检查速度时使用补丁后的body系`twist`；不要再把它旋转一次。输出参考点仍是相机IMU原点。

正常退出时先在终端B按Ctrl+C，等待VINS报告退出码0且日志没有`terminate called`或FastDDS清理错误；再在终端A按Ctrl+C，等待相机退出。随后在ROS域42终端执行`ros2 node list`，确认`/camera/camera`与VINS节点已消失，并用`ps -eo pid,ppid,args | grep -E '[r]ealsense2_camera_node|[v]ins_node'`核对本次进程。若使用过ROS CLI daemon，再执行`ros2 daemon stop`。无输入等待阶段的退出已离线验证；初始化后或Estimator正在处理传感器数据时的SIGINT仍待现场检查。观察器与VINS均不具备停止Go2机器人的能力。

## 适用边界和下一步

本准备记录对应的软件构建轮次，没有运行RealSense节点、打开viewer或操作Go2。2026-11-06另做了输入-only rosbag回放，未连接相机。相机ROS 4.58.4和librealsense 2.58.4的版本及现场数据流来自已有记录；当前证据不证明图像/IMU硬件同步、IMU标定、时间偏差估计收敛、动态速度准确、相机到Go2基座外参或现场实时接收链路可靠。

首次相机-only静止跟踪观察已完成；此后初始化后SIGINT退出、可靠订阅候选现场运行及一次视频参考平移也已记录。当前下一项只推荐：先做固定安装后的USB链路只读核对，确认D435i实际协商速率、经过的hub/port和可用profile，判断USB2能否满足当前双IR+IMU配置及后续深度需求；核对前不更换线缆或扩展坞、不安装驱动。核对后再安排电脑只读、原生遥控的受控低速运动观察，不命令机器人按项目控制链运动，不据此启用Recovery。结果与限制见[视频参考平移记录](20261106_VINS_VIDEO_REFERENCE_TRANSLATION.md)、[机载只读检查](20261106_ONBOARD_D435I_READONLY_CHECK.md)及[配对缺口诊断](20261106_VINS_STEREO_PAIRING_GAP_DIAGNOSIS.md)。

## 主要来源

- [VINS-Fusion ROS 2移植及D435i配置](https://github.com/zinuok/VINS-Fusion-ROS2/tree/72023bc5d2abb14faa4c02a5c040d84f996b587a)
- [VINS-Fusion原始项目](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)
- [RealSense ROS驱动文档与unite_imu_method定义](https://github.com/realsenseai/realsense-ros/blob/ros2-master/README.md)
- [RTAB-Map D435i双目接线示例，仅作参考](https://github.com/introlab/rtabmap_ros/blob/ros2/rtabmap_examples/launch/realsense_d435i_stereo.launch.py)
- [librealsense跨流外参API示例](https://github.com/realsenseai/librealsense/blob/master/examples/sensor-control/api_how_to.h)
