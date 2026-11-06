# VINS 双目配对缺口离线诊断

日期：2026-11-06。仅使用已有rosbag和固定提交的VINS，在ROS域42、localhost-only环境中离线回放；未访问相机、Go2或启动Go2链路。原始记录及已有CSV未改。

## 结论

目前没有证据证明VINS的3 ms FIFO配对算法本身有实现错误。原记录显示其收到的左右流如何进入VINS队列不完整；只播放三路传感器输入的离线回放中，原有配对逻辑在对应时间段持续配对并产生优化里程计，未复现2.6 s空档。最值得怀疑的是现场图像订阅/回调交付或实时调度层，但现有证据无法在DDS队列丢样、回调延迟及现场QoS差异之间定因。

因此本轮不改配对算法、不放宽3 ms阈值，也不调标定或求解器。只为下次排查增加默认关闭的接收、配对/丢弃和派发诊断日志。回放结果不能证明现场实时链路已修复。

## 代码与原始记录

固定上游为 `zinuok/VINS-Fusion-ROS2` 提交 `72023bc5d2abb14faa4c02a5c040d84f996b587a`，源码位于 `/tmp/go2_vins_fusion_ros2`。当前 `rosNodeTest.cpp` 中左右图像订阅使用 `SensorDataQoS()`，即KeepLast深度5的传感器QoS；IMU订阅使用同类QoS并将深度设为2000。回调把消息按到达顺序放入进程内FIFO。独立同步循环约每2 ms检查一次队首，比较header时间：早于另一侧超过3 ms就丢弃较旧帧；在容差内配对。完成配对后，队列两侧都多于一帧时会成对丢掉旧缓存帧。同步线程释放队列锁后调用 `Estimator::inputImage()`。

这批源录包的原始计数为左725、右626、IMU9659、原优化odom287、传播输出9311。按图像header对齐，99/725（13.7%）左帧在3 ms内无右帧；626张右图均有3 ms内左图。现场原日志只记录 `throw img0`，没有接收时间、队列长度或配对决策，不能从日志确定“采集器收到”是否等同“VINS订阅回调收到”。

现场优化odom最大接收间隔为2.61737 s，header间隔2.60031 s。对应header区间为 `[1793932779.2958992, 1793932781.8962054]`。该区间内原采集记录按接收时窗统计到39张左图、28张右图、523条IMU和523条传播输出；原优化odom只有区间两端的2条。求解耗时峰值约19.27 ms，单凭该数值不能解释整个2.6 s空档。

## 单次原算法回放

回放只从原包送入以下三路输入，没有播放原包中的VINS输出：

```text
/camera/camera/infra1/image_rect_raw
/camera/camera/infra2/image_rect_raw
/camera/camera/imu
```

环境为 `ROS_DOMAIN_ID=42`、`ROS_LOCALHOST_ONLY=1`。回放使用ROS sensor QoS语义（KeepLast 5、best effort、volatile），配置见 `logs/real_go2/20261106_vins_pairing_offline/baseline/playback_qos.yaml`。复现时需在VINS终端启用诊断：

```bash
env -i HOME=/home/lidio PATH=/usr/bin:/bin LANG=C.UTF-8 TERM=xterm \
  bash --noprofile --norc -c '
source /opt/ros/humble/setup.bash
source /tmp/go2_vins_build/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 VINS_SYNC_DIAGNOSTICS=1
ros2 run vins vins_node \
  /home/lidio/quadruped_robots/scripts/go2_vins/config/d435i_640x480/realsense_stereo_imu.yaml \
  --ros-args -r __ns:=/vins
'
```

另一个终端以相同ROS环境运行输入-only回放：

```bash
env -i HOME=/home/lidio PATH=/usr/bin:/bin LANG=C.UTF-8 TERM=xterm \
  bash --noprofile --norc -c '
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
ros2 bag play \
  /home/lidio/quadruped_robots/logs/real_go2/20261106_103906_vins_manual_right_translation/input_and_odometry \
  --rate 1.0 --disable-keyboard-controls \
  --topics /camera/camera/infra1/image_rect_raw \
           /camera/camera/infra2/image_rect_raw \
           /camera/camera/imu \
  --qos-profile-overrides-path \
  /home/lidio/quadruped_robots/logs/real_go2/20261106_vins_pairing_offline/baseline/playback_qos.yaml
'
```

第三个终端在同一ROS域记录新输出（每次复现使用新的输出目录）：

```bash
source /opt/ros/humble/setup.bash
source /tmp/go2_vins_build/install/setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
output_dir="/tmp/vins_pairing_outputs_$(date +%Y%m%d_%H%M%S)"
ros2 bag record -o "$output_dir" /vins/odometry /vins/imu_propagate
```

回放结束后先对Recorder按Ctrl+C并等待其写出缓存，再对VINS按Ctrl+C。实际分析产物为[回放统计](../../../logs/real_go2/20261106_vins_pairing_offline/baseline/replay_analysis.json)、[VINS同步日志](../../../logs/real_go2/20261106_vins_pairing_offline/baseline/vins.log)及[新输出录包](../../../logs/real_go2/20261106_vins_pairing_offline/baseline/replayed_vins_outputs/)；原始输入包未改。

对应时段按header时间筛选后，回放VINS收到40张左图、29张右图，配成29对；11张较旧左图按既有算法丢弃、没有右图丢弃，29对均进入 `Estimator::inputImage()`。左右输入计数与原采集器的39/28相差各1，是header时间窗和原采集器接收时窗边界不同导致的筛选口径差异。全回放诊断计数为左725、右625、配对625、丢弃左100/右0、派发624。原包有626张右图，回放VINS只收到625张；差的1张（0.16%）可能来自回放/订阅投递，现有信息不能确定原因。全程配对事件和派发事件差1，原因也未定；但目标空档区间二者均为29。

| 指标 | 原记录目标空档 | 同时间戳回放目标区间 |
|---|---:|---:|
| 优化odom条数 | 2 | 15 |
| 优化odom最大接收间隔 | 2.61737 s | 0.73461 s |
| 优化odom最大header间隔 | 2.60031 s | 0.73342 s |
| 传播输出条数 | 520 | 520 |
| 传播最大接收间隔 | 5.610 ms | 5.648 ms |
| 传播最大header间隔 | 5.002 ms | 5.002 ms |

回放该区间两端优化位姿的净位移，原记录为 `(0.02791, -0.36871, 0.01944) m`、模长0.37028 m；回放为 `(0.00766, -0.24098, -0.01105) m`、模长0.24136 m，回放较小0.12892 m（34.8%）。全记录起止净位移模长，原记录1.07998 m、回放1.01193 m，差−0.06805 m（−6.3%）。两次估计器初始化及运行状态不相同，这些只是估计轨迹的差异，不是速度或位移真值，也不是优化结果优劣指标。

回放日志出现FastDDS `getifaddrs`/UDP socket `Operation not permitted`。尽管VINS和录包仍收到数据并写出结果，此沙箱网络权限可能改变DDS传输/时序，因此回放不能代表现场实时负载。源录包的 `offered_qos_profiles` 为空；本次显式设置为best-effort、volatile、深度5，与VINS图像订阅要求相容，但D435i现场发布端实际QoS尚未核实。

## 本轮修改与证据边界

在 `vins/src/rosNodeTest.cpp` 加入由 `VINS_SYNC_DIAGNOSTICS=1` 控制的最小日志，记录左右回调到达时间（steady clock）、header时间、回调后FIFO长度、配对/丢弃决策与原因、header差、配对后旧帧数，以及 `inputImage()` 返回时刻。默认关闭。算法、3 ms容差、QoS、输入、队列策略、标定和求解器均未改变。

增量构建已通过：`camera_models`、`vins` 两包成功，约30.3 s，仅有已有ROS消息指针弃用警告。可复现补丁已同步到 `scripts/go2_vins/patches/0001-vins-humble-cpu-and-odometry.patch`；从固定提交新建worktree后 `git apply --check`、应用补丁和 `git diff --check` 均通过。未做第二轮回放，因为未定位到可证明的算法缺陷。

本轮对 `ros2 bag play` 的自然结束码记录为0。停止VINS和录包器时，外层进程包装器返回−2（被SIGINT中断），不能把它当作VINS内层进程退出码0。VINS日志记录到rclcpp SIGINT处理、无abort/清理错误；录包器日志显示写出缓存并停止；输出包含9351条传播和302条优化odom。进程核对时没有残留 `vins_node`、`ros2`、bag player、recorder或构建进程。初始化后VINS内层返回码未单独捕获，故不宣称其退出码已验证。

回放支持的结论仅为：在本次回放交付给VINS的消息和QoS条件下，原算法能在现场大空档对应时间段继续配对并发布odom；严重间隔未复现。它不能区分现场DDS KeepLast队列丢帧、回调未执行、源数据的实际到达/时间戳问题或算力/调度影响，也不能证明现场已修复。

## 下一项工作

只推荐一次**相机-only VINS诊断采集**：不启动Go2链路，使用实际D435i现场发布QoS，打开 `VINS_SYNC_DIAGNOSTICS=1`，同时保存左右IR、IMU与VINS输出，并记录节点端点QoS。重点检查短时移动时两侧 `receive` 是否齐全、队列长度、drop原因、`dispatched` 时间与优化odom间隔是否再次出现约2.6 s空档。出现同类间隔即可按诊断日志定位到回调交付或配对后处理；若不复现，则保留现场故障未复现的结论，不据此修改算法。

**后续（2026-11-06）：**一次相机-only诊断采集已执行（[现场记录](../../../logs/real_go2/20261106_112612_vins_live_pairing_diagnostic/RESULT.md)）。现场仍出现VINS右图回调最大间隔1.333 s、优化odom最大接收间隔1.498 s；右图交付缺口未消除，根因仍未区分DDS交付、executor调度与回调等待。见[右图交付QoS离线诊断](20261106_VINS_RIGHT_IMAGE_DELIVERY_QOS_DIAGNOSIS.md)。
