# Go2 原生遥控慢走速度对照观察准备

日期：2026-11-05。仅准备只读采集与现场方案；本轮未初始化DDS、连接设备或运行观察器。

## 要解决的问题

2026-11-04 的30秒趴伏观察显示现有Estimator静态输出有限且稳定，但没有动态参考，不能判断走动速度方向、幅值或响应延迟。本次准备原生遥控慢走时的并行状态记录；机器人动作完全由用户控制，电脑只订阅状态。

## 工具改动与数据语义

复用 `go2_readonly_estimator`、现有 `Estimator` 和Go2 URDF/KDL模型。现在同一进程只订阅 `rt/lowstate` 与 `rt/sportmodestate`：

- 主CSV按Estimator 200Hz更新周期记录LowState最近快照的IMU四元数/角速度/加速度、12关节q/dq/tau_est、四足原始足力、阈值1接触判断、Estimator世界系速度和转换后的机体系速度。还保留LowState接收时刻、时龄、间隔、接收序号、原始tick、重复样本标记及每次Estimator调用间隔。
- 同目录生成追加后缀 `.sport.csv` 的SportModeState记录，逐条保存本机接收时刻、接收间隔、序号、原始stamp、mode/error_code、原始position与velocity。DDS回调仅把样本入队，由主循环写文件；主CSV另记录该topic最近样本的接收年龄、序号及原始值，便于画图时查看当时持有的样本。
- 两个文件用同一进程的本机 `steady_clock` 起点计时。两路异步，主CSV一行并不表示相同采样时刻；逐消息SportModeState文件保留完整接收序列。SportModeState未收到时主CSV将该组字段标空并标记 `sport_received=0`，不补零、不回退。若没有SportModeState，仍能记录Estimator与LowState，但本次速度对照资料不完整。
- Estimator仍每5ms调用一次（固定dt=0.005s），不会按约500Hz每条LowState调用；样本重复通过接收序号与 `repeated_sample` 显示。保持当前接触阈值1、输入次序与模型观测不变。

Estimator线速度的单位/坐标语义来自现有实现及上次恢复审查：`Estimator::getVelocity()`输出世界系状态，单位按现有Recovery接口为m/s；主CSV的机体系速度是每次输出按当前估计旋转执行 `R.transpose()*v_world` 的结果。SportModeState的position/velocity坐标系、单位、机器人stamp含义仍未知；字段刻意命名为`*_raw`，只能在语义核实后作方向和幅值比较。它是原生状态对照，不是独立真值。

## 离线构建与合成检查

本机SDK2/ROS依赖前缀下的准确命令：

```bash
cmake -S /home/lidio/quadruped_robots/quadruped_ros2_control_humble/controllers/rl_quadruped_controller \
  -B /tmp/go2_estimator_build/controller \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -Dcontrol_input_msgs_DIR=/home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/control_input_msgs/share/control_input_msgs/cmake \
  -Dcontroller_common_DIR=/home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/controller_common/share/controller_common/cmake \
  -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch \
  -DCaffe2_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Caffe2 \
  -DBUILD_GO2_READONLY_ESTIMATOR_OBSERVER=ON \
  -Dunitree_sdk2_DIR=/home/lidio/Libraries/unitree_sdk2/lib/cmake/unitree_sdk2
cmake --build /tmp/go2_estimator_build/controller --target go2_readonly_estimator --parallel 2
```

现场启动命令（仅在用户安排后使用；运行前确认`enp7s0`仍是Go2链路、domain仍为0）：

```bash
env LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu:/home/lidio/Libraries/unitree_sdk2/lib \
  /tmp/go2_estimator_build/controller/go2_readonly_estimator \
  --domain 0 --interface enp7s0 \
  --urdf /home/lidio/quadruped_robots/quadruped_ros2_control_humble/descriptions/unitree/go2_description/urdf/robot.urdf \
  --csv /tmp/go2_estimator_dynamic_20261105.csv --duration-s 120
```

输出为 `/tmp/go2_estimator_dynamic_20261105.csv` 和 `/tmp/go2_estimator_dynamic_20261105.csv.sport.csv`。正常结束方式：完成动作、用户已用遥控停止且机器人静止后按Ctrl+C；或120秒到时自动退出。若100秒时流程尚未完成，用户结束动作并让机器人静止，再按Ctrl+C。数据过期、输入非有限或Estimator输出非有限时，程序记录最后故障行、返回非零并进入清理退出；会关闭两路订阅、写出队列中的SportModeState记录、刷新CSV并释放SDK通道。现场出现异常时，操作者应优先用原生遥控停止运动；采集进程退出不代表机器人已停止，程序没有机器人停止能力。

本轮在隔离目录离线构建目标成功；`--self-check --urdf ...`不初始化DDS，合成异步记录检查确认两路有不同接收时刻时分别写出时间、年龄、序号和值，且tau_est与SportModeState字段可按列解析。原Estimator静态输入/坐标变换检查也通过。验证只说明构建、CSV字段与离线计算路径，不证明动态估计准确。

## 最小现场步骤

开始前只需先确认少量现场条件：

1. 是否有约2m无遮挡、平整、无需转弯的直线路段及足够的停止空间；
2. 若电脑使用有线连接，电缆余量是否覆盖全路线，走线能否避开足部、关节和人员行走区域，且移动时不会被拉紧或形成绊脚环；
3. 是否能取得一个独立的距离/时间参考（例如现场测量起终点并计时，或安全固定机位视频）；操作者能否在不分心的情况下启动采集并操作原生遥控器。没有安全可靠条件时不走动。

建议一次最多记录120秒，预计有效动作阶段约40–60秒：机器人趴伏准备 → 开始采集 → 用原生遥控站立并静止约10秒 → 原生遥控直线慢走约2m（目标约0.1–0.2m/s，对应约10–20秒，仅为规划建议，不代表遥控器已标定可达到该速度）→ 原生遥控停止并静止约10秒 → 按现场熟悉的原生遥控操作结束观察并维持安全静止状态。若实际速度明显高于建议范围或无法稳定控制，不继续追求幅值对照；先停止，再保存记录。

全程原生运动服务保持运行。电脑只接收状态，不接管、不发布、不要求低层控制。采集开始前不要为观察释放服务、启动项目控制器或加载策略。

**停止条件：**步态或姿态明显异常、网线接近足部或被拉紧、原生遥控停止响应异常、任一路数据中断/无效，或现场人员要求停止。终端每秒显示LowState与SportModeState的序号变化、时龄和Sport数值有限标记；不对Sport擅设固定超时阈值。现场出现异常时，操作者应优先用原生遥控停止运动，再结束采集；如果程序因LowState过期/无效先行退出，仍须用原生遥控停止运动，不能把进程退出视为机器人停止。若遥控停止响应异常，按现场人员判断停止整个观察，不依赖采集器处置。程序本身不能停止机器人。

## 参考与分析方法

- 最容易取得的参考是测定直线路段距离并记录起步到遥控停止的时间：路程/时间给出该段平均速度，可检查Estimator积分量级和方向是否大致相符，不能提供瞬时速度真值，也无法单独区分加速/减速段误差。手机/固定机位视频若现场确实可安全布置，可辅助标出起停事件、机身朝向和位置随时间变化；未经空间标定的视频只能作事件/方向线索，不当作精确速度基准。
- 先分段报告站立静止、起步、稳定直走、停止后静止。分别按各自接收时间画Estimator速度与Sport原始速度，不把主CSV同一行当同步样本。原生速度的坐标系/单位未确认前，不给出有物理意义的方向/幅值误差结论；可报告原始曲线、事件先后和候选延迟，并标出Sport样本年龄。可识别的延迟受接收频率、消息异步与时间语义限制。
- 静止段报告世界/机体系速度均值、标准差和偏置；运动段检查预计前向分量的符号与方向是否一致、曲线变化形状、起步与停止响应；有实测路程时对Estimator世界系速度按其真实更新时间积分，并与实测距离比较。单次路段没有适用精度阈值，不自行判“合格”，先给误差数值供审阅。
- 查看12关节dq、tau_est和原始足力是否出现突变、非有限值或明显与步态不符的模式。tau_est为设备估计字段，不能据此声称力矩已独立准确标定。足力的比例/物理单位仍未知。

静态记录可以复核静止偏差；SportModeState对照只能反映原生服务提供的状态一致性。两者都不能证明关闭原生服务后的行为，也不能证明项目低层控制或Recovery下Estimator表现。慢走结果即使良好，也只是是否值得继续验证Estimator动态速度的证据，不是低层运动授权。

## 未知与下一步

关键未知包括SportModeState的坐标系、单位和stamp时基；动态Estimator的方向、幅值、响应延迟、脚滑/IMU偏置影响；现场直线路段、独立距离参考和网线安全余量是否满足。原生服务下观察不到原生关闭服务后的Estimator表现。

**唯一推荐下一步：**审阅本方案并确认上述三项现场条件后，由主助手另行安排一次原生服务保持运行的原生遥控短直线慢走只读观察；完成条件是得到完整双CSV、记录独立的路程/时间参考和操作事件，且所有异常/中止如实标出。此任务不涉及关服务、控制权切换、项目低层控制或策略运行。
