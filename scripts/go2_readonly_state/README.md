# Go2只读状态观察器

独立订阅Unitree SDK2的`rt/lowstate`和`rt/sportmodestate`，不启动ROS controller或硬件插件，不创建LowCmd发布器，不调用运动控制/控制权接口，也不加载策略模型。程序必须显式给出DDS domain和网卡名。

## 构建

本机已安装SDK2到`/home/lidio/Libraries/unitree_sdk2`，可运行：

```bash
cmake -S scripts/go2_readonly_state -B /tmp/go2_readonly_state_build \
  -DCMAKE_PREFIX_PATH=/home/lidio/Libraries/unitree_sdk2 \
  -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/go2_readonly_state_build --parallel 2
```

其他机器将`CMAKE_PREFIX_PATH`改为该机器已有的Unitree SDK2安装前缀，不需要安装ROS或模型推理依赖。

## 本机无实机检查

```bash
/tmp/go2_readonly_state_build/go2_readonly_state --domain 0 --interface lo
```

当前没有Go2或仿真消息发布到loopback时，两路都显示“尚未收到数据”，值字段不显示占位零。按Ctrl+C退出。loopback不支持组播时SDK会提示关闭组播，这是本机无数据检查的预期情况。

## 将来连接Go2

先根据实际网络与DDS配置填写，不要照搬旧手册中的接口名或假设domain：

```bash
REAL_NETWORK_INTERFACE="<已确认的网卡名>"
DDS_DOMAIN="<已确认的domain编号>"
/tmp/go2_readonly_state_build/go2_readonly_state \
  --domain "$DDS_DOMAIN" --interface "$REAL_NETWORK_INTERFACE"
```

该程序不释放原生运动控制权，因此SportModeState在释放控制权后的更新情况仍需其他经审阅的受控步骤验证；本程序本身不能完成该项验证。

## 输出解释

- LowState与SportModeState分开计数、计频、计最近接收时龄。计时来自本机`steady_clock`接收回调，不代表机器人内部传感器采样频率。
- 接收频率是启动后首条至最近一条消息的平均频率`(n−1)/(t_last−t_first)`；少于两条时显示样本不足。若消息停更，频率保留已观测平均值，最近接收时龄会持续增大。
- 数值展示为各topic最近一次收到的样本，旁边总显示该样本接收时龄，不伪装成新数据。
- 两路分别显示运行以来最大本机接收间隔，供现场核对暂定100ms期限。SportModeState另显示原始机器人stamp、距最近stamp变化的本机时长、mode及error_code；这些字段的实际语义需实机确认，持续收到消息本身不证明里程有效。
- 关节顺序为FR、FL、RR、RL，每腿hip/thigh/calf，与当前Go2硬件映射一致。q/dq显示接口预期rad/rad/s并提示待实机核实。
- 静止接口核对另显示12关节的tau_est（接口预期Nm，估计值）、电机温度原始值、IMU加速度、LowState原始tick和version。version字段不直接当作固件版本；温度和加速度单位仍须结合实机与设备说明核对。显示值为约1Hz快照，不是逐条500Hz原始数据。
- IMU四元数按SDK/现有硬件映射的w,x,y,z显示；角速度保留SDK原始值，单位待实机核实。足端力显示SDK原始值，不擅自换算成牛顿。
- SportModeState的position/velocity原样显示；坐标系、单位及其实际定位含义待实机核实。
- 日志文件默认关闭；本版本只刷新终端，不写文件。

## 曲线与前置相机窗口

在机器人已连接、网络参数已确认时，可以使用[只读可视化窗口](../go2_live_view.py)。它运行本目录的两个独立SDK2程序：状态订阅器的`--json`模式每100ms输出最新快照，相机程序只调用`VideoClient::GetImageSample()`取图；均不发布LowCmd或切换运动模式。已有终端模式仍每秒刷新一次。

构建上面的CMake项目会同时生成`go2_readonly_state`和`go2_readonly_camera`。本机已经验证的运行命令（无需sudo）：

```bash
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/home/lidio/Libraries/unitree_sdk2/lib \
/usr/bin/python3 -s /home/lidio/quadruped_robots/scripts/go2_live_view.py \
  --domain 0 --interface enp7s0
```

这里的domain和网卡适用于2026-11-04已实际收到消息的本机连接。`-s`使用系统配套的NumPy/Matplotlib/PyQt5，避免本机用户目录NumPy 2与系统Matplotlib的ABI冲突；没有修改或降级已有Python环境。

窗口显示最近30秒的曲线：选择一条腿看关节角、角速度、估计力矩与温度，同时显示IMU、位置、线速度、四足原始力值及12关节数值表。状态快照约10Hz，绘图目标5Hz；DDS实际接收频率单独显示，不把图形刷新率当作控制频率。DDS接收龄和显示样本龄分别显示，过期来源的曲线留空，不用零值填补缺失数据。原始mode是机器人SportModeState状态，不是项目Agile/Recovery模式；本窗口不运行策略。

右侧为Go2前置彩色相机，不是深度或障碍距离输入；相机返回错误时显示实际SDK返回码，不自动切换或开启机器人服务。画面显示距最近取图成功的本机时长，不当作传感器采样时龄。图像和状态可能不同步。

点击“结束只读监视”、关闭窗口或按Ctrl+C，会结束本窗口启动的两个读取进程；不会停止或操控机器人。进程错误日志路径会打印在终端；默认不录制视频、不保存全频状态数据。

## Estimator只读观察（需ROS/控制器构建依赖）

`rl_quadruped_controller`包提供可选目标`go2_readonly_estimator`。它复用包内的`Estimator`和`QuadrupedRobot`，不复制算法。实时模式只订阅`rt/lowstate`，不创建`LowCmd`发布器、`MotionSwitcher`客户端、ROS controller或策略模型；`--self-check`不初始化DDS。

### 离线构建与自检

本机已有ROS工作区、CPU LibTorch和Unitree SDK2。以下命令使用隔离于仿真配置的构建目录：

```bash
cmake -S /home/lidio/quadruped_robots/quadruped_ros2_control_humble/controllers/rl_quadruped_controller \
  -B /tmp/go2_estimator_build/controller \
  -DPython3_EXECUTABLE=/usr/bin/python3 \
  -Dcontrol_input_msgs_DIR=/home/lidio/quadruped_ros2_control_humble/install/control_input_msgs/share/control_input_msgs/cmake \
  -Dcontroller_common_DIR=/home/lidio/quadruped_ros2_control_humble/install/controller_common/share/controller_common/cmake \
  -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch \
  -DCaffe2_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Caffe2 \
  -DBUILD_GO2_READONLY_ESTIMATOR_OBSERVER=ON \
  -Dunitree_sdk2_DIR=/home/lidio/Libraries/unitree_sdk2/lib/cmake/unitree_sdk2
cmake --build /tmp/go2_estimator_build/controller --target go2_readonly_estimator --parallel 2
```

若该机器的Torch CMake配置含CUDA依赖，应先使用本机可用的CPU LibTorch CMake目录；本命令不安装或切换软件包。

```bash
env LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu:/home/lidio/Libraries/unitree_sdk2/lib \
  /tmp/go2_estimator_build/controller/go2_readonly_estimator \
  --self-check \
  --urdf /home/lidio/quadruped_robots/quadruped_ros2_control_humble/descriptions/unitree/go2_description/urdf/robot.urdf
```

自检构造真实Go2 URDF/KDL链和现有Estimator，输入明确标记的合成静态状态，检查足端链可构造、阈值1接触标志、初始静止速度有限且接近零，以及90度偏航下Estimator矩阵转置与Recovery逆四元数旋转的一致性。它不使用历史10Hz快照重放，不证明实机准确度或动态速度质量。

### 后续现场静止观察命令

只有用户安排现场观察后才运行。保留原生运动服务运行，不释放或恢复控制权，不启动本项目控制器，不要求机器人运动。已实际通信验证的本机参数是domain 0和`enp7s0`；运行前仍应确认网卡仍是当前Go2链路。

```bash
env LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu:/home/lidio/Libraries/unitree_sdk2/lib \
  /tmp/go2_estimator_build/controller/go2_readonly_estimator \
  --domain 0 --interface enp7s0 \
  --urdf /home/lidio/quadruped_robots/quadruped_ros2_control_humble/descriptions/unitree/go2_description/urdf/robot.urdf \
  --csv /tmp/go2_estimator_static_20261104.csv --duration-s 30
```

### 频率、记录与判据

- Go2配置中controller manager为1000Hz，而`rl_quadruped_controller.update_rate`为200Hz；Estimator构造时以控制器频率设固定`dt=0.005s`。观察器按200Hz调用现有Estimator，使其固定dt与当前RL控制器配置一致；不会按每条LowState更新。LowState在各Estimator周期之间若未变化，CSV会以`repeated_sample=1`标出，并记录接收序号、原始tick、接收间隔和样本时龄。
- 原始输入逐周期写入CSV：FR/FL/RR/RL各腿hip/thigh/calf的q/dq，IMU四元数wxyz、角速度和加速度，四路原始足力。IMU顺序与硬件插件一致；四足接触列使用Estimator当前比较规则`force >= 1`，不改阈值。原始足力物理单位/标定仍未知。
- 同行记录Estimator世界系速度与`R.transpose()*v_world`机体系速度，角度转换用于Recovery契约对照。采集运行时间和间隔使用本机steady clock，不是机器人传感器时间戳。
- CSV包含相对本机steady起点的Estimator更新时间和LowState接收时间、两次Estimator调用间隔、样本时龄、DDS回调间隔、本地接收序号及原始tick。LowState未到达3秒则不更新并退出；已收到的样本若时龄超过100ms、输入非有限/四元数为零或估计输出非有限，先记录最后一行再停止。按Ctrl+C可提前正常结束。完成后查看CSV中的接收间隔、时龄、重复标记、输入/估计有效标志、姿态稳定性、接触判断以及静止速度是否稳定/明显漂移。
- 静止速度接近零只验证输入链路、初始化和静态数值基础表现。趴伏四足力14–16仍会按阈值1判为接触，按用户要求保留并单独解释；此姿态不能用来证明支撑接触语义正确。静止观察不能证明走动时速度的方向、幅值、延迟或Recovery兼容性。
- 如果设备姿态/支撑不稳定、有人要控制机器人、状态过期/非有限值触发退出，或任何数据映射不符，立即停止观察；工具本身不承担机器人的停止动作。此方案不验证电脑失联停止、遥控停止效果或控制权交还。
