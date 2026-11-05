# Go2 Estimator慢走动态异常：原实现回放与最小修复

日期：2026-11-05。本轮只读处理两次既有慢走CSV；未连接机器人、初始化DDS、启动控制器或策略。

## 结论

原Estimator可由记录输入忠实回放：两批完整行对现场CSV中世界系速度的最大绝对差分别为`2.77e-12`和`2.13e-12 m/s`。因此回放足以用于同数据A/B/C方程对照。

确认了共享Estimator中的实现缺陷：世界系状态方程直接使用机体系KDL足端测量；固定支撑足速度约束漏掉基座角速度项。现已将修正作为显式Estimator模式加入，可供离线回放和只读观察器选用；所有既有控制器构造仍默认`LegacyBodyAsWorld`，仿真默认行为保持不变。

同数据对照显示，坐标修正和角速度项**没有消除前向速度振荡，也没有让积分位移更接近用户粗略报告**。修复的是确定的物理方程不一致；它不是本次动态异常的完整解释，不能据此把Estimator用于实机Recovery。

## 方程、坐标与适用假设

- Estimator状态为`x=[p_base_world, v_base_world, p_foot_1_world...p_foot_4_world]`。C矩阵对每条腿的位置测量为`p_foot_world-p_base_world`，速度测量为`-v_base_world`；四个足高测量为零，等价于对世界系地面高度的约束。初始化是`x_hat=0`、`P=100 I`；模型质量非零时每次`update()`用固定`dt=1/frequency=0.005s`推进一次。
- `quatToRotMat`按`[w,x,y,z]`计算标准主动旋转。结合`getFootPos()`和Recovery现有逆四元数速度转换，代码中的`R`语义是机身向世界的旋转。它与LowState四元数的实体轴/IMU安装关系仍需Go2实机资料确认。
- `QuadrupedRobot`由URDF中的`base`到FR/FL/RR/RL foot KDL链求位置；`J(q)dq`是足端相对base的线速度，表达在base链坐标系，参考点为足端。关节读取顺序为FR、FL、RR、RL，每腿hip/thigh/calf；硬件插件直接取LowState下标0–11的q/dq，不另换序或改符号，代码字段合同为rad和rad/s。仓库实现顺序自洽，但未凭这些日志证明本机固件电机编号/正方向与URDF完全一致。
- 修正版按固定足关系推导：`p_foot_world=p_base_world+R*p_body`；`0=v_base_world+R(omega_body×p_body+J(q)dq)`。由于C矩阵速度预测量是`-v_base_world`，观测向量应为`R(omega_body×p_body+J(q)dq)`，角速度项符号为正。B模式只把足端位置和`Jdq`旋到世界系；C模式再加`omega×p`。噪声矩阵中位置/速度向量块随同一旋转做`T R Tᵀ`坐标变换，不改变其数值参数；这依赖`RInit_`原本表达在KDL机体系测量基底的解释，源码没有给出独立噪声坐标说明。
- gyro由LowState原样进入接口，Estimator现有`getGyroGlobal()`执行`R*gyro`，据此C模式暂按机身坐标使用，单位预期rad/s，不做缩放。官方Unitree Go2 SDK2 [IMUState IDL](https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/idl/go2/IMUState_.hpp)只定义浮点数组，未在IDL中说明轴和单位；Unitree [Go1 EDU资料](https://github.com/UnitreeSupport/Unitree_Docs/blob/master/docs/get_started/Go1_Edu.md)记载四元数wxyz、gyro为rad/s，并提到角速度可能在上电初始坐标系，但该资料适用Go1 HighState，不能代替本机Go2 EDU固件/IMU轴证据。因此C结果仍以gyro与base轴重合且为机体系为假设，必须现场确认。C将gyro项当确定输入，没有另加gyro噪声协方差。
- `force < 1`时Estimator放大该腿足位置过程噪声及足速度/足高观测噪声；否则该腿按接触更新。阈值与其影响完全未改。两次动态窗口四足每行均判接触；高原始足力不是接触误判证据。

## 回放与短行

回放程序复用现有`Estimator`、`QuadrupedRobot`、Go2 URDF/KDL；没有重写滤波算法。按CSV逐行顺序将200Hz记录的最新LowState快照装入原接口，每行分别调用A/B/C一次，`dt`固定5ms；Estimator构造和零状态初始化与观察器一致。没有把约500Hz LowState中间消息插值或补入。三模式共用同一关节、IMU、足力序列、接触阈值、初始化和协方差数值。

启动短行的缺列位于尾部Sport/Estimator输出，不在模型输入区。程序按表头字段名读取到最后一个必需输入（足力），并检查短行长度仍覆盖该字段，所以第一次4行、第二次3行都能作为Estimator初始化输入恢复；这些行的原Estimator输出列不可信，A对现场输出的校核只用完整行。原始文件未改。A与现场CSV对照：

| 记录 | 回放行数（含短行） | 可比完整行 | A世界速度最大差 | A世界速度RMS差 |
|---|---:|---:|---:|---:|
| 第一次 | 23,998（恢复4行输入） | 23,994 | 2.77e-12 m/s | 1.41e-13 m/s |
| 重复 | 21,816（恢复3行输入） | 21,813 | 2.13e-12 m/s | 1.50e-13 m/s |

这说明输入映射、逐行更新、初始化及输出实现复现了记录中的原实现；不是完整500Hz传感器流重放，也不证明物理估计准确。

复现命令（仓库根目录；构建目录已配置为`BUILD_GO2_READONLY_ESTIMATOR_OBSERVER=ON`）：

```bash
rtk cmake --build /tmp/go2_estimator_build/controller --target go2_readonly_estimator --parallel 2
rtk cmake --build /tmp/go2_estimator_build/controller --target go2_estimator_replay --parallel 2
rtk cmake --build /tmp/go2_estimator_build/controller --target rl_quadruped_controller --parallel 2
rtk env LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu /tmp/go2_estimator_build/controller/go2_estimator_replay \
  --input logs/real_go2/20261105_104408_native_slow_walk/estimator.csv \
  --urdf quadruped_ros2_control_humble/descriptions/unitree/go2_description/urdf/robot.urdf \
  --output logs/real_go2/20261105_estimator_replay/first.csv
rtk env LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu /tmp/go2_estimator_build/controller/go2_estimator_replay \
  --input logs/real_go2/20261105_104924_native_slow_walk_repeat/estimator.csv \
  --urdf quadruped_ros2_control_humble/descriptions/unitree/go2_description/urdf/robot.urdf \
  --output logs/real_go2/20261105_estimator_replay/repeat.csv
rtk env MPLCONFIGDIR=/tmp/go2_estimator_replay_mpl python3 logs/real_go2/20261105_estimator_replay/analyze_replay.py
```

无DDS定向检查命令：

```bash
rtk env LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu /tmp/go2_estimator_build/controller/go2_estimator_replay --self-check
rtk env LD_LIBRARY_PATH=/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu:/home/lidio/Libraries/unitree_sdk2/lib \
  /tmp/go2_estimator_build/controller/go2_readonly_estimator --self-check \
  --urdf /home/lidio/quadruped_robots/quadruped_ros2_control_humble/descriptions/unitree/go2_description/urdf/robot.urdf \
  --foot-measurement-mode world_gyro
```

## A/B/C动态对照

A=原方程；B=仅将机体系足位置和速度观测旋到世界系；C=B再加`omega×p`项。窗口固定沿用事后探索区间：第一次78–90s，重复25–40s；不是精确人工事件边界。统计不扣静态偏差、不平滑、不缩放。各列中的速度是机体系`vx/vy/vz`，单位m/s；位移由世界系速度按记录时间梯形积分。前进/左右使用走动前1秒IMU航向，未补偿窗口内转向，因此是估计净投影，不是真值；水平净位移也不是累计路程。

| 记录/模式 | body vx 均值±SD [min,max] | body vy 均值±SD [min,max] | body vz 均值±SD [min,max] | 世界系净位移(x,y,z) m；水平净长；前进/左投影 m |
|---|---|---|---|---|
| 第一次 A | .0096±.2663 [-.645,.832] | -.0178±.0978 [-.363,.213] | -.0144±.0993 [-.455,.503] | (-.174,.176,-.172); .248; .133/-.209 |
| 第一次 B | .0051±.2714 [-.882,.623] | -.0050±.0520 [-.255,.234] | -.0099±.1129 [-.467,.488] | (-.076,.039,-.109); .085; .066/-.054 |
| 第一次 C | .0080±.2677 [-.879,.646] | -.0031±.0513 [-.283,.225] | -.0092±.1133 [-.480,.493] | (-.102,.007,-.102); .103; .099/-.028 |
| 重复 A | .0128±.2718 [-.707,.841] | -.0152±.1056 [-.358,.238] | -.0098±.0944 [-.422,.565] | (-.262,.165,-.139); .309; .224/-.214 |
| 重复 B | .0037±.2771 [-.850,.711] | -.0042±.0528 [-.254,.159] | -.0048±.1095 [-.463,.569] | (-.072,.034,-.059); .080; .064/-.048 |
| 重复 C | .0057±.2731 [-.845,.661] | -.0025±.0531 [-.271,.176] | -.0044±.1095 [-.470,.568] | (-.092,.003,-.054); .092; .089/-.021 |

坐标修正后`vy`波动减小，但前向`vx`标准差仍约0.27m/s且正负大幅振荡；C相对B的前向投影分别只变化约+0.033m和+0.025m。没有适用精度阈值，故不判合格/不合格，也不以用户约2m近似报告作标定目标。

前/后静止探索窗的body vx均值±SD（m/s）如下；完整xyz统计在`metrics.json`：

| 记录/阶段 | A | B | C |
|---|---:|---:|---:|
| 第一次运动前35–45s | .01454±.00153 | .00558±.00157 | .00758±.00166 |
| 第一次停止后100–110s | .01057±.00152 | .00376±.00149 | .00519±.00153 |
| 重复运动前10–20s | .01333±.00164 | .00557±.00156 | .00696±.00160 |
| 重复停止后45–55s | .01142±.00148 | .00486±.00161 | .00625±.00163 |

C固定足残差定义为`r_i=v_base_world+R(omega×p_i+J_i dq_i)`，假定该腿固定支撑时应接近零。动态窗四足force均`>=1`（第一次最小值FR/FL/RR/RL为10/13/13/14；重复为8/13/12/9），但C残差范数P95（FR/FL/RR/RL）第一次为`1.44/1.56/1.38/0.89 m/s`，重复为`1.46/1.67/1.39/0.98 m/s`；中位数约`0.037–0.046`和`0.067–0.077 m/s`。部分时刻与固定足关系相容、尾部仍有大残差，说明“全腿都接触”标签不足以说明全腿都满足固定足约束。没有独立摆/支撑真值，残差自身又依赖C估计和gyro假设，不能据此认定具体哪条腿摆动或把阈值调到20。

图中绘制未平滑的A/B/C机体系`vx`及同窗世界速度的前进投影积分：

- [第一次A/B/C对照图](../../../logs/real_go2/20261105_estimator_replay/first_comparison.png)
- [重复A/B/C对照图](../../../logs/real_go2/20261105_estimator_replay/repeat_comparison.png)
- [逐行回放及统计](../../../logs/real_go2/20261105_estimator_replay/)

## 最小修复、验证与影响

- `Estimator`增加三种测量模式。构造参数默认`LegacyBodyAsWorld`，未改变任何已有控制器构造/Go2仿真配置；`RotateToWorld`与`RotateToWorldWithGyro`仅供本次离线回放和观察器显式指定。观察器新增`--foot-measurement-mode legacy|world|world_gyro`，默认仍是legacy；没有把修正版接入实机launch或Recovery候选配置。
- 90°偏航定向几何检查确认body-x经R变为world-y；非零`omega_z=+2 rad/s`且`p_body=+x`时`omega×p=+y`，再经R变为world-`x`，确认C项符号。三模式的Go2 URDF静止合成输入均有限且速度接近零；既有CSV有/无Sport、无效输出89列自检继续通过。
- 离线构建通过：`go2_readonly_estimator`、`go2_estimator_replay`及共享`rl_quadruped_controller`。构建只证明编译/接口兼容，不是实机验证。
- 仿真控制器仍使用默认旧模式，因此本轮修复代码没有改变当前仿真实验行为。切换到修正模式会改变共享Estimator速度输出；须经后续审阅后显式选择，不能与既有仿真结果混用。

## 仍未知与唯一下一步

动态异常仍未完全解释：四腿contact标记没有提供实际支撑真值；足力单位、脚滑/摆动时刻、Go2固件关节符号/ID、IMU相对base轴、Go2 LowState gyro坐标与RInit协方差坐标约定仍未知。SportModeState不是标定真值；没有独立瞬时速度/位移真值。B/C离线输出只用于方程因果区分，不代表实际Go2动态精度，也不支持启用实机Recovery。

**唯一推荐下一步：**先获取并核对本机Go2 EDU固件适用的IMU到base轴/角速度坐标说明，再安排一次原生服务保持运行、只读记录并带安全固定机位视频标注脚接触相位的短直线慢走。完成条件是能独立标出支撑/摆动区间，并确认gyro轴映射；再决定是否保留C项及如何正确解释足力阈值。本轮不安排现场操作。
