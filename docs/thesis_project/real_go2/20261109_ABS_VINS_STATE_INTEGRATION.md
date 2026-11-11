# 2026-11-09 ABS 独立状态来源（VINS 机身里程计）接入

范围：离线实现与无设备验证。把一个**显式选择**的独立机身状态来源接进 ABS 的状态消费点。未连接 Go2／相机、未初始化实机 DDS、未启动 ROS 节点/策略/VINS/运动、未读取正在录制的 bag、未改模型/观测维度/切换参数/仿真配置、未提交 Git。另一个 A→B 与 D435i 精度实验的配置、采集目录与构建树未改动。

## 1. 针对的问题

原生运动服务释放后 `SportModeState` 已实测停更，而 ABS 的部分状态路径仍读它的 odom 接口（世界位置/速度）。已有独立的 D435i/VINS 机身转换程序（`/go2/vins/base_odometry`），但未接入 ABS。本轮要解决：

1. 导航位置、航向和策略线速度分别从哪里读；
2. 坐标系是否一致、是否重复旋转；
3. RA、Recovery 是否真正使用新来源；
4. 来源缺失或停更时是否会误用旧值。

## 2. 数据流（实际实现）

```text
VINS imu_propagate（相机 IMU 位姿 + 机体系速度）
  → scripts/go2_vins/go2_body_odometry.py（T_BI 刚体转换 + 杠杆臂项）
  → /go2/vins/base_odometry（nav_msgs/Odometry，frame=vins_world, child=base）
  → LeggedGymController 订阅回调（仅 navigation_state_source: vins_body 时创建）
      · 校验 frame/child_frame，校验全部数值有限、stamp 存在
      · 写入 CtrlComponent.external_body_state_（互斥保护的单个最新样本）
      · 时间回退 → 锁存失效；重复戳 → 丢弃且不刷新时龄（见第 7 节）
  → StateRL::runModel（每个策略周期）
      · 线速度：直接取机体系 twist，不再旋转
      · 导航位置/航向：进入 RL 会话时捕获一次导航原点，之后同一刚体变换
      · 锁存/缺失/超时 → safetyVeto → PASSIVE（不读 odom、不读 estimator）
  → RA 19 维观测（分量 0/1/2 = 机体系 vx/vy/vz）
  → 导航目标计算（body_x/body_y/heading_cmd、path_start、resample）
  → StateRLRec（RL_REC 入口门控 + 其 obs_.lin_vel）
  → StateRL::enter()（唯一的导航会话边界）：清锁存 + 重置原点 + 恢复配置目标
```

`navigation_state_source` 的默认值是 `odom`，此时**不创建订阅、不分配邮箱**，行为与改动前完全一致。

## 3. 坐标约定（关键）

| 数据 | 来源 | 表达坐标系 | 是否旋转 |
|---|---|---|---|
| 线速度（odom 路径，默认） | 里程计/`SportModeState` | 世界系 | 用 LowState 四元数转机体系（原逻辑，未改） |
| 线速度（vins_body） | `/go2/vins/base_odometry` twist | **已经是 base 机体系** | **不再旋转**（`requiresWorldToBodyRotation()` 返回 false） |
| 导航位置（odom，默认） | odom 世界 x/y | 世界系 | 不旋转 |
| 导航位置（vins_body） | VINS base 位姿 → 导航系 | 导航系 | 进入 RL 时捕获原点与航向，`R(-yaw0)` + 平移 |
| 导航航向（odom，默认） | LowState IMU 四元数 yaw | 世界系 | — |
| 导航航向（vins_body） | 同一 VINS 四元数 yaw | 导航系 | `wrap(yaw - yaw0)` |
| 策略 `gravity_vec`、body quaternion | LowState IMU | body | 不用于导航航向 |

要点：

- **位置与航向同源同变换**：vins_body 路径下二者都来自同一个 base 位姿和同一次原点捕获，不能一个用 VINS、一个用 LowState。
- **不会重复旋转**：机体系 twist 直接进观测；若再按 body quaternion 转一次，90° 航向下 (0.30, 0.20) 会变成 (0.20, −0.30)，测试中有对照断言。
- **RA 的 19 维**：`[vx, vy, vz, wx, wy, wz, cmd_x, cmd_y, ray2d(11)]`，分量 0/1/2 就是机身线速度。
- **Recovery 的 49 维实际不含线速度**（见第 5 节），这一条与先前假设不同，已按代码核实。
- **时间回退是锁存的**：一旦检测到 stamp 回退，后续正常消息不能自动恢复，必须重新进入 RL（新会话）才能再用该来源（见第 7 节）。

## 4. 改动文件

| 文件 | 改动 |
|---|---|
| `controllers/rl_quadruped_controller/include/.../FSM/NavigationStateSource.hpp` | 新增。来源选择、样本/原点结构、导航系变换、互斥邮箱、`selectSample()`。无 ROS/设备/torch 依赖。 |
| `.../include/.../control/CtrlComponent.h` | 增加 `navigation_state_source_`、`external_body_state_`、`external_state_timeout_ns_`。 |
| `.../src/RlQuadrupedController.h/.cpp` | 声明 5 个参数；仅在 vins_body 时创建订阅；frame/有限性校验；正常收尾反馈对该来源保持不可用；RL_REC 入口门控改名并带来源日志。 |
| `.../src/FSM/StateRL.h/.cpp` | 硬件新鲜度门限按来源分支；`obs_.lin_vel` 分支；导航位置/航向；导航原点每会话重置；正常收尾速度分支。 |
| `.../src/FSM/StateRLRec.h/.cpp` | `selectedLinearVelocity()` 返回「值 + 是否已是机体系」；vins_body 时不再旋转。 |
| `.../CMakeLists.txt`、`package.xml` | `nav_msgs` 依赖；`navigation_state_source` 测试目标。 |
| `.../config/real_go2_vins_state.candidate.yaml` | **新增候选配置，未被任何 launch 引用**（已 grep 确认）。 |
| `.../test/navigation_state_source.cpp` | 新增离线检查（49 项，含时间回退锁存/重复戳/新会话序列）。 |

## 5. 重要发现：Recovery 的 49 维观测不含机身线速度

代码依据：

- 本仓库 `src/FSM/AbsObservationContract.cpp::recovery()` = contact(4)+ang_vel(3)+gravity(3)+commands(3)+dof_pos(12)+dof_vel(12)+actions(12) = **49**，其中 **没有 lin_vel**。
- `config/rec/config.yaml`：`num_observations: 49`，obs 列表同样没有 `lin_vel`。
- 论文参考实现 `ABS/training/legged_gym/.../base/legged_robot_rec.py::compute_observations()` 的 `obs_buf` 也是 contact/ang_vel/projected_gravity/commands/dof_pos/dof_vel/actions（+可选 ray2d），**不含 base_lin_vel**。

因此：

- 真正需要机身线速度的是 **RA（19 维）和导航**；Recovery 只用优化出的 twist 命令和角速度、重力、关节量。
- 部署代码 `StateRLRec::runModel()` 仍会计算 `obs_.lin_vel` 并传入契约，但 `recovery()` 不使用它；该值本轮仍按所选来源正确产生（不再二次旋转），**未删除、未改维度、未改语义**。
- 为避免"看起来接了但没用"，Recovery 侧真正起作用的是 **RL_REC 入口门控**：新来源不可用时拒绝进入 Recovery，而不是静默放行。

## 6. 离线验证

隔离构建（独立目录，未覆盖其他任务的 `build/`、`install/`、`/tmp` VINS 树）：

```bash
cd /home/lidio/quadruped_robots/quadruped_ros2_control_humble
source /opt/ros/humble/setup.bash && source ./install/setup.bash
colcon --log-base /tmp/abs_nav_state_build/log build --packages-select rl_quadruped_controller \
  --build-base /tmp/abs_nav_state_build/build --install-base /tmp/abs_nav_state_build/install \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DTorch_DIR=/home/lidio/Libraries/libtorch-cpu-2.0.1/share/cmake/Torch
# 结果：1 package finished，无编译错误/警告
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/opt/ros/humble/lib \
  /tmp/abs_nav_state_build/build/rl_quadruped_controller/navigation_state_source
# 结果：33/33 PASS，exit 0
```

（`LD_LIBRARY_PATH` 只是绕开用户目录 conda 的旧 `libstdc++`，与本仓库既有的 `PYTHONNOUSERSITE` 处理同类，未改系统库。）

覆盖的风险与结果：

| 要验证的 | 结果 |
|---|---|
| 默认来源仍是 odom、拼写错误不被接受 | PASS（`parseSource` 拒错而不是回退默认） |
| 前向/左向速度进入对应观测分量 | PASS（RA 分量 0/1/2 = 0.30/0.20/0.05） |
| 90° 航向下不重复旋转 | PASS（规则函数返回「不需要旋转」；对照断言证明二次旋转会得到交换后的分量） |
| 导航位置与航向同一参考系 | PASS（捕获点→原点、沿捕获航向 1 m→导航 +x、源 −x 1 m→导航 +y、航向差 0.3 rad、跨 ±π 连续） |
| 旧 odom 不会遮蔽新来源 | PASS（`selectSample(odom,...)` 永远返回不可用；缺失邮箱返回不可用而不是 odom） |
| 新来源缺失/非有限/停更 ≠ 复用旧值 | PASS（首次无数据、超时、NaN、显式失效、到达早于查询时钟均返回不可用；无补零） |
| 时间回退在连续消息下仍被保留 | PASS（100→90→91 全部写入后再读一次仍不可用；后续 92…96 仍不可用；选择器同样拿不到） |
| 恢复只能通过显式新会话 | PASS（`beginSession()` 计数生效；单独开新会话仍无样本；新会话后的样本才可用，且属于新会话） |
| 重复时间戳不会让旧测量"保鲜" | PASS（重复被计数且不刷新到达时龄；原样本只保留自己的到达时间；不触发回退锁存） |
| 新会话不沿用旧原点 | PASS（第二个会话以新样本为原点；旧样本不再位于原点） |
| 默认仿真来源选择未改变 | PASS（默认参数 odom，且未在 `robot_control.yaml` 中新增任何参数） |

**这些检查证明的是**：来源选择与锁存逻辑、邮箱行为、坐标数学、RA 观测拼接在本机按代码意图工作。**没有证明**：真实 ROS/DDS 收发、完整控制器/FSM 运行、策略推理、真实机器人行为或任何停止能力。不能据此写成"整条链路已验证通过"。

## 7. 时间回退、重复时间戳与恢复流程

判据与处理（与实际实现一致）：

| 情形 | 判定 | 处理 |
|---|---|---|
| `stamp > 已存 stamp` | 正常 | 接受并刷新样本与到达时间 |
| `stamp == 已存 stamp` | 重复，不是回退 | 丢弃，**不刷新到达时间**，不锁存；计数 `duplicateRejections()` |
| `stamp < 已存 stamp` | 时间回退 | **锁存失效**：清空当前样本并永久拒绝后续消息（`rejectedAfterTrip()` 计数），直到显式新会话 |
| 已锁存 | — | 任何新消息都不再被接受，`fresh()`/`selectSample()` 恒为不可用 |

因此 `100 → 90 → 91` 的处理是：100 接受；90 触发锁存并清空；91 被拒绝。即使这三个回调都发生在两个控制周期之间、控制器只读一次，读到的仍然是"不可用"。之后再有 92、93、200 也不会自动恢复。

恢复流程（唯一入口）：控制器进入 PASSIVE 后，操作员重新进入 RL。`StateRL::enter()` 是导航会话边界，它同一次转移里做三件事：`beginSession()` 清除锁存、`navigation_origin_` 重置、目标恢复为配置值（`configured_goal_*`）。此后第一个到达的样本才被接受，并从它捕获新原点。运动过程中没有任何消息能触发这一步。

## 8. 仍未知/未验证

- **安装变换未测量**：`go2_body_odometry.py` 对未测量的 `go2_mount.unmeasured.json` 仍以退出码 2 拒绝运行，所以现场根本不会有 `/go2/vins/base_odometry`；选中 vins_body 时只会 fail-closed。这是刻意的现场前置条件，不用填零绕过。
- **未做在线消息验证**：本轮未启动 ROS/DDS，订阅帧校验、QoS、锁存日志与真实话题只在代码层成立，未收发一条真实消息。
- **世界原点静默变化不可判定**：接口只知道 frame 名和单调时间戳。一次不带时间回退的世界原点变化（例如 VINS 在时间继续增长的前提下换了世界系）无法区分，不做没有依据的跳变阈值。时间回退这一可检测情形已被锁存，不再依赖"下一条消息自动恢复"。
- **超时 300 ms 是暂定值**，未实机验证；它只决定"样本缺席多久算失效"，不代表来源真实延迟或精度。
- **动态速度/尺度/姿态精度未验证**，VINS z 不是离地高度：正常收尾（诊断用）在该来源下保持不可用，不编造 base 高度。
- **PASSIVE 不等于停住**：来源异常走的是既有的 `safetyVeto → PASSIVE` 路径，PASSIVE 发出的是 Unitree 停止哨兵（卸力请求），不保证制动、停稳或保持站立。本轮没有验证机器人在来源异常时的实际运动学行为；现场必须依赖独立的人工停止/支撑措施。
- 未启动策略推理，也未验证 torch 前向在真实观测下的数值行为。

## 9. 仿真影响

- 默认 `navigation_state_source: odom`：状态观测、导航、RA、Recovery、正常收尾全部走原路径，未改任何数值语义；重复/回退锁存只在 vins_body 的邮箱里存在，默认路径不创建邮箱。
- `robot_control.yaml`、launch、模型、观测维度/顺序/缩放、切换参数、仿真配置均未修改。
- 新代码在默认路径上只多一次枚举比较与一次 `else` 分支，不创建订阅、不读新话题。

## 10. 现场接入前剩余条件

1. 测量 base→相机安装变换并填写 `mount_measured=true`（`scripts/go2_vins/BODY_FRAME.md`）。
2. 只读核对 `go2_body_odometry.py` 输出的 base 位姿/速度方向与符号（原地转向、直行）。
3. **消息接收验证不得直接启动完整 `real_go2.launch.py`**（那会创建 LowCmd 发布器并释放原生模式）。应先采用不激活硬件、不创建 LowCmd、不切换模式的方式：只启动相机/VINS/转换节点，用独立只读订阅或 `ros2 topic echo`/`ros2 topic hz` 核对 `frame_id`、`child_frame_id`、时间戳单调性、发布频率与数值有限性；控制器侧的行为只在之后按批准流程单独验证。本轮只说明方式，不实现也不执行。
4. 确认新来源中断/回退时控制器确实离开 RL，并检查日志出现 `[NAV-STATE] source time regression latched` 或相应的 veto 记录；同时保留独立的人工停止手段，不以 PASSIVE 作为停稳保证。
5. 低层停止/交接链仍未闭环，ABS 实机运动继续 NO-GO；本轮不改变该结论。

## 11. 文件与复现

- 候选配置：`quadruped_ros2_control_humble/controllers/rl_quadruped_controller/config/real_go2_vins_state.candidate.yaml`
- 离线检查：`quadruped_ros2_control_humble/controllers/rl_quadruped_controller/test/navigation_state_source.cpp`（49 项，`exit 0`）
- 构建目录：`/tmp/abs_nav_state_build`（临时，可删；不覆盖其他任务使用的工作空间 install/build）
- 数据来源说明：[机身坐标转换](../../../scripts/go2_vins/BODY_FRAME.md)、[转换程序](../../../scripts/go2_vins/go2_body_odometry.py)
- 本轮未连接设备、未初始化 DDS、未提交 Git。
