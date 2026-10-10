# S1-06 Execution Report — 部署通道排列离线核验

**状态：COMPLETE — Director 有条件通过。** 通过范围：生产 observation/order helper 的唯一标记值离线检查，以及实际 Agile、inline Recovery 和独立 Recovery 状态类动作转换源码核对。此结论不证明私有动作成员已被直接调用，也不证明本轮运行时接口取值、策略推理或机器人动作。

## 本轮范围与执行

- 按当前权威文档和 `CURRENT_STATE.md` 中 2026-10-10 的动作顺序证据执行；未恢复旧 P1 任务。
- 只查看了 Agile/RA/Recovery 状态转换、Go2 控制器参数、Go2 ros2_control sensor 声明及 Unitree MuJoCo 对应接口发布代码。
- 直接构建已存在的控制器组件：`cmake --build quadruped_ros2_control_humble/build/rl_quadruped_controller --target rl_quadruped_controller -j2`，结果 `[100%] Built target rl_quadruped_controller`。
- 离线标记检查源：[S1-06_channel_order_check.cpp](S1-06_channel_order_check.cpp)，结果：[S1-06_channel_order_check.log](S1-06_channel_order_check.log)。测试链接本次构建的 controller shared library，调用 `AbsObservationContract.cpp` 中生产 helper；没有运行模型 forward、仿真或实机。
- 未改动控制器、映射、模型或运行配置。没有发现可由离线排列检查直接判定的错误；足接触的接口数值语义仍有待来源核实。

## 部署具名关节对应表

控制器具名关节配置顺序为 FR、FL、RR、RL，每腿 hip/thigh/calf；策略顺序为 FL、FR、RL、RR，每腿 hip/thigh/calf。控制器到策略读取索引为 `[3,4,5,0,1,2,9,10,11,6,7,8]`；策略动作到控制器槽位使用相同索引排列（该置换为自身逆变换）。

| 策略动作通道 | 控制器槽位 | 控制器具名关节 |
|---|---:|---|
| FL hip | 3 | `FL_hip_joint` |
| FL thigh | 4 | `FL_thigh_joint` |
| FL calf | 5 | `FL_calf_joint` |
| FR hip | 0 | `FR_hip_joint` |
| FR thigh | 1 | `FR_thigh_joint` |
| FR calf | 2 | `FR_calf_joint` |
| RL hip | 9 | `RL_hip_joint` |
| RL thigh | 10 | `RL_thigh_joint` |
| RL calf | 11 | `RL_calf_joint` |
| RR hip | 6 | `RR_hip_joint` |
| RR thigh | 7 | `RR_thigh_joint` |
| RR calf | 8 | `RR_calf_joint` |

**证据位置：**具名控制器关节及 `stand_pos` 位于 [robot_control.yaml](../../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml:150)；策略顺序设置在 [Agile config.yaml](../../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml:1) 和 [Recovery config.yaml](../../../../quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/rec/config.yaml:1)；转换实现在 [AbsObservationContract.cpp](../../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/AbsObservationContract.cpp:10)、[StateRL.cpp](../../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp:221) 和 [StateRLRec.cpp](../../../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRLRec.cpp:189)。

## 逐项核验

### Actor 输出：Agile 与 inline Recovery

- Agile `StateRL::runModel()` 对 Agile `forward()` 输出和 inline Recovery `rec_model_.forward()` 输出都调用 `StateRL::policyToCtrlDofOrder()`，随后执行 controller-order 的 hip 缩放、裁剪、`default_dof_pos` 相加，并按 controller 槽位写关节目标（`StateRL.cpp:1958-2013`）。
- 独立 Recovery 状态 `StateRLRec::runModel()` 对输出调用 `StateRLRec::policyToCtrlDofOrder()`，再写 `output_dof_pos_` 和 controller-order 电机目标（`StateRLRec.cpp:472-500`）。本合同所说 inline Recovery 指 Agile 状态内的 Recovery 分支；独立 Recovery 状态也使用同样置换。
- 两个状态类的私有输出函数均直接执行 `index_select`，索引为 `[3,4,5,0,1,2,9,10,11,6,7,8]`。离线可执行检查通过生产库中的 `abs_observation::policyToControllerDof()` 检查同一输出排列；没有构造控制器对象去直接调用两个私有成员，也没有执行策略 forward。此范围限制保留在报告中。
- 输出操作的后续逐关节缩放索引来自配置 `[0,3,6,9]`，映射到四腿的 hip 槽位；scale、PD gain、target 累加均按 controller 槽位逐元素处理，没有额外的关节置换证据。

### 关节位置、速度、默认位置、bias 与前一动作

- 运行时配置中的控制器关节及 `stand_pos` 是 FR、FL、RR、RL 顺序；`StateRL` 和 `StateRLRec` 都把该 `stand_pos` 作为 `target_pos`，并原序构造 `default_dof_pos`（`RlQuadrupedController.cpp:335-336`、两个状态构造器及 `loadYaml()`）。所以默认位置属于 controller 顺序后再交给 observation builder 转换；每槽默认值随对应具名关节移动。
- 两份模型配置中的 `dof_bias` 是长度 12 的零向量，文件注释和 `ModelParams` 声明它为 controller 顺序。状态构造时通过状态类 `ctrlToPolicyDofOrder()` 先变为 policy 顺序，再传入 `abs_observation::agile()` / `recovery()`；位置项按 `(q_policy - default_policy - bias_policy) * dof_pos_scale` 计算。
- 实际关节状态 `q/dq` 和 `obs_.actions` 保存在 controller 槽位；两个 observation builder 对 `q`、default、`dq` 和 prior action 使用同一 DOF 索引转换。速度再乘配置 `dof_vel_scale=0.2`。测试使用每个关节不同的标记值，验证 q、default、bias、dq 和 prior action；Agile/Recovery 输入段布局均 PASS。
- `policyToControllerDof(controllerToPolicyDof(previous_action))` 离线往返 PASS。状态代码将已 remap 的 controller-order `clamped_actions` 存入 `obs_.actions`，下一周期 builder 再转换到策略顺序。因此前一动作没有额外交换通道。

### 足接触与具名来源

- 当前 RL controller 参数要求的 `foot_force_interfaces` 为 `FR, FL, RR, RL`（`robot_control.yaml:222-227`）；Go2 `ros2_control.xacro` 对 `foot_force` 声明的 state interface 名也为 `FR, FL, RR, RL`（`xacro/ros2_control.xacro:172-177`）。`state_interface_configuration()` 按此参数名形成请求，`on_activate()` 将 `state_interfaces_` 中 prefix 为 `foot_force` 的接口按收到顺序追加到 `foot_force_state_interface_`；Agile/Recovery 读取该 vector 并将四个布尔接触值作为 controller-order 向量。生产转换再按 `[1,0,3,2]` 变为 policy `FL, FR, RL, RR`，离线 helper 检查 PASS。
- `robot_control.yaml` 中较早出现的 `FL, RL, FR, RR` 属于 `ocs2_quadruped_controller` 的配置块，不是下方 `rl_quadruped_controller` 块；不能据此描述为 RL 接口排列。
- **UNKNOWN：**本轮没有启动 ROS/controller manager，因此没有直接观察它交付给 controller 的四个实际 `LoanedStateInterface` 名称顺序。另一个来源边界是 `hardware_unitree_mujoco/HardwareUnitree.cpp` 按索引把 SDK `low_state_.foot_force()[i]` 复制到 `foot_force_[i]`，而硬件接口名称来自模型 sensor 声明；当前直接检查的源码没有证明 SDK 数组每个索引的解剖足名与 FR/FL/RR/RL 一一对应。故这里只能确认请求/声明/转换约定相互一致，不能声称本轮已证实物理接触值的运行时具名来源。

## 离线检查结果

日志中的所有项目均 PASS：q/default/bias/dq/prior-action 读取重排、四维接触重排、Agile 与 Recovery 输出 helper 重排、prior-action 往返、Agile 与 inline Recovery observation 字段布局。每个关节标记各不相同，因此腿序或腿内关节错位会触发断言。

检查调用 production observation helper；动作输出类方法则源代码路径已核对，并确认使用相同的 index_select 向量，测试用 production shared helper 覆盖该数学置换。两者的直接调用范围如上如实说明。控制器组件增量构建成功。

## 结论和保留事项

### Director 审阅结论

Director 有条件通过 S1-06。接受范围仅为生产 helper 的离线检查和状态类生产源码路径核对。保留限制：`StateRL` / `StateRLRec` 的私有动作转换成员没有由本轮 harness 直接调用；实际 loaned contact interface 顺序和底层 SDK foot-force 数组的逐索引具名语义仍 UNKNOWN。不得将通过描述为完整动态路径或运行时接触语义已验证。

- **已核对：**部署代码规定的 FL/FR/RL/RR × hip/thigh/calf 策略通道到具名 controller joint 映射；q、dq、default、bias、prior action 的转换关系；Agile 与 inline Recovery 的 observation/action 源码排列；独立 Recovery 状态也使用同一动作重排。
- **未发现错位：**对本次离线验证的数值映射未发现错位，未修改运行时映射。
- **UNKNOWN：**当前 ROS 实际下发的 foot-force interface loan 顺序；MuJoCo SDK `low_state_.foot_force()` 数组的逐索引具名语义；本轮未直接调用 StateRL/StateRLRec 私有输出成员或观测实际 runtime values；模型 artifact 本身不含关节名元数据。训练/导出多源证据支持 actor 的策略顺序，但不是本任务对 runtime interface 的新实测。
- **后续最小核实：**若足接触接口语义将用于结论或安全判定，在已授权的正常控制器启动中只读取并记录 loaned interface 的具名名称序列，以及硬件/仿真提供方对应的四个具名源值到 interface 的绑定；不改序、不试排列、不依靠步态判断。对输出私有成员如需严格动态调用证据，可在离线测试构建中暴露测试 seam，仍不加载/执行策略。

## 文件与边界

- 新建 `tasks/S1-06.md`、本执行报告、唯一标记值检查源及日志。
- 更新 `CURRENT_STATE.md`，记载 S1-06 当前进展。
- 本任务按 Director 有条件通过标记 COMPLETE。没有启动 S2 技术执行；没有训练、推理、仿真、实机、正式实验或 Git 提交。
