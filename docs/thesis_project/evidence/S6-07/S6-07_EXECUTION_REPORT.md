# S6-07 执行报告：启动就绪等待与 ROS 会话复用

**最新状态：COMPLETE — Director 有条件通过（2026-10-13）。** 通过范围为条件就绪等待、同一 ROS 查询/发布会话复用、新鲜具名关节采样、完整启动计时，以及正常和碰撞终局后的清理流程。平地 A 为 `VALID / ARRIVED`，random04 A 为 `VALID / COLLISION_TERMINATION`；两次均保存周期0、完整终止帧和自动分析，相关进程退出。MuJoCo 启动至 RL 请求分别约 **7.26 秒**和 **7.52 秒**；不计算未经证实的提速比例。本批属于流程诊断，不能给出算法性能或参数优劣结论。

## 最新定向补证：关节采样、会话退出与两次 A 诊断（2026-10-13）

### `/joint_states` 发布端和采样

实际启动中的唯一 ROS graph endpoint 为 `/joint_state_broadcaster` 发布 `sensor_msgs/msg/JointState` 到 `/joint_states`。持久会话通过 `get_publishers_info_by_topic()` 将端点来源和 QoS 写入本次 STATUS。两次实际 run 观察到相同 QoS：Reliable、Transient Local、Automatic liveliness；History 为 System Default / UNKNOWN，depth=0。订阅端使用 `qos_profile_sensor_data`（Best Effort、Volatile、Keep Last、depth=5），两次都实际收到消息；这说明当前仿真链兼容，不改变发布端 QoS。

采样器现在只接受请求单调时刻之后的新消息；还核对 publisher 恰好唯一且身份/消息类型正确、关节名无重复且集合恰好为 FR/FL/RR/RL 各 hip/thigh/calf 共 12 项、position 与 velocity 各 12 个且均为有限数。拒绝样本会保留单调时间、来源、名称和原因；无合格样本继续阻止 RL command 3。两次实际消息都通过，q/dq 及 effort 数组、name 顺序、header stamp、`/usr/bin/python3`、RMW/ROS 环境、采集时间和发布端点均写入各自 `run_context.json` 的 `initial_state.pre_rl_joint_state`。两次消息观察到的 name 顺序均为 `RL_calf, FL_hip, FR_thigh, RL_thigh, RR_thigh, RL_hip, FL_calf, FL_thigh, RR_calf, RR_hip, FR_hip, FR_calf`（各名称带 `_joint` 后缀）；校验不假设数组顺序，而是使用同一消息中的名称解释 q/dq。样本都在请求后约 3.0–3.3 ms 到达，采样响应时年龄约 2.1–2.3 ms，并早于 RL 请求。

### 离线验证

`scripts/test_s6_07_startup.py` **27/27** 通过；覆盖生产 ROS 入口和握手、publisher 缺失/重复/身份类型不符时不放行、请求前样本拒绝、12 个预期名称、缺长度/重复或错误名称、非有限 q/dq、采样新鲜度，以及 EOF 关闭的退出结果。`py_compile` 对生产会话、启动诊断及定向检查文件通过。没有改动模型、配置值、阈值、保持、策略或安全门；本轮无 C++ 改动和构建需求。

### 两次运行与阶段耗时

固定配置均为 A、E=-0.05、goal `(7,0)`、到达半径 0.5 m、运动上限 20 仿真秒/120 墙钟秒、到达后站立观察 5 仿真秒；使用同一安装 controller、模型、初态准备流程、启动增益交接与安全规则。每组各一次，不是严格初态重放，也不是正式实验。实际配置来源和唯一 controller/writer/session 均在原始 `run_context.json` 中。`scene_flat.xml` 与 `scene_random_04.xml` 分别使用独立批次 overlay，未写回源码配置。

| 场景 / run_id | 结果与数据 | 关节采样、cycle 0 与终止帧 | 任务概况 |
|---|---|---|---|
| `scene_flat.xml` / `fd4b212a534b49878a365dbb91fdb42c` | `VALID / ARRIVED`；190 策略帧，1 个周期缺口，10 个无效策略时钟帧；周期频率中位数 49.934 Hz、间隔 p50=20.037 ms、p95=27.612 ms。 | `/joint_states` 合格采样；cycle 0 身份 `(session=20741439602704, cycle=0, monotonic_ns=20741499055493)`；终止帧同一快照持久化，`cycle=190, source_sequence=382, sim_time=10.286 s`。 | 到达距离 0.4868 m；Recovery 0 次；运动窗口安全覆盖完整，未记录碰撞、足端撞击或确认跌倒；站立保持 5.006 s 完成。有效连续运动时间 3.354 s / 3.812 s elapsed，路径 5.915 m，平均有效速度 1.764 m/s。 |
| `scene_random_04.xml` / `4087cd227f414c17965e72d3647c0634` | `VALID / COLLISION_TERMINATION`；415 策略帧，2 个周期缺口，1 个无效策略时钟帧；周期频率中位数 50.033 Hz、间隔 p50=19.988 ms、p95=28.975 ms。 | `/joint_states` 合格采样；cycle 0 身份 `(session=20786536300306, cycle=0, monotonic_ns=20786601439550)`；终止帧同一快照持久化，`cycle=416, source_sequence=834, sim_time=15.034 s`。 | 在距目标 3.7443 m 时发生真实碰撞终局：robot body `RL_calf` / geom 61 与 `ppt_obstacle_15` / geom 15。记录 2 次足端接触、2 次足端撞击；运动期跌倒结果 UNKNOWN，有未确认姿态候选，不能报未跌倒。PASSIVE 转换确认；该 run 未进入正常到达后站立观察。 |

两次真实阶段时间来自事件处的单调时间戳，单位秒；均为单次观测：

| 阶段 | flat A | random04 A |
|---|---:|---:|
| MuJoCo Popen → 当前 physics LIVE | 0.8431 | 0.8551 |
| ROS launch → 持久 session READY | 0.6177 | 0.5722 |
| ROS launch → controller active / 来源与唯一性就绪 | 2.1995 | 2.4475 |
| 首个 STAND_STEP → 连续站立准入 | 4.1395 | 4.1324 |
| 站立准入 → RL 请求 | 0.0492 | 0.0451 |
| MuJoCo 启动 → RL 请求（完整启动链） | 7.2603 | 7.5174 |
| RL command 发布返回 → 首个策略周期 0 | 0.0654 | 0.0714 |

flat 与 random04 两次 command 3 均在相同 ROS 会话中发布；实际策略首周期为 cycle 0。命令发布返回与 MuJoCo 消费仍是不同证据层级。两次均完成周期0到终止帧的记录闭环，策略频率约 50 Hz。上述数据证明当前流程能够走通且耗时可测；由于旧基线起点时间戳不精确、环境及阶段定义不同，本报告不计算或声称节省秒数/百分比。

### 会话退出码调查与清理

早前 `f7d107...` run 的查询/发布进程 exit 1、未被强制终止且整体 closeout complete。旧 `run_context` 没保存该 helper 的 stderr/异常栈，所以无法把它定性为正常退出或程序错误；结合当时代码是对仍等待 stdin 的 ROS 会话进程组发送 SIGINT，怀疑是中断式关闭，但这仍是解释而非已证实根因。生产清理现先关闭本次 session stdin，给 helper 有界时间处理 EOF 并正常销毁 ROS 节点；超时才降级信号。flat 与 random04 两次均记录 session exit 0、MuJoCo exit 0、ROS launch exit 0，未强制退出，closeout errors 为空。这证明新关闭路径本次正常完成，不能追溯证明旧 exit 1 的内部原因。

flat 正常到达后完成站立观察；random04 碰撞后 command 1/PASSIVE 状态转换被 controller 确认。两次收尾均先关闭 MuJoCo，再关闭 ROS/controller，最后以 EOF 结束本次查询/发布节点。逐项进程扫描确认本次 MuJoCo、controller、ROS launch、查询/发布节点及辅助进程均无残留；未关闭无关会话。

原始证据位于：

- flat：[运行目录](../../../../logs/manual_abcd_compare/20261013_151420_a6b65d/A/rep_01/fd4b212a534b49878a365dbb91fdb42c/)、[批次结果](../../../../logs/manual_abcd_compare/20261013_151420_a6b65d/comparison.md)
- random04：[运行目录](../../../../logs/manual_abcd_compare/20261013_151505_7d600c/A/rep_01/4087cd227f414c17965e72d3647c0634/)、[批次结果](../../../../logs/manual_abcd_compare/20261013_151505_7d600c/comparison.md)

此前 `f7d107...` 的 q/dq 准备失败、早期 UUID 启动失败及历史原始数据均保留，不改判。本轮仍只属每场景一次非正式流程核验，不能据此推断成功率、性能排名、参数最优或一般启动表现；无 Recovery 的 flat run 不检验切换效果。切换过程、碰撞和跌倒结论各自仍受周期及时钟覆盖边界约束，MuJoCo 命令消费未知。

## 前一轮定向修正与最小补证（历史记录，2026-10-13）

## 本轮定向修订与补证（2026-10-13）

### 生产入口、记录回归和分析器

- 将调用链实际使用的 UUID 节点名生成与 ROS shell 命令构造提取为 `startup_ros_session_node_name()`、`startup_ros_session_command()`；实际 `start_prepared_stop_publisher()` 直接使用这两个生产函数。离线检查覆盖唯一合法名称、`/usr/bin/python3` helper、overlay/AMENT 环境、原 ROS 环境传递、READY 身份握手及 topic/service、空 STATUS 保持 pending、握手拒绝时清理本次进程。最初的 UUID 符号错误保留在旧 INVALID run 中。
- `test_s1_04_contract.py` 原 Recovery 时长失败是测试期望未跟随当前指标合同：主统计窗口自首个 `rl_entered && rl_active` 周期起，fixture 四周期中有效相邻区间为 0.2+0.4=0.6 s，Recovery 有效区间为 0.4 s，占主窗口有效时间的 2/3；历史 0.10 m 位移辅助窗口仍为 0.4 s。仅据此更新 fixture 预期，没有修改这些指标定义。
- 终止帧缺失检查发现一个真实分析器回归：虽已判定终止载荷不完整，分析器仍能输出主策略窗口切换/路径数据。现主窗口仅在终止帧载荷完整且身份可配对时开放；缺终止载荷时主运动指标保持 UNKNOWN。终止帧写入/去重、缺帧拒绝、cycle 0 身份关联和生产收尾分别独立运行核验。老 fixture 同步补齐当前 `FrameCapture` 所需的候选/阈值/场景来源字段。
- 上次自动分析在该 INVALID run 上因 `/usr/bin/python3` 用户 site 的 NumPy 2.2.6 与系统 Matplotlib 3.5.1 二进制不兼容而失败。仅给分析子进程禁用用户 site 并设置临时 `MPLCONFIGDIR`；之后对同一已保存 INVALID run 运行生产分析器并重建批次表/时间线成功，未改动原始记录。该结果证明无帧失败记录可被分析和报告，不证明有效策略 run 的完整分析链已通过。
- S6-07 定向验证：`scripts/test_s6_07_startup.py` **24/24**；`scripts/test_s1_04_contract.py` 全部通过；`scripts/test_s6_03_preparation.py` 通过，并另行单独执行 cycle 0 用例；终止帧完整持久化、缺失终止帧拒绝、生产收尾/错误仍清理分别单独通过；相关 Python 文件 `py_compile` 通过。无 C++ 或控制器运行代码变更，无增量构建。

### 启动会话和真实准备尝试

桌面预检在批准的沙箱外最小调用中通过：`DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority`，GLFW 3.4.0 窗口创建成功；OpenGL vendor `Intel`、renderer `Mesa Intel(R) Graphics (RPL-P)`、version `4.6 Compatibility / Mesa 23.2.1`。未改 X 授权、驱动或系统配置。

本次唯一场景 run：`f7d107c6c04242e7bf38ab017e865dcf`，原始目录为 [run 目录](../../../../logs/manual_abcd_compare/20261013_145717_ec32fb/A/rep_01/f7d107c6c04242e7bf38ab017e865dcf/)。使用 A、`E=-0.05`、`X=-0.05`、`H=0`、`scene_flat.xml`、goal `(7,0)`、到达半径 `0.5 m`；诊断计划为 20 仿真秒/120 墙钟秒、到达后站立 5 仿真秒。独立 overlay、安装 controller library、A 配置和模型来源、唯一 writer/controller、同一 ROS session 节点、活动 controller 均有实际记录；loaned 足端 interface 顺序为 FR/FL/RR/RL，符合生产转换。

同一个 ROS 会话 `s6_07_startup_session_7f2cb5b018` 完成 graph/service/active/source 查询，并发布了两次 command 2；控制器 `[STAND-READY]` 及原有连续物理站稳门通过。command 1/PASSIVE 停止请求也由会话发布并得到 controller transition 确认。RL 前的 12 关节 q/dq 样本则由旧的单次 ROS CLI 采样器请求，返回 `rc=2`、无样本；因此安全门拒绝 command 3。这是本次直接的准备失败，不是控制器来源冲突或策略跌倒。运行日志显示 `joint_state_broadcaster` 已激活；为何订阅端未收到样本尚无直接证据。代码检查发现旧采样器新建独立 ROS 节点并使用默认 depth-10 QoS；现已改为在持久 ROS 会话节点内订阅 `/joint_states`，采用 sensor-data QoS，并只接受采样请求之后的新鲜、12个唯一具名且 q/dq 有限的样本。此修正尚无新的 live run 验证。

记录器已就绪但 JSONL 没有策略帧，周期 0/首态 companion、运动终止帧及策略指标因此均不可用。原始终局为 `PREPARATION_FAILURE`，分析结果为 `INVALID / INCOMPLETE_UNTRUSTED_RECORD`；分析表及 `timeline.png` 在关闭 NumPy 用户 site 后生成。不得将其改称 `STARTUP_FAILURE` 或有效运动结果。

### 本次事件时间

以下为 run_context 在事件发生时写入的单调时间戳差值，不使用旧 sleep 后才写的阶段标记：

| 阶段 | 本次实际耗时 | 证据与边界 |
|---|---:|---|
| MuJoCo 进程 Popen → 本 run physics LIVE 确认 | 0.857342 s | 同 capture、flat scene 和模型指纹的新鲜递增 authority；物理就绪门通过。 |
| ROS launch → 持久会话 READY | 0.528304 s | 同一会话进程与节点 `READY`；不是 controller ready。 |
| ROS launch → controller 唯一且 active / 来源确认 | 2.547015 s | graph、list_controllers、唯一 manager/writer、配置和 library 检查同时通过。 |
| 首个 STAND_STEP 发布调用 → 连续站稳准入 | 4.130922 s | 原起立插值及 0.50 仿真秒物理稳定门完整保留；两次 STAND_STEP。 |
| RL 请求 → 首策略周期 | UNKNOWN | 没有 command 3，也没有进入 RL。 |
| MuJoCo 进程启动这一统一起点 → RL | UNKNOWN | 在起立后关节采样失败；到 PREPARATION_FAILURE 为 10.358398 s，不能拿它替代启动至 RL。 |

这次没有测到实际 policy/RA 频率。旧 S6-06 时间戳边界不准确、含固定等待，仍只能作历史描述；不能计算精确节省秒数或百分比。等待上限是 deadline，不是固定等待。

### 停止与清理

准备失败后立即停止，不启动 scene_random_04。原始 run 记录 MuJoCo exit 0，随后 ROS launch exit 0，查询/发布会话进程最终 exit 1；均未强制杀除，closeout complete。会话 exit 1 如实保留，不称正常零退出。收尾顺序为先关 MuJoCo 再关 controller/ROS 和会话。最终精确进程扫描未发现本次 MuJoCo、controller、ROS launch、RViz、采集器、停止发布器或辅助进程残留。

**历史证据结论（该轮尝试结束时）：**MuJoCo 与 ROS 的条件就绪、同会话 controller/source 查询、起立、停止发布及确认已在一次非正式启动尝试中真实执行；周期0、command 3、策略运行、终止帧、有效策略指标、随机04场景与端到端提速当时尚未验证。该结论早于后续定向补证，不能代表最新状态。

## 前一轮执行记录（历史，2026-10-13 较早尝试）

- 移除了启动链中的固定 MuJoCo 3 秒与 ROS 8 秒等待。MuJoCo 现在等本次可执行文件产生两份新鲜、递增且身份匹配的物理 authority；ROS 启动现在等同一个查询会话返回完整 graph、controller-manager 服务、唯一 active controller、唯一 manager/writer、配置和库来源证据。
- 同一持久 ROS 节点负责状态查询和起立、RL、停止命令发布；正常启动不再等待 daemon graph，也不重复启动 ROS CLI 来查询 controller 状态或发布单条命令。RL 准入前仍通过同一会话刷新 controller 状态。
- 当时定向离线验证及 Python 编译检查通过：`scripts/test_s6_07_startup.py` **18/18**。该次运行实际证明了 MuJoCo 就绪门能拒绝旧 authority 并在当前 authority 到达时通过；未能实测 ROS 查询会话进入 ready 状态。后续 24 项当前检查及真实 ROS 会话证据见上方最新补证。
- 唯一一次平地尝试因 `uuid4` 名称生成符号错误，在 ROS 查询节点创建前结束为 `STARTUP_FAILURE`，0 个策略帧。该缺陷已修正，但本任务规定基础设施失败后停止后续场景运行，所以没有重跑平地或启动 scene_random_04。

## 启动时序与就绪门（实现说明；最新实际耗时见上文）

| 顺序 | 动作及继续条件 | 上限与失败处理 |
|---|---|---|
| 1 | 检查实际包路径、独立 overlay、模型文件及唯一性；启动并回读 recorder meta，确认为本 run 可读写 | 失败直接进入统一收尾，不启动场景 |
| 2 | 启动本次 MuJoCo，核对 `/proc/<pid>/exe`；等待本次 capture/model/scene 身份、仿真时间、sequence、physics step 和时间戳均有效且新鲜的 authority 连续递增两次 | 单一 8 秒 deadline；每轮约 20 ms 轮询；进程退出立即失败；预运行共享快照不能通过 |
| 3 | 启动本次 ROS launch 和一个 `/usr/bin/python3` 查询/命令节点；复用该节点的 graph 发现与 controller-manager 服务客户端 | 从 ROS launch 实际启动时刻起 15 秒总 deadline；查询只使用剩余时间。服务未就绪/发现延迟继续等待；重复节点、多个 manager/writer、来源或环境冲突立即失败 |
| 4 | 同一状态快照同时检查 graph 中 manager/controller 唯一性、服务返回的 controller=`active`、manager 进程属于本次 launch、controller library/config/model 来源、ROS domain/RMW 及控制 topic 订阅者 | graph 空或服务不可用不是 ready；manager 存活或“模型加载完成”日志也不单独放行。daemon 信息仅可另作诊断 |
| 5 | 通过同一 publisher 按原顺序发起立命令；等待真实 controller 状态转换，再等待原站立/关节/安全准入。刷新同一 ROS 会话后才允许 command 3 | 各阶段沿用原有有界超时与准入条件；未确认状态不推进，不自动重发 command 3 |
| 6 | 继续原有初态、cycle 0、任务起点与策略记录；停止命令仍优先，正常和异常收尾顺序不变 | 启动异常、Ctrl+C、用户中止进入共同清理：先关闭本次 MuJoCo，再关闭本次 ROS launch、查询/发布节点，并检查退出事实 |

超时数值是失败上限，不是固定睡眠。就绪证据缺失时不会把它当成成功；发现启动进程退出会立即失败，不等待剩余时间。没有新建进程管理框架，也未改控制、模型、参数、S6-05 增益交接、站稳、安全、指标或收尾语义。

## 旧启动耗时与时间戳边界（历史基线）

旧数据取自 S6-06 的 `logs/manual_abcd_compare/20261013_101052_11b8e3/<group>/rep_01/<run_id>/run_context.json`。同一文件的 `events` 有旧启动阶段时间点：

| 阶段 | 旧 A/B/C/D 观测 | 解释 |
|---|---:|---|
| recorder ready → 旧事件 `mujoco_started` | 3.008 / 3.005 / 3.005 / 3.004 s | 对应旧流程约 3 秒固定等待。但旧字段 `mujoco_started` 在 sleep 之后记录，不能当作实际进程 Popen 时刻；因此旧数据没有精确的 MuJoCo 进程启动→physics LIVE 时长 |
| 旧 `mujoco_started` → authority LIVE | 0.004 / 0.025 / 0.003 / 0.004 s | 仅是晚记的启动事件到首次确认 LIVE 的间隔，不能解释成 MuJoCo 总启动耗时 |
| 旧 `mujoco_started` → controller active | 9.380 / 9.717 / 9.385 / 10.680 s | 混有旧 ROS 固定 8 秒等待、launch/service/graph 启动；旧记录没有足够精确的 ROS launch 起点，不能从中单独归因节省值 |
| controller active → 旧 publisher ready | 5.502 / 10.661 / 10.858 / 5.753 s | 旧流程另起发布器/CLI并重复等待发现；这段也受站立准备和运行环境影响，不作为纯 CLI 开销测量 |
| publisher ready → 站立确认 | 5.315 / 5.633 / 5.357 / 5.384 s | 原站立判据和准备耗时；本任务未缩短或弱化 |

旧阶段取自不同组各一次 S6-06 运行，属于带条件基线；任务组别、墙钟负载和启动时序有差异，不用于性能比较。

较早 UUID 失败 run 的生命周期时间戳（历史，事件发生时用 supervisor 单调时钟写入）：

| 事件 | 与上一事件间隔 | 证据/状态 |
|---|---:|---|
| MuJoCo Popen → 两个本 run 新鲜递增 LIVE 快照确认 | 0.824549 s | authority 首样本 `(seq=22, physics_step=10, sim_time=0.020)`，确认样本 `(seq=42, physics_step=20, sim_time=0.040)`；两者均绑定本次 capture、flat 场景与模型指纹 |
| physics LIVE → ROS launch 启动 | 0.004287 s | 已进入 ROS launch，但还未形成本次 ROS 查询会话 |
| ROS launch → STARTUP_FAILURE | 0.000054 s | 查询节点名生成调用 `uuid4` 未定义即抛错；不是 ROS graph 查询结果 |

该较早尝试实际展示了 MuJoCo 条件就绪在约 0.825 秒通过，不再固定等待 3 秒；它不代表可节省 `3−0.825` 秒，因为旧进程启动边界时间戳不准确，且该次没有完成 ROS 到 RL 的启动链。当前补证的实时时序见报告顶部。

## 较早尝试的代码与针对性验证（历史）

- `scripts/s1_run_diagnostic.py`：MuJoCo 有界就绪门以实际 Popen 时间为起点；ROS deadline 使用同一个绝对结束时刻；复用查询会话读取 graph/service/active 状态并核对进程、配置、动态库及通信环境；RL 前刷新状态；命令通过持久 publisher 发布。旧 daemon/CLI 查询函数仍在文件中作为未调用的诊断代码，普通启动路径不调用它们。
- `scripts/s2_03_stop_publisher.py`、`scripts/s6_07_startup_protocol.py`：一个唯一查询/命令 ROS 节点，一条现有 `/control_input` topic/消息语义，发布前检查订阅者，并区分 publish 返回、controller 状态确认和 MuJoCo 消费。
- `scripts/manual_ab_compare.py`：可用 `--classification` 标记诊断批次；补足 INVALID 运行中 readiness/lifecycle 为 `null` 时的汇总处理。原始运行记录未改写。
- 当时离线测试 18/18 覆盖 deadline、当前/旧/过期 authority、提前退出、graph 延迟与重复、服务可用但 controller 非 active、多 writer、来源/环境冲突、发布失败/无订阅、command 3 准入、停止优先、收尾和 INVALID 汇总。`py_compile` 对五个直接相关 Python 文件通过；新增生产入口测试见报告顶部。
- 直接相关的 S6-03 准入回归 fixture 已加入“controller readiness 必须新鲜”条件，`scripts/test_s6_03_preparation.py` 通过。此前运行 `scripts/test_s1_04_contract.py` 时，主窗口 Recovery 时长旧期望失败，终止帧/收尾用例因此没有执行；该失败的 fixture 口径及终止 payload 生产回归已在本轮顶部修正，并独立重验。当前真实场景仍无策略周期，因此 cycle 0、终止帧和站立收尾未获 live run 证据。
- 无 C++ 或运行时算法文件修改，故本轮没有 controller/MuJoCo 构建。
- 桌面预检：现有 `DISPLAY=:0` 与 Xauthority 下 X11 连接通过；短 GLFW 窗口创建成功，renderer 为 `Mesa Intel(R) Graphics (RPL-P)`，OpenGL 4.6 Compatibility / Mesa 23.2.1。没有更改授权、驱动或系统配置。ROS 查询节点单独以 READY 启动并正常退出的预检曾通过；但本次场景 run 在查询节点创建前失败，不能把单独 smoke 说成完整运行会话证据。

针对性测试不是全仓库测试。未启动全仓库测试、训练或正式实验。

## 较早有限实际诊断（历史 UUID 失败）

| 次序/场景 | Run ID | 实际配置 | 终局及记录 | 后续状态 |
|---|---|---|---|---|
| 1 / `scene_flat.xml` | `e8b15af3b4a34424ad365f0283c4ea03` | A，E=-0.05，goal=(7,0)，radius=0.5 m；overlay 独立副本；MuJoCo scene authority 确认 `flat`、障碍数0 | `STARTUP_FAILURE`，0 policy frames，记录 `INVALID/no_frame_lines`；未创建 command/query 节点、未站立准入、未发 command 3、未进入 RL；无可解释导航指标 | controller active/source、cycle 0、起立/站稳、策略切换和正常站立收尾均未测；按停止规则，scene_random_04 未启动 |

唯一原始运行目录：[run 目录](../../../../logs/manual_abcd_compare/20261013_144145_e18899/A/rep_01/e8b15af3b4a34424ad365f0283c4ea03/)，含 `run_context.json`、`runtime_record.jsonl`、`terminal.json`、`process_facts.json`、`metrics.json`、启动/场景日志。原始终局和运行文件保留。其原始批次汇总在 INVALID/null readiness 情况下未纳入行；修正后的生产汇总代码从同一 `metrics.json` 与 `process_facts.json` 离线生成，未触碰原始运行：[修正派生对照表](flat_attempt_reanalysis/comparison.md)、[修正派生机器汇总](flat_attempt_reanalysis/batch_summary.json)。

## 可复制命令与历史补证边界

下面保留的是此前单次 A 平地尝试的命令形式。该尝试后来以 UUID 错误失败；之后获授权的第二次平地尝试见上文并以准备失败结束。按停止规则没有启动随机04；不得将此命令视为本轮继续运行授权：

```bash
rtk proxy env ROS_HOME=/tmp/s6_07_ros_runtime ABS_PLOT_PYTHON=/home/lidio/anaconda3/bin/python3 \
  /usr/bin/python3 scripts/manual_ab_compare.py \
  --groups A --repeats 1 --scene scene_flat.xml \
  --entry-threshold -0.05 \
  --classification 'S6-07 ordinary A startup-flow verification; not a formal experiment' \
  --display :0 --xauthority /run/user/1000/gdm/Xauthority \
  --plot-python /home/lidio/anaconda3/bin/python3
```

**历史缺口说明（后续两次 A 诊断前）：**当时仍需取得平地 A 和 `scene_random_04.xml` A 的有效运行，并核对唯一节点、配置、cycle 0、终止帧、正常控制及清理。该缺口已由报告开头和“最新定向补证”一节记录的后续运行补齐；旧记录不改写。

## 本次平地准备失败尝试的清理详情

本次 run 的 `process_facts.json` 记录 MuJoCo exit 0、ROS launch exit 0、`shutdown_complete=true`、`closeout_complete=true`、`forced_termination=false`；命令节点未成功创建，因而不在该 run 的已启动子进程中。run_context 中曾记录第二次幂等清理访问空停止状态时的 `None.get` 诊断项；它不改变已记录的进程退出事实。本轮已在生产清理路径将该可选状态按空映射处理，并增加空生命周期字段离线回归检查。后续精确进程扫描没有发现本项目 MuJoCo、controller/ROS launch、RViz、采集器或启动辅助节点残留。沙箱预检的临时 ROS 日志已检查并删除；精确扫描未发现相关残留进程。

**较早通俗总结（历史 UUID 尝试）：**这次当时只做到离线检查和平地启动预检，UUID 名称错误使它未进入 ROS 查询阶段。该 run 后来保留为 INVALID，不应视为本轮最新结果。

**历史通俗总结（对应当时的准备失败记录，非本任务最新结果）：**MuJoCo/ROS 条件就绪与同一 ROS 会话查询、起立和停止链已在一次启动诊断中实际走到站稳；关节状态采样未取得，安全门因此没有放行 RL。修正采样器和分析器后，针对性离线检查通过，但新采样链尚未 live 验证。那次记录没有策略周期、场景04运行或准确的启动至 RL 总耗时；其环境已关闭。该总结不包含后续成功补证。

## Director 审阅后的最新通俗总结（2026-10-13）

S6-07 已由 Director 有条件通过并标记 **COMPLETE**。条件等待、同一 ROS 会话查询和命令发布、新鲜且具名的 12 关节 q/dq 采样、启动各阶段计时，以及正常到达和碰撞终局后的清理均有本轮证据。两次从 MuJoCo 启动到 RL 请求约为 **7.26 秒**和 **7.52 秒**，没有可靠旧基线可用于计算提速比例。

平地运行到达；random04 运行因 RL 小腿与障碍碰撞终止。random04 的跌倒结论仍为 UNKNOWN，策略周期和仿真时钟有缺口；早期 ROS 会话 exit 1 的原因 UNKNOWN，MuJoCo 是否消费已写出的命令也未确认。历史失败与 INVALID 记录保留。参数比较尚未启动；本任务两次运行涉及的环境均已核对清理退出。
