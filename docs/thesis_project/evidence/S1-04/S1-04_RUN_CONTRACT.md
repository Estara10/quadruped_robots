# S1-04 单次诊断 run 合同（修订稿，待 Director 审阅）

本合同只适用于 S1-04 的短时非正式诊断，不是正式实验参数冻结。一个 run 由唯一 `run_id` 和证据目录标识。运行准备、站立、策略运动、运动终止、停止策略请求/确认、进程清理是不同阶段；清理记录不得反向改写已经确定的运动终局。

## 来源与配置绑定

- 运行前确认没有未结束的 `ros2_control_node`/controller manager；发现实例时先记录进程、父子关系、参数文件和来源，再决定是否属于本任务残留。不得直接终止来源不明的进程。
- package prefix 检查和 launch 使用同一个 shell 顺序：加载 ROS 及工作区环境，再把本次 overlay 放在 `AMENT_PREFIX_PATH` 首位。实际 launch 的 `robot_control.yaml` 参数路径、控制器加载路径日志和第一条策略帧须相互一致。
- 策略开始后绑定第一条策略 `session_id`、候选和阈值。每个周期持续检查它们；session/candidate/阈值发生非预期变化时保留全部原始周期并将该 run 标为 INVALID，不能过滤某个来源后继续计算。
- 每个 `StateRL` 写入全局 `/mujoco_rt_frame`。只有策略帧 ABI 本身不携带 OS writer PID；运行日志须把 session、writer PID 和模型/配置路径关联起来。没有对应 writer 记录的帧不可归属到当前控制器。
- 诊断上下文分别保存“请求配置”和“运行时直接观察配置”。目标、候选、阈值或模型路径不一致时不开始运动；运行中发现变化立即终止并标 INVALID。

## 阶段与运动边界

- 记录器先启动，然后启动 MuJoCo 并取得当前 capture/model/scenario 绑定的 `LIVE` 碰撞快照，最后启动 ROS/controller。准备和站立时间均记录，不计入运动指标。
- 策略首个有效周期只提供初始世界平面位姿，不表示运动开始。运动起点定义为首次观察到位姿相对初始位姿移动至少 `0.10 m` 的策略周期；同时记录前一有效周期和触发周期构成的仿真时间区间，不插值。没有此事件时运动起点和运动指标为 UNKNOWN。
- 目标使用世界坐标、米；位姿取 `world_pose[0:2]`，距离为欧氏距离。到达诊断阈值 `0.50 m` 必须由有效策略帧直接满足，并记录 session、周期、仿真时间和距离。
- 运动终止边界由触发终局的事件确定，保存事件时间以及可可靠配对时的 session、周期、仿真时间。安全日志若含控制器 `detection_ns`，同时保留该事件时间和执行器发现日志的时间；两者不可混称。
- 终止条件由策略快照触发时，先把触发判断使用的同一份一致性原始快照同步写入并核对 `session_id/sequence/rl_step`、RA、模式、动作、位姿与时钟字段，再关闭策略帧采集窗口和发送清理命令；不得依赖异步采集线程赶帧或事后重读共享内存。采集器为单写入者，重复周期只写一次，session 漂移仍使记录无效。保存失败须把 `terminal_frame_capture.complete=false`，分析器拒绝完整运动指标。运动切换、Recovery、距离、速度及 timeline 使用运动起点至持久化终止周期（含边界），清理尾段不计入。外部安全日志没有对应策略快照时，只记录其事件时刻和最后已保存周期，仿真终止时间保持 UNKNOWN。

## 终局、安全事件和收尾

运动阶段的终局判据依次为：启动失败；控制器安全 veto/PASSIVE；权威障碍碰撞终止；有效策略帧到达；诊断仿真时限；人工停止；墙钟保护/意外进程退出导致的不完整。终局仅确定一次；同时保留所有其他已观察事件。

- 障碍碰撞依据 capture/model/scenario 绑定且 `LIVE` 的快照，并有 `robot_obstacle_contacts > 0` 或碰撞边沿。正常地面接触不算障碍碰撞。碰撞事实与本诊断采用的首次权威障碍接触即终止规则分开记录；未覆盖全程时不能声称无碰撞。
- 当前没有本任务可用的权威跌倒检测，跌倒结果为 UNKNOWN。`6 s` 为本次诊断运动阶段的仿真时间上限；`50 s` 墙钟保护只防止挂起，触发时记为不完整，不能代替仿真时间超时。
- `[ABS-CONTRACT] ... transition_request=PASSIVE`、冻结位置 `[EMERGENCY]` 或全局 hard-stop 是运动阶段安全事件；从事件日志提取可用的控制器单调事件时间和日志接收时间。任一运动期安全事件进入本 run 的安全终局。
- 运动终局之后出现的安全事件仍写入 `cleanup_events`，标记 `phase=cleanup`；它不能凭优先级改写此前的运动终局。原始事件与主要终局同时保留。
- command 1/PASSIVE 只表示停止请求，不表示策略已停止。若发送该请求，记录请求时间；只在观察到 FSM 转入 `passive` 的直接日志或其他明确策略停止状态/后续周期证据后，才记相应层级的确认。到 SIGINT 时仍无确认，应报告收尾未确认以及请求至 SIGINT 的持续时间，不称为正常停止。策略 FSM 转入 PASSIVE 也不证明 MuJoCo 已消费最后一条命令。
- ROS/controller、RViz、MuJoCo 和采集器的退出状态及清理完成时间逐项记录。若终局后策略仍输出，报告停止确认失败和可由记录支持的输出持续区间；不得以进程组退出反推最后命令或 MuJoCo 消费时刻。

## 指标和缺口

- 策略周期质量单独统计全采集范围：相邻同 session 且 `rl_step` 连续的单调时间差；报告周期中位数、P95、最小/最大及 `有效间隔数 / 总单调时长` 的整体频率。明确实际纳入的帧数/周期对数。缺周期、无效帧和候选/session 变化均留痕；多 session 记录不能合并成一个频率或 run 指标。
- 运动指标仅用明确运动起点周期至终局周期（含边界周期）之间的记录。切换次数以周期内 `mode_before → mode_after` 的真实变化为准；总转换、Agile→Recovery、Recovery→Agile、相邻切换间隔及短时反向切换使用同一组边界。短时窗口 `W=0.50 s` 仅是本次诊断值，不是正式冻结值。
- Recovery 区间从进入 Recovery 的周期计至首次切回 Agile 的周期时间；区间终点使用该首个 Agile 周期，不使用前一帧 Recovery。累计 Recovery 时间和有效运动时间占比只对连续且仿真时钟有效的周期对计算。
- 缺周期、无效时钟、仿真时间回退或 session 变化时不跨缺口累加、不插值。缺口影响的 Recovery 区间记为删失/区间/UNKNOWN；未闭合尾段不得用最后一帧 Recovery 伪造终点。
- 目标距离、路径长度和平均路径速度只使用运动边界内同一 session 的相邻有效位姿周期。路径平均速度为可用路径段长度之和除以对应有效仿真时长；缺口两侧不跨越。
- timeline 只画运动边界内有效仿真时钟周期、RA、进入/退出阈值、模式和终局；目标距离可用时再绘制。准备和收尾事件可以在运行表中单独显示。INVALID 记录仅生成说明图，不拼接来源。

## 启动命令与本次状态

```bash
DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
python3 scripts/s1_run_diagnostic.py \
  --output-root docs/thesis_project/evidence/S1-04/diagnostic_runs_YYYYMMDD \
  --overlay /tmp/s1_04_overlay --goal-x 1.0 --sim-limit-s 6 --wall-cap-s 50
```

脚本须在启动任何仿真进程前检查无 controller manager、同一 shell 能解析预期 `go2_description` overlay 和工作区控制器包，并核对临时配置与三个模型文件存在。run 内再以实际 launch 参数、控制器来源日志、writer PID/session、策略候选和目标日志确认配置确已生效。

2026-10-08 补证发现此前混流源于一个仍运行的 S1-03 overlay ROS/controller 进程与 S1-04 workspace-configured controller 共写 `/mujoco_rt_frame`。该残留组已按 SIGINT 停止并确认退出。之后一次配置正确的短诊断在第一策略周期即触发 `monotonic_clock_order` 安全 veto，没有运动起点；依合同停止后续 run 尝试。因此本合同实现与测试结果仍待审阅，S1-04 两次相同设置且可计算结果的运行条件尚未满足。
