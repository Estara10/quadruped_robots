# S2-03 执行报告

**最新审阅状态：COMPLETE — Director 有条件通过（2026-10-10）。**

**通过范围：**两次手动运行接触对象核对且未发现地面误分类；停止发布通道在 RL 前就绪；本次终局后约 2.972 ms 开始发布 PASSIVE，controller 在发布开始后约 0.414 ms 完成转换；固定等待时间不再作为策略输出持续时间；运动终局与收尾观察分离；一次 VALID / ARRIVED 诊断支持修正后的正常运行闭环。此通过不表示 A 组稳定避障性能已经验证。

本报告只覆盖 S2-03 的接触对象复核、停止通道修正和一次有界短诊断，不是 A 组避障性能结论。两次手动运行的原始记录未改写。

## 1. 两次手动运行的接触对象

接触快照给出了具名/编号几何体；与当前 `scene_obstacle.xml` 的编译模型对应后，geom 2 是静态障碍 box，不是地面。记录中的短接触按现有规则保留为机器人—障碍接触，不把它升级解释成肉眼明显撞击或严重碰撞。

| Run | 终止位置 / 距目标 | 原始接触 | 核对结果 |
|---|---|---|---|
| `133db6b96ed448fbba5ac5f0b4c6e232` | `(3.9979,-0.2248) m` / `3.0105 m` | geom 25 ↔ geom 2，physics step 13161–13162，约 2 ms | geom 25 是 FL 小腿碰撞圆柱；geom 2 是静态障碍 box。 |
| `239400f9ed724fd39f56ca854fd20ef0` | `(4.4341,-0.3288) m` / `2.5869 m` | geom 27 ↔ geom 2，physics step 13944–13953，约 18 ms | geom 27 是 FL 足端碰撞球；geom 2 是同一障碍 box。 |

直接记录见 `logs/manual_single_run_01/<run-id>/{runtime_record.jsonl,terminal.json,run_context.json}`。这两个旧快照的名称字段部分为空，几何编号与部位的对应来自 Director 对当前场景编译模型的只读核对。旧记录没有 contact distance/force，无法判断 margin 接触、穿透量或冲击强度。第二次手动运行的终局后策略继续移动证据仍保留在原 ROS 日志；终止帧以后的位移不计作运动终局或到达。

## 2. 修正内容与定向核验

- `scripts/s2_03_stop_publisher.py` 在 RL 请求前启动持久 `/control_input` 发布器；等待实际订阅者后才报告 READY。终局时通过该通道发布 command 1，停止路径不再新建 ROS CLI 或加载 ROS 环境。
- `scripts/s1_run_diagnostic.py` 将运动记录在终止快照后立即封口，再发送 PASSIVE。发布器调用、`publish()` 返回、控制器确认、有限收尾观察、进程退出使用各自的单调时间戳/状态。收尾策略帧另存于 `run_context.json`，不进入 `runtime_record.jsonl` 的运动帧或运动指标。
- `RlQuadrupedController.cpp` 在全局 hard-stop 分支完成 PASSIVE 的 exit/enter 和 FSM 指针/模式更新后写出 `[HARD-STOP-CONFIRMED]`，含 command、PASSIVE 状态、`steady_ns` 和是否发生状态改变。停止确认代表 controller 完成状态转换，不代表 MuJoCo 已消费最后命令。
- 去掉 `output_after_request_duration_ns=250000000` 固定等待假测量。无后续策略帧时只报告实际观察窗口和 UNKNOWN 边界，不推断实际输出持续时长。
- final log 扫描中没有嵌入 controller 单调事件时间的行现在使用 `event_monotonic_ns=null` 并另记日志观察时刻。已保存的 `ros_launch.log` 与 `runtime_record.jsonl` 保持原样；原 terminal 行里的 `observed_monotonic_ns` 对 GLOBAL_HARD_STOP 实际是最终扫描日志的接收时刻，不是事件发生时刻，`run_context.json` 明确保留此解释。结构化 `terminal.json`/分析数据对本次 hard-stop 的可证时间界限使用 PASSIVE 发布开始至 hard-stop 完成确认的区间。
- `scripts/test_s2_03_stop_semantics.py` 覆盖有效的 post-transition 确认、早于请求/仅有转换前日志/普通 FSM 文本不得冒充确认，以及终局优先级。Python 编译通过，定向检查 `2 passed`。控制器增量构建通过：`colcon build --packages-select rl_quadruped_controller --symlink-install`。

## 3. 一次短诊断

运行 ID：`ac319e1d38984ed996a6934f7d1ffcf5`。配置来源和模型路径见该 run 的 `run_context.json`、`ros_launch.log`：`scene_obstacle.xml`、当前 go2_description overlay、A / `paper_faithful_switch`、entry/exit `-0.05`、无滞回/保持、目标 `(7,0) m`、到达半径 `0.5 m`、运动仿真上限 `6 s`、墙钟保护 `60 s`。Agile、RA、Recovery 模型均从记录的当前 overlay 绝对路径加载；唯一 controller/writer、session 与运行来源匹配通过。

桌面预检使用 `DISPLAY=:0` 和 `/run/user/1000/gdm/Xauthority`。X11、16×16 GLFW context 和窗口创建均通过；renderer 为 `Intel / Mesa Intel(R) Graphics (RPL-P) / OpenGL 4.6 Compatibility Profile, Mesa 23.2.1-1ubuntu3.1~22.04.4`。没有改系统显示配置或授权。

**运动终局与指标：**终止原因为 `ARRIVED`，终止策略周期 247、仿真时间 `30.686 s`，距 `(7,0)` 为 `0.4996366 m`，按诊断半径到达。运动窗口为 cycle 52 至 247（`26.786–30.686 s`），终止帧载荷完整且分析器核对了终止周期边界。结果表判定 VALID：208 帧、策略周期无缺号、4 个无效时钟帧、总体策略频率 `50.002 Hz`、运动窗口内 12 次模式转换（6 次 Agile→Recovery、6 次 Recovery→Agile）。4 个无效时钟帧保留为无效；总体策略频率按全采集 LIVE 帧的单调时间间隔统计，可包含仿真时钟无效的帧；模式转换和运动指标限于 cycle 52 至终止 cycle 247，仿真时长及驻留指标只用连续且仿真时钟有效的周期，缺口按分析器规则保留。此次诊断没有 cleanup 策略帧进入记录。这些只是单次短诊断统计，不说明切换性能。

本运动窗口分析没有发现机器人—障碍碰撞 episode 或确认跌倒，且该窗口的安全来源覆盖完整；终止接触列表为空。逐帧接触统计仍观察到地面接触（最多 4 个足—地面、1 个非足—地面），不应将其误报为障碍碰撞。本次没有安全 veto。以上仅描述此 run 的覆盖区间，不扩大为全场景、所有物理步或无碰撞保证。

**停止与收尾：**停止发布器在 RL 前 READY，确认有 `/control_input` 订阅者。终止策略帧单调时间 `21861470710728 ns`；停止发布器的 publish 开始时间 `21861473682312 ns`，即终局后 `2.972 ms`。publish 调用返回时间 `21861473795408 ns`。同一 launch 日志中 controller hard-stop 完成 PASSIVE 的 `steady_ns=21861474096468`，比 publish 开始晚 `0.414 ms`；该日志来自本次唯一 controller manager（PID 与 session 绑定记录见 `run_context.json`）。这证明请求到达控制器并完成 PASSIVE 转换。其后的有限收尾观察窗口约 `5.82 ms`，未采到新的策略帧；不能据此报告一个实际“输出持续时间”，也不能证明物理运动为零或 MuJoCo 已消费命令。

## 4. 结果和证据位置

- 本次原始记录：`diagnostic_run_20261010/ac319e1d38984ed996a6934f7d1ffcf5/{runtime_record.jsonl,terminal.json,run_context.json,process_facts.json,ros_launch.log,mujoco.log}`。
- 单次结果表和时间线：`diagnostic_run_20261010/analysis/run_results.md`、`analysis/run_results.json`、run 目录下 `metrics.json`、`timeline.png`。
- 显示/构建/进程预检：`diagnostic_run_20261010/preflight.json`。
- 原两次手动运行的接触数据保持原位于 `logs/manual_single_run_01/`，没有重写或重分类。

## 5. Remaining UNKNOWN 与边界

- 两个历史几何接触缺少 contact distance/force，接触是否包含 margin 影响、实际穿透或冲击大小 UNKNOWN。
- 当前停止确认是控制器 FSM 转入 PASSIVE 的直接证据；MuJoCo 对最终写出命令的消费、实际输出停止到物理运动停止的间隔仍 UNKNOWN。
- 本次收尾窗口没有采到后续策略帧；窗口以外是否仍有策略帧无法由本次采样证明。固定等待常量不再被当作输出持续时间。
- 本次存在 4 个无效时钟帧；它们保留在原始记录和统计说明中，不据此推断这些周期对应的仿真时间。
- 两次旧运行终局位置均未进入 0.5 m 到达范围；第二次终局后的继续运动是清理尾段，不能算到达。此次短诊断恰在半径边界内到达，但不是正式性能结论。
- 通用 terminal 行的 `reached_goal` 字段仍为 UNKNOWN，因为原始 runtime frame 不含该字段；本 run 的监督器结果和 `arrival_observed` 通过同周期位姿、目标和距离给出 ARRIVED。两者含义不同，不能混写。

本次使用的 ROS launch、controller manager、MuJoCo 与停止发布器均报告正常退出；以记录 PID 检查时进程不存在，相关项目进程扫描为空。RViz 未启动。临时 GLFW 探针已退出。
