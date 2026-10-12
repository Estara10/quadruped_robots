# S6-01 四组运行结果、来源与初态

本文件将 2026-10-12 首批 A/B 与本次 C/D 补证合并作**流程核验**。两批运行不构成严格配对：初态没有完全相同，也没有进行重放。每组只有一次有效任务运行，不能据此估算成功率、排名或参数优劣。本任务中的 C 首次启动失败记录仍保留，见“历史 INVALID”。

固定设置：`scene_random_04.xml` / `S2-08-r1`；目标 `(7,0)`，到达半径 `0.5 m`；运动上限 `20 s` 仿真时间、`120 s` 墙钟；到达后站立观察目标 `5 s` 仿真时间。控制器库及模型/动作链保持相同；C/D 使用新建的隔离 overlay。

## 四组流程核验结果

| 组 | 实际配置；run/session/writer | 数据与任务终局 | 位移起点至终局：经过时间；有效连续时间；有效片段路径/平均/峰值速度 | 模式与 RA | 站立/停止收尾；运动期安全 |
|---|---|---|---|---|---|
| A | A；`[local run ID omitted]` / session `1719084890764` / writer-controller PID `15116` | `VALID / ARRIVED`；终距 `0.4808 m`；终止 payload 完整；834 帧；0 周期缺口、4 无效时钟帧 | `15.860 s`；`15.682 s`；`8.515 m` / `0.543 m/s` / `2.406 m/s` | 全部 `322`（A→R 161、R→A 161）；有有效 sim_time 的 `320`；Agile/Recovery `495/300` 周期；Recovery `6.164 s`，占有效时间 `39.31%`；RA `[-0.9989,0.3715]` | 5 s 站立完成；未请求 PASSIVE / 未发生 command 9；运动期外部安全事件 0。检测到 RL 足端接触与撞击各 1 次。跌倒 `UNKNOWN`（有未确认姿态候选）；无碰撞结论覆盖不足。 |
| B | B；`[local run ID omitted]` / session `1776891406391` / writer-controller PID `20618` | `VALID / ARRIVED`；终距 `0.4910 m`；终止 payload 完整；531 帧；0 缺口、0 无效时钟帧 | `9.980 s`；`9.980 s`；`7.947 m` / `0.796 m/s` / `2.749 m/s` | 全部 `102`（各向 51）；有有效 sim_time `102`；Agile/Recovery `341/159`；Recovery `3.268 s`，占有效时间 `32.75%`；RA `[-0.9991,0.3397]` | 站立观察未完成；正常 command 6 未确认，观察到 `STANDING_HOLD` 阶段物理 authority 变为 MISSING，之前站立确认已到达、约 `2.766 sim s` 观察，但 hold 条件未完成；之后 command 9 回退并由 controller 确认 PASSIVE。运动期外部安全事件 0。足端接触/撞击各 1 次。跌倒否定未由正式参数验证；无碰撞结论覆盖不足。 |
| C | C：`E=-0.05, H=0.30 s`，无滞回；run `[local run ID omitted]` / session `9279184632536` / writer-controller PID `38328` | `VALID / ARRIVED`；终距 `0.4937 m`；终止 payload 完整；634 帧；0 周期缺口、1 无效时钟帧 | `12.000 s`；`11.960 s`；`9.003 m` / `0.753 m/s` / `3.104 m/s` | 全部 `24`（各向 12）；有效 sim_time `24`；Agile/Recovery `409/192`；Recovery `3.830 s`，占有效时间 `32.02%`；RA `[-0.9989,0.2475]`。保持审计 12/12 episodes 时钟完整、规则 PASS。风险进入→配对 A→R `12/17`，配对内部决定延迟中位 `670 ns`；其余 5 次为已在 Recovery 时风险重入。 | 5 s 站立完成；无 PASSIVE / 无 command 9；运动期外部安全事件 0。未记录足端接触/撞击。存在未确认跌倒候选，故跌倒 `UNKNOWN`；无碰撞否定覆盖不足。 |
| D | D：`E=-0.05, X=-0.08, H=0.30 s`；run `[local run ID omitted]` / session `9331242659334` / writer-controller PID `42662` | `VALID / ARRIVED`；终距 `0.4996 m`；终止 payload 完整；585 帧；0 周期缺口、5 无效时钟帧 | `11.018 s`；`10.816 s`；`8.232 m` / `0.761 m/s` / `3.473 m/s` | 全部 `22`（各向 11）；有有效 sim_time 的 `22`；Agile/Recovery `381/171`；Recovery `3.354 s`，占有效时间 `31.01%`；RA `[-0.9989,0.2050]`。保持审计 `9/11` 时钟覆盖完整，另 2 个 episode 不完整，整体 `incomplete`，未发现违规不等于所有周期已核验。风险进入→配对 A→R `11/18`，配对内部决定延迟中位 `475 ns`；另 7 次在 Recovery 时重入。 | 站立观察目标 5 s，physics authority 变为 MISSING 前只观察 `2.092 s`，未完成；正常 command 6 未确认，command 9 回退后 controller 明确确认 PASSIVE（发布开始至确认约 `4.795 ms`）。运动期外部安全事件 0；收尾 hard-stop 是终局后的停止措施。未记录足端接触/撞击。当前诊断规则下跌倒为 false，但无碰撞否定覆盖不足。原始清理事件中的 controller-state 字段误写 `FIXEDSTAND`；直接 controller 日志确认当时已转 `PASSIVE`，见 Remaining notes。 |

“经过时间”以首次位移达到 `0.10 m` 的历史起点到终止策略周期为界，含该区间完整经过时长；“有效连续时间”、路径和速度只用相邻、同 session、周期连续且仿真时钟有效的间隔。站立、清理阶段排除。模式边沿数按周期身份统计，即使该周期时间无效也保留；精确时间量只计有效时钟覆盖。策略频率来自全采集策略帧墙钟间隔：A `50.0035 Hz`，B `49.9965 Hz`，C `50.0004 Hz`，D `50.0144 Hz`；单次数据不能归因或排除记录开销。

## RL/任务起点与历史位移起点

历史统计以“相对首个有效策略位姿位移达到 `0.10 m`”为起点，因此会排除 RL 已活跃后的准备运动周期。新增分析同时报告首个持久化 `rl_entered=1 && rl_active=1` 周期到终局的边界，作为更适合正式方案讨论的候选起点；它代表策略活跃帧，不等于 MuJoCo 命令消费时刻。

| 组 | 首个策略活跃周期 → 终止周期 | RL 活跃至终局经过时间 / 有效连续时间 | 有效连续路径 / 平均速度 | RL 活跃后被旧位移边界排除的周期数 |
|---|---:|---:|---:|---:|
| A | 10 → 843 | `16.660 / 16.482 s` | `8.729 m / 0.530 m/s` | 39 |
| B | 14 → 544 | `10.600 / 10.600 s` | `8.085 m / 0.763 m/s` | 31 |
| C | 13 → 646 | `12.660 / 12.620 s` | `9.158 m / 0.726 m/s` | 33 |
| D | 13 → 597 | `11.678 / 11.476 s` | `8.390 m / 0.731 m/s` | 33 |

新旧起点只是同一原始记录的两种明确统计边界：历史位移统计保留，不被新口径静默重定义。正式实验建议在方案冻结前采用首个有效策略活跃周期作为任务指标起点，并把 `command 3` 调用区间单独记录；如要定义“运动开始”，另列位移阈值指标。风险条件到模式决定的纳秒级 monotonic 差只代表内部决策路径耗时，不代表完整避障响应、命令写出、动作生效或 MuJoCo 消费延迟。

## 初态与关节实测

详细逐关节数据及时间戳见[初态与来源记录](initial_state_and_sources.md)。A/B 首批运动起点关节采样失败，保留历史原因，未补造值。C/D 均在 PASSIVE/FIXEDSTAND、RL 前通过 `/usr/bin/python3` 和 ROS Humble `rclpy` 取得 12 个具名关节位置/速度样本；样本与 RL 活跃帧相隔约 `1.935 s`（C）及 `1.605 s`（D），不冒充同周期。运动位移锚点后的异步 joint sample 分别约晚 `0.784 s`（C）、`0.953 s`（D），也不声称周期同步。C/D 实测初态不同，不是严格重放。

## ROS graph 来源核对

- 旧 C `[local run ID omitted]` 保留为 `INVALID / STARTUP_FAILURE`。当时直接查询空、daemon 与 controller list 显示 active；原始日志未记录各查询的 ROS 环境、PID/RMW 和 monotonic 时间，不能回填根因。
- 本次 C 的 daemon 与 fresh direct 查询各见一个 manager/controller；D 的 daemon 缓存各列出两个同名 manager/controller，而同一 overlay/domain 下的 fresh direct query 各见一个。两组 `ros2 control list_controllers` 均只显示一个 active controller，launch 进程组各有一个 manager/writer，writer PID 与策略 session/config 来源相符。因此本次 D 的重复名称来自 daemon 缓存视图，未发现第二个活动 controller/writer；缓存本身不作为放行依据。
- 每组 fresh direct query 有界运行一次，发现唯一节点后即放行。C direct query 用时约 `2.815 s`，D 约 `2.769 s`；查询状态、单调起止时间、实际进程 PID/UID/executable/cmdline、ROS 环境变量、daemon 进程身份和节点计数保存在对应 C run_context（原始运行/环境明细保留本地，未纳入分享提交） 与 D run_context（原始运行/环境明细保留本地，未纳入分享提交）。`ROS_DOMAIN_ID` 未设置（默认 0），RMW 覆盖变量未设置；`ros2` CLI 的实际查询 PID 是 `/usr/bin/python3.10`，shell 中用于打印环境的 Python 只是 Anaconda 元数据解释器。ROS daemon 命令行和 controller 映射确认 `rmw_fastrtps_cpp`；额外 system-Python ROS RMW probe 也选中该实现，但 sandbox 禁止其 UDP transport，因此该 probe 仅核对选择，不作为节点发现证据。
- 新记录有界重查将直接空结果与查询失败分开，并立即拒绝重复节点；daemon 只作诊断记录。新证据说明旧 C 可能与发现时序有关，但旧日志不足以断言旧故障的唯一根因。

## 历史 INVALID

首批 C 的零帧 `INVALID / STARTUP_FAILURE` 和当时的直接/daemon 不一致原样保留；不能改判有效或替换 raw 证据。第二批 C 是一次独立、有效诊断，不覆盖第一条历史记录。D 仅在本次新批次运行一次。

## 周期、截图和原始证据

- 全部策略周期及模式时间线分别见 A/B 原始 run 的 `runtime_record.jsonl`、`timeline.png` 和更新后的 `metrics.json`；C/D 见本批对应 run 子目录及 `analysis/run_results.json`。
- 本次真实 MuJoCo framebuffer LIVE 图： [C LIVE](screenshots/C_live_[local run ID omitted].png)、[D LIVE](screenshots/D_live_[local run ID omitted].png)。源 PPM 原始画面保存在 run 子目录；PNG 仅由原始 framebuffer 无损转换。C 图显示 Group C、当前 Agile、RA 与 E、保持未计时；D 图显示 Group D、当前 Agile、RA 与 E/X、保持已计时约 1 ms。
- 所有原始日志、`run_context.json`、策略帧、terminal 与 `process_facts.json` 保存在 `logs/manual_abcd_compare/` 对应批次；原始数据未改写。指标与时间线由现行 analyzer 从原始策略帧重算。

## 收尾及限制

| 组 | 任务结果 | 正常站立收尾 | 运动期急停/安全中止 | 终局后 hard-stop | 数据有效 |
|---|---|---|---|---|---|
| A | ARRIVED | 完成 | 否 | 否 | 是 |
| B | ARRIVED | 未完成 | 否 | 是，command 9 后 controller PASSIVE 确认 | 是 |
| C | ARRIVED | 完成 | 否 | 否 | 是 |
| D | ARRIVED | 未完成 | 否 | 是，command 9 后 controller PASSIVE 确认 | 是 |

到达只表示终距小于 `0.5 m`。C/D 未记录足端接触/撞击；这不能证明碰撞覆盖完整。C 跌倒为 UNKNOWN（候选姿态跨运动窗口）；D 在当前诊断判据下未确认跌倒，但不代表正式跌倒参数已验证。D 的策略记录里 5 个运动周期的时钟无效；模式边沿仍保留，短反向和保持时序覆盖因此受限，2/11 个 D hold episode 不完整。风险到模式决定延迟是内部时长。command 9/controller PASSIVE 不是 MuJoCo 消费确认。

本次 C/D 均 `closeout_complete=true`，退出码 0，清理事件记录 MuJoCo 先退出、之后 controller launch 与 stop publisher 退出；运行后 PID/comm 核对未发现 MuJoCo、controller、RViz、采集器或本任务辅助进程残留。**D 原始清理事件的 `controller_state_before_stop` 因旧硬编码记作 `FIXEDSTAND`，与同一 run 的 controller hard-stop 日志确认 `PASSIVE` 冲突。原始记录保留；以 controller 日志判定状态。清理记录工具现已改为读取实际停止证据，未重跑场景。**
