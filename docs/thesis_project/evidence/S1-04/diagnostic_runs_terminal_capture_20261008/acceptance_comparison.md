# S1-04 终止帧验收对照（2026-10-08）

两次请求配置相同：`scene_obstacle.xml`、`paper_faithful_switch`、世界目标 `(1.0, 0.0) m`、到达半径 `0.50 m`、entry/exit `−0.05`、同一 overlay 与 Agile/RA/Recovery 模型，仿真运动上限 `6 s`、墙钟保护 `50 s`。目标“到达”需由有效策略帧直接满足 0.5 m 阈值。

| Run | 来源/终局 | 终止帧和数据 | 可用结果 | S1-04 验收 |
|---|---|---|---|---|
| `cb47356b98104e40b258b81a8246db17` | `VALID / ARRIVED`；writer/controller PID `189998`，session `20302804686619`；终止周期 74，仿真时间 24.524 s，距离 `0.4843 m`。 | 终止周期已持久化；75 帧，周期缺口 0、1 个无效时钟帧。运动从 cycle 49（24.026 s）至 cycle 74，路径长 `0.4292 m`、平均路径速度 `0.9331 m/s`，0 次模式转换、0 Recovery。 | 全采集相邻周期频率 `51.7758 Hz`（中位数 `19.988 ms`，P95 `26.186 ms`）。碰撞快照虽为 LIVE，但物理步有 641 个缺口，不能声称无碰撞。 | 完整参考运行。详见 [原始 run 指标和时间线](../diagnostic_runs_source_matcher_20261008/cb47356b98104e40b258b81a8246db17/metrics.json)。 |
| `7089073a2a684328b6523efa8c06c384` | source binding `confirmed`；writer/controller PID `203918`，session `22412744739286`。运行结果未持久化，**不能标作 ARRIVED**。 | runtime record 保存 38 个 LIVE 帧，session 单一、cycle 38–75 连续、38 帧时钟有效。最后 payload 是 cycle 75、sequence 152，RA `−0.9972`，Agile，action source 1，sim time `23.812 s`，距目标 `0.499772 m`。但 `terminal_frame_ref`、唯一 terminal line、`terminal.json` 均缺失。 | 原始周期描述性统计：37 对、约 `50.3865 Hz`（中位数 `20.102 ms`、P95 `27.753 ms`、min/max `6.982/28.653 ms`）；因 run record invalid，不作为验收频率或运动指标。运动起点、终局边界、路径和速度为 UNKNOWN。38 个 LIVE 碰撞采样有 325 个物理步缺口。 | **不完整，不能充当第二个完整 run。** 收尾写 context 时因访问不存在的 `RunRecordRecorder.error` 抛出 AttributeError，finalize 未运行。原始文件保留；没有第二次补跑。 |

本轮实际 run 配置和来源见 [run_context.json](7089073a2a684328b6523efa8c06c384/run_context.json)，写入/收尾故障见 [postrun_failure.json](7089073a2a684328b6523efa8c06c384/postrun_failure.json)，结构校验结果见 [metrics.json](7089073a2a684328b6523efa8c06c384/metrics.json)，说明性图表见 [timeline.png](7089073a2a684328b6523efa8c06c384/timeline.png)。保存的 ROS 日志未发现 `ABS-CONTRACT` veto 或 `EMERGENCY` 事件；`[HARD-STOP] command=1` 是终局后的停止请求日志，不是 MuJoCo 消费确认。模型动作关节顺序、碰撞完整覆盖、跌倒检测和策略停止确认继续 UNKNOWN。

## 完整收尾验证后的验收对照（2026-10-08）

本节保留上面的历史 INVALID 结果，不补造或更改其终局。完整收尾生产路径新增临时目录端到端检查通过后，唯一获准的新场景运行 `7172581a85b6411db9c1122043ce279b` 与原完整运行 `cb47356b98104e40b258b81a8246db17` 组成两次完整、同配置诊断对照：

| Run | 来源、配置和终局 | 终止周期与数据完整性 | 可计算运动结果 |
|---|---|---|---|
| `cb47356b98104e40b258b81a8246db17` | `VALID / ARRIVED`；paper，1.0 m 目标，0.5 m 判据，`entry=exit=-0.05`；session `20302804686619`。 | cycle 74，`sim_time=24.524 s`，距离 `0.484334 m`；75 帧，0 缺周期、1 个无效时钟帧，terminal payload 完整。 | 运动 cycle 49–74，0.460 s；0 次转换、0 Recovery；路径 `0.429225 m`，平均路径速度 `0.93310 m/s`；全采集频率 `51.7758 Hz`。 |
| `7172581a85b6411db9c1122043ce279b` | `VALID / ARRIVED`；唯一 controller/writer PID `212324`，session `23546368334100`；paper，1.0 m 目标，0.5 m 判据，运行帧 `entry=exit=-0.0500000007`。 | cycle 74 / sequence 150，`sim_time=26.488 s`，距离 `0.498959 m`；38 帧，0 缺周期、0 个无效时钟帧；terminal_ref 唯一匹配同 session/cycle/sequence/monotonic 的完整 payload，runtime record 恰有一条 terminal line，`terminal.json`、run context 和 process facts 均保存。 | 运动 cycle 48–74，0.520 s；0 次转换、0 Recovery；路径 `0.471739 m`，平均路径速度 `0.90719 m/s`；全采集频率 `49.8321 Hz`。 |

两个完整运行使用同一 `scene_obstacle.xml`、`/tmp/s1_04_overlay` 三个模型路径、paper 候选、1.0 m 目标、0.5 m 到达半径、`−0.05` 进入/退出阈值及 6 s 仿真 / 50 s 墙钟上限。到达仅指策略终止帧测得进入 0.5 m 范围；不表示没有碰撞。两个运行都没有运动期 veto，但碰撞快照物理步有缺口（第一次 641、第二次 326），仍不能声称无碰撞。两次 cleanup 都有 PASSIVE/hard-stop 请求，FSM 停止确认未观察到；MuJoCo 命令消费仍 UNKNOWN。

本次 system GLFW 预检直接记录 renderer `Mesa Intel(R) Graphics (RPL-P)` / Intel，OpenGL 4.6。原完整运行未记录 renderer，故不推断 renderer 相同。详细收尾和运行产物：[新 run context](7172581a85b6411db9c1122043ce279b/run_context.json)、[process facts](7172581a85b6411db9c1122043ce279b/process_facts.json)、[terminal.json](7172581a85b6411db9c1122043ce279b/terminal.json)、[指标](7172581a85b6411db9c1122043ce279b/metrics.json)、[结果表](7172581a85b6411db9c1122043ce279b/run_results.md)、[时间线](7172581a85b6411db9c1122043ce279b/timeline.png)；环境探针见 [retry2_preflight.json](retry2_preflight.json)。
