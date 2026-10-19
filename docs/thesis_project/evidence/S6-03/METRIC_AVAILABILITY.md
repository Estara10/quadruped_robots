# S6-03 指标可用性与统计口径

**审阅状态：S6-03 COMPLETE（Director 有条件通过，2026-10-12）。** 接受范围为准备准入、配置实际消费、cycle-0 与初态关联、任务窗口记录、失败终局分析和清理。工具准备基本完成，但目前不具备启动参数比较的条件。A/D 均在进入 RL 约 0.46 秒后 `FALL_TERMINATION`，未接近障碍且未触发 Recovery；结果不能归因于退出阈值或保持时间，也不能用于算法优劣判断。周期缺口、无效时钟、初态差异、Recovery 与到达后站立收尾未覆盖、PASSIVE 不等于 MuJoCo 已消费等限制保留。历史记录和原始数据未改写。

本表定义后续候选参数比较的共同统计边界。最新 A/D 补证（A run `[local run ID omitted]`、D run `[local run ID omitted]`）均在首周期时钟有效、cycle-0 完整帧与 companion 精确关联、任务安全边界可核对的条件下完成；均为 `VALID / FALL_TERMINATION`，均无 Recovery 且未到达，因此不用于比较参数性能。两次起立门槛及连续物理稳定检查通过；任务窗口确认跌倒。先前 A `PREPARATION_FAILURE`、`INVALID/no_frame_lines`、首批基础设施 INVALID 和 D 未启动记录均作为历史保留，不改写、不合并。

## 本轮证据状态

- A、D 独立 overlay 均通过生产数值校验；两次 controller 均直接记录各自 group/candidate/E/X/H、目标、模型路径及 session，实际 controller/writer 唯一绑定。controller PID 的映射检查指向同一次构建并安装的库。
- 两次 recorder 均在 command 3 前回读确认可读写；实际 `[STAND-READY] percent=1.504167` 证明 FIXEDSTAND 插值已跨过 command 3 的生产门槛，随后物理稳定窗口通过。具名 q/dq 采样有效；真实任务前 safety boundary 为 LIVE。
- 两次 `extract_first_active_policy_state()` 对 `[ABS-INITIAL-STATE]` companion 和 runtime JSONL 均返回 `CAPTURED`：cycle 0 完整帧已实际持久化，session/cycle/monotonic_ns 完全匹配，sim clock 有效。两组周期0实测 q/dq 有小幅差异，不能称为严格初态重放。
- 起点边界快照都保留了从 sim 0.250 s 开始、0.550 s 确认的早期跌倒 episode；A 在 task start 前 sim 21.854 s 关闭，D 在 task start 前 sim 23.876 s 关闭。两次任务窗口内的确认跌倒是另一个新 episode，不能被准备期历史掩盖，也不把已关闭的准备事件错计入运动期。
- A 与 D 的 supervisor 和 analyzer 均为 `FALL_TERMINATION / VALID`；任务窗口跌倒=true、声明运动窗口的碰撞覆盖完整且未观察到非足端碰撞。A 有2个无效时钟帧和3个策略步缺口；D 无无效时钟帧但有2个策略步缺口。两组均无 Recovery，站立收尾目标（ARRIVED后5秒）不适用；command 1 后 controller PASSIVE 转换有确认。终止 payload 完整保存。
- 先前 A 的 PREPARATION_FAILURE 与 analyzer `INVALID/no_frame_lines`、未启动 D 状态仍保留为各自历史批次。旧失效记录不能因本轮新运行改判。
- 后续单次 prepare-only run `[local run ID omitted]` 只验证准备阶段：episode 于 sim 26.478 s 关闭，稳定门槛 26.716–27.222 s 通过。不得把它计作任务运行、运动安全结论或 A 配置周期证据；其 terminal JSON 没有策略帧，任务窗指标仍不适用。

## 主窗口

- 起点：必须是已持久化的 `rl_step=0`、`rl_entered=1 && rl_active=1` 策略周期，并与 companion 的同 session/cycle/monotonic 时间完全匹配。保留 sequence 和 `sim_clock_valid/sim_time`。周期0时钟无效或缺失时起点身份仍保留，但时间型统计为 UNKNOWN；不能用较晚的有效周期替换。若周期0帧或完整 payload 未保存，首态/准备完整性为 UNKNOWN，批次不得当作有效比较运行。
- 终点：发生真实终局的同一策略周期，terminal payload 完整落盘后封口，终止周期包含在窗口内。
- 启动准备、到达后站立观察及清理尾段独立记录，不进入任务指标。若碰撞/跌倒 episode 从任务开始前延续进窗口，累计历史按 episode 起止边界保留并判定是否相交。
- 首个位移达到 0.10 m 的旧起点仍写入 `legacy_displacement_window_metrics`；只作辅助，不能覆盖任务起点统计。

## 指标清单

| 指标 | 工具状态 | 定义与数据来源 | 覆盖缺口与后续比较处理 |
|---|---|---|---|
| 任务终局 | 口径可用，本批2个有效合法失败 | 监督器对 ACTIVE 策略周期读取安全快照、目标距离和策略状态；安全优先级为 safety abort、非足端碰撞、确认跌倒、到达、仿真超时。A/D 均为 VALID / FALL_TERMINATION。 | 无效来源/记录单列 INVALID，不计为算法失败或成功；原因、数量仍须报告。合法跌倒保留为有效任务结果。 |
| 终点距目标 | 部分可用 | 终止策略帧世界 XY 到目标的欧氏距离。 | 缺终止帧/位姿或时钟窗口身份不完整时 UNKNOWN；ARRIVED 只指不超过 0.5 m 判据。 |
| 起点至终局经过时间 | 部分可用 | 首个 active 周期至 terminal 周期的 `sim_time` 差；边界时间均有效且顺序正确时报告。 | 首/末时钟无效、重置或终止载荷不完整则 UNKNOWN；不使用墙钟或延后起点替代。 |
| 有效连续时间 | 部分可用 | 仅对同 session、策略周期连续、两端 sim clock 有效且时间单调的相邻帧累加 `dt`。 | 不桥接无效帧、周期缺口、session 变化；同时报告有效累计值和覆盖比例 `有效累计 / 完整经过`。 |
| 路径长度、平均/峰值速度 | 部分可用 | 同一有效连续相邻策略帧的世界 XY 位移求和；均速=`有效路径/有效连续时间`；峰值为最大有效区间位移速度。 | 仅代表有效连续片段，不冒充全程路径/速度；覆盖不足时附带缺口，若无有效间隔则 UNKNOWN。 |
| 全部模式切换 | 可用作离散计数 | 主窗口内每条有效记录的 `mode_before != mode_after` 边沿按 session/cycle 计数。 | 时钟无效不删除边沿；报告总边沿数与有效时间边沿数。策略周期缺口仍报告，不推断缺失边沿。 |
| 短反向切换（0.5 s） | 部分可用 | 主窗口内时间相邻的反向边沿对，其有效 sim-time 间隔在 `[0,0.5] s`；统计对象、窗口和方向需随结果输出。 | 缺周期或无效时钟时只报告已观察计数，coverage 标为 partial/unknown；不能称为完整统计或补插时间。 |
| Agile / Recovery 周期与驻留 | 周期数可用，时间部分可用 | 活跃主窗口按周期身份统计模式周期；Recovery 时间只累计有效、连续且起点模式为 Recovery 的区间；占比=`该有效 Recovery 时间 / 主窗口全部有效连续时间`。 | 区间跨无效时钟/缺周期/终局截断则删失，审计整体 incomplete；不能用最后一个 Recovery 帧闭合缺失区间。 |
| 风险到 Recovery 模式决定延迟 | 内部决定时间可用但含义有限 | 仅风险首次由假变真时仍在 Agile，且与同策略周期 Agile→Recovery 转换可配对的事件，计算单调时钟决策时间差。 | 已在 Recovery 时再次出现风险边沿只计风险事件，不算新进入响应。该 ns 值不是完整避障响应、命令写出、动作生效或 MuJoCo 消费延迟。 |
| 足端接触/撞击、非足端碰撞 | 部分可用，依赖 physics authority v5 | 生产 PhysicsLoop 累计的足端 contact/impact 事件和非足端机器人—障碍 episode；仅四个具名足端球作为足端。 | 需要 task-start 前边界快照到 terminal 的有效连续发布、历史无溢出、接触来源无 UNKNOWN。覆盖不足则 UNKNOWN，不从未见事件推断无碰撞。 |
| 跌倒 | 部分可用，依赖累计事件和窗口交叠 | 当前诊断判据的已确认跌倒 episode 与主窗口相交时 true；完整 v5 覆盖、无 invalid posture overlap/未确认候选时才可报告该窗口 false。 | 未确认姿态、历史缺口、缺 task-start 边界或 episode 跨窗口未能判定时 UNKNOWN。正式跌倒阈值尚未冻结。 |
| 安全中止 | 可用作运行事件，物理消费 UNKNOWN | 记录安全 veto、终局原因、stop publisher 写出、controller 状态转换和清理事实。 | 明确报告确认层级。publisher 返回或 controller PASSIVE 均不代表 MuJoCo 已消费命令。 |
| 超时 | 部分可用 | 仿真超时由首个 active 策略周期的有效 sim_time 到达 20 s 上限；墙钟 120 s 单独作为诊断监督上限。 | 起点时间无效时 sim-time timeout 为 UNKNOWN/无法判断；墙钟 guard 记为不完整，不补成仿真时长。 |
| 时钟/策略周期质量 | 可用作质量描述 | 全部 JSONL 帧墙钟周期、policy period/frequency；主窗口单独报告 sim clock invalid 帧、周期缺口和连续有效区间。 | 全采集频率不能代替主窗口时间覆盖。单次测量不能归因或排除记录开销。 |
| 初态 | 部分可用；本批 cycle-0 实测记录已可用 | scene reset probe、固定站立后具名 `/joint_states`、首个 active 周期 companion 的实测姿态/速度分别记录时间身份；最新 A/D 的 cycle-0 q/dq 和 companion 已与 runtime payload 精确配对。 | 构造态、异步 ROS 样本和策略周期状态不等价；A/D 首态存在实测差异，不是严格初态配对。`framework-native` 四元数顺序/参考系未由直接来源证明，不用于相似度判断。 |

## 未来小样本汇总规则

对每组分别列 planned、started、VALID、INVALID、not started、终局类别、正常站立收尾、PASSIVE 回退及 UNKNOWN。到达率分母为该组所有数据有效的任务运行数，同时列出 INVALID 数和原因。另定义“任务安全成功”为 ARRIVED 且窗口无终止碰撞、无确认跌倒；UNKNOWN 不当作安全成功。合法碰撞、跌倒、超时和安全中止都保留为有效任务结果，不能只汇总成功运行。未到达时记录任务耗时/终止类别，不把20秒上限冒充到达时间。少量运行只逐次描述，不估计稳定成功率、排名或最优参数。
