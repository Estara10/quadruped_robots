# A/B/C/D 策略切换合同（S1-01 已通过）

> 来源：S1-01 步骤 1–5。合同已通过 Director 最终审阅；S1-01 为 COMPLETE。运行时实现和基础仿真核验属于后续任务。

## 1. 符号和风险方向

- `RA` 越大表示风险越高。
- `E` 是四组共用的进入阈值。
- `X` 是 B/D 共用的退出阈值，必须满足 `X < E`。A/C 不使用独立退出阈值，其退出阈值就是 `E`。
- `H` 表示 C/D 共用的 `recovery_min_hold_s`，单位为秒；其配置和计时语义见第 6 节。
- 本表假定 RA 有效且有限。无效 RA 不得驱动组间转换；运行时已有 RA 输入/输出安全故障路径，故障终止当前策略路径，不作为 RA 切换。

## 2. 转移表

| 当前模式 | 组别/条件 | 下一模式 | 本周期动作来源 | 计时处理与原因 |
|---|---|---|---|---|
| Agile | 任一组，`RA < E` | Agile | Agile | 无保持计时；未满足进入条件 |
| Agile | 任一组，`RA >= E` | Recovery | Recovery | C/D 从本次进入启动 `H`；A/B 不启动保持；原因 `risk_enter`。进入周期即使用 Recovery 动作 |
| Recovery | A，`RA >= E` | Recovery | Recovery | 无保持；风险仍达到单阈值 |
| Recovery | A，`RA < E` | Agile | Agile | 无保持；原因 `risk_exit_single`。退出周期即使用 Agile 动作 |
| Recovery | B，`RA >= X` | Recovery | Recovery | 无保持；未满足退出条件 |
| Recovery | B，`RA < X` | Agile | Agile | 无保持；原因 `risk_exit_hysteresis`。退出周期即使用 Agile 动作 |
| Recovery | C，`H` 未到期（任意有限 RA） | Recovery | Recovery | 保持继续；RA 下降不缩短保持；不重启 `H` |
| Recovery | C，`H` 已到期且 `RA >= E` | Recovery | Recovery | 保持完成；单阈值退出条件未满足；不重启 `H` |
| Recovery | C，`H` 已到期且 `RA < E` | Agile | Agile | 原保持结束；原因 `hold_and_risk_exit_single` |
| Recovery | D，`H` 未到期（任意有限 RA） | Recovery | Recovery | 保持继续；RA 下降不缩短保持；不重启 `H` |
| Recovery | D，`H` 已到期且 `RA >= X` | Recovery | Recovery | 保持完成；滞回退出条件未满足；不重启 `H` |
| Recovery | D，`H` 已到期且 `RA < X` | Agile | Agile | 原保持结束；原因 `hold_and_risk_exit_hysteresis` |

### 边界值

- `RA == E`：Agile 状态转入 Recovery；A/C 在 Recovery 状态仍留在 Recovery。
- B/D 在 Recovery 状态下，`RA == X` 留在 Recovery；只有 `RA < X` 才满足退出条件。
- `RA < E` 包含低于阈值的所有有限值；在 Agile 状态不触发切换。
- 保持到期恰好发生在本周期时，本周期按“已到期”处理，立即检查对应退出条件；条件满足则本周期返回 Agile 动作，否则继续 Recovery。计时到期的时钟与比较细节由步骤 3 冻结。

## 3. 生命周期和周期规则

- 新 run 或重新进入 RL 会话时，初始模式为 Agile；Recovery 保持状态和缓存的 Recovery twist 清零。每个 run 的组别及阈值固定，不热更新。
- 首个有效 RA 到来前不进行 RA 驱动的模式转换，也不将该周期记作风险进入。若 RA 输入/输出无效，沿用当前运行时安全故障路径结束策略输出；不假定 Agile 或 Recovery 动作有效。
- 首个有效 RA 若满足 `RA >= E`，在该策略周期进入 Recovery，并在同周期选择 Recovery 动作。
- 只在 Agile→Recovery 的状态边沿启动 C/D 的 `H`。Recovery 中 RA 再次满足进入条件不构成新进入边沿，不重新起算或延长保持。
- Recovery→Agile 的周期立即选择 Agile 动作；Agile→Recovery 的周期立即选择 Recovery 动作。状态判定先于当周期策略动作选择。
- 到达、超时、碰撞/跌倒终止、操作停止或安全故障等 run 终止事件优先于后续 RA 转换；终止后不再记录策略模式转换。具体终止判定沿后续实验合同定义。
- 会话退出并再次进入视为新 run：回到 Agile，清除保持与恢复缓存状态。终止后不得以旧 RA 或旧计时状态继续切换。

## 4. 四组不变量

- 四组共用 `E` 和进入比较符 `RA >= E`。
- A/C 共用单阈值退出规则 `RA < E`；B/D 共用滞回退出规则 `RA < X` 且 `X < E`。
- A/B 不启用最小保持；C/D 启用完全相同的 `H` 起算点、到期规则和到期周期判定。
- 保持只限制 Recovery 退出，不影响 Agile→Recovery 的进入条件。
- 四组共用 Agile、RA、Recovery 模型及 observation/action、joint 顺序、单位与缩放、限幅、控制输出和 Recovery 命令生成路径。Recovery 优化、twist 缓存策略或控制链不得作为额外组间变量。
- RA 无效处理和实验终止规则四组相同，不由切换组别改变。

## 5. 当前候选实现与合同差异

- `paper_faithful_switch` 使用 `RA >= E` 进入，`RA < E` 退出，无滞回和保持；有限 RA 的比较边界与 A 候选一致，但当前代码没有 A/B/C/D 的独立配置合同。
- `stabilized_switch` 当前使用严格 `RA > E` 进入，Recovery 时无条件递减步数计数，并在计数到期且 `RA < X` 时退出。严格 `>` 与本合同 `>=` 在 `RA == E` 不同；其 `X` 由 `E - 0.03` 推导，且保持按步计，不是本合同当前定义的抽象保持条件。不可直接当作正式 B、C 或 D。
- 当前控制器在进入 Recovery 边沿执行 `computeRecoveryTwist()` 并缓存 twist，随后在 Recovery 动作路径中复用；本合同要求四组共用该命令路径，避免缓存/优化差异成为第三个研究因素。[StateRL.cpp:1634–1667](../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp)
- 当前 `StateRL::enter()` 会将模式、保持计数和缓存 twist 重置，支持上述重新进入规则的源码意图；首个有效 RA 前的安全故障路径及终止边界仍需在后续基础闭环核验。[StateRL.cpp:400–406](../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp)

## 6. 配置与保持时间合同（步骤 3）

### 唯一配置来源

每个 run 的切换配置只从部署配置中的 `abs.switching` 读取。字段如下：

```yaml
abs:
  switching:
    group: "A"
    entry_threshold: -0.05        # E；A exits at E; no hysteresis or hold.
```

这是符合 A 组字段规则的示例。数值仅示意字段格式，**不是冻结的正式参数值**。每个 run 的研究切换配置唯一从 `abs.switching` 读取；A 的合法入口只含 `group: A` 与 `entry_threshold: E`。有效退出阈值由 E 推导为 E，滞回与保持关闭。不得同时配置 `exit_threshold`、`recovery_min_hold_s` 或 A 不允许的任何字段。不得在 `abs` 根下并列填写 legacy `switching_mode`、`ra_threshold`、`recovery_hold_steps`；它们与新配置同时存在时视为冲突并拒绝。

运行时当前只支持 A。解析器明确拒绝已知但未实现的 B/C/D，并拒绝未知组、缺失/非有限/越界阈值、未知字段、互斥字段和 legacy 并列字段；不静默默认或回退。B/C/D 的后续表格定义是合同规范，不表示当前已实现。

| 组别 | 滞回 | 最小保持 | 必填字段 | 推导/禁止字段 |
|---|---|---|---|---|
| A | 关闭 | 关闭 | `group`, `entry_threshold` | 有效退出阈值由 `entry_threshold` 推导；不得填写 `exit_threshold` 或 `recovery_min_hold_s` |
| B | 开启 | 关闭 | `group`, `entry_threshold`, `exit_threshold` | 不得填写 `recovery_min_hold_s` |
| C | 关闭 | 开启 | `group`, `entry_threshold`, `recovery_min_hold_s` | 有效退出阈值由 `entry_threshold` 推导；不得填写 `exit_threshold` |
| D | 开启 | 开启 | `group`, `entry_threshold`, `exit_threshold`, `recovery_min_hold_s` | 无 |

`group` 唯一决定滞回和保持是否启用；不得另设可填写的 `enable_hysteresis`、`enable_hold` 布尔权威。A/C 的有效退出阈值是程序按定义取 `E`，不再由第二个配置值重复表达。未知字段、互斥字段、缺字段或非法字段均视为配置错误，拒绝开始该 run，不静默回退。

### 合法性与跨组一致性

- 合同定义的 `group` 为 `A`、`B`、`C`、`D`，每个 run 只出现一次；当前运行时只接受 A，B/C/D 明确报告 unsupported。
- `entry_threshold` 必须存在、为有限标量，且在 RA 模型 `tanh` 输出范围内：`-1 < E < 1`。
- B/D 的 `exit_threshold` 必须存在、为有限标量、在 `(-1, 1)` 内，并满足 `X < E`。`X >= E` 与“高 RA 更危险、退出阈值低于进入阈值”的滞回方向矛盾，拒绝配置。
- A/C 不接受显式 `exit_threshold`；其退出规则唯一地使用 `E`。这样不会出现隐藏阈值或阈值冲突。
- C/D 的 `recovery_min_hold_s` 必须存在、为有限且严格大于零的秒数。负数、零、NaN、正/负无穷均拒绝配置；A/B 不接受该字段。C/D 的零保持会消去保持因素，不能当作 C/D 的有效设置。
- 参数缺失、类型不符、非有限值或字段冲突在 run 启动前报告为配置错误；不猜默认值，不静默夹取、替换或将非法值改为零。A 运行时解析对新配置字段实行严格允许列表，并拒绝三个旧根字段作为并列权威。
- 正式配对矩阵冻结配置时逐项核查：四组 `entry_threshold` 完全相同；B/D 的 `exit_threshold` 完全相同；C/D 的 `recovery_min_hold_s` 完全相同；模型、观测/动作接口、控制参数、场景与终止规则等非研究变量保持相同。该一致性核查属于正式批次配置检查，不在此建设通用 schema/验证框架。
- 各阈值和保持值的具体实验数值仍需在允许的参数预实验/正式冻结节点确定。本合同没有把当前 `-0.05`、由旧差值推得的 `-0.08` 或旧 30 步换算值认定为最终 A/B/C/D 参数。

### `recovery_min_hold_s` 的计时语义

- **仿真时钟：**仿真中的 `H` 按 MuJoCo `mjData.time` 前进的仿真秒数计算。现有 MuJoCo 端会发布 `/mujoco_sim_clock`，包含 `sim_time`；但当前 `StateRL` 控制器没有读取该时钟，时间源尚未接入切换器。后续 S1 需实现并核验当前 run 的 sim-time 读取、有效性和生命周期绑定。[仿真时钟接口](../../common/abs_sim_clock_contract.h) [MuJoCo 发布位置](../../unitree_mujoco/simulate/src/main.cc:702)
- **实机时钟：**实机中的 `H` 按 `std::chrono::steady_clock` 语义计算单调运行时间，不使用墙上日历时间。控制器已有单调纳秒时间读取函数，但当前没有用于 Recovery hold 计时；该用途同样属于后续 S1 实现。[StateRL.cpp:28–33](../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp)
- **起算事件：**一次有效 RA 判断使模式从 Agile 转入 Recovery 时，记录本次 `policy_mode_changed` 的时刻 `t_enter`；该周期选择 Recovery 动作。仿真保存 MuJoCo 仿真时刻，实机保存单调运行时刻。C/D 的到期判断为对应时钟的 `now - t_enter >= H`；A/B 不读取或启动该计时。`H` 表示 Recovery 模式最短驻留时间，不保证从首条 Recovery 命令写出或实际生效时起满 `H` 秒；事件周期和时刻定义见第 8 节。
- **判断周期：**仿真和实机都只在有效策略决策周期检查保持是否到期及执行模式转换。对 C/D，该周期还必须有有效的对应时钟样本；若时钟样本无效，则本周期不执行 RA 驱动的模式转换，保持当前模式并选用其动作，同时记录计时无效。保持到期本身不会在周期之间触发转换；到期周期立即检查 RA 退出条件，满足则本周期选 Agile 动作，否则继续 Recovery。
- **仿真暂停与速度变化：**在可确认的 MuJoCo 暂停状态下，`sim_time` 不前进，`H` 不消耗；即使暂停期间出现策略决策，C/D 也不得因墙钟经过而退出。若时钟样本陈旧且无法确认是暂停还是来源失效，应按时钟无效处理。仿真运行速度变化时，`H` 仍按仿真秒数计算，因此对应的墙钟持续时间随仿真快慢变化。
- **延迟与漏周期：**两种时钟都在下一有效策略周期读取。仿真中漏过策略周期时，使用下一决策时最新有效的 `sim_time`；实机使用该周期的单调时刻。经过时间跨过 `H` 时，在第一个可用且 RA 满足退出条件的决策周期退出，不补造中间转换；实测驻留时间可长于 `H`。
- **时间异常：**仿真 `sim_time` 非有限、在同一 run 内回退、来源不可用或无法确认属于当前 run 时，该周期的保持时间无效，不得判定到期或因保持条件退出；留在 Recovery，保留原 `t_enter` 并记录计时无效。实机若单调时间无效或检测到回退，同样不判定到期、不重置起算点；待有效时间恢复且达到原 `t_enter + H` 后再正常判断。检测到新 run/仿真重置则按新会话生命周期清除旧计时器，而非视为同一 run 内时钟回退。
- **会话生命周期：**新 run/重新进入 RL 时清除旧 `t_enter`；run 终止后计时失效。参数在单个 run 中固定，不热更新。

### 频率证据与未决实测

- 步骤 1 已查到一次历史仿真捕获的策略/RA 周期均值约 20.014 ms（49.97 Hz，1,249 个间隔）；这是该次捕获的观测，不是当前所有运行或 Recovery 活动态的保证。另有配置及旧注释声称 125 Hz/8 ms，但与该捕获实测不符。持久记录见 [P1-08 v2 计时摘要](../evidence/P1-08/P1-08_v2_baseline_capture_20260902.md) 和 [该次捕获的计时统计](../evidence/P1-08/capture_20260901_v2/timing_stats.json)。
- 因为仿真按 MuJoCo 仿真时间、实机按单调运行时间计算，保持合同不需要从 30 步、50 Hz 或 125 Hz 推算 `H`。当前计划运行的实际策略/RA 频率仍是 **UNKNOWN**；也没有 Recovery 活动态的保持实测。
- 后续 S1 最小工作/实测：先把 MuJoCo `sim_time` 时间源接入控制器并确认其对应当前 run；基础仿真中记录连续有效策略决策周期的 RA、策略序号和 `sim_time`，触发一次受控 Recovery 后核对起算、到期判断及退出时刻。另用 `rt_frame` 单调时间戳测策略/RA 的墙钟频率；若开展实机阶段，再用单调运行时钟核实实机保持。以上均不启动正式场景或参数比较。

## 7. 本步骤未冻结事项

- `entry_threshold`、B/D 的 `exit_threshold` 和 C/D 的 `recovery_min_hold_s` 具体实验取值：后续参数预实验/正式配置冻结时确定。
- 当前目标运行环境的实际策略/RA 频率及 Recovery 活动态保持实测：仍为 UNKNOWN，按第 6 节的方法留待 S1 基础仿真实测。
- 手工序列已完成并见第 10 节；S1-01 是否完成以 Director 最终审阅为准，审阅状态见第 11 节。

## 8. 事件与策略周期对应（步骤 4）

### 事件定义

所有事件须关联到同一个 run/session 内产生它的策略周期。使用以下标识和时间：

- 主关联键：实验 `run_id`、控制器 `session_id`、`rl_step`。`session_id` 在一次 RL 会话进入时生成；`rl_step` 是该会话的策略步序号。
- 帧/记录关联：`source_sequence` 用于识别共享内存帧版本及记录顺序，不能代替 `rl_step`。同一帧的事件都保留相同 `session_id` 和 `rl_step`。
- 时间：仿真 run 的事件时间以所属周期的 MuJoCo `sim_time` 为研究时间；实机事件时间以单调运行时钟为研究时间。若还记录 `monotonic_ns`，它是墙钟诊断/调度时间，不能代替仿真 `sim_time`。当前 runtime frame 只有 `monotonic_ns`，没有 `sim_time`，因此仅靠现有帧无法生成精确的仿真时间延迟指标。

| 事件 | 定义及周期归属 | 可用于什么 |
|---|---|---|
| `risk_condition_met` | 在周期 `k` 的 RA 输出可用后，用该 run 的进入规则判断；成立的样本记在 `k`，事件时刻锚定该周期 RA 判断。目标合同中为 `RA[k] >= E`。 | 标记当前样本满足进入条件；持续为真时每周期可视为条件成立，但不能把每周期都算作一次新触发。 |
| `risk_condition_entered` | `risk_condition_met` 从假变真时，本次风险区间的首次有效采样周期；时间戳沿用该周期 RA 判断时刻。只在同 session 的前一有效周期紧邻（`rl_step[k]=rl_step[k-1]+1`）且前周期条件为假时，才能精确确认边沿。 | 风险事件始终记录。只有该边沿发生时的 `mode_before` 为 Agile，且能与同一风险区间、同一 session 内由本次进入条件导致的 Agile→Recovery `policy_mode_changed` 正确配对，才计算“进入 Recovery 的切换响应延迟”。若风险条件在 `mode_before=Recovery` 时再次由假变真，只记该风险事件，不计算新的进入响应延迟。若 session 首个有效样本已为真，记作左删失/起点 UNKNOWN；若前后样本有缺口，只能把边沿限制在最后一个有效假样本与首个有效真样本之间，不能声称精确时刻。 |
| `policy_mode_changed` | 在周期 `k` 的有效 RA 判定使模式从 `mode_before` 变为 `mode_after` 时记一次；同时保存两模式、组别、转移原因。时间戳锚定状态机作出转换决定的时刻，即 RA 判定后、当周期策略动作选择前；不是帧写完或关节动作被 MuJoCo 消费的时刻。 | 仅对符合上一行配对条件的风险边沿，计算 `risk_condition_entered → Agile→Recovery policy_mode_changed` 的切换响应延迟；另可报告同一风险区间到 Recovery 命令写出的延迟。 |
| `recovery_command_issued` | Recovery 模型动作通过有限值/限幅/关节目标检查后，首次由 `setCommand()` 写入控制器 command interfaces 的周期；时间戳取该接口写入时刻。事件关联生成该命令的 `source_policy_step`，并另存实际接口写入的 controller-cycle 序号。 | 衡量切换后到 Recovery 命令进入控制输出的时间。`policy_mode_changed` 或策略模型产生动作本身不等于已写入控制接口。 |
| `recovery_command_applied` | 仅在 MuJoCo 一侧能够确认消费了同一条命令，并可关联至 command/policy 周期时记录；时间戳是该消费确认时刻。 | 衡量命令被仿真执行的时间；当前现有 frame/run record 不证明该事件，保持 UNKNOWN。 |

若风险条件在首个采样周期已成立，响应延迟的风险起点是左删失，不计算伪精确的 `risk_condition_entered` 延迟。若帧缺失、`rl_step` 不连续、RA/时钟无效或跨 session，风险边沿和相应延迟记为 UNKNOWN 或区间界限，不跨缺口插值。

### 现有记录可判断范围

- 当前 `StateRL::runModel()` 每个策略周期先调用 RA，再执行切换器、选取 Agile/Recovery 策略动作、生成关节目标；其末尾写出一个 runtime frame，然后递增 `rl_step_count_`。[StateRL.cpp:1597–1599, 1625–1629, 1657–1758](../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp)
- 因此单个完整有效 runtime frame 的 `ra_value`、`policy_state`、`action_raw`、`action_clipped`、`joint_target_rad` 属于同一次 `runModel()` 策略计算：`policy_state` 是切换后的模式，动作来自该模式。该帧以 `session_id`、写出前的 `rl_step` 和 `monotonic_ns` 关联；Python recorder 保留这些字段以及原始动作字段。[帧结构](../../common/abs_rt_frame_contract.h) [run_record.py:152–185](../../scripts/run_record.py)
- 相邻连续帧的 `policy_state` 可推导模式变化发生在两帧之间，并将变化归属到新模式所在的当前 `rl_step`；但当前帧不保存 `mode_before`、模式转移原因、进入/退出阈值、hold 状态或 `sim_time`。`monotonic_ns` 是帧时间锚点，不是切换决定的精确事件时间。
- 现有 `[RA-REC]` 文本在状态更新和 Recovery twist 优化之后输出，没有 `session_id`、`rl_step` 或独立事件时间；现有 `[ABS-LIVE-CMD] rl_command_write` 是控制接口写入遥测，但按遥测序号抽样，且不带对应 `rl_step`、策略模式或动作来源。因此两者均不能单独建立精确的事件周期链。[StateRL.cpp:1634–1653, 826–862](../../quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp)
- 当前 frame 中有关节目标和力矩计算值，但它们是控制器生成/计算的输出；没有 MuJoCo 消费确认及同一 command ID 的关联，不能据此记为 `recovery_command_applied`。现有记录也没有仿真 `sim_time`，所以策略周期内的仿真秒级响应时间目前 UNKNOWN。

### 最小补充记录需求

后续 S1 若要精确报告上述事件，最小补充是在不改变控制决策的前提下，把以下信息和现有帧按 `run_id + session_id + rl_step` 对齐：

- 每个策略周期的 `sim_time`（仿真）或 `monotonic_ns`（实机），RA、组别、进入/退出阈值、模式判定前后值、hold 启用/起算/到期状态、动作来源和转移原因。
- 模式变化事件在决策周期内的时间戳；Recovery command 写控制接口时的时间戳、接口写入周期号，以及关联的 `source_policy_step`/command ID。不要用抽样的通用命令遥测替代首条 Recovery 写入事件。
- 只有在 MuJoCo 侧能以相同 command ID 或等价序号确认该命令被消费时，才增加 `recovery_command_applied`；否则该事件继续标 UNKNOWN，不以命令写出推定已执行。
- 风险条件首次进入可由相邻连续的有效 RA 帧离线推导；若要把该边沿作为正式指标，应保留每周期 RA、阈值配置、run/session 和无缺口的周期序号。丢帧时按上述规则报告 UNKNOWN/区间，不伪造边沿时间。

本步只冻结记录定义与最小补足位置，不要求改造运行时日志。S1 的基础闭环记录应优先补入策略周期与仿真时间源；不要求为 `recovery_command_applied` 建设复杂的新执行追踪，除非后续 MuJoCo 接口能直接给出可信消费证据。

### 候选实现映射

- `paper_faithful_switch` 是 A 类单阈值、无保持候选：当前 RA 满足 `RA >= E` 时，在该策略周期作 Agile→Recovery 转换；未满足时为 Agile。它没有滞回/hold 状态，因此持续高 RA 不是重复的 mode-change 或 risk-entered 事件。
- `stabilized_switch` 仅是 D 类候选：当前实际进入条件为 `RA > E`，退出条件为保持步数到期且 `RA < X`，其中当前 `X` 由 `E - 0.03` 得出；既有严格进入边界、硬编码差值和按步保持都与本合同 A/B/C/D 配置/时间合同存在差异。不得直接把其日志视作正式 D 结果。
- 两个候选都在当前策略周期完成状态判断并由新状态选择 Recovery 或 Agile 动作；`stabilized_switch` 只在 Agile→Recovery 边沿计算并缓存 Recovery twist。若未来四组共用该优化和缓存路径，它属于公共 Recovery 命令链；当前缺少与具体 `rl_step` 绑定的 `recovery_command_issued` 事件证据。

## 9. 步骤 4 剩余 UNKNOWN

- 当前日志能把 RA、后切换模式及该模式动作关联在一个 runtime frame/策略周期，但不能给出精确 `policy_mode_changed` 事件时刻或 Recovery command interface 首写时刻。
- Recovery 命令被 MuJoCo 实际消费的时刻仍 UNKNOWN；仅有生成的目标/力矩或控制接口写入，不构成消费证据。
- 当前 frame 没有仿真 `sim_time`，仿真秒级事件延迟和步骤 3 仿真 hold 的运行态闭环仍待后续 S1 接线与基础仿真核验。
- 以上 UNKNOWN 不阻止手工状态序列核验；有关日志的最小补充边界已定义于第 8 节。

## 10. 手工序列核验（步骤 5）

以下只按第 2 节状态表离线推演，不调用模型或运行控制器。使用符号阈值 `X < E`、`H > 0`；选足够小的 `ε > 0`，使表中 `E±ε`、`X±ε` 均在 RA 的 `(-1,1)` 范围内，且需要时 `X < E-ε < E`；`M=(X+E)/2`，故 `X < M < E`。选 `0<δ<H`。每条独立组别序列及“另一分支”均代表独立 run；该段内 `s/K` 表示同一 `session_id=s` 的 `rl_step=K`，各行均假定周期连续且 RA/时钟有效。

事件对齐约定：风险条件首次从假变真、Agile→Recovery、当周期动作来源，都可关联到同一 `s/K`；首条 Recovery 控制接口写出记录其生成源 `s/K`，实际接口写入周期可在其发生时另记。表中“Recovery 命令”指状态机应选 Recovery 路径；不是声称接口写入或 MuJoCo 应用已实测。`recovery_command_applied` 在所有离线序列中均不产生实测事实。

### 共同进入边界与 A/B 阈值往返

| 组/周期 | RA | 原模式 | hold（周期前→后） | 新模式 | 原因/事件 | 预期动作来源 |
|---|---:|---|---|---|---|---|
| A/B/C/D · `s/0` | `E-ε` | Agile | 关闭/未启动 | Agile | 进入条件假 | Agile |
| A/B/C/D · `s/1` | `E` | Agile | A/B 关闭；C/D `未启动→t₀` | Recovery | `risk_condition_entered`；`policy_mode_changed`；`risk_enter` | Recovery；命令源策略周期 `s/1` |
| A · `s/2` | `E+ε` | Recovery | 关闭 | Recovery | 进入条件仍真；无新状态边沿 | Recovery |
| A · `s/3` | `E-ε` | Recovery | 关闭 | Agile | `risk_exit_single` | Agile |
| A · `s/4` | `E+ε` | Agile | 关闭 | Recovery | 新风险区间；本周期再次 `risk_condition_entered` | Recovery；命令源策略周期 `s/4` |
| B · `s/2` | `M` | Recovery | 关闭 | Recovery | `RA<X` 为假；阈值间隙内保持 Recovery | Recovery |
| B · `s/3` | `X` | Recovery | 关闭 | Recovery | 等于退出阈值不退出 | Recovery |
| B · `s/4` | `X-ε` | Recovery | 关闭 | Agile | `risk_exit_hysteresis` | Agile |
| B · `s/5` | `E` | Agile | 关闭 | Recovery | `risk_condition_entered`；`RA==E` 进入 | Recovery；命令源策略周期 `s/5` |

A 在进入边界附近往返时会按单阈值即时切换；B 在 `X < RA < E` 区间留在 Recovery，并且 `RA==X` 仍不退出。此差异来自滞回因素。

### C/D 保持与到期边界

| 组/周期 | RA | 相对进入时刻 | 原模式 | hold（周期前→后） | 新模式 | 原因 | 预期动作来源 |
|---|---:|---:|---|---|---|---|---|
| C · `s/1` | `E` | `t₀` | Agile | 未启动→`t₀` | Recovery | 进入并起算 `H` | Recovery |
| C · `s/2` | `E-ε` | `t₀+H-δ`，`δ>0` | Recovery | 未到期→未到期 | Recovery | 风险下降但保持阻止退出 | Recovery |
| C · `s/3` | `E-ε` | `t₀+H` | Recovery | 到期→结束 | Agile | 到期当周期满足 `RA<E`，`hold_and_risk_exit_single` | Agile |
| C · `s/1`（另一分支） | `E` | `t₀` | Agile | 未启动→`t₀` | Recovery | 进入并起算 `H` | Recovery |
| C · `s/2`（另一分支） | `E+ε` | `t₀+H/2` | Recovery | 未到期→未到期 | Recovery | 风险再次满足进入条件，不重启 `H` | Recovery |
| C · `s/3`（另一分支） | `E` | `t₀+H` | Recovery | 到期→已到期 | Recovery | 保持结束但单阈值退出条件不满足 | Recovery |
| C · `s/4`（另一分支） | `E-ε` | `t₀+H+δ` | Recovery | 已到期→结束 | Agile | 单阈值退出条件成立 | Agile |
| D · `s/1` | `E` | `t₀` | Agile | 未启动→`t₀` | Recovery | 进入并起算 `H` | Recovery |
| D · `s/2` | `X-ε` | `t₀+H-δ` | Recovery | 未到期→未到期 | Recovery | 风险下降但保持阻止退出 | Recovery |
| D · `s/3` | `X` | `t₀+H` | Recovery | 到期→已到期 | Recovery | 到期但退出边界相等，仍 Recovery | Recovery |
| D · `s/4` | `X-ε` | `t₀+H+δ` | Recovery | 已到期→结束 | Agile | `hold_and_risk_exit_hysteresis` | Agile |
| D · `d/0`（独立 run） | `E-ε` | 尚未进入 | Agile | 未启动 | Agile | 进入条件假 | Agile |
| D · `d/1`（独立 run） | `E` | `t₀` | Agile | 未启动→`t₀` | Recovery | `risk_condition_entered`、Agile→Recovery；在此周期起算 `H` | Recovery |
| D · `d/2`（独立 run） | `E+ε` | `t₀+H/2` | Recovery | 未到期→未到期 | Recovery | 风险仍高；不重启保持 | Recovery |
| D · `d/3`（独立 run） | `E` | `t₀+H` | Recovery | 到期→已到期 | Recovery | 保持到期但风险仍高；不退出、不重启保持 | Recovery |
| D · `d/4`（独立 run） | `X-ε` | `t₀+H+δ` | Recovery | 已到期→结束 | Agile | `RA<X`，`hold_and_risk_exit_hysteresis` | Agile |

这些序列覆盖未到期时风险回落、到期当周期可退出、到期但风险条件不满足退出，以及 Recovery 中再次满足进入条件不重新起算保持。C 与 A 的退出边界相同，D 与 B 的退出边界相同；保持只增加 C/D 的最短驻留限制。

### 非有限 RA 与时间异常

| 组/输入周期 | RA / 时间 | 原模式 | 计时状态 | 结果模式/动作 | 处理与事件 |
|---|---|---|---|---|---|
| A/B/C/D · 有效周期后 | `RA=NaN` | Agile 或 Recovery | 任意 | 无下一策略动作；不产生模式转换 | 当前 `runRAModel()` 在切换器前检测非有限 RA 并触发 safety veto；`risk_condition_met` 无效，不生成 mode-change 或 Recovery-issued 事件 |
| A/B/C/D · 有效周期后 | `RA=+∞` 或 `RA=-∞` | Agile 或 Recovery | 任意 | 无下一策略动作；不产生模式转换 | 与 NaN 同属非有限 RA；不得按比较结果推断风险方向 |
| C/D · `s/K` | `RA>=E`，但对应时钟 NaN/Inf/不可用，或同一 run 内 `now<t₀` | Agile | hold 尚无有效起点 | 保持 Agile；本周期不做切换 | 时钟样本无效，不算有效切换决策周期；记录计时无效 |
| C/D · `s/K` | RA 低于退出阈值，但仿真明确暂停、`sim_time` 保持 `t₀+H-δ` | Recovery | 未到期 | Recovery | 仿真时间未经过 `H`；墙钟经过不能触发退出 |
| C/D · `s/K` | RA 低于退出阈值，时钟有效且首次恢复到 `now>=t₀+H` | Recovery | 原起算点 `t₀`，不重置 | Agile | 在此有效周期判定已到期并满足退出条件 |

`+∞/-∞` 在实际 RA 入口和纯切换 helper 中都属于 invalid；生产路径的 RA safety veto 优先于模式表。保持时间配置的负值、零、NaN/Inf 已由第 6 节定义为 run 启动前配置错误，不进入有效序列推演。仿真重启/新 run 清除旧计时起点；同一 run 内回退不重置起点。

### 手工核验结论与最终交付边界

- 所有有效有限 RA 序列均符合第 2 节转移表；`RA==E` 的入场、`RA==X` 的滞回出口边界，以及 hold 到期周期行为都已覆盖。
- A/B/C/D 的差异仅由退出使用 `E` 或 `X`（滞回）和 hold 是否启用产生。各组共享进入条件、状态动作选择、RA/Agile/Recovery 调用及命令路径；序列没有引入第三个研究变量。
- 手工推演展示的是合同期望事件及其策略周期归属，不是运行日志、模型输出或仿真事实。当前 `risk_condition_entered` 可从连续帧离线推导；精确模式决定时刻、首条控制接口写入及 MuJoCo 消费仍按第 9 节标为 UNKNOWN。
- 候选实现映射、配置合法性和时钟语义分别见第 5、6、8 节；本文件构成 S1-01 最终合同审阅稿。

## 11. S1-01 执行报告索引与下一步边界

最终合同：本文件 `docs/thesis_project/SWITCH_CONTRACT.md`。章节索引：RA 方向与状态合同见第 1–5 节；配置、保持时间及频率事实见第 6 节；周期事件与现有记录边界见第 8–9 节；离线序列及核验结论见第 10 节。

S1-01 没有实现运行时代码、修改模型/配置、启动仿真或正式实验，也没有开展实机测试。该合同已通过最终审阅；后续关键接口核对和基础仿真时基诊断由 S1-02 单独承担，仿真时间源接入与完整基础闭环仍须按阶段结果安排。
