# S7-01 Cycle0 仿真时钟缺口定向复核

日期：2026-10-21  
对象：开发补证 A run `798d518a8fb84d419cb00987ed6a8538`（m02.xml / A）  
状态：**根因已定位；Director已同意，分析侧决策时钟重建已统一接入正式分析；原始发布态仍保留，默认 `calculate` 仅用于敏感性对照。未启动任何场景，未改写原记录，未提交 Git。**  
S7-01 已 COMPLETE（Director 有条件通过）；S7-02 为唯一 ACTIVE / AWAITING_RUN_AUTHORIZATION；240 项仍未授权。

## 1. 结论摘要

Cycle0 不是“控制器没有仿真时钟”，也不是“帧字段丢失”。原始证据显示：

- 控制器在 cycle0 捕获时读到了有效时钟：`S6-04` controller trace 在 policy_cycle=0 记录 `sim_clock_valid=true`、`sim_time_s=7.126`；
- 同一 cycle0 的 runtime frame 在**发布时**被 `writeRtFrame` 的新鲜度复查标记为 `sim_clock_valid=false`、`sim_clock_status=2 (STALE)`，因为发布时刻距该时钟样本已 **108,484,356 ns**（约 108.5 ms），超过控制器 100 ms 新鲜度阈值；
- 原始 frame 仍保留 `sim_time_s=7.125999999999437`、`sim_clock_sequence=7128`、`sim_clock_monotonic_ns=3882168845991` 和 age；不是字段缺失；
- `s1_run_diagnostic` 在写 `context.lifecycle.task_start` 时按 `frame.sim_clock_valid` 决定是否保留 sim time，因此把 cycle0 记为 `sim_time_s=null`，导致耗时、部分切换覆盖和安全起点判断不完整。

因此属于：

> **有效时钟被读到并通过 decision-time 校验，但在 frame 发布时的 freshness 复查中被判 stale；记录层又只保留 frame 的发布态，导致 cycle0 的 decision-time 时钟事实在 context/task_start 中丢失。**

不是“控制器确实没有有效时钟”，也不是“有效时钟存在但读取/校验失败在读取端”，也不是“字段在帧中丢失”。更精确地说，是**decision-time 有效性通过、publication-time 新鲜度失败**，以及记录层未保留 decision-time 事实。

根因机制明确；首次策略周期为何耗时到让样本跨过 100 ms 的**具体子原因**（冷推理、调度、渲染/锁竞争等）没有单独埋点，不能唯一归因。下文只把已由时间戳和代码路径证明的部分称为 FACT。

## 2. 授权范围与未做事项

本轮只读取 A/D 原始记录、`ros_launch.log` 中的 controller-authored `S6-04` trace、相关 C++/Python 代码，并做离线派生分析。

未做：

- 不启动 MuJoCo、ROS、controller、模型推理或留出图；
- 不重跑 A，不消耗已用尽的两次开发补证额度；
- 不修改地图、参数、安全阈值、控制逻辑或原始记录；
- 不清理数据、不提交 Git；
- 不把重建用于改变控制行为或任务起点；只接入正式分析口径，并保留原始发布态与敏感性对照。

## 3. 最小事件时间线

### 3.1 A

| 阶段 | 时间值 | 时间基准/来源 |
|---|---|---|
| MuJoCo 进程启动 | `3874243913490` | controller/observer steady ns（lifecycle） |
| MuJoCo authority LIVE | `3875080031379`，physics_step=18，sim=0.036 s | steady ns + MuJoCo sim |
| 起立准入通过 | `3882077171461`，sim interval [6.522, 7.032] s | steady ns + MuJoCo sim |
| command 3 请求 | `3882142943182` / `3882143295872` | lifecycle steady ns |
| StateRL 进入 | `3882156089415` / `3882156159664` | controller steady ns |
| 最后一个 cycle0 前 sim-clock 样本 | `3882168845991`，sim=7.126 s，sequence=7128 | producer steady ns + MuJoCo sim |
| cycle0 runtime frame | `3882277330347`，age=108,484,356 ns，status=STALE，raw sim=7.126 s | observer/recorder steady ns |
| cycle0 controller trace | `3882277656062`，policy_cycle=0，gain_step=3，sim_clock_valid=true，sim=7.126 s | controller steady ns |
| cycle1 frame | `3882286232171`，age=3,216,829 ns，sim=7.240 s，sequence=7242 | observer/recorder steady ns |
| ARRIVED / terminal | `3888676433015`，sim=13.628 s，rl_step=325 | steady ns + MuJoCo sim |
| 清理完成 | `3895214555315`；MuJoCo/ROS/查询会话 exit 0 | steady ns + process facts |

同一时间基准内可比较的差值：

- 最后样本 → cycle0 frame：**108.484 ms**（producer/observer steady ns）；
- cycle0 frame → cycle0 trace：**0.326 ms**（controller/observer steady ns）；
- cycle0 frame → cycle1 frame：**8.902 ms**；
- cycle0 样本 sim 7.126 s → cycle1 sim 7.240 s：**0.114 仿真秒**（MuJoCo sim，不与 wall ns 相减）。

### 3.2 D（链路背景，不作性能对照）

| 阶段 | 时间值 | 时间基准/来源 |
|---|---|---|
| MuJoCo 进程启动 | `4199804265922` | steady ns |
| authority LIVE | `4200602596720` | steady ns |
| 起立准入 | `4207245394855` | steady ns |
| command 3 请求 | `4207299178956` | steady ns |
| cycle0 frame | `4207380940846`，age=53,801,025 ns，status=FRESH，sim=6.752 s | observer/recorder steady ns |
| cycle1 frame | `4207411276480`，age=28,688,050 ns，sim=6.808 s | observer/recorder steady ns |
| ARRIVED / terminal | `4213936440553`，sim=13.352 s，rl_step=330 | steady ns + MuJoCo sim |

D 的 cycle0 在 100 ms 新鲜度内，因此没有触发同一路径。D 只用于证明链路和阈值行为，不用于 A/D 性能比较。

## 4. 代码路径与证据

### 4.1 控制器

`quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp`：

1. `StateRL::runModel()` 开头调用 `captureSimClock()`；
2. `captureSimClock()` 读取 `/mujoco_sim_clock` seqlock 快照，检查 session/segment/sequence/时间顺序，并以 `kSimClockFreshnessNs = 100'000'000` 判断样本年龄；通过后写 `cycle_trace_.sim_clock_valid=1`、`cycle_trace_.sim_time_s=sample.sim_time`；
3. `writeRtFrame()` 把 `cycle_trace_` 复制到私有 `local` 帧后，又用 frame 发布时刻 `now_ns` 重算年龄；若 `now_ns - sim_clock_monotonic_ns > 100 ms`，把 **local frame** 的 `sim_clock_valid` 置 0、`sim_clock_status` 置 STALE，但保留 `local.sim_time_s`、`sim_clock_sequence`、`sim_clock_monotonic_ns` 和 age；
4. `S6-04-TRACE` 在 `setCommand()` 中读取 `cycle_trace_`，因此记录的是 **capture-time** 的 valid=true；runtime frame 记录的是 **publication-time** 的 valid=false。

这是同一条 cycle0 的两个时间点，不是两个不同 cycle，也不是后续 cycle 回填。

### 4.2 A 的直接证据

- frame：`sim_clock_valid=false`、`sim_clock_status=2`、`sim_clock_age_ns=108484356`、`sim_time_s=7.125999999999437`、`sequence=7128`；
- trace：同一 session `3882150905299`、policy_cycle=0、`sim_clock_valid=true`、`sim_time_s=7.126`；
- 由于 `captureSimClock()` 是 `cycle_trace_.sim_clock_valid` 的唯一写入者，而 trace 读到 true，故 capture 时样本年龄必在 100 ms 内并已通过身份/顺序检查；
- 由于只有 `writeRtFrame()` 会在 capture 后把 **frame** 的 valid 置 0，且 frame age 108.48 ms > 100 ms，故 cycle0 frame 的无效发生在发布时的 freshness 复查。

### 4.3 记录层

`s1_run_diagnostic.py` 在首个 `rl_entered=1 && rl_active=1` 帧写入 `lifecycle.task_start`：

```python
"sim_time_s": frame.sim_time_s if frame.sim_clock_valid else None,
"sim_clock_valid": bool(frame.sim_clock_valid),
```

因此 context/task_start 只保留了 publication-time 的 `false/null`，没有保留 trace 中同一 cycle 的 decision-time `true/7.126`。这不是字段在 JSONL 中丢失，而是记录策略把 decision-time 事实丢在了 context 之外。

### 4.4 不能确认的部分

- 首次策略周期为何长到让样本跨过 100 ms：没有单独的冷推理/调度/锁等待埋点。时间戳只能证明“capture 时未超限、发布时超限”，不能唯一归因到冷启动或某个线程。
- 历史 A 的 9 个 `sim_clock_valid=false` context（含兼容路径重复）与 B/C/D 的历史 true 结果，不能证明 A 组有独立代码路径；A 往往是批次首个/唯一组，存在“首个进程/冷启动”混杂。没有证据支持“A 组天然更容易”或“B/C/D 不受影响”。
- 因此，机制是确定的；具体延迟子原因和组别概率差异是 UNKNOWN。

## 5. 统一接入的分析侧修正

### 5.1 改了什么

在 `scripts/s1_analyze_run.py` 增加只读 helper `resolve_decision_start_clock()`：

- 仅在首个 active frame 为 **publication-stale**（`sim_clock_status=STALE` 且 `sim_clock_age_ns >= 100 ms`）时查找 `ros_launch.log`；
- 只接受同一 `session_id`、同一 `rl_step`、`controller-authored S6-04 trace` 中 **最早一条**、`sim_clock_valid=true`、`sim_time_s` 与 raw frame 的 `sim_time_s` 在 1e-9 内一致的记录；
- session/cycle 不符、trace 缺失、trace 在 capture 时无效、sample 不一致、frame status 不是 STALE、age 为 0/过小、frame sim time 缺失、future/reset/identity 异常，全部拒绝；
- 通过后生成 `task_start_clock` 事实，记录 raw frame 的 stale 状态和 age；原始 frame/JSONL/context/terminal 不变。
- 该重建依赖 controller-authored `S6-04-TRACE`。当前 dev/formal 单run入口均携带 `--startup-gain-trace`，因此 trace 存在；若未来入口关闭该选项，helper 返回 trace missing，保持原 UNKNOWN，不猜测。

在 `scripts/safety_task_completion.py` 增加可选 `start_clock_override`：

- 只有 `task_start_clock.status == VALID_AT_DECISION` 时才用同一 cycle0 的 decision sim time 作为安全窗口起点；
- 仍保留 raw frame 的 `sim_clock_valid=false` 和 `raw_task_start_time_s=null`；
- 不改变任务起点 cycle（仍为 cycle0），不移动边界，不插值，不用墙钟，不放宽时钟/身份/新鲜度阈值。

### 5.2 正式分析状态

- 正式分析入口 `analyze_s7_formal_240.py` 通过 `recompute_scene_start_summaries(..., decision_clock_override=True)` 对 A/B/C/D **统一应用**同一规则；`s7_dev_supplement.analyze_dev_batch` 复用同一函数和规则。
- `calculate(..., decision_clock_override=False)` 仍是直接调用/敏感性对照的默认；正式分析显式传 true。原始摘要与默认发布态可通过同一开关复算，不另写第二套流水线。
- 正式汇总新增 `reconstruction_sensitivity`：重建 run 数、安全分类改变数、主要时间指标改变数及逐 run 前后对照。
- `check_s7_cycle0_clock.py` 用 true 模式生成派生对照，所有派生结果独立保存，不覆盖原 metrics。

### 5.3 为什么没有改控制器/加等待

- 时钟在 decision-time 已经有效；问题发生在 frame 发布时的复查和记录层，而不是“控制器没等到时钟”。
- 若 capture 时样本已接近 100 ms，单纯在决策前等待一个新样本不保证能覆盖随后首次推理的发布时间；增加等待还可能改变启动时序。
- 因此本轮只做记录/派生层的 decision-time 事实保留，不改控制、起立准入、四组语义或安全阈值。
- 如果 Director 要求从控制侧消除该风险，应另立最小任务，在首个 RL 决策前做有界、保留原控制状态的 readiness/warm-up，并用单独授权的开发运行验证；本轮不做。

## 6. 定向离线结果

执行：

```sh
/usr/bin/python3 -m py_compile scripts/s1_analyze_run.py scripts/safety_task_completion.py scripts/check_s7_cycle0_clock.py
/usr/bin/python3 scripts/check_s7_cycle0_clock.py
```

结果：**10/10 PASS**。机器结果：[check_results.json](check_results.json)。

| # | 检查 | 结果 |
|---:|---|---|
| 1 | A cycle0 decision-time valid、publication-time stale，sample 同一 | PASS |
| 2 | 原始发布态分析保持 UNKNOWN，task start/elapsed 为空 | PASS |
| 3 | 统一重建恢复 task start=7.126、elapsed=6.502、timing invalid=0、short reverse complete；raw invalid 仍为 1 | PASS |
| 4 | A 原始 JSONL/context/terminal/metrics 哈希前后不变 | PASS |
| 5 | A trace/session/cycle/sample 身份一致 | PASS |
| 6 | D FRAME_VALID，无重建，分析结果不变 | PASS |
| 7 | 构造输入拒绝：trace 缺失/空、session/cycle 不符、sample 不符、capture 时无效、frame reset、age=0、sim time 缺失、frame 已有效、task start 缺失 | PASS |
| 8 | A/B/C/D 共用 decision-path，不存在组别特有等待 | PASS |
| 9 | 真正 fall UNKNOWN 不会被重建清成 False；确认跌倒时 `fall=True`，安全任务完成=`False` | PASS |
| 10 | 正式分析 recompute/build_views 使用统一规则；重建1 run、安全分类改变1、时间指标改变1 | PASS |

A/D 前后对照（正式分析口径）：

| 项目 | 原始发布态/默认对照 | 统一正式分析 |
|---|---|---|
| A safety_task_completion | `UNKNOWN` | `True` |
| A task start sim time | `None` | `7.126 s` |
| A elapsed task | `None` | `6.502 s` |
| A valid continuous | 6.388 s | 6.502 s |
| A raw invalid clock frames | 1 | 1（raw 不改） |
| A timing invalid clock frames | 1 | 0 |
| A short reverse coverage | `partial_or_unknown` | `complete` |
| A 非足端碰撞 | UNKNOWN | False，coverage complete |
| A 跌倒 | UNKNOWN | False，coverage complete |
| A base_link 越界 | False | False |
| A 足端接触/撞击 | UNKNOWN | 1 / 1（FL vs ppt_obstacle_01，sim 10.704–10.716 s，独立报告） |
| A 任务起点 cycle | 0 | 0（未移动） |
| D safety_task_completion | `True` | `True` |
| D task start / elapsed | 6.752 s / 6.600 s | 6.752 s / 6.600 s |
| D 重建应用 | 否 | 否 |

统一分析结果：A/D 主结果均为 `CONFIRMED_SAFE_COMPLETE`；original/default 对照为 A `UNKNOWN`、D `True`。重建 1 run、安全分类改变 1、主要时间指标改变 1、unresolved 0。A 的变化来自同一 cycle0 decision-time 时钟恢复后，collision/fall/越界覆盖共同满足；不是直接强制 True。足端撞击仍单列，不并入非足端碰撞否决。

### 6.1 回归

- `check_s7_safety_completion.py`：9 条真实记录 + 11 条构造输入 PASS；
- 开发补证离线检查：11/11 PASS；
- 正式运行器离线检查：21/21 PASS；
- 正式分析统一接入检查：`recompute_scene_start_summaries`/`build_views` 对 A/D 统一得到两 True，敏感性计数 1/1/1；
- 本轮未运行全库无关回归，未启动场景。

## 7. 对正式 240 项的影响

1. **是确定性问题还是偶发？**  
   机制是确定的：只要 cycle0 在 capture 时有效、但在 frame 发布时样本年龄超过 100 ms，frame 就会被标记 stale，context 就会丢失 task start sim time。是否发生取决于首个策略周期的 decision-to-publication 延迟；A 发生了，D 没有。所以是**确定性的记录语义缺陷 + 偶发的延迟触发**，不是每次必现，也不是纯随机数据损坏。

2. **四组概率/路径是否不同？**  
   A/B/C/D 共用 `StateRL::runModel`、`captureSimClock`、`writeRtFrame` 和分析路径；代码上没有组别特有分支。历史扫描中 A 有 false 记录、B/C/D 未出现，但 A 往往是批次首个/唯一组，存在冷启动/首个进程混杂；没有证据支持 A 组天然更易发生，也没有证据支持 B/C/D 免疫。正式 240 的首项是 B，不能据此推断 B 更安全。

3. **修正是否改变控制行为/起立准备/研究变量？**  
   本轮正式分析接入的是**只读记录/派生分析**修正：不改控制、起立准入、阈值、地图、模型、四组语义；不移动任务起点 cycle；不插值；不放宽新鲜度/身份要求。它只把 controller trace 中同一 cycle0 的 decision-time 有效性保留下来，正式入口对 A/B/C/D 统一使用。原始发布态仍保留在 `task_start_clock` 和 `raw_invalid_clock_frames` 中，供敏感性对照。

4. **当前缺口怎样影响耗时、安全分类和正式分母？**  
   - 原始发布态对照：受影响 run 的 task start sim time 缺失，elapsed/valid coverage/部分 short-reverse 不可判；安全分类至少因起点缺失为 UNKNOWN，已确认的 collision/fall 仍按原证据处理。
   - 统一正式分析：同一 cycle0 的 decision-time sim time 恢复耗时和 coverage；collision/fall 覆盖由“起点缺失”变为完整时，安全分类随之按证据重算。A 的具体变化是 `UNKNOWN → True`，同时足端撞击 1 次单独保留。所有组别使用同一规则，不能按组别/终局/结果是否有利选择启用。
   - 分母：原始对照会把受影响计划项计入 SAFETY_UNKNOWN；统一分析若其他安全维度完整，则进入 CONFIRMED_SAFE_COMPLETE，改变组内下界。正式汇总的 `reconstruction_sensitivity` 同时报告两口径差异，不隐藏原始 UNKNOWN。

5. **是否需要另行授权一次有限开发补证？**  
   - 对“记录层 decision-clock 重建”本身：**不需要新场景**。A 的原始 runtime record、context 和 controller ROS trace 已足够；本轮 10/10 离线检查已证明匹配和拒绝路径。
   - 对“控制侧 readiness/warm-up”或“在新运行中验证决策前等待”：需要一次有限开发运行（仍为 m02，A 组或指定组；不使用留出图；1 次实际场景启动；独立授权）。当前两项开发补证额度已用尽，不能自行追加。
   - Director已同意接入 analysis-side 重建；只需对 A 及后续正式记录统一重算，不需要新增运行。若 Director 进一步要求从控制侧消除，再另行授权 1 次开发补证。

## 8. 交付与环境

- 原始 A/D 记录未改写；A 的 runtime record/context/terminal/metrics 哈希前后一致。
- 派生材料：`logs/07_formal_preparation/cycle0_clock_review/`；本报告同目录的 [check_results.json](check_results.json)。
- 本轮未启动 MuJoCo/ROS/controller/模型推理，无本轮进程需要关闭；未清理数据；未提交 Git。
- S7-01 已 COMPLETE；后续由 S7-02 承接，240 项仍未授权。

## 9. 通俗总结

1. **为什么 cycle0 时钟无效？**  
   控制器在 cycle0 开始时确实读到了一个有效时钟（7.126 s），但第一次策略周期从读取到写帧用了太久，写帧时这个样本已经过期 108.5 ms，超过 100 ms 新鲜度阈值，所以写进运行记录的帧被标成 stale。记录器只保存帧的“过期”标志，没保存控制器 trace 里“决策时有效”的事实，于是任务起点 sim time 变成空。证据足以确定这个机制；但“第一次推理为什么这么慢”没有单独日志，不能唯一归因。

2. **改了什么或为什么没改？**  
   把已有的只读重建统一接入正式分析：当帧是发布时 stale、但 controller trace 证明同一 cycle0 决策时有效且样本一致时，用同一 cycle0 的 decision-time 时钟恢复耗时/覆盖；原始记录和原始 STALE 状态不变。正式入口对 A/B/C/D 统一使用；没有改控制器、阈值、地图或安全规则。A 从 UNKNOWN 变为 True 的依据是同一 cycle0 时间恢复后 collision/fall/越界覆盖共同满足；足端撞击仍单列。

3. **离线验证结果？**  
   10/10 PASS：原始发布态 A 仍是 UNKNOWN；统一正式分析得到 A/D 两 True、A task start=7.126、elapsed=6.502、short reverse complete、collision/fall False/complete、足端撞击 1 次单列；D 不受影响；错误 session/cycle/sample/age/reset/future 等全部拒绝；真正 fall UNKNOWN 不会被清成 False，确认跌倒时 fall=True、安全任务完成=False。安全、开发、正式运行器定向回归均通过。

4. **还需不需要真实补证？**  
   验证记录层重建不需要新场景；若要改控制器做决策前就绪等待，才需要另行授权 1 次 m02 开发运行（不使用留出图）。当前两项额度已用尽，本轮不自行追加。
