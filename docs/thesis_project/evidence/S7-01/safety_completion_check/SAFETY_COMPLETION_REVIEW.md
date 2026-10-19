# S7-01 安全任务完成分类定向复核

状态：**定向离线核对完成，提交 Director 审阅；S7-01 保持 ACTIVE / PREPARATION。240 项正式运行未授权、未启动。**

本复核只针对正式主指标“安全任务完成”的字段、窗口、覆盖和分类一致性。原始 JSONL、context、terminal 与旧 metrics 均保持不变；本目录保存人工预期及独立派生结果。

## 1. 先行分类定义与字段来源

运动任务窗口为首次真实策略帧 `rl_entered=1 && rl_active=1` 至 supervisor 任务终止帧，首尾均含。生产源为 `run_context.lifecycle.task_start` / `policy_active_start`、`lifecycle.terminal_frame_ref` 和 `terminal.run_terminal_frame_ref`；分析边界在 `metrics.rl_task_metric_boundary`。不是以首次位移达到 0.10 m 起算。若起点、终止周期或身份无法配对，记录边界不完整，不通过后续帧回填。

| 指标/维度 | 生产字段 | 时间范围 | 需要的覆盖/身份证据 | UNKNOWN / 无效条件 |
|---|---|---|---|---|
| 原始任务终局 | `run_context.result_contract.classification`; `terminal.run_terminal_result`; 派生 `metrics.terminal_result` | supervisor 声明的任务终止时刻；保留各来源原值 | 同一 `run_id`；context、terminal 与策略终止帧身份一致 | 原始值互相冲突时保留各值并标终局冲突；不得改写 supervisor 终局 |
| 记录有效性 | `metrics.record_validity`, `record_validity_reasons`; `run_record.summarize_record()` | 全记录结构与来源验证；用于准入，不替代任务安全判断 | session/周期连续性、唯一终止帧、record schema 和关闭事实满足生产合同 | INVALID/关键身份冲突不得转成安全成功或安全失败；它是工程/证据状态 |
| 任务边界 | `metrics.rl_task_metric_boundary`; `lifecycle.task_start`; terminal frame refs | cycle0/首个active策略周期至终止周期（含） | 同一 run/session；cycle0及终止帧完整、序列/时间身份可核对 | 首帧缺失、周期缺口、终止帧不完整或 ref 不一致时边界不完整；不以较晚周期替代 |
| 非足端障碍碰撞 | `metrics.task_window_safety.nonfoot_collision_failure`, `.collision`, `.collision_events`, `.coverage_complete_for_motion_interval`, `.coverage_evidence` | 每个物理事件自身的发生步/仿真时间；限制在任务边界 | v4/v5权威物理快照、publisher覆盖、无历史溢出、无未分类接触；同capture/场景/模型 | 无事件但覆盖不完整、unknown contact、身份不符或事件跨边界不能定位时 UNKNOWN；足端事件不计为非足端失败 |
| 跌倒 | `metrics.task_window_safety.fall`, `.fall_events`, `.fall_lifecycle_assessments`, `.fall_event_timing_conflicts` | 以 producer 的事件 start/confirmation/close 时间或 physics step 归属；以任务终止边界重建当时 lifecycle | fall history完整、无overflow、姿态有效、身份一致；确认时刻/步落在任务窗口 | 无效姿态、历史溢出、周期/时间冲突、确认/关闭无法按事件合同定位时 UNKNOWN；不能用终止后关闭抹掉任务内确认 |
| 系统安全中止 | 原始终局 + `terminal.external_safety_events`; `result_contract.external_safety_boundary` | 任务运行期间的安全中止；准备期事件单独分类 | 明确安全终局、controller/session/run关联；不要求碰撞/跌倒子指标可判才认定没有完成 | 只有无关/身份不明的外部安全事件或任务起点未建立时不推断为任务期中止；来源冲突标工程/边界异常 |
| base_link 越界 | `metrics.declared_boundary_physics_observation.status`; `terminal.base_bounds_coverage`及任务前/终止累计计数 | task_start物理步至终止物理步 | 同 run/capture/scene/model；连续 PhysicsLoop 覆盖；边界定义与base pose有效 | 缺边界快照、coverage断裂、身份不符或累计增量不能定位时 UNKNOWN；只判base_link XY，不代表整机足迹或物理步间连续轨迹 |
| 足端接触/撞击（独立报告） | `metrics.task_window_safety.foot_contact_count`, `.foot_impact_count`, `.foot_*_events` | 事件自身物理步/时间，限任务窗 | 足端历史与力数据覆盖；对象/足端身份有效 | 不完整历史或 `foot_impact_unknown_overlaps_motion` 时事件数量/否定判断 UNKNOWN；不自动变成非足端碰撞失败 |
| 安全任务完成 | 本次薄分类层：原始终局 + `record_validity` + 上述四个安全维度及覆盖 | 固定任务时限内、运动任务窗口 | ARRIVED、记录/终止帧可用；碰撞、跌倒、系统中止、base_link边界均足以判断 | 到达但任一必要安全判断 UNKNOWN→UNKNOWN；记录/身份无效→UNKNOWN/无效；有效任务未到达→False。命令消费不纳入该复合条件 |
| 站立观察与清理（独立维度） | `terminal.cleanup_events`, `normal_shutdown`, `shutdown_complete`, `process_exit_code`; `context.lifecycle.standing_observation`, `cleanup_processes`, `closeout_errors` | 任务终止后 | 各自观察/退出事实和进程来源 | 未观察/未确认时 UNKNOWN 或 incomplete；不得把站立或关闭事件并入运动安全窗口 |
| MuJoCo命令消费 | 当前 `command_consumption=UNKNOWN` | 命令发送至仿真执行时间（尚无消费确认） | 同一命令的消费端确认目前缺失 | 保持 UNKNOWN；不阻断按现方法定义的主指标，也不声称物理消费已确认 |

复合分类不把所有维度压成一个无语义布尔值。有效且安全证据充分的 ARRIVED 为 `True`；有效但未到达为 `False`；ARRIVED且任一必要安全证据未知为 `UNKNOWN`；已确认任务期安全否决项使安全完成为 `False`，同时检查终局合同冲突。INVALID/关键身份冲突单独保留，不当作安全失败。ARRIVED、record VALID、无日志事件任一单独都不足以判 True。

## 2. 人工预期案例（在运行分类器前冻结）

人工预期先依据原始 context、terminal 和已有 metrics 登记；下列 run_id 不会被分类器输出反向选取。真实 run 的人工预期仅复用已有派生字段和原始身份，不改原数据。

| 案例 | run_id / 类型 | 人工预期 | 判定依据（分类器运行前） |
|---|---|---|---|
| R1 | `16465c346b8a4fd1b85c4548169fd6a2`，T2 48项，ARRIVED | True | context/terminal均ARRIVED、VALID、cycle0–376、非足端碰撞False且覆盖完整、跌倒False且历史完整、base_link物理步越界增量0 |
| R2 | `0f148c54215e409fbbcacb23955510de`，B1保留记录，ARRIVED | UNKNOWN | ARRIVED/VALID；任务期base bounds不可判且终止frame记录ref不完整，不能仅凭ARRIVED/VALID判True。旧metrics的fall字段与当前生产重算不同，见第3节 |
| R3 | `b40d5daa14b1403a8d0f050df55cd683`，T2 48项，碰撞终止 | False | 有效任务、原始终局COLLISION_TERMINATION，确认非足端碰撞 |
| R4 | `2547e48a82304e9ca0aac05ce518e92d`，T2 180项，跌倒终止 | False | 有效任务、确认跌倒事件位于任务窗口、原始终局FALL_TERMINATION |
| R5 | `c91ce19ba1d14b53b0b14ddaf9207d97`，T2 48项，仿真限时 | False | 有效任务且固定任务期限内未ARRIVED；不能因安全子项均False称其成功 |
| R6 | `1699cb934b7a4c73b2a666867eadf353`，T2 180项，系统安全中止 | False | policy task_start cycle0已记录，随后本run/controller绑定的POLICY_SAFETY_VETO及FAULTED cycle147；即使其物理安全子项UNKNOWN也不是安全完成 |
| R7 | `fd11b43b53d04ff1ab26cf3c0e6274f4`，T2 48项，足端撞击 | True | ARRIVED/VALID；足端接触和撞击各1，非足端失败False、碰撞/跌倒覆盖完整且False、base bounds全步覆盖且界内；足端事件单列 |
| R8 | `9348a1844e444336b3c6dfa48c2bf2d5`，T2 180项，ARRIVED但越界 | False | ARRIVED/VALID；生产边界观察记录任务窗口base_link越界物理步 |
| R9 | `8fecf8c227d54c199a63b7530e51eab0`，T1历史记录，跌倒终止 | False；历史派生差异单列 | 原始context与terminal均FALL_TERMINATION；当前生产读取重建出confirmation sim 13.862s，早于terminal sim 13.874s，fall=True。保存的旧metrics曾列`INVALID_TERMINAL_SAFETY_CONFLICT`/fall UNKNOWN；该旧派生不改写，也不遮盖当前从原始事件记录重评的结果 |
| C1 | 构造：任务前确认且已关闭的低姿态事件 | True（在其余输入充分且到达时） | adapter只消费生产`task_window_safety`，不把任务前事件计入；真实同类边界另由R1核对 |
| C2 | 构造：终止后才发生/确认的事件 | True（在其余输入充分且到达时） | 任务终止后事件不污染任务窗；若事件合同无法按自身时间定位，则应UNKNOWN而非猜测 |
| C3 | 构造：ARRIVED但非足端碰撞确认 | False + terminal conflict flag | 安全否决为True，复合安全完成False；若supervisor仍写ARRIVED，单列合同冲突，不改原终局 |
| C4 | 构造：base bounds覆盖缺口或累计变化不能定位 | UNKNOWN | 碰撞/跌倒可单独保持False；边界为UNKNOWN，复合分类不能判True |
| C5 | 构造：终止frame ref的session/cycle/sequence与记录不匹配 | INVALID / UNKNOWN | 关键身份冲突阻断复合结论；不能把raw ARRIVED映射为False或True |
| C6 | 构造：记录有效性为INVALID | UNKNOWN，记录状态保留INVALID | 工程无效与安全失败分开，不自动折算为False |
| C7 | 构造：ARRIVED但跌倒历史/覆盖UNKNOWN | UNKNOWN | 必要安全维度未知时，不把有效记录或ARRIVED直接判为安全完成 |
| C8 | 构造：任务期确认跌倒，但原始终局仍ARRIVED | False + terminal conflict flag | 保留原始终局；复合结果否决并显式报告终局冲突 |
| C9 | 构造：任务期同会话安全中止，周期在任务边界内 | False + terminal conflict flag | 安全中止否决安全完成；物理碰撞/跌倒仍可各自UNKNOWN |
| C10 | 构造：同会话安全中止事件位于终止周期之后 | True（其余证据充分时） | 终止后事件不污染运动任务窗口 |
| C11 | 构造：SYSTEM_SAFETY_ABORT且物理子项UNKNOWN | False | 已开始任务且明确安全中止足以否决安全完成；不外推为无碰撞或无跌倒 |

### 原始案例位置

- R1/R3/R5/R7：`logs/02_T2_minimum_hold/combined_check_48/20261019_1358_01_seed2026101901/`，分别见上表及对应`runs/.../<run_id>/`目录。
- R4/R6/R8：`logs/02_T2_minimum_hold/hold_scan_180/20261018_192507_t2scan180/runs/.../<run_id>/`。
- R2：`logs/04_B1_recording_validation/runs/m1dod01_D/D/rep_01/<run_id>/`。
- R9：`logs/01_T1_exit_threshold/legacy_coarse_S6_12/b_exit_coarse/20261015_192706_3/runs/083_m1dfd01_x05_r1/result/A/rep_01/<run_id>/`。
- C1–C11为离线构造输入，单独标明，绝不记为已运行run。

R1是现成的任务前事件样本：producer fall history显示确认事件从sim 0.250s持续至4.978s并关闭，首个有效策略周期/任务起点为sim 6.636s；当前生产任务窗分析仍为fall=False，未让准备期历史污染任务期。当前没有现成真实样本证明任务后才确认并应排除的跌倒、ARRIVED且任务期fall UNKNOWN、ARRIVED且已确认非足端碰撞，或终止ref跨session/sequence冲突；分别用构造输入覆盖分类边界。R2有真实ARRIVED但base bounds为UNKNOWN，且记录terminal frame ref不完整。R2保存的旧metrics为fall UNKNOWN，而当前生产`calculate()`读取同一原始记录时认定该candidate已在任务内关闭且未确认、无剩余active candidate，故fall=False；主分类仍为UNKNOWN，因为越界否定证据与终止frame ref不足。R9也存在旧派生与当前重读差异；两份旧metrics均保留、不覆盖，本次新结果单独落盘。此处是派生分析版本/口径差异，不能据此改写原始事实或笼统称原始记录损坏。

## 3. 定向核验结果

复合适配在`scripts/safety_task_completion.py`；生产`scripts/s1_analyze_run.py::calculate()`调用现有`motion_safety()`和`physical_base_bounds_window()`后生成`task_source_identity`与`safety_task_completion`，同时在 run summary 暴露`safety_task_completion_status`。它只合并生产结果，不重做碰撞、跌倒或越界检测。`scripts/check_s7_safety_completion.py`通过生产`calculate()`重读9条选定真实记录；另对生产`motion_safety()`使用R1策略帧副本及一份明确标记的终止后物理快照构造输入。原始记录未写入，旧metrics未覆盖。

人工案例结果（这是为覆盖分支挑选的定向样本，不是总体统计）：

| 分类 | 案例 | 数量 | 说明 |
|---|---|---:|---|
| `True` | R1、R7 | 2 | R1正常到达且任务窗证据完整；R7到达并有足端接触/撞击，仍无非足端失败，足端独立报告 |
| `False` | R3、R4、R5、R6、R8、R9 | 6 | 碰撞终止、跌倒终止、仿真限时、安全中止、ARRIVED但base_link越界、跌倒终局；R8保留原始ARRIVED并附终局/安全合同冲突标记 |
| `UNKNOWN` | R2 | 1 | ARRIVED，但base_link范围证据UNKNOWN且终止frame记录引用不完整 |

这些9条真实记录来自不同开发批次且经类别选取，计数仅证明分支可追溯，不是成功率、组间排名或性能估计。R1的准备期fall history在sim 0.250–4.978s已经关闭，task_start为6.636s；当前生产任务窗`fall=False`，说明任务前已结束的确认事件没有污染任务期。R7的1次FL足端接触和1次足端撞击为任务期事件，但`nonfoot_collision_failure=False`且碰撞覆盖完整，按已审阅规则仍为`True`，足端事件单列。

R2和R9保存的旧`metrics.json`与当前生产分析器从相同原始`runtime_record.jsonl`/context重算的跌倒字段不同：R2旧值`UNKNOWN`、当前重算`False`（事件在任务内关闭但未确认，完整历史无active候选）；R9旧值`UNKNOWN`并派生`INVALID_TERMINAL_SAFETY_CONFLICT`、当前重算确认跌倒`True`（confirmation sim 13.862s、terminal sim 13.874s），原始context与terminal均为`FALL_TERMINATION`。本轮只把新结果写到独立派生文件，不覆盖旧metrics；这是旧派生与当前读取/分析路径的差异，不是原始数据被改写。R9复合分类为`False`，且当前原始终局一致，没有终局冲突。

构造适配用例C1–C11逐项通过：任务前已结束事件不污染任务窗；任务后跌倒由生产`motion_safety()`副本核验排除、同会话终止周期之后的安全中止不污染任务窗；ARRIVED+确认非足端碰撞、确认跌倒或任务周期内安全中止→False并标合同冲突；越界覆盖不足或终止ref身份冲突→UNKNOWN；INVALID记录保留INVALID并使复合结果UNKNOWN；ARRIVED+跌倒UNKNOWN→UNKNOWN；明确的系统安全中止使已开始任务为False，同时碰撞/跌倒子项可保留UNKNOWN。每个分类构造输入重复运行结果一致。

另有生产`motion_safety()`副本核验P1–P5：P1在task_end=14.156s后追加sim=14.176s发布、确认时间14.166s的跌倒事件，fall仍为False且覆盖完整；P2在task窗内出现未分类接触时collision与nonfoot为UNKNOWN，但fall仍False；P3任务窗无效姿态使fall为UNKNOWN；P4历史溢出与P5 physics publisher coverage gap均使否定结论为UNKNOWN、覆盖不完整。它们均在R1策略/物理记录副本上添加明示构造字段，不是新运行证据。

### UNKNOWN分母及情景方向核对

安全任务完成率按全部计划项给缺失结果情景界：`[已确认True数/N, (已确认True数+UNKNOWN数)/N]`；INVALID/未启动仍独立列示，不偷换为安全失败。每组known-case分母与全计划分母同时保留；本次人工样本刻意抽取，不用作率估计。

定向公式样例均按“候选−参考”核算：

| 样例 | 指标方向 | 候选/参考(已知正向、已知反向、UNKNOWN) | 候选-参考数学范围 | 最不利 / 最有利 |
|---|---|---|---|---|
| U1 | 安全完成率，越高越好 | (2,1,1)/(1,2,1) | [0.00, 0.50] | 0.00 / 0.50 |
| U2 | 安全完成率，越高越好 | (2,2,0)/(1,1,2) | [−0.25, 0.25] | −0.25 / 0.25 |
| U3 | 安全完成率，越高越好 | (2,1,1)/(2,2,0) | [0.00, 0.25] | 0.00 / 0.25 |
| U4 | 安全完成率，无UNKNOWN | (3,1,0)/(2,2,0) | [0.25, 0.25] | 0.25 / 0.25 |
| U5 | 非足端碰撞率，越低越好 | (1,2,1)/(1,1,2) | [−0.50, 0.25] | 0.25 / −0.50 |

所有范围按`lower=candidate_lower−reference_upper`、`upper=candidate_upper−reference_lower`计算。U5最不利取数学上界；U1–U4最不利取数学下界。它们是UNKNOWN敏感性边界，不是bootstrap置信区间。正式汇总仍先在地图内汇总重复，再地图等权；不会因可判run数不同悄悄改变地图权重。

机器输出：[分类逐案JSON](../../../../../logs/07_formal_preparation/safety_completion_check/classification_check_results.json)、[分类对照CSV](../../../../../logs/07_formal_preparation/safety_completion_check/classification_check.csv)。脚本退出状态`PASS`：真实9例及构造11例均符合人工预期；生产终止后边界检查通过；UNKNOWN差值方向/范围检查通过；三个Python文件编译通过。首次检查曾因检查脚本把额外JSON列写入固定列CSV而失败，修正导出字段后相同案例通过；失败是派生检查脚本输出错误，不影响原数据或生产记录。

## 4. 结论与限制

**结论：**正式主指标可通过当前薄分类层在既有生产字段上做一致、可追溯的`True / False / UNKNOWN`分类；记录有效性/字段冲突、任务安全维度、组合指标和站立/清理状态彼此分开。安全中止依据任务周期和session归属；任务后安全中止不污染运动窗口。没有发现会改变本次所选真实案例主分类的适配器错误；但R2/R9旧metrics与当前生产重算的跌倒字段不同，正式汇总必须明确用冻结的同一分析器重算派生指标，保留原metrics及版本说明，不能混合新旧派生口径。

仍无真实样本覆盖：ARRIVED且fall UNKNOWN、ARRIVED且确认非足端碰撞、任务终止后确认事件、终止frame ref错配，以及base bounds累积覆盖不足的完整真实例。这些已用明确构造输入核对分类边界，不是实际运行验证。R6系统安全中止子项的物理碰撞/跌倒/边界为UNKNOWN，主分类False只由“已开始任务且明确安全中止、未ARRIVED”支持，不外推为无碰撞或无跌倒。base_link逐物理步界内不是整机足迹/步间连续界内；MuJoCo命令消费仍UNKNOWN且不作为当前主指标阻塞。

建议正式240项开始前把本薄分类层和结果分母接入经审阅的正式派发/分析流程；本轮没有实现完整运行器，也没有新增场景运行、参数变更或历史批次全量重分析。此离线样本不重排名、不改参数、不代表240项正式结果。
