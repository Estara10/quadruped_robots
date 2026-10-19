# B1 指标可用性与覆盖边界

**当前审阅状态：B1 COMPLETE（Director 有条件通过）。** 通过范围与限制见[执行报告](B1_EXECUTION_REPORT.md)。本文件后续按日期保留各批历史指标可用性分析；旧表格或段落中的ACTIVE / INCOMPLETE仅代表当时状态，不覆盖当前审阅结论。任务窗口内的False仅适用于明确完整覆盖及当前判据，不外推到全采集、收尾或一般安全。参数预实验尚未获授权；留出集不用于策略测试或参数选择。

## 最新补充：dev03 D 任务期安全与收尾窗口独立核验（2026-10-15）

本次对同一D原始记录调用生产`calculate()`独立重分析，没有覆盖源JSONL、context、terminal或历史metrics。任务cycle 0–565所附566个物理snapshot均为LIVE v5、同capture、发布覆盖连续、事件锁存完整且无碰撞/跌倒历史溢出；任务前边界及终止周期物理边界匹配。任务窗碰撞、非足端碰撞、足端接触、足端撞击和按当前确认规则的跌倒均可报告为False（接触/撞击计数为0）。任务期一条姿态候选在sim 10.294–10.438 s异常，后于终点前关闭且未确认；因此False意为“当前确认规则未判为跌倒”，不等于无姿态异常或安全成功。

全采集安全覆盖仍不可用：49条终局后的收尾策略记录含STALE物理快照，捕获级附属样本序列有重复/间隔，不能支持全采集无事件结论；任务内LIVE样本各自声明的producer物理覆盖完整，二者范围不同。不能外推任务窗结论至站立/清理。整份run仍VALID / ARRIVED；任务时间、路径、切换和9/9保持审计与旧metrics一致。派生JSON、摘要和timeline见[独立D重分析目录](../../../../logs/04_B1_recording_validation/historical_reanalysis/dev03_D_task_safety_scope_20261015/)。之前“该run碰撞和跌倒均UNKNOWN，因为v5不兼容”的解释是修正前分析状态，现由本段取代；v5在生产路径受支持。

## 历史运行证据：dev03 D 收尾顺序真实核验（安全解释见上方独立重分析）

本次唯一新D运行（[metrics](../../../../logs/04_B1_recording_validation/closeout_runtime_verification/dev03_D_20261015/D/rep_01/6a0e402de1754d9a828704976d8ee32f/metrics.json)、[timeline](../../../../logs/04_B1_recording_validation/closeout_runtime_verification/dev03_D_20261015/D/rep_01/6a0e402de1754d9a828704976d8ee32f/timeline.png)）整份记录为`VALID`，cycle0–565任务窗完整。终止后额外49个策略帧在运行记录中正常保存，但任务耗时、路径、速度、模式边沿和Recovery指标均截至终止cycle565，不包含站立/收尾尾段。任务窗11.300s、路径10.070m、平均速度0.891m/s；18次模式边沿，Recovery 2.790s；9/9保持episode完整通过。最终drain producer=consumer=615，unread loss=0，slot reuse=0，冲突=0。

当时首次生成的metrics将碰撞/跌倒记为UNKNOWN，之后的独立任务窗口重分析已按上述范围修正；整份run和站立观察事实未变，命令消费层仍UNKNOWN。该运行只证明一次记录关闭流程正常，不给出D性能或一般安全结论。早先同场景D整份INVALID及48帧收尾缺口是独立历史记录，没有被修复或覆盖。

下表保留原始八条有效来源运行的历史可用性评估。Director重审后另有两次B尝试，但因帧顺序错误为INVALID，不进入指标汇总。分析不插值、不把缺失记零；`VALID`只说明来源/记录校验通过。下方历史表中dev25 A跌倒`UNKNOWN`已由本页末尾的独立重分析更新；原始指标文件未改写。

之后获准的唯一dev25 A组补证为独立运行：来源与周期记录`VALID`，cycle 0–441连续，终止payload已精确核对，任务窗时钟连续；它更新了公共记录链的运行证据，但不回填或改写原八条及两次INVALID B的指标。

| 指标 | 定义/来源 | 本批可用性 | 影响与边界 |
|---|---|---|---|
| 终局与终点距离 | supervisor原始终局＋终止周期位置，目标半径0.5m；run_context/terminal及生产分析 | 可靠（8条均有终局；距离可从终止摘要/指标复核） | `ARRIVED`表示进入判据，不代表全程无碰撞、无跌倒或在边界内 |
| RL/任务起点至终局经过时间 | 有效sim_time的运动边界差 | 部分可用 | 起点无效/终点无效时不以墙钟替代；C的仿真运动上限为合法终局 |
| 有效连续时间、路径长度、均速/峰速 | 相邻保存帧的时钟有效且周期连续区间；metrics.json | 部分可用 | 缺帧和无效时间区间不桥接；局部路径/速度不等于全程，覆盖比例应同行报告 |
| 保存周期整体频率 | 保存帧cycle-ID跨度÷monotonic跨度；同周期跨度统计；metrics.json | 可计算但周期覆盖部分/UNKNOWN | 本批约50.13–50.60 Hz，周期缺失2–17；不代表无缺口实时频率 |
| 全部模式边沿与方向 | JSONL相邻保存帧mode_before/after及RA状态 | 部分可用 | 保存帧可证实边沿数不受无效sim_time删除；周期缺口使完整转换数未知 |
| 0.5s短反向切换 | 连续、有效sim_time且周期相邻的方向反转窗口 | 部分/不完整 | 本批全部显示`partial_or_unknown`；不能跨缺口拼接或作组间完整比较 |
| Recovery驻留、累计时间、占比 | Recovery连续有效sim_time；占比分子为可计Recovery秒，分母为有效连续运动秒 | 部分可用 | C/D hold audit存在不完整episode；切换缺口/时钟删失不能当0，末端episode右删失 |
| RA风险→模式转换响应 | 同策略周期内部risk条件成立至mode切换的monotonic delta | 部分可用 | 只描述内部决策时间，不是完整避障响应、动作写出、生效或MuJoCo消费延迟 |
| 足端接触/足端撞击/非足端碰撞 | 物理步碰撞authority事件历史及任务边界 | 部分可用 | 正事件可报告；无事件仅在任务窗覆盖完整时才能否定。接触力/碰撞覆盖仍按每run记录 |
| 跌倒 | 物理步fall tracker历史、姿态有效性和覆盖 | 不可用于普遍否定 | 多条运动窗结果为UNKNOWN；未观察事件不能改写为False |
| base越界 | 声明矩形相对base XY的连续物理覆盖 | 不可用作全程否定 | 本批未建立能支持完整base越界否定的充分证据；机器人外形越界更无结论 |
| 安全中止/碰撞终局 | 原始supervisor终局及同身份物理事件链 | 正事件可用，否定按覆盖条件 | 本批无额外安全中止；C为sim-limit；命令发布与物理消费层次分开 |
| 起点实测初态 | cycle0 companion具名q/dq、位姿/速度/IMU；reset probe qpos | 可用作描述性公平性记录 | cycle0精确持久化，组间仍有差异；reset probe不含reset dq，未达严格配对 |
| 正常到达站立收尾 | 实际FIXEDSTAND观察与5s仿真观察记录 | 本批已完成的8条均Complete | 不等于MuJoCo已消费停止命令；终局后观察不计入运动指标 |
| 墙钟保护原因 | supervisor边界、阶段计时和sim/wall推进 | 历史原因仍UNKNOWN | 新运行未复现历史M1 WALL_CLOCK_GUARD_INCOMPLETE，不证明原因解决 |

## 选参准备结论

**目前不建议开始参数比较。** 原计划密集图B首次运行与唯一重试虽到达，但记录均INVALID；后续唯一dev25 A运行中周期合并/写盘链取得一次VALID运行证据。原八条周期仍有缺口，短反向切换覆盖不完整，若干跌倒结论UNKNOWN，C/D保持审计有删失片段；base-bounds生产链只能证明base_link点的记录范围，不能证明整机footprint/足端始终在界内。新A旧metrics中的跌倒字段曾为UNKNOWN；本轮独立重分析依据同run终点前已关闭且未确认的姿态候选，按当前确认规则得到跌倒False，同时保留该姿态异常事件。到达后站立观察仍未完成。该次运行不补足其他组的机制证据，也不构成组间性能比较；B1及参数比较仍需Director后续决定。

## 最新补充：去重安全快照与跌倒历史重分析（2026-10-15）

- 去重帧仍只有一条持久化JSONL记录；调用方现在能从一致重复结果拿回该记录附带的LIVE/MISSING等物理snapshot及独立sample identity。生产组合测试证明：非足端碰撞和确认跌倒仍按原优先级处理，MISSING/STALE/INVALID或空安全证据不通过正常收尾gate。
- dev25 A原始记录的安全快照publisher覆盖完整、同capture且历史无溢出；任务期0.092 s姿态候选在终点前已关闭且从未确认，修复后的独立分析为“按当前规则未判定跌倒”，并保留候选起止时刻。该结果不表示全程姿态正常，也不抹去低高度事件。
- 同一run有一次FL足端撞击`ppt_obstacle_02`，所以不能将无非足端终止碰撞写为无碰撞。原normal-closeout gate未收到安全快照，5 s站立观察未执行；离线健康分支通过不替代运行态收尾验证。
- 派生结果见[独立重分析metrics](../../../../logs/04_B1_recording_validation/historical_reanalysis/dev25_A_safety_review/metrics.json)、[事件摘要](../../../../logs/04_B1_recording_validation/historical_reanalysis/dev25_A_safety_review/README.md)及[生产组合离线用例](../../../../logs/04_B1_recording_validation/offline_merge_validation/dedupe_safety/summary.json)。源run原始记录和旧metrics保持原样。

| 指标/判断 | 最新可用性 | 范围和剩余限制 |
|---|---|---|
| 已保存同一策略帧上的物理安全证据传递 | 离线生产路径已验证 | 一致去重返回已存payload；真实运行态修复尚未补跑验证 |
| dev25 A 跌倒事件生命周期 | 独立重分析：False（当前确认规则） | 一条任务期候选已关闭且未确认；不是“未发生姿态异常”或安全成功 |
| dev25 A 正常到达站立收尾 | 未完成 | 原运行gate因payload未传入而拒绝；新gate只离线验证，5 s站立仍需最小运行补证 |
| dev25 A 碰撞 | 有正足端事件 | 1次FL足端撞击；无非足端终止事件不等于无碰撞，整机足迹安全不在此结论内 |

本轮结论不使B1进入参数比较；周期/时钟缺口、C/D保持删失、其他run的跌倒/越界UNKNOWN以及MuJoCo命令消费限制仍按各自覆盖记录保留。

## Director重审后的实现状态（2026-10-15）

| 项目 | 当前实现/证据 | 对历史指标的影响 |
|---|---|---|
| 风险触发计数 | 枚举仅值1计true，值2为UNKNOWN；八条历史JSONL审计与metrics配对计数一致 | 平地A均为0次风险进入，稀疏C为28次；原JSONL未改写 |
| 周期历史读取 | 有界环按当前实时帧sequence读取，避免新帧先于旧直接快照落盘；溢出/session边界仍显式保留；定向helper测试通过 | 两条B运行发生在最终主循环修复之前，均INVALID；之后的唯一dev25 A运行VALID，周期0–441完整，实际只验证此run的正常读取路径 |
| 策略时钟 | 一致快照读取后再取同一个单调校验时刻，供年龄、未来时间和有效性检查；controller构建/安装通过 | 不追改原八条STALE帧；未来有效运行可用，历史时钟指标仍按原覆盖解释 |
| 安全/越界 | fall历史及任务窗口重核；物理步base-bounds authority与生产reader握手通过，按冻结矩形评估base_link XY点 | 受控探针和INVALID运行的base点证据不等于有效导航结果或全身/足端无越界；未覆盖结论仍UNKNOWN |

重新运行的定向检查：history C++→Python、base-bounds reader及生产握手、motion-safety、risk enum、scene alias、run record 59/59、S1-04生产收尾/分析路径、受影响Python文件编译均通过。S1-04合同fixture曾因未模拟周期历史环而触发预期的fail-closed结果；fixture补入有界历史输入后通过，生产要求未降低。该结果证明针对性工具/记录检查，不证明四组比较所需的全部历史/运行指标完整。

## 最新补充：合并写盘离线验证（2026-10-15）

| 项目 | 当前生产口径 | 离线验证 | 仍需运行确认 |
|---|---|---|---|
| 策略帧写入次序 | 单一受保护的`FrameCapture.drain_history()`入口补读环历史；环帧不超过当前实时快照sequence；session、source sequence、monotonic时间和活动策略cycle必须递增 | 真实生产FrameCapture/record/closeout/analyzer联测验证cycle0到终止、短暂未提交slot重读、持久不一致拒绝、同身份冲突和时钟回退拒绝；dev25 A有效运行保存cycle0–441并对应terminal | 已验证一次正常完整场景记录；真实竞态出现频率和罕见拒绝路径仍未在场景运行中覆盖 |
| 相同帧去重 | session、source sequence相同且原始payload字节一致时只保留一份并计入一致重复数；内容冲突明确拒绝 | 终止实时N与环中N相同内容只落一行；改变N内容会生成merge错误并判记录无效；dev25 A记录计一致重复1896、内容冲突0 | 该run的真实竞态频率不可由累计重复数单独推断；不同负载下频率未知 |
| 环形槽位复用 | `producer_overwritten_count`保留历史含义，表示累计槽位复用；不是数据丢失数 | C++ `publishHistory`产出超过2048容量的数据，Python reader确认消费者读到最早可用槽位时`slot_reuse=2`且无未读损失；FrameCapture跟读2051帧时`slot_reuse=3`、`unread_loss=0`并完整；dev25 A运行中slot reuse和unread loss均为0 | 实时负载下接近容量边界的读取余量尚未实测 |
| 消费者未读丢失 | 由消费者游标与`oldest=max(0, produced-capacity)`比较；落后被覆盖时显式记数量及索引范围，计数不符、reset、session变化和缺slot均拒绝完整性 | 真实C++→Python reader验证溢出区间`[0,2)`、错计数、counter reset、session change、未提交slot；FrameCapture验证未读区间`[0,3)`只计一次且final drain不完整；dev25 A实际运行未读丢失0 | 实际运行未覆盖溢出、reset等异常拒绝路径 |
| 终止与收尾边界 | `task_termination_boundary`只覆盖cycle0至终止帧；`final_closeout_drain`单独覆盖清理阶段最后一次排空 | 离线交错用例及dev25 A实际运行均确认终止payload与引用一致，任务边界完整；分析终点为cycle441，收尾信息未并入运动窗 | 本次到达后站立观察未完成；不是终止帧保存失败，需分开评价收尾状态 |
| 写入失败和清理 | terminal帧写入失败使记录不完整；closeout仍先执行本次进程清理 | 注入终止帧写入错误后，清理callback执行、分析器判`INVALID`，原始失败原因留在合并审计 | 实际文件系统异常尚未实测 |

可复核输出：[`offline_merge_validation/summary.json`](../../../../logs/04_B1_recording_validation/offline_merge_validation/summary.json)、[代表性JSONL/context/terminal/metrics](../../../../logs/04_B1_recording_validation/offline_merge_validation/terminal_race/)与[实际C++发布器→Python reader结果](../../../../logs/04_B1_recording_validation/offline_merge_validation/cpp_reader/summary.json)。这些是隔离离线fixture，不是场景运行。故本补证只提高生产工具链的离线可用性，**不解除当前参数比较阻塞**；原八条运行的周期缺口、无效时钟、保持删失、跌倒/越界UNKNOWN以及两条B INVALID记录均保持原结论。


## 历史评估：dev03 C/D补证初次自动分析（2026-10-15；D后续重分析见下节）

- **C策略与安全窗口：可靠到本run覆盖范围。** cycle0–841连续、全策略时钟有效；完整生产分析给出36条边沿、18个C保持episode全部完整通过、无保持违规。碰撞终局有效，包含RL_calf碰撞及FL/RL足端撞击；因此不能声称无碰撞或成功到达。任务窗跌倒False仅依据完整历史中两个候选关闭且未确认；运动前还存在已关闭的启动确认低高度事件。
- **D任务边界保持机制：有限可用。** 终止边界0–694的策略帧连续且时钟有效；生产`audit_hold_episodes()`单独审计该完整边界，11/11 episode均完整通过。该范围支持H机制在D任务期的功能观察。
- **D整份run指标/安全：不可用。** 到达后站立观察中最终drain发现producer=743、consumer=695，48帧未持久化，整份记录仍INVALID；批次分析器将组级指标与任务期碰撞/跌倒状态保留UNKNOWN。任务边界单项审计不覆盖缺失的closeout尾帧，不改写VALIDITY。
- **站立收尾：D本次实际完成。** 226个物理样本覆盖5.024仿真秒，controller反馈FIXEDSTAND且支撑/姿态样本正常；这是独立的收尾证据，即便策略记录整份INVALID仍可陈述。C因非足端碰撞没有走正常站立观察。controller状态和物理快照不等于MuJoCo command consumption确认。
- **安全解释：** C/D运动前各有一个低高度episode，分别在sim 4.690s和5.036s关闭，随后策略任务才开始；历史保留，不计为任务期终局。不能把它们称作无异常启动。

原始日志、自动分析和限定审计见[本批目录](../../../../logs/04_B1_recording_validation/cd_runtime_verification/dev03_C_D_20261015_170521/)、[D任务边界保持审计](../../../../logs/04_B1_recording_validation/cd_runtime_verification/dev03_C_D_20261015_170521/D_task_window_hold_audit.json)及[执行报告最新章节](B1_EXECUTION_REPORT.md)。两次单次诊断不构成有效C/D整体对照；（历史状态：当时B1保持ACTIVE / INCOMPLETE并等待审阅；现已获有条件通过。参数预实验仍须另行授权。）

## 最新补充：收尾顺序修正与 dev03 D 任务窗口重分析（2026-10-15）

运动终局时冻结任务边界，writer保持打开。生产进程按MuJoCo、controller/ROS及辅助会话顺序退出并确认后，`FrameCapture.finish()`才最终排空producer历史并关闭记录器。收尾策略帧与终端边界在同一JSONL，但任务分析仍只到终止cycle。碰撞/跌倒物理观察、站立观察和清理事实继续单独保存。

生产分析器仍把D整份记录标为`INVALID / policy_cycle_history_incomplete`，并新增独立的任务窗口完整性结果。只有终止边界明确完整、cycle0到终止帧连续、时间和身份有效、终止payload匹配、缺失只位于边界之后的closeout尾段时，才允许计算任务窗口指标；任何任务期缺口、冲突、无效时钟或无法定位的丢失仍输出UNKNOWN。此规则是通用判断，不是为单个run添加例外。

| D指标 | 独立生产分析 | 适用范围 |
|---|---:|---|
| 原始终局 / 整份有效性 | ARRIVED / INVALID | 48个closeout producer条目未持久化；不将ARRIVED改称整份记录有效 |
| 任务窗口 | cycle0–694，695帧；无周期缺口或无效策略时钟 | 任务终止payload与session/cycle/sequence/monotonic身份一致 |
| 到达与移动 | 距目标0.4982m；13.880s；有效连续13.880s；路径10.605m；均速0.764m/s | 任务起点cycle0至终止cycle694，不含站立和清理 |
| 模式与保持 | 22条模式边沿；A→R 11次、R→A 11次；Recovery 4.584s，占33.03%；11/11保持episode通过 | 完整任务周期和仿真时间覆盖 |
| 安全事件 | 非足端碰撞False；1次RL足端撞击/接触，约0.012s；跌倒按当前规则False | 完整任务期物理覆盖；两个已关闭未确认姿态候选仍作为异常事件报告，不是无姿态异常或安全成功 |
| 站立观察 | 5.024仿真秒、226个物理样本 | 独立收尾证据，不计入任务耗时或路径；命令的MuJoCo消费仍UNKNOWN |

“非足端碰撞False”不能简写成“无碰撞”：任务期间有RL足端接触/撞击`ppt_obstacle_02`。任务窗口指标从独立生产分析器结果得出，但整份记录仍无效，不能当作有效完整run或正式结果。派生文件见[D任务窗口重分析](../../../../logs/04_B1_recording_validation/historical_reanalysis/dev03_D_closeout_review/README.md)及[metrics JSON](../../../../logs/04_B1_recording_validation/historical_reanalysis/dev03_D_closeout_review/metrics_task_window.json)；原始自动metrics与记录未覆盖。新关闭顺序仍只有离线验证，未由新的运行验证。
