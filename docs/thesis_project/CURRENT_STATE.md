# 当前状态

## 今日收尾：T1粗选暂停，等待下一轮指令（2026-10-15）

用户要求今日结束并保存、提交Git及推送GitHub。T1仍为唯一ACTIVE / INCOMPLETE任务，96项中66项有运行、累计68次attempt、65项有效任务结果、1项安全中止且任务边界不完整、30项未启动。第66项日志为ray_reason=writer_in_progress_or_unarmed，暂未区分瞬时读写交错与持续来源异常；碰撞/跌倒UNKNOWN，原始记录保留。下次先定向排查射线读取及中止边界，不自动重试/续跑、不细选、不用留出集。此前提示词的续跑许可已随故障停批耗止，需下一轮指令。详细记录及轻量批次快照见[今日收尾](evidence/S6-12/END_OF_DAY_20261015.md)。本轮未启动仿真或测试。


## 当前任务：T1 B组退出阈值预实验（原S6-12，2026-10-15）

用户同意使用T1作为当前任务名称，整体仍处于S6参数预实验阶段。[tasks/T1.md](tasks/T1.md)为唯一ACTIVE任务；旧S6-12任务和提示词仅作转向入口，日志/脚本/冻结计划/evidence/S6-12路径保留。T2、T3只是后续命名建议，不创建或激活。

最新续跑：计划项2一次补做VALID / ARRIVED后，按冻结顺序继续至第66项。第66项原始记录为`SYSTEM_SAFETY_ABORT`，PASSIVE转换与清理已确认，但终止周期/物理窗口边界缺失，碰撞和跌倒窗口分析为UNKNOWN，故按记录覆盖停止规则停批。当前96项计划中66项曾尝试、68次attempt、65项VALID_TASK_RESULT、1项安全覆盖UNKNOWN停止、30项未启动。首项有效结果和两条历史失败均保留；没有重试合法终局，无参数修改、细选、留出集使用或Git提交。详见[T1任务](tasks/T1.md)、[最新执行报告](evidence/S6-12/S6-12_EXECUTION_REPORT.md)和[冻结批次目录](../../logs/s6_12/b_exit_coarse/20261015_192706_3/)。T1保持ACTIVE / INCOMPLETE，等待Director审阅；下文较早S6-12内容为历史记录。


## 历史审阅与授权：S6-12准入失败原因更正及离线修复（2026-10-15）

Director核对第2项RL准入原始边界快照为`INVALID / future_timestamp`、age=-163750 ns；独立的`observed_monotonic_ns`不是该reader校验时刻，terminal中更晚的STALE也不是准入根因。生产reader读一致快照后才取默认校验时刻，已通过定向离线竞态、future/stale、身份与格式拒绝及pre-RL gate验证；受影响Python文件编译通过。当前没有授权仿真或批次续跑，也不改参数或提交Git。保留首项有效超时、第2项INVALID及全部原始数据；仍为96项计划、2项尝试、94项未启动、历史3次attempt、有效任务结果1条。S6-12为唯一ACTIVE / INCOMPLETE任务；修复后的运行读取未验证，是否补做计划项2和续跑由Director决定。详见[reader离线验证说明](evidence/S6-12/S6-12_BASE_BOUNDS_READER_OFFLINE.md)、[S6-12执行报告](evidence/S6-12/S6-12_EXECUTION_REPORT.md)及[执行器提示词](S6-12_EXECUTOR_PROMPT.md)。下文旧STALE归因以本节更正为准。


## 最新授权：S6-12权限环境核对与冻结计划续跑（2026-10-15）

历史续跑事实：用户授权保留原GLFW失败、在同环境补做首项一次，再继续冻结顺序；首项VALID / DIAGNOSTIC_SIM_TIME_LIMIT，计划项2为PREPARATION_FAILURE / INVALID，故停止。**该段当时把计划项2概括为STALE，现已由本文件顶部更正：准入快照是future_timestamp；稍后terminal的STALE属于独立覆盖检查。** 96项计划中2项曾尝试、94项未启动；含历史首错共3次attempt，仅1条有效阈值结果。GLFW权限环境、结果与清理证据见[S6-12执行报告](evidence/S6-12/S6-12_EXECUTION_REPORT.md)。


## 历史状态：S6-12首次粗选尝试（2026-10-15；当前状态见顶部）

[S6-12](tasks/S6-12.md)仍为唯一ACTIVE任务。此处是续跑前历史快照：96项计划冻结后第一次MuJoCo启动在GLFW初始化处失败，首项INVALID、其余95项未启动；当时按规则停止、未重试。随后Director另行授权保留该失败并在一致执行环境补做首项，再继续其余项；最新状态见顶部及[S6-12执行报告](evidence/S6-12/S6-12_EXECUTION_REPORT.md)。


## 历史规划草案：S6-12 B组退出阈值预实验（2026-10-15；已被执行授权取代）

用户指定退出阈值从-0.05到-0.19，每档降低0.02；最初建立[S6-12计划](tasks/S6-12.md)。-0.05为零滞回A参考，其余七档为B，进入阈值固定-0.05且关闭保持。该草案曾计划六张开发图、每档两次，粗选共96条（A12、B84）；之后任务获准并已部分执行，当前状态见本文件顶部。留出集不参与选参。


## 最新审阅状态：B1 COMPLETE（Director 有条件通过，2026-10-15）

B1独立于S系列。Director有条件通过范围包括策略周期记录、去重、有界历史读取、最终排空与终止帧关联；运动、站立和清理窗口分离；去重路径安全证据传递及碰撞/跌倒覆盖分析；C/D限定保持审计；以及dev03 D一次真实收尾验证。该运行任务cycle 0–565共566帧、收尾49帧，最终producer=consumer=615，整份记录VALID，站立观察5.022仿真秒，9个保持episode通过。同一D记录独立重分析确认任务窗口覆盖完整，按当前规则碰撞/跌倒为False、足端接触/撞击为0；关闭未确认姿态候选和异常事实保留。

限制仍包括：全采集及收尾安全覆盖不足，任务期False不代表无姿态异常、普遍安全或实机安全；历史失败、INVALID、缺帧、无效时钟、UNKNOWN、初态差异和保持删失均保留。base_link点不等于整机足迹始终在界内；命令写出或状态确认不等于MuJoCo已消费。单次/不同批次结果不能支持正式成功率、性能排名或最优参数结论。M1地图及30/30开发/留出划分不变，留出集未用于策略测试或参数选择。S6整体仍未完成。当前没有已授权的下一项Active Task；仅建议后续规划开发集阈值与保持时间预实验，不创建任务、不开始比较。详见[B1执行报告](evidence/B1/B1_EXECUTION_REPORT.md)、[指标可用性](evidence/B1/B1_METRIC_READINESS.md)、[运行矩阵](evidence/B1/B1_RUN_MATRIX.md)及[独立D重分析](../../logs/b1/historical_reanalysis/dev03_D_task_safety_scope_20261015/README.md)。


## 历史进展：B1 dev03 D 组真实收尾核验（当前审阅状态见顶部）

（历史运行事实；当前B1状态及该运行的最新解释见顶部。）按一次性授权，在`dev03.xml / m1ssd01`完成D组真实收尾核验，固定E=-0.05、X=-0.08、H=0.30仿真秒和原目标/时限。终止cycle565完整持久化；任务cycle0–565连续，之后49个收尾策略帧单独保存。进程退出后producer=consumer=615，无未读损失、槽位复用、冲突或closeout错误，整份记录VALID。ARRIVED后站立观察5.022仿真秒/238样本完成；保持审计9/9完整通过。后续独立重分析确认任务期安全窗口覆盖完整、碰撞/足端接触/撞击/跌倒按当前规则均False；一个关闭未确认的姿态候选仍记录，全采集安全覆盖仍不完整。MuJoCo命令消费未确认。详见[B1执行报告](evidence/B1/B1_EXECUTION_REPORT.md)、[运行矩阵](evidence/B1/B1_RUN_MATRIX.md)和[独立D重分析](../../logs/b1/historical_reanalysis/dev03_D_task_safety_scope_20261015/README.md)。（历史状态：当时尚待Director审阅且B1为ACTIVE / INCOMPLETE；现已COMPLETE（Director有条件通过）。）


## 本轮前进展：B1 收尾生命周期离线定向修正（历史，2026-10-15）

该阶段修正了任务边界封口与最终策略历史排空顺序，完成生产收尾路径离线联测，并只读重分析了更早的dev03 D原始记录。那份历史D记录仍为INVALID，终止后缺48条生产记录；它没有被本次新run修复或覆盖。详见[B1执行报告](evidence/B1/B1_EXECUTION_REPORT.md)、[历史D派生分析](../../logs/b1/historical_reanalysis/dev03_D_closeout_review/README.md)和[离线联测摘要](../../logs/b1/offline_closeout_lifecycle_20261015/summary.json)。其后的新关闭顺序真实运行证据见本节顶部。


## B1既有 C/D 运行结果（本轮收尾修正前的背景，2026-10-15）

**历史进展（收尾修正前）：**dev03 C/D各一次补证已结束。C为VALID / COLLISION_TERMINATION，cycle0–841连续且无无效策略时钟帧；18个保持episode全部完整通过，终局为RL calf与ppt_obstacle_02非足端碰撞，未进入正常站立观察。D supervisor终局ARRIVED、距目标0.4982m，完成约5.024仿真秒/226个物理样本的FIXEDSTAND观察；但关机收尾阶段48个已生成策略帧未写盘，故整份D记录为INVALID。D终止边界0–694周期自身连续、时钟有效；生产保持helper对该任务窗口11个episode的限定审计为11/11完整通过，不覆盖整份记录的INVALID状态。C和D启动阶段均有已确认低高度episode，均在任务起点前关闭。没有重试；本轮运行额度已用完。详情见[B1执行报告](evidence/B1/B1_EXECUTION_REPORT.md)及[B1运行矩阵](evidence/B1/B1_RUN_MATRIX.md)。

**上一轮离线修正（C/D本轮运行前）：B1去重路径安全证据和跌倒历史修正已完成并提交Director重审。** 一致重复现在会带回附属于同一已持久化策略记录的物理快照及其独立物理时间/身份；终止帧安全检查、碰撞事件和正常收尾gate在离线生产链联测通过。修复后的分析器把dev25 A任务期候选重新评为“已关闭、未达到确认条件”，不是“没有异常”或“安全成功”。该阶段没有启动场景；其后按新授权完成了本轮C/D各一次运行。（历史状态：当时B1仍为唯一ACTIVE / INCOMPLETE；现已COMPLETE（Director有条件通过）。）

**上一轮dev25 A运行状态（历史，见上方最新独立重分析）：** 结果为`VALID / ARRIVED`；442个连续策略周期从cycle 0保存到终止cycle 441，terminal payload与引用精确对应。到达后未完成5秒站立观察，按PASSIVE路径收尾。该轮记录中的normal-closeout gate当时为空值；本轮已定位为FrameCapture一致去重分支漏返安全快照，旧运行并未执行正常站立观察。原始数据保留不改写，详情见[B1执行报告](evidence/B1/B1_EXECUTION_REPORT.md)。

**历史阶段记录：**周期合并写盘与缓冲区复用的离线修复/组合验证已通过；当时尚无场景运行证据。其后获准的唯一dev25 A组运行已完成，当前结果以上段及执行报告最新章节为准。此前运行安排均为历史，不产生新的运行额度。

**补证前历史审阅状态：**周期合并修复尚无场景运行证据、A未启动。当时两次dev25 B尝试均因帧序无效并按规则停止。之后新增授权的唯一dev25 A补证现已完成；当前结果以上面的最新状态及[B1执行报告](evidence/B1/B1_EXECUTION_REPORT.md)为准。

[B1：仿真实验可信度定向补强](tasks/B1.md)现为COMPLETE（Director有条件通过）。原始八条诊断、两次B尝试及失败记录均保留；留出集没有运行策略。唯一dev25 A补证已使合并写盘/终止/分析链获得一次有效运行证据，但存在足端撞击、跌倒UNKNOWN和到达后站立观察未完成等限制，不能据此进入参数比较。B1不计入S编号，S6和M1状态不变；本轮没有授权下一任务。

## MuJoCo实时速度显示补充（2026-10-15）

用户授权在面板顶部补充实际水平速度。显示从当前MuJoCo模型的base_link自由关节读取世界坐标平移速度，输出`Speed (world XY)`与有符号`World vx/vy`，单位m/s（每仿真秒）；在物理互斥锁内读取，不依赖策略帧，因此RL前和到达后站立时也可显示。暂停时标明`PAUSED snapshot`；缺少模型、自由关节或有效速度时显示N/A。原Body XY策略速度仍保留，两个来源不混用。主程序增量构建、现有面板离线检查和无时间步进的坐标核对通过；真实窗口显示尚未核验，本轮没有启动GUI、ROS或策略。该补充不改变当前研究阶段和任务授权。

## 当前地图名称与清理（2026-10-15）

用户另行授权删除S6-08～S6-11旧图片，已精确清理这些阶段designs/evidence目录中114张图片（约7.89 MiB），保留Markdown、JSON布局和原始运行数据；旧报告图片链接因此仅作为历史引用，不再保证可打开。当前M1的60张编号预览和两张总览均保留，本次不删除压缩归档中的历史副本。

用户授权后，冻结M1图现以`dev01.xml`～`dev30.xml`、`holdout01.xml`～`holdout30.xml`使用；PNG同名，几何版本仍为m1-mapset/1.0，名称修订为dev-holdout/1.0。内部M1 ID、XML字节、种子和划分保持不变，新旧对应表保留。旧测试XML从活动目录精确清理，删除前验证压缩备份；默认基础场景、机器人模型和共享资源保留。旧日志不改写，原批次不得跨布局hash强制resume。当前入口见[地图说明](designs/M1/library_v1/README.md)，改名范围和核验见[报告](evidence/M1/MAP_RENAMING_REPORT.md)。本次不运行策略，不改变M1已完成状态或新增Active Task。

> 更新日期：2026-10-15
>
> 当前状态：B1已COMPLETE（Director有条件通过），独立于S系列；S6整体仍未完成。M1已获Director有条件通过并标记COMPLETE，`m1-mapset/1.0`已冻结，留出集未运行策略。当前没有已授权的Active Task；开发集阈值与保持时间预实验仅为后续规划建议，尚未授权。任务窗口内的有限安全结论及其覆盖限制见本文件顶部和[B1执行报告](evidence/B1/B1_EXECUTION_REPORT.md)。

## M1收尾状态（2026-10-15）

Director有条件通过范围为地图集 `m1-mapset/1.0` 的地图内容、命名、开发/留出划分及运行前确定的规则，以及30张开发图各3次的流程记录。地图集、划分和规则保持固定；留出集继续不运行策略。M1不代表正式实验批准或性能结论，详情见[M1执行报告](evidence/M1/M1_EXECUTION_REPORT.md)。

用户已完成12张S6-11短障碍开发图A组各一次（`logs/manual_a_map_survey/20261015_103924_621d54/`）：全部记录VALID，8到达、3碰撞、1跌倒；3次碰撞均为RL_calf接触ppt_obstacle_02；密集局部绕行02有120次切换。均有cycle0和终止帧、自动图表及清理记录；周期/时钟缺口和部分跌倒UNKNOWN保留。未观察到已保存策略帧中的base越出声明区域，不代表连续物理足迹无越界。本批仅作历史背景；与修改几何后的新图是否兼容、能否计入M1重复数须阶段4逐图判断，不用于正式性能结论。

开发集90条计划项均有运行记录，每张图3次；另有一次来源基础设施失败启动，原样保留并仅重试该计划项一次。两条 `WALL_CLOCK_GUARD_INCOMPLETE` 是不完整运行，不是普通仿真超时；`VALID`仅表示来源/记录校验通过，不代表全部指标完整。78次ARRIVED不等于78次已证实安全且全程界内到达。此前12条S6-11记录使用7.0m目标，不计入本批。碰撞分析不一致、跌倒UNKNOWN、越界UNKNOWN、时钟/周期缺口和短反向统计覆盖限制均保留。不得按A结果删图；留出集只做几何和生产加载/身份核验，未运行策略。旧测试地图已退出M1当前使用入口；有历史引用的XML、资源和原始数据保留。阶段1–2图审及历史过程见[阶段报告](evidence/M1/STAGE12_REVIEW.md)。

下一步建议先定向修正记录/指标覆盖并调查两次墙钟保护，再规划开发集参数预实验。本轮没有创建或启动该任务；当前无已授权Active Task。

## S6-11地图工作历史（M1继承背景）

用户已暂停家具相关追加工作，现有家具样例保留但不纳入地图遍历。S6-11当时新增A组手动入口`scripts/manual_a_map_survey.py`，固定选择两类短障碍的12张开发图，每图一次；其余18张旧长障碍开发图、全部留出图及家具变体排除。结果索引保存到`logs/manual_a_map_survey/<批次>/survey.md`，每图运行数据仍在`logs/manual_abcd_compare/`。当时仅做了只列图及Python编译检查；S6-11未完成的地图工作后来转入M1。**历史状态：**M1执行期间曾是唯一Active Task，现已完成。

用户追加授权制作一张静态凳子开发样例，已生成 `scene_stool_dev_01.xml`、布局、俯视和斜视示意图，属于S6-11历史补充，不计入60张或留出集。两张固定凳子共10个实际碰撞构件；射线保守使用二维投影，不支持钻凳。核验及运行状态见[样例说明](designs/S6-11/stool_sample/README.md)和[补充报告](evidence/S6-11/STOOL_SAMPLE_REPORT.md)。未运行机器人策略。

S6-11又追加四张常见物体开发变体，不计入60张地图：稀疏/中等局部绕行图各一张，稀疏/中等前向错列图各一张，使用固定凳子、小桌/办公桌及圆柱代理的垃圾桶、花盆和路桩。布局、俯视图和非MuJoCo斜视示意、生产C++ MuJoCo 3.3.3加载/指纹、实际MuJoCo主程序构建及生产publisher→Python reader 4/4 LIVE握手均已完成。俯视几何按0.65 m包络+0.10 m余量检查，碰撞构件有对象/部位名称；没有运行策略。来源开发图和留出图均未修改。两个offset变体替换后的实测XY投影占地为5.329%与9.169%，低于各自继承的稀疏/中等原密度带下界，因此当前只称来源密度变体，不声称仍满足原分档。四张图先交用户审看，再决定手动运行哪几张。详情见[常见物体变体报告](evidence/S6-11/COMMON_OBJECT_VARIANTS_REPORT.md)。

用户指出 S6-09 总览最后两列（多路线分叉、不规则障碍群）需要从大范围侧面绕行，与当前前方有限射线下的局部避障研究不匹配。授权将该24张从下一版主实验候选中移出，以两类局部绕行布局替代；旧地图与原始结果保留为历史版本，不按组间成败筛图。其余36张只做同规则静态审查，尚未认定全部适配。

S6-11 已完成24张替代地图与新版60张清单：原24张多路线分叉/不规则障碍群只从新版主候选清单排除，旧地图与运行证据仍保留；新图为短障碍局部绕行和前向错列短障碍各12张，开发/留出共30/30。几何检查和生产C++ publisher→Python reader场景身份握手24/24通过，实际MuJoCo主程序已增量构建。其余36张按同一规则静态审查后均发现超出新局部障碍尺寸的长构件或较大参考转角，已列清单供 Director 审阅，未擅自修改。

以上S6-11 ACTIVE和图审文字描述的是M1启动前的历史状态，现已由M1取代。其旧XML、布局、预览及运行数据仍保留。S6-11地图边界非实体墙、几何通路不保证策略可达、包络未作步态验证等限制继续适用；M1阶段3–5所需轨迹/越界记录与统计口径已在[M1冻结规则](designs/M1/MAP_RULES.md)及[M1执行报告](evidence/M1/M1_EXECUTION_REPORT.md)中完成并提交审阅。留出集仍未用于策略运行。

## M1阶段1–5执行结果（2026-10-15）

- 冻结规则见[MAP_RULES.md](designs/M1/MAP_RULES.md)：保留0.65m Go2平面包络和0.10m工程余量；场地范围x=[-0.5,8.0]m、y=[-3,3]m，目标7.5m。S型是“可行参考路线”，不输入策略，也不强迫策略遵循；主要障碍位置、尺寸、朝向和错列方式在全库变化。密度是几何标签，不代表风险或难度。
- 四张新旧对照图：[缓和S稀疏](designs/M1/stage2_samples/comparison_gentle_s_sparse.png)、[缓和S中等](designs/M1/stage2_samples/comparison_gentle_s_medium.png)、[错位稀疏](designs/M1/stage2_samples/comparison_offset_route_sparse.png)、[错位中等](designs/M1/stage2_samples/comparison_offset_route_medium.png)。Director已通过图审，接受7.5m目标与扩大场地边界；阶段3全库已生成，详情见[阶段报告](evidence/M1/STAGE12_REVIEW.md)与[M1执行报告](evidence/M1/M1_EXECUTION_REPORT.md)。
- 阶段3生产检查：60张（30开发/30留出）可复现布局通过几何、生产绑定和publisher→Python reader 60/60 LIVE握手；MuJoCo 3.3.3真实主程序构建通过。阶段4只运行开发集，留出集没有策略运行。90条计划项均有记录、每张开发图3次；首次来源基础设施故障仍为INVALID，修复后只重试一次。结果表和运行链接见[M1执行报告](evidence/M1/M1_EXECUTION_REPORT.md)及[批次结果索引](../../logs/manual_m1_a/library_v1/m1_results/development_A_results.md)。
- supervisor终局为78 ARRIVED、8 COLLISION_TERMINATION、2 DIAGNOSTIC_SIM_TIME_LIMIT、2 WALL_CLOCK_GUARD_INCOMPLETE。后两条墙钟保护记录为不完整运行，不是普通仿真超时；`VALID`只表示来源/记录校验通过。78次ARRIVED不等于已证实安全且全程界内到达：88条边界结论因覆盖不全为UNKNOWN，另2条不适用；跌倒、碰撞事件链、周期与时钟覆盖等限制详见执行报告。该批不作为成功率或性能结论。
- Director已对运行前冻结的边界、几何和规则给出有条件通过；`m1-mapset/1.0` 内容、命名、开发/留出划分及规则保持固定，没有按结果删图或更改规则。进程及批次包装器均核对已退出，不存在本次相关残留。当前没有已授权Active Task；先建议定向修正记录/指标覆盖并调查两次墙钟保护，再规划开发集参数预实验。

## 今日收尾记录（2026-10-13）

- S6-06～S6-10 均已在各自范围内完成并获 Director 有条件通过：四组公共启动交接核验、条件就绪启动与 ROS 会话复用、六张地图样例、60 张开发/留出地图及宽松交替绕行变体。
- 两次完整启动链实测从 MuJoCo 启动到 RL 请求约 7.26 / 7.52 秒；不据旧时间戳计算提速比例。地图库为 30 张开发、30 张留出，留出集尚未用于策略运行。
- 新地图实际运行暴露了主程序未更新、内部 ID 超长和生成/运行 MuJoCo 版本导致指纹不兼容的问题；已分别完成构建、唯一短 ID 和运行端指纹元数据修复。地图几何和留出划分未因身份修复改变。
- 原交替长障碍开发图 A/B 存在后退碰撞；原 C 停在 RL 前准备阶段，不能作为保持逻辑失败证据。宽松变体 A/C 各一次到达并完成站立观察；A 安全结论 UNKNOWN、C 一个保持 episode 不完整，单次结果不用于性能排名。
- 研究重点保持为滞回与最小保持的切换稳定性、安全及任务代价，不继续迭代复杂布局或扩展导航能力。用户可继续观看开发地图；已提供宽松变体四组手动对照命令，截至本次收尾未收到该四组运行结果。
- **当时状态（2026-10-13）：**没有 Active Task，下一步待规划开发集参数预实验候选值、场景、重复与越界规则。该状态后被 S6-11 取代。正式参数、正式实验、性能结论和实机验证尚未完成。
- 已提交的诊断报告记录本次相关仿真与辅助进程退出；本次收尾仅记录与 Git 提交，不启动或关闭用户其他会话。原始运行日志留在本地，本地提交后由用户推送 GitHub。

## S6-10 审阅状态（2026-10-13）

S6-10 已标记 **COMPLETE（Director 有条件通过）**。通过范围：澄清原 C 组准备失败、完成宽松开发变体、修复运行端指纹元数据、通过生产 publisher→reader LIVE 握手，并在变体上完成 A/C 各一次有效 ARRIVED 及5秒站立观察。

保留限制：A 的碰撞/跌倒安全结论为 UNKNOWN；C 保持审计4个 episode 中3个完整、1个不完整，聚合审计仍为 incomplete；两组各一次且初态不同，不能据此比较性能或成功率。67个场景仅更新运行端指纹元数据，XML、几何、布局及开发/留出划分未变。后续地图生成须采用与实际运行端一致的指纹计算方式，避免登记不兼容值；本轮不扩展实现。原复杂地图和历史失败记录保留，不再迭代该布局。环境已核对无本次相关残留。详情见 [S6-10执行报告](evidence/S6-10/S6-10_EXECUTION_REPORT.md)。

**历史状态（S6-10审阅时，2026-10-13）：**当时建议规划开发集参数预实验，先明确候选值、场景、次数、顺序、停止条件和评价口径；参数运行尚未授权。该状态已由后续任务取代；当前唯一Active Task为本文件顶部所列M1。

## S6-09 审阅状态（2026-10-13）

S6-09 原地图库及身份修复补证均获 Director 有条件通过并标记 COMPLETE。原地图库包括 60 张可复现地图、30/30 开发与留出、五类结构几何检查，以及 XML 和生产场景身份核验。后续实际启动发现内部 `scene_id` 长于碰撞共享快照的 32 字节字段，曾导致 authority 无法发布完整身份、Python reader 将物理状态判为 INVALID；两次历史 `STARTUP_FAILURE` 原始记录均保留。修复使用唯一短内部 ID，保留描述性 XML/图片文件名和全部地图几何；60 个 ID 均唯一且字段长度合规，MuJoCo 主程序构建通过。一次稀疏开发图 A 补跑通过 publisher→reader 的 LIVE 身份链并以 `VALID / ARRIVED` 结束，证明范围仅限这次基础运行链，不代表策略性能。详见[身份修复补证报告](evidence/S6-09/S6-09_IDENTITY_REPAIR_REPORT.md)。

保留限制：同类布局有模板相似性，地图边界没有实体墙；后续开发集试跑需记录机器人轨迹和是否越出声明区域，并在实验前确定越界终止规则。几何连通不保证策略可达，包络未经步态验证。此次单次运行有6个策略周期缺口、6个无效策略时钟帧，并存在任务起点前发生且已关闭的低高度确认事件；不能据此推断普遍安全或性能。留出集尚未用于策略测试，不得用于参数选择或按策略表现删改。**历史状态（S6-09身份补证审阅时，2026-10-13）：**当时没有已授权的Active Task，参数比较和开发集试跑尚未启动；该状态随后被M1取代。

## S6-08 审阅状态（2026-10-13）

[S6-08](tasks/S6-08.md) 已 **COMPLETE（Director 有条件通过）**。通过范围为可复现地图生成器、六张开发样例、几何检查、生产场景接入及两次短诊断。中等 `medium_01` 与密集 `dense_01` 各一次普通 A 组诊断均 `VALID / ARRIVED` 并完成站立观察；中等 run 有 92 条模式边沿、85 个 Recovery 周期和 2 次足端撞击，密集 run 未触发 Recovery。运行环境已清理并核对退出。完整证据见 [S6-08 执行报告](evidence/S6-08/S6-08_EXECUTION_REPORT.md)。

几何表中的“中心线包络宽度”是参考路径沿线最小障碍净距的两倍，不等于两侧障碍实际间距。密集样例所列 4–6 mm 是超过既定机器人包络和工程余量要求的额外宽度，不是实际通道宽度。密度档位存在重叠；保守包络未经步态实测；运行存在周期/时钟缺口，每个诊断场景只有一次运行。**历史记录（S6-08 完成时）：**最终 60 张地图的生成规则、密度分档及开发/留出划分尚待规划；当时建议先规划这些内容和批量生成流程。该建议随后由 S6-09 完成；S6-09 当前状态及后续建议见本文件顶部。

## S6-07 审阅状态（2026-10-13）

[S6-07](tasks/S6-07.md) 已由 Director 有条件通过并标记 **COMPLETE**。通过范围为条件就绪等待、同一 ROS 查询/发布会话复用、新鲜具名关节采样、完整启动计时，以及正常和碰撞终局后的清理流程。两次从 MuJoCo 启动至 RL 请求分别约 **7.26 秒**和 **7.52 秒**；没有可靠基线，不计算提速比例。`/joint_states` 实际发布者为 `/joint_state_broadcaster`，消息为 `sensor_msgs/msg/JointState`，采样器只接受请求后的新鲜消息及恰好 12 个预期具名关节的完整有限 q/dq。

平地 A 单次流程核验 `fd4b212a534b49878a365dbb91fdb42c` 为 `VALID / ARRIVED`，终点距离 0.4868 m，首策略 cycle 0 与完整终止帧均有记录，约 49.93 Hz，并完成约 5.006 仿真秒站立观察。random04 A 单次流程核验 `4087cd227f414c17965e72d3647c0634` 为 `VALID / COLLISION_TERMINATION`，记录 RL_calf 与 `ppt_obstacle_15` 接触，距离目标 3.7443 m；PASSIVE controller 转换确认，运动期跌倒保持 UNKNOWN。两组阶段耗时、策略频率、周期/终止帧身份、publisher QoS 和原始记录目录见 [S6-07 执行报告](evidence/S6-07/S6-07_EXECUTION_REPORT.md)。旧 PREPARATION_FAILURE、UUID STARTUP_FAILURE 和历史数据原样保留。

两次均通过 EOF 有界关闭持久 ROS 会话，session、ROS launch 和 MuJoCo exit 0、未强制；逐项进程扫描无本次相关残留。早先会话 exit 1 缺少 helper stderr/异常栈，原因仍 UNKNOWN；不能追溯认定为正常退出或程序错误。每个场景仅一次，不能推断成功率或性能优劣；random04 碰撞、跌倒 UNKNOWN、周期与时钟缺口、MuJoCo 命令消费未确认等限制保留。历史失败和 INVALID 原始记录未改写。参数比较尚未启动。**历史状态（S6-07审阅时，2026-10-13）：**当时没有已授权的Active Task；该状态随后被M1取代。完整审阅后报告见 [S6-07 执行报告](evidence/S6-07/S6-07_EXECUTION_REPORT.md)。

## S6-06 审阅状态（2026-10-13）

[S6-06](tasks/S6-06.md) 已 **COMPLETE（Director 有条件通过）**。通过范围包括新公共启动交接下 A/B/C/D 各一次普通运行、实际配置消费、RA 驱动切换、cycle 0 与初态关联、终止帧、机制审计、正常站立收尾及环境清理。四组在 `scene_random_04.xml` / S2-08-r1 下均为 `VALID / ARRIVED`，并完成约 5 仿真秒站立观察；0.30 控制运行秒公共增益交接有连续接口写值证据。

保留限制：A 有 FL 足端撞击；B 运动窗口跌倒为 UNKNOWN；C 保持审计 6/17 片段不完整；四组短反向切换统计覆盖不完整；初态存在差异，并有缺帧和无效时钟；接口写出不等于 MuJoCo 已消费。各组各一次不能证明成功率、性能排名或参数优选。进程均已核对退出；prepared stop publisher 退出码 1，closeout 完成。完整表格、初态、机制审计和时间线见 [S6-06 执行报告](evidence/S6-06/S6-06_EXECUTION_REPORT.md)。

S6-06 审阅时没有已授权 Active Task；该状态已由本轮新建的 S6-07 取代。后续候选参数预实验仍须等待用户提出候选阈值与保持时间，再确定场景、次数、运行顺序和评价规则；新比较统一使用 S6-05 公共启动交接，不混入旧控制链数据，不追加目标平滑或自动运行参数比较。

## 最近完成任务（2026-10-12）

**S6-05 最新审阅状态：COMPLETE（Director 有条件通过）。** 通过范围是四组共用的实际增益连续交接、0.30 控制运行秒到达策略目标、定向核验及构建、平地/场景04各一次普通 A 诊断和正常站立收尾。两次诊断均未启用低高度延期。生产 helper 定向检查、A/B/C/D 切换配置回归、日志解析、Python 编译及 controller/Go2 配置增量构建安装通过。

S6-05 的平地与场景04各完成一次普通 A 组诊断，均 ARRIVED 并完成 5 仿真秒站立观察。运行记录直接显示 12 个接口增益从 80/3 起步、约 200 Hz 线性变化，并在 0.30 秒达到 30/0.65。两次都保留了在策略进入前开始、确认并关闭的准备期低高度 episode；RL 周期快照未见当前 candidate/confirmed，最低保存高度约为平地 0.2865 m、场景04 0.2346 m。相较 S6-04 特殊观察 run 的最低保存高度 0.07598 m，这只是初步较轻的观测，运行方式、采样覆盖和初态不同，不能证明因果或普遍改善；下沉风险没有确证消除。没有触发 Recovery，不能判断切换效果。接口写出不等于 MuJoCo 消费，实机未验证；关节目标和前一动作未修改，0.30 秒是工程交接时间而非 Recovery 保持时间。每次运行及场景04结束后均核对本次相关进程退出，未提交 Git。后续候选参数比较必须四组共用新交接代码，旧控制链数据不能直接混入新对照。等待用户提出候选阈值和保持时间后，再确定场景、次数与运行方案。详见 [S6-05 执行报告](evidence/S6-05/S6-05_EXECUTION_REPORT.md) 与两组原始运行目录。

**S6-04 最新审阅状态：COMPLETE（Director 有条件通过）。** 通过范围为显式仿真启动观察、实际增益与动作记录、恢复判定修正，以及本次低高度事件随后恢复的证据。原始 `STARTUP_OBSERVATION_UNRECOVERED_LIMIT` 保留；生产 helper 离线重评为已恢复，观察达到上限并正常结束。reader 采样跳步不等于 producer 物理覆盖断裂。详细派生时间线见 [S6-04恢复重评](evidence/S6-04/S6-04_RECOVERY_REASSESSMENT_3F6F.md)。本次确有低高度确认及触地，不能称无跌倒或安全成功；单次恢复不证明普遍安全。策略缺帧和 STALE 保留；增益爬升与下沉重叠不证明唯一因果；平滑交接未实现；普通运行规则未修改，普通任务早停问题仍未解决；MuJoCo 具体命令消费仍为 UNKNOWN。

定向验证：`scripts/test_s6_04_startup_observation.py` 12/12通过，覆盖稀疏reader且生产累计覆盖完整、区间新异常、无效姿态、历史溢出、producer覆盖缺口、来源变化和历史不完整情况；Python编译检查通过。以上为此前技术执行证据；S6-04 收尾时仅同步文档，没有运行测试、仿真、模型、ROS或桌面应用。方案A未实施。当时建议规划的“四组共用启动交接修复”后来经用户授权成为 S6-05，现已由 Director 有条件通过；该历史建议不再是待办 Active Task。

**S6-04 更早的原方案B运行记录（历史）：**先前 A 组观察因共享帧缺少 policy cycle 1、2 被判 `STARTUP_OBSERVATION_INVALID`；两个已保存物理样本显示高度约0.314→0.300 m，但覆盖约0.106仿真秒，不能判断是否恢复。controller trace 保存了实际 q/dq、目标、动作及接口增益。该历史结论未改写。后续用户手动 run `[local run ID omitted]` 的原始终局和纠正后派生判断见顶部；S6-04已由 Director 有条件通过并标记 COMPLETE，平滑交接未实施。

[S6-01：四组预实验流程与指标可用性核验](tasks/S6-01.md) 已标记 **COMPLETE（Director 有条件通过）**。通过范围：四组流程可执行、实际配置来源可核对、真实 LIVE 面板证据、具名关节采集可工作，以及指标定义和可用性评估。完整结果见 [S6-01 执行报告](evidence/S6-01/S6-01_EXECUTION_REPORT.md)、[四组结果](evidence/S6-01/group_results.md) 和 [指标可用性清单](evidence/S6-01/METRIC_AVAILABILITY.md)。

该结论不代表严格初态配对、正常收尾全部通过、安全指标完整可用、参数选择完成或性能结论成立。A/B 缺关节初态，C/D 关节样本与 RL 起点不同步；B/D 正常站立观察未完成并使用 command 9 回退；碰撞否定覆盖不足、部分跌倒为 UNKNOWN；D 两个保持 episode 不完整。首批原始 INVALID、cleanup 状态错误及纠正说明均保留；旧安全测试 fixture 不匹配的失败不概括为全部相关检查通过。四组来自不同批次，不能比较成功率或性能优劣。所有运行结果仍排除正式实验统计。

Director 已对 [S6-02：初态记录、碰撞/跌倒覆盖及正常收尾的定向修正](tasks/S6-02.md) 有条件通过并标记 **COMPLETE**。通过范围包括首周期实测初态日志、运动窗口碰撞/跌倒覆盖判定、当前格式针对性核验、构建、A/D 各一次诊断，以及 D 一次正常到达后的站立收尾。周期0由 controller companion 日志保存，A/D JSONL 分别从周期12/15开始，任务开头记录不完整；后续无周期缺口不等于从任务起点完整采集，且两组初态不完全相同。安全否定结论只适用于声明的运动窗口，不能外推到首个策略周期或全采集范围。A跌倒为 UNKNOWN，D保持审计4/8 episode不完整；旧站立 MISSING 的唯一原因未证实，四元数顺序和 MuJoCo 命令消费仍有限制。单次结果不证明性能优劣或成功率。完整证据见 [S6-02 执行报告](evidence/S6-02/S6-02_EXECUTION_REPORT.md) 和[指标可用性](evidence/S6-02/METRIC_AVAILABILITY.md)。运行后相关项目环境已核对退出，未提交 Git。

Director 已对 [S6-03：参数比较前的运行、采集与统计准备](tasks/S6-03.md) 有条件通过并标记 **COMPLETE**。通过范围仅为准备准入、配置实际消费、周期0与初态关联、任务窗口记录、失败终局分析和进程清理。`BaseFixedStand` 记录内部 `percent_>=1.5` 的 `[STAND-READY]` command 3 门槛；准备观察器看到该证据后才开始连续0.50仿真秒物理稳定窗口。A run `[local run ID omitted]` 与 D run `[local run ID omitted]` 均确认加载本轮安装库、实际配置及唯一 controller/writer；记录器在 RL 前就绪，实际具名 q/dq 可采集，两个 cycle-0 完整帧分别与同 session/cycle/monotonic 的 companion 精确匹配，任务起点时钟有效，终止帧及分析结果完整。

两次结果均为 **VALID / FALL_TERMINATION**：A 21帧、3个策略周期缺口、2个无效时钟帧；D 22帧、2个周期缺口、0个无效时钟帧。进入 RL 后约0.46秒，两组均确认跌倒，距目标约7.08米，未接近障碍、未到达、未触发 Recovery，切换0次。结果不能归因于退出阈值或保持时间，也不能证明算法优劣。碰撞否定仅限各自覆盖完整的声明运动窗口。两次起点 q/dq 有小幅差异，不是严格初态配对。到达后站立观察因提前跌倒不适用；command 1 后 controller PASSIVE 转换分别在约4.605 ms和3.017 ms确认。清理观察器虽在请求后读到周期23，但该帧时间戳早于请求，不能当成请求后新策略决策。stop publisher 以退出码1退出但未残留，closeout errors 为空；MuJoCo 命令消费未知。早先 PREPARATION_FAILURE、INVALID 和 prepare-only 证据原样保留。完整结果与剩余限制见 [S6-03 执行报告](evidence/S6-03/S6-03_EXECUTION_REPORT.md)、A/D 补证摘要（原始运行/环境明细保留本地，未纳入分享提交）、[首周期/初态证据](evidence/S6-03/INITIAL_STATE_AND_WINDOWS.md) 和[指标可用性](evidence/S6-03/METRIC_AVAILABILITY.md)。所有本次相关仿真、controller、ROS及辅助进程均已核对退出。

S6-03 工具准备基本完成，但当时不具备启动参数比较的条件。S6 尚未整体完成。**S6-04 历史核对与审阅结论：**已核对 ABS 部署参考启动阶段、FIXEDSTAND→RL 交接、Go2 训练 reset 及终止语义，并对照既有轨迹。用户提供的 `3f6f...` run 经修正 helper 离线重评确认该次低高度 episode 随后恢复，不代表全部启动下沉可恢复。S6-04 已有条件通过；具体限制见 [S6-04 执行报告](evidence/S6-04/S6-04_EXECUTION_REPORT.md)。当时建议先规划四组共用启动交接修复，优先核对增益连续性，再决定目标过渡；此建议后来由S6-05执行。该段是历史状态，不表示当前存在Active Task。

## 今日收尾（2026-10-11，以下为历史记录）

用户要求今日停止。A/B/C/D 基础实现与最小功能核验已完成并有条件通过；尚未开展参数预实验或正式对照，不能形成性能优劣或成功率结论。

- 手动脚本 `scripts/manual_ab_compare.py` 默认已改为依次运行 A/B/C/D 各一次，沿用 E=-0.05、B/D 的 X=-0.08、C/D 的 H=0.30 仿真秒；独立配置、自动分析和逐组清理门槛保留。新批次输出到 `logs/manual_abcd_compare/`。本次修改后没有实际运行四组批次，也没有运行测试。
- MuJoCo 策略面板发现旧校验只接受截至 A 的组别编号，导致 B/C/D 正常帧被判 INVALID。`common/abs_mujoco_live_panel.h` 已支持四组编号及保持字段校验，显示组别和保持剩余/到期/无效状态；实际启动路径 `unitree_mujoco/simulate/build2/unitree_mujoco` 已重新构建成功。没有运行测试或仿真，真实窗口显示仍待下次核验。正常到达后退出策略并站立时，不显示旧策略值。
- 已向用户提供独立只读代码导师提示词；教学不推进开发 Active Task，不启动仿真或修改代码。
- 上述 2026-10-11 “尚未运行 S6-01”及“建议规划 S6-01”是历史状态；本任务于 2026-10-12 获授权并执行。首批 C 的 INVALID 记录保持原样；本次 C/D 补证结束后 MuJoCo、controller、ROS、RViz、采集器、停止发布器及辅助进程均已核对无残留。
- 未提交 Git。**历史状态：**C/D 补证提交审阅时，S6-01 曾保持 ACTIVE；Director 随后有条件通过并将其标记 COMPLETE。未启动参数扫描或下一任务。

## 当前授权与最近审阅（2026-10-11）

S3-01 已获 Director 有条件通过并标记 COMPLETE：通过 B 生产双阈值机制、配置消费及同代码 A/B 各一次功能诊断；不冻结正式参数，不证明性能改善。无效时钟、姿态候选、初始关节实测缺失和 MuJoCo 命令消费等限制继续保留。overlay 配置符号链接误写已恢复的历史保留；实际隔离流程核验纳入下一任务。

Director 已对 [S4-01：C组最小保持实现与A/C功能诊断](tasks/S4-01.md) 有条件通过并标记 **COMPLETE**。通过范围包括 C 配置实际消费、按仿真秒执行最小保持、进入起算/未到期保持/到期退出/风险再次升高不重启计时、同代码 A/C 各一次功能诊断，以及到达后约 5 仿真秒站立观察和环境清理。A run `[local run ID omitted]` 为 `VALID / COLLISION_TERMINATION`；C run `[local run ID omitted]` 为 `VALID / ARRIVED`。保持审计覆盖 19 个 Recovery episode，其中 10 个完整、9 个不完整，聚合状态仍为 `incomplete`；未发现违规不代表缺失周期已验证。单次 A 碰撞与 C 到达不能证明性能优劣；未取得实时 C 面板截图，碰撞否定结论、运动期跌倒、暂停恢复、时钟重连和 MuJoCo 命令消费仍有限制。H=0.30 秒仅为诊断参数；stop publisher 退出码 1 和历史失败记录保留。详情见 [S4-01 报告](evidence/S4-01/S4-01_EXECUTION_REPORT.md)。

Director 已对 [S5-01：D组组合机制实现与B/D功能诊断](tasks/S5-01.md) 有条件通过并标记 **COMPLETE**。通过范围包括 D 生产配置实际消费、滞回与最小保持组合生效、定向验证和构建、同代码 B/D 各一次功能诊断、到达后站立观察及环境清理。A/B/C/D 已完成基础实现与最小功能核验；参数预实验、正式对照和论文性能结论尚未完成。

S5-01 限制：16 个 D Recovery episode 中 15 个完整符合规则、1 个因周期 196 时钟 STALE 而不完整；整体保持审计仍为 `incomplete`，缺失周期未获验证。单次 B/D 不能证明性能优劣或成功率；初始关节实测向量未保存；D 跌倒为 UNKNOWN，碰撞否定结论受覆盖限制；有效 D 实时面板截图未取得；参数未正式冻结；MuJoCo 命令消费等既有 UNKNOWN 保留。分析器浮点边界修正说明、历史失败和原始运行记录保留。详情见 [S5-01 执行报告](evidence/S5-01/S5-01_EXECUTION_REPORT.md)、[B/D 对照](evidence/S5-01/comparison.md) 和[保持审计](evidence/S5-01/hold_hysteresis_audit.md)。

截至 2026-10-11，当时没有已授权的下一项 Active Task，并建议规划 S6-01；该建议随后于 2026-10-12 经用户授权执行。此处保留的是任务执行期间的历史状态，不表示 S6-01 目前仍 ACTIVE。当前状态见本文件顶部。

## 今日先前执行记录（2026-10-11，历史状态）

S2-08 已由 Director 有条件通过并标记 **COMPLETE**。通过范围：四张地图有直线阻挡、五张地图保留绕行通道；布局、预览、XML 与身份同步；01/02/03 三张修改图各完成一次有效到达及侧向绕行观察。布局修订 `S2-08-r1` 由 seed 3101–3105 的原布局确定性编辑而来：01障碍01、02障碍03、03障碍04穿过起点到目标线段；04障碍15保持阻挡，05无阻挡。旧布局与旧身份备份到 `evidence/S2-08/previous_layouts/`。保留路线最小保守净距分别为 0.589/0.659/0.698 m，且大于 0.55 m 半宽。C++ MuJoCo 生产探针加载五图通过，射线与碰撞仍用同一障碍选择函数。

01/02/03 各完成一次有效 A 组非正式诊断，均 VALID / ARRIVED，终点距离分别为0.4987/0.4833/0.4921 m；01、02各记录4次模式切换（进入/退出 Recovery 各2），03没有 Recovery。三次轨迹在直线障碍 x 范围内均观察到侧向绕行并到达；单次结果不代表成功率或性能。碰撞覆盖不完整，不能据此声称无碰撞；每次 run 在运动前均有跌倒 tracker 已确认历史，01运动期另有短暂未确认候选。三次到达后均在 FIXEDSTAND 保持约5仿真秒，正常路径未趴下或卸力；停止发布器退出码1保留，MuJoCo先于ROS/controller关闭，运行后相关进程均已退出。配置、频率、RA、接触/跌倒和指标详见 [S2-08 执行报告](evidence/S2-08/S2-08_EXECUTION_REPORT.md)。

用户已授权 [S3-01：B组双阈值滞回实现与A/B功能诊断](tasks/S3-01.md)，当时为唯一 **ACTIVE** 任务并提交 Director 审阅（现已有条件通过，最新授权见上）。生产配置解析和状态转换已增加 B 组双阈值滞回，A 组单阈值未回退；controller 构建、定向边界检查、记录/来源检查均通过。S2-08-r1 随机场景04在同一新版代码、相同模型和参数下各完成一次 A/B 有效功能诊断，两次均 ARRIVED，并完成约 5 仿真秒站立保持。B 实际消费 E=-0.05、X=-0.08、滞回开启、无保持；周期审计中 92 次转换符合规则，26 个滞回带内 Recovery 周期保持 Recovery。A 有 124 次转换。每组仅一次，不能推断性能或统计改善；B 有无效仿真时间和未确认姿态候选，A 有足端撞击及运动前姿态事件。初始关节实测值未记录；B 参数尚非正式冻结值，MuJoCo 命令消费仍 UNKNOWN。配置 overlay 符号链接曾使 B 配置短暂写入源码配置，已用保存快照恢复并核对无差异，具体经过见 [S3-01 执行报告](evidence/S3-01/S3-01_EXECUTION_REPORT.md)。本轮运行及临时环境已清理。未授权 S3-02；没有启动后续任务，未提交 Git。

## 最近任务状态（2026-10-10）

### 今日收尾（2026-10-10）

S2-06 五个固定随机静态场景和 S2-07 到达后稳定站立均已完成并有条件通过。用户手动复测场景04，run `[local run ID omitted]` 为 ARRIVED，终点距目标约0.4999 m，站立保持完成，正常路径未请求 PASSIVE，终端报告相关进程已退出；分析器已生成本地结果表。该记录位于 `logs/manual_scene_runs/scene_random_04_standing/`，属于手动诊断，不并入正式实验。上述“当日停止、没有已授权 Active Task、建议规划 S3-01”是2026-10-10当时的历史状态；其后建立的 S2-08 当前进展见本文件顶部。

**S2-07 最新范围与结果：** Director 有条件通过 [S2-07：到达后受控停止与稳定站立](tasks/S2-07.md)，状态为 **COMPLETE**。通过范围：到达后停止运动、确认站稳并连续保持约 5 仿真秒；正常路径不自动趴下或卸力；运动指标排除收尾；先关闭 MuJoCo，再关闭 controller 等进程。平地与随机场景04两次诊断均 ARRIVED，分别有 5.002 s、5.004 s 仿真时间的站立保持记录。ARRIVED 周期封口运动指标，保持观察单独保存。random_04 有真实 MuJoCo framebuffer 画面；flat 首次截图因相对路径未落盘，路径已修正，但没有超出运行次数补跑。详见 [S2-07 执行报告](evidence/S2-07/S2-07_EXECUTION_REPORT.md)。

**S2-07 保留限制：** 两次 run 的完整运动区间碰撞覆盖不足以声称无碰撞；全采集跌倒历史不能外推为保持窗口事件。随机04记录有 17 个无效时钟帧、1 个周期缺口。两次收尾时速度命令均为零，非零命令平滑 ramp 由生产 helper 离线检查，而非运行观察。健康路径没有确认 PASSIVE，因为它没有请求 PASSIVE；真实动作和 MuJoCo 命令消费仍为 UNKNOWN。停止发布器以退出码 1 结束，但所有项目进程均已退出，无强制结束。旧下降/卸力失败记录原样保留。本任务不授权实机运行，实机仍为 NO-GO。

此前趴下/卸力目标的实现和失败属于历史范围，已经被稳定站立目标替代；详见执行报告历史章节。停止发布器退出码 1、截图缺失和所有其余限制及历史失败均保留。S2-07 已有条件通过并完成。**历史状态（S2-07审阅时）：**当时没有已授权的下一项 Active Task；当前状态见本文件顶部。

Director 已对 [S2-05](tasks/S2-05.md) 有条件通过并标记 **COMPLETE**。规则为：足端轻微接触单独记录、不算终止碰撞；足端满足 Fxy > 2*abs(Fz)+10 N 时记为足端撞击但仍不终止；非足端机器人—障碍接触继续终止。v5 生产快照、Python reader/记录/分析链已定向核验；唯一一次 sparse 诊断记录到 RL 足端撞击和同时发生的 RL_calf 非足端终止碰撞。独立轻微足端接触、成功率、其他场景表现和 MuJoCo 命令消费等限制继续保留。

Director 已对 [S2-04：单次仿真选择场景](tasks/S2-04.md) 有条件通过并标记 **COMPLETE**。`--scene` 默认仍为 `scene_obstacle.xml`，支持原固定、平地、稀疏、中等和密集候选；场景身份贯穿 MuJoCo、射线、碰撞、记录和分析，五场景模型与障碍集合由生产路径探针核对。平地实际运行 VALID / ARRIVED，地面接触未误判为障碍碰撞；稀疏实际运行 VALID / COLLISION_TERMINATION。S2-03 及时 PASSIVE 和运动终局/收尾分离流程保留。**历史状态（S2-04审阅时）：**当时没有已授权的下一项 Active Task；当前状态见本文件顶部。

Director 已对 [S2-03](tasks/S2-03.md) 有条件通过并标记 **COMPLETE**。通过范围包括两次手动运行的接触对象核对、RL 前准备停止发布通道、hard-stop 后 PASSIVE 转换确认、运动终局与收尾观察分离，以及一次 VALID / ARRIVED 诊断支持修正后的基础闭环；不代表 A 组稳定避障性能通过。历史“没有 Active Task”的表述只指相应任务关闭时的状态。

## FACT：可以直接复用的基础

- 仓库已有 Unitree Go2、MuJoCo、ROS 2 控制器和仿真启动资产；
- 已有 Agile、RA Value、Recovery 部署模型；
- 控制器中已有三模型调用和 Agile/Recovery 切换路径；
- `paper_faithful_switch` 可作为 A 组逻辑候选；
- `stabilized_switch` 同时包含滞回和保持，只能作为 D 类参考，不能直接视为正式 D；
- 已有包含 RA、policy state、位姿、速度、ray2d 和动作的运行时记录基础；
- 已有碰撞检测、场景、配对运行和指标工具可选择性复用；
- 已有部分实机通信、映射、PASSIVE/硬急停和 sport mode 释放基础。

## 尚未完成

- S4-01 的 C 组按仿真秒最小保持实现及同代码 A/C 单次功能诊断已由 Director 有条件通过；S5-01 的 D 组组合机制也已通过基础实现与最小核验。保持审计仍有时钟覆盖不完整；正式参数冻结未完成；
- S2–S8 的基线、滞回、保持、组合、场景/参数预实验、正式配对仿真、统计分析及论文结果尚未完成；
- S9–S10 实机感知、安全准备与代表性低速验证尚未完成；在安全准备通过前维持 NO-GO。

## 未决事项与保留限制

- Agile/Recovery 动作关节顺序现有多源证据支持 FL/FR/RL/RR（每腿 hip/thigh/calf），但证据性质不同：训练者来源确认、留存训练代码、Go2 资产与顺序捕获、actor/export 链和 checkpoint→部署权重关联共同支持；它不是由模型内嵌元数据或历史不可变 run 配置单独证明。详见下方“动作顺序证据更新”。
- `recovery_hold_steps=30` 是旧步数设置，实际持续时间未由它或 125 Hz/8 ms 注释证实。合同定义仿真以 MuJoCo 仿真秒、实机以单调运行秒计 H，并只在有效策略决策周期判断到期/切换。S4-01 已实现 C 仿真秒保持；本轮确认 H 配置消费及策略周期起算、保持/退出行为，但 9/19 episode 时钟覆盖不完整。暂停恢复、实机计时和同名时钟重连未经运行验证。
- 正常连续运行中策略周期与仿真时钟已关联，并在具体捕获中测得约 50 Hz；这不是所有运行的频率保证。暂停后策略恢复及同名共享时钟重连仍未运行验证；暂停尝试中安全停止后没有策略帧恢复。暂停/中止后的 run 不直接续接。
- Recovery 选择和控制接口命令写出已有周期/来源证据；MuJoCo 是否消费具体命令、以及真实动作生效的精确时刻仍 UNKNOWN。
- S1-05 的无碰撞/未跌倒结论只适用于该次覆盖完整的运动窗口及其诊断规则，不能扩展到全采集范围、其他场景、正式跌倒参数或长期稳定性。S1-04 两次完整运行未出现 Recovery，不能判断切换效果；其碰撞覆盖不完整，不能据此声称无碰撞。
- S1-05 诊断的 XAUTHORITY 参数路径记录有误，实际 renderer 未保存；S1-04 原完整运行也未保存 renderer，不能声称跨运行 renderer 一致。
- 记录开销与周期抖动的因果关系、三档候选场景的可达性/区分度，以及实机 ray2d 来源/标定/时延、硬件层站立与 RL 状态仍待适用阶段核实。

## S1 当前状态

S1-01 至 S1-06 均已由 Director 审阅并标记 COMPLETE；S1-02、S1-03、S1-04、S1-05、S1-06 为有条件通过。各状态只表示对应任务限定范围被接受，不表示所有 S1 验收项均无保留，也不自动等于 S1 整体完成。执行期间的 INCOMPLETE、启动失败和 INVALID 记录保留原状；后续修正和新证据不改写历史 run 分类。

其余历史 UNKNOWN、旧 P1 验收缺口、非必要 hash/manifest 和工作树历史修改均不是当前 Blocker。

## 动作顺序证据更新（2026-10-10）

**来源确认（用户提供）：**Agile 与 Recovery 的训练均在 `ABS_fuwuqi/ABS` 完成，由 ABS 的 Go1 配置适配为 Go2；训练代码和资产仍保留。此为用户对训练来源的确认，不是机器从历史训练命令或不可变运行配置中恢复出的事实。

**仓库直接核验：**

- `ABS_fuwuqi/ABS/training/legged_gym/legged_gym/envs/__init__.py:66-67` 注册 `go2_pos_rough` 和 `go2_rec_rough`；当前保存的两份配置分别在 `go2_pos_config.py:78`、`go2_rec_config.py:81` 指向 `resources/robots/go2/urdf/go2.urdf`。这核实当前留存代码/资产路径，不冒充历史不可变 run 配置。
- 该 Go2 URDF 含 Isaac Gym 捕获中的同一组 12 个可动关节名。需区分文本顺序与运行顺序：URDF 中 `<joint>` 声明顺序是 FR、FL、RR、RL；留存的 Isaac Gym `get_asset_dof_names` 运行顺序捕获 `archive/artifacts/p1_01_contract.json` 记为 FL、FR、RL、RR（每腿 hip、thigh、calf），足接触捕获也是 FL、FR、RL、RR。因此，当前核验到的是具名关节集合对应、且运行顺序捕获为 FL-first；不是说 URDF 文本声明顺序与运行捕获相同。训练基类直接取 `gym.get_asset_dof_names(robot_asset)`（`legged_robot.py:737-766`）。
- `legged_robot.py:80-92,377-399` 将 actor 输出裁剪后按同索引传入按 DOF 数组逐元素计算的控制路径；没有额外关节置换。位置、速度、default/bias 与动作目标均使用同一环境 DOF 索引序列。
- Agile 导出路径（`scripts/play.py:71-79`、`utils/helpers.py:184-194`）复制并 TorchScript 导出 actor；Recovery 导出路径（`export_rec_policy.py:41-89`）按 actor 层键加载权重并 TorchScript 保存。源码中没有重排输出通道的步骤。
- `archive/artifacts/manifest.yaml` 和其引用的 `docs/evidence/P1-01/provenance_recovery.json` 将 Agile `go2_pos_rough/.../model_4000.pt`、Recovery `go2_rec_rough/.../model_15000.pt` 与各自导出/部署权重关联；记录的 actor 张量等价和导出/部署字节等价支持两个部署模型对应这些 checkpoint。
- 控制器当前配置显式将 policy 顺序设为 `ros1_fl_fr_rl_rr`；观测侧对关节位置、速度、前一动作和足接触使用已有重排函数，策略输出再映射到 controller order（`AbsObservationContract.cpp`、`StateRL.cpp`、`StateRLRec.cpp`）。这是部署映射证据，不代替训练顺序来源证据。

**当前判断：**最有证据支持的训练/actor 通道顺序为 **FL、FR、RL、RR；每腿 hip、thigh、calf**。这比“完全 UNKNOWN”更有依据，但仍是用户来源确认与保留源码、资产/运行捕获及权重关联的合并判断。没有历史不可变训练配置，也没有模型内置的具名关节元数据；现有运行行走事实不作为顺序证据。原 S1-02 报告和归档 manifest 中关于缺少历史绑定/metadata 的限制仍成立，不改写其历史记录。

**S1-06 定向核验结果（Director 有条件通过）：**唯一标记值通过生产 `AbsObservationContract` helpers 核对 controller→policy 的 q/default/bias/dq/prior-action/contact 排列、Agile/Recovery observation 字段布局及逆变换；Director 接受的范围还包括对 Agile、inline Recovery 和独立 Recovery 状态类实际动作转换源码的核对。动作对应表和证据见 [S1-06 执行报告](evidence/S1-06/S1-06_EXECUTION_REPORT.md)。私有 actor 输出方法未由本轮 harness 直接调用。没有加载或推理模型，也没有运行机器人。实际 ROS loaned contact interface 次序和 SDK foot-force 索引的具名语义仍 UNKNOWN。

## 非阻塞技术债

- 历史训练环境和部分模型来源信息不完整；
- 旧 P1-10 exact replay/common-start 未闭环；
- 历史 evidence 存在产物丢失记录；
- 部分脚本和文档仍使用旧 Phase/P1 术语；
- 演示场景尚未正式化。

只有当其中某项实际影响当前技术任务或论文实验解释时，才升级处理。

## 当前任务安排（S1 验收对照）

| S1 验收项 | 已证实 | 保留限制 / 未完成 |
|---|---|---|
| 切换合同 | [SWITCH_CONTRACT.md](SWITCH_CONTRACT.md) 的转移、阈值边界、配置互斥、H 时钟/周期语义及事件计时定义已通过 S1-01 最终审阅；RA 越大代表风险越高。S4-01 核对了 C 保持；S5-01 已实现 D 并完成一次 B/D 运行核对。 | C 有 9/19 episode、D 有 1/16 episode 的时钟覆盖不完整；A/B/C/D 诊断参数均不等于正式参数冻结。暂停恢复、同名时钟重连和实机计时仍未验证。 |
| 关键模型接口 | [S1-02 报告](evidence/S1-02/S1-02_EXECUTION_REPORT.md)整理了 Agile `61→12`、RA `19→1`、Recovery `49→12` 的输入构造及控制器处理；训练者来源确认、ABS 代码、Go2 资产与 Isaac Gym 顺序捕获、导出路径和权重关联共同支持 actor 顺序 FL/FR/RL/RR（每腿 hip/thigh/calf）。S1-06 有条件通过：生产 observation/order helpers 的唯一标记值检查通过，且 Director 接受了各策略状态类实际动作转换源码核对。 | 此顺序不是由历史不可变 run 配置或模型内嵌关节元数据单独证明；旧报告/manifest 中的 UNKNOWN 是当时证据结论，仍须保留为来源绑定限制。RA learned score 的物理单位仍 UNKNOWN。私有动作转换成员未直接调用。S1-06 当时未观察仿真 loaned contact order；S2-01 已核实 MuJoCo 仿真传感器→SDK slot→ROS interface 并实测 loaned 名称顺序。实机 Go2 foot-force 物理语义仍 UNKNOWN；见 [S1-06 报告](evidence/S1-06/S1-06_EXECUTION_REPORT.md) 与 [S2-01 报告](evidence/S2-01/S2-01_EXECUTION_REPORT.md)。正常行走不作为证据。 |
| 频率与时间语义 | S1-02 连续捕获约 50.14 Hz；S1-03/S1-05/S4-01 runs 均有各自约 50 Hz 的运行捕获。S1-03 将仿真时钟样本配对到正常连续策略周期；合同规定仿真 H 用 MuJoCo 仿真秒、实机以单调运行秒计，并只在有效策略周期判断到期。S4-01 C run 约 50.045 Hz。 | 频率仅对具体 run 成立，不能排除记录开销。旧 `recovery_hold_steps=30` 与 125 Hz/8 ms 注释不是实际 H/频率证据；本轮 C 保持审计有 9/19 episodes 覆盖不完整。暂停恢复和同名时钟重连未经运行验证。 |
| Agile/RA/Recovery 基础链 | S1-02/03 非正式运行按周期关联 RA、模式和动作来源，观察到 Agile↔Recovery 转换；Recovery 命令与来源周期关联到控制接口写出。 | 写出不证明 MuJoCo 消费或动作生效。S1-04 两次完整运行和 S1-05 一次正常诊断均未出现 Recovery，不能判断切换效果；legacy 候选不等于正式 A 组结果。 |
| 记录、指标、安全事件及清理 | S1-04 两次完整基础诊断具有终止帧、terminal、指标、时间线和收尾证据；S1-05 定向核验了 v3/3864、v4/5760 的版本/长度读取规则及真实 v4 C++→Python→记录→分析链，并完成一次正常诊断。 | S1-04 碰撞覆盖不完整；S1-05 无碰撞/未跌倒结论限于该次覆盖完整的运动窗口和诊断规则，不适用于全采集区间。正式跌倒参数、其他场景和长期稳定性未验证。S1-05 的 XAUTHORITY 记录路径错误且实际 renderer 未保存。历史 INVALID/失败记录保留。 |

### S2-01 实际进展（2026-10-10）

动作顺序由训练者来源确认、保留训练代码、资产及 Isaac Gym 顺序捕获、导出路径和权重关联共同支持为 FL/FR/RL/RR（每腿 hip/thigh/calf），但不来自模型内嵌元数据或历史不可变配置的单独证明。S1-06 的 helper 离线检查和状态类源码核对获 Director 有条件通过；私有动作成员未直接调用。S2-01 进一步核实 MuJoCo 仿真接触链，并在一次有效诊断的固定站立准备阶段直接观察到 controller loaned 顺序 `foot_force/FR, FL, RR, RL`；这不外推为实机 SDK 物理语义验证。

[S2-01：A 组单阈值实现与功能基线核验](tasks/S2-01.md) 已由 Director 有条件通过并标记 **COMPLETE**。通过范围为 A 配置生产解析/切换函数检查、init-only 配置消费证据、既有 legacy 候选真实 RA 行为及仿真足接触来源核对；这不是正式实验授权，也不表示新配置已有运动周期证据。

S2-01 的三类证据分别限定为：旧 `paper_faithful_switch` 诊断中的真实 RA 行为；新 A 配置生产解析器与 helper 的 27 项离线核验；init-only 原始日志独立重验确认新配置被 controller 初始化消费。新 A 配置没有策略帧或运动周期验证。init-only supervisor 的 `INVALID_RUNTIME_SOURCE` 分类与原始文件保留不变。旧诊断的障碍碰撞结果保留，5 个 STALE 时钟帧造成的驻留统计删失仍成立。策略停止确认、MuJoCo 命令消费、实机接口语义等限制继续保留。后续 B 组核验应同时确认统一 A/B 配置的运行消费，并保持两组除滞回外的控制链一致。详见 [S2-01 执行报告](evidence/S2-01/S2-01_EXECUTION_REPORT.md)。

各任务最终结论与证据：[S1-01 任务/合同](tasks/S1-01.md)、[S1-02 报告](evidence/S1-02/S1-02_EXECUTION_REPORT.md)、[S1-03 报告](evidence/S1-03/S1-03_EXECUTION_REPORT.md)、[S1-04 报告及验收对照](evidence/S1-04/S1-04_EXECUTION_REPORT.md)、[S1-05 报告及跨语言核验](evidence/S1-05/S1-05_EXECUTION_REPORT.md)、[S1-06 任务/部署通道报告](tasks/S1-06.md)。

S2-02 已由 Director 有条件通过并标记 **COMPLETE**：MuJoCo 只读实时面板及实际 LIVE 截图通过审阅，显示逻辑不改变控制器。限制仍包括目标距离 N/A、Body XY speed 不等于倾斜时的世界水平速度、非 LIVE 策略状态仅做离线核验、本次 PASSIVE 后策略停止确认未获证，以及 MuJoCo 命令消费未知。面板通过不代表 A 组避障性能通过。以上“尚无 Active Task / 建议规划完整目标任务”的文字只描述 S2-02 审阅时状态；后续 S2-03 已被授权并完成，当前 S2-04 状态见下文。S3-01 暂缓。S2 A 基线有条件通过不代表整个 S2 阶段完成；历史审阅结论和限制保持原状。

### S2-03 实际进展（2026-10-10）

两次手动运行的终止接触均为机器人与 geom 2 静态障碍 box：第一次 geom 25（FL 小腿碰撞体），第二次 geom 27（FL 足端碰撞体），Director 结论为未发现地面误分类。历史记录不含接触距离/力，不能判断穿透或冲击强度。原始记录保留不变。

S2-03 已将终局 PASSIVE 通道改为运行前建立并确认订阅者的持久发布器；控制器在 hard-stop 分支完成 PASSIVE 状态变更后写出带单调时间的确认日志。运动记录在终止快照封口，收尾观察单独保存并排除运动指标。移除了将固定 250 ms 等待写成实际策略输出持续时间的字段语义。

唯一一次补充短诊断为 VALID / ARRIVED，按 0.5 m 目标半径终止，距离 0.4996 m；终局后约 2.972 ms 开始发布 PASSIVE，controller 在发布开始后约 0.414 ms 记录转换完成。运动终局与收尾观察已分开，收尾窗口没有采到新策略帧。运行含 4 个无效时钟帧，统计保留各自有效范围。历史接触距离/冲击强度、MuJoCo 命令消费和物理运动停止时刻仍 UNKNOWN；有限窗口没有策略帧不外推至窗口外。单次到达不代表 A 组稳定避障性能。完整结果见 [S2-03 执行报告](evidence/S2-03/S2-03_EXECUTION_REPORT.md)。该任务关闭时没有已授权的下一项任务；随后用户授权 S2-04，当前进展见下文。

### S2-02 实际进展（2026-10-10）

MuJoCo 现从只读 `/mujoco_rt_frame` 面板显示 Body XY speed（不称世界水平速度）、Agile/Recovery/FAULTED、未进入 RL/策略停止状态、帧内 RA 与实际 entry/exit 阈值、仿真时间及数据状态和年龄。未进入、停止、故障以及 MISSING、INVALID、STALE 时不显示旧策略值；读取在渲染线程执行，每帧重新打开共享对象，不缓存旧 session。仿真时间仅在帧时钟有效时显示。当前帧没有权威目标坐标，因此目标距离明确显示 N/A。`scripts/abs_live_hud.py` 保持不变。

定向离线检查与 MuJoCo 目标构建通过。Run `[local run ID omitted]` 按 A 配置启动，记录 159 个 LIVE 策略帧和真实速度/RA/模式/阈值/仿真时间，最终因运动期障碍碰撞终止；controller、ROS launch 和 MuJoCo 清理均确认退出。运行没有保存 renderer。

首次截图路径当时失败的历史事实已保留；本轮改用 MuJoCo framebuffer 读取并取得真实 LIVE 面板图像。Body XY speed 不是世界水平速度；未进入 RL、策略停止、FAULTED 及数据过期通过离线函数核验，未分别保存这些状态的真实窗口画面。目标距离仍显示 N/A；本轮 PASSIVE 后策略停止确认字段为 false。详情见 [S2-02 执行报告](evidence/S2-02/S2-02_EXECUTION_REPORT.md)。

### S2-04 实际进展（2026-10-10）

S2-04 现支持原固定场景、平地和三种 PPT 候选场景。共用目录中记录每个场景的文件、ID、障碍数量、XML/asset closure 与编译模型指纹；C++ ray writer 与碰撞 authority 使用同一障碍集合函数。ground contact 不进入障碍集合，flat 空集合是有效状态。reader、逐 run record meta、run_context、结果表和时间线都绑定/显示本次 scene；不含新字段的旧记录继续按历史场景兼容读取。

本次平地 run `[local run ID omitted]` VALID / ARRIVED，距离目标 `0.4965 m`，策略频率约 `50.0039 Hz`，motion window 的碰撞/跌倒结论均为 false 且覆盖完整；实际有 foot-floor contact，但 robot-obstacle contacts 为 0。稀疏 run `[local run ID omitted]` VALID / COLLISION_TERMINATION，记录到机器人 geom 42 与 `ppt_obstacle_03` 接触，策略频率约 `49.9959 Hz`。稀疏场景本次有 4 个无效时钟帧；平地有 1 个，原始记录和分析边界均保留。两次运行均确认 controller PASSIVE 转换及 `shutdown_complete=true`，相关进程事后检查为空。

Director 对 S2-04 有条件通过，认可 `--scene` 默认值和五种场景支持、跨 MuJoCo/射线/碰撞/记录/分析的身份绑定、五场景生产路径探针、平地 VALID / ARRIVED 与稀疏 VALID / COLLISION_TERMINATION，以及平地 ground contact 未误判为障碍碰撞；S2-03 的及时 PASSIVE 和收尾分离流程保留。

限制：原固定、中等和密集场景尚未实际运行策略；单次平地到达和稀疏碰撞不代表成功率或难度排序；S2-04 稀疏运行只能确认 robot geom 42 与 ppt_obstacle_03 接触，不能确定具体肢体或冲击强度。修改任何受支持场景 XML 或资源后须同步更新目录身份。MuJoCo 命令消费等既有 UNKNOWN 保留。历史失败、INVALID 和原始运行数据不改写。详情见 [S2-04 执行报告](evidence/S2-04/S2-04_EXECUTION_REPORT.md) 和[单次场景使用说明](evidence/S2-04/SINGLE_RUN_GUIDE.md)。S2-04 审阅后当时没有 Active Task；随后 S2-05 获得授权并成为当前唯一 Active Task。

### S2-05 实际进展（2026-10-10）

新 v5 collision authority 把足端 contact/impact 与非足端终止碰撞分开，严格按四个命名球体识别足端，力按 contact frame 转世界坐标并逐物理步汇总已知外力。v5 使用 14,472-byte snapshot，扩展起点 5,760；reader 拒绝版本/长度错配并保留 v3/v4 旧语义。记录器与分析器输出 navigation terminal、nonfoot failure、foot contact/impact、arrival flags 与 ABS 参考碰撞；时间线区分三类事件。定向 C++/跨语言检查和必要构建通过。

唯一一次 sparse 诊断 [local run ID omitted] 为 VALID / COLLISION_TERMINATION，目标距离 2.1471 m，149 个策略帧，49.9956 Hz。实际记录到 RL 足端与 ppt_obstacle_03 接触和撞击：Fxy=216.8427 N、abs(Fz)=35.0042 N、阈值=80.0084 N；同时无名 robot geom 46 / body 12 RL_calf 与同一障碍接触，非足端事件触发终止。因此导航终局是碰撞，foot contact=1、foot impact=1、ARRIVED flags 为 false、ABS reference collision=true。足端 episode 覆盖五个物理步，完整采集约 0.010 s；event/snapshot 结束于 29.638 s，terminal policy 周期为 29.636 s，运动窗口 duration 保持 UNKNOWN。controller PASSIVE 确认、完整清理事实与任务进程核对均通过。时间线、结果表、原始记录与跨语言探针详见 [S2-05 执行报告](evidence/S2-05/S2-05_EXECUTION_REPORT.md)。

S2-05 已获 Director 有条件通过并标记 COMPLETE。唯一 run 不代表切换性能或总体避障能力。实际没有单独出现轻微足端障碍接触；v3 历史二进制未找到，仅做精确布局兼容 fixture 和历史 v4 JSON 记录接受核验。运动边界 duration、MuJoCo 命令消费、其他场景和正式性能均有适用限制，详见报告。任务进程已关闭。后续用户授权的 S2-06 已成为当前唯一 Active Task。

### S2-06 最终状态（Director 有条件通过，2026-10-10）

S2-06 的五套固定布局已生成 `scene_random_01.xml` 至 `scene_random_05.xml`，接入十项共用场景目录、`--scene`、MuJoCo、ray2d、碰撞 authority、逐周期记录、分析和时间线。MuJoCo 生产加载与十场景 authority 探针通过。五条浅蓝保留通道仅作几何预检，没有输入控制器；控制仍由目标、状态观测和 11 条射线驱动。

五个场景各取得一次有效 A 组诊断，五次都在 0.5 m 到达半径内 ARRIVED。场景 01、02、03、05 未发生 Recovery。场景 04 按运动周期记录共发生 136 次模式变化（Agile→Recovery 68 次、Recovery→Agile 68 次）；其中 128 次具有有效仿真时间，8 次时间无效。时钟无效不会抹去已记录的模式变化，但不能用于精确时间指标。五场景统计均区分起点至终点经过时间、有效连续间隔累计时间，以及只覆盖有效连续片段的路径长度和平均速度；场景 04 分别为 11.760 s 和 11.108 s。原 117 次短反向切换仅描述分析器既有 0.5 s 指标范围，不是全部切换总数。所有 run 均有 PASSIVE 确认和清理事实，原始记录、指标与时间线已保存。第 4 场景箱体按冻结 JSON 原坐标超出声明 y 下界约 0.0564 m，未移动；跌倒判断仍为 UNKNOWN。

Director 已对 [S2-06](tasks/S2-06.md) 有条件通过并标记 **COMPLETE**。通过范围限于场景接入、单次诊断及记录/指标链可复核。每场景一次非正式观察不能计算成功率、比较场景难度或宣称 A 组性能/切换效果通过。详见 [S2-06 执行报告](evidence/S2-06/S2-06_EXECUTION_REPORT.md) 和[单次对照表](evidence/S2-06/diagnostic_runs_20261010/scene_comparison.md)。**历史状态（S2-06审阅时）：**当时没有已授权的下一项 Active Task；当前状态见本文件顶部。

## 历史收尾记录（2026-10-07，保留）

以下内容只描述 2026-10-07 当时状态。后续 S1-03 已将仿真时钟接入并在正常连续运行中记录配对；这不改变当时记录，也不表示暂停恢复或时钟重连已验证。

- S1-01 切换合同已通过审阅；S1-02 基础接口、策略周期与仿真时钟核对已获 Director 有条件通过。
- S1-02 的非正式诊断记录了 1,848 个连续策略帧、约 50.14 Hz 的整体策略频率，以及 Agile 与 Recovery 的进入和退出；这些记录不计入正式实验。
- 截至当日，Agile 和 Recovery 部署模型的真实动作关节顺序仍记为 UNKNOWN（2026-10-10 的多源证据更新见上文；正式实验前仍须处理来源绑定限制）。MuJoCo `sim_time` 当时尚未接入控制器，合同中的时间保持尚未实现和核验。
- 今日工作到此停止；没有已授权的下一项 Active Task，未启动 S1-03。
