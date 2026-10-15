# M1 执行报告：地图修订、重复核验与候选冻结

**最新状态：COMPLETE（Director有条件通过，2026-10-15）。** 通过范围为 `m1-mapset/1.0` 的地图内容、命名、开发/留出划分、运行前冻结规则及开发集每图3次的流程记录。此批是开发地图流程和运行链核验，不是正式实验。

**冻结与解释边界：**`m1-mapset/1.0` 的地图内容、命名、开发/留出划分及运行前确定的规则保持固定；留出集继续不运行策略。90条计划项均有记录，每张开发图3次。2条 `WALL_CLOCK_GUARD_INCOMPLETE` 为运行不完整，不作为完整导航结果，也不是普通仿真超时。`VALID` 仅表示来源/记录校验通过，不表示全部指标完整。78次 `ARRIVED` 不等于78次已证实安全且全程界内到达。原始INVALID、终局、运行记录及其限制不改写。

## 结论摘要

- Director通过阶段1–2图审，接受目标 `(0,0)→(7.5,0)m` 和声明范围 `x=[-0.5,8.0]m, y=[-3,3]m`。场界不是实体墙。
- 阶段3生成60张固定地图，30张开发、30张留出；几何、版本和生产身份检查通过，MuJoCo 3.3.3实际C++模型指纹与60/60生产publisher→Python reader握手已核对。实际MuJoCo主程序构建成功。
- 阶段4对30张开发图按轮转顺序各执行3次，90条计划项均有运行记录；每张开发图3次。另有一次来源基础设施启动失败保留为INVALID，定向修正后只重试该计划项一次，共实际发起91次诊断进程。留出集0次策略运行。
- 90条计划项对应运行的supervisor终局为：78 ARRIVED、8 COLLISION_TERMINATION、2 DIAGNOSTIC_SIM_TIME_LIMIT、2 WALL_CLOCK_GUARD_INCOMPLETE。两条墙钟保护记录属于运行不完整，不作为完整导航结果，也不是普通仿真时间上限。所有90条的首个活跃策略cycle 0均持久化。80条完成到达后站立观察；8条碰撞和2条墙钟保护没有完成站立观察。所有运行由诊断器报告完成关闭；结束后的独立进程核对见“环境收尾”。
- ARRIVED是原始任务终局。由于88条越界覆盖为 `UNKNOWN_INCOMPLETE_COVERAGE`、另2条因墙钟保护没有可评边界窗口，本批没有证据把这些ARRIVED进一步归为“声明区域内有效到达”。未观察到越界策略周期样本，但这不等于确认全程无越界。

## 版本、规则与入口

阶段1–2获批后，阶段3–5使用以下运行前冻结版本；阶段5没有因运行结果修改几何规则、阈值或时限：

| 项目 | 冻结值 |
|---|---|
| 地图库 | `m1-mapset/1.0` |
| 生成器 | `m1-map-library/1.0` |
| M1几何规则 | `M1-rules-stage3-5/1.0`；SHA-256记录于运行批次计划 |
| M1 A运行合同 | `M1-development-A/1.0`；首次策略运行前记录于批次计划 |
| 场地与起终点 | x=[-0.5,8.0]m、y=[-3,3]m；(0,0)→(7.5,0)m；场地边界非实体墙 |
| 到达及运动上限 | 半径0.5m；20仿真秒；120墙钟秒 |
| 到达后站立 | 5仿真秒；30墙钟秒上限 |
| A组 | E=-0.05；无滞回、无最小保持 |
| 公共启动增益交接 | 0.30 controller运行秒，从实际入口增益连续到策略增益；不是Recovery保持时间 |
| 安全及策略计时 | 普通跌倒、安全与碰撞规则；不启用启动低高度延期。训练/策略输入中的9秒归一化episode timer保持原样，不作为本任务运动硬上限 |
| 越界口径 | 每个有效策略周期记录base XY与身份；超界样本单列；无效时钟/缺周期不插值。supervisor终局不改写；有观测越界的ARRIVED不算声明范围内有效到达，覆盖不足保持UNKNOWN |

12条 S6-11 历史A运行使用7.0m目标，目标/任务语义与本批不相容，计入本次开发重复数为0。布局种子与运行记录分开保存；没有声称不同run使用完全相同初态或严格重放。

## 阶段3：地图生成和生产接入

地图数据、单图预览及清单位于 [`designs/M1/library_v1/`](../../designs/M1/library_v1/)；旧图和阶段2原型不覆盖。每个密度×路线结构单元有4图（2开发、2留出），按布局文件内预先固定的独立种子划分。

- 5类结构：短障碍单侧绕行、缓和S型连续绕行、错位短障碍通道、短障碍局部绕行、前向错列短障碍；每类覆盖稀疏/中等/密集，每档开发2、留出2。
- 稀疏/中等/密集是互斥的投影占地比例标签；另存障碍实际间距、尺寸、路线长度、绕行比和净距。它们不是风险或难度等级。
- 主要障碍在全库中改变位置、尺寸、朝向和错列方式；变化不限于外围填充物。42/60张起终点直线有障碍阻挡。
- Go2保守半径0.65m加0.10m工程余量，参考路线至真实障碍净距至少0.75m；0.05m栅格找路，0.025m沿实际旋转箱体/圆柱轮廓连续核验。起终点净距、边界、重叠、间距、通路与有限重采样均纳入验证。包络尚未经步态实测。
- S型均标为“可行参考路线”，仅用于展示和几何核验，不输入策略。机器人可选择更直接的局部绕行，未为迫使S型通过而新增封堵。
- 两张总览：[开发集30张](../../designs/M1/library_v1/overview_development_30.png)、[留出集30张](../../designs/M1/library_v1/overview_held_out_30.png)。总览和每图预览是静态几何图，不是机器人轨迹或仿真截图。

生产验证：布局复现与几何检查报告 `verify_m1_library.py` 通过；60个短内部ID唯一且满足32字节字段长度；MuJoCo主程序及身份探针增量构建通过。60张均使用实际MuJoCo 3.3.3的C++生产指纹算法生成并绑定；publisher→Python生产reader 60/60返回LIVE，场景ID与障碍数量相符。射线和碰撞使用同一生产障碍选择器。结果位于 [`production_handshake_60.json`](production_handshake_60.json)。

**唯一地图入口和复用命令**：通过 `scripts/manual_m1_a_batch.py` 消费 `library_v1/layouts.json` 中的开发清单；批次计划固定轮转顺序、配置及独立结果目录。完成批次后的命令（仅显示已完成，不会覆盖记录）：

```bash
rtk proxy /usr/bin/python3 -u scripts/manual_m1_a_batch.py --resume --batch-root logs/manual_m1_a/library_v1
```

运行时逐图目标、时限、候选和图身份由runner传递，controller配置、run context、原始记录和分析器均核对目标 `(7.5,0)`。A组运行参数和90条结果索引见 [`m1_a_batch_plan.json`](../../../../logs/manual_m1_a/library_v1/m1_a_batch_plan.json) 及 [`development_A_results.md`](../../../../logs/manual_m1_a/library_v1/m1_results/development_A_results.md)。

M1当前地图使用入口由 `scripts/manual_m1_a_batch.py` 读取 `designs/M1/library_v1/layouts.json` 的开发清单；旧测试地图不属于该M1清单/批次入口。历史场景XML、资源及其引用因既有记录需要继续保留，不作批量删除。

## 阶段4：开发集A组运行

### 来源故障和限定重试

计划项 `m1sfd01 / repeat 1` 的首次诊断未进入策略，`ros2 pkg prefix go2_description` 仍选原安装路径。检查发现对比器用共享detached overlay做了配置预检，但启动诊断时传入了不存在的逐次overlay路径。修正为预检和启动共用同一解析路径，并做Python编译、路径helper与同shell ROS包来源定向核验后，仅重试这一个计划项。重试到达且记录VALID。首次 `STARTUP_FAILURE` 和原始文件不改写，分别保存于：

- [首次INVALID记录](../../../../logs/manual_m1_a/library_v1/m1sfd01/rep_01_attempt_01/)
- [定向修复后的有效重试](../../../../logs/manual_m1_a/library_v1/m1sfd01/rep_01_attempt_02/)

### 结果和覆盖

| 指标 | 本批观测 | 解释边界 |
|---|---:|---|
| 有记录的计划项 / 开发图 | 90 / 30 | 每图3次；另有一次基础设施失败启动及其单次修复重试。`VALID`只表示来源/记录校验通过，不保证各指标完整 |
| supervisor终局 | 78 ARRIVED、8碰撞终止、2仿真时间上限、2墙钟保护不完整 | 两条 `WALL_CLOCK_GUARD_INCOMPLETE` 不是完整导航结果，也不是普通仿真超时；每图3次不用于成功率或性能排名 |
| ARRIVED解释 | 78次 | 原始任务终局；不等于78次已证实安全且全程界内到达，越界覆盖限制见下文 |
| 到达后站立观察 | 80完成、10未到达或未知 | 两个仿真时间上限运行完成站立观察；碰撞和墙钟保护运行未完成 |
| 策略频率 | 中位数约50.000Hz，范围49.676–50.553Hz | 90条记录按实际周期时间计算；不是无记录开销对照 |
| 模式边沿 | 3,172次；Agile→Recovery 1,586次、Recovery→Agile 1,586次 | 保留所有记录模式边沿；不可用时钟边沿不用于精确驻留时间 |
| Recovery | 6,576个策略周期；可计时连续片段合计127.308s | 仅为本批有效连续片段汇总；Recovery最小保持规则在A中未启用 |
| RA | 已记录值范围−0.99963至+0.65460 | 31,752个读取到RA的周期值；不表示风险分布或性能 |
| 0.5秒短反向切换 | 已统计2,896次 | 88条运行覆盖为partial/unknown，2条UNKNOWN，不是全批完整次数 |
| 周期/时间缺口 | 333个策略周期缺口；534个时钟无效帧 | 不补插；对应时间类指标按各run覆盖声明解释 |
| 任务跌倒 | 48条False、42条UNKNOWN、0条确认True | UNKNOWN不解释为未跌倒 |
| 碰撞覆盖 | 78条无碰撞事件记录、7条碰撞为True、5条UNKNOWN | 8条supervisor碰撞终局中一条未能在分析安全事件链完整确认为True；不得写成碰撞否定覆盖完整 |
| 足端事件 | 85条覆盖可评的运行中记录足端接触合计27次、足端撞击26次；另5条安全覆盖UNKNOWN | 次数是周期/事件汇总，不是独立地图/运行数；详细事件保留在metrics与原始记录 |
| 声明范围越界 | 0个已观测策略周期越界样本；88条UNKNOWN，2条NOT_APPLICABLE | 未采到越界样本不证明物理连续轨迹、机体/足端全程在界内；无越界否定结论 |

有效运动连续片段路径长度中位数7.372m，平均速度中位数1.179m/s；这些指标来自连续有效相邻样本，仅报告有数据的片段。终止周期纳入运动窗口，站立与清理尾段排除。到达距离中位数约0.490m，ARRIVED条件为进入目标0.5m范围。轨迹和时间线按每次运行单独保存；结果索引链接每次记录目录和指标，目录中含 `timeline.png`、`runtime_record.jsonl`、`run_context.json`、`terminal.json`、`metrics.json` 与清理证据。

碰撞已记录的障碍geom为 `ppt_obstacle_01` 3次、`ppt_obstacle_02` 3次、`ppt_obstacle_03` 1次；第8个碰撞终局的分析碰撞事件链覆盖不足。robot geom名称在部分记录为空，不能从本报告推断具体肢体。两条 `WALL_CLOCK_GUARD_INCOMPLETE` 跑到约12–13秒仿真时间即触发已冻结120秒保护；PASSIVE/停止确认有直接日志，终点距离和边界不作推断。这与 `DIAGNOSTIC_SIM_TIME_LIMIT` 区分报告。

这批记录中的 `VALID` 只指来源/记录链校验通过，不代表所有指标覆盖完整。90条计划项均有记录，实际初态和RL cycle0 companion在90/90记录中保存；各run之间是相同准备流程，不宣称初态完全一致。两个墙钟保护运行保持不完整分类。失败、碰撞、时钟/周期缺口、未知跌倒和越界UNKNOWN均保留；没有删除或修改任何地图以迎合A组结果。

## 阶段5：M1地图库冻结（Director有条件通过）

Director已对以下地图集和本批流程给出有条件通过。`m1-mapset/1.0` 现冻结；冻结不等于正式实验批准：

1. 地图内容及命名固定为 `m1-mapset/1.0` 的60张；开发/留出为布局中的30/30固定划分，留出未跑策略。新内容变化须升地图版本，不与旧ID或旧run混用。
2. 几何分类只按运行前冻结的边界、起终点净空、碰撞轮廓、重叠、通路包络与绕行结构规则。A失败不等于地图技术无效，A成功不证明总体研究有效；未按运行成绩增删或改图。
3. A诊断条件固定为E=-0.05、目标7.5m、半径0.5m、仿真/墙钟运动上限20/120秒、站立观察5/30秒、0.30 controller运行秒公共增益交接及普通安全规则。越界与UNKNOWN口径保持上述版本。
4. 后续B/C/D应使用相同地图、起点/目标、运行/安全规则及当前公共启动增益交接，不按组挑图。进入正式实验前仍需Director批准参数与统计方案；本报告不是正式结果。

**建议下一步**：先定向修正记录/指标覆盖并调查两条 `WALL_CLOCK_GUARD_INCOMPLETE` 的触发，再规划开发集小规模参数预实验；留出图继续封存，不用于选参。不要把本批结果作为成功率、难度排序或参数优选证据。

## 未解决限制

- 越界否定覆盖不足；边界非实体墙，策略帧也不能证明帧间物理轨迹始终在界内。
- 42条跌倒结论UNKNOWN；5条碰撞/安全覆盖UNKNOWN，另有一条碰撞终局与安全分析事件覆盖不完全对应。
- 0.5秒短反向统计绝大多数覆盖不完整；策略周期缺口和无效时钟限制Recovery驻留等时间类指标。
- Go2保守平面包络及工程余量未经步态实测；几何有路不等于策略可达。
- 有限仿真、不实机、不训练、不做正式实验；A组每图3次不是性能比较或成功率评估。
- PASSIVE controller确认不等于MuJoCo已消费命令；具体命令消费状态仍UNKNOWN。

## 修改、证据和清理

- 地图库/XML/图片：[`designs/M1/library_v1/`](../../designs/M1/library_v1/)、`unitree_mujoco/unitree_robots/go2/scene_m1_*.xml`。
- 生成与核验：`scripts/generate_m1_library.py`、`scripts/verify_m1_library.py`、`scripts/test_s6_11_scene_handshake.py`；runner/分析：`scripts/manual_m1_a_batch.py`、`scripts/manual_ab_compare.py`、`scripts/s1_run_diagnostic.py`、`scripts/s1_analyze_run.py`、`scripts/analyze_m1_a_batch.py`。
- 注册及构建：`common/abs_scene_catalog.def`、MuJoCo CMake目标。实际主程序`unitree_mujoco/simulate/build2/unitree_mujoco`已构建；M1运行绑定指纹均由实际MuJoCo 3.3.3 C++生产算法生成。
- 错误修正：`manual_ab_compare.py`现对共享overlay预检与诊断启动使用同一个解析路径；首次错误启动记录仍保留。
- 结果：[`development_A_results.md`](../../../../logs/manual_m1_a/library_v1/m1_results/development_A_results.md)、[`development_A_results.csv`](../../../../logs/manual_m1_a/library_v1/m1_results/development_A_results.csv)、[`m1_a_batch_plan.json`](../../../../logs/manual_m1_a/library_v1/m1_a_batch_plan.json)。
- 每条诊断结束时process_facts均报告 `shutdown_complete=true` 与 `closeout_complete=true`。整个批次结束后检查确认本次run wrapper及相关MuJoCo/controller/RViz/ROS进程已退出；没有清理或影响其他会话。无Git提交。

