# R1：切换约束作用与代价研究

当前运行目录中的R1地图使用`R1_01.xml`–`R1_12.xml`，顺序为d09宽/中/窄、m02宽/中/窄（v1后接v2）；旧名仍可输入。见[完整地图文件名对照](../MAP_FILENAME_CROSSWALK.md)。

## 最新批次：Periodic A/B双阈值比较（2026-11-03）

45项冻结排程已结束，43项取得有效任务结果，2项启动故障未重试；到达20、碰撞12、仿真限时11，安全完成TRUE/FALSE/UNKNOWN为20/23/2。d09与m02上的B_E04_X07呈现局部改善线索；challenge27两档B到达均少于A；同E的B_E05_X10未显示跨图一致改善。样本每格4–5项，均为开发图，尚不支持选择B进入新布局验证。当前仿真保持几何射线＋Periodic P=0.06；范围仅A/B，C/D暂不加入。深度预测优化暂停，实机导航由独立规划器负责。R1保持ACTIVE并等待审阅。见[分析报告](../../evidence/R1/PERIODIC_AB_45/ANALYSIS_REPORT.md)、[逐次结果](../../evidence/R1/PERIODIC_AB_45/per_run_results.csv)、[九格汇总](../../evidence/R1/PERIODIC_AB_45/map_setting_summary.csv)、[冻结运行计划](../../../../logs/09_R1_switch_constraints/periodic_ab_45/20261103_r1_periodic_ab_45/run_plan.json)。

## 已完成阶段：射线预测输入复核（2026-11-03）

已核对当前MuJoCo相机与微调输入代码、深度/标签语义和控制器射线定义；未发现直接实现错误。固定12姿态误差仍显示开阔右侧束偏短与challenge27误差较大。4项有限动态观看中scene_flat几何/预测到达、challenge27几何到达而CUDA预测碰撞障碍02；无200ms过期中止。当前优先一次ray-only有限微调后复评，不扩大控制策略训练。见[输入复核报告](../../evidence/R1/GO2_RAY_PRED_12/input_link_review/RAY_INPUT_REPAIR_REPORT.md)、[固定姿态](../../evidence/R1/GO2_RAY_PRED_12/input_link_review/FIXED_POSES.csv)、[动态结果](../../evidence/R1/GO2_RAY_PRED_12/input_link_review/DYNAMIC_RESULTS.csv)。R1仍ACTIVE，实机未连接。

## 历史阶段：challenge27 A/C × Frozen/Periodic本机小批量完成（2026-11-02）

阶段一只读分析已通过审阅；Periodic Twist最小实现P=0.06仿真秒、Frozen默认。challenge27 A/C×Frozen/Periodic 40项作为历史结果保留。原12项动态射线比较已完成：两项几何先行ARRIVED，第3项CPU预测因帧龄218.217ms超过200ms触发`SYSTEM_SAFETY_ABORT`并保留，第4—12项已续跑。后续定位为CPU推理长尾及预测器子进程环境遮蔽CUDA；修正后独立60秒最大帧间隔76.22ms，一次独立scene_flat动态核验ARRIVED，419个周期读取有效预测射线、最大间隔57.24ms，无过期中止，但安全完成UNKNOWN且收尾记录DDS timeout FATAL。该独立核验不计入原批次；原剩余9项已按冻结排程续跑完成。详见[Go2核验报告](../../evidence/R1/GO2_RAY_PRED_12/STATUS_REPORT.md)和[准备计划](GO2_REAL_PREPARATION_PLAN.md)。R1仍为唯一ACTIVE，实机未连接。

## 上一批结果：图05 A/C保持时间比较（2026-11-02）

图05 `r1_ch15_left_x05_v2.xml` 上A H=0与C H=0.02/0.10/0.12，每设置10次，40/40有效记录、40次场景启动，无补做或安全中止。实际E/X均−0.05、滞回关闭；A到达3/10，C三档分别1/10、2/10、2/10；碰撞分别7/10、9/10、8/10、8/10，32次均接触03号障碍。上一批图05的H=0.02为2/10，本次1/10，正向差异没有重现；较长H仅比H=.02多一次到达，仍低于同期A，区间很宽，不能选定代表H。分析、逐次表、保持审计、排程及原始清单见[结果报告](../../evidence/R1/CH15_X05_HOLD_40/20261102_r1_ch15_x05_hold_40/RESULTS_SUMMARY.md)、[实际启动与清理清单](../../evidence/R1/CH15_X05_HOLD_40/20261102_r1_ch15_x05_hold_40/actual_scene_starts.csv)、[冻结计划](../../../../logs/09_R1_switch_constraints/ch15_x05_hold_40/20261102_r1_ch15_x05_hold_40/run_plan.json)。R1保持ACTIVE，等待审阅并停止，不追加实验。

## 地图版本：新增圆柱并加宽01—09净空20cm

用户确认取消长墙，在01与05之间加入11号圆柱（直径0.70m、高0.55m），并将09沿远离01的方向外移20cm。09新中心(3.683290,1.473857)m，01—09轮廓净空从约0.895m增加到1.095m。03六档前后位置保留，六图同步修改，无重叠。地图说明与本批实测路线见[六图说明](challenge15_left_route_v2/README.md)。

## 2026-11-02：六张布局生成阶段记录（后续已接入并完成比较）

已按用户确认生成challenge15的六张前后位置变体：只将03号圆柱x移动−0.30/−0.20/−0.10/+0.10/+0.20/+0.30m，横向位置及其他障碍不动。六份XML、单图预览、同尺度总览和位置表已生成；解析、几何差异及无重叠检查通过。该段记录地图刚生成时的待接入状态；之后已完成场景身份登记、必要构建及120项A/C比较，现行结果见本页开头。原challenge15及旧结果未改。[布局与预览](challenge15_x_variants/README.md)。

以下保留既有实验与规划记录，历史状态不代表当前仍待执行。

更新：2026-11-02。状态：**ACTIVE / 阶段一通过；Periodic Twist单图小批量40项完成，等待审阅**。这是独立于S7的后续项目；三图120项记录作为上一批保留。R1-5至R1-11、旧280项及新120项结果分批保留。

## 研究目标

通过已有问题分析，建立符合机器人感知和运动能力的高难度地图，找到合适的退出阈值X和最小保持时间H，使切换约束产生实际任务收益。进入阈值E可重新讨论，候选设置须与原实验分开记录。

频繁、反向切换是正常现象。主要看安全完成、碰撞、到达、超时、推进与耗时，切换次数仅用于解释。

## 已有产物

| 阶段 | 内容 | 入口 |
|---|---|---|
| R1-1 | M3中A组失败、C/D超时分析 | [报告](../../evidence/R1/M3_event_progress_20261030/分析报告.md) |
| R1-2 | Recovery碰撞方向分析；后退问题暂缓 | [报告](../../evidence/R1/M3_recovery_collision_20261030/分析报告.md) |
| R1-3 | M2同图历史A/B/C对照，跨批次开发线索 | [报告](../../evidence/R1/M2_A_failure_comparison_20261030/分析报告.md) |
| R1-4 | 三图A/B/C各10次，共90项开发确认 | [报告](../../evidence/R1/M2_ABC_repeat_90/分析报告.md)、[逐次结果](../../evidence/R1/M2_ABC_repeat_90/逐次运行明细.csv) |
| R1-5 | 五张M2 selected_v2地图、四种阈值设置、各6次，共120项，完成待审阅 | [结果报告](../../evidence/R1/M2_threshold_position_120/分析报告.md)、[冻结清单](../../../../logs/09_R1_switch_constraints/m2_threshold_position_120/20261030_r1_m2_threshold120/run_plan.json) |
| R1-6 | 六张R1v2地图、A_REF与三种双阈值、各3次，共72项，完成待审阅 | [结果报告、分层图与轨迹](../../evidence/R1/R1v2_threshold_72/分析报告.md)、[运行清单](../../../../logs/09_R1_switch_constraints/r1_v2_threshold_72/20261031_r1_v2_threshold72/run_plan.json) |
| R1-7 | m02 v2三图、两种双阈值、各3次，共18项，完成待审阅 | [补测报告、六候选对照和轨迹](../../evidence/R1/R1_m02_threshold_refine_18/分析报告.md)、[运行清单](../../../../logs/09_R1_switch_constraints/r1_m02_threshold_refine_18/20261031_r1_m02_threshold_refine18/run_plan.json) |
| R1-8 | m02 v2三图、六种宽度0.04双阈值、各3次，共54项，完成待审阅 | [54项报告与图表](../../evidence/R1/R1_m02_width04_54/分析报告.md)、[运行清单](../../../../logs/09_R1_switch_constraints/r1_m02_width04_54/20261031_r1_m02_width04_54/run_plan.json) |
| R1-9 | m02 v2三图、五种宽度0.05双阈值、各5次，共75项，完成待审阅 | [75项报告与图表](../../evidence/R1/R1_m02_width05_75/分析报告.md)、[运行清单](../../../../logs/09_R1_switch_constraints/r1_m02_width05_75/20261031_r1_m02_width05_75/run_plan.json) |
| R1-10 | m02 v2三档×3个B候选、各10次，共90项，完成待审阅 | [90项报告](../../evidence/R1/M02_CONFIRM_90/分析报告.md)、[运行清单](../../../../logs/09_R1_switch_constraints/r1_m02_confirm_90/20261031_r1_m02_confirm_90/run_plan.json) |
| R1-11 | m02 v2宽/中/窄及d09 v2窄档×A与两档B、各10次，共120项，完成待审阅 | [120项报告、汇总及轨迹](../../evidence/R1/AB_TWO_CANDIDATES_120/分析报告.md)、[批次清单](../../../../logs/09_R1_switch_constraints/ab_two_candidates_120/20261031_ab_two_candidates_120/run_plan.json) |
| A/C HOLD 280（上一批） | 五张困难图×A及六档C，每格8项；280项计划与280次启动均有记录，含1项保留的系统安全中止 | [结果报告及表图](../../evidence/R1/AC_HOLD_280/RESULTS_SUMMARY.md)、[35格表](../../evidence/R1/AC_HOLD_280/map_setting_35.csv)、[批次清单](../../../../logs/09_R1_switch_constraints/ac_hold_280/20261101_r1_ac_hold_280/run_plan.json) |
| A/C HOLD确认120 | challenge15、challenge27、m02中档×A及C三档H，每格10项；120/120记录有效、无工程补做，等待审阅 | [结果报告、逐次表及新旧对照](../../evidence/R1/AC_HOLD_CONFIRM_120/20261101_r1_ac_hold_confirm_120/RESULTS_SUMMARY.md)、[12格表](../../evidence/R1/AC_HOLD_CONFIRM_120/20261101_r1_ac_hold_confirm_120/map_setting_12.csv)、[批次清单](../../../../logs/09_R1_switch_constraints/ac_hold_confirm_120/20261101_r1_ac_hold_confirm_120/run_plan.json) |
| CH15 x05 A/C HOLD 40 | 单图A与C H=0.02/0.10/0.12，各10次；40/40有效，A到达3/10、C分别1/10、2/10、2/10，碰撞均接触03号障碍；等待审阅 | [报告、逐次表及保持审计](../../evidence/R1/CH15_X05_HOLD_40/20261102_r1_ch15_x05_hold_40/RESULTS_SUMMARY.md)、[实际排程和启动清单](../../evidence/R1/CH15_X05_HOLD_40/20261102_r1_ch15_x05_hold_40/actual_schedule.csv)、[批次计划](../../../../logs/09_R1_switch_constraints/ch15_x05_hold_40/20261102_r1_ch15_x05_hold_40/run_plan.json) |
| CH15 A/C H=0.02确认120 | 六张challenge15同源布局×A/C，每格10项；120/120记录有效，A到达13/60、C到达14/60，结果方向不一致，等待审阅 | [结果报告及图表](../../evidence/R1/CH15_AC_120/20261102_r1_ch15_ac_120/RESULTS_SUMMARY.md)、[12格表](../../evidence/R1/CH15_AC_120/20261102_r1_ch15_ac_120/map_setting_12.csv)、[批次清单](../../../../logs/09_R1_switch_constraints/ch15_ac_120/20261102_r1_ch15_ac_120/run_plan.json) |

R1-9完成75项：到达17、碰撞57、仿真限时1，碰撞均指向障碍03。宽/中/窄到达为7/25、4/25、6/25；E01/X06为5/15、E05/X10为4/15，其余2–3/15。中心轨迹多进入03—07近似截面，但不能确认整机净空。三档暂留，E01与E05仅作后续审阅候选；跨批次宽度对照不能作因果解释。R1-4原有结果保持不变。

## 新路线与下一步

重点研究连续障碍间距、有限空间通道宽度，通过少量有规律的布局变化找到参数的适用条件。单障碍不再做完整选参，只保留必要的耗时比较。

用户已选d09、m02，v1六档地图完成A组观看后调整为[六图v2](generated_maps_v2/README.md)。六图v2已完成72项分层比较，39次到达、33次碰撞；d09未出现B设置超过A，m02的B_E04_X07呈现局部改善线索，B_E06_X09略弱，不能外推为普遍收益。详见[R1v2结果报告](../../evidence/R1/R1v2_threshold_72/分析报告.md)。每格仅3次，仍等待结果审阅；不自动开展H或独立验证。

本机负责设计和小规模选参，后续批量验证优先使用实验室服务器并在服务器保存原始数据。本机主要取回汇总与典型案例，减轻磁盘占用。服务器环境尚未核实；分工和迁移安排见[计划第6节](PLAN.md)。

## 当前决策与数据

R1-8完成54项：到达6、碰撞48，全部碰撞记录指向障碍03；宽/中/窄到达为1/18、2/18、3/18。窄档B_W04_E01_X05有2/3到达的局部线索，其余宽度0.04设置至多1/9，无跨布局一致改善。同E宽度0.03对照来自不同批次，不作因果或显著性判断。与R1-5、R1-6及R1-7报告一并审阅；R1保持ACTIVE，不自动追加实验。

R1-10完成三候选×m02 v2宽/中/窄×10次，共90/90记录VALID；实际启动90次、补做0次，到达17、碰撞72、仿真限时1。三个候选到达为5/30、4/30、8/30；E05/X10与R1-9聚合比例相同，是弱线索重复，不足以称为改进。72次碰撞均接触障碍03，中心经过03—07近似截面不能证明整机净空。停止在当前三图扩展E/X网格，暂不自动运行H或新图批次；R1保持ACTIVE、等待审阅。详见[90项报告](../../evidence/R1/M02_CONFIRM_90/分析报告.md)、[9格汇总](../../evidence/R1/M02_CONFIRM_90/地图设置汇总.csv)、[每轮趋势](../../evidence/R1/M02_CONFIRM_90/每轮到达变化.png)、[轨迹](../../evidence/R1/M02_CONFIRM_90/代表轨迹与障碍.png)及[原始批次清单](../../../../logs/09_R1_switch_constraints/r1_m02_confirm_90/20261031_r1_m02_confirm_90/run_plan.json)。

R1-11完成A_REF和两档B在m02宽/中/窄及d09窄档上的120项同期比较，120/120记录VALID、补做0次；到达36、碰撞83、仿真限时1。m02各设置30次到达为5/30、4/30、7/30；d09各10次为6/10、6/10、8/10。B_W05−A到达率差分别+6.7和+20个百分点，但95%区间分别−13.8至+26.6及−18.7至+52.1，未验证普遍改善。m02碰撞主要指向障碍03，d09主要指向障碍02。两类地图分层报告，不把四张开发图解释成一般成功率；所有运行已清理。详见[120项报告与轨迹图](../../evidence/R1/AB_TWO_CANDIDATES_120/分析报告.md)及[批次清单](../../../../logs/09_R1_switch_constraints/ab_two_candidates_120/20261031_ab_two_candidates_120/run_plan.json)。R1-11完成后等待结果审阅，不自动启动H或新布局验证；当前状态见本页下方A/C保持批次记录。

上一批（280项）：2026-11-01启动A参考下C保持时间比较，五张困难图×A H=0与C六档H，每格8次，共280项。第29项challenge08/C_H0.04触发控制器POLICY_SAFETY_VETO：策略周期37的ray帧序列在复制期间由11944变为11946，读取耗时1.17 ms，超过现有250 μs预算；有界重读机制已存在，但该次读取未能在预算内获得一致快照。控制器依既有安全规则请求PASSIVE。未发现读取实现错误，不改代码、不放宽安全检查。经本次明确授权，从第30项完成剩余251次；现有280项计划全部有对应运行记录，终局为到达96、碰撞145、仿真限时37、跌倒1、安全中止1，安全完成TRUE/FALSE/UNKNOWN为96/184/0。第29项仍作为`STOPPED_FAILURE / SYSTEM_SAFETY_ABORT`单列，分母保留，不重跑或替代。完整报告见[结果报告](../../evidence/R1/AC_HOLD_280/RESULTS_SUMMARY.md)、[逐次结果](../../evidence/R1/AC_HOLD_280/per_run_results.csv)、[35格设置表](../../evidence/R1/AC_HOLD_280/map_setting_35.csv)、[图表目录](../../evidence/R1/AC_HOLD_280/charts/)及[批次记录](../../../../logs/09_R1_switch_constraints/ac_hold_280/20261101_r1_ac_hold_280/run_plan.json)。每项shutdown/closeout均确认；运行器、仿真、ROS/controller、RViz及辅助进程均无残留。R1保持ACTIVE，停止等待审阅，不自动启动D或其他批次。

上一批（三图确认120项）：challenge15、challenge27、r1_m02_space_middle_v2各比较A H=0和C H=0.02/0.10/0.12，每格10次。120项记录有效、实际启动120次，终局到达32、碰撞58、仿真限时29、墙钟保护1，安全中止0；安全完成TRUE/FALSE/UNKNOWN为32/87/1。总体率差的区间仍包含0，逐图只在challenge15看到方向重复，H=0.10/0.12仍不能选定。H=0.02保持审计识别出440/1528个episode在H到期前RA已低于E且随后退出；实际驻留中位数0.040秒。challenge08此前长保持退化继续保留为限制。详见[120项结果与新旧对照](../../evidence/R1/AC_HOLD_CONFIRM_120/20261101_r1_ac_hold_confirm_120/RESULTS_SUMMARY.md)。本批清理后约1.6GiB；四份配置快照保留，临时overlay清理，每项进程均退出。R1保持ACTIVE、等待审阅，不自动追加H、D或新地图批次。


- [原始90项批次与冻结清单](../../../../logs/09_R1_switch_constraints/m2_ABC_repeat_90/20261030_r1_m2_abc90/)
- [全项目状态](../../CURRENT_STATE.md)、[实验记录导航](../../../../logs/README.md)

原T1/T2/S7资料保留原归属；本项目产物在evidence/R1和logs/09_R1_switch_constraints中。R1-5的仿真、controller、ROS、RViz和运行器进程已确认退出；批次结束后的进程扫描未发现残留。

## 2026-10-31收尾

[当日进度与下次续接](SESSION_20261031.md)：R1-11完成，暂留E05/X10作有限候选；H短保持60项仅为建议，尚未授权或运行。R1继续ACTIVE，原始日志与本地周报不上传。
