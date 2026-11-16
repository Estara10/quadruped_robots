# Go2接口适配与射线预测核验：阶段状态

更新：2026-11-02。R1仍是唯一ACTIVE任务。本报告记录已完成的离线实现、静态对比、渲染/启动修正及原12项动态核验的安全停止点；不代表实机验证或12项批次完成。

## 已完成的实现与离线检查

- `StateRL`现在要求显式`runtime_mode`：MuJoCo使用原仿真时钟，硬件模式使用进程启动后的单调时钟。Recovery进入起计、退出清除、重进重新起计，控制器重启建立新会话；H仍是最早退出时间约束。Periodic固定P=0.06秒，既有切换判定先于到期更新；墙钟仅测量优化耗时，实机运行时间不写入仿真时钟字段。
- Unitree状态回调分别记录LowState与SportModeState本机接收时刻。硬件读接口超过可配置100ms暂定时限或从未收到时将对应状态标成非有限值；硬件控制器在推理和控制输出前检查关节、IMU、足力和位置/速度接口，并复用既有安全Veto/PASSIVE路径。100ms是按当前高频订阅实现给出的暂定值，必须以实机接收率复核；本机接收时龄不证明传感器采样时钟可靠。
- `ray_predictor.py`使用显式指定的微调检查点；预测值按现有版本2 header、seqlock、单调完成时间戳和11个`log2(m)` payload写入`/mujoco_ray2d*`。`s1_run_diagnostic.py`预测模式关闭MuJoCo几何写入，并在启动控制器前等待一帧新鲜有效的预测数据；没有接管或静默几何回退。逐帧记录渲染、推理和写入耗时。几何模式保留现有几何源。
- Go2硬件插件、StateRL/controller构建通过；`s2_01_switch_config`为72项通过。`periodic_twist_schedule`定向检查通过，覆盖deadline前、首个到期、跨周期最多一次、退出/重进、时钟段改变及回拨。现有C++射线帧校验器编译运行退出码为0。新增Python文件通过语法检查。

这些是构建和离线代码检查，不能证明实机实时性或硬件消息新鲜度行为已验证。

## 模型与静态预测对比

选择[显式检查点](../../../../../archive/ray_pred_finetune/mujoco_finetune_soft_safety_20260611/ray_pred_mujoco_finetuned_best.pt)：清单显示它由medium+near两套数据、16,633样本（训练14,970、验证1,663）从medium checkpoint微调6轮，near权重5、false-safe权重10。没有使用缺失的默认文件做fallback，也未训练/下载新模型。模型在静态脚本中成功加载；输入是160×90米制深度、裁剪0.1–6m、`log2`后复制三通道，输出11维`log2`距离，索引0到10对应−45°到+45°、每9°一束。

几何参考已按场景注册表选择障碍：`scene_flat.xml`使用零障碍，`challenge27.xml`只使用注册的7个`ppt_obstacle_*`静态碰撞体；射线起点为机体局部x=−0.05m，量程0.1–6m，与控制器几何源定义一致。两类对比均在相同地图、位姿上用相同qpos分别算几何参考和预测；左右反序误差更大，没有迹象表明左右束顺序颠倒。

| 地图与静态姿态数 | 平均绝对距离误差 | RMSE | 最大误差 | False-safe / False-danger | 平均渲染 / 推理 |
|---|---:|---:|---:|---:|---:|
| challenge27，4 | 1.355m | 2.067m | 4.963m | 0 / 44，19 / 44 | 31.4 / 35.4ms |
| scene_flat，4 | 0.582m | 1.058m | 2.662m | 0 / 44，12 / 44 | 33.7 / 35.4ms |

静态样本中预测距离总体偏短，表现为偏保守和较多false-danger；这不保证动态或相机输入同样如此。CPU静态测量的渲染加推理约67ms/次，低于名义30Hz；它不是运行批次中的并发时延，也不证明控制周期可接受。

静态表和图：[`challenge27`](static/challenge27/ray_pred_compare.md)、[`scene_flat`](static/scene_flat/ray_pred_compare.md)。已将比较器和预测器导入顺序调整为先Torch、后MuJoCo；差异检查中原顺序会触发`free(): invalid pointer`，调整后的一位姿静态比较正常写完并以0退出，动态预测器也完成资源关闭、退出码0。证据支持问题与Torch/MuJoCo native库导入/卸载顺序有关；未吞异常、跳过释放或强退。

## 12项小批量动态核验：第3项安全中止后暂停

计划仍是两图×两种射线来源×每格3次，共12项；配置为A、E=有效X=−0.05、H=0、滞回关闭、Periodic P=0.06秒，目标(7.5,0)m、到达半径0.5m、20/120秒、到达后观察2秒。采用隐藏的surfaceless EGL pbuffer、RViz关闭、帧缓冲采集关闭。C++日志确认实际渲染器为NVIDIA GeForce RTX 4060 Laptop GPU；预测器用`MUJOCO_GL=egl`并实际输出预测帧。两种射线条件沿原固定排程交错；模型、地图、控制与安全条件未改。

| 项 | 冻结排程 | 实际状态 | 记录与安全分析 |
|---:|---|---|---|
| 1 | challenge27 / 几何 | 到达，VALID | 确认安全完成TRUE；终点0.492m，任务运动5.92仿真秒；无碰撞，任务窗跌倒FALSE。该计划项此前有4次未形成有效任务的启动/来源核验attempt，全部保留，最终有效终局只计这一次。 |
| 2 | scene_flat / 几何 | 到达，VALID | 安全完成UNKNOWN；终点0.468m，任务运动3.98仿真秒。碰撞/跌倒任务窗覆盖完整且均为FALSE；UNKNOWN来自scene_flat未声明有效有限场地边界，base_link越界维度不可用。 |
| 3 | challenge27 / 预测 | `SYSTEM_SAFETY_ABORT`，VALID记录 | 安全完成FALSE；碰撞、跌倒和任务终点为UNKNOWN。控制器周期30检测射线过期218.217ms（现有上限200ms），请求PASSIVE；未发生几何回退。单列保留，不重跑。 |
| 4 | scene_flat / 预测 | 未启动 | 排程保留。 |
| 5 | challenge27 / 几何 | 未启动 | 排程保留。 |
| 6 | scene_flat / 预测 | 未启动 | 排程保留。 |
| 7 | challenge27 / 预测 | 未启动 | 排程保留。 |
| 8 | scene_flat / 几何 | 未启动 | 排程保留。 |
| 9 | challenge27 / 预测 | 未启动 | 排程保留。 |
| 10 | scene_flat / 预测 | 未启动 | 排程保留。 |
| 11 | challenge27 / 几何 | 未启动 | 排程保留。 |
| 12 | scene_flat / 几何 | 未启动 | 排程保留。 |

因此是12个计划项中2个完成导航终局、1个有效安全中止、9个未启动；不是完成12项的比较。实际MuJoCo场景进程启动7次：计划项1前有4个已记录的工程启动/核验失败，随后该项到达；计划项2到达；计划项3安全中止。另有2次无场景进程的预启动编排错误。4个计划1失败attempt和全部原始日志均保留，没有把它们算作碰撞、超时或导航终局。冻结排程及每次启动状态见[运行计划](../../../../../logs/09_R1_switch_constraints/go2_ray_pred_12/20261102_r1_go2_ray_pred_12/run_plan.json)。

### 预测链路、时延与开销

预测模式启动时的预检为`FRESH_FRAME`；MuJoCo日志记载`Source: external/ray_pred`，运行上下文确认几何写入器关闭，射线读取故障行包含共享帧序号、时间戳和读取时龄。这确认预测数据进入控制器读取路径，没有静默切回几何射线。

安全中止前预测器已发布215帧：渲染平均1.16ms（最大6.17ms），推理平均17.92ms、P95 36.96ms、最大352.18ms，渲染到写出平均20.15ms、P95 38.75ms、最大354.81ms。发布间隔平均39.21ms（约25.5Hz），P95 56.52ms、最大369.96ms；214个间隔中8个超过100ms、4个超过200ms。触发点引用的最后帧是frame 214；它的推理耗时202.48ms。frame 215最终记录到246.69ms推理和253.08ms发布间隔，但其输出在控制器已经触发安全中止后才写出。控制器在frame 214写出后218.217ms触发既有200ms时龄保护。故障依据是长推理尾延迟造成预测停更，不是检查器误报；没有放宽新鲜度期限。

按已记录策略周期总耗时超过20ms统计：challenge27几何为2/297，scene_flat几何为4/200；预测安全中止前31帧中为11/31。预测运行策略周期耗时均值21.36ms、P95 36.86ms、最大131.90ms。Twist更新独立计量：challenge27几何17次（Recovery入口5、Periodic 12），优化耗时均值3.54ms、P95 10.33ms、最大20.10ms；预测中止前4次（入口1、Periodic 3），均值3.69ms、最大4.29ms。所有这些更新日志都标记已写入Recovery输入。scene_flat几何无Recovery更新。

渲染、推理和Twist优化是在并行进程/控制器中运行；现有记录未提供与每个策略周期的完整跨进程排程因果分析，不能把同时观测到的周期变慢全部归因于预测推理。预测器没有记录每张图像qpos读取的单独时间戳，也没有把某帧的位姿读取、渲染开始、写入和控制器首次消费完整关联；因此仅报告可测推理耗时与控制器读到的frame时龄，不把写入时龄冒充图像采集到控制使用的总延迟。任务窗运动距离、成功耗时、碰撞/跌倒及其安全完成不能从预测中止项恢复，保留UNKNOWN。

两项几何到达的代表轨迹见[challenge27几何到达轨迹](../../../../../logs/09_R1_switch_constraints/go2_ray_pred_12/20261102_r1_go2_ray_pred_12/runs/01_challenge27_geometric_attempt_05/A/rep_01/7c57fca825a54937ab6749f342cf7447/timeline.png)、[scene_flat几何到达轨迹](../../../../../logs/09_R1_switch_constraints/go2_ray_pred_12/20261102_r1_go2_ray_pred_12/runs/02_scene_flat_geometric_attempt_01/A/rep_01/9ca16b8cdff14c2ab3406d6c79a44143/timeline.png)；预测安全中止项的策略轨迹见[预测安全中止轨迹](../../../../../logs/09_R1_switch_constraints/go2_ray_pred_12/20261102_r1_go2_ray_pred_12/runs/03_challenge27_prediction_attempt_01/A/rep_01/2fc559108a734afcadc1f7ea0e3454b7/timeline.png)，预测帧及异常日志保存在对应run目录。几何运行任务窗分别为challenge27安全完成TRUE、scene_flat安全完成UNKNOWN；预测项安全完成FALSE，碰撞/跌倒UNKNOWN。静态对比仍显示预测距离偏短并有较多false-danger；本次动态中止只证实输出尾延迟越过既有安全期限，样本不足以判断动态射线误差或导航优劣。

启动问题的依据和修正也记在排程中：桌面X11 socket不可连接，且sandbox下SDK2无法枚举网卡；MuJoCo配置的DDS接口仍为`lo`。已用仓库显式可选的surfaceless EGL pbuffer后端解决窗口依赖，并通过正常权限申请在loopback配置下运行。ROS自动生成的唯一本地参数文件只含`runtime_mode: mujoco`；来源检查器现要求它是精确的唯一本地补充且内容匹配，避免将合法启动参数误判为实验配置冲突。EGL的首个正式运行日志出现一次`OpenGL error 0x502 in or before mjr_makeContext`警告，但随后仿真、帧记录和预测运行均实际启动；该渲染初始化警告保留为限制。历史来源检查脚本因仓库缺少其引用的旧运行夹具而无法整套执行；本次对实际捕获的ROS参数文件和运行日志直接调用来源核对并通过。

所有本轮启动的7个MuJoCo进程、5个controller会话、1个预测器均已退出；预测中止后PASSIVE状态确认，overlay已清理，RViz未启动，没有本轮残留进程。计划项4–12未继续派发，原因是第3项真实安全中止。实机和真实相机仍未连接；100ms实机状态新鲜度期限仍为暂定。

## 后续独立诊断：CUDA推理与scene_flat动态核验（2026-11-02）

本节记录原12项安全中止后的单独诊断，不改写冻结排程、原第3项或已完成计划项，也不把独立核验当作剩余计划项的替代。

**停顿原因。** 原第3项预测器通过`torch.jit.load(..., map_location="cpu")`和CPU张量运行模型；长停顿不只发生在冷启动。frame 23推理352.183ms、渲染1.156ms，渲染至发布354.814ms；frame 214推理202.483ms，正是控制器最后读取帧，帧龄达到218.217ms后触发200ms保护。总计214个相邻帧间隔中8个超过100ms、4个超过200ms。现有逐帧记录未包含完整CPU调度/运行队列数据，故CPU线程争用可能性不能量化；直接证据表明同步CPU推理长尾是主要停顿段，渲染不是主要耗时。旧预测器未分开记录预处理、位姿读取、传输和调度等待，因此这些子项无法从旧记录精确拆分。

**最小修正。** `ray_predictor.py`增加显式`RAY_PRED_DEVICE`选择和启动预热，原默认仍为CPU，指定CUDA不可用时明确失败；逐帧记录位姿读取、渲染、预处理、设备传输、推理、输出复制及发布时间。`s1_run_diagnostic.py`和手动运行器增加设备参数。查明启动器为所有子进程统一注入CPU LibTorch路径，导致Python预测器的CUDA Torch误报不可用；仅从预测器子进程环境去除该CPU路径，controller仍保留自身所需路径。无模型、地图、阈值或安全规则变化。scene_flat同一深度输入的CPU/CUDA输出最大差为0.00573m。

**独立持续运行。** 使用scene_flat既有reset qpos固定姿态，隐藏EGL renderer及RTX 4060 CUDA推理；3次预热耗时2.286秒，独立运行60.012秒输出1791帧（29.84Hz）。相邻预测最大间隔76.22ms，无大于100ms或200ms间隔。渲染耗时中位/P95/最大为2.05/3.21/15.31ms，推理为5.84/11.44/46.89ms，设备传输为0.49/0.80/15.91ms。该静态位姿、单预测器检查不代表与controller并发时限已通过。逐帧记录和汇总见[60秒汇总](../../../../../logs/09_R1_switch_constraints/go2_ray_pred_12/20261102_r1_go2_ray_pred_12/diagnostics/predictor_cuda_steady_60s/summary.json)及[帧记录](../../../../../logs/09_R1_switch_constraints/go2_ray_pred_12/20261102_r1_go2_ray_pred_12/diagnostics/predictor_cuda_steady_60s/frames.jsonl)。

**独立动态核验。** 原12项剩余9项未启动，本轮只做一次独立scene_flat预测核验。一次运行前编排错误和一次CUDA环境启动失败均单独保留；二者均在策略周期前停止，未计导航样本，分别见[编排attempt](../../../../../logs/09_R1_switch_constraints/go2_ray_pred_12/20261102_r1_go2_ray_pred_12/diagnostics/scene_flat_prediction_cuda_dynamic/console.log)及[CUDA启动失败attempt](../../../../../logs/09_R1_switch_constraints/go2_ray_pred_12/20261102_r1_go2_ray_pred_12/diagnostics/scene_flat_prediction_cuda_dynamic/attempt_01/A_rep_01_console.log)。修正预测器子进程环境后，唯一策略运行使用E/X=−0.05、H=0、A、Periodic P=0.06秒、预测射线独占写入、关闭几何写入和200ms原限值。控制器来源日志为`external/ray_pred`；419个任务窗策略周期全部读取有效预测帧，无几何回退、过期Veto或预测停顿引发的安全中止。753帧发布间隔中位/P95/最大33.42/42.55/57.24ms；控制器读到的帧龄中位/P95/最大18.49/32.08/49.83ms。帧龄从预测发布时刻计，不等于位姿采集到控制消费的完整感知延迟；逐帧记录了位姿读取、渲染、推理和发布时刻，但没有把位姿样本与控制周期消费做精确跨进程配对。运行ARRIVED，起终点距7.539m/0.499m；由于首个任务周期仿真时钟无效且task-start边界缺失，任务窗碰撞/跌倒及安全完成仍为UNKNOWN，不能记成安全完成。2秒站立观察完成。正常收尾日志另记录DDS超时后强制PASSIVE的FATAL事件；它发生在清理阶段，不是任务窗预测安全中止，进程退出码0且closeout确认。动态完整记录见[run context](../../../../../logs/09_R1_switch_constraints/go2_ray_pred_12/20261102_r1_go2_ray_pred_12/diagnostics/scene_flat_prediction_cuda_dynamic/attempt_02/A/rep_01/8c67c084355b48e68f3eb716fe08bee7/run_context.json)、[终局](../../../../../logs/09_R1_switch_constraints/go2_ray_pred_12/20261102_r1_go2_ray_pred_12/diagnostics/scene_flat_prediction_cuda_dynamic/attempt_02/A/rep_01/8c67c084355b48e68f3eb716fe08bee7/terminal.json)和[predictor日志](../../../../../logs/09_R1_switch_constraints/go2_ray_pred_12/20261102_r1_go2_ray_pred_12/diagnostics/scene_flat_prediction_cuda_dynamic/attempt_02/A/rep_01/8c67c084355b48e68f3eb716fe08bee7/ray_predictor.log)。

**scene_flat原几何项UNKNOWN。** 原计划项2记录有效且ARRIVED；任务窗碰撞、跌倒均为FALSE且覆盖完整。UNKNOWN仅来自base_link越界检查：scene_flat注册信息没有有限矩形边界，bounds快照为`available=false`/`identity_or_bounds_invalid`，因此该安全维度不可判定。这是场景边界适用性/分析分类的UNKNOWN，不是缺失碰撞或跌倒样本；不更改原始记录，也不将安全完成提升为TRUE。新的独立动态项另因task-start时钟/边界缺失而UNKNOWN，二者原因不同。

新诊断显示在显式CUDA和当前环境下，独立与一次并发动态运行均未再出现200ms预测间断，建议将这作为恢复原冻结比较的技术前置条件已满足；但动态只有一次且安全完成仍UNKNOWN，不能证明一般实时性或导航效果。原排程仍暂停，剩余9项不在本轮续跑，需审阅后另行决定是否恢复。原第3项`SYSTEM_SAFETY_ABORT`继续原样保留。真实相机、实机消息和100ms实机时限仍未验证。

## 下一步

等待审阅后决定是否恢复原12项冻结比较的剩余9项；若恢复，预测器使用显式CUDA并保留现有200ms保护，原第3项不重跑、不替换。当前核验只说明这一环境下一次独立运行与一次scene_flat并发运行未再观察到长间断，不构成实时性、普遍安全或预测导航收益证明。scene_flat几何项的base-bounds UNKNOWN不由补跑解决；独立动态项的任务窗安全状态也仍UNKNOWN。实机型号、主机和相机仍待用户提供/现场确认。


## 2026-11-03续跑与记录核对

原独立CUDA动态项按` s1_analyze_run.resolve_decision_start_clock `既有规则检查。生命周期记录的任务起点是session内rl_step=0，但该决策时刻`sim_clock_valid=false`且`sim_time_s=null`；同周期发布帧同样没有有效仿真时钟。规则只允许在发布帧因推理延迟变为STALE、同时控制器S6-04 trace证明该同周期采样时刻有效且时间匹配时恢复。此处不满足前提，后续有效帧不能替代任务起点。原始记录未修改，任务窗碰撞、跌倒和安全完成继续UNKNOWN；未发现可据以重建的解析错误。

DDS收尾顺序核对：原记录先有`NORMAL_SHUTDOWN_REQUESTED`（单调时刻39363938319944），约6.57秒后完成2秒仿真站立观察（39370506976232），再约1.18秒后记录DDS超时观察（39371689550397）；ROS日志中DDS FATAL先于launch收到SIGINT并向controller发送SIGINT。故超时发生在正常停止请求之后、ROS/controller资源实际关闭之前，处于收尾静止阶段，不是任务期通信异常。现有证据不足以归因于DSS序列/状态判断错误；该独立核验已完成并退出，未改任务期安全处理或通过降级日志隐藏FATAL。该次清理仍确认最终退出。

原排程第4—12项已按原顺序续跑完成。第4项首次受限启动因SDK2无法枚举接口而在策略周期前失败，另一次启动使用已授权loopback环境后完成；该普通路径失败保留、不计导航终局。完整启动attempt与计划项状态见[原批次计划](../../../../logs/09_R1_switch_constraints/go2_ray_pred_12/20261102_r1_go2_ray_pred_12/run_plan.json)。第4项ARRIVED；第5项challenge27几何限时；第6项scene_flat预测限时；第7项challenge27预测碰撞障碍02；第8项scene_flat几何ARRIVED；第9项challenge27预测碰撞障碍02；第10项scene_flat预测ARRIVED；第11项challenge27几何ARRIVED；第12项scene_flat几何ARRIVED。各次真实终局均保留，不因结果重跑。第4—12项的有效记录均VALID，预测射线均由预测器独占写入并使用CUDA、预热和原200ms时龄保护；没有静默几何回退。

12项计划现为：几何结果共6项（5次到达、1次challenge27限时；其中3次scene_flat到达的越界安全维度因无有限边界保持UNKNOWN）；预测条件含原第3项CPU安全中止（218.217ms过期）、独立诊断不入排程，以及第4—12项中的预测结果（scene_flat两次到达、一次限时；challenge27两次碰撞障碍02）。到达、碰撞和限时均分开报告；不把原CPU中止、独立CUDA诊断和本轮CUDA预测合并成12次同配置有效导航。计划第1项实际保留5次attempt（4次工程/来源启动失败及1次ARRIVED）；第4项3次attempt（2次预启动失败及1次ARRIVED）；其他项各1次。任务推进、Recovery占比及逐周期时延以各项派生`analysis/run_results.md`和逐帧预测记录为准，需按计划项读取；不同终局窗口不同，不合并成总体平均。挑战27预测两次均碰撞障碍02且未到达，几何项有两次到达和一次20秒限时，显示在这些少量样本中预测条件未消除该绕行碰撞风险；样本不足以归因普遍性能差异。

本轮出现一次受限启动失败后使用loopback环境继续；所有12个计划项的实际终局完成，运行器、MuJoCo、ROS/controller、预测器、RViz及辅助进程已扫描确认无残留。原批次状态为COMPLETE。


## 预测模型动态结果复核入口

本批预测模型是否值得继续使用的只读分析见[模型复核报告](RAY_MODEL_REVIEW.md)和[总体结果表](RAY_MODEL_GROUP_SUMMARY.csv)。结论为暂缓扩展当前模型动态试验；两次challenge27预测碰撞均接触障碍02，但碰撞点缺乏同周期、同位姿的预测与几何射线配对，不能认定模型漏判或高估。scene_flat预测限时项推进至距目标0.718m，Recovery占比偏高是观察线索，不能据此断言较短射线错误或Recovery导致超时。


## 射线输入链路复核与有限观看（2026-11-03）

只读代码/训练来源核对未发现可直接修正的相机、单位、预处理、束序或标签转换错误。当前动态器和MuJoCo微调数据生成器共用相机位姿/深度处理；12个冻结姿态比较显示scene_flat参考全6m但输出右侧束系统偏短（MAE 0.582m，FD 18/66），challenge27 MAE 2.058m、FD 37/66、FS 0/66。无近障false-safe并不消除整体大误差。详细相机、标签语义和图像证据见[输入链路复核报告](input_link_review/RAY_INPUT_REPAIR_REPORT.md)，固定姿态见[清单](input_link_review/FIXED_POSES.csv)，逐姿态数据保留于两份比较目录。

按条件继续的4次观看结果：scene_flat几何/预测均ARRIVED，预测末距0.497m、Recovery时长占比21.0%；challenge27几何ARRIVED，预测在16.830–16.834s接触障碍02、末距3.501m，之后PASSIVE确认。预测两项最大帧间隔分别57.94ms和54.21ms，最大控制器读取时龄49.05ms和43.60ms，均未触发200ms保护。scene_flat的安全完成因地图无有限边界仍UNKNOWN；challenge27预测为碰撞终局，安全完成FALSE。预测scene_flat在完成站立观察后的收尾有已知DDS FATAL，不扩大排查。四项使用相同地图构造/reset姿态及名义任务设置，实际首个策略周期状态保存在run_context；启动时速度略有差异，非严格状态配对。四项原始记录、分析和完整指标见[动态结果表](input_link_review/DYNAMIC_RESULTS.csv)及运行日志`logs/09_R1_switch_constraints/ray_input_review_20261103/`。所有环境已关闭。

本轮没有代码或权重修正；固定样本中高误差更指向当前网络对相机深度图到水平2D射线标签的适应不足。建议先做一次受控、仅ray-head/network输出的有限微调，按场景/姿态留出验证，再决定是否恢复更广动态测试；Agile、RA、Recovery不纳入重训。原[模型复核](RAY_MODEL_REVIEW.md)中的“暂缓当前动态投入”结论由本节更新为“先微调射线网络，再评估扩展”，不是当前模型动态表现通过。
