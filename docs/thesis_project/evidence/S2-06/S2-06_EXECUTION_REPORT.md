# S2-06 执行报告

**最新状态：COMPLETE — Director 有条件通过。** 通过范围包括五套场景接入、预检、每场景一次非正式 A 组诊断及本次按原始周期记录修订的统计口径；不构成正式实验或成功率估计。以下历史执行记录和原始数据保持不变。

## 完成内容

- 从 `designs/random_static_scenes/layouts.json` 生成 `scene_random_01.xml` 至 `scene_random_05.xml`，并新增可重复生成工具 `scripts/generate_random_static_scenes.py`。障碍顺序、中心、形状、尺寸和高度按 JSON 原值写入；圆柱 MJCF 的高度按 MuJoCo 半高表达。预览 PNG 没有用于运行输入，五条 `path_waypoints_m` 也没有传给控制器。
- 扩展 `common/abs_scene_catalog.def` 和 `common/abs_scene_catalog.h`，登记 scene ID、障碍数、XML/closure 哈希及 MuJoCo 编译模型指纹；`scripts/abs_scene.py` 现在识别十个受支持场景。原五个目录项保持原身份。`--scene` 通过 `resolve_scene()` 校验后交给 MuJoCo 加载；run context、collision authority、runtime record、指标和时间线保留同一 scene 身份。
- ray writer 与 collision authority 分别调用同一个 `abs_scene::selectObstacleGeoms()`。对新场景，两端选中的均为 `ppt_obstacle_XX` 具名静态障碍；floor 被显式排除。控制器仍只收到目标、观测和 11 条 ray2d，不读取保留通道。

## 五场景预检

`unitree_mujoco`、`s2_06_scene_probe` 和扩展后的 `s2_04_scene_probe` 增量构建成功。

`scripts/generate_random_static_scenes.py --check` 通过；生产 MuJoCo 编译加载探针 `s2_06_scene_probe` 对五份 XML 全部加载成功，报告名称、类型、位置、尺寸、数量和模型指纹。扩展后的 `s2_04_scene_probe` 对全部十个登记场景运行生产碰撞 authority 发布路径，均成功；`scene_flat` 仍为空障碍集合，floor 未选作障碍。布局对照、MuJoCo 探针原始输出分别见 [layout_geometry.json](preflight/layout_geometry.json)、[s2_06_scene_probe 输出](preflight/mujoco_scene_probe.txt) 和 [十场景 authority 输出](preflight/all_scene_authority_probe.txt)。

保留通道采用障碍物外接圆到每段路径折线的保守净距核对。五场景最小净距依次为 0.6700、0.6586、0.6980、0.6504、0.6631 m，均大于 1.10 m 通道半宽 0.55 m。距起点最近的障碍净距最小为 1.2574 m，距目标最近为 0.9706 m。地面未包含在这些障碍数中。

**冻结布局的边界备注：** 第 4 场景第 14 个箱体在 y 方向下缘为 −3.0564 m，比 JSON 声明的 y 下界 −3.0 m 超出约 0.0564 m。我按冻结布局原样生成，没有裁切或移动该障碍。其位置不侵入起点、目标或 1.10 m 保留通道，场景地面范围覆盖该处。生成器会把声明场地边界的超出作为提示输出。

模型身份指纹使 XML 或 include/resource 改动会导致预检拒绝；更改布局后须重新生成 XML 和目录身份，不能沿用本次身份。

## 运行设置和证据

五次调用均使用同一安装 overlay、模型、目标和启动流程：A / `paper_faithful_switch`、实际 entry/exit 均为 −0.05、无滞回和保持、目标 `(7.0, 0.0) m`、到达半径 0.5 m、运动仿真上限 20 s、墙钟上限 120 s。运行配置来自 `/home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/go2_description`；逐 run 的 controller 配置和模型来源绑定在 `run_context.json`，仿真加载场景身份由 MuJoCo authority 日志和 v5 collision snapshot 核实。五次均为独立 session 和 run ID。命令参数保存在 [run_invocations.json](diagnostic_runs_20261010/run_invocations.json)。

同一初始条件按相同新建 MuJoCo 进程、相同机器人 `go2.xml` 默认 `qpos0` 和 PASSIVE 准备流程执行；初始运动策略帧位置均在起点附近。运行记录没有单独保存每次完整 qpos 向量，因此“相同默认模型初始化”有代码和启动路径支持，逐关节初态逐位相同未从运行载荷直接核验。

运动时间与路径口径：起点至终点经过时间是运动边界仿真时间之差；有效累计时间只累加有效连续仿真间隔。路径长度和平均速度只覆盖有效连续时间片段，不代表全程路径或全程平均速度。模式周期数和模式变化按运动策略周期窗口计数；时钟无效的周期仍可证明模式已变化，但不用于精确时间指标。

| 场景 | Run ID | 终局 / 终点距离 | 经过时间 / 有效累计时间 / 有效连续路径 / 有效片段平均速度 | Agile / Recovery 有效时钟周期 | 全部模式变化（A→R / R→A） | 有效时间模式变化 / 无效时间变化 | RA 范围（运动窗口） | 足端接触 / 撞击；非足端碰撞；跌倒 | 策略频率；无效时钟 / 周期缺口 | PASSIVE / 清理 |
|---|---|---|---|---|---|---|---|---|
| 01（15 障碍） | `a1b6a46f7e694b2e8eca86b904baf6eb` | ARRIVED / 0.4772 m | 3.300 / 3.260 s / 6.4029 m / 1.9641 m/s | 165 / 0 | 0 (0 / 0) | 0 / 0 | [−0.9996, −0.7139] | 0 / 0；false；false | 49.9886 Hz；1 / 0 | 已确认 / 已完成 |
| 02（17 障碍） | `8f2ee979a36d49a2a8951c5e8eac6f07` | ARRIVED / 0.4799 m | 3.460 / 3.380 s / 6.3583 m / 1.8811 m/s | 172 / 0 | 0 (0 / 0) | 0 / 0 | [−0.9994, −0.4535] | 0 / 0；false；false | 50.0326 Hz；2 / 0 | 已确认 / 已完成 |
| 03（18 障碍） | `68447ad2375a4b28b3abbe10d4a1977d` | ARRIVED / 0.4908 m | 3.424 / 3.424 s / 6.6104 m / 1.9306 m/s | 172 / 0 | 0 (0 / 0) | 0 / 0 | [−0.9995, −0.6924] | 0 / 0；false；false | 49.9648 Hz；0 / 0 | 已确认 / 已完成 |
| 04（19 障碍） | `1173da3d307f474d91f034ebf1582476` | ARRIVED / 0.4874 m | 11.760 / 11.108 s / 7.7485 m / 0.6976 m/s | 407 / 167 | 136 (68 / 68) | 128 / 8 | [−0.9984, 0.2985] | 0 / 0；false；UNKNOWN* | 50.0126 Hz；15 / 0 | 已确认 / 已完成 |
| 05（20 障碍） | `0eb23aa93ec94f52968248d971fa832a` | ARRIVED / 0.4524 m | 3.530 / 3.282 s / 6.0780 m / 1.8519 m/s | 171 / 0 | 0 (0 / 0) | 0 / 0 | [−0.9994, −0.5060] | 0 / 0；false；false | 49.8107 Hz；6 / 0 | 已确认 / 已完成 |

上述足端统计是足端—障碍接触/撞击，普通足端—地面支撑接触不计入；非足端碰撞与跌倒结论按 motion window 和 S2-05 规则读取。第 4 场景有一段未达到确认条件的姿态异常候选，故跌倒为 UNKNOWN，不写成“未跌倒”。碰撞结果只适用于分析器判定完整的运动窗口；没有把它扩展为全采集区间结论。

五场景单次结果和逐 run 指标见 [对照表](diagnostic_runs_20261010/scene_comparison.md) 与 [结构化对照](diagnostic_runs_20261010/scene_comparison.json)。各自时间线在场景目录的 `<run_id>/timeline.png`；原始策略周期、scene binding、终局及收尾证据保留在 `<run_id>/runtime_record.jsonl`、`run_context.json`、`terminal.json`、`process_facts.json` 和 `metrics.json`。五条记录均为 `VALID`、含完整终止周期 payload、无策略周期缺口，实际模式/RA/位姿和碰撞快照可逐周期复核。策略周期间隔中位数约 19.961–20.026 ms，P95 约 24.514–30.733 ms；policy 频率约 49.811–50.033 Hz。频率统计覆盖完整采集记录，不能作为无观测开销对照。

第 4 场景按原始运动窗口周期（含边界）核对，共有 136 次模式变化：68 次 Agile→Recovery、68 次 Recovery→Agile；其中 128 次具有有效仿真时间（方向分别为 62/66），另 8 次发生在仿真时间无效的周期。无效时间不抹去已观察到的模式变化，但这 8 次不能用于精确驻留/切换时间指标。现有分析器另报告 117 次“0.5 s 短反向切换”：其口径是在有效时钟运动窗口内，对相邻、方向相反且间隔不超过 0.5 s 的转换对计数；它不含无效时间转换，也不是全部切换数的完整统计。上述均只描述这一 run，不证明该布局或 A 组的一般切换表现。其他四场景这次没有进入 Recovery。五场景都在到达半径内终止、没有非足端碰撞，足端障碍接触和撞击均为 0；其中 04 的跌倒状态按上文保留 UNKNOWN。

五次 run 的每条策略周期中 ray2d 均有效，且同周期 collision snapshot 的 scene ID 与编译指纹均和该 run 绑定一致：01 为 178/178，02 为 186/186，03 为 184/184，04 为 601/601，05 为 188/188。

## 桌面与清理

默认执行环境中、未显式设置 XAUTHORITY 的 `:0` 探针返回 unable to open display；这不足以单独判定是沙箱或授权哪一项阻断。按任务授权进行的一次最小沙箱外预检显式使用 `DISPLAY=:0` 和 `/run/user/1000/gdm/Xauthority`，X11、GLFW 窗口创建及 OpenGL 预检通过，renderer 为 `Mesa Intel(R) Graphics (RPL-P)`，OpenGL 4.6。预检证据见 [display_preflight.json](preflight/display_preflight.json)；未修改桌面授权、系统权限、驱动或全局环境。

每次运行的 supervisor 都确认 controller 转入 PASSIVE，`shutdown_complete=true`、`closeout_complete=true`、`closeout_errors=[]`。逐次运行后检查了 `unitree_mujoco`、controller manager/ros2_control_node、RViz 进程，没有残留。本轮结束后再次核对，相关运行进程为空；探针和窗口进程均已退出。没有删除任何来源不明的共享内存对象。

最终进程及探针共享内存核对结果见 [final_process_check.json](preflight/final_process_check.json)。

## 限制与结论边界

- 每个布局只有一次短诊断；ARRIVED 只说明该次进入 0.5 m 目标半径，不能据此计算成功率、排序难度或宣称正式 A 组避障性能通过。
- 五次诊断均没有障碍接触；第 4 场景虽有 Recovery 和频繁切换，但不是重复性或性能证据。其他四场景没有观察到 Recovery，不能用来评估 Recovery 动作表现。
- 第 4 场景未确认的姿态候选使跌倒结论为 UNKNOWN。所有“无非足端碰撞”结论限定在运动窗口及当前诊断规则覆盖范围内。
- 五次运行分别有 1、2、0、15、6 个无效时钟帧（按场景 01 至 05 排序），周期缺口均为 0。无效时钟帧不插值；受影响的全采集频率与驻留时间仅按记录范围解释。
- 同一模型默认初始化有源代码/运行流程证据，但没有逐次完整关节 qpos 初态快照。模型真实动作关节顺序已有 S1-06 的条件性证据；实机 SDK 语义仍不能由本次仿真确认。MuJoCo 是否消费控制器写出的具体命令仍 UNKNOWN。

## 文件与后续

本任务添加五份场景 XML、固定数据生成工具、场景目录项、生产 MuJoCo 场景探针和五场景摘要工具，并更新单次运行指南。原 A 切换规则、模型、观测/动作、速度和安全检查未改；未训练、未做正式实验，未提交 Git。Director 有条件通过 S2-06 并标记 COMPLETE；通过范围限于场景接入、单次诊断和记录/指标链可复核，不代表避障性能或切换效果通过。箱体越界备注和场景 04 跌倒 UNKNOWN 继续保留。

通俗说明：五个场景这次都到达了目标半径。只有第 4 场景发生了明显的 Recovery 往返切换；其他四个没有 Recovery。第 4 场景有一段未确认的姿态异常，所以跌倒判断保留 UNKNOWN。每场只有一次观察，不能说明整体成功率或稳定性能。五次环境都已关闭，并核对过相关进程退出。
