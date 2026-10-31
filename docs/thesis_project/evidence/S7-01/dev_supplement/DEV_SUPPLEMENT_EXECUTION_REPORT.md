# S7-01 开发图补证实际执行报告（m02 A/D）

日期：2026-10-21  
批次：`dev-m02-ad-20261021`  
授权：`DEV-M02-AD-20261021-01`（开发补证，最多 2 次实际场景启动、0 次替代）  
数据类别：**`DEV_SUPPLEMENT_NOT_FORMAL`**。这不是正式 240 项数据，不用于 A/D 性能比较或成功率推断。  
S7-01 后续已 COMPLETE（Director 有条件通过）；本报告为开发补证历史记录。240 项正式实验未启动、未获运行授权。

## 1. 执行摘要

| 计划项 | 组别 | run_id | 终局 | 记录有效性 | 安全任务完成 | 实际参数 |
|---|---|---|---|---:|---|---|
| `DEV-M02-A` | A | `798d518a8fb84d419cb00987ed6a8538` | `ARRIVED` | `VALID` | 原始`UNKNOWN`；统一分析`True` | E=−0.05；无 X；无保持；H=0 |
| `DEV-M02-D` | D | `5859b72dc8994e8a9e9d6ad0e0cd8e2d` | `ARRIVED` | `VALID` | `True` | E=−0.05；X=−0.10；H=0.20 仿真秒 |

- 实际场景启动：**2**；工程替代：**0**；编排调用：2；唯一 run_id：2。
- 顺序：A → D；A 只启动一次，D 只启动一次。
- 分析一致性：`PASS`，issues 为空；A 在 D 之前；A、D 各 1 次场景启动。
- 两场均隐藏 MuJoCo、未启动 RViz；均完成 2 仿真秒站立观察；均确认进程退出与收尾排空。
- 本报告不把两次结果解释为 A/D 优劣；A 原始发布态为 `UNKNOWN`、统一决策时钟分析为 `True`，D 为 `True`；n=2，不能估计成功率。

## 2. 环境与批次前置状态

| 项目 | 实际值 |
|---|---|
| 分支/提交 | `feat/ray-pred-source-switch` / `42518fa` |
| 显示环境 | `DISPLAY=:0`，`XAUTHORITY=/run/user/1000/gdm/Xauthority` |
| 可用磁盘 | 约 60 GiB，高于 15 GiB 保护线 |
| GLFW 实际渲染 | `[MUJOCO-WINDOW] backend=GLFW requested=hidden visible=0 renderer=Mesa Intel Graphics` |
| 本轮启动前遗留进程 | 未发现本批 MuJoCo/ROS/controller/RViz 进程 |

**A 在本轮开始前已经存在于批次中。** 首次状态查询显示：

- `batch_state=PAUSED`，`pause_reason=scene_creation_unknown_after_launch`；
- A 的 attempt 为 `scene_created=null`、`run_id=null`；
- 但磁盘上已有完整的 A run 目录、`runtime_record.jsonl`、`run_context.json`、`terminal.json`、`process_facts.json` 和 metrics。

按“已有记录先查询状态、不重新创建或删除批次”的要求，没有重跑 A。根因是运行器的 run 目录发现逻辑把 `result/analysis/` 也当成一个 run 目录，导致同一 attempt 被视为“多个 run 目录”，场景创建事实被判为 UNKNOWN。

## 3. 最小修正与版本修订

只做了两项定向修正，未改控制、安全阈值、模型、地图或实验计划：

1. 运行目录发现只接受包含 `runtime_record.jsonl`、`run_context.json`、`terminal.json`、`process_facts.json`、`mujoco.log` 或 `console.log` 的目录，排除 `analysis/`；
2. 从磁盘恢复 attempt 时，同时读取并核对 run context 中的实际配置、source binding、session、收尾和清理事实。

修正后重新运行：

- 正式运行器离线回归：**21/21 PASS**；
- 开发补证离线检查：**11/11 PASS**。

批次清单记录了显式版本修订 `DEV-M02-AD-VERSION-AMENDMENT-01`：旧/新 `s7_formal_runner.py` SHA-256、变更原因、影响说明。未改任何原始 run 记录，未重跑 A，修订前后模型/配置/诊断器/分析器哈希一致。

## 4. A 的实际结果

- run_id：`798d518a8fb84d419cb00987ed6a8538`
- 地图/场景：`m02.xml` / `m2v2_m02`；map version `m2-selected-development/2.1`
- 实际配置：group A，E=−0.05，exit=E（单阈值），hysteresis=false，hold=false，H=0，switching_mode=1
- 目标 `(7.5,0)`；任务上限 20 仿真秒/120 墙钟秒；到达距离 0.4971 m（<0.5 m），到达仿真时刻 13.628 s
- 任务窗口：cycle 0 → 325；cycle 身份完整；终止帧 payload 完整且 `same_snapshot_persisted=true`
- 站立观察：complete，实测 2.008 仿真秒，104 个样本
- 终局/记录：`ARRIVED` / `VALID`；context、terminal、派生终局一致，无合同冲突
- 原始发布态安全任务完成：**UNKNOWN**。原因：
  - `arrival_or_task_start_sim_time_missing`；
  - 非足端碰撞覆盖不完整；
  - 跌倒历史覆盖不完整。
- 统一决策时钟分析：**True**。同一 cycle0 decision-time 时钟恢复后，task start=7.126 s、elapsed=6.502 s；非足端碰撞/跌倒 False 且覆盖完整；base_link 界内；足端撞击 1 次单列。
- 安全维度：非足端碰撞 UNKNOWN、跌倒 UNKNOWN、系统安全中止 false、base_link 越界 false、足端事件 UNKNOWN
- 配置消费：`runtime_source_validation=confirmed / all_source_events_match`；实际组别/E/X/H 与计划一致；source binding controller/writer PID 存在
- 清理：process exit 0，shutdown_complete=true，forced=false，closeout_complete=true，最终排空 complete；MuJoCo/ROS/查询会话均退出
- 隐藏窗口：`visible=0`；RViz requested disabled；未截图/录像

A 的原始发布态 `ARRIVED` 不能升级为“已确认安全完成”；统一分析依据同一 cycle0 decision-time 时钟恢复后达到安全完成条件。原始 STALE 状态仍保留。

**Cycle0复核补充（2026-10-21）：**A 的原始 UNKNOWN 已定位为 decision-time 有效、publication-time stale 的 cycle0 时钟记录问题。经Director同意，分析侧重建已统一接入正式分析：同一 cycle0 样本得到 True、elapsed 6.502 秒、collision/fall 覆盖完整，并保留足端撞击 1 次单列；原始 STALE 和原始 UNKNOWN 对照保留。正式汇总敏感性为重建1、安全改变1、时间改变1。详见[CYCLE0_CLOCK_REVIEW.md](../cycle0_clock_review/CYCLE0_CLOCK_REVIEW.md)。

## 5. D 的实际结果

- run_id：`5859b72dc8994e8a9e9d6ad0e0cd8e2d`
- 地图/场景：`m02.xml` / `m2v2_m02`
- 实际配置：group D，E=−0.05，X=−0.10，hysteresis=true，H=0.20，hold=true，switching_mode=4
- 目标 `(7.5,0)`；到达距离 0.4745 m，到达仿真时刻 13.352 s
- 任务窗口：cycle 0 → 330；cycle 身份完整；终止帧 payload 完整且 `same_snapshot_persisted=true`
- 站立观察：complete，实测 2.030 仿真秒，97 个样本
- 终局/记录：`ARRIVED` / `VALID`；无终局合同冲突
- 安全任务完成：**True**。非足端碰撞 false 且覆盖完整；跌倒 false 且历史完整；系统安全中止 false；base_link 全程界内；足端接触/撞击 0
- 配置消费：`runtime_source_validation=confirmed / all_source_events_match`；实际组别/E/X/H 与计划一致；source binding PID 存在
- 清理：process exit 0，shutdown_complete=true，forced=false，closeout_complete=true，最终排空 complete；所有 owned 进程退出
- 隐藏窗口：`visible=0`；RViz requested disabled；未截图/录像

## 6. 恢复与台账一致性

- A 已有效选定，`resume` 未重新派发 A：A 只有 1 个 scene start、1 个 run_id、1 个 attempt。
- D 是 `resume` 唯一新派发的计划项；D 只有 1 个 scene start。
- 总实际场景启动 2，替代 0，唯一 run_id 2，顺序 A→D。
- A 的检查点是**恢复后补记**的，`retrospective=true`；原因是原始 A 调用因上述发现逻辑误判为 `scene_creation_unknown`，没有在 A 边界写出 stage 检查点。未把它描述成当时已写。
- 分析器从两份原始记录重算，`analyzer_sha256=96a575b…`；原 `metrics.json` 保留未覆盖。
- 运行器台账与分析器对同一记录的解释一致：A `ARRIVED/VALID/UNKNOWN`，D `ARRIVED/VALID/True`。
- 分析一致性：`PASS`，issues 为空；`A_before_D=true`。

分析产物：

- `docs/thesis_project/evidence/S7-01/dev_supplement/dev_supplement_analysis.json`
- `docs/thesis_project/evidence/S7-01/dev_supplement/DEV_SUPPLEMENT_ANALYSIS.md`
- `docs/thesis_project/evidence/S7-01/dev_supplement/dev_supplement_execution_summary.json`
- `docs/thesis_project/evidence/S7-01/dev_supplement/status_after_execution.json`
- 原始批次：`logs/07_formal_preparation/dev_supplement_m02_AD/dev-m02-ad-20261021/`

## 7. 尚未验证与边界

1. A 的原始发布态安全任务完成保持 `UNKNOWN`；统一分析为 `True`，依据是同一 cycle0 decision-time 时钟恢复后其他安全覆盖满足。不能把原始发布态 UNKNOWN 改写成 True；两种口径并列保留。
2. 两次运行不构成 A/D 性能比较、成功率、显著性、非劣效或泛化证据。
3. MuJoCo 命令消费仍为 **UNKNOWN**；只确认了 controller/source 侧配置消费、策略帧、终局和安全分类。
4. 正常 A 后检查点/恢复不等于“运动中断恢复”：本轮没有故意中断运动中的仿真，也没有做真实故障注入。
5. A 的 stage 检查点是恢复后补记；正常运行器路径会写检查点，但本次原始 A 调用因发现逻辑缺陷未写。
6. 版本修订仅涉及运行目录发现和磁盘恢复证据读取；未改控制、安全规则、阈值、模型或地图。
7. 未运行任何留出图，未启动 240 项正式实验，未提交 Git，未清理原始数据。
8. 本轮所有 owned MuJoCo/ROS/controller/查询会话均已退出；最终进程扫描未发现本批残留。未关闭或影响无关会话。

## 8. 结论

两项开发补证均完成：A `ARRIVED/VALID/安全UNKNOWN`，D `ARRIVED/VALID/安全True`；实际配置、隐藏窗口、目标/时限、站立观察、cycle0/终止帧、排空与清理、attempt/run_id 绑定均通过核对；A 未重跑，D 只恢复一次；分析一致性 `PASS`。这是开发链路补证，不是正式数据，也不构成 240 项授权。S7-01 已 COMPLETE；后续由 S7-02 承接，等待正式运行授权。
