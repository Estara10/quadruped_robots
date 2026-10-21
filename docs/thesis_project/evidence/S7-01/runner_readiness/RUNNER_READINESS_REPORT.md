# S7-01 最小正式运行器准备与离线联测报告

日期：2026-10-21  
状态：**S7-01 已 COMPLETE（Director 有条件通过）；S7-02 为唯一 ACTIVE / AWAITING_RUN_AUTHORIZATION。方法已冻结，`authorized_to_run=false`；240 项未启动、未获运行授权。**  
本轮只实现运行器/分析入口并做定向离线联测，未启动 MuJoCo、ROS、controller、模型推理或任何留出图场景。

**执行后修订（2026-10-21）：** 在后续 m02 A/D 开发补证执行中发现 `result/analysis/` 被 run 目录发现逻辑误计为第二个 run，导致已有的 A 被误判为 `scene_creation_unknown`。仅修正 run 目录发现和磁盘恢复证据读取，未改控制/安全阈值/模型/地图；正式离线回归21/21、开发补证离线11/11。版本修订与执行偏差见[开发补证执行报告](../dev_supplement/DEV_SUPPLEMENT_EXECUTION_REPORT.md)第3节。

## 1. 实际环境与版本

| 项目 | 实际值 |
|---|---|
| 分支 | `feat/ray-pred-source-switch` |
| 最近提交 | `42518fa`（未提交本轮改动） |
| 工作区 | 存在用户既有未提交改动；本轮未清理、未回退、未把无关改动当阻塞 |
| Python | 3.10.12（`/usr/bin/python3`） |
| 可用磁盘（只读检查） | 约 60.0 GiB，高于 15 GiB 保护线 |
| 冻结计划 `plan_sha256` | `95c764729a7466455924518ae2b498e060f35a20d946e065af9f226f25c1c301` |
| 矩阵 CSV SHA-256 | `604c7ce526c826871d5478b4e70f5a84e7b7523e8003ebbcfa10258461fd3cd0` |
| 矩阵 JSON SHA-256（冻结状态更新后） | `a7b92c2ad88dfa1d447b8e8c35cd89cb91adef8d1319c34b72316632149266c0` |

本轮没有启动任何场景或运行环境，因此不存在需要关闭的本轮 MuJoCo/ROS/controller/采集器进程；也没有清理任何数据。离线检查进程均正常退出。

## 2. 方法冻结状态同步

`docs/thesis_project/designs/S7/formal_matrix_240.json` 仅做元数据状态更新：

- `status`: `CANDIDATE_FROZEN_PENDING_DIRECTOR_REVIEW` → `METHOD_FROZEN_AUTHORIZED_TO_RUN_FALSE`；
- 保留 `authorized_to_run=false`；
- 新增 `method_freeze`：冻结日期、A/B/C/D 参数、30 图×2 重复区组×4 组、调度种子 `2026102001`、Williams 顺序、bootstrap 10,000 次/种子 `2026102002`、统一运行条件、资源与证据规则；
- **240 条记录未重新生成、未重排、未改字段**；`matrix_id` 保留原名以便追溯；
- CSV 未修改。

冻结后重新校验：240 项、30 图、60 区组、每组 60 项、四条 Williams 序列各 15 次、12 种相邻组别转换各 15 次，全部 PASS。计划摘要与冻结前一致。

A/B/C/D 固定为：

| 组别 | E | X | H |
|---|---:|---|---|
| A | −0.05 | 不配置 | 不配置 |
| B | −0.05 | −0.10 | 不配置 |
| C | −0.05 | 不配置 | 0.20 仿真秒 |
| D | −0.05 | −0.10 | 0.20 仿真秒 |

## 3. 新增入口

| 文件 | 作用 |
|---|---|
| `scripts/s7_formal_runner.py` | 薄批次核心：读冻结计划、独立 overlay、显式授权、串行派发、原子 attempt 台账、断点恢复、磁盘/额度保护、安全事实读取 |
| `scripts/run_s7_formal_240.py` | CLI：`plan` / `preflight` / `status` / `run`（默认拒绝） |
| `scripts/s7_formal_analysis.py` | 三层汇总、地图等权、四个主对比+交互、UNKNOWN 情景端点、bootstrap |
| `scripts/analyze_s7_formal_240.py` | 正式汇总 CLI；默认用同一冻结分析器从原始记录重算 |
| `scripts/check_s7_runner_readiness.py` | 定向离线联测；只写 `logs/07_formal_preparation/runner_readiness/` |

复用现有生产入口，不复制控制/安全判断：

- `manual_ab_compare.py`：`prepare_overlay`、`verify_overlay_detached`、目标写入；
- `s1_run_diagnostic.py`：单 run 生产入口与 overlay 配置校验；
- `s1_analyze_run.py`：`calculate()` 为正式派生分析；
- `safety_task_completion.py`：安全任务完成分类；
- `formal_attempt_accounting.py`：attempt 台账与替代规则。

## 4. 运行器行为

### 4.1 默认拒绝与授权

`run` 在没有独立授权文件时返回 `REFUSED / missing_explicit_batch_authorization`，退出码 2。授权记录必须同时满足：

- `schema=s7-formal-run-authorization-v1`、`authorized_to_run=true`；
- 独立 `authorization_id`、Director 审阅引用与日期；
- `matrix_id`、`plan_sha256`、240/12/252、6 GiB、15 GiB 与冻结计划一致；
- `batch_id` 与本次批次一致；
- `fixture_only` 记录被生产入口拒绝。

没有 `--force`、环境变量或其他绕过开关。**本轮未生成任何有效运行授权。**

### 4.2 派发与目录

- 按 `global_order` 串行派发，不重排；每项唯一 `plan_item_id`、组别、地图、重复区组、attempt ID。
- 每个批次准备 A/B/C/D 四个独立实体 overlay；不通过符号链接写回源码。A/C 配置中没有 `exit_threshold`，A/B 没有保持字段；CLI 的 X/H 回退值只记为入口兼容值。
- 每项输出 `runs/<plan_item_id>/attempt_NN/result/<run_id>/`。
- 批次目录：`logs/08_formal_experiment/S7_ABCD_240/<batch-id>/`（本轮未创建正式批次）。
- 离线构造数据：`logs/07_formal_preparation/runner_readiness/offline_joint_test/`，与正式目录分开。

### 4.3 场景事实与恢复

`scene_created` 只接受独立证据：`run_context.lifecycle.mujoco_process_started_monotonic_ns`、`mujoco.log` 启动标记，或 console 中的 MuJoCo 启动标记。**不凭目录存在或缺 run_id 推断场景已创建。**

- 启动前写入唯一 attempt 目录与调用意图，原子写入台账；
- 场景创建事实不明时暂停并标待核实，不自动重跑；
- 恢复时先检查已有目录、日志和记录，再从原始产物恢复已完成事实；
- 已有效到达/碰撞/跌倒/仿真限时等计划项不重跑；
- 同一批次并发入口由 `ledger/batch.lock` 拒绝；失效 PID 锁归档后继续；
- 恢复入口先做只读`/proc`扫描：若仍有命令行指向本批次目录/overlay的进程，暂停并报告，不自行结束；
- 恢复时比较批次清单中的Git commit、源配置、三份模型和关键脚本SHA-256；发生实质变化则暂停；
- 台账采用临时文件 + `os.replace` 原子替换。

### 4.4 attempt 落账与替代

- 区分编排调用、实际场景启动、进入 RL、计划项和选定分析结果；
- 场景创建前失败不消耗场景启动额度；场景创建后即使无 run_id 也计入启动；
- 基础 240 次、最多 12 次有证据的工程替代、同项一次、总启动 ≤252；
- 替代必须引用原 attempt、独立目录、Director 授权和独立证据；系统安全中止缺少独立佐证时不自动替代；
- 全启动层保留首次失败和被替代 attempt。

### 4.5 安全与资源

- 每次启动前检查可用空间 ≥15 GiB、批次占用 ≤6 GiB；达到限制暂停，不删除正式原始记录。
- 每 12 次实际场景启动或每个完整地图区组写检查点。
- 每次 run 后记录 `shutdown_complete`、exit code、closeout；未确认收尾即暂停后续派发。
- 运行后只读扫描是否仍有命令行指向本批次目录/overlay的进程；有残留即暂停并报告。
- 核对 run context 的 `runtime_source_validation`、`runtime_config_observed`、source binding PID 与 preflight 配置验证：要求状态 `confirmed`、`all_source_events_match`，且实际组别/E/X/H/hysteresis/hold 与冻结计划一致；不一致即暂停。这些证据来自 controller/source 运行事件，不是静态解析；仍不等于 MuJoCo 命令消费确认。
- 安全任务完成直接调用 `safety_task_completion.py`，不复制判断；任务窗口与站立/清理窗口分开。

## 5. 三层汇总与分析

`analyze_s7_formal_240.py` 默认对**每个实际场景启动**调用 `s1_analyze_run.calculate()` 从原始记录重算（不仅是选定结果），派生文件写入 `analysis/derived_scene_starts/`，原 `metrics.json` 保留并记录 SHA-256。正式模式拒绝 `--use-stored-metrics`；该选项只允许离线构造矩阵。批次清单的 `plan_sha256` 与分析输入不一致时拒绝混合分析。

输出：

1. **计划项主结果**：每项一条，独立于成绩选择；仍报告已确认安全完成、已知未完成、安全 UNKNOWN、未解决工程/场景项、未启动。
2. **全部实际启动可靠性账**：保留被替代的首次 attempt、准备失败、运动期安全中止、记录无效和清理异常。
3. **仅首次实际启动敏感性**：逐计划项列出首次实际启动的终局/有效性/指标；首次未进 RL 不伪造导航结果，首次安全中止不被后续替代抹去。输出`first_scene_sensitivity.csv`。

统计：

- 每图先汇总两个重复，再 30 图等权；
- 地图簇 bootstrap 10,000 次，种子 `2026102002`；每次重抽保留该图四组和两区组；
- 完整输出 B−A、C−A、D−B、D−C 和 (D−C)−(B−A)；
- 率用百分点、频率用次/有效分钟，其他指标保留单位；
- UNKNOWN 情景差值固定为 `candidate_lower−reference_upper` 到 `candidate_upper−reference_lower`，再按指标方向标最不利/最有利；
- 情景边界明确标注为敏感性范围，不是置信区间；
- 不使用辅助综合分排名。

非正式三层汇总示例（构造 4 图×4 组数据）：
[example_three_layer_summary_NON_FORMAL.json](example_three_layer_summary_NON_FORMAL.json)（原始离线目录：`logs/07_formal_preparation/runner_readiness/offline_joint_test/`）。文件内明确标记 `NON_FORMAL_CONSTRUCTED_DATA`，不得混入正式数据。

## 6. 离线联测结果

执行：

```sh
/usr/bin/python3 -m py_compile \
  scripts/s7_formal_runner.py scripts/run_s7_formal_240.py \
  scripts/s7_formal_analysis.py scripts/analyze_s7_formal_240.py \
  scripts/check_s7_runner_readiness.py
/usr/bin/python3 scripts/check_s7_runner_readiness.py
```

结果：**21/21 PASS**。机器结果：
[check_results.json](check_results.json)（原始离线目录：`logs/07_formal_preparation/runner_readiness/offline_joint_test/`）。

覆盖：

| # | 检查 | 结果 |
|---:|---|---|
| 1 | 240 项读取完整，参数/调度不变 | PASS |
| 2 | A/B/C/D overlay 目录与消费字段正确（A/C 无 X，A/B 无 H） | PASS |
| 3 | 默认及缺少授权时拒绝真实运行（含 fixture 授权被生产入口拒绝） | PASS |
| 4 | 有效合法终局不补跑 | PASS |
| 5 | 场景前失败、创建后无 run_id、已进入 RL 分别落账 | PASS |
| 6 | 中断恢复不重复启动、不重复计数 | PASS |
| 7 | 创建事实不明时暂停，不盲目补跑 | PASS |
| 8 | 错误目录、run_id 跨项复用、替代绑定错误被拒绝 | PASS |
| 9 | 同项第二次替代、全批第 13 次替代被拒绝 | PASS |
| 10 | 安全中止缺独立工程证据时不自动替代 | PASS |
| 11 | 三层汇总保留原失败，分母不重复 | PASS |
| 12 | UNKNOWN/无效/未启动与已确认安全失败正确区分 | PASS |
| 13 | 任务窗口与站立/清理窗口分离 | PASS |
| 14 | 磁盘/替代额度达到上限时停止派发 | PASS |
| 15 | 同一输入重复生成汇总完全一致 | PASS |
| 16 | 构造小矩阵的四个主比较、交互与情景端点正确 | PASS |
| 17 | 已有真实开发记录读取与分类接入（R1=True、R6=False） | PASS |
| 18 | 台账只有意图、磁盘上已有完整产物时可安全恢复且不重跑 | PASS |
| 19 | 分析CLI在构造批次上端到端运行，两次`formal_summary.json`逐字节一致 | PASS |
| 20 | 替代成功不抹去首次实际启动的安全中止；全启动层保留两次事实 | PASS |
| 21 | 运行上下文记录的组别/阈值与计划不一致时暂停，不继续派发 | PASS |

真实记录核对只读取既有 T2 记录并把派生结果写入离线目录，未覆盖原 `metrics.json`：

- R1 `16465c346b8a4fd1b85c4548169fd6a2`：ARRIVED、VALID、安全完成 True、清理确认；
- R6 `1699cb934b7a4c73b2a666867eadf353`：SYSTEM_SAFETY_ABORT、VALID、安全完成 False、清理确认。

这两条只证明分类接入可追溯，不是成功率或组间结果。

## 7. 仍未被验证的边界

以下只能由后续真实运行或另行授权补证，不能由本轮离线结果宣称：

1. `s1_run_diagnostic.py` 在正式批次目录中的实际派发、controller 配置实际消费、首周期证据；
2. 真实中断后的进程/目录/记录恢复，以及真实 `shutdown_complete`/退出码核对；
3. 正式 240 项批次的分析 CLI 端到端运行；
4. MuJoCo 命令消费仍为 UNKNOWN；
5. 真实 GLFW/OpenGL 环境下隐藏窗口与 RViz 关闭；
6. 实机安全、训练和其他任务均未涉及。

建议的**最小开发场景补证**（需另行授权，不使用留出图）：

- 1 次开发图 A 组正式入口运行，核对实际配置消费、attempt/run_id/目录绑定、指标与收尾；
- 1 次开发图有意中断后恢复，核对真实场景事实落账和断点恢复；
- 1 次场景前失败注入，核对编排调用不消耗场景额度。

## 8. 未来命令说明

只读查看冻结计划：

```sh
/usr/bin/python3 scripts/run_s7_formal_240.py plan
```

无场景预检（不启动任何进程）：

```sh
/usr/bin/python3 scripts/run_s7_formal_240.py preflight
```

准备并检查独立 overlay（仍不启动场景）：

```sh
/usr/bin/python3 scripts/run_s7_formal_240.py preflight \
  --prepare-overlays --batch-root logs/08_formal_experiment/S7_ABCD_240/<batch-id>
```

状态查询：

```sh
/usr/bin/python3 scripts/run_s7_formal_240.py status \
  --batch-root logs/08_formal_experiment/S7_ABCD_240/<batch-id>
```

未来真实运行/恢复（本轮**没有**有效授权，命令会拒绝）：

```sh
/usr/bin/python3 scripts/run_s7_formal_240.py run \
  --batch-root logs/08_formal_experiment/S7_ABCD_240/<batch-id> \
  --batch-id <batch-id> --authorization <director-authorization.json>
```

恢复在同一命令加 `--resume`；仍要求同一有效授权。分析：

```sh
/usr/bin/python3 scripts/analyze_s7_formal_240.py \
  --batch-root logs/08_formal_experiment/S7_ABCD_240/<batch-id>
```

## 9. 版本与配置记录

批次开始时 `batch_manifest.json` 一次性记录：

- Git commit、分支、是否有未提交改动及状态摘要 SHA-256；
- 运行器/诊断器/分析器/安全分类/台账/overlay 脚本 SHA-256；
- 源配置与三份关键模型文件 SHA-256；
- 矩阵 `plan_sha256`、CSV/JSON SHA-256；
- 授权 ID、审阅引用、授权文件 SHA-256；
- A/B/C/D overlay 的配置与模型 SHA-256、独立文件列表。

运行中代码、模型或参数发生实质变化时，恢复入口因计划/授权绑定不一致而拒绝静默混批。

## 10. 结论

运行器和分析入口已具备可审阅的最小准备：默认拒绝未授权运行、冻结顺序派发、独立 overlay、原子 attempt 台账、场景事实不明暂停、安全恢复、替代额度、资源保护、三层汇总和同一冻结分析器重算。定向离线联测 21/21 通过。

这仍不是运行态验证，也不构成 240 项授权。**S7-01 已 COMPLETE；后续由 S7-02 承接。240 项未启动，`authorized_to_run=false`。**
