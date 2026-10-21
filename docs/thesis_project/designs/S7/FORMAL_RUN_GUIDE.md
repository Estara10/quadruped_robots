# S7-02 正式执行说明

状态：**S7-02 ACTIVE / AWAITING_RUN_AUTHORIZATION。240 项尚未获运行授权。**  
本说明只记录现有真实 CLI 的用法；不生成授权、不启动场景。所有 shell 命令以 `rtk` 开头。

## 0. 固定输入

- 冻结计划：`docs/thesis_project/designs/S7/formal_matrix_240.csv` / `.json`
- 计划摘要：`95c764729a7466455924518ae2b498e060f35a20d946e065af9f226f25c1c301`
- 正式批次根目录：`logs/08_formal_experiment/S7_ABCD_240/<batch-id>/`
- 运行入口：`scripts/run_s7_formal_240.py`
- 分析入口：`scripts/analyze_s7_formal_240.py`

`<batch-id>` 由正式执行任务在获得授权时确定；`<director-authorization.json>` 是待提供的独立运行授权占位。**当前仓库中不存在有效的 240 项运行授权。**

## 1. 计划查看（只读）

```sh
rtk proxy /usr/bin/python3 scripts/run_s7_formal_240.py plan
```

可选把输出保存到文件：

```sh
rtk proxy /usr/bin/python3 scripts/run_s7_formal_240.py plan --output <plan_snapshot.json>
```

该命令只读冻结矩阵，不启动 MuJoCo/ROS，不创建批次。

## 2. 运行前环境核对

确认没有本批遗留的项目进程（只查看，不自动结束无关会话）：

```sh
rtk proxy pgrep -af 'unitree_mujoco|s1_run_diagnostic|ros2_control|rviz2'
```

检查磁盘（保护线 15 GiB；正式批次预算 6 GiB）：

```sh
rtk proxy df -h .
```

检查批次是否已存在；如存在，先查询状态，不删除、不重建：

```sh
rtk proxy ls -d logs/08_formal_experiment/S7_ABCD_240/<batch-id>
```

执行无场景预检（不启动任何场景；`--prepare-overlays` 只准备独立 overlay）：

```sh
rtk proxy /usr/bin/python3 scripts/run_s7_formal_240.py preflight \
  --prepare-overlays \
  --batch-root logs/08_formal_experiment/S7_ABCD_240/<batch-id>
```

预期：冻结计划 240 项、30 图、60 区组、每组 60 项；A/B/C/D overlay 字段正确；磁盘高于 15 GiB。预检不是 controller 配置消费验证。

## 3. 正式启动（待授权，占位）

**在获得 Director 的独立授权文件前不要执行。** 真实入口没有绕过授权的开关；缺少或不匹配授权会返回 `REFUSED`。

```sh
rtk proxy /usr/bin/python3 -u scripts/run_s7_formal_240.py run \
  --batch-root logs/08_formal_experiment/S7_ABCD_240/<batch-id> \
  --batch-id <batch-id> \
  --authorization <director-authorization.json>
```

授权必须匹配冻结矩阵 `matrix_id`、计划摘要、批次 ID、240/12/252 额度、6 GiB 预算和 15 GiB 保护线。首次运行会创建批次清单、台账和 A/B/C/D 独立 overlay，然后按冻结顺序派发。

## 4. 状态查询（只读）

```sh
rtk proxy /usr/bin/python3 scripts/run_s7_formal_240.py status \
  --batch-root logs/08_formal_experiment/S7_ABCD_240/<batch-id>
```

状态输出包含批次状态、`pause_reason`、实际场景启动数、替代数、计划项结果、attempt 与 run_id。暂停后先查状态和对应日志，不要直接重跑。

## 5. 暂停后恢复（显式）

恢复使用同一批次、同一 batch ID、同一授权，并显式 `--resume`：

```sh
rtk proxy /usr/bin/python3 -u scripts/run_s7_formal_240.py run \
  --batch-root logs/08_formal_experiment/S7_ABCD_240/<batch-id> \
  --batch-id <batch-id> \
  --authorization <director-authorization.json> \
  --resume
```

恢复会先核对已有进程、目录、日志和台账；已完成且合法的计划项不重跑，冻结顺序不变。场景创建事实不明时暂停，不能当作未启动自动重跑。

## 6. 分析

正式分析从原始记录重算，默认使用统一决策时钟重建规则、地图等权、10,000 次地图簇 bootstrap 和种子 `2026102002`：

```sh
rtk proxy /usr/bin/python3 scripts/analyze_s7_formal_240.py \
  --batch-root logs/08_formal_experiment/S7_ABCD_240/<batch-id>
```

默认输出到 `<batch-root>/analysis/formal_summary/`；可用 `--output-dir` 指定独立派生目录。正式分析不要使用 `--use-stored-metrics`（该选项只用于离线构造夹具），也不要使用 `--allow-constructed-plan`。

## 7. 输出目录与文件

```text
logs/08_formal_experiment/S7_ABCD_240/<batch-id>/
  batch_manifest.json          # 计划/授权/版本/overlay/限额
  batch_state.json             # 当前状态与暂停原因
  ledger/attempts.json         # attempt 台账、事件、检查点
  overlays/A|B|C|D/            # 独立实体 overlay
  runs/<plan_item_id>/attempt_NN/result/<run_id>/
  analysis/formal_summary/
    formal_summary.json
    primary_plan_items.csv
    first_scene_sensitivity.csv
    comparisons.csv
    reconstruction_sensitivity.json
    SUMMARY.md
```

原始 run、失败 attempt、台账、版本清单和派生结果保留；不删除正式原始记录腾空间。

## 8. 常见停止原因

状态查询中的 `pause_reason` 是准确定位；常见包括：

| 原因 | 含义/处理 |
|---|---|
| `missing_explicit_batch_authorization` / `authorization_rejected` | 没有独立授权、授权不匹配或缺失；停止，等待 Director |
| `existing_batch_requires_explicit_resume` | 批次已存在；先查状态，再用 `--resume` |
| `batch_plan_binding_mismatch` / `batch_matrix_id_mismatch` | 批次与冻结计划不一致；不混批，报告 |
| `version_binding_changed` | 代码/模型/配置/脚本版本变化；暂停审阅，不静默续跑 |
| `batch_processes_still_running` / `residual_processes_after_run` | 仍有本批进程；核对来源，确认退出后再决定 |
| `ledger_audit_rejected` | run_id/目录/替代/台账冲突；保留原始记录，报告 |
| `free_space_below_guard` / `batch_budget_exceeded` | 低于 15 GiB 或超出 6 GiB；暂停，不删原始数据 |
| `scene_start_cap_reached` / `substitution_quota_exhausted` | 达到 252 次启动或 12 次替代；停止，需另行授权 |
| `scene_creation_unknown` / `scene_creation_unknown_after_launch` | 创建事实不明；暂停核实，不自动重跑 |
| `record_validity_unknown` / `rl_entry_fact_unknown` | 记录或 RL 进入事实不足；保留并报告 |
| `terminal_source_conflict` / `run_context_setting_mismatch` / `runtime_config_consumption_not_confirmed` | 终局或实际配置消费不一致；暂停核对 |
| `run_cleanup_unconfirmed` / `run_analysis_failed` | 收尾或单 run 分析未确认；暂停，先核对日志 |
| `operator_interrupted` | 操作者中断；保留中断事实，确认清理后再决定 |

合法碰撞、跌倒、超时不是补跑理由；UNKNOWN 不填 False/0。

## 9. 版本与配置

批次开始时 `batch_manifest.json` 自动记录 Git commit、分支、是否有未提交改动、代码/诊断器/分析器/安全分类/台账脚本、源配置、三份关键模型、矩阵摘要、授权和 overlay 哈希。正式运行前只需确认该清单与当前环境一致；不要求清空工作树、恢复历史 overlay 或额外建立哈希体系。无关未提交文件只记录，不自动作为阻塞。

## 10. 清理与停止

每次 run 由单 run 诊断器清理其拥有的 MuJoCo/ROS/controller/查询会话；runner 在收尾未确认或发现残留时暂停。批次结束或暂停后，用第 2 节的进程检查核对本次进程；不能仅凭窗口关闭或已发送中断声称清理完成。只处理本项目本次批次相关进程，不影响无关会话。

**当前仍未授权 240 项；本说明不构成运行授权。**
