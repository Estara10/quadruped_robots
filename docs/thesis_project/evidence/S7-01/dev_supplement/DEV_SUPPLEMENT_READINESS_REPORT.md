# S7-01 开发图补证入口准备（m02 A/D）

日期：2026-10-21  
状态：**入口已准备；随后已实际执行 A/D，见[执行报告](DEV_SUPPLEMENT_EXECUTION_REPORT.md)。** 本轮准备阶段未启动场景、未提交 Git。这是**开发运行补证**，不是正式 240 项数据，也不授权正式实验。  
S7-01 后续已 COMPLETE（Director 有条件通过）；本报告为准备阶段历史记录。240 项矩阵未修改、未启动、未获运行授权。

## 1. 范围

固定两项，仅在开发图 `m02.xml`（`m2v2_m02`）上运行：

| 顺序 | 计划项 | 组别 | E | X | H |
|---:|---|---|---:|---|---:|
| 1 | `DEV-M02-A` | A | −0.05 | 不配置 | 不配置 |
| 2 | `DEV-M02-D` | D | −0.05 | −0.10 | 0.20 仿真秒 |

统一条件：目标 `(7.5,0)`、半径 0.5 m、20 仿真秒/120 墙钟秒、到达后站立观察 2 仿真秒、隐藏 MuJoCo、关闭 RViz。

- 不使用留出图，不修改 240 项矩阵。
- 最多 **2 次实际场景启动**，**0 次工程替代**，不做自动补跑。
- 合法碰撞、跌倒、仿真限时或墙钟保护都是结果，不以 ARRIVED 作为补证通过条件。
- 不故意中断运动中的仿真，不做真实故障注入。
- 输出目录：`logs/07_formal_preparation/dev_supplement_m02_AD/<batch-id>/`
- 数据标记：`DEV_SUPPLEMENT_NOT_FORMAL`；manifest、attempt、诊断 classification 和分析输出均带非正式标记。

## 2. 新增入口

| 文件 | 作用 |
|---|---|
| `scripts/s7_dev_supplement.py` | 两项计划、独立授权校验、复用 `s7_formal_runner.run_batch`、开发一致性分析 |
| `scripts/run_s7_dev_supplement.py` | CLI：`plan` / `preflight` / `status` / `run`（只跑 A 后暂存）/ `resume`（只跑 D）/ `analyze` |
| `scripts/check_s7_dev_supplement.py` | 定向离线联测；只写 `logs/07_formal_preparation/dev_supplement_m02_AD/readiness_check/` |
| `docs/thesis_project/designs/S7/dev_supplement/m02_AD_plan.json` | 两项冻结计划 |
| `docs/thesis_project/designs/S7/dev_supplement/m02_AD_authorization.json` | 独立两项授权，最多 2 次启动、0 替代 |

复用而非复制批次核心：实际派发、overlay、run_id/目录绑定、场景创建事实、原子 attempt 台账、恢复、配置消费核对、安全分类、清理确认和三层视图都调用 `s7_formal_runner.py` / `s7_formal_analysis.py`。开发入口只增加自己的计划、授权、额度、输出目录和单步暂存逻辑。

## 3. 计划与授权

- 计划摘要：`264b7f1660c094e1123d8ac8d473cc2d3db6975b83aba1a0313f3500a06aae38`
- 计划文件 SHA-256：`d8629d7132129cedf9f5c4e327103e9575aa8c693bd4a1e857f46078e68c0342`
- 授权 ID：`DEV-M02-AD-20261021-01`
- 固定 batch ID：`dev-m02-ad-20261021`
- 授权范围：`DEV_SUPPLEMENT`，`not_formal_data=true`，`holdout_prohibited=true`，`no_automatic_retry=true`，`intentional_motion_interruption=false`，`failure_injection=false`
- 额度：`max_scene_starts=2`、`max_substitutions=0`、批次预算 1 GiB、可用空间保护线 15 GiB
- 正式 240 项授权仍不存在；开发授权与正式授权 schema/plan 不同，正式入口会拒绝该开发授权。
- 离线检查确认：缺少授权时拒绝且不创建批次目录；开发授权不能被正式入口接受；`fixture_only` 授权被开发入口拒绝。

## 4. A → 检查点 → D

- 首次 `run` 只派发 A；A 结束、进程收尾确认且无残留进程后，写检查点和 `batch_state`，返回 `status=STAGED`，`next_pending=["DEV-M02-D"]`。
- 之后由用户显式调用 `resume`；计划项 A 已有效选定，恢复不会重跑 A，只派发 D。
- A 是合法碰撞/超时也照样暂存，D 仍可恢复；不以 ARRIVED 为通过条件。
- A 若场景前失败、记录无效、配置消费不一致或收尾未确认，批次 `PAUSED`；开发入口禁止自动补跑。
- 全批最多 2 次实际场景启动；超过即暂停。

## 5. 定向离线联测

执行：

```sh
/usr/bin/python3 -m py_compile \
  scripts/s7_dev_supplement.py scripts/run_s7_dev_supplement.py \
  scripts/check_s7_dev_supplement.py
/usr/bin/python3 scripts/check_s7_dev_supplement.py
```

结果：**11/11 PASS**。机器结果：[check_results.json](check_results.json)。

| # | 检查 | 结果 |
|---:|---|---|
| 1 | 计划恰为 m02 A/D 两项，参数/顺序正确，无留出图 | PASS |
| 2 | 开发授权可用；正式入口拒绝开发授权；缺授权拒绝且不建批次目录；fixture 授权被拒 | PASS |
| 3 | 首次运行只跑 A，A 后写检查点并暂存，D 待运行 | PASS |
| 4 | 恢复只跑 D，不重跑 A；场景启动共 2 次，顺序 A→D | PASS |
| 5 | A 合法碰撞仍暂存，D 可恢复；不因未 ARRIVED 补跑 | PASS |
| 6 | A 合法仿真超时仍暂存，D 可恢复；不因未 ARRIVED 补跑 | PASS |
| 7 | A 场景前失败暂停，恢复不自动重试 | PASS |
| 8 | 运行上下文配置消费不一致时暂停 | PASS |
| 9 | 第 3 次启动额度、替代额度和 fixture 授权均被拒绝 | PASS |
| 10 | 分析核对派发/配置/台账/安全/清理一致，A 在 D 前 | PASS |
| 11 | 状态查询显示暂存阶段和下一项 D | PASS |

这些是离线构造检查，**没有启动 MuJoCo/ROS/controller/模型推理**，不宣称运行态恢复已验证。

另执行了无场景的 `preflight --prepare-overlays`：只生成A/D两份独立overlay并验证配置字段（A无X/H，D有X=−0.10/H=0.20），未创建正式运行批次目录。结果见[overlay_preflight.json](overlay_preflight.json)。

## 6. 运行命令

以下命令会真实启动场景。**准备轮没有执行；实际执行已使用这些命令完成，结果见[执行报告](DEV_SUPPLEMENT_EXECUTION_REPORT.md)。** 运行前确认无其他 MuJoCo/ROS/controller 会话、显示环境可用、磁盘高于 15 GiB。

### 6.1 首次运行 A

```sh
cd /home/lidio/quadruped_robots
/usr/bin/python3 scripts/run_s7_dev_supplement.py run \
  --batch-root logs/07_formal_preparation/dev_supplement_m02_AD/dev-m02-ad-20261021 \
  --batch-id dev-m02-ad-20261021 \
  --authorization docs/thesis_project/designs/S7/dev_supplement/m02_AD_authorization.json
```

预期：A 完成并退出为 `status=STAGED`（退出码 0），终端打印 D 的恢复命令。如果返回 `PAUSED`（退出码 3），停止并检查原因，不要自动重跑。

### 6.2 恢复运行 D

在确认 A 已结束且本次进程已退出后：

```sh
cd /home/lidio/quadruped_robots
/usr/bin/python3 scripts/run_s7_dev_supplement.py resume \
  --batch-root logs/07_formal_preparation/dev_supplement_m02_AD/dev-m02-ad-20261021 \
  --batch-id dev-m02-ad-20261021 \
  --authorization docs/thesis_project/designs/S7/dev_supplement/m02_AD_authorization.json
```

预期：只派发 D，不重跑 A；完成后 `status=COMPLETE`。合法碰撞/超时同样是完成结果。

### 6.3 状态查询

```sh
cd /home/lidio/quadruped_robots
/usr/bin/python3 scripts/run_s7_dev_supplement.py status \
  --batch-root logs/07_formal_preparation/dev_supplement_m02_AD/dev-m02-ad-20261021
```

### 6.4 分析

```sh
cd /home/lidio/quadruped_robots
/usr/bin/python3 scripts/run_s7_dev_supplement.py analyze \
  --batch-root logs/07_formal_preparation/dev_supplement_m02_AD/dev-m02-ad-20261021
```

输出默认写入批次的 `analysis/`：`dev_supplement_analysis.json` 和 `DEV_SUPPLEMENT_ANALYSIS.md`。分析只核对两项的一致性，不计算正式组间比较、成功率或推断结论。

### 6.5 无场景预检（可选）

```sh
/usr/bin/python3 scripts/run_s7_dev_supplement.py preflight \
  --prepare-overlays \
  --batch-root logs/07_formal_preparation/dev_supplement_m02_AD/dev-m02-ad-20261021
```

预检不启动场景，也不证明 controller 已消费配置；实际消费证据由每次运行后的 `runtime_config_observed` / `runtime_source_validation` 核对。

## 7. 当前环境与限制

- 准备轮未启动场景；实际执行见[执行报告](DEV_SUPPLEMENT_EXECUTION_REPORT.md)。
- 未修改 240 项矩阵；正式 `authorized_to_run=false`。
- 未提交 Git。
- 仍未验证：真实 A/D 派发、真实 controller 配置消费、真实中断后的恢复、真实 GLFW/RViz 和清理、分析入口在真实记录上的结果。
- 若 A 或 D 真实运行返回 `PAUSED`，保留台账和原始记录，先查 `status` 与分析，不擅自追加第三次启动。
