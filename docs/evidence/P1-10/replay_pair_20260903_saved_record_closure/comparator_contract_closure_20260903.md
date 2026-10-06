# P1-10 Saved-Record Comparator Contract Closure

状态：**REJECT 修复完成，FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW**。

本次只处理 Independent Reviewer 已确认的 saved-record comparator fail-closed
缺口：terminal domain、artifact origin、binding 回填和 production context
输出。没有启动 MuJoCo、ROS2、Run A、Run B、benchmark、FormalRun 或
P1-11/P1-12/P1-13；没有重试历史失败 pair；没有生成 pair runtime evidence；
没有 commit/push。

## REJECT 与修复范围

原 comparator 只读取 pair manifest 和两份 runtime JSONL，因而可能接受没有
process facts、resolved manifest 或 P1-10 context 的记录，也允许通过外部
run JSONL 路径绕过冻结 pair。本次修复：

- CLI 只接受 `--pair-dir`，由冻结 `pair_manifest.json` 严格推导直接子目录
  `run_A` 和 `run_B`；拒绝任意 `--run-a`/`--run-b` 路径、路径逃逸和错误 pair。
- 每个 run 必须存在并解析四份文件：`process_facts.json`、
  `runtime_record.jsonl`、`scenario_resolved_manifest.json`、
  `p1_10_context.json`。
- process facts 强制验证 coordinator exit/shutdown/forced/SIGINT、required
  child wait rc、PID/PGID、signal escalation、cleanup 和 run binding。
- runtime record 强制要求 VALID authoritative source、唯一且末尾 terminal、
  完整 terminal success fields、terminal/process/run/session 一致性；缺失、
  `None` 或不允许的 `UNKNOWN` fail closed。
- 两份 context 与 pair manifest 完整匹配 scenario/suite hash、variant/binding、
  seed、window、baseline document hash、canonical baseline identity 和
  `scene_default / mj_makeData:qpos0` initial-state binding；缺失 binding 不从
  pair、另一份 context 或默认值回填。
- `scenario_resolved_manifest.json` 和 `p1_10_context.json` 都必须显式提供
  全部 pair-required binding，包括 `scene_root_sha256`、`model_closure_sha256`
  和 `initial_state_qpos_sha256`；任何 None/UNKNOWN/错误类型或 hash 均拒绝。
- production writer `scripts/p1_08_baseline_capture.py::write_p1_10_context`
  现在从 validated resolved scenario context 显式复制完整 scene binding，
  包含 `scene.root_xml_sha256`、`scene.model_closure_sha256` 及其他 full-binding
  scene/context 字段；不从 comparator、pair manifest 或默认值取值。
- pair 根目录、run 目录和四份 required artifact 均经 `lstat` 检查为 frozen
  pair 内的普通目录/文件；symlink、resolve 后越界、类型或 lstat 失败均拒绝。
- terminal domain 严格绑定实际 recorder：`termination_reason` 为五个 recorder
  值之一，安全 fault 字段为 bool，当前四个无权威 producer 的事件字段只能是
  精确字符串 `UNKNOWN`。
- exact/numeric/excluded 投影规则保持冻结；numeric 仍为 canonical JSON exact
  equality并报告诊断 delta；只读落盘 records，不访问 live shared memory；
  四个 comparison 输出不可覆盖，并增加确定性的 Markdown report。

## 离线测试

`scripts/test_p1_10_saved_record_compare.py` 的所有测试都从真实临时
`repo/docs/evidence/P1-10/<pair>/run_A|run_B` 布局读取文件，并通过 CLI
入口验证，不只调用内部理想对象。覆盖：

- 有效完全一致 fixture 通过，excluded metadata 变化通过；
- numeric 差异失败并报告 diagnostics；
- process facts 的 missing/None/UNKNOWN、非零退出、shutdown 不完整、非正常
  shutdown、forced termination、错误 shutdown source、child wait 非零；
- terminal 缺失字段、重复 terminal、terminal 不在末尾；
- scenario、variant、baseline、seed、window、initial-state drift；
- 外部 run path / 错误 pair-dir、四个 required 文件缺失；
- output overwrite 拒绝。
- pair/run/artifact symlink 与 resolve 越界拒绝；两边同时使用非法
  `termination_reason` 也拒绝；terminal bool/event 字段的非法类型和值均拒绝。
- required binding 的 missing/None/UNKNOWN/错误类型，以及 process-facts
  内嵌 context 缺失均拒绝。
- production writer → 实际 `p1_10_context.json` → 完整 pair-dir → comparator
  闭环通过；producer 输出的三个 scene/initial-state 字段分别删除或篡改均拒绝。

本次离线 comparator 测试结果：**17 个测试方法 PASS**（包含 filesystem
layout、terminal domain、binding、process-facts、numeric 和 overwrite 子测试组）。

## 冻结状态

pair：`P1-10-REPLAY-20260903-saved-record-closure-flat_goal_forward-stabilized`

manifest：`pair_manifest.json`，schema
`abs-go2-p1-10-same-seed-replay-pair/v3`，状态
`FROZEN_OFFLINE_PENDING_INDEPENDENT_REVIEW`。

最终 pair manifest SHA-256：
`86ae55914db294d269d6f70909bfad1878c287c644f5a85c4075fa758f923a6c`。

comparator SHA-256：
`09a736e420531179f79e0f947307ff36b33bd748df1a354de57b2d24a2ffc9c9`。

Operator runbook SHA-256：
`b6a8ce0f433b8e9a81e7b5dc72cbe330ccb1667bcd2cec4df318d8236d6b4dc8`。

production context writer SHA-256：
`97e4fe7190148cccd70e41832b1f2c8668fbd0badb3b405d0726db6d0d7bf9ec`。

baseline identity document SHA-256：
`6c3563c25d45cc275db6b083f9f0fc0cc2067b48bc8f4a93dcace9f6d42817ea`。

canonical baseline identity：
`59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0`。

在独立 Reviewer 明确通过离线 pair 冻结和 comparator contract 之前，
Operator 不得执行 A/B；P1-10 总体仍为 **IMPLEMENTED / AWAITING INDEPENDENT
REVIEW**。
