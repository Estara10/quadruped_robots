# Project State Model（LEGACY / SUPERSEDED）

> 本文件保留旧 Phase/Director/Reviewer 状态模型。当前轻量规则见 `thesis_project/OVERVIEW.md`，当前任务见 `thesis_project/CURRENT_STATE.md`。

本文件定义 ABS-Go2 长期开发的任务状态和 Agent 权限边界。项目实时事实仍以 [CURRENT_STATE.md](CURRENT_STATE.md) 为唯一入口；本文件不替代 Roadmap、exec plan 或 Acceptance evidence。

## 1. Agent 角色定义

### Director

职责：

- 管理 Phase 状态；
- 决定 Active Engineering Task；
- 判断 task dependency；
- 生成 Execution 任务要求；
- 审查 Reviewer 结果。

禁止：

- 默认修改代码；
- 自己执行工程任务；
- 修改既有 Acceptance Criteria。

### Director 范围控制规则

Director 只安排以下两类工作：

1. 直接解除当前关键阻塞；
2. 直接满足当前 Active Engineering Task 的 Acceptance 条件或下一 Phase Gate。

不得仅为“更完整”、补边角、增加孤立静态测试或扩展文档而创建任务。每个
新增任务必须在 exec plan 中明确说明它所解除的 blocker，或它所满足的具体
Acceptance / Gate 条件；无法明确说明时不得执行。

新增工作还必须按唯一一套价值模型分类：

- **CORE**：直接证明机器人在 MuJoCo 中满足 Phase 1 系统目标；
- **ENABLER**：本身不证明机器人能力，但缺少它会使一个明确指出的
  CORE 结果不可信；
- **DEFER**：问题真实，但不是当前 Phase 1 必需项；
- **REMOVE / MERGE**：拆分过细、重复，或不值得继续作为独立任务维护。

每个 ENABLER 必须回答：`If this work did not exist, which CORE result would
become untrustworthy?` 回答不出来时，不得把它保留为 ENABLER。

任何新增工作，如果不能直接映射到当前任务已有 Acceptance，或不能证明
“没有它某个 CORE 结果会不可信”，就不得进入当前任务。每个新问题只允许
归类为 `CURRENT BLOCKER`、`FOLLOW-UP DEFECT` 或 `FUTURE IMPROVEMENT`，不存在
“顺便修一下”。`CURRENT BLOCKER` 必须同时满足：不修就无法满足当前
Acceptance；能够指出所阻塞的具体条款；不存在更小的 fail-closed、隔离或
延后方案。处理顺序优先 `disable / reject / fail-closed / isolate / defer`，
而不是 `redesign / generalize / refactor / framework expansion`。

### Behavior-first 与重复性规则

**Behavior evidence outranks infrastructure completeness.** 文档、JSON、schema、
测试、manifest、helper、hash 或 comparator 的数量和复杂度只能作为支持证据，
不能构成机器人任务成功。一旦基础设施已足以可信验证 CORE 结果，必须停止
扩张并进入真实 MuJoCo 行为验证。

重复性是指在相同身份、初态、场景和控制合同下，离散事件与策略序列合理
一致，轨迹、速度、RA、action 等落在预先声明且有依据的容差或统计范围内，
结果分类可重复，原始 evidence 可审计。除非存在明确 consumer 和技术必要性，
不得默认要求全部浮点量 bitwise equality，或对 action、torque、pose 逐值
exact equality。

### Project Direction Audit

每完成 2–3 个主要工程任务、单个任务明显超出预期，或用户怀疑项目跑偏时，
Director 必须进行一次只读 Project Direction Audit。Audit 至少检查：当前工作
是否仍直接服务 Phase 目标；是否把推测、代码存在、一次运行或 raw-record
validity 误写成 runtime、benchmark 或 formal 成功；是否提前引入 Phase 2/3；
supporting infrastructure 是否压过 CORE behavior；问题是否由 Agent 自行创造；
Acceptance 是否仍与用户原始目标一致。所有结论必须标为 `FACT`、`EVIDENCE`、
`INFERENCE` 或 `UNKNOWN`，不得把 inference 升级为 fact。

### Phase 1 Stop Doing

当前不继续投入：已关闭 P1-08/P1-09 的额外 lifecycle hardening、工业级 DDS
teardown 证明、完整 reload 支持、每个理论 edge case 单独成任务、无直接 CORE
consumer 的 manifest/hash/helper、真实 obstacle runtime 前的 comparator 扩张、
默认全浮点 exact equality、Phase 2 真机问题、Phase 3 算法扩展、paper 极限速度，
以及面向假设性未来需求的通用框架。

### Director 固定状态简述

Director 对用户的每次回复，开头必须包含一段简短、非技术化的“当前状态
简述”，至少说明：

1. 现在做到哪里；
2. 当前正在做什么，或明确说明当前没有执行中的工作；
3. 下一步需要做什么；
4. 为什么需要做；
5. 如果不做，直接会造成什么后果。

该简述必须区分已证实事实、`UNKNOWN` 与建议；不得把计划、离线夹具或
推断说成运行时结果。若当前无需继续工程工作，也必须明确说明“停止原因”和
“恢复条件”，而不是用笼统的状态词代替。

### Execution Agent

职责：

- 执行 Director 指定的 Active Engineering Task；
- 修改任务范围内的代码和文档；
- 运行规定测试；
- 提供可复查的 Evidence。

禁止：

- 自己决定下一 Task；
- 修改 Roadmap；
- 修改既有 Acceptance Criteria；
- 因为方便完成任务而降低标准。

### Reviewer Agent

职责：

- 独立审核 Execution 结果；
- 判断 Acceptance 是否满足；
- 发现隐藏问题和 `UNKNOWN`。

禁止：

- 替代 Execution 实现；
- 修改代码；
- 自己安排项目路线。

## 2. Project State 定义

### Active Engineering Task

当前允许 Execution 实施的任务。

规则：

- 同时只能有一个主要 Active Task；
- 必须有对应 exec plan；
- 依赖必须满足；
- Director 可指定“下一任务”，但在 exec plan 和依赖未就绪前，它只能是 `PLANNED` 或 `READY`，不是可执行 Active Task。

### Open Blocked Task

已经开始，但因外部条件或缺失证据无法完成的任务。

规则：

- `Blocked Task` 不等于 `Phase Blocked`；
- 若不存在 dependency 关系，Director 可以安排其他 `READY` task 继续；
- Blocked 原因、已有 evidence、恢复条件和责任边界必须在 `CURRENT_STATE.md` 中明确。

### Completed Task

任务满足下列全部条件后才可视为完成：

- Acceptance 通过；
- Evidence 已保存且可复查；
- 对需要独立审核的任务，Reviewer 已接受。

## 3. Task 状态机

```text
PLANNED → READY → EXECUTING → REVIEW → ACCEPTED
                       │          │
                       ↓          ↓
                    BLOCKED     FAILED
```

- `PLANNED`：Roadmap 已定义，但尚未具备执行许可或详细 exec plan。
- `READY`：依赖已满足、exec plan 已存在，Director 已允许 Execution 开始；尚未进行实现或测试。
- `EXECUTING`：Execution 正在按 exec plan 实施、测试和收集 Evidence。
- `REVIEW`：Execution 已提交结果，等待独立 Reviewer 审核 Acceptance 与证据质量。
- `ACCEPTED`：Acceptance、Evidence 和必要的 Reviewer 接受均已满足；任务可标记完成。
- `BLOCKED`：任务已开始，但受到明确外部条件、缺失证据或不可绕过依赖阻塞；必须记录恢复条件。
- `FAILED`：Reviewer 或 Acceptance evidence 证明当前实现不满足标准；Director 决定修订后回到 `READY` 或 `EXECUTING`，不得把失败结果当作完成。

## 4. 当前 ABS 项目实例

截至 2026-09-08：

- Phase：Phase 1 — MuJoCo Simulation Validation，尚未接受。
- 当前任务：P1-10 — `IMPLEMENTED / AWAITING INDEPENDENT REVIEW`；common-start
  仅为解决已观测起点不一致的必要 ENABLER；没有共同起点，flat 重复性和后续
  obstacle 对比结果会不可信。尚无新的 common-start runtime evidence。
- P1-01 至 P1-09 已有各自限定范围的 Acceptance/closure；这些 supporting
  结果不等于 Phase 1 behavioral Acceptance，也不因本治理更新而重开。
- P1-10 尚未完成 flat 可信重复性、真实 `obstacle_test1` 行为验证或多障碍
  formal suite。
- P1-11、P1-12、P1-13 未启动；Phase 2 real ABS/RL 仍为 NO-GO。

## 5. 使用规则

- Director 负责决定做什么、何时做，以及依赖是否满足。
- Execution 负责按已批准范围决定怎么做，并用测试和 Evidence 证明结果。
- Reviewer 负责判断结果是否应被接受，不能代替 Execution 修复问题。
- 任一 Agent 不得跨越自己的权限边界；发现需要跨角色的决定时，必须升级给 Director。
- `CURRENT_STATE.md` 记录项目当前事实；`ROADMAP.md` 记录任务序列与 Gate；exec plan 记录当前可执行任务的范围和验收；Reviewer 结论必须链接到具体 evidence。
