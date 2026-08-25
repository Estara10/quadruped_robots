# Project State Model

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

截至 2026-08-25：

- Phase：Phase 1 — MuJoCo Simulation Validation。
- Director 指定的下一项工程任务：P1-02 — Formal Experiment Contract。
- P1-02 执行状态：`PLANNED`；本次状态治理工作不创建 exec plan，也不开始实施。它只有在 Director 提供 exec plan 且依赖核验完成后才可进入 `READY` / Active Engineering Task。
- Open Blocked：P1-01 — Policy Artifact Provenance and Joint/Contact/Action Order Contract。
- P1-01 阻塞原因：Training server unavailable；服务器侧 checkpoint/export/run、RA dataset binding 与 provenance-backed artifact order 证据尚未闭环。
- Phase：未被 P1-01 单独阻塞；P1-01 与无依赖关系的任务可由 Director 另行安排。

P1-01F 的部署 contract 修正与 live fault-injection evidence 已通过，但它不解除 P1-01 的 provenance/order blocker，也不等同于 Phase 1 Acceptance。

## 5. 使用规则

- Director 负责决定做什么、何时做，以及依赖是否满足。
- Execution 负责按已批准范围决定怎么做，并用测试和 Evidence 证明结果。
- Reviewer 负责判断结果是否应被接受，不能代替 Execution 修复问题。
- 任一 Agent 不得跨越自己的权限边界；发现需要跨角色的决定时，必须升级给 Director。
- `CURRENT_STATE.md` 记录项目当前事实；`ROADMAP.md` 记录任务序列与 Gate；exec plan 记录当前可执行任务的范围和验收；Reviewer 结论必须链接到具体 evidence。
