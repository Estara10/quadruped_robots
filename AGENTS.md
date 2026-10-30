# Project

Complete the passed undergraduate thesis project on risk-driven safe policy switching for Unitree Go2 in complex static-obstacle environments.

@/home/lidio/.codex/RTK.md

## Current Source of Truth

Read only this lightweight set for current planning:

- `docs/thesis_project/OVERVIEW.md`
- `docs/thesis_project/ROADMAP.md`
- `docs/thesis_project/CURRENT_STATE.md`
- `docs/thesis_project/EXPERIMENT_PLAN.md`
- the single active file under `docs/thesis_project/tasks/`

The passed thesis proposal is the highest research authority. Code and direct repository evidence determine what is actually complete.

## Legacy Boundary

Old P1 tasks, gates, reviewers, exec plans, evidence bundles and top-level governance Markdown are `LEGACY`. Preserve them, but do not continue, repair or close them unless the current thesis task explicitly needs a reusable technical asset.

## Working Rules

- Use minimum sufficient verification: enough to trust the next step and the thesis result, not an industrial assurance programme.
- Keep only one main Active Task.
- A blocker must prevent the next technical step, invalidate the experiment, create a clear real-robot safety risk, or deviate from the passed proposal.
- Do not require nonessential hashes, manifests, validators, clean worktrees, historical provenance recovery or legacy P1 closure.
- Before formal experiments, verify only the interfaces and semantics that can change the result: dimensions, orders, units, frequency, thresholds, hold time, RA/Recovery behavior and key metrics.
- Record Git/model/config versions once when freezing the formal experiment; hash only critical models when useful.
- Simulation precedes real-robot testing. Real testing stays low-speed, monitored and `NO-GO` until basic safety readiness passes.
- Never present PLANNED or UNKNOWN work as completed fact.
- Do not train models, start formal experiments or perform real RL unless the active thesis stage authorizes it.

## 本科毕设范围与防止过度工程化

- 本项目是本科毕业设计，不是企业级工程或安全认证项目。严谨性与必要安全要求必须保留，但不得无休止地增加防御性编码和验证流程。
- 采用最小充分验证：只解决会影响实验结论、阻止下一技术步骤或造成实际安全风险的问题。每项工作先明确完成条件，满足后进入下一阶段，不自动扩展检查范围。
- 不反复新增非必要的哈希校验、探针、快照、证据包、门禁、授权文件、夹具、冒烟测试或基线验证。使用这些手段前，必须能说明它将解决哪个具体问题；不得仅为防范假设风险而建设新机制。
- 已验证且本次未实质改变的功能直接复用，不要求每次重新证明。正式实验冻结时记录一次代码、模型和主要配置版本，不为每次运行重复建立复杂身份链。
- 不影响当前主结论的 UNKNOWN 如实保留为局限，不要求全部清零，不以此无限追加补证、运行或历史审计。
- 优先复用已有运行器、记录器和分析代码。只做当前任务必需的最小修改，不建设与论文研究问题无关的通用框架或工业级容错系统。
- 实验失败和负面结果可以是研究结论，不为了让某组获胜而持续调参、重跑或扩大开发范围。
- 必要的安全停止、真实故障处理、原始实验数据保存和本次进程清理继续执行；精简工程流程不得变成放宽实际安全规则或伪造完整性。
- 默认不自动追加新检查或实验。只有任务中已授权且确有必要的工作才执行；一般授权沿用当前会话，不为可逆操作和已授权步骤反复索要确认，也不额外建设审批流程。

## 最小实现与如实沟通

- 不把假设当事实，不根据未查证的路径、接口、数据或运行状态推进工作。不确定或困惑时主动说明；影响当前任务正确性或范围的疑问必须澄清，不能静默猜测。
- 主动暴露权衡取舍，说明选择的理由、代价及尚不能确定的部分，不掩盖问题，不把失败包装成通过。
- 只写解决当前问题所需的最小代码，不做任何推测性功能。优先复用现有实现，只修改必须改的地方，不顺手重构、优化或清理无关内容；负责修复本次工作自己产生的问题。
- 开始前明确定义成功标准，围绕该标准验证并持续修正，验证通过后停止扩展。成功标准是实现和实验能够可信回答当前问题，不要求实验得到正向结果；不得通过重跑合法失败或改参数制造通过。
- 只在系统边界校验，例如配置输入、外部数据和进程或设备交互；内部使用已确认的约定，不重复叠加校验。
- 禁止为不可能发生的场景添加错误处理、回退、空值检测或验证；不得为假想风险添加防御分支，也不能用默认值、吞异常或放宽规则掩盖真实错误。
- 禁止为一次性操作创建辅助函数、工具类或抽象；直接表达当前操作，不为未来可能的需求建设通用框架。
- 保留实际安全边界和已知外部故障的必要处理。遇到真实故障，明确记录发生了什么、影响哪些结果、已采取什么修正，不静默跳过、不伪造完整性。

## 策略切换研究的当前共识

- 后续切换约束作用与代价研究单列为R1项目，不继续叠加到S7；既有S7实验、结果、日志和选参历史保留原归属。项目入口为`docs/thesis_project/designs/R1/README.md`，当前任务以CURRENT_STATE为准。
- 接受短时间内频繁、反向切换是现有风险驱动策略的正常运行现象，不预设它有害，也不把证明它有害作为后续研究的前置任务。
- 研究重点是在这种正常现象存在的情况下，比较双阈值、最小保持时间及其组合对碰撞、到达、超时、任务推进和耗时的作用与代价。
- 切换次数和Recovery占比用于描述、解释行为；不能仅因切换更少就宣称方法更好、更安全或更有效。
- 优先利用已有数据查清碰撞、超时和推进不足，再决定是否调整E、X或H。是否继续试验取决于明确的问题，不以让B/C/D胜过A为目标。
- 不追改已完成实验的指标、选参规则或结果。新分析标明用途，后续优化配置单独命名；原A/B/C/D及其结果保留。
- 规划和对用户说明使用简洁、通俗的中文，先讲要解决的问题、打算怎么做和完成条件，再提供必要技术细节。

## 运行与任务收尾

- 每次测试、诊断运行或任务结束后（包括失败、中止和异常退出），关闭本次使用完的仿真环境、控制器、ROS 启动进程、RViz、采集器及相关辅助进程。清理前确认进程来源，只关闭属于本次任务或已确认遗留的项目进程，不影响无关会话。
- 清理后核对相关进程是否已退出；不能仅凭窗口关闭或已发送中断就声称清理完成。如有残留或无法确认，明确记录并说明。
- 每次运行结束及任务提交时，用简洁、通俗的中文说明：做了什么、结果如何、还有什么问题，以及环境是否已关闭。未完成或未验证的内容如实说明。
