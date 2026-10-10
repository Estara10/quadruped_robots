# ABS-Go2 Thesis Proposal Context（LEGACY SNAPSHOT）

> 本文件是 2026-09-08 的旧写作快照，并非 2026-10-06 纳入治理的已答辩开题报告。不得用它覆盖 `thesis_project/OVERVIEW.md`、`thesis_project/CURRENT_STATE.md` 或 `thesis_project/ROADMAP.md`。

> 这是给没有仓库访问权限的独立写作对话使用的项目事实快照，不是动态状态文件、Roadmap、Acceptance 或新的项目规划。快照内容以仓库 Source of Truth 为准，Snapshot date 为 2026-09-08。后续项目变化须由用户提供更新，不能从本文件自行推断。

## 事实等级说明

- **FACT**：由当前仓库代码、配置、Source of Truth 或已保存 evidence 支持。
- **PROJECT DECISION**：项目已经明确采用的工程或治理决定。
- **PLANNED**：未来计划、候选工作或尚未完成的实验。
- **UNKNOWN**：现有证据不足，不能补全。
- **PAPER REFERENCE**：来自 ABS 原论文，不是本项目 Go2 结果。

代码、文件名、模型加载、离线夹具或一次运行本身，不能自动证明训练来源、算法有效性、正式 benchmark 或 Phase Acceptance。

## 1. Snapshot Information

- Snapshot date：**2026-09-08**。
- Branch：`feat/ray-pred-source-switch`。
- HEAD commit：`990e8572466588b8874366a95dce5abcbdf65b04`。当前工作树存在未提交变更，因此该 commit 只是版本定位信息，不代表本快照的完整内容。
- Current project phase：**FACT — Phase 1 — MuJoCo Simulation Validation**。
- Current Phase Gate：**FACT — Phase 1 NOT ACCEPTED**；Phase 2 为 **NO-GO**；Phase 3 为 **NOT STARTED**。
- Current P1-10 state：**FACT — IMPLEMENTED / AWAITING INDEPENDENT REVIEW**。
- 本文件是写作上下文快照，不替代 `docs/CURRENT_STATE.md` 这一动态状态 Source of Truth。

## 2. Graduation Project Definition

### 2.1 研究对象

**FACT**：毕业设计研究对象是 ABS（*Agile But Safe*）四足机器人运动体系在 Unitree Go2 与 MuJoCo 平台上的复现、平台适配、工程化验证和可审计实验系统建设。ABS 的核心组成是 Agile Policy、Reach-Avoid（RA）Value、Recovery Policy 以及由 RA 驱动的 Agile/Recovery switching。

### 2.2 项目总目标

**FACT / PROJECT GOAL**：在 Unitree Go2 上复现 ABS 的核心结构，并建立一个正确、稳定、可观测、可测量、可复现的 MuJoCo 实验系统，随后在满足安全门槛的条件下进行低速 Sim-to-Real 验证。

项目优先级为：**Correctness > Stability > Observability > Safety > Performance > Paper Speed**。达到论文最高速度不是早期目标。

### 2.3 三个阶段

#### Phase 1 — MuJoCo 系统仿真

**FACT / PROJECT GOAL**：在 Unitree Go2 + MuJoCo + ROS 2 控制栈中建立可信的系统级行为证据。最终应覆盖：

- 稳定、快速的平地运动；
- 在正式障碍场景中向固定目标运动并记录到达、碰撞、跌倒、超时和耗时；
- Agile、RA、Recovery 的真实运行使用，而不只是代码存在或模型成功加载；
- 结构化的 Agile/Recovery switching 事件、原因、RA 状态、Recovery 时长和结果；
- runtime HUD，显示速度、当前策略、RA、耗时、目标/轨迹及必要的碰撞/安全状态；
- post-run statistics，包括到达、耗时、平均/峰值速度、碰撞、跌倒、超时、RA、Recovery、轨迹和必要安全统计；
- 运行身份、来源、配置、场景、模型和结果的 correctness/provenance；
- 固定场景和设置下的合理重复性与 auditability。

**PROJECT DECISION**：平地 replay 只是 P1-10 的内部基础设施/重复性子门槛。它不能单独构成 P1-10 final acceptance，也不能单独授权 P1-11 或 P1-12。P1-10 final behavioral validation 必须包含接受的多障碍 scenario suite 和真实行为证据。

#### Phase 2 — Unitree Go2 实机验证

**FACT / PROJECT GOAL**：Phase 1 Gate 通过后，进行保守、分阶段的 Unitree Go2 实机测试，包括真实传感器和控制、独立 Safety Supervisor、低速调试、Agile、Recovery、静态障碍和分阶段多障碍试验。

**PROJECT DECISION**：在 Phase 1 Acceptance 和 Phase 2 安全门通过前，真实机器人 ABS/RL 为 **NO-GO**。实机结果必须与论文、MuJoCo 结果分开报告。

#### Phase 3 — 拓展与研究

**PLANNED**：在前述阶段完成后开展性能和速度改进、新算法/策略/感知方法、更多场景、论文方法对比，以及最终毕业设计实验包整理。Phase 3 不是当前 Phase 1 的前置证明。

## 3. Research Motivation

以下只整理项目事实基础，不代替文献综述，也不构成未经检索的“国内外研究现状”。

- **FACT**：四足机器人需要在动态、复杂和含障碍的环境中同时保持运动速度、稳定性和安全性；高速运动使碰撞风险、状态切换和恢复动作之间的关系更重要。
- **PAPER REFERENCE**：ABS 原论文将 Agile locomotion 与 Reach-Avoid Value、Recovery Policy 结合，通过风险/可达性价值驱动策略切换，目标是在速度和安全之间取得平衡。
- **FACT**：本项目需要把论文中的 Go1/原始仿真与部署体系迁移到 Unitree Go2、MuJoCo 和 ROS 2 控制栈，因此必须验证 observation、action、关节顺序、射线、RA、Recovery、切换和运行证据链，而不能只复制文件或观察画面。
- **PROJECT DECISION**：MuJoCo 是毕业设计第一阶段的平台，因为它提供可控制的 Go2 模型、物理状态和可审计的仿真运行环境；Phase 1 先解决仿真系统正确性与行为证据，再进入有安全门的实机验证。
- **UNKNOWN**：未经重新检索论文，不能在开题材料中声称具体的国内外研究结论、研究空白或某方法的普遍优越性。

## 4. Original ABS Paper

本节的数值和实验结果均标为 **PAPER REFERENCE**，不可写成当前 Go2 结果。

### 4.1 Agile Policy

**PAPER REFERENCE**：Agile Policy 的 observation 为 61 维：

| 组成 | 维度 |
|---|---:|
| 足端接触 | 4 |
| base angular velocity | 3 |
| base-frame projected gravity | 3 |
| goal command：relative x、relative y、heading | 3 |
| time left | 1 |
| joint position relative to default | 12 |
| joint velocity | 12 |
| previous action | 12 |
| log ray distances | 11 |
| 合计 | **61** |

**PAPER REFERENCE**：Agile Policy 输出 12 维关节位置目标。论文部署 PD 形式为：

```text
tau = Kp * (q_target - q) - Kd * q_dot
```

论文实机增益为 `Kp=30`、`Kd=0.65`。

### 4.2 Reach-Avoid Value

**PAPER REFERENCE**：RA Value 的 observation 为 19 维：base linear velocity 3、base angular velocity 3、goal relative x/y 2、log ray distances 11。关节位置和速度不属于 RA 输入。

论文从 reach signal `l(s)` 与 avoid/collision signal `g(s)` 构造 reach-avoid objective，并给出 discounted Bellman 形式（论文 Eq.4–5）。记录的训练细节包括 `gamma=0.999999`、最近十个时间步的 collision target softening，以及基于 `tanh(log(d_goal / sigma_tight))` 的 reach signal（Eq.18）。当前项目对 exact dataset、checkpoint lineage 和全部训练元数据的证据不足，见第 8 节 UNKNOWN。

### 4.3 Recovery Policy 与 Safe-Twist Optimization

**PAPER REFERENCE**：Recovery Policy observation 为 49 维：足端接触 4、角速度 3、投影重力 3、安全 twist 3、关节位置 12、关节速度 12、previous action 12。Recovery 没有直接的 exteroceptive ray 输入，障碍信息通过优化后的 safe twist 影响 Recovery。

**PAPER REFERENCE**：Recovery 输出 12 维关节位置目标并使用 PD。论文 Recovery training twist 范围为 `vx ±1.5 m/s`、`vy ±0.3 m/s`、`wz ±3 rad/s`；这些不是当前 Go2 实机速度限制。

**PAPER REFERENCE**：Eq.21 是在 RA Value 低于切换阈值的约束下，以短时域目标位置偏差为目标选择 twist；论文描述从当前 twist 初始化，并在五次以内完成梯度优化。Eq.22 对 `delta_t=0.05 s` 的平面位移近似包含 yaw coupling：

```text
delta_x = vx * delta_t - 0.5 * vy * wz * delta_t^2
delta_y = vy * delta_t + 0.5 * vx * wz * delta_t^2
```

### 4.4 Switching、频率和射线

**PAPER REFERENCE**：论文切换概念为 RA Value 高于/达到阈值时进入 Recovery，低于阈值时回到 Agile；记录的阈值为 `-0.05`。论文部署频率为：PD 200 Hz、Agile 50 Hz、Recovery 50 Hz、RA 50 Hz、Ray-Pred 40 Hz。

**PAPER REFERENCE**：论文感知使用 160×90 深度图和 ResNet18-based predictor 产生射线表示。当前项目不能把自己的 MuJoCo 几何射线机械地宣称为论文感知系统的完全等价物。

### 4.5 论文实验指标

**PAPER REFERENCE**：论文仿真报告使用 3 个 policy seeds、每个 seed 10,000 个随机评估 episode：

| 指标 | 论文报告值 |
|---|---:|
| ABS Success | 79.1 ± 4.4% |
| ABS Collision | 5.7 ± 2.9% |
| ABS Timeout | 15.2 ± 2.1% |
| ABS Peak speed | 3.48 ± 0.06 m/s |
| ABS Average speed | 2.08 ± 0.01 m/s |
| Agile-only Success | 77.3% |
| Agile-only Collision | 21.7% |
| Agile-only Timeout | 1.0% |
| Agile-only Peak speed | 3.55 m/s |
| Agile-only Average speed | 2.39 m/s |

论文实机试验使用 Go1、Orin NX 和 ZED Mini，不能写成 Go2 结果。

## 5. Current System Architecture

### 5.1 面向论文写作的逻辑数据流

```text
MuJoCo / sensor-like state
        │
        ├─ body state, contacts, goal, timer, joint state, rays
        │
        ▼
  observation construction
        │
        ├───────────────┐
        ▼               ▼
   Agile Policy      RA Value
        │               │
        │       RA threshold / switching logic
        │               ▼
        │         Recovery twist optimizer
        │               │
        └───────────────┴──────► Agile / Recovery selection
                                      │
                                      ▼
                         action / joint-position target
                                      │
                                      ▼
                          PD target → MuJoCo Go2
```

**FACT / SOURCE-VERIFIED**：当前 ROS 2 `StateRL` 负责 observation、Agile/RA 推理、Recovery twist、切换和 action/target 转换；部署 TorchScript 模型通过 libtorch/相关模型接口加载。控制器和 MuJoCo 的关节执行顺序为 `FR, FL, RR, RL`；当前声明的 policy 候选顺序为 `FL, FR, RL, RR`，通过显式 remap 连接二者。该 policy artifact 历史顺序的独立来源证明仍为条件性/UNKNOWN，见第 8 节。

### 5.2 运行和证据组件

- **FACT**：ROS 2 Humble + `ros2_control` 提供控制器管理和 `rl_quadruped_controller`。
- **FACT**：MuJoCo 提供 Go2 model、`mjModel`/`mjData` 和物理推进。
- **FACT**：Unitree SDK2/DDS bridge 为模拟控制栈提供相关接口。
- **FACT**：`/mujoco_rt_frame` 是 StateRL runtime frame 的共享内存来源；它承载 policy/RL step、姿态、动作、RA、策略状态等结构化运行数据。`/mujoco_sim_clock` 提供版本化 sim-clock；ray 使用版本化 stamped frame。共享内存读取须通过版本、序列、时间新鲜度和有限性检查。
- **FACT**：HUD 是运行时可视化/操作观测工具；它不能替代落盘的正式 telemetry。
- **FACT**：`RunRecordRecorder`/`scripts/run_record.py` 和 P1-09 formal runtime binding 将真实 runtime frame、structured events、process facts 和 terminal 事实写入 JSONL/JSON 证据。缺失权威来源时保留 `UNKNOWN` 或使记录 `INVALID`，不使用日志文本推断。
- **FACT**：post-run statistics 由保存的 runtime record、timing、process facts 和场景/身份绑定计算；不能用 HUD 或视觉印象替代。

### 5.3 已知约束

**PROJECT DECISION**：`paper-faithful`、`stabilized` 和 `agile-only` 必须作为不同变体分别配置、记录和报告。当前 P1-10 capture-eligible 变体是 `stabilized`，使用 `stabilized_switch`；paper-faithful 和 agile-only 的当前 P1-10 capture binding 为 UNSUPPORTED。

## 6. Platform Differences From Paper

| 维度 | Original ABS paper | Current graduation project |
|---|---|---|
| Robot | Unitree Go1 | Unitree Go2 |
| Simulator/training context | Isaac Gym/PhysX 体系 | MuJoCo deployment validation；Isaac Gym 是训练参考环境 |
| Middleware | ROS1 deployment | ROS 2 Humble + `ros2_control` + Unitree SDK2/DDS |
| Perception | ZED Mini、深度图和 paper Ray-Pred pipeline | MuJoCo geometric rays；另有 ZED/D435i prototype，但有效运行时 source mode 仍可能 UNKNOWN |
| Control/model artifacts | 论文的 Go1 训练/部署工件 | 当前 Go2 配置、ROS 2 controller、MuJoCo model 和已部署 TorchScript artifacts |
| 工程目标 | 高速且安全的 agile locomotion | correctness、stability、observability、safety、provenance、reproducibility 优先 |

这些差异意味着：

- **FACT / PROJECT DECISION**：论文数值只能作为参考基线，不能直接作为 Go2+MuJoCo 的结果或承诺。
- **FACT**：Go2 joint/controller order、MuJoCo model、ROS 2 interface 和观测 assembly 需要独立绑定；论文平台的 order、物理和感知语义不能直接移植。
- **UNKNOWN**：当前部署 policy 是否具有完整、可复原的历史训练 config、seed、commit、导出命令和 dataset lineage；这些缺失不能从文件名或模型输出补全。
- **PROJECT DECISION**：跨平台结果必须分别报告；差异应解释为 robot、dynamics、perception、policy artifact 或 runtime contract 的可能来源，而不能机械追求所有绝对数值相等。

## 7. Current Verified Engineering Progress

下表是当前 Source of Truth 的状态摘要，不展开历史补丁流水账。

| Task | 当前状态 | 已证实范围与边界 |
|---|---|---|
| P1-01 | **ACCEPTED WITH KNOWN ISSUES** | 当前部署 mapping、61/19/49 local contract、artifact byte/tensor lineage 和 simulation operational chain 有证据；历史训练 metadata、精确 export invocation、real Go2 foot-force slot order 等保留为 deferred/UNKNOWN。 |
| P1-02 | **ACCEPTED / COMPLETED（offline contract）** | formal schema、writer/validator、comparison gate 和 fixture-level contract 已接受；authoritative runtime event、telemetry、seed/provenance 的完整连接不等同于 benchmark。 |
| P1-03 | **ACCEPTED / COMPLETED（offline trace）** | paper-to-code formula/parameter trace 已完成；不是 paper equivalence 或 runtime proof。 |
| P1-04 | **ACCEPTED WITH KNOWN ISSUES** | Agile 61-D observation 的 source/field parity 和 real-training-oracle local test 已覆盖；goal/timer 等工程变体及 downstream lineage 边界仍明确记录。 |
| P1-05 | **ACCEPT WITH KNOWN ISSUES** | RA 19-D 输入、标签结构和 switching 关系已离线追踪；RA dataset/checkpoint exact provenance 与 RA↔Agile 历史绑定仍 UNKNOWN/OPERATOR_DECLARED。 |
| P1-06 | **ACCEPT WITH KNOWN ISSUES** | Recovery Eq.21/Eq.22 三方比较已完成；当前 deployment 的 Eq.22 yaw-coupling、gradient clip、iteration 等差异已记录，未自动修复。 |
| P1-07 | **ACCEPT WITH KNOWN ISSUES** | `paper_faithful_switch` 与默认 `stabilized_switch` 已分离并离线测试；运行时 switching 统计、完整 paper-faithful ABS 仍 UNKNOWN。 |
| P1-08 | **ACCEPT WITH KNOWN ISSUES** | Go2 MuJoCo baseline、model/config/artifact closure、sim-clock 和一次真实 flat capture 已冻结；Recovery 未观察到，orphan/callback cadence 等仍有边界。 |
| P1-09 | **ACCEPTED；runtime-record subchain ACCEPT WITH KNOWN ISSUES** | 真实 MuJoCo + StateRL → HUD → runtime JSONL → two-phase finalize → process facts → P1-02 validator chain 已演示；代表性 formal record 可诚实判为 INVALID，不能写成成功 episode。 |
| P1-10 | **IMPLEMENTED / AWAITING INDEPENDENT REVIEW** | flat suite、saved-record comparator、障碍候选地图离线形式化、Stage-B collision authority 和 Stage-A common-start gate/anchor 已实现或准备；共同起跑尚无新的 runtime evidence，障碍行为尚未验证。 |
| P1-11 | **NOT STARTED / UNAUTHORIZED** | 需要接受的 P1-10 多障碍 suite 和行为证据。 |
| P1-12 | **NOT STARTED / UNAUTHORIZED** | 需要 P1-11 pilot 和预注册设置。 |
| P1-13 | **NOT STARTED / UNAUTHORIZED** | 需要 P1-12 evaluation 后作 Phase 1 Go/No-Go。 |

**FACT**：Phase 1 仍为 **NOT ACCEPTED**，P1-11/P1-12/P1-13 不会自动启动。

## 8. Current Behavioral Evidence

这是写作时最重要的边界：基础设施证据不等于机器人性能证据。

### 8.1 已有真实运行支持的事实

- **FACT / RUNTIME EVIDENCE**：P1-08 accepted baseline 包含一次真实的 Go2 MuJoCo/controller flat capture；记录了固定 25 s 窗口、0.002 s physics timestep、约 500 Hz physics、约 49.97 Hz Policy/RA frame、真实 runtime frame 和 process facts。P1-08 的 scope 是 model/timing/dynamics baseline，不是 ABS benchmark 或障碍 effectiveness。
- **FACT / RUNTIME EVIDENCE**：P1-09 真实运行链证明了 MuJoCo + StateRL 产生 runtime frame，HUD 可读，recorder 可保存 JSONL，two-phase finalize 可结合真实 process facts；P1-02 validator 对缺少权威 outcome source 的代表性记录给出 `INVALID`，这是 fail-closed 证据，不是失败后伪造成功。
- **FACT / RUNTIME EVIDENCE**：runtime record 中可以保存策略状态、RA、动作、姿态、时间、sequence 和 process facts 等结构化字段；这些字段支持运行可审计性，但不自动证明 Recovery 真实贡献、障碍到达或成功率。
- **FACT / OFFLINE EVIDENCE**：flat `flat_goal_forward` 和 `flat_goal_lateral` scenario 的 scene、closure、initial qpos、goal、variant 和 baseline bindings 已 hash-bound。`flat_goal_forward` 使用固定 world goal `[7.0, 0.0]` m。

### 8.2 尚未证明的事实

- **UNKNOWN / NOT YET PROVEN**：正式障碍场景中的到达、碰撞、跌倒、超时和 elapsed-time 结果。
- **UNKNOWN / NOT YET PROVEN**：五张历史障碍地图的正式 suite 结果；`obstacle_test1` 只有离线 collision authority/preparation，尚无 runtime validation；`obstacle_test2`–`obstacle_test5` 仍 UNSUPPORTED。
- **UNKNOWN / NOT YET PROVEN**：Recovery 在障碍行为中的真实使用和贡献、完整 Agile/RA/Recovery switching 统计、collision-to-switch/terminal 的因果关系。
- **UNKNOWN / NOT YET PROVEN**：formal multi-seed statistics、Success/collision/fall/timeout rate、论文指标的 Go2 重现或性能等价。
- **UNKNOWN / NOT YET PROVEN**：common-start gate 和 producer-side RL-enter anchor 的真实运行效果。Stage-A 2026-09-07 pair 的历史 exact replay comparison 为 FAIL，且其 raw archive 后来不完整；该 pair 不可 retry，不可补写，也不是当前成功证据。
- **UNKNOWN / NOT YET PROVEN**：Sim-to-Real performance、安全通过和真实机器人 ABS/RL 结果。Phase 2 仍 NO-GO。

**PROJECT DECISION**：任何开题或论文文字都应把“runtime frame/recorder/HUD 已有证据”与“机器人已经在障碍中成功运行”严格分开。

## 9. Current Core Technical Problems

以下只保留会影响毕业设计理解的主要问题。

1. **平台与模型差异**：Go1 与 Go2 的模型、执行器、接触和控制 mapping 不同；原始 Isaac Gym/PhysX 训练参考与 MuJoCo deployment validation 也不同。
2. **policy artifact provenance**：Agile、RA、Recovery 的部署工件 hash/shape/运行绑定已有证据，但完整历史训练 seed、config、commit、export invocation、dataset lineage 不完整；不能根据文件名推断训练来源。
3. **observation semantics**：当前 61/19/49 维 layout 已有 local parity，但 goal shaping 是工程变体；training noise/randomized bias 与 deployment nominal bias 的语义差异仍存在。
4. **RA semantics**：RA 形状、输入 layout、输出范围和当前 switching 使用路径已追踪；精确 RA dataset/训练来源及其与 Agile 的历史绑定仍 UNKNOWN 或 OPERATOR_DECLARED。
5. **Recovery parity**：当前 deployment 与论文/恢复 testbed 在 Eq.22 yaw coupling、gradient clipping、iteration 等方面存在已记录差异；这些不是本快照中被默认修复的事项。
6. **switching behavior**：paper-faithful 和 stabilized 语义已分离，但 MuJoCo/ROS 运行中的真实切换频率、Recovery 进入/退出和行为效果仍需证据。
7. **obstacle navigation authority**：障碍 geom 存在不等于发生 collision；正式碰撞需要场景绑定的 MuJoCo contact authority。goal/fall/controller-timeout 的完整权威定义仍有 UNKNOWN 边界。
8. **repeatability**：在 common-start 机制的 runtime evidence、预声明的合理重复性指标和障碍场景重复性闭合前，不能把一次 valid record 写成可重复性能结论。
9. **Sim-to-Real**：真实 Go2 的传感器、foot-force slot semantics、延迟、风险和安全停机必须通过 Phase 2 专门门槛，不可由仿真结果代替。

## 10. Phase 1 Remaining Work

遵循 **Behavior-First** 原则：支持性 infrastructure 只在缺失它会使 CORE 行为结果不可信时继续投入。

当前路线为：

```text
可信平地行为 / common-start review
        ↓
一次新的、受审查和授权的 flat A/B replay
        ↓
预先声明并有技术依据的合理重复性判断
        ↓
obstacle_test1 runtime validation
        ↓
obstacle_test2–5 按同一合同逐步验证
        ↓
正式多障碍 scenario suite
        ↓
P1-11 pilot
        ↓
P1-12 multi-seed evaluation
        ↓
P1-13 Phase 1 Go/No-Go
```

- **FACT**：当前 common-start gate/producer-side anchor 是离线实现，尚无 runtime evidence。
- **PLANNED**：在 Independent Reviewer 审核和 Director 单独授权后，建立新的 fresh pair；旧的 2026-09-07 pair 不得 retry。
- **PLANNED**：先完成 `obstacle_test1` 的场景绑定、authority 和一次真实行为运行，再扩展其余障碍地图。
- **PLANNED**：只有当障碍定义、collision/terminal authority、runtime record、repeatability 和 independent review 均闭合后，才形成正式 scenario suite。
- **PROJECT DECISION**：不得因为 flat replay 或 infrastructure tests 通过而提前授权 P1-11/P1-12。

## 11. Phase 2 Plan

以下都是 **PLANNED**，不是已完成事实：

- real Unitree Go2 的安全、低速、分阶段 commissioning；
- hazard analysis、latency budget、speed limits 和独立 Safety Supervisor；
- simulation/dry-run/HIL、PASSIVE、lifted joints、standing、low-speed locomotion 等逐级门槛；
- real perception adapter 与深度相机 pipeline；
- 在安全门通过后分别验证 Agile、Recovery、静态障碍和 staged multi-obstacle trials；
- 对 Sim-to-Real 差异进行分解，单独报告仿真和实机。

Phase 2 只能在 Phase 1 Acceptance 后开始；当前为 NO-GO。

## 12. Phase 3 Plan

以下属于 **PLANNED / FUTURE EXTENSION**：

- 在正确性、稳定性、可观测性和安全门通过后的性能/速度改进；
- 新算法、策略、感知方法或额外场景；
- Paper vs Go2 MuJoCo vs Go2 real 的统一指标比较；
- 对残余差异归因于 robot、dynamics、perception 或 policy；
- 根据证据决定是否需要 domain randomization、system identification、fine-tuning 或 retraining；
- 冻结毕业设计图表、报告和可复现实验包。

这些扩展不能在当前写作中被描述为已经发生，也不能反向成为 Phase 1 已关闭的事实。

## 13. Candidate Thesis Research Questions

下列只是 **CANDIDATE**，不是最终论文结论或已确认创新点：

1. ABS 从原始 Go1/Isaac Gym/ROS1 体系迁移到 Go2 + MuJoCo + ROS 2 后，哪些 observation、action、控制顺序和运行合同需要适配？
2. 在明确区分 paper-faithful 与 stabilized variant 的前提下，Agile–RA–Recovery switching 体系能否在 Go2 MuJoCo 中形成可审计、可重复的运行链？
3. 在正式障碍场景中，Full ABS 与 Agile-only 的行为差异如何通过到达、碰撞、跌倒、超时、速度、RA 和 Recovery 事件衡量？
4. Go1/原始仿真环境与 Go2/MuJoCo 之间的结果差异，能否由机器人模型、动力学、感知射线语义、policy provenance 或 runtime contract 解释？
5. 如何用 hash-bound scene/model/config、结构化 runtime record 和预注册统计规则建立适合毕业设计的可复现实验流程？

新写作对话不得把这些候选问题改写成“本项目已经证明的创新”。

## 14. Candidate Thesis Contents

这是适合本科毕业设计开题和论文的候选结构，属于 **PLANNED**：

1. 绪论：研究背景、问题定义、研究对象、目标和论文结构；
2. ABS 理论与相关技术：Agile Policy、RA Value、Recovery Policy、switching、PD 控制和射线表示；
3. 系统平台与迁移问题：Go1/原始仿真与 Go2/MuJoCo/ROS 2 的差异，模型、I/O、关节顺序和 policy contract；
4. ABS-Go2 仿真系统设计：MuJoCo、ROS 2 controller、libtorch、observation/action、shared-memory telemetry、HUD 和 recorder；
5. 正确性、来源和实验方法：scenario manifest、model/config hash、runtime identity、validity、UNKNOWN 和 reproducibility contract；
6. 平地与障碍场景实验：flat foundation、`obstacle_test1` 及后续 scenario 的行为证据、策略切换、collision/terminal 和 repeatability；
7. 结果分析：Full ABS、Agile-only、paper-faithful/stabilized 的分组结果，平台差异和误差来源；
8. Go2 实机与 Sim-to-Real：仅在安全门通过后写入真实结果；
9. 总结与展望：结论边界、局限性和 Phase 3 扩展。

最终章节安排必须服从学校学院模板；学院模板出现后，其格式要求优先。

## 15. Candidate Technical Route

以下是毕业设计层面的 **PLANNED** 技术路线：

```text
ABS 原论文阅读与理论分析
        ↓
现有训练/参考/部署代码追踪
        ↓
Go2 模型、关节顺序、observation/action 合同核对
        ↓
MuJoCo + ROS 2 + libtorch 系统适配
        ↓
paper-faithful / stabilized 变体分离
        ↓
correctness、provenance、runtime observability 验证
        ↓
flat 行为与合理重复性验证
        ↓
obstacle_test1 行为实验
        ↓
obstacle_test2–5 与正式 scenario suite
        ↓
统计评估与失败归因
        ↓
安全门控的 Go2 Sim-to-Real
        ↓
扩展实验、对比和论文写作
```

## 16. Planned Milestones

ROADMAP 使用 W1–W12 的项目级阶段，而不是学校正式校历。下表是适合计划书的提炼；具体日期 **UNKNOWN**，不得自行编造。

| 毕业设计阶段 | 对应计划/范围 | 状态 |
|---|---|---|
| 文献调研与理论分析 | ABS paper、P1-03 paper-to-code trace、公式和参数 | P1-03 已完成离线 trace；论文综述仍需独立检索 |
| 系统搭建与策略部署 | Go2/MuJoCo、ROS 2、policy I/O、P1-01～P1-08 foundations | 大部分基础工程已具备，保留已记录 UNKNOWN |
| 运行观测与数据链 | P1-09 runtime frame、HUD、recorder、process facts、formal validity | 基础链已有真实运行证据，完整 outcome authority 仍有边界 |
| MuJoCo 平地验证 | P1-10 Stage A flat replay/common-start | 旧 pair 不可 retry；新的 common-start runtime evidence 尚未产生 |
| 第一张障碍地图实验 | P1-10 Stage B，优先 `obstacle_test1` | planned；尚未 runtime validated |
| 其余障碍实验与统计 | P1-10 Stage C，`obstacle_test2`–`obstacle_test5`、formal suite | planned；当前候选仍 UNSUPPORTED |
| Pilot 与 multi-seed evaluation | P1-11、P1-12 | planned/unauthorized |
| Phase 1 结论 | P1-13 Go/No-Go | planned/未启动 |
| 实机安全验证 | Phase 2 | planned；当前 NO-GO |
| 扩展、平台比较与毕业材料 | Phase 3 | planned；当前未启动 |

## 17. Expected Results

以下全部是 **EXPECTED / PLANNED**，不是 Already Achieved：

- 在 MuJoCo 中得到可解释、稳定的 Go2 ABS 运行；
- 在正式障碍场景中记录固定目标的到达、碰撞、跌倒、超时和耗时；
- 观察并量化 Agile、RA、Recovery 的真实使用和 switching；
- 获得 runtime HUD、结构化 runtime record、process facts 和 post-run statistics；
- 在预注册的场景、种子和指标规则下进行合理重复性与 multi-seed 统计；
- 在安全门通过后进行低速 Go2 实机验证；
- 解释论文平台、Go2 MuJoCo 和 Go2 实机之间的差异；
- 形成可追溯的毕业设计实验、图表和论文材料。

论文中的高速数字不能直接作为上述承诺；Go2 数值必须来自真实且有效的 Go2 运行证据。

## 18. Known Risks and Limitations

- **平台差异**：Go1 与 Go2 的模型、执行器、接触和控制 mapping 不同。
- **训练 provenance**：当前部署模型的完整历史 config、seed、command、commit、export 和 dataset metadata 不全。
- **仿真差异**：MuJoCo 与原训练/仿真环境的 dynamics、solver、contact 和 numerical behavior 不同。
- **感知差异**：论文 Ray-Pred、当前 MuJoCo geometric rays 和未来真实深度相机不是自动等价；有效 source、frame、freshness 和 invalid semantics 必须记录。
- **observation/RA 语义**：goal shaping、bias/noise、RA dataset 和历史 RA↔Agile binding 仍存在明确边界。
- **Recovery 效果**：Recovery 的真实触发率、约束可行性和障碍行为贡献还没有被本项目结果证明。
- **terminal/outcome**：goal、fall、controller-timeout 和全 episode collision-free coverage 的权威来源仍有 UNKNOWN。
- **重复性与样本规模**：一次 capture 或 valid record 不等于 benchmark；正式统计需要预注册的 scenario、seed、样本量和区间。
- **实机安全**：real Go2 需要独立安全机制、分阶段限速和紧急停止；不能用仿真性能替代安全验证。
- **时间与实验规模**：本科毕业设计的 scenario 数量、seed 数量和实机试验规模受时间与设备条件限制；没有学校正式日程时不应写死日期。

## 19. Terms and Numbers Reference

| 项目 | 数值/定义 | 来源等级 |
|---|---|---|
| Agile observation | 61-D | PAPER / PROJECT |
| Agile action | 12-D joint-position targets | PAPER / PROJECT |
| RA observation | 19-D | PAPER / PROJECT |
| Recovery observation | 49-D | PROJECT contract；paper-side完整部署证据需以原论文核对 |
| Recovery action | 12-D joint-position targets | PAPER / PROJECT |
| Paper PD | `Kp=30`, `Kd=0.65` | PAPER REFERENCE |
| Go2 ABS PD | `Kp=30`, `Kd=0.65` | PROJECT config |
| Action scale | `0.25` | PROJECT config / training reference |
| Observation clip | `±100` | PROJECT config |
| ABS action output clip | `±4` | PROJECT config |
| Paper Agile / Recovery / RA rate | 50 / 50 / 50 Hz | PAPER REFERENCE |
| Paper Ray-Pred rate | 40 Hz | PAPER REFERENCE |
| Project controller update rate | 200 Hz for `rl_quadruped_controller`; manager 1000 Hz | PROJECT config |
| Project policy decimation | 4 | PROJECT config |
| Project observed physics timestep | `0.002 s`, approximately 500 Hz | PROJECT, P1-08 accepted runtime evidence |
| Project observed Policy/RA frame | approximately 49.97 Hz | PROJECT, P1-08 accepted runtime evidence |
| Direct controller callback cadence | UNKNOWN; 5 ms is only derived from policy/decimation | UNKNOWN |
| Ray count | 11 | PAPER / PROJECT |
| Ray angles | `−π/4` to `+π/4`, spacing `π/20` | PAPER REFERENCE; current equivalence UNKNOWN |
| Ray range | `0.1–6.0 m` | PAPER / PROJECT declaration |
| Ray representation | logarithmic distance; repository reference `log2` | PAPER/PROJECT reference; exact current source equivalence may be UNKNOWN |
| Training ray origin | x `−0.05 m`, y `0` | PAPER/PROJECT reference |
| Paper switching threshold | `−0.05` | PAPER REFERENCE |
| Stabilized enter | `RA > −0.05` | PROJECT DECISION / config |
| Stabilized exit | `RA < −0.08` after 30 policy steps | PROJECT DECISION / config |
| Stabilized hold | 30 policy steps | PROJECT config |
| Recovery twist bounds | `vx ±1.5`, `vy ±0.3`, `wz ±3.0` | PAPER / PROJECT |
| Recovery twist `tau` | `0.05 s` | PROJECT config; paper relation described in Eq.22 |
| Recovery optimizer `lambda` / `lr` / `eps` | `10.0` / `0.5` / `0.05` | PROJECT config; paper numeric equivalence partly UNKNOWN |
| Recovery iterations | deployment 3; recovered testbed 10 | PROJECT evidence; not paper numeric MATCH |
| Current flat goal | `[7.0, 0.0]` world m | PROJECT scenario binding |
| Formal capture window | `25.0 s` | PROJECT experiment binding |
| Initial state | `scene_default / mj_makeData:qpos0`; no keyframe reset | PROJECT scenario/runtime binding |
| Initial qpos hash | `a604dd11dc57ea655bf6d746dcf068a91e80a0a1eddc73d20c1a3800468f59d8` | PROJECT evidence |
| Flat scene root hash | `9ce83b3e61c722a523d0359536cee803f17610f95d2275fc32e96801ec3c1908` | PROJECT evidence |
| Flat model closure hash | `8d9218de0dc02978fc0ef4ba1c790fa3b968fbdbfdb945e14522436a2574ea07` | PROJECT evidence |
| P1-10 suite hash | `eb81d60742864fe9c870e957ba3ab601e80da3e64bc48a42c26f849570f3152d` | PROJECT evidence |
| P1-08 canonical baseline identity | `59dd13fed5ebd026ec519f2659643237502be8e4d8df5174a65b7d35ceb4f7e0` | PROJECT evidence |
| P1-08 identity document hash | `6c3563c25d45cc275db6b083f9f0fc0cc2067b48bc8f4a93dcace9f6d42817ea` | PROJECT evidence |
| Paper ABS nominal success/collision/timeout | `79.1 ± 4.4% / 5.7 ± 2.9% / 15.2 ± 2.1%` | PAPER REFERENCE |

Where a row has both PAPER and PROJECT, the values are not evidence of platform equivalence. Where the source column says UNKNOWN, the writing dialogue must preserve UNKNOWN.

## 20. Writing Guardrails For Future ChatGPT

新的写作 ChatGPT 必须遵守：

- 不把 `PLANNED` 写成已完成的 `FACT`；
- 不把论文结果写成 Go2、MuJoCo 或实机结果；
- 不虚构创新点、技术突破或算法有效性；
- 不虚构实验数据、成功率、碰撞率、Recovery 贡献或 benchmark；
- 不虚构国内外研究现状；国内外研究现状必须重新检索论文并保留出处；
- 不根据文件名、模型存在、代码路径、一次运行或视觉行为推断训练来源和 runtime 语义；
- 遇到证据不足时写 `UNKNOWN`，遇到明确不支持的场景写 `UNSUPPORTED`；
- 当前项目状态以本快照和用户后续明确更新为准；需要动态状态时回到 `docs/CURRENT_STATE.md` 核验；
- 学院模板出现后，以学院模板作为最高写作格式要求；
- 开题阶段使用“拟、计划、将、预期”等准确措辞；
- 当前完成的工程成果可以作为可行性依据，但不要把毕业设计提前写成已经完成；
- 平地基础设施通过不等于 P1-10 final acceptance，也不等于可以自动进入 P1-11/P1-12；
- 任何正式结论都必须说明平台、policy artifact、变体、scenario、seed、指标规则和证据等级。

## 21. Important Source Files

以下索引用于以后回到 Codex/仓库核验。路径均相对于仓库根目录。

| Topic | Repository path |
|---|---|
| 项目目标与三阶段 | `docs/PROJECT_GOAL.md` |
| 动态项目状态 | `docs/CURRENT_STATE.md` |
| 项目级路线 | `docs/ROADMAP.md` |
| 状态模型与角色边界 | `docs/PROJECT_STATE_MODEL.md` |
| ABS 论文定义与纸面数值 | `docs/ABS_PAPER_NOTES.md` |
| 指标定义与初始统计门槛 | `docs/METRICS.md` |
| 正式实验协议 | `docs/EXPERIMENT_PROTOCOL.md` |
| Go2 policy I/O 与顺序合同 | `docs/POLICY_IO_CONTRACT.md` |
| 架构决策 DEC-001～DEC-012 | `docs/DECISIONS.md` |
| P1-01～P1-10 计划与最终状态 | `docs/exec-plans/P1-01.md` … `docs/exec-plans/P1-10.md` |
| 已部署模型/工件 manifest | `artifacts/manifest.yaml` |
| P1-01 contract 与模型输出 | `artifacts/p1_01_contract.json` |
| P1-01～P1-04 observation contract | `quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/AbsObservationContract.cpp` 和对应 `include/` |
| Agile/RA/Recovery runtime | `quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRL.cpp` |
| Recovery runtime state | `quadruped_ros2_control_humble/controllers/rl_quadruped_controller/src/FSM/StateRLRec.cpp` |
| Switching helper | `quadruped_ros2_control_humble/controllers/rl_quadruped_controller/include/rl_quadruped_controller/FSM/RASwitchingLogic.hpp` |
| ABS controller config | `quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/abs/config.yaml` |
| Recovery config | `quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/rec/config.yaml` |
| ROS 2 rates/order/plugin config | `quadruped_ros2_control_humble/descriptions/unitree/go2_description/config/robot_control.yaml` |
| MuJoCo simulator config | `unitree_mujoco/simulate/config.yaml` |
| MuJoCo scene | `unitree_mujoco/unitree_robots/go2/scene_flat.xml` |
| P1-10 flat/candidate scenarios | `scenarios/p1_10/` |
| P1-10 suite manifest | `scenarios/p1_10/scenario_suite_manifest.json` |
| P1-08 accepted baseline manifest | `docs/evidence/P1-08/P1-08_baseline_manifest.json` |
| P1-08 accepted canonical identity | `docs/evidence/P1-08/P1-08_simulation_baseline_identity.json` |
| P1-02 formal writer/validator | `scripts/formal_experiment_contract.py` |
| Runtime record/reducer | `scripts/run_record.py`、`scripts/formal_runtime_binding.py` |
| P1-10 capture harness | `scripts/p1_08_baseline_capture.py` |
| P1-10 saved-record comparator | `scripts/p1_10_saved_record_compare.py`、`scripts/p1_10_stage_a_saved_record_compare.py` |
| P1-10 historical five-map evidence | `docs/evidence/P1-10/historical_five_map_formalization_20260903.md`、`docs/evidence/P1-10/historical_five_map_inventory_20260903.json` |
| P1-10 Stage-A common-start evidence | `docs/evidence/P1-10/stage_a_common_start_implementation_20260907.md`、`docs/evidence/P1-10/stage_a_common_start_execution_manifest_20260907.json` |
| P1-10 2026-09-07 pair incident | `docs/evidence/P1-10/stage_a_pair_20260907_evidence_integrity_incident.md`、`docs/evidence/P1-10/stage_a_pair_20260907_evidence_integrity_inventory.json` |
| P1-02/P1-09/P1-10 formal evidence | `docs/evidence/P1-02/`、`docs/evidence/P1-09/`、`docs/evidence/P1-10/` |
| Original project rules | `AGENTS.md` |

### Snapshot closing boundary

**FACT**：截至本快照，项目已经具备较完整的 ABS-Go2 仿真工程基础和部分真实 runtime 证据，但尚未完成正式障碍行为验证、正式多场景统计、Phase 1 Acceptance、实机验证或论文级平台等价性证明。

**PLANNED**：后续写作应把上述工程基础转化为可核查的毕业设计方法、实验设计、预期成果和风险说明；不要把尚未运行或尚未接受的内容写成既成结果。
