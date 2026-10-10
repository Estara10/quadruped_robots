# 开题报告—仓库能力差距矩阵（FROZEN SNAPSHOT）

> 本文件只保留第一轮审计快照，不再作为活跃治理文件。当前事实与缺口统一维护在 `thesis_project/CURRENT_STATE.md`。

| 开题要求 | 当前仓库事实 | 状态 | 关键差距 | 关闭证据/阶段 |
|---|---|---|---|---|
| 原始单阈值 A | `paper_faithful_switch` 与离线真值表存在 | PARTIAL | 尚缺正式配置合同、运行消费与基线行为分析 | R1 |
| 仅滞回 B | 无独立模式 | MISSING | 需与 A 仅相差退出阈值 | R3 |
| 仅保持 C | 无独立模式 | MISSING | 需与 A 仅相差 Recovery 保持 | R3 |
| 滞回+保持 D | `stabilized_switch` 为 D 类候选 | PARTIAL | 相等性、硬编码退出阈值、步数时钟均未对齐 | R1/R3 |
| RA—模式—动作时序 | runtime frame 有 RA、policy state、动作和单调时间 | PARTIAL | 缺统一事件定义和动作生效派生验证 | R1/R2 |
| 切换稳定性指标 | 可从帧派生部分指标 | PARTIAL | 短时窗口、事件 reducer、完整性检查未冻结 | R2/R4 |
| 安全/任务终局 | 碰撞链部分存在 | PARTIAL | 到达、跌倒、超时与碰撞权威未统一闭合 | R2 |
| 稀疏/中等/密集场景 | 有 PPT 场景候选和历史场景 | PARTIAL | 尚非正式、可达性和碰撞集合未验收 | R4 |
| 四组公平配对 | 有场景/种子/配对工具资产 | PARTIAL | 尚无提案四组预注册和正式批次 | R4/R5 |
| 参数分析 | 当前只有 legacy 参数 | MISSING | 需预实验选择阈值差、保持时长和窗口 | R4/R6 |
| 论文统计与图表 | 有记录器/HUD/旧报告工具 | PARTIAL | 缺开题指标汇总、配对分析和失败分析 | R5/R6 |
| 低速实机验证 | 部分硬件、安全和部署代码/记录存在 | PARTIAL/NO-GO | 站立复验、真实感知、仿真准入和安全检查未完成 | R7/R8 |
| 可追溯证据包 | 模型 manifest、schema、evidence 较丰富 | PARTIAL | 训练来源有 UNKNOWN；新实验包规则需落实 | R5/R9 |

## 距离毕业设计目标的判断

基础工程并非从零开始，但“研究问题闭环”尚处于前期：已有组件可支持快速进入机制研究，核心论文证据仍为 PLANNED。当前最大距离不是再搭建一般性平台，而是把 A/B/C/D 因果变量、权威终局和三档场景组合成可重复的正式实验。

## 复用与降级

### 直接复用

- 三模型与 artifact manifest；
- Go2+MuJoCo+ROS 2 控制链；
- 约 50 Hz 的策略运行基线；
- runtime frame、记录器、进程清理和哈希工具；
- 碰撞 instrumentation、场景解析和部分配对运行工具；
- 实机映射、sport mode 释放、PASSIVE/硬急停等安全资产。

### 需改造后复用

- `RASwitchingLogic.hpp` 与配置解析；
- `stabilized_switch`；
- PPT/历史障碍场景；
- P1 formal schema 与 metrics；
- common-start/配对工具；
- ZED、RealSense 和 ray-pred 脚本。

### 降级为历史参考

- 旧 Phase/Gate/P1 任务序列；
- 旧论文参考性能 Gate；
- 已失败或丢失原始产物的 exact replay pair；
- 以“基础设施完成度”代替论文研究结果的验收方式。
