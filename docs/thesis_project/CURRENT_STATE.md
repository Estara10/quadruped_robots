# 当前状态

> 更新日期：2026-10-10
>
> 当前阶段：S2 正常终局收尾改进；目前没有已授权的下一项 Active Task

## 最近任务状态（2026-10-10）

### 今日收尾（2026-10-10）

S2-06 五个固定随机静态场景和 S2-07 到达后稳定站立均已完成并有条件通过。用户手动复测场景04，run `4a45d1d1f6cb494c809445208b3ebeee` 为 ARRIVED，终点距目标约0.4999 m，站立保持完成，正常路径未请求 PASSIVE，终端报告相关进程已退出；分析器已生成本地结果表。该记录位于 `logs/manual_scene_runs/scene_random_04_standing/`，属于手动诊断，不并入正式实验。今天停止工作，目前没有已授权的 Active Task；下次建议先规划 S3-01 B组滞回，尚未创建或启动。

**S2-07 最新范围与结果：** Director 有条件通过 [S2-07：到达后受控停止与稳定站立](tasks/S2-07.md)，状态为 **COMPLETE**。通过范围：到达后停止运动、确认站稳并连续保持约 5 仿真秒；正常路径不自动趴下或卸力；运动指标排除收尾；先关闭 MuJoCo，再关闭 controller 等进程。平地与随机场景04两次诊断均 ARRIVED，分别有 5.002 s、5.004 s 仿真时间的站立保持记录。ARRIVED 周期封口运动指标，保持观察单独保存。random_04 有真实 MuJoCo framebuffer 画面；flat 首次截图因相对路径未落盘，路径已修正，但没有超出运行次数补跑。详见 [S2-07 执行报告](evidence/S2-07/S2-07_EXECUTION_REPORT.md)。

**S2-07 保留限制：** 两次 run 的完整运动区间碰撞覆盖不足以声称无碰撞；全采集跌倒历史不能外推为保持窗口事件。随机04记录有 17 个无效时钟帧、1 个周期缺口。两次收尾时速度命令均为零，非零命令平滑 ramp 由生产 helper 离线检查，而非运行观察。健康路径没有确认 PASSIVE，因为它没有请求 PASSIVE；真实动作和 MuJoCo 命令消费仍为 UNKNOWN。停止发布器以退出码 1 结束，但所有项目进程均已退出，无强制结束。旧下降/卸力失败记录原样保留。本任务不授权实机运行，实机仍为 NO-GO。

此前趴下/卸力目标的实现和失败属于历史范围，已经被稳定站立目标替代；详见执行报告历史章节。停止发布器退出码 1、截图缺失和所有其余限制及历史失败均保留。S2-07 已有条件通过并完成；目前没有已授权的下一项 Active Task。

Director 已对 [S2-05](tasks/S2-05.md) 有条件通过并标记 **COMPLETE**。规则为：足端轻微接触单独记录、不算终止碰撞；足端满足 Fxy > 2*abs(Fz)+10 N 时记为足端撞击但仍不终止；非足端机器人—障碍接触继续终止。v5 生产快照、Python reader/记录/分析链已定向核验；唯一一次 sparse 诊断记录到 RL 足端撞击和同时发生的 RL_calf 非足端终止碰撞。独立轻微足端接触、成功率、其他场景表现和 MuJoCo 命令消费等限制继续保留。

Director 已对 [S2-04：单次仿真选择场景](tasks/S2-04.md) 有条件通过并标记 **COMPLETE**。`--scene` 默认仍为 `scene_obstacle.xml`，支持原固定、平地、稀疏、中等和密集候选；场景身份贯穿 MuJoCo、射线、碰撞、记录和分析，五场景模型与障碍集合由生产路径探针核对。平地实际运行 VALID / ARRIVED，地面接触未误判为障碍碰撞；稀疏实际运行 VALID / COLLISION_TERMINATION。S2-03 及时 PASSIVE 和运动终局/收尾分离流程保留。目前没有已授权的下一项 Active Task。

Director 已对 [S2-03](tasks/S2-03.md) 有条件通过并标记 **COMPLETE**。通过范围包括两次手动运行的接触对象核对、RL 前准备停止发布通道、hard-stop 后 PASSIVE 转换确认、运动终局与收尾观察分离，以及一次 VALID / ARRIVED 诊断支持修正后的基础闭环；不代表 A 组稳定避障性能通过。历史“没有 Active Task”的表述只指相应任务关闭时的状态。

## FACT：可以直接复用的基础

- 仓库已有 Unitree Go2、MuJoCo、ROS 2 控制器和仿真启动资产；
- 已有 Agile、RA Value、Recovery 部署模型；
- 控制器中已有三模型调用和 Agile/Recovery 切换路径；
- `paper_faithful_switch` 可作为 A 组逻辑候选；
- `stabilized_switch` 同时包含滞回和保持，只能作为 D 类参考，不能直接视为正式 D；
- 已有包含 RA、policy state、位姿、速度、ray2d 和动作的运行时记录基础；
- 已有碰撞检测、场景、配对运行和指标工具可选择性复用；
- 已有部分实机通信、映射、PASSIVE/硬急停和 sport mode 释放基础。

## 尚未完成

- A/B/C/D 运行时机制尚未全部按已通过合同实现；B、C、D 独立组及按秒保持的运行时实现留待相应阶段；
- S2–S8 的基线、滞回、保持、组合、场景/参数预实验、正式配对仿真、统计分析及论文结果尚未完成；
- S9–S10 实机感知、安全准备与代表性低速验证尚未完成；在安全准备通过前维持 NO-GO。

## 未决事项与保留限制

- Agile/Recovery 动作关节顺序现有多源证据支持 FL/FR/RL/RR（每腿 hip/thigh/calf），但证据性质不同：训练者来源确认、留存训练代码、Go2 资产与顺序捕获、actor/export 链和 checkpoint→部署权重关联共同支持；它不是由模型内嵌元数据或历史不可变 run 配置单独证明。详见下方“动作顺序证据更新”。
- `recovery_hold_steps=30` 是旧步数设置，实际持续时间未由它或 125 Hz/8 ms 注释证实。合同已定义仿真以 MuJoCo 仿真秒、实机以单调运行秒计 H，并只在有效策略决策周期判断到期/切换；按秒的 hold 尚未接入切换器或运行验证。
- 正常连续运行中策略周期与仿真时钟已关联，并在具体捕获中测得约 50 Hz；这不是所有运行的频率保证。暂停后策略恢复及同名共享时钟重连仍未运行验证；暂停尝试中安全停止后没有策略帧恢复。暂停/中止后的 run 不直接续接。
- Recovery 选择和控制接口命令写出已有周期/来源证据；MuJoCo 是否消费具体命令、以及真实动作生效的精确时刻仍 UNKNOWN。
- S1-05 的无碰撞/未跌倒结论只适用于该次覆盖完整的运动窗口及其诊断规则，不能扩展到全采集范围、其他场景、正式跌倒参数或长期稳定性。S1-04 两次完整运行未出现 Recovery，不能判断切换效果；其碰撞覆盖不完整，不能据此声称无碰撞。
- S1-05 诊断的 XAUTHORITY 参数路径记录有误，实际 renderer 未保存；S1-04 原完整运行也未保存 renderer，不能声称跨运行 renderer 一致。
- 记录开销与周期抖动的因果关系、三档候选场景的可达性/区分度，以及实机 ray2d 来源/标定/时延、硬件层站立与 RL 状态仍待适用阶段核实。

## S1 当前状态

S1-01 至 S1-06 均已由 Director 审阅并标记 COMPLETE；S1-02、S1-03、S1-04、S1-05、S1-06 为有条件通过。各状态只表示对应任务限定范围被接受，不表示所有 S1 验收项均无保留，也不自动等于 S1 整体完成。执行期间的 INCOMPLETE、启动失败和 INVALID 记录保留原状；后续修正和新证据不改写历史 run 分类。

其余历史 UNKNOWN、旧 P1 验收缺口、非必要 hash/manifest 和工作树历史修改均不是当前 Blocker。

## 动作顺序证据更新（2026-10-10）

**来源确认（用户提供）：**Agile 与 Recovery 的训练均在 `ABS_fuwuqi/ABS` 完成，由 ABS 的 Go1 配置适配为 Go2；训练代码和资产仍保留。此为用户对训练来源的确认，不是机器从历史训练命令或不可变运行配置中恢复出的事实。

**仓库直接核验：**

- `ABS_fuwuqi/ABS/training/legged_gym/legged_gym/envs/__init__.py:66-67` 注册 `go2_pos_rough` 和 `go2_rec_rough`；当前保存的两份配置分别在 `go2_pos_config.py:78`、`go2_rec_config.py:81` 指向 `resources/robots/go2/urdf/go2.urdf`。这核实当前留存代码/资产路径，不冒充历史不可变 run 配置。
- 该 Go2 URDF 含 Isaac Gym 捕获中的同一组 12 个可动关节名。需区分文本顺序与运行顺序：URDF 中 `<joint>` 声明顺序是 FR、FL、RR、RL；留存的 Isaac Gym `get_asset_dof_names` 运行顺序捕获 `archive/artifacts/p1_01_contract.json` 记为 FL、FR、RL、RR（每腿 hip、thigh、calf），足接触捕获也是 FL、FR、RL、RR。因此，当前核验到的是具名关节集合对应、且运行顺序捕获为 FL-first；不是说 URDF 文本声明顺序与运行捕获相同。训练基类直接取 `gym.get_asset_dof_names(robot_asset)`（`legged_robot.py:737-766`）。
- `legged_robot.py:80-92,377-399` 将 actor 输出裁剪后按同索引传入按 DOF 数组逐元素计算的控制路径；没有额外关节置换。位置、速度、default/bias 与动作目标均使用同一环境 DOF 索引序列。
- Agile 导出路径（`scripts/play.py:71-79`、`utils/helpers.py:184-194`）复制并 TorchScript 导出 actor；Recovery 导出路径（`export_rec_policy.py:41-89`）按 actor 层键加载权重并 TorchScript 保存。源码中没有重排输出通道的步骤。
- `archive/artifacts/manifest.yaml` 和其引用的 `docs/evidence/P1-01/provenance_recovery.json` 将 Agile `go2_pos_rough/.../model_4000.pt`、Recovery `go2_rec_rough/.../model_15000.pt` 与各自导出/部署权重关联；记录的 actor 张量等价和导出/部署字节等价支持两个部署模型对应这些 checkpoint。
- 控制器当前配置显式将 policy 顺序设为 `ros1_fl_fr_rl_rr`；观测侧对关节位置、速度、前一动作和足接触使用已有重排函数，策略输出再映射到 controller order（`AbsObservationContract.cpp`、`StateRL.cpp`、`StateRLRec.cpp`）。这是部署映射证据，不代替训练顺序来源证据。

**当前判断：**最有证据支持的训练/actor 通道顺序为 **FL、FR、RL、RR；每腿 hip、thigh、calf**。这比“完全 UNKNOWN”更有依据，但仍是用户来源确认与保留源码、资产/运行捕获及权重关联的合并判断。没有历史不可变训练配置，也没有模型内置的具名关节元数据；现有运行行走事实不作为顺序证据。原 S1-02 报告和归档 manifest 中关于缺少历史绑定/metadata 的限制仍成立，不改写其历史记录。

**S1-06 定向核验结果（Director 有条件通过）：**唯一标记值通过生产 `AbsObservationContract` helpers 核对 controller→policy 的 q/default/bias/dq/prior-action/contact 排列、Agile/Recovery observation 字段布局及逆变换；Director 接受的范围还包括对 Agile、inline Recovery 和独立 Recovery 状态类实际动作转换源码的核对。动作对应表和证据见 [S1-06 执行报告](evidence/S1-06/S1-06_EXECUTION_REPORT.md)。私有 actor 输出方法未由本轮 harness 直接调用。没有加载或推理模型，也没有运行机器人。实际 ROS loaned contact interface 次序和 SDK foot-force 索引的具名语义仍 UNKNOWN。

## 非阻塞技术债

- 历史训练环境和部分模型来源信息不完整；
- 旧 P1-10 exact replay/common-start 未闭环；
- 历史 evidence 存在产物丢失记录；
- 部分脚本和文档仍使用旧 Phase/P1 术语；
- 演示场景尚未正式化。

只有当其中某项实际影响当前技术任务或论文实验解释时，才升级处理。

## 当前任务安排

| S1 验收项 | 已证实 | 保留限制 / 未完成 |
|---|---|---|
| 切换合同 | [SWITCH_CONTRACT.md](SWITCH_CONTRACT.md) 的 A/B/C/D 转移、阈值边界、配置互斥、H 时钟/周期语义及事件计时定义已通过 S1-01 最终审阅；RA 越大代表风险越高。 | 合同不等于运行时实现：B/C/D 正式机制和按秒 H 尚未接入；正式参数留待预实验/冻结。 |
| 关键模型接口 | [S1-02 报告](evidence/S1-02/S1-02_EXECUTION_REPORT.md)整理了 Agile `61→12`、RA `19→1`、Recovery `49→12` 的输入构造及控制器处理；训练者来源确认、ABS 代码、Go2 资产与 Isaac Gym 顺序捕获、导出路径和权重关联共同支持 actor 顺序 FL/FR/RL/RR（每腿 hip/thigh/calf）。S1-06 有条件通过：生产 observation/order helpers 的唯一标记值检查通过，且 Director 接受了各策略状态类实际动作转换源码核对。 | 此顺序不是由历史不可变 run 配置或模型内嵌关节元数据单独证明；旧报告/manifest 中的 UNKNOWN 是当时证据结论，仍须保留为来源绑定限制。RA learned score 的物理单位仍 UNKNOWN。私有动作转换成员未直接调用。S1-06 当时未观察仿真 loaned contact order；S2-01 已核实 MuJoCo 仿真传感器→SDK slot→ROS interface 并实测 loaned 名称顺序。实机 Go2 foot-force 物理语义仍 UNKNOWN；见 [S1-06 报告](evidence/S1-06/S1-06_EXECUTION_REPORT.md) 与 [S2-01 报告](evidence/S2-01/S2-01_EXECUTION_REPORT.md)。正常行走不作为证据。 |
| 频率与时间语义 | S1-02 连续捕获约 50.14 Hz；S1-03/S1-05 也有各自约 50 Hz 的运行捕获。S1-03 将仿真时钟样本配对到正常连续策略周期；合同规定仿真 H 用 MuJoCo 仿真秒、实机 H 用单调运行秒，并只在有效策略决策周期检查到期和转换。 | 频率仅对具体 run 成立，不是全局保证，也不能排除记录开销。旧 `recovery_hold_steps=30` 与 125 Hz/8 ms 注释不是实际 H/频率证据；H 尚未接入切换器。暂停恢复和同名时钟重连未经运行验证。 |
| Agile/RA/Recovery 基础链 | S1-02/03 非正式运行按周期关联 RA、模式和动作来源，观察到 Agile↔Recovery 转换；Recovery 命令与来源周期关联到控制接口写出。 | 写出不证明 MuJoCo 消费或动作生效。S1-04 两次完整运行和 S1-05 一次正常诊断均未出现 Recovery，不能判断切换效果；legacy 候选不等于正式 A 组结果。 |
| 记录、指标、安全事件及清理 | S1-04 两次完整基础诊断具有终止帧、terminal、指标、时间线和收尾证据；S1-05 定向核验了 v3/3864、v4/5760 的版本/长度读取规则及真实 v4 C++→Python→记录→分析链，并完成一次正常诊断。 | S1-04 碰撞覆盖不完整；S1-05 无碰撞/未跌倒结论限于该次覆盖完整的运动窗口和诊断规则，不适用于全采集区间。正式跌倒参数、其他场景和长期稳定性未验证。S1-05 的 XAUTHORITY 记录路径错误且实际 renderer 未保存。历史 INVALID/失败记录保留。 |

### S2-01 实际进展（2026-10-10）

动作顺序由训练者来源确认、保留训练代码、资产及 Isaac Gym 顺序捕获、导出路径和权重关联共同支持为 FL/FR/RL/RR（每腿 hip/thigh/calf），但不来自模型内嵌元数据或历史不可变配置的单独证明。S1-06 的 helper 离线检查和状态类源码核对获 Director 有条件通过；私有动作成员未直接调用。S2-01 进一步核实 MuJoCo 仿真接触链，并在一次有效诊断的固定站立准备阶段直接观察到 controller loaned 顺序 `foot_force/FR, FL, RR, RL`；这不外推为实机 SDK 物理语义验证。

[S2-01：A 组单阈值实现与功能基线核验](tasks/S2-01.md) 已由 Director 有条件通过并标记 **COMPLETE**。通过范围为 A 配置生产解析/切换函数检查、init-only 配置消费证据、既有 legacy 候选真实 RA 行为及仿真足接触来源核对；这不是正式实验授权，也不表示新配置已有运动周期证据。

S2-01 的三类证据分别限定为：旧 `paper_faithful_switch` 诊断中的真实 RA 行为；新 A 配置生产解析器与 helper 的 27 项离线核验；init-only 原始日志独立重验确认新配置被 controller 初始化消费。新 A 配置没有策略帧或运动周期验证。init-only supervisor 的 `INVALID_RUNTIME_SOURCE` 分类与原始文件保留不变。旧诊断的障碍碰撞结果保留，5 个 STALE 时钟帧造成的驻留统计删失仍成立。策略停止确认、MuJoCo 命令消费、实机接口语义等限制继续保留。后续 B 组核验应同时确认统一 A/B 配置的运行消费，并保持两组除滞回外的控制链一致。详见 [S2-01 执行报告](evidence/S2-01/S2-01_EXECUTION_REPORT.md)。

各任务最终结论与证据：[S1-01 任务/合同](tasks/S1-01.md)、[S1-02 报告](evidence/S1-02/S1-02_EXECUTION_REPORT.md)、[S1-03 报告](evidence/S1-03/S1-03_EXECUTION_REPORT.md)、[S1-04 报告及验收对照](evidence/S1-04/S1-04_EXECUTION_REPORT.md)、[S1-05 报告及跨语言核验](evidence/S1-05/S1-05_EXECUTION_REPORT.md)、[S1-06 任务/部署通道报告](tasks/S1-06.md)。

S2-02 已由 Director 有条件通过并标记 **COMPLETE**：MuJoCo 只读实时面板及实际 LIVE 截图通过审阅，显示逻辑不改变控制器。限制仍包括目标距离 N/A、Body XY speed 不等于倾斜时的世界水平速度、非 LIVE 策略状态仅做离线核验、本次 PASSIVE 后策略停止确认未获证，以及 MuJoCo 命令消费未知。面板通过不代表 A 组避障性能通过。以上“尚无 Active Task / 建议规划完整目标任务”的文字只描述 S2-02 审阅时状态；后续 S2-03 已被授权并完成，当前 S2-04 状态见下文。S3-01 暂缓。S2 A 基线有条件通过不代表整个 S2 阶段完成；历史审阅结论和限制保持原状。

### S2-03 实际进展（2026-10-10）

两次手动运行的终止接触均为机器人与 geom 2 静态障碍 box：第一次 geom 25（FL 小腿碰撞体），第二次 geom 27（FL 足端碰撞体），Director 结论为未发现地面误分类。历史记录不含接触距离/力，不能判断穿透或冲击强度。原始记录保留不变。

S2-03 已将终局 PASSIVE 通道改为运行前建立并确认订阅者的持久发布器；控制器在 hard-stop 分支完成 PASSIVE 状态变更后写出带单调时间的确认日志。运动记录在终止快照封口，收尾观察单独保存并排除运动指标。移除了将固定 250 ms 等待写成实际策略输出持续时间的字段语义。

唯一一次补充短诊断为 VALID / ARRIVED，按 0.5 m 目标半径终止，距离 0.4996 m；终局后约 2.972 ms 开始发布 PASSIVE，controller 在发布开始后约 0.414 ms 记录转换完成。运动终局与收尾观察已分开，收尾窗口没有采到新策略帧。运行含 4 个无效时钟帧，统计保留各自有效范围。历史接触距离/冲击强度、MuJoCo 命令消费和物理运动停止时刻仍 UNKNOWN；有限窗口没有策略帧不外推至窗口外。单次到达不代表 A 组稳定避障性能。完整结果见 [S2-03 执行报告](evidence/S2-03/S2-03_EXECUTION_REPORT.md)。该任务关闭时没有已授权的下一项任务；随后用户授权 S2-04，当前进展见下文。

### S2-02 实际进展（2026-10-10）

MuJoCo 现从只读 `/mujoco_rt_frame` 面板显示 Body XY speed（不称世界水平速度）、Agile/Recovery/FAULTED、未进入 RL/策略停止状态、帧内 RA 与实际 entry/exit 阈值、仿真时间及数据状态和年龄。未进入、停止、故障以及 MISSING、INVALID、STALE 时不显示旧策略值；读取在渲染线程执行，每帧重新打开共享对象，不缓存旧 session。仿真时间仅在帧时钟有效时显示。当前帧没有权威目标坐标，因此目标距离明确显示 N/A。`scripts/abs_live_hud.py` 保持不变。

定向离线检查与 MuJoCo 目标构建通过。Run `4fff5053b6ed486a8a0a8dc74ff30c88` 按 A 配置启动，记录 159 个 LIVE 策略帧和真实速度/RA/模式/阈值/仿真时间，最终因运动期障碍碰撞终止；controller、ROS launch 和 MuJoCo 清理均确认退出。运行没有保存 renderer。

首次截图路径当时失败的历史事实已保留；本轮改用 MuJoCo framebuffer 读取并取得真实 LIVE 面板图像。Body XY speed 不是世界水平速度；未进入 RL、策略停止、FAULTED 及数据过期通过离线函数核验，未分别保存这些状态的真实窗口画面。目标距离仍显示 N/A；本轮 PASSIVE 后策略停止确认字段为 false。详情见 [S2-02 执行报告](evidence/S2-02/S2-02_EXECUTION_REPORT.md)。

### S2-04 实际进展（2026-10-10）

S2-04 现支持原固定场景、平地和三种 PPT 候选场景。共用目录中记录每个场景的文件、ID、障碍数量、XML/asset closure 与编译模型指纹；C++ ray writer 与碰撞 authority 使用同一障碍集合函数。ground contact 不进入障碍集合，flat 空集合是有效状态。reader、逐 run record meta、run_context、结果表和时间线都绑定/显示本次 scene；不含新字段的旧记录继续按历史场景兼容读取。

本次平地 run `da57865e43b7425886294b11f6ec5ff6` VALID / ARRIVED，距离目标 `0.4965 m`，策略频率约 `50.0039 Hz`，motion window 的碰撞/跌倒结论均为 false 且覆盖完整；实际有 foot-floor contact，但 robot-obstacle contacts 为 0。稀疏 run `1900daaf23ca480b8e9401e360989668` VALID / COLLISION_TERMINATION，记录到机器人 geom 42 与 `ppt_obstacle_03` 接触，策略频率约 `49.9959 Hz`。稀疏场景本次有 4 个无效时钟帧；平地有 1 个，原始记录和分析边界均保留。两次运行均确认 controller PASSIVE 转换及 `shutdown_complete=true`，相关进程事后检查为空。

Director 对 S2-04 有条件通过，认可 `--scene` 默认值和五种场景支持、跨 MuJoCo/射线/碰撞/记录/分析的身份绑定、五场景生产路径探针、平地 VALID / ARRIVED 与稀疏 VALID / COLLISION_TERMINATION，以及平地 ground contact 未误判为障碍碰撞；S2-03 的及时 PASSIVE 和收尾分离流程保留。

限制：原固定、中等和密集场景尚未实际运行策略；单次平地到达和稀疏碰撞不代表成功率或难度排序；S2-04 稀疏运行只能确认 robot geom 42 与 ppt_obstacle_03 接触，不能确定具体肢体或冲击强度。修改任何受支持场景 XML 或资源后须同步更新目录身份。MuJoCo 命令消费等既有 UNKNOWN 保留。历史失败、INVALID 和原始运行数据不改写。详情见 [S2-04 执行报告](evidence/S2-04/S2-04_EXECUTION_REPORT.md) 和[单次场景使用说明](evidence/S2-04/SINGLE_RUN_GUIDE.md)。S2-04 审阅后当时没有 Active Task；随后 S2-05 获得授权并成为当前唯一 Active Task。

### S2-05 实际进展（2026-10-10）

新 v5 collision authority 把足端 contact/impact 与非足端终止碰撞分开，严格按四个命名球体识别足端，力按 contact frame 转世界坐标并逐物理步汇总已知外力。v5 使用 14,472-byte snapshot，扩展起点 5,760；reader 拒绝版本/长度错配并保留 v3/v4 旧语义。记录器与分析器输出 navigation terminal、nonfoot failure、foot contact/impact、arrival flags 与 ABS 参考碰撞；时间线区分三类事件。定向 C++/跨语言检查和必要构建通过。

唯一一次 sparse 诊断 bf84dbe4946b43b29a30200dc24d6ec5 为 VALID / COLLISION_TERMINATION，目标距离 2.1471 m，149 个策略帧，49.9956 Hz。实际记录到 RL 足端与 ppt_obstacle_03 接触和撞击：Fxy=216.8427 N、abs(Fz)=35.0042 N、阈值=80.0084 N；同时无名 robot geom 46 / body 12 RL_calf 与同一障碍接触，非足端事件触发终止。因此导航终局是碰撞，foot contact=1、foot impact=1、ARRIVED flags 为 false、ABS reference collision=true。足端 episode 覆盖五个物理步，完整采集约 0.010 s；event/snapshot 结束于 29.638 s，terminal policy 周期为 29.636 s，运动窗口 duration 保持 UNKNOWN。controller PASSIVE 确认、完整清理事实与任务进程核对均通过。时间线、结果表、原始记录与跨语言探针详见 [S2-05 执行报告](evidence/S2-05/S2-05_EXECUTION_REPORT.md)。

S2-05 已获 Director 有条件通过并标记 COMPLETE。唯一 run 不代表切换性能或总体避障能力。实际没有单独出现轻微足端障碍接触；v3 历史二进制未找到，仅做精确布局兼容 fixture 和历史 v4 JSON 记录接受核验。运动边界 duration、MuJoCo 命令消费、其他场景和正式性能均有适用限制，详见报告。任务进程已关闭。后续用户授权的 S2-06 已成为当前唯一 Active Task。

### S2-06 最终状态（Director 有条件通过，2026-10-10）

S2-06 的五套固定布局已生成 `scene_random_01.xml` 至 `scene_random_05.xml`，接入十项共用场景目录、`--scene`、MuJoCo、ray2d、碰撞 authority、逐周期记录、分析和时间线。MuJoCo 生产加载与十场景 authority 探针通过。五条浅蓝保留通道仅作几何预检，没有输入控制器；控制仍由目标、状态观测和 11 条射线驱动。

五个场景各取得一次有效 A 组诊断，五次都在 0.5 m 到达半径内 ARRIVED。场景 01、02、03、05 未发生 Recovery。场景 04 按运动周期记录共发生 136 次模式变化（Agile→Recovery 68 次、Recovery→Agile 68 次）；其中 128 次具有有效仿真时间，8 次时间无效。时钟无效不会抹去已记录的模式变化，但不能用于精确时间指标。五场景统计均区分起点至终点经过时间、有效连续间隔累计时间，以及只覆盖有效连续片段的路径长度和平均速度；场景 04 分别为 11.760 s 和 11.108 s。原 117 次短反向切换仅描述分析器既有 0.5 s 指标范围，不是全部切换总数。所有 run 均有 PASSIVE 确认和清理事实，原始记录、指标与时间线已保存。第 4 场景箱体按冻结 JSON 原坐标超出声明 y 下界约 0.0564 m，未移动；跌倒判断仍为 UNKNOWN。

Director 已对 [S2-06](tasks/S2-06.md) 有条件通过并标记 **COMPLETE**。通过范围限于场景接入、单次诊断及记录/指标链可复核。每场景一次非正式观察不能计算成功率、比较场景难度或宣称 A 组性能/切换效果通过。详见 [S2-06 执行报告](evidence/S2-06/S2-06_EXECUTION_REPORT.md) 和[单次对照表](evidence/S2-06/diagnostic_runs_20261010/scene_comparison.md)。目前没有已授权的下一项 Active Task。

## 历史收尾记录（2026-10-07，保留）

以下内容只描述 2026-10-07 当时状态。后续 S1-03 已将仿真时钟接入并在正常连续运行中记录配对；这不改变当时记录，也不表示暂停恢复或时钟重连已验证。

- S1-01 切换合同已通过审阅；S1-02 基础接口、策略周期与仿真时钟核对已获 Director 有条件通过。
- S1-02 的非正式诊断记录了 1,848 个连续策略帧、约 50.14 Hz 的整体策略频率，以及 Agile 与 Recovery 的进入和退出；这些记录不计入正式实验。
- 截至当日，Agile 和 Recovery 部署模型的真实动作关节顺序仍记为 UNKNOWN（2026-10-10 的多源证据更新见上文；正式实验前仍须处理来源绑定限制）。MuJoCo `sim_time` 当时尚未接入控制器，合同中的时间保持尚未实现和核验。
- 今日工作到此停止；没有已授权的下一项 Active Task，未启动 S1-03。
