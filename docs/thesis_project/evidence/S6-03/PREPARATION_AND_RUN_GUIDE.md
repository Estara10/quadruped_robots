# S6-03 运行准备规则与使用说明

本文件说明工具入口，不授权新的参数比较或额外场景运行。2026-10-12 最新获准补证已分别运行 A、D 一次；两次都观察到准备跌倒恢复/关闭、controller `[STAND-READY]` 门槛和物理稳定门槛通过，进入 RL 后保存完整周期0帧及同周期初态 companion。两次任务均以合法的 `FALL_TERMINATION` 结束，均未到达或触发 Recovery，不能用来比较参数。先前的 A 准备失败、INVALID 和 D 未启动记录继续作为历史保留。本任务授权的 A/D 场景运行额度及 prepare-only 观察额度均已用尽；进一步场景运行需另行授权。

## 一次运行的准备次序

1. 在默认桌面环境确认 MuJoCo GLFW/OpenGL 可以启动；只在批准范围内临时使用已有 DISPLAY/XAUTHORITY。不要更改桌面授权或全局设置。
2. 批次目录先保存 `batch_plan.json`。每组使用独立实体 overlay 和独立 run_id/output 子目录；程序检查 overlay、目标、组、阈值、场景身份、模型文件及运行来源。
3. recorder 在 command 3 前创建、flush 并回读 meta 的 run_id/capture_id；MuJoCo 的 LIVE 物理时钟/安全 authority 和 controller 的唯一 writer/source binding 未确认时不得进入策略阶段。
4. 通过固定 `PASSIVE → FIXED DOWN → FIXED STAND` 准备后，须先从实际 controller 日志读到 `[STAND-READY] command3_gate=passed`，证明内部插值计数 `percent_≥1.5` 且 command 3 的既有转换门槛已满足；随后再等待连续 0.50 仿真秒稳定观察，准备墙钟最多 20 秒。要求 PhysicsLoop LIVE、物理覆盖有效、未知接触为 0、支撑足不少于 2、base height≥0.25 m、|roll|/|pitch|≤0.30 rad、无活动跌倒候选/确认和机器人—障碍接触。仅有 FIXEDSTAND 状态或 `[STAND-SYMM]` 不代表插值完成。若启动期跌倒 episode 已确认，生产检查继续观察而不清除历史；episode 必须由有效正常姿态关闭，之后重新满足完整稳定窗口才能继续准备。持续异常、无效来源、未见 `[STAND-READY]` 或超时仍拒绝 RL。
5. 在 RL 前用 ROS Humble `/joint_states` 采 12 个具名关节 position/velocity；样本必须唯一、有限，max |dq|≤0.50 rad/s。样本是异步 ROS 采样，要保存 monotonic 时间、与 RL 请求/首 active 周期时间差，不能声称同周期。
6. RL 前再取 LIVE physics safety boundary。最终 entry gate 同时检查 recorder、standing、joint sample/speed 和 safety boundary；任何条件缺失，记 PREPARATION_FAILURE，不发送 RL 请求。
7. 首个 active policy frame 的同周期 companion 和 runtime JSONL 必须精确匹配 session/cycle/monotonic_ns。任何错过、身份切换、周期缺口或无效时钟都按实际情况保留。terminal 周期完整保存后封口；正常清理先关 MuJoCo，再停 controller/ROS、publisher 与采集辅助程序。

准备期出现异常时保留其 authority 历史和 observations。已确认跌倒且 episode 尚未恢复关闭、当前姿态/支撑不满足条件、无效 authority、缺具名关节样本、过高关节速度或 20 s 墙钟超时均不得进入 RL。恢复关闭后可继续统一准备检查，但异常历史仍记录在准备事件中；不得延迟任务窗口隐藏异常，也不得连续重启直到出现好初态。

仅在之后明确授权准备观察时，`scripts/s1_run_diagnostic.py --prepare-only --prepare-observation-wall-cap-s 12` 可在 PASSIVE/FIXEDSTAND 下观察恢复并在关节采集、RL 前 gate 与 command 3 之前结束。它不代替普通 A/D 运行，也不自动获准重复执行。

## 参数与命令

脚本支持显式 `--entry-threshold E --exit-threshold X --hold-s H`、`--scene`、`--groups`、`--repeats`。E、X、H 默认分别为 `-0.05`、`-0.08`、`0.30` 仿真秒；只将适用于所选组的字段写入该组 overlay。E/X/H 必须有限，`X<E`、`H>0`；组别不适用字段不进入新配置。数值以 double 解析并比较，因此 `0.30` 和 `0.3` 等价。

任务固定诊断条件示例（仅供 Director 后续重新授权后使用；本轮准备失败后不能据此自行重跑）：

```bash
rtk proxy /usr/bin/python3 scripts/manual_ab_compare.py \
  --scene scene_random_04.xml \
  --groups A D --repeats 1 \
  --entry-threshold -0.05 --exit-threshold -0.08 --hold-s 0.30
```

批次自动保存计划、各组 overlay、console、每次 run、metrics、Markdown/JSON 对照、timeline。遇到来源/配置冲突、INVALID、首周期精确配对失败、未确认清理或必要证据缺失时停止后续组；不尝试结果更好的重跑。

成功结果必须同时报告任务终局/安全、切换与 Recovery 时间、时间覆盖和效率；不按最少切换单指标排序，不将单次流程核验并入正式数据。

## 当前尝试的具体 blocker

首批 batch `20261012_134434_5e0425` 的 A 在 recorder/controller/MuJoCo 启动前失败：overlay 实际 E 文本 `-0.050000000000000003` 与旧的字符串检查 `-0.05` 不同。之后的生产数值校验已修复且本轮 A/D 实体 overlay 均通过。最新 A batch `20261012_135511_10cb82` 则因准备期间持续确认跌倒而拒绝 RL；该运行没有运动任务窗口。D 保持 NOT_STARTED。原始两类失败均保留，未来运行不得清除或隐藏准备期跌倒历史。
