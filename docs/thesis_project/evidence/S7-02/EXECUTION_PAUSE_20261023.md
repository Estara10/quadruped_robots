# S7-02 正式批次执行暂停记录

日期：2026-10-23。批次 `s7-abcd240-20261023` 按用户本轮正式执行提示词授权建立，矩阵顺序和参数未改。

## 已执行

- 冻结矩阵读取通过：240项、30张留出图、每组60项；矩阵摘要 `95c764729a7466455924518ae2b498e060f35a20d946e065af9f226f25c1c301`。
- 磁盘空间检查为58.118 GiB，高于15 GiB保护线；批次预算6 GiB。独立A/B/C/D overlay已准备，字段对应 E=−0.05、B/D X=−0.10、C/D H=0.20 s。
- 记录的代码提交：`26e7b5381759bb9a0ac97efc2c6fd2ba559c58e6`。批次清单记下本次实际模型、配置、运行和分析版本；仓库中已有的未提交工作区改动未清理。
- 正式首项 `S7-001`：holdout08 / B，run `f958f37f19d14f9c9d946b4a5ac80998`。MuJoCo启动后在等待物理权限时退出 code 1，0策略帧。原始 `mujoco.log` 明确写有 `ERROR: could not initialize GLFW`。因此这是启动/显示环境故障，不能解释为导航失败或安全表现。
- 该场景已计入基础启动1次。runner 因 `runtime_config_consumption_not_confirmed` 暂停，未派发其余239项。没有自动补跑或改参数。
- 冻结分析器已按原始记录重算：240项分母，`NOT_STARTED=239`、`UNRESOLVED_SCENE_ATTEMPT=1`，实际场景启动1次；当前没有有效导航样本，不能报告A/B/C/D组间效果或机制结论。决策时钟重建数为0。

## 清理和续接

run记录 `shutdown_complete=true`、MuJoCo退出码1、ROS/controller未启动；结束后检查 `/proc` 未发现本批MuJoCo、诊断器、ros2_control或RViz进程。检查时可用空间仍约59 GiB。

下一步需要在能连接 `DISPLAY=:0` 的获批执行环境中恢复。由于 S7-001 已消耗一次基础启动，其下一次场景启动必须按冻结规则登记为该计划项唯一替代；本批当前授权文件没有该项替代条目，因此在对应替代批准登记前保持暂停。此前有碰撞、跌倒、超时等合法导航终局的项仍不得重跑。批准后沿用同一批次、同一矩阵与原授权，使用runner的 `--resume`。

## 原始与派生产物

- 批次、台账、原始run及清单：`logs/08_formal_experiment/S7_ABCD_240/s7-abcd240-20261023/`
- 首项MuJoCo日志：`runs/S7-001/attempt_01/result/f958f37f19d14f9c9d946b4a5ac80998/mujoco.log`
- 首项控制台记录：`runs/S7-001/attempt_01/console.log`
- 部分正式分析：`analysis/formal_summary/`（从原始数据重算，不是构造数据）
