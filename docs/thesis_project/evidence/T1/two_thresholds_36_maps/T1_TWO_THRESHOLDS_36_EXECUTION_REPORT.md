# T1 两档退出阈值、36 张开发地图执行报告

**状态：INCOMPLETE / STOPPED_OPERATOR；T1 保持 ACTIVE，提交 Director 审阅。**

## 本轮范围与固定条件

执行依据：[授权提示词](../../../../../docs/thesis_project/T1_TWO_THRESHOLDS_36_MAPS_PROMPT.md)。计划为 M1 开发图 30 张与 M2 开发图 6 张，全部 B 组、`E=-0.05`、`X∈{-0.07,-0.12}`、无保持；每图每档两次，共 144 项。目标 `(7.5,0)`、到达半径 `0.5 m`，运动上限 `20 sim s / 120 wall s`，到达后站立观察 `5 sim s / 30 wall s`。留出集未使用，历史批次不抵扣本批重复。本批不是正式实验，不用于参数最优结论。

运行计划、独立配置副本和原始数据位于[`logs/t1/two_thresholds_36_maps/20261016_211029_3/`](../../../../../logs/t1/two_thresholds_36_maps/20261016_211029_3/)。冻结调度种子为 `2026101604`，每轮 36 个随机地图块、每块两档相邻；首轮顺序 18/18 平衡，第二轮逐图反转。预检与运行使用同一执行上下文的 `DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority`、`MUJOCO_GL=glfw`、系统 `/usr/bin/python3` 和本地临时库路径。GLFW 预检为 PASS，renderer 为 Intel / Mesa Intel(R) Graphics (RPL-P)，OpenGL 4.6；窗口已关闭。

## 运行与停止结果

启动 5 次场景：4 次完整有效、1 次中断且无效；另有 139 个计划项未启动。没有基础设施补做。四条完整运行均为 `ARRIVED`、任务窗口碰撞/跌倒判定 `False`、终止周期完整保存、策略周期缺口 0、无效时钟帧 0、有效时间覆盖 100%、约 50.10–50.26 Hz，并完成到达后的站立观察和进程收尾。它们仅覆盖两张 M1 开发图的一次重复，不形成完整地图×阈值单元，不能比较或排名。

| 顺序 | 地图 / X | 终局 | 任务耗时 | 短反向 / 有效任务分钟 | Recovery占比 | 分数 | 记录/覆盖 | 站立收尾 |
|---:|---|---|---:|---:|---:|---:|---|---|
| 1 | dev08 / −0.07 | ARRIVED | 5.580 s | 37 / 完整 | 0.1036 | 88.352 | VALID；周期0至终止帧完整 | COMPLETE |
| 2 | dev08 / −0.12 | ARRIVED | 5.380 s | 17 / 完整 | 0.1230 | 90.382 | VALID；周期0至终止帧完整 | COMPLETE |
| 3 | dev19 / −0.12 | ARRIVED | 9.604 s | 25 / 完整 | 0.3165 | 86.623 | VALID；周期0至终止帧完整 | COMPLETE |
| 4 | dev19 / −0.07 | ARRIVED | 11.800 s | 66 / 完整 | 0.3241 | 82.545 | VALID；周期0至终止帧完整 | COMPLETE |
| 5 | dev07 / −0.12 | UNKNOWN（操作中断） | UNKNOWN | UNKNOWN | UNKNOWN | UNKNOWN | INVALID；部分记录，无 terminal/process facts | 未证实 |

四条有效记录来源均核实为 `group=B`、`E=-0.05`、对应 X、场景 dev08/dev19、目标 `(7.5,0)`、唯一 writer/session；配置源记录 `hold=false`、保持时长 0。运行上下文保存了 cycle 0 的具名关节 q/dq、姿态、位置、速度和同周期身份。各次初态的 sim_time、位置、关节速度等并非完全相同，重复号不表示完全重放。运行记录和辅助逐 run 分数分别见 [`per_run_results_144.csv`](../../../../../logs/t1/two_thresholds_36_maps/20261016_211029_3/per_run_results_144.csv)、[`initial_state_and_coverage_144.csv`](../../../../../logs/t1/two_thresholds_36_maps/20261016_211029_3/initial_state_and_coverage_144.csv) 和 [`map_comparison_36.csv`](../../../../../logs/t1/two_thresholds_36_maps/20261016_211029_3/map_comparison_36.csv)。

第 5 项 run_id 为 `d4e8d5f464314977b9951abf5d3a5507`，保留了 336 个策略帧、run context、MuJoCo/ROS 日志及 framebuffer；未生成终局、metrics 或 process facts，故不能归为超时、碰撞、跌倒或有效任务结果。批次台账明确标记 `STOPPED_OPERATOR / INVALID`，139 项仍为未运行。详细的中断原因和收尾信号见[`operator_interrupt_cleanup_005.json`](../../../../../logs/t1/two_thresholds_36_maps/20261016_211029_3/operator_interrupt_cleanup_005.json)。中断后向本次 MuJoCo 及 ROS launch 发送 SIGINT；最终沙箱外进程清单为空，见[`post_stop_process_inventory.json`](../../../../../logs/t1/two_thresholds_36_maps/20261016_211029_3/post_stop_process_inventory.json)。不声称第 5 项正常终止或正常站立收尾。

## 停止原因与影响

在 4 条有效运行完成后，发现复用的 `manual_t1_expanded_400.run_one()` 将旧标签 `T1 expanded 400 development parameter validation; nonformal` 硬编码到本轮子进程参数，并在控制台也显示 `.../400`。本批实际 group/X 与配置源仍匹配，控制器行为参数没有偏离；但运行上下文的批次说明标签错误，削弱了运行来源和数据批次的清晰性。第 5 项已启动，因此按任务“运行开始后不可静默更换代码”的停止点结束；没有继续启动第 6 项及后续场景。运行控制台还显示 `H=0.3s`，但这是共用 CLI 参数回显；生产配置及来源记录明确为 B 组保持关闭，不应误读为启用了 C/D 保持。

后续最小修复是让本批实际调用路径接收本批 classification（或由新包装器直接组装既有 `manual_ab_compare.py` 参数），并在离线检查中断言子进程参数、run context 标签和本批 plan 一致，同时确认 B 配置仍无保持字段。修复后需由 Director/用户决定是否允许在原批次内保留 4 条有效结果、把第 5 项作为一次已消耗失败额度的独立重做，再继续未运行项；不得覆盖第 5 项部分原始文件。当前已授权的最多 4 次基础设施补做尚未使用，但本轮因明确停止点不自行续跑。

## 准备、记录和分析检查

- 离线检查通过：144 个计划键唯一；两档各 72 项；每图每档两次；首轮先后 18/18 平衡；第二轮逐图反转；冻结计划可复现。
- 生产配置解析检查通过：两个独立 overlay 均由实际配置检查器接受，组别 B、E/X 数值正确、无 `recovery_min_hold_s`；overlay 文件与安装源为独立副本。
- 当前生产控制器库、MuJoCo 主程序、三个部署模型和地图 XML/index/layout 身份快照见 [`expanded_400_plan.json`](../../../../../logs/t1/two_thresholds_36_maps/20261016_211029_3/expanded_400_plan.json)（文件名沿用既有运行 helper 接口；实际 schema 为 `t1-two-thresholds-36-v1`，计划数 144）。无运行代码或模型构建变更。
- Python 编译、计划与生产解析器定向检查、分析/绘图临时输入 smoke check 通过。对当前批次只做部分统计，产物位于 `logs`：完整 144 行模板中其余未运行项明确为 UNKNOWN；没有完整地图单元的分数差，bootstrap 估计为空，不作参数比较。
- 截图图表是部分批次可视化，不代表完整样本。8 张预选运行上限未超；无视频。

## 修改文件和当前限制

新增批次包装工具 [`manual_t1_two_thresholds_36.py`](../../../../../scripts/manual_t1_two_thresholds_36.py)、冻结方法说明 [`FROZEN_MATRIX_AND_METHOD.md`](FROZEN_MATRIX_AND_METHOD.md)；计划和运行数据均为独立批次，不改既有运行、地图、模型或阈值。

当前只有两张 M1 开发图各一次重复的四条有效运行；M2 没有开始。本批不足以生成 36 图双阈值比较、M1/M2 汇总结论或可解释的阈值权衡。碰撞/跌倒 False 只适用于这四个完整任务窗口；部分第 5 项安全结论 UNKNOWN。命令写出/控制器记录不证明 MuJoCo 对命令的实际消费。批次没有 A 参考，不比较 B 相对 A；不推成功率、性能排名或最优参数。

**通俗总结：** 准备与两档配置检查通过，前四次都正常到达并完成站立观察。第五次运行时发现运行说明沿用了旧的“400项”标签，于是按规则停止；第五条只留有部分数据，不能算结果，后面 139 项也没跑。相关仿真和 ROS 进程已核实关闭。T1 仍 ACTIVE，等待 Director 审阅；没有启动下一阶段，没有提交 Git。
