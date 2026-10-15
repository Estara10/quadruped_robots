# B1 本批运行矩阵

原始主批仍是非正式流程/机制诊断，不是正式实验。其八条已启动运行均来源校验VALID；`VALID`不等于所有指标完整。下方主表保留原八条数据。Director重审后另有两次密集图B尝试，见补充表；两次均为记录INVALID，不能并入有效结果。之后的dev25 A、dev03 C/D和本页最新dev03 D收尾核验均各自独立列示，不与旧run拼接作性能比较。

## 最新补充：dev03 D 收尾顺序真实运行核验

| 运行 | 实际配置/结果 | 记录与任务窗口 | 收尾及安全边界 |
|---|---|---|---|
| [dev03_D_closeout_once](../../../../logs/b1/closeout_runtime_verification/dev03_D_20261015/D/rep_01/6a0e402de1754d9a828704976d8ee32f/) | D，E=-0.05、X=-0.08、H=0.30s；ARRIVED，距目标0.4985m；独立overlay来源确认 | 整份VALID；615策略帧、cycle缺口0、无效时钟0、约50.144Hz；终止cycle565同快照持久化。任务周期0–565共566帧；收尾周期566–614另49帧 | 最终producer=consumer=615、未读损失0、槽位复用0、冲突0；站立观察5.022s/238样本完成。保持9/9通过；首次自动metrics的任务安全字段曾为UNKNOWN，现由独立生产重分析修正为任务窗覆盖完整、碰撞/足端接触/撞击/跌倒均False；整段收尾覆盖仍不完整。见[独立重分析](../../../../logs/b1/historical_reanalysis/dev03_D_task_safety_scope_20261015/README.md)、[指标](../../../../logs/b1/closeout_runtime_verification/dev03_D_20261015/D/rep_01/6a0e402de1754d9a828704976d8ee32f/metrics.json)、[timeline](../../../../logs/b1/closeout_runtime_verification/dev03_D_20261015/D/rep_01/6a0e402de1754d9a828704976d8ee32f/timeline.png)和[分析表](../../../../logs/b1/closeout_runtime_verification/dev03_D_20261015/D/rep_01/analysis/run_results.md) |

该单次运行验证了新收尾顺序的实际排空和任务边界隔离；不代表D性能、安全成功或重复运行稳定性。原dev03 D整份INVALID记录及48条缺失尾段仍保持原状。

**任务期安全重分析（同一run，非新运行）：**已证实cycle 0–565的566个策略周期安全快照LIVE、同capture且物理publisher连续；策略终止后49帧的快照为STALE。独立分析输出任务窗非足端碰撞False、足端接触/撞击0、按当前确认规则跌倒False；一条关闭且未确认的姿态候选仍保留。全采集安全结论继续不可用，不将任务窗结果延伸到站立/收尾。详见[独立派生结果](../../../../logs/b1/historical_reanalysis/dev03_D_task_safety_scope_20261015/)。

| 运行 | 组/地图 | 终局 / 终点距目标 | 经过 / 有效连续时间 (s) | 路径 / 均速 / 峰速 | 保存帧 / 周期缺口 / 无效时钟 | 周期跨度频率 Hz | 已观察A→R/R→A（有效时钟边沿） | Recovery秒 / 占有效运动时间 | 短反向数/覆盖 | 碰撞/跌倒/运动窗覆盖 | C/D保持完整/不完整；违规 |
|---|---|---|---:|---|---:|---:|---|---|---|---|---|
| [flat_A_01](../../../../logs/b1/runs/flat_A_01/A/rep_01/be491dadee3f4f229e3f6ddc8c1034d0/) | A/flat | ARRIVED / 0.465 m | 3.980 / 3.840 | 6.98 / 1.82 / 2.95 | 198 / 2 / 2 | 50.56 / partial_or_unknown | 0/0 (0) | 0.000 / 0.000 | 0 / partial_or_unknown | False / False / complete | — |
| [flat_A_02](../../../../logs/b1/runs/flat_A_02/A/rep_01/e0d59210618a41c98138380c04480322/) | A/flat | ARRIVED / 0.468 m | — / 3.936 | 6.87 / 1.74 / 2.86 | 206 / 2 / 5 | 50.60 / partial_or_unknown | 0/0 (0) | 0.000 / 0.000 | 0 / partial_or_unknown | UNKNOWN / UNKNOWN / UNKNOWN | — |
| [m1ssd01_A](../../../../logs/b1/runs/m1ssd01_A/A/rep_01/2788e030fbdb496cbb6d77d58f0a5ddb/) | A/m1ssd01 | ARRIVED / 0.494 m | 7.700 / 7.520 | 8.13 / 1.08 / 2.66 | 382 / 4 / 2 | 50.33 / partial_or_unknown | 16/16 (32) | 1.458 / 0.194 | 30 / partial_or_unknown | False / UNKNOWN / complete | — |
| [m1ssd01_B](../../../../logs/b1/runs/m1ssd01_B/B/rep_01/923705d16d3b4e63b66f8e6ef3a95702/) | B/m1ssd01 | ARRIVED / 0.497 m | 12.240 / 12.020 | 10.37 / 0.86 / 2.40 | 611 / 2 / 4 | 50.16 / partial_or_unknown | 23/23 (46) | 3.470 / 0.289 | 40 / partial_or_unknown | False / UNKNOWN / complete | — |
| [m1ssd01_C](../../../../logs/b1/runs/m1ssd01_C/C/rep_01/69ddda58e6bd4d8ca9d4ed23e556a84b/) | C/m1ssd01 | DIAGNOSTIC_SIM_TIME_LIMIT / 3.030 m | 20.020 / 19.332 | 10.34 / 0.53 / 2.41 | 985 / 17 / 1 | 50.13 / partial_or_unknown | 22/24 (46) | 8.370 / 0.433 | 34 / partial_or_unknown | False / UNKNOWN / complete | 16/6;0 |
| [m1ssd01_D](../../../../logs/b1/runs/m1ssd01_D/D/rep_01/b735497bb66345619c599e254b108254/) | D/m1ssd01 | ARRIVED / 0.498 m | 11.480 / 10.980 | 10.00 / 0.91 / 2.44 | 573 / 2 / 11 | 50.20 / partial_or_unknown | 9/9 (18) | 2.978 / 0.271 | 12 / partial_or_unknown | False / False / complete | 4/5;0 |
| [m1dod01_D](../../../../logs/b1/runs/m1dod01_D/D/rep_01/0f148c54215e409fbbcacb23955510de/) | D/m1dod01 | ARRIVED / 0.477 m | 7.060 / 6.880 | 8.04 / 1.17 / 2.71 | 350 / 4 / 2 | 50.35 / partial_or_unknown | 3/3 (6) | 1.140 / 0.166 | 2 / partial_or_unknown | False / UNKNOWN / complete | 2/1;0 |
| [m1dod01_C](../../../../logs/b1/runs/m1dod01_C/C/rep_01/a0c8d5a30cbb46718ccd1acb7cb7f994/) | C/m1dod01 | ARRIVED / 0.474 m | 7.160 / 6.854 | 7.60 / 1.11 / 2.82 | 356 / 3 / 6 | 50.23 / partial_or_unknown | 4/4 (8) | 1.222 / 0.178 | 5 / partial_or_unknown | False / UNKNOWN / complete | 3/1;0 |

帧数是保存的runtime策略帧；总体策略频率按保存帧cycle ID跨度/单调时间跨度计算，频率覆盖均为`partial_or_unknown`。模式边沿计数来自持久化相邻帧；时间无效不会删除边沿，但周期缺口使总边沿数及短反向完整性未知。路径/速度只由实际可用连续区间计算，不跨缺口补插。

## 安全与接触明细

| 运行 | 足端接触数 | 足端撞击数 | 非足端碰撞终止 | 碰撞事件对象 | 跌倒状态 | 运动窗覆盖 | base越界 |
|---|---:|---:|---|---|---|---|---|
| flat_A_01 | 0 | 0 | False | 无记录（完整覆盖） | False | complete | UNKNOWN |
| flat_A_02 | UNKNOWN | UNKNOWN | UNKNOWN | UNKNOWN | UNKNOWN | UNKNOWN | UNKNOWN |
| m1ssd01_A | 0 | 0 | False | 无记录（完整覆盖） | UNKNOWN | complete | UNKNOWN |
| m1ssd01_B | 1 | 1 | False | 无记录（完整覆盖） | UNKNOWN | complete | UNKNOWN |
| m1ssd01_C | 0 | 0 | False | 无记录（完整覆盖） | UNKNOWN | complete | UNKNOWN |
| m1ssd01_D | 0 | 0 | False | 无记录（完整覆盖） | False | complete | UNKNOWN |
| m1dod01_D | 0 | 0 | False | 无记录（完整覆盖） | UNKNOWN | complete | UNKNOWN |
| m1dod01_C | 0 | 0 | False | 无记录（完整覆盖） | UNKNOWN | complete | UNKNOWN |

## 输出证据

- flat_A_01: [run context](../../../../logs/b1/runs/flat_A_01/A/rep_01/be491dadee3f4f229e3f6ddc8c1034d0/run_context.json) · [metrics](../../../../logs/b1/runs/flat_A_01/A/rep_01/be491dadee3f4f229e3f6ddc8c1034d0/metrics.json) · [timeline](../../../../logs/b1/runs/flat_A_01/A/rep_01/be491dadee3f4f229e3f6ddc8c1034d0/timeline.png) · [分析结果表](../../../../logs/b1/runs/flat_A_01/A/rep_01/analysis/run_results.md)
- flat_A_02: [run context](../../../../logs/b1/runs/flat_A_02/A/rep_01/e0d59210618a41c98138380c04480322/run_context.json) · [metrics](../../../../logs/b1/runs/flat_A_02/A/rep_01/e0d59210618a41c98138380c04480322/metrics.json) · [timeline](../../../../logs/b1/runs/flat_A_02/A/rep_01/e0d59210618a41c98138380c04480322/timeline.png) · [分析结果表](../../../../logs/b1/runs/flat_A_02/A/rep_01/analysis/run_results.md)
- m1ssd01_A: [run context](../../../../logs/b1/runs/m1ssd01_A/A/rep_01/2788e030fbdb496cbb6d77d58f0a5ddb/run_context.json) · [metrics](../../../../logs/b1/runs/m1ssd01_A/A/rep_01/2788e030fbdb496cbb6d77d58f0a5ddb/metrics.json) · [timeline](../../../../logs/b1/runs/m1ssd01_A/A/rep_01/2788e030fbdb496cbb6d77d58f0a5ddb/timeline.png) · [分析结果表](../../../../logs/b1/runs/m1ssd01_A/A/rep_01/analysis/run_results.md)
- m1ssd01_B: [run context](../../../../logs/b1/runs/m1ssd01_B/B/rep_01/923705d16d3b4e63b66f8e6ef3a95702/run_context.json) · [metrics](../../../../logs/b1/runs/m1ssd01_B/B/rep_01/923705d16d3b4e63b66f8e6ef3a95702/metrics.json) · [timeline](../../../../logs/b1/runs/m1ssd01_B/B/rep_01/923705d16d3b4e63b66f8e6ef3a95702/timeline.png) · [分析结果表](../../../../logs/b1/runs/m1ssd01_B/B/rep_01/analysis/run_results.md)
- m1ssd01_C: [run context](../../../../logs/b1/runs/m1ssd01_C/C/rep_01/69ddda58e6bd4d8ca9d4ed23e556a84b/run_context.json) · [metrics](../../../../logs/b1/runs/m1ssd01_C/C/rep_01/69ddda58e6bd4d8ca9d4ed23e556a84b/metrics.json) · [timeline](../../../../logs/b1/runs/m1ssd01_C/C/rep_01/69ddda58e6bd4d8ca9d4ed23e556a84b/timeline.png) · [分析结果表](../../../../logs/b1/runs/m1ssd01_C/C/rep_01/analysis/run_results.md)
- m1ssd01_D: [run context](../../../../logs/b1/runs/m1ssd01_D/D/rep_01/b735497bb66345619c599e254b108254/run_context.json) · [metrics](../../../../logs/b1/runs/m1ssd01_D/D/rep_01/b735497bb66345619c599e254b108254/metrics.json) · [timeline](../../../../logs/b1/runs/m1ssd01_D/D/rep_01/b735497bb66345619c599e254b108254/timeline.png) · [分析结果表](../../../../logs/b1/runs/m1ssd01_D/D/rep_01/analysis/run_results.md)
- m1dod01_D: [run context](../../../../logs/b1/runs/m1dod01_D/D/rep_01/0f148c54215e409fbbcacb23955510de/run_context.json) · [metrics](../../../../logs/b1/runs/m1dod01_D/D/rep_01/0f148c54215e409fbbcacb23955510de/metrics.json) · [timeline](../../../../logs/b1/runs/m1dod01_D/D/rep_01/0f148c54215e409fbbcacb23955510de/timeline.png) · [分析结果表](../../../../logs/b1/runs/m1dod01_D/D/rep_01/analysis/run_results.md)
- m1dod01_C: [run context](../../../../logs/b1/runs/m1dod01_C/C/rep_01/a0c8d5a30cbb46718ccd1acb7cb7f994/run_context.json) · [metrics](../../../../logs/b1/runs/m1dod01_C/C/rep_01/a0c8d5a30cbb46718ccd1acb7cb7f994/metrics.json) · [timeline](../../../../logs/b1/runs/m1dod01_C/C/rep_01/a0c8d5a30cbb46718ccd1acb7cb7f994/timeline.png) · [分析结果表](../../../../logs/b1/runs/m1dod01_C/C/rep_01/analysis/run_results.md)

## 逐组概要

- 平地A两次均VALID/ARRIVED，终点距离约0.465/0.468 m，站立观察完成；flat_A_02碰撞和跌倒否定结论均UNKNOWN。
- 稀疏图A、B、D到达，C在20仿真秒运动上限结束、终点距目标3.030 m。A/B各有RA驱动双向切换；C有大量Recovery episode且保持审核存在不完整片段。
- 密集图已完成D、C，均到达。其余B、A未启动，不能推断这些组在该图的结果。
- 本批到达只表示满足0.5m到达判据；碰撞或跌倒UNKNOWN不因ARRIVED清零。
- 所有已完成运行的站立观察为COMPLETE；正常路径记录了command 6请求与FIXEDSTAND状态，但不等于MuJoCo消费命令的确认。
- 整体无可据以断言普遍安全或性能排序的样本量；这8条不用于成功率、参数优选或论文结论。

## RA与风险/模式边沿摘要

RA范围按每条runtime JSONL中实际持久化的有限策略帧计算，不代表producer未保存周期。risk触发和mode边沿同样限于保存帧；缺帧时不得称完整。

| run | 保存帧RA最小–最大 | risk edge true / false / UNKNOWN | 保存帧A→R / R→A | paired风险→A→R | 内部决策延迟中位数 / 范围(ns) |
|---|---:|---:|---:|---:|---:|
| flat_A_01 | -0.9994…-0.7542 | 0 / 197 / 1 | 0 / 0 | 0 | UNKNOWN |
| flat_A_02 | -0.9994…-0.7490 | 0 / 205 / 1 | 0 / 0 | 0 | UNKNOWN |
| m1ssd01_A | -0.9994…0.2665 | 16 / 365 / 1 | 16 / 16 | 16 | 360 / 84–923 |
| m1ssd01_B | -0.9989…0.5044 | 28 / 582 / 1 | 23 / 23 | 23 | 751 / 336–2109 |
| m1ssd01_C | -0.8523…0.4155 | 28 / 956 / 1 | 22 / 24 | 22 | 616 / 256–1635 |
| m1ssd01_D | -0.9992…0.3933 | 14 / 558 / 1 | 9 / 9 | 9 | 484 / 189–914 |
| m1dod01_D | -0.9993…0.3754 | 4 / 345 / 1 | 3 / 3 | 3 | 346 / 273–1682 |
| m1dod01_C | -0.9993…0.3724 | 5 / 350 / 1 | 4 / 4 | 4 | 296 / 237–853 |

纳秒延迟仅是同策略周期risk判定到mode切换的内部决策耗时，不能解释为动作写出、生效、机器响应或完整避障延迟。

修订说明：旧表将枚举UNKNOWN(2)按非零误计为风险进入；本表已按`risk_condition_entered == 1`修正，并单列UNKNOWN。逐run独立复核与metrics配对结果见[B1风险枚举审计](B1_RISK_EDGE_AUDIT.md)。

## Director重审后的补充运行（不改主批）

以下是m1dod01/B原计划项的首次运行和唯一一次基础设施重试。两条都ARRIVED，但runtime因落盘帧次序回退而INVALID；不纳入有效导航结果或组间比较。两次清理完整，正常5秒站立观察未完成。

| 运行 | 配置/来源 | supervisor终局 | 记录判定 | 记录故障 | 清理/站立观察 |
|---|---|---|---|---|---|
| [m1dod01_B 首次](../../../../logs/b1/runs/m1dod01_B/B/rep_01/6edf02f841a64b508c25235aad9deba2/) | dev25 / m1dod01；B，E=-0.05、X=-0.08、H=0；本次controller及唯一writer/session来源已绑定 | ARRIVED，距目标0.4922 m | INVALID | monotonic time、rl_step、source sequence均非严格递增；history ring自身未报告overflow | process shutdown与closeout完成；安全PASSIVE转换有记录；正常站立观察未完成 |
| [m1dod01_B 唯一重试](../../../../logs/b1/runs/m1dod01_B_retry_01/B/rep_01/8fa9e43785f34f41b73aa26af7b5ea34/) | 同上，独立run；配置/模型来源核对通过 | ARRIVED，距目标0.4863 m | INVALID | 同类帧次序回退；本次重试在主循环第二个无界history drain修复之前 | process shutdown与closeout完成；安全PASSIVE转换有记录；正常站立观察未完成 |

（原10项主批当时状态）重试额度已用尽，计划中的A未启动，不能继续该主批。最终有界drain修复在后续dev25 A单次补证中取得运行证据；补证独立列于下表，不回填原批次。

## 唯一dev25 A组运行补证（独立于原主批）

| 运行 | 配置/来源 | supervisor终局 | 分析及记录边界 | 收尾/限制 |
|---|---|---|---|---|
| [dev25 A 单次补证](../../../../logs/b1/runs/m1dod01_A/attempt_01/A/rep_01/a79d150a1e544bc98c75f4db38345b94/) | `dev25.xml / m1dod01`；A、E=-0.05、目标7.5m；独立overlay；来源/控制器库确认；场景物理authority LIVE | ARRIVED；终点距目标0.489m | VALID；442帧，cycle 0–441连续、时钟有效帧442/442；终止payload与引用精确一致；任务边界完整、未读丢失0；40条边沿、37次短反向（覆盖完整）；Recovery 2.222s | FL→`ppt_obstacle_02`足端接触/撞击1次；跌倒UNKNOWN。physics snapshot非LIVE导致5秒站立观察未完成；controller PASSIVE转换确认，进程清理完整 |

该次运行仅证明最终周期合并/写盘/终止/分析链在此run上的表现，不纳入原八条主批统计，不构成A/B组间性能证据。metrics、结果表和图分别见run目录中的`metrics.json`、`analysis/run_results.md`和`timeline.png`。

### dev25 A安全指标的后续独立重分析（不改运行矩阵原始行）

上方A行中的“跌倒UNKNOWN”对应当时保存的旧metrics，保留作历史分析状态。Director指出其任务期姿态候选后来已有关闭且未确认的同run物理历史。本轮按快照时序重评为：任务窗候选step 5717、sim 11.434 s至step 5763、sim 11.526 s，任务终点前已`closed=true, confirmed=false`；在物理连续、同capture、无历史溢出且无无效姿态重叠的覆盖条件下，修复后的生产分析输出“按当前确认规则未判定跌倒”。这不等于没有下沉/异常或安全成功。独立结果见[重分析摘要](../../../../logs/b1/historical_reanalysis/dev25_A_safety_review/README.md)及[metrics](../../../../logs/b1/historical_reanalysis/dev25_A_safety_review/metrics.json)。

原run实际另有一次FL→`ppt_obstacle_02`足端接触/撞击，持续约0.018 s；“非足端终止碰撞为0”不能写成“无碰撞”。原normal-closeout gate记录physics status为空，当前已定位为FrameCapture一致去重分支没有把同一JSONL帧附带的物理snapshot返回给监督器。此次修复和离线生产联测不改变该运行事实：原run未完成5秒站立观察。终止策略帧时间15.682 s，附带物理sample为LIVE、sim_time 15.686 s、physics_step 7843（约晚4 ms）；这是独立物理采样，不是精确同周期同步测量。


## 最新dev03 C/D补证（独立于原八项主批）

| 组 | supervisor终局 | 整份记录 | 任务边界/机制 | 安全和收尾 |
|---|---|---|---|---|
| [C](../../../../logs/b1/cd_runtime_verification/dev03_C_D_20261015_170521/C/rep_01/e1243039761e43ddaa909f431bb5011c/) | COLLISION_TERMINATION；距目标1.473m | VALID；842帧，cycle0–841连续，0丢失、0无效策略时钟；50.197Hz | 36条模式边沿均有有效仿真时间；Recovery 7.022s/41.75%；短反向27次；18/18 H episode完整通过、0违规 | RL_calf(body12/geom46)→ppt_obstacle_02非足端碰撞终止；FL和RL各一次足端撞击；未进入站立观察，PASSIVE确认且清理完成。运动前另有已确认并关闭的低高度episode |
| [D](../../../../logs/b1/cd_runtime_verification/dev03_C_D_20261015_170521/D/rep_01/e8cae530962645ca8f0d25b0359e0345/) | ARRIVED；距目标0.4982m | INVALID：final closeout producer743/consumer695，48帧未持久化；不把导航终局改写 | task terminal边界0–694共695帧连续且时钟有效；生产hold helper对该边界审计11/11完整PASS、0违规；整体metrics仍UNKNOWN。审计见[D task-window hold JSON](../../../../logs/b1/cd_runtime_verification/dev03_C_D_20261015_170521/D_task_window_hold_audit.json) | 健康gate为LIVE/eligible；正常FIXEDSTAND观察5.024仿真秒/226样本完成；未发PASSIVE/趴下；过程退出与closeout确认。安全窗总体仍按INVALID保留UNKNOWN；运动前确认低高度episode已关闭 |

C、D各仅一次，初态有差异；不作为性能、成功率或参数优劣比较。D的限定任务窗保持审计不覆盖终止后的丢帧，也不将D整份记录提升为VALID。完整run_context、metrics、terminal、运行表和timeline均在各自目录。
