# M2 第二轮冻结矩阵与方法

批次：`20261016_174309_9`。本文件记录实际启动前冻结的条件；权重和归一化常数沿用用户在旧400项后确认的辅助评分，本批运行前已固定。

## 固定条件

- 地图：M2开发候选 `m2dev01.xml`～`m2dev06.xml`（内部ID `m2d01`～`m2d06`），布局文件和场景身份哈希保存在批次计划中；本轮不改变地图。
- 组别与参数：全部B，E=−0.05，X={−0.06, −0.07, −0.08, −0.09, −0.10, −0.11, −0.12, −0.13}；保持关闭。
- 每图每档3次，3轮；每轮随机排列六图，每个地图×轮次内随机排列八档；种子2026101602。144条完整矩阵及顺序见[冻结批次计划](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/m2_round2_plan.json)。
- 起点(0,0)m，目标(7.5,0)m，到达半径0.5m；任务上限20仿真秒/120墙钟秒；到达后站立观察5仿真秒/30墙钟秒。模型、观测、动作、频率和安全规则沿用生产链。
- 评分：到达0.40；任务窗无终止性非足端碰撞0.30；效率0.20，ARRIVED时`max(0,1−task_time/20)`，其他合法任务终局为0；稳定性0.10，逐run先算`1/(1+f/257.8)`，f为0.5秒短反向切换/有效任务分钟。总分为加权分×100。缺碰撞结论、任务计时或完整短反向频率则该run分数UNKNOWN，不补零。
- 汇总顺序：每图×阈值先平均3次，再对六图等权；仅覆盖完整项可报部分结果，不把不完整覆盖包装成完整排名。Bootstrap以地图为簇，10,000次、固定种子2026101603；地图是目的开发图，区间只作描述。
- 预算：批次目录4GiB；系统可用空间保护线15GiB。每项前估算，每8项核对实际用量。预选第一轮首图八个阈值运行抓帧，最多8次运行、16张图；不录像。

## 实际执行停止点

在第5条计划项 `005_r1_m2d06_x11` 的固定站立准备阶段未能在墙钟上限内确认稳定站立，run记录为`PREPARATION_FAILURE / INVALID`，没有进入RL。该准备失败触发批次停止，不重试、不启动剩余139条。前4项是有效ARRIVED任务结果，但只覆盖同一张图的四个X各一次，不作完整矩阵推断。

## 可复制命令与恢复边界

准备一个新批次（仅在Director/用户另行授权后）：

```bash
rtk proxy env DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority MUJOCO_GL=glfw LD_LIBRARY_PATH=/home/lidio/Libraries/unitree_sdk2/lib:/home/lidio/Libraries/libtorch-cpu-2.0.1/lib:/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu /usr/bin/python3 -u scripts/manual_t1_m2_round2.py --prepare-only
```

正式运行必须在同一个获准桌面命令上下文中先调用GLFW预检，再将预检结果路径传给该批的`--batch-root <batch>` `--resume`命令；本批实际调用见[批次控制台日志](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/)和[预检记录](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/glfw_preflight.json)。

恢复命令模板仅接受状态为`FROZEN`、`RUNNING`或`STOPPED_OPERATOR`且无未解决失败的批次：

```bash
rtk proxy env DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority MUJOCO_GL=glfw LD_LIBRARY_PATH=/home/lidio/Libraries/unitree_sdk2/lib:/home/lidio/Libraries/libtorch-cpu-2.0.1/lib:/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu /usr/bin/python3 -u scripts/manual_t1_m2_round2.py --batch-root <batch> --resume
```

本批状态为`STOPPED`（第5项准备失败），恢复入口会拒绝它；不应通过编辑计划状态绕过拒绝。是否为失败原因另行授权补证或新批次，等待审阅决定。

## 运行前与代码冻结

GLFW隐藏窗口预检与实际诊断位于同一个沙箱外命令父上下文，使用`DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority`、`MUJOCO_GL=glfw`、系统`/usr/bin/python3`和本地临时`LD_LIBRARY_PATH`。renderer为Intel / Mesa Intel(R) Graphics (RPL-P)，OpenGL 4.6。预检结果见批次[GLFW记录](../../../../../logs/01_T1_exit_threshold/m2_exit_round2/20261016_174309_9/glfw_preflight.json)。

实际构建阶段、controller库、MuJoCo可执行文件、模型和场景/布局身份哈希见批次计划。M2目录预先完成模型加载与生产身份登记；本轮四个有效run的运行来源及实际消费参数也分别写入其`run_context.json`。初次无场景启动的编排失败只因生产阶段快照带易变`created_at`字段；已从稳定比较字段中排除此时间戳，并保留失败说明。该修正不改变控制器、模型、地图或阈值。没有用该次启动失败替代任何运行项。

## 执行后补充（2026-10-16；保留原运行前冻结条件）

原第73项首个被拒样本为LIVE，身份匹配但`physics_coverage_complete=0`；较晚样本才为STALE。原始记录没有覆盖首次断裂时刻，根因仍UNKNOWN。新增首次覆盖丢失诊断复用生产连续性条件，未改变阈值或安全门；MuJoCo主程序增量构建成功。第73项attempt_02通过生产链得到有效记录，但没有重现coverage loss。原attempt_01保留。原第69项任务终局ARRIVED，跌倒状态UNKNOWN，因此正常站立收尾未获准执行；没有重跑。

按本次明确授权，实际场景启动额度由145修订为146：授权前已确认74次，本轮只新增第73项一次补做及原74–144项各一次，共72次，无其他场景重试。最终场景启动146；144/144计划项均有有效结果和完整评分，4条确认无场景的前置/编排失败独立计数。冻结的场景、模型、阈值、运行时限和评分公式未更改。最终结果表和评分位于批次目录的`m2_run_results_144.csv`及`weighted_score_item73_repair_final/`。

8档描述性平均辅助分为：X=−0.06 65.320；−0.07 66.412；−0.08 62.495；−0.09 62.884；−0.10 66.414；−0.11 68.168；−0.12 71.142；−0.13 61.367。区间宽，且属于开发集批次；不得据此宣称唯一最优、成功率或正式性能结论。M2无A参考，未使用留出集；1项跌倒状态UNKNOWN及收尾/覆盖限制仍单列。历史计划和数据均保留。
