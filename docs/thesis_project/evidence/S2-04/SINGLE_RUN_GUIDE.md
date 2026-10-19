# S2-04 单次场景诊断使用说明

从仓库根目录运行。`--scene` 接受以下十个文件名；省略时沿用原来的 `scene_obstacle.xml`：

| 参数 | 场景 ID | 当前 XML 中障碍数 | 起点到目标直线阻挡 |
|---|---|---:|---|
| `scene_obstacle.xml` | `obstacle_test1` | 7 | — |
| `scene_flat.xml` | `flat` | 0 | — |
| `scene_ppt_sparse.xml` | `ppt_sparse` | 4 | — |
| `scene_ppt_medium.xml` | `ppt_medium` | 7 | — |
| `scene_ppt_dense.xml` | `ppt_dense` | 11 | — |
| `scene_random_01.xml` | `random_01` | 15 | `ppt_obstacle_01`, x=[2.489, 3.011] m |
| `scene_random_02.xml` | `random_02` | 17 | `ppt_obstacle_03`, x=[2.561, 3.039] m |
| `scene_random_03.xml` | `random_03` | 18 | `ppt_obstacle_04`, x=[2.585, 3.415] m |
| `scene_random_04.xml` | `random_04` | 19 | `ppt_obstacle_15`, x=[2.556, 3.102] m |
| `scene_random_05.xml` | `random_05` | 20 | none |

`scene_random_01.xml` 至 `scene_random_05.xml` 由 `docs/thesis_project/designs/random_static_scenes/layouts.json` 中的固定布局生成。当前布局修订为 `S2-08-r1`：保留原种子，以确定性手工编辑将 01 的障碍 01、02 的障碍 03、03 的障碍 04 移到直线阻挡位置；没有重新随机。浅蓝保留通道只用于几何预检，不传给控制器。旧 JSON、预览、XML 和旧场景目录身份保存在 [S2-08 previous_layouts](../S2-08/previous_layouts/)；更早运行仍按各自 `run_context.json` 中保存的旧场景身份解释，不套用本修订坐标。random_04 原有障碍 14 超出声明 y 下界 0.056375 m，本次未移动。

场景路径、根 XML、include/asset closure 和 MuJoCo 编译模型指纹必须与共用目录一致。缺失、未支持或身份不符时会在进入 RL 前拒绝。平地没有障碍是合法输入；地面接触仍单独记为 ground contact。

当前部署配置的目标为 `(7.0, 0.0) m`，到达半径为 `0.5 m`。运行命令显式填写 `--goal-x 7.0`，必须与所选 controller 配置相同。A / `paper_faithful_switch`、模型、阈值和安全规则由当前配置提供；运行期间不热更新。诊断运动上限为 8 仿真秒，墙钟保护为 90 秒。终局按现有 supervisor 规则处理安全中止、机器人—障碍碰撞、跌倒、到达和仿真时限；墙钟保护到期属于不完整运行。

下面是可直接复制的独立运行示例。每个场景使用自己的目录 `logs/manual_scene_runs/<scene-name>`，不要把不同场景写进同一个目录；每次运行仍会在该场景目录下建立独立 run 记录。重复运行同一场景时，可另建带日期或编号的场景目录。普通手动运行记录保存在 `logs/`，不要写入 `docs/thesis_project/evidence/`。这些示例不表示各场景已完成正式测试。桌面会话需允许当前用户使用 `DISPLAY` 和当前桌面会话提供的 `XAUTHORITY`。

```bash
PROJECT_ROOT="$(git rev-parse --show-toplevel)"
cd "$PROJECT_ROOT"
rtk proxy /usr/bin/python3 scripts/s1_run_diagnostic.py --scene scene_obstacle.xml --goal-x 7.0 --sim-limit-s 20 --wall-cap-s 90 --overlay "$PROJECT_ROOT/quadruped_ros2_control_humble/install/go2_description" --output-root logs/manual_scene_runs/scene_obstacle --display "$DISPLAY" --xauthority "$XAUTHORITY"
```

```bash
PROJECT_ROOT="$(git rev-parse --show-toplevel)"
cd "$PROJECT_ROOT"
rtk proxy /usr/bin/python3 scripts/s1_run_diagnostic.py --scene scene_flat.xml --goal-x 7.0 --sim-limit-s 20 --wall-cap-s 90 --overlay "$PROJECT_ROOT/quadruped_ros2_control_humble/install/go2_description" --output-root logs/manual_scene_runs/scene_flat --display "$DISPLAY" --xauthority "$XAUTHORITY"
```

```bash
PROJECT_ROOT="$(git rev-parse --show-toplevel)"
cd "$PROJECT_ROOT"
rtk proxy /usr/bin/python3 scripts/s1_run_diagnostic.py --scene scene_ppt_sparse.xml --goal-x 7.0 --sim-limit-s 20 --wall-cap-s 90 --overlay "$PROJECT_ROOT/quadruped_ros2_control_humble/install/go2_description" --output-root logs/manual_scene_runs/scene_ppt_sparse --display "$DISPLAY" --xauthority "$XAUTHORITY"
```

```bash
PROJECT_ROOT="$(git rev-parse --show-toplevel)"
cd "$PROJECT_ROOT"
rtk proxy /usr/bin/python3 scripts/s1_run_diagnostic.py --scene scene_ppt_medium.xml --goal-x 7.0 --sim-limit-s 20 --wall-cap-s 90 --overlay "$PROJECT_ROOT/quadruped_ros2_control_humble/install/go2_description" --output-root logs/manual_scene_runs/scene_ppt_medium --display "$DISPLAY" --xauthority "$XAUTHORITY"
```

```bash
PROJECT_ROOT="$(git rev-parse --show-toplevel)"
cd "$PROJECT_ROOT"
rtk proxy /usr/bin/python3 scripts/s1_run_diagnostic.py --scene scene_ppt_dense.xml --goal-x 7.0 --sim-limit-s 20 --wall-cap-s 90 --overlay "$PROJECT_ROOT/quadruped_ros2_control_humble/install/go2_description" --output-root logs/manual_scene_runs/scene_ppt_dense --display "$DISPLAY" --xauthority "$XAUTHORITY"
```

每次 run 有自己的 `runtime_record.jsonl`、`run_context.json`、`terminal.json`、进程清理事实和启动日志。执行分析命令会为该目录中的每个 run 生成 `metrics.json`、`timeline.png`，并在指定的场景目录 `analysis/` 子目录汇总结果表；不要把不同场景混放或把不同 run 合并成一个轨迹。

以下以平地和稀疏场景为例。`PYTHONNOUSERSITE=1` 避免用户目录中的 NumPy 覆盖系统 Matplotlib 所需版本；Matplotlib 缓存写入 `/tmp`。

```bash
PROJECT_ROOT="$(git rev-parse --show-toplevel)"
cd "$PROJECT_ROOT"
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/s204-mpl FONTCONFIG_PATH=/tmp/s204-fontconfig rtk proxy /usr/bin/python3 scripts/s1_analyze_run.py --runs-root logs/manual_scene_runs/scene_flat --output-dir logs/manual_scene_runs/scene_flat/analysis --task-label S2-04-flat
```

```bash
PROJECT_ROOT="$(git rev-parse --show-toplevel)"
cd "$PROJECT_ROOT"
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/s204-mpl FONTCONFIG_PATH=/tmp/s204-fontconfig rtk proxy /usr/bin/python3 scripts/s1_analyze_run.py --runs-root logs/manual_scene_runs/scene_ppt_sparse --output-dir logs/manual_scene_runs/scene_ppt_sparse/analysis --task-label S2-04-sparse
```

```bash
PROJECT_ROOT="$(git rev-parse --show-toplevel)"
cd "$PROJECT_ROOT"
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/s204-mpl FONTCONFIG_PATH=/tmp/s204-fontconfig rtk proxy /usr/bin/python3 scripts/s1_analyze_run.py --runs-root logs/manual_scene_runs/scene_ppt_medium --output-dir logs/manual_scene_runs/scene_ppt_medium/analysis --task-label S2-04-medium
```

```bash
PROJECT_ROOT="$(git rev-parse --show-toplevel)"
cd "$PROJECT_ROOT"
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/s204-mpl FONTCONFIG_PATH=/tmp/s204-fontconfig rtk proxy /usr/bin/python3 scripts/s1_analyze_run.py --runs-root logs/manual_scene_runs/scene_ppt_dense --output-dir logs/manual_scene_runs/scene_ppt_dense/analysis --task-label S2-04-dense
```

```bash
PROJECT_ROOT="$(git rev-parse --show-toplevel)"
cd "$PROJECT_ROOT"
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/s204-mpl FONTCONFIG_PATH=/tmp/s204-fontconfig rtk proxy /usr/bin/python3 scripts/s1_analyze_run.py --runs-root logs/manual_scene_runs/scene_obstacle --output-dir logs/manual_scene_runs/scene_obstacle/analysis --task-label S2-04-obstacle
```

结果表含 scene 与 scene ID；无效时钟、来源错误、终止帧不完整或 coverage 缺口按原分析合同保留，不插值。单次非正式诊断用于核对运行与记录闭环，不代表跨场景可达性或性能结论。

## S2-08 修订地图的单次诊断参数

S2-08 对修改后的 `scene_random_01.xml`、`scene_random_02.xml` 和 `scene_random_03.xml` 各执行一次 A 组非正式诊断：E=-0.05，目标 `(7,0)`，到达半径 0.5 m，运动上限 20 仿真秒、墙钟上限 120 秒，到达后固定站立观察 5 仿真秒。实际命令、run ID、A 配置消费、模型与场景绑定记录见 S2-08 run_invocations.json（原始运行/环境明细保留本地，未纳入分享提交）；三次结果对照见 [S2-08 scene_comparison.md](../S2-08/diagnostic_runs_20261011/scene_comparison.md)。这些单次观察不构成成功率或正式性能结论。

后续对当前布局做单次手动诊断时，场景目录仍分开保存，命令示例如下；对 02/03 替换场景文件名和输出目录。它采用任务诊断上限，与上面通用示例中的 8 秒/90 秒设置不同。

```bash
PROJECT_ROOT="$(git rev-parse --show-toplevel)"
cd "$PROJECT_ROOT"
rtk proxy /usr/bin/python3 scripts/s1_run_diagnostic.py --scene scene_random_01.xml --goal-x 7.0 --sim-limit-s 20 --wall-cap-s 120 --standing-observation-sim-s 5 --standing-observation-wall-cap-s 30 --overlay "$PROJECT_ROOT/quadruped_ros2_control_humble/install/go2_description" --output-root logs/manual_scene_runs/scene_random_01_s208 --display "$DISPLAY" --xauthority "$XAUTHORITY"
```

## S2-05 接触结果的读取方式

当前记录版本区分三种结果：足端轻微障碍接触会记录但不停止；具名 FL、FR、RL、RR 足端球满足 Fxy > 2*abs(Fz)+10 N 时记录为足端撞击但仍不停止；大腿、小腿、机身等非足端与障碍碰撞继续终止。未命名碰撞几何的部位从事件中的 body 名称读取。平地接触不属于障碍接触。

S2-05 的有界稀疏场景诊断命令为：

~~~bash
PROJECT_ROOT="$(git rev-parse --show-toplevel)"
cd "$PROJECT_ROOT"
rtk proxy /usr/bin/python3 scripts/s1_run_diagnostic.py --scene scene_ppt_sparse.xml --goal-x 7.0 --sim-limit-s 10 --wall-cap-s 120 --overlay "$PROJECT_ROOT/quadruped_ros2_control_humble/install/go2_description" --output-root logs/manual_scene_runs/scene_ppt_sparse --display "$DISPLAY" --xauthority "$XAUTHORITY"
~~~

逐 run 的 metrics.json 与 run_results.json 分列 navigation_terminal_result、nonfoot_collision_failure、foot contact/impact 次数和持续时间、到达时是否出现足端接触/撞击，以及 abs_reference_collision。ARRIVED 只表示到达目标半径，不自动表示无足端撞击；ABS 参考碰撞可以同时为 true。缺少有效力、物理覆盖或完整事件历史时，对应结论显示 UNKNOWN。v3/v4 历史记录保留旧碰撞字段语义，没有足端力与事件字段，不能补判历史足端撞击。

## S2-07 正常到达收尾

当前 supervisor 仅在有效 ARRIVED（或健康诊断时限）终局、scene-bound PhysicsLoop 快照仍为 LIVE，且无安全故障、碰撞、未分类接触或跌倒候选/确认时，才会发送正常收尾命令。运动终局快照先封口；减速、站立、下降和清理记录保存在 `run_context.json`，不计入运动指标。

controller 以实测状态确认策略速度稳定、FIXEDSTAND 关节位置/速度与机身状态持续稳定后才进入 FIXEDDOWN；以同类姿态、关节和支撑数据确认趴下后才转 PASSIVE，并读取 controller 命令接口增益/力矩为零。硬停优先于所有正常阶段；保护事件或阶段失败时不继续趴下，supervisor 改发 command 9。日志中的 `RELEASED` 只证明 controller 输出接口读回为零，不证明 MuJoCo 消费该命令。

S2-07 的一次平地诊断在确认站稳后开始下降，但下降过程触发了 PhysicsLoop 安全保护，之后按 command 9 转 PASSIVE；未确认趴下或正常卸力。此情况说明基础运行命令可执行，不表示正常趴下闭环已经通过。普通手动诊断仍写入 `logs/manual_scene_runs/<scene-name>` 独立场景目录；收尾阶段和异常原因需连同 run_context 与 terminal 一起复核。
