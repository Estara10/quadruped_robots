# S2-08 执行报告：随机场景增加直线阻挡障碍

**最新状态：COMPLETE（Director 有条件通过）。** 通过范围为四张地图有直线阻挡、五张地图保留绕行通道；布局、预览、XML 与场景身份同步；三张修改地图各一次有效到达及侧向绕行观察。此结论不扩展为正式性能或无碰撞结论。本报告保留 S2-08-r1 几何/生产场景核对、三次各自唯一的非正式 A 组诊断及全部历史限制。没有启动 S3，没有训练、正式实验或实机测试，也没有提交 Git。

## 做了什么

- 先将旧 `layouts.json`、五张旧预览、五份旧 XML 和旧 `abs_scene_catalog.def` 保存到 [`previous_layouts/`](previous_layouts/)。后续历史记录仍绑定它们自己的旧场景身份，没有套用新坐标。
- 保留原 seed 3101–3105，只作确定性手工移动：场景01障碍01从原位置移至 `(2.75,0)`；场景02障碍03移至 `(2.80,0)`；场景03障碍04移至 `(3.00,0)`。形状、尺寸和高度不变，没有重新随机。布局版本记为 `S2-08-r1`。场景04/05 JSON 的障碍坐标未修改，04 原有 y 下界越界 0.056375 m 保留。
- 从新版 JSON 生成五张带标记的顶视预览和五份 XML。预览区分起点到目标的有限线段、直线阻挡物和 1.10 m 折线保留通道，并标明该通道仅用于几何约束、不作为机器人路径输入。
- 更新场景01/02/03 的 root、closure 和编译模型身份；场景04/05 的 XML 与旧文件逐字节一致。ray2d writer 与碰撞 authority 仍调用相同的 `abs_scene::selectObstacleGeoms()`（`unitree_sdk2_bridge.h` 与 `obstacle_collision_authority.h`）。
- 生成工具代码只改了预览/XML 可重复生成、直线阻挡和形状重叠核验；没有改切换逻辑、控制器、模型、动作、阈值或安全规则。

## 几何及生产加载核验

阻挡规则要求障碍的真实水平投影与从 `(0,0)` 到 `(7,0)` 的有限线段有正长度交集；切点不算。保留通道按 `layouts.json` 的折线中心节点核验，并要求障碍保守外接圆到中心线的最小净距大于 0.55 m。

| 场景 | 直线相交障碍 | 与线段交叠的 x 区间（m） | 折线保留通道最小边缘净距（m） | 起点/目标最近障碍净距（m） |
|---|---|---:|---:|---:|
| 01 | 圆柱 `ppt_obstacle_01` | `[2.489, 3.011]` | 0.589 | 1.946 / 0.973 |
| 02 | 方箱 `ppt_obstacle_03` | `[2.561, 3.039]` | 0.659 | 1.292 / 0.997 |
| 03 | 方箱 `ppt_obstacle_04` | `[2.585, 3.415]` | 0.698 | 1.444 / 1.051 |
| 04 | 圆柱 `ppt_obstacle_15`（原有） | `[2.556, 3.102]` | 0.650 | 1.356 / 1.156 |
| 05 | 无 | — | 0.663 | 1.257 / 1.170 |

五图均无障碍实体重叠；场景04障碍14仍超出声明 y 下界 0.056375 m，没有借本任务改动。逐障碍结果见 [`layout_geometry.json`](layout_geometry.json)。`scripts/generate_random_static_scenes.py --check` 确认五份 XML 和预览与 JSON 一致；场景目录身份检查（原始运行/环境明细保留本地，未纳入分享提交）通过。

`unitree_mujoco` 和 `s2_06_scene_probe` 增量构建通过。C++ MuJoCo 生产探针实际加载五个随机场景，报告的 obstacle count 分别为 15/17/18/19/20，编译模型指纹与更新后场景目录一致，且地面未进入障碍集合。原始探针输出见 `mujoco_scene_probe.txt`（原始运行/环境明细保留本地，未纳入分享提交），构建和生成核验分别见 `incremental_build.txt`（原始运行/环境明细保留本地，未纳入分享提交） 与 `layout_xml_preview_check.txt`（原始运行/环境明细保留本地，未纳入分享提交）。

## A 组运行设置与来源

运行前确认无相关项目仿真、controller、RViz 或采集进程。隐藏 16×16 GLFW 预检使用原有 `the active DISPLAY value` 和 `current desktop-session authorization file`，X11 与窗口创建均通过；renderer 为 `Mesa hardware renderer`，OpenGL 4.6。预检窗口、二进制和临时日志均已清理，没有修改桌面、授权文件、驱动或系统设置。证据见 `display_preflight.json`（原始运行/环境明细保留本地，未纳入分享提交）。

三次 run 使用同一实际 overlay 配置和模型：group A、candidate `paper_faithful_switch`、entry/有效 exit threshold `-0.05`、hysteresis=false、hold=false、goal `(7,0)`、到达半径 0.5 m；运动上限 20 仿真秒、wall cap 120 秒、健康到达后 FIXEDSTAND 观察目标 5 仿真秒（独立墙钟 cap 30 秒）。运行期间未热更新参数。controller 初始化前后记录验证实际 A 配置和唯一 controller manager writer；每个 run 的 source event 绑定候选、阈值、目标、session，碰撞 reader 为新 scene binding 的 LIVE 状态。

统一模型路径为：

- Agile：`quadruped_ros2_control_humble/install/go2_description/share/go2_description/config/abs/policy.pt`
- RA：`quadruped_ros2_control_humble/install/go2_description/share/go2_description/config/abs/ra_value.pt`
- Recovery：`quadruped_ros2_control_humble/install/go2_description/share/go2_description/config/rec/policy.pt`

完整实际命令、session、scene binding、配置验证、模型路径与进程事实见 `run_invocations.json`（原始运行/环境明细保留本地，未纳入分享提交）。控制器仍只接收目标和状态/11条射线观测；`path_waypoints_m` 未传给控制器。

## 三次诊断结果

| 场景 / run ID | 终局与距目标（m） | 经过时间 / 有效累计时间（s） | 有效片段路径 / 平均速度 | 模式切换（全部 / 有效时间） | Recovery | RA 范围；策略频率 | 站立保持 |
|---|---|---|---|---|---|---|---|
| 01 / `[local run ID omitted]` | VALID / ARRIVED；0.4987 | 4.438 / 4.278 | 6.354 m / 1.485 m/s | 4 / 4；A→R 2，R→A 2 | 两段，合计 0.368 s | `[-0.9995, 0.1617]`；50.450 Hz | FIXEDSTAND 5.012 s，完成 |
| 02 / `[local run ID omitted]` | VALID / ARRIVED；0.4833 | 3.840 / 3.768 | 6.522 m / 1.731 m/s | 4 / 4；A→R 2，R→A 2 | 两段，合计 0.120 s | `[-0.9992, 0.1580]`；50.015 Hz | FIXEDSTAND 5.006 s，完成 |
| 03 / `[local run ID omitted]` | VALID / ARRIVED；0.4921 | 3.820 / 3.820 | 6.848 m / 1.793 m/s | 0 / 0 | 无；本次 RA 范围未触发 | `[-0.9993, -0.3867]`；50.022 Hz | FIXEDSTAND 5.010 s，完成 |

经过时间从实际运动起点算至 ARRIVED 周期；有效累计时间、路径和速度只累加连续、时钟有效的区间，终止周期包括在内，清理/站立观察不计入。RA 范围只取同一 session、运动窗口内 LIVE 且时钟有效的有限值；运动有效 RA 样本数分别为 219/223、191/193、192/192。全部切换数独立于计时有效性；这三次切换的仿真时间均有效。场景01/02的 0.5 秒短反向切换指标为2/3，场景03为0；该指标按自身 0.5 秒窗口定义，不等于另一种切换性能结论。

三条实际 base trajectory 在阻挡物 x 区间的机身 y 观测分别为 `[0.5188,0.5412]`、`[0.6318,0.6863]`、`[0.8020,0.8992]` m；均表现为向正 y 侧绕行后 ARRIVED。该单次轨迹观察表明本次运行到达时确有侧向绕行，但不形成成功率或稳定避障结论。完整对照见 [`scene_comparison.md`](diagnostic_runs_20261011/scene_comparison.md) 和 `scene_comparison.json`（原始运行/环境明细保留本地，未纳入分享提交）。每个 run 保存原始 `runtime_record.jsonl`、`terminal.json`、`run_context.json`、`process_facts.json`、分析 `metrics.json`、`timeline.png` 和实际 MuJoCo framebuffer `standing_hold.ppm`。

## 接触、跌倒及数据边界

- 三次分析均没有记录足端障碍接触/撞击或非足端碰撞终局，也没有 collision event；但 collision authority 没提供可支持整个运动窗口“无碰撞”的完整 episode 覆盖边界。因此这些否定结论为 **UNKNOWN**，不是“无碰撞”。
- 跌倒 tracker 在每次运动开始前均有两段已确认历史：01 为 `[0.250,19.962]` 与 `[21.212,21.810] s`；02 为 `[0.250,20.428]` 与 `[21.678,22.276] s`；03 为 `[0.250,19.196]` 与 `[20.446,21.038] s`。scene_random_01 运动期另有 `[23.814,23.842] s` 的未确认候选，未以 FALL_TERMINATION 结束；02/03 的运动区间无 fall history 项。最终物理样本三个 run 的 fall candidate/confirmed 都为 false。报告如实保留 tracker 记录，不据此声称全 run 无跌倒。
- invalid clock frame 数为 5/2/0，运动窗口策略周期缺口均为 0。01/02 的运动经过时间因此大于有效连续累计时间；所有时间/速度均按分析器有效区间口径解释。
- 每次正常到达后均确认停止导航并在 FIXEDSTAND 保持支撑约5仿真秒；没有自动趴下或卸力请求。process facts 记录 MuJoCo 已退出后再关闭 ROS/controller，二者 exit code 均为0、无强制终止、`shutdown_complete=true`、`closeout_complete=true`。prepared_stop_publisher 退出码为1（非强制终止），该事实保留。每次运行后及最终检查均未发现相关项目进程残留。

## 环境与剩余限制

本轮三次诊断全部有效且到达；合法失败规则没有触发，不因结果追加运行。每张图仅一次运行，不能据此给出成功率、难度排序或 A 组避障性能结论。场景04/05没有重跑；05仍不阻挡直线；场景04边界备注保留。碰撞全覆盖不足；01有运动期未确认跌倒候选；真实碰撞强度、命令是否由 MuJoCo 消费、长时稳定性和正式配对实验等既有 UNKNOWN 继续保留。

三次运行后 MuJoCo、ROS launch/controller、prepared stop publisher 均退出，预检窗口和临时文件已关闭/删除；没有遗留任务环境。碰撞覆盖不完整，不能据此声称无碰撞；姿态异常历史、停止发布器退出码1及所有历史数据和其他限制均保留。当前没有已授权的 Active Task，S3-01 仍为 DRAFT。后续 S3-01 使用 `S2-08-r1` 布局并现场取得同代码 A/B 对照；旧布局运行不能充当新地图对照。
