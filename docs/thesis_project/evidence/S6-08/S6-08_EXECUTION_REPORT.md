# S6-08 执行报告：随机静态障碍地图生成与通路核验

**最新状态：COMPLETE（Director 有条件通过）。** 通过范围为可复现生成器、六张开发样例、几何检查、生产场景接入及两次短诊断。历史运行数据和历史结论保留；本轮没有生成或冻结最终 60 张图库，也没有开始参数比较、训练、正式实验或实机测试。

保留限制：密度档位有重叠；保守通行包络未经过步态实测；诊断有策略周期/仿真时钟缺口；每个运行场景仅一次，不能支持成功率或性能结论。当前没有已授权的 Active Task。

## 结果摘要

- 稀疏、中等、密集地图各生成两张，共六张；生成器版本 `s6-08-mapgen/1.0`，种子 608001–608006。相同种子可复现相同障碍布局、MJCF、预览和通路数据。CLI 支持按密度增加样例数（例如未来每类 20 张），但本轮实际只生成六张。
- 六张均在起点 `(0,0)` 与目标 `(7,0)` 的直线上放置了两到三处障碍，同时保留一条通过几何检查的绕行参考路线。路线只供检查和预览，未作为策略输入。
- 六张 MJCF 均通过 MuJoCo 编译和生产场景探针。保留的五张旧随机地图也在同一探针中通过；未覆盖旧场景或历史记录。
- 一次中等样例和一次密集样例的 A 组短诊断均为 `VALID / ARRIVED`，都完成到达后的约 5 仿真秒站立观察。中等样例出现足端接触/撞击及多次模式切换；密集样例没有 Recovery。各场景仅一次运行，不作成功率、难度排序、性能比较或策略可达性结论。

## 六张开发样例

边界是障碍放置区域 `x=[-0.5,7.5] m, y=[-3,3] m`，不是物理墙。参考路径搜索区域为 `x=[0,7] m, y=[-2.9,2.9] m`。占地比例以障碍物二维投影面积除以放置区域面积计算。密度类别以占地比例档位（稀疏 5–9%、中等 8.5–14%、密集 14–22%）和配置最小障碍间距（依次 0.28/0.12/0.06 m）区分；占地档位在 8.5–9% 重叠，三档都另要求参考路线中心线包络宽度至少 1.50 m。表中“中心线包络宽度”定义为参考路径沿线最小障碍净距的两倍；它不是两侧障碍之间的实际间距。表中“超出包络+余量要求的额外宽度”是相对既定 0.65 m 机器人包络半径加 0.10 m 工程余量（中心线所需净距 0.75 m，即宽度 1.50 m）的额外裕度；密集图的 4–6 mm 不是实际通道宽度。该指标说明保存的路径中心线满足所用静态包络阈值，不证明真实步态可通过；该保守包络未经过步态实测验证。

| 场景 | 种子 | 障碍数 | 投影占地比例 | 最小障碍间距 | 中心线包络宽度 | 超出包络+余量要求的额外宽度 | 直线阻挡数 | 预览 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| `s6_08_sparse_01` | 608001 | 8 | 5.26% | 0.327 m | 1.520 m | 0.0195 m | 2 | [预览 PNG](../../designs/S6-08/s6_08_sparse_01.png) |
| `s6_08_sparse_02` | 608002 | 8 | 5.06% | 0.337 m | 1.533 m | 0.0332 m | 2 | [预览 PNG](../../designs/S6-08/s6_08_sparse_02.png) |
| `s6_08_medium_01` | 608003 | 12 | 8.99% | 0.148 m | 1.521 m | 0.0207 m | 3 | [预览 PNG](../../designs/S6-08/s6_08_medium_01.png) |
| `s6_08_medium_02` | 608004 | 12 | 8.67% | 0.165 m | 1.517 m | 0.0171 m | 2 | [预览 PNG](../../designs/S6-08/s6_08_medium_02.png) |
| `s6_08_dense_01` | 608005 | 14 | 14.58% | 0.106 m | 1.504 m | 0.0042 m | 3 | [预览 PNG](../../designs/S6-08/s6_08_dense_01.png) |
| `s6_08_dense_02` | 608006 | 14 | 14.66% | 0.064 m | 1.506 m | 0.0059 m | 3 | [预览 PNG](../../designs/S6-08/s6_08_dense_02.png) |

完整障碍名称、形状、坐标、尺寸、朝向、路径坐标及逐场景身份见 [生成布局数据](../../designs/S6-08/generated_scenes.json)。XML 分别位于 `unitree_mujoco/unitree_robots/go2/scene_s6_08_*.xml`。橙色虚线表示起终点直线，红色实体阻挡直线，青色带状区为参考路线及 Go2 包络/余量，图中标注了坐标边界和 m/px 比例。预览中的 4–6 mm 数值表示超过包络和工程余量要求的额外宽度，不表示实际两侧障碍间距或实际通道宽度。

## 几何方法与依据

障碍按随机化三次曲线两侧分布，方法参考留存 ABS `testbed.py` 的随机路线和两侧安全距离放置思路；当前实现保留 Go2 场景单位和固定场地条件。形状由箱体、圆柱和旋转长条组成；旋转箱体使用真实二维旋转矩形，圆柱使用圆形轮廓。障碍高度 0.32–0.62 m；当前生产 ray2d 将障碍投影到 xy 平面计算箱/圆交点，并不按 z 高度裁剪，因此样例的障碍投影与射线障碍集合一致。场地保持平面；没有改变物理参数、摩擦或机器人模型。

Go2 规划包络半径采用 0.65 m：依据 `go2.xml` 中约 0.1934 m 的髋部水平偏置、两段各约 0.213 m 的腿段、约 0.027 m 的足端球半径，覆盖腿部近似伸直后的水平范围，并超过约 0.335 m 的机身碰撞横向范围。另加 0.10 m 工程余量，中心点到未膨胀障碍的要求净距为 0.75 m。该圆形包络是保守的静态几何假设，不是由步态实测验证的机器人足迹。

路径先在 0.05 m 网格上以 `0.75 m + 半个网格对角线` 的障碍膨胀做 8 邻接 A*；再作直线段简化，并以实际旋转矩形/圆形距离、每不超过 0.025 m 的采样复核全路径。生成最多尝试 96 条随机路线，每个障碍位点最多抽样 1400 次；失败会带最后拒绝原因停止，不无限循环。无路拒绝由 `grid_path()` 实际针对横贯搜索区域的墙体 fixture 核验。六张起终点净空、边界、障碍间距、连通、路线净距及直线阻挡均通过定向检查。

## 生产链接入与核验

- `common/abs_scene_catalog.def` 登记六张场景；`common/abs_scene_catalog.h` 使用由目录内容推导的场景表大小；`scripts/abs_scene.py` 接受新增目录条目。现有 `--scene` 参数可按 XML 文件名选择样例，默认场景不变。
- MuJoCo ray2d 与碰撞权威读取共用 `abs_scene::selectObstacleGeoms()`，射线端和碰撞端都从同一场景登记/名称集合选择障碍；Python reader、记录和分析继续按同一 scene identity 绑定。
- 新构建的 `s2_06_scene_probe` 返回码 0；一次枚举编译核对了原五张随机图和新六张图的 XML、登记数量、障碍名称集合与模型指纹。运行时中等/密集样例的 `collision_reader_scene_binding` 均为 LIVE，登记障碍数和运行模型指纹匹配；MuJoCo authority 日志也确认了实际 scene ID、根文件身份和障碍数。运行时策略帧因此证明新场景走通现有 `--scene`、场景身份与记录分析链。
- 本轮重建 `unitree_mujoco/simulate/build2/unitree_mujoco`，目标 `unitree_mujoco` 构建成功。两次运行上下文记录的实际 `/proc/<pid>/exe` 与该构建路径匹配；controller library 映射为预期安装库，runtime source 状态 confirmed。
- 运行前使用当前 `DISPLAY=:0`、`XAUTHORITY=/run/user/1000/gdm/Xauthority` 创建并立即关闭 16×16 GLFW 隐藏窗口，OpenGL context 成功；vendor `Intel`、renderer `Mesa Intel(R) Graphics (RPL-P)`、OpenGL 4.6 Compatibility / Mesa 23.2.1。未改系统授权、驱动或全局环境。
- 两次均由各自独立的 A overlay 实际加载 `share/go2_description/config/abs/config.yaml`、`config/abs/policy.pt`、`config/abs/ra_value.pt`、`config/rec/policy.pt`；实际配置来源分别是 `logs/manual_abcd_compare/20261013_160736_425b6f/overlays/A/share/go2_description/` 与 `logs/manual_abcd_compare/20261013_160817_aaf32f/overlays/A/share/go2_description/`。配置消费记录均为 group=A、candidate=`paper_faithful_switch`、E=-0.05、有效退出阈值=-0.05、hysteresis=false、hold=false、goal=(7,0)；控制器库路径为 `quadruped_ros2_control_humble/install/rl_quadruped_controller/lib/rl_quadruped_controller/librl_quadruped_controller.so`。overlay 工具核验三个模型和配置均为独立普通文件。
- 种子复现、六图几何属性、路线净距、直线阻挡、无路拒绝和 XML 编译的针对性离线检查通过；生成器、检查器、场景 reader、诊断器及分析器 Python 编译通过。详细输出：[生成器复核](generator_check.txt)、[生产场景探针](production_scene_probe.txt)。
- Python 生产 `resolve_scene()` 对六张 XML 均按 scene ID、障碍数、根文件和 include closure 身份校验成功，记录见 [Python 场景解析结果](python_scene_resolution.txt)。

## 两次 A 组场景接入诊断

两次都使用 `scene_s6_08_*.xml`、A / `paper_faithful_switch`、E=-0.05（有效退出阈值同为 -0.05，无滞回/保持）、目标 `(7,0)`、到达半径 0.5 m、20 仿真秒/120 墙钟秒上限、到达后 5 仿真秒站立观察及既有启动增益、安全规则。没有暂停、重置、改阈值或追加运行。每个 run 均在 RL 前让记录器就绪；首个活跃 cycle 0 完整持久化，身份与运行帧相同周期关联。

| 密度/场景 | Run ID | 结果 | 距目标 | 经过时间 | 有效连续时间 | 有效连续路径 / 平均速度 | 策略频率 | 模式/Recovery | 接触与安全 | 收尾 |
|---|---|---|---:|---:|---:|---|---:|---|---|---|
| 中等 `medium_01` | `95b4c4299d6949b6b3993f0ad14460bd` | VALID / ARRIVED | 0.497 m | 7.620 s | 7.420 s | 6.868 m / 0.926 m/s | 49.976 Hz | 92 条模式边沿（91 条有有效仿真时间）；Agile 295、Recovery 85 周期 | 2 次 FL 在 `ppt_obstacle_02` 的足端接触，均记为足端撞击；无非足端碰撞或跌倒事件；运动窗覆盖完整。ABS 参考碰撞为 true | 约 5 仿真秒站立观察完成；PASSIVE 转换未请求/不适用；MuJoCo、ROS/controller 与会话进程均退出，未强杀 |
| 密集 `dense_01` | `d37f3005476643008c81a2641f1fb9d4` | VALID / ARRIVED | 0.479 m | 4.160 s | 4.058 s | 6.976 m / 1.719 m/s | 49.986 Hz | 0 条切换；0 Recovery 周期 | 未记录接触、撞击、非足端碰撞或跌倒事件；声明运动窗覆盖完整 | 约 5 仿真秒站立观察完成；PASSIVE 转换未请求/不适用；MuJoCo、ROS/controller 与会话进程均退出，未强杀 |

运行原始记录、`run_context.json`、`terminal.json`、`process_facts.json`、`metrics.json` 与 `timeline.png` 位于：

- [中等样例运行目录](../../../../logs/manual_abcd_compare/20261013_160736_425b6f/A/rep_01/95b4c4299d6949b6b3993f0ad14460bd/)
- [密集样例运行目录](../../../../logs/manual_abcd_compare/20261013_160817_aaf32f/A/rep_01/d37f3005476643008c81a2641f1fb9d4/)

中等 run 有 2 个策略周期缺口、4 个无效时钟帧；密集 run 有 2 个周期缺口、1 个无效时钟帧。有效连续路径、速度和驻留时间仅按分析器认可的连续有效区间累计，不能当成全程连续覆盖。中等 run 短反向切换统计覆盖为 `partial_or_unknown`；纳秒级风险到模式决策差值是内部决策时间，不是完整避障响应或命令生效延迟。接口命令写出不证明 MuJoCo 消费。ARRIVED 只代表进入 0.5 m 判据，单次到达与一次碰撞均不构成性能证据。

两次诊断结束后检查本任务相关 MuJoCo、controller manager、ROS launch、RViz、采集器、会话节点及 wrapper 进程均无残留；运行器记录 `shutdown_complete=true`、`closeout_complete=true`、`forced_termination=false`。预检 GLFW 隐藏窗口已在探测进程内销毁。没有触及其他用户进程。

## 改动文件与限制

- 新增 `scripts/generate_s6_08_scenes.py`、`scripts/test_s6_08_geometry.py`、六个 XML、六张 PNG、布局数据与本报告/探针输出。
- 更新 `common/abs_scene_catalog.def`、`common/abs_scene_catalog.h`、`scripts/abs_scene.py` 以登记并解析六个新场景；保留既有五张随机场景身份。
- 更新 `tasks/S6-08.md` 与 `CURRENT_STATE.md` 记录实际进展。没有修改模型、控制算法、A 参数、安全规则或既有场景 XML。

在生成最终 60 张前，需要评估并明确地图净距余量是否应比本批当前保留的 0.10 m 工程余量更宽，复核不同密度的占地、最窄通道分布与场景代表性，确定互斥的最终密度分档、开发/留出分组与唯一场景种子方案，并对生成的完整库重新做生产登记、XML、ray/collision、边界及几何核验。本批 sparse/medium 占地目标档位在 8.5–9% 有轻微重叠，且三类当前都采用相同 1.50 m 路线宽度下限；因此当前类别主要由占地比例和障碍间距分开，实际通道宽度仍逐图量化，最终库应预先确定更清晰的组合分档。六张样例目前都是开发样例，不能进入留出测试集。这里的几何连通不保证策略可达；地图运行每张仅一次，不能推断成功率、算法优劣或密度难度排序。

**通俗总结：**六张可复现地图已生成并接入当前场景链，两次短诊断都到达且站立观察完成；中等图有足端撞击和 Recovery 切换，密集图没有触发 Recovery。Director 对 S6-08 有条件通过。4–6 mm 是超出既定机器人包络与工程余量要求的额外宽度，不是实际通道宽度。周期/时钟缺口、短反向统计覆盖和步态真实通行能力仍有限制；两次仿真及相关 ROS/控制进程都已核对关闭。最终 60 张地图与参数比较尚未开始，目前没有已授权的 Active Task。
