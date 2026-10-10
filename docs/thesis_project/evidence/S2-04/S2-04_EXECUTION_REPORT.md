# S2-04 Execution Report

**最新状态：COMPLETE — Director 有条件通过。** 通过范围是场景选择、场景身份贯通、生产路径模型/障碍集合核对，以及各一次平地和稀疏短诊断的记录闭环；不代表正式场景性能实验。本报告以下保留原始执行证据和当时的限制。

## 完成内容

- 在 `scripts/s1_run_diagnostic.py` 增加 `--scene`，默认仍为 `scene_obstacle.xml`。未知或缺失场景在启动 RL 前拒绝；运行命令将同一文件名传给 MuJoCo。
- 新增共用场景目录 `common/abs_scene_catalog.def`。C++ 与 Python 都读取该目录，包含五个支持文件、scenario ID、障碍数量及根 XML / closure / MuJoCo 模型指纹。
- 新增 `common/abs_scene_catalog.h`，集中选择障碍几何：原固定场景使用具名之外的七个静态几何签名；PPT 场景只选 `ppt_obstacle_*`；平地允许零障碍。floor、机器人 group 2/3、plane/hfield/mesh 和其他非障碍几何不会进入障碍集合；未识别的静态 world collision geom 会拒绝 authority。
- MuJoCo 几何 ray writer 与碰撞 authority 共用同一选择函数。平地不把 floor 纳入射线或障碍碰撞；ray/collision authority 绑定场景 ID、root、closure 和当前编译模型指纹。
- `scripts/abs_scene.py` 在启动前核对支持列表、root 和 include/asset closure。运行上下文保存所选文件绝对路径、ID、数量、hash/fingerprint 和模型路径；MuJoCo 启动日志写出加载 authority 身份，Python reader 必须收到同一绑定的 LIVE snapshot 才允许启动 controller。
- `RunRecordRecorder` 在新 run 的 meta 中保存场景绑定，快照 reader、终止事实和分析器按绑定核验。没有新字段的历史记录仍按旧 obstacle_test1 约定分析；分析器结果表加入 scene / scene ID，时间线标题显示场景。
- 保留 S2-03 预先就绪的 PASSIVE 发布器、硬停止确认、运动终局与 cleanup 分离和原有进程清理流程；未改 A 规则、模型或配置。

## 定向核验与构建

- `unitree_mujoco/simulate/build2` 中 `unitree_mujoco` 和新增 `s2_04_scene_probe` 均构建成功。探针实际加载五个 MuJoCo XML、计算 production model fingerprint、调用生产障碍选择函数，并用唯一临时共享内存调用生产 `ObstacleCollisionAuthority::publish()`。
- 探针逐场景核对结果：固定场景 7、平地 0、稀疏 4、中等 7、密集 11 个障碍。所有 authority 均为有效；平地发布 4 个 ground contact、0 个 robot-obstacle contact。探针输出见 `single_runs_20261010/scene_probe_output.txt`，源代码见 `unitree_mujoco/simulate/test/s2_04_scene_probe.cpp`。唯一共享内存对象由探针结束时 unlink。
- `/usr/bin/python3 -m py_compile` 对本轮修改的 scene reader、collision reader、recorder、run script 和 analyzer 通过；`scripts/test_run_record.py` 为 54/54 通过。把一份既有 S2-03 运行复制到 `/tmp` 后用更新分析器处理，仍为 VALID / ARRIVED，证明历史记录没有因新 meta 约定失效。
- `--scene unsupported.xml` 在目录创建、MuJoCo 和 RL 启动前返回 `SCENE_PREFLIGHT_REJECTED`；没有回退默认场景。
- 旧 `test_p1_10_collision_authority.py` 依赖当前 reader 中不存在的 `SNAPSHOT_STRUCT`，因此不作为本任务核验依据，也未改造该旧脚本。生产路径核验由上述 C++ 探针、记录链和实际 run 完成。

## 桌面和启动预检

使用 `DISPLAY=:0` 与 `/run/user/1000/gdm/Xauthority`。`xdpyinfo -display :0` 成功；临时系统 g++ GLFW 探针创建并销毁 16×16 隐藏窗口成功，OpenGL 为 Intel / `Mesa Intel(R) Graphics (RPL-P)` / `4.6 (Compatibility Profile) Mesa 23.2.1-1ubuntu3.1~22.04.4`。预检详情见 `single_runs_20261010/preflight.json`。没有更改显示授权、系统权限、服务、驱动或全局设置。

运行前确认没有相关 controller、MuJoCo、RViz 或任务辅助进程；两次启动的 controller manager 预检也为空。配置实际来自 `/home/lidio/quadruped_robots/quadruped_ros2_control_humble/install/go2_description/share/go2_description/config/abs/config.yaml`：A、entry/exit `-0.05`、hysteresis=false、hold=false、目标 `(7,0)`；候选为 `paper_faithful_switch`，到达半径 `0.5 m`，运动上限 8 仿真秒、墙钟保护 90 秒。Agile、RA 和 Recovery 模型均从对应 overlay 的预期路径加载。两次 loaned foot-force interface 顺序均记录为 `FR, FL, RR, RL`，与现有转换一致。

## 实际短诊断

| run | 实际场景 / 障碍 | session / writer PID | 终局与运动结果 | 记录与频率 |
|---|---|---|---|---|
| `da57865e43b7425886294b11f6ec5ff6` | `scene_flat.xml` / `flat` / 0 | `23115916554953` / `121632` | `ARRIVED`；距 `(7,0)` 为 `0.4965 m`，进入 `0.5 m` 范围。运动窗口 cycle 49–218；motion-safety 分析为 collision=false、fall=false、coverage complete。全帧观测到最多 3 个 foot-floor contact，障碍 contact 始终为 0。 | 181 个策略帧；碰撞 snapshot 全部 LIVE、unknown contacts=0；1 个无效时钟帧；无周期缺号；policy 约 `50.0039 Hz`。平地 ray 全部为 `log2(6 m)=2.58496`。VALID。 |
| `1900daaf23ca480b8e9401e360989668` | `scene_ppt_sparse.xml` / `ppt_sparse` / 4 | `23177251954220` / `123763` | `COLLISION_TERMINATION`；距目标 `2.2426 m`。运动窗口 cycle 49–183；碰撞事件 step 13459–13462。直接记录机器人 geom 42 与具名障碍 `ppt_obstacle_03` 接触。没有将其改写为地面接触。 | 146 个策略帧；碰撞 snapshot 全部 LIVE、unknown contacts=0；4 个无效时钟帧；无周期缺号；policy 约 `49.9959 Hz`。运动窗口记录 6 次模式转换、Recovery 有效时长占比约 10.25%。VALID。 |

两次 run 的配置、唯一 source/writer/session、模型路径、scene authority、终止帧和分析边界均分别保存在各自目录。稀疏 run 的 ray 值随已选障碍变化（本次记录范围 `log2(distance)=-1.4335…2.58496`）；平地射线为空场最大距离。每次终局后预备 PASSIVE 通道均发布停止请求，controller 记录了 PASSIVE 确认，`shutdown_complete=true`、`closeout_errors=[]`。两次记录各自生成的 `metrics.json`、`timeline.png` 和结果表在对应 `flat_analysis/`、`sparse_analysis/` 目录。

## 证据位置

- 运行前显示、配置、进程和预检：`single_runs_20261010/preflight.json`。
- 五场景 MuJoCo 生产 authority 探针：`single_runs_20261010/scene_probe_output.txt`。
- 平地原始记录及来源：`single_runs_20261010/flat/da57865e43b7425886294b11f6ec5ff6/`。
- 平地结果与图：`single_runs_20261010/flat_analysis/run_results.md`、run 目录中的 `metrics.json` 和 `timeline.png`。
- 稀疏原始记录及来源：`single_runs_20261010/sparse/1900daaf23ca480b8e9401e360989668/`。
- 稀疏结果与图：`single_runs_20261010/sparse_analysis/run_results.md`、run 目录中的 `metrics.json` 和 `timeline.png`。
- 用户命令：本目录的 `SINGLE_RUN_GUIDE.md`。

## 限制与状态

- 只有平地和稀疏候选完成实际策略运行；原固定、中等和密集只由 C++ 实际 MuJoCo 加载及 authority 探针核对，没有运行策略诊断。
- 单次平地到达和稀疏碰撞只证实本次记录链与终局，不说明任何场景的稳定可达性、难度排序或 A 组性能。S2-04 不是正式实验。
- 稀疏 run 的 robot geom 名称为空字符串；本次只能明确到 geom ID 42 和具名 `ppt_obstacle_03`，不推断是哪条腿或碰撞强度。
- 无效时钟帧在分析中保留；其余周期频率由本次记录的单调时钟计算，不用于解释算法差异。Recovery 控制命令的 MuJoCo 消费、跌倒/碰撞规则以外的故障覆盖和其他未知接口仍按既有边界，不由本任务扩展。
- 新的 `scene_obstacle.xml` binding 使用本轮实际解析出的完整 closure `7b9052…`；缺少新 scene meta 的历史记录仍以原历史 closure `6ca5da…` 读取。原始记录未改写。
- Director 有条件通过并将本任务标记 **COMPLETE**。原固定、中等和密集场景尚未实际运行策略；单次平地到达及稀疏碰撞不代表成功率或难度排序。稀疏运行只能确认 robot geom 42 与 `ppt_obstacle_03` 接触，无法确定具体肢体或冲击强度。修改受支持场景 XML 或资源后，必须同步更新场景目录身份。MuJoCo 命令消费等既有 UNKNOWN 保留；历史失败、INVALID 和原始记录未改写。目前没有已授权的下一项 Active Task。

**通俗总结：**现在可以用 `--scene` 选五种已列出的环境；平地脚接触不会被记成障碍碰撞。平地短诊断到达目标，稀疏短诊断真实记录到一个障碍碰撞；两次的记录、分析和 PASSIVE 收尾均通过本轮核对。中等、密集还只核对了模型和障碍集合，没有跑策略。仿真、controller、停止发布器、探针和 RViz 均已核对退出。
