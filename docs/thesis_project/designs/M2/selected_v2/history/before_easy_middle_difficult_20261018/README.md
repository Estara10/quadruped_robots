# M2新版18图开发集

用户于2026-10-18确认选图方案：保留原M2 dev02–06，并将选中的13张M1开发图复制为M2别名。当前入口为[18图清单](MAP_INDEX.md)、[机器可读索引](scene_index.csv)、[布局](layouts.json)和[18图总览](overview_18.png)。

## 当前编号与来源

为了避免再次改变用户熟悉的M2 dev02–06，保留这些名称。新增13图使用m2dev07–19，总计18图，**当前入选范围是m2dev02–19，不是m2dev01–18**。m2dev01仍可用于历史复核，但不在当前选图清单中，没有删除。

| M2文件 | 来源 |
|---|---|
| m2dev02–06.xml | 原M2相同编号；04使用当前r5 |
| m2dev07.xml | M1 dev03 |
| m2dev08.xml | M1 dev04 |
| m2dev09.xml | M1 dev08 |
| m2dev10.xml | M1 dev11 |
| m2dev11.xml | M1 dev19 |
| m2dev12.xml | M1 dev24 |
| m2dev13.xml | M1 dev13 |
| m2dev14.xml | M1 dev17 |
| m2dev15.xml | M1 dev26 |
| m2dev16.xml | M1 dev29 |
| m2dev17.xml | M1 dev01 |
| m2dev18.xml | M1 dev12 |
| m2dev19.xml | M1 dev22 |

XML位于`unitree_mujoco/unitree_robots/go2/`。新增别名仅改变XML顶层模型名称以区分身份，源障碍几何、机器人模型、起终点和边界不变；M1源XML没有修改或移动。实际生产ID为m2d02–m2d19。

## 分层与解释

- 高风险候选8图：m2dev03、05、07、08、09、10、11、12。
- 修改后待确认1图：m2dev04。不能使用旧几何成绩判定当前r5的风险或阈值区分能力。
- 中风险/切换敏感6图：m2dev02、06、13、14、15、16。
- 低风险参考3图：m2dev17、18、19。

这些是依据历史开发数据作出的候选标签，不是统计上确定的风险等级。高风险候选不保证碰撞由退出阈值引起；后续仍需核对Recovery触发和碰撞的时间关系。M2-04当前物体间距低于旧0.60m候选规则，精确用户编辑保留，未放宽原包络。

复制M1到M2不会产生新的独立地图；今后不能把M1源图和对应M2别名同时当作独立样本。此集经过历史成绩筛选，仅用于开发参数研究，M1留出30图保持不动。

## 接入与运行范围

18张XML均由生产C++ MuJoCo 3.3.3静态加载，指纹与障碍集合登记见[runtime_identities.txt](runtime_identities.txt)。场景身份已同步到公共catalog，生产MuJoCo主程序增量构建成功。13张新增图的worldbody与M1源图一致、源XML摘要未变。静态加载没有推进物理、启动ROS/策略或证明地图可达。本轮不执行216次实验、不修改旧实验计划或用新图覆盖旧批次数据，不提交Git。

手动查看某一张时可使用既有`manual_ab_compare.py --groups B --exit-threshold -0.07 --scene m2dev07.xml ...`；这里只说明入口，没有启动该命令。

复建工具为`scripts/prepare_m2_selected_maps.py`，从当前M1/M2源布局复制，输出当前选择资料；原M2六图生成器不自动管理这18图清单。若用户后续改变源布局，需要显式刷新选择资料和生产身份，并记录版本，不能静默替换已开始的实验地图。
