# M3：历史筛选挑战地图集（30个XML布局）

版本：`s7-challenge-test/1.4`。最新仅在04图红色03号与GOAL之间增加11号圆柱，位置(5.8,0.3)m，直径0.40m、高0.50m。左侧通路与前2.5m通行段保留，参考路线重新计算；其余29张不变。当前文件名为M3_01–M3_30.xml；旧challenge名称仍兼容。

[04最新新增障碍对照](comparison_04_center_obstacle.png) · [04此前左侧通道对照](comparison_04_left_opening.png) · [02/04/06此前修改前后对照](comparison_02_04_06_initial_opening.png) · [具体修改及静态射线说明](INITIAL_OPENING_REVISION.md)

[新版30张总览](overview_30.png)

## 手动观看

```bash
rtk proxy /usr/bin/python3 scripts/manual_map_view.py
```

输入1–30选择M3地图，或输入M3编号文件名；旧challenge名称仍兼容，输入q退出。默认A组、可见视窗、不截图，收尾确认后删除临时记录。启动时可追加 `--group B`、`--group C` 或 `--group D` 选择组别；B/D使用X=−0.10，C/D使用H=0.20仿真秒。切换组别时先退出当前脚本，等待收尾完成，再用新组别启动。

## 修订与筛选依据

- 奇数01、03、…、29：保留原M3布局；偶数02、04、…、30：以15张不同历史源地图替换近似副本；其中02/04/06在v1.2又按用户反馈调整了位置，已不等同于源几何。
- 候选来自M1开发图与已测过的原M1留出图；剔除已被当前M2中等/困难模板引用的M1别名，避免重复来源。
- 历史依据：M1的90项A诊断、T1的400项扫描、S7第一轮240项逐项结果。仅纳入VALID导航终局，按各组/参数汇总的未到达比例降序，再按未到达数、文件名排序。未按B/C/D相对A胜负选择。
- 历史筛选分母只含VALID导航终局。未到达包括碰撞、确认跌倒终局、仿真限时；准备失败、系统中止、墙钟保护等不计入该筛选比例，仍应在原始记录中保留。未来任何批次均须按其既定规则报告所有计划项和失败，不能沿用这个筛选分母。
- 批次条件和重复数不同；例如3/3、8/8及36/43来自不同样本规模与条件，比例只用于历史候选筛选，不是精确风险概率或可直接排名的真实风险率。历史全部失败图可作压力案例，不预设任何退出机制能解决。
- 这是看过历史结果后的挑战开发/验证集合，包含历史原图复用，不能称为新的独立留出集。旧正式结果不被替换。
- 本版本包含30个XML文件、不同源地图/几何；文件数和几何差异不等于30种独立路线结构或结构家族。替换布局标注了有限的路线结构类型，保留布局不据此推断结构独立性。
- 02/04/06前向开口经修订，04后续调整左侧通道并新增圆柱。当前版本为v1.4；包络和碰撞规则未放宽，参考路线净距至少0.75m、物体间距至少0.50m。几何通路不保证策略可达。

## 位置

本目录与M1、M2并列。运行XML位于 `unitree_mujoco/unitree_robots/go2/M3_01.xml`–`M3_30.xml`。完整旧名对照见[地图文件名对照](../MAP_FILENAME_CROSSWALK.md)。旧M3 v1.0的布局、总览和XML保存在 `history/v1_before_distinct_layouts/`；M1/M2源地图未修改。

## 当前地图

| M3预览 | 源地图 | 本轮处理 | 历史未到达/有效导航 |
|---|---|---|---|
| [M3_01.xml](challenge01.png) | M2_23.xml | 保留 | — |
| [M3_02.xml](challenge02.png) | M1_39.xml | v1.2前向开口修订 | 8/8（源图历史） |
| [M3_03.xml](challenge03.png) | M2_24.xml | 保留 | — |
| [M3_04.xml](challenge04.png) | M1_60.xml | v1.4左侧通道及新增圆柱 | 8/8（源图历史） |
| [M3_05.xml](challenge05.png) | M2_25.xml | 保留 | — |
| [M3_06.xml](challenge06.png) | M1_30.xml | v1.2前向开口修订 | 3/3（源图历史） |
| [M3_07.xml](challenge07.png) | M2_26.xml | 保留 | — |
| [M3_08.xml](challenge08.png) | M1_55.xml | 替换 | 6/7 |
| [M3_09.xml](challenge09.png) | M2_27.xml | 保留 | — |
| [M3_10.xml](challenge10.png) | M1_15.xml | 替换 | 36/43 |
| [M3_11.xml](challenge11.png) | M2_28.xml | 保留 | — |
| [M3_12.xml](challenge12.png) | M1_57.xml | 替换 | 6/8 |
| [M3_13.xml](challenge13.png) | M2_29.xml | 保留 | — |
| [M3_14.xml](challenge14.png) | M1_43.xml | 替换 | 5/8 |
| [M3_15.xml](challenge15.png) | M2_30.xml | 保留 | — |
| [M3_16.xml](challenge16.png) | M1_45.xml | 替换 | 4/8 |
| [M3_17.xml](challenge17.png) | M2_31.xml | 保留 | — |
| [M3_18.xml](challenge18.png) | M1_28.xml | 替换 | 1/3 |
| [M3_19.xml](challenge19.png) | M2_32.xml | 保留 | — |
| [M3_20.xml](challenge20.png) | M1_33.xml | 替换 | 2/8 |
| [M3_21.xml](challenge21.png) | M2_33.xml | 保留 | — |
| [M3_22.xml](challenge22.png) | M1_46.xml | 替换 | 2/8 |
| [M3_23.xml](challenge23.png) | M2_34.xml | 保留 | — |
| [M3_24.xml](challenge24.png) | M1_47.xml | 替换 | 2/8 |
| [M3_25.xml](challenge25.png) | M2_35.xml | 保留 | — |
| [M3_26.xml](challenge26.png) | M1_56.xml | 替换 | 2/8 |
| [M3_27.xml](challenge27.png) | M2_36.xml | 保留 | — |
| [M3_28.xml](challenge28.png) | M1_37.xml | 替换 | 1/8 |
| [M3_29.xml](challenge29.png) | M2_37.xml | 保留 | — |
| [M3_30.xml](challenge30.png) | M1_44.xml | 替换 | 1/8 |

## 运行状态

用户已审看并接受v1.4全部30张图，随后240项A/B/C/D比较已完成：165项确认安全完成、75项确认未完成。结果见[最终报告](../../evidence/S7-02/M3_ABCD_240_FINAL_REPORT_20261023.md)和[索引](../../evidence/S7-02/M3_ABCD_240_RESULT_INDEX.md)。当前不继续按成绩改图或追加运行；下一步只规划[碰撞和超时分析](../R1/PLAN.md)。文件名中的challenge/test不代表独立留出集。
