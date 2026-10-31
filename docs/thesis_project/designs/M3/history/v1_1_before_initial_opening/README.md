# M3：历史筛选挑战地图集（30个XML布局）

版本：`s7-challenge-test/1.1`。保留15张奇数编号地图，替换全部15张偶数编号近似重复图。文件名与手动运行命令不变。

[新版30张总览](overview_30.png)

## 手动观看

```bash
rtk proxy /usr/bin/python3 scripts/manual_map_view.py
```

输入1–30选择地图，输入q退出。默认A组、可见视窗、不截图，收尾确认后删除临时记录。启动时可追加 `--group B`、`--group C` 或 `--group D` 选择组别；B/D使用X=−0.10，C/D使用H=0.20仿真秒。切换组别时先退出当前脚本，等待收尾完成，再用新组别启动。

## 修订与筛选依据

- 奇数01、03、…、29：保留原M3布局；偶数02、04、…、30：直接采用15张不同历史源地图的几何，不再给同一模板制造小扰动副本。
- 候选来自M1开发图与已测过的原M1留出图；剔除已被当前M2中等/困难模板引用的M1别名，避免重复来源。
- 历史依据：M1的90项A诊断、T1的400项扫描、S7第一轮240项逐项结果。仅纳入VALID导航终局，按各组/参数汇总的未到达比例降序，再按未到达数、文件名排序。未按B/C/D相对A胜负选择。
- 历史筛选分母只含VALID导航终局。未到达包括碰撞、确认跌倒终局、仿真限时；准备失败、系统中止、墙钟保护等不计入该筛选比例，仍应在原始记录中保留。未来任何批次均须按其既定规则报告所有计划项和失败，不能沿用这个筛选分母。
- 批次条件和重复数不同；例如3/3、8/8及36/43来自不同样本规模与条件，比例只用于历史候选筛选，不是精确风险概率或可直接排名的真实风险率。历史全部失败图可作压力案例，不预设任何退出机制能解决。
- 这是看过历史结果后的挑战开发/验证集合，包含历史原图复用，不能称为新的独立留出集。旧正式结果不被替换。
- 本版本包含30个XML文件、不同源地图/几何；文件数和几何差异不等于30种独立路线结构或结构家族。替换布局标注了有限的路线结构类型，保留布局不据此推断结构独立性。
- 位置、尺寸、包络和碰撞规则未人为缩小来适配失败地图。所有地图保持参考路线净距至少0.75m、物体间距至少0.50m。几何通路不保证策略可达。

## 位置

本目录与M1、M2并列。运行XML位于 `unitree_mujoco/unitree_robots/go2/challenge01.xml`–`challenge30.xml`。旧M3 v1.0的布局、总览和XML保存在 `history/v1_before_distinct_layouts/`；M1/M2源地图未修改。

## 当前地图

| M3预览 | 源地图 | 本轮处理 | 历史未到达/有效导航 |
|---|---|---|---|
| [challenge01.xml](challenge01.png) | m01.xml | 保留 | — |
| [challenge02.xml](challenge02.png) | holdout09.xml | 替换 | 8/8 |
| [challenge03.xml](challenge03.png) | m02.xml | 保留 | — |
| [challenge04.xml](challenge04.png) | holdout30.xml | 替换 | 8/8 |
| [challenge05.xml](challenge05.png) | m03.xml | 保留 | — |
| [challenge06.xml](challenge06.png) | dev30.xml | 替换 | 3/3 |
| [challenge07.xml](challenge07.png) | m04.xml | 保留 | — |
| [challenge08.xml](challenge08.png) | holdout25.xml | 替换 | 6/7 |
| [challenge09.xml](challenge09.png) | m05.xml | 保留 | — |
| [challenge10.xml](challenge10.png) | dev15.xml | 替换 | 36/43 |
| [challenge11.xml](challenge11.png) | m06.xml | 保留 | — |
| [challenge12.xml](challenge12.png) | holdout27.xml | 替换 | 6/8 |
| [challenge13.xml](challenge13.png) | d01.xml | 保留 | — |
| [challenge14.xml](challenge14.png) | holdout13.xml | 替换 | 5/8 |
| [challenge15.xml](challenge15.png) | d02.xml | 保留 | — |
| [challenge16.xml](challenge16.png) | holdout15.xml | 替换 | 4/8 |
| [challenge17.xml](challenge17.png) | d03.xml | 保留 | — |
| [challenge18.xml](challenge18.png) | dev28.xml | 替换 | 1/3 |
| [challenge19.xml](challenge19.png) | d04.xml | 保留 | — |
| [challenge20.xml](challenge20.png) | holdout03.xml | 替换 | 2/8 |
| [challenge21.xml](challenge21.png) | d05.xml | 保留 | — |
| [challenge22.xml](challenge22.png) | holdout16.xml | 替换 | 2/8 |
| [challenge23.xml](challenge23.png) | d06.xml | 保留 | — |
| [challenge24.xml](challenge24.png) | holdout17.xml | 替换 | 2/8 |
| [challenge25.xml](challenge25.png) | d07.xml | 保留 | — |
| [challenge26.xml](challenge26.png) | holdout26.xml | 替换 | 2/8 |
| [challenge27.xml](challenge27.png) | d08.xml | 保留 | — |
| [challenge28.xml](challenge28.png) | holdout07.xml | 替换 | 1/8 |
| [challenge29.xml](challenge29.png) | d09.xml | 保留 | — |
| [challenge30.xml](challenge30.png) | holdout14.xml | 替换 | 1/8 |

## 运行状态

本版本策略运行数为0。用户对v1.0重复布局的目视反馈不代表v1.1已获人工布局确认；v1.1的用户布局审阅状态待确认。文件名中的“challenge/test”是历史标识，不表示独立留出集。
