> 历史归档：2026-10-23文档整理前的内容。原位置：`docs/thesis_project/designs/M2/README.md`。仅修正归档后的相对链接；旧计划和状态不代表当前指令。

# M2：第二轮参数开发地图候选

## 当前入口已重命名：easy / middle / difficult（2026-10-18）

用户要求难度命名，当前18图为 **e01–e03.xml、m01–m06.xml、d01–d09.xml**。原M2 03、05、02指定为difficult，分别对应d01、d02、d03；修改后dev04为m01，暂归middle但仍待实测。当前[总览](../../../../../../../docs/thesis_project/designs/M2/selected_v2/overview_18.png)、[来源表与手动命令](../../../../../../../docs/thesis_project/designs/M2/selected_v2/README.md)、[机器索引](../../../../../../../docs/thesis_project/designs/M2/selected_v2/scene_index.csv)均使用新名。

旧M1与M2文件、旧数据保留历史，不与新名重复计入实验。此轮只重命名，不改几何；下方m2dev02–19选图编号及r1–r5记录均为历史来源说明。

## 当前开发选图入口：新版M2共18张（2026-10-18）

用户确认保留M2 dev02–06，并从M1复制13图，当前清单为m2dev02–19共18张。新增07–19与M1源图一一对应，M1源地图及旧M2/实验数据不删除。当前入口：[18图说明与来源](../../../../../../../docs/thesis_project/designs/M2/selected_v2/README.md)、[逐图清单](../../../../../../../docs/thesis_project/designs/M2/selected_v2/MAP_INDEX.md)、[总览](../../../../../../../docs/thesis_project/designs/M2/selected_v2/overview_18.png)。原m2dev01保留历史但不入选本清单。

分层为8张高风险候选、1张修改后待确认的dev04、6张中风险/切换敏感、3张低风险参考；不是把dev04伪称已验证高风险。18图完成生产静态模型身份登记，没有策略运行。本轮仅整理地图，不启动216次扫描；阈值/筛选规则及新矩阵仍需运行前单独确定。下方六图创建和修改记录继续保留，历史结果不变。

## 当前修订r5：02障碍左移0.3m（2026-10-18）

`ppt_obstacle_02`沿初始机器人左侧（世界+Y）移动0.3m，中心(3.858587,−0.207272)→(3.858587,0.092728)m。01保留r4位置(2.362170,0.548765)m，其余障碍不变。当前XML、[预览](../../../../../../../docs/thesis_project/designs/M2/candidate_v1/m2dev04.png)、总览、布局、生成编辑与生产身份同步，修订`m2dev04-r5-20261018`。参考通路净距约0.764m；物体间距仍约0.514m，低于原0.60m候选要求。见[r5记录](../../../../../../../docs/thesis_project/designs/M2/revisions/m2dev04_r5_20261018/CHANGELOG.md)，下方r4及更早位置为历史。

## 当前修订r4：纠正上一轮移动方向（2026-10-18）

用户指出“后移1m”执行方向反了，已撤销r3的−X移动，再从r2位置沿目标方向+X移动1m：首障碍当前中心(2.362170,0.548765)m，其余障碍不变。当前XML、[预览](../../../../../../../docs/thesis_project/designs/M2/candidate_v1/m2dev04.png)、总览、布局、生成器与生产身份已同步，标记`m2dev04-r4-20261018`。

当前参考通路净距约0.764m，已恢复有效参考带；物体最小间距约0.514m，仍低于原0.60m候选规则。下方r3“靠近起点、通路失效”只描述已撤销的历史位置。见[r4记录](../../../../../../../docs/thesis_project/designs/M2/revisions/m2dev04_r4_20261018/CHANGELOG.md)，历史实验仍只对应各自旧几何。

## 当前地图修订r3：M2-04首障碍再后移1m（2026-10-18）

用户在r2基础上要求`ppt_obstacle_01`再后移1m，当前中心为(0.362170,0.548765)m，其余障碍不动。当前XML、[预览](../../../../../../../docs/thesis_project/designs/M2/candidate_v1/m2dev04.png)、布局、生产身份和生成编辑步骤已同步，修订`m2dev04-r3-20261018`。

首障碍已接近起点，现有参考路线最小净距约0.298m，不满足原0.75m包络加余量要求；预览不再绘制失效的可通行参考带。保留用户精确坐标，不据此宣称地图适合参数比较或机器人可以安全起立。静态模型可加载不等于导航验证。详见[r3记录](../../../../../../../docs/thesis_project/designs/M2/revisions/m2dev04_r3_20261018/CHANGELOG.md)；下方r2及r1为历史。

## 当前地图修订：M2-04首障碍位置调整r2（2026-10-18）

按用户要求直接替换`m2dev04.xml`：`ppt_obstacle_01`首次左移0.8m后，再沿初始机器人右侧（世界−Y）移动0.3m、后方（世界−X）移动0.4m。当前中心为(1.362170,0.548765)m。其余障碍不变；[单图预览](../../../../../../../docs/thesis_project/designs/M2/candidate_v1/m2dev04.png)、总览、布局和生产身份同步更新。当前修订`m2dev04-r2-20261018`，见[当前改动记录](../../../../../../../docs/thesis_project/designs/M2/revisions/m2dev04_r2_20261018/CHANGELOG.md)；[r1历史](../../../../../../../docs/thesis_project/designs/M2/revisions/m2dev04_r1_20261018/CHANGELOG.md)保留。

本次调整后01与08间距约0.280m，仍低于原0.60m物体间距规则；参考通路净距仍约0.764m。保留精确用户坐标，不据此宣称策略可达或更难。旧M2及36图批次的dev04成绩只属于旧几何，不能与新图作为相同地图重复混算。下方历史生成与实验状态描述不代表本次修改后的新运行结果。

## 最新续跑停止状态（2026-10-16）

最终139项续跑入口在第6项场景启动前因恢复授权参数未传到生产运行helper而停止，未启动新场景。修正后的入口仅完成离线门核验，没有重新调用。停止后台账为10次总attempt、6次场景启动、4条无场景失败、5项有效、139项未完成；原147额度保留，显式授权的148上限下只剩138次，不足以完成余项。等待Director/用户审阅，不自动续跑。原M2地图、几何及运行记录不变，详情见[T1 M2第二轮执行报告](../../../../../../../docs/thesis_project/evidence/T1/m2_round2/T1_M2_ROUND2_EXECUTION_REPORT.md)。

## 历史M2第二轮状态（第5项补做后、最终续跑前；当前见顶部）

第二轮B组矩阵STOPPED / INCOMPLETE。第5项 attempt_02 为有效ARRIVED，原attempt_01 PREPARATION_FAILURE / INVALID保留。第6项场景未启动：目录分配问题已修复，恢复入口额度断言过时导致一次失败，随后已修正并离线核验但未再次启动。剩余139个计划场景未完成，当前总attempt额度只余138；不再自动重试。5条有效结果均在m2dev06，不能作为阈值排名或性能结论。第5项原准备失败字段仍UNKNOWN。留出集未用于策略运行。完整状态和证据见[T1 M2第二轮执行报告](../../../../../../../docs/thesis_project/evidence/T1/m2_round2/T1_M2_ROUND2_EXECUTION_REPORT.md)。

## 历史运行授权快照（2026-10-16；最新执行结果见顶部）

用户已确认全部6图用于第二轮B组参数开发：X=−0.06至−0.13、步长0.01，每图每档3次，合计144项；E=−0.05，保持关闭。实际启动与停止结果见顶部执行报告。地图几何及版本不变，不代表已有策略普遍可达或难度验证证据。

状态：6张M2开发候选地图已生成并登记，已用于本次已停止的第二轮部分运行；不代表地图集已通过所有策略可达性验证。M2独立于冻结的`m1-mapset/1.0`，没有改动M1地图、划分或运行记录。T1仍为唯一Active Task，M2属于其开发参数工作，不另开并行主任务。

## 地图与预览

[六图总览](../../../../../../../docs/thesis_project/designs/M2/candidate_v1/overview_m2_6.png) · [布局与几何数据](../../../../../../../docs/thesis_project/designs/M2/candidate_v1/layouts.json) · [地图索引](../../../../../../../docs/thesis_project/designs/M2/candidate_v1/scene_index.csv)

| 文件 | 预览 | 局部挑战设计 | 障碍数 |
|---|---|---|---:|
| `m2dev01.xml` | [M2-01](../../../../../../../docs/thesis_project/designs/M2/candidate_v1/m2dev01.png) | 连续正面短障碍，绕行后再次接近障碍 | 10 |
| `m2dev02.xml` | [M2-02](../../../../../../../docs/thesis_project/designs/M2/candidate_v1/m2dev02.png) | 左右错列混合障碍，保留宽缓S参考通路 | 9 |
| `m2dev03.xml` | [M2-03](../../../../../../../docs/thesis_project/designs/M2/candidate_v1/m2dev03.png) | 两段偏置开口，短箱体与圆柱组合 | 10 |
| `m2dev04.xml` | [M2-04](../../../../../../../docs/thesis_project/designs/M2/candidate_v1/m2dev04.png) | 斜箱、圆柱交替，改变障碍轮廓和接近角 | 9 |
| `m2dev05.xml` | [M2-05](../../../../../../../docs/thesis_project/designs/M2/candidate_v1/m2dev05.png) | 前段开放，后段连续局部阻挡 | 10 |
| `m2dev06.xml` | [M2-06](../../../../../../../docs/thesis_project/designs/M2/candidate_v1/m2dev06.png) | 混合错位物体，反向S参考通路 | 9 |

XML位置：仓库根目录下`unitree_mujoco/unitree_robots/go2/m2dev01.xml`至`m2dev06.xml`。内部ID为`m2d01`至`m2d06`，与M1不同。地图版本为`m2-development-candidates/0.1`；当前是候选版，尚未冻结第二轮实验矩阵。

## 公共条件

- 起点(0,0)，目标(7.5,0)m；建议沿用到达半径0.5m和现有统一任务时限，便于与M1开发结果解释性比较。
- 声明场地x=[-0.5,8]m、y=[-3,3]m，没有实体边墙。
- 仍使用0.65m保守平面包络、0.10m工程余量，参考路径到障碍真实轮廓净距至少0.75m。
- 所有物体轮廓间距至少0.60m，保持M1的单体投影上限x≤0.85m、y≤0.75m；不用长墙或密排短块制造封堵。
- 6张都有起终点直线阻挡；参考路线均单调向前、最大方向角不超过45°，参考长度约8.17–8.90m。浅青带半宽0.75m，表示包络与余量，**不是实际两侧障碍开口宽度**。
- 参考线仅证明一种几何通路，不送入策略，也不要求机器人走该线。特别是S形参考线不代表必须走S形；策略可以选择其他合法通路。

## 地图生成与生产加载核验（运行前阶段）

生成器`scripts/generate_m2_maps.py`按固定种子生成XML、布局、逐图PNG和总览。生成期间按真实箱体/圆柱轮廓计算参考通路净距、物体间距、起终点净距和单体投影，并拒绝不符合规则的布局。

6份XML均由与生产运行相同的C++ MuJoCo 3.3.3加载并生成模型指纹，使用生产`abs_collision_model::compute`，输出见[运行端模型身份](../../../../../../../docs/thesis_project/designs/M2/candidate_v1/runtime_model_identities.txt)。6张登记到`common/abs_scene_catalog.def`，MuJoCo主程序增量构建完成。这里只做了模型加载和身份生成，**没有推进物理、运行策略、启动GUI/ROS/controller或执行生产publisher→reader握手诊断**。本轮未新增或运行测试套件，未提交Git。

## 使用与研究限制

“挑战”指设计意图，**没有运行证据证明这6张比M1更难、能区分阈值或一定可达**。前向参考路线与短物体设计不保证11条前向射线可以持续看见所有障碍；包络也未经步态实测。不要在开始运行前给地图标注策略成功率或正式难度等级。

（历史地图创建阶段说明；本轮M2运行状态见本文件顶部。）M2用于第二轮参数开发，创建动机来自已有实验观察，不能作为完全独立的最终测试集。M1留出集仍未用于选参；最终参数冻结后再另行验证。用户随后已确认到达0.40、无终止性碰撞0.30、效率0.20、稳定性0.10的辅助评分，并完成原400次的事后计算，见[评分报告](../../../../../../../logs/01_T1_exit_threshold/b_exit_validation_400/20261016_131943_26/weighted_score_v1/REPORT.md)。当时尚未启动M2策略或第二轮批次，地图和重复次数未冻结。

（历史规划建议，已由顶部用户授权的144项矩阵取代。）布局审阅后原建议先用A组逐图观察，再固定第二轮地图、参数、评分、次数和无效记录规则；合法碰撞/超时保留，不以策略到达成绩删改图。

单张人工查看示例（本轮没有执行）：

```bash
rtk proxy /usr/bin/python3 -u scripts/manual_ab_compare.py \
  --groups A --repeats 1 --scene m2dev01.xml --goal-x 7.5 \
  --entry-threshold -0.05 --sim-limit-s 20 --wall-cap-s 120 \
  --plot-python /home/lidio/anaconda3/bin/python3 \
  --display :0 --xauthority /run/user/1000/gdm/Xauthority \
  --classification "manual M2 candidate viewing; not a formal experiment"
```

## 复现

几何和预览生成使用`rtk proxy /usr/bin/python3 scripts/generate_m2_maps.py`，只写6张M2图和本目录的候选资产，不生成M1。重新生成后需使用当时生产C++ MuJoCo重新取得模型身份，再以`--runtime-fingerprints <身份文件>`登记并构建MuJoCo；不要将Python MuJoCo的不同版本指纹覆盖生产指纹，也不要在批次运行中重生成地图。当前身份文件为MuJoCo 3.3.3版本。

本轮模型加载工具已退出，未启动仿真运行环境；临时C++工具和可执行文件已清理。
