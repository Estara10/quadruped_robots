# D435i近障补充数据与60轮全网微调结果（2026-11-17）

## 结论

新增场景后，**组合验证集上的可见近障高估有下降**。按同一套checkpoint复评，epoch13的可见近障高估为88/916，P95为0.774m；新候选epoch3为52/916，P95为0.530m。画内为29/578，画外但障碍表面可见为23/338。相对于medium（211/916），也有下降。

但新候选总体MAE为0.988m，差于epoch13的0.791m；远处过度低估为2232/3522，亦差于epoch13的1803/3522。最大可见近障高估仍有1.977m，完全不可见组近障高估为49/84。**这只是本次用于训练和选轮的组合验证集结果，不是独立测试、真实相机精度或控制验证。**之前已查看过的final holdout没有读取、预测或用于本次选轮。

既定可见近障规则选中第3轮：它相对medium满足可见目标及画内近障高估数/P95不劣、远处过度低估数不劣的资格条件；符合条件的轮次按可见近障高估数、P95、总体MAE排序。epoch13只作固定checkpoint诊断对照，没有参与修改规则。

## 数据与固定训练执行

补充数据位于[`near_coverage_supplement_v1`](../../../logs/real_go2/20261117_d435i_near_coverage_supplement_v1/)。新增train 180个位姿（3个布局、360张clean/masked图）和独立validation 90个位姿（1个布局、180张图），共540张；数据目录32,322,176字节，生成耗时46.0秒。审计通过：输入有限且为160×90、数值范围0.1–6m、clean/masked同位姿标签一致、train/validation布局隔离、没有空输入；未生成test。

与v2合并后，训练集为1,020个位姿/2,040张输入，验证集为270个位姿/540张输入、5,940个射线值。训练配置与固定启动脚本一致：medium初始化、全网微调、60轮、batch16、AdamW、lr `1e-5`、weight decay `1e-4`、near weight 4、false-safe weight 8、false-danger weight 0、seed 20261117、`num_workers=0`、`cuda:0`。未传入test目录。

训练目录为[`medium_fullnet_near_coverage_60ep`](../../../logs/real_go2/20261117_d435i_near_coverage_training/medium_fullnet_near_coverage_60ep/)；60轮完成，耗时174.7秒，CUDA峰值已分配304.6 MiB、保留388 MiB。目录91,415,918字节（约87.2 MiB），包含best/last权重（各44,862,301字节）、manifest及验证对照产物。训练器原manifest未保存weight decay字段；本次按启动命令`--weight-decay 1e-4`及训练器AdamW实际使用`args.weight_decay`补录为0.0001，未更改训练或权重，并已让后续manifest记录该参数。

## 同验证集三模型对照

所有表中近障均为标签`<=1.5m`；高估事件为预测误差`>=0.5m`。高估P95按近障正误差（负误差记0）计算；远处过度低估为标签`>=3m`且预测低至少0.5m。每个模型均在同一组540张clean/masked输入上复评。

| 模型 | 总体MAE | 可见近障高估 | 可见近障P95 / 最大 | 画内高估 | 画外但表面可见高估 | 完全不可见近障高估 | 远处过度低估 |
|---|---:|---:|---:|---:|---:|---:|---:|
| medium | 2.056m | 211/916 (23.0%) | 1.265 / 2.746m | 78/578 | 133/338 | 51/84 | 3451/3522 (98.0%) |
| 旧epoch13 | **0.791m** | 88/916 (9.6%) | 0.774 / 3.455m | 40/578 | 48/338 | 56/84 | **1803/3522 (51.2%)** |
| 新epoch3 | 0.988m | **52/916 (5.7%)** | **0.530 / 1.977m** | **29/578** | **23/338** | 49/84 | 2232/3522 (63.4%) |

新epoch3的总体MAE为0.988m（medium 2.056m，epoch13 0.791m）；总体近障高估P95分别为medium 1.543m、epoch13 1.472m、新epoch3 0.856m。新epoch3在可见近障指标上比epoch13好，但远距离低估和总体MAE变差；不能把某一组指标单独解释为整体更优。

训练时GPU验证manifest对新epoch3记录的远处过度低估为2231/3522；固定checkpoint复评得到2232/3522。差异仅1条，具体数值差异来源未单独定位；可能与训练中GPU前向和保存checkpoint复评的浮点计算有关。上表三模型统一采用checkpoint复评结果；轮次资格仍按原训练manifest中实际使用的数值判定。

### 可见近障逐束高估

每格为“高估数/该束可见近障数”。束序按机身左侧−45°至右侧+45°。

| 角度 | medium | epoch13 | 新epoch3 |
|---:|---:|---:|---:|
| −45° | 10/36 | 10/36 | 5/36 |
| −36° | 11/66 | 9/66 | 2/66 |
| −27° | 17/76 | 3/76 | 0/76 |
| −18° | 24/94 | 5/94 | 5/94 |
| −9° | 39/118 | 13/118 | 12/118 |
| 0° | 25/130 | 5/130 | 5/130 |
| +9° | 22/118 | 9/118 | 4/118 |
| +18° | 16/94 | 4/94 | 2/94 |
| +27° | 23/82 | 7/82 | 4/82 |
| +36° | 12/60 | 12/60 | 4/60 |
| +45° | 12/42 | 11/42 | 9/42 |

新epoch3仍以+45°边束的比例最高（9/42，21.4%），其次−45°（5/36，13.9%）和−9°（12/118，10.2%）。因此，近覆盖补充后边束风险并未消失；当前相机视野盲区和合成几何的泛化限制仍需分开评价。

### clean/masked配对

每个数对同位姿、同射线、标签`<=1.5m`的500组clean/masked配对统计。误差变化是`masked绝对误差−clean绝对误差`的配对均值；正数表示masked误差平均更大。

| 模型 | clean高估 | masked高估 | 配对误差变化 | masked更差/更好 | clean→masked新增高估 / 反向消除 |
|---|---:|---:|---:|---:|---:|
| medium | 132/500 | 130/500 | −0.028m | 231 / 269 | 16 / 18 |
| epoch13 | 69/500 | 75/500 | +0.004m | 250 / 249 | 16 / 10 |
| 新epoch3 | 52/500 | 49/500 | −0.010m | 240 / 260 | 12 / 15 |

配对结果没有显示新候选在masked输入上稳定恶化或稳定改善；错误方向也有少量双向转换。不能据此证明候选补洞与真实D435i空洞分布相符。

## 产物与复现

- [`summary.json`](../../../logs/real_go2/20261117_d435i_near_coverage_training/medium_fullnet_near_coverage_60ep/validation_comparison/summary.json)：按模型、可见性类别、数据来源、逐束和clean/masked配对汇总。
- [`per_sample_per_ray.csv`](../../../logs/real_go2/20261117_d435i_near_coverage_training/medium_fullnet_near_coverage_60ep/validation_comparison/per_sample_per_ray.csv)：540张输入逐样本、逐束标签及三模型预测/误差。
- [`validation_comparison.png`](../../../logs/real_go2/20261117_d435i_near_coverage_training/medium_fullnet_near_coverage_60ep/validation_comparison/validation_comparison.png)：可见性分组、逐束高估率与clean/masked对照图。
- [`分析脚本`](../../../scripts/analyze_d435i_near_coverage_validation.py)和[`固定训练启动脚本`](../../../scripts/run_d435i_near_coverage_fullnet_20261117.sh)。

复评命令（仅v2与补充validation，不包含test）：

```bash
cd /home/lidio/quadruped_robots
rtk proxy /home/lidio/anaconda3/envs/abs/bin/python -u \
  scripts/analyze_d435i_near_coverage_validation.py \
  --output-dir logs/real_go2/20261117_d435i_near_coverage_training/medium_fullnet_near_coverage_60ep/validation_comparison \
  --device cuda:0
```

该输出目录已存在，复跑前需另指定新目录；本次没有复跑训练或读取最终留出集。训练日志位于训练输出根目录的同名`.log`文件。

## 限制与下一步

本轮结果只说明：新增布局下epoch3在合成验证集的可见近障高估统计比epoch13低，同时整体/远处低估较差；不能证明场景多样性是改善原因，训练轮次、过拟合或其他数据因素的作用无法由这一轮分离。合成空洞处理、D435i真实深度误差、安装候选误差、相机视场盲区以及完全不可见目标均未解决。新权重未接入部署。

**唯一推荐的下一步：**先建立一份未参与训练和选模、按完整新场景布局隔离的独立离线评价集，并一次性比较medium、epoch13和epoch3。只有该独立评价仍支持可见近障改善后，再由主助手决定是否进行相机/机器人静态只读观察；本轮不安排现场测试。此前final holdout已查看，不能充作这份新评价集。

本轮未连接机器人、相机、Orin或服务器，未启动ROS/DDS/控制器/策略，不改部署权重引用和R1 ACTIVE。数据生成、训练及验证进程均已退出。
