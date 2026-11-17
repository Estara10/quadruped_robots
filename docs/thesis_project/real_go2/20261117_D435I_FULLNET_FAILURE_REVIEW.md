# D435i全网适配近障高估复核

> 归档说明：本报告固定比较medium与按既定验证规则选中的epoch13。最终留出集已经查看，后续不得再用它选模或调参。本分析是离线模型失败分析，不代表真实机器人精度验证，也不代表控制通过。
> 本目录产物在此前U盘备份完成后才归档，因此不包含在那次备份内；如需备份，需由主助手安排后续备份。


## 结论

只复核了既定medium权重与已选epoch13，没有评价其他训练轮次，也没有用最终留出集重新选模。**高估尾部的主要集中点是画外射线，特别是−45°边束、完全没有障碍像素的目标；但画内近障也有实质误报，不能把问题归结为边束或未知深度。**masked输入没有在同位姿总体上稳定加重错误。

直接模型明细为最终留出集360张输入（180位姿×clean/masked）×11束。近障定义沿用训练器：几何标签`<=1.5m`且预测高出标签`>=0.5m`。630条近障输入-射线记录实际上是315个位姿-射线对各含clean/masked两条，不能当作630个独立姿态。medium有142条误报，epoch13有144条。78/630（12.4%）从medium未越过误报阈值变成epoch13越过；76/630（12.1%）反向改善，净增2条。按连续绝对误差，candidate在360/630（57.1%）条记录上更接近标签、270/630（42.9%）条更差。尽管二分类数量接近，近障高估P95由1.237m升至2.207m，最大高估由3.352m升至5.158m。

## 第13轮为何入选，训练曲线说明什么

manifest的选模规则要求总体近障误报、远处低估、近障高估P95，以及画内近障误报数/P95均不劣于medium验证基线；再按画内近障误报数、其P95、总体MAE排序。300轮里11轮合格；epoch13的排序键为`(11, 0.4287m, 0.7933m)`，在合格轮次中最小，因此按事前规则入选。验证基线为画内近障误报51/330、P95 0.881m；epoch13为11/330、0.429m。总体近障P95也仅以1.419m对1.445m的微小余量通过。

训练损失持续下降：epoch13为0.2136，epoch300为0.00890。验证MAE从epoch13的0.793m降至epoch300的0.589m，但epoch300总体近障误报升至106、近障高估P95升至2.083m，故不合格。曲线呈现总体误差与近障尾部权衡，**不能据此确定轮数不足或过拟合是根因**。在300轮中反复按同一validation选轮会产生验证集选模偏差，是可能的解释之一，不是已证实因果。

![300轮训练与验证轨迹](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/training_validation_trajectory.png)

完整300行保存为[training_history.csv](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/training_history.csv)。

## 错误集中位置

| 分组 | 近障记录数 | medium误报 | epoch13误报 | epoch13高估P95 | epoch13最大高估 |
|---|---:|---:|---:|---:|---:|
| 画内交点 | 384 | 37 (9.6%) | 36 (9.4%) | 0.694m | 1.807m |
| 画外、障碍表面仍有像素 | 196 | 69 (35.2%) | 62 (31.6%) | 1.931m | 3.836m |
| 画外、渲染中没有该障碍像素 | 50 | 36 (72.0%) | 46 (92.0%) | 4.480m | 5.158m |
| 合计 | 630 | 142 | 144 | 2.207m | 5.158m |

因此，candidate的144条误报中108条（75%）对应画外交点；其中46条来自没有障碍像素的组。此类输入缺少直接目标表面信息，模型输出不能解释为可靠测距。画内也有36条误报；epoch13相对medium有30条阈值回退、31条改善，净计数几乎持平，但P95/最大值仍更差。画内仅表示交点投影在图像内，不保证没有遮挡。

角度上，−45°束为61/70，+45°为31/48；两端合计92/118（78%），其余角度为52/512（10%）。−45°贡献了37条medium正确→epoch13误报的回退。与此同时，+45°误报仍有31条，与medium的31条持平。边束集中明显，但方向不对称且不能单凭此证明相机安装或标签错误。

| 几何标签距离 | 记录数 | medium误报 | epoch13误报 | medium正确→候选误报 | medium误报→候选改善 |
|---|---:|---:|---:|---:|---:|
| 0.45–0.75m | 76 | 54 (71.1%) | 26 (34.2%) | 0 | 28 |
| 0.75–1.00m | 150 | 36 (24.0%) | 25 (16.7%) | 16 | 27 |
| 1.00–1.50m | 404 | 52 (12.9%) | 93 (23.0%) | 62 | 21 |

距离劣化主要出现在1.00–1.50m段；这类候选误报93/404（23.0%），medium为52/404（12.9%）。更近的0.45–1.00m总体误报反而减少。训练正式集合不含<0.45m标签，此处无结论。

| 新测试布局 | 近障记录数 | medium误报 | epoch13误报 | medium正确→候选误报 | 候选P95 / 最大高估 |
|---|---:|---:|---:|---:|---:|
| final_staggered_occlusion | 318 | 64 | 60 | 34 | 1.736 / 3.836m |
| final_wide_wall_offset | 312 | 78 | 84 | 44 | 2.574 / 5.158m |

最坏尾部集中在wide-wall-offset布局；staggered布局误报计数略降但P95也上升。两个布局方向的表现不同，不足以说任何一种场景单独造成失败。

## clean/masked配对

315个唯一近障位姿-射线对均有clean/masked输入，标签完全相同。逐对比较epoch13的绝对误差：masked比clean更差154/315（48.9%）对、改善160/315（50.8%）、持平1对；平均变化−0.003m，中位数−0.005m。candidate误报从clean的71/315（22.5%）变为masked的73/315（23.2%）；clean→masked新增11/315（3.5%）对、反向消失9/315（2.9%）。medium相应为74/315（23.5%）→68/315（21.6%）、新增6、消失12。按二值误报和配对绝对误差，**没有稳定、普遍的masked加重证据**。

总体分组的candidate近障高估P95为clean 2.146m、masked 2.369m，提示masked组尾部稍高；但逐对结果方向近乎各半，且此处masked是生成数据的历史零值掩码移植/补洞候选，不是实时D435i原始故障重放。不能据此认定空洞处理是根因。代表性极端配对中，同一姿态−45°目标1.352m、交点画外但仍有障碍像素，medium误差从0.360增至0.743m，epoch13从2.152增至3.836m；该例按最大增量挑选，不代表发生率。

![同姿态clean与masked极端配对](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/case_masked_vs_clean.png)

## 验证与最终留出差异

v2 validation近障标签比例15.15%，final holdout为15.91%，近/中/远/6m截断的整体比例接近。可见性组成有差异：validation真实几何命中里画内1192/1445（82.5%）、画外但表面可见203（14.0%）、无表面像素50（3.5%）；final holdout分别1248/1576（79.2%）、276（17.5%）、52（3.3%）。最终留出多了约3.5个百分点“画外但仍有表面”的命中，完全不可见比例接近。布局和空间结构也换成了此前未参与训练/选模的两组。

epoch13在validation通过规则，但final holdout的画外/边束尾部恶化。这支持“新布局和可见性分布揭露了验证未覆盖的错误组合”；不能单独区分布局泛化、交点可见性、网络输入与标签几何交互、验证选模偏差或其他因素，也不能把final holdout的输出解释为真实机器人距离精度。

## 少量失败案例

- 画内：staggered布局、clean、0°目标1.038m；medium 1.360m，epoch13 2.845m。
- 画外但障碍仍可见：staggered布局、masked、−45°目标1.352m；medium 2.095m，epoch13 5.188m。
- 无障碍像素：wide-wall布局、clean、−45°目标0.842m；medium 3.530m，epoch13饱和到6m，最大高估5.158m。

![画内失败例](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/case_in_frame.png)

![画外但仍有表面的失败例](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/case_out_frame_surface_visible.png)

![没有障碍像素的极端例](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/case_no_obstacle_pixels.png)

## 方法、证据边界与下一步

逐样本明细由现有`RayDataset`读取最终holdout的保存米制160×90输入，沿训练/验证相同的clip、log2、三通道复制和exp2还原路径；在CPU上各对medium和epoch13推理一次。重新计算的总体MAE、误报数与manifest吻合（MAE差约0.00001m、误报计数一致；分位数有约0.002m浮点差异）。CSV逐条保留11束、模型输出、几何标签、布局、输入类型和可见性；没有触碰源数据或checkpoint。

**已定位的现象：**误报尾部集中于画外目标和±45°，尤其−45°；1.0–1.5m段增加；画内近障仍存在高估。**尚未定位的因果：**是视场盲区、布局/遮挡组合、训练掩码候选，还是验证选模偏差主导；单一checkpoint对照不能分开这些因素。

**唯一推荐下一步：**做一次不训练的“误报几何/输入对应审阅”，只复核CSV列出的三类固定失败样本及其配对帧，对照MuJoCo场景的实际障碍表面、射线交点投影和采样后的90×160区域，检查几何标签可见性标记是否与输入一致。完成条件是逐例确认或否定“标签障碍可见但采样输入缺失/被遮挡”这一数据链假设；不据这一步修改权重或选轮。无需机器人。

## 产物

- [per_sample_per_ray.csv](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/per_sample_per_ray.csv)：3960条模型输入-射线明细。
- [clean_masked_pairs.csv](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/clean_masked_pairs.csv)：1980个位姿-射线配对。
- [by_ray.csv](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/by_ray.csv)、[by_distance_bin.csv](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/by_distance_bin.csv)、[by_visibility.csv](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/by_visibility.csv)、[by_layout.csv](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/by_layout.csv)、[by_input_flavor.csv](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/by_input_flavor.csv)：拆分统计。
- [training_history.csv](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/training_history.csv)：300轮训练/验证明细。
- [summary.json](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/summary.json)：汇总、误报转变和极端案例。
- [analyze.py](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/analyze.py)、[plot_history.py](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/plot_history.py)、[plot_mask_pair.py](../../../logs/real_go2/20261117_d435i_fullnet_failure_review/plot_mask_pair.py)：复现分析脚本已归档到项目日志目录；原始临时来源仍保留在`/tmp/d435i_fullnet_failure_review_20261117/`。

本轮没有训练、模型选择、设备访问、项目文件写入或控制链操作。
