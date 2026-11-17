# D435i射线模型独立选型评价（2026-11-17）

## 结论

用三组全新布局生成180个位姿（360张clean/masked输入）后，对四份固定权重各评价一次。**没有任何模型全面最优**：

- **近障最好**：近障补充训练的epoch3（`logs/real_go2/20261117_d435i_near_coverage_training/medium_fullnet_near_coverage_60ep/...`）。可见近障高估105/652（16.1%）、P95 1.233m、配对口径49/326，三项均为最好。
- **远障最好**：原300轮的epoch13（`logs/real_go2/20261117_d435i_fullnet_preparation/training_runs/d435i_fullnet_300epoch_20261117/...`）。远障过度低估1339/2510（53.3%），总体MAE 0.909m，也最好。
- **建议主候选**：近障补充epoch3。代价是远障过度低估67.1%（比epoch13多344/2510）且总体MAE 1.157m，是三个微调模型中总体最差的；完全不可见近障高估8/12（该组仅12对，只作描述）。

关键新事实：在未见布局上，epoch13的可见近障高估由原验证集的88/916（9.6%）升至134/652（20.6%），最大值5.189m，说明它此前的近障优势没有完全迁移到新布局。本批数据只用于独立选型评价，不称为最终测试集。

## 四份权重（按完整路径区分）

| 名称 | 完整路径 | 文件字节 | sha256前16位 |
|---|---|---|---|
| medium | `archive/ray_pred_finetune/mujoco_finetune_medium_20260611/ray_pred_mujoco_finetuned_best.pt` | 44,862,301 | bc9521efb048c237 |
| epoch13 | `logs/real_go2/20261117_d435i_fullnet_preparation/training_runs/d435i_fullnet_300epoch_20261117/ray_pred_mujoco_finetuned_best.pt` | 44,862,301 | d2829ab4c1ad7172 |
| 近障补充epoch3 | `logs/real_go2/20261117_d435i_near_coverage_training/medium_fullnet_near_coverage_60ep/ray_pred_mujoco_finetuned_best.pt` | 44,862,301 | a2546a05d024cbee |
| 远障权重0.5 epoch3 | `logs/real_go2/20261117_d435i_far_underestimate_training/medium_fullnet_far_danger_0p5_60ep/ray_pred_mujoco_finetuned_best.pt` | 44,862,301 | f005d3ffdc5f804b |

## 新布局与数据

生成器`scripts/prepare_d435i_ray_adaptation_data.py`新增`--independent-eval-v1`模式，只含三个全新整布局，均为评价用、不生成train/test划分：

- `indep_eval_offset_pair_a`（60位姿，近/中/远配额 .60/.25/.15）：宽box+窄cylinder近障、左后错位目标；
- `indep_eval_offset_pair_b`（60，.55/.30/.15）：右侧近障、反向遮挡次序、左侧小目标；
- `indep_eval_open_mix_c`（60，.35/.35/.30）：开阔场景，近中远混合。

三个布局的名称和障碍摆位均与`build_layouts`各测试集、`final_holdout_v1`及`coverage_supplement_v1`不重合。生成后未按任何模型结果调整。相机安装候选`(0.300,+0.020,+0.132)m`、下俯3°、base高度0.348m、实测K `[480,270,244.134,244.134,245.186,134.22]`、中心采样、clip 0.1–6m、log2三通道、11束标签合同和masked补洞流程均与既有训练数据一致。命令：

```bash
cd /home/lidio/quadruped_robots
MUJOCO_GL=egl /home/lidio/anaconda3/envs/abs/bin/python -u \
  scripts/prepare_d435i_ray_adaptation_data.py --independent-eval-v1 \
  --output-root logs/real_go2/20261117_d435i_independent_eval_v1 \
  --seed 20261118 --max-attempts 40000 --skip-geometry-checks \
  --camera-position-m .300 .020 .132 --camera-pitch-range-deg 3 3 \
  --base-height-range-m .348 .348 \
  --depth-intrinsics 480 270 244.134 244.134 245.186 134.22
```

生成180位姿/360输入，耗时26.1秒，21.6MB；审计通过：无空帧、输入有限且为160×90、数值0.1–6m、clean/masked标签一致、无train/test划分。位姿级近/中/远为90/54/36；射线级近(0.45–1.5m) 338条、中387条、远1255条（含6m截断491条）；可见性为交点画内1,220、画外交点但表面可见251、完全不可见18、6m无命中491（合计1,980个位姿—射线对）。

## 四模型对照

**输入—射线记录口径**：分母3,960条记录（360输入×11），clean与masked分别计数；可见近障分母652，远障分母2,510。

| 模型 | 总体MAE | 可见近障高估 | 可见近障P95 / 最大 | 远障过度低估 | 完全不可见近障高估 |
|---|---:|---:|---:|---:|---:|
| medium | 2.043m | 164/652 (25.2%) | 1.428 / 4.014m | 2416/2510 (96.3%) | 24/24 |
| epoch13 | **0.909m** | 134/652 (20.6%) | 1.493 / 5.189m | **1339/2510 (53.3%)** | 23/24 |
| 近障补充epoch3 | 1.157m | **105/652 (16.1%)** | **1.233 / 3.262m** | 1683/2510 (67.1%) | **16/24** |
| 远障权重0.5 epoch3 | 1.094m | 112/652 (17.2%) | 1.343 / 3.658m | 1540/2510 (61.4%) | 19/24 |

**位姿—射线配对口径**：分母1,980对，clean/masked先配对平均，配对不作为独立实验。

| 模型 | 总体MAE | 可见近障高估 | 远障过度低估 | 完全不可见近障高估 |
|---|---:|---:|---:|---:|
| medium | 2.038m | 89/326 | 1223/1255 | 12/12 |
| epoch13 | **0.896m** | 65/326 | **667/1255** | 12/12 |
| 近障补充epoch3 | 1.147m | **49/326** | 860/1255 | **8/12** |
| 远障权重0.5 epoch3 | 1.081m | 55/326 | 792/1255 | 11/12 |

细分（配对口径，可见近障高估）：

| 可见性 | 分母 | medium | epoch13 | 近障epoch3 | 远障0.5 epoch3 |
|---|---:|---:|---:|---:|---:|
| 交点画内 | 202 | 30 | 17 | **11** | 15 |
| 画外交点、表面可见 | 124 | 59 | 48 | **38** | 40 |
| 完全不可见 | 12 | 12 | 12 | **8** | 11 |

逐束可见近障高估（记录口径，分母为该束近障可见记录数）以边束为主：−45°为18/24/15/16（顺序同上，分母38），+45°为25/41/39/41（分母54）。+45°在三个微调模型中均接近饱和，是本批布局下所有模型的共同弱点，不构成某模型的独特缺陷。其余束多在5–15之间，近障epoch3在−9°、+9°、−18°最低。

## 选型建议与代价

- **近障（安全相关）**：近障补充epoch3最好（计数、P95、配对口径、画内与画外两组均最低），远障权重0.5次之，epoch13最差。
- **远障与总体误差**：epoch13最好；远障权重0.5在“远障更保守”方向优于近障epoch3（远障过度低估61.4%对67.1%，MAE 1.094对1.157），代价是可见近障高估多7条记录。
- **主候选建议**：近障补充epoch3。理由是本项目既定门槛把可见近障高估作为安全相关指标，而它在未见布局上仍最好；epoch13在旧验证集上的近障优势没有迁移过来。
- **代价**：远障过度低估67.1%（三个微调模型中最差），总体MAE 1.157m；完全不可见目标仍8/12高估，但该组只有12对，只作描述。
- 若后续更看重远障而非近障，远障权重0.5是更平衡的替代；本轮不因结果追加训练或调参。

## 限制

- 本批是合成几何的独立选型评价，不是最终测试集，也不是实机精度或控制结论；相机安装外参仍是近似候选，mask为旧实拍空洞移植而非传感器物理仿真。
- 1,980对只来自180个位姿，11束彼此关联；完全不可见组仅18对（近障12对），计数只作描述，不做显著性判断。
- 本批布局比原组合验证集更难：所有模型的可见近障高估均上升，边束贡献最大；不能据此断言模型过拟合旧布局，只能说明该优势未完全迁移。
- 0.45m以下无正式标签；±45°视野外束仍是物理信息缺失限制。

## 产物

- 数据与生成配置：[`logs/real_go2/20261117_d435i_independent_eval_v1/`](../../../logs/real_go2/20261117_d435i_independent_eval_v1/)（`summary.json`记录布局、相机、内参、合同、处理流程与审计；`ray_projection_visibility.csv`、`data/validation/`）。
- 评价结果：[`evaluation/four_model_comparison.json`](../../../logs/real_go2/20261117_d435i_independent_eval_v1/evaluation/four_model_comparison.json)（四份权重的路径、字节与sha256，记录/配对两种口径的全部指标与逐束结果）。
- 评价脚本：`scripts/evaluate_d435i_independent_eval_v1.py`（复跑会因输出文件已存在而拒绝覆盖）。
- 生成器新增`--independent-eval-v1`模式，默认布局与其他模式行为不变。

本轮未训练、未连接设备、未读取其他留出集；未修改模型、训练器、控制器、相机配置、CURRENT_STATE或其他项目文档；未提交Git。生成与评价进程均已退出。
