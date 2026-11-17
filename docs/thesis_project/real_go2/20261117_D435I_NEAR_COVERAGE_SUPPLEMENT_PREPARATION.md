# D435i近障射线数据补充与全网训练准备（2026-11-17）

> 本文记录生成和训练前冻结方案。完整补充数据及固定60轮训练已于2026-11-17执行；结果与组合验证集对照见[训练结果报告](20261117_D435I_NEAR_COVERAGE_TRAINING_RESULT.md)。

## 结论

v2训练集并不缺少0.45–1.5m射线标签：840个位姿产生1,711条近障射线标签，其中0.45–0.75m有471条、0.75–1.0m有441条、1.0–1.5m有799条；1.0–1.5m在11束上各有58–88条。近障高估不能简单归因于“近障数量太少”。更值得补的是**不同布局中的目标尺寸、左右错位和前后遮挡组合**：原训练只有8个布局（其中2个随机障碍布局），验证只有2个布局。近障标签中979条交点画内、539条交点画外但同一障碍表面仍有图像像素、193条完全没有障碍像素。完全不可见是当前视野限制，不能靠训练恢复不存在的图像信息。

因此准备了三个新的训练布局和一个整布局隔离的验证布局。小探针生成已确认这些场景确实产生近障、侧向和可见遮挡样本。它只证明生成路径和覆盖意图成立，不证明网络预测改善。

## v2训练覆盖审查

统计只读v2的train和validation可见性CSV与summary；本轮没有用任何已经查看的最终留出集样本或预测设计新布局、选轮或调参。每条标签为一个位姿的一束射线；clean/masked共用同一标签，不是两个独立几何样本。

| 划分 | 位姿 | 近障射线数 | 0.45–0.75m | 0.75–1.0m | 1.0–1.5m | 交点画内 | 画外、表面可见 | 完全不可见 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| train | 840 | 1,711 | 471 | 441 | 799 | 979 | 539 | 193 |
| validation | 180 | 300 | 104 | 71 | 125 | 165 | 106 | 29 |

train每束0.45–1.5m标签数按−45°到+45°顺序为`143, 143, 157, 172, 170, 159, 170, 173, 140, 134, 150`。1.0–1.5m各束为`68, 69, 71, 76, 81, 73, 76, 88, 78, 58, 61`。因此没有哪束完全缺少近障标签；但训练仍只有8个独立布局，且没有低于0.45m的正式标签（这是当前生成器的净空筛选范围）。

现有布局包含固定的近中心/左右、遮挡对、走廊、混合场景及两个随机障碍布局；几何由box/cylinder构成，单个布局重复采姿。数据量无法覆盖所有尺寸和遮挡排列。这里把“布局/组合多样性有限”列为可由数据结构确认的局限，不宣称它已被证明是模型高估的根因。v2的masked输入是少量既有实拍零值掩码移植并按librealsense mode 1候选处理，不是相机空洞物理仿真；本轮不改变此流程。

## 新布局与小探针

`prepare_d435i_ray_adaptation_data.py`增加显式`--coverage-supplement-v1`模式，原默认布局和参数不变。新增布局为：

- `supp_near_size_mix`：近处宽box、窄cylinder和小box并有中远处参照物；
- `supp_near_partial_left` / `supp_near_partial_right`：左右错位前后目标，分别改变box/cylinder尺寸和遮挡次序；
- `supp_validation_staggered_sizes`：不同尺寸和错列遮挡，整个布局只用于validation。

本轮探针每个布局8个位姿：train 24个、validation 8个，共32个位姿和64张配对输入。train探针得到57/264条近障射线标签，其中0.45–0.75m 19条、0.75–1.0m 13条、1.0–1.5m 25条；可见性为交点画内34条、画外交点但表面可见17条、完全不可见6条。验证探针为16/88条近障标签（画内10、表面可见6、完全不可见0）。这证明新布局能产生三类标签及1.0–1.5m左右目标；样本很少，不能用于模型训练或稳定性结论。

生成与检查：

```bash
cd /home/lidio/quadruped_robots
MUJOCO_GL=egl /home/lidio/anaconda3/envs/abs/bin/python -u \
  scripts/prepare_d435i_ray_adaptation_data.py \
  --coverage-supplement-v1 \
  --output-root logs/real_go2/20261117_d435i_near_coverage_supplement_probe \
  --poses-per-layout 8 --seed 20261117 --max-attempts 40000 --skip-geometry-checks \
  --camera-position-m .300 .020 .132 \
  --camera-pitch-range-deg 3 3 --base-height-range-m .348 .348 \
  --depth-intrinsics 480 270 244.134 244.134 245.186 134.22
```

实际生成32个位姿/64张输入，耗时7.65秒，占用3.94MB；生成器审计通过：无空帧，160×90输入有限且在0.1–6m范围，clean/masked的11维标签一致，train与validation布局互斥，未生成test目录。mask augmentation沿用原配置，探针train/validation平均原始未知比例约7.61%/7.21%，mode 1候选填入约3.30%/1.39%，剩余未知占位后模型输入记录约4.22%/5.70%。这些是候选数据处理统计，不表示6m占位代表空旷。

代码审查：

- 保持当前安装候选`(0.300,+0.020,+0.132)m`、下俯3°、base高度0.348m、实测K `[480,270,244.134,244.134,245.186,134.22]`；镜头壳体/玻璃参考、偏航/侧倾零假设及base高度误差仍未精密标定。
- ABS射线合同不变：从`(-0.05,0,0)m`出发，机身水平面11束`−45°…+45°`、9°间隔，米制0.1–6m标签。
- 输入仍为480×270米制深度，中心采样`(3i+1,3j+1)`到160×90，clip 0.1–6m、log2、三通道。masked继续用既有实拍零值mask移植、mode 1补洞和残余未知6m候选；本轮未换补洞方式或损失。
- 没有删除完全不可见目标的标签；报告和选模单独分组。±45°视野外束仍是物理信息缺失限制。

## 一次固定训练计划（待用户另行安排）

### 数据规模

全量补充时不传`--poses-per-layout`：三个新train布局各60个位姿，共180个train姿态/360张clean-masked图；新增验证布局90个位姿/180张图。与v2合并后，train为1,020姿态/2,040张图，validation为270姿态/540张图；约比v2训练图增加21%，预估新增数据约33MB。validation包括原两布局和新增独立布局，按完整布局分隔；clean/masked配对必须留在相同split。

通过探针后生成全量补充（新目录必须不存在）：

```bash
MUJOCO_GL=egl /home/lidio/anaconda3/envs/abs/bin/python -u \
  scripts/prepare_d435i_ray_adaptation_data.py \
  --coverage-supplement-v1 \
  --output-root logs/real_go2/20261117_d435i_near_coverage_supplement_v1 \
  --seed 20261117 --max-attempts 40000 --skip-geometry-checks \
  --camera-position-m .300 .020 .132 \
  --camera-pitch-range-deg 3 3 --base-height-range-m .348 .348 \
  --depth-intrinsics 480 270 244.134 244.134 245.186 134.22
```

### 固定训练设置

- medium既有权重初始化：`archive/ray_pred_finetune/mujoco_finetune_medium_20260611/ray_pred_mujoco_finetuned_best.pt`；从头训练和从epoch13续训都不采用。
- 全网络解冻；60 epoch上限；batch 16；AdamW（现有trainer）、学习率`1e-5`、weight decay`1e-4`；near weight 4、false-safe weight 8、false-danger weight 0；seed `20261117`、`num_workers=0`、CUDA `cuda:0`。
- 训练目的只是一次有界的数据多样性适配。单次预算到60轮即结束；无合格checkpoint时报告失败，不追加轮数、不调损失或换模型。
- 以medium在同一validation集为基线。eligible规则要求：可见近障（交点画内或画外但命中障碍表面仍有像素）的false-safe数和近障高估P95不劣于medium；画内近障两项也不劣；远处低估`>=0.5m`数量不劣。合格轮次依次按可见近障false-safe数、其高估P95、总体MAE最小选择。若没有合格轮次，不选模型。
- 完全不可见目标和6m range-cap分别报告，但不作为可见近障选模门槛；远处低估仍报告并作为非回退门槛。总体MAE是次级排序项，不能单独宣布成功。
- train/validation以全布局分割；clean/masked同位姿配对不跨集合。此前已查看的`final_holdout_v1`和v2 test仅作历史记录，不进入本轮训练/选模，也不再用于调参。最终评价需另留全新未读布局，在checkpoint冻结后一次性评价。

新最终集尚未生成：后续需单独生成至少一个与train/validation布局XML和障碍摆位都不同的新布局（建议90个位姿，保持相同深度/标签合同），作为`final_holdout_v2`独立目录封存；在60轮选模完成、权重固定之前不运行候选模型或查看预测。它不进入上面的训练启动脚本。

`finetune_mujoco_ray_pred.py --selection-priority visible_near`已做最小改动：可见性统计区分画内、画外但表面可见、完全不可见和6m无命中，并允许多个validation可见性CSV；visible_near选择不再把完全不可见目标计为近障门槛。默认`overall_mae`流程、模型、损失和优化器不变。旧epoch13结果是按旧代码和旧选模规则产生的历史结果，不重算。

### 训练命令入口与资源

完整训练启动脚本已准备但未执行：

```bash
bash scripts/run_d435i_near_coverage_fullnet_20261117.sh
```

脚本只传train/validation目录，不含`--test-datasets`，不会读取最终留出预测；它检查所需数据存在并拒绝覆盖同名run。RTX 4060 Laptop 8GB上，先前300轮全网、batch16、1,680个训练图实测耗时约652秒，峰值CUDA reserved约386MiB；按训练图数和60轮线性估算本轮约160秒，实际需给环境启动和I/O留余量。该估算不是新运行实测。新增完整补充预计约33MB；按已有best/last各约44.9MB，训练输出约90MB级，另留日志空间。

## 尚未解决与边界

- 1.0–1.5m有标签但覆盖只有8个训练布局；补充布局增加尺寸、左右错位和遮挡组合，是否能降低高估必须用新validation观察，当前不能归因。
- `0.45m`以下没有训练标签；若目标应用包含更近距离，本方案不能声称已覆盖。
- 完全不可见束缺少图像信息，保留为限制，不要求模型证明可恢复。
- mask是有限旧实拍掩码移植，不是真实场景空洞的物理仿真；mode1＋残余6m仍是候选输入合同。
- 安装几何仍近似；相机视场约89°，机身±45°射线合同不变，画外和无像素目标需要分别评价。
- 本轮只生成/审计数据，没有模型推理、训练或机器人验证。R1 ACTIVE、部署权重、控制器、仿真默认设置和go2_navigation均未改。

## 本轮收尾

探针产物保存在`logs/real_go2/20261117_d435i_near_coverage_supplement_probe/`：包括`summary.json`、`ray_projection_visibility.csv`、`data_records.json`、按train/validation分开的配对`.npy`和布局XML；没有保存额外渲染PNG或模型预测。

生成器在7.65秒正常退出；Python语法编译、帮助入口和生成文件数值/配对审计通过。未连接设备/服务器，未启动ROS/DDS、策略或训练；无本轮残留进程。日志为本地gitignore路径；本报告和脚本改动尚未提交，新增数据尚未进入任何U盘备份。
