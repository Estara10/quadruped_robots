# D435i 射线模型下一轮数据与训练准备（2026-11-13）

## 结论与建议

修正后的生成流程已按候选相机内参和当前安装位置生成小批量深度图，射线标签仍由 MuJoCo 场景几何独立计算。修正主点后，中心与左右方向的投影和束序可以检查；但当前相机并不能直接观测所有 ABS 射线的近端交点。尤其在候选安装下，水平地面高度上的 ±45° 标签交点在 0.1–6m 均不落入图像，±36° 也分别要到约 1.77m 和 1.545m 才进入画面。某些近距离目标完全不在图像中。

因此**当前不建议把小批数据扩大后直接上服务器训练**。先确认最终光学中心相对 `base_link` 的安装变换，并决定对不可观测射线如何处理（保持 ABS 原标签的模型能力边界，或另行设计传感/接口方案）。增加样本或调损失不能恢复相机没有观测到的场景信息。本轮没有训练、推理模型或改模型权重。

## 已修正并检查的生成路径

- `scripts/generate_mujoco_ray_dataset.py` 旧默认仍保留；显式相机模式同时设置 MuJoCo 单目渲染所用的两只虚拟相机离轴 frustum，从而将 `ppx/ppy` 按像素中心约定实际用到渲染中。`--save-native-depth` 只在提供完整安装位置、俯角、原生分辨率及内参时可用，先保存 480×270 浮点深度，再复用现有 `(3i+1,3j+1)` 中心采样得到 160×90 输入。
- 几何标签由 `compute_geometric_ray_log(..., scene_filename, ...)` 经场景碰撞几何计算，保持 ABS 的 11 束、−45°到+45°每9°、起点 `base_link` 前向−0.05m、距离0.1–6m。生成器及消费者的默认仿真行为未变。
- 小批场景同时包含中心、左/右偏置目标和 ±45°边缘目标。既有单场景已知几何检查给出中心0°标签2.00m、偏左目标落在图像左侧且对应正角度束变近；本次 `ray_projection_visibility.csv` 逐条核对了标签距离与独立 `mujoco.mj_ray` 命中距离，差异超过0.1mm即停止。示例中心样本中，+18°/+27°标签投影到图像左半部，−18°标签投影到右半部；中心目标约0.60m时主点投影为 `u=245.186`，与给定 `ppx` 一致。图像及每束投影见 `candidate_v1/geometry_projection_samples.png`。
- 边缘已知摆放在 `test_center100_edges` 中：中心1.00m交点投影约 `(245.2,169.6)`；−45°/+45°目标2.25m交点分别投影到 `u≈557.7` 与 `u≈−67.4`，落在480像素宽图外。这和修正后的内参视场一致，不再按旧主点生成样本。
- 相机候选为壳体中心代理 `(0.305, 0, 0.140)m`、向下4°、`480×270, fx=fy=244.134, ppx=245.186, ppy=134.22`。壳体中心到真实光学中心偏移、俯角及站立高度仍是近似值；图示不是安装标定结果。

## 近侧覆盖与画外标签

`candidate_v1/label_hitpoint_coverage_by_angle.csv` 在 base 平面射线与相机像平面的候选几何下按1mm距离步进，给出标签**交点**首次进入像面的距离：

| 束角 | 首次落入画面 | 含义 |
|---:|---:|---|
| −45° / +45° | 0.1–6m范围内没有 | 该机身射线标签交点始终画外；不是说所有相应方向障碍都必然不可见。 |
| −36° / +36° | 1.770m / 1.545m | 更近的标签交点画外。 |
| −27° / +27° | 0.829m / 0.787m | 近距离交点画外。 |
| −18° / +18° | 0.601m | 近距离交点画外。 |
| −9° / +9° | 0.578m | 近距离交点画外。 |
| 0° | 0.571m | 近距离交点画外。 |

这是基于候选安装参数和 base 平面交点的几何计算，不等同于“障碍物全不可见”。渲染分割的98个有碰撞标签中，62个交点落在图像内、36个在外；36个画外交点中，20个对应障碍仍有部分表面进入画面，16个在该小批场景中完全没有障碍像素。后一项只说明这16个合成摆放的障碍未成像，不能泛化到所有障碍形状或真实安装。

0.1–0.45m **未进入训练候选**。只做了0.25m/0.35m两个诊断摆放，未纳入 train/validation/test；0.25m中央障碍在该渲染中没有可见像素。ABS 的0.1m下限没有在这批数据中被证明可从当前相机直接估计。训练可能拟合相关场景先验，但不能让完全不可观测的目标变成测量值。当前支架先保留，不建议为了训练立即重装；若系统要求全11束、近到0.1m均由图像直接支持，则需要先重新审视传感覆盖或接口能力，而不是只加训练量。

## 小批量候选数据

产物在 `logs/real_go2/20261113_d435i_ray_next_training_preparation/candidate_v1/`：

- 8个几何位姿，按整布局分为 train 4、validation 2、held-out test 2；每个位姿有 clean 与 masked 两张输入，共16张。邻近重复位姿没有跨 split。
- train 标签分布为 0.45–1.5m:20、1.5–3m:24；validation 为3/19，test 为2/20。三份 split 都没有 >3m/6m 截断束；因此小批量可检查近障和左右几何覆盖，但**不能作为可训练语料**，也没有覆盖远距离清晰视野分布。
- 空洞增强将真实D435i无效掩码转移到合成原生深度，模式1使用仓库已有 librealsense 2.58.4 源码等价离线实现，残余零值填6m；clean/masked共用同一独立标签。训练输入路径仍走当前中心采样、0.1–6m裁剪、`log2`及三通道复制。
- 掩码迁移不是相机成像仿真；真实掩码来源场景与合成障碍不匹配，填6m仍是待验证的模型输入候选，不能当作“测得无遮挡”。线上设备 SDK 是2.51.1，已保存过该版本实际补洞输出，但本批增强使用的是离线2.58.4实现，不应冒称现场逐像素复现。
- `summary.json`、`data_records.json`、`ray_projection_visibility.csv`、`label_hitpoint_coverage_by_angle.csv` 和图像用于审阅输入分布、空洞与可见性；没有用任何神经网络预测当标签，也不包含模型准确率或安全结论。

复现小批量命令（使用新输出目录；脚本拒绝覆盖已有 `candidate_v1`）：

```bash
rtk proxy env MUJOCO_GL=egl /home/lidio/anaconda3/envs/abs/bin/python \
  logs/real_go2/20261113_d435i_ray_next_training_preparation/prepare_small_candidate.py \
  --output-dir logs/real_go2/20261113_d435i_ray_next_training_preparation/reproduction_01
```

## 下一轮评价与选模规则

现有旧 pilot 只按总 MAE 选轮：总 MAE下降时，近障高估≥0.5m却从98/360增至121/360。下一轮应固定medium权重为候选起点，保留其原文件；不直接追加旧pilot轮数。

已为 `scripts/finetune_mujoco_ray_pred.py` 增加显式 split、可见性分组和非回归选择支持。使用独立 `--validation-datasets` 时，训练目录仅作为训练集；validation 用于逐轮报告总体 MAE/RMSE/bias、逐束 MAE/bias、近障 false-safe 数/率及高估P95/max、远处过度低估（false-danger）和过度低估数。若三个安全侧指标——近障 false-safe 数、远处 false-danger 数、近障高估P95——任一比 medium validation 基线更差，该轮不入选；符合门槛的轮次再按总体MAE最小选择，平手按近障高估P95较低选择。无人通过则不产生“best”，保留medium作为当前模型。test目录仅在选轮之后作一次报告，不参与选模。

传入 `--validation-visibility-csv` 和 `--test-visibility-csv` 时，额外分别报告标签交点在相机画面内/外的误差和逐束统计。画外分组只说明标签信息不可直接观测，不用其单独证明网络错误。训练前应确保完整数据的几何CSV对应各split同一批样本。验证/测试也要按完整场景和位姿区块留出，不能随机拆相邻帧。

画内/画外分组先作为审阅报告项，不设单独硬门槛：画外标签包含物理上不可由该帧直接观测的目标，需先确定系统如何处理这类信息；不应借一个任意阈值把它们包装成训练已解决。

对旧pilot在正确几何下出现的近障高估，不通过减阈值或改标签抹掉。近障危险高估和过度低估都保留报告；当前损失中的 false-danger 权重默认为0，评估仍报告该项。若后续考虑改变损失，必须另建候选并预先固定规则，不能本轮临时调参。

## 服务器计划（命令模板，尚未执行）

仅在安装几何、标签可见性边界和数据生成范围经审阅后再扩展数据。一个有限初始方案是约1200个**独立几何位姿**，不是1200帧相邻采样；按完整布局/位姿块约70/15/15分成训练/验证/最终检查，所有区域包含左右束和至少三段距离：0.45–1.5m、1.5–3m、3–6m，限制6m截断样本占比。0.1–0.45m单列为可观测性诊断，不伪装成常规训练能力。每个位姿保留clean/masked配对并使用同一标签。具体分层数应由射线标签和视野可见性统计决定，不能让远处样本继续压过近障。

建议固定预算4轮、medium初始化、仅训练 `layer4+fc`、batch 32、学习率 `1e-4`，不扫参。1200个pose配对约2400张160×90 float32输入约132MiB；若留存每个pose的480×270 float32原生深度另约0.58GiB；含best/last模型、manifest和日志，预留约1.5GiB磁盘即可覆盖该批及少量余量。GPU显存需求取决于服务器环境；ResNet18、batch32的粗估至少预留4GiB可用显存，不是已测结果。服务器GPU/显存、CUDA与PyTorch版本、实际路径和磁盘空间均未知，需登录前由主助手只读确认；本轮未连接服务器。

数据目录确认后可审阅下面的训练命令模板；将 `REPO`、`DATA_ROOT`、`RUN_ROOT` 替换成服务器上的真实绝对路径。所有输出写入新run目录，`cuda:0`不可用时应先停下查环境，不去掉显式设备选择或静默改CPU：

```bash
REPO=/absolute/path/to/quadruped_robots
DATA_ROOT=/absolute/path/to/d435i_ray_dataset_v1
RUN_ROOT=/absolute/path/to/ray_training_runs
PYTHON=/absolute/path/to/python
RUN_NAME=d435i_medium_layer4fc_20261113

"$PYTHON" "$REPO/scripts/finetune_mujoco_ray_pred.py" \
  --base-model "$REPO/archive/ray_pred_finetune/mujoco_finetune_medium_20260611/ray_pred_mujoco_finetuned_best.pt" \
  --datasets "$DATA_ROOT/train" \
  --validation-datasets "$DATA_ROOT/validation" \
  --test-datasets "$DATA_ROOT/test" \
  --validation-visibility-csv "$DATA_ROOT/ray_projection_visibility.csv" \
  --test-visibility-csv "$DATA_ROOT/ray_projection_visibility.csv" \
  --output-root "$RUN_ROOT" --run-name "$RUN_NAME" \
  --device cuda:0 --trainable layer4_fc --epochs 4 \
  --batch-size 32 --lr 1e-4 --num-workers 0
```

该命令可调用现有训练入口，但当前小批量数据绝不能代替 `DATA_ROOT` 的正式候选。训练器沿用既有损失；新规则只改变有显式validation时的选轮，不改变无新参数时的旧训练默认行为。旧合并训练进程退出码134的异常仍未定位；新训练按单进程运行并保留原退出码，不能视为已经修复。

## 代码、边界与剩余问题

- 修改：[生成器](../../../scripts/generate_mujoco_ray_dataset.py)增加可选native depth保存；[训练器](../../../scripts/finetune_mujoco_ray_pred.py)增加可分离数据目录、设备/可训练层选择、误差指标和validation非回归选轮及可见性分组。旧命令默认不变。
- 小批量驱动及结果：[准备脚本](../../../logs/real_go2/20261113_d435i_ray_next_training_preparation/prepare_small_candidate.py)、[summary](../../../logs/real_go2/20261113_d435i_ray_next_training_preparation/candidate_v1/summary.json)、[逐束可见性](../../../logs/real_go2/20261113_d435i_ray_next_training_preparation/candidate_v1/ray_projection_visibility.csv)、[角度覆盖表](../../../logs/real_go2/20261113_d435i_ray_next_training_preparation/candidate_v1/label_hitpoint_coverage_by_angle.csv)、[投影图](../../../logs/real_go2/20261113_d435i_ray_next_training_preparation/candidate_v1/geometry_projection_samples.png)。
- 不改模型权重、相机标定/深度比例尺、ABS标签、控制器、共享内存协议、仿真配置或R1任务状态。旧pilot结果不覆盖。
- 仍需确认的主要问题：最终相机光学外参；哪种训练输入处理能代表D435i未知深度；固定ABS束在当前视野内的不可观测覆盖该如何对系统表达；服务器硬件和软件路径。当前不能声称候选模型适配完成或可用于控制。

离线验证：使用现有 `abs` Python环境对三份脚本执行 `py_compile`，生成器和训练器 `--help` 正常；`RayDataset`能读取候选train/validation/test各8/4/4张图。用零输出dummy网络只验证可见/画外分组汇总代码能按CSV映射并返回指标，数字不代表模型评估。几何检查脚本已在生成时验证中心采样复现native输入、标签与 `mj_ray` 对齐。未训练模型、未连接硬件/服务器、未启动ROS/DDS。
