# D435i射线网络完整训练准备（2026-11-16）

## 建议

建议做一次**从medium权重开始、全ResNet18参数可训练的有限域适配**。这里“完整训练”指全网解冻，不是随机初始化从零训练，也不是逐字复现ABS原作者302轮设置。当前可用训练集是当前安装候选v2；在任何训练前，先用一轮短程试跑核对RTX 4060 Laptop 8GB的全网反向传播显存和速度。正式适配轮数固定上限20，不因结果不理想自动续训。

现有v2训练数据足够开展一次受限适配，不建议盲目扩充训练集；但它不支持广泛泛化结论。由于v2 test已被查看，只能作历史回归参考。正式适配前还应准备并封存全新的最终留出布局，训练和验证期间不加载、不比较。

## 方法依据与差异

| 项目 | ABS原作者脚本能直接确认的内容 | 当前v2建议 |
|---|---|---|
| 初始化 | `train_depth_resnet.py`构造torchvision ResNet18，骨干加载`IMAGENET1K_V1`，再随机替换为11维全连接头；随后全网训练。仓库有2026-05-28 epoch250产物，但没有原始运行完整参数记录，不能据文件名推断实际轮数或命令。 | 从medium射线权重初始化，解冻并更新全网。不是ImageNet从零头部重训；保留已有射线表征以适配小规模当前安装数据。 |
| 输入/标签 | 图像、11束标签分别取`log2`，网络回归log2标签；单通道输入复制为3通道。 | 保持同一合同；标签来自当前安装几何生成器，模型不生成标签。 |
| 损失/优化器 | 普通MSE(log2空间)；SGD，脚本默认lr=0.01、momentum=0.9。 | 复用当前微调器的近障加权log2 MSE与AdamW：lr=1e-5、weight_decay=1e-4、near_weight=4、false_safe_weight=8、false_danger_weight=0。它们是适配选择，不称原作者超参数。验证规则同时监测近障高估和远处低估。 |
| 数据/训练预算 | 脚本默认随机留10%测试；`num_epochs=302`、`batch_size=320`、保存间隔50。均为脚本默认，未找到足以证明它们就是原权重训练实际配置的日志。 | 使用显式整布局train/validation目录，最多20轮；batch初始候选16，待单轮测速确认。不能把默认302或320当复现。 |
| 增强/保存 | 默认启用左右翻转(prob. 0.5)、高斯模糊(prob. 0.5,k=5,sigma 0.1–2)、加噪(prob. 0.5,std均匀0–0.3)、随机擦除(prob. 0.5,`randint(5,10)`故每边5–9像素，值为log2(0.25)或log2(6))。脚本每`save_interval`轮保存TorchScript，默认间隔50；由于epoch从0计，首次循环就保存epoch0。 | 不再叠加随机模糊/噪声/擦除：v2已含clean/masked配对，真实相机遮挡机制证据有限；横向翻转也暂不加，当前相机y偏置+0.02m，镜像会对应另一安装侧。微调器逐轮写last，并只在通过既定验证非回退规则时更新best。 |

此前v2的4轮和10轮试验均从medium开始、只训练`layer4_fc`、AdamW lr=1e-4、batch32；损失使用现有近障加权形式。它们不是全网训练。10轮记录指出不同运行的seed未固定实际Torch/DataLoader随机序列，不能作受控轮数比较。当前修复只解决后续运行的随机源控制，不改既有数据、权重或结果。

## v2覆盖与是否扩数据

- train/validation为840/180个位姿，各有clean/masked两张输入，即1680/360个样本；按完整布局隔离。train有8个布局，validation有2个布局。
- train标签：0.45–1.5m占18.5%，1.5–3m占25.7%，3–6m占37.0%，6m截断18.8%；validation近/中/远/截断约15.2/23.8/34.0/27.0%。正式split没有<0.45m标签。若将来要声称覆盖ABS的0.1m下限，需先确认相机几何是否能观测该范围；不可见的近处交点不能靠训练补齐。
- 11束每束各有840个train和180个validation标签，但画外交点并不少：总计10589个命中标签中，2117个交点在图外；其中428个在渲染中完全没有该障碍表面像素。其余画外交点仍有部分障碍表面可见。训练不会恢复完全不可见的信息。
- 掩码增强复用有限的历史实拍零值掩码；mode 1补洞＋残余未知填6m是候选输入流程，不是已经认可的安全处理，也不是空旷测量。此训练量够做一次有界更新，无证据支持为追求“更多数据”先扩成大数据集；真实空洞形态和安装误差仍限制外推。

## 选模和最终留出

训练前固定规则，不看test挑轮：以medium在新validation布局的结果为基线。沿用`visible_near`规则，候选必须满足总体近障高估数、远处低估数、近障高估P95不劣于基线，同时画内近障高估数和P95也不劣于基线；合格轮次按画内近障高估数、画内近障高估P95、总体MAE依次排序。无轮次合格就报告未选中，不延长训练。每轮和最终报告总体MAE/RMSE/bias、逐束误差、近障高估数/比例/P95/最大值，以及hitpoint画内/画外分组；画内不等于无遮挡，画外也要区分“部分表面可见”和“完全不可见”。不以总体MAE单项或训练损失选模型。

当前v2 test已看过，降为历史回归集合。建议在训练开始前，另生成两组全新、几何独立且不与既有布局同源的最终布局：一组宽墙与不同距离/横向偏置障碍；一组左右交错遮挡与有限通道。整布局封存，只在选中候选后与medium各评价一次；不可据该test再选epoch或调参。为此后续需给数据生成器增加独立`final_holdout_v1`布局集并输出独立目录；本轮未生成。test不能证明真实相机精度，后续仍需已知障碍的实机独立对照。

## 后续执行命令（本轮未运行）

下面命令假设新的最终留出目录已单独生成并冻结；在此之前，可先运行不带test参数的训练命令，但不能声称有最终留出评价。`batch-size 16`只是全网显存测速的起始候选，不是已验证可容纳值。

```bash
cd /home/lidio/quadruped_robots
source /home/lidio/anaconda3/etc/profile.d/conda.sh
conda activate abs
export CUDA_VISIBLE_DEVICES=0

python scripts/finetune_mujoco_ray_pred.py \
  --base-model archive/ray_pred_finetune/mujoco_finetune_medium_20260611/ray_pred_mujoco_finetuned_best.pt \
  --datasets logs/real_go2/20261116_d435i_ray_adaptation_data/adaptation_v2_current_mount/data/train \
  --validation-datasets logs/real_go2/20261116_d435i_ray_adaptation_data/adaptation_v2_current_mount/data/validation \
  --validation-visibility-csv logs/real_go2/20261116_d435i_ray_adaptation_data/adaptation_v2_current_mount/ray_projection_visibility.csv \
  --test-datasets logs/real_go2/20261116_d435i_ray_adaptation_data/adaptation_v2_final_holdout/data/test \
  --test-visibility-csv logs/real_go2/20261116_d435i_ray_adaptation_data/adaptation_v2_final_holdout/ray_projection_visibility.csv \
  --output-root logs/real_go2/20261116_d435i_ray_adaptation_data/training_runs \
  --run-name d435i_v2_medium_fullnet_seed20261117 \
  --device cuda:0 --trainable all --epochs 20 --batch-size 16 \
  --lr 1e-5 --weight-decay 1e-4 --near-weight 4 \
  --false-safe-weight 8 --false-danger-weight 0 \
  --selection-priority visible_near --seed 20261117 --num-workers 0
```

命令中的final_holdout目录是未来待生成路径；必须先生成并封存该独立数据，再执行整条命令。脚本会先用train/validation选轮，然后只对该test做一次medium/候选评价，test指标不参与选模。模型产物不自动替换部署引用。

### RTX 4060 Laptop 8GB资源核对

本机记录确认显卡8GB、PyTorch 2.0.1+cu118。历史layer4_fc batch32跑过，但不能证明全网batch32/320可用。v2数据约146MB；既有TorchScript best/last各约42.8MB，按两份文件估计检查点约86MB，实际可能随权重略变。全网AdamW训练峰值显存与耗时没有测量，本报告不预测运行时长。

短程测速建议：检查GPU空闲后，从medium开始，在独立`..._fullnet_benchmark`目录跑**1轮、batch16**，记录实际秒/epoch、step吞吐和`nvidia-smi`峰值显存；若OOM或余量不足，停止该次并将batch改为8后重做一次测速，不改变其他参数。测速产物不参与选模或部署。确认后再冻结正式命令与资源预算。此步骤本轮未运行。

## 本轮改动与边界

`scripts/finetune_mujoco_ray_pred.py`现在在训练入口固定Python、NumPy、Torch CPU/CUDA随机源，并为训练DataLoader显式传入带seed的generator、为worker初始化Python/NumPy种子；manifest写入seed和受控随机源。它不保证不同GPU/库版本逐位相同。只进行了py_compile及CPU合成DataLoader/随机增强序列检查，没有训练或读取新test。R1 ACTIVE、控制器、部署权重、共享仿真配置和go2_navigation均未改；未连接设备或服务器，本轮进程已退出。
