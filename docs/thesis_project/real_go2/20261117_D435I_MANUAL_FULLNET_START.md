# D435i全网适配：手动启动准备

## 用户确认的最终预算：300轮

用户完成1轮全网batch16测速：train+validation约8.903秒，运行总计约11.95秒，CUDA allocated峰值304.6MiB、reserved峰值386MiB。未通过选模是该轮总体近障P95为1.498m，高于medium基线1.445m，不是训练失败；测速未读取最终test。

用户随后明确选择300轮，覆盖下文历史20轮准备预算。启动器train现为300轮，独立目录`training_runs/d435i_fullnet_300epoch_20261117`；medium起点、全网、batch16、lr1e-5、原损失和visible_near规则不变。每轮仅更新best/last，最终test只在选模后评价。按测速线性估算约45分钟，实际受温度和负载影响；不称原作者训练复现。本助手仅修改命令，未启动训练。

```bash
cd /home/lidio/quadruped_robots
bash scripts/run_d435i_fullnet_20261117.sh train
```

2026-11-17。用户要求自行输入训练命令；本轮只做离线准备，没有启动训练/测速或连接设备。

## 已准备

- 复用生成器，新增`final_holdout_v1`，仅生成test，不重复train/validation。两套新的障碍布局：宽墙与偏置目标、左右错列遮挡通道；与旧v2布局XML不同，180个位姿、360张clean/masked图，约21.6MB，生成35.35秒，进程exit0。既有输入有限性/范围、成对标签和布局检查通过。保存目录`logs/real_go2/20261117_d435i_fullnet_preparation/final_holdout_v1`。
- 只检查几何和数据格式，没有预测误差或模型评价；该集合留给选模后的最终medium/候选比较。掩码仍复用历史test来源，并非新实测缺陷，不能称普遍独立实机验证。相机安装与盲区局限沿用v2。
- 微调器新增每轮train+validation耗时、整次耗时和Torch峰值allocated/reserved显存记录，便于用户测速。峰值不是整机所有进程的显存总量。未改变模型、损失或选模规则。
- `scripts/run_d435i_fullnet_20261117.sh`把benchmark与train分开，均需用户手动启动，不自动衔接。输出存在则拒绝覆盖；benchmark不读test。

## 第一步：用户测速

首次用户启动在加载数据/训练前失败：新增显存统计调用发生于CUDA设备初始化前，报`Invalid device argument 0: did you call init?`。这是本轮准备代码的问题，不是显存不足。已在reset前显式set_device；独立执行这两步通过，没有运行训练。首次失败.log保留，启动脚本支持第二参数指定新run_name，避免覆盖：

```bash
rtk proxy bash scripts/run_d435i_fullnet_20261117.sh benchmark d435i_fullnet_benchmark_retry1_20261117
```

此修复仅确认初始化与统计可执行，全网训练显存/耗时仍待用户重启测速确认。

```bash
cd /home/lidio/quadruped_robots
rtk proxy bash scripts/run_d435i_fullnet_20261117.sh benchmark
```

medium起点，完整网络可训练，batch16、1轮、AdamW lr1e-5、既定加权log2 MSE、seed20261117。结束后查看manifest的耗时/峰值显存，再决定是否适合正式运行。1轮含初始化、验证和写模型开销，不代表长期精确吞吐。若OOM，保留日志并停止，不自行连续重跑。

输出目录`logs/real_go2/20261117_d435i_fullnet_preparation/training_runs/d435i_fullnet_benchmark_20261117`。没有合格epoch也可能正常exit0，须看selection_status。

## 第二步：测速后手动正式适配

```bash
cd /home/lidio/quadruped_robots
rtk proxy bash scripts/run_d435i_fullnet_20261117.sh train
```

此入口明确采用昨天准备报告的**20轮全网适配预算**，不是原作者302轮ImageNet初始化训练复现。保持medium、全网解冻、batch16、lr1e-5、近障加权AdamW与visible_near规则，best按验证选取；最后test只评价medium与best。没有合格best则不运行test，不以last冒充通过。现有v2 train/validation继续使用，新final仅在选定后使用。若用户希望302轮或其他预算，先明确修改计划与run_name，不静默把20轮改名为原作者完整复现。

输出目录`.../training_runs/d435i_fullnet_20epoch_20261117`。本轮不运行这两条命令；测速模型不部署，也不拿新test挑参数。后续报告所有近障/画内外局限，不只报总体MAE。

## 本轮检查与状态

Python语法、CLI help与shell语法通过；数据生成使用既有审计并核对新布局与v2不同。未训练、未测GPU吞吐，未承诺耗时或显存足够。生成进程退出，无本轮常驻进程；未修改R1 ACTIVE、控制器、部署权重、旧数据或scene.txt，未提交Git。
