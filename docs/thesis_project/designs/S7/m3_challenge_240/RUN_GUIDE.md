# M3 挑战图四组比较：运行命令

> 当前说明（2026-10-23）：本批次已COMPLETE，240项已完成且原始记录保留；以下启动/恢复命令仅作历史操作参考，不要重新派发。结果见[最终报告](../../../evidence/S7-02/M3_ABCD_240_FINAL_REPORT_20261023.md)，后续见[新规划](../../R1/PLAN.md)。

用户已审看 M3 v1.4 的 30 张地图，并同意按每图 A/B/C/D 各两次运行。冻结矩阵为本目录的 `matrix.csv`、`matrix.json`；独立批次授权为 `authorization.json`。240 项的次序沿用原 M1 批次的平衡组序，将 `holdoutNN` 对应为 `challengeNN`。M3 是利用历史结果选出的挑战图，分析时与原 M1 批次分开展示。

在仓库根目录执行。此命令串行运行 240 项；合法碰撞、跌倒和超时都保存为结果。MuJoCo 窗口隐藏、RViz 关闭，仍需要可用的 X11/OpenGL。到达后观察 2 仿真秒。保持原始逐帧和 ROS 日志，不在本批结束后删除。

```bash
rtk proxy /usr/bin/python3 -u scripts/run_s7_formal_240.py \
  --matrix-csv docs/thesis_project/designs/S7/m3_challenge_240/matrix.csv \
  --matrix-json docs/thesis_project/designs/S7/m3_challenge_240/matrix.json \
  run \
  --batch-root logs/08_formal_experiment/M3_ABCD_240/m3-abcd240-20261023 \
  --batch-id m3-abcd240-20261023 \
  --authorization docs/thesis_project/designs/S7/m3_challenge_240/authorization.json \
  --display :0 \
  --xauthority /run/user/1000/gdm/Xauthority
```

若返回 `PAUSED`，先查询状态和错误原因，不直接重复启动：

```bash
rtk proxy /usr/bin/python3 scripts/run_s7_formal_240.py \
  --matrix-csv docs/thesis_project/designs/S7/m3_challenge_240/matrix.csv \
  --matrix-json docs/thesis_project/designs/S7/m3_challenge_240/matrix.json \
  status --batch-root logs/08_formal_experiment/M3_ABCD_240/m3-abcd240-20261023
```

确认进程已退出并处理暂停原因后，使用与首次运行相同的参数，在命令最后增加 `--resume`。工程替代仍按已有规则登记；合法任务终局不重跑。不要对原 M1 批次目录执行这些命令。

批次 `COMPLETE` 后运行统一分析：

```bash
rtk proxy /usr/bin/python3 scripts/analyze_s7_formal_240.py \
  --matrix-csv docs/thesis_project/designs/S7/m3_challenge_240/matrix.csv \
  --matrix-json docs/thesis_project/designs/S7/m3_challenge_240/matrix.json \
  --batch-root logs/08_formal_experiment/M3_ABCD_240/m3-abcd240-20261023
```

分析保存在该批次的 `analysis/formal_summary/`。它复用既有安全任务完成、UNKNOWN、地图等权和地图簇 bootstrap 口径，并将数据集标为 M3 挑战图。尚未运行时不能引用任何 M3 组间效果。
