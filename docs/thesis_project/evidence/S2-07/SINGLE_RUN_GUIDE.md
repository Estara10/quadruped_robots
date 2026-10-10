# S2-07 单次到达后站立诊断说明

本流程仅用于 MuJoCo 非正式诊断，不是正式性能实验或实机操作。S2-07 本轮授权的两次场景运行已完成；不要为本任务再启动额外运行。后续若另有书面授权，先确认场景、配置、唯一 writer 和采集器就绪，并在每次运行后核对进程退出。

## 固定诊断设置

- 切换组 A，candidate `paper_faithful_switch`，entry/exit threshold `-0.05`；hysteresis 和 hold 均关闭。
- 目标 `(7, 0) m`，到达半径 `0.5 m`；运动仿真时限 `20 s`、wall cap `120 s`。
- 站立观察默认 `5` 仿真秒，参数范围 `0–30 s`；单独墙钟保护默认 `30 s`，范围 `1–60 s`。墙钟保护仅防止仿真暂停时无限等待。
- 场景仅通过 `--scene` 指定；本任务两次场景为 `scene_flat.xml` 与 `scene_random_04.xml`。

## 授权运行时命令模板

在仓库根目录执行；按场景使用各自的 output root，脚本会在其下创建唯一 run_id 子目录。按需要将场景名替换为获准场景：

```bash
rtk proxy /usr/bin/python3 -u scripts/s1_run_diagnostic.py \
  --output-root logs/manual_scene_runs/scene_flat \
  --overlay quadruped_ros2_control_humble/install/go2_description \
  --scene scene_flat.xml --goal-x 7 --sim-limit-s 20 --wall-cap-s 120 \
  --standing-observation-sim-s 5 --standing-observation-wall-cap-s 30 \
  --classification 'bounded nonformal S2-07 arrival and standing hold'
```

健康 ARRIVED 路径会记录减速、站稳确认和保持物理样本，在 MuJoCo 画面显示 `ARRIVED - HOLDING STAND`；保持足够的仿真时间后，先关闭 MuJoCo，再关闭 controller/ROS。故障或急停仍可抢占，不应关闭安全检查。正常路径不发送趴下、卸力或 PASSIVE 请求。

## 结果分析

运行结束后可用分析器生成 motion metrics 和 timeline。当前系统 Python 的 Matplotlib 与 NumPy 二进制版本不兼容；本轮使用已存在的 Anaconda Python 3.13 / Matplotlib 3.10.0，未安装组件：

```bash
rtk proxy /home/lidio/anaconda3/bin/python3 scripts/s1_analyze_run.py \
  --runs-root logs/manual_scene_runs/scene_flat \
  --output-dir logs/manual_scene_runs/scene_flat/analysis \
  --task-label S2-07
```

该分析器按 ARRIVED 终止周期封口运动指标；站立观察在 `run_context.json` 的 `lifecycle.standing_observation` 和 `lifecycle.normal_shutdown` 中另行保存，不计入运动统计。站立图像为 MuJoCo framebuffer 截图；若截图缺失，应如实报告，不得从其他运行的图片代替。
