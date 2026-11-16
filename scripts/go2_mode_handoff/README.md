# Go2趴伏控制权交接检查

**已冻结：2026-11-04实机检查中，恢复ai后机器人起立，未满足保持趴伏目标。当前程序在初始化DDS之前直接拒绝运行，返回2；下文仅保留已执行流程说明，不是可继续使用的实机入口。** [实际结果](../../docs/thesis_project/real_go2/20261104_MODE_HANDOFF_RESULT.md)。

**该程序会实际调用ReleaseMode/SelectMode，不是只读观察器。** 仅用于本台已确认原生ai模式的Go2 EDU，在机身可靠支撑、现场监护、四肢活动范围清空后执行一次有限检查。服务切换可能使关节松弛或产生电机响应。程序不创建LowCmd发布器，不加载模型，不发站立、前进或后退命令。

由独立状态观察器先记录基线，并在交接全程继续采样。程序确认form=0、mode=ai后仅发一次ReleaseMode；查询确认释放，观察约2秒，然后恢复进入前的ai模式并查询确认。仅对SDK明确的BUSY=7002作有限等待重试，不重复整个交接试验。恢复未确认返回3，保持支撑并处理人工恢复，不继续站立。

SIGINT/SIGTERM会进入恢复流程；这不是物理急停，不能保证恢复成功，亦不能覆盖SIGKILL、电脑掉电或网络中断。机器人已有其他原生模式时不会释放；恢复时出现其他活动模式不强行覆盖。

构建（不运行机器人）：

```bash
cmake -S scripts/go2_mode_handoff -B /tmp/go2_mode_handoff_build \
  -DCMAKE_PREFIX_PATH=/home/lidio/Libraries/unitree_sdk2 -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/go2_mode_handoff_build --parallel 2
```

实际调用只在现场条件和本次执行范围明确后安排，不应把它当日常只读检测入口。
