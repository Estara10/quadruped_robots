# 2026-11-04 趴伏交接检查准备：实际切换未执行

> 本页为执行前历史记录。后续用户明确同意后已进行一次实际检查，发生意外起立；当前结果与暂停边界见[实际记录](20261104_MODE_HANDOFF_RESULT.md)。

用户确认机器人处于L2+A趴伏状态、范围内无杂物、遥控器在手边，并要求先不前进后退。已准备有限原生服务释放/恢复程序，但实际模式切换被自动审批拒绝：用户确认准备条件，尚未明确批准这一具体物理操作。没有执行ReleaseMode/SelectMode，没有运行控制器/模型，没有LowCmd、站立或前后运动指令。

## 已完成的软件与离线检查

新增[scripts/go2_mode_handoff](../../../scripts/go2_mode_handoff/README.md)，程序仅支持本次已知连接：domain0、显式非lo网卡，且CheckMode必须确认form0、原生模式ai；否则在释放前拒绝。单次ReleaseMode后查询确认释放，观察约2秒；无论正常结束、异常还是SIGINT/SIGTERM，已发起释放就进入恢复原ai流程并查询。SDK明确BUSY=7002时可有限重试（最多5轮恢复查询/必要请求），不重复整个交接试验。出现其他活动模式不强行覆盖；恢复未确认返回3，不宣称遥控可用。

程序不创建LowCmd发布器，不加载模型，不发站立/移动命令。但服务释放/恢复本身可能改变电机支撑，必须在机身可靠支撑和现场监护条件下执行；退出恢复不能覆盖SIGKILL、电脑掉电、网络失联。此程序不证明低层LowCmd接管或实际遥控器接管成功。

构建通过；直接编译生产源码、替代SDK的8个离线场景通过：成功交接、初始模式不符、查询失败、释放未生效、释放应答丢失但恢复、恢复BUSY后成功、恢复拒绝明确失败、SIGINT后恢复。离线测试没有建立DDS或访问机器人。真实恢复时序仍未知。

## 已完成的只读部分

独立状态观察器JSON10Hz及read_only=true的LowState遥控字段监听覆盖约60秒；两者退出码0。实际切换命令在创建进程前被拒绝，因此这些记录全部是未切换的基线，不能解读为释放前后比较。

新只读CheckMode成功：CheckMode return=0 form=0 name=ai。状态快照599条，末次显示LowState约499.95Hz、SportModeState约499.68Hz；最大接收间隔分别5.198/7.409ms。

[状态基线](../../../logs/real_go2/20261104_mode_handoff/state.jsonl)、[遥控字段基线](../../../logs/real_go2/20261104_mode_handoff/readonly_remote_keys.txt)、[只读模式查询](../../../logs/real_go2/20261104_mode_handoff/readonly_mode_query.txt)、[未执行原因](../../../logs/real_go2/20261104_mode_handoff/NOT_EXECUTED.txt)、[汇总](../../../logs/real_go2/20261104_mode_handoff/summary.json)。

## 当前续接点

自动审批要求用户明确同意具体模式切换，方可再次申请执行。已关闭本次只读记录，确认后续无本轮进程残留；当前仍是原生ai，未实际释放。本轮没有验证释放后LowState/遥控字段是否持续、SportModeState是否停更或原生恢复是否成功，也没有验证遥控器接管或稳定站立。不改变R1唯一ACTIVE任务和仿真结论。
