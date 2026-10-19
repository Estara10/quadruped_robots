# T2 48项组合报告计数更正记录

日期：2026-10-19。只更正派生报告文字与摘要，不改变逐run CSV、run context、terminal、metrics或任何原始记录。

| 位置/旧表述 | 逐run表和terminal核对 | 修正后 | 影响 |
|---|---|---|---|
| 组合报告“d07：A 1/2到达、1次碰撞、1次时限” | d07/A两个run分别为`DIAGNOSTIC_SIM_TIME_LIMIT`和`COLLISION_TERMINATION`；无ARRIVED | d07/A为0/2到达、1次碰撞终止、1次仿真时限 | 旧句将三种终局写在仅两次运行中，属于摘要计数错误；总体48项计数不变 |
| 组合报告“e03、m02：四组均4/4到达” | 每图每组各2个run，均为ARRIVED | e03和m02各组均2/2到达；每图跨四组总计8/8 | 旧句把每图跨组总数误作每组重复数；总体计数不变 |
| m06 D repeat1“Recovery约44.7%、71条边沿” | 对应run `ffa12a3c9f81441c8d891a78588d92bf` 的逐run派生值 | Recovery有效时间占比42.5%，全部模式边沿70；repeat2为46.9%、72 | 改正派生摘要；原始帧及分析基础数据不变 |
| D组超时机制描述 | 四个D run均逐帧核对E/X/H、周期、仿真时钟、模式边沿和保持episode审计 | 四条均为E=−0.05、X=−0.10、H=0.20；完整episode无保持违规；d07/D repeat1有一条在H前被终局截断的右删失episode | 仅将直接证据范围内的机制判断加入离线复核报告；不将截断episode补作通过或违规 |

碰撞终局与`nonfoot_collision`保持为不同字段：48项中`COLLISION_TERMINATION`为A2/B2/C1/D0，`nonfoot_collision=True`也分别为2/2/1/0且UNKNOWN为0；本批数字相同不意味着定义合并。UNKNOWN对比仍按候选最不利=候选上界−参考下界、候选最有利=候选下界−参考上界。成功子集到达时间不用于宣称总体加速。站立、controller状态确认、进程退出及MuJoCo命令消费各自分开表述。

原48项派生表与可视化继续在[结果索引](../combined_check_48/RESULT_INDEX.md)中链接。此记录更正汇总文字，不重跑场景。
