# 新版M2：18张开发地图

仅本清单中的m2dev02–19为本轮候选；m2dev01保留为历史，不自动加入实验。
M1源地图未移动或修改；新增M2别名保留源几何。风险标签是开发选择依据，不是风险概率或已证实的因果关系。

| 当前地图 | 来源 | 标签 | 理由 |
|---|---|---|---|
| [m2dev02.xml](m2dev02.png) | M2 m2dev02.xml | medium_switching | 旧M2第二轮23/24到达，切换活动较多 |
| [m2dev03.xml](m2dev03.png) | M2 m2dev03.xml | high_candidate | 旧M2第二轮17到达7碰撞；最新两档有超时差异 |
| [m2dev04.xml](m2dev04.png) | M2 m2dev04.xml | challenge_unverified | 采用用户修改后的r5；旧几何成绩不用于新图风险判定 |
| [m2dev05.xml](m2dev05.png) | M2 m2dev05.xml | high_candidate | 旧M2第二轮9到达13碰撞2超时 |
| [m2dev06.xml](m2dev06.png) | M2 m2dev06.xml | medium_switching | 旧M2第二轮24/24到达，但切换活动较多 |
| [m2dev07.xml](m2dev07.png) | M1 dev03.xml | high_candidate | 两档最近3到达1碰撞；旧400次也有碰撞和超时 |
| [m2dev08.xml](m2dev08.png) | M1 dev04.xml | high_candidate | 两档最近3到达1碰撞，终局有差异 |
| [m2dev09.xml](m2dev09.png) | M1 dev08.xml | high_candidate | 两档最近3到达1碰撞，终局有差异 |
| [m2dev10.xml](m2dev10.png) | M1 dev11.xml | high_candidate | 两档最近2到达2碰撞；旧实验有较多超时 |
| [m2dev11.xml](m2dev11.png) | M1 dev19.xml | high_candidate | 两档最近3到达1碰撞，Recovery占用较高 |
| [m2dev12.xml](m2dev12.png) | M1 dev24.xml | high_candidate | 两档最近3到达1碰撞，终局有差异 |
| [m2dev13.xml](m2dev13.png) | M1 dev13.xml | medium_switching | 最近4次均到达，但较宽滞回的短反向更多 |
| [m2dev14.xml](m2dev14.png) | M1 dev17.xml | medium_switching | 最近4次均到达、切换频繁；历史有跌倒记录 |
| [m2dev15.xml](m2dev15.png) | M1 dev26.xml | medium_switching | 最近4次均到达，两档短反向频率差异明显 |
| [m2dev16.xml](m2dev16.png) | M1 dev29.xml | medium_switching | 最近4次均到达、切换频繁；旧实验有碰撞/超时 |
| [m2dev17.xml](m2dev17.png) | M1 dev01.xml | low_reference | 低风险参考，最近两档均无Recovery |
| [m2dev18.xml](m2dev18.png) | M1 dev12.xml | low_reference | 中等几何密度下的低风险参考 |
| [m2dev19.xml](m2dev19.png) | M1 dev22.xml | low_reference | 较密布局下的低风险参考，有一定Recovery活动 |

构成：高风险候选8张＋修改后尚无运行证据的dev04一张＋中风险/切换敏感6张＋低风险参考3张。
没有为凑9张高风险而将新版dev04标成已验证高风险。dev15/dev30原图作为压力备选保留，不纳入此18图。

新版dev04为r5，与旧实验不同；01和08障碍间距约0.514m，低于原0.60m候选规则，参考路径净距约0.764m。
复制到M2不产生独立地图样本：后续按源地图/几何区组统计，不能将M1与对应M2别名当成两张独立地图。
没有使用留出集；没有启动216次扫描。今后阈值、筛选规则、顺序和重试规则需在运行前单独固定。

生产登记状态：18 production static identities registered; MuJoCo main build complete; no policy runs。静态加载与构建不等于策略可达性验证。
