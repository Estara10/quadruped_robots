# 新版M2：18张开发地图

当前入口：e01–e03（easy）、m01–m06（middle）、d01–d09（difficult），均为.xml文件。
M1源地图未移动或修改；旧M2别名保留历史，不与新名重复计入实验。难度是开发标签，不是风险概率或已证实的因果关系。

| 当前地图 | 旧M2名 | 来源 | 难度 | 理由 |
|---|---|---|---|---|
| [e01.xml](e01.png) | m2dev17.xml | M1 dev01.xml | easy | 低风险参考，最近两档均无Recovery |
| [e02.xml](e02.png) | m2dev18.xml | M1 dev12.xml | easy | 中等几何密度下的低风险参考 |
| [e03.xml](e03.png) | m2dev19.xml | M1 dev22.xml | easy | 较密布局下的低风险参考，有一定Recovery活动 |
| [m01.xml](m01.png) | m2dev04.xml | M2 m2dev04.xml | middle | 采用用户修改后的r5；旧几何成绩不用于新图风险判定 |
| [m02.xml](m02.png) | m2dev06.xml | M2 m2dev06.xml | middle | 旧M2第二轮24/24到达，但切换活动较多 |
| [m03.xml](m03.png) | m2dev13.xml | M1 dev13.xml | middle | 最近4次均到达，但较宽滞回的短反向更多 |
| [m04.xml](m04.png) | m2dev14.xml | M1 dev17.xml | middle | 最近4次均到达、切换频繁；历史有跌倒记录 |
| [m05.xml](m05.png) | m2dev15.xml | M1 dev26.xml | middle | 最近4次均到达，两档短反向频率差异明显 |
| [m06.xml](m06.png) | m2dev16.xml | M1 dev29.xml | middle | 最近4次均到达、切换频繁；旧实验有碰撞/超时 |
| [d01.xml](d01.png) | m2dev03.xml | M2 m2dev03.xml | difficult | 旧M2第二轮17到达7碰撞；最新两档有超时差异 |
| [d02.xml](d02.png) | m2dev05.xml | M2 m2dev05.xml | difficult | 旧M2第二轮9到达13碰撞2超时 |
| [d03.xml](d03.png) | m2dev02.xml | M2 m2dev02.xml | difficult | 用户指定difficult；旧M2第二轮23/24到达，切换活动较多 |
| [d04.xml](d04.png) | m2dev07.xml | M1 dev03.xml | difficult | 两档最近3到达1碰撞；旧400次也有碰撞和超时 |
| [d05.xml](d05.png) | m2dev08.xml | M1 dev04.xml | difficult | 两档最近3到达1碰撞，终局有差异 |
| [d06.xml](d06.png) | m2dev09.xml | M1 dev08.xml | difficult | 两档最近3到达1碰撞，终局有差异 |
| [d07.xml](d07.png) | m2dev10.xml | M1 dev11.xml | difficult | 两档最近2到达2碰撞；旧实验有较多超时 |
| [d08.xml](d08.png) | m2dev11.xml | M1 dev19.xml | difficult | 两档最近3到达1碰撞，Recovery占用较高 |
| [d09.xml](d09.png) | m2dev12.xml | M1 dev24.xml | difficult | 两档最近3到达1碰撞，终局有差异 |

构成：difficult 9张、middle 6张、easy 3张；用户明确原M2 03、05、02作为d01、d02、d03。
m01是修改后的旧M2 dev04，暂归middle但仍待实测；M1 dev15/dev30作为压力备选保留，不纳入此18图。

新版dev04为r5，与旧实验不同；01和08障碍间距约0.514m，低于原0.60m候选规则，参考路径净距约0.764m。
复制到M2不产生独立地图样本：后续按源地图/几何区组统计，不能将M1与对应M2别名当成两张独立地图。
没有使用留出集；没有启动216次扫描。今后阈值、筛选规则、顺序和重试规则需在运行前单独固定。

生产登记状态：18 renamed production static identities registered; MuJoCo main build complete; no policy runs。静态加载与构建不等于策略可达性验证。
