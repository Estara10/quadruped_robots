# 新版M2：18张开发地图

当前入口：M2_20–M2_22（easy）、M2_23–M2_28（middle）、M2_29–M2_37（difficult），均为.xml文件；旧e/m/d文件名仍兼容。 [旧名对照](../../MAP_FILENAME_CROSSWALK.md)
M1源地图未移动或修改；旧M2别名保留历史，不与新名重复计入实验。难度是开发标签，不是风险概率或已证实的因果关系。

| 当前地图 | 重命名前内部编号 | 来源 | 难度 | 理由 |
|---|---|---|---|---|
| [M2_20.xml](e01.png) | M2_17.xml | M1 M1_01.xml | easy | 低风险参考，最近两档均无Recovery |
| [M2_21.xml](e02.png) | M2_18.xml | M1 M1_12.xml | easy | 中等几何密度下的低风险参考 |
| [M2_22.xml](e03.png) | M2_19.xml | M1 M1_22.xml | easy | 较密布局下的低风险参考，有一定Recovery活动 |
| [M2_23.xml](m01.png) | M2_04.xml | M2 M2_04.xml | middle | 采用用户修改后的r5；旧几何成绩不用于新图风险判定 |
| [M2_24.xml](m02.png) | M2_06.xml | M2 M2_06.xml | middle | 旧M2第二轮24/24到达，但切换活动较多 |
| [M2_25.xml](m03.png) | M2_13.xml | M1 M1_13.xml | middle | 最近4次均到达，但较宽滞回的短反向更多 |
| [M2_26.xml](m04.png) | M2_14.xml | M1 M1_17.xml | middle | 最近4次均到达、切换频繁；历史有跌倒记录 |
| [M2_27.xml](m05.png) | M2_15.xml | M1 M1_26.xml | middle | 最近4次均到达，两档短反向频率差异明显 |
| [M2_28.xml](m06.png) | M2_16.xml | M1 M1_29.xml | middle | 最近4次均到达、切换频繁；旧实验有碰撞/超时 |
| [M2_29.xml](d01.png) | M2_03.xml | M2 M2_03.xml | difficult | 旧M2第二轮17到达7碰撞；最新两档有超时差异 |
| [M2_30.xml](d02.png) | M2_05.xml | M2 M2_05.xml | difficult | 旧M2第二轮9到达13碰撞2超时 |
| [M2_31.xml](d03.png) | M2_02.xml | M2 M2_02.xml | difficult | 用户指定difficult；旧M2第二轮23/24到达，切换活动较多 |
| [M2_32.xml](d04.png) | M2_07.xml | M1 M1_03.xml | difficult | 两档最近3到达1碰撞；旧400次也有碰撞和超时 |
| [M2_33.xml](d05.png) | M2_08.xml | M1 M1_04.xml | difficult | 两档最近3到达1碰撞，终局有差异 |
| [M2_34.xml](d06.png) | M2_09.xml | M1 M1_08.xml | difficult | 两档最近3到达1碰撞，终局有差异 |
| [M2_35.xml](d07.png) | M2_10.xml | M1 M1_11.xml | difficult | 两档最近2到达2碰撞；旧实验有较多超时 |
| [M2_36.xml](d08.png) | M2_11.xml | M1 M1_19.xml | difficult | 两档最近3到达1碰撞，Recovery占用较高 |
| [M2_37.xml](d09.png) | M2_12.xml | M1 M1_24.xml | difficult | 两档最近3到达1碰撞，终局有差异 |

构成：difficult 9张、middle 6张、easy 3张；用户明确原M2 03、05、02作为d01、d02、d03。
m01是修改后的旧M2 dev04，暂归middle但仍待实测；M1 dev15/dev30作为压力备选保留，不纳入此18图。

新版dev04为r5，与旧实验不同；01和08障碍间距约0.514m，低于原0.60m候选规则，参考路径净距约0.764m。
复制到M2不产生独立地图样本：后续按源地图/几何区组统计，不能将M1与对应M2别名当成两张独立地图。
没有使用留出集；没有启动216次扫描。今后阈值、筛选规则、顺序和重试规则需在运行前单独固定。

生产登记状态：18 renamed production static identities registered; MuJoCo main build complete; no policy runs。静态加载与构建不等于策略可达性验证。
