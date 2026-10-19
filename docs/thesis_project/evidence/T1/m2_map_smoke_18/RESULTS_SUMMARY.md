# 新版M2 18图B组单次诊断（非正式实验）

- 批次：`20261018_154940_m2v2_18`
- 分类：新版M2 18图B组单次地图诊断，非正式实验
- 固定参数：B / E=-0.05 / X=-0.07 / 保持关闭；目标 (7.5, 0.0)，到达半径 0.5 m
- 时限：20 仿真秒 / 120 墙钟秒；到达后站立观察 2 仿真秒
- 计划/有效任务结果：18/18
- 终局：{"ARRIVED": 14, "COLLISION_TERMINATION": 3, "DIAGNOSTIC_SIM_TIME_LIMIT": 1}
- 确认场景启动：18（上限 20）

| 序号 | 地图 | 难度 | 来源 | run_id | 终局 | 记录 | 到达距离m | 任务s | 路径m | 频率Hz | 周期缺口 | Recovery进/出 | 短反向 | 足端撞击 | 非足端碰撞 | 跌倒 | 越界步 | 站立2s | 清理 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | e01.xml | easy | M1:dev01.xml | 2a62f7489cf440209c0942cb7123c8f4 | ARRIVED | VALID | 0.4890904311750044 | 4.020000000000725 | 7.3106630360666065 | 50.26451727742194 | 0 | 0/0 | 0 | 0 | False | False | 0 | COMPLETE | True/True |
| 2 | m01.xml | middle | M2:m2dev04.xml | ecb6dc6c8eef4084aec6b61341951f7a | ARRIVED | VALID | 0.49305591869769105 | 9.622000000002455 | 8.230505754345222 | 50.13036686004589 | 0 | 25/25 | 46 | 2 | False | False | 0 | COMPLETE | True/True |
| 3 | d01.xml | difficult | M2:m2dev03.xml | 75ef29a4b4a149bd810cdba6f96100a1 | ARRIVED | VALID | 0.49318494826630577 | 6.440000000001406 | 7.803965056158399 | 50.18441358002 | 0 | 8/8 | 14 | 1 | False | False | 0 | COMPLETE | True/True |
| 4 | e02.xml | easy | M1:dev12.xml | d9f1ba0503f7472f88835abcd8cc7839 | ARRIVED | VALID | 0.4794544672920783 | 4.860000000001623 | 7.521216254085478 | 50.27718990215412 | 0 | 2/2 | 3 | 0 | False | False | 0 | COMPLETE | True/True |
| 5 | m02.xml | middle | M2:m2dev06.xml | 5df9fca6ff444dfd817aa08e2198df4a | ARRIVED | VALID | 0.48661721420110904 | 7.940000000001293 | 7.631423413665631 | 50.1468664584264 | 0 | 51/51 | 100 | 0 | False | False | 0 | COMPLETE | True/True |
| 6 | d02.xml | difficult | M2:m2dev05.xml | bc38312bed064c85931bfb505a2aa1f1 | COLLISION_TERMINATION | VALID | 4.382584893610047 | 5.498000000001836 | 4.100236006786703 | 50.300730305767594 | 0 | 7/6 | 11 | 0 | True | False | 0 | NOT_REACHED_OR_UNKNOWN | True/True |
| 7 | e03.xml | easy | M1:dev22.xml | 5b28d70b3fb646d7aec3fee45d062dcc | ARRIVED | VALID | 0.496216053856832 | 4.800000000000739 | 7.674752166769063 | 50.24930791426863 | 0 | 2/2 | 3 | 0 | False | False | 0 | COMPLETE | True/True |
| 8 | m03.xml | middle | M1:dev13.xml | f4f6a8d0fcda40efad227d968c8f17da | ARRIVED | VALID | 0.49940364279722543 | 6.6600000000013635 | 7.717850786556005 | 50.19761901055217 | 0 | 12/12 | 22 | 0 | False | False | 0 | COMPLETE | True/True |
| 9 | d03.xml | difficult | M2:m2dev02.xml | 5ef759e319f24eedbc24d21dcd268173 | ARRIVED | VALID | 0.4902131806731464 | 8.38200000000193 | 8.869726815114342 | 50.12933894635883 | 0 | 13/13 | 22 | 0 | False | False | 0 | COMPLETE | True/True |
| 10 | m04.xml | middle | M1:dev17.xml | 1ccd7a6339e2451f92372d8890a84432 | ARRIVED | VALID | 0.48662840491480386 | 5.0800000000016965 | 7.484661650420758 | 50.251026200507766 | 0 | 7/7 | 13 | 0 | False | False | 0 | COMPLETE | True/True |
| 11 | d04.xml | difficult | M1:dev03.xml | f4f07c86bec74622858cd5db007baeba | ARRIVED | VALID | 0.4992572573889851 | 6.640000000001487 | 7.470992247922706 | 50.17098533851294 | 0 | 18/18 | 33 | 0 | False | False | 0 | COMPLETE | True/True |
| 12 | m05.xml | middle | M1:dev26.xml | 7e3bae4908a04eb5afb1c960fdba0a0a | DIAGNOSTIC_SIM_TIME_LIMIT | VALID | 4.741883795087413 | 20.01999999999665 | 3.4566069600085894 | 50.066057522976834 | 0 | 5/5 | 8 | 0 | False | False | 0 | COMPLETE | True/True |
| 13 | d05.xml | difficult | M1:dev04.xml | f7dfab08bf9d4bed8da226e24f2ee55f | ARRIVED | VALID | 0.49056065003945565 | 5.66000000000127 | 7.811322246559481 | 50.234749126113854 | 0 | 4/4 | 6 | 0 | False | False | 0 | COMPLETE | True/True |
| 14 | m06.xml | middle | M1:dev29.xml | af5e52581fe24cfeb033d920bd378593 | ARRIVED | VALID | 0.47437920297660285 | 4.840000000001616 | 7.691443041051568 | 50.25232026139171 | 0 | 4/4 | 6 | 0 | False | False | 0 | COMPLETE | True/True |
| 15 | d06.xml | difficult | M1:dev08.xml | ab8014075052448383e00adc1805fab8 | ARRIVED | VALID | 0.49542874818035687 | 6.7000000000016176 | 8.29707904333173 | 50.17174653417047 | 0 | 8/8 | 14 | 0 | False | False | 0 | COMPLETE | True/True |
| 16 | d07.xml | difficult | M1:dev11.xml | 1afa93c4f34f43e0a0802320daea11bf | COLLISION_TERMINATION | VALID | 4.6445808640488915 | 18.53999999999739 | 9.164740341704265 | 50.08744196998785 | 0 | 64/64 | 121 | 0 | True | False | 0 | NOT_REACHED_OR_UNKNOWN | True/True |
| 17 | d08.xml | difficult | M1:dev19.xml | 274a7a8d38a04695a1c9dcc8ae83e30b | COLLISION_TERMINATION | VALID | 3.7984619748569552 | 7.820000000001887 | 4.644786989928792 | 50.17058858536511 | 0 | 37/37 | 72 | 2 | True | False | 0 | NOT_REACHED_OR_UNKNOWN | True/True |
| 18 | d09.xml | difficult | M1:dev24.xml | f877888ecc994a7583a741ddbfc45796 | ARRIVED | VALID | 0.4967276506322918 | 16.13999999999896 | 10.983438232662303 | 50.0823238225825 | 0 | 35/35 | 65 | 2 | False | False | 0 | COMPLETE | True/True |

次数、覆盖率、终局与安全UNKNOWN分开保留；单次地图诊断不估计成功率、不比较阈值、不自动删图。
