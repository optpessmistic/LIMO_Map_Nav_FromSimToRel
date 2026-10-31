# run20 独立离线结果核对

运行代码 `b80aa8de6799520b16fb7f70dcb1ac43d21d5f2a`；epoch `76f2f4d2-461c-469f-b8b4-40405854f92d`。本核对仅读取已闭合证据，未启动 ROS/Gazebo，也未重新解码录包。

本轮完成 5 个 frontier 和 1 个 home 导航终态，6 个均报告成功；5 段观察窗在线 ledger 各接受 3 帧。任务 `done`、`returned_pose_verified=true`、`map_complete=true`，但 `visual_coverage_complete=false`、`target_definition_verified=false`、`mission_success=false`。`done` 只表示流程结束，不是完整项目验收成功。

## 在线观察窗与独立证据

| 窗口 | 在线接受帧数 | 跨度/s | 位置误差/mm | 航向误差/rad | 末帧缺失原始数据 |
|---|---:|---:|---:|---:|---|
| goal-1 | 3 | 0.200 | 28.026 | 0.141731 | depth |
| goal-2 | 3 | 0.600 | 28.591 | 0.137111 | depth |
| goal-3 | 3 | 0.200 | 28.977 | 0.141998 | rgb |
| goal-4 | 3 | 0.500 | 20.283 | 0.109193 | depth |
| goal-5 | 3 | 0.200 | 27.171 | 0.147619 | depth |

闭合录包末帧审计为 **0/5 完整一致**：目标戳仅找到 4 张 RGB 与 1 张 depth；4 段缺 depth，第 3 段缺 RGB。5 组 base/camera TF 都能按原戳重建一致，无所选动态 TF 或静态 TF 冲突。这不能替代缺失的 RGB-D 原始证据，也不能把在线 15 帧计数当作独立验收。保留这些失败样本，不改抽样时间点。

## 返航与静止

地图内 home 到 terminal：XY 31.489 mm，yaw 0.131790 rad。frontier 观察到位阈值仍 0.05 m，home 验证阈值实际为 0.15 m / 0.26 rad，二者应区分。

原生模型在初始 survey 后、mission home 附近的位姿为 (-0.000007992, 0.000000510, yaw=-0.023028349)；最后为 (0.037921490, 0.000019503, yaw=-0.154309537)。相对该 home 原生 XY 位移 37.929 mm，yaw 变化 -0.131281 rad。若相对 survey 前近零原始朝向，yaw 变化为 -0.154310 rad；不混用两种起点。

终态后最终原生静止尾窗有 967 样本，跨度 29.993 sim s，XY/yaw 跨度均为零，最大采样间隔 0.036 s。

## 地图与目标的结论边界

最终规划器 frontier=0、reachable=2575、unobserved=0；这是运行中的可达区域几何终止判据，没有独立证明整个迷宫的地图精度。视觉 uncovered=2575、visual coverage=false；识别到 2 个 confirmed 红色替身不代表完成正式目标识别。target_count_prior=null。起点已披露 0.30 m 半径、减 0.05 m 裕量的 clearance prior；它不是传感器观测，raw SLAM map 未由该 prior 修改。

## 不用目标拟合的坐标锚点与描述性目标误差

起点原生静止窗 93.401–100.070 sim s 共 216 个样本；采用 100.075 s 同戳 map→base_link / map→odom / odom→base_link 链，与 home_recorded 位姿完全相等。原生前后括点 100.070/100.101 s 无位姿变化。展开 URDF 的 base_footprint→base_link 固定关节只有 0.15 m Z 偏移，XY/yaw 为零。

据此固定 world_xy = R(0.041872907153) map_xy + (-0.019672710523, 0.002884567605)。未使用任何目标位置拟合，也未用最终返航点重新拟合。调用 `tools/evaluate_targets.py`，显式 red_box_surrogate→red_object、0.20 m 描述性配对阈值，2 个预测匹配 2 个 world 目标，原始 XY 残差为 22.683 mm, 4.171 mm。该阈值是在审计时用于关联，不是预注册的验收通过标准。

锚点仅在静止采样窗内没有观测到变化；未建立置信区间。末段对照采用 255.575 s TF 与静止原生括点 255.569/255.601 s。起点到末段 map→odom XY 改变量 7.857 mm、yaw 改变量 -0.000509 rad；固定起点锚点映射末段 map 位姿与原生位姿残差 7.590 mm。目标是跨时刻累积的 map 坐标，不是随位姿图回溯优化的地标，必须保留 SLAM 校正影响。

当前 RGB-D 输出可见表面点，metadata 是 0.03×0.06×0.20 m 盒子的中心；没有做隐藏的表面到中心修正。误差只比较平面 XY，不验证 z/高度、物体身份、识别召回或视觉覆盖。所有 world target 坐标只用于本次运行结束后的离线评价，没有送入控制。

## 唯一实例、参数、导出与收尾

early、before-motion、terminal 三个原生 scene 查询均仅有一个 LIMO，ID=114；observer 采样中也只有该 ID。结论是这些快照和采样一致，不推导任意未观测时刻的永久唯一性。

实际参数查询确认 Nav2 XY=0.03 m、yaw=0.26 rad、general_goal_checker.stateful=false、RPP.stateful=false；mission margin=0.05 m、require_processed_inspection=true、3 帧且最小跨度 0.2 s。Nav2 XY 参数也影响 RPP 末端朝向切换，不把本轮结果解释为只改变一个数学验收判据。

save_map/serialize_map 均 result=0；原始 OccupancyGrid 保存 4452 个单元，宽高 84×53，数据长度正确；PGM/YAML/posegraph/data 文件哈希与 final-save 记录一致。

录包目录含 2 个文件（1 个 MCAP），共 55,550,360 B；metadata 为 335,008 条、219.904 sim s。record supervisor 持续 513.497 wall s，外层 SIGINT 后 recorder exit0；限制为 1650 wall s / 8 GiB，terminal fallback tail=120 wall s。

录包日志累计 transport lost=2355；它是全录制累计值，不能分配给某观察窗、某图像通道，也不能单凭该值确定缺图的具体原因。闭合包扫描达到存储流末端不等于无丢失或全部数据完整。

编排器总 wall 645.380 s，errors=[]、all_owned_groups_empty=true，内层 recorder 进程组复核为空，无 TERM/KILL 升级。**perception exit1**：其日志为 SIGINT 后重复 `rclpy.shutdown()` 导致 `rcl_shutdown already called`；因此不能写‘所有组件均优雅退出’。已无遗留自有进程组与组件退出码全零是不同结论。

下一轮应先改善原始 RGB-D 留证完整性，并保留本轮缺图失败；不能以换抽样点来抹去失败。正式物品、视觉范围标定和未知场景视觉搜索验收仍未完成，本报告也不声称实车通过。

机器可读数据及来源哈希见同目录 `run20-outcome-review.json`；完整末帧审计来源为 `outputs/ros_smoke/run20-arrival-margin/inspection-audit/audit.json`。
