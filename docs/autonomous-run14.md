# run14：首次完成探索、替身标注与返航

2026-10-28，代码 `7b300d9`，地图代次 `bd3559e3-9e9a-46b2-b15f-19c7dd68e6d3`。本轮在 Gazebo Harmonic 的 `smoke_maze` 中完成四个探索目标和一次返航，确认两个红色仿真替身。任务记录为 `done`，`map_complete=true`、`returned_pose_verified=true`，没有失败条目。

**`mission_success=false` 保持原值**：视觉搜索覆盖尚未验证，正式物品定义尚未确认。单轮红盒全部匹配，不能替代完整未知物品搜索，也不能代表实车通过。

## 改动和运行条件

本轮将 run13 的 `FollowPath` 整块替换为 Nav2 1.3.13 的 [Regulated Pure Pursuit](https://github.com/ros-navigation/navigation2/tree/1.3.13/nav2_regulated_pure_pursuit_controller)。该块以外的导航参数与 run13 相同。RPP 使用 .15 m/s 期望速度、.30 m 固定前视距离、.4 rad/s 原地转向速度，开启转向与碰撞预测，关闭倒车；速度平滑器和 collision monitor 继续串接到最终底盘命令。

保留实验 SLAM 粗回环窗 1 m、距离方差惩罚 .05；它们不是经过跨迷宫验证的通用参数。回环仍开启，未验证正确回环的接受能力。SLAM 2.8.5 加载[独立最大边界补丁](slam-grid-boundary.md)，运行时库路径、哈希和实际参数另存。与 run13 是新场景运行对照，传感器输入不同，不能称固定输入下唯一变量的因果试验。

任务/单目标/返航预算分别为 900/120/240 **墙时秒**。起点使用独立核验的 .30 m 净空及 .05 m 裕量，启动环视前没有任务客户端。原生车体最大累计旋转 6.408941 rad，回正后累计 6.264418 rad，最大中心位移 .0654 mm。旋转后规划副本仅填一个未知格，原始地图未改；独立先验来源字符串沿用同一 world 哈希的 run13 布置检查，不代表沿用旧地图或旧位姿。

## 独立评分

![原始地图、实际轨迹、目标标注与返航细节](assets/run14-overview.png)

图中统一为 world 坐标，原始地图只经过冻结初始变换，蓝线/橙线为实际 native 轨迹。地图时刻 325.5 sim 秒，轨迹为 136.0–276.575 sim 秒；这是结果叠图，不是同一时刻的动态成本地图。右侧放大区保留 map home 与实际 home 的差异。

对齐取自旋转前 29.375 sim 秒的同时间 map/native 初始位姿，后续 2 秒的 17 帧仅作验证。所有评分使用这个固定 SE2，**没有利用目标、终点或最终地图重新配准**；世界真值仅用于离线评价。

| 验证项 | 本轮结果 | 适用范围 |
|---|---|---|
| 导航终态 | 四次探索、一次返航均成功 | 单轮 0.8 m 通道的小迷宫 |
| 替身匹配 | TP=2、FP=0、FN=0；平面误差 4.6323/1.7600 cm | .20 m 一对一匹配阈值；RGB-D 表面点对盒中心，未评价 Z |
| 返航 map 位姿 | 距保存 home 5.6942 cm，yaw 差 .237971 rad | 任务 TF 验收阈值 .15 m / .26 rad |
| 独立实际返航 | 距记录 home 时的 native 位姿 6.4297 cm，yaw 差 .232484 rad | 同样通过上述误差阈值；不等于零误差 |
| 终点定位误差 | map/native 差 1.3242 cm、−.015446 rad | 终态附近 +.030 sim 秒的同 stamp 样本 |
| 真实自由区的 raw-free 比例 | 91.89785%，分母 7.0364 m² | 1 cm 评价网格；其余约 8.10% 标为 occupied/intermediate，未知/地图外为 0 |
| R=.25 真实安全中心的 raw-free 比例 | 100%，该子集面积 2.5288 m² | 仅评价中心标签，不等于整个圆盘都安全 |
| false-free 面积 | .0337 m²，占全域 raw-free 采样 .51846% | 固定初始对齐，未消除墙边离散化误差 |

5 mm 评价网格敏感性检查得到 raw-free 91.88008%、安全中心 raw-free 100%、false-free .03495 m²。没有将“无剩余可达前沿”的在线 `map_complete` 等同于真值地图 100% 准确或视觉已覆盖。原方案地图质量 ≥95% 的指标仍需按明确分母和墙边误差规则完成验收，不能用安全中心的 100% 替代。

## 控制复核与返航耗时

四次探索分别耗时 29.475、24.297、25.830、30.790 wall 秒；返航耗时 151.715 wall / 74.100 sim 秒。返航约 63.44 wall 秒首次进入起点 5 cm 范围，当时仍差约 3.09 rad 朝向；随后约 88.28 wall 秒主要在近点转向。严格新鲜的 TF/odom 样本累计绝对转角 12.382 rad，XY 跨度仅约 6.16/1.75 mm。Nav2 内部发生一次 `Failed to make progress` 后恢复成功，不能将任务 `failures=[]` 描述成底层全程没有重试。

完整录包任务时间窗内，`/cmd_vel_nav`、`/cmd_vel_smoothed`、`/cmd_vel` 分别有 2531/5364/5361 条记录，vx 均为 0–.15 m/s，wz 均在 ±.4 rad/s 内，无超限记录。平滑器配置 `scale_velocities=false`；2318 组有界时间关联的有限曲率样本中，112 组曲率差超过 .1/m，不能宣称严格保曲率。Twist 异步接收和加减速暂态仍限制归因。

24 份事件快照中，最初两份在自主首目标之前各有同一个中心未知格；启动先验仅填规划副本。后续 22 份快照，包括五份终态关联快照，原始图上的实际 R=.25 圆盘、格中心 R=.25 圆盘及 padded 矩形均清。离散快照不是全程连续碰撞证明。四个探索终态快照距目标均小于 .05 m；这些 TF 与成功源事件相差 −.025、+.850、−.005、0 sim 秒，不是 GoalChecker 内部测量。

返航 Action 成功与最终 map 距离 5.69 cm 不矛盾：固定 [ControllerServer 1.3.13](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_controller/src/controller_server.cpp#L578) 保留最后路径末点的 274.1 s 时间戳，再转换至 odom 供 GoalChecker；[RPP PathHandler](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_regulated_pure_pursuit_controller/src/path_handler.cpp#L89) 使用当期 robot 时间戳。同期 map→odom 更新了 1 cm。保留录包中重复 TF 值的两种插值，旧 stamped goal 距离为 4.694–4.894 cm，而 274.6 s 当前 map 距离为 5.694 cm；yaw 误差约 .24746 rad。

这是源码与录包的一致重建，**没有读取实际 GoalChecker 内部状态**，不能据此声称其实现出错。它说明 `SUCCEEDED` 不能独自证明当期 map 中距原请求目标 ≤.05 m。固定地图的候选裕量公式仍须以当期 TF 和原图足迹复核；本轮返航另有 .15 m/.26 rad TF 验收，实际通过。近点反复转向和到达时间语义仍是后续改进项。

## 数据和停机

闭合后的 MCAP 中 490,790 条存储记录全部可读，逐话题计数与 metadata 一致，解析前后哈希未变。录制日志没有报告传输丢失；这不是无损采集证明。保存了 256 条 `/plan`，RPP 的变换后路径、前视点、碰撞弧和转向状态各 2,525 条。

`/received_global_plan` 实际位于 `base_link`；碰撞弧位于 `odom`，仅有位置投影，消息中的默认单位四元数不代表预测朝向。三段 Twist 和转向 Bool 没有源时间戳，保留的是录包接收时间；不能假定每条命令已精确关联到同一控制周期。解释依据见 [RPP 实现](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp)与[碰撞检查发布代码](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_regulated_pure_pursuit_controller/src/collision_checker.cpp)。

冻结的观察器混合请求了 MPPI/RPP 专属参数，rclcpp 因未声明字段拒绝整批，三次尝试后保留错误并非零退出。独立运行时参数采集成功，实际 RPP 库亦已核对；未把独立证据补写为原观察器成功。修复版先查询插件身份、再读取专属参数，43 项纯测试及隔离的 RPP/MPPI 真实 C++ 参数服务回归通过，详见[观察器说明](mission-observer.md)。

末段 30 sim 秒有 963 个新鲜原生车体样本，最大间隔 .037 s，x/y/yaw 跨度均为零。任务退出另采到 20 个静止 odom 样本，持续 .5638 wall 秒，目标终态、取消/停车检查无错误。原始地图保存成功，位姿图序列化 result=0；本轮全部已记录进程均核实退出。

## 复现入口与保留证据

可选配置为 `src/limo_maze_bringup/config/nav2_rpp.yaml` 和 `slam_smoke_experimental.yaml`；默认 `nav2.yaml` / `slam.yaml` 保留。具体启动顺序见 [RPP 小迷宫复现](rpp-smoke-reproduction.md)。新运行须使用新 epoch 和输出目录，不能沿用本轮已关闭文件。

任务 JSON、替身评分、原始最终快照和地图导出的未修改副本已存入 [精简结果存档](../experiments/run14/README.md)，文件哈希可核对。其余原始大文件在忽略的 `outputs/` 中保留，未上传 GitHub：

- `outputs/setup/run14-experiment-manifest.json`、两份运行 YAML 与启动脚本：准备时冻结；其中 `prepared_not_started` 是准备时状态，不是最终实验结果。
- `outputs/ros_smoke/mission_run14/`：正式任务结果和退出审计。
- `run14-provenance.json`、`run14-independent-alignment.json`：实际库/参数及固定初始对齐。
- `run14-mission-observer.json` 及其快照、`run14-inputs-bag/`、`run14-bag-analysis/`：连续位姿、三段速度、路径和成本地图；原始快照各自异步，非控制器同一事务。
- `run14-target-evaluation.json`、`run14-geometric-coverage.json`、`run14-return-evaluation.json`：独立评分、采样规则和输入哈希。
- `run14-control-audit.{json,md}`、`run14-return-timing-analysis.{json,md}`：全部目标时序、速度、事件足迹及返航重建。
- `run14-final-map.{yaml,pgm}`、`run14-final-graph.{posegraph,data}`：原始地图和位姿图。

录包 MCAP SHA256：`b8870130c54c898df89f2da337dae14053f7583159c2164111a408018f0a2b23`。任务 JSON SHA256：`3903e276aed6c3f2c8266e2e965719f43ae54621c8419e7fb5ebbf0b3e060087`。冻结初始对齐 SHA256：`a57cbda8d908c5c263edcd2d257f3acf9c6cc8be4dc403de83c9d2240676d71f`。

下一阶段是有图像证据的自主观察、正式物品识别及不同迷宫的重复实验；实车环境与相机仍须现场确认。不能仅因本轮两个红盒均匹配，就打开视觉覆盖或正式物品验收开关。
