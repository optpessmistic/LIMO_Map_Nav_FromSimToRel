# TF 时钟核查与仿真容差实验

2026-11-01，按用户建议检查时钟一致性并尝试适度放宽限制。本实验使用 WSL / ROS2 Jazzy，未在实车验证。

后续独立测量确认了此前 ROS 消息时钟检查未覆盖的问题：WSL 系统钟会回跳，SLAM 使用的 SYSTEM `Rate` 随之出现秒级休眠；同时间段 STEADY `WallRate` 正常。已提供仅更换 TF 发布循环休眠时基的上游补丁，详见 [系统钟诊断与验证范围](wsl-clock-tf.md)。

## 最新对照：run30 稳态 TF 发布

2026-11-03，运行 `a335f4c`。在 run29 配置上，只把 SLAM TF 发布循环的 `rclcpp::Rate` 换为 `rclcpp::WallRate`；已完整构建独立 overlay，并核实运行进程加载的新库。ROS 消息仍用仿真时间，保持 `restamp_tf=false`、SLAM 时间戳前移 0.2 秒、RPP 等待 0.5 秒、任务 TF 年龄上限 1.5 秒和恢复参数 1/3；没有进一步增大容差或预算。

| 项目 | run29 | run30 |
|---|---:|---:|
| frontier 导航成功 / 下发 | 1/9 | 5/5 |
| home 导航成功 / 下发 | 1/4 | 1/1 |
| TF_ERROR=102 终态 / 导航外推日志行 | 11 / 11 | 0 / 0 |
| `map→odom` 接收间隔 >0.15 sim 秒次数 | 21 | 1 |
| 最大相邻接收间隔 | 0.793 s | 0.475 s |
| 严格推进 header 之间的最大接收间隔 | 0.937 s | 1.725 s |
| 地图完成 / 返航位姿检查 | false / true | true / true |
| 原生返航位置误差 | 1.59 cm | 4.12 cm |
| 严格 RGB-D 留存审计 | 3/3 通过 | 33/36，失败 |

run30 同期 65 秒独立探针仍记录到系统钟相对稳态钟回跳约 1.218 秒。独立 C++ 对照先前已复现 SYSTEM Rate 的秒级阻塞，而 STEADY WallRate 没有对应长休眠；本轮整车结果支持更换发布调度时基。仍需区分“重复发布已有 TF”和“扫描时间戳继续推进”：后一指标的最坏间隔并未改善，不能宣称解决所有 TF 新鲜度问题。bag 接收时间属于记录器 ROS 时钟，也不能重建 Nav2 私有缓存。原始逐话题统计与输入哈希见 [闭合录包审计](../experiments/run30/validation/run30-tf-delivery-audit.json)。

两个红色替身已标注，地图/位姿图捕获和独立 PGM/YAML 导出通过，返航后静止及自有进程关闭通过。观察阶段 360 wall 秒到期时仍未完成多朝向队列；36 个接受帧中三个缺少已确认 consumer 回执，虽原文件均已保存且 producer 无 fault，严格审计仍失败。保留编排 exit 1、`visual_coverage_complete=false`、`mission_success=false`。地图完成标志只是当前探索判据，不替代真值评分、完整视觉搜索或正式目标识别。详见 [run30 结果](../experiments/run30/run30-outcome-review.md)。

## run29 恢复等待

2026-11-03，运行 `9e0a326`，仅相对 run28 启用任务恢复参数 `1.0/3.0`，原 TF 容差和任务期限保留。11 次失败均触发等待，并在约 1 秒墙钟后看到 map→odom 时间戳推进；未发生恢复超时。9 次 frontier 导航中 1 次成功，4 次 home 导航中最后一次成功。独立原生位姿返航位置误差 1.59 cm，两个红色替身已标注，但完整地图与视觉覆盖仍为 false。

11 次导航失败的原始 error_code 均为 102，安装的 Nav2 1.3.13 `FollowPath.action` 定义其为 TF_ERROR；对应日志明确位于 RPPPathHandler 的未来外推。此定义属于 FollowPath，不能仅依据 NavigateToPose 状态 ABORTED 推断。实际读回 local costmap 的 `transform_tolerance=0.3`、`use_sim_time=true`，RPP 等待仍为 0.5 秒。这是两个不同调用路径的参数，本轮首发故障来自 RPP。

3/3 接受的 RGB-D 通过严格审计；地图和位姿图保存、终态后 30 sim 秒静止、自有进程关闭通过。编排器退出 0 只表示运行与证据收尾正常，不改变 `mission_success=false`。日志中 TF 故障间隔近 31 秒墙钟；恢复等待没有消除它，仍需调查发布或传输延迟，不能据两轮返航结果证明因果改善。见 [run29 原始结果](../experiments/run29/run29-outcome-review.md)。该紧凑档案不复制大 bag 和原图，外部证据路径与哈希单列保存。

## run24 原始数据核查

参数回读中，SLAM、Nav2 控制器、任务和识别节点均为 `use_sim_time=true`；启动检查发现一个 `/clock` 发布者。原始录包的 `/clock`、`map→odom`、`odom→base_footprint`、`/odom` 和 `/scan` 时间戳均无倒退，没有发现系统时间与仿真时间混用的证据。

录包时间与消息时间戳的最大差分别为：`map→odom` 0.150 s、`odom→base_footprint` 0.195 s、`/scan` 0.225 s。独立 observer 的 TF 链年龄却曾达到 1.019 s，同时出现约 1.885 s 的回调采样间隔。这支持接收或回调处理暂时落后的解释，尚不能确定具体节点或 DDS 根因。录包本身启用了仿真时间，其记录时间也是记录器缓存的 ROS 时间，并非独立墙钟；不能代替每个节点私有 TF 缓存的观测。

原 bag SHA256：`a4afa08357a7cfd8b027ee5b7dae08f4afb0cce2afc8cc9a9f1543a6985d531c`。离线脚本和结果见 [时钟核查档案](../experiments/run24-clock-audit/README.md)，上一轮日志和实验结果见 [run23/24](run23-24-reliability.md)。

## 两种限制分别调整

| 参数 | run24 | run25 试验 | 作用 |
|---|---:|---:|---|
| RPP `FollowPath.transform_tolerance` | 0.1 s | 0.5 s | 等待请求时刻的 TF 可用 |
| 任务 `tf_max_age_s` | 1.0 s | 1.5 s | 最新机器人位姿 TF 的最大允许年龄 |

RPP 将该参数传给指定时间戳 TF 变换调用的等待期限，参见 [Nav2 1.3.13 源码](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_regulated_pure_pursuit_controller/src/path_handler.cpp)。C++ Buffer 使用所绑定的 ROS 时钟判断等待截止时间，轮询休眠使用墙钟；启用仿真时间时，0.5 秒不是固定的 0.5 秒墙钟。[tf2 Buffer 源码](https://github.com/ros2/geometry2/blob/jazzy/tf2_ros/src/buffer.cpp)。增大任务年龄上限本身不能修复 RPP 的未来外推错误。SLAM 的 `transform_timeout` 同时影响发布 TF 的时间戳前移，见 [SLAM Toolbox 2.8.5 源码](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp)，本轮保持其 0.2 s 和 `restamp_tf=false`。

通用任务默认值仍为 1.0 s，实验必须明确传入 1.5 s。未来时间戳检查、定位跳变、障碍避碰、任务/取消期限及完整探索验收不变。两项同时调整属于组合容差试验，不能单独归因其中某一项。

## 配置入口

独立导航配置 [nav2_rpp_tf_experimental.yaml](../src/limo_maze_bringup/config/nav2_rpp_tf_experimental.yaml) 从 run24 完整配置派生，只改 RPP 的 TF 等待；包含原 run24 的 XY 0.03 m 容差。通用 `configure_nav2.py` 会恢复通用参数，不用于覆盖这份实验配置。

在已加载 ROS、模型和本工作区 overlay，并已启动 Gazebo 的 WSL 终端中，导航命令为：

```bash
ros2 launch limo_maze_bringup mapping_navigation.launch.py \
  use_sim_time:=true \
  nav_params:="$PWD/src/limo_maze_bringup/config/nav2_rpp_tf_experimental.yaml" \
  slam_params:="$PWD/src/limo_maze_bringup/config/slam_smoke_experimental.yaml"
```

沿用既有完整任务命令，增加 `-p tf_max_age_s:=1.5`；若通过 `mission.launch.py` 启动，则传 `tf_max_age_s:=1.5`。先构建更新后的 mission 包，使 launch 参数转发生效。完整观察/留存实验还需识别节点、初始观测和留存参数，不能只靠这一个参数启动。

可用以下只读命令核实实际参数：

```bash
ros2 param get /controller_server FollowPath.transform_tolerance
ros2 param get /limo_maze_mission tf_max_age_s
ros2 param get /slam_toolbox use_sim_time
ros2 param get /controller_server use_sim_time
ros2 param get /limo_maze_mission use_sim_time
ros2 param get /red_rgbd_sim_baseline use_sim_time
```

Gazebo / RViz 界面入口见 [可视化指南](visualization.md)。实验结果须以实际运行记录为准，容差配置本身不代表已完成任务。

## 实测结果：run25 / run26

代码 `b23c983` 的新增 7 项 TF 边界/诊断测试和配置一致性测试通过，既有 19 项留存/阻断原因回归通过；两个 ROS 包构建及安装源码核对通过，[该版本 CI](https://github.com/optpessmistic/LIMO_Map_Nav_FromSimToRel/actions/runs/41403085692) 通过。

run25 在运动前因准备脚本写出的 `epoch.txt` 为 CRLF 失败：Bash 去掉换行后保留回车，录包器校验拒绝启动。本轮不能评价 TF 效果。原始失败见 [run25 档案](../experiments/run25/README.md)。修正为明确 ASCII/LF 并增加启动前 UUID 字节检查后，使用全新目录、会话和进程启动 run26。

run26 实际参数回读确认 RPP 等待 0.5 s、任务年龄上限 1.5 s，四个关键节点均使用仿真时间，SLAM 时间戳设置保持原值。运行约 652.5 wall 秒后结束：

| 项目 | 观测结果 |
|---|---|
| 自主导航 | 6 个 frontier 目标均成功 |
| 任务 TF 年龄拒绝 / 导航 extrapolation 日志 | 均未记录 |
| 独立 observer 的 TF 年龄 | 1,473 个有效样本，最大约 1.079 s；2 个样本超过 1 s，没有超过 1.5 s |
| 目标标注 | 2 个已确认红色替身；正式物品定义仍待确定 |
| 原图留存 | mission 接受 18 帧，producer 记录 15 stored、3 failed；整体严格审计失败 |
| 地图导出 / 终态静止 | PGM/YAML 回读通过；终态后 30 sim 秒原生位姿静止采样通过 |
| 完整地图 / 搜索覆盖 / 返航 / 总任务 | 均未通过 |

observer 的缓存不能代表任务节点私有缓存；两项参数同时改变且只有一次运行，因此这说明新配置下未复现原故障，不能证明根因已消除或分别量化两项改动的作用。

本次实际阻断为 `frame_retention_failed`，producer 原因是 `writer_wall_timeout`。第 16 个写盘任务在约 5.05 wall 秒后被终止，随后两个排队请求中止。5 秒预算包括进程启动、调度、管道传输、写文件和回执；现有证据不足以单独归因磁盘性能。该超时使用单调墙钟，不读取任务 TF 容差。

原严格审计因 producer fault 在枚举前返回失败 0/0，这不等于没有接受帧。档案分别保留原审计和实际 18 帧清单，逐项列出缺失的 3 帧文件；部分文件压缩包的哈希核对不替代严格审计。自有进程组和 writer 已确认退出，原 fault 保留。见 [run26 完整档案](../experiments/run26/README.md)。

后续已保持原期限完成文件系统与输入管道的隔离对照，结果支持优先改善输入传输余量，详见 [writer 管道对照](writer-pipe-timing.md)。实车当前不可访问，以上结果均为仿真结果。

## run28：容差增加后仍复现，快速重试放大影响

运行版本 `e8e72e8`；collector 启动计时的 13 项测试和 [CI](https://github.com/optpessmistic/LIMO_Map_Nav_FromSimToRel/actions/runs/41596459218) 通过。运动观察器在外层 60 wall 秒预算内，于 17.215 秒被确认就绪；初始环视、路径/处理帧预检及运行参数检查通过。709.667 wall 秒后结束，全部自有进程组与 writer 退出，地图导出和终态后 30 sim 秒停车采样通过。原始失败、脚本、源代码、参数、评分与哈希见 [run28 档案](../experiments/run28/README.md)。

| 项目 | 结果 |
|---|---|
| 探索导航 | 下发 12 个，1 成功、11 失败 |
| 返航导航 | 下发 4 个，全部失败；最终原因 `return_navigation_failed` |
| 导航外推日志 | 5 行；另有 `tf_help` 的旧变换错误，行数不等于独立故障数 |
| 任务自身 `base_tf_age_rejected` | 未记录；不能据此认为 Nav2 的独立缓存正常 |
| 停车观察/留存 | 1 个窗口、3 组 RGB-D，producer 全部 stored，三个 writer 均使用 1 MiB 管道且没有 fault |
| 严格留存审计 | 失败：`consumer_disabled_or_faulted`；原审计 0/0 是提前拒绝，实际接受数为 3 |
| 目标评分 | TP=2、FP=0、FN=0；两目标 XY 误差 5.24/24.02 mm |
| 地图评分 | 起点连通真值自由区中 raw-free 90.4596%，unknown 0.1876%，occupied/intermediate 9.3528%，图外 0 |
| 实际返航 | 最后原生位置距起点约 1.171 m；未通过 |

目标与地图评分只在运行结束后读取世界真值。坐标变换冻结于初始 Spin 后、首次自主 goal 前的首个合格观测（120.899 sim s；首 goal 132.3 s），后续 23 个样本仅核验，不拟合目标或末端位置。因此不能评价初始 Spin 期间的漂移。目标按可见表面测点对盒中心作 XY 匹配，阈值 0.20 m，无高度或表面至中心校正。地图主分母为完整的起点四连通真值自由区 7.0364 m²；1 cm 评分的 false-free 为 0.0847 m²，占真值障碍 0.5513%、占预测自由区 1.3132%。5 mm 复算的 raw-free 为 90.4614%。安全中心子集的 100% 不能替代完整主分母。

本轮原始 bag SHA256 为 `b099a1585a7868f6c978bfc117a4ef145ee3fe31837fcf0a503fd08de1769b33`。离线解码 `/clock`、`map→odom`、`odom→base_footprint`、`/odom` 与 `/scan` 未见时间戳倒退，四个关键节点参数均为 `use_sim_time=true`。记录时间减 header 时间最大分别约为 0.120、0.125、0.112、0.112、0.171 sim 秒。`map→odom` 相邻变更时间戳的记录间隔最大 1.508 sim 秒。录包时间也是记录器的 ROS 时间，不能据此重建 Nav2 私有 TF 缓存或断定调度根因。

源码核查发现之前遗漏的一处独立限制：`controller_server::isGoalReached()` 调用 `nav_2d_utils::transformPose()`，使用 **local_costmap 的 `transform_tolerance`**。该函数先尝试精确转换，外推失败后取 latest，并比较输入姿态时间与 latest 的差；它不受 `FollowPath.transform_tolerance` 控制。本配置没有显式设置 local_costmap 参数，上游默认 0.3 秒；本轮未单独回读该默认值，所以这是配置/源码推断。日志中 153.600−153.075=0.525 秒与此拒绝条件一致。输入姿态时间也不能直接当作日志产生时的现在。[ControllerServer](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_controller/src/controller_server.cpp)、[tf_help](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_dwb_controller/nav_2d_utils/src/tf_help.cpp)、[Costmap2DROS](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_costmap_2d/src/costmap_2d_ros.cpp)。

`ControllerTFError` 会使 Nav2 停止并终止当前 FollowPath；`failure_tolerance` 的等待仅处理 `NoValidControl`，不能用它恢复 TF 错误。任务收到失败后可在同一 tick 选择下一目标，这解释了短时间内耗尽多个候选和返航重试的放大路径。后续修复应在已确认导航终止后作有限、非阻塞的恢复等待，并检查 TF 时间继续推进；保留取消、任务和返航总期限。不能将所有 ABORTED 状态都猜成 TF 故障。

已实现默认关闭的[导航恢复等待](navigation-recovery.md)，18 项新测试、167 项旧回归及一个真实 ROS 暂停时钟用例通过；其中纯 Python 旧回归另有 1 项 ROS 启动检查跳过。新 Gazebo 对照结果另行记录，不把测试通过当作故障已消除。

SLAM 在扫描回调入口更新 `scan_header`，早于扫描筛选；`restamp_tf=false` 时发布 `scan_header.stamp + transform_timeout`。如果扫描交付或回调停滞，单独缩短 TF 发布周期只会重复旧时间戳。耗时扫描优化与 TF 发布使用不同互斥锁，当前证据不支持“优化一直占住发布锁”的归因；暂不改变 `restamp_tf`。
