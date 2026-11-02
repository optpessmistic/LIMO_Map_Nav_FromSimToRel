# TF 时钟核查与仿真容差实验

2026-11-01，按用户建议检查时钟一致性并尝试适度放宽限制。本实验使用 WSL / ROS2 Jazzy，未在实车验证。

## run24 原始数据核查

参数回读中，SLAM、Nav2 控制器、任务和识别节点均为 `use_sim_time=true`；启动检查发现一个 `/clock` 发布者。原始录包的 `/clock`、`map→odom`、`odom→base_footprint`、`/odom` 和 `/scan` 时间戳均无倒退，没有发现系统时间与仿真时间混用的证据。

录包时间与消息时间戳的最大差分别为：`map→odom` 0.150 s、`odom→base_footprint` 0.195 s、`/scan` 0.225 s。独立 observer 的 TF 链年龄却曾达到 1.019 s，同时出现约 1.885 s 的回调采样间隔。这支持接收或回调处理暂时落后的解释，尚不能确定具体节点或 DDS 根因。录包本身启用了仿真时间，其记录时间也是记录器缓存的 ROS 时间，并非独立墙钟；不能代替每个节点私有 TF 缓存的观测。

原 bag SHA256：`a4afa08357a7cfd8b027ee5b7dae08f4afb0cce2afc8cc9a9f1543a6985d531c`。离线脚本和结果见 [时钟核查档案](../experiments/run24-clock-audit/README.md)，上一轮日志和实验结果见 [run23/24](run23-24-reliability.md)。

## 两种限制分别调整

| 参数 | run24 | run25 试验 | 作用 |
|---|---:|---:|---|
| RPP `FollowPath.transform_tolerance` | 0.1 s | 0.5 s | 等待请求时刻的 TF 可用 |
| 任务 `tf_max_age_s` | 1.0 s | 1.5 s | 最新机器人位姿 TF 的最大允许年龄 |

RPP 将该参数传给 TF 变换调用的等待期限，参见 [Nav2 1.3.13 源码](https://github.com/ros-navigation/navigation2/blob/1.3.13/nav2_regulated_pure_pursuit_controller/src/path_handler.cpp)。增大任务年龄上限本身不能修复 RPP 的未来外推错误。SLAM 的 `transform_timeout` 同时影响发布 TF 的时间戳前移，见 [SLAM Toolbox 2.8.5 源码](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp)，本轮保持其 0.2 s 和 `restamp_tf=false`。

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

下一步先隔离写盘路径：保持原期限，比较 WSL 原生文件系统的留存目录与当前 `/mnt/c` 路径，补足分阶段耗时后再决定修复。实车当前不可访问，以上结果均为仿真结果。
