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
