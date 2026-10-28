# LIMO 上游实现与版本核查

核实日期：2026-10-28。范围：官方文档、公开 GitHub 分支、文件树及关键源码。本报告的“已核实”表示已读取相应来源；尚未在用户车辆或 ROS 环境中编译、运行、测量。手册内容作为技术资料，不作为执行指令。

## 结论与复用边界

LIMO 的底盘驱动、激光建图、定位导航、Gazebo 模型均已有实现，应优先复用。项目主要新增工作是把自主探索、目标识别确认、目标位姿记录、返航和异常恢复接成一个可靠任务流程。没有核实到官方可直接开箱完成“未知迷宫内找特定物品并返回起点”的完整系统。

用户已确认使用 **ROS2**，目标为 **完整探索建图、在地图中标注特殊物品、完成后返航，不拿取物品**；相机和物品定义待核实。因此项目主线采用 **slam_toolbox + Nav2 + frontier 探索 + 物品地图标注 + 任务状态机**。最新确认的开发环境为 **WSL Ubuntu 24.04.4 LTS，当前 ROS_DISTRO 为空**，故仿真环境选用 **ROS2 Jazzy + Gazebo Harmonic**，详见文末附录；此前核查的 Humble / Classic 示例仅作为模型及算法配置参考，不能直接当作当前可执行路径。发现物品后应确认并记录标注、继续完成探索；探索完成或满足明确的停止条件后再返航。ROS1 部分保留为上游对照和避免混用手册的参考，不作为当前实施主线。

不能仅根据“LIMO”产品名选择软件栈；标准版、Pro、ROS2 版以及不同时期的雷达、相机配置存在差异。ROS2 发行版、Ubuntu 版本及传感器型号仍需读取实车确认。

初版建议采用四轮差速模式，利用原地转向简化迷宫探索。阿克曼模式需要足够的转弯空间和相容的规划控制配置。这是工程建议，实际可行性仍需按走廊宽度、车身 footprint、定位误差和地面条件验证。

## 官方仓库与已核实版本

| 仓库 / 分支 | 本次读取到的提交 SHA | 已核实内容 |
| --- | --- | --- |
| [limo_ros / master](https://github.com/agilexrobotics/limo_ros/tree/master) | `4c78efc674cfc154012fe851fbda89c50be5b983` | ROS1 实车驱动；GMapping、Cartographer、AMCL、move_base 的 launch 和参数 |
| [ugv_gazebo_sim / master / limo](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/limo) | `27633a956c845903ee630538afeb17fe70afdd84` | ROS1 差速和阿克曼仿真；激光、深度相机、IMU 仿真宏 |
| [limo_ros2 / humble](https://github.com/agilexrobotics/limo_ros2/tree/humble) | `dcc5a8637f83236ed6778af86ad826e46a20cd92` | 默认分支；ROS2 底盘驱动、limo_car 阿克曼仿真、模型 |
| [limo_ros2 / humble-dev](https://github.com/agilexrobotics/limo_ros2/tree/humble-dev) | `9197fe089a2c852acb6159d3e7a588fd1efa3c1a` | 含 limo_bringup、Cartographer、navigation2 配置和 ydlidar_ros2 |
| [limo_ros2 / foxy](https://github.com/agilexrobotics/limo_ros2/tree/foxy) | `2faf97f82993dbf3422a435eeccc7949a1ff2958` | 含 limo_bringup，但不可据此推断默认 humble 分支具有同样文件 |
| [limo_ros2 / master](https://github.com/agilexrobotics/limo_ros2/tree/master)、[old_version](https://github.com/agilexrobotics/limo_ros2/tree/old_version) | `cc2ab2114b72204aa02258406ae6b235c0ab29f1` | 两分支此次指向同一提交 |
| [agilex_open_class / master / limo](https://github.com/agilexrobotics/agilex_open_class/tree/master/limo) | `8e799754af623c3cec954b74b186d63d6ce8fa62` | ROS2 Humble 教学仿真，含差速模型、slam_toolbox、Cartographer、Nav2、RTAB-Map 配置和例程 |

分支证据：[limo_ros2 branches API](https://api.github.com/repos/agilexrobotics/limo_ros2/branches)。实现时应固定提交并记录依赖版本；分支链接可能随上游更新而变化。

本次访问 `https://github.com/agilexrobotics/limo_sim` 返回 404。不能将该地址列为已存在的官方仿真仓库。ROS1 仿真的已核实位置是 `ugv_gazebo_sim/limo`；该目录 README 仍使用旧 clone 名 `ugv_sim`，应以当前真实仓库为准。

## ROS1：现成能力及实时建图的正确组合

### 底盘、传感器与控制接口

[limo_base 驱动](https://github.com/agilexrobotics/limo_ros/blob/master/limo_base/src/limo_driver.cpp) 订阅 `/cmd_vel`，发布 `/odom`、`/imu`、`/limo_status`。`/odom` 消息发布与 odom TF 发布是两件事：即使关闭 TF，驱动仍发布里程计消息。

[limo_base.launch](https://github.com/agilexrobotics/limo_ros/blob/master/limo_base/launch/limo_base.launch) 的默认值为：

```xml
<arg name="odom_frame" default="odom" />
<arg name="base_frame" default="base_link" />
<arg name="pub_odom_tf" default="false" />
```

[limo_start.launch](https://github.com/agilexrobotics/limo_ros/blob/master/limo_bringup/launch/limo_start.launch) 启动底盘、`ydlidar_ros/X2L.launch`，并发布 `base_link → laser_link`、`base_link → imu_link` 静态 TF；没有启动相机。当前公共版本也没有声明或向内部转发 `pub_odom_tf` 参数。因此，其他型号手册里的 `roslaunch limo_bringup limo_start.launch pub_odom_tf:=false` 不能直接视为该公共版本可接受的命令；应先检查车上实际文件，或在项目包装 launch 中明确传参。

相机必须按实际型号单独安装和启动。不能把 X2L、EAI T-mini Pro、Dabai 等不同产品手册中的驱动名称混用。

### GMapping + move_base 在线探索：TF 的唯一发布者

建议第一条 ROS1 集成路径为：底盘及传感器 → 一个里程计 TF 发布者 → GMapping → 仅 move_base 导航节点 → frontier 探索 / 任务控制器。预期 TF 链为：

```text
map --由 GMapping 发布--> odom --由一个选定的里程计模块发布--> base_link
                                                               ├─ laser_link
                                                               └─ imu_link / camera frames
```

[官方 limo_gmapping.launch](https://github.com/agilexrobotics/limo_ros/blob/master/limo_bringup/launch/limo_gmapping.launch) 已启动 `robot_pose_ekf`，并设 `output_frame=odom`、`base_footprint_frame=base_link`、将 `imu_data` 重映射为 `imu`。因此按此 launch 工作时，**由 robot_pose_ekf 独占 `odom → base_link` TF，底盘保持 `pub_odom_tf=false`**。robot_pose_ekf 上游源码明确以这两个 frame 发布 TF；其滤波姿态 topic 是私有命名空间内的 `odom_combined`，类型是 `PoseWithCovarianceStamped`，不能误认为它自动替换了驱动的 `/odom` `nav_msgs/Odometry`。[robot_pose_ekf 源码](https://github.com/ros-planning/robot_pose_ekf/blob/master/src/odom_estimation_node.cpp)

若改用底盘直接提供 odom TF，需移除 / 停用该 EKF，并在底盘节点启用 `pub_odom_tf=true`。若在 Gazebo 中已经由驱动插件发布 odom TF，则应先沿用仿真插件作为唯一发布者，并从建图包装 launch 中去掉 EKF；不能在未处理重复 TF 的情况下原样叠加官方 gmapping launch。

**不要将官方 GMapping launch 与官方差速导航 launch 原样叠加。** [limo_navigation_diff.launch](https://github.com/agilexrobotics/limo_ros/blob/master/limo_bringup/launch/limo_navigation_diff.launch) 同样启动名为 `robot_pose_ekf` 的节点，并启动 AMCL 和 move_base。直接叠加会出现重复 / 同名 EKF 节点，且 AMCL 与 SLAM 会争夺定位 TF。应抽取 move_base 与所需 costmap / planner 配置，单独建立项目的在线建图导航 launch。

实时 SLAM 阶段：

- 保持 GMapping 为 `/map` 与 `map → odom` 的发布者，停用 AMCL。
- 不启动发布固定地图的 map_server，避免与 SLAM 争夺 `/map`。注意：本次核查的公共 `limo_navigation_diff.launch` **本身没有启动 map_server**；部分出厂镜像或外层导航 launch 可能包含，应检查实际文件，不能将它们混为一谈。
- move_base 全局代价地图订阅 SLAM 的持续更新地图。`StaticLayer` 是地图图层名称，并不意味着必须由固定地图 map_server 供图；仍可接收 SLAM 发布的 OccupancyGrid。应验证地图尺寸增长、未知区域处理、完整地图更新是否正确，不要只改 `static_map` 一个开关就认为接通在线探索。
- 若使用地图增量 topic，确认发布者真的提供该 topic 后再配置 `subscribe_to_updates`；GMapping 的整幅 `/map` 更新也应正确进入全局代价地图。
- 全局坐标系采用 `map`；局部 rolling costmap 建议采用连续的 `odom`，以降低 SLAM 回环修正对局部控制的影响。上游 [local_costmap_params.yaml](https://github.com/agilexrobotics/limo_ros/blob/master/limo_bringup/param/diff/local_costmap_params.yaml) 实际为 `global_frame: map`，这是需要项目审查和验证的参数，不是已经修好的配置。

完成探索后返航可以继续保持同一个 SLAM 会话，向已保存的起点位姿发导航目标。无需为了“返航”强制切换 AMCL。若任务流程明确切换到静态地图定位，应先保存地图并有序停止 SLAM 的地图 / 定位发布，再启用 map_server + AMCL、初始化位姿并核实 TF 所有权；不能同时保留两个 map 定位来源。

### 其他已核实的 ROS1 配置细节

公共差速导航使用 `global_planner/GlobalPlanner` 和 `base_local_planner/TrajectoryPlannerROS`，并非差速路径一定使用 TEB。阿克曼参数目录含 TEB 配置。证据见 [差速导航 launch](https://github.com/agilexrobotics/limo_ros/blob/master/limo_bringup/launch/limo_navigation_diff.launch) 与 [阿克曼参数目录](https://github.com/agilexrobotics/limo_ros/tree/master/limo_bringup/param/ackerman)。

GMapping 默认 `map_update_interval=5.0`、`linearUpdate=1.0`、`angularUpdate=0.5`、`delta=0.05`。这些是现成基线，不是狭窄迷宫已验证的最佳值；地图更新与小步移动触发频率需按车速、走廊尺度和算力调试。[GMapping launch](https://github.com/agilexrobotics/limo_ros/blob/master/limo_bringup/launch/limo_gmapping.launch)

Cartographer 不是 GMapping 的无差别替换：公共 [build_map_2d.lua](https://github.com/agilexrobotics/limo_ros/blob/master/limo_bringup/param/build_map_2d.lua) 为 `published_frame=base_link`、`provide_odom_frame=false`、`use_odometry=false`、`use_imu_data=true`，不能据此假定它已经采用 `map → odom → base_link` 的标准导航链。其 [launch](https://github.com/agilexrobotics/limo_ros/blob/master/limo_bringup/launch/limo_cartographer.launch) 还硬设 `/use_sim_time=false`。用于仿真或与外部 odom 融合时，必须单独设计 frame 和时钟配置并验证。

### ROS1 仿真

官方 [ugv_gazebo_sim/limo README](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/limo) 提供：

```bash
roslaunch limo_gazebo_sim limo_four_diff.launch
roslaunch limo_gazebo_sim limo_ackerman.launch
```

[差速 gazebo 文件](https://github.com/agilexrobotics/ugv_gazebo_sim/blob/master/limo/limo_description/urdf/limo_four_diff.gazebo) 确实启用了 laser、depth camera、IMU 宏，使用 `libgazebo_ros_skid_steer_drive.so`，配置 `broadcastTF=true`、`robotBaseFrame=base_footprint`。这与实车默认 `base_link` 存在 frame 差异，必须统一或通过正确的固定 TF 接通；不能同时让其他节点再发布同一 odom TF。

## ROS2：应复用什么、哪些内容尚需修正

### limo_ros2 默认 humble 分支并非完整导航包

当前 [humble 根目录](https://github.com/agilexrobotics/limo_ros2/tree/humble) 只有 `limo_base`、`limo_car`、`limo_description`、`limo_msgs` 等，**没有 `limo_bringup`**，尽管 README 列出了它。官网手册里的 `ros2 launch limo_bringup limo_nav2.launch.py` 不能当作默认 clone 后必然可执行的命令。

`foxy` 和 `humble-dev` 的确包含 `limo_bringup`，但其中名称为 `cartographer.launch.py`、`navigation2.launch.py`、`limo_navigation.launch.py` 等；仍须与所用手册和系统镜像逐一匹配。

默认 humble 的 [gazebo_models_diff.launch.py](https://github.com/agilexrobotics/limo_ros2/blob/humble/limo_description/launch/gazebo_models_diff.launch.py) 默认指向仓库文件树中未找到的 `worlds/neighborhood.world`；[limo_four_diff.gazebo](https://github.com/agilexrobotics/limo_ros2/blob/humble/limo_description/urdf/limo_four_diff.gazebo) 中传感器宏调用及原始控制插件被注释。该路径不能未经验证就作为“已经能用的差速建图仿真”交付。

`limo_car` 有 [sensor.xacro](https://github.com/agilexrobotics/limo_ros2/blob/humble/limo_car/gazebo/sensor.xacro) 定义 LaserScan、RGB-D、IMU 插件，以及 [ackermann_gazebo.launch.py](https://github.com/agilexrobotics/limo_ros2/blob/humble/limo_car/launch/ackermann_gazebo.launch.py)。不过 world 路径和参数传递、相机 frame/topic 仍需运行核实。

### 官方 agilex_open_class 更适合作为 Humble 仿真起点

[agilex_open_class/limo](https://github.com/agilexrobotics/agilex_open_class/tree/master/limo) 含较完整的差速仿真及教学配置，建议先提取所需包到独立工作空间，避免与其他仓库同名的 `limo_description`、`limo_bringup`、`limo_msgs` 相互覆盖。

- [差速 Gazebo 模型](https://github.com/agilexrobotics/agilex_open_class/blob/master/limo/limo_description/urdf/limo_four_diff.gazebo) 已启用 laser 和 depth camera，使用 `libgazebo_ros_diff_drive.so`，订阅 `cmd_vel`，发布 `odom` 和 odom TF，base frame 为 `base_footprint`；IMU 宏关闭。
- [传感器宏](https://github.com/agilexrobotics/agilex_open_class/blob/master/limo/limo_description/urdf/limo_gazebo.gazebo) 激光配置为 240°、720 点、8 Hz，输出 `scan`、frame `laser_link`。相机 remap 使用 `custom_camera/...`，但 camera_name 为 `limo`，因此不能仅凭文字认定真实 topic 名。需以 `ros2 topic list`、消息 frame 和图像显示核查。
- [limo_diff_spawn.launch.py](https://github.com/agilexrobotics/agilex_open_class/blob/master/limo/limo_gazebo_sim/launch/limo_diff_spawn.launch.py) 支持出生点参数，设置 `use_sim_time=True`，适合作为自定义迷宫 world 的机器人生成部分。
- [limo_slam_box.launch.py](https://github.com/agilexrobotics/agilex_open_class/blob/master/limo/limo_bringup/launch/humble/limo_slam_box.launch.py) 明确启动 `slam_toolbox/sync_slam_toolbox_node`。
- [limo_nav2_explore.launch.py](https://github.com/agilexrobotics/agilex_open_class/blob/master/limo/limo_bringup/launch/humble/limo_nav2_explore.launch.py) 启动 Nav2 的 `navigation_launch.py`，没有在此文件中启动 AMCL / map_server。可作为由外部 SLAM 提供地图及定位的在线导航集成参考。
- [explore.cpp](https://github.com/agilexrobotics/agilex_open_class/blob/master/limo/limo_learnning/src/explore.cpp) 仅在 `[-2,2] × [-2,2]` 随机生成目标，调用 `NavigateToPose`；它没有地图前沿检测、覆盖度计算或系统性迷宫搜索，**不能将文件名 explore 解释为已有可靠 frontier 探索器**。

[README](https://github.com/agilexrobotics/agilex_open_class/blob/master/limo/README.md) 还混有 `source devel/setup.bash`、`rosrun` 等 ROS1 写法；ROS2 实际编译环境应使用相应 colcon 工作空间的 `install/setup.bash`。README 中出现的指令只是来源证据，本文未执行它们。

## 官方已有自动探索路线，但不能直接视作完整项目

[LIMO Pro ROS1 手册](https://github.com/agilexrobotics/limo_pro_doc/blob/master/Limo%20Pro%20user%20manual%28EN%29.md) 描述了 GMapping + move_base + `rrt_exploration simple.launch` 自动探索建图，需要人工设置四个边界点和一个起始点。这证明存在现成组合路线；是否预装在用户标准版 LIMO、相应 launch 是否相同，需要检查实车镜像。该流程不包含用户所需的完整目标确认和返航任务控制。

建议复用成熟 frontier 探索器，比较地图覆盖、失败目标黑名单、重规划、窄通道处理及探索结束条件；官方随机目标示例只能作 Nav2 action 调用演示。物品识别结果应经过多帧确认，并带时间戳、相机 frame 和测距信息，经 TF 转换后记录到地图；识别成功不意味着探索结束。完整探索结束后，取消尚在执行的探索目标并确认取消完成，再切换到返航目标。地图标注的持久化文件应与最终地图一并保存；地图回环修正可能改变坐标估计，需保留观测来源以便校正标注。

## 支持周期与兼容边界

| 软件 | 已核实支持日期 / 边界 | 对项目的影响 |
| --- | --- | --- |
| ROS1 Noetic | 2025-05-31 EOL | 截至核查日 ROS1 已结束官方支持；现有镜像可用于复现实验，需固定环境 |
| Gazebo Classic 11 | 2025-01-29 EOL | 当前多数上述官方 LIMO 仿真基于 Classic；不是 Gazebo Harmonic 配置 |
| ROS2 Humble | 支持至 2027-05；Ubuntu 22.04 | 与官方现成教学包较接近；适合短期完成现有硬件实验 |
| ROS2 Jazzy | 支持至 2029-05；Ubuntu 24.04 | 长期方案可考虑，但需迁移 Classic 插件和验证驱动；不是直接换发行版名称即可 |

来源：[ROS1 官方 EOL 公告](https://discourse.ros.org/t/ros-noetic-end-of-life-may-31-2025/43160)、[Gazebo Classic 官方](https://classic.gazebosim.org/)、[Classic 迁移说明](https://community.gazebosim.org/t/gazebo-classic-end-of-life/2563)、[Humble 官方发布](https://discourse.ros.org/t/ros-2-humble-hawksbill-released/25729)、[Jazzy 官方发布](https://discourse.ros.org/t/ros-2-jazzy-jalisco-released/37862)。

Jazzy + Harmonic 不能直接加载 Classic 的 `gazebo_ros` 插件。选择最新系统应建立在愿意承担迁移工作的前提上；不要为了版本新而先破坏已工作的车辆出厂环境。

## 下一步运行验证标准

1. 记录实车型号、计算平台、Ubuntu / ROS 发行版、雷达及相机型号、运动模式，读取实际 launch 与参数并固定提交。
2. 验证 `/cmd_vel`、`/odom`、`/scan`、图像及 CameraInfo，核对时间戳、单位、消息类型和 frame。
3. 逐项检查 TF 的唯一发布者；所有传感器连到车体；仿真所有相关节点使用仿真时钟。
4. 先跑遥控建图和已知目标导航，再运行边探索边建图；确认地图增长时 move_base / Nav2 仍能稳定规划。
5. 在多个迷宫布局、不同目标位置和“未发现目标”的情况下检验探索、确认、返航、超时和失败分支；保存 rosbag、地图及任务事件作为证据。

当前状态：已完成来源及源码核查；上述运行标准尚未完成，不应将本报告表述为实机或 Gazebo 测试通过。

## 附录：WSL Ubuntu 24.04 的 Jazzy + Harmonic 路径

附录核实日期：2026-10-28。本附录根据用户新确认的开发环境调整软件栈；没有在本次只读调研中安装或运行 ROS。项目版本管理目标是用户指定的 [LIMO_Map_Nav_FromSimToRel](https://github.com/optpessmistic/LIMO_Map_Nav_FromSimToRel)，本报告仍位于既有项目目录。

### 选择结论

Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic 是官方配对；通过 ROS 仓库安装 `ros-jazzy-ros-gz` 可获得配套 Gazebo 依赖，不必为本项目混装 Classic 或 Humble 的 `gazebo_ros` 插件。[Gazebo 官方安装配对](https://gazebosim.org/docs/harmonic/ros_installation/)

当前没有核实到 AgileX 官方可直接运行的 LIMO Jazzy/Harmonic 完整仿真包。`limo_ros2` 没有 Jazzy 分支；`ugv_gazebo_sim` 分支仅见 master、humble、old_version，humble 的递归文件树未见 `limo/` 路径。官方 Humble 教学包使用 Classic，不能直接在 Harmonic 加载其插件。

最短的已有模型复用候选为社区 [nevlife/hunter_gazebo / harmonic](https://github.com/nevlife/hunter_gazebo/tree/harmonic)，核实提交 **`8e1d4aeb459e77d549262ca20cac819d9c7a32ee`**。虽然仓库名为 Hunter，它确实包含独立的 LIMO 四轮差速模型、LIMO launch、RGB-D 和二维激光传感器。源码使用原生 Harmonic 系统插件，下面列出的补丁完成后适合接入本项目；本次尚未证明实际运行成功。

### 已排除的名称相近候选

| 候选 | 已核实事实 | 结论 |
| --- | --- | --- |
| [brianlmerritt/limo_jazzy_desktop_core](https://github.com/brianlmerritt/limo_jazzy_desktop_core) | main SHA `ab5ebe859f8dd119502a46a3168fd0524b3c21b1`；README 写明当前是 Ubuntu 22.04 容器 + ROS2 Humble，Jazzy 迁移待后续；Dockerfile 基于 `ros:humble-ros-base-jammy`；面向 Jetson Orin Nano 实车 | 名称带 jazzy 不能证明它是 Jazzy 仿真。不是当前 WSL 仿真的最短路径 |
| [mattiadutto/ugv_gazebo_sim / jazzy-harmonic](https://github.com/mattiadutto/ugv_gazebo_sim/tree/jazzy-harmonic) | SHA `d26c75098d34ec2a62bad1b4c91b9576bc4b06c4`；README 支持表中 Bunker、Scout Mini 有勾，LIMO 空白 | 不能视作已经移植好的 LIMO 仿真 |
| [ricardoGrando/limo_tello_jazzy_spawn](https://github.com/ricardoGrando/limo_tello_jazzy_spawn) | README 声明 Jazzy/Gazebo Sim，主要演示无人机视觉跟踪地面 LIMO | 本次未深入核实地面车建图传感器和导航接入，不作为主选 |

第一项证据：[README](https://github.com/brianlmerritt/limo_jazzy_desktop_core/blob/main/README.md)、[Dockerfile](https://github.com/brianlmerritt/limo_jazzy_desktop_core/blob/main/Dockerfile)、[compose.yaml](https://github.com/brianlmerritt/limo_jazzy_desktop_core/blob/main/compose.yaml)。

### 主选候选的源码能力

[limo_gazebo.xacro](https://github.com/nevlife/hunter_gazebo/blob/harmonic/src/gazebo_harmonic/urdf/limo_gazebo.xacro) 定义：

- `gz-sim-diff-drive-system` / `gz::sim::systems::DiffDrive`：四轮两侧差速，轮半径 0.045 m，控制轮距 0.172 m，订阅 `/cmd_vel`，输出 `/limo/odometry` 和 Gazebo `/limo/tf`，frame 为 `odom → base_footprint`。
- `gz-sim-joint-state-publisher-system`：输出 `/limo/joint_states`。
- `gpu_lidar`：输出 `/limo/scan`，240°、720 点、8 Hz、0.05–8 m。
- `rgbd_camera`：输出 `/limo/camera` 下图像、CameraInfo、深度及点云，640 × 480、10 Hz、约 80° 水平视场。
- IMU：输出 `/limo/imu`，100 Hz。

[empty_mobile.sdf](https://github.com/nevlife/hunter_gazebo/blob/harmonic/src/gazebo_harmonic/world/empty_mobile.sdf) 使用标准 Physics、Sensors、Imu、UserCommands、SceneBroadcaster 系统，渲染器 ogre2。**LIMO 的这条启动链不引用 RGL 插件，不需要为了 LIMO 构建仓库里的 CUDA/RGL 项目**；激光和相机仍需要可工作的渲染上下文，WSLg 下应单独验证。

[limo_sim_start.launch.py](https://github.com/nevlife/hunter_gazebo/blob/harmonic/src/gazebo_harmonic/launch/limo_sim_start.launch.py) 使用 `ros_gz_sim/gz_sim.launch.py`、`ros_gz_sim/create` 和 `ros_gz_bridge/parameter_bridge`。支持 `start_x`、`start_y`、`start_z`、`start_yaw`、`headless`；默认世界为 `empty_mobile.sdf`，没有可替换 world 参数。

| Gazebo topic | 当前 ROS topic / 类型 | 已有桥接方向 |
| --- | --- | --- |
| `/clock` | `/clock`，Clock | Gazebo → ROS |
| `/cmd_vel` | `/cmd_vel`，Twist | 当前双向；项目可收紧为 ROS → Gazebo |
| `/limo/odometry` | 同名，Odometry | Gazebo → ROS |
| `/limo/joint_states` | 同名，JointState | Gazebo → ROS |
| `/limo/scan` | 同名，LaserScan | Gazebo → ROS |
| `/limo/imu` | 同名，Imu | Gazebo → ROS |
| `/limo/camera/image` | 同名，Image | Gazebo → ROS |
| `/limo/camera/camera_info` | 同名，CameraInfo | Gazebo → ROS |
| `/limo/camera/depth_image` | 同名，Image | Gazebo → ROS |
| `/limo/camera/points` | 同名，PointCloud2 | Gazebo → ROS |
| `/limo/tf` | **没有桥接到 ROS `/tf`** | 需补齐 |

### 最短基础运行尝试

以下是根据源码核实的待执行路径，前提是已按官方方式安装并配置 ROS2 Jazzy 的软件源。可以在独立仿真工作空间执行，不需要 recursive clone，也不构建 RGL：

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-tf2-ros \
  python3-colcon-common-extensions python3-rosdep build-essential
source /opt/ros/jazzy/setup.bash

mkdir -p ~/limo_harmonic_ws/src
cd ~/limo_harmonic_ws/src
git clone --branch harmonic https://github.com/nevlife/hunter_gazebo.git
git -C hunter_gazebo checkout 8e1d4aeb459e77d549262ca20cac819d9c7a32ee
cd ~/limo_harmonic_ws
rosdep install --from-paths src/hunter_gazebo/src --ignore-src --rosdistro jazzy -y
colcon build --packages-select limo_base hunter_base scout_base gazebo_harmonic
source install/setup.bash
ros2 launch gazebo_harmonic limo_sim_start.launch.py headless:=true
```

`rosdep` 首次使用需要初始化并更新数据库；该步骤应由环境安装流程处理。这里显式选择四个包是因为 `gazebo_harmonic/package.xml` 声明了三个机器人模型包依赖，而 LIMO launch 实际只加载 LIMO 模型。[package.xml](https://github.com/nevlife/hunter_gazebo/blob/harmonic/src/gazebo_harmonic/package.xml)、[CMakeLists.txt](https://github.com/nevlife/hunter_gazebo/blob/harmonic/src/gazebo_harmonic/CMakeLists.txt)

`headless:=true` 仅在此 launch 中加 `gz sim -s`、关闭 GUI，并不等同已经配置 EGL 无显示渲染。若 WSLg 图形可用，可用 `headless:=false` 验证 GUI；若没有有效图形上下文，需要另行配置 Harmonic headless rendering，不能通过关闭所有传感器绕过建图所需数据。

### 接入导航前必须完成的小幅适配

1. **补 odom TF 桥接。** 插件已产生 Gazebo `/limo/tf`，但 launch 没有桥接。可在项目 bridge YAML 添加：

   ```yaml
   - ros_topic_name: /tf
     gz_topic_name: /limo/tf
     ros_type_name: tf2_msgs/msg/TFMessage
     gz_type_name: gz.msgs.Pose_V
     direction: GZ_TO_ROS
   ```

   由该插件独占 `odom → base_footprint`；不要再启动另一套 odom TF 发布者。slam_toolbox 独占 `map → odom`。

2. **接通 joint_states。** 桥接目前输出 `/limo/joint_states`，而 robot_state_publisher 没有 remap、默认订阅 `/joint_states`。在桥接侧映射到 `/joint_states`，或在 RSP 节点把 `joint_states` 重映射到 `/limo/joint_states`。

3. **统一导航数据接口。** 可将 Gazebo `/limo/scan` 映射为 ROS `/scan`、`/limo/odometry` 映射为 `/odom`；也可直接配置 slam_toolbox 与 Nav2 使用带前缀的话题。应统一一种做法，避免多个节点猜测默认名称。

4. **补正确相机 optical frame。** [基础模型](https://github.com/nevlife/hunter_gazebo/blob/harmonic/src/limo_base/urdf/limo_four_diff.xacro) 只有 `camera_link`，没有 optical frame；现有 launch 为 scoped sensor frame 添加零旋转静态 TF。做目标三维反投影时，应增加 `camera_optical_frame`（光学轴 z 向前、x 向右、y 向下）及相对 camera_link 的正确旋转，让 RGB、深度和 CameraInfo 的 header 指向相容的光学 frame，不能把 camera_link 直接当光学坐标系。Harmonic 支持 `gz_frame_id` / 相机 optical frame 配置，但应实际核对所有消息 header；尤其点云不要未经确认就只改 frame 字符串。[官方迁移说明](https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/)、[SDFormat Camera OpticalFrameId](https://gazebosim.org/api/sdformat/12/classsdf_1_1SDF__VERSION__NAMESPACE_1_1Camera.html)

5. **开放 world 参数并接迷宫。** 当前 launch 固定 `empty_mobile.sdf`。项目包装 launch 应接收自定义 world 路径，并保留 Physics、Sensors、Imu、UserCommands、SceneBroadcaster 系统。未知迷宫的真值模型仅供仿真物理和评价使用，不输入 SLAM / 探索器。

6. **避免包名冲突。** 该仓库的 `limo_base` 是模型包，而 AgileX 官方 `limo_base` 是实车串口驱动。不能不加区分地放进同一 colcon 工作空间。最短演示可使用隔离仿真工作空间；长期整合建议将移植模型改名为项目专用 description 包，并保留原版权及许可证。

7. **限制仿真控制速度。** 候选模型默认上限 1.5 m/s、加速度 2 m/s²，不能直接作为迷宫初始控制值。项目 Nav2 和速度限制器应使用经过设计的低速参数；模型几何、碰撞尺寸、有效轮距和实物 footprint 仍需比较。

模型包 [LICENSE](https://github.com/nevlife/hunter_gazebo/blob/harmonic/src/limo_base/LICENSE) 为 BSD-3-Clause，保留 AgileX 与 nevlife 的版权声明；仿真包标注 Apache-2.0。复制或重命名时应保留对应声明。

### 当前验证边界

已完成：原生 Harmonic 插件、LIMO 专用 launch、依赖、传感器和桥接源码核查，并定位两个会影响导航的直接缺口（odom TF 桥接、joint_states remap）。未完成：WSL 安装、colcon 编译、Gazebo 渲染、topic/frame 实测、Nav2 联调及迷宫实验。本附录提供可操作的最短尝试路径，不宣称上游已经通过本项目验收。
