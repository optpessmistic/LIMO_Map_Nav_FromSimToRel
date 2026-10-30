# RPP 小迷宫仿真复现

此入口复用 run14 的配置，目标是重新运行探索、红色替身标注和返航。它不将 `mission_success=false` 改为成功，也不适用于尚未标定的实车。run14 使用 Ubuntu 24.04、ROS2 Jazzy、Nav2 1.3.13、Gazebo Harmonic 8.15.0 和带边界补丁的 SLAM Toolbox 2.8.5；安装包升级后应重新核对实际参数。

## 环境与配置

先按 [README](../README.md) 完成依赖、vendor 和工作区构建，按[边界补丁说明](slam-grid-boundary.md)完成独立 SLAM overlay。所有命令在项目根目录的 WSL Bash 中执行，退出 Conda，使用系统 Python。

两份配置已经入库，可直接使用。需要重新生成 RPP 配置时使用固定的仓库基线：

```bash
/usr/bin/python3 tools/configure_nav2.py --controller rpp \
  --source src/limo_maze_bringup/config/nav2.yaml
source /opt/ros/jazzy/setup.bash
colcon build --packages-select limo_maze_bringup
```

省略 `--source` 会读取**本机已安装**的 Nav2 默认参数；升级后可能与 run14 不同。`--controller rpp` 默认写 `nav2_rpp.yaml`，整块替换 `FollowPath`，不会默认覆盖 `nav2.yaml`。`slam_smoke_experimental.yaml` 仅将正式 SLAM 的粗回环窗 8→1 m、距离方差惩罚 .5→.05；回环仍开启。正确回环、大迷宫和实车漂移尚未验证。

## 启动顺序

使用新运行目录，例如 `outputs/ros_smoke/rpp_trial_01`。先结束自己上一轮的任务、录制、导航和仿真，不能有旧任务客户端或遥控器同时发命令。每次重新启动建图会生成新 `/mission/map_epoch`。

终端 1，启动已核验的 smoke world：

```bash
source scripts/env.sh
ros2 launch limo_sim_bringup sim.launch.py world:="$PWD/worlds/smoke_maze.world"
```

终端 2，加载补丁及两份可选配置：

```bash
source scripts/env.sh
source outputs/slam-boundary-ws/install/local_setup.bash
ros2 pkg prefix slam_toolbox
ros2 launch limo_maze_bringup mapping_navigation.launch.py \
  nav_params:="$PWD/src/limo_maze_bringup/config/nav2_rpp.yaml" \
  slam_params:="$PWD/src/limo_maze_bringup/config/slam_smoke_experimental.yaml"
```

`slam_toolbox` prefix 应指向本项目的 `outputs/slam-boundary-ws/install/slam_toolbox`。两份 YAML 只是启动输入，实际插件和参数还应在运行中核对，不能仅凭文件名认定加载正确。

终端 3，传感器和导航检查；都通过后再进入下一步：

```bash
source scripts/env.sh
/usr/bin/python3 scripts/check_sim_runtime.py \
  --output outputs/ros_smoke/rpp_trial_01/sim-runtime.json
/usr/bin/python3 scripts/check_navigation_runtime.py \
  --output outputs/ros_smoke/rpp_trial_01/navigation-runtime.json
ros2 param get /controller_server FollowPath.plugin
ros2 param get /slam_toolbox loop_search_space_dimension
```

## 初始观察与自主任务

run14 的初始净空依据只适用于未移动障碍、默认 `(0,0,0)` 起点的同一静态 world。其 SHA256 为 `1ef2017583d448e36fb35825bbef6735536c2fe3cbf3a7a57380cf365c708811`；最近地上静态障碍距车心 .4 m，允许 .30 m 起始净空声明。核对的是**实验布置**，不把真值地图交给控制器。修改 world、起点或添加动态物体后须重新核验，不能照抄声明。

确认仍是上述布置、没有 mission 正在运行后，先执行初始环视并检查其停车结果：

```bash
sha256sum worlds/smoke_maze.world
/usr/bin/python3 scripts/initial_survey.py --execute --verified-clear-radius .30 \
  --output outputs/ros_smoke/rpp_trial_01/initial-survey.json
```

环视后检查原始地图、当前 TF 和 .25 m 车体安全包络。地图出现重复墙影、定位跳变或实际运动异常时保留失败结果，不能靠填墙/清墙继续。起始盲区先验仅覆盖固定 home 附近的规划副本，不改原图；详见[任务节点说明](ros2-mission.md)。

完成初始环视及上述检查后，在另一个终端启动[只读观察器](mission-observer.md)，保留后续任务状态、路径、速度与原生轨迹；文件存在会拒绝覆盖：

```bash
source scripts/env.sh
/usr/bin/python3 scripts/observe_sim_mission.py --costmaps --seconds 1350 \
  --output outputs/ros_smoke/rpp_trial_01/observer.json
```

观察器必须在环视结束后启动：初始环视的协作检查拒绝节点名含 `mission` 或 `explor` 的节点，观察器的名字 `limo_sim_mission_observer` 也会触发该检查。这个顺序下观察器不记录初始环视；如果需要独立验证环视时的物理运动，应在环视前另行启动原生轨迹采集，并保留其时间和来源记录。

先在独立终端启动红盒感知；本仿真同一 RGB-D 传感器确实提供注册后的深度，实车不能直接沿用该声明：

```bash
source scripts/env.sh
ros2 run limo_maze_perception red_rgbd_detector --ros-args \
  -p use_sim_time:=true -p enabled:=true -p depth_registered_to_color:=true
```

确认上述条件后，启动唯一的任务导航客户端。以下使用 run14 的墙时预算，`initial_clearance_source` 说明本轮所依赖的独立布置：

```bash
source scripts/env.sh
ros2 run limo_maze_mission mission --ros-args \
  -p use_sim_time:=true -p autonomous:=true \
  -p initial_clearance_radius_m:=0.30 \
  -p initial_clearance_source:=smoke_world_sha256_1ef2017583d448e36fb35825bbef6735536c2fe3cbf3a7a57380cf365c708811 \
  -p mission_timeout_s:=900.0 -p goal_timeout_s:=120.0 -p return_timeout_s:=240.0 \
  -p output_dir:=outputs/ros_smoke/rpp_trial_01/mission
```

任务开始后不手动点击导航目标、不遥控、不重置 SLAM。保护停止、超时或需要接管都保留记录。本配置不传目标数量、位置或预定路线；感知输入只来自实际渲染图像。

## 结束和解释

从任务 JSON 分别读取 `map_complete`、`returned_pose_verified`、`visual_coverage_complete`、`target_definition_verified`、`mission_success`。`done` 表示流程结束，不表示所有验收项通过。红色替身节点没有有效覆盖声明，后两项未验证时完整成功仍为 false。

终态后继续记录至少 30 sim 秒，确认观察器的新鲜 native 静止窗口；按 Ctrl-C 结束观察器只保存证据，不会停止机器人。任务自身退出审计应有目标终态与静止 odom；随后按各终端的 Ctrl-C 关闭感知、导航及仿真，核对自有进程退出。保存原始地图和位姿图应在 SLAM 退出前完成。

本页是本轮配置的可操作运行顺序；不同调度、噪声和初始时序可能产生不同路线与结果。跨迷宫成功率、图像观察证据、正式物品识别、实车校准都仍是下一阶段工作。离线评分规则和本轮独立误差见 [run14 实验记录](autonomous-run14.md)。
