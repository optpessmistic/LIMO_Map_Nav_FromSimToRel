# 本机试跑：建图与目标标注

当前任务是自主探索未知迷宫，保存地图并标注已确认物品，结束时停车。默认 `return_home=false`；需要回到起点时单独启用。没有抓取、搬运或解救动作。目标类别暂未确定，下面继续用可替换的红色 RGB-D 仿真识别器。

需要增加岔路、死胡同和回路时，使用[复杂迷宫试跑](complex-maze-trial.md)。该场景有独立的任务及留存预算配置，应重新启动整套建图会话，不能直接沿用下面的小迷宫起点指纹和运行目录。

这些命令基于本机已有的 WSL Ubuntu 24.04、ROS2 Jazzy、Gazebo Harmonic 及 `manual_trial_01` 使用的配置。本次代码与文档修改没有启动新仿真，不能据此声称修改后的整轮任务已通过。历史试跑仍保留原结果。

## 从 Git 更新源码后的安装

使用本轮已构建的本机安装时，直接从下一节开始。以后从 Git 获取新源码时，先结束自己上一轮的任务，再运行下面的构建；不要在已有自主任务运行时替换安装源码。从 Windows 终端执行 `wsl -d Ubuntu-24.04` 进入 Ubuntu；若提示符有 `(base)`，执行 `conda deactivate`。以下命令均在 Ubuntu Bash 中执行：

```bash
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
source /opt/ros/jazzy/setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/install/local_setup.bash
source outputs/slam-steady-map-ws/install/local_setup.bash
colcon build --base-paths src --packages-select limo_maze_mission limo_maze_perception limo_maze_bringup
```

此步骤安装任务、感知及启动文件，不启动机器人；本机已有的独立 SLAM 安装不需要因此重建。当前本机已安装上一轮修复，本次新增 YAML 从源码目录直接读取，无需重新构建。

## 五个终端的共同环境

先按文末顺序关闭上一轮会话。打开五个 Ubuntu 终端（Windows 中执行 `wsl -d Ubuntu-24.04`；有 `(base)` 时执行 `conda deactivate`），每个都先执行：

```bash
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
source /opt/ros/jazzy/setup.bash
source /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/install/local_setup.bash
source outputs/slam-steady-map-ws/install/local_setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=73
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=limo_maze_jazzy
unset GALLIUM_DRIVER MESA_D3D12_DEFAULT_ADAPTER_NAME
mkdir -p "$HOME/limo_runs"
export LIMO_TRIAL="$HOME/limo_runs/simple_maze_trial_01"
```

下次运行将五个终端的 `simple_maze_trial_01` 一起换成新的名称，保留旧结果。结果写入 WSL Linux 文件系统。此环境顺序先加载 F 盘模型依赖，再加载 C 盘的 SLAM 和当前源码安装。`ros2 pkg prefix slam_toolbox` 应指向当前仓库的 `outputs/slam-steady-map-ws/install/slam_toolbox`；`ros2 pkg prefix limo_maze_mission` 应指向当前仓库的 `install/limo_maze_mission`。

## 启动场景、导航与初始检查

**终端 1：Gazebo 三维场景。** 仅在没有上一轮仿真服务器时执行；不要只关窗口后再重复启动。

```bash
mkdir "$LIMO_TRIAL" &&
ros2 launch limo_sim_bringup sim.launch.py \
  world:="$PWD/worlds/smoke_maze.world" \
  headless:=false \
  spawn_artifacts:="$LIMO_TRIAL/spawn"
```

首次启动可能需要一两分钟。目录已存在时 `mkdir` 会拒绝，先选择新名称。不要删除旧结果来绕过它。

**终端 2：SLAM 与 Nav2。**

```bash
ros2 launch limo_maze_bringup mapping_navigation.launch.py \
  nav_params:="$PWD/src/limo_maze_bringup/config/nav2_rpp_tf_experimental.yaml" \
  slam_params:="$PWD/src/limo_maze_bringup/config/slam_smoke_experimental.yaml"
```

**终端 3：运行检查并原地环视。** 下述 `.30 m` 起点净空声明仅用于未修改的 `smoke_maze.world` 和默认起点；修改障碍或起点后不能照抄。该 world 的 SHA256 应为 `1ef2017583d448e36fb35825bbef6735536c2fe3cbf3a7a57380cf365c708811`。

```bash
sha256sum worlds/smoke_maze.world
/usr/bin/python3 scripts/check_sim_runtime.py \
  --timeout 120 --output "$LIMO_TRIAL/sim-runtime.json" &&
/usr/bin/python3 scripts/check_navigation_runtime.py \
  --output "$LIMO_TRIAL/navigation-runtime.json" &&
/usr/bin/python3 scripts/initial_survey.py \
  --execute --verified-clear-radius .30 \
  --output "$LIMO_TRIAL/initial-survey.json"
```

环视会使仿真中的差速 LIMO 原地旋转，以补充初始地图与相机视野。确认环视结果 `success: true`、`stop_verified: true` 再继续；失败时保留输出并停止后续步骤。该命令不能直接用于阿克曼模式的实车。

## 感知、可视化与自主探索

**终端 4：红色替身识别及图像留存。**

```bash
ros2 run limo_maze_perception red_rgbd_detector --ros-args \
  --params-file "$PWD/config/simple_maze.yaml" \
  -p retention_output_dir:="$LIMO_TRIAL/retained"
```

**终端 5：RViz 地图、相机、导航路径和目标标注。**

```bash
bash scripts/view_sim.sh rviz \
  --runtime /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel
```

Gazebo 看物理运动，RViz 看机器人接收到的地图与感知结果。两个窗口的含义、图层与常见问题见[可视化说明](visualization.md)。

**回到终端 3：启动唯一的自主任务。** `simple_maze.yaml` 绑定小迷宫起点指纹，默认关闭返航：

```bash
ros2 run limo_maze_mission mission --ros-args \
  --params-file "$PWD/config/simple_maze.yaml" \
  -p output_dir:="$LIMO_TRIAL/mission"
```

现在小车会探索、识别、确认目标位置并标注；找到目标不会提前终止探索。地图检查及所配置的观察阶段结束后，停在当前地点验证静止并保存。`return_timeout_s` 在返航关闭时不支配探索完成流程。任务仍可能因地图过期、导航或观察失败等原因停止，新范围不绕过这些检查。

小迷宫配置沿用最新留存期限：单个 writer 15 秒、排队请求 60 秒、整次观察 90 秒、感知退出 20 秒，正常完成即继续。3 帧与持久化回执要求保持。最多 3 个观察站、每站 4 个朝向；观察阶段上限为 720 秒，总任务上限为 1800 秒，单导航目标 180 秒，给慢速 WSL 留出余量。这些是墙钟上限，不是预计耗时。留存上限 96 帧、64 窗口，元数据 4 MiB、文件 256 MiB；目标数量和真值地图不输入任务。

如需返航，在任务命令最后追加 `-p return_home:=true`。到家停稳复查后，若还有可达探索边界，且尚未超过任务总时限、补图次数和累计尝试上限，可以再次出发补图；此时释放上一段返航期限，后续真正再次返航时重新获得该段返航预算。同一次返航内部的失败重试不会延长期限。若最终检查仍有缺口，照实记录未完成。

也可以在自行配置的 launch 工作流中传 `ros2 launch limo_maze_mission mission.launch.py return_home:=true`；该 launch 默认 `autonomous:=false`，且未提供上面全部试跑参数，不能用这条单独替代完整试跑命令。

## 查看保存结果

任务终态写入 `$LIMO_TRIAL/mission/mission_<timestamp>_<id>.json`，同名 `_map` 目录保存：

| 文件 | 内容 |
|---|---|
| `map.pgm` / `map.yaml` | 可由 ROS 地图工具加载的地图，含分辨率与旋转原点 |
| `map.json` | 原始占据值、原点、时间戳、`frame_id` 和 `map_epoch` |
| `targets.json` | 已确认目标的编号、类别、地图 XYZ 坐标和同一会话身份 |
| `annotated.svg` | 可在浏览器打开的地图及目标编号、平面坐标 |
| `manifest.json` | 上述五个文件的大小和 SHA256 校验值 |

在任意完成环境设置的 WSL 终端执行 `explorer.exe "$(wslpath -w "$LIMO_TRIAL/mission")"` 打开 Windows 资源管理器，进入 `_map` 子目录，双击 `annotated.svg` 查看。已接受并留存的 RGB-D 在 `retained` 中。标注不改变障碍栅格；中间占据值保守保存为未知，原数值仍在 `map.json` 中。

任务 JSON 的主要字段：

| 字段 | 判定含义 |
|---|---|
| `map_complete` | 当前探索器对地图完整性的检查结果；不是独立测绘精度证明 |
| `mapping_annotation_complete` | 地图完成、有效坐标和会话、近期感知输入、停车验证以及标注包保存均满足 |
| `task_success` | 上述交付通过，且无任务超时/未解决失败；启用的有限观察动作全部完成；若要求返航，还须通过返航验证 |
| `completion_stop_verified` | 通过新鲜轮式里程计验证停稳，不等于独立物理制动证据 |
| `artifacts.status` | `saved` 表示文件包已校验保存，`failed` 的错误保留在任务 JSON 中 |
| `all_targets_found` | 当前为 `null`，不宣称所有真实物品均被发现 |
| `mission_success` | 保留历史严格完整视觉覆盖、正式目标定义及返航口径；不是当前任务成功的唯一判据 |

`status=done` 只表示停止任务调度，不等于 `task_success=true`。有目标坐标或看起来完整的地图也不单独证明成功。失败任务在坐标仍有效时可保存明确标为未完成的部分地图；无地图、无效坐标或导出失败不会生成可用标注包。目标数量未知，因此空目标列表合法，但不能解释成“已证明环境中没有目标”。

本手动流程没有自动实验的完整录包、独立原生轨迹及关闭后严格审计；需要正式实验记录时另按对应证据流程执行。

## 停止与下次重跑

先在终端 3 按 `Ctrl+C`，等待目标取消和停车检查完成，再在终端 4、2、1、5 依次按 `Ctrl+C`，关闭感知、导航、仿真及 RViz。任务已显示 `done` 或 `blocked` 仍应完成这个关闭流程。只关 Gazebo 窗口不保证全部进程已退出。

下一次使用新运行目录，并重新启动整套建图会话。不要运行绑定历史输出及源码校验的 `start-run34.sh`；不要同时启动多个任务节点、点击 RViz 导航目标或重置当前场景。
