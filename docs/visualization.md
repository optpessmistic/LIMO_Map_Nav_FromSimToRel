# 查看 Gazebo 和 RViz 仿真界面

Gazebo 显示三维迷宫、LIMO 和红色替身；RViz 显示机器人收到的传感器、SLAM 地图、导航路径与确认后的目标标注。两个窗口可以同时打开。以下命令在 **WSL Ubuntu-24.04 的 Bash 终端**执行，先进入仓库根目录，退出 Conda。

## 已经有仿真在运行

每个窗口使用一个终端：

```bash
# 三维场景：只启动 GUI，连接已有 Gazebo server。
bash scripts/view_sim.sh gazebo
```

```bash
# 建图和任务已经启动：打开预设好的地图、相机等显示。
bash scripts/view_sim.sh rviz
```

脚本加载 `scripts/env.sh`，使用本项目的 ROS domain 73、localhost 发现和 `GZ_PARTITION=limo_maze_jazzy`。它只打开查看客户端，不启动新仿真服务器、SLAM、导航或自主任务，也不发送运动指令。服务器未运行时，单独打开客户端不会产生仿真。

如果**当前只启动了仿真，尚未启动 SLAM**，请改用：

```bash
bash scripts/view_sim.sh rviz --frame odom
```

此时能看车体、激光和相机；地图、路径和目标层暂无数据。之后启动 SLAM，在 RViz 左侧 `Global Options → Fixed Frame` 改回 `map`。预设默认使用仿真时间，俯视图可以用鼠标拖动与滚轮缩放；`Views` 中可切换 Orbit 三维视角。

## 代码与完整运行环境位于两个目录

新 checkout 如果只构建了任务/感知包，可以用 `--runtime` 指向已经构建好模型和仿真的工作区。预设使用当前 checkout 的文件，ROS 环境和模型资源来自指定工作区；无须复制旧的 `build`、`install`、`outputs`。

本机已有完整运行环境的示例（在新 checkout 根目录执行）：

```bash
bash scripts/view_sim.sh gazebo \
  --runtime /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel
```

```bash
bash scripts/view_sim.sh rviz --frame odom \
  --runtime /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel
```

建图已启动时省略 `--frame odom`。其他电脑将 `--runtime` 改为自己的完整工作区路径。脚本找不到模型包时会报错，不会自动安装或切换到其他工作区。`--dry-run` 仅输出最终命令，可用于检查路径和环境。

## 本机从零打开场景与实时地图

以下使用本机 F 盘的完整运行环境，以及 C 盘当前 checkout 的 RViz 预设。仅在没有已有仿真时执行第一步；三个终端都保持打开。若终端提示 `(base)`，先执行 `conda deactivate`。

**终端 1：启动三维迷宫及 Gazebo 窗口。**

```bash
cd /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel
source scripts/env.sh
ros2 launch limo_sim_bringup sim.launch.py \
  world:="$PWD/worlds/smoke_maze.world" headless:=false
```

**终端 2：启动在线建图和导航服务。**

```bash
cd /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel
source scripts/env.sh
source outputs/slam-boundary-ws/install/local_setup.bash
ros2 launch limo_maze_bringup mapping_navigation.launch.py \
  nav_params:="$PWD/src/limo_maze_bringup/config/nav2_rpp.yaml" \
  slam_params:="$PWD/src/limo_maze_bringup/config/slam_smoke_experimental.yaml"
```

加载边界补丁后不要再次 `source scripts/env.sh`，以免改变 overlay 顺序。

**终端 3：打开 RViz 地图、雷达和相机。**

```bash
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel
bash scripts/view_sim.sh rviz \
  --runtime /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel
```

如果省略终端 2，给终端 3 的命令加 `--frame odom`，即可只查看模型和传感器。以上步骤没有启动自主探索，机器人保持等待；自主实验还须执行[初始检查、环视、感知与任务启动](rpp-smoke-reproduction.md)。不要同时启动第二套自主任务。任务结束后仍可保留这些窗口查看现场。

## RViz 各图层的含义

| 图层 | 话题 | 何时有数据 |
|---|---|---|
| Map | `/map` | SLAM 开始建图后；显示在线估计地图 |
| LIMO | `/robot_description` 和 TF | 模型与 TF 正常后 |
| LaserScan | `/scan` | 仿真雷达输出后，青色点 |
| Navigation plan | `/plan` | Nav2 规划路径时，绿色线；不是历史行驶轨迹 |
| Confirmed targets | `/mission/targets` | 感知与任务节点确认目标后，橙色球及文字 |
| RGB camera | `/camera/color/image_raw` | 仿真相机输出后，独立图像面板 |
| TF (optional) | `/tf`、`/tf_static` | 默认关闭，需要查看坐标轴时勾选 |

`config/simulation.rviz` 已设置 QoS：图像和激光为 Best Effort / Volatile；地图、模型描述、目标标注为 Reliable / Transient Local；路径为 Reliable / Volatile。目标层为空可能表示还未确认目标，不代表感知一定出错。过去的标注和路径也不证明当前消息仍在更新。

预设只保留移动视角、选择和聚焦工具。自主运行期间不要另加/使用 `2D Goal Pose`、`2D Pose Estimate` 或 Nav2 控制面板。Gazebo GUI 本身仍有暂停、重置与编辑场景功能；观看实验时不要操作这些控制，以免改变实验状态。

## 当前没有仿真时

先按[复现步骤](rpp-smoke-reproduction.md)启动仿真、建图和任务，再用以上查看脚本连接。仅启动三维迷宫、查看传感器时，在**完整工作区**运行：

```bash
source scripts/env.sh
ros2 launch limo_sim_bringup sim.launch.py \
  world:="$PWD/worlds/smoke_maze.world" headless:=false
```

这条命令会新建仿真 server，并自带 GUI；只在没有已有仿真时使用。车不会因打开窗口自动执行探索。只关查看脚本打开的 GUI/RViz 窗口不会停止独立运行的仿真 server；如需结束实验，按复现步骤关闭各运行终端。

## 窗口或画面异常

- **没有窗口，提示 DISPLAY/Wayland**：确认从 WSLg 正常终端启动。可在 WSL 检查 `echo "$DISPLAY $WAYLAND_DISPLAY"`，在 Windows PowerShell 检查 `wsl --version`。不要将 `DISPLAY` 随意改成旧式 X server 地址。
- **Gazebo 窗口空白、没有模型**：确认原仿真终端仍在运行，`--runtime` 指向其完整工作区，分区同为 `limo_maze_jazzy`。不要反复新开 sim launch。脚本已补齐独立 GUI 所需的模型资源搜索路径。
- **RViz 报 Fixed Frame [map] does not exist**：尚无 `map → odom` 变换；只看仿真时用 `--frame odom`，需要地图时按文档启动 SLAM。
- **RViz 地图/目标为空**：先按上表核对相应节点是否已启动。每个终端须加载同一项目的环境；没有任务运行时不会有目标标注。
- **明显卡顿**：WSL 的软件渲染会同时承担 Gazebo 传感器与两个窗口。可先只开一个查看窗口，关闭 RViz 的 TF 或相机图层；不要以界面帧率推断仿真物理时间。

查看脚本本身不安装软件。如果 `rviz2` 或 `gz` 不存在，请先检查运行环境选择，再参照[环境安装说明](jazzy-simulation.md)。
