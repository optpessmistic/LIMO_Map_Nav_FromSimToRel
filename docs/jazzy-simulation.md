# LIMO：ROS2 Jazzy / Gazebo Harmonic 仿真

本接入用于 WSL Ubuntu 24.04；只启动仿真，不启动串口底盘、真实雷达或真实相机。导航、探索与物品标注由项目其他包接入。

## 来源与复用方式

模型来源为社区 [nevlife/hunter_gazebo](https://github.com/nevlife/hunter_gazebo/tree/harmonic)，固定提交
`8e1d4aeb459e77d549262ca20cac819d9c7a32ee`，在 2026-10-28 核实源码。它使用原生 `gz-sim-diff-drive-system`、`gpu_lidar`、`rgbd_camera` 和 IMU；不使用 Gazebo Classic 插件。

`vendor.repos` 记录版本，`scripts/fetch_sim_vendor.sh` 下载必要目录到 Git 忽略的 `vendor/hunter_gazebo` 并应用 `patches/0001-limo-harmonic-project-interfaces.patch`。模型与网格不进入本仓库 Git。补丁将模型包重命名为 `limo_harmonic_description`，避免与实车驱动包 `limo_base` 冲突；上游 BSD-3-Clause 和 Apache-2.0 许可证随安装保留。

没有构建或启动 Hunter、Scout、RGL/CUDA 插件。fetch 脚本为仅提供传感器 Xacro 的上游 `gazebo_harmonic` 目录写入 `COLCON_IGNORE`，防止常规 `colcon build` 误构建其额外机器人依赖。LIMO 的激光和 RGB-D 传感器仍需要可工作的图形渲染上下文。

上游仿真硬编码 ±120°，与用户提供的《LIMO用户手册》PDF 第 46 页 §5.1 中 X2L 的标称 0–360°扫描范围不同；[AgileX LIMO 硬件文档](https://docs.trossenrobotics.com/agilex_limo_docs/specifications/limo.html)也列出该硬件规格。**360°发射范围不等于安装后的360°无遮挡视野**：本项目实测 360°会产生大量后向车身回波，因此默认保留 ±120°前方有效扇区，360°仅作为可选对照参数。安装遮挡与实体一致性尚未认证，没有关闭车体可见性或用真值填补盲区。

720 点、8 Hz、最小量程 0.05 m 与高斯噪声 σ=0.01 m 仍是沿用的仿真基线；它们不构成完整的 X2L 标定模型。手册另列 3000 次/秒、推荐 6 Hz、0.12–8 m 等参数，实机迁移需重新对齐采样率、最小距离和误差模型。

## 准备与编译

先完成 ROS2 Jazzy 官方软件源及基础环境安装。Jazzy 与 Harmonic 是官方配对；`ros-jazzy-ros-gz` 安装配套依赖。[官方安装配对说明](https://gazebosim.org/docs/harmonic/ros_installation/)

在 WSL 的本仓库根目录执行：

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher ros-jazzy-tf2-ros \
  ros-jazzy-teleop-twist-keyboard python3-colcon-common-extensions
source scripts/env.sh
bash scripts/fetch_sim_vendor.sh
colcon build --base-paths src vendor/hunter_gazebo/src/limo_base \
  --packages-up-to limo_sim_bringup --symlink-install
source scripts/env.sh
```

重复运行 fetch 脚本不会覆盖已经应用的补丁。若 vendor URL、提交或补丁状态不符，脚本会报错并保留已有文件。离线检查可用 `bash scripts/fetch_sim_vendor.sh --check-only`。

上面仅构建仿真依赖；构建完整项目时可去掉 `--packages-up-to limo_sim_bringup`。请勿将官方实车驱动误装为这个模型包的替代品。

每个新终端先在仓库根目录执行 `source scripts/env.sh`。它设置本实验的 `ROS_DOMAIN_ID=73`、本机 DDS 发现范围和 `GZ_PARTITION=limo_maze_jazzy`；仿真、SLAM、导航、检测与检查工具必须使用相同环境。

## 启动

最小空场景：

```bash
ros2 launch limo_sim_bringup sim.launch.py
```

自定义迷宫：

```bash
ros2 launch limo_sim_bringup sim.launch.py \
  world:="$(pwd)/worlds/example_maze.world"
```

默认 `headless:=true`，仅运行 Gazebo server。显示 GUI 时加 `headless:=false`。模型默认生成在 `(x,y,yaw)=(0,0,0)`，与世界生成器的 home 约定一致；可设置 `start_x`、`start_y`、`start_z`、`start_yaw`。该模型基座高度由 URDF 定义，默认 `start_z=0`。

雷达默认 `lidar_fov_deg:=240`；研究后向遮挡时设置 `lidar_fov_deg:=360`，并在检查脚本加 `--expected-lidar-fov-deg 360`。`worlds/smoke_maze.world` 采用 0.8 m 单元用于初次集成；`example_maze.world` 保留 0.6 m 单元作为窄通道压力用例。扩大首轮场景未改变 σ=0.01 m 噪声、机器人 footprint 或任务安全半径 0.24 m，不能据此宣称解决了 0.6 m 场景。

`headless` 仅指关闭 GUI。传感器仍通过 ogre2 渲染。正常 WSLg 可保留其 DISPLAY；若环境已正确支持 EGL，可设置 `headless_rendering:=true`。该开关不保证主机 GPU 一定支持 EGL。遇到渲染错误应解决 WSLg/GPU 环境，不能关闭激光或用地图真值假装建图数据。

world 参数接受 SDF world 文件；启动时读取其 `<world name>`，用于正确的 Gazebo spawn 服务。迷宫世界必须提供 Physics、UserCommands、SceneBroadcaster、Sensors（ogre2）和 Imu 系统。项目内置空场景有这些插件，可用于排除世界配置问题。世界真值只参与物理仿真与实验评价，不输入 SLAM 或目标检测节点。

## ROS 数据接口

| ROS topic | 类型 | frame / 说明 |
| --- | --- | --- |
| `/cmd_vel` | geometry_msgs/Twist | ROS → Gazebo，仅接收非 stamped 速度命令 |
| `/odom` | nav_msgs/Odometry | `odom`，child `base_footprint`；车轮里程计 |
| `/scan` | sensor_msgs/LaserScan | `laser_link`；默认240°前方有效扇区，可切换360°；720 点、8 Hz |
| `/imu` | sensor_msgs/Imu | `imu_link`；100 Hz |
| `/joint_states` | sensor_msgs/JointState | RSP 使用的关节位置 |
| `/tf` | tf2_msgs/TFMessage | Gazebo DiffDrive 提供 `odom → base_footprint`；RSP 提供关节 TF |
| `/tf_static` | tf2_msgs/TFMessage | RSP 提供固定连接 |
| `/clock` | rosgraph_msgs/Clock | Gazebo 仿真时钟 |
| `/camera/color/image_raw` | sensor_msgs/Image | `camera_optical_frame`，640 × 480、10 Hz |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | 与 RGB、深度相容的内参 |
| `/camera/depth/image_raw` | sensor_msgs/Image | 同一 RGB-D 相机的对齐深度；消费者须检查 encoding/单位 |
| `/camera/depth/camera_info` | sensor_msgs/CameraInfo | 同一相机内参的第二个 ROS 别名 |
| `/camera/depth/points` | sensor_msgs/PointCloud2 | optical frame；lazy bridge，仅有订阅时传输 |

传感器桥接使用 `SENSOR_DATA` QoS；Python 检测节点应使用 SensorDataQoS / best effort 兼容订阅，不能只依赖默认 reliable。所有处理节点使用 `use_sim_time=true`，TF 查询使用观测时间戳。

预期坐标链为：

```text
map                         # 由 slam_toolbox 发布到 odom
└── odom
    └── base_footprint       # 仅 Gazebo DiffDrive 发布 odom→base_footprint
        └── base_link       # 固定 z=0.15 m；导航可用该 frame
            ├── laser_link
            ├── imu_link
            └── camera_link
                └── camera_optical_frame
```

`camera_optical_frame` 遵循 x 向右、y 向下、z 向前；相对 camera_link 的 RPY 是 `(-π/2, 0, -π/2)`。补丁使用 Harmonic 的 `<camera><optical_frame_id>` 设置图像、CameraInfo 和 RGB-D 点云 header，不在 ROS 侧仅靠重命名 frame 掩盖坐标方向错误。[Gazebo RGB-D 源码](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/RgbdCameraSensor.cc)、[CameraInfo 源码](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/CameraSensor.cc)

不要另外发布 `odom → base_link`，否则会给 base_link 引入第二个父链。在线建图时 slam_toolbox 独占 `map → odom`，不同时启动 AMCL。Nav2 应配置 `robot_base_frame: base_link`、`odom_topic: /odom`、激光 `/scan`，并保持 `/cmd_vel` 为 `geometry_msgs/Twist`，或在接口边界显式转换。

仿真模型将线速度限制为 ±0.2 m/s、角速度为 ±0.6 rad/s，线加速度 ±0.3 m/s²、角加速度 ±0.8 rad/s²。Nav2 的限制应与之相容。它们是初始保守值，不是实车已经验证的性能指标。模型碰撞尺寸、定位误差、轮距与真实车体应在实机阶段另行校准。

## 验证与交付边界

启动后先确认 `/clock`、`/scan`、`/odom` 持续输出及相机 header；使用 `ros2 run tf2_ros tf2_echo odom base_link` 和 `ros2 run tf2_ros tf2_echo base_link camera_optical_frame` 检查变换。空场景激光可能主要为无穷距离，因此有墙的迷宫才适合判断测距、建图和避障。检测节点不得读取 Gazebo 目标位姿、真值地图或目标数量。

另一个已 `source scripts/env.sh` 的终端可以运行只读检查：

```bash
python3 scripts/check_sim_runtime.py --timeout 30
```

该检查订阅实际 ROS 消息，核实传感器帧、RGB/深度编码、内参、720 束激光、期望视场、每路只有一个 ROS publisher，以及 RGB 观测时间的 `odom → camera_optical_frame` TF；将时间戳、内参与检查结果写入 `outputs/ros_smoke/sim_runtime.json`，并以退出码表示是否通过，不发送速度命令。新启动后的探针可加 `--max-initial-sim-time 30` 检查时钟是否意外沿用旧服务器；这个选项不适用于已运行超过 30 仿真秒的场景。后向扇区统计和选取的实际激光束用于揭示近场遮挡，接口通过本身不代表视野无遮挡。

2026-10-28 已在本机 WSL Ubuntu 24.04.4 / ROS2 Jazzy / Gazebo Sim 8.15.0 完成以下第一轮实际验证（当时为240°、0.6 m smoke场景；保留证据不覆盖）：

- `limo_harmonic_description` 与 `limo_sim_bringup` 编译通过，2 包共 28.1 秒。
- 以 `worlds/smoke_maze.world`、`headless=true`、`headless_rendering=false` 启动成功，模型停留在默认 home，检查脚本没有发送速度命令。
- 9 路接口持续收包、时间戳推进，激光 720 束均测得有限距离；RGB 为 `rgb8`、深度为 `32FC1` 米，两者均 640×480，收到共同时间戳。双路 CameraInfo 与图像都是 `camera_optical_frame`，`fx=fy=381.36115`、`cx=320`、`cy=240`、畸变系数为零。
- `odom → base_footprint → base_link`、雷达和 optical frame 坐标链通过；能在 RGB 观测时间查询到 `odom → camera_optical_frame`。首轮结果在 `outputs/ros_smoke/sim_runtime.json`，初始化后的复查在 `sim_runtime_warm.json`。
- 渲染初始化后的检查仅用 0.62 秒墙时收齐数据。加入 SLAM/Nav2 后，10 秒原生 world stats 的 `ΔsimTime/ΔrealTime≈0.486`，记录为 `outputs/ros_smoke/world_stats_10s.ndjson`。Ogre 实际报告 `Mesa / llvmpipe (LLVM 20.1.2)`，属于 CPU 软件渲染；`/dev/dxg` 的存在不能单独证明本进程使用 GPU。后续大场景需要重新测量性能并改善图形加速。

Gazebo 对 `<gz_frame_id>` 打印“schema 未定义，保留扩展子元素”的警告；运行消息的实际 frame 已通过检查。上游模型材质在首次渲染时还产生部分材质/纹理预算警告，RGB 与深度仍正常输出。上述证据确认仿真传感器和 TF 接入，不代表完整自主探索、找物返航或实车实验已经通过；这些结果由全流程实验另行记录。

## 360° 发射与进程重启复核（2026-10-28）

240°与 360°参数均经过 Xacro 展开检查；仅重建 `limo_sim_bringup` 成功。run1 使用 0.6 m smoke，其世界另存 `outputs/ros_smoke/smoke_maze_run1.world` 和对应 metadata；run3 改为 0.8 m smoke、实验性 360°发射范围。最终 run4 保持 0.8 m smoke，默认恢复 240°有效前扇区。第一次验证与各轮失败证据均保留。

重启过程中发现，原先包含的 `ros_gz_sim/gz_sim.launch.py` 使用 shell 包装进程；本次通过 SIGINT 停止 ROS launch 后，其 shell 退出而 Ruby Gazebo server 残留。第二次启动因此出现旧服务器与新服务器同 partition 并存：`sim_runtime_run2.json` 的时钟约 695 秒，与新启动不符。**run2 是无效的混合实例证据，不能作为验收结果。**清理时只对经过命令行与父链核实的本任务 PID 发信号，没有使用宽泛 `pkill`。

本项目 launch 改为直接、无 shell 的 `ExecuteProcess` 启动 `gz sim ... --force-version 8`，Gazebo 退出时关闭同一 launch。第三次启动得到 `sim_runtime_run3.json`：首次 `/clock=0.004 s`，九路 ROS publisher 各一个，原生 GZ `/clock` 与 `/limo/scan` 也各一个（`gz_clock_publishers_run3.txt`、`gz_scan_publishers_run3.txt`），传感器与 TF 检查通过。最终退出时仍应核实本次记录的 launcher/server/bridge/RSP PID 均已结束，不能只凭 launch 返回判断没有残留进程。

保持 home 静止的后向复核 `sim_runtime_run3_rear.json` 表明模型存在明显近场遮挡：

- 实际发射范围为约 360°、720 束；后方 `|角度|>120°` 有 240 束。
- 后方 206 束为有限距离，其中 202 束不超过 0.12 m，15 束位于 0.05 m 最小量程；另外 34 束是 `-inf`。
- ±130°至 ±180°的抽样回波约 0.055–0.071 m，而 ±120°约 0.45 m、正前方约 3.47 m。模型未移动，附近墙面不在这些极近距离；结合雷达前置、与车身重叠的扫描高度，这是模型自身遮挡的强证据。

因此，360°是依据手册标称发射范围设置的实验性对照，不能称为正确的模型修复；它**没有消除后方盲区**，并可能把车身近场回波送入 SLAM。不能将这些束直接解释为空闲空间；也不能为了让起点检查通过而隐藏模型、抬高雷达、把 `-inf` 替换为最大量程，或使用世界真值清空地图。需要在原始地图中复核近场回波的影响，并以经验证的传感器过滤与初始观察流程处理；真实安装遮挡尚待实机测量。

进一步核实，[AgileX 官方教学模型](https://github.com/agilexrobotics/agilex_open_class/blob/master/limo/limo_description/urdf/limo_four_diff.xacro)同样使用 `base_footprint → base_link` 高度 0.15 m、`base_link → laser_link=(0.103,0,-0.034)`；[官方 ROS2 描述](https://github.com/agilexrobotics/limo_ros2/blob/dcc5a8637f83236ed6778af86ad826e46a20cd92/limo_description/urdf/limo_four_diff.xacro)亦相同。手册 PDF 第 8 页把 X2L 标于前显示器下方、前部凹槽中；该图支持车身遮挡在几何上合理，但不能标定具体安装高度和盲区角度。没有物理证据支持通过移动传感器修复问题，也未找到上游解释 240°选择的注释。

完成上述对照后，默认恢复为 240°，保留 360°实验选项。run3 只向已核实 launcher PID 发送 SIGINT，Gazebo/RSP/bridge 均正常退出，`processes_run3_after_shutdown.txt` 确认旧 PID 已不存在；这验证了直接启动 Gazebo 的停机修复。run4 默认 240°、0.8 m smoke 的 `sim_runtime_run4.json` 全部检查通过：720 束均为有限距离，最小约 0.371 m，九路 ROS 和原生 GZ clock/scan 均只有一个 publisher。探针在启动后接入，首次时钟 14.848 s，未沿用 run3 时钟。

初始整圈观察必须具备独立验证的旋转净空。离线实验布置检查 `outputs/ros_smoke/check_setup_rotation.py` 读取本次 smoke SDF 的所有地上静态碰撞盒，验证 home `(0,0)` 至最近障碍为 0.4 m，0.30 m 半径圆盘余量为 0.10 m；输出 `smoke_initial_rotation_setup.json` 记录 world SHA256。**这是实验起点布置先验，仅用于批准一次受保护的初始观察，不是机器人已知地图，也不是任意地点旋转的许可。**世界/目标真值不交给任务规划器，运行时仍须检查扫描、时钟、里程计、漂移与速度命令所有权，并通过真实激光积分获得起点地图。

## SLAM Toolbox 2.8.5 的纯旋转扫描筛选（2026-10-28）

run4b 的 Nav2 Spin 动作和里程计检查成功：`initial_survey_spin_run4b.json` 记录最大累计转角 6.40333 rad、回正后累计角 6.25854 rad、最大平移约 2.14×10⁻¹² m，动作结束且停车验证通过。但 `run4-after-survey-prior-comparison.json` 的原始地图仍为 75×22，最小 x 约 −0.12949 m，未扩展到后方。**该轮只能记录旋转动作成功，不能记录地图初始化通过。**当时没有记录整段 Gazebo 世界位姿，动作/里程计证据也不能替代独立的模型运动轨迹核对。

本机 Debian 包版本为 `ros-jazzy-slam-toolbox 2.8.5-1noble.20260915.204151`，安装的 `package.xml` 同样标为 2.8.5。对照官方 [2.8.5 发布](https://github.com/SteveMacenski/slam_toolbox/releases/tag/2.8.5)及该 tag 源码，发现两层不同的扫描筛选：

1. 异步前端先获取扫描时刻的底盘 odom pose，再调用 `shouldProcessScan()`，通过后才调用后端 `addScan()`；参见 [slam_toolbox_async.cpp，第 34–58 行](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_async.cpp#L34)。此处检查的是底盘位置，前置雷达原地旋转形成的圆周位移不会帮助通过平移门槛。
2. `check_min_dist_and_heading_precisely` 的默认值为 `false`；参见 [参数加载，第 413–418 行](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp#L413)。在这个分支中，前端只要检测到平方平移小于 `0.8 × minimum_travel_distance²` 就拒绝扫描，未同时检查角度。距离阈值为 0.1 m 时，对应约 0.08944 m 的平移；纯旋转始终不能通过。只有显式设为 `true`，前端才在“平移不足且转角不足”时拒绝。另有首帧、暂停、节流、时间间隔和初始扫描数检查；参见 [完整 shouldProcessScan，第 756–815 行](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp#L756)。
3. Karto 后端确实支持转角门槛，但此前已被前端拒绝的扫描到不了这里；参见 [Mapper::HasMovedEnough，第 3147–3177 行](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/lib/karto_sdk/src/Mapper.cpp#L3147)。只阅读后端文档会遗漏这个前置条件。

run4b 配置只设了 `minimum_travel_distance=0.1`、`minimum_travel_heading=0.1`，未设上述布尔开关。这是纯旋转建图的一个配置缺口，但后续独立物理测量证明它不是当前故障的完整解释。初次 CLI 查询的 `Node not found` 不能证明节点停止；后续使用 `--no-daemon` 绕过陈旧发现缓存，读到 run4c 活跃参数确为 `true`。

最小修正是在启动 YAML 显式设 `check_min_dist_and_heading_precisely: true`，保留 0.1 m、0.1 rad、0.2 s 门槛，并重新配置/启动 SLAM。该成员在配置阶段读取，不能仅用参数服务返回成功推断运行中的成员已更新。后续 run4c 必须重新执行受保护的观察与地图验证，结果另记，不追认 run4b。

最小复查同时记录扫描时间戳、odom/TF 转角、SLAM 实际接纳扫描所发布的 pose（由 node info 确认解析后的 topic）和原始 `/map` 边界/已知格数。源码在成功处理并加入 dataset 后才发布 pose，见 [第 871–885 行](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp#L871)。若启用开关后仍无接纳/扩图，再在受控旋转期间同步记录原生 `/world/limo_maze/dynamic_pose/info` 中 LIMO 的世界姿态，用于离线比较 Gazebo yaw 与 odom yaw；不得把这些真值送给 SLAM、导航或目标检测，也不能用当前静止姿态推断之前的完整运动过程。

## 原生车体运动对照（run4d）

实际采集使用 `/world/limo_maze/pose/info` 中名称为 `limo` 的 model 位姿；同层级的 `base_footprint` 相对 model 为恒等，避免把固定父架误当运动底盘。`world_odom_spin_run4d_motion.json` 保存原始样本及 3637 组时间对齐记录：

| 量 | 轮式里程计 | Gazebo 真实车体 |
|---|---:|---:|
| 最大累计转角 | 6.39642 rad | 0.0000509 rad |
| 最大离开起点距离 | 约 3×10⁻¹² m | 0.12507 m |

同时间的 `map → odom` 累计转角约 −6.23314 rad，SLAM 正在补偿轮式里程计的虚假转角。该运行的 Action 成功与 odom 停车检查均不能计为真实物理转向通过。完整场景随后停止，进一步检查关节方向、接触及摩擦模型；不能放宽探索安全检查来绕过此故障。

可复用只读采集器为 `scripts/capture_motion_evidence.py`。先 source 仿真环境，再执行：

```bash
python3 scripts/capture_motion_evidence.py --duration 60 --motion-wait 90 \
  --output outputs/ros_smoke/motion_evidence_new.json
```

它等待另一个已授权控制流程的运动，原生世界位姿只保存在离线证据中；不发布 ROS/Gazebo 命令或真值，不自动给出运动验收结论，也拒绝覆盖旧证据。
