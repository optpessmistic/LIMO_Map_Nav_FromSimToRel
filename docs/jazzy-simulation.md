# LIMO：ROS2 Jazzy / Gazebo Harmonic 仿真

本接入用于 WSL Ubuntu 24.04；只启动仿真，不启动串口底盘、真实雷达或真实相机。导航、探索与物品标注由项目其他包接入。

## 来源与复用方式

模型来源为社区 [nevlife/hunter_gazebo](https://github.com/nevlife/hunter_gazebo/tree/harmonic)，固定提交
`8e1d4aeb459e77d549262ca20cac819d9c7a32ee`，在 2026-10-28 核实源码。它使用原生 `gz-sim-diff-drive-system`、`gpu_lidar`、`rgbd_camera` 和 IMU；不使用 Gazebo Classic 插件。

`vendor.repos` 记录版本，`scripts/fetch_sim_vendor.sh` 下载必要目录到 Git 忽略的 `vendor/hunter_gazebo` 并应用 `patches/0001-limo-harmonic-project-interfaces.patch`。模型与网格不进入本仓库 Git。补丁将模型包重命名为 `limo_harmonic_description`，避免与实车驱动包 `limo_base` 冲突；上游 BSD-3-Clause 和 Apache-2.0 许可证随安装保留。

没有构建或启动 Hunter、Scout、RGL/CUDA 插件。fetch 脚本为仅提供传感器 Xacro 的上游 `gazebo_harmonic` 目录写入 `COLCON_IGNORE`，防止常规 `colcon build` 误构建其额外机器人依赖。LIMO 的激光和 RGB-D 传感器仍需要可工作的图形渲染上下文。

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

`headless` 仅指关闭 GUI。传感器仍通过 ogre2 渲染。正常 WSLg 可保留其 DISPLAY；若环境已正确支持 EGL，可设置 `headless_rendering:=true`。该开关不保证主机 GPU 一定支持 EGL。遇到渲染错误应解决 WSLg/GPU 环境，不能关闭激光或用地图真值假装建图数据。

world 参数接受 SDF world 文件；启动时读取其 `<world name>`，用于正确的 Gazebo spawn 服务。迷宫世界必须提供 Physics、UserCommands、SceneBroadcaster、Sensors（ogre2）和 Imu 系统。项目内置空场景有这些插件，可用于排除世界配置问题。世界真值只参与物理仿真与实验评价，不输入 SLAM 或目标检测节点。

## ROS 数据接口

| ROS topic | 类型 | frame / 说明 |
| --- | --- | --- |
| `/cmd_vel` | geometry_msgs/Twist | ROS → Gazebo，仅接收非 stamped 速度命令 |
| `/odom` | nav_msgs/Odometry | `odom`，child `base_footprint`；车轮里程计 |
| `/scan` | sensor_msgs/LaserScan | `laser_link`；240°、720 点、8 Hz |
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

该检查订阅实际 ROS 消息，核实传感器帧、RGB/深度编码、内参、720 束激光，以及 RGB 观测时间的 `odom → camera_optical_frame` TF；将时间戳、内参与检查结果写入 `outputs/ros_smoke/sim_runtime.json`，并以退出码表示是否通过，不发送速度命令。

当前文件记录已实现的接口与启动方式。实际运行验证结果会在 ROS 安装完成、编译和传感器冒烟测试后补充；在这些检查完成前，不将仿真、导航或实车实验描述为已经通过。
