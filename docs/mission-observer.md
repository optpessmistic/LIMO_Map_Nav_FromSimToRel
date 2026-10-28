# 仿真任务只读观察器

`scripts/observe_sim_mission.py` 保存 Gazebo 任务执行证据，用于分析探索、返航停滞和速度链。它不启动仿真、不发送导航目标或速度、不取消目标、不修改参数，也不改变地图。**仅用于本项目的 Gazebo 仿真诊断，不接实车。** 原生 world 位姿只写入离线证据，不提供给控制器。

## 运行与停止

先由主控启动本项目仿真与 Nav2；推荐在 mission 启动前另开观察会话，以记录初始参数和第一条目标。观察器要求 `ROS_DOMAIN_ID=73`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`，并使用仿真时钟。Gazebo 分区、ROS 环境由项目脚本设置：

```bash
source scripts/env.sh
python3 scripts/observe_sim_mission.py --seconds 1000 \
  --output outputs/ros_smoke/mission-observer.json
```

默认输出就是上述路径。主 JSON 或相邻 `mission-observer-snapshots/` 目录已经存在时拒绝运行；每次实验使用新名字，保留已有证据。输出位于忽略的 `outputs/` 中。

`--seconds` 是 **10–1500 秒墙时**窗口，默认 1000 秒。完整任务应覆盖任务预算、返航预算和终态后的静止观察，并为实际实时率和启动延迟留余量。例如任务 600 wall 秒、返航 180 wall 秒，RTF 约 0.5 时再采 30 sim 秒需要约 60 wall 秒，1000 wall 秒可作为起始预算；应按实际日志确认。10 秒短窗口只适用于接口诊断，不能证明已静止 30 sim 秒。

收到 `blocked` 或 `done` **不会提前结束观察器**。达到墙时上限后自动保存；也可按 Ctrl-C（SIGINT）提前保存。SIGINT 只结束观察器，不会停止机器人或取消 mission。主控须自行管理任务与仿真生命周期。脚本每 10 wall 秒打印 `NATIVE_TAIL`；只有 `ready_for_operator_to_end_observer=true` 才表示已取得下述终态后静止窗口，不代表任务成功。

## 保存的证据

| 数据 | 记录方式与用途 |
| --- | --- |
| `continuous_tf_samples` | 最多 5 Hz 墙时，先取最新 `map→base_link`，以其精确源时间戳查询 `map→odom`、`odom→base_link`。保存三者 xyz、四元数、yaw、实际 stamp、缺失错误与时间推进情况。三条齐全同 stamp 才标 `same_stamp_chain_complete`。 |
| 最新 `/odom` | 附于连续 TF 样本，保存 Odometry header stamp、parent/child、位姿和完整线/角速度、接收时间及相对 TF 的时间差。**没有把最新 odom 插值成与 TF 同时刻的测量。** |
| `command_samples` | 最多 10 Hz 墙时保存 `/cmd_vel_nav`、`/cmd_vel_smoothed`、`/cmd_vel` 最新 Twist 的 6 个速度分量。Twist 无 header，时间明确是 ROS/wall **接收时间**。序号、接收 age 和新增消息数区分新命令与陈旧缓存；缺失消息不替换为零。 |
| `plan_samples` | 最多 1 Hz 墙时保存最新收到的完整 `/plan`（nav_msgs/Path）。保留整条及每个 pose 的源 stamp/frame、xyz、yaw、四元数、接收时间、消息序号和接收/采样时目标 token。相同消息不重复写；中间发布的路径可能被跳过，消息计数保留。仅更新时间戳不视为几何改变；无效/未设置四元数的 yaw 为 null。 |
| 任务状态与事件 | `/mission/status`、`/mission/map_epoch`、目标 Marker、原始目标观测、已接受的 SLAM `/pose`，以及 mission 的 `home_recorded`、`goal_sent`、`goal_terminal`、`cancel_requested` 日志。 |
| 地图快照 | phase 改变、goal terminal 日志到达、其后约 1 wall 秒以及观察器退出时，保存原始 `/map`、当前 TF、关联目标及误差。按真实 TF 位姿检查半径为实际 mission `robot_radius_m`（查询前回退 .25 m）的圆盘是否碰到原始未知/占据格或地图边界；不填充起点先验。 |
| 原生 world 轨迹 | 订阅 `/world/limo_maze/pose/info` 的 `limo` 模型，保留仿真 stamp、xyz、yaw、实体 ID、接收墙时。用于独立物理运动与静止诊断，不用轮里程计替代。 |
| 实际参数 | 只读查询 `/limo_maze_mission/get_parameters` 和 `/controller_server/get_parameters` 各一次。记录 mission 半径、到达裕量、超时、起点先验等，以及 `controller_plugins`、`FollowPath.GoalAngleCritic.threshold_to_consider/enabled/cost_weight`；未声明参数明确 unavailable，不猜 YAML 值。 |

`FollowPath`、`map/odom/base_link`、上述 topic 与模型名是本项目约定。观察器取消了实验轮次硬编码；它不是自动发现任意机器人接口的通用工具。

## 静止证据与结果解释

末段窗口取最近 **30 sim 秒**的原生 world 样本。`evidence_sufficient` 要求至少 29.9 秒时间跨度、严格递增的仿真时间、最大样本间隔不超过 .1 秒，并且最后原生消息距保存时不超过 1 wall 秒。此后还要求 XY 包围盒对角线不超过 1 mm、yaw 跨度不超过 .002 rad，才标 `stationary_within_1mm_002rad`。整个窗口都晚于第一次终态消息时，才标 `ready_for_operator_to_end_observer`。

仿真暂停、短窗口、消息陈旧、时间重复/回退、采样缺口或末段运动都不能成为停车证明。即使数值跨度为零，也只证明在记录精度和采样窗口内没有变化，不外推至之后。

主 JSON 的 `status=window_complete/operator_interrupt_saved` 表示观察会话已保存；**不是 mission success**。任务结果须读取 `final_status` 与正式 mission JSON。缺少原生轨迹或 mission status 标为 `missing_required_evidence`；参数/路径等证据缺口、回调错误记录于 `errors`，返回码为 2。返回码 0 也不替代静止条件或任务验收。发生可捕获异常时仍尽力写失败结果并释放自己的订阅；进程被强制杀死或磁盘写失败无法保证完整输出。

还需注意：

- `/plan` 是发布的全局路径，**不是** MPPI 局部预测或 controller 内部变换、裁剪后的路径。完整尾点可用于与真实请求 goal 比较，但单凭这个 topic 不能证明内部裁剪行为。
- 三段速度是异步接收记录，不是经过身份匹配的同一条管线消息。10 Hz 之间可能发生短暂变化；单个零速或缺失消息不能直接归因给 collision monitor。
- 5/10/1 Hz 均为墙时采样率；回调繁忙时可能更低，不补写重复样本。分析应使用实际源时间、接收时间和 age。
- 地图快照与日志不是原子事务；记录了各自时间。历史目标 map 标注不是参与位姿图优化的 landmark。
- world 轨迹与 map 标注不可直接叠加；需另有独立、固定且可核验的坐标变换。目标数量真值、世界几何和轨迹不进入任务控制。

## 纯回归

```bash
python3 -m unittest tools.test_sim_mission_observer -v
```

测试使用合成消息对象及临时文件，覆盖采样节流、陈旧消息、完整路径/时间戳、参数解码、原生静止窗口、原始地图圆盘判定与拒绝覆盖。不导入或启动 ROS/Gazebo，不发送运动。通用化来自已有实验观察器，但新入口的纯测试不等于新一轮实时 ROS 验证。
