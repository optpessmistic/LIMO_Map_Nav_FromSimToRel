# 停稳后的 RGB-D 处理等待

此功能把红色替身的处理证据接入任务节点。它默认关闭；启用后，每次非返航导航成功都必须通过当前位姿、停车及新图像检查，才能继续原有探索选择。它不生成观察站位/扫视朝向队列，也不声明视觉覆盖或正式目标识别完成。

## 启用

在已经验证就绪的 Gazebo、SLAM、Nav2 和地图 epoch 会话中，分别启动感知和任务节点。不要与另一份任务节点并行发送导航目标。其他前置步骤及实验配置见 [run14 复现](rpp-smoke-reproduction.md)。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select limo_maze_mission limo_maze_perception
source scripts/env.sh
ros2 run limo_maze_perception red_rgbd_detector --ros-args \
  -p use_sim_time:=true -p enabled:=true -p depth_registered_to_color:=true \
  -p publish_frame_processing:=true
```

另一终端启动任务，保留本次实验所需的其他参数：

```bash
source scripts/env.sh
ros2 launch limo_maze_mission mission.launch.py \
  autonomous:=true require_processed_inspection:=true
```

深度注册参数只能在实际验证后设置。没有 `/mission/frame_processing`、新 epoch、合格里程计或采样时刻 TF 时，观察不会通过。关闭此功能仍使用原有 `observation_dwell_s` 等待，历史 run14 未启用本功能。

## 判据及失败行为

| 步骤 | 默认判据 |
|---|---|
| 导航终态 | 本节点拥有的 Action 返回 SUCCEEDED；不单独作为观察完成证据 |
| 当前位姿 | 相对当前目标位置不超过 `goal_arrival_margin_m=0.05 m`，yaw 不超过 `inspection_yaw_tolerance_rad=0.26` |
| 停稳 | 连续 0.5 wall 秒、至少 3 条推进的里程计，采样跨度至少 0.2 ROS 秒；XY 速度 ≤0.005 m/s，各角速度 ≤0.02 rad/s |
| 里程计新鲜度 | 采样年龄和接收间隙均 ≤0.3 秒；样本必须晚于 Action 终态，来源为 odom 到 base_link/base_footprint |
| 采样屏障 | 以上条件成立时记录 ROS 纳秒时间；RGB 与 Depth 都必须严格晚于屏障 |
| 图像处理窗口 | 至少 3 组独立的新 RGB/Depth 时间戳，两个流首尾跨度都 ≥0.2 ROS 秒；sequence 也必须推进 |
| 处理有效性 | `status=processed`，元数据、标定/配置摘要、处理统计及时间关系一致；采样和处理完成年龄 ≤1.5 秒 |
| 采样位姿 | 在 RGB 采样时刻分别查 map→base_link、map→optical TF，且底盘位姿在上述目标容差内 |
| 总预算 | 从 Action 终态接收起 12 wall 秒，包括停稳与等图；可配置 `inspection_timeout_s`，范围 (0,300] |

里程计停止只是轮式估计证据，不是独立的物理制动测量。帧的 RGB/Depth 采样时间还须不晚于已收到的最新合格里程计，处理完成时间不得早于两路采样时间。缺少精确采样时刻 TF 的消息直接拒收；后续新帧可再次尝试，没有无界缓存或 latest-TF 替代。

运动、位姿超差或里程计中断会清空当前窗口，重新停稳后建立新屏障，但不延长原截止时间。检测节点重启、配置/标定变化会清空窗口并在接收时刻建立新屏障，不能混用前后两组帧。时钟回退及地图/定位失效遵守现有阻断规则。超时产生 `inspection_wall_timeout_incomplete` 并进入 blocked，保存失败前的证据，不将该方向视为已观察。

启用时，唯一任务调度器使用 STEADY_TIME；因此暂停 `/clock` 不会冻结 wall-time 超时。测量时间仍使用 ROS 时钟。返航验证也要求新鲜且推进的停车里程计，不能用冻结的 TF 反复读取来满足稳定等待。

## 保存与可替换边界

`/mission/status` 和任务 JSON 增加 `inspection`；事件记录 `inspection_started`、`inspection_window_reset`、`inspection_complete` 或失败/中断。账本仅保留一个窗口、一个生产端身份、计数和最后一帧的摘要/位姿，不累积所有图像。成功事件保留完成窗口快照；实际 RGB/Depth、CameraInfo、TF 和证据主题仍需单独录包，摘要不能代替图像存档。

新目标进入 `settling` 时立即建立该目标的空账本，清除前一目标的帧、生产端身份和计数；这时仍不接收图像。`sampling_after_stamp_ns` 仅在连续停稳后设置，区别于 Action 终态的初始时间下界。`conditions` 保留当前位置/yaw 误差、odom 新鲜度和阻塞条件；`last_inspection_failure` 在账本失效后仍保存当前目标的失败前快照。新建请求不会恢复已失效的地图 epoch，也不延长截止时间。

现有目标观测接口仍可由其他识别器实现。当前处理证据 schema 1 的校验器针对红色替身，包含颜色检测配置及拒绝原因；仅改 `frame_processing_source` 不足以接入任意 AprilTag/YOLO 输出。替换识别器需要对应的证据适配器或新版本校验器。

合格空检测可以满足“处理过图像”的等待；存在但无法定位的候选也保留计数。两者都不证明没有物品，也不证明目标消息已被任务节点接收。`Coverage.frames`、`target_definition_verified` 和完整任务成功判据不因此改变。

## 验证范围

纯逻辑回归验证元数据、重复帧、身份改变、采样 TF、位姿、时钟和有界状态。隔离 ROS 回归使用实际 DDS Clock、地图、Odometry、动态 TF 和证据消息；导航终态由显式 Future 测试夹具给出，没有启动 Nav2 或 Gazebo，也没有发布运动指令。具体测试结果见 [验证记录](validation.md)。

[run15](autonomous-run15.md) 已在真实 Gazebo 中完成五个处理窗口，各末帧的原始数据和采样 TF 离线一致；第六点因停车后 map 位置误差 52.898 mm 超过 50 mm 而停止，未完成建图/返航。该轮还暴露了停稳前失败误引用上个账本的缺陷，上述归属修复在其后完成，尚未重新运行 Gazebo。下一轮须复验到位余量和完整流程，随后增加有限观察朝向队列及替身识别范围标定；消息回归不能替代物理仿真或实车验收。
