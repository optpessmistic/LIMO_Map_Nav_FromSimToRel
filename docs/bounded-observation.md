# 有界主动观察与返航复查

任务节点提供默认关闭的 `bounded_processed_frames` 模式：在当前地图满足几何完成判据后，从已观测地图生成有限的站位与底盘朝向，逐个导航、停稳并等待新的 RGB-D 处理证据，再返航。它不读取世界真值、预设目标数量或目标位置。

该模式已经接入 `observation.py` 和 `node.py`。两项隔离 ROS Action/DDS 测试已通过，覆盖双朝向各三帧后返航，以及暂停仿真时钟后的限时取消；测试使用受控 ActionServer 和合成地图/TF/里程计，没有启动 Nav2 或 Gazebo。新队列尚未完成物理仿真或实车验收，旧 run20 的结果不作为它的验证。

## 启用

从仓库根目录、已加载依赖的 Jazzy 环境构建当前代码：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select limo_maze_mission limo_maze_perception
source scripts/env.sh
```

另行启动同一 ROS domain 下的仿真、SLAM、Nav2 与 `/mission/map_epoch` 会话，确认地图、TF、里程计和相机就绪；现有实验配置见[复现步骤](rpp-smoke-reproduction.md)。深度和彩色图像已验证注册时，在感知终端运行：

```bash
source scripts/env.sh
ros2 run limo_maze_perception red_rgbd_detector --ros-args \
  -p use_sim_time:=true -p enabled:=true -p depth_registered_to_color:=true \
  -p publish_frame_processing:=true
```

在另一终端启动唯一的任务导航客户端。以下命令会在就绪后允许仿真自主运动，其他适配参数应沿用本次已核验配置：

```bash
source scripts/env.sh
ros2 run limo_maze_mission mission --ros-args \
  -p use_sim_time:=true -p autonomous:=true \
  -p require_processed_inspection:=true \
  -p post_mapping_observation_mode:=bounded_processed_frames \
  -p observation_station_spacing_m:=1.0 \
  -p observation_max_stations:=3 -p observation_headings_per_station:=4 \
  -p observation_max_dispatches:=12 -p observation_stage_timeout_s:=180.0 \
  -p home_frontier_resume_limit:=1 -p home_frontier_return_reserve_s:=15.0 \
  -p output_dir:=outputs/mission/bounded-observation
```

观察模式必须同时设置 `require_processed_inspection:=true`，否则构造节点直接报错，不退回固定时长等待。`autonomous` 默认仍为 `false`。不要同时运行另一任务节点、`explore_lite` 或从 RViz 发送导航目标。此命令不启动仿真、保存地图或录制原始图像；录包见[限时录包](bounded-recording.md)。

## 参数

| 参数 | 默认值 | 实际含义与范围 |
|---|---:|---|
| `post_mapping_observation_mode` | `off` | 仅允许 `off`、`bounded_processed_frames`。 |
| `require_processed_inspection` | `false` | 有界观察必须为 `true`；同时约束之前的非返航探索到达。 |
| `observation_station_spacing_m` | 1.0 | 站位去重/分散采样间距，有限正数；不是相机有效距离。 |
| `observation_max_stations` | 3 | 站位数量上限，整数 1–64。 |
| `observation_headings_per_station` | 4 | 每站均匀分布的底盘 yaw 数，整数 1–16。 |
| `observation_max_dispatches` | 12 | 队列视角及尝试次数上限，整数 1–1024；每视角最多尝试一次。 |
| `observation_stage_timeout_s` | 180.0 | 从冻结队列起计的 wall 时间，有限正数；截止还受原任务截止限制。 |
| `observation_max_grid_cells` | 250000 | 输入原始地图的宽×高上限，整数 1–1000000；后续扩图也检查。 |
| `home_frontier_resume_limit` | 1 | 到家复查后最多允许几次补探索，整数 0–8；0 禁用补探索。 |
| `home_frontier_return_reserve_s` | 15.0 | 补探索必须为再次返航保留的 wall 时间，有限正数；不保证这段时间足够返航。 |
| `mission_timeout_s` | 900.0 | 从记录 home 起的探索总预算，包含主动观察及补探索，不因阶段切换重置。 |
| `return_timeout_s` | 180.0 | 第一次取消闭合并进入 returning 时建立绝对返航截止；补探索不重置。 |
| `goal_timeout_s` / `cancel_timeout_s` | 90.0 / 10.0 | 单次非返航导航、取消等待预算；都使用 wall 时间。 |
| `max_goal_attempts` | 2 | 原探索失败策略参数；此模式下全部 home 发送上限为它乘以 `home_frontier_resume_limit+1`，默认 4。不增加单观察视角的尝试次数。 |
| `robot_radius_m` / `goal_arrival_margin_m` | 0.25 / 0.05 | 公共机器人包络及到达余量，用于候选安全检查；不能为取得候选随意缩小。 |

停车和帧窗口沿用[停稳观察等待](settled-inspection.md)：默认当前位置误差 ≤0.05 m、yaw 误差 ≤0.26 rad，连续停稳 0.5 wall 秒，再接收至少 3 组新帧，两路采样跨度均 ≥0.2 ROS 秒。`inspection_timeout_s=12.0` 从当前 Action 成功终态接收起算，包含停稳与等图，范围为 (0,300]。不因重建窗口或接收新消息延期。

## 队列、失败与时限

进入条件是原始地图计算出的 frontier 为零、先验副本中仍未观测的格数为零、当前位姿安全，且没有未解决的前沿失败。它是当前已知可达区域的几何判据，不是整个未知世界的完整地图证明。

每个任务只从第一次满足条件时的**原始已观测地图**冻结一条队列，不使用启动净空先验填充格。候选沿用核心的圆形包络安全检查和已知自由格四邻域可达性；终点还保留到达余量与半格对角线余量。先取较近站位，再分散选择其他站位；按朝向轮转站位，最多生成 `min(站位数×朝向数, observation_max_dispatches)` 个动作，不保证最短路径。队列 yaw 已是底盘朝向，不再减去 `camera_heading_offset_rad`。

派发前按当前原始地图重新检查可达性与终点安全。变得不安全或不可达的视角被跳过；新揭示区域不自动补入队列，地图更新、返航和补探索都不会补回次数、重建队列或延长其截止。地图超过格数预算会明确 blocked。队列使用独立的 `observation_queue` 目标类别，动作完成不依赖 `Coverage.contains()`。

每次成功必须关联同一 epoch、action/dispatch ID、导航 token/UUID、SUCCEEDED 终态和当前处理账本。迟到回调或旧帧不能代替新窗口；导航成功本身也不代表观察完成。失败行为如下：

- 导航拒绝、已确认的失败终态或导航超时：记录该尝试失败；超时须先完成 owned goal 的取消/终态检查，再选下一视角。取消 ACK 不等于终态，无法确认终态会 blocked。
- **已到位且持续停稳，仅新处理帧不足**：到达窗口截止时记一次 `inspection_failed`，保留部分证据，下一次调度才选其他视角；不会给失败视角计成功或重复尝试。
- 未停稳、位姿超差、里程计不新鲜，或 epoch/时钟/坐标/账本身份失效：不能借上述“帧不足”路径继续，保留阻塞证据。无效 epoch 也不会被新请求重新授权。
- 队列为空、耗尽、次数用尽或阶段时间到期：关闭队列，经取消闭合后返航。正常阶段到期只退休当前请求；故障导致的失效仍需遵守地图/时钟恢复规则。

启用处理等待后，任务定时器使用 STEADY_TIME；暂停 `/clock` 不会暂停上述 wall 截止。派发前、到家最终规划后都会重新读取单调时钟，防止规划耗时越过截止仍发送或报完成。截止限制调度决策；同步回调/规划耗时和有界取消清理仍可能使进程晚于截止退出，不能把它解释成严格的整进程运行时间上限。

## 到家复查与补探索

返航须由 owned home Action 成功，再由当前 TF 检查位置、朝向及稳定时间，并满足新鲜推进的停车里程计。默认 home 容差为 0.15 m / 0.26 rad、稳定 2 秒。随后重新执行规划，检查返航途中是否出现新前沿，不沿用出发前的 `map_complete`。

只有最新规划返回可执行的 **frontier** 目标，且状态/epoch/账本仍有效、原探索预算未耗尽、未达补探索次数及 home 发送总次数上限、绝对返航截止前仍留有 reserve，才恢复探索。视觉覆盖空缺本身不触发这个补探索入口。下一轮会按最新地图再次选目标；原观察队列始终关闭，不重新扫视。

补探索沿用原 `mission_timeout_s` 和第一次返航建立的绝对截止；到 reserve 边界即切回返航。超过绝对返航截止会 blocked，不能再记录 `returned_pose_verified=true`。若已完成到家验证，但新前沿因预算、次数或状态约束未被继续处理，可以输出 `status=done` 和 `returned_pose_verified=true`，同时保留 `map_complete=false` 与不完整原因；`done` 不等于完整项目成功。

## 证据与限制

`/mission/status` 与 `mission_*.json` 的 `observation_actions` 保存队列状态、原截止、当前未闭合动作、所有已退休尝试以及 `continuation` 的返航截止/次数/补探索决策。每次尝试保留 action/dispatch ID、epoch、站位/朝向、导航 token/UUID/终态、处理账本及结束原因。预留后因截止拒绝发送的动作保留取消记录，导航字段为 null，不伪装成已派发。无法确认终态的动作继续保留在 `active`，不强行记入完成次数。事件包括 `observation_queue_frozen`、`observation_action_finished`、`home_frontier_decision`。

`queue.all_planned_actions_completed` 只表示这条有限队列全部动作取得合格处理窗口。处理帧不写入 `Coverage.frames`；本模式明确输出 `visual_coverage_claimed=false`、`visual_coverage_complete=false`，因此 `mission_success` 始终为 false，即使地图与返航检查通过、目标数非零或 `target_definition_verified=true`。没有标定过的有效距离、目标高度、遮挡和识别范围模型，不能据站位间距、四个朝向或空检测认证完整搜索。

帧摘要不能替代原始 RGB/Depth、CameraInfo、TF 和处理证据录包；处理消息也不证明目标消息已到达任务节点。停止验证来自轮式里程计，尚非独立物理停车证明。二维安全网格、坐标重置检测和正常取消路径保留原任务节点的适用边界，见[任务节点](ros2-mission.md)及[图像处理证据](frame-processing-evidence.md)。正式目标识别和新区域的视觉覆盖仍是后续工作，不能以有限队列完成缩小课程验收目标。
