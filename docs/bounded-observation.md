# 有界主动观察与返航复查

任务节点提供默认关闭的 `bounded_processed_frames` 模式：在当前地图满足几何完成判据后，从已观测地图生成有限的站位与底盘朝向，逐个导航、停稳并等待新的 RGB-D 处理证据，再返航。它不读取世界真值、预设目标数量或目标位置。

该模式已经接入 `observation.py` 和 `node.py`。隔离 ROS Action/DDS 测试覆盖双朝向各三帧后返航，以及暂停仿真时钟后的限时取消；测试使用受控 ActionServer 和合成地图/TF/里程计。实际 Gazebo run30 完成 6 个观察窗口后达到 360 秒预算；后续 run31 完成 5/12，另 7 个视图被安全重验跳过，随后返航。两轮均未完成整条观察队列或实车验收，详见 [run31 结果](../experiments/run31/run31-outcome-review.md)。旧 run20 的结果不作为它的验证。

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

以上命令保持局部重定位关闭。需要启用时，在同一任务启动命令中加入 `-p observation_relocation_radius_m:=0.15`，允许尚未派发的观察目标在原站附近最多调整 0.15 m；安全条件见下文。该值不是机器人包络、到达容差或相机覆盖半径。

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
| `observation_relocation_radius_m` | 0.0 | 默认关闭；有限数，满足 `0 ≤ r ≤ min(0.5, observation_station_spacing_m/4)`。启用后仅允许派发前在原站局部安全分量内调整实际 XY。 |
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

每个任务只从第一次满足条件时的**原始已观测地图**冻结一条队列，不使用启动净空先验填充格。原站 anchors、动作 ID、station/heading ID、计划 yaw 和动作集合保持不变；启用局部重定位时，尚未派发动作的实际 XY 可以按当前地图调整。候选沿用核心的圆形包络安全检查和已知自由格四邻域可达性；终点还保留到达余量与半格对角线余量。先取较近站位，再分散选择其他站位；按朝向轮转站位选出最多 `min(站位数×朝向数, observation_max_dispatches)` 个动作。队列 yaw 已是底盘朝向，不再减去 `camera_heading_offset_rad`。

2026-11-03 起，生成动作后的执行顺序按站点优化：每站连续完成已选朝向，再按冻结安全网格四邻域路径距离选择最近剩余站位；同站朝向按上一计划朝向的最小转角排列。保留原动作集合、ID 和预算不足时的截断选择，不把更多朝向集中到更少站位来缩小要求。它是确定性贪心排序，不保证全局最短或实际 Nav2 行驶时间；失败重试仍进入队尾，逐次派发的安全复查不变。run31 确实先完成同站四朝向，已派发序列仅一次跨站，但 7 个跳过项使其无法验证完整路线的节时效果。

派发前按当前原始地图重新检查可达性与终点安全。`observation_relocation_radius_m=0.0` 保持原行为：原 XY 不安全或不可达便跳过该视角。启用后，先尝试复用同站上一次选择的实际 XY，每次都重新检查；如果它不再满足条件，才在原 anchor 周围寻找替代点：

- 原 anchor 必须仍在当前位姿可达的全局 R 安全分量中，并通过 `R+offset` 的足迹检查。R 是未缩小的机器人半径；offset 是冻结 world XY 到当前格心的偏移，向上取整保留余量。原 anchor 已不支持机器人足迹时，不寻找新替代点。
- 候选来自距**原 anchor** 不超过配置半径的当前 R 安全格，其局部四邻域分量必须与原 anchor 相连。仅靠全局可达、但需绕墙远行的附近坐标不合格；后续调整也不会以先前替代点为圆心逐步漂移。
- 候选还须满足原终点检查：机器人半径、到达余量、半格对角线及实际 world-to-cell 偏移全部保留。未知格、中间占用值、占用格和地图外区域仍阻挡；不缩安全边界，也不使用先验填格或目标真值。
- 从合格候选中按距原 anchor 最近、全局安全网格路径步数、格索引确定性选择，实际 yaw 和原 anchor 不变。没有合格候选则保留 skip，不自动扩半径或等待地图变化后复活该动作。

局部调整只发生在新动作派发前，不修改正在执行的导航目标。同站已有成功朝向仍保留其原实际位姿和记录；后续朝向可以使用新的实际 XY，并取得自己的导航 token、UUID 和新处理窗口。它保留同站附近和相同朝向的采样意图，不能证明调整前后的相机视野等价。

新揭示区域不自动补入队列，地图更新、重定位、返航和补探索都不会补回次数、重建队列或延长截止；已关闭队列不会因此重新打开。一次实际派发仍只消费原动作的一次尝试，没有独立“移动到替代点”的免费导航。地图超过格数预算会明确 blocked。队列使用独立的 `observation_queue` 目标类别，动作完成不依赖 `Coverage.contains()`。

每次成功必须关联同一 epoch、action/dispatch ID、导航 token/UUID、SUCCEEDED 终态和当前处理账本。迟到回调或旧帧不能代替新窗口；导航成功本身也不代表观察完成。失败行为如下：

- 导航拒绝、已确认的失败终态或导航超时：记录该尝试失败；超时须先完成 owned goal 的取消/终态检查，再选下一视角。取消 ACK 不等于终态，无法确认终态会 blocked。
- **已到位且持续停稳，仅新处理帧不足**：到达窗口截止时记一次 `inspection_failed`，保留部分证据，下一次调度才选其他视角；不会给失败视角计成功或重复尝试。
- 未停稳、位姿超差、里程计不新鲜，或 epoch/时钟/坐标/账本身份失效：不能借上述“帧不足”路径继续，保留阻塞证据。无效 epoch 也不会被新请求重新授权。
- 队列为空、耗尽、次数用尽或阶段时间到期：关闭队列，经取消闭合后返航。正常阶段到期只退休当前请求；故障导致的失效仍需遵守地图/时钟恢复规则。

启用处理等待后，任务定时器使用 STEADY_TIME；暂停 `/clock` 不会暂停上述 wall 截止。派发前、到家最终规划后都会重新读取单调时钟，防止规划耗时越过截止仍发送或报完成。截止限制调度决策；同步回调/规划耗时和有界取消清理仍可能使进程晚于截止退出，不能把它解释成严格的整进程运行时间上限。

## 到家复查与补探索

返航须由 owned home Action 成功，再由当前 TF 检查位置、朝向及稳定时间，并满足新鲜推进的停车里程计。默认 home 容差为 0.15 m / 0.26 rad、稳定 2 秒。随后重新执行规划，检查返航途中是否出现新前沿，不沿用出发前的 `map_complete`。

只有最新规划返回可执行的 **frontier** 目标，且状态/epoch/账本仍有效、原探索预算未耗尽、未达补探索次数及 home 发送总次数上限、绝对返航截止前仍留有 reserve，才恢复探索。视觉覆盖空缺本身不触发这个补探索入口。下一轮会按最新地图再次选目标；已经创建过的观察队列保持关闭，不重新扫视。

若第一次因地图未完成而返航，观察阶段从未创建（没有队列、开始时间、截止或尝试记录），成功验证到家后的有限补探索可以重新允许它首次启动。事件 `unstarted_observation_stage_reenabled` 保留此前关闭原因。首次启动时仍受原任务截止限制，并额外受共享返航截止减去 reserve 限制；不延长已有截止，不补回已使用次数。

补探索沿用原 `mission_timeout_s` 和第一次返航建立的绝对截止；到 reserve 边界即切回返航。超过绝对返航截止会 blocked，不能再记录 `returned_pose_verified=true`。若已完成到家验证，但新前沿因预算、次数或状态约束未被继续处理，可以输出 `status=done` 和 `returned_pose_verified=true`，同时保留 `map_complete=false` 与不完整原因；`done` 不等于完整项目成功。

## 证据与限制

`/mission/status` 与 `mission_*.json` 的 `observation_actions` 保存队列状态、原截止、当前未闭合动作、所有已退休尝试以及 `continuation` 的返航截止/次数/补探索决策。每次尝试保留 action/dispatch ID、epoch、站位/朝向、导航 token/UUID/终态、处理账本及结束原因。预留后因截止拒绝发送的动作保留取消记录，导航字段为 null，不伪装成已派发。无法确认终态的动作继续保留在 `active`，不强行记入完成次数。事件包括 `observation_queue_frozen`、`observation_action_finished`、`home_frontier_decision`。

重定位增加以下可核对字段，关闭时也保留派发/跳过检查：

| 字段 | 含义 |
|---|---|
| `active/attempts[].planned_goal` | 冻结动作的原目标，原 anchor 和 yaw 不变。 |
| `active/attempts[].goal` | 本次实际导航目标；与队列 `active_view`、发送给 Nav2 的目标一致。 |
| `active/attempts[].selection` | 本次选择依据：地图 geometry/SHA256/header stamp、当前 pose、原始/偏好/选定 XY 的可达性与净空检查、有效安全半径、位移及原因。 |
| `queue.planned_stations_xy` / `station_dispatch_xy` | 原站坐标与同站复用的实际派发坐标，分别保存。 |
| `queue.selection_history` | 有界选择记录，包含未派发的 skip；不以缺少导航记录掩盖被跳过的动作。 |
| `inspection.expected_pose` | 当前实际 `goal` 的 x/y/yaw；新请求绑定新导航 token/UUID，不借用原位置或上一朝向的处理帧。 |

真实地图消息的 header stamp 由节点带入选择记录；没有此元数据的合成夹具显式记为 null。地图哈希绑定该次输入的几何和原始格值，不能单凭相邻 observer 快照推定它就是决策时输入。规划耗时跨过原截止时仍拒绝发送，保留已预留但未执行的实际目标及选择记录，不退回次数。

局部重定位新增的 16 项队列纯测试和 6 项 adapter 纯测试已通过，覆盖安全边界、固定动作身份、同站复用、实际目标/账本绑定和原截止。

Linux 上包含真实 ROS/DDS 闭环的 `test_observation*.py` 验证共运行 103 项，102 项通过、1 项失败。失败的是新增重定位用例：第一朝向已接受 3 帧，第二次实际导航目标返回成功后，检查窗口仍未取得新帧；记录显示 `/clock` 和里程计投递停顿，里程计墙钟年龄约 2.06 s，触发 `stationary_odometry_unavailable`，最终断言为 `fixture /clock was not consumed`。原失败的 `result.json` 和 `tests.log` 保留在 `outputs/validation/observation-relocation-dds/`；当前证据不能确定投递停顿的根因。

随后以相同生产/测试源码及相同过期/停车阈值独立启动单例复测，结果为 1 项通过。与首次整套运行相比，测试选择改为 `-k test_map_update_relocates`，外层测试进程超时从 180 s 改为 90 s，测试期间日志写入 Linux 原生 `/tmp`，进程闭合后再复制到 `outputs/validation/observation-relocation-dds-native-log/`。两个独立导航 UUID 分别取得 3 帧检查窗口；第二朝向的实际 XY 移动约 0.10 m，每次 `inspection.expected_pose` 均与本次实际目标一致，原计划目标和观察阶段截止保持不变。这证明该受控用例可通过真实 ROS/DDS 完成重定位后的观察绑定；它使用受控 ActionServer 和合成地图、位姿及帧，**不是 Gazebo 运动验证，也不是稳定性证明**。单次成功不能证明日志落盘位置导致了首次失败，不能覆盖或撤销失败记录；用例中的 `mission_success` 仍为 false。原复测 `result.json` 的 `scope` 将变化概括为仅日志位置，原文保留，解释时应同时计入上述测试选择和外层超时变化。

对 run31 小型快照的[离线重定位复算](../experiments/observation-relocation/README.md)中，半径 0 复现原来的 7 项 skip；半径 0.15 m 可依次选出 7 个假想派发目标。复算固定使用保存地图和位姿，并对每个假想派发立即注入合成 `canceled` 结果以枚举下一项，因此启用后的最终计数仍是 **completed=5、failed=7**，没有新增观察成功。它未启动机器人/仿真，也不能证明 Nav2 到达或新图像有效。原 run31 的任务失败、观察不完整及全部 12 项分母保持不变。

`queue.all_planned_actions_completed` 只表示这条有限队列全部动作取得合格处理窗口。处理帧不写入 `Coverage.frames`；本模式明确输出 `visual_coverage_claimed=false`、`visual_coverage_complete=false`，因此 `mission_success` 始终为 false，即使地图与返航检查通过、目标数非零或 `target_definition_verified=true`。没有标定过的有效距离、目标高度、遮挡和识别范围模型，不能据站位间距、四个朝向或空检测认证完整搜索。

帧摘要不能替代原始 RGB/Depth、CameraInfo、TF 和处理证据录包；处理消息也不证明目标消息已到达任务节点。停止验证来自轮式里程计，尚非独立物理停车证明。二维安全网格、坐标重置检测和正常取消路径保留原任务节点的适用边界，见[任务节点](ros2-mission.md)及[图像处理证据](frame-processing-evidence.md)。正式目标识别和新区域的视觉覆盖仍是后续工作，不能以有限队列完成缩小课程验收目标。
