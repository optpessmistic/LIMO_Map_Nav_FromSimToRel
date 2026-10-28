# ROS 2 Jazzy 单车任务包

实现日期：2026-10-28。包路径：`src/limo_maze_mission`。任务核心和真实 Jazzy 节点启动回归已运行；在线 SLAM 与 Nav2 接口检查通过。首次自主任务因起点盲区停止，没有发送运动目标，后续联调状态见 [验证记录](validation.md)。这些检查不能替代完整探索或实车验收。

本包负责“未知地图探索、未知数量物品的观测确认与地图标注、继续探索、最后返航”。它复用外部启动的 SLAM Toolbox 和 Nav2，不实现 SLAM，不直接发底盘速度，不读 Gazebo 实体坐标，也没有预置目标数量或期望目标坐标。

## 运行方式

在 Ubuntu 24.04 / ROS 2 Jazzy 环境构建：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select limo_maze_mission
source install/setup.bash
ros2 launch limo_maze_mission mission.launch.py
```

默认 `autonomous:=false`，节点等待 `/map`、TF 和 Nav2 就绪后记录起点并进入 `dry_run`，不会发送导航目标。先独立启动仿真、SLAM Toolbox 与 Nav2，并确认全链路后，才对仿真显式启用：

```bash
ros2 launch limo_maze_mission mission.launch.py \
  autonomous:=true use_sim_time:=true output_dir:=mission_results
```

这不是实车启动命令。正式车辆的运动模式、相机、制动、footprint 和传感器话题必须先完成独立适配。运行任务时只保留本节点作为 `NavigateToPose` 客户端；不要同时启动 `explore_lite`、其他任务导航器或在 RViz 发送目标。

默认任务安全半径为 0.25 m，覆盖 Nav2 的 0.36×0.24 m 矩形及每边 0.02 m padding（外接圆约 0.244 m）。不应通过缩小此半径或无依据地把未知格标为空闲来消除初始盲区；需先完成有依据的初始观察。实车应以测得的包络重新配置。

前置且有后方盲区的雷达即使整圈扫描，也可能始终看不到车心附近。为表达独立核验过的启动区，提供默认关闭的 `initial_clearance_radius_m=0.0`。只有实验布置确实保证净空时，才同时设置正半径和 `initial_clearance_source` 来源说明；默认另扣 0.05 m 启动位姿与定位裕量。先验固定在该会话记录的 home，不跟随机器人移动。

规划副本只填充四角全部位于先验圆盘内的未知格，保留测得的占用/中间值，不扩展地图，不改 `/map`。前沿、视觉待观察区域及失败解除均从原始观测图计算；副本中的未观测格单独计数，仍会阻止地图完成。结果 JSON 的 `initial_clearance_prior` 记录来源、半径、裕量、epoch 和填充数量。地图重置或定位跳变时仍中止任务。Nav2 使用原始地图及自身 costmap，因此副本通过检查并不代表执行层已经能成功出发或返航，必须以实际运行验证。

## 接口

| 输入/输出 | 默认名称 | 类型与用途 |
|---|---|---|
| 地图输入 | `/map` | `nav_msgs/msg/OccupancyGrid`，可靠、transient-local QoS。 |
| 位姿输入 | `map → odom → base_link` | TF。运动规划使用已估计的地图位姿。 |
| 目标观测输入 | `/mission/target_observations` | `std_msgs/msg/String`，下面的 JSON 协议。 |
| 地图代次输入 | `/mission/map_epoch` | `std_msgs/msg/String`，SLAM reset/reload 协调器发布的代次 ID，可靠、transient-local。 |
| 导航 | `navigate_to_pose` | `nav2_msgs/action/NavigateToPose`，本节点独占目标句柄。 |
| 状态输出 | `/mission/status` | `std_msgs/msg/String` JSON，包括阶段、原因、已确认数量、覆盖状态和未处理失败。 |
| 地图标注输出 | `/mission/targets` | `visualization_msgs/msg/MarkerArray`，已确认物品的球体与标签。 |
| 任务证据输出 | `output_dir/mission_*.json` | 起点、最终位姿、已确认物品地图坐标、候选、来源、失败和事件。 |

目标观测示例：

```json
{
  "schema_version": 1,
  "stamp": 123.45,
  "frame_id": "camera_depth_optical_frame",
  "source": "red_rgbd_sim_baseline",
  "detections": [
    {
      "class_id": "red_object",
      "position": [0.10, -0.04, 1.25],
      "confidence": 0.92,
      "instance_id": "optional-stable-id"
    }
  ]
}
```

- `stamp` 是 ROS 时钟下的**图像采样时刻，单位秒**，可含小数；也兼容 `{"sec":123,"nanosec":450000000}`。不能填 Unix 时间替代仿真时间。
- `position` 是 `frame_id` 中的米制三维坐标。通常使用相机 optical frame：右为 +X、下为 +Y、前为 +Z。节点在该采样时刻查询 TF 后转换到 `map`。
- `source` 必填，并记录在结果中。仿真红色替代物必须标为 `red_rgbd_sim_baseline`；不代表真实目标身份已经定义或识别器已经验证。
- `class_id` 由感知器给出；`confidence` 在 0 到 1 之间；`instance_id` 可省略。感知器不得使用 Gazebo 真值生成这些字段。
- 每帧可包含零个、一个或多个检测。未知目标数量不影响任务停止条件。零检测仅表示该帧没有接受的检测。
- 节点默认在 3 秒窗口内收到 3 个不同时间戳、置信度至少 0.70、地图平面距离在 0.30 m 内的一致观测后确认目标。相同时间戳重放不累计次数；不一致远点不会更新旧目标。
- 相同类别且距离小于聚类半径的相邻物体可能被合并。若实验物体很密集，需要更好的实例跟踪或唯一标记，不能用这个最小空间聚类器承诺物理物体数量完全准确。

## 可选视觉覆盖证据

仅靠二维激光覆盖不能证明物品搜索完成。感知器尚未对有效检测距离、视野、目标高度和遮挡作验证时，不应发送 `coverage`。本包依然可以完成几何地图探索、记录已确认物品并返航，但会输出 `perception_unverified_incomplete` 或其他不完整原因，`mission_success` 为 `false`。

经过验证的检测器可以在已成功处理的同一帧上补充：

```json
"coverage": {
  "valid": true,
  "near_m": 0.20,
  "far_m": 2.0,
  "horizontal_fov_rad": 1.0
}
```

这声明：在约定的目标高度/可见性范围内，该帧具有经过实验验证的搜索能力。节点把 optical +Z 转成地图中的朝向，用距离、水平视角、已知自由空间和遮挡射线计算独立的视觉搜索覆盖；不会直接把整张雷达地图当视觉已搜索。俯视/仰视过大的相机轴会被拒绝。

这个二维近似不能证明所有高度、所有遮挡背面均被观察。`target_definition_verified` 默认 `false`，只有真实目标定义、相机视野与检测范围经实验验证后才可修改；不能为了让结果显示成功而改参数。即使所有地图格已观察，红色仿真替代物也不自动满足正式实物识别验收。

## 探索、失败与完成

纯核心位于 `limo_maze_mission/core.py`。规划只接收当前 OccupancyGrid、机器人估计位姿和已接受观测。值为 0 的格子才允许走；未知格、地图之外和非零占据概率均保守视为路径障碍。安全格要求圆形车体 footprint 落在已知自由空间内，ROS 节点默认半径 0.25 m。它适配当前差速导航基线，不能直接作为阿克曼规划器。

前沿是与未知区域相邻的已知自由格。候选必须属于机器人可达的安全连通分量，并在前沿附近有可见路径；目标方向朝向前沿。几何探索结束后，若检测器提供有效覆盖，规划器继续选择视觉未覆盖区域的观察位姿；否则明确记录视觉未验证。

前沿候选还必须离当前实际估计位姿严格超过 `frontier_min_translation_m`，默认 0.20 m，并排除站位与前沿 anchor 重合、朝向向量为零的情况。0.20 m 是当前仿真 Nav2 `xy_goal_tolerance=0.15 m` 加余量的配置，避免目标一发出即被判到达；实车必须随实际 goal checker 核对，不能直接照搬。可通过 `mission.launch.py frontier_min_translation_m:=0.20` 配置；仓库没有额外的 `mission.yaml`。此限制仅用于前沿目标，视觉覆盖目标仍允许有意义的原地转向。目标距离超过阈值不保证实际行驶同样距离，也不保证产生新观测；完成判据仍依据原始地图和有效视觉证据。过滤后没有候选时保留不完整结果，不用先验填充或缩小 footprint 放行。

本版按距离排序寻找首个可行观察区域，再在该区域内选择已知格网路径较短的站位，不保证全局最优探索顺序。圆形 footprint 用逐行前缀和检查，避免对所有格反复枚举整个圆盘。2026-10-28 Windows 配套 Python 的空房测试中，0.05 m 分辨率下 31×31、51×51、201×201 地图的覆盖候选选择约为 0.005、0.014、0.248 秒；这是本机微基准，不是 ROS 实时性能保证。更大地图、复杂不可达前沿和真实相机覆盖流仍需测量回调延迟。

失败按区域保存，默认每区域最多 2 次、重试冷却 8 秒。导航失败、被拒绝、超时以及到达后没有获得新观测都会计入失败。只有实际观测证明该前沿/覆盖目标已解决时才清除对应记录。“无候选”和“所有目标被拉黑”均不直接表示成功；未处理黑名单、不可达候选、视觉未验证和任务超时均保留不完整状态后返航。

任务默认探索时间上限 900 秒，单目标上限 90 秒，返航总上限 180 秒。这里使用主机单调时钟衡量看门狗时间，避免 ROS 仿真时钟暂停时永远等待。探索超时一旦发生，`mission_timed_out` 永久记录；即使返航途中补齐覆盖，最终仍是不完整结果。原始 ROS 时间重置会中止旧任务，要求新的任务实例。

最终 `mission_success=true` 要同时满足：

1. 当前已知可达区域没有待处理 frontier，且无未解决的前沿失败。
2. 当前检测器仍健康，有有效视觉覆盖证据，所需可达自由区域均被观察。
3. 没有未解决的探索/观察失败，正式目标定义与检测范围已确认。
4. 返航 Action 成功，并由 TF 独立检查位置误差不超过 0.15 m、朝向误差不超过 0.26 rad，位姿稳定保持 2 秒。

返回途中新观测可能暴露额外前沿或视觉空缺。因此在起点验收前会重新计算；剩余探索时间允许且存在可执行候选时继续探索，避免沿用过期的“已完成”判断。未知数量场景允许零个目标，但必须有完整搜索证据才能把它视为已完成；无检测器不能据此宣布“没有目标”。

## 导航控制权和取消协议

节点只有一个 ActionClient、一个活动句柄和一个递增 token。一个目标未接受/未结束/未取消时不会发下一个目标。暂停、超时或切换返航时先停止生成目标，请求取消，再等待目标终态。取消响应中的 `goals_canceling` 只表示取消已接受，不足以解锁下一个目标；确认目标终态才可以切换。已经成功/失败终止的目标也不需要额外的取消 ACK。

过期回调按 token 丢弃。迟到的旧目标接受响应会触发取消。Action 结果异常不被当作终态证明；节点保留句柄并进入阻塞处理。取消超时将输出 `cancel_timeout_stop_not_confirmed`，不继续发导航目标。它表示无法证明停止，需检查导航器或使用独立急停；程序退出也不被当作车辆已制动的证明。

SIGINT/SIGTERM 退出时，主进程保持 ROS context 存活并停止任务定时器，通过同一取消路径等待自己目标的明确终态。取消 ACK、发送响应异常和 `STATUS_UNKNOWN` 都不作为目标已结束的证据；结果请求同步异常也不能跳过取消。之后要求至少 0.5 s、3 个以上连续新鲜里程计样本，线速度不超过 0.005 m/s、角速度不超过 0.02 rad/s，才记录 wheel-odometry `stop_verified`。这是里程计证据，不能替代独立物理停车测量。

有界清理结束时若目标还在等待接受，客户端销毁后的迟到接受仍可能发生：当前尚无控制器租约/看门狗来根除这一窗口。审计明确保留 `late_acceptance_after_client_destroy_possible` 并报告 `stop_unverified`，不得据此进行无人值守实车部署。每次退出写独立 `shutdown_*.json`，保留此前任务结果；异常和证据不足不覆盖为成功。

## TF、闭环优化与地图重置

起点在非空 `map_epoch`、地图、TF 与 Nav2 就绪后从 `map` 中实测保存，绝不默认 `(0,0,0)`。代次缺失或空白时不会记录起点或发送目标；启动等待到期记录 `startup_no_map_epoch`。运行中空白或改变代次会取消任务、使坐标失效，并保留原任务绑定代次供审计。目标位置使用图像时刻的 TF；TF 过旧、地图停止更新或明显定位跳变会取消任务并停止继续自主发目标，而不是拿不可信坐标返航。

地图尺寸和 X/Y 原点随 SLAM 扩展是正常现象，支持继续运行。地图 frame、分辨率或原点旋转改变会要求重新锚定。SLAM reset/reload 协调器必须在任务前发布 `/mission/map_epoch`，每次重置/加载新地图时改变 ID；本包在代次改变后停止并要求启动新任务。**同坐标系、同分辨率的静默地图替换不可能仅凭 OccupancyGrid 可靠识别，因此该代次协议是集成要求。**

节点监测 `map→odom` 的突变，默认单次超过 0.75 m 或 0.7 rad 即阻塞；地图/定位锚定失效时清除 RViz 标记，输出 `coordinates_valid=false`，保留历史观测仅作为审计证据。它不将历史里程计起点乘以当前 TF 来伪装修正结果。

本版记录的目标是地图坐标点，尚未绑定 SLAM 位姿图关键帧。小幅或渐进式闭环优化对旧目标位置的影响仍需要后续重观测或位姿图关联来解决；真实目标定位精度及返航实际落点应独立测量，不能只读取 Action 成功状态。起点物理标记可用于进一步验证，但本包尚未实现标记对准。

## 验证

无需 ROS 的核心测试：

```bash
python -m unittest discover -s tests -p test_ros_mission_core.py -v
```

测试覆盖旋转地图原点、未知/障碍 footprint、优化膨胀与直接碰撞检查一致性、对角遮挡、可达安全目标、前沿最小平移阈值和实际位姿偏移、零朝向候选、先验不计观测完成、地图与视觉覆盖分离、过期检测器、黑名单与重试边界、多目标确认、重复时间戳、异常点和置信度，以及取消超时后迟到接受响应仍会被取消。取消回调回归通过提取真实函数并用假 Future 执行验证，不运行 ROS executor，也不证明实际回调交错已通过。可选 Jazzy 启动回归在独立 domain 74、`autonomous=false` 下验证节点与参数接线；没有 ROS 的环境明确跳过。配置回归读取实际 Nav2 YAML，检查任务半径覆盖 padded footprint，且前沿最小平移大于 XY goal tolerance。尚待完整 ROS 流程验证的项目包括 Nav2 取消与拒绝回调链、运动中的时钟/TF 重置、目标视觉输入、晚到前沿及完整返航。

上游接口依据：[Nav2 NavigateToPose](https://api.nav2.org/actions/jazzy/navigatetopose.html)、[Nav2 Simple Commander](https://docs.nav2.org/jazzy/configuration_and_development/simple_commander_api/simple_commander_api/)、[ROS REP 105](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)。具体环境可能需要调节 TF 时间容差、车体 footprint 和候选距离，必须以联调结果为准。
