# 激光、TF 与仿真时间对齐诊断

`scripts/check_scan_alignment.py` 将原生 Gazebo 激光、ROS `/scan`、扫描时间的 odom TF 与原生模型位姿放在同一时间线上，再用给定世界的盒状障碍做离线射线对照。它只订阅数据，从不创建速度、动作或真值发布器，不启动/停止仿真，也不改变 SLAM。世界真值仅进入诊断报告，不能接入机器人任务规划。

## 适用范围

当前明确支持：一个 SDF world、静态且不带运动插件的独立模型、每个 link 的 visual/collision **均为 box 且尺寸和位姿一一匹配**，使用默认 visual 可见性；允许 box 旋转，也允许地板用 box 表示。`worlds/smoke_maze.world` 满足这一条件。不是通用 Gazebo 场景验证器：mesh、plane、圆柱、透明视觉物体、自定义 `visibility_flags`、动态障碍、joint、include、nested model、显式 frame/relative_to 等会记录 `unsupported` 并非零退出。

机器人参数要求使用**与实际 spawn 对应的已展开 model SDF**，其中目标 `gpu_lidar` 在 canonical link 上；局部传感器位移和旋转均参与计算。当前只支持单层、零仰角的瞬时扫描，ROS `time_increment` 必须为 0；不支持逐束运动补偿。机器人自己的遮挡几何不参加世界射线模型，因此应使用已验证不含车身回波的前方有效扇区；本项目的 360°自遮挡实验不能直接作为该校验的通过依据。

世界文件、模型 SDF、回放输入都记录绝对路径和 SHA256。**这些哈希只是输入文件身份，不能证明运行服务器实际加载了相同文件。**调用者仍须核实本轮 spawn 使用的内容、没有额外动态物体，以及采样过程中不 reset/重启或修改世界。输出中的 world/odom 位置差假设两个原点对齐；脚本不会拟合变换来掩盖偏差。不同原点的场景应先解释这个差值，不能把常量偏移直接判作桥接错误。

## 使用

在仓库根目录工作。`--output` 必须是不存在的文件，脚本拒绝覆盖；失败或不支持的输入会留下说明 JSON。回放只需要 Python 3 与 NumPy，不加载 ROS/Gazebo Python 模块。

```bash
# 回放已有原始证据，不建立 ROS/Gazebo 连接，不触发任何运动。
python3 scripts/check_scan_alignment.py \
  --world worlds/smoke_maze.world \
  --model-sdf outputs/physics_diag/formal.sdf \
  --model limo \
  --replay outputs/ros_smoke/scan_motion_run5b.json \
  --output outputs/ros_smoke/scan_alignment_replay_run5b.json
```

旧 run5b 文件没有记录采集时世界/模型哈希，回放会明确标注 `Legacy replay lacks capture-time world/model hashes`；补写当前输入哈希不会追认当时的来源认证。`geometry_provenance=legacy_unverified` 在再次回放该报告时继续保留，不会自动升级。回放新版报告时，输入世界/模型哈希必须与原报告一致，否则拒绝分析。保留旧证据，不需要修改它。

实时观察前，按项目说明 source 环境，并指定当前服务器实际使用的模型 SDF。下面两种方式都只读取已有场景：

```bash
source scripts/env.sh

# 立即观察 15 秒；静止场景也可使用。
python3 scripts/check_scan_alignment.py \
  --world worlds/smoke_maze.world --model-sdf outputs/physics_diag/formal.sdf \
  --output outputs/ros_smoke/scan_alignment_static_new.json --duration 15

# 等待由另一个已授权流程发起的运动；脚本自身不会发起 Spin。
python3 scripts/check_scan_alignment.py \
  --world worlds/smoke_maze.world --model-sdf outputs/physics_diag/formal.sdf \
  --output outputs/ros_smoke/scan_alignment_motion_new.json \
  --motion-wait 120 --duration 100
```

可用 `--model`、`--sensor-name`、`--native-scan-topic`、`--ros-scan-topic`、`--laser-frame`、`--odom-frame` 选择已存在的接口。原生模型位姿 topic 从 world 名生成。`--map-frame` 仅用于附加诊断，可传空字符串禁用；map TF 缺失不会筛掉有效的 odom/激光数据。时长最多 180 秒，等待运动最多 180 秒，均按墙时计；退出会取消订阅。没有运动时不输出动态延迟结论。

## 结果含义

- `native_ros_finite_ranges_max_difference` 比较同一时间戳逐束测距；非有限值的 NaN/+inf/−inf 分布单独比较。JSON 将非有限原始量编码成字符串，保留类型并避免非法 JSON 数字。
- 新实时采集保留 ROS `range_min`、`range_max`、`intensities`，以及角度、时间字段和原始 ranges，便于后续离线重建 LaserScan。旧 run5b 缺少的 ROS 量程/强度字段原样保持缺失，回放不会借用原生值伪造补齐。
- 相同时间戳的 ROS odom→scan frame TF 与 native model pose 组合的传感器外参比较。Gazebo `LaserScan.world_pose` 在本机保持传感器初始/局部 pose，**不将该字段直接当作动态世界真值**。
- 用相邻模型样本的平移线性插值与四元数 SLERP 得到扫描时刻位姿；最大允许括号间隔为 60 ms。另报最近样本时间偏差和“插值 vs 最近样本”位置差，避免把采样间隔误记为传感器延迟。不能插值的帧会被明确跳过。
- 仅在原生转速超过 0.05 rad/s 的帧比较延迟候选。默认包含 −100、−50、−20、−5、−1、0、1、5、20、50、100 ms；可用 `--lags-ms=-20,-5,0,5,20` 修改。候选值表示给定 scan 使用 `stamp + offset` 的模型位姿，均是离线计算。
- 保留全部有效射线的残差；另外把相邻预测量程相差超过 0.1 m 的两束标为边缘，给出补充非边缘统计。这不会删除原始数据或替代全量结果。

`status=complete` / 退出码 0 只表示分析完成且有对齐帧，**不表示误差合格或 SLAM 成功**。`unsupported`、`capture_error`、`insufficient_data` 非零退出。应查看帧数、运动帧数、跳过原因、来源限制和残差；采集丢帧或没有覆盖运动时不能得出时序正确的结论。

## 2026-10-28 回放结果

在不连接 ROS/Gazebo、没有新运动的情况下，用 NumPy 回放 run5b，输出 `outputs/ros_smoke/scan_alignment_replay_run5b.json`：

| 指标 | 结果 |
|---|---:|
| 同时间戳对齐帧 / 运动帧 | 579 / 137 |
| 运动射线数 | 98,640 |
| 0 ms 全量 RMS | 0.01298033 m |
| −1 / +1 ms RMS | 0.01432998 / 0.01463706 m |
| −20 / +20 ms RMS | 0.04084445 / 0.04243790 m |
| 非边缘射线数 / RMS | 96,978 / 0.01017863 m |
| world/odom 传感器 yaw 最大差 | 0.00003562 rad |
| 最近模型样本时间偏差最大值 | 0.006 s |

0 ms 在这些候选中残差最小，非边缘误差与配置的 0.01 m 激光噪声相符。原始全量最大残差 0.863 m 仍保留，没有把边缘异常藏掉。这些证据不支持明显的桥接、外参或运动时间偏移解释；旧 SLAM 地图仍可能失真，必须用独立的新地图实验验证。

## 仓库内离线回归

```bash
python3 tools/test_scan_alignment.py -v
```

2026-10-28 在目标 WSL Ubuntu 24.04 环境实际运行 **11 项全部通过**。测试只使用临时目录中的最小合成 SDF 与帧，不依赖 vendor、未跟踪 outputs、ROS 或 Gazebo，也不启动实时连接。覆盖有效盒状场景、relative_to/动态/非盒状/visual-collision不一致/自定义可见性拒绝、缺少 map 时保留 odom、非零传感器局部旋转、连续回放不升级 legacy 来源且不补造缺失字段、哈希不一致拒绝及输出禁止覆盖。缺少 NumPy 时明确跳过这 11 项，不把跳过显示为实际通过。
