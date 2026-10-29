# 已录制激光的隔离 SLAM 回放

`scripts/replay_slam_diagnostic.py` 将保存的 **ROS LaserScan 和同时间戳的 odom→laser TF** 送入新启动的真实 `slam_toolbox` 节点，保留 ROS 前端筛选、Karto 匹配和 YAML 指定的求解器。它用于快速比较单个参数，避免每次重新运行完整 Gazebo 场景。

脚本只在 `ROS_DOMAIN_ID=75`、`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` 下运行，并拒绝发现已有节点的域。它不会启动 Gazebo、Nav2 或任务节点，不创建速度或 action 发布器，也不会停止其他域的节点。它会发布回放用的 `/scan`、`/tf`、`/tf_static`、`/clock`；这些 ROS 数据并不是新的传感器测量。

## 输入与边界

- `--input` 支持旧采集格式 `ros_scans + aligned[].tf.odom_to_laser_link`，以及新版 `check_scan_alignment.py` 的 `raw.ros_scans + raw.transforms[stamp].odom`。原生 Gazebo 激光、模型位姿和世界真值字段不参与回放，不会向 SLAM 发布真值。
- `--runtime-evidence` 使用已通过接口检查的 JSON：`scan.range_min/range_max` 和 `tf_chain["base_link->laser_link"]`。旧 run5b 的 ROS 扫描缺少量程上下限，脚本明确记录补充字段的计数、数值、来源和文件 SHA256；已有量程与证据冲突时拒绝。文件哈希只能标识文件，不能追认采集来源或证明它与另一次运行一致，调用者须选择同一次采集对应的接口证据。
- 当前只支持 `laser_link` 的瞬时单层扫描，`time_increment=0`，正角增量，与数量一致的角度范围，以及已测得旋转为单位阵的 `base_link→laser_link` 外参。位移按完整 odom 四元数旋转后扣除；不把激光位置直接冒充车体位置。范围中的 NaN/Inf 保持原观测语义；intensities 若已记录则保留，未记录时使用空可选数组并报告缺失计数，不能声称还原了未采集的强度数据。
- 动态 TF 必须与扫描时间戳完全一致。缺失配对的数量会记录；被选片段中相邻配对不得间隔超过 0.5 秒。输入文件限制 128 MiB。
- 根据测得 odom 角速度超过 0.05 rad/s 的时间范围，自动保留运动前后各约 1 秒；最多 600 帧、40 秒。`--segment first_half/second_half` 按同一运动区间中点分割，分别使用新 SLAM 图。它们是不同初始朝向与历史的诊断，仍然复用同一噪声记录。

## 运行

从仓库根目录在已安装 ROS Jazzy 的 WSL 中运行。`scripts/env.sh` 默认使用实验域 73，因此必须随后显式切换到 75。运行会等待新 SLAM 的生命周期进入 active，然后按原始时间间隔 1×发布数据；启动等待最多 20 秒，整个运行具有 90 秒墙时限，并另留有界进程回收时间。

```bash
source scripts/env.sh
export ROS_DOMAIN_ID=75
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# 当前配置：仓库默认相关核已为 0.01 m；完整片段。
python3 scripts/replay_slam_diagnostic.py \
  --input outputs/ros_smoke/scan_motion_run5b.json \
  --runtime-evidence outputs/ros_smoke/sim_runtime_run5.json \
  --params src/limo_maze_bringup/config/slam.yaml \
  --output-dir outputs/slam_replay/baseline_new

# 仅把相关核改回旧值 0.10 m；仍回放相同完整片段。
python3 scripts/replay_slam_diagnostic.py \
  --input outputs/ros_smoke/scan_motion_run5b.json \
  --runtime-evidence outputs/ros_smoke/sim_runtime_run5.json \
  --params src/limo_maze_bringup/config/slam.yaml \
  --override correlation_search_space_smear_deviation=0.10 \
  --output-dir outputs/slam_replay/old_smear010_full_new
```

每次仅允许一个保留原类型的标量 `--override`，不支持参数扫描；基线 YAML 本身可以另存为明确的实验配置。时钟、固定 TF 帧和扫描 topic 不能通过覆盖改变。输出目录必须不存在，脚本拒绝覆盖旧证据。

以上两个示例都是 full/full 对照。需要独立初始朝向诊断时，在相同配置命令上另加 `--segment first_half` 或 `--segment second_half`，每次指定新输出目录；对比参数应使用相同 half。整段与半段的建图历史和接纳时间集合不同，不能直接把它们的终值当作同输入参数对照。默认配置可能随项目进展变化，应以每份报告记录的参数和 YAML SHA256 为准。

## 输出和失败行为

`result.json` 保存输入路径及 SHA256、补充字段来源、选取与实际接纳的扫描时间戳、发布计数、SLAM 位姿、map→odom 序列及末值、完整栅格地图、使用参数和清理证据。`params.yaml` 是本次实际加载的配置，`slam.log` 是所拥有启动进程的日志。地图数据位于 `result.json.map`：行优先 `data`、尺寸、分辨率、原点 xyz/quaternion；绘图和真值评分应在回放之外进行。

为区分系统包与补丁 overlay，报告额外记录 `ros2 pkg prefix slam_toolbox`、所选 async executable 路径与 SHA256，并在生命周期 active 后读取**所拥有的 async 进程** `/proc/<pid>/exe` 和 `/proc/<pid>/maps`。`libtoolbox_common`、`libkarto` 等映射条目保留原始路径、inode/device 证据，并在当前文件身份一致时计算 SHA256。`mapped_files_recorded` 表示已找到所需映射并取得对应文件哈希；它不是内存代码快照。只找到安装路径、权限不足、库被替换/删除或所需映射缺失时，明确记录 `partial/unavailable`，不能拿所选安装前缀冒充实际加载证明。

脚本只回收自己通过 `start_new_session` 创建的进程组。即使启动器已经退出，只要子进程仍在该组内，仍继续 SIGINT→SIGTERM→SIGKILL 的有界清理；输出记录信号、启动器退出码、组内残留 PID 和 `confirmed_empty`。不能确认回收时标记 `cleanup_failed`。普通异常与 KeyboardInterrupt 会保留失败 JSON，并先回收所拥有进程；第二次 Ctrl-C 不会打断清理。输出路径本身不可写或已存在时无法在其中保存新报告，会直接拒绝。

`status=complete` 只说明回放与输出收集完成，不代表建图准确、导航可用或任务成功。比较参数时必须检查 **接纳时间戳相同**，再比较首次、最大、最终位姿修正和地图形状。原型使用 run5b 记录表明：禁止近邻链/回环后的结果与基线相同，缩小相关核能明显减轻纯旋转漂移；这只是定位连续匹配敏感性的证据，不是实车标定或完整场景验收。启动姿态、前端筛选历史和 ROS 调度也可能与原采集不同。

纯输入、来源、隔离和进程回收边界测试不需要 ROS/Gazebo，也不依赖 `outputs/`：

```bash
python3 -m unittest discover -s tools -p 'test_slam_replay.py' -v
```
