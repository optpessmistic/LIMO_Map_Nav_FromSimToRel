# 单次生成模型与里程计完整性检查

run18 在初始环视的运动前检查中失败：345 个 odom 时间戳各有两条不同位姿，同戳平面差约 0.134 微米。`sim.log` 也出现两次 DiffDrive 初始化。虽然数值很小，同一时刻存在两组机器人状态仍说明数据来源需要排查，不能通过放宽静止门槛掩盖。[原始证据](../experiments/run18/README.md)保留失败和稍后的进程清理复查。

本机 `ros-jazzy-ros-gz-sim` 为 `1.0.24-1noble.20261006.035056`。[对应官方 create.cpp](https://github.com/gazebosim/ros_gz/blob/1.0.24/ros_gz_sim/src/create.cpp#L238-L258)在同步创建请求未于 5 秒内完成时，会循环重发请求。run18 记录到一次等待提示后成功，重复驱动输出与这种风险一致；由于旧运行没有原生模型 ID 快照，**不能最终证明 run18 的全部因果链**。

## 启动流程修改

`sim.launch.py` 改用本包的 `spawn_once.py`，先发现唯一且类型匹配的创建服务，再把已展开 URDF、位姿、参数及请求意图留档，至多调用一次创建请求。发现最多等待 120 wall 秒，回复最多等待 30 wall 秒，总体合作式预算 150 wall 秒。不确定结果失败退出，不自动重发。

launch 在生成器退出非零时关闭本次仿真；另有独立 180 wall 秒定时器开始关闭未完成的生成流程。Shutdown 后进程还需要信号处理/升级时间，所以 180 秒不是所有进程已经消失的保证。启动前检查已安装 helper 的文件和执行权限，避免缺失程序仅靠定时器回收。

生成材料默认留在 launch 日志目录的 `limo-spawn-*`；可用 `spawn_artifacts:=/绝对路径/全新目录` 指定。`result/attempt-intent.json` 表示可能已发出请求，`report.json` 的正响应只表示创建被确认，不能单独证明模型唯一或允许运动。出现未知结果时结束本次服务器、保留日志，再用新会话排查；不要对原服务器重复创建。

所用 Python transport/message 模块来自本机 `gz_transport_vendor` / `gz_msgs_vendor`，已作为包运行依赖声明。它们依赖加载 Jazzy 环境，参见[安装与启动说明](jazzy-simulation.md)。

## 两层只读核验

`scripts/check_sim_runtime.py` 保留九路传感器、RGB-D 同步、TF 和每路一个 ROS 发布者的要求，并增加：有效 odom 至少 50 个唯一整数纳秒时间戳、odom 和 `/clock` 观察跨度至少 1 sim 秒；拒绝倒退、非有限数据及同时间戳不同 pose/twist。完全相同的重复消息只计数，不增加有效样本或时间跨度。原 `initial_survey.py` 的运动门槛保持。

原生场景身份另查，避免把一个 ROS bridge 发布者当成一个 Gazebo 上游实例：

```bash
# 加载同一仿真环境；输出文件父目录须存在，文件须全新。
ros2 run limo_sim_bringup inspect_scene_once.py \
  --world limo_maze --name limo --output outputs/scene-first.json --execute
```

该工具只查询一次 `/world/limo_maze/scene/info`，检查当前响应中恰好一个顶层 `limo`、正模型 ID、无重复 ID，记录快照中的名称和 ID。间隔一段推进的仿真时间后，用不同输出名再查并比较 ID。它不发速度或导航目标；场景数据仅用于诊断，不输入未知迷宫策略。

快照和有限 odom 样本只能证明各自观察范围，不能替代全程监测、无损录包、真实运动或视觉覆盖验证。

## 验证

单次生成与场景查询的 31 项假 transport 测试通过，包含延迟发现、重复服务、超时、意图写失败、异常、错误回复、旧目录拒绝和多个同名模型等反例。里程计 helper 13 项纯测试通过，run18 同量级差异明确拒绝。完整离线回归：核心 217 通过/14 跳过，工具 135 通过。ROS 专用跳过项不计在线通过。

run19 的 sensor-only 复验已通过：新生成器实际发送一次请求并收到正响应；冷态 38.59 wall 秒、热态 1.94 wall 秒通过，各有 101 个唯一 odom / 1 sim 秒，无重复或冲突。两个原生场景快照均只含一个顶层 LIMO、ID 同为 114；日志只有一次 DiffDrive 初始化。全部自有进程组退出，没有 TERM/KILL 升级；RSP 的 SIGINT / -2 退出保留。精简原始材料见[run19 存档](../experiments/run19/README.md)。

这支持单次生成方案在该新运行中消除了重复输出现象，但不追认 run18，也不证明每次冷启动、全程导航或视觉覆盖均已通过。下一轮自主实验仍须使用新目录和 epoch，重新检查启动、环视、到达观察和返航。
