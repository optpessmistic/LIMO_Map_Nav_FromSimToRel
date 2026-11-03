# WSL 系统时钟跳变与 TF 发布停顿

2026-11-03，对 run29 故障开展只读诊断。ROS 关键节点的 `use_sim_time=true` 与消息时间戳无倒退，不能排除节点内部另用系统时钟调度的线程受到校时影响。

## 已观察到的证据

* 独立 Python 探针在未运行 Gazebo 时，每 0.1 秒用单调时钟前后包围一次系统时间读取。65.052 秒内观测到系统钟相对单调钟的三组跳变：`+1.659/-1.830`、`+1.497/-1.336`、`+1.456/-1.451` 秒。采样包围间隔最大 87 微秒，单调时钟正常推进。这确认本机 WSL 系统钟不连续；未判定是何种服务或宿主机操作触发。
* run29 原始 bag 中，11 次 RPP 未来外推错误都对应 `map→odom` 接收中断，缺口为 0.560–0.793 ROS 秒。同期 `/clock` 最大间隔仅 0.005–0.008 秒，`/odom` 为 0.011–0.016 秒。此局部对照支持 TF 发布或传输链路停顿，不是所有录包流一起停止；不能从 bag 独自定位具体线程。
* 该版本 SLAM Toolbox 的 TF 发布循环使用 `rclcpp::Rate` 默认系统时钟。Jazzy 的 `WallRate` 明确使用 `RCL_STEADY_TIME`。系统钟在休眠期间向后跳，会延长等到系统时刻的等待。

来源：[SLAM Toolbox 2.8.5 发布循环](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp)、[Rate 默认时钟](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/include/rclcpp/rate.hpp#L131-L137)、[WallRate 实现](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/src/rclcpp/rate.cpp#L92-L97)、[Clock 系统钟等待](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/src/rclcpp/clock.cpp#L129-L138)。

## 最小修复与验证范围

已实现的补丁只把 TF 发布循环的 `Rate` 换成 `WallRate`。周期仍为 0.02 秒；`restamp_tf=false`，TF 时间戳仍取扫描 ROS 时间加 0.2 秒；Nav2 和任务容差保持原值。它不改变系统时间，不给旧校正重新标注当前时间，也不修改扫描匹配。

真实 Jazzy rclcpp 28.1.22 的独立双线程对照已完成：两线程各以 50 Hz 调用实际 `Rate`/`WallRate`，以单调时钟测量 65.002 秒。程序仅初始化 context，未创建 ROS Node、未更改系统时间。`Rate` 的 3039 次休眠中两次超过 0.1 秒，分别为 1.918 秒、2.356 秒，对应系统钟相对偏移回跳 1.884 秒、2.338 秒；`WallRate` 的 3250 次休眠中没有超过 0.1 秒，最大 0.021718 秒。同一时段两线程均观测到系统钟变化，支持系统钟等待被校时延长而单调钟等待保持正常的解释。原始 CSV 包含调度耗时，不能把每一微秒都归因于时钟。

原始采样、C++ 对照源程序、日志、CSV 和 bag 离线审计见 [时钟证据包](../experiments/wsl-clock-tf-20261103/README.md)。源码补丁与获取流程见 [patches](../patches/README.md)。

完整 SLAM 包已在独立工作区 `/tmp/limo-slam-steady-20261103` 构建成功，安装复制到当前 C 盘仓库的 `outputs/slam-steady-ws/install`。包前缀、依赖解析和 `WallRate` 符号检查通过；Karto 库与旧边界补丁 overlay 的哈希一致。旧 overlay 与 run29 原始证据保留。[构建回执](../experiments/run30/validation/slam-steady-build-receipt.json)记录的是构建和链接检查，任务运行另见下文。

## run30：已完成的一次有限对照

run30 使用运行版本 `a335f4c2104b8421bd8ae249f20229c81f9f8c54` 和新 SLAM overlay，原参数容差及任务期限不变。13 个导航目标中 12 个成功，另一个观察目标因 360 秒观察阶段期限到达而取消。导航终态 `error_code=102` 为 0，导航日志外推匹配行数为 0；这仅描述本轮记录，不证明所有运行都不会再发生 TF 故障。[运行摘要与原始证据](../experiments/run30/README.md)

任务判据 `map_complete=true`，地图内确认了 2 个红色替身目标，`returned_pose_verified=true`；独立原生位姿返航距离约 0.04115 米。完整验收仍未通过：`visual_coverage_complete=false`、`mission_success=false`，正式目标定义仍未确认；严格留存审计仅 33/36 通过，另外 3 项为 `consumer_receipt_not_confirmed`，不能按文件存在补记通过。编排器退出码为 1，保留该审计失败。

闭合 bag 的 `map→odom` 全序列对比如下。间隔以录包器的 **ROS 仿真时间**计算，不是独立单调墙钟到达时间：

| 指标 | run29 | run30 |
|---|---:|---:|
| 相邻接收间隔超过 0.15 秒的次数 | 21 | 1 |
| 最大相邻接收间隔（秒） | 0.793 | 0.475 |
| 两次严格推进 header 时间戳的接收间隔最大值（秒） | 0.937 | 1.725 |
| 最大「录包接收时间 − header 时间」（秒） | 0.550 | 1.550 |

接收中断减少，但时间戳推进的最长间隔及最大接收时间差反而增大。连续收到重复时间戳的 TF，不能证明新扫描或 SLAM 校正持续推进；header 推进本身也不证明完成了新一次优化。这与补丁只改变发布调度、保留扫描时间戳语义相符。两次运行路径和时长不同，bag 也不能区分发布、传输与录包调度，不能把这些全程极值当作匹配负载下的因果证明。[完整指标定义、输入哈希与限制](../experiments/run30/validation/run30-tf-delivery-audit.json)

run30 同期的独立 65 秒时钟探针仍记录到约 1.218 秒的系统钟相对单调钟回跳；这说明本轮零外推错误并非以证明系统钟已修复为前提。探针与 bag 使用不同时间轴，不能将两者数值直接相减定位某次 TF 延迟。当前 GUI 的新 overlay 加载顺序见[可视化说明](visualization.md)。

## run31：继续使用同一补丁与时限

run31 在保持新 SLAM 库、仿真消息时钟、TF 容差和阶段预算的情况下，12 次导航全部成功，TF 过期拒绝及外推日志均为 0。标注两个替身并返航，原生仿真返航距离约 1.33 cm；33/33 留存审计通过。观察只有 5/12 完成，另外 7 项被安全重验跳过，故完整任务仍失败。原进程关闭检查失败和后续只读复查分别保留，见[本轮证据](../experiments/run31/run31-outcome-review.md)。本轮没有再做独立时钟探针或 TF bag 间隔审计，不把零错误扩大为系统钟或全部 TF 新鲜度已经修复。
