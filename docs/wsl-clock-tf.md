# WSL 系统时钟跳变与 TF 发布停顿

2026-11-03，对 run29 故障开展只读诊断。ROS 关键节点的 `use_sim_time=true` 与消息时间戳无倒退，不能排除节点内部另用系统时钟调度的线程受到校时影响。

## 已观察到的证据

* 独立 Python 探针在未运行 Gazebo 时，每 0.1 秒用单调时钟前后包围一次系统时间读取。65.052 秒内观测到系统钟相对单调钟的三组跳变：`+1.659/-1.830`、`+1.497/-1.336`、`+1.456/-1.451` 秒。采样包围间隔最大 87 微秒，单调时钟正常推进。这确认本机 WSL 系统钟不连续；未判定是何种服务或宿主机操作触发。
* run29 原始 bag 中，11 次 RPP 未来外推错误都对应 `map→odom` 接收中断，缺口为 0.560–0.793 ROS 秒。同期 `/clock` 最大间隔仅 0.005–0.008 秒，`/odom` 为 0.011–0.016 秒。此局部对照支持 TF 发布或传输链路停顿，不是所有录包流一起停止；不能从 bag 独自定位具体线程。
* 该版本 SLAM Toolbox 的 TF 发布循环使用 `rclcpp::Rate` 默认系统时钟。Jazzy 的 `WallRate` 明确使用 `RCL_STEADY_TIME`。系统钟在休眠期间向后跳，会延长等到系统时刻的等待。

来源：[SLAM Toolbox 2.8.5 发布循环](https://github.com/SteveMacenski/slam_toolbox/blob/2.8.5/src/slam_toolbox_common.cpp)、[Rate 默认时钟](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/include/rclcpp/rate.hpp#L131-L137)、[WallRate 实现](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/src/rclcpp/rate.cpp#L92-L97)、[Clock 系统钟等待](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp/src/rclcpp/clock.cpp#L129-L138)。

## 最小修复与验证范围

候选补丁只把 TF 发布循环的 `Rate` 换成 `WallRate`。周期仍为 0.02 秒；`restamp_tf=false`，TF 时间戳仍取扫描 ROS 时间加 0.2 秒；Nav2 和任务容差保持原值。它不改变系统时间，不给旧校正重新标注当前时间，也不修改扫描匹配。

真实 Jazzy rclcpp 28.1.22 的独立双线程对照已完成：两线程各以 50 Hz 调用实际 `Rate`/`WallRate`，以单调时钟测量 65.002 秒。程序仅初始化 context，未创建 ROS Node、未更改系统时间。`Rate` 的 3039 次休眠中两次超过 0.1 秒，分别为 1.918 秒、2.356 秒，对应系统钟相对偏移回跳 1.884 秒、2.338 秒；`WallRate` 的 3250 次休眠中没有超过 0.1 秒，最大 0.021718 秒。同一时段两线程均观测到系统钟变化，支持系统钟等待被校时延长而单调钟等待保持正常的解释。原始 CSV 包含调度耗时，不能把每一微秒都归因于时钟。

原始采样、C++ 对照源程序、日志、CSV 和 bag 离线审计见 [时钟证据包](../experiments/wsl-clock-tf-20261103/README.md)。源码补丁与获取流程见 [patches](../patches/README.md)。

新 SLAM 正在独立 overlay 中构建，随后进行 Gazebo 对照。旧 overlay 和 run29 原始证据保留。只有实际加载新库、参数读回和任务运行结果齐备，才能评价 TF 故障是否消失；休眠对照本身不等同于完整任务通过。
