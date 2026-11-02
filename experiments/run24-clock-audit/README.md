# run24 原始时钟离线核查

此目录是 2026-11-01 对已关闭 run24 原始 MCAP 的独立只读分析，不修改 run24 原始档案或结果。

- `clock-audit.json`：各时钟/TF/传感器时间戳计数、单调性和故障附近窗口，以及四个关键节点的实际仿真时间参数。
- `audit-source.py`：运行时脚本原样副本。脚本没有启动节点、回放、发布话题或控制运动；路径按原 `outputs/setup` 位置解释。
- `files.json`：上述两个文件的大小和 SHA256。

原 bag 位于本地 `outputs/ros_smoke/run24-startup-gate/recording/bag/bag_0.mcap`；SHA256 为 `a4afa08357a7cfd8b027ee5b7dae08f4afb0cce2afc8cc9a9f1543a6985d531c`。录包数据时间为记录器的 ROS 仿真时钟，不是独立墙钟。没有倒退或数量级混用的证据，但不能由此证明每个订阅者当时都已收到相同 TF。

分析和参数实验说明见 [TF 时钟记录](../../docs/tf-timing-experiment.md)。
