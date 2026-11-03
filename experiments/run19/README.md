# run19：单次模型生成的传感器复验

本轮只启动 Gazebo、RSP、bridge、单次生成器和只读检查，无导航、环视、mission 或速度发布。总计 76.10 wall 秒，runner 退出 0，全部五个自有进程组退出。

本轮使用在 `5d3229f` 之上尚未提交的修复；`inputs.json` 保留实际源码、世界与 runner 摘要，后续归档提交不是运行时 HEAD。构建后的 C 盘 simbringup 已与源码逐字节核对，模型来自 F 盘 underlay。展开模型 SHA256 见 spawn 报告，URDF 本体没有重复入库。

| 证据 | 结果 |
|---|---|
| 创建请求 | 唯一服务；只发 1 次；约 3.64 wall 秒后正响应 |
| 冷态传感器 | 38.59 wall 秒通过；101 个唯一 odom，跨度 1 sim 秒，重复/冲突为 0 |
| 热态传感器 | 1.94 wall 秒通过；101 个唯一 odom，跨度 1 sim 秒，重复/冲突为 0 |
| 原生模型身份 | 两个独立快照均只有 1 个顶层 `limo`，ID 均为 114 |
| 驱动初始化日志 | 仅 1 条 DiffDrive 订阅初始化记录 |
| 关闭 | Gazebo/bridge 退出 0；RSP 收到 SIGINT 后退出 -2；没有 TERM/KILL 升级，全部自有进程组清空 |

两次探针也通过九路传感器、ROS 发布者数、RGB-D 同步、内参、激光和 TF 检查。使用 GLX / llvmpipe，未修改渲染方式。两份约一秒的观测窗和两次场景快照不构成全程无损、永久实例唯一或物理运动证明。本轮没有 GUI 像素验收，也没有验证导航到位、完整探索、视觉覆盖或返航。

16 份文件按字节归档，执行 `sha256sum -c SHA256SUMS` 验证。`runner*.txt` 是包含本机路径的审计副本，不是通用启动入口；启动应使用[可视化说明](../../docs/visualization.md)。设计与限制见[单次生成诊断](../../docs/single-spawn-diagnosis.md)。
