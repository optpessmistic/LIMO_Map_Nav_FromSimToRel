# run18：运动前发现重复里程计

代码基线 `5d3229f006557197deb6516365d8025452992f67`，map epoch `6816e273-0c13-4a8c-ac7f-8563b578ce14`。运行使用原 `ros_gz_sim create`；后续单次创建修复不属于本次输入。

传感器检查在 41.06 wall 秒通过，Nav2 服务及候选参数加载通过。但初始环视在 preflight 阶段报 `Pose changed at duplicate odometry timestamp`，没有发送 Spin goal，`motion_started=false`。mission 没有启动；不能用本轮评价 Nav2 3 cm 到达容差的实际导航效果。

闭合录包内 690 条 odom 对应 345 个整数纳秒时间戳，每个时间戳恰好两条不同位姿，平面差约 1.34042e-7 m；同戳 TF 也包含两组相同数值分支。这不是浮点时间戳混叠。`sim.log` 有两次 DiffDrive 初始化，但未记录原生模型数量，尚不能仅凭这些记录最终断定重复模型的产生机制。原生 motion 采集器中断后未保存文件，不能用 odom 自证真实运动。

`orchestration.json` 原始清理检查因读取一个退出中的进程身份不确定而报告 false；稍后的独立只读 `post-shutdown-recheck.json` 确认全部九个外层进程组及内层录包组已清空。两个报告同时保留，不改写原始失败。

14 份历史文件按字节归档，另附修复后离线生成的 `new-guard-replay.json`；执行 `sha256sum -c SHA256SUMS` 核对全部 15 份。新 guard 对全部 690 条历史样本报告 345 组 pose/twist 冲突、结果 false，未改写原始数据。完整原始 bag 留在本机 `outputs/ros_smoke/run18-arrival-margin/recording/bag`，不入 Git。bag 有 6445 条存储消息、跨度 3.642 sim 秒，传输层累计报告丢失 88 条，不能称无损。这里只保存 metadata 和 odom 审计摘要，不能代替完整录包复验。

新启动方案及后续实测见[单次模型生成诊断](../../docs/single-spawn-diagnosis.md)。本轮失败不构成几何探索、视觉覆盖、返航或实车成功。
