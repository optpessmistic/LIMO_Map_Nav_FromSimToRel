# 仿真冷启动：run16 失败与 run17 传感器复查

2026-10-31，在 WSL Ubuntu 24.04 / Jazzy / Gazebo Harmonic 8.15.0 中，原 30 秒等待不足以覆盖本次测得的冷启动时间。新检查默认上限为 120 秒，收到完整数据即可提前结束；没有放宽消息、同步或 TF 判据。可视化操作见[Gazebo / RViz 窗口](visualization.md)。

两轮均使用 `smoke_maze.world`，SHA256 `1ef2017583d448e36fb35825bbef6735536c2fe3cbf3a7a57380cf365c708811`。运行时主仓库 HEAD 为 `2c7c45aa24c7a704783007be5b25a68d04391435`；本轮 runner 位于忽略目录，已另存原始副本和输入摘要。完整仿真包来自 F 盘构建目录，任务/感知 overlay 来自 C 盘 checkout；没有重新构建或更换模型。

## 失败没有进入导航

run16 计划单独将 Nav2 `general_goal_checker.xy_goal_tolerance` 从 .05 m 改为 .03 m，mission 的 .05 m 验收条件保持。实际只进入仿真启动和传感器检查：30.03 wall 秒内 `/clock`、`/imu` 各收到两条，其余主要数据为零。九路桥接发布者都存在，但仿真时间没有持续推进，说明话题存在本身不构成就绪证据。

未启动导航、录包、感知、环视或 mission；不能根据本次失败判断 3 cm 配置的效果。runner 共运行 58.60 wall 秒后退出 1，两个自有进程组均清空。launch 内部对 Gazebo 升级至 SIGKILL，外层 `forced_kill=false` 只反映外层未升级，不代表 Gazebo 正常退出。

## 单独验证传感器启动

run17 只启动 Gazebo、模型状态发布者、桥接、模型生成和只读探针。保留 `headless:=true headless_rendering:=false`，实际 Ogre 日志为 GLX、Mesa llvmpipe、OpenGL 4.5。先将相同探针显式设置 `--timeout 120 --max-initial-sim-time 30`；通过后间隔 10 wall 秒，以独立输出再检查一次。

| 检查 | 实测等待 | 首次仿真时刻 | 结果 |
|---|---:|---:|---|
| 冷启动 | 45.32 wall 秒 | .002 sim 秒 | 九路数据、同步 RGB-D 和 TF 全部通过 |
| 热态复查 | .72 wall 秒 | 7.601 sim 秒 | 同一组判据全部通过 |

两次的九路 ROS 发布者均恰好一个，各有至少三条消息、时间戳推进；每次都收到三组同时间戳 RGB-D。检查包含 720 束 / 240° 激光、640×480 图像、RGB8 / 32FC1 编码、内参、TF 链及 RGB 采样时刻的 odom→camera 变换。

整个诊断约 73.41 wall 秒，退出 0，未尝试 EGL。Gazebo 和 bridge 正常退出，RSP 收到 SIGINT 后退出码 -2；所有自有进程组清空，没有 SIGTERM/SIGKILL 升级。不能把这一结果描述为所有组件均正常退出。

## 结论与限制

本次 GLX 路径能在延长等待后正常提供数据，30 秒默认等待在此环境可能误判冷启动。没有证据要求切换 EGL 或强制更换渲染器，因此没有修改渲染环境变量。由于是独立新运行，不能证明原 run16 再等一定能恢复，也不能保证每次初始化都在 120 秒内完成。

将检查脚本默认等待改为 120 秒，仍支持显式覆盖；外层自动化的 wall budget 也应覆盖启动及检查时间。GUI 显示尚未做像素验收。下一轮自主实验须使用新目录和 epoch；视觉覆盖、正式物品识别、跨迷宫重复性和实车验收仍未完成。

原始精简证据与 SHA256 清单见[实验存档](../experiments/startup-20261031/README.md)。渲染日志位置和 OpenGL 排查方法参考 [Gazebo Harmonic 官方说明](https://gazebosim.org/docs/harmonic/troubleshooting/)。
