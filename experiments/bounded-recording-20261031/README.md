# 有界录制验证

18 项纯标准库测试通过，包含真实隔离 dummy 进程的 flush、信号升级、子进程清理和外部进程保留。测试不需要 ROS。源码与测试摘要、六份原始报告/日志的 SHA256 见 `manifest.json`。

另在 WSL Jazzy / domain 76 / LOCALHOST 执行真实 `ros2 bag record`，输入仅为合成 `/clock`：15 秒 wall 上限开始关闭，约 0.203 秒完成清理，录制器退出 0，未使用 SIGTERM/SIGKILL，组内无剩余存活成员。metadata 声明 223 条时钟消息。没有启动 Gazebo、导航或运动指令。

首次 smoke helper 的预检没有给自建 ROS Context 配显式 executor，启动 recorder 前即失败；保留 `preflight-failure.json`。修正 helper 后使用新输出目录，不覆盖首轮目录。

这验证录制生命周期及关闭行为，不是 MCAP 全内容/无丢包审计，也不验证机器人停车、任务成功或 GUI。原始 MCAP 与完整本地测试文件位于忽略目录 `outputs/ros_smoke/bounded-recording-20261031/`。
