# 2026-10-31 仿真传感器启动证据

结论与限制见[启动诊断](../../docs/simulation-startup-diagnosis.md)。运行基线为 `2c7c45aa24c7a704783007be5b25a68d04391435`；本存档随后加入版本管理，不能把存档提交当成本次运行代码。

- `run16/`：30 秒预检失败的原始报告、Gazebo/Ogre 日志、候选配置与 runner 输入。没有启动导航或运动；3 cm 候选配置尚未实际验证。Gazebo 经 launch 内部 SIGKILL 后退出，两个自有进程组已清空。
- `run17/`：相同 GLX/llvmpipe 路径，显式 120 秒冷态检查在 45.32 秒通过，独立热态检查 .72 秒通过。无导航或运动，全部自有进程组已退出；没有尝试 EGL。日志保留 RSP 的 SIGINT / -2 退出结果。
- `runner.py.txt`、`runner-env.sh.txt` 是实际一次性运行器的原始文本副本，仅供审计；含当时机器路径，不能当作通用启动脚本运行。使用仓库[可视化说明](../../docs/visualization.md)或[实验复现步骤](../../docs/rpp-smoke-reproduction.md)启动新会话。

20 份原始文件按字节复制，`SHA256SUMS` 记录摘要，`.gitattributes` 对原始扩展名禁用换行转换。可在本目录执行 `sha256sum -c SHA256SUMS`。本目录保留关键失败/成功证据，不包含所有 ROS 日志、安装树或依赖。

冷启动通过不追认先前失败，也不证明 GUI 像素显示、完整探索、视觉搜索覆盖或实车实验已通过。检查器随后仅将默认 `--timeout` 从 30 改为 120 秒，完整消息/TF 判据保持；下一轮 runner 必须同时增加外层等待时间，并使用新输出目录和 epoch。
