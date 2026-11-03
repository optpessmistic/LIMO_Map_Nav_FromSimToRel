# 有界观察点调整：离线复算与 ROS 验证

2026-11-03。run31 的 12 个计划视图只完成 5 个，其余 7 个在更新地图上未通过原终点安全检查。本次增加可选的 `observation_relocation_radius_m`：默认 0 保持原行为，实验值 0.15 m 允许尚未派发的动作在原站附近选择安全格心。原动作、朝向、机器人半径、到达余量、次数和阶段截止保持不变；不会重写正在执行的目标。规则和启用方式见[有界观察说明](../../docs/bounded-observation.md)。

## 已取得的结果

| 验证 | 结果 | 适用范围 |
|---|---|---|
| 固定 run31 原始地图和位姿复算 | 默认复现 7 项跳过；0.15 m 时可选择 7 个假想目标 | 只证明选择可行，没有新增实际成功 |
| Linux 观察相关完整测试 | 103 项中 102 通过、1 失败；其中 99 纯测试及 3 个旧 DDS 用例通过 | 保留新 DDS 用例首次失败 |
| 新 DDS 用例单独复测 | 1/1 通过；两个不同导航 UUID，各接受 3 帧 | 受控 ROS Action/DDS，未运行 Nav2 或 Gazebo |
| 任务包构建与安装核对 | colcon 成功；新环境导入三个模块，字节哈希与源码一致 | 初始验证脚本的导入失败仍保留 |

复算中两个站分别移动约 4.25 cm 和 5.75 cm，保留原 0.25 m 机器人半径、0.05 m 到达余量及半格对角线余量。候选必须与原 anchor 在指定半径内通过安全格四邻域相连；不能跨墙或逐次向外漂移。复算对每个假想目标立即注入合成 `canceled` 以继续枚举，因此最终仍是 **5 个原成功、7 个合成失败**。这不是导航失败统计，也不是 7 个新观察成功。

ROS 单例在第一个目标执行中更新地图，原执行目标保持不变；第二个朝向派发到距离原站约 0.10 m 的新位置。ActionServer 收到的目标、节点实际目标和 `inspection.expected_pose` 一致，原计划 XY 不变。两个窗口有不同 token、UUID 和请求 ID，各接受 3 帧，跨度均为 0.2 ROS 秒，首帧均晚于本窗口的采样门槛。`deadline_unchanged` 指测试逐次核对的 **observation stage deadline**，不是对所有计时量的独立测量。

## 保留的失败和限制

首次完整测试在第二窗口等待时出现 `/clock` 投递停顿和里程计过期，新 DDS 用例失败。随后使用同一生产源码、测试源码和新鲜度/停车阈值进行单例复测，日志先写 Linux `/tmp`，进程结束后再复制回 Windows。外层测试进程超时也从 180 秒改为 90 秒，生产期限未改。**单次复测通过不能确定投递停顿的根因或证明长期稳定性。**

原 `dds-retest/result.json` 的 `scope` 写成 “log sink changed only”，并不精确：还改变了整套/单例进程隔离与外层超时。原记录按字节保留，`validation.json.receipt_erratum` 和本说明给出勘误，不能据其得出日志 I/O 的单因素因果结论。

Windows 首次测试有 5 个旧用例在临时目录清理时遇到 `WinError 5`，另 4 个 ROS 用例跳过；未计为通过。首次 WSL 包装脚本在 `set -u` 下加载 ROS 环境时失败，测试尚未开始，后改为加载环境后启用该选项。构建后，初始父进程环境导入包失败；colcon 自身成功，重新启动 shell 加载安装环境后的导入及哈希核对通过。摘要、原始日志及实际脚本均保留。

本归档没有新 Gazebo 运动、物理停车、完整视觉覆盖或实车证据。仍须在新闭合仿真中验证调整后的导航、停稳、RGB-D 留存及返航；原 run31 的 `visual_coverage_complete=false`、`mission_success=false` 不变。

## 文件与复算

- `replay/`：生产队列复算及几何探查的脚本字节副本、JSON 和说明；生产复算是主要结果。
- `dds-first/`、`dds-retest/`：首次完整测试和单例复测的原日志与回执。
- `build/`：colcon 日志及新环境安装核对，保留原包装脚本失败。
- `sources/`：相关生产和测试源码原字节副本；`.txt` 后缀避免误执行。
- `runners/`：实际使用的包装脚本，保留当时的本机路径及固定输出目录；复跑须改为新的输出位置。
- `validation.json`：计数、源码绑定、复测条件、勘误和适用范围。
- `SHA256SUMS`：本目录除清单自身以外各文件的原字节哈希。

生产队列复算不依赖 ROS，也不读取目标真值。需要本仓库已归档的 run31 地图/任务输入及本次生产源码。WSL 在仓库根目录执行：

```bash
mkdir -p outputs/setup
cp experiments/observation-relocation/replay/run31-relocation-replay.py.txt outputs/setup/replay-relocation.py
/usr/bin/python3 -B outputs/setup/replay-relocation.py --output-prefix outputs/setup/relocation-recheck
```

输出前缀必须尚不存在；程序拒绝覆盖旧结果。辅助几何探查另依赖其 JSON 所列的深度缺口报告，不是上述生产复算的前置条件。核对本归档：

```bash
cd experiments/observation-relocation
sha256sum -c SHA256SUMS
```
