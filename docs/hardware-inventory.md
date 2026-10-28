# LIMO 实机环境只读清单

当前尚未确认实际小车的 ROS 版本、相机和雷达型号。运行 `tools/collect_limo_inventory.py` 可以收集最小环境信息；报告说明的是**运行脚本的机器**，不会把开发机、WSL 或带 ROS 的电脑自动认定为 LIMO，也不是驱动就绪或运动安全证明。

在实际小车的 Linux 终端运行，使用系统 Python 3.6 或更高版本，无第三方 Python 依赖：

```bash
python3 tools/collect_limo_inventory.py --output limo-inventory.json
```

文件必须是新路径，父目录须已存在。工具拒绝覆盖；省略 `--output` 时只向 stdout 输出 JSON。命令缺失、权限不足、文件不存在及查询超时分别记录 unknown/error，不把没有结果解释成小车没有该设备。勿将本项目早期 `probe_environment.py` 的默认 ROS graph 行为与本工具混淆。

默认采集内容：

- `/etc/os-release` 的 ID、VERSION_ID、PRETTY_NAME，以及 CPU 架构。
- `/proc/device-tree/model` 型号，不读取设备树 serial-number。
- 仅一个环境变量 `ROS_DISTRO`；未 source ROS 时可能为空。
- `/opt/ros` 下发行版目录名，仅代表安装痕迹。
- Debian 包数据库中 ROS、LIMO、RealSense/Orbbec/Astra/UVC 及常见雷达包的已安装版本。源码工作空间、pip 包和 ROS overlay 版本不在此范围内，不能凭清单保证相机驱动兼容。
- 基本 `lsusb` 的 VID/PID 与厂商/产品标签；不使用 `-v`/`-D` 查询序列号，不访问串口。WSL 未透传 USB、非 USB 相机或雷达都可能不出现在这里。

不查询用户名、主机名、IP、MAC、序列号、完整环境或凭据；不会启动驱动、修改参数、发送运动指令。命令 stderr 不写入报告。为识别 WSL，只检查 kernel release 是否含 microsoft，不保存完整 kernel 字符串；WSL 报告明确标为 `wsl_development_environment`。报告的 `excluded` 指这些身份字段没有被主动查询，不是任意文本的自动脱敏保证：设备产品标签和可选 ROS 名称可能包含用户自己写入的标识。

## 可选 ROS 2 graph

默认不访问 ROS graph。若需要现有驱动节点/话题名，在已按实际设备正常流程配置 ROS 环境的终端显式运行：

```bash
python3 tools/collect_limo_inventory.py --ros-graph --output limo-inventory-with-graph.json
```

工具先分别检查当前 `ros2 node list --help` 与 `ros2 topic list --help`，只有确认支持 `--no-daemon` 和 `--spin-time` 后，才执行：

```bash
ros2 node list --no-daemon --spin-time 1
ros2 topic list --no-daemon --spin-time 1
```

2026-10-28 已用本机 Jazzy CLI help 核准这两项参数；其 help 明确 `--no-daemon` 不启动也不使用已有 daemon。工具不会执行 daemon 子命令，不自动 source 工作空间，也不进行 ROS 1 master 查询。显式 graph 查询会建立短暂 DDS discovery 参与者，返回当前 shell 所在 discovery 范围的节点/话题名；它们可能属于其他设备，空列表也不证明没有驱动。报告保留 ROS 名称，不保留 CLI 的任意诊断行。

每条外部命令最多 4 wall 秒，所有外部命令共享 25 wall 秒预算；超时停止该命令，剩余不足时跳过后续命令。读取固定系统文件和写 JSON 的本地文件操作不属于硬实时超时保证。

## 验证与下一步

```bash
python3 -m unittest discover -s tests -p test_limo_inventory.py -v
```

测试覆盖默认不访问 graph、环境变量白名单、命令缺失与超时、总预算、CLI 安全参数验证、输出拒绝覆盖、USB/包解析及 WSL 标识。WSL 上试跑只能证明开发机可运行工具，不能回答实车型号；需将脚本放到实际小车执行，再结合清单和设备标签确认 ROS/相机/雷达，之后才选择相应 bringup 与检测方案。
