# WSL 仿真速度与本机 GPU 设置

2026-11-11 在复杂迷宫 `complex_maze_trial_02` 的读取结果：Gazebo 累计仿真时间约 221 秒、累计运行时间约 704 秒，即约 0.31 倍实时。瞬时倍率波动较大，不用单条 `real_time_factor` 判断全程速度。Gazebo server、GUI、RViz 的进程平均 CPU 均约 150%（100% 为一个逻辑核）；WSL 可见 20 个逻辑核，不能简单归因为没有分配足够核心。

本轮 `~/.gz/rendering/ogre2.log` 中的 `GL_RENDERER` 为 `llvmpipe (LLVM 20.1.2, 256 bits)`，即软件渲染。WSL 能访问 `/dev/dxg` 和 RTX 3060，并不表示 Gazebo 已使用该 GPU。

## 下一轮优先使用 GPU

结束本轮任务、感知、导航和仿真后，按[复杂迷宫步骤](complex-maze-trial.md)开新会话。五个终端加载完 ROS 环境后，都可执行以下两行；实际需要的是启动 Gazebo 和 RViz 的终端：

```bash
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
```

本机 16×16 离屏 GLX 与 EGL 测试均已验证：默认渲染器为 llvmpipe；只设置适配器名称仍为 llvmpipe；同时设置这两项后均为 `D3D12 (NVIDIA GeForce RTX 3060)`，OpenGL 4.6。探针不启动 ROS 或 Gazebo。此结果证明新上下文可以使用 GPU，尚未证明完整 Gazebo 场景的提速倍数，也不会改变正在运行的进程。

启动新仿真后核对实际 Ogre 日志：

```bash
grep 'GL_RENDERER' ~/.gz/rendering/ogre2.log | tail -1
```

应检查新启动时间对应的记录是否出现 `D3D12 (NVIDIA GeForce RTX 3060)`。如果仍是 llvmpipe，保留该轮日志，不以 `nvidia-smi` 中出现显卡为成功依据。这是本机验证的设置，其他显卡/驱动环境需重新检查。微软的[WSLg GPU 选择说明](https://github.com/microsoft/wslg/wiki/GPU-selection-in-WSLg)解释了适配器名称的选择规则。

## 减少窗口负载

如果主要看地图，可在新一轮终端 1 使用：

```bash
mkdir "$LIMO_TRIAL" &&
ros2 launch limo_sim_bringup sim.launch.py \
  world:="$PWD/worlds/complex_maze.world" \
  headless:=true \
  spawn_artifacts:="$LIMO_TRIAL/spawn"
```

其余终端照原指南，保留 RViz。这里的 `headless` 只关闭 Gazebo GUI，物理仿真、RGB-D 和激光仍运行；本项目的 `headless_rendering` 是另一个 EGL 选项，此处保持默认。Gazebo [server-only 模式](https://gazebosim.org/docs/harmonic/getstarted/)和[传感器离屏渲染](https://gazebosim.org/api/sim/8/headless_rendering.html)是不同设置。

RViz 中可将 `Global Options → Frame Rate` 从当前 20 改为 5–10，并取消不需要的相机图像显示，保留 Map、RobotModel、LaserScan 和目标标记。用户现有 `config/simulation.rviz` 未被本次修改。仅调显示帧率，不降低传感器采样率或导航控制频率。

## 减少跨文件系统写盘

当前图像结果位于 `/mnt/c/...`。后续可将输出放在 WSL 的 Linux 文件系统中；仓库和启动目录可以继续保持原位置。五个终端统一设置一个新的运行名，例如：

```bash
mkdir -p "$HOME/limo_runs"
export LIMO_TRIAL="$HOME/limo_runs/complex_maze_trial_04"
```

再按上述命令启动。若目录已存在，更换名称，保留旧结果。需要查看文件时，在 WSL 执行 `explorer.exe "$LIMO_TRIAL"`。这是微软[跨文件系统性能建议](https://learn.microsoft.com/en-us/windows/wsl/filesystems)对应的优化；本轮尚未做 C 盘与 Linux 目录的写盘对照，不承诺固定加速比例。

## 本轮观察超时的处理

三帧原始数据已写成，但消费者 12 秒总观察窗口先结束，之后才有完整持久化回执。复杂迷宫配置已设 `inspection_timeout_s: 30.0`，下一次启动任务会读取，无需重新构建。3 帧要求、留存确认、TF 和停车检查继续启用；生产者请求和 worker 超时未触发，保持原值。较长观察上限是为慢速运行留余量，不会主动等待满 30 秒，也不会提高仿真速度。

机器人已离开出生点，重试时应完整关闭旧会话并重新生成小车，使用新结果目录，再执行运行检查和初始环视。不要在失败后的当前位置直接复用出生点净空声明重启任务。
