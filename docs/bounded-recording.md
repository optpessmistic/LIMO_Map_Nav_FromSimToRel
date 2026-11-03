# 仿真录包的限时关闭

`scripts/record_bounded.py` 管理它自己启动的一个 ROS2 bag recorder。它持续检查实际经过时间、本轮任务终态和磁盘容量，在触发条件后停止录制并保存关闭报告。它不控制机器人，不停止任务、Nav2、Gazebo 或 RViz。

run15 的独立录制器在任务终态后继续运行，最终留下约 17.2 GB 记录。ROS2 的 `--max-bag-size` 和 `--max-bag-duration` 只负责**分卷**，不能限制整个录制任务的生命周期。后续实验应使用本入口，不直接留下无人监管的 `ros2 bag record`。

## 启动

在 WSL Ubuntu-24.04 的项目根目录执行，先加载本轮使用的 ROS 环境。建图启动后、任务启动前，在独立终端运行；`--mission-dir` 必须与任务的 `output_dir` 相同。每轮使用新目录，已有 `mission_*.json` 或已有录制输出都会被拒绝。

```bash
(
set -eo pipefail
source scripts/env.sh
mkdir -p outputs/ros_smoke/rpp_trial_01/mission
# 查询当前建图会话，不自行编造或复用上轮 epoch。
record_epoch=$(timeout --signal=TERM --kill-after=2s 10s \
  ros2 param get --hide-type --no-daemon --timeout 5 /limo_map_session epoch)
test -n "$record_epoch"

/usr/bin/python3 scripts/record_bounded.py \
  --output outputs/ros_smoke/rpp_trial_01/recording \
  --mission-dir outputs/ros_smoke/rpp_trial_01/mission \
  --map-epoch "$record_epoch" \
  --max-wall-seconds 1800 \
  --post-terminal-wall-seconds 90 \
  --max-bytes 8589934592 --min-free-bytes 4294967296 \
  --max-bag-size 268435456 \
  --qos-profile-overrides-path config/recording_qos.yaml \
  --topics /clock /scan /tf /tf_static /odom /map /plan \
    /cmd_vel_nav /cmd_vel_smoothed /cmd_vel /rosout \
    /mission/map_epoch /mission/status /mission/targets \
    /mission/target_observations /mission/frame_processing \
    /camera/color/image_raw /camera/depth/image_raw \
    /camera/color/camera_info /camera/depth/camera_info
)
```

请完整复制包括外层括号的代码，参数查询失败会退出这个子 shell，不会继续启动 recorder。新脚本位于 C 盘 checkout，旧 F 盘 checkout 尚未包含它；本机请先进入 `/mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel` 再运行。录制器只需当前 checkout 的 ROS 环境，模型由已运行的仿真提供。输出路径按执行时的目录解析。

本机的任务与录制应使用同一 C 盘结果目录。若任务终端仍位于 F 盘，任务的 `output_dir` 须显式设为 `/mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel/outputs/ros_smoke/rpp_trial_01/mission`；两个 checkout 中同名的相对路径并不是同一个目录。看错目录时无法检测终态，但录制的总体 wall 上限仍会生效。

这个主题列表是任务、图像与地图证据的基本集合；导航算法诊断可能需要额外主题。原生 Gazebo 位姿仍由[观察器](mission-observer.md)独立采集，ROS bag 不替代物理轨迹证据。录制器不发布运动指令，节点名称也不含 `mission` 或 `explor`，不会因名称触发初始环视的协作检查。

## 停止条件与结果含义

- 最多录制 1800 wall 秒后开始关闭，不依赖 `/clock` 是否推进。
- 首次读到本轮 epoch 的完整 `mission_*.json`，且 `status` 为 `done` 或 `blocked` 后，最多再留 90 wall 秒；重复读取或新增终态文件不会延长这个窗口。总体 1800 秒上限仍然有效。
- 输出达到 8 GiB 或剩余空间低于 4 GiB 时提前关闭。这是轮询软阈值，检查间隔和缓存冲刷会使实际大小超出阈值，不能当作硬磁盘配额。
- 按 Ctrl-C 同样进入有界关闭：先给 SIGINT 最多 10 秒冲刷记录，必要时给 SIGTERM 3 秒、SIGKILL 2 秒的检查窗口。关闭耗时独立于录制时间预算；调度或 I/O 延迟还会增加墙时。检查报告是否使用了强制终止、是否仍有进程或错误。

输出包括 `bag/`、`recorder.log` 和 `supervisor.json`。`metadata.yaml` 存在只表示发现了 metadata，不保证内容完整或无丢包；日志、实际消息和采样证据仍需检查。任务终态也不等于 `mission_success=true`，应读取任务结果中的各验收字段。

90 wall 秒不是 30 sim 秒静止证明。终态后的原生静止窗口、任务取消与停车审计、地图及位姿图保存仍按[实验结束步骤](rpp-smoke-reproduction.md#结束和解释)独立执行。录包已停止也不能据此认定机器人已停。

监管只覆盖本脚本创建并核验所有权的 recorder 进程组。脚本必须保持运行；WSL 被关闭、监督进程遭强制杀死、进程脱离其会话或系统故障不在这项 Python 监管的保证范围内。脚本遇到无法确认所有权的进程时拒绝发信号，并在报告中留下失败，不会扩大清理范围去关闭其他实验。
