# 保留停车观察实际采用的原图

`require_processed_inspection` 确认停车观察时处理过新 RGB-D 帧；新增的 `require_retained_inspection` 进一步要求这些**实际被接受的帧**取得源端存档回执。两项默认关闭。识别结果仍通过原接口发布，存档协议独立于红色替身算法。

感知节点在处理该组 RGB-D 的同一个回调中缓存原始 `Image.data`，包括行 padding。任务节点冻结实际接受的帧身份、原纳秒时间戳、该时刻的 TF、任务编号和观察窗口，再发送存档请求。一次窗口接受了几帧，就保留几帧；不能事后挑选其他图替换缺失帧。文件写入由一个受控子进程执行，任务节点等待匹配的文件回执及检查点；等待期间原有停车、姿态和截止时间检查仍生效。

## 启用

先按 [仿真复现说明](rpp-smoke-reproduction.md) 启动仿真、建图及导航，再在感知命令中添加：

```bash
ros2 run limo_maze_perception red_rgbd_detector --ros-args \
  -p enabled:=true -p depth_registered_to_color:=true \
  -p use_sim_time:=true -p publish_frame_processing:=true \
  -p retention_enabled:=true \
  -p retention_output_dir:="$PWD/outputs/ros_smoke/retained_trial_01/frames" \
  -p retention_max_jobs:=64 -p retention_max_file_bytes:=268435456
```

存档目录必须是本次实验的新目录，存在则拒绝启动。启动一个任务节点，在原任务参数后添加：

```bash
-p require_processed_inspection:=true \
-p require_retained_inspection:=true \
-p retention_max_total_samples:=64 \
-p retention_max_metadata_bytes:=4194304
```

若还要启用建图后的有限观察队列，另加 `-p post_mapping_observation_mode:=bounded_processed_frames`，详见[有限观察说明](bounded-observation.md)。不要另启动第二套感知或任务节点。

这里的 64 次写入、256 MiB 文件及 4 MiB 任务元数据只是实验预算示例；不能保证任意迷宫都够用。默认存档最多 32 次写入、96 MiB 文件，任务默认最多接受 32 帧，每窗最多 32 帧。预算耗尽或超时会明确失败，不会偷偷丢弃接受记录或重置预算。像素缓存默认 80 MiB，另有进程、Python 对象与传输开销，不能视为进程 RSS 上限。

## 实验后核对

先结束任务，再在感知终端按 Ctrl+C，等待打印 `retention_shutdown`。确认 `owned_writer_exit_confirmed` 和 `final_checkpoint_persisted` 均为 true，且无 fault。随后运行：

```bash
/usr/bin/python3 tools/audit_frame_retention.py \
  --consumer outputs/ros_smoke/retained_trial_01/mission/mission_实际时间.json \
  --retention-root outputs/ros_smoke/retained_trial_01/frames
```

`--consumer` 接受任务结果 JSON，或单独保存的 `Consumer.snapshot()`。工具只读文件，重新计算每个固定样本目录中的 `rgb.bin`、`depth.bin`、`producer.json`、`camera_info.json`、`request.json` 和 `manifest.json` 的哈希，并与最终 `checkpoint.json` 核对。输出 `passed`、`accepted_count`、`verified_count` 及每帧错误；失败返回非零退出码。

高频状态里的 `current_window` 指最近建立的留存窗口；下一目标仍在停车阶段时可能还显示上一窗口计数。核对某个目标时，应使用最终结果 `frame_retention.windows` 中的完整 `binding`，不能只按相邻状态消息归属样本。

早先任务 JSON 内的成功回执不能覆盖源端后来出现的身份冲突、写入失败或未闭合检查点。关闭后未收到的失败需要用最终检查点发现。文件审计通过只证明接受清单与保存字节一致；TF 的测量精度、实际停车、完整可见区域覆盖、物品类别和召回率还需要单独验证。有限观察队列和存档成功都不会自动把完整任务成功设为 true。

Gazebo/RViz 的打开方式见[可视化界面](visualization.md)。

## 全分辨率运行的限制

run21 实际接受三组 640×480 RGB-D，最终文件均保存，但第三组持久化检查点回执未在任务窗口截止前到达，任务按原规则停止；这不是一次留存验收通过。严格审计在 consumer fault 前置检查即返回失败，报告中的 `accepted_count=0` 不表示该次任务实际未接受帧，实际清单仍在任务 JSON 中。详见 [run21 与写盘对照](run21-retention-deadline.md)。

写盘 helper 只依赖标准库与共享协议。其启动使用 `python -S -B`，`PYTHONPATH` 仅包含主进程实际加载的共享协议目录，清除继承的 Python 启动覆盖变量；这样避免重复加载 ROS、用户 site 和 `.pth` 启动代码。子进程所有权、原始字节、文件持久化、关闭确认以及 5/10/8 秒期限不变。单次隔离对照不能保证并发仿真负载下按时完成，仍须真实 Gazebo 复验。

随后 [run22](run22-retention-verification.md) 在相同迷宫及期限下完成五个实际停车观察窗口，15/15 原图通过关闭后的严格文件审计，writer 无 fault 且确认退出。这是该轮留存链路的验证，不是完整课程任务验收：地图覆盖不足，PGM/YAML 服务导出失败，终态完整静止尾窗未取得。
