# 有界 RGB-D 标定采集

这个工具运行 `tools/build_red_calibration.py` 生成的有限场景，保存实际收到的固定六组 RGB-D、CameraInfo、采集时刻 TF 和当前检测器结果。每个场景使用独立 Gazebo 分区、新输出目录和自己的进程组；不启动导航或自主任务。采集器不读取目标真值、目标数量或场景几何。

先按 [场景生成说明](../../docs/red-surrogate-calibration.md) 生成 manifest。以下在仓库根目录执行，manifest 和输出路径可自行更换：

```bash
# 仅检查场景和输出执行计划，不创建输出或启动 ROS。
/usr/bin/python3 tools/red_calibration/runner.py \
  --manifest outputs/red-calibration/new-fixtures/manifest.json \
  --output outputs/red-calibration/new-blank --case 01_blank

# 实际运行一个空场景；runtime 指向已构建的模型依赖工作区。
bash tools/red_calibration/env.sh \
  /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel \
  --manifest outputs/red-calibration/new-fixtures/manifest.json \
  --output outputs/red-calibration/new-blank --case 01_blank \
  --case-wall 300 --batch-wall 600 --execute
```

确认单场景的采集、原图和退出报告后，用 `--all` 替代 `--case`，并将 `--batch-wall` 设为 `7200`，运行全部 12 场景。输出必须不存在；失败样本不被替换，任一场景失败或进程清理无法确认时停止后续场景。不要复用失败目录或根据检出结果重采样。`--egl` 是显式渲染选项，没有自动回退重试。

环境脚本先加载 runtime，再加载当前 checkout 的 install。当前 checkout 须已构建 `limo_sim_bringup`；脚本逐字节核对安装的 launch 和单次生成/模型检查脚本。ROS domain 默认为 77，启动前要求该 domain 暂无其他节点；这个检查不是原子互斥锁。每个场景的随机 Gazebo 分区写在 `result.json`，与普通迷宫查看脚本默认的 domain 73 / `limo_maze_jazzy` 不同。

每个场景执行：隔离检查 → 单次生成及传感器完整性检查 → 原生模型 ID 快照 → 固定六组采集 → 第二次 ID 快照 → 清理自有进程。模型快照只验证数量和 ID，不证明物理静止、真实相机位置或几何精度；`stationarity_verified=false` 保持显式。

`capture/summary.json` 区分采样完成、输入/TF 有效、源码未变和 native 清理完成。样本包含原始消息 bytes、步长/字节序/编码、相机标定、PNG、原深度和米制深度 NPY、红色 mask、检测/拒绝理由及 SHA-256。`raw-checkpoint.json` 在 native 清理前 flush/fsync，清理异常时仍保留已经保存的证据。缺少最终 summary 不能视作清理成功。

选择规则是回调中首先可用、时间戳递增且晚于开始屏障的六组精确配对；不是全局最早六个传感器时间戳。拒绝帧和空检测也占用名额。RGB、depth 和双方 CameraInfo 要求同一整数时间戳；动态 TF 在该时刻查询，禁止 latest 查询。已选时间戳的重复冲突使最终判定失败，已保存原帧不被覆盖。

每次 capture 默认最多 256 MiB（CLI 硬上限 1 GiB），最多 32 对；runner 固定每场景 6 对。该容量只覆盖 capture 产物，不覆盖 runner 的全部日志。单场景活动时间默认 300 wall seconds，batch 默认 7200 seconds 并为最多六个子进程组保留清理时间；进程使用既有 PID 身份校验和 pidfd 发送信号。阻塞系统 I/O 或调度仍可能超出协作式截止时间，不能声称严格 OS 强制时限。

`capture_passed` 仅表示这组输入证据有效，不等同于类别识别正确、检出率、有效检测距离、覆盖完整或不存在目标。绝对世界坐标误差需要另行测量 settled native pose；不能直接比较 odom/map 的相机高度和世界坐标真值。

纯测试不启动 ROS/Gazebo：

```bash
/usr/bin/python3 -B -m unittest discover -s tools/red_calibration -p 'test_*.py' -v
bash -n tools/red_calibration/env.sh
```
