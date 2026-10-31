# run22：15 对图像留存通过，建图与收尾仍不完整

运行代码 `f6831d8c980ed1040510bf7a4d12703c07ace460`，epoch `3f4297ec-2699-4489-85df-ae52050cda1e`。原始 Gazebo 任务已关闭，编排器退出 1、耗时 704.635 wall 秒。五次 frontier 和一次 home 导航均报告成功，五个已完成观察窗接受的 **15 对 RGB-D 全部通过严格闭合审计**；另有三个未接受任何样本的已退役窗口。

mission 为 `done`、`returned_pose_verified=true`，但 `map_complete=false`、`visual_coverage_complete=false`、`mission_success=false`，没有进入地图完成后的有限多朝向观察。两个红色替身标注不代表正式目标识别或全迷宫视觉搜索通过。[实验文档](../../docs/run22-retention-verification.md)和[机器摘要](run22-outcome-review.json)列出范围。

`save_map` 服务返回 255，PGM/YAML 未生成；`serialize_map` 返回 0。编排器在导出失败后提前进入关闭，原定 30 sim 秒终态静止尾窗未验证。`recorded-map-and-targets.png` 是 `final-save.json` 内原始 OccupancyGrid 的离线渲染，**不是成功的地图服务导出**；没有创建替代 PGM/YAML 混入本轮结果。

## 原图压缩包及复验

`retained-frames.tar.gz` 包含 15 个固定 accepted 样本的 90 个原始文件，以及 `retained/checkpoint.json`，共 91 个普通文件。每个样本含 RGB、depth、producer、CameraInfo、request/精确 TF 与 manifest。内容无损，tar 的时间和用户元数据统一处理。`retained-files.json` 在压缩包外列出每个成员的原路径、长度、SHA256；`retained-checkpoint.json` 额外保留可直接查看的关闭状态。

已逐成员回读压缩包，核对原始 producer receipt 的文件哈希，并临时解压后再次运行严格审计。结果保存在 `archive-reopened-audit.json`，仍为 15/15 通过。此检查验证文件、固定身份与提交的 TF 一致性，不验证 TF 测量精度、视觉覆盖或物品召回率。

在仓库根目录使用 Python 3.12，可重复只读审计；不需要 ROS：

```bash
cd experiments/run22
sha256sum -c SHA256SUMS
review_dir="$(mktemp -d -t limo-run22-review.XXXXXX)"
tar -xzf retained-frames.tar.gz -C "$review_dir"
python3 -B ../../tools/audit_frame_retention.py \
  --consumer mission/mission_20261031_055739.json \
  --retention-root "$review_dir/retained"
```

源审计工具 SHA256 记录在 `retained-files.json`；原始运行版本见上述 commit。`pack-retained.py` 是归档构建与回读 helper，引用本机原始输出并拒绝覆盖已有包，普通复验应使用上面的解压命令。

## 证据与限制

- 原始 mission、严格审计、关闭 checkpoint、参数、provenance、收尾失败日志、精确 runner 与构建安装证据均保留；安装报告确认六个模块与当前工作区源码逐字节一致。
- observer、初始 survey 和初始运动以无损 `.json.gz` 保存；`archive-manifest.json` 列出解压后原始哈希。所选两份小栅格快照和后续诊断用于复核未完成前沿，不含 world 目标真值。
- MCAP、posegraph 和 data 未收录，原文件路径、长度、SHA256 留在归档清单；不凭录包 metadata 宣称录包无损或完整性已验证。
- 自有进程组、嵌套 recorder 与 producer writer 的退出已确认；全局 `/proc` 环境扫描仍有权限不足，原始 `all_run_token_processes_absent=false` 保留，没有全局无残留承诺。
- 原生 home 静止括点到最终采样 XY 位移约 22.478 mm、yaw 变化 -0.08563 rad；map 内 home 到最终位姿误差约 30.396 mm、0.13448 rad。两种坐标比较分开报告，没有假定 map 与 world 重合。
- observer 的约 30 秒滚动尾窗包含终态前转向，`stationary_within_1mm_002rad=false`、`entire_window_after_terminal=false`。最终 native 样本仅在首次终态状态接收后约 18.804 sim 秒，不能称本轮终态静止尾窗通过。

`SHA256SUMS` 覆盖全部归档文件，清单本身除外。归档身份校验、图像留存通过、返航验证和完整项目验收是不同结论；本目录保留建图、地图导出及终态尾窗未完成的原始事实。
