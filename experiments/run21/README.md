# run21：留存确认超时的失败证据

运行代码 `2d36634caf0665e6b0af88e7452575ae69c0d0dd`，epoch `2cfdd336-5f22-4977-aa21-29fc5a0c9c64`。真实 Gazebo 运行已结束，编排器退出 1，耗时 493.421 wall 秒。任务因 `frame_retention_failed` 阻塞：未完成建图、未进入地图完成后的多朝向观察、未返航。原始失败结果没有用离线回放或后来保存的图像改写。

说明见[本轮实验文档](../../docs/run21-retention-deadline.md)，简要机器结果见 [run21-outcome-review.json](run21-outcome-review.json)。图像 `map-and-targets.png` 只画记录的部分地图、home 和一个红色替身标注；`final_pose=null`，没有假造地图终点。

`retained/` 保存这次 mission 接受的全部三个固定样本：每个目录含原始 RGB、depth、标定、producer 元数据、精确 request/TF 和 manifest。目录名是 request SHA256。三个不同时间戳的 RGB 内容哈希相同，depth 内容哈希也相同；这是原始文件状态，未去重、换帧或重写时间戳。mission 仅在时限内确认前两个持久化回执，第三个仍 pending；producer 关闭时三者均为 stored。

`retention-audit.json` 原样为 **failed**，错误 `consumer_disabled_or_faulted`。其 `accepted_count=0 / verified_count=0` 是前置条件拒绝后未遍历帧的计数，**不表示运行实际接受零帧，也不表示图像字节丢失**。`archive-verification.json` 仅核对归档副本、三个 request 身份和 producer receipt 列出的 18 个文件哈希；不补发 mission 回执，不追认观察通过。

`orchestration.json` 的自有进程组已清空，`retention-shutdown.json` 确认 writer 退出和最终 checkpoint 保存。原始全局 `/proc` token 扫描因两个 `PermissionError` 未完成，`all_run_token_processes_absent=false` 保留。后来 recheck/context 的 systemd 用户会话旁证属于不同 PID，不能追溯证明原来不可读进程的身份或全局无残留。

归档内容：

- 原始 mission、checkpoint、审计失败、关闭证据、实际参数、provenance，以及选取的小日志。
- `observer.json.gz`、`initial-survey.json.gz`、`initial-motion.json.gz` 是原文件无损 gzip；解压后 SHA256 与 `archive-manifest.json` 一致。
- `runner/*.txt` 是包括本机路径的精确 helper 副本，配合记录的代码提交和运行参数重建实验配置。不要把已有输出目录当作可重跑的启动入口。
- `writer-benchmark/` 保存两次已闭合离线回放的元数据、trace、checkpoint、manifest、runner 与原 README。baseline 第三请求超时，候选 `python -S -B` 在单次回放中取得三个回执；未把这个小样本结果当成真实 Gazebo 通过。重复的回放图像未再次收录，原始三对图像完整保留。
- 后续生产源码最小启动改动的定向回归为 34/34，通过日志、执行 helper 和前后源码哈希也在 `writer-benchmark/`。这属于失败后的修复验证，不是原始 run21 的运行代码。重现原 baseline 应使用上面记录的 `2d36634` 代码与原 inputs 环境；不能把后续源码当作原 baseline。
- MCAP、posegraph、data 及冗余 costmap 未收录；前三类大文件的本机路径、长度和 SHA256 记在归档清单。这里不能据 metadata 单独验证整个录包。

在本目录运行 `sha256sum -c SHA256SUMS` 可校验全部归档文件（清单本身除外）。Git 属性在本目录固定原始字节，避免换行转换破坏证据。归档校验通过仅证明这些副本完整，不代表任务成功、全迷宫视觉覆盖、正式目标识别或实车实验完成。
