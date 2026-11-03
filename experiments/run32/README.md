# run32：地图更新超时，观察点调整尚未执行

2026-11-03，运行版本 `c15d0f3af8cf4ea9b18cb81275f6f92286b137ed`。本轮相对 run31 启用 `observation_relocation_radius_m=0.15`，保留原安全距离、图像留存要求和时间预算。实际加载的是 `0002+0003` 的旧 SLAM 安装；后续 `0004` 地图调度补丁不属于本次运行。

根会话 **74954 实际退出 1**，耗时约 509.03 秒。所有本次持有的进程组退出检查通过；原始严格图像审计失败使整轮结果仍失败。部分其他同 UID 进程的环境不可读，不能据此宣称系统中不存在任何其他 ROS 进程。

| 项目 | 原始结果 |
|---|---|
| 任务终态 | `blocked / map_stale` |
| 探索导航 | 第一个成功；第二个因地图过期主动取消，状态 5 |
| 观察队列 | 尚未创建；实际计划视图分母为 `null` |
| 0.15 m 观察点调整 | 尚未执行，未验证 |
| 替身标注 | 1 个 |
| 地图完成 / 返航 / 完整视觉覆盖 | 全部为 false |
| 完整任务成功 | false |
| 严格留存审计 | false，`consumer_disabled_or_faulted` |

首次探索停车窗口记录 3 组接受并已写入的 RGB-D。后续任务故障使 consumer 失效；原严格审计因此短路，记录 accepted=0、verified=0。不能将 3 个写入状态改记为严格审计通过，也不能由审计的 0 推断原图丢失。独立原生位姿显示末段已停车，但未验证返航；起点至终点距离约 1.94 m。

地图过期采用 **5 秒单调墙钟接收间隔**。闭合录包在故障附近记录 2.565 ROS 秒的地图消息间断，同期扫描、里程计与 TF 继续更新。控制器日志出现倒退时间戳，但本轮没有同步的系统钟探针；地图锁等待、计算或传输仍未排除。[诊断及后续补丁](../../docs/map-publish-clock.md)不能替代本次原失败结论。

## 证据与复核

- [任务、停车和留存结果](analysis/run32-outcome-review.md)及其 [JSON](analysis/run32-outcome-review.json)。
- [地图传递审计](analysis/run32-map-delivery-audit.json)：闭合后只读解码，28 份输入哈希前后保持一致。
- [终点选择审计](analysis/run32-selection-audit.json)：因没有观察队列退出 1，结果不可用。
- `preparation.json`、`installed-check.json`、参数门禁和 `provenance-live.json` 绑定源码、配置与实际加载库；`runner/` 和 `analysis/` 保存当时脚本的原字节 `.txt` 副本。
- `orchestration.json`、`retention-audit.json`、`retention-shutdown.json` 保留关闭和审计原值；`archive-validation.json` 只证明归档副本核对通过。

两份大型 JSON 以 `mtime=0` 的 gzip 保存。`archive-provenance.json` 记录原文件及压缩副本的 SHA256，解压可恢复原字节。`external-inputs.json` 仅记录 21 份 MCAP、RGB-D 和 posegraph/data 文件的路径、大小与 SHA256，原文件仍在 `outputs/ros_smoke/run32-observation-relocation/`；191 份 costmap 快照未复制。此紧凑档案不能独立重跑完整图像或录包审计，请保留原输出目录。

在本目录运行 `sha256sum -c SHA256SUMS` 核对归档字节；该命令不验证外部原文件。归档完整性、构建成功和 CI 均不等于实验成功。
