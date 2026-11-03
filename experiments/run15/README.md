# run15 精简证据：五次观察通过，第六点超差停止

本轮运行代码为 `5416c08`，地图 epoch 为 `59062299-c86d-4023-8748-babca088fafc`。**任务未完成建图或返航，`mission_success=false`**。完整解释见 [实验记录](../../docs/autonomous-run15.md)。

这些文件按原始字节复制，来源、大小和 SHA256 见 [manifest.json](manifest.json)。`mission.json` 保留旧实现中第六点失败误带第五点账本的错误；后续代码修复不改写该结果。

- `mission.json`、`shutdown.json`：任务结果和退出审计。
- `goal6-diagnosis.json`：独立 TF、odom、native 轨迹与离散足迹诊断；停稳后当前 map 目标距离为 52.898 mm，超过 50 mm。
- `preflight.json`、`parameters-early.json`、`provenance-live.json`：启动前图像/地图检查、实际参数与加载路径证据。
- `final-save.json`、`final-map.yaml/.pgm`：完整原始 OccupancyGrid 快照和地图导出；原始快照比三态地图导出保留更精确的几何信息。
- `frame-audit.json`、`sample-001` 至 `sample-005`：五个运行时完成窗口各自**最后一帧**的源数据摘要、RGB 预览、标定、生产记录与 TF 输入。五帧一致；未独立重建全部十五帧，未证明视觉覆盖。
- `recording-metadata.yaml`、`recording.log`：完整录制的元数据和日志。末帧审计只读取绝对接收时间 ≤700 s 的前缀；整包累计 49,190 条传输丢失无法在此归因到任务时间窗。

完整 MCAP、RGB/Depth 原始字节与 CDR、深度数组、连续原生轨迹和位姿图仍保留于本地忽略目录 `outputs/ros_smoke/run15-settled-inspection/`，未上传。目录中的审计报告引用这些原始路径，不意味着 Git 存档包含所有原始数据。
