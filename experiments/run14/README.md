# run14 精简结果存档

这是已经关闭的实验结果，运行代码 `7b300d9`，不是下一轮未知迷宫的输入。完整说明见 [run14 报告](../../docs/autonomous-run14.md)。

- `mission.json`：原始任务结果，保存 home、目标标注、事件和 `mission_success=false`。
- `target-evaluation.json`：固定初始对齐后的独立替身匹配，仅评价平面位置。
- `final-snapshot.json`：原始 OccupancyGrid、采样时位姿/TF/扫描；地图与其他数据各有时间，不是假定同步的控制器输入。
- `run14-final-map.yaml` / `.pgm`：map server 三值导出，供查看/加载该次结果；几何精度及阈值与原始快照区分。
- `manifest.json`：每个原文件的仓库相对来源、字节数和 SHA256。复制未改变内容。

原始录包、连续观察器、位姿图和详细离线分析仍保存在本机忽略的 `outputs/ros_smoke/` 中。本目录仅提供轻量可审阅结果，不声称包含完整原始数据。不得把本轮地图、目标或世界真值作为后续未知环境探索的先验。
