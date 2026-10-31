# run21 离线结果摘要

终态与原始证据闭合后生成；不读取目标真值，不启动 ROS。

```json
{
  "status": "blocked",
  "reason": "frame_retention_failed",
  "mission_success": false,
  "returned_pose_verified": false,
  "map_complete": false,
  "visual_coverage_complete": false,
  "mission_timed_out": false,
  "target_definition_verified": false,
  "coordinates_valid": true
}
```

| 导航类型 | 下发 | 成功终态 | 失败终态（含取消） | 无记录终态 |
|---|---:|---:|---:|---:|
| frontier | 2 | 1 | 1 | 0 |

观察动作结果：`{}`。
关闭原因：`blocked:frame_retention_failed`；显式 timeout 原因的观察动作数：0。

保存的 consumer 原始接受数：3；离线文件审计：passed=False，verified=0/0。

地图内标注 1 个目标；返航地图误差：`null`。

原生 home 括点到最终采样：`{"xy_distance_m": 1.158502456082745, "yaw_delta_rad": -1.1776962130663002}`；只有相同模型且 home 前后采样位姿相同时计算，详见 JSON 括点时间与跨度。

静止尾窗：`{"sample_count": 965, "first_sim_s": 149.619, "last_sim_s": 179.601, "span_s": 29.982, "max_gap_s": 0.036000000000001364, "x_span_m": 0.0, "y_span_m": 0.0, "yaw_span_rad": 0.0, "latest_native_receipt_fresh": true, "evidence_sufficient": true, "stationary_within_1mm_002rad": true, "entire_window_after_terminal": true, "ready_for_operator_to_end_observer": true}`。

所有具体阈值、失败原因、accepted ID、producer 状态、原始审计及来源 SHA256 见同名 JSON。

**本摘要不把 done、有限多朝向观察或原图留存通过解释为全迷宫视觉覆盖、正式目标识别或实车验收。**
