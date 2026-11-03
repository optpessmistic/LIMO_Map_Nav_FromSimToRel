# run22 离线结果摘要

终态与原始证据闭合后生成；不读取目标真值，不启动 ROS。

```json
{
  "status": "done",
  "reason": "unreachable_frontiers_or_coverage_incomplete;final_observation_checks_incomplete",
  "mission_success": false,
  "returned_pose_verified": true,
  "map_complete": false,
  "visual_coverage_complete": false,
  "mission_timed_out": false,
  "target_definition_verified": false,
  "coordinates_valid": true
}
```

| 导航类型 | 下发 | 成功终态 | 失败终态（含取消） | 无记录终态 |
|---|---:|---:|---:|---:|
| frontier | 5 | 5 | 0 | 0 |
| home | 1 | 1 | 0 | 0 |

观察动作结果：`{}`。
关闭原因：`unreachable_frontiers_or_coverage_incomplete`；显式 timeout 原因的观察动作数：0。

保存的 consumer 原始接受数：15；离线文件审计：passed=True，verified=15/15。

地图内标注 2 个目标；返航地图误差：`{"xy_distance_m": 0.03039619377884381, "yaw_error_rad": 0.134481174460359}`。

原生 home 括点到最终采样：`{"xy_distance_m": 0.02247818761843492, "yaw_delta_rad": -0.08562822308891557}`；只有相同模型且 home 前后采样位姿相同时计算，详见 JSON 括点时间与跨度。

记录的尾窗：`{"sample_count": 966, "first_sim_s": 246.469, "last_sim_s": 276.436, "span_s": 29.966999999999985, "max_gap_s": 0.036000000000001364, "x_span_m": 0.0006854749297662416, "y_span_m": 0.00014076180826384354, "yaw_span_rad": 2.806708646578688, "latest_native_receipt_fresh": true, "evidence_sufficient": true, "stationary_within_1mm_002rad": false, "entire_window_after_terminal": false, "ready_for_operator_to_end_observer": false}`。
编排器完整终态尾窗验证：`False`；采样跨度及是否全部处于终态后以原始字段为准，不把部分尾窗当成完整通过。

编排器状态：`orchestration_failed`；错误：`["RuntimeError: save-final exited 1"]`。
地图服务导出：passed=`False`；错误：`["export_incomplete:/slam_toolbox/save_map", "expected_export_files_missing_or_empty_at_capture"]`。

所有具体阈值、失败原因、accepted ID、producer 状态、原始审计及来源 SHA256 见同名 JSON。

**本摘要不把 done、有限多朝向观察或原图留存通过解释为全迷宫视觉覆盖、正式目标识别或实车验收。**
