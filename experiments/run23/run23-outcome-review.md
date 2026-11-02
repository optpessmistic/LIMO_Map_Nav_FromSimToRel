# run23 离线结果摘要

终态与原始证据闭合后生成；不读取目标真值，不启动 ROS。

```json
{
  "status": "blocked",
  "reason": "operator_sigint_stop_verified",
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
| frontier | 1 | 0 | 1 | 0 |

观察动作结果：`{}`。
关闭原因：`shutdown:operator_sigint`；显式 timeout 原因的观察动作数：0。

保存的 consumer 原始接受数：0；离线文件审计：passed=None，verified=None/None。

地图内标注 1 个目标；返航地图误差：`null`。

原生 home 括点到最终采样：`{"xy_distance_m": 0.09931799777480507, "yaw_delta_rad": 0.12613902863783052}`；只有相同模型且 home 前后采样位姿相同时计算，详见 JSON 括点时间与跨度。

记录的尾窗：`{"sample_count": 321, "first_sim_s": 95.751, "last_sim_s": 105.678, "span_s": 9.926999999999992, "max_gap_s": 0.036000000000001364, "x_span_m": 0.0991711865422173, "y_span_m": 0.005471093524955433, "yaw_span_rad": 0.12613902863783052, "latest_native_receipt_fresh": true, "evidence_sufficient": false, "stationary_within_1mm_002rad": false, "entire_window_after_terminal": false, "ready_for_operator_to_end_observer": false}`。
编排器完整终态尾窗验证：`False`；采样跨度及是否全部处于终态后以原始字段为准，不把部分尾窗当成完整通过。

编排器状态：`orchestration_failed`；错误：`["RuntimeError: save-early exited 1", "Retention closure audit: RuntimeError: No terminal mission result for retained-frame audit"]`。
地图快照与 posegraph 捕获：passed=`None`；错误：`null`。
save_map 服务是否调用：`None`；独立捕获栅格导出：passed=`None`。
独立栅格导出与原生终态尾窗结果分别保留，不互相替代，也不称为修复 save_map 服务。

所有具体阈值、失败原因、accepted ID、producer 状态、原始审计及来源 SHA256 见同名 JSON。

**本摘要不把 done、有限多朝向观察或原图留存通过解释为全迷宫视觉覆盖、正式目标识别或实车验收。**
