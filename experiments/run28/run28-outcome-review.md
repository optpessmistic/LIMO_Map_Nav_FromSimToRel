# run28 离线结果摘要

终态与原始证据闭合后生成；不读取目标真值，不启动 ROS。

运行版本：`e8e72e89914a20c6be28d7a5f98b67d524861a4d`；配置与测试/构建回执分开保留，不预设结果。

```json
{
  "status": "blocked",
  "reason": "return_navigation_failed",
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
| frontier | 12 | 1 | 11 | 0 |
| home | 4 | 0 | 4 | 0 |

启动计时（CLOCK_MONOTONIC，非 ROS 仿真时钟）：

父进程记录状态：`observed_within_budget`；请求预算：`60` 秒；有限且有序的时间戳是否确认观察发生在预算内：`True`。

| 阶段 | 状态 | 非负时长（秒） | 原有符号差（秒） |
|---|---|---:|---:|
| parent_popen_call | observed_nonnegative_interval | 0.0006417040000314955 | 0.0006417040000314955 |
| parent_identity_persistence | observed_nonnegative_interval | 0.028390306000005694 | 0.028390306000005694 |
| parent_gate_write_call | observed_nonnegative_interval | 2.7899000031084142e-05 | 2.7899000031084142e-05 |
| gate_write_start_to_environment_wrapper | observed_nonnegative_interval | 11.923884190000024 | 11.923884190000024 |
| gate_write_completed_to_environment_wrapper | observed_nonnegative_interval | 11.923856290999993 | 11.923856290999993 |
| environment_wrapper_to_main_enter | observed_nonnegative_interval | 0.862414092999984 | 0.862414092999984 |
| main_enter_to_runtime_load_start | observed_nonnegative_interval | 0.17179274700004044 | 0.17179274700004044 |
| runtime_load | observed_nonnegative_interval | 2.9219392939999693 | 2.9219392939999693 |
| runtime_loaded_to_subscriptions_ready | observed_nonnegative_interval | 0.6311163600000214 | 0.6311163600000214 |
| subscriptions_to_collector_both_streams | observed_nonnegative_interval | 0.5827123869999582 | 0.5827123869999582 |
| collector_both_streams_to_parent_observation | observed_nonnegative_interval | 0.16008295300002828 | 0.16008295300002828 |
| parent_wait_to_marker_observation | observed_nonnegative_interval | 17.214943827000013 | 17.214943827000013 |
| parent_effective_wait_budget | observed_nonnegative_interval | 60.0 | 60.0 |
| parent_observation_to_wait_deadline | observed_nonnegative_interval | 42.78505617299999 | 42.78505617299999 |
| gate_write_start_to_parent_observation | observed_nonnegative_interval | 17.253942024000025 | 17.253942024000025 |

环境时间是轻量包装器入口观测，包含包装器启动成本；main_enter 不含顶层导入。父进程读到标记的时间不同于 collector 观察双流、打印标记的时间。负差值或缺失不算零耗时，也不构成就绪或运动通过。gate 写入前后形成释放事件的边界，子进程可能早于父进程写后采样运行；具体阶段解释和原字段见 startup_timing。


观察动作结果：`{}`。
TF age 首次拒绝事件：0；navigation extrapolation 匹配日志行数：5。
TF 事件保留判据 now、TF stamp、age、阈值与 frames；日志行计数不是独立故障数，不用零条日志推断 TF 无故障。
关闭原因：`perception_unverified_incomplete`；显式 timeout 原因的观察动作数：0。

保存的 consumer 原始接受数：3；离线文件审计：passed=False，verified=0/0。

保留的 producer fault：`None`；缺少预期文件的 accepted 帧数：0；producer 未 stored 的 accepted 帧数：0。
缺失 ID 与逐文件路径见 JSON accepted_file_inventory；文件存在性不等于严格审计通过。严格审计可能因 producer fault 提前返回 failed 0/0，其计数不替代 mission 原始接受数。

有记录的唯一 writer 操作：3；具有管道诊断：3；实际容量分布：`{"1048576": 3}`；writer 超时记录：0。
逐操作初始/请求/实际容量、扩容错误、发送/剩余字节、EAGAIN、首末进展及关闭时间、最大 poll 间隔均在 writer_transport。历史有长度上限，不代表全部任务；父端时间不等于子进程读取或写盘时间。

地图内标注 2 个目标；返航地图误差：`null`。

原生 home 括点到最终采样：`{"xy_distance_m": 1.1709808013693874, "yaw_delta_rad": 2.882272986347519}`；只有相同模型且 home 前后采样位姿相同时计算，详见 JSON 括点时间与跨度。

记录的尾窗：`{"sample_count": 968, "first_sim_s": 221.894, "last_sim_s": 251.894, "span_s": 30.0, "max_gap_s": 0.03699999999997772, "x_span_m": 0.0, "y_span_m": 0.0, "yaw_span_rad": 0.0, "latest_native_receipt_fresh": true, "evidence_sufficient": true, "stationary_within_1mm_002rad": true, "entire_window_after_terminal": true, "ready_for_operator_to_end_observer": true}`。
编排器完整终态尾窗验证：`True`；采样跨度及是否全部处于终态后以原始字段为准，不把部分尾窗当成完整通过。

编排器状态：`terminal_observed`；错误：`["Retained-frame offline audit failed"]`。
地图快照与 posegraph 捕获：passed=`True`；错误：`[]`。
save_map 服务是否调用：`False`；独立捕获栅格导出：passed=`True`。
独立栅格导出与原生终态尾窗结果分别保留，不互相替代，也不称为修复 save_map 服务。

所有具体阈值、失败原因、accepted ID、producer 状态、原始审计及来源 SHA256 见同名 JSON。

**本摘要不把 done、有限多朝向观察或原图留存通过解释为全迷宫视觉覆盖、正式目标识别或实车验收。**
