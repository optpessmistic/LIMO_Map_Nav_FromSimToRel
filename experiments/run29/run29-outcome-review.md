# run29 离线结果摘要

终态与原始证据闭合后生成；不读取目标真值，不启动 ROS。

运行版本：`9e0a32687abe52eca1751a1e0e24771618835c38`；配置与测试/构建回执分开保留，不预设结果。

```json
{
  "status": "done",
  "reason": "perception_unverified_incomplete;home_frontier_return_attempts_exhausted;final_observation_checks_incomplete",
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
| frontier | 9 | 1 | 8 | 0 |
| home | 4 | 1 | 3 | 0 |

恢复等待 started=11、ready=11；最终原因为 recovery timeout：`False`。
导航终态 status 计数：`{"6": 11, "4": 2}`；error_code 计数：`{"102": 11, "0": 2}`；未知 error_code：0。
每次恢复的 monotonic 等待时长、后续实际 goal_sent 和终态 error_code/error_msg 见 navigation.recovery；ready 日志早于最终派发检查，不能单独证明已再次导航。

run28 对照（同类目标分别比较；不据两次运行自动判定冷却有因果改善）：

| 运行 | 导航类型 | 下发 | 成功 | 失败含取消 | 缺终态 |
|---|---|---:|---:|---:|---:|
| run28 | frontier | 12 | 1 | 11 | 0 |
| run28 | home | 4 | 0 | 4 | 0 |
| run29 | frontier | 9 | 1 | 8 | 0 |
| run29 | home | 4 | 1 | 3 | 0 |

原始 mission/返航标志、TF extrapolation 日志行数、目标数及留存审计并列保留在 run28_comparison；不把缺失字段计为零，也不把新 error_code=0 或 ABORTED 单独解释成 TF 故障。


启动计时（CLOCK_MONOTONIC，非 ROS 仿真时钟）：

父进程记录状态：`observed_within_budget`；请求预算：`60` 秒；有限且有序的时间戳是否确认观察发生在预算内：`True`。

| 阶段 | 状态 | 非负时长（秒） | 原有符号差（秒） |
|---|---|---:|---:|
| parent_popen_call | observed_nonnegative_interval | 0.0006307449999667369 | 0.0006307449999667369 |
| parent_identity_persistence | observed_nonnegative_interval | 0.08396447799998441 | 0.08396447799998441 |
| parent_gate_write_call | observed_nonnegative_interval | 3.20310000461177e-05 | 3.20310000461177e-05 |
| gate_write_start_to_environment_wrapper | observed_nonnegative_interval | 12.06295499600003 | 12.06295499600003 |
| gate_write_completed_to_environment_wrapper | observed_nonnegative_interval | 12.062922964999984 | 12.062922964999984 |
| environment_wrapper_to_main_enter | observed_nonnegative_interval | 2.1968347750000135 | 2.1968347750000135 |
| main_enter_to_runtime_load_start | observed_nonnegative_interval | 0.2737887679999744 | 0.2737887679999744 |
| runtime_load | observed_nonnegative_interval | 1.7716068290000067 | 1.7716068290000067 |
| runtime_loaded_to_subscriptions_ready | observed_nonnegative_interval | 0.4724472689999857 | 0.4724472689999857 |
| subscriptions_to_collector_both_streams | observed_nonnegative_interval | 0.12500497000002042 | 0.12500497000002042 |
| collector_both_streams_to_parent_observation | observed_nonnegative_interval | 0.06615782299996908 | 0.06615782299996908 |
| parent_wait_to_marker_observation | observed_nonnegative_interval | 16.928562532 | 16.928562532 |
| parent_effective_wait_budget | observed_nonnegative_interval | 60.0 | 60.0 |
| parent_observation_to_wait_deadline | observed_nonnegative_interval | 43.071437468 | 43.071437468 |
| gate_write_start_to_parent_observation | observed_nonnegative_interval | 16.96879543 | 16.96879543 |

环境时间是轻量包装器入口观测，包含包装器启动成本；main_enter 不含顶层导入。父进程读到标记的时间不同于 collector 观察双流、打印标记的时间。负差值或缺失不算零耗时，也不构成就绪或运动通过。gate 写入前后形成释放事件的边界，子进程可能早于父进程写后采样运行；具体阶段解释和原字段见 startup_timing。


观察动作结果：`{}`。
TF age 首次拒绝事件：0；navigation extrapolation 匹配日志行数：11。
TF 事件保留判据 now、TF stamp、age、阈值与 frames；日志行计数不是独立故障数，不用零条日志推断 TF 无故障。
关闭原因：`perception_unverified_incomplete`；显式 timeout 原因的观察动作数：0。

保存的 consumer 原始接受数：3；离线文件审计：passed=True，verified=3/3。

保留的 producer fault：`None`；缺少预期文件的 accepted 帧数：0；producer 未 stored 的 accepted 帧数：0。
缺失 ID 与逐文件路径见 JSON accepted_file_inventory；文件存在性不等于严格审计通过。严格审计可能因 producer fault 提前返回 failed 0/0，其计数不替代 mission 原始接受数。

有记录的唯一 writer 操作：3；具有管道诊断：3；实际容量分布：`{"1048576": 3}`；writer 超时记录：0。
逐操作初始/请求/实际容量、扩容错误、发送/剩余字节、EAGAIN、首末进展及关闭时间、最大 poll 间隔均在 writer_transport。历史有长度上限，不代表全部任务；父端时间不等于子进程读取或写盘时间。

地图内标注 2 个目标；返航地图误差：`{"xy_distance_m": 0.027716891462896076, "yaw_error_rad": 0.13897465332856124}`。

原生 home 括点到最终采样：`{"xy_distance_m": 0.0158747402524572, "yaw_delta_rad": -0.14285409490046536}`；只有相同模型且 home 前后采样位姿相同时计算，详见 JSON 括点时间与跨度。

记录的尾窗：`{"sample_count": 977, "first_sim_s": 270.101, "last_sim_s": 300.101, "span_s": 30.0, "max_gap_s": 0.03699999999997772, "x_span_m": 0.0, "y_span_m": 0.0, "yaw_span_rad": 0.0, "latest_native_receipt_fresh": true, "evidence_sufficient": true, "stationary_within_1mm_002rad": true, "entire_window_after_terminal": true, "ready_for_operator_to_end_observer": true}`。
编排器完整终态尾窗验证：`True`；采样跨度及是否全部处于终态后以原始字段为准，不把部分尾窗当成完整通过。

编排器状态：`terminal_observed`；错误：`[]`。
地图快照与 posegraph 捕获：passed=`True`；错误：`[]`。
save_map 服务是否调用：`False`；独立捕获栅格导出：passed=`True`。
独立栅格导出与原生终态尾窗结果分别保留，不互相替代，也不称为修复 save_map 服务。

所有具体阈值、失败原因、accepted ID、producer 状态、原始审计及来源 SHA256 见同名 JSON。

**本摘要不把 done、有限多朝向观察或原图留存通过解释为全迷宫视觉覆盖、正式目标识别或实车验收。**
