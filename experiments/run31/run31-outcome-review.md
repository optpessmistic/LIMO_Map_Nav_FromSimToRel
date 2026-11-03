# run31 离线结果摘要

终态与原始证据闭合后生成；不读取目标真值，不启动 ROS。

运行版本：`acf3419cfaa78ae994b704f009c3d9cb6d337609`；配置与测试/构建回执分开保留，不预设结果。

```json
{
  "status": "done",
  "reason": "bounded_observation_exhausted;final_observation_checks_incomplete",
  "mission_success": false,
  "returned_pose_verified": true,
  "map_complete": true,
  "visual_coverage_complete": false,
  "mission_timed_out": false,
  "target_definition_verified": false,
  "coordinates_valid": true
}
```

| 导航类型 | 下发 | 成功终态 | 失败终态（含取消） | 无记录终态 |
|---|---:|---:|---:|---:|
| frontier | 6 | 6 | 0 | 0 |
| home | 1 | 1 | 0 | 0 |
| observation_queue | 5 | 5 | 0 | 0 |

恢复等待 started=0、ready=0；最终原因为 recovery timeout：`False`。
导航终态 status 计数：`{"4": 12}`；error_code 计数：`{"0": 12}`；未知 error_code：0。
每次恢复的 monotonic 等待时长、后续实际 goal_sent 和终态 error_code/error_msg 见 navigation.recovery；ready 日志早于最终派发检查，不能单独证明已再次导航。

run30 对照（同类目标分别比较；不据两次运行自动判定调度或旧窗回执有因果改善）：

| 运行 | 导航类型 | 下发 | 成功 | 失败含取消 | 缺终态 |
|---|---|---:|---:|---:|---:|
| run30 | frontier | 5 | 5 | 0 | 0 |
| run30 | home | 1 | 1 | 0 | 0 |
| run30 | observation_queue | 7 | 6 | 1 | 0 |
| run31 | frontier | 6 | 6 | 0 | 0 |
| run31 | home | 1 | 1 | 0 | 0 |
| run31 | observation_queue | 5 | 5 | 0 | 0 |

原始 mission/返航标志、TF extrapolation 日志行数、目标数及留存审计并列保留在 run30_comparison；不把缺失字段计为零，也不把 error_code=0 或 ABORTED 单独解释成 TF 故障。


启动计时（CLOCK_MONOTONIC，非 ROS 仿真时钟）：

父进程记录状态：`observed_within_budget`；请求预算：`60` 秒；有限且有序的时间戳是否确认观察发生在预算内：`True`。

| 阶段 | 状态 | 非负时长（秒） | 原有符号差（秒） |
|---|---|---:|---:|
| parent_popen_call | observed_nonnegative_interval | 0.0005423970000038025 | 0.0005423970000038025 |
| parent_identity_persistence | observed_nonnegative_interval | 0.005607934000011028 | 0.005607934000011028 |
| parent_gate_write_call | observed_nonnegative_interval | 1.2612999995553764e-05 | 1.2612999995553764e-05 |
| gate_write_start_to_environment_wrapper | observed_nonnegative_interval | 10.331702683999993 | 10.331702683999993 |
| gate_write_completed_to_environment_wrapper | observed_nonnegative_interval | 10.331690070999997 | 10.331690070999997 |
| environment_wrapper_to_main_enter | observed_nonnegative_interval | 1.0476582599999915 | 1.0476582599999915 |
| main_enter_to_runtime_load_start | observed_nonnegative_interval | 0.21414748200001554 | 0.21414748200001554 |
| runtime_load | observed_nonnegative_interval | 1.7516218519999995 | 1.7516218519999995 |
| runtime_loaded_to_subscriptions_ready | observed_nonnegative_interval | 0.4428368840000019 | 0.4428368840000019 |
| subscriptions_to_collector_both_streams | observed_nonnegative_interval | 0.018688393999980235 | 0.018688393999980235 |
| collector_both_streams_to_parent_observation | observed_nonnegative_interval | 0.07415262500001063 | 0.07415262500001063 |
| parent_wait_to_marker_observation | observed_nonnegative_interval | 13.872303990999995 | 13.872303990999995 |
| parent_effective_wait_budget | observed_nonnegative_interval | 59.99999999999997 | 59.99999999999997 |
| parent_observation_to_wait_deadline | observed_nonnegative_interval | 46.127696008999976 | 46.127696008999976 |
| gate_write_start_to_parent_observation | observed_nonnegative_interval | 13.880808180999992 | 13.880808180999992 |

环境时间是轻量包装器入口观测，包含包装器启动成本；main_enter 不含顶层导入。父进程读到标记的时间不同于 collector 观察双流、打印标记的时间。负差值或缺失不算零耗时，也不构成就绪或运动通过。gate 写入前后形成释放事件的边界，子进程可能早于父进程写后采样运行；具体阶段解释和原字段见 startup_timing。


冻结观察队列（完整计划分母，不扣除跳过/失败/待执行项）：

完成 5/12；原始状态分布：`{"completed": 5, "skipped_unsafe_or_unreachable": 7}`；全部计划动作完成：`False`。
实际派发站位/朝向顺序：`[(0, 0), (0, 1), (0, 2), (0, 3), (2, 3)]`；跨站切换 1 次，重复进入站块：`[]`。
这里只统计已记录派发；不把站间直线距离或动作总耗时当成实际轨迹/纯运动时间。

退休窗口 0 个、接受 0 帧、stored 0；存储回执严格早于原 deadline：0；达到/超过截止：0；时间未知：0。
逐帧 stored_receipt_wall、原 deadline、最后记录 closed_wall 和 binding 见 retention.retired_receipts。退休存储确认不计为观察完成或新窗 credit；credited 回调返回值未保存，不能编造其运行时计数。late_results 也不是成功存储数；原严格文件审计结果独立保留。


观察动作结果：`{"inspection_satisfied": 5}`。
TF age 首次拒绝事件：0；navigation extrapolation 匹配日志行数：0。
TF 事件保留判据 now、TF stamp、age、阈值与 frames；日志行计数不是独立故障数，不用零条日志推断 TF 无故障。
关闭原因：`bounded_observation_exhausted`；显式 timeout 原因的观察动作数：0。

保存的 consumer 原始接受数：33；离线文件审计：passed=True，verified=33/33。

保留的 producer fault：`None`；缺少预期文件的 accepted 帧数：0；producer 未 stored 的 accepted 帧数：0。
缺失 ID 与逐文件路径见 JSON accepted_file_inventory；文件存在性不等于严格审计通过。严格审计可能因 producer fault 提前返回 failed 0/0，其计数不替代 mission 原始接受数。

有记录的唯一 writer 操作：16；具有管道诊断：16；实际容量分布：`{"1048576": 16}`；writer 超时记录：0。
逐操作初始/请求/实际容量、扩容错误、发送/剩余字节、EAGAIN、首末进展及关闭时间、最大 poll 间隔均在 writer_transport。历史有长度上限，不代表全部任务；父端时间不等于子进程读取或写盘时间。

地图内标注 2 个目标；返航地图误差：`{"xy_distance_m": 0.023392305320010763, "yaw_error_rad": 0.13278933271368448}`。

原生 home 括点到最终采样：`{"xy_distance_m": 0.01328360603721483, "yaw_delta_rad": 0.13395633432050463}`；只有相同模型且 home 前后采样位姿相同时计算，详见 JSON 括点时间与跨度。

记录的尾窗：`{"sample_count": 962, "first_sim_s": 306.201, "last_sim_s": 336.195, "span_s": 29.99399999999997, "max_gap_s": 0.03699999999997772, "x_span_m": 0.0, "y_span_m": 0.0, "yaw_span_rad": 0.0, "latest_native_receipt_fresh": true, "evidence_sufficient": true, "stationary_within_1mm_002rad": true, "entire_window_after_terminal": true, "ready_for_operator_to_end_observer": true}`。
编排器完整终态尾窗验证：`True`；采样跨度及是否全部处于终态后以原始字段为准，不把部分尾窗当成完整通过。

编排器状态：`terminal_observed`；错误：`[]`。
原始 all_owned_groups_empty=`False`；根执行 session=26612、exit_code=1。errors=[] 不代表原关闭成功。
独立后续只读复查确认 20 个原记录进程组当前为空；详见 closure_supplement。它不改写原 false/exit1，不证明原关闭期限达标，也不声称系统全局进程为空。
地图快照与 posegraph 捕获：passed=`True`；错误：`[]`。
save_map 服务是否调用：`False`；独立捕获栅格导出：passed=`True`。
独立栅格导出与原生终态尾窗结果分别保留，不互相替代，也不称为修复 save_map 服务。

所有具体阈值、失败原因、accepted ID、producer 状态、原始审计及来源 SHA256 见同名 JSON。

**本摘要不把 done、有限多朝向观察或原图留存通过解释为全迷宫视觉覆盖、正式目标识别或实车验收。**
