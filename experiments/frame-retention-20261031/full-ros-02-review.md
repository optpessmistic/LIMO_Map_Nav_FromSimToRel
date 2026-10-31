# Full ROS 02 失败及针对性重跑复核

完整测试第二轮仍是失败结果：**460 tests / 123.666 s / failures=3 / skipped=1 / exit 1**。随后只重跑这三个失败用例，得到 **3 tests / 30.612 s / OK / exit 0**。重跑前唯一源码差异是在 `test_mission_inspection_ros.py` 显式添加共享协议包的 source import 路径；生产代码、安全阈值、夹具等待时长、固定帧数均未改变。

这次重跑证明三个用例在该次调度中能通过，不能覆盖原失败、证明完整测试已通过，或证明时序不稳定的唯一原因已经解决。本轮不继续修改夹具等待策略，也不通过反复重跑筛选成功结果。

## 已核对来源与范围

只读检查了 `outputs/setup/frame-retention-full-ros-02.log/.json` 与 `outputs/setup/frame-retention-targeted-ros-03.log/.json`；本目录归档为 [完整测试日志](full-ros-02.log)、[完整测试来源](full-ros-02.json)、[针对性日志](targeted-ros-03.log)、[针对性来源](targeted-ros-03.json)。针对性命令恰包含原失败的三个测试，无替换用例。前后生产 `node.py` 均为 SHA-256 `b37e82de359649c54e0dddb8ab5f211a9a53f326b7088e95535f20eaf53974ec`；观察 DDS fixture 均为 `8cbc3e84143b7b5e4ae82732a19d900a5016bfdf53ea060f12a16c00a8d3b717`。共享依赖路径修复后的 inspection fixture 为 `51f540b3d80ffc701825a6641a530a4d1a4d09b55b881ee457d49a4e6a3c5680`。

本复核者未运行这些 ROS 测试、启动 ROS/Gazebo、修改生产源码或测试。下面两个 observation 失败做了独立日志分析；第三项的首次停车超时也从 traceback 核对，根任务负责其进一步诊断。真实 ROS 用例使用合成地图、TF、里程计和受控 ActionServer，不是 Nav2/Gazebo 物理停车或实车验收。

三个测试均未启用 `require_retained_inspection`；默认 false，新增 Consumer 不创建，相关分支短路。日志没有表明新留证门槛介入了这三个失败，也未建立默认关闭时的生产控制回归因果。不能由一次成功重跑推导整个默认关闭路径已无任何回归风险。

## 失败一：第二个目标隔离用例在首次停车阶段超时

`test_second_goal_moving_timeout_does_not_reuse_first_goal_evidence` 调用 `second_goal_timeout → settle` 时，期望 inspecting，实际已 canceling，原因 `inspection_wall_timeout_incomplete`。该 fixture 使用 2 s inspection timeout；失败发生在准备首个已验证窗口时，尚未到测试原本要检查的第二个移动目标阶段。因此本次失败不能解读为旧目标证据已经串入新目标。后续同阈值针对性重跑通过；完整测试负载下的停车时间可靠性仍未闭合。

## 失败二：暂停时钟用例的 ACK 等待先于实际取消请求到期

`test_paused_clock_stage_deadline_waits_for_owned_terminal_before_return` 的最终状态为 canceling，原因 `bounded_observation_wall_timeout`，ROS 时钟保持 10.3 s。日志中队列创建时间为 1793448534.624341987，`cancel_requested` 为 1793448539.214572501，间隔 **4.590230514 s**；测试 stage budget 为 3 s，而 ACK 等待到名义 stage 截止加 1 s。实际取消派发已经晚于这个夹具等待截止，测试在刚进入取消阶段后断言未收到 ACK。

未见 `cancel_error`。现有失败快照没有 `cancel_started`、server cancel 回调 wall 或 client ACK 回调 wall，无法确定延迟发生在 timer 调度、请求送达、服务处理还是响应消费，也不能据此认定取消协议失效。失败后的 ABORTED terminal 来自 cleanup 释放受控服务器 gate，不能追认为该测试期待的正常终态。

若将来需要修复夹具，应先记录实际派发/服务端/客户端时间，再考虑把“有界等待取消派发”与“按实际派发时间有界等待 ACK”分开。不能改生产 stage/cancel 阈值、重发取消、提前释放 terminal gate，或把 ACK 当成 terminal。本轮未实施这项候选调整。

## 失败三：两个观察窗已完成，多余传感器 pulse 跨过退休阶段

`test_two_headings_use_real_action_uuid_and_new_dds_windows_then_return` 的两个观察目标都已生成 `inspection_satisfied` 记录，各含三帧、各自 token/UUID 和正确采样 barrier。第二窗最后一帧 stamp 为 13.2 s，里程计 receipt wall 为 319.211840055。

发布第三帧后，测试等待 attempt 数达到 2，内部又调用了一次 `pulse()`：13.3 s 的辅助传感器数据在 wall 319.313587412 发出，same-stamp TF/odom 等待于 319.313977148 开始，当时 phase=inspecting，预算 0.2 s。至 319.527970246 失败时，任务已完成第二窗、记录该 attempt、关闭观察队列并转为返航取消。因此待等待的任务条件已满足，但夹具仍因额外传感器等待失败。

夹具已在阶段/token 改变后不再要求旧里程计被消费，却仍要求这个已无必要的 13.3 s camera TF。这个相位切换竞态与三条实际选定处理帧是否有效是两件事。后续重跑通过，但原竞态仍存在。

最小候选调整是第三条固定处理帧发布后用 `pulse=False` 等待该 attempt 退休，让原 timer/callback 运行而不再发多余 sensor pulse；或者显式记录等待目标已满足、旧传感器等待被取代。两者都不能把未送达 TF 记为已送达，也不能在三帧不足时补第四帧，或放宽里程计新鲜度。本轮未实施这些调整。

## 依赖修复与尚未验证的事项

mission 现在显式依赖 `limo_evidence_protocol`。独立 source 测试需要把该包加入 import 路径；否则完整 discovery 可能偶然依赖先执行的其他测试注入路径。`test_mission_inspection_ros.py` 现已显式设置该 source root；`test_observation_ros.py` 导入这个 helper，因此获得相同设置。安装后的环境仍必须构建并 source 已声明的共享包。这是直接运行测试所需的依赖修复，不改变时序与安全语义。

保留完整测试的三项失败和针对性重跑的成功，分别报告。当前仍未完成完整 suite 在同一最终版本、相同负载下的稳定通过验证；没有因此新增 Gazebo/实车成功、未知目标覆盖、原始图像留证或独立物理静止的结论。
