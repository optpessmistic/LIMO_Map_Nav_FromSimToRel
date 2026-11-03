# Observation × association 合并生产源码独立审查

结论：本次直接审查工作区中已合并的生产源码，184/184 项纯测试通过，其中 3 项为独立添加的合并交叉场景；静态/纯测试没有发现下述真实 ROS 类型问题。首轮真实 Action DDS 测试暴露的 UUID JSON 序列化缺陷已修复并独立核对；**随后 DDS03 为 2/2 通过，root 的 full-ros-02 为 343 tests / 79.556 s / OK（skipped=1）/ exit 0，生产源码前后一致。** 之前 DDS02、full-ros-01 的失败原始证据保留；成功范围限于测试覆盖，不代表完整 Gazebo 迷宫或原图留存验收。本审查没有修改生产源码、启动 ROS/Gazebo 或运行真实 Action server。

这不是以原草稿 review 代替合并审查：读取了 `git diff`、合并后的完整相关函数，并对照了合并版与已冻结 continuation 版的差异。生产 `node.py` 的合并增量明确包含批量目标适配、`TargetStore.frame_epoch` 保护，以及坐标失效时在 terminal phase 也清除旧 marker 的修复。

## 真实 Action DDS 首轮失败：UUID 元素不保证为 Python int

已只读核对 root 生成的 `outputs/setup/observation-action-dds-01.log` 和 `.json`。两个真实 ROS Action DDS 测试均 errors，测试报告为 **2 tests / 5.917 s / exit 1**；运行来源 node SHA 与下表初版一致。两条 traceback 均在 `publish_status → json.dumps(value)` 抛 `TypeError: Object of type uint8 is not JSON serializable`。

对应初版生产 `on_goal_response` 以 `list(handle.goal_id.uuid)` 保存 `nav_goal_uuid`。真实 ROS UUID 的元素为 NumPy uint8，list 只转换容器，未转换元素；已有纯 fixture 的 UUID 元素全部是 builtin int，因此未触发失败。

首轮失败证据必须保留，不以之后通过覆盖。本问题不改变下述 epoch/cancel/deadline 控制流审查，但在真实 observation action acceptance 后可使状态发布抛异常，是生产运行的实质缺陷。新版本确认前不得把“184 纯测试通过”写成“真实 Action 生命周期已通过”。

### UUID 修复与第二轮真实结果

已核对生产 `on_goal_response` 改成 `[int(value) for value in handle.goal_id.uuid]`，新 node SHA 为 `33c2316d1a32fc90b78843ac34e9dec81208e5b00ab7f28fc9a4c39e16c3835e`。新纯回归 `test_ros_integer_uuid_is_serializable_in_active_and_completed_status` 使用非 builtin 整数 UUID，检查 active 和 completed 状态。我独立单项执行 **1 test / 0.009 s / exit 0**。该改动只规范化审计字段，不改变导航 UUID 本体、取消 ownership、epoch 绑定或 deadline 分支。

已只读核对 root 的 `observation-action-dds-02.log/.json`：source before/after 均为上述新 SHA，**2 tests / 22.230 s / 1 failure / exit 1**。不再出现 UUID TypeError，但 `test_two_headings_use_real_action_uuid_and_new_dds_windows_then_return` 在首条处理帧断言 `accepted_frames: 0 != 1`，之前日志有 `stationary_odometry_unavailable` 导致窗口 reset。目前仅据日志不能断定是 fixture 时序还是生产问题，不能将第二轮改写为通过；保留等待 root 后续诊断的新证据。

## 重点结论

### Epoch、坐标与两个数据通路

- `on_observations` 先检查 map epoch readiness、coordinates validity、blocked/done 和取消目标为 blocked；失效后不会查询 TF、改写 targets 或发布新标记。整帧先完成输入转换，再调用一次 `TargetStore.add_frame`，同帧的两个近距离独立检测不会被顺序逐项合并。
- pre-home 已绑定 target epoch 后切换 epoch 会停止任务，保留旧 target 数据供审计，并清除 marker。`block` 在 blocked/done 的早退之前处理坐标失效，因此已停止调度也不会继续显示旧坐标为有效结果。它不重启已终结任务，也不改写既有结果文件。
- 有界观察期间 epoch 变化，`on_epoch` 先快照旧 request 的部分 inspection，再改变 Ledger。队列失效保留当前 Action ownership；pending send、已接受 handle 不会因清除目标显示而被遗忘。
- 目标关联通路不写 `InspectionLedger`；确认两个目标不能满足处理帧窗口。反向也成立：处理帧 satisfied 不证明目标 observation 通路已收到消息。目标数量和位置不参与 `ObservationQueue` 候选生成或导航授权。

### 导航取消与到期

- `on_cancel_response` 只设置 ACK；`on_goal_result` 只在 SUCCEEDED/CANCELED/ABORTED 时建立 terminal。pending send 和 result-channel exception 均保留未知 ownership。取消闭合要求 terminal，ACK 不会让队列退休当前动作或派发下一个。
- `send_goal` 在消耗导航 token/home attempts 前重新读取 wall，覆盖规划跨 mission/stage/reserve/绝对返航截止的情况。未真正发送的队列 reservation 保留 canceled 记录，Nav 字段为 null，不回填有限次数。
- home 最终 planner 返回后重新读取 wall，并在 map-complete、resume 或 `done/save_result(True)` 前检查绝对返航截止；之前发现的“晚到终态”旁路已存在于合并生产代码中的正确位置。
- `return_deadline_wall` 只在第一次取消闭合并进入 returning、且截止仍为 None 时建立。补探索沿用原 mission 起点、绝对返航截止和全局 home 次数；它不重建观察队列。
- 正常观察阶段 wall budget 到期的退休入口只结束当前有效 request，不重新授权 epoch。处理窗口 timeout 的继续入口另要求 owned SUCCEEDED/UUID、完整绑定、当前姿态与持续新鲜停稳里程计成立，且仅缺处理帧；其他 fault 仍 block。

### 默认关闭与确认语义

- Node 和 launch 中 `post_mapping_observation_mode` 均默认 `off`，`autonomous` 仍默认 false；默认模式不建立观察队列、绝对返航 budget 或有限观察状态，并沿用旧的 inspection timeout 阻塞行为与返航尝试重置。
- “默认 off”不表示逐项目标融合旧行为保留：逐帧关联、整数 acquisition stamp 检查、原子批量拒绝和 target epoch 守卫在两种模式都生效，这是此次目标关联改动的明确范围。空的合法目标帧也会绑定 consumer epoch，pre-home epoch 切换因而可能要求新任务。
- 有界观察始终保留 `visual_coverage_complete=false`，`mission_success` 不会因有限队列或非零目标数被置 true。连续三个处理帧仍不等于原始图片独立留存通过。

## 实际执行的验证

新增 `outputs/setup/review-observation-association-merged.py`，只读导入生产纯 core/Ledger/queue，并通过已有 AST fixture 执行合并后的真实 `MissionNode` 方法。它增加三项交叉验证：

1. 两个相距 0.18 m 的目标已确认；观察目标仍 send-pending 时发生 epoch 变化。随后模拟迟到 acceptance 和 cancel ACK：当前 handle/action 保留，只有真实 terminal 后退休为 canceled，原 targets 仅保留审计，marker 清除一次。
2. 两个目标已确认，但 inspection accepted count 仍为零。实际接收一条合格 processing record 后发生 epoch 变化：attempt 保留旧 epoch 的一帧证据，不能 credit success，Ledger 与坐标失效。
3. mode=off、pre-home 已确认两个目标后 epoch 改变：没有观察队列、绝对返航截止或导航发送，目标 epoch 守卫仍执行。

同次测试执行所有当前 observation adapter/continuation/final-repair/queue/repair、mission inspection、target frame association、target epoch cleanup 纯测试，以及 `test_ros_mission_core` 除 `RosStartupTests` 外的测试类。

**184 tests / 4.259 s / 0 failures / 0 errors / 0 skips / exit 0。** 明确不加载 `test_observation_ros`、`test_mission_inspection_ros` 的运行测试，不执行 `RosStartupTests`。未运行 shell sourced ROS 环境，也未初始化 ROS context。

证据：

- `outputs/setup/observation-association-merged-pure-tests.log`
- `outputs/setup/observation-association-merged-pure-tests.json`
- `outputs/setup/review-observation-association-merged.py`

以上新证据均 exclusive 写入。测试前后生产源码 hash 一致；其后再读 hash 仍一致。

| 生产文件 | SHA-256 |
| --- | --- |
| `core.py` | `6e56ed052df3c08da6d413de3f8a455fbcf4f683706f3ee1ebcb5b0f2a30ac8e` |
| `node.py` | `1ad1bb1b8c3410197e0c45fee05f6a1466b3fc1530dac36b0ec73415ecbb0d22` |
| `inspection.py` | `5e705ec7fc327e62b1182d166f52998a8180c3ca27674a07fdc6fef16116a108` |
| `observation.py` | `0c21d87865317767c0fe9cb8a355f48ab0f67a63c7b7bdf19e5f96b5e8e7f9c0` |
| `launch/mission.launch.py` | `800d6a64dfe3fabf7e98e56498ca8e5ce3ec06d96cbccc6144142a72c0f9fe04` |

## 保留的限制

当前目标 observation v1 publisher 仍不提供 producer epoch/session；consumer 对缺省 map_epoch 使用自己的当前 epoch。故 `TargetStore.frame_epoch` 是任务侧坐标生命周期保护，**不是端到端 producer epoch 认证**。实际 processing evidence 的 epoch/producer identity 是另一条接口，不能在报告里合并宣称。该限制并非此次合并引入，未来 transport/留存适配应明确关联身份。

纯测试不能验证 DDS 时序、Nav2 真实 action 生命周期、native 物理停车、传感器渲染或原始图像留存。几何近邻一对一匹配也不能证明物体身份、识别准确率或所有未知目标都已发现。真实 ROS 测试由 root 执行，本报告的追加核对以已闭合日志/JSON 为依据。

另只读核对了当前 `docs/bounded-observation.md`：绝对返航截止的起点/不重置，以及“停稳但帧不足才允许 timeout 续下一视角”的描述与本次源码一致。

## DDS 第三轮追加核对

已只读核对 root 执行的 `outputs/setup/observation-action-dds-03.log/.json`：**2 tests / 25.629 s / OK / exit 0**（外层计时 27.771 s）。生产 `node.py` before/after 均为 `33c2316d1a32fc90b78843ac34e9dec81208e5b00ab7f28fc9a4c39e16c3835e`；fixture 为 `62398a3c4aad96467b505ea4131a045c9e18f3830036b7b810a1cb4aa4f0aeea`。该轮加入 Clock→TF/odom 实际送达屏障和提前图像模块初始化；未改生产安全阈值、重发目标或补采帧。

本轮证明这两个真实 DDS/Action 用例在该 fixture 下通过，并消除了此前未闭合的测试状态；不能据此追认 DDS02 的唯一失败原因，也不覆盖 Gazebo 物理停车、原图独立留存或完整未知迷宫任务。DDS01、DDS02 的原始失败证据仍保留。本子任务未启动 ROS。

## ROS 完整测试第二轮追加核对

已只读核对 `outputs/setup/observation-association-full-ros-02.log/.json`：**343 tests / 79.556 s / OK（skipped=1）/ exit 0**，外层计时 90.207 s。唯一 skip 是未启用独立 initial survey ROS opt-in。生产 `node.py` before/after 仍是 `33c2316d1a32fc90b78843ac34e9dec81208e5b00ab7f28fc9a4c39e16c3835e`。fixture 非到位后的送达等待改为 2 s；post-arrival 等待仍为 0.2 s，生产阈值和控制逻辑未改变。

首轮 `observation-association-full-ros-01.log` 的 fixture 送达断言失败继续保留，不用第二轮成功替换失败记录，也不单凭重复运行推定 DDS02 唯一因果。此后 consumer 留证草稿的离线测试是不同范围，不能计入这 343 项生产测试或宣称它已被 ROS 验证。
