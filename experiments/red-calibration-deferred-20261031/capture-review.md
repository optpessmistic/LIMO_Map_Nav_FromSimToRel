# 延后处理采集草稿：独立审查

结论：**冻结稿未发现剩余的阻断性实现漏洞。** 采集循环中的 detector / PNG / fsync 已移出；本结论不是 DDS 丢帧改善或在线场景通过的证据。未改草稿、生产代码或 runner，未启动 ROS/Gazebo。

审查范围：`outputs/setup/red-capture-deferred-draft/{capture_core.py,capture_red_samples.py,test_deferred.py}`、原有两个测试文件及 README；读取正式 runner 的 capture 时限，检查未来接入边界。

冻结 SHA-256 已独立重读匹配：

| 文件 | SHA-256 |
| --- | --- |
| `capture_core.py` | `f739ddbc6a727f23e24106228b5d75121c12c798fd99eab578f2552b052bec67` |
| `capture_red_samples.py` | `1e4acd7198b2dfb82a450acdb92ddc8f2a5d77319a0cde0f74b4c04fb622352c` |
| `test_deferred.py` | `9b17a9d170ec95612e87152cc3a234ad8185aebf7371e4553615ce54c2bb5d3c` |

## 审查中发现并已修正的问题

`persist_frozen()` 原先只在 evaluate 之前检查保存期限。如果一次检测耗尽预算，它仍会开始 PNG、派生文件写入和 fsync，最后才承认超时。已反馈实现者；冻结稿在 `capture_red_samples.py:235` 增加检测后的期限检查，不启动已知超时后的派生写盘。

独立合成反例以 1 s 保存预算、检测后假时钟推进到 2 s 验证冻结稿：`derived_write_started_at_s=[]`，raw checkpoint 存在，`completed=0, pending=1, persistence_complete=false`，报告保存期限错误，没有替换或假称完成样本。结果在同目录 `red-capture-deferred-deadline-repro.json`。该检查未 import rclpy。

## 关键不变量核对

- **固定 N，不按结果补选：** 选择仍只由同步 RGB/Depth 整数戳和 callback availability 决定；槽位在检测前确定，`pending+completed` 上限保持，拒绝/缺校准样本也占槽。冻结后的原始消息 detached，CameraInfo tuple 不被后续修改。保存失败留 pending，未重新选择图像。
- **精确戳与 TF：** 收集阶段为固定图像戳查询三个 TF；没有 latest 查询替代。缺少 CameraInfo 仍可尝试 TF，但输入无效不能通过；clock 无覆盖时明确 `tf_not_queried`，不伪装成 TF 传输缺失。离线处理用冻结的 TF 与 clock/wall 事实，不以写盘完成时间重新判断图像年龄。
- **采集无派生阻塞：** `freeze_ready()` 不调用 evaluate、PNG、writer 或 fsync；`persist_frozen()` 要求 `window.closed`。初始化仍会写 manifest/source，采集过程中仍有有界 metadata/hash/TF 工作，不能宣称 executor 完全无计算负担或无丢帧。
- **中断/错误留证：** 先保存每个已选槽位的原始 bytes、校准及 TF，再保存 collection checkpoint，之后做 detector/PNG；native cleanup 前再保存 raw checkpoint。SIGINT/SIGTERM 请求会阻止检测并标明中断；部分保存、超时、检测异常不会成为 passed。checkpoint 与各样本使用 exclusive create，已写证据不覆盖。
- **内存范围：** 默认 128 MiB 只统计缓存及所选槽位持有的不同 detached 图像 payload，入缓存前检查；耗尽时终止采集而不逐出已选帧。图像单条 8 MiB / 1,048,576 像素、CameraInfo/string 长度、缓存/签名历史均有界。它不是 RSS 上限，复制、数组、Python/DDS/native 内存仍在指标之外；CLI 最大可调到 1 GiB。当前 README 已正确区分。
- **迟到冲突：** 已选戳的四 topic signatures 持续 pin，冻结后的冲突仍影响最终 verdict；冲突观察到 `window.closed` 为止，停止 spin 后的 queued DDS 消息不算已审查。这一有限边界在 collection_boundary/报告限制中显式记录，不能解释为发布端历史无冲突。
- **来源：** 新逻辑全部位于 core/runtime 两份文件，没有遗漏的第三份 helper。`provenance([__file__])` 同时保存这两份源码及当前感知 core/evidence/node；结束检查保持。来源变化不能通过，但检查点明示是在 native cleanup 前执行。

## 接入前需保持的配套条件

正式 `tools/red_calibration/runner.py` 目前给 capture 75 wall seconds；草稿新增独立保存阶段默认 10 s，采集仍为 60 s。接入时应明确传入采集/保存预算，并给外层捕获任务留清理余量（实现者建议 90 s），同时继续受原 case/batch 绝对期限约束。否则临近采集截止才凑够样本时可能被旧外层期限打断；即使 raw checkpoint 已保存，也只能记不完整结果。

内部截止检查位于操作之间，不能抢占阻塞的 detector、文件 I/O 或 native cleanup；内外两层的时间含义不可写成严格进程寿命保证。`collection_complete`、`persistence_complete`、`cleanup_complete`、`passed` 必须继续分开，不能把 collection 成功当作标定通过。

## 测试证据与限制

实现者最终运行 48/48 纯测试，exit 0、6.410 s；`offline-tests-result.json` 的文件哈希与本次独立读取一致。包含旧 36 项以及新增选帧/冻结/内存/中断/保存阶段回归。独立审查没有重复整组测试，另运行了上述检测耗尽预算反例。

仍须通过新的独立输出目录做受控在线验证，才知道这次结构调整是否改善实际采集；不能据此改写既有 case02 或 run20 的失败记录，也不能单凭这些测试认定同步写盘是其唯一原因。物理静止、目标类别、检测范围和完整搜索覆盖均不在本次通过范围内。
