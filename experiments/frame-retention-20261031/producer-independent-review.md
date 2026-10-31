# Producer retention adapter 独立审查

本轮只读审查 `frame-retention-producer-adapter-draft/reviewed-v1` 的冻结 adapter/worker/node，以及原 final Core 和新增测试。没有改生产或草稿，没有启动 ROS/Gazebo，没有重复运行作者的全部 48 项测试。以下结论是代码审查及明确控制流核对；作者的纯子进程测试不等于 ROS 相机或 mission 留存链验证。

## 需要补齐的两项

**P2：已关闭后的新故障不推进检查点版本。** `ProducerRetention.close()` 在进入 `if not self.closed` 之前可以将 `self.fault` 从 None 改为新的错误，但仅第一次 close 才 `dirty()`。一个正常关闭且最终 checkpoint 已完成的 owner 再接收 fault 时，Core 的 closed/results/ownership 不变，tick 的 Core fingerprint 也不变；revision 仍可等于 persisted_revision。因此磁盘 checkpoint 保留 fault=None，`shutdown.final_checkpoint_persisted` 却继续为 true。这个问题与“旧 stored digest 被故障包装后沿用 checkpoint flag”相关，但只绑定 durable fault 仍不足以解决新故障本身没有被调度持久化的问题。最小修复是故障值改变也推进 revision，不重建 Core 或任何预算；需覆盖正常关闭已 durable 后追加故障的回归。已通知实现代理，其已确认该分支。

**P2：退出未确认时没有把当前子进程身份输出到退出日志。** `shutdown()` 超时返回的完整 snapshot 包含 writer.pid、operation_id、fault 等事实，但冻结 `node.destroy_node()` 仅输出 `report['shutdown']`，main 之后只返回非零退出码。该 worker 的输入 checkpoint 是启动进程前生成的快照，里面 writer 可以为 null，即使 Core 的 writing=true；它不能记录尚未创建的子进程 PID。如果这个子进程阻塞、最终 checkpoint 无法更新，现有日志及旧 checkpoint 可能都缺少外层核查所需的当前子进程身份。最小修复是在退出日志保留有界 writer snapshot、adapter fault/reason 和现有 shutdown flags；无需打印整份 Core、增加恢复系统或宣称 SIGKILL 一定完成。应有提取 destroy_node 的未确认退出报告测试。该缺口不使存档状态变绿，但与声明的未退出责任核查直接相关。

## 作者已复现的三项，独立核对代码后认同

- `OwnedWriter.poll()` 首次 drain 后子进程可能写入最后 reply 并退出；随后 poll 看到 exit 0，而内存 output 仍空。确认退出后再次非阻塞 drain 是必要的，不能在未收齐管道结果时认定 reply 无效。
- 合法 JSON 但非 dict 的 reply 会在 `.get()` 抛未捕获 AttributeError；应验证 dict、严格 bool ok 及相应身份/字段，错误仍必须保留为失败而非绕过 owner 退出处理。
- adapter 全局 fault 把已 stored 结果包装成 failed 时，不能仅靠旧 Core result digest 保留 checkpoint_persisted=true；持久状态匹配也应包含该 fault。这个修复还需要上面的 revision 更新，才能实际调度新检查点。

这些是作者在 `test_owner_races.py` / `races-before-repair.log` 提供的确定性反例，本报告没有把它们表述为独立重新执行的测试结果。

## 未发现额外阻断的主要路径

- ROS on_pair 的实际 color/depth 消息，经 FrameEvidence.finish 后以 memoryview 送入 Core.offer，再发布同一 metadata；没有订阅第二份图像或用后来的帧补洞。仅 processed 帧进入缓存，Core 仍校验 raw hash、padding/layout、config/calibration 并取得不可变副本。
- Core 由构造 owner 的线程操作；main 用同线程 spin_once，writer 只收到不可变字节，不调用 Core。clock hook 用 Event 把留存失效交给 executor。不能把此契约外推为任意 MultiThreadedExecutor 都安全。
- 一次只有一个 Popen 对象；stdin/out/err 为非阻塞、有界 transfer/保留量。信号用当前 child 的 pidfd，不按名称或任意进程组终止。确认退出之前不调用 Core.finish 释放 writing 所有权、不启动下一 writer。
- timeout/close 会保持 queued/writing 的失败与责任；迟到完成不会取消 Core 的故障。shutdown 对无法确认退出明确报告 false，文档也保留不可中断内核 I/O 的限制。
- 文件名与 digest 目录来自 owned Job，原帧文件 exclusive create；checkpoint 使用固定 pending 文件、file fsync 和 atomic replace。`stored` 的持久 gate 需等结果状态进入后续 checkpoint，而不是第一次写完原图即发布成功。
- 新输出目录必须不存在；同 run 不因 epoch/clock 失效重新建 Core 或退还文件预算。checkpoint 只保留有界结果/历史，不是文件真实性的独立证明。

尚未验证真实 DDS 相机消息配对、executor 在真实图像负载下的延迟、native I/O 故障、source→mission accepted 清单、raw artifacts 闭合审计或进程外监管。payload 上限不是父子进程总 RSS 限额。该草稿未接入生产，不应改写 run20 原来的缺图结论。

## 冻结审查身份

| reviewed-v1 文件 | SHA-256 |
| --- | --- |
| retention_adapter.py | `8d38b869aa7763dcabae70fc25038fd75e5a7007094ee3e686380015db34468e` |
| retention_worker.py | `da83fa982745b5b370ac298526a706b8664ea0597b02999726c0728c13027561` |
| node.py | `7fd006d166595cf6bd850691cab54dcc438e5414e21907040334bf3a2d791180` |
| final Core retention.py | `ac2a885da8afc8b69710ed6caa1a7967221ce55421aa18a1eefc06c2a7ea0d2c` |

实现代理正在新版本修复上述项目。修复后需按新 SHA 做增量复核；此报告尚不宣称新版本通过。

## 最终增量复核（2026-10-31）

已按下列实际文件 SHA 比较 reviewed-v1 与最终草稿。上述五项均由最小改动闭合：确认子进程退出后再 drain；拒绝非 object/非 bool ok 并清空无效 reply；持久结果同时绑定 durable_fault；已经 closed 后首次新 fault 也推进 revision；退出日志保留有界 writer（含 owner_token）、fault、reason 与 shutdown flags。对应的确定性 race 测试、closed 后二次持久化测试及提取 destroy_node 的退出日志测试覆盖原反例。没有发现本轮增量带来新的功能阻断。

本次独立工作为静态增量复核，未自行运行全部测试或启动 ROS/Gazebo。已读取作者的 all-tests-04.log：53 项纯测试，26.846 秒，OK；这是作者执行的证据，不能替代待进行的真实 DDS 留存集成验证。保留前述单 owner、退出无法绝对有界、checkpoint 不是独立原图审计或 writer-exit 证明的限制。

| 最终 reviewed 文件 | SHA-256 |
| --- | --- |
| retention_adapter.py | `a1f4d6aa5452dedc4f517bd96cbff8ea9d5f0df0bb805a9b660a4f27c3a5bcc8` |
| retention_worker.py | `da83fa982745b5b370ac298526a706b8664ea0597b02999726c0728c13027561` |
| node.py | `f1bb1b7ed5b76916ffdb2c46bf64bd3b150c0e8bec21a673e84be9d1c2cdccc0` |
| test_adapter.py | `fe1cee1d2aa9d755ed0c5c45c41e06b495ddfb7f54be620be6f409850acfd1ad` |
| test_owner_races.py | `913c8877facdb4943b115d92428cab28ca491e103c26ed31c0602a8b15c40c88` |

原 Core 未改（`ac2a885da8afc8b69710ed6caa1a7967221ce55421aa18a1eefc06c2a7ea0d2c`）。生产集成改变 import/package 后，应另记录生产源码身份；本结论只对应此处冻结草稿。

随后仅复核了 main 默认关闭兼容修改：old_signals 初值为空 dict，threading.Event 在启用 retention 分支才创建，未启用时仍用原 rclpy.spin；最终报告用 getattr 保留缺省 fixture 兼容。该增量不改变启用链的 owner、终止或持久 gate。对应最终 draft node SHA 更新为 `c721a46c8eab04dcb15c854e64ed18b1eb08acbaddf760c58e693b9476f86dbd`；53 项作者测试在这几行兼容改动之前执行，迁移后的旧生命周期测试仍由作者补跑。
