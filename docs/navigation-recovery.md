# 导航失败后的有限恢复等待

run28 出现 TF 外推后，Nav2 立即终止目标，任务在同一 tick 或紧接的 tick 派发下一目标，最终快速耗尽候选与返航次数。新增默认关闭的恢复等待，用于验证这种连续重试是否可以避免。它不更改 Nav2、SLAM 或位姿 TF 的有效性阈值，也不把 ABORTED 一概归因为 TF 故障。

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `navigation_failure_cooldown_s` | 0.0 | 失败后最短墙钟等待；0 完全关闭此功能 |
| `navigation_recovery_timeout_s` | 3.0 | 从失败处理开始计算的恢复墙钟上限 |

两值须满足 `0 <= cooldown < timeout <= 30`。在既有完整任务命令中添加：

```bash
-p navigation_failure_cooldown_s:=1.0 -p navigation_recovery_timeout_s:=3.0
```

使用 `mission.launch.py` 时，传同名 launch 参数而不加 `-p`。完整运行仍须先启动模型、SLAM/Nav2、感知及所需初始观察；这些参数本身不是完整启动命令。

恢复只在已确认失败终态或明确拒收后建立。任务记录此时 `map→odom` 的最新时间戳，随后继续处理 ROS 消息；只有最短等待已过且该时间戳严格推进，才允许派发。成功目标和正常主动取消不触发等待；导航超时引发取消时，仍先确认动作结束。状态未知或通信异常继续保留目标所有权，不能借恢复流程绕过取消检查。

启用恢复后，任务定时器使用单调墙钟；即使没有开启停车观察，也不会因 `/clock` 暂停而无限等待。原任务、返航、观察阶段的期限优先，不因恢复重新计时。TF 查询前后、就绪日志后及真正派发前均检查预算；等待不会消耗尚未派发的导航 token、返航次数或观察队列次数。超过恢复期限则以 `navigation_recovery_timeout` 阻断；原时钟倒退、无效/过旧 TF、地图失效检查保持有效。

`goal_terminal` 事件新增 `status`、`error_code`、`error_msg`，旧 action IDL 缺失的字段保存为 null。状态及结果中的 `navigation_recovery` 保留失败 token、原截止时间和 TF 时间戳；事件 `navigation_recovery_started` / `navigation_recovery_ready` 用于解释等待。ready 仅表示通过恢复条件，之后仍可能因原预算结束而不派发。

本地验证：新增 18 项调度回归通过；受影响旧回归运行 168 项，167 通过，1 项因纯 Python 环境无 ROS 而跳过。真实 Jazzy domain 74 的独立 Action/DDS 用例通过：服务端报告失败后暂停仿真时钟，任务仍按原 3 秒墙钟恢复上限阻断，没有派发第二目标，fixture 完成关闭。该测试使用合成地图、TF、里程计和受控 ActionServer，没有运行 Nav2/Gazebo或验证车辆实际制动。

跳过的 ROS 启动用例随后单独通过（1/1），包括实际节点构造、参数默认值与覆盖值、spin 和关闭。任务包构建成功，安装后的 node/launch 与源码逐字节一致。原始日志与源码摘要见 [恢复逻辑验证档案](../experiments/navigation-recovery-20261103/README.md)；分次验证不会改写原日志中的 skip。

任务缓存中 TF 时间推进不保证 Nav2 的独立缓存已同步。随后 [run29](../experiments/run29/run29-outcome-review.md) 只启用 `1.0/3.0`，沿用 run28 的 RPP 等待 0.5 s、任务 TF 年龄 1.5 s及其他预算，并读回 local costmap 容差 0.3 s。11 次恢复均 ready，最后一次返航成功，但仍有 11 次 TF 导航失败，完整探索未完成。因此该等待机制已在 Gazebo 触发验证，尚不能称为 TF 根因修复；默认仍关闭。
