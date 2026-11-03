# 主动观察与同帧关联：整合验证（2026-10-31）

新增默认关闭的有限观察队列、返航后前沿复查，以及同帧目标一对一关联。本目录保留实际失败与通过记录；源码版本由同次 Git 提交及各 JSON 的 source_before/source_after 定位。

| 记录 | 发现测试数（含跳过） | unittest 秒数 | 退出码 |
|---|---:|---:|---:|
| `observation-association-pure-01` | 342 | 12.605 | 1 |
| `observation-association-pure-02` | 342 | 12.044 | 0 |
| `observation-action-dds-01` | 2 | 5.917 | 1 |
| `observation-action-dds-02` | 2 | 22.23 | 1 |
| `observation-action-dds-03` | 2 | 25.629 | 0 |
| `observation-association-full-ros-01` | 343 | 72.534 | 1 |
| `observation-association-full-ros-02` | 343 | 79.556 | 0 |

完整统计以每份日志的末尾为准。pure-01 的失败来自旧 AST 夹具缺失新状态/方法，pure-02 补齐夹具后通过；原有断言保留。DDS-01 暴露真实 ROS UUID 元素为 uint8、无法直接 JSON 序列化的生产缺陷；修复为内置 int 后，DDS-02 的暂停时钟取消用例通过，但双朝向用例触发停车证据失效。DDS-03 的夹具加入逐次 Clock、TF/odom 消费屏障与初始化预热，两项通过；没有放宽生产停车阈值、替换 mission 回调或通过额外图像补齐失败帧。DDS-02 未记录失效时的所有消息年龄，不能据后续成功断定它的唯一原因。

新 DDS 测试使用真实 rclpy ActionServer、UUID、取消/终态消息和 DDS 传感器消息。双朝向各接收恰好三帧合成空白 RGB-D 的真实 FrameEvidence 后返航；暂停 /clock 用例确认墙钟截止、取消 ACK 仍保留目标所有权、CANCELED 终态后才能返航。地图、TF、里程计及 Action 执行结果由测试夹具提供。没有运行 Nav2、Gazebo、物理运动、GUI 或真实物品识别，没有验证全场景视觉覆盖或原图留存。

full-ros-01 发现 343 项，唯一失败是在 canceling、尚无停车窗口时恢复 /clock 的夹具 0.2 秒送达等待；日志保留时钟尚未消费的诊断。后续仅将初始化/导航/取消阶段的夹具等待上限改为 2 秒，settling/inspecting/home_verification 仍为 0.2 秒，生产 0.3 秒里程计新鲜度要求和所有动作截止未改。不重发消息、不补帧，不将这次延迟唯一归因为 DDS 或主机调度。

build.log 对应一个 mission 包构建；installed.json 记录实际 import 的安装路径及摘要，四个模块与 launch 文件已 cmp 相等。review 文档的各阶段结论有自己的来源摘要，不将早期纯测成功当作当时的真实 ROS 成功。

association-captured-case11-final.json 是此前封闭标定数据的回放：六帧每帧两个相邻检测，原逐项加入得到一个确认轨迹，新逐帧一对一关联得到两个，正/反序一致。它验证已录制观测的关联行为，不证明物理实例数、定位精度或识别召回。原始标定证据见 ../red-calibration-deferred-20261031。

所有日志/报告按原始字节复制，SHA256SUMS 可核对。本轮不改变 run20 的原图缺失事实。后续仍需精确接受帧留存、新自主 Gazebo 实验、覆盖策略及实车验证。
