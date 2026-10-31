# 已闭合 case11 保存观测的关联回放

本次只读取 `outputs/red-calibration/deferred-batch-12-01/11_close_pair_180mm` 的已闭合证据。owner 明确确认闭合后才执行；脚本再次核对 result.status=captured、all_owned_groups_empty=true。没有读取仍在运行的 case12，没有 ROS/Gazebo、world/target truth 输入，也没有重新运行检测器。

输入六个固定原序号的 source stamp 为 6.4、6.5、6.6、6.7、6.8、6.9 s；每帧保存的 candidate_count=2、detection_count=2。全部原 RGB/Depth bytes、同戳两份 CameraInfo 元数据、precleanup checkpoint 的文件哈希链、保存的 detector/capture 源码快照均匹配。使用各帧保存的精确整数戳 `odom <- camera_optical_frame` TF 将两个光学坐标点转换到同一 odom 参考系。未使用 latest TF、地图/世界对齐或事后拟合。

| 算法 | 检测顺序 | 最终 track 数 / 确认数 | 每条命中数 |
|---|---|---:|---|
| 原逐项 add | 保存顺序 | 1 / 1 | 6 |
| 原逐项 add | 反转 | 1 / 1 | 6 |
| 候选 add_frame | 保存顺序 | 2 / 2 | 6、6 |
| 候选 add_frame | 反转 | 2 / 2 | 6、6 |

两保存观测点在六帧中相同，odom XYZ 约为 `(0.884000131, +0.090203225, 0.106578770)` 与 `(0.884000131, -0.088105476, 0.106578770)`，XY 间距 0.178308701 m。原算法仅保留每帧先到的点，顺序反转后保留点改变；候选对两种顺序保留完全相同的 UID 和位置，两条记录均在第三帧满足三次确认。

这是这些**保存观测的关联行为**，不是新在线实验、检测准确率或目标位置误差。case 名称中的数字没有作为算法输入；没有读取物体数量/中心位置真值。保存的两个检测仍可能受分割与身份歧义影响，本回放不证明物理物体数量。候选使用的 `offline_capture:<session>` 只是本回放隔离键，不是测得的 map epoch。

回放命令退出 0。输出 `captured-case11-v1.json` 保存逐帧观测、TF、分配和状态，以及 30 个输入文件的 SHA-256，共 13,119,078 个唯一输入字节（完整性复读不计入这个唯一大小）。其 SHA-256 为 `2ed7fa9a065dd2d3a2a357a4f3e011e2bdb3e959a65bb25182658a39d3e27499`。原样本、原冻结候选和生产均未改写。
