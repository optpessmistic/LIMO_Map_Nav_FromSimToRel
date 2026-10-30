# RGB-D 图像处理证据

此功能为后续自主观察提供输入证据：区分算法已处理的 RGB-D 帧、有效空检测、无法定位的候选，以及被拒绝的输入。它不证明物品不存在，不声明视觉搜索覆盖，也不改变正式目标定义或任务成功条件。

## 启用与边界

新输出默认关闭。完成感知包构建，在已有仿真和地图会话中显式启用：

```bash
source scripts/env.sh
ros2 run limo_maze_perception red_rgbd_detector --ros-args \
  -p use_sim_time:=true -p enabled:=true -p depth_registered_to_color:=true \
  -p publish_frame_processing:=true
```

`depth_registered_to_color` 仍是已核验注册的运行前提，不能仅凭两图尺寸相同就设置。新证据使用 `/mission/frame_processing` 的 `std_msgs/String` JSON；现有 `/mission/target_observations` 格式保持不变。两路发布不是原子事务，处理证据不证明目标消息已被任务节点接收。

mission 现在可通过默认关闭的 `require_processed_inspection` 订阅此主题，在到达后停稳并等待新处理帧，详见[停稳观察等待](settled-inspection.md)。尚未新增主动观察站位/朝向队列；记录处理帧不会写入 `Coverage.frames`，不会自动使 `visual_coverage_complete` 或 `mission_success` 成立。

## 记录内容

每条记录含 schema 版本、固定 source、每次节点启动新建的 `detector_session_id`、递增 `sequence`、所见 `map_epoch`、`status` 和拒绝原因。`sequence` 是处理尝试序号，不是合格图像数。

RGB、深度分别保留采样时刻的纳秒整数、光学 frame、编码、宽高、step、字节序、原始数据长度及 SHA256。摘要计算于原始 `Image.data` 字节，包含行 padding，未先经过 cv_bridge 转换。不同时间戳的相同像素内容可以是静止场景中的正常新帧；图像哈希不用作时间去重。

CameraInfo 保留采样时刻和完整 K/D/R/P、尺寸、ROI、binning、畸变模型及稳定的 `calibration_id`；摘要不包含随帧变化的采样时刻。检测配置及其摘要另存。回调时刻和 `processed_stamp_ns`（本次处理尝试结束的 ROS 时刻）与图像采样时刻分开，不用接收时刻替代原时间戳；拒收记录也有尝试结束时间，是否成功必须看 `status`。

**摘要不等于原图存档。** 后续实验还需录制实际 RGB、深度、CameraInfo、证据主题及 TF，按采样时间和字节摘要核对。若只保留这个 JSON，就不能事后重跑图像算法或人工复核画面。

## processed 与 rejected

`processed` 只在输入校验通过、算法实际执行返回、全帧有效深度数至少达到 `min_depth_pixels` 后产生。默认阈值 8 仅是最低输入可用性门槛，不能解释成整个视场具有可识别能力。算法只运行一次，同时返回检测结果与统计；原 `detect_red_objects()` 列表接口保留。

统计包括全帧像素/有效深度数、达到颜色和面积门槛的候选数、成功定位数、无法定位数及原因。无检测时：

- `no_red_candidates`：没有达到当前阈值的红色候选，不表示真实物品不存在。
- `all_candidates_rejected`：有候选，但均未通过中心像素数、有效深度、混合表面、内点数或有限投影检查。

部分候选被拒绝而其他候选被定位时，分别保留两个计数。完全无效的深度不能形成合格处理帧。编码、数组/消息尺寸、frame、同步、内参、畸变、ROI/binning 不合格会产生 `rejected`；缺少或不能可靠解释的元数据用 null，不能序列化 NaN/Infinity 为测量值。

新证据的严格判据仅约束这一主题；旧目标输出保持既有规则。因此可能收到旧格式空检测，同时看到该次处理证据被拒绝。未来观察消费者应读取证据的明确状态，不能把任意目标空消息当作合格观察。

## 时间与地图会话

证据生产端订阅 transient-local 的 `/mission/map_epoch`。首次有效代次或代次改变时，以接收公告的 ROS 时刻建立采样屏障；RGB 和 Depth 的采样时刻都必须晚于屏障。重复公告同一个 epoch 不重置屏障。每次已接受处理后，两路各自记录纳秒水位，任何一路重复或倒序都不能再次计为新合格帧。

通过 ROS clock jump 回调和回调时刻检查监控时钟，回退或切换时钟源后锁住成功证据；即使两张图像之间发生回退、随后时间又追上，也不会漏过锁定。需新地图代次才能恢复，相同 epoch 的重复公告不会解除锁。节点重启有新的 session ID。没有 epoch 时不产生 `processed`。这些规则不能检测从未公告的 SLAM 静默重置，仍需遵守现有地图会话协议。

状态只保留当前代次、水位、序号和一次回调的数据，不累积无界图像历史。生产端不查询 TF，也不宣称机器人已经停止或处于某个观察朝向。可选 mission 消费者另行核对采样新鲜度、采样时刻 TF、实际姿态/停车、观察请求屏障和有界等待时间；仅有递增时间戳或 `processed` 不能替代这些条件。
