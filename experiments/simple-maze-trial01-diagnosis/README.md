# 简单迷宫 trial01：地图过期与里程计到达抖动

原任务因 `map_stale` 进入 `blocked`。15 个导航目标均成功，曾满足算法建图条件并进入额外观察，12 个计划视图完成 7 个；故障时第 8 个视图观察未完成。保存了地图和 2 个已确认替身标注，但最终复查未完成，`task_success=false` 保持原值。生产者记录 78 份 stored、fault 为 null；这不是再次发生 writer 超时，也不替代关闭后的严格文件审计。

原条件中里程计 `odom_wall_age_s=0.326208506` 超过 0.3 秒阈值，`odom_ros_age_s=0`、速度样本 stationary。此原因会重置观察窗口；真正阻断原因仍是地图有效回调超过 5 秒未更新。用户确认未主动暂停或休眠。

对仍运行的场景进行了 45.018 秒只读订阅：地图 46 条，最大墙钟间隔 1.0107 秒；里程计 1473 条，最大墙钟间隔 0.30592 秒，2 次超过 0.3 秒；时钟前进 14.724 仿真秒。独立观察器循环最大间隔 0.02457 秒。现场回读双方 use_sim_time=true，SLAM map_update_interval=1，任务地图上限5秒、里程计上限0.3秒。实际 SLAM 使用 steady-map overlay，其地图线程是 WallRate，不能把本次地图过期简单归因于“1仿真秒在低速环境中变长”。

地图断档未在后续采样重现，原报告缺少有效接收间隙和回调耗时，尚不能区分发布、传输或任务执行器阻塞。保存的地图 header 已到297秒，也不能追溯替代故障瞬间的接收记录。

本轮新增 `inspection_odom_wall_max_age_s`，默认0表示沿用原ROS年龄阈值；两个仿真配置显式设1秒。仅为到达间隙提供余量，ROS年龄仍0.3秒，严格递增时间戳、连续停车、最少样本、帧后里程计核对、原观察与任务期限均保留。它修复已复现的过严到达检查，不宣称已定位原地图断档。

地图5秒限制保持；新增 `map_freshness_failed` 事件和结果字段 `last_map_freshness_failure`，记录实际接收年龄、最近8个间隙、最大间隙及回调耗时。地图header时间是源数据时间，不当成发布心跳；回调耗时不包含执行器排队时间。后续再发生即可留下更明确证据。

`summary.json` 来自下面的只读命令，包含源文件哈希；现场连续采样原始数据保留在本机忽略输出中：

```bash
python3 experiments/simple-maze-trial01-diagnosis/diagnose.py \
  /home/jxy/limo_runs/simple_maze_trial_01 \
  outputs/validation/simple-trial01-stale/live-sample.json
```

验证范围见[验证记录](../../docs/validation.md)。本轮没有重新启动或驱动仿真小车。
