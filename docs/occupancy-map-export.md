# 从已捕获地图导出 PGM/YAML

`tools/export_occupancy_map.py` 只读一次保存的 JSON，以其中 `snapshot.valid=true` 的 OccupancyGrid 生成独立 Nav2 trinary 地图。它不启动 ROS、不调用地图服务，不修改源报告，也不把原 `save_map` 的失败改成成功。

```bash
mkdir -p outputs/maps
python3 -B tools/export_occupancy_map.py \
  --snapshot outputs/ros_smoke/run22-isolated-writer/final-save.json \
  --output-prefix outputs/maps/run22-captured \
  --free-thresh 0.25 --occupied-thresh 0.65
```

成功输出 `.pgm`、`.yaml`、`.provenance.json` 和防止并发重用的 `.reservation.json`；CLI 输出 JSON、退出 0。前缀本身或任何 `前缀.*` 已存在时拒绝覆盖。写入/回读失败会退出非零、尽可能保存 `.failure.json`，保留部分文件供检查；换用新前缀，不能把缺少有效 provenance 或带 failure 文件的产物当成成功地图。

输入可为完整 capture 报告或 `{valid, errors, map}` snapshot。工具独立校验非空 frame、整数时间戳、正有限分辨率、有限坐标/范围、维度、数据长度和 `-1..100` 整数单元。拒绝不能写进二维 YAML 的非零 z、roll/pitch（浮点容差 1e-9），以及非单位四元数；允许平面 yaw。读取上限 64 MiB、单元上限 400 万，超限明确失败。

分类遵循 [Nav2 1.3.13 的 trinary 保存/重载实现](https://github.com/ros-navigation/navigation2/blob/f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501/nav2_map_server/src/map_io.cpp)：输入阈值乘 100 后最近偶数舍入；`-1` 保持未知，`值≤free` 为空闲，之后 `值≥occupied` 为占用，其余为未知。默认边界为 25 / 65，对应灰度 254 / 0 / 205。未知不填充、不平滑；中间概率变为未知，原始概率仍保存在源 JSON。

CLI 阈值按字面值解释，包括 0；这里不采用 Nav2 保存服务将 0 当作“未指定”后替换默认值的参数约定。

**输入分类阈值与 YAML 重载阈值不同**：YAML 固定 `free_thresh: 0.196`、`occupied_thresh: 0.65`、`negate: 0`、`mode: trinary`。若误将 0.25 写作 YAML free 阈值，灰度 205 会被重载成空闲。图片按行上下翻转，保持 `grid[y*width+x]` 的 y=0 为底行；YAML origin 是 `(0,0)` 单元左下角的米制 `[x,y,yaw]`，没有半格平移或左右镜像，保留数值精度。图片路径仅为相对于 YAML 目录的文件名。

工具写完后重新打开实际 PGM/YAML，检查头、尺寸、像素数、元数据与相对路径，并按 trinary 规则恢复全部栅格，与期望分类逐格比较。provenance 包含源文件 SHA256、map 时间/frame、完整原始地图元数据、输入/输出阈值、输出文件哈希及回读结果；YAML 本身没有 frame 字段，使用 map_server 时应配置 provenance 记录的 frame。

此验证证明捕获栅格的文件导出和分类一致性，不证明完整建图、视觉搜索、导航可行性或任务成功。后续运行可以把这个独立导出步骤与 posegraph 序列化分开记录；某一步失败也不应省略已计划的终态静止验证。
