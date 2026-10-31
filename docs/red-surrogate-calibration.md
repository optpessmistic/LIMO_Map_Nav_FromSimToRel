# 红色替身的可重复标定场景

`tools/build_red_calibration.py` 生成 12 个静态 Gazebo 场景，用于测量目前红色 RGB-D 检测器的距离、角度、遮挡与实例区分能力。识别接口保持可替换；正式课程物品尚未确定。

| 场景 | 要测的问题 |
|---|---|
| 空场景 | 无目标时是否误报 |
| 正面 0.4 / 0.8 / 1.6 / 3.2 m | 检出率、有效深度及表面定位误差随距离的变化 |
| 0.8 m、方位 ±35° | 靠近图像边缘时能否检测 |
| 表面入射角 45° / 75° | 斜视导致像素减少与深度变化的影响 |
| 大红色平板 | 红色像素是否被误当成指定薄盒替身 |
| 间距 0.18 m 的两个红盒 | 检测及地图融合能否区分相邻实例 |
| 部分遮挡 | 可见部分减少时的漏检与定位偏差 |

距离指相机中心到红盒正面中心的水平距离，不是光轴深度。正样本沿用当前迷宫的 0.03×0.06×0.20 m 红盒，背靠灰色支撑。大红色平板只在这个受控薄盒实验中定义为负样本；它不替代正式物品定义，现有颜色检测器可能将其检出。两个目标的 0.18 m 间距小于当前 0.30 m 融合半径，刻意用于暴露实例合并问题。

## 生成

在仓库根目录运行。下面使用本机 F 盘已有模型源码；其他工作区把两条 Xacro 路径改为自己的 vendor 路径。

```bash
/usr/bin/python3 tools/build_red_calibration.py \
  --output outputs/red-calibration/new-batch \
  --model-xacro /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/vendor/hunter_gazebo/src/limo_base/urdf/limo_four_diff.xacro \
  --sensor-xacro /mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/vendor/hunter_gazebo/src/gazebo_harmonic/urdf/limo_gazebo.xacro
```

输出目录必须尚不存在。`worlds/` 保存 SDF，`offline_truth/` 保存独立评分真值，`manifest.json` 记录源码与场景摘要、样例清单和几何检查。机器人节点只订阅传感器，不读取真值目录。

生成器从当前支持的固定相机结构提取名义位姿和内参；不能解析的相机旋转、额外 sensor/camera pose 或自定义投影会报错。每个场景检查物体实体相交、起点净空、背板贴合/承托、目标朝向、视锥裁切及解析射线遮挡。实际物理沉降、渲染自遮挡和有效深度仍须在 Gazebo 中测量。

## 验证状态与下一步

24 项纯测试覆盖正常场景及破坏几何的反例。12 个场景已用现有 Xacro 生成并通过解析检查；**尚未完成 12 场景的真实渲染和检测实验**。manifest 因此保留 `status=designed_not_run`、`effective_detection_range=null`，不发布有效视觉覆盖声明。

后续每场景用全新服务器与输出目录，先检查模型唯一性和传感器，再保存同步 RGB、深度、CameraInfo、图像时刻 TF、检测及处理帧记录。按预先固定的采样窗统计误报/漏报和定位误差，同时保留失败样例；完成后才能选择有效检测范围与观察站位间距。解析可见面积不等于实际检测率，也不授权完整搜索成功。
