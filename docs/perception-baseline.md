# RGB-D 红色目标仿真替身

`limo_maze_perception` 将同一仿真 RGB-D 相机中的红色连通区域投影成相机光学坐标系下的米制位置，供任务节点按采样时间转换到 `map` 并做多帧归并。目标身份尚未确定，因此这只是红盒替身；它既不代表正式物品识别，也不发送视觉搜索覆盖证据。目标位置、数量以及 Gazebo 模型列表均不输入本节点。

节点默认停用，必须同时显式设置 `enabled:=true` 和 `depth_registered_to_color:=true`。第二个参数是运行者确认深度已注册到彩色图的声明；尺寸相同并不能自动证明注册正确。当前仿真约定同一个 RGB-D 传感器生成两路图像，共用 `camera_optical_frame`。

## 接口

| 方向 | 默认 topic | 类型 / 约定 |
|---|---|---|
| 输入 | `/camera/color/image_raw` | `sensor_msgs/Image`，`rgb8` 或 `bgr8` |
| 输入 | `/camera/depth/image_raw` | `sensor_msgs/Image`，同尺寸、已注册；`32FC1` 米或 `16UC1` 毫米 |
| 输入 | `/camera/color/camera_info` | `sensor_msgs/CameraInfo`，同 optical frame、有限有效内参 |
| 输出 | `/mission/target_observations` | `std_msgs/String`，下列 JSON |

```json
{"schema_version":1,"stamp":123.45,"frame_id":"camera_optical_frame","source":"red_rgbd_sim_baseline","detections":[{"class_id":"red_object","position":[0.1,-0.2,2.0],"confidence":0.9}]}
```

`stamp` 保留彩色图像采样时刻的 ROS 秒数，不替换成节点处理时刻；仿真应设置 `use_sim_time:=true`。`position` 遵循 optical frame：x 向图像右、y 向下、z 向前。输出可以有零个或多个检测。检测节点不分配持久 ID、不查询目标总数、不做 TF；任务节点负责采样时刻 TF、3 帧一致确认与约 0.3 m 空间聚类。`confidence` 只是深度有效比例生成的质量分数，未经识别概率校准。

## 处理与拒绝条件

1. RGB、深度以 `SENSOR_DATA` 的 best-effort QoS 订阅，使用 `ApproximateTimeSynchronizer`，默认容差 0.05 秒。CameraInfo 最长允许与 RGB 相差 2 秒；仿真需持续发布它。
2. 检查有效时间戳、同一非空 frame、注册声明、三个数据源尺寸一致。拒绝未标定 `fx/fy`、非有限内参、非零畸变、未经校准调整的裁剪/缩放。真实相机应先完成注册与去畸变。
3. HSV 红色阈值同时覆盖 hue 的 0° 与 360° 邻域；饱和度至少 0.55、亮度至少 0.30。使用 8 连通区域，默认至少 30 个像素；OpenCV 可用时加速连通区计算，缺失时使用离线小图像备用实现。
4. 仅采样红色连通区中心 60% 范围内的深度。排除 NaN、Inf、零、负值、0.15–6 m 之外的值。至少 8 个有效样本、有效比例至少 50%；中位数和 MAD 抑制少数离群点。混合深度表面离散程度过大时丢弃该检测。
5. 以稳健像素中心和深度 `z` 投影：`x=(u-cx)z/fx`、`y=(v-cy)z/fy`。得到的是可见表面的代表点，不是物体体积中心。有效的空检测也会发布，但不表示这一视场已完成正式目标搜索。

相接红物体可能合并为一个区域；遮挡、光照变化、红色墙面会造成漏检或误检。此替身没有训练模型，也没有实车精度结论。

## 构建和运行

在已有 ROS 2 Jazzy 的 Ubuntu 工作区：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select limo_maze_perception
source install/setup.bash
ros2 run limo_maze_perception red_rgbd_detector --ros-args \
  -p use_sim_time:=true -p enabled:=true -p depth_registered_to_color:=true
```

依赖在 `package.xml` 声明，包括 `cv_bridge`、`message_filters`、NumPy 与 OpenCV；可通过项目统一的依赖安装流程准备。topics、同步容差、深度范围、最小区域和样本数支持启动参数；启用开关按启动时值读取。

离线核心测试从仓库根目录运行：

```bash
python -m unittest discover -s tests -p 'test_perception_core.py' -v
```

测试覆盖多连通区、RGB/BGR、米/毫米、坐标轴方向、遮挡对应的无效深度、离群深度、混合表面、内参与图像尺寸错误、时间同步、未注册深度、空/冲突 frame、JSON 有限数值。通过这些测试不能证明 ROS topic 接通或 Gazebo/实车定位通过；运行验证另记入项目实验记录。

## 官方接口依据

- [ROS 2 Image 消息定义](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/Image.msg)：采样时间、光学坐标轴、图像编码。
- [ROS 2 CameraInfo 消息定义](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/CameraInfo.msg)：内参、畸变、标定状态与裁剪含义。
- [ROS 2 message_filters 实现](https://github.com/ros2/message_filters/blob/jazzy/src/message_filters/__init__.py)：订阅 QoS 与时间同步接口。
- [OpenCV 色彩空间转换文档](https://docs.opencv.org/4.x/d8/d01/group__imgproc__color__conversions.html)：RGB/BGR 与 HSV 约定。
