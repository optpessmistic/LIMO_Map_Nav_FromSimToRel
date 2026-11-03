# 留存 RGB-D 的离线深度诊断

`tools/visibility` 用已经留存并通过审计的 RGB-D、CameraInfo 和采样时刻的相机 TF，检查指定三维采样点与测得表面的深度关系。结果用于查找观测机会和盲区，不能证明目标被识别、整件物品可见或迷宫已完整搜索，也不会改写原任务结果。

计算只读取固定任务文件、原留存审计、留存文件及 `final-save.json` 中的地图，不读取 world、native scene 或目标真值位置/数量，不按检测数量挑选帧。

## 在 WSL 中运行

在仓库根目录使用 Python 3。计算和审计使用标准库；`--plot` 另外需要当前 Python 环境中有 `numpy`、`matplotlib`，不需要启动 ROS、Gazebo 或 RViz。缺少绘图库时，可先去掉 `--plot` 生成数据；命令不会自动安装依赖。

run31 的示例命令：

```bash
cd /mnt/c/Users/Admin/.codex/worktrees/0dd9/Personal_Plan/LIMO_Map_Nav_FromSimToRel

/usr/bin/python3 -B -m tools.visibility \
  --run-root outputs/ros_smoke/run31-observation-route \
  --mission mission_20261103_014821.json \
  --output outputs/visibility/run31-nominal-01 \
  --floor-z -0.15 \
  --height-reference run31_tf_urdf_nominal_base_footprint_not_measured_ground \
  --heights 0.02 0.10 0.18 \
  --min-depth 0.15 \
  --max-depth 6.0 \
  --uncertainty-bound unknown \
  --depth-registered \
  --depth-semantics optical_z \
  --optical-axes x_right_y_down_z_forward \
  --assumed-size 0.06 0.20 \
  --plot
```

输出目录必须尚不存在，并且位于原运行目录之外；重复实验需换一个新输出目录。输入文件不会被修改。程序异常时可能保留部分输出和 `failure.json`，这种目录不能作为计算完成的证据。

`--mission` 必须是 `run-root/mission/` 内的准确文件名，不会自动选择最新任务文件。原始 RGB-D 二进制位于原运行目录的 `retained/<request_sha256>/`，没有随 Git 仓库提交，也不会被此工具复制到新输出目录。只有 `experiments/run31/` 的小型归档时，仍需取得与审计身份匹配的原始留存目录才能复算。

## 本例的假设与分母

该命令已在 WSL Ubuntu-24.04 实际执行并退出 0：33/33 帧纳入、0 帧排除、440748 次判定。相关 44 项测试全部通过，包含 Linux 软链接拒绝检查。原始输入和源码指纹、完整支持索引与图见 [run31 离线归档](../experiments/run31-visibility/README.md)。每个高度均保留全部 4452 格；其中原地图自由格共 2584：

| 名义离地高度 | 自由格有名义 clear | 自由格遮挡标签 | 自由格混合/表面标签 | 自由格无 clear 合计 |
|---|---:|---:|---:|---:|
| 0.02 m | 1786 | 794 | 4 | 798 |
| 0.10 m | 1827 | 753 | 4 | 757 |
| 0.18 m | 1841 | 739 | 4 | 743 |

“遮挡标签”只表示至少一帧有遮挡且没有 clear/混合支持，不表示每一帧都看到遮挡。每层均有 1229 个原始未知格；它们仍是未知地图区域。不同高度不能合并成整件物品或全迷宫覆盖率。

本例输入为 33 帧、84×53 即 4452 个地图格、3 个高度层，完整计算应包含 **33×4452×3=440748 次帧—点分类**。每层都保留地图的完整有限矩形，包括 free、occupied、unknown 和中间占用值；不会按深度范围、可通行区域或图像投影裁剪地图分母。矩形外区域和高度层之间的空间不在本次采样域内。

`--floor-z -0.15` 是由该轮 TF/URDF 坐标链得到的名义 `base_footprint` 平面，未经过地面测量。地图 `origin.z=0` 不能直接当作地面。相对该参考平面的 0.02、0.10、0.18 m 三层，分别对应 map 坐标中的 z=-0.13、-0.05、+0.03 m。地面参考误差没有已知界；换高度参数属于另一个假设，不能挑选较有利结果来宣称完成搜索。

`--min-depth 0.15 --max-depth 6` 是本次选择的**测量深度过滤参数**，不是经过标定的传感器可靠量程，更不是物品可识别距离。它只过滤读取到的表面深度样本。查询点自身的光轴 Z 是否在该范围内，另由 `query_depth_relation` 记录：

| 字段值 | 查询点光轴 Z 的关系 |
|---|---|
| `behind_camera` | Z≤0 |
| `below_min_depth` | 0<Z<最小深度 |
| `within_depth_bounds` | 最小深度≤Z≤最大深度 |
| `above_max_depth` | Z>最大深度 |

查询点超出上述范围不会被从地图删除，也不会单凭这一关系自动判为可见或不可见。例如近处查询点仍可能具有 `depth_clear` 的名义几何关系；这不意味着相机在该距离具备可靠测量或识别能力。`query_depth_relation_counts` 统计的是帧—点关系次数，不是独立地图格数或识别成功次数。

`--depth-registered` 明确声明深度已经与 RGB/CameraInfo 配准；`optical_z` 表示深度是相机光轴方向的 Z，而非到相机的欧氏距离。光学坐标约定为 x 向右、y 向下、z 向前。这些参数是输入声明，不能替代实际配准和标定验证。

## 审计和单帧比较

可用帧取固定任务接受清单中“**原审计 verified ∩ 当前重新审计 verified**”的交集。原审计失败的帧不会因为后来文件可读而恢复资格；当前文件哈希失败的帧也会排除，并留下原因。全局审计错误、缺失/冲突的身份、epoch 或会话不一致会拒绝加载。工具同时检查闭合的 producer checkpoint、文件长度和 SHA256、请求/manifest/回执绑定、路径逃逸与符号链接，并复核加载期间输入是否发生变化。

每帧要求 RGB、depth、CameraInfo 的时间戳、坐标系和分辨率一致，TF 精确绑定到该采样戳，标定内容与 calibration ID 对应。深度支持带行填充和大小端的 `32FC1` 米、`16UC1` 毫米；NaN、Inf、零值不会被替换成远处表面。总解码载荷有限额，超过限制会拒绝，不会静默截断帧清单。详见[逐帧留存](frame-retention.md)。

投影使用请求中保存的历史 map←camera_optical 变换。对图像内的投影点，从 `floor(u),floor(v)` 起读取 2×2 原始深度样本，不跨前景/背景边缘做双线性平均。任一样本无效、超出所选测量过滤范围或邻域越界，该次判定为 `invalid_depth`。

四个有效样本组成的比较量为 `delta_min=min(D)-Z`、`delta_max=max(D)-Z`。给定误差假设 ε 时，规则如下：

| 单帧状态 | 含义 |
|---|---|
| `behind_camera` | 查询点在相机后方或 Z=0 |
| `outside_image` | 投影不在图像范围内 |
| `invalid_depth` | 缺少完整有效的 2×2 深度邻域 |
| `depth_clear` | `delta_min > ε`，四条采样射线测得的表面都在查询点之后 |
| `occluded` | `delta_max < -ε`，四条采样射线测得的表面都在查询点之前 |
| `depth_ambiguous` | 深度混合、边界相等或处于误差假设范围内 |

`--uncertainty-bound unknown` 保留“误差界未知”，只用零界做名义比较；不能解释为误差为零。改为一个数值时，该数值也只是尚未验证的假设，工具不会将 `depth_uncertainty_verified` 改为 true。

## 图和文件如何阅读

图包含原始占用地图与三个高度层，范围和朝向一致。红色空心三角形表示留存时的相机位置。未知地图格始终保留灰色斜线；占用值大于零的格叠加灰黑色点。某个未知格有名义视线机会，地图状态仍然是 unknown，不能解释成已经探索。

每格每层用下列顺序产生一个展示标签：

| 展示标签 | 聚合条件 |
|---|---|
| `nominal_clear` | 至少一帧为 `depth_clear` |
| `mixed_or_surface` | 没有 clear，但存在 `depth_ambiguous` |
| `occluded` | 没有以上支持，但存在 `occluded` |
| `no_valid_depth` | 没有以上支持，但存在 `invalid_depth` |
| `no_view` | 所有帧均在相机后方或图像外 |

展示颜色采用上述优先级；其他帧的遮挡、混合和无效深度证据仍保留在数据中。`nominal_clear` 只表达至少一次名义几何机会，不是概率、置信度或识别覆盖率。同一观察窗口中的相邻帧高度相关，不能视为多次独立可靠性试验。

| 输出 | 用途 |
|---|---|
| `inputs.json` | 输入文件和计算源码的指纹、原/当前审计、包含和排除帧及原始路径 |
| `layers.json.gz` | 全地图矩形、逐层标签、各状态支持帧索引、查询深度关系、余量与假设尺寸 |
| `report.json` | 输入规模、逐层统计、原任务标志、假设/限制和输出文件哈希 |
| `visibility-diagnostic.png` | 加 `--plot` 时生成的四面板图 |

`layers.json.gz` 中的格索引为 `row*width+column`；支持帧索引指向同文件的 `frames` 列表。`counts` 是互斥展示标签的格数，`counts_by_map_class` 按原始地图类别拆分；`support_frame_indices` 则保留每次帧—点分类，同一格可在不同帧具有不同状态。最有利余量 `max_of_delta_min_m` 也不是已验证的安全余量。

`--assumed-size 0.06 0.20` 只估计在该查询点光轴 Z 处、平行于像面的假想 0.06×0.20 m 矩形所占像素尺寸。它没有检查整件物品的轮廓、姿态、遮挡比例、纹理或检测器阈值；不能据此推断真实物品可被识别。

## 结论边界

历史采样位姿尚未通过最终 SLAM 图优化重新投影；相同 epoch 不足以证明历史 TF 与最终地图在厘米级一致。二维最终地图只是共同显示背景，地面参考、深度、姿态和配准误差均未得到完整标定。

单个格心、三个离散高度和四条离散深度射线不能证明整格、层间空间或整件物品都可见。没有留存的连续图像不补算为 clear；没有检测也不等于没有目标。

输出固定保持 `diagnostic_only=true`、`recognition_coverage_verified=false`、`mission_success_claimed=false`，并保留 `original_mission_flags`。run31 原任务失败和不完整观察结论不会因本次离线诊断改变；图中的绿色不能用于重写原任务成功标志。
