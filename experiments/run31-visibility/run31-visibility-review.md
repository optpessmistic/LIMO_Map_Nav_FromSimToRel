# run31 三高度层可见性诊断：高度参考与聚合审查

2026-11-03。仅核对已关闭 run31 的小型 JSON、URDF 和现有离线原型；没有启动 ROS、读取 bag、解码 RGB-D 二进制、读取目标真值位置或修改任务成功规则。本文件是实现前审查，不是可见性计算结果。

结论：`map z=-0.15 m` 可作为本轮明确标注的**名义 base_footprint 平面**；不能称为经过测量的地面。离地 0.02/0.10/0.18 m 三层在该假设下对应 `map z=-0.13/-0.05/+0.03 m`。整个有限地图矩形都应参与诊断，未知地图格必须继续显示为未知。

## 1. 实际地图、会话和文件绑定

- 原始根目录：`outputs/ros_smoke/run31-observation-route/`。
- 冻结副本：`experiments/run31/final-save.json`。其与原始 `final-save.json` 的 SHA256 均为 `b220cbf8d9a130a071a2281d40a6a494c4df66cf5cfbda70c6cf3ea7d494aadc`，与 `experiments/run31/snapshot-map.provenance.json.source_sha256` 一致。
- 实际字段是 `final-save.snapshot.map`、`snapshot.epoch` 和 **`snapshot.transforms`**，不是 `snapshot.tf`。
- `snapshot.valid=true`；`captured_ros_ns=319704000000`。地图 frame 为 `map`，map stamp 为 `319250000000`。
- epoch 为 `7d6ec34d-3da2-4de8-a483-aeaf2faeb56e`，与 `experiments/run31/epoch.txt`、33 个留存请求的 `map_epoch` 和 `sample.map_epoch` 相同。
- 原始 OccupancyGrid 为 84×53，共 4452 格；resolution=`0.05000000074505806` m；origin.xyz=`[-0.5018672475186436,-2.1192623631253813,0.0]`；origin quaternion=`[0,0,0,1]`。原始值分布：free(0)=2584、occupied(100)=639、unknown(-1)=1229。
- 地图 origin.z=0 是二维栅格坐标约定，**不能作为实际地面高度**。显示格心须使用 `(column+0.5,row+0.5)*resolution` 加地图原点；JSON 原始行按 y 递增，PGM 显示行上下翻转。

`experiments/run31/retention-audit.json` 原结果为 passed=true、accepted_count=33、verified_count=33。按该文件的固定 `frames[].request_sha256` 选择原始根下 `retained/<sha>/`，不要靠扫描目录或检测计数挑选有利帧。

本次逐个读取这 33 个 request 与 CameraInfo 得到：

- frame 均为 `camera_optical_frame`；RGB、depth、request.tf、CameraInfo 的时间戳逐帧完全相等；时间范围 132800000000–277300000000 ns。
- calibration_id 均为 `063b5170a4f810696ab4f7769088cfe3faeb98edf740f47cfbd6357090642f93`。CameraInfo 为 640×480，fx=fy=381.36115024798124、cx=320、cy=240、D=0、R=I、P=[K|0]，无 ROI 裁剪/binning。
- `request.tf.camera_pose` 的 z 范围为 `[0.029999999999999916,0.030000000000000082]` m。这种浮点一致性只验证坐标链，不能证明地面/传感器实际精度。
- `request.tf.camera_pose=[x,y,z,qx,qy,qz,qw]` 是 map←camera_optical 的采样变换；`base_pose` 只有 x/y/yaw，没有 z，不能从它直接读出地面。

这里只重算了上述小文件的指定哈希与元数据关系；未重新验证所有 RGB/depth 二进制。实现执行时仍须验证每个实际使用文件的 manifest/请求/producer/原审计关系与字节长度和 SHA。

## 2. 高度参考的证据及限制

最终 snapshot 中三条变换都为 stamp `319700000000`，roll/pitch 为零：

| 变换（父坐标←子坐标） | z，米 |
|---|---:|
| map←odom | -0.1500000000000001 |
| odom←base_link | 0.14999999999999997 |
| map←base_link | -1.3877787807814457e-16 |

`outputs/ros_smoke/run31-observation-route/spawn/robot.urdf` 的 SHA256 为 `0a7b5d7dbcff95286de3473140e543f2805e5c192251476ad896ec854dcd8502`。其中：

- `base_footprint_joint`：base_footprint→base_link，xyz=(0,0,0.15)。
- `camera_joint`：base_link→camera_link，xyz=(0.084,0,0.03)。
- `camera_optical_joint`：camera_link→camera_optical_frame，无平移，rpy=(-π/2,0,-π/2)。
- DiffDrive 的 child_frame_id 是 base_footprint。

由该链可得到名义 map←base_footprint z=-0.15、map←camera z=+0.03。采样相机 TF 与此一致。直接使用每帧采样 TF 做投影，不用最终 snapshot 的相机外推代替历史采样位姿。

**具体的精度限制：**该 URDF 的轮轴相对 base_link z=-0.1，碰撞球半径 0.045 m，碰撞中心的额外位移仅在 y；因此球最低点为 base_link.z-0.145，比 base_footprint 平面高 5 mm。这已经说明名为 footprint 的参考平面不等于严格的碰撞支撑平面。仅凭该结构不能断言最终接触沉降、地面坡度、仿真接触误差或真实相机离地高度；不能把 5 mm 当作完整误差界。

允许的报告字段为 `floor_z_m=-0.15`、`floor_provenance=tf_urdf_nominal_base_footprint`、`floor_reference_verified=false`、`floor_error_bound_m=null`。若另画 -0.145 等敏感性图，必须标为另一假设；不能选结果更好的高度来宣称搜索完成。本轮无需用目标位置/目标检测高度来反推地面。

same epoch 也不证明历史 map←camera 已按最终回环优化重投影。最新地图仅是共同显示背景；必须保留 `final_map_reprojection_verified=false`，不得把诊断边界当成厘米级最终几何保证。

## 3. 每帧投影和聚合的关键检查

1. **计算域不缩水。**每层保留 84×53 全矩形网格，包括 occupied/unknown。每格独立保存原始 map_value 和 row/column；三层不取最有利层代替整物体。若每帧处理全部格心，33×4452×3=440748 个帧—点判定应能完整对账；没有被评估的输入必须显式记为缺失，不能计为 clear。
2. **变换和深度语义固定。**`p_camera=R(q)^T*(p_map-t)`；只用 camera optical Z 比较 axial depth，不用欧氏距离。按 producer metadata 解码行步长、字节序和格式。保留 NaN/Inf/零值；不把无效深度替换成远平面。
3. **逐帧状态互斥且可追溯。**保留 behind_camera、outside_image、invalid_depth（含 2×2 stencil 越界）、depth_ambiguous、occluded、depth_clear。每个有效投影使用 floor(u/v) 起的 2×2 原始样本；不平均跨遮挡边缘的前景/背景。保存 delta_min=min(D)-Z、delta_max=max(D)-Z，以及对应 frame/request ID。深度范围必须显式记录，超范围维持 invalid，而非当成无遮挡。
4. **误差界未知时只做名义分类。**`uncertainty_bound_m=null` 时可按零界计算名义正/负/混合余量，但不得输出 verified visibility/recognition。给定 ε 时用 delta_min>ε、delta_max<-ε；边界等号、异号或处于界内均 ambiguous。任何一项无效不能靠另外三像素投票通过。
5. **any clear 只表达存在一次名义视线机会。**每格每层保存各状态计数/支持帧；any_nominal_clear 可由 clear_count>0 导出，但不得删除该格另有 invalid/ambiguous/occluded 的证据。另存 has_invalid、has_ambiguous、has_occluded、no_clear_support、全部在视锥外等原因。这样可同时表达“一帧有机会、其他帧深度缺失”，避免单色优先级吞掉不确定性。
6. **相关帧不是独立可靠性试验。**同时记录唯一时间戳数、inspection window/goal 分组；同窗口三个相邻样本不能解释成三次独立覆盖验证。不把 detection_count、没有红色检测或 processed inspection 成功等同于空间点没有目标。
7. **统计限定为本次采样域。**输出每高度层的格数、各 map class 下的支持计数及相应分母；状态混合时明确其集合可重叠。不输出“全迷宫覆盖率”、整盒可识别百分比或正式识别召回率。未留存连续图像、矩形外区域、采样高度之间都保持未知。

## 4. 小图避免把未知地图画成已探索

建议左侧固定一张 OccupancyGrid 背景（free 浅色、occupied 深色、unknown 灰色/斜线），右侧三张同范围、同朝向的高度层。最小安全实现可只在已知 free 格上着色显示名义 clear，并在图例明确“仅显示掩膜，全部地图类别都已计算”；occupied/unknown 原背景不得被绿色覆盖。完整 occupied/unknown 点状态仍保存在 JSON/表格，并按 map class 分开统计，不能因显示掩膜从分母或数据中丢掉。

若确实需要直接显示 unknown 格的视觉机会，用独立点/轮廓符号并始终保留灰色斜线底纹，图例写“未知地图格上的名义视线机会，地图仍未知”；不要把整格填成与已知 free clear 相同的绿色。另一种更清楚的做法是单独的全矩形几何诊断面板，叠加 unknown/occupied 的固定纹理，而不称其为探索地图。

图面标题应含“离线、名义地面 -0.15 m、非识别/完整搜索证明”；每层注明离地高度和 map z。可以用单独细点/描边表示该格同时存在 invalid 或 ambiguous 支持，或另附不确定支持小图；主 JSON 必须完整保留，不能因缩略图色数有限而丢数据。

最终输出固定保持 `diagnostic_only=true`、`recognition_coverage_verified=false`、`visual_coverage_complete=false`、`mission_success=false`；原始 run31 mission/retention/关闭结果不变。完成此诊断也不能把 run31 的 5/12 观察动作重写为 12/12。
