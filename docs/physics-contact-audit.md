# Gazebo / DART 接触参数语义审查

本记录来自 2026-10-28 的只读源码与本机头文件核对，不包含模型运动试验结果，也不把参数候选视为实车标定。

本机 ROS Jazzy 使用 `gz-physics 7.8.0` 和 vendored `DART 6.13.2`。版本分别由 `/opt/ros/jazzy/opt/gz_physics_vendor/include/gz/physics7/gz/physics/config.hh` 与 `/opt/ros/jazzy/opt/gz_dartsim_vendor/include/dart/config.hpp` 核实；[Jazzy DART vendor 配置](https://github.com/gazebo-release/gz_dartsim_vendor/blob/jazzy/CMakeLists.txt)也指定 DART 6.13.2。

| 项目 | 已核实语义 | 对本项目的含义 |
|---|---|---|
| `fdir1` 默认参考系 | 碰撞形状的 `ShapeFrame`；并非轮 link 坐标系。 | 必须连同 collision 的相对旋转解释向量。 |
| 显式参考系 | gz-physics 仅在 `fdir1` 有 `gz:expressed_in` 属性且解析到对应 BodyNode 时调用 `setFirstFrictionDirectionFrame`。 | 本项目未设置该属性，采用默认 collision 参考系。 |
| `mu`、`mu2` 合成 | 两个接触面的主、次摩擦系数分别取较小值。 | 轮端摩擦高于地面摩擦时，提高轮端值不会提高对应接触上限。 |
| `slip1`、`slip2` | gz-physics 分别映射到 DART 主、次方向的 slip compliance。DART 默认处理器将两接触面的 compliance 相加，并在创建约束时按接触点数量缩放。 | 它们描述接触约束的柔顺性，不是摩擦系数，也不是固定的滑移百分比。 |

`DynamicsAspectProperties` 的构造函数将摩擦方向 frame 指针置为 `nullptr`，getter 原样返回；接触处理器在该指针为空时使用 `shapeNode->getWorldTransform()` 转换方向。gz-physics 为每个 collision 创建 ShapeNode，并应用 collision pose。因此默认参考系结论来自实际实现，不依赖文档中可能含糊的 “body-fixed” 用语。[DART 构造与 getter，52–60、217–219 行](https://github.com/dartsim/dart/blob/v6.13.2/dart/dynamics/ShapeFrame.cpp#L52)；[方向变换，292–315 行](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/ContactSurface.cpp#L292)；[gz-physics 参数导入与 collision pose，785–900 行](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.8.0/dartsim/src/SDFFeatures.cc#L785)。

当前轮碰撞体是绕局部 X 轴旋转约 π/2 的圆柱，关节轴在轮 link 中为 ±Y。因此 **collision 局部 Z 轴与轮轴平行或反平行**，对应地面上的横向侧滑方向；符号不影响对称摩擦约束。使用精确 π/2 时严格平行。collision 局部 Y 是径向，会随轮角改变，不能据此假设它始终代表横向。对各向异性参数而言，`fdir1="0 0 1"` 使 `mu/slip1` 对应侧向，`mu2/slip2` 对应地面上与其正交的滚动方向；实际约束方向由接触法向与该向量共同构造。[DART 切向基构造，709–759 行](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/ContactConstraint.cpp#L709)。

接触系数公式为 `mu_eff=min(mu_A,mu_B)`、`mu2_eff=min(mu2_A,mu2_B)`；相乘的是 restitution，而非摩擦系数。未声明地面 surface 时，SDF 的 `mu`、`mu2` 默认均为 1，所以原轮端 `10/10` 与该地面接触的系数为 `1/1`。这一判断限于默认接触处理器，另加运行时接触参数回调可覆盖它。[DART 合成规则，93–125 行](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/ContactSurface.cpp#L93)；[SDF 默认值](https://github.com/gazebosim/sdformat/blob/sdf14/sdf/1.9/surface.sdf#L50)；[gz-physics 接触回调覆盖](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.8.0/dartsim/src/SimulationFeatures.cc#L307)。

官方 [Gazebo Sim 8 四轮差速示例](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/diff_drive_skid.sdf#L117)使用球形轮碰撞体，配置 `fdir1=0 0 1`、`mu=mu2=1`、`slip1=0.035`、`slip2=0`。这是可比较的侧向柔顺设置，并非 LIMO 的推荐标定。可分别比较“保持几何不变，降低侧向 `mu`、保留纵向 `mu2`”和“保持两个摩擦系数不变，仅增加侧向 `slip1`”两种假设。改变碰撞形状应作为独立对照；不能将改善的轮式里程计等同于世界坐标中的车体运动改善。模型试验结果另行记录。
