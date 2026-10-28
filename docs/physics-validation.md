# LIMO Harmonic 轮接触模型与独立运动验真

核实日期：2026-10-28。环境为 WSL Ubuntu 24.04.4、ROS2 Jazzy、Gazebo Sim 8.15.0、gz-physics 7.8.0 / DART 6.13.2。上游仍固定为 [nevlife/hunter_gazebo](https://github.com/nevlife/hunter_gazebo/tree/8e1d4aeb459e77d549262ca20cac819d9c7a32ee)；模型留在忽略的 vendor 目录，项目仅保存补丁，未复制模型资产或更改原始许可证。

## 故障与判定依据

run4d 的 Nav2 Spin 报成功，轮式 odom 最大累计转角为 6.39642 rad，但同步采集的原生 Gazebo model `limo` 位姿转角只有 0.0000509 rad，平移却达到 0.12507 m。SLAM 的 `map → odom` 在补偿这个虚假的轮式转角。`base_footprint` 对 model 是恒等变换，世界位姿采集对象正确。这轮不能计为真实旋转或地图初始化成功；详见 [接入记录](jazzy-simulation.md)。

随后停止主场景全部仿真、SLAM、Nav2、感知进程，在独立 `GZ_PARTITION=limo_physics_diag_20261028` 的空场景中进行有界低速对照。命令直接发给原生 Gazebo `/cmd_vel`，并订阅命令回显、车体 world pose、原生 odom、joint states 与 IMU；没有 ROS bridge 或其他运动控制器。故障在原生命令下复现，不能归因于 ROS 速度桥接。原生世界真值只写入离线证据，从不送给建图、导航或检测。

## 源码与轴语义核对

- 右轮 joint 原来的 roll≈π、局部 axis=(0,−1,0)，变换到底盘后约为 (0,+1,−2.65×10⁻⁶)，与左轮 +Y 同向。已用本机 sdformat14 `resolve_xyz('base_footprint')` 核对四轮，并从实际 wheel-link 四元数增量验证正关节角对应的模型轴。直接反转右轮轴没有修复转向，不采用此修改。gz-physics 7.8.0 的 [ConvertJointAxis / ConstructSdfJoint](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.8.0/dartsim/src/SDFFeatures.cc) 也支持上述解释。
- 未指定 `gz:expressed_in` 时，DART 摩擦方向属于 collision ShapeFrame。DART 6.13.2 [ShapeFrame 构造与 getter](https://github.com/dartsim/dart/blob/v6.13.2/dart/dynamics/ShapeFrame.cpp) 保持默认空 frame；[ContactSurface 的空 frame 分支](https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/ContactSurface.cpp) 使用 shape 的 world transform。轮碰撞相对 wheel-link 绕 X 旋转 π/2，因此 collision 局部 Z 平行轮轴，也就是接地面的侧向；原来的局部 X 随轮转动，不能作为固定侧向。
- 本次地板未指定 surface，默认摩擦系数为 1；DART 的接触系数取两个碰撞面的较小值。因此仅把轮 μ 从 10 降到 1 基本不改变有效摩擦，不能据此排除摩擦影响。
- [Gazebo 官方四轮 skid-steer 示例](https://github.com/gazebosim/gz-sim/blob/gz-sim8/examples/worlds/diff_drive_skid.sdf) 使用球形轮接触、局部 Z 摩擦方向和 `slip1=0.035, slip2=0`。我们借鉴接触方式，**未将示例参数解释为真实 LIMO 轮胎标定值**。

四轮定轴转向需要接地点发生横向滑移。以下消融说明不带横向柔顺的接触约束是当前模型的主要问题；它们没有证明某个 DART 实现缺陷，也没有标定真实轮胎。

## 实测对照

短旋转命令均为 ±0.15 rad/s、8 秒墙时；实际仿真时长略有差异，故逐次以原生 odom 对照。直行为 0.05 m/s。每例结束持续发布零速 2 秒，并回收本例独立进程组。

| 配置 | world 转角 rad | odom 转角 rad | 最大车体平移 m | 解释 |
|---|---:|---:|---:|---|
| 原模型正旋 | 0.0000062 | 1.20510 | 0.070831 | 轮子运动不等于车体旋转 |
| 仅 μ=μ2=1 | 0.0000062 | 1.20495 | 0.070808 | 与地板取 min 后基本不变 |
| 仅 fdir1 改 collision Z | 0.000598 | 1.20465 | 0.070644 | 单独修方向不足 |
| 精确 π/π/2 + fdir Z，原圆柱 | 0.0000011 | 1.20465 | 0.008000 | 对称性改善，仍未转向 |
| 上项 + 侧向 μ=.1 / 纵向 μ2=1 | 0.882950 | 1.12935 | 0.085049 | 恢复转向但漂移过大，不采用 |
| 精确角 + fdir Z + slip .035/0，原圆柱 | 0.942058 | 1.20495 | 0.000019 | 旋转中心稳定，转角仍有显著偏差 |
| 上项换同轮心球碰撞 | 1.183850 | 1.20450 | 0.000026 | 接触近似稳定，余差约 1.7% |
| 球碰撞反旋 | −1.184145 | −1.20480 | 0.000026 | 正反向一致 |
| 球碰撞 45 秒正旋 | 6.638553 | 6.75435 | 0.000024 | 已真实完成整圈 |
| 球碰撞 + 几何轮距 .175 | 1.2046486 | 1.2046500 | 0.000027 | 轮距偏差消除 |

球接触直行的真实位移 0.401749775 m、odom 0.401750 m，非命令 yaw 约 0。45 秒整圈的 IMU 角速度积分为 6.638517 rad，与 native world 6.638553 rad 一致；几何轮距短旋转 IMU 积分为 1.204725 rad。上述通过例最后约 0.5 仿真秒的 world 位姿不再变化，所有本例 server 都 exit 0，所属进程组无残留。原始 JSON、SDF、服务器日志及 process 记录保存在忽略的 `outputs/physics_diag/`，紧凑汇总为 `summary.json`，失败对照未覆盖。

## 正式补丁

`patches/0001-limo-harmonic-project-interfaces.patch` 在现有接口补丁上追加以下最小接触修改：

1. 右轮 joint roll 使用 Xacro `${pi}`；四轮 collision roll 使用 `${pi / 2}`，保留原来的左右轴符号、关节位置与轮心。
2. 轮碰撞从圆柱改为半径 0.045 m 的球，视觉网格与质量/惯量保留原值。
3. `fdir1=0 0 1`；增加侧向 `slip1=0.035`、纵向 `slip2=0`。保留原 μ10/10、kp/kd/minDepth；验证场景地板默认 μ1，具体地板变化需重验。
4. DiffDrive 的轮距设为 `${track + wheel_width}`，即 .13+.045=.175 m。轮心在 y=±(.065+.0225)=±.0875 m；原值 .172 与实测转角比 `.172/.175` 相符。这是从仿真模型几何推导，**真实底盘有效轮距仍待标定**。

`slip1/slip2` 通过 `<gazebo reference="wheel_link"><collision><surface><friction><ode>…` 扩展加入。裸写 `<gazebo><slip1>` 不足以保证转换生效；[sdformat URDF parser 的 collision blob 处理](https://github.com/gazebosim/sdformat/blob/sdf14/src/parser_urdf.cc) 说明这种嵌入方式。实际 `xacro → gz sdf -p` 已检查四轮：sphere、精确角、fdir、slip、μ与 .175 轮距全部保留。`fetch_sim_vendor.sh --check-only` 通过。没有修改传感器位置、视场、噪声、视觉网格、质量或基础惯量来制造通过。

球接触会增加侧面碰撞宽度：轮心 y=±.0875 m、球半径 .045 m，合计宽 .265 m；前后轮 x=±.1 m，球接触 AABB 半尺寸为 (.145,.1325) m，其保守外接圆半径约 .1964 m。当前 Nav2 footprint 半尺寸 (.18,.12) 加 padding .02 后为 (.20,.14) m，覆盖这组球碰撞；任务安全半径 .25 m 也覆盖 Nav2 的 .24413 m 外接圆。**未加 padding 的原 footprint 宽 .24 m 并不覆盖球碰撞，后续不能直接去掉 padding。**这是仿真接触近似的边界，不代表真实车体外形或轮胎已经认证。

## 可重跑验证与通过范围

先按项目说明构建并 `source scripts/env.sh`。脚本读取已安装的模型包；输出目录必须不存在。

```bash
# 默认仅展开/审计模型；不启动 Gazebo，也不发送运动命令。
python3 scripts/check_sim_physics.py --output outputs/physics_check_geometry

# 显式执行：每条命令自己启动、拥有并回收一个隔离空场景。
python3 scripts/check_sim_physics.py --execute --case forward --output outputs/physics_check_forward
python3 scripts/check_sim_physics.py --execute --case spin --output outputs/physics_check_spin
python3 scripts/check_sim_physics.py --execute --case reverse_spin --output outputs/physics_check_reverse
python3 scripts/check_sim_physics.py --execute --case full_turn --output outputs/physics_check_full_turn
```

执行模式强制生成 UUID 后缀的独立 `GZ_PARTITION`，不用主场景 partition，不接 ROS 速度链；为降低渲染干扰，诊断副本仅去掉相机/激光及渲染系统，保留 IMU、碰撞、质量、关节、摩擦。短例限 8 秒墙时；整圈以 odom 累计 `2π+0.1` 为命令停止条件，最大 60 秒墙时，world truth 只用于事后验真。订阅失鲜会中断发命令，finally 发布零速 2 秒，独立测量停车后的 world 平移和角速度，并对自身 server group 有界停机与残留核验。输出保存展开模型 SHA256、原始 samples、检查结果和进程记录；拒绝覆盖旧目录。没有明确 `--execute` 的调用只做模型审计。

脚本验收门槛包括实际旋转方向/整圈、world/odom 转角误差 <.02 rad、旋转漂移 <.005 m、IMU 积分差 <.03 rad、尾段停车和零速回显。默认无运动模式与正式安装模型短旋转已实测通过，短旋转 world 1.2046486 / odom 1.2046500 rad、IMU 1.2048043 rad，server 已回收。

正式安装模型的完整圈也已由该脚本实测通过，证据在 `outputs/physics_diag/formal_script_full_turn/`：world 累计 6.404542595 rad、odom 6.404549999 rad、IMU 积分 6.405592029 rad，最大平移 0.00002454 m；最后约 0.5 仿真秒的 world 线/角速度为 0，零速回显、观察失鲜检查和 server 回收检查均通过。`geometry.json` 记录本次展开模型 SHA256，`samples.json` 保留原始数据，`process.json` 确认自身进程组无残留。

脚本审查后补充了结束时的全传感器 freshness 复核，以及按初始 heading 投影的有符号前进距离，防止将停止后的失联或倒退误记为成功。最终版本的 `formal_script_forward_final/` 已复跑通过全部检查并回收 server；同次运行也从实际 URDF 轮心与当前 Nav2 YAML 核对了几何包络覆盖。反旋证据 `formal_script_reverse/` 为 world −1.204498608 / odom −1.204500000 rad、最大漂移 0.00002721 m。

恢复完整传感器、正式 0.8 m smoke 世界及 ROS 速度链后的 run5，再次由独立原生 model pose 观测到真实整圈：最大累计 yaw 6.3999927 rad，对应 odom 6.4000000 rad，最大漂移 0.0000654 m。证据为 `outputs/ros_smoke/motion_evidence_run5.json`。这确认接触修复在完整场景有效；该轮 SLAM 的地图一致性仍未通过，不能把动力学通过写成完整建图或任务通过。

这些检查只证明当前空地、低速条件下仿真动力学自洽。正式迷宫中的传感器、原地观察后地图扩展、窄通道、物品标注与返航仍须在正式安装模型上独立重验；旧的 Action success 不追认。0.8 m smoke 场景通过也不能宣称 0.6 m 压力场景或实车实验已完成。
