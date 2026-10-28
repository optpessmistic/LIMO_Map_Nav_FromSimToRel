# 验证记录

验证日期：2026-10-28。以下记录区分逻辑测试、ROS 接口、物理仿真和实车；某一层通过不代表其他层通过。完整课程任务仍未验收。

## 已运行的检查

| 检查 | 环境与命令 | 结果与适用范围 |
|---|---|---|
| 核心测试 | WSL 系统 Python 3.12；`python -m unittest discover -s tests -v` | 94 项，93 通过、1 项显式集成测试默认 skip：二维仿真 22、任务核心/ROS 适配 41、感知 12、旋转守卫 18、可选真实 Action 1。覆盖先验与观测分离、超时锁存、epoch 门槛、取消异常、重复里程计及前沿有效平移过滤。 |
| 真实 ROS 节点启动回归 | source Jazzy 后运行 `test_ros_mission_core.py` | 包含真实 ROS 构造/spin/销毁、启动先验配置拒绝与结果保存检查；独立 domain 74，autonomous=false，无运动目标。修复 `Node.handle` 只读属性冲突。无 ROS 时明确 skip。 |
| 真实 ROS Action 回归 | `RUN_ROS_SURVEY_TEST=1 ROS_DOMAIN_ID=74 python -m unittest discover -s tests -p test_initial_survey_ros.py -v` | 单项集成测试通过，覆盖真实 Action server 成功与延迟接受后同 UUID 取消两条路径；专用测试 Action，不连接速度话题。 |
| 世界导出、地图坐标与导航配置 | `python -m unittest discover -s tools -p 'test_*.py' -v` | 本部分 16 项通过；检查贴墙目标净距、SDF、地图旋转/边界、DWB/MPPI 低速配置及速度消息链。与离线评分合计 37 项。 |
| 离线目标评分 | `python -m unittest discover -s tools -p test_target_evaluation.py -v` | 新增 21 项通过，含 80 组穷举最优匹配对照；检查显式对齐、一对一匹配、重复/合并实例、零目标和非法输入。 |
| 激光对齐诊断工具 | `python -m unittest discover -s tools -p test_scan_alignment.py -v` | 11/11 通过，不依赖 ROS/vendor/本地输出；包含不支持几何拒绝、时间插值/外参、缺 map TF、证据来源和禁止覆盖。全部离线工具合计 48/48 通过。 |
| 二维任务场景 | `python -m sim.suite` | 15/15 通过；完整策略完成可达范围内的理想探索、搜索和返航。含 0/1/3 个目标、不可达区域、遮挡、预算终止等情况。 |
| 上游固定版本 | `bash scripts/fetch_sim_vendor.sh --check-only` | 固定源 commit 和补丁检查通过；模型包与实车驱动包名称分离。 |
| ROS / Gazebo 安装 | WSL Ubuntu 24.04；`sudo bash tools/setup_jazzy.sh` | 安装成功，退出码 0。Jazzy、Harmonic 8.15.0、Nav2 1.3.13、SLAM Toolbox 2.8.5。 |
| ROS 包构建 | `colcon build` | 模型、仿真、集成、任务、感知共 5 包通过；修复后任务包也已重新构建。 |
| 仿真传感器与 TF | `scripts/check_sim_runtime.py` | 通过：9 路消息、推进的时间戳、720 束激光、640×480 RGB/深度、内参、4 段 TF 及图像时刻变换。 |
| 在线 SLAM / Nav2 就绪 | `scripts/check_navigation_runtime.py` | 通过：持续地图更新、map→base_link、NavigateToPose、6 个核心生命周期节点 active。没有发送运动目标。 |
| RGB-D 实际观测 | `scripts/capture_rgbd_evidence.py` | 同一 260.3 s 采样时刻保存 RGB、深度、标定及检测 topic；视野中的 1 个红色替身有三维观测。不等于全场找齐。 |
| 自主任务首次运行 | `mission.launch.py autonomous:=true` | 起点记录成功，但因起点 footprint 不在已知安全连通区而 blocked；未发运动目标。正在核对 240° 雷达盲区和起始地图。 |
| 初始旋转与地图检查 | `scripts/initial_survey.py`，独立确认初始 0.30 m 净空后显式执行 | Nav2 Spin、里程计累计转角、回正与停止检查通过；地图仍未覆盖后方，故不计物理转向或完整扫描通过。 |
| 独立运动对照（修复前） | 原生 model pose、ROS odom 与同时间 TF；run4d | 3637 组对齐记录：odom 最大累计 6.39642 rad，实际车体仅 0.0000509 rad，实际最大位移 0.12507 m。物理转向失败；保留失败记录。 |
| 轮接触模型修复 | `scripts/check_sim_physics.py`，隔离原生 Gazebo 场景 | 正式安装模型完整圈：world 6.404542595 / odom 6.404549999 rad，最大漂移 0.00002454 m；IMU、停车及自身 server 回收通过。球接触/侧向柔顺是仿真近似，详见 [物理验证](physics-validation.md)。 |
| 正式迷宫真实旋转 | run5，物理修复后的原生 model pose / ROS odom 对照 | 1431 组对齐记录：真实累计最大 6.399992745 / odom 6.399999999 rad，最大漂移约 0.065 mm。传感器/TF 九路检查及导航就绪复验通过。 |
| 旋转后地图连通性 | run5 原始 `/map` 和任务 R=0.25 m | 起始 footprint 已安全，但地图产生约 −32° 的重复墙影；安全区分裂为 160、158、33 格，起点仅在 33 格分量，无可达 frontier。SLAM 仍阻断自主任务，未发送平移导航目标。 |
| 前沿候选回归 | run6 保存快照离线规划，WSL Jazzy 任务测试 | 过滤实际距离不超过 0.20 m 和零朝向候选；新候选距离约 0.202 m，保留 3 个未观测格。41/41 任务测试通过，任务包重建通过；尚未宣称该候选已实际导航成功。 |
| 运动中激光输入 | run5b 同时间戳原生 scan / ROS scan / TF / world pose | 137 运动帧、98,640 束，ranges 原生/ROS 逐值一致；零偏移测距 RMS 1.298 cm，优于 ±1/20/100 ms 对照。仅诊断当前静态 box 场景，不证明 SLAM 成功。 |
| SLAM 单变量对照 | 新模型和 epoch 的 run6 / run7 | 关闭匹配时错位墙影消失；恢复匹配但只关闭重心选取仍出现约 −0.517 rad 修正。详见 [调查记录](slam-rotation-investigation.md)；诊断配置未替代正式 SLAM 默认值。 |

二维测试使用精确位姿和理想局部观测；它不含真实图像识别误差、SLAM 漂移、物理碰撞或轮胎滑移。当前 Gazebo 使用 llvmpipe 软件渲染；SLAM/Nav2 开启后的实测实时率约 0.49，不能据此宣称已启用 GPU 加速。

GitHub Actions 的提交 `11f4514` [运行通过](https://github.com/optpessmistic/LIMO_Map_Nav_FromSimToRel/actions/runs/40743656941)。该 CI 验证纯逻辑与离线工具，不含 ROS/Gazebo；本机 ROS/物理验证须分别报告，后续修改需核对对应提交的检查结果。

生成的 `outputs/` 不入 Git。正式记录应保留测试日志、JSON、地图、回放与 commit；CI 的产物保留规则以 GitHub Actions 配置为准。

## ROS 联调顺序

1. 在独立仿真 domain 73 中启动模型，运行 `scripts/check_sim_runtime.py`。检查推进的模拟时钟、激光、里程计、IMU、关节、RGB、深度、标定和图像时刻的 TF。
2. 启动在线 SLAM 与 Nav2，验证 `/map`、`map → odom → base_link`、导航 Action 与生命周期状态。仅启动进程或存在话题名不足以通过。
   当前就绪脚本只验证接口；另需核对 `/cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed → collision_monitor → /cmd_vel` 的实际输出。项目启动文件不启动会直接发布 `/cmd_vel` 的无关 docking server。
3. 在已观察的自由空间做短距离导航和返回，核对实际位姿、速度限幅与碰撞情况。
4. 启用红色替代物检测，验证相机三维观测、时间戳 TF 转换、跨帧确认和地图标注；不从世界元数据生成检测。
5. 开启完整探索，保存事件、最终地图和任务 JSON。分别报告几何探索、视觉覆盖、目标标注和返航结果，不把部分完成记作完整成功。

这些步骤的运行证据尚未全部取得。真实物品类别与相机未确定，所以当前红色替代物节点不发布有效视觉覆盖声明，`target_definition_verified` 保持 `false`。

## 实车仍需的证据

- 小车本机 ROS 2 发行版、运动模式、雷达及相机型号，实际话题和 TF 树。
- 真实目标定义、尺寸/高度/可见性条件，识别准确率与定位误差。
- SLAM 闭环后历史标注与起点坐标的一致性、失联停止和实际返航误差。
- 未调参迷宫上的完整多次实验，包含失败运行和人工干预次数。

当前结果不能用于宣称已完成实车实验，也不能宣称“所有真实目标均已找到”。
