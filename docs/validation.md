# 验证记录

验证日期：2026-10-28。以下记录区分逻辑测试、ROS 接口、物理仿真和实车；某一层通过不代表其他层通过。完整课程任务仍未验收。

## 已运行的检查

| 检查 | 环境与命令 | 结果与适用范围 |
|---|---|---|
| 核心测试 | WSL 系统 Python 3.12；`python -m unittest discover -s tests -v` | 101 项，100 通过、1 项显式集成测试默认 skip：二维仿真 22、任务核心/ROS 适配 48、感知 12、旋转守卫 18、可选真实 Action 1。覆盖先验与观测分离、超时锁存、epoch 门槛、取消异常、重复里程计及前沿有效平移过滤。 |
| 真实 ROS 节点启动回归 | source Jazzy 后运行 `test_ros_mission_core.py` | 包含真实 ROS 构造/spin/销毁、启动先验配置拒绝与结果保存检查；独立 domain 74，autonomous=false，无运动目标。修复 `Node.handle` 只读属性冲突。无 ROS 时明确 skip。 |
| 任务信号退出与取消异常 | 48 项任务回归，独立复核，任务包重建 | 真实 ROS main 的 SIGINT/静止 odom 退出已通过；迟到接受、结果请求异常、UNKNOWN/活动状态、发送响应异常及慢速非停车由行为回归覆盖。超时 pending 仍可能在客户端销毁后接受，明确报告未验证；尚未证明在动实车或在动 Nav2 退出。 |
| 真实 ROS Action 回归 | `RUN_ROS_SURVEY_TEST=1 ROS_DOMAIN_ID=74 python -m unittest discover -s tests -p test_initial_survey_ros.py -v` | 单项集成测试通过，覆盖真实 Action server 成功与延迟接受后同 UUID 取消两条路径；专用测试 Action，不连接速度话题。 |
| 世界导出、地图坐标与导航配置 | `python -m unittest discover -s tools -p 'test_*.py' -v` | 本部分 16 项通过；检查贴墙目标净距、SDF、地图旋转/边界、DWB/MPPI 低速配置及速度消息链。与离线评分合计 37 项。 |
| 离线目标评分 | `python -m unittest discover -s tools -p test_target_evaluation.py -v` | 新增 21 项通过，含 80 组穷举最优匹配对照；检查显式对齐、一对一匹配、重复/合并实例、零目标和非法输入。 |
| 激光对齐诊断工具 | `python -m unittest discover -s tools -p test_scan_alignment.py -v` | 11/11 通过，不依赖 ROS/vendor/本地输出；包含不支持几何拒绝、时间插值/外参、缺 map TF、证据来源和禁止覆盖。全部离线工具合计 48/48 通过。 |
| 隔离 SLAM 回放工具 | `python -m unittest discover -s tools -p test_slam_replay.py -v` | 14/14 通过；新增后全部离线工具 62/62 通过。真实回放 159 发出/45 接纳，与原型时间戳和栅格一致；实际进程来源和 SIGINT 失败报告/自有进程组回收也已验证。 |
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
| SLAM 匹配尺度对照 | 同一 159 帧记录、新建真实 SLAM 实例，45 个相同接受时刻 | 默认 smear=0.10 的最终 yaw 修正 −0.597 rad；仅改为 0.01 后为 −0.01745 rad、平移约 1.03 cm。禁用链匹配与改变相关搜索网格的对照未解决问题。半段独立初始化结果也保留；不宣称对所有地图或实车通用。 |
| 新场景匹配参数复验 | run8，新模型/epoch、匹配与重心选取开启、仅 smear=0.01 | 2064 组独立位姿对照：实际最大转角 6.40706 rad、中心漂移 0.0654 mm；最终 map→odom 修正 −0.02094 rad（约 −1.20°）、平移 0.968 cm。原始安全区恢复为单一 292 格分块，但起点有 3 格未观测；先验规划副本可选有效前沿目标，尚未执行。该参数已采用为小迷宫仿真配置并重建 bringup。 |
| Karto 最大边界修复 | 固定 2.8.5 源码，两行补丁、独立完整构建、实际 `CreateFromScans` 回归 | 原版 5 例均越界；补丁 5 例共 40 端点全部在界内且为 occupied，原点不变。未覆盖系统安装，未声称上游全套测试通过。 |
| 补丁场景与稳定性反例 | run9，新模型/epoch、实际加载库/参数已核对 | 真实转角 6.39968 rad、漂移 0.0654 mm；上边界墙恢复。yaw 修正 −.02443 rad，但轴向平移修正约 17 cm，显示 run8 参数尚不足以稳定建图。起点先验副本可连通不代表定位可信；尚未发送平移导航目标。 |
| 首次自主平移、扩图及标注 | run10，180 s 探索预算、120 s 返航预算 | 实际最大位移 1.46670 m；已观测面积增加 1.8175 m²；确认 1 个红色替身。4 个目标中 2 个成功、1 个 ACK 超时中止、1 个受控取消。第 4 个到达后因安全包络检查 blocked，未返航/未完成；[详细证据](first-autonomous-run.md)。 |
| 到达误差与候选安全裕量 | `8a8e7ed`，WSL Jazzy domain 74 回归及冻结地图 | 54/54 任务核心与 ROS 参数测试、5/5 配置生成测试通过，4 个冻结快照与预检一致；mission/bringup 两包重建成功。候选终点预留 0.05 m 到达误差及半格对角线，路径半径仍为 0.25 m；Nav2 XY=0.05 m、stateful=false。新场景实际导航效果另行验证。 |
| 到达修复后的自主流程 | run11，600 s 探索/180 s 返航预算 | 3 个探索目标成功且终态附近 XY 均在 5 cm 内、3 个探索目标超时取消；7 个终态附近原始地图 R 圆均安全。地图观测面积 3.855→8.170 m²，未以此声称真值覆盖率；返航中途超时，完整任务仍未通过。 |
| 两个替身的独立评分 | run11，初始静止帧建立固定 map→world 对齐，阈值 0.20 m | TP=2、FP=0、FN=0；平面误差 8.49/2.22 cm，RMSE 6.21 cm。只比较可见表面点与盒中心，不证明视觉覆盖或正式目标识别。末段独立原生位姿确认 29.983 sim 秒静止；[完整记录](autonomous-run11.md)。 |
| 真值几何与冻结路径复核 | run11，固定初始对齐，纯离线 | 真实自由区 7.0364 m² 中约 91% 被原图标为自由；细网格和地图分辨率不同，未分摊墙边离散化误差。真实 NavFn 库可在冻结地图上规划 3.3564 m 返航路径，743 密采点均通过 R=.25；不是实际历史 `/plan` 恢复。 |
| 连续仿真观察器 | `python3 -m unittest tools.test_sim_mission_observer -v` | 23/23 通过，WSL 与 Windows 均验证；只读订阅与参数查询、同 stamp TF、速度接收时刻、完整全局路径、原生静止窗口、拒绝覆盖。冻结实验版在 run12 实际采得 1739 组完整 TF 链、180 条路径；通用入口尚未独立在线运行，详见[说明](mission-observer.md)。 |
| 可选原始代价地图记录 | 同一观察器测试，新增 `--costmaps` | 合计 34/34 纯测试通过，WSL 用真实 Jazzy `Costmap` 消息构造验证通过且未初始化节点；原始 253/254/255 保留，两路独立限频、时间与序号明确、缺图报错。实时订阅仍待下一轮，不能仅凭纯测试称在线已通过。 |
| 定位跳变保护与末端旋转诊断 | run12，实验 GoalAngle 阈值 .05 m，其他控制参数保持 | 目标 1 成功；目标 2 到达 XY 容差后仍在旋转时 90 wall 秒预算到期。目标 3 因 map→odom 约 2.23 m 跳变受控取消，历史坐标无效；同时间原生车体与 odom 仅正常前进约 2 cm。末 29.999 sim 秒原生静止，全部进程退出，未返航；[详细记录](autonomous-run12.md)。 |

二维测试使用精确位姿和理想局部观测；它不含真实图像识别误差、SLAM 漂移、物理碰撞或轮胎滑移。当前 Gazebo 使用 llvmpipe 软件渲染；SLAM/Nav2 开启后的实测实时率约 0.49，不能据此宣称已启用 GPU 加速。

GitHub Actions 的提交 `db6c9d7` [运行通过](https://github.com/optpessmistic/LIMO_Map_Nav_FromSimToRel/actions/runs/40764090390)。该 CI 验证纯逻辑与离线工具，不含 ROS/Gazebo；本机 ROS/物理验证须分别报告，后续修改需核对对应提交的检查结果。

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
