# SLAM Toolbox 2.8.5 占用栅格边界补丁

核实日期：2026-10-28。本补丁仅让占用栅格包含其最大量化索引，不修改扫描、图优化、地图原点、阈值或规划算法。它不解决此前独立存在的扫描匹配漂移，也不代表自主任务已通过。

## 版本与原理

官方 [2.8.5](https://github.com/SteveMacenski/slam_toolbox/tree/2.8.5) 固定提交为 `ec8f7635dea317b531c419f798f87d90a336f32e`。源代码及原有许可证保存在忽略的 `vendor/slam_toolbox` 中；项目只跟踪补丁、获取脚本和回归测试。已安装原版为 `ros-jazzy-slam-toolbox 2.8.5-1noble.20260915.204151`，未被覆盖。官方 tag 的原始 Karto.h 与本机 apt 头文件 SHA256 相同：`277fd3ddbb4c655408660e7bab8ff39f49875af709bd7a83ff3d029190d44052`。

[ComputeDimensions](https://github.com/SteveMacenski/slam_toolbox/blob/ec8f7635dea317b531c419f798f87d90a336f32e/lib/karto_sdk/include/karto_sdk/Karto.h#L6090) 原实现取 `round(span / resolution)` 作为宽、高；[WorldToGrid](https://github.com/SteveMacenski/slam_toolbox/blob/ec8f7635dea317b531c419f798f87d90a336f32e/lib/karto_sdk/include/karto_sdk/Karto.h#L4421) 也用同一 round 计算端点索引。因此最大端点的索引等于尺寸，而合法索引只到尺寸减一。射线经过的合法单元仍增加 pass 计数，但 [RayTrace](https://github.com/SteveMacenski/slam_toolbox/blob/ec8f7635dea317b531c419f798f87d90a336f32e/lib/karto_sdk/include/karto_sdk/Karto.h#L6202) 不给非法端点增加 hit；[UpdateCell](https://github.com/SteveMacenski/slam_toolbox/blob/ec8f7635dea317b531c419f798f87d90a336f32e/lib/karto_sdk/include/karto_sdk/Karto.h#L6244) 随后可能把边缘墙前的单元标为 free。

补丁仅将两轴尺寸改为 `round(span / resolution) + 1`。这一改法与既有量化约定一致：所有包围盒内点的索引都在分配范围内。单独改成 ceil 对整数跨度仍少一格；把越界点 clamp 到旧边缘会改变测量位置，因此未采用。原点、已存在索引和每条射线不变；新增行、列仍由真实扫描的 pass/hit 计数生成，未向地图填入墙体真值。

上游 [issue #689](https://github.com/SteveMacenski/slam_toolbox/issues/689) 报告了相同边界缺墙现象，2026-10-28 API 查询仍为 open。维护者指出应检查占用栅格边界；讨论中的 clamp 建议并非本项目采用的方法。当日默认 `ros2` 分支头文件也仍使用未加一的尺寸公式。这里没有向上游提交 issue、评论或 PR。

## run6 离线证据

保存的 `outputs/ros_smoke/run6-after-survey-snapshot.json` 为 81×23、分辨率 0.050000000745 m，原点 `(-0.433028549242, -0.737915966181)`，map→odom 平面旋转约零。OccupancyGrid 的行号随 y 增大；顶行 row22 中心 y=0.387084050583，78 格 free，只有列 0、1、80 为 occupied，对应左右端墙。中间整段上墙确实缺失，并非绘图上下翻转。

`worlds/smoke_maze.world` 中 `wall_0_0` 至 `wall_6_0` 的下表面均为 y=0.4 m，连续覆盖 x=-1.2 至 4.4 m；run6 Gazebo 日志记录加载的就是该 world。名义上墙端点按既有量化落在 row23，超过合法 0..22。下墙 y=-0.4 则落在合法 row7，该行有 63 格 occupied。当前文件哈希仅证明离线所用输入，不能回溯证明启动时未记录的文件哈希。

单帧扫描 stamp106.125 与停车后的最近 pose stamp106.2 投影对照：335 束预计击中上墙，315 束（94.0%）量化到非法 row23，20 束进入 row22；端点 y 范围 0.372362..0.435860 m，均值 0.400378 m。该近时刻对照用于解释几何，不是假称重建了整段历史 pass/hit 计数。雷达高斯距离噪声仍为 σ=0.01 m。

快照 SHA256：`ac529cceec1993595f1817c070f104a9e60e15515f65ec052c491ae3aaa471dc`。世界文件 SHA256：`1ef2017583d448e36fb35825bbef6735536c2fe3cbf3a7a57380cf365c708811`。

## 获取与独立构建

在项目根目录的 WSL Bash 中执行。获取脚本拒绝错误 origin、错误提交、意外源码改动和不匹配的补丁；重复执行只校验，不重置已有文件。

```bash
bash scripts/fetch_slam_vendor.sh
bash scripts/fetch_slam_vendor.sh --check-only
source /opt/ros/jazzy/setup.bash
python3 - <<'PY'
from pathlib import Path
import shutil
root = Path.cwd()
source = root / 'vendor/slam_toolbox'
destination = root / 'outputs/slam-boundary-ws/src/slam_toolbox'
# Refuse an existing destination; do not silently overwrite a previous build.
shutil.copytree(source, destination, ignore=shutil.ignore_patterns('.git', 'COLCON_IGNORE'))
PY
cd outputs/slam-boundary-ws
CMAKE_BUILD_PARALLEL_LEVEL=2 MAKEFLAGS=-j2 colcon build \
  --base-paths src --packages-select slam_toolbox --parallel-workers 1 \
  --allow-overriding slam_toolbox --cmake-args -DBUILD_TESTING=OFF
```

构建只写此独立工作区的 `build`、`install`、`log`；完整包包含 Karto、SLAM 节点、服务和插件。主项目的 install 与 `/opt/ros/jazzy` 不变。源码快照携带上游 LICENSE 和文件内许可证。`vendor/slam_toolbox/COLCON_IGNORE` 防止普通 colcon 意外将其加入主工作区。

只有完成构建和回归后，才在新的受控实验进程中 source 此 overlay：

```bash
source scripts/env.sh
source install/setup.bash
source outputs/slam-boundary-ws/install/local_setup.bash
ros2 pkg prefix slam_toolbox
```

预期 prefix 是 `outputs/slam-boundary-ws/install/slam_toolbox` 的绝对路径。不能仅凭 source 命令成功证明加载正确；实验同时记录该 prefix 和运行进程所加载的 `libtoolbox_common.so`/`libkartoSlamToolbox.so` 路径。现有进程不会因新 shell source 自动切换。回到原版需新 shell，只 source `/opt/ros/jazzy` 和主项目 overlay。

## 原生边界回归

`tests/native/test_karto_grid_boundary.cpp` 直接调用 `OccupancyGrid::CreateFromScans`，执行实际 `ComputeDimensions`、分配、AddScan/RayTrace 和 Update 路径；不是手工创建大一格地图后直接填数据。5 个案例覆盖正负方向、四个角、整格跨度、非整数跨度、正负原点。每例检查 8 个边界端点都在范围内且为 occupied，并验证中央射线经过区域为 free、原点仍等于包围盒最小坐标。

原版与补丁分别编译同一回归源文件。下列测试从尚未 source 补丁 overlay 的新 WSL shell 开始，避免 `LD_LIBRARY_PATH` 让原版对照误加载补丁库。原版命令使用 `--expect-stock-failure`，将“5 例均暴露缺陷”作为预期对照；JSON 中各例 passed 仍明确为 false。补丁命令要求全部通过。

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p outputs/ros_smoke
g++ -std=c++17 -O0 tests/native/test_karto_grid_boundary.cpp \
  -I/opt/ros/jazzy/include -I/usr/include/eigen3 \
  -L/opt/ros/jazzy/lib -Wl,-rpath,/opt/ros/jazzy/lib \
  -lkartoSlamToolbox -lboost_serialization -o outputs/ros_smoke/karto_grid_stock
outputs/ros_smoke/karto_grid_stock --expect-stock-failure \
  > outputs/ros_smoke/karto_grid_stock_origin.json
patched="$PWD/outputs/slam-boundary-ws/install/slam_toolbox"
source outputs/slam-boundary-ws/install/local_setup.bash
g++ -std=c++17 -O0 tests/native/test_karto_grid_boundary.cpp \
  -I"$patched/include" -I/usr/include/eigen3 \
  -L"$patched/lib" -Wl,-rpath,"$patched/lib" \
  -lkartoSlamToolbox -lboost_serialization -o outputs/ros_smoke/karto_grid_patched
outputs/ros_smoke/karto_grid_patched > outputs/ros_smoke/karto_grid_patched.json
```

2026-10-28 本机实测：完整包构建成功，耗时 11 分 44 秒；没有安装额外依赖。原版 5 例全部复现缺陷，每例 8 个端点中 5 个越界；补丁 5 例全部通过，40 个端点全部合法且 occupied，`center_free` 和 `origin_preserved` 全部为 true。run6 尺寸形状案例从 81×23 变为 82×24。使用 `BUILD_TESTING=OFF` 构建完整包，然后单独执行以上原生回归；未声称运行全部上游测试。

原始原版结果 `outputs/ros_smoke/karto_grid_stock.json` 保留不变。含原点检查的新对照为 `karto_grid_stock_origin.json`，补丁结果为 `karto_grid_patched.json`，都在同一输出目录。`karto_grid_stock_origin_ldd.txt` 确认原版加载 `/opt/ros/jazzy/lib/libkartoSlamToolbox.so`；`karto_grid_patched_ldd.txt`、`slam_boundary_common_ldd.txt` 与 `slam_boundary_node_ldd.txt` 确认补丁测试、common 库和 async 节点解析到独立 install 的 Karto/common 库，未发现缺失依赖。`slam_boundary_prefix.txt` 记录的包前缀是 `/mnt/f/study/Personal_Plan/LIMO_Map_Nav_FromSimToRel/outputs/slam-boundary-ws/install/slam_toolbox`。这属于启动前链接检查，运行中的 `/proc/<pid>/maps` 仍由场景实验核实。

补丁安装头文件 SHA256：`3a91277a6fb16bd5796f41d484c2d405384f9bcfef6956f90511fa7c4391512c`；`libkartoSlamToolbox.so`：`3078c8ea6bfd2f979c38a8de4079a726028304bb7632409d6a96b3b2bb44f515`；`libtoolbox_common.so`：`beb1c4f416324a39a50a6b729aa9e83f8cbec772f08f34c34cab71b5a0c95047`。完整记录在 `slam_boundary_install_sha256.txt`；构建日志在 `outputs/slam-boundary-ws/build.log`。

先前独立最小复现也保留：原版 y=+0.4 端点越界、最后合法行 pass=4/hit=0 为 free，y=-0.4 为 occupied；局部 +1 尺寸对照上端点 pass=8/hit=4 为 occupied，输出为 `outputs/ros_smoke/karto_boundary_probe_result.log`。下一步是在独立场景轮次加载补丁，比较实际地图边界；当前回归通过本身不等同于该场景或导航任务通过。
