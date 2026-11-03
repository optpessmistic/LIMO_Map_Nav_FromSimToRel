# 感知节点信号退出复验

run20 原始 perception 日志在 SIGINT 后重复调用 rclpy.shutdown，退出1。后续源码的 main 改用 try_shutdown，并处理 context 已停止时的 ExternalShutdownException；普通构造、callback 和销毁异常仍传播。

8项纯 AST lifecycle 回归通过；两个真实 ROS source-import 子进程分别接受 SIGINT/SIGTERM，均 main returned、退出0、无 Traceback/RCLError，精确 pidfd 信号且自有进程组清空。domain79 LOCALHOST，运行前有界空域检查通过；没有图像、Gazebo或运动。两次验证使用源码摘要见 inputs.json，当时修复尚未提交。

随后仅 limo_maze_perception 单包构建通过（包耗时7.61秒，colcon总20.7秒），安装 node.py 与源码 cmp 相同，安装路径导入成功。这不构成图像处理或自主仿真复验。

SHA256SUMS 校验复制的13份原始材料，不含本 README。child.py.txt/verify.py.txt 是本机审计副本。原始 run20 exit1 没有更改。
