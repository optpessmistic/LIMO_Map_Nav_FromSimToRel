# run20 原始证据

代码 `b80aa8de6799520b16fb7f70dcb1ac43d21d5f2a`，epoch `76f2f4d2-461c-469f-b8b4-40405854f92d`。运行与全部自有进程关闭已结束。5 个探索目标、2 个红色替身和返航完成；视觉覆盖及完整成功仍为 false。

**独立 RGB-D 末帧审计为 0/5 完整一致**，四段缺 depth、一段缺 RGB；五组采样时刻 TF 一致。在线 ledger 的 5×3 帧计数不等于原始图像已独立复验。录包累计 transport lost=2355。perception 原始退出1保留；其他进程与该进程组均已清空。

结果与限制见 [实验说明](../../docs/autonomous-run20.md)和[独立复核](run20-outcome-review.md)。`archive-manifest.json` 包含125份源材料的原始/归档 SHA256，以及留在本机的 MCAP、posegraph 和 data 哈希。gzip 条目解压后与原文件逐字节一致；原始输出未改。

`SHA256SUMS` 校验原始存档及归档清单，不含本 README。`runner/*.txt` 是包含本机路径的精确审计副本，不是通用启动入口。`inspection-audit` 保留缺失项及可取得的图像、CDR、标定和 TF，未用其他帧替换缺失样本。

实时窗口启动见 [可视化说明](../../docs/visualization.md)。本目录的图像是离线结果。
