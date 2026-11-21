# moving_free 图像核对

Nano 现场使用 `config_motion_ruler_digits_20261121.json`。对应的 digits 全帧回放在 895 帧上有 462 帧有效；`new_moving_free/measurements.csv` 的 13 帧有效来自第一套新配置。先前把这 13 帧和现场的差异归到了处理序列，遗漏了配置变更。

## 相同 digits 配置

按原片帧表校验每条日志的帧号和采集时间戳。独立复跑使用 frozen_v1/Jetson 的 digits 配置和 motion_observe，在图像进入参考定位和跟踪器之前，只保留 Nano 实际处理的 366 个 JPEG，按原始顺序和时间戳从头运行。

| 对照口径 | 总帧 | 有效 | 无效 |
|---|---:|---:|---:|
| digits 原始全帧回放 | 895 | 462 | 433 |
| 全帧处理后按现场帧号筛选 | 366 | 188 | 178 |
| 仅输入现场处理过的帧独立复跑 | 366 | 159 | 207 |
| Nano 现场 | 366 | 159 | 207 |

子序列复跑与现场的 366 个有效位逐帧一致；参考有效位、轨沿原因、检测原因、跟踪原因和参考定位模式也逐帧一致。159 个有效球心的最大二维差为 2.3 × 10⁻¹³ px。[子序列原始回放输出](digits_moving_free_subsequence.csv)

全帧处理后再筛选的 188 帧中，157 帧与现场共同有效，31 帧仅全帧回放有效；另有 2 帧仅现场有效，176 帧双方无效。在这 31 帧中，现场有 30 帧正在建立跟踪，第 195 帧拒绝身份；第 323、676 帧现场有效，全帧回放拒绝身份。保持配置并复现输入序列后，上述 33 个有效位差异全部消失。另有 274 个 digits 全帧有效输出位于 Nano 跳过的帧号上。

## 错误目标核对

从原片逐格查看 digits 全帧、现场和子序列的全部有效位置。三路并集为 464 帧，共同有效位置的坐标一致到远小于一个像素，因此每个帧号只需一个裁图。以黄线投影交点是否落在可见钢珠上为判据，464 格均在钢珠上，未见误认纸尺、端挡或轨道反光。快速段的高光有横向拖影，本次核对仅判断目标身份。图中 F、N、S 分别为 digits 全帧、Nano 及现场子序列复跑。[逐帧三路坐标、有效位、目视结论与截图索引](digits_frame_review.csv)

[核对图 1](digits_positions_1.jpg) · [图 2](digits_positions_2.jpg) · [图 3](digits_positions_3.jpg) · [图 4](digits_positions_4.jpg) · [图 5](digits_positions_5.jpg) · [图 6](digits_positions_6.jpg)

## 第一套配置的历史对照

frozen 旧版有效 0/895，第一套新配置有效 13/895。第一套新配置与 Nano digits 只有 4 帧共同有效。此前记录的 155 个仅 Nano 有效帧包含配置和处理时序两项差异：第一套新配置在其中 136 帧参考失效、19 帧仍在建立跟踪。旧图中的 N 或 new 表示第一套新配置，L 或 live 表示现场 digits。[无标记原片](moving_free_unmarked.jpg) · [历史网格](moving_free_grid.jpg) · [历史变化与现场失败](moving_free_changes.jpg) · [历史样本索引](moving_free_samples.csv)
