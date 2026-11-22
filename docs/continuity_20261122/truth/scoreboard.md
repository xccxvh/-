# 同帧钢珠像素输出评分

正确位置采用原始 1280×720 图像球心欧氏距离 ≤ 12 px。界内正确率分母含该视图全部明确界内帧。界外有效表示真球心已越界仍发出有效位置。原始时轴中的跳帧计缺测。处理子集取现场 measurements.csv 帧号。

## 全部采集帧

| 场景 | 输出 | 帧数 | 界内正确/界内 | 界内有效但错位 | 界外有效/界外 | 边界有效/边界 | 未知有效/未知 | 最长连续界内缺测（帧数，首末帧，采集跨度） |
|---|---|---:|---:|---:|---:|---:|---:|---|
| moving_fast_confirmed | live | 895 | 390/654 | 22 | 0/177 | 17/64 | 0/0 | 14，111–124，0.440 s |
| moving_fast_confirmed | baseline | 895 | 390/654 | 22 | 0/177 | 17/64 | 0/0 | 14，111–124，0.440 s |
| moving_fast_confirmed | release_digits | 895 | 398/654 | 23 | 0/177 | 20/64 | 0/0 | 14，111–124，0.440 s |
| moving_fast_confirmed | release_points | 895 | 398/654 | 23 | 0/177 | 20/64 | 0/0 | 14，111–124，0.440 s |
| parked_free | live | 895 | 25/283 | 0 | 0/596 | 1/16 | 0/0 | 65，55–119，2.151 s |
| parked_free | baseline | 895 | 68/283 | 0 | 0/596 | 4/16 | 0/0 | 39，195–233，1.277 s |
| parked_free | release_digits | 895 | 68/283 | 0 | 0/596 | 4/16 | 0/0 | 39，195–233，1.277 s |
| parked_free | release_points | 895 | 68/283 | 0 | 0/596 | 4/16 | 0/0 | 39，195–233，1.277 s |
| moving_free | live | 895 | 151/600 | 0 | 0/214 | 8/81 | 0/0 | 33，742–774，1.071 s |
| moving_free | baseline | 895 | 151/600 | 0 | 0/214 | 8/81 | 0/0 | 33，742–774，1.071 s |
| moving_free | release_digits | 895 | 151/600 | 0 | 0/214 | 8/81 | 0/0 | 33，742–774，1.071 s |
| moving_free | release_points | 895 | 167/600 | 0 | 0/214 | 8/81 | 0/0 | 33，742–774，1.071 s |
| fixed_O_tilt | live | 895 | 87/895 | 0 | 0/0 | 0/0 | 0/0 | 780，0–779，26.143 s |
| fixed_O_tilt | baseline | 895 | 675/895 | 0 | 0/0 | 0/0 | 0/0 | 12，866–877，0.381 s |
| fixed_O_tilt | release_digits | 895 | 698/895 | 0 | 0/0 | 0/0 | 0/0 | 5，0–4，0.144 s |
| fixed_O_tilt | release_points | 895 | 708/895 | 0 | 0/0 | 0/0 | 0/0 | 5，0–4，0.144 s |

## 现场处理子集

| 场景 | 输出 | 帧数 | 界内正确/界内 | 界内有效但错位 | 界外有效/界外 | 边界有效/边界 | 未知有效/未知 | 最长连续界内缺测（帧数，首末帧，采集跨度） |
|---|---|---:|---:|---:|---:|---:|---:|---|
| moving_fast_confirmed | live | 833 | 390/614 | 22 | 0/160 | 17/59 | 0/0 | 12，111–124，0.440 s |
| moving_fast_confirmed | baseline | 833 | 390/614 | 22 | 0/160 | 17/59 | 0/0 | 12，111–124，0.440 s |
| moving_fast_confirmed | release_digits | 833 | 398/614 | 23 | 0/160 | 20/59 | 0/0 | 12，111–124，0.440 s |
| moving_fast_confirmed | release_points | 833 | 398/614 | 23 | 0/160 | 20/59 | 0/0 | 12，111–124，0.440 s |
| parked_free | live | 528 | 25/155 | 0 | 0/361 | 1/12 | 0/0 | 44，55–117，2.088 s |
| parked_free | baseline | 528 | 68/155 | 0 | 0/361 | 4/12 | 0/0 | 27，195–233，1.277 s |
| parked_free | release_digits | 528 | 68/155 | 0 | 0/361 | 4/12 | 0/0 | 27，195–233，1.277 s |
| parked_free | release_points | 528 | 68/155 | 0 | 0/361 | 4/12 | 0/0 | 27，195–233，1.277 s |
| moving_free | live | 366 | 151/246 | 0 | 0/89 | 8/31 | 0/0 | 14，742–773，1.040 s |
| moving_free | baseline | 366 | 151/246 | 0 | 0/89 | 8/31 | 0/0 | 14，742–773，1.040 s |
| moving_free | release_digits | 366 | 151/246 | 0 | 0/89 | 8/31 | 0/0 | 14，742–773，1.040 s |
| moving_free | release_points | 366 | 167/246 | 0 | 0/89 | 8/31 | 0/0 | 14，742–773，1.040 s |
| fixed_O_tilt | live | 725 | 87/725 | 0 | 0/0 | 0/0 | 0/0 | 628，0–779，26.143 s |
| fixed_O_tilt | baseline | 725 | 675/725 | 0 | 0/0 | 0/0 | 0/0 | 9，866–877，0.381 s |
| fixed_O_tilt | release_digits | 725 | 698/725 | 0 | 0/0 | 0/0 | 0/0 | 4，533–537，0.124 s |
| fixed_O_tilt | release_points | 725 | 708/725 | 0 | 0/0 | 0/0 | 0/0 | 4，0–3，0.103 s |

## 超容差或非明确界内的有效标记人工核对

| 场景 | 输出 | 需核对 | 真球或球缘拖影 | 确认错物 | 未确认身份 |
|---|---|---:|---:|---:|---:|
| moving_fast_confirmed | live | 39 | 39 | 0 | 0 |
| moving_fast_confirmed | baseline | 39 | 39 | 0 | 0 |
| moving_fast_confirmed | release_digits | 43 | 43 | 0 | 0 |
| moving_fast_confirmed | release_points | 43 | 43 | 0 | 0 |
| parked_free | live | 1 | 1 | 0 | 0 |
| parked_free | baseline | 4 | 4 | 0 | 0 |
| parked_free | release_digits | 4 | 4 | 0 | 0 |
| parked_free | release_points | 4 | 4 | 0 | 0 |
| moving_free | live | 8 | 8 | 0 | 0 |
| moving_free | baseline | 8 | 8 | 0 | 0 |
| moving_free | release_digits | 8 | 8 | 0 | 0 |
| moving_free | release_points | 8 | 8 | 0 | 0 |
| fixed_O_tilt | live | 0 | 0 | 0 | 0 |
| fixed_O_tilt | baseline | 0 | 0 | 0 | 0 |
| fixed_O_tilt | release_digits | 0 | 0 | 0 | 0 |
| fixed_O_tilt | release_points | 0 | 0 | 0 | 0 |

## 相对 baseline 的配对变化

每行比较同一现场处理子集。增减为帧数；全部采集帧视图包含共同跳过的帧。

| 场景 | 配置 | 界内正确变化 | 界内错位变化 | 界外有效变化 | 最长连续界内缺测变化（帧） |
|---|---|---:|---:|---:|---:|
| moving_fast_confirmed | release_digits | +8 | +1 | +0 | +0 |
| moving_fast_confirmed | release_points | +8 | +1 | +0 | +0 |
| parked_free | release_digits | +0 | +0 | +0 | +0 |
| parked_free | release_points | +0 | +0 | +0 | +0 |
| moving_free | release_digits | +0 | +0 | +0 | +0 |
| moving_free | release_points | +16 | +0 | +0 | +0 |
| fixed_O_tilt | release_digits | +23 | +0 | +0 | -5 |
| fixed_O_tilt | release_points | +33 | +0 | +0 | -5 |

完整 CSV 同时列出界内有效、明确无效、跳帧、界外命中真球或错位、无中心的有效输出、按帧数和按时长各自最严重的区间及其两端原始 read_s。非界内帧始终中断区间。子集跨度可跨过未处理的界内帧，首末同帧的跨度为零。
