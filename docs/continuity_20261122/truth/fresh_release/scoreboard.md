# 同帧钢珠像素输出评分

正确位置采用原始 1280×720 图像球心欧氏距离 ≤ 12 px。界内正确率分母含该视图全部明确界内帧。界外有效表示真球心已越界仍发出有效位置。原始时轴中的跳帧计缺测。处理子集取现场 measurements.csv 帧号。

## 全部采集帧

| 场景 | 输出 | 帧数 | 界内正确/界内 | 界内有效但错位 | 界外有效/界外 | 边界有效/边界 | 未知有效/未知 | 最长连续界内缺测（帧数，首末帧，采集跨度） |
|---|---|---:|---:|---:|---:|---:|---:|---|
| parked_free_new | live | 895 | 548/712 | 10 | 0/175 | 0/8 | 0/0 | 28，438–465，0.903 s |
| parked_free_new | baseline | 895 | 548/712 | 8 | 0/175 | 0/8 | 0/0 | 28，438–465，0.903 s |
| parked_free_new | release_points | 895 | 549/712 | 9 | 0/175 | 0/8 | 0/0 | 28，438–465，0.903 s |
| moving_free_new | live | 895 | 434/720 | 13 | 0/74 | 34/101 | 0/0 | 17，50–66，0.537 s |
| moving_free_new | baseline | 895 | 434/720 | 13 | 0/74 | 34/101 | 0/0 | 17，50–66，0.537 s |
| moving_free_new | release_points | 895 | 434/720 | 13 | 0/74 | 34/101 | 0/0 | 17，50–66，0.537 s |
| moving_dual_live | live | 895 | 663/895 | 0 | 0/0 | 0/0 | 0/0 | 5，0–4，0.143 s |
| moving_dual_live | baseline | 895 | 663/895 | 0 | 0/0 | 0/0 | 0/0 | 5，0–4，0.143 s |
| moving_dual_live | release_points | 895 | 663/895 | 0 | 0/0 | 0/0 | 0/0 | 5，0–4，0.143 s |
| fixed_O_observed | live | 895 | 798/895 | 0 | 0/0 | 0/0 | 0/0 | 4，0–3，0.104 s |
| fixed_O_observed | baseline | 895 | 777/895 | 0 | 0/0 | 0/0 | 0/0 | 4，0–3，0.104 s |
| fixed_O_observed | release_points | 895 | 798/895 | 0 | 0/0 | 0/0 | 0/0 | 4，0–3，0.104 s |

## 现场处理子集

| 场景 | 输出 | 帧数 | 界内正确/界内 | 界内有效但错位 | 界外有效/界外 | 边界有效/边界 | 未知有效/未知 | 最长连续界内缺测（帧数，首末帧，采集跨度） |
|---|---|---:|---:|---:|---:|---:|---:|---|
| parked_free_new | live | 773 | 548/625 | 10 | 0/142 | 0/6 | 0/0 | 22，438–465，0.903 s |
| parked_free_new | baseline | 773 | 548/625 | 8 | 0/142 | 0/6 | 0/0 | 22，438–465，0.903 s |
| parked_free_new | release_points | 773 | 549/625 | 9 | 0/142 | 0/6 | 0/0 | 22，438–465，0.903 s |
| moving_free_new | live | 762 | 434/614 | 13 | 0/60 | 34/88 | 0/0 | 14，50–66，0.537 s |
| moving_free_new | baseline | 762 | 434/614 | 13 | 0/60 | 34/88 | 0/0 | 14，50–66，0.537 s |
| moving_free_new | release_points | 762 | 434/614 | 13 | 0/60 | 34/88 | 0/0 | 14，50–66，0.537 s |
| moving_dual_live | live | 667 | 663/667 | 0 | 0/0 | 0/0 | 0/0 | 4，0–4，0.143 s |
| moving_dual_live | baseline | 667 | 663/667 | 0 | 0/0 | 0/0 | 0/0 | 4，0–4，0.143 s |
| moving_dual_live | release_points | 667 | 663/667 | 0 | 0/0 | 0/0 | 0/0 | 4，0–4，0.143 s |
| fixed_O_observed | live | 802 | 798/802 | 0 | 0/0 | 0/0 | 0/0 | 4，0–3，0.104 s |
| fixed_O_observed | baseline | 802 | 777/802 | 0 | 0/0 | 0/0 | 0/0 | 4，0–3，0.104 s |
| fixed_O_observed | release_points | 802 | 798/802 | 0 | 0/0 | 0/0 | 0/0 | 4，0–3，0.104 s |

## 超容差或非明确界内的有效标记人工核对

| 场景 | 输出 | 需核对 | 真球或球缘拖影 | 确认错物 | 未确认身份 |
|---|---|---:|---:|---:|---:|
| parked_free_new | live | 10 | 10 | 0 | 0 |
| parked_free_new | baseline | 8 | 8 | 0 | 0 |
| parked_free_new | release_points | 9 | 9 | 0 | 0 |
| moving_free_new | live | 47 | 47 | 0 | 0 |
| moving_free_new | baseline | 47 | 47 | 0 | 0 |
| moving_free_new | release_points | 47 | 47 | 0 | 0 |
| moving_dual_live | live | 0 | 0 | 0 | 0 |
| moving_dual_live | baseline | 0 | 0 | 0 | 0 |
| moving_dual_live | release_points | 0 | 0 | 0 | 0 |
| fixed_O_observed | live | 0 | 0 | 0 | 0 |
| fixed_O_observed | baseline | 0 | 0 | 0 | 0 |
| fixed_O_observed | release_points | 0 | 0 | 0 | 0 |

## 相对 baseline 的配对变化

每行比较同一现场处理子集。增减为帧数；全部采集帧视图包含共同跳过的帧。

| 场景 | 配置 | 界内正确变化 | 界内错位变化 | 界外有效变化 | 最长连续界内缺测变化（帧） |
|---|---|---:|---:|---:|---:|
| parked_free_new | release_points | +1 | +1 | +0 | +0 |
| moving_free_new | release_points | +0 | +0 | +0 | +0 |
| moving_dual_live | release_points | +0 | +0 | +0 | +0 |
| fixed_O_observed | release_points | +21 | +0 | +0 | +0 |

完整 CSV 同时列出界内有效、明确无效、跳帧、界外命中真球或错位、无中心的有效输出、按帧数和按时长各自最严重的区间及其两端原始 read_s。非界内帧始终中断区间。子集跨度可跨过未处理的界内帧，首末同帧的跨度为零。
