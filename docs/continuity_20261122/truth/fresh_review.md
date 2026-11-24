# 新现场原片可见性和对象身份复核

| 场景 | 采集 / 处理 | 可见钢珠 | inside / outside / border | 全段球心 x 范围（px） | 已处理界内正确 / 已处理界内 | 全部界内正确 / 全部界内 |
|---|---:|---:|---:|---:|---:|---:|
| moving_dual_live | 895 / 667 | 895 | 895 / 0 / 0 | 302–302 | 663/667 | 663/895 |
| parked_free_new | 895 / 773 | 895，含工具部分遮挡 | 712 / 175 / 8 | 98–796.5 | 548/625 | 548/712 |
| moving_free_new | 895 / 762 | 895 | 720 / 74 / 101 | 172.5–530.5 | 434/614 | 434/720 |
| fixed_O_observed | 895 / 802 | 895，贴固定挡具 | 895 / 0 / 0 | 625–625 | 798/802 | 798/895 |

本表的输出只引用各段同期 measurements.csv。parked_free_new 的现场程序来自中间冻结版本。两段名带 moving 的录像都没有独立的车速真值，操作者的车辆运动时序待确认。

- **moving_dual_live** 的 30.007 秒内，球始终在左侧同一段轨道，整数像素中心 (302,575)，当帧球面模板的 NCC 最低 0.976。初标曾把 336–340、495、497 帧指到右侧塑料反光；已在原片中核对左侧球体并重新从当帧定位。现场最长连续界内缺测为原始 0–4 帧，跨度 0.143 秒。该录像检验的是静止可见球。
- **parked_free_new** 先由无标记全轨图目视定位，再将全帧裁片与球面、端挡、手和工具比较。0–139 和 382–415 帧用原图人工粗锚点限定复读窗口，逐帧用当帧球面细化球心。细金属杆压住部分球体，在推球时还有明显拖影。两段的 center_method 已逐帧记录。576–894 帧的球心 x 均在最终位置的 5 px 内。
- **moving_free_new** 全段可见同一颗球，在轨道内多次折返。相邻情形中虽然球会越左界，仍可在全轨原图看见；74/895 帧明确越界，另有 101/895 帧在边界不确定带。
- **fixed_O_observed** 的独立像素真值为名义 O 处的固定球，中心 (625,602)。纸尺和球的相对位移在整数像素下为零。绝对机械零点及该机位的厘米误差没有独立验证。

### 有效标记是否落到钢珠上

人工复看所有新现场超过 12 px 的有效坐标，以及所有位于 border、outside 或 unknown 帧的有效输出。fresh_live/output_review.csv 给出全部条目及图块编号。对应 *_outputs.jpg 左图以图像真值为中心，右图以现场输出为中心；绿十字为真值，品红叉为输出。两张图来自同一帧。

| 场景 | 界内超 12 px | 边界有效 | 界外有效 | 未知有效 | 人工判读 |
|---|---:|---:|---:|---:|---|
| parked_free_new | 10 | 0 | 0 | 0 | 10 个输出均在可见球体或其运动模糊下缘。最大误差 20.6 px，帧 157。 |
| moving_free_new | 13 | 34 | 0 | 0 | 13 个超容差标记仍在球面或下缘拖影上。34 个边界有效标记均落在球上。 |
| moving_dual_live、fixed_O_observed | 0 | 0 | 0 | 0 | 全部有效输出均在容差内。 |

这 57 个特别复核的标记中，未确认有输出落在手、工具、空轨或其他独立物体上。定位误差是相对可见球面中心的距离。单张拖影图不足以反推曝光瞬间的精确几何中心。完整分母及缺测时间戳见 [现场评分](fresh_live/scoreboard.md) 和 [机器可读数据](fresh_live/scoreboard.csv)。

```bash
python3 -B "$truth/fresh_scene.py" preview "$new" parked_free_new moving_free_new moving_dual_live
python3 -B "$truth/fresh_scene.py" label "$new" parked_free_new moving_free_new moving_dual_live --seed-runs "$old"
python3 -B "$truth/scoreboard.py" --runs "$new" --scene parked_free_new --scene moving_free_new --scene moving_dual_live --scene fixed_O_observed --out "$truth/fresh_live"
python3 -B "$truth/review_outputs.py" "$new" parked_free_new moving_free_new moving_dual_live fixed_O_observed --out "$truth/fresh_live"
```

三段自由球原始 MJPEG SHA-256 依次为 parked_free_new 的 `12161aca203e480759c81a093f1a0998a933cb8c8ae4b9db119e5139a7f44c39`，moving_free_new 的 `ca097ab399453cabaa67b0ebab8abcb9bc97e41bf4442515d2c3c6c5ffca9691`，moving_dual_live 的 `d58ab2fe4bc38aecdacfdebe9f253cbb56962b0fa73edee1fe6073191caf0487`。新停车与双配置段的专门复读只用于上述匹配散列的录像。
