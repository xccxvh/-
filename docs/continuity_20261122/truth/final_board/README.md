# 最终实板现场结果

三段均由原始图像独立标注，全部 2661 帧可辨认钢珠。厘米有效计数来自该段现场日志的 cm_valid。界内中心正确使用原图欧氏误差 ≤12 px。每段的同期现场、1e30933 基线和冻结 release_points 使用相同处理帧号和读帧时间。[四种分母与逐模式计数](scoreboard.csv) · [评分表与人工对象身份](scoreboard.md)

| 录像 | 采集 / 处理 | 界内 / 界外 / 边界 / 未知 | 现场正确 / 全部界内 | 现场正确 / 处理界内 | 现场界内超容差 | 现场界外 / 边界有效 | 现场厘米有效 / 处理 |
|---|---:|---:|---:|---:|---:|---:|---:|
| fixed_m5_final | 895 / 691 | 895 / 0 / 0 / 0 | 615/895 | 615/691 | 2 | 0 / 0 | 617/691 |
| parked_final | 871 / 809 | 838 / 22 / 11 / 0 | 710/838 | 710/780 | 0 | 0 / 0 | 705/809 |
| moving_final | 895 / 677 | 634 / 134 / 127 / 0 | 374/634 | 374/487 | 14 | 0 / 37 | 269/677 |

三段全时轴内，最长连续界内未得到正确位置分别为 46 帧（329–374，1.518 秒）、71 帧（145–215，3.141 秒）、19 帧（23–41，0.603 秒）。停车段移开挡具时球仍在轨道内，曝光变化和手入镜期间的球心已从原图复读。行驶段球心 x 覆盖 140.5–470.5 px，界外也仍可看见球。

### −5 试验与滑移

首帧钢珠贴黑色挡具右端，位于印刷尺 7 附近，球心 (349,584)。按每帧钢珠、局部刻纹和独立挡具纹理的匹配测得，第 0–409 帧球心相对刻纹的水平位移为 0 px。按试验记录从 410 帧起停止赋予名义 −5 cm。第 410 帧仍在初始像素位置，第 426 帧首次偏离超过 5 px，第 455 帧相对刻纹右移 99 px、相对挡具右移 82 px。第 600 帧曾接近初始位置，末帧仍比初始刻纹位置右移 23 px。第 410–894 帧仅提供实际图像位置。初始标记是摆放名义值，原图能判断相对滑移，无法独立证明绝对机械坐标。[刻纹与挡具原图](fixed_m5_marker.jpg) · [全部位置与位移](../fixed_m5_final.csv)

前 410 个采集帧有 339 帧进入处理，284 帧给出有效厘米值。仅在这 284 帧上，对名义 −5 cm 的最大绝对偏差为 0.2823 cm，中位输出 −4.9405 cm。后段有 333 个有效厘米输出，未对 −5 cm 计算准确度。

### 对象身份与冻结回放

现场 16 个界内超容差标记中，15 个仍落在钢珠或拖影上。固定 −5 第 410 帧标记靠右侧球缘与阴影，独立物体身份无法确认；保留为超容差且身份不确定。行驶段全部 37 个边界有效标记都在真球上。三段未确认手、工具或空轨错锁。按原图坐标去重后，已复看现场、基线及 release_points 的全部需核对输出。[固定 −5 判读图](fixed_m5_final_outputs.jpg) · [行驶判读图](moving_final_outputs.jpg) · [逐标记索引](output_review.csv)

基线 → release_points 的界内正确帧数分别为 609 → 615、710 → 710、362 → 374。三段 release_points 与现场的正确、超容差及界外有效计数一致。此处比较像素输出及其可用性；板上执行时间必须引用实际运行的源文件和配置。[冻结文件散列](../frozen_source.sha256) · [本次评分输入散列](score_inputs.csv)

从仓库根目录复现。变量 truth、old、new、paired 的值见上级 README。先核对无标记轨道，再运行标注及评分。

```bash
python3 -B "$truth/fresh_scene.py" preview "$new" parked_final moving_final
python3 -B "$truth/fresh_scene.py" label "$new" parked_final moving_final --seed-runs "$old"
python3 -B "$truth/label_minus5.py" "$new"
python3 -B "$truth/scoreboard.py" --runs "$new" --paired "$paired" --points-only-comparison --scene fixed_m5_final --scene parked_final --scene moving_final --out "$truth/final_board" --release-manifest "$truth/frozen_source.sha256"
python3 -B "$truth/review_outputs.py" "$new" fixed_m5_final parked_final moving_final --paired "$paired" --points-only-comparison --out "$truth/final_board"
```
