# 原始帧钢珠位置真值和同帧评分

本目录包含历史四段、第一批新场景和最终实板三段。最终实板逐场覆盖、厘米有效数与 −5 滑移单列在 [最终实板报告](final_board/README.md)。前一批新录像的物理可见性见 [现场人工复核](fresh_review.md)，冻结回放见 [新景同帧评分](fresh_release/scoreboard.md)。历史四段使用 [现场与冻结三路对比](scoreboard.md)。所有评分的超容差/边界有效标记都另行人工核对目标身份，结果在 manual_identity.csv。

每个场景同名 CSV 与采集时钟一一对应。frame 从 0 起，原始 1280×720 图像左上为原点。自由球三段的每帧都能识别到球体。中心是曝光期间可见球体的像素中心估计，半像素小数来自缩放和多个模板取中位数。高速拖影限制瞬时中心精度。

| 场景 | 总帧数 | inside | outside | border | 中心可读 |
|---|---:|---:|---:|---:|---:|
| moving_fast_confirmed | 895 | 654 | 177 | 64 | 895 |
| parked_free | 895 | 283 | 596 | 16 | 895 |
| moving_free | 895 | 600 | 214 | 81 | 895 |
| fixed_O_tilt | 895 | 895 | 0 | 0 | 895 |
| fixed_O_observed（新采集） | 895 | 895 | 0 | 0 | 895 |

### 自由球的证据

逐个解码 raw.mjpg 内的 JPEG 并按帧号连接 frames.csv。先看全轨原片，从三段中目视选择 11 个模板中心，覆盖清晰球、横向拖影、手入镜和左端挡接触。每一帧独立在包含两侧端挡的整条轨道带做半分辨率归一化灰度模板匹配，合并相近位置和分数的峰。此过程只读取原片、采集时钟和已知参考尺几何。钢珠检测器、跟踪状态、有效位和坐标日志没有参与标注。停车段初标第 19 帧曾误跟到尺纹，经原片判读补入左挡样本，重新生成所有标注。

界内指球心位于随纸尺移动的搜索范围。参考图两端的横坐标为 255.080 和 1064.372 px，对应目标区加恢复余量后的 −7 至 +7 搜索锚点。13 个独立纸尺纹理窗口提供候选匹配点，通过 RANSAC 拟合当帧二维相似变换。signed_margin_px 是球心至最近边界的有符号水平距离，正为界内。边界带内统一记 border。基本半宽为 8 px，匹配分数低于 0.78 时为 13 px，另加尺纹拟合 RMS 向上取整。moving_free 的半宽下限为 18 px。模板和尺纹证据不足时允许 uncertain 或空中心。上述带宽是审计规则，缺少统计置信区间的含义。

- 三张 *_all.jpg 已目视核对，每行 32 个裁片，从左到右、从上到下。第 k 行第 j 列对应帧 32k+j，行列均从 0 起。三段共 2685 张，右下角最后一格为填充。每个裁片能看到钢珠或球体拖影。每段 *_examples.jpg 有完整横向轨道，蓝线标搜索边界，绿十字标估计中心。
- 每段另外做 12 次目视复读。八帧按固定帧号，其余取最低模板分、最靠边、最高相邻位移及最后一帧。*_reread.jpg 使用粗定位生成裁片，隐藏中心及 ROI，边缘短刻度每 10 px；manual_centers.csv 存复读原图坐标，reread_windows.csv 存裁片原点。相同复核者掌握裁片的大致位置，样本也有针对性，不能据此推算总体准确率。36 次中心差的中位数 2.55 px，最大 6.04 px。分段最大值依次为 6.04、5.15、5.52 px。快跑第 115、117 帧的钢珠明显横向拉长。
- check_ruler.py 用 SIFT 尺面特征独立重估变换。每 30 帧、最后一帧及全部 border 帧均入表，三段分别核对 93、47、109 帧。两法左界最大绝对差分别为 3.28、4.35、15.39 px，右界为 3.42、6.61、10.57 px。moving_free 对端点的估计最不稳定。两法共用参考锚点，无法发现参考尺本身的系统误差。逐帧结果见 ruler_crosscheck.csv。
- 快跑左端折返中，148–157 帧在界内，158–159 帧压边，160–194 帧在左界外，195 帧压边，196 帧明确回到界内。图像每一帧都能定位，border 保留了判断侧别的不确定性。

### 固定球

fixed_O_tilt 用第 0 帧钢珠和挡具的接触区域为模板。第 178 帧缺 JPEG 尾标，按 JFIF 起始位置和采集时钟保留该帧，球和尺纹仍可读。source_missing_eoi 记录这一情况。fixed_O_observed 先对从首到尾 12 张原图做等距判读，钢珠始终在挡具右端、12 刻线附近。再按第 0 帧目视中心 (625,602) 核查所有 895 帧：整数像素球心全程 (625,602)，与邻近尺面相对位移的 x、y 分量均为 0 px。球模板 NCC 最低 0.988，局部纸尺 NCC 最低 0.992。支持“名义 O 处固定”，证据精度为本机位的像素分辨率；绝对机械零点、实际倾角与厘米误差仍无独立测量。两段各有 *_examples.jpg。

### 重现本次标注

在本仓库根目录运行，使用 OpenCV 4.8.1、NumPy 1.24.4。比较替代参数时可用单独输出目录。输入 raw.mjpg 的 SHA-256 依表顺序如下。

```text
faf2cd6578b49b6a6da244ae6cdf590e3b94084d11d834aa3a6f0e6c66623c8b
964156e1b39819286d62a98edffabaa6c58683c6a302f2d7e368e0ad805aaf03
35fdd5d30e50a6cafbce3c5327602dfb7dad4c8c9e6d29f48b21298cb2f06322
98f1a56007f9a966939aca87a0dffb3f63695365c557c3cfe59ef989d730a2e7
428e07e82e470acdf34251ac8e8dc4a5fbaffd4616f3b6d432950880cc4b77c8
```

```bash
truth=docs/continuity_20261122/truth
old=/home/user/桌面/jetson_nano/7/motion_20261121/runs
new=/home/user/桌面/jetson_nano/8/continuity_20261122/runs
paired=/home/user/桌面/jetson_nano/8/continuity_20261122/paired
python3 -B "$truth/label_raw.py" "$old"
python3 -B "$truth/audit_images.py" "$old"
python3 -B "$truth/check_ruler.py" "$old"
python3 -B "$truth/label_fixed.py" "$old"
python3 -B "$truth/label_fixed.py" "$new" --scene fixed_O_observed --seed 625 602
```

### 新采集的自由球

两阶段 CLI 从 frames.csv 获取每个场景的帧数。先生成无输出叠加的全轨原片图，核对球体、端挡、尺面刻线、亮度及倾斜；再从历史原片提取模板，标注并复核。可一次给多个已采集的场景名，也可只给一个。

```bash
python3 -B "$truth/fresh_scene.py" preview "$new" parked_free_new moving_free_new
# 打开每段 *_preview.jpg 核对原片。
python3 -B "$truth/fresh_scene.py" label "$new" parked_free_new moving_free_new --seed-runs "$old"
# 查看 *_review.jpg 的全轨背景、两端边界及 *_all.jpg 的每一帧。
```

新景中亮度、反射和角度可能超出历史模板范围。若候选指向工具、手、尺纹或空轨，应回到该原帧重新读中心；不能读时保留 uncertain 并留空中心。新机位或更换纸尺时需要重新确认锚点。对于全新数据，可直接提供人工标注 frame、read_s、label、center_x_px、center_y_px 供评分。缺失真值行计入 unknown。

### 逐帧评分和新景现场日志

scoreboard.py 仅使用 Python 标准库。输入为同名真值 CSV、原始 frames.csv 与现场 measurements.csv，以及可选的三组回放。脚本检查重复帧、ID、原始时钟（允许 1 微秒舍入）和参与处理的帧集合。回放可含全帧或与现场相同的处理集合。本批提供的回放只处理了现场相同的子集。raw 视图给出所有采集帧的可用性，未处理的界内帧计缺测。processed 视图只对现场实际处理子集评分。分数只看实际 pixel_valid 和原图 x_px/y_px。points 配置输出的 measurement_x_px/y_px 在点畸变校正之前记录。

位置容差默认取欧氏距离 12 px，约为所见钢珠半径的一半，且高于三段复读样本的最大差。阈值附近的输出对读数敏感，可用 --tol-px 做敏感性比较。错位包含仍落在球身、但超过中心容差的输出。界外有效输出另列，可区分命中可见球或错位。border 和 unknown 各有分母和有效数。最长缺测按连续的明确界内帧计数，缺失输出、无效或错位都计入。任何界外、border、unknown 或无可读中心都会切断区间。跨度取首末缺测帧 read_s 之差，完整 CSV 同时给出按帧数和按跨度选出的区间及其绝对采集时刻。

```bash
# 四个历史场景。回放目录名固定为 baseline、release_digits、release_points。
python3 -B "$truth/scoreboard.py" --runs "$old" --paired "$paired" --truth "$truth" --out "$truth"
# 新场景已有原片、现场日志及同名真值，省略 --paired 可立即给出现场分数。
python3 -B "$truth/scoreboard.py" --runs "$new" --scene parked_free_new --scene moving_free_new --truth "$truth" --out "$truth/fresh_scores"
```

对新场景复现同一处理子集，可用上一层 paired_replay.py，在 baseline 版本 1e30933 及同一份冻结候选快照上各跑对应配置。以下变量 BASE_JETSON 和 RELEASE_JETSON 指向对应快照的 Jetson 目录。保留冻结快照的源文件 SHA-256 清单及运行所用配置。

```bash
scene=moving_free_new
capture="$new/$scene"
replay=docs/continuity_20261122/paired_replay.py
python3 -B "$replay" "$BASE_JETSON" config_motion_ruler_digits_20261121.json "$capture" "$paired/baseline/$scene.csv" --processed "$capture/measurements.csv"
python3 -B "$replay" "$RELEASE_JETSON" config_motion_ruler_digits_20261121.json "$capture" "$paired/release_digits/$scene.csv" --processed "$capture/measurements.csv"
python3 -B "$replay" "$RELEASE_JETSON" config_motion_dual_points_20261122.json "$capture" "$paired/release_points/$scene.csv" --processed "$capture/measurements.csv"
python3 -B "$truth/scoreboard.py" --runs "$new" --paired "$paired" --scene "$scene" --truth "$truth" --out "$truth/fresh_scores"
```

现场历史记录和当前回放使用不同处理配置时，以 baseline 为算法增量的比较起点。早期 paired/final_digits 和 paired/final_points 在并发编辑期间生成，缺少单一冻结源版本，已从正式评分中撤下。CSV 哈希标识该次输入；源码版本应以回放生成方的冻结清单为准。此次评分只测原帧输出覆盖与像素位置，耗时性能需在对应冻结源的独立运行中报告。


历史四段合计 2432 个明确界内帧，基线正确 1284，release_digits 正确 1315（+31），release_points 正确 1341（+57）。三组在 987 个明确界外帧上都没有有效输出。人工复核图存于 release_audit。新场景提供 baseline 与 release_points，使用 --points-only-comparison；同期 live 单列。冻结目录 final/Jetson 中 40 个顶层 Python、运动配置与参考资源的相对路径 SHA-256 清单为 frozen_source.sha256，清单 SHA-256 为 `e2ee09a803ef8117d01a1b9960c52c34f354450430fc185a54eae6468abcf5d7`。可在 final/Jetson 下运行 sha256sum -c 加清单绝对路径核对。现场实际执行的二进制/源文件身份须另以板上运行出处证明。
