> 2026-11-12 候选版更新：标定采集命令已改为人工确认的录像帧区间；新增默认关闭的 UART。
> 请先阅读 [当前使用说明](docs/JETSON_GUIDE.md) 和 [UART 协议](docs/UART_PROTOCOL.md)。
> 下文保留历史测试记录；旧 `calibrate.py add --cm ...` 命令已不适用。
> 尚未完成真实厘米标定、动态厘米坐标和闭环实测，几何亮斑误锁仍需验证。

# Jetson Nano 钢珠实时识别系统

2026 电赛 H 题《车载平衡滚球运动控制系统》的视觉识别部分。
当前阶段只做: 摄像头采集 → 轨道检测 → 钢珠识别 → 位置跟踪 →
坐标标定 → 可视化/记录。**不含**舵机、PID、无线图传。

## 1. 运行环境

- Jetson Nano, Ubuntu 18.04.6, L4T R32.7.3
- Python 3.6.9
- OpenCV 4.1.1 (NVIDIA 自带构建)
- numpy 1.19.5

本机 numpy 的 OpenBLAS 在 Cortex-A57 上会误判 CPU 类型, 直接运行会
`Illegal instruction (core dumped)`。**所有 Python 命令前必须加环境变量**:

```bash
export OPENBLAS_CORETYPE=ARMV8
```

没有升级 Ubuntu / JetPack / 系统 Python, 也没有替换已有 OpenCV。

摄像头当前 V4L2 参数(实测): 手动曝光 30 ms(`exposure_auto=1`,
`exposure_absolute=300`)、`gain=45`、自动白平衡开启、
`power_line_frequency=50 Hz`; 没有 V4L2 对焦控制, 对焦需手动拧镜头。
手动曝光是为了避免自动曝光在暗背景下把曝光拉到 60 ms、帧率降到
16.8 fps; 换机位/光照后需要重新检查曝光和增益, 不要盲目沿用。

## 2. 目录结构

```
ball_vision/
├── main.py             # 主程序: 采集/离线视频 + 检测 + 跟踪 + 输出
├── camera.py           # USB 摄像头(V4L2)/视频文件封装
├── rail_detector.py    # 轨道上缘直线检测(Sobel + RANSAC)
├── ball_detector.py    # 钢珠候选: 颜色 + 上缘凸起几何
├── ball_tracker.py     # 跨帧跟踪 + DETECTED/UNCERTAIN/LOST 状态机
├── calibration.py      # 像素 -> 厘米映射(线性/分段)
├── video_writer.py     # 异步 ffmpeg/libx264 标注视频写入
├── visualization.py    # 画面叠加
├── capture_raw.py      # 用 ffmpeg 保存摄像头原始 MJPG
├── set_camera.sh       # 固定当前机位的曝光/增益(30 ms, gain 45)
├── calibrate.py        # 采集 -5/0/+5 cm 标定点并拟合
├── run_pixel_tests.sh  # 无尺子阶段的像素级测试采集(静态/无球/滚动)
├── analyze_track.py    # 汇总 CSV: 检出率/抖动/跳变/延迟
├── config.json         # 全部参数
├── calibration.json    # 标定结果(存在且 ready 时才输出 x_cm)
├── recordings/         # 原始视频、标注视频、单帧
├── logs/               # CSV
└── README.md
```

## 3. 快速开始

```bash
cd ~/ball_vision
export OPENBLAS_CORETYPE=ARMV8

# 实时画面(需要 X11; SSH 用 ssh -X Jetson 登录)
python3 main.py --display

# 实时 + 保存标注视频 + CSV, 限时 10 秒
python3 main.py --display --record --csv --duration 10 --label live_720p

# 离线跑一段已录视频
python3 main.py --video recordings/A_static_center_try1_1080p.avi \
    --record --csv --label offline_test

# 当前机位: 固定 30 ms 曝光/增益, 避免自动曝光把帧率降到 16.8 fps
./set_camera.sh

# 保存原始画面(不经过 OpenCV 解码/编码)
python3 capture_raw.py --label no_ball --duration 5 \
    --width 1280 --height 720

# 汇总一段 CSV 的检出率/抖动/跳变/延迟
python3 analyze_track.py logs/track_xxx.csv
```

按 `q` 退出显示窗口。

## 4. 输出接口

`main.py` 每帧产生统一结果:

| 字段 | 说明 |
|---|---|
| `timestamp` | Unix 时间戳 |
| `x_px` | 钢珠中心像素横坐标; LOST 时为 `None` |
| `x_cm` | 标定后的厘米坐标; **未标定或非 DETECTED 时为 `None`** |
| `valid` | 仅 `DETECTED` 为 `true`, 预测/不确定均为 `false` |
| `confidence` | 0~1 |
| `state` | `DETECTED` / `UNCERTAIN` / `LOST` |

状态含义:

- **DETECTED**: 当前帧有可靠测量, 可作为控制输入。
- **UNCERTAIN**: 有候选但置信度不足, 或刚丢失后用预测位置顶替。
  `predicted=true` 的位置**不是测量值**, 不应作为有效坐标使用。
- **LOST**: 丢失, `x_px=None`。不输出旧坐标。

CSV 额外包含帧号、视频时间、FPS、算法耗时、轨道线参数, 便于测试分析。
`wait_ms` 是帧在采集队列中的等待时间, `pipe_ms` 是从采集线程拿到该帧
到产生检测/跟踪结果的耗时; `pipe_ms` 不含曝光时间和显示延迟。

## 5. 标定

前提: 钢珠静止在轨道上, 轨道上标出中心 0 cm、两侧 ±5 cm。

```bash
python3 calibrate.py add --cm -5
python3 calibrate.py add --cm 0
python3 calibrate.py add --cm +5
python3 calibrate.py fit
python3 calibrate.py show
```

- 每次 `add` 采集 6 s, 只统计 `DETECTED` 的 `x_px`, 取中位数。
- 有效样本 < 20 或 p5–p95 抖动 > 25 px 时拒绝写入, 需让钢珠静止后重试。
- 三点线性残差 ≤ 0.15 cm 用线性模型; 否则用分段线性模型。
- **标定与采集分辨率绑定**: 标定文件记录采集时的分辨率, 运行时分辨率
  不一致则拒绝输出 `x_cm`(启动信息显示 `not ready (calibrated at ...)`)。
  移动相机、变焦、重新对焦或改分辨率后, 旧标定全部失效, 必须重新标定。
- 标定文件就绪且分辨率一致时, `x_cm` 才会出现在输出中; 否则明确显示
  `-- (uncalibrated)`。

## 6. 参数

全部在 `config.json`, 按画面比例设置, 720p/1080p 通用:

- `camera`: 设备、分辨率、帧率、MJPG、缓冲区。
- `rail`: 搜索带(画面高度比例)、梯度阈值、RANSAC 容差、刷新间隔。
- `ball`: 颜色阈值(`B-R`、`B`)、包络阈值、凸起阈值、宽高/面积先验。
- `tracking`: 最大帧间跳变、UNCERTAIN/LOST 帧数、速度平滑系数。
- `calibration`: 标定文件路径。

换背景或光照后颜色阈值需要复核。当前机位(深色衣物背景、近景)几何
凸起是主判据, 颜色只作确认; `color_only_scale=0.5` 会压低仅颜色候选
的置信度。

## 7. 当前状态与限制(如实说明)

已完成:

- Jetson 摄像头/分辨率/帧率实测: 720p MJPG 实时约 29–30 fps
  (`CAP_PROP_BUFFERSIZE=8` + 采集线程); 1080p 实录约 19–20 fps。
  实时运行推荐 1280×720。
- 近景 1080p 静态视频(97 帧)离线结果: DETECTED 86 帧(88.7%),
  UNCERTAIN 7 帧, LOST 4 帧。DETECTED 的 x 均值 866.8 px、
  标准差 7.9 px、范围 853.0–884.3 px; 相邻 DETECTED 帧间跳变
  平均 3.65 px、最大 18.3 px; 置信度最低 0.76、平均 0.91。
  DETECTED 中没有出现 800–950 px 之外的假检。
- 当前近景 720p 稳定片段(298 帧, 钢珠 x≈536, 黑色衣物背景,
  手动 30 ms/gain 45): 默认融合 DETECTED 294 / UNCERTAIN 4 /
  LOST 0, x 标准差 0.8 px; 仅几何同样 294 帧, 仅颜色 265 帧。
  实时 8 s: 227/231 DETECTED, 28.7 fps, 算法约 19 ms。
  颜色单独候选已降权(`color_only_scale=0.5`), 防止轨道上缘蓝色
  反光被当成钢珠。
- 方法对比: 旧 1080p 浅色背景片段上, 仅颜色 86/7/4, 仅几何只有
  17 帧 DETECTED 且 16 帧锁在 x≈266–292 px 的背景凸起; 换到当前
  720p 深色衣物背景并拉近后, 仅几何 294/298。说明成像改善后几何
  凸起成为可靠主判据, 颜色不再是主力。
- 录制文件容器头帧率不可信(1080p 文件头写 60 fps, 实测 19.27 fps),
  `capture_raw.py` 已写同名 `.json` 侧车记录 `actual_fps`,
  `VideoFile` 离线读取时自动采用。
- 模块化程序、状态机、标定工具(含分辨率绑定)、CSV/视频记录已实现。
- Test E 初测(1280×720, 实时, 无显示/无录像, 30 s, 钢珠静止在
  x≈534 px): 886 帧, 29.5 fps; DETECTED 869(98.1%)、UNCERTAIN 17、
  LOST 0、预测 6 帧; DETECTED 的 x 均值 534.49 px、标准差 2.53 px;
  相邻 DETECTED 帧跳变均值 1.21 px、最大 9.38 px; 算法耗时均值
  11.43 ms、p95 18.63 ms; 采集到结果均值 11.68 ms、p95 18.88 ms。
  手动曝光 30 ms, 因此“曝光开始→结果”保守估计约 41.7 ms(均值)/ 
  48.9 ms(p95), 不含显示延迟。
- 带标注录像(异步 H.264, 15 s): 439 帧, 29.2 fps; DETECTED 435
  (99.1%)、UNCERTAIN 4、LOST 0; DETECTED 的 x 均值 532.68 px、
  标准差 1.72 px; 相邻帧跳变均值 0.93 px、最大 7.33 px; 写帧耗时
  均值 1.04 ms; 采集到结果均值 13.19 ms、p95 21.03 ms。
  此前 OpenCV MJPG 写 1280×720 单帧约 98 ms, 会把循环拖到 7.9 fps;
  现改为独立线程 ffmpeg/libx264 ultrafast 编码, 主循环不再被阻塞。
- 跟踪器只在候选置信度 ≥ 0.55 时建立新跟踪; 钢珠短暂丢失后, 低置信
  的背景蓝色物体会显示为 UNCERTAIN 但不会成为新目标。

尚未完成 / 不能声称的:

- **Test A-px 只完成了中间点**: 10 s 静止, 294/298 DETECTED
  (98.7%), x 均值 552.40 px, 标准差 0.80 px, 范围
  546.99–553.52 px, 相邻跳变均值 0.66 px/最大 5.88 px。左端点
  采集无效(球不在轨道上, 检测锁到背景白字 logo x≈460), 右端点
  尚未采集; 因此不能声称全轨道静态精度。
- **Test B 滚动尚未取得有效数据**: 第一次 25 s 因手臂遮挡,
  DETECTED 仅 40.4%, 7–13 s 大量 LOST; 第二次 25 s 因黑色磁铁板
  和手入镜, 钢珠约 2 s 后丢失, 跟踪器锁到左端轨道亮斑 x≈167,
  而实际钢珠在 x≈990, DETECTED 62.1% 不能作为滚动性能。两段
  标注视频和 CSV 已保存, 但都不计入 Test B。
- **Test C 的无球基线**: 10 s 无钢珠, 0 DETECTED, 71 UNCERTAIN
  (全部为低置信颜色候选, 最高 0.47), 227 LOST; 当前背景下没有
  几何误检。背景变化/人员走动尚未正式测试。
- **Test D 轨道倾斜尚未测试**。
- **Test E 已有 30 s 实时初测和 15 s 异步录像结果**; 还没有带
  显示时的延迟结果。当前数字来自近景机位, 不能外推到广角或
  不同光照。
- **跟踪器在钢珠丢失后可能被静止的轨道亮斑重新捕获**: rolling2
  中左端 x≈167 的亮斑 confidence 0.82(method=geom), 在钢珠被
  遮挡丢失后成为新跟踪; 实际钢珠在 x≈990 被忽略。该问题尚未
  修改算法, 需要先取得无遮挡的滚动数据再决定是否限制左端搜索
  范围。
- **尚未完成现场标定**, 因此现在运行时 `x_cm` 为空。
- 颜色线索依赖轨道上缘的蓝色反光和钢珠顶部的偏蓝成像, 只作确认;
  换光照后需要复核。
- 广角全景测试(1280×720, 轨道仅占约 30 px、蓝色箱包入画)出现
  假 DETECTED: x≈649 px、置信度 0.95, 实为蓝色箱包上沿, 持续
  11 帧。该视角不满足识别条件, 不能用于标定或正式测试。
- Jetson 无 RTC 电池且当前未联网校时, 系统时钟不准(面板显示
  2026-03-24, 实际日期 2026-10-23); CSV 的 `timestamp` 跟随系统
  时钟, 帧间对齐请用 `frame` / `video_time_s`。这不影响算法和
  相对时序, 但会影响日志的绝对时间。
- 钢珠只露出上半部分, 直径约 1 cm; 当前黑色衣物背景使几何凸起
  可靠, 但背景是站立的人, 人员移动会改变画面。标定/测试期间
  相机、轨道、人员必须保持不动; 有固定哑光背景板会更稳。
- 当前机位已经可以继续标定和测试, 不再要求改硬件; 但人员不能
  随意走动。若无法保持静止, 需要固定深色哑光背景板, 这不是靠
  继续增加算法复杂度能解决的。

## 8. 测试计划

前置条件: 近景机位、深色背景(当前为黑色衣物, 测试时人员保持不动)、
钢珠清晰对焦, 测试期间相机/轨道/光照保持不变; 标定与测试必须使用
同一分辨率(统一 1280×720)。每项测试同时保存原始视频、标注视频、
CSV 和 setup 截图。

1. Test A: 钢珠静止于 -5 / 0 / +5 cm, 各 ≥10 s, 统计均值/最大误差、抖动。
   没有尺子时先做 **Test A-px**: 钢珠放在轨道可见段的左/中/右三个
   位置, 各 10 s, 只统计像素抖动和检出率; 厘米误差等标定完成后再补。
2. Test B: 钢珠滚动, 统计检出率、跟踪连续性、最大跳变。
3. Test C: 更换背景/人员走动, 统计误检率。
4. Test D: 轨道倾斜(垫高一端), 检查轨道线跟随与检测保持。
5. Test E: 实时 FPS、单帧算法耗时、端到端延迟。

每项测试保存原始视频、标注视频和 CSV, 结果写入测试报告。

当前没有尺子, 先运行 `./run_pixel_tests.sh` 采集 Test A-px、无钢珠
背景和 Test B 的原始视频; 该脚本不产生厘米坐标, 也不写入标定文件。
