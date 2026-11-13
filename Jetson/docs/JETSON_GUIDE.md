## 2026-11-13 离板修复（尚未部署）

实时 CSV 的 `video_time_s` 改为首帧起的采集返回时间差，保留丢帧间隔；视频回放使用媒体帧率时间，支持低于 1 FPS。采集时间仍不是曝光硬件时间戳。

新增 `cm_valid`、`cm_status`、`reference_x_px`、`reference_ms`。原 `valid` 仍指像素测量有效；厘米有效不等于 UART 可用。预览继续禁止串口。`vision` 表示无有效球测量，`rail` 表示轨道无效，`reference_*` 表示参考匹配状态，`measurement_unmatched` 表示无法关联当前候选，`below_calibrated_range`/`above_calibrated_range` 表示超出标定范围，`ok_preview` 表示有效实验性厘米坐标。无有效球或轨道时跳过匹配并清除旧变换。

端点外不外推、不钳位；因此 ±5 cm 附近抖动仍可能使厘米坐标无效。要改善端点连续性，需要后续采集工作范围外的已知位置扩展标定，不能把未知位置当作标准。

离线验证：104 项测试通过；7 段原始录像共 1490 帧，修改前后像素、厘米和跟踪输出逐帧一致。滚动录像像素检测 564/596。详情见 `test_results/offboard_20261113/comparison.json`。本机 OpenCV 结果不冒充 Nano 实测。未修改检测器、参考配准算法或标定文件，未连接/部署 Nano；板端帧率仍需供电正常后的实测。

## 2026-11-13：纸尺参考厘米预览（实验模式）

当前Nano工程为 `/home/jetson/ball_vision_highlight_20261113`。可在原主程序中显式启用：

```bash
OPENBLAS_CORETYPE=ARMV8 python3 main.py --config config_fusion_v2.json --reference-preview reference_preview_20261113.json --csv --record --duration 10
```

此模式复用原检测、跟踪和录像，画面厘米值标注 `PREVIEW`，CSV的 `feedback_valid` 恒为0、原因 `reference_preview`，且命令行拒绝与 `--serial` 同时使用。省略该选项时仍使用原标定和姿态保护，不会自动启用新映射。

参考图为 `ruler_reference_20261113.png`（静止12cm录像第4帧），三块纸尺纹理用于逐帧估计图像平移、旋转和等比例缩放。每帧三处必须匹配且相互一致；歧义、遮挡、分辨率变化和超出匹配范围时清除变换。搜索范围为参考位置左右32px、上下24px；不能用它宣称任意倾角/透视变化均受支持。

清单绑定参考图、检测器、参考算法及轨道/钢珠配置的SHA256；修改相关文件/参数后需要重新验证和生成清单，不能只改哈希绕过。模型为三点分段线性，不允许超出训练像素范围外推，因此端点附近噪声或越界可能使厘米值暂时无效。该限制尚未满足闭环越界检测需求。

本轮验证条件：1280×720、固定深色背景、手动曝光请求10ms（exposure_absolute=100）及gain=48。主程序不会自动设置曝光。实测工具运行后恢复auto=3、exposure_absolute=312、gain=0；重新测试前需恢复验证曝光条件。相机或纸尺位置变化后，应重新检查参考匹配和独立刻度误差。

已验证范围是当前装置的两侧静态独立点，以及固定零点的小幅摆动录像；不是±2°/±3°完整机械倾角验证，也未开启UART或舵机闭环。离线评估入口 `evaluate_ruler_reference.py` 可复核补偿像素、匹配耗时和失效帧，报告为实验数据。

> 最新修复入口（2026-11-13）：`/home/jetson/ball_vision_highlight_20261113`，使用 `config_fusion_v2.json`。本机及Nano73项测试通过。固定10ms/gain48的10秒实时检测+录像267帧、262有效、5待确认、无LOST，平均26.7fps；不是全轨道/倾斜精度通过。完整证据 `test_results/highlight_20261113/assessment.json`。
>
> 实时 `--record` 默认640×360、最高15fps标注预览，检测仍1280×720，每个处理帧都写CSV。`--record-fps 30 --record-scale 1` 可恢复高负荷全尺寸录像，可能降低测量帧率。离线回放默认仍保留源尺寸/帧率。`--display` 开启时仍需绘制全尺寸显示，不能套用无display性能结果。
>
> `.mp4.timestamps.jsonl` 逐帧列出提交时间、是否接受、录像索引和丢弃原因；最后summary.complete必须为true才可相信完整索引。固定FPS视频播放时长不能代替实际时间。编码队列满会丢录像帧，不能让录像阻塞位置测量。
>
> 高光恢复输出 `geom_recovered` / `hybrid_recovered`，只恢复像素跟踪；main和calibrate拒绝将该新轮廓坐标直接套用旧厘米标定。原geom/hybrid坐标与四点精度保持。后续需验证恢复坐标及动态参考点。
>
> 复测时明确设置相机参数：`v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1`，再设置 `--set-ctrl=exposure_absolute=100,gain=48`，运行 `OPENBLAS_CORETYPE=ARMV8 python3 main.py --config config_fusion_v2.json --csv --record --duration 10`。本轮测试后已恢复auto3、曝光寄存器312、gain0；自动模式下不能沿用手动曝光帧率结论。

> 最新板端状态：候选已部署 `/home/jetson/ball_vision_dynamic_20261113`，47项测试和四点历史静态回归通过，但现场手动10ms/gain48未检出球、带标注录像明显降速。见 `test_results/dynamic_20261113/nano_test_assessment.json`。下方“尚未部署”的记录为历史状态。

> 2026-11-13 软件候选：本机 ball_detector.py / ball_tracker.py / main.py / visualization.py 已更新，Nano 暂时SSH超时，尚未验证部署。旧Nano入口仍为 ball_vision_fusion_20261112_2102。测试与回放结论见 test_results/dynamic_20261113/assessment.json；不要将本机通过当作板端验证。
>
> 跟踪 update(candidates, ts) 的 ts 为递增秒，实时传 monotonic/read 时间，离线传视频时间；速度为像素/秒。新可选 tracking 参数 max_gap_s=0.25、max_speed_ratio_s=1.2、position_noise_ratio=0.015、ambiguity_ratio=0.005、acquire_confidence_margin=0.05。它们是图像关联阈值，不是实测机械限值。颜色和几何坐标融合版本及标定文件未改变。倾斜厘米标定和可信采集时间仍需完善。

> 最新：颜色融合跳变已修复，当前使用 `config_fusion_v2.json` / `calibration_fusion_v2.json`，Nano目录为 `/home/jetson/ball_vision_fusion_20261112_2102`。详见 [修复报告](FUSION_FIX_REPORT.md)。旧算法标定不能直接使用。

# Jetson 标定与反馈使用说明（2026-11-12）

当前版本是测量候选版本，不是已验证的闭环系统。G3519 固件不在本次修改范围。
本地：`/home/user/ball_vision_src`；Nano 候选：`/home/jetson/ball_vision_candidate_20261112_2015`。
Nano 原目录 `/home/jetson/ball_vision` 保留不变。完整备份见 `backup.json`。

## 已完成与限制

- 三点标定必须包含 -5/0/+5 cm；检查有限值、单调性、分辨率，支持左右方向相反。
- 优先线性拟合；线性拟合残差超过 0.15 cm 才选分段线性。拟合残差不是独立精度验证。
- 超出已标定像素区间时返回空值，绝不夹成 ±5 cm。因此只有 -5/+5 两个端点时，越过目标将失效；正式控制前必须扩展标定到整个预期运动范围并验证，不能靠外推补齐。
- 轨道重新拟合失败立即失效并重新搜索。默认每 3 帧更新，启用串口或厘米标定就绪时每帧更新。
- 低置信候选不能更新已建立跟踪；连续丢失后清除旧跟踪。
- **几何亮斑误锁尚未解决**，未宣称全轨道可靠。需要无遮挡滚动和无球/背景变化试验。
- 厘米坐标要求当前未平滑轨道拟合与每个标定点的轨道两端高度差均 ≤3 px。该阈值是保守软件门槛，不是已测量的厘米误差上界。
- **这不是动态厘米坐标系**：直线无法给出沿轨道零点，无法检测相机沿轨道移动、焦距变化。当前倾斜可能使厘米输出失效，这是预期保护。后续需实拍参考点/纸尺图像建立动态零点、尺度并验证透视误差。
- 不生成虚构的 `calibration.json`，不写入占位增益，不输出 PWM。

## 采集与标定

固定相机、对焦、光照和背景，摆杆近似水平。使用可拆卸定位工具让真实位置已知，工具不得遮挡或产生假凸起。
先用现有 `capture_raw.py --help` 查看录像命令。不要用带标注视频标定。
检查原视频，找到钢珠仍在已知位置的区间；移除工具后发生滚动的画面不可选。
所有帧号从 **0** 开始，起止帧均包含。程序从开头运行跟踪预热，只采纳所选区间内 DETECTED 的帧。
`--known-position` 是操作者对真实位置的确认，不是算法自动认证。稳定亮斑也可能通过检测，必须核对画面。

```bash
cd /home/jetson/ball_vision_candidate_20261112_2015
# 以下文件名与帧区间是示例，必须替换成现场人工确认的实际画面。
OPENBLAS_CORETYPE=ARMV8 python3 calibrate.py add --cm -5 --video recordings/minus5.avi --start-frame 30 --end-frame 40 --known-position
OPENBLAS_CORETYPE=ARMV8 python3 calibrate.py add --cm 0 --video recordings/zero.avi --start-frame 30 --end-frame 40 --known-position
OPENBLAS_CORETYPE=ARMV8 python3 calibrate.py add --cm 5 --video recordings/plus5.avi --start-frame 30 --end-frame 40 --known-position
python3 calibrate.py fit
python3 calibrate.py show
```

允许只选择一个仍在已知位置的可靠帧，但不能据此声称精度或抖动已验证。默认拒绝所选样本像素峰峰值 >5 px 或轨道端点变化 >3 px。
保存原视频路径/SHA-256、所选范围、有效帧号、像素和轨道参数，便于复核。
新增或替换点后模型立即失效，需要显式重新 `fit`。相机变化后使用 `--calibration 新文件.json` 重新标定，不复用旧结果。

## 独立精度验证

用未参加拟合的 -4/-2/+2/+4 cm 等刻度。每个刻度单独选取真实位置已知的区间：

```bash
OPENBLAS_CORETYPE=ARMV8 python3 calibrate.py verify --cm 2 --video recordings/check_plus2.avi --start-frame 30 --end-frame 59 --known-position --report check_plus2.json
```

报告包含有符号平均误差、平均绝对误差、最大绝对误差、标准差、峰峰值和有效比例。
单段 `passed` 要求至少 20 个有效样本、有效比例 ≥95%、最大绝对误差 ≤0.3 cm。报告以新建方式写入，拒绝覆盖。
这仅代表该刻度、该姿态、该区间，不代表全轨道或倾斜状态通过。

## 实时测量与录像

```bash
OPENBLAS_CORETYPE=ARMV8 python3 main.py --csv --record --duration 30 --label unobstructed
```

CSV 保留 `valid` 为原有像素检测有效标记，新增 `rail_valid`、`pose_ok`、`capture_monotonic_ms`、`feedback_valid`、`feedback_reason`。
没有可靠姿态或没有标定时 `x_cm` 为空，即使像素 `valid=1`。
`feedback_valid` 表示本帧允许用于串口反馈，不等于实测控制成功。默认值为 0；具体原因保留为 pose/vision/calibration/source_time 等，测量条件均满足但未启用时才是 monitor。
离线录像的 `capture_monotonic_ms` 是解码时刻；运动分析必须用 `video_time_s`，不能把解码速度当实际帧率。

倾斜 0/±2/±3°、全轨道无遮挡滚动、无球/背景变化应分别保存原视频和 CSV。
不把 DETECTED 比例当正确率；必须逐段核对真实钢珠。旧 rolling2 是已知遮挡/误锁负例，不是合格滚动测试。

## UART 联调

详见 `UART_PROTOCOL.md`。仅发送测量，不含启停舵机、目标位置或 PD 指令。
先由 G3519 负责人实现协议、CRC、去重、接收超时和安全失效策略，再联调。

```bash
# 监测模式：所有数据 valid=0，位置字段为空。
OPENBLAS_CORETYPE=ARMV8 python3 main.py --serial /dev/ttyTHS1 --csv
# 未来补齐可信采集时间、现场标定、动态姿态与误锁验证、接收端安全策略后，此参数才具备启用有效反馈的前提：
# OPENBLAS_CORETYPE=ARMV8 python3 main.py --serial /dev/ttyTHS1 --uart-enable-valid --csv
```

**当前 OpenCV 摄像头后端无法提供可信的传感器采集时间，实时 UART 即使带 `--uart-enable-valid` 也保持 VALID=0（其余检查通过时原因 source_time）。** 不能将读帧完成时间误当曝光时间。后续需要接入真实 V4L2 时间戳或验证有界采集链路，再解除此保护；本次仅完成监测传输及编码接口，未交付可启用的闭环位置反馈。

`/dev/ttyTHS1` 只是命令示例，接线和端口尚未确认。串口无自动重连，故障退出，避免重放旧队列。
离线 `--video` 与 `--serial` 互斥。Ctrl+C/SIGTERM/SIGHUP 尽力发送无效帧后关闭；断电/强杀无法保证发送，接收端必须自行超时。

## 软件测试

```bash
OPENBLAS_CORETYPE=ARMV8 PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -v
```

软件测试使用临时文件、合成图像、伪终端，无需接摄像头或控制器。通过不能替代实际厘米精度、无遮挡滚动或 5 s 闭环实验。
