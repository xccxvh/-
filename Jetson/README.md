# Jetson Nano 视觉测量代码

本目录是电赛 H 题中 Jetson 端的视觉采集、轨道/钢珠检测、跟踪、标定及日志录像代码，不包含 G3519 固件。此次导入保留现有源码与算法哈希，不修改 `Car2/`、`BSP/` 或循迹工程。

## 当前状态（2026-11-13）

- 本机及 Nano 曾通过104项软件测试。供电恢复后1280×720实时处理实测约29.4 FPS；这不代表检测准确率合格。
- 相机被碰动后正在重做标定。新机位有浅球顶漏检及纸尺假凸起误锁；最新固定背景右侧149帧参考匹配成功，但钢珠0帧有效。**当前不能用于自动稳球闭环。**
- 新机位使用 `config_newpose_20261113.json`，旧厘米标定已在该配置中关闭。已采7/12/17 cm录像，尚未生成通过验证的新厘米模型。
- `reference_preview_20261113.json` 与 `ruler_reference_20261113.png` 属于旧机位实验参考；清单绑定检测器/配置/参考图哈希。不能用于当前机位，不能修改哈希来绕过保护。
- 曲率阈值试调仅是离线诊断，未修改或提交正式检测器。待解决漏检、误锁后再做独立刻度与动态验证。
- Jetson现有串口为 ASCII **BV1**，仓库 `Car2/` 说明中的接口为二进制帧，两者尚未完成协议对齐。当前相机时间戳未验证，UART有效反馈保持关闭；参考预览禁止开启串口。不要直接接入闭环。

## 环境与运行

板端已用环境：Jetson Nano / JetPack 4 系列，Python 3.6.9、OpenCV 4.1.1、NumPy 1.19.5；另需系统 `ffmpeg`、`ffprobe`、`v4l2-ctl`。优先复用板载环境，不为运行本代码升级JetPack或替换OpenCV。

在本目录执行（Nano需设置 `OPENBLAS_CORETYPE=ARMV8`）：

```bash
export OPENBLAS_CORETYPE=ARMV8
# 新机位仅像素测量；无已验证厘米坐标、无UART
python3 main.py --config config_newpose_20261113.json --csv --record --duration 15
# 回放原始录像（录像不随仓库提交）
python3 main.py --config config_newpose_20261113.json --video /path/to/raw.avi --csv
# 软件回归（包含必要测试图片）
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s tests -q
```

新机位最近采集条件为手动10ms（`exposure_auto=1`, `exposure_absolute=100`）、增益63。此参数尚未解决全范围检测，不能视为最终推荐值。`set_camera.sh` 是旧机位脚本，不要把其中参数与本轮结果混用。当前默认 `config.json` 和历史配置也不是新机位入口。

## 文件职责

| 文件 | 职责与限制 |
|---|---|
| `camera.py`, `capture_raw.py` | USB采集、视频回放、原始MJPG录像；读帧时间不等于传感器曝光时间 |
| `rail_detector.py`, `ball_detector.py`, `ball_tracker.py` | 轨道拟合、钢珠候选、按时间连续跟踪；检测valid不等于正确率 |
| `calibration.py`, `calibrate.py` | 已知位置采集、映射拟合；禁止未知位置或滚动后的画面充当标准 |
| `ruler_reference.py`, `reference_preview.py`, `evaluate_ruler_reference.py` | 纸尺参考补偿、严格失效清除、实验预览及离线验证 |
| `main.py`, `visualization.py` | 主循环、CSV与显示；cm_valid和像素valid分别记录 |
| `video_writer.py`, `recording_schedule.py` | 有界异步录像；实际时间以时间戳侧车文件为准 |
| `position_feedback.py` | BV1编码与显式开启的串口监测，不负责舵机控制 |
| `tests/` | 单元/集成测试及真实图像回归样本 |

使用细节见 [视觉指南](docs/JETSON_GUIDE.md)，协议见 [BV1协议](docs/UART_PROTOCOL.md)。历史说明见 [原README](docs/HISTORICAL_README.md)，其旧命令、参数和进度仅供追溯，以本页当前状态为准。

原始现场录像、CSV、工作站上下文、缓存和临时试调未上传。`source_manifest.json` 提供本次导入的源文件校验值；它证明代码来源，不证明算法已通过当前现场验收。
