# 数字备用参考厘米标定，2026-11-24

绑定 ID 为 `4cc0299fd6a5f885b4e94690f0db6640a93a76f592b658f6f81109d11ea17fd9`。按新配置真实加载 `rail_cm_static_reference_c.json`，原 RailCalibration 重算结果为 `validated`。实际参考模块 SHA-256 为 `219e271d214054f1a06a60866bacdaebe29aebddb48e471817414ba4b8f27d74`。旧 C 模型与新绑定交叉加载均拒绝；修改任一数字框坐标后，新模型同样拒绝加载。旧坐标类及原八个绑定源与提交 7a741ef 逐字节一致。[模型与输入摘要](verification.json)

## 原始 MJPEG 多点验收

采用两轮历史已确认的实际摆放和原始整段帧区间。每次通过 `make_rail_detector` 创建参考检测器，经原球检测、C 补测、跟踪及新坐标类生成轴坐标。原 `collect` 收集样本，原 `build_model` 以 −6、0、+6 cm 三个位置拟合线性模型。每段 150 帧，未删减获取或缺测帧。旧 −6 拟合片缺采集时钟，沿用 30 fps 名义时钟。其余 16 段使用对应的原始帧时钟。

9 个独立验收片在 −5、−3、+3、+5 cm 共生成 1312/1350 个有效厘米值，加权 MAE 0.05200 cm，最大绝对误差 0.28245 cm。14 个非拟合摆放逐段均通过 ≥95% 覆盖、最大绝对误差 ≤0.3 cm 的门限。正式模型内嵌 3 个拟合会话及 9 个独立验收会话；已逐个重算视频、CSV 和绑定 sidecar 摘要。[每段原片与误差](placements.csv)

17 段共 2550 帧，其中原主参考 1947，原备用参考 600，参考无效 3，数字备用参考 0。此项验证了原有摆放在新绑定下的静态厘米表现。它没有提供数字备用分支独自工作的已知刻度误差上界。

## 正式文件入口交叉检查

同一模型经实际 `main --video` 和 VideoFile 文件解码、名义 30 fps 运行。9 个预定验收片均通过，合计 1313/1350，最大误差 0.21459 cm。全部 14 个非拟合片中 13 个通过。开发回归片 `new_verify_m5` 的完整结果为 132/150，覆盖 88%，第 35 帧输出 −4.54591 cm，绝对误差 0.45409 cm。该段数字备用参考命中 0 帧。此失败保留在文件入口证据中。上述 MJPEG 标定入口使用 imdecode，文件入口使用 VideoCapture。[全部文件入口摆放](file_placements.csv) · [逐帧结果](file_frames.csv)

另取刷机后独立补光样片检查数字参考到厘米的完整通路。实际解码 60 帧，59 帧通过数字参考，55 帧发布数字参考下的有效厘米，范围 −0.08184 至 −0.04238 cm。此片缺独立确认的逐帧实物位置，因此未纳入多点精度验收。

## 复现

从仓库根目录运行以下标定命令。两个输出均需为新路径。模型包含采样证据路径，更换重放目录会改变模型文件摘要。省略配置参数可重走旧 C 的静态和动态重放；本轮核对旧归档的全部 24 个证据文件均逐字节一致。

```bash
python3 -B Jetson/docs/static_control_c/replay.py --config Jetson/config_static_control_ruler_c.json /home/user/桌面/jetson_nano /tmp/static-reference-evidence /tmp/rail_cm_static_reference_c.json
```

[file_check.py](file_check.py) 使用本目录的 placements.csv 对主入口做独立复核，单独记录数字参考来源。其输出文件也采用排他新建。
