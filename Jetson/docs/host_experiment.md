# Nano 自动化静态滚球实验

本次 AUTO 板测使用 G3519 固件 03991c7 和方案 C。Nano 收到 SET_MODE、SET_TARGET、SET_PD、START_AUTO、STOP 的成功回执。接入舵机电池后，板端 AUTO 遥测覆盖 12.02 秒，钢珠从纸尺 8.73 cm 到 8.70 cm，未达到 12 cm 目标。121 帧有效 AUTO 遥测中的指令均为 +1°，已达到固件限幅。真实杆角仍待独立测量。[原始试验日志](host_zero_20261126/)

160534 是现场反馈舵机未全程供电的首轮；160832 是接电后的复测。文件名沿用 Nano 本地时钟，间隔按单调时间计算。

## AUTO 板测入口

在 Jetson 目录运行。命令中的 KP、KD、T 和端点由现场实验确定。12 cm 纸尺绝对值对应参数 0，视觉采集和反馈使用 config_static_fast_c.json。

```sh
python3 -B nano_experiment.py --serial "$PORT" auto --target-cm 0 --kp "$KP" --kd "$KD" --seconds "$T"
```

控制适配器按 vision_link.h 编解码 10 字节控制负载与 6 字节执行回执。视觉与实验线程共用同一串口。

AUTO 流程逐条等待同序号、同命令回执。以有效 0x81 遥测核对目标、钢珠位置和 AUTO 状态。启动前视觉等待 1 秒，启动回执后状态等待 0.5 秒，运行时遥测空档超过 0.3 秒退出。当次实验结束或失败均尝试 STOP，同时检查随后收到的板端状态。回执或停机状态缺失时日志会记录异常。两侧回中需要从实际观测确认的正、负两侧分别起始，检查每段轨迹中到达和保持 O 点的时间。

## REMOTE 扫描状态

03991c7 的软件中位为 1324 ticks，REMOTE 允许 1280～1368 ticks。扫描和自适应入口均使用此范围并接受更窄的现场边界。[自适应板测与失败记录](adaptive_return_20261126/README.md)

扫描使用现有 0x02 命令，仅接受现场确认的 tick 区间；先发中位获取回显，再发 START_REMOTE。按约 20 Hz 发送目标，核对每个平台的序号、限速后 PWM 和状态。STOP 后确认控制解除和软件 PWM 回中。回执日志、钢珠轨迹与完整 0x81 记录在同次运行的 experiment、track 和 link CSV 中。

将扫描与视觉日志用 join_servo_scan.py 按本地单调时钟对齐，再用下列命令估算每个平台的滚球加速度。噪声门限应由同机位的静球记录确定。每段保留拟合残差和有效覆盖时长。正、反扫描分别看加速度换向区间，靠近零位用较细步长重复。用独立试次复核最小能产生可重复滚球响应的步长。钢珠碰挡具、滑移或起始速度不一致时需要复核原片。精确机械杆角、角死区和机械回差需要独立量角数据。

```sh
python3 -B join_servo_scan.py "$SCAN_CSV" "$TRACK_CSV" joined.csv
python3 -B analyze_servo_response.py joined.csv --accel-noise "$MEASURED_NOISE" > plateaus.csv
```

离板复核可运行 `python3 -B -m unittest discover -s tests -p 'test_*experiment*.py' -v`。伪终端测试使用模拟应答，AUTO 实验有上述实板日志，独立机械标定和两侧回中仍待验证。

## 自适应 REMOTE 控制更新（离板）

扫描和自适应控制都接受 `--board-profile`。默认清单 `g3519_remote_03991c7.json` 对应 03991c7 的构建参数：104° 舵机中位、REMOTE ±5°、20°/s 限速。该固件的 0x82 只提供已接受的目标、实际输出、状态与序号，未提供行程端点。更换固件后使用该构建对应的清单，单位为 ticks 和 ticks/s，命令行给出现场确认的更窄区间。进入 REMOTE 后检查解除使能状态的中位回显；遇到板端 CLAMP 或非预期回显会 STOP。

试探递增和递减 PWM 应分别运行。日志标记 `probe_motion` 时记录该次试探推断出的方向，需结合起始速度、相对刻度位移及反向试探复核。证据不足时不会给出方向确认。确定极性后在回中命令显式传入 +1 或 -1。

```sh
python3 -B nano_experiment.py --serial "$PORT" adaptive --board-profile "$PROFILE" --min-ticks "$LOW" --max-ticks "$HIGH" --probe-sign 1
python3 -B nano_experiment.py --serial "$PORT" adaptive --board-profile "$PROFILE" --min-ticks "$LOW" --max-ticks "$HIGH" --probe-sign -1
python3 -B nano_experiment.py --serial "$PORT" adaptive --board-profile "$PROFILE" --min-ticks "$LOW" --max-ticks "$HIGH" --polarity "$POLARITY" --seconds 15
```

启动时在短窗内拟合真实测量速度，支持安全区内正在滚动的钢珠。停滞加力按目标距离及可用行程缩放，观察时间包含已测 PWM 回应滞后与剩余行程时间。高速靠近目标时可立即卸力或反向制动。短暂缺测撤回驱动，在恢复可靠速度前保持中位；220 ms 无可靠运动测量则停止。100 ms 曝光年龄要求、通信丢失退出和轨道端部保护继续生效。软件测试覆盖新版宽窗口、移动启动、制动和失测处理。新控制律尚无实板回中或保持数据。
