# Jetson 与 G3519 的 USB 监测记录

2026-11-20 实测。Jetson Nano 识别到 USB 设备 1a86:7022（CH32 CMSIS-DAP），cdc_acm 驱动分配 ttyACM0。jetson 用户取得该设备的读写 ACL 后，以 115200 8N1 打开成功。

| 实验 | 实际结果 |
|---|---|
| Nano 只监听 6 秒 | TX 0，收到 6323 字节，0 帧 0x81，0 个已识别遥测帧的 CRC 错误。passive_01.csv 仅有表头。 |
| 独立原始采样 3 秒 | passive_usb_sample.bin 共 3151 字节。可读灰度输出 gray:0,0,0,0,0,0,0,0，未出现 A5 5A 同步字。 |
| 完整软件测试 | 电脑 245 项通过，Nano 245 项通过，包括在 Linux 伪串口双向传输协议帧并记录 CSV。 |

首次只听时实机双向协议及手动滚球仍待验证。当时核对的固件提交 41d78b0e 中，Car2/User/main.c 在编译期选取应用，默认 CAR_APP_LINE_FOLLOW。Car2/BSP/VISION/vision_link.c 在 UART4 (PB10/PB11) 运行滚球协议。需由 G3519 侧确认板上固件确实运行 ball_test，并确认该板的 Type-C 虚拟串口桥接到实际运行 vision_link 的 UART。灰度文本只证明 USB 上行可读。

## 复测命令

电脑使用 `ssh jetson@192.168.55.1` 登录 Nano，在 Nano 终端执行下列命令。每次使用新的 CSV 文件名。

```bash
cd ~/topview_usb_monitor_20261120
PORT=/dev/serial/by-id/usb-wch.cn_CH32_CMSIS-DAP_B7AD8F061FFECDEFG-if01
lsusb -d 1a86:7022
readlink -f "$PORT"
python3 -B position_feedback.py "$PORT" rx_probe_02.csv --seconds 6
```

只监听时可以保持摄像头预览。若重新插拔后权限失效，可在 Nano 执行 `sudo setfacl -m u:jetson:rw "$(readlink -f "$PORT")"`。收到 CRC 正确的 0x81 且确认板处于手动或禁用后，独占相机做静态双向测试。当前部署的 `bash run_live.sh usb_static 8` 可临时释放原始预览，其视觉命令为

```bash
OPENBLAS_CORETYPE=ARMV8 python3 -B main.py --config config_topview_circle_20261120.json --serial "$PORT" --csv --label usb_static --duration 8
```

在板端核对接收序号与日志最后的 position_seq，再在 O 点附近做手动测试。遥测自己的 telemetry_seq 独立计数，不能证明位置帧到达。正常的滚球应用约每 100 ms 回传一帧。

## CSV 与有效性

logs 下的 link CSV 每行对应一次实际发送或接收。local_monotonic_ms 以 Nano 单调时钟记录写完或读到的时刻。TX 中 capture_ms32 取本帧图像读取完成后的时间，按 32 位回卷。RX 中 g3519_ms 来自板端时钟。分析动作至运动的表观延迟，需在 Nano 本地时间轴上对齐事件，并计入遥测周期和未验证的曝光时刻。

当前帧检测、轨道、厘米标定及本地新鲜度均通过时，0x01 可携带真实 rail_x_cm 和置信度进行监测；标定有效由 bit1 表示。控制有效 bit0 始终为 0，timestamp_verified 保留实际值。失效帧线上占位为零且置信度为零，CSV 厘米栏留空。beam_estimated_deg 为软件估计角。当前实测尚无舵机遥测、运动方向或响应时间数据。

## 切换滚球固件后的复测

同一 Type-C 线重新枚举后，在 Nano 恢复设备 ACL。只听 6 秒收到 60 帧 CRC 正确的 0x81，共 2461 字节。随后使用实时相机运行 8 秒，记录见 ball_mode 子目录。文件名沿用 Nano 本地日期；时序分析使用单调时钟。usb_static 是测试标签，遥测表明摆杆指令在此期间发生了变化。

| 项目 | 结果 |
|---|---|
| 位置 TX | 213 个视觉帧加 1 个退出失效帧，共 214 个。控制有效位全为 0，最后位置帧序号 213。 |
| 遥测 RX | 80 个，序号 2974 至 3053 连续，CRC 错误 0。手动状态 1，自动使能位全为 0。 |
| 相机与厘米测量 | 1280×720，26.6 FPS。72/213 帧（33.8%）输出当前厘米位置，范围 −3.418 至 +2.463 cm。剩余 141 帧厘米栏为空。 |
| 连续性 | 最长 69 帧连续无厘米值。视觉帧 122 和 192 两次有效读帧时间相隔 2.548 s。相邻且连续有效的帧，最大绝对位置变化 0.429 cm。现场未提供逐帧位置真值。 |
| 舵机与延迟 | 回传指令在 −10°、0°、+10° 之间切换，PWM 1611～1833 µs，软件估计角 −10° 至 +10°。多次指令变化与长缺测区重叠，无法从该段可靠确定单次动作的运动方向和响应时间。 |

原始与矫正预览均已恢复，两个 HTTP 端口均实读到 JPEG。link CSV 按同一 Nano 单调时钟记录摄像头位置与舵机遥测。板端遥测携带的 board_ball_cm 在此次手动测试中为 0。板端接收位置帧的独立确认依据是 LCD 的接收序号。

## 按远端 main 的 b81519b 核对现场固件

该提交的 vision_link.h 将视觉协议迁移到板载 DAPLink 对应的 UART0（PA10 TX、PA11 RX），速率仍为 115200 8N1。此前关于 UART4 的记录对应旧提交。照片中的 ROLL、RUN、POS、3/3、+10.00 和 500/200 ms 与新固件 LCD 显示一致。ROLL 使用手动舵机通道按时间交替下发倾角，因此遥测 control_state=1 可与屏幕 RUN 同时出现。

ball_test.c 在所有模式下先调用 Vision_Update。协议解析器在收到通过格式和 CRC 检查的新位置帧时更新 last_seq，控制有效位清零不妨碍记录序号。ROLL 页将序号行替换为 Stage/Run/Ab。由 ROLL 用 K1+K2 切换到 MANUAL 会停止循环、请求回平衡位，并在 LCD 显示 link.last_seq。切换模式保留链路序号。未重启且没有后续位置帧时，上一段现场日志对应的板端预期值为 213。0x81 的遥测序号属于另一个计数器。

现场操作员随后读到 G3519 LCD 的实际 Seq 为 213，与 link_usb_static_20260324_183210.csv 末行 position_seq 完全一致，确认板端已解码该 0x01 退出失效帧。结合 Nano 实收 80 个连续、CRC 正确的 0x81 遥测帧，本次 Type-C/DAPLink UART0 监测模式双向收发验证通过。该次记录未逐帧确认每个位置值，也未启用自动控制。
