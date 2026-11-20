# Jetson 与 G3519 的 USB 监测记录

2026-11-20 实测。Jetson Nano 识别到 USB 设备 1a86:7022（CH32 CMSIS-DAP），cdc_acm 驱动分配 ttyACM0。jetson 用户取得该设备的读写 ACL 后，以 115200 8N1 打开成功。

| 实验 | 实际结果 |
|---|---|
| Nano 只监听 6 秒 | TX 0，收到 6323 字节，0 帧 0x81，0 个已识别遥测帧的 CRC 错误。passive_01.csv 仅有表头。 |
| 独立原始采样 3 秒 | passive_usb_sample.bin 共 3151 字节。可读灰度输出 gray:0,0,0,0,0,0,0,0，未出现 A5 5A 同步字。 |
| 完整软件测试 | 电脑 245 项通过，Nano 245 项通过，包括在 Linux 伪串口双向传输协议帧并记录 CSV。 |

实机双向协议及手动滚球仍待验证。Car2/User/main.c 在编译期选取应用，默认 CAR_APP_LINE_FOLLOW。Car2/BSP/VISION/vision_link.c 在 UART4 (PB10/PB11) 运行滚球协议。需由 G3519 侧确认板上固件确实运行 ball_test，并确认该板的 Type-C 虚拟串口桥接到实际运行 vision_link 的 UART。灰度文本只证明 USB 上行可读。

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
