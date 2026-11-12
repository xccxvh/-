#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
G3519 滚球控制串口联调工具。

功能：
1. send    —— 以 30 Hz 向 G3519 发送视觉位置帧（固定值、测试 B 场景循环或正弦）；
2. decode  —— 接收 G3519 的遥测帧并打印/保存为 CSV，便于分析响应时间、超调、稳态误差；
3. sim     —— 半实物闭环：接收遥测里的实际摆杆倾角，用滚球动力学积分出钢珠位置，
              再发回 G3519，可在没有真实钢珠/摄像头时验证整条控制链路。

协议与 Car2/BSP/VISION/vision_link.h 完全一致：
帧 = A5 5A | ver(1) type(1) len(1) seq(2) | payload(len) | crc16(2, 小端)
CRC16-CCITT-FALSE，覆盖 ver 到 payload 末尾。

依赖：pip install pyserial
运行示例：
  python tools/ball_link.py send   --port COM7 --scenario
  python tools/ball_link.py decode --port COM7 --csv telemetry.csv
  python tools/ball_link.py sim    --port COM7 --csv sim_log.csv
"""

import argparse
import csv
import math
import struct
import sys
import time

SYNC0 = 0xA5
SYNC1 = 0x5A
VERSION = 0x01
TYPE_BALL = 0x01
TYPE_TELEMETRY = 0x81

FRAME_OVERHEAD = 9  # 同步 2 + 版本/类型/长度/序号 5 + CRC 2

# 测试 B 的静态场景：(钢珠位置 cm, 是否有效)
TEST_B_SCENARIOS = [
    (-2.0, True),
    (2.0, True),
    (0.0, True),
    (3.0, True),
    (0.0, False),
]


def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT-FALSE：初值 0xFFFF，多项式 0x1021。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(frame_type: int, seq: int, payload: bytes) -> bytes:
    """按协议组一帧。"""
    body = struct.pack("<BBBH", VERSION, frame_type, len(payload), seq) + payload
    crc = crc16_ccitt(body)
    return bytes([SYNC0, SYNC1]) + body + struct.pack("<H", crc)


def build_ball_payload(x_cm: float, valid: bool = True, calibrated: bool = True) -> bytes:
    """视觉位置帧负载：时间戳、x、置信度、标志位、保留。"""
    frame_time_ms = int(time.monotonic() * 1000.0) & 0xFFFFFFFF
    flags = (0x01 if valid else 0x00) | (0x02 if calibrated else 0x00)
    return struct.pack("<IfBBH", frame_time_ms, float(x_cm), 95, flags, 0)


class FrameParser:
    """增量解析器：从串口字节流中恢复 A5 5A 帧。"""

    MAX_PAYLOAD = 64

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data: bytes):
        self.buf.extend(data)
        frames = []

        while True:
            start = self.buf.find(bytes([SYNC0, SYNC1]))
            if start < 0:
                # 保留最后一个可能的 SYNC0，其余丢弃
                if self.buf and self.buf[-1] == SYNC0:
                    del self.buf[:-1]
                else:
                    self.buf.clear()
                break

            if start > 0:
                del self.buf[:start]

            if len(self.buf) < 5:
                break

            payload_len = self.buf[4]
            if payload_len > self.MAX_PAYLOAD:
                del self.buf[0]
                continue

            total = FRAME_OVERHEAD + payload_len
            if len(self.buf) < total:
                break

            frame = bytes(self.buf[:total])
            del self.buf[:total]

            body = frame[2:7 + payload_len]
            crc_rx = struct.unpack_from("<H", frame, 7 + payload_len)[0]
            if crc16_ccitt(body) != crc_rx:
                continue

            frame_type = frame[3]
            seq = struct.unpack_from("<H", frame, 5)[0]
            payload = frame[7:7 + payload_len]
            frames.append((frame_type, seq, payload))

        return frames


def parse_telemetry(payload: bytes):
    """解析 32 字节遥测负载，返回字典；长度不符返回 None。"""
    if len(payload) != 32:
        return None

    (time_ms, target, x_cm, v_cm_s, error_cm, beam_cmd,
     pulse_us, state, flags, beam_actual) = struct.unpack("<IfffffHBBf", payload)

    return {
        "time_ms": time_ms,
        "target_cm": target,
        "ball_x_cm": x_cm,
        "ball_v_cm_s": v_cm_s,
        "error_cm": error_cm,
        "beam_cmd_deg": beam_cmd,
        "servo_pwm_us": pulse_us,
        "state": state,
        "vision_valid": bool(flags & 0x01),
        "auto_enabled": bool(flags & 0x02),
        "fault": bool(flags & 0x04),
        "beam_actual_deg": beam_actual,
    }


STATE_NAMES = {0: "IDLE", 1: "MANUAL", 2: "AUTO", 3: "VLOST", 4: "FAULT"}


def open_serial(port: str, baud: int):
    try:
        import serial  # type: ignore
    except ImportError:
        print("缺少 pyserial，请先执行: pip install pyserial", file=sys.stderr)
        raise SystemExit(2)
    return serial.Serial(port, baud, timeout=0)


class CsvWriter:
    FIELDS = [
        "host_time_s", "time_ms", "target_cm", "ball_x_cm", "ball_v_cm_s",
        "error_cm", "beam_cmd_deg", "servo_pwm_us", "state", "state_name",
        "vision_valid", "auto_enabled", "fault", "beam_actual_deg",
    ]

    def __init__(self, path):
        self.file = open(path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=self.FIELDS)
        self.writer.writeheader()

    def write(self, telemetry, host_time_s):
        row = {"host_time_s": f"{host_time_s:.3f}", "state_name": STATE_NAMES.get(telemetry["state"], "?")}
        for key in ("time_ms", "target_cm", "ball_x_cm", "ball_v_cm_s", "error_cm",
                    "beam_cmd_deg", "servo_pwm_us", "state", "vision_valid",
                    "auto_enabled", "fault", "beam_actual_deg"):
            row[key] = telemetry[key]
        self.writer.writerow(row)

    def close(self):
        self.file.close()


def run_send(ser, args):
    """按 30 Hz 发送位置帧。"""
    parser = FrameParser()
    period = 1.0 / args.rate
    next_t = time.monotonic()
    seq = 0
    scenario_index = 0
    scenario_start = time.monotonic()

    print(f"send: {args.rate:.0f} Hz, mode={args.mode}, Ctrl+C 退出")
    while True:
        now = time.monotonic()

        if args.mode == "fixed":
            x_cm, valid = args.x, True
        elif args.mode == "sine":
            x_cm = 5.0 * math.sin(2.0 * math.pi * args.freq * now)
            valid = True
        else:  # scenario
            if now - scenario_start >= 2.0:
                scenario_start = now
                scenario_index = (scenario_index + 1) % len(TEST_B_SCENARIOS)
            x_cm, valid = TEST_B_SCENARIOS[scenario_index]

        seq = (seq + 1) & 0xFFFF
        ser.write(build_frame(TYPE_BALL, seq, build_ball_payload(x_cm, valid)))

        # 顺带读取 G3519 遥测，避免接收缓冲堆积
        data = ser.read(256)
        if data:
            for frame_type, _, payload in parser.feed(data):
                if frame_type == TYPE_TELEMETRY:
                    telemetry = parse_telemetry(payload)
                    if telemetry:
                        print(
                            f"\rS={STATE_NAMES.get(telemetry['state'], '?'):5s} "
                            f"x={telemetry['ball_x_cm']:+.2f} "
                            f"v={telemetry['ball_v_cm_s']:+.2f} "
                            f"beam={telemetry['beam_cmd_deg']:+.2f} "
                            f"vis={int(telemetry['vision_valid'])}",
                            end="", flush=True,
                        )

        next_t += period
        sleep_time = next_t - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)
        else:
            next_t = time.monotonic()


def run_decode(ser, args):
    """接收遥测并打印/写 CSV。"""
    parser = FrameParser()
    writer = CsvWriter(args.csv) if args.csv else None
    start = time.monotonic()

    print("decode: Ctrl+C 退出")
    try:
        while True:
            data = ser.read(512)
            if not data:
                continue
            for frame_type, _seq, payload in parser.feed(data):
                if frame_type != TYPE_TELEMETRY:
                    continue
                telemetry = parse_telemetry(payload)
                if telemetry is None:
                    continue
                if writer:
                    writer.write(telemetry, time.monotonic() - start)
                print(
                    f"t={telemetry['time_ms']:>8d} "
                    f"st={STATE_NAMES.get(telemetry['state'], '?'):5s} "
                    f"tgt={telemetry['target_cm']:+6.2f} "
                    f"x={telemetry['ball_x_cm']:+6.2f} "
                    f"v={telemetry['ball_v_cm_s']:+7.2f} "
                    f"e={telemetry['error_cm']:+6.2f} "
                    f"ang={telemetry['beam_cmd_deg']:+6.2f} "
                    f"act={telemetry['beam_actual_deg']:+6.2f} "
                    f"pwm={telemetry['servo_pwm_us']:4d} "
                    f"vis={int(telemetry['vision_valid'])}"
                )
    finally:
        if writer:
            writer.close()


def run_sim(ser, args):
    """
    半实物闭环仿真。

    每一帧遥测里的 beam_actual_deg 作为摆杆真实倾角，按滚动钢珠模型：
        x'' = (5/7) g sin(theta)
    积分出钢珠位置，再以 30 Hz 发回位置帧。G3519 完全不知道数据是模拟的，
    因此可以验证 UART、PD、舵机映射和状态机整条链路。
    """
    parser = FrameParser()
    writer = CsvWriter(args.csv) if args.csv else None
    period = 1.0 / args.rate
    next_t = time.monotonic()
    last_t = next_t

    x_cm = args.start_x
    v_cm_s = 0.0
    beam_actual = 0.0
    seq = 0

    print(f"sim: start_x={x_cm:+.1f} cm, {args.rate:.0f} Hz, Ctrl+C 退出")
    try:
        while True:
            now = time.monotonic()
            dt = min(max(now - last_t, 0.0), 0.1)
            last_t = now

            # 读取遥测，更新实际摆杆倾角
            data = ser.read(512)
            if data:
                for frame_type, _seq, payload in parser.feed(data):
                    if frame_type != TYPE_TELEMETRY:
                        continue
                    telemetry = parse_telemetry(payload)
                    if telemetry is None:
                        continue
                    beam_actual = telemetry["beam_actual_deg"]
                    if writer:
                        writer.write(telemetry, now)

            # 钢珠动力学积分（小角度下 sinθ ≈ θ）
            accel = (5.0 / 7.0) * 981.0 * math.sin(math.radians(beam_actual))
            v_cm_s += accel * dt
            x_cm += v_cm_s * dt

            # 摆杆两端挡板
            if x_cm > args.travel:
                x_cm = args.travel
                v_cm_s = -0.25 * v_cm_s
            elif x_cm < -args.travel:
                x_cm = -args.travel
                v_cm_s = -0.25 * v_cm_s

            seq = (seq + 1) & 0xFFFF
            ser.write(build_frame(TYPE_BALL, seq, build_ball_payload(x_cm, True)))

            print(
                f"\rsim x={x_cm:+6.2f} v={v_cm_s:+7.2f} "
                f"beam_act={beam_actual:+6.2f}",
                end="", flush=True,
            )

            next_t += period
            sleep_time = next_t - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                next_t = time.monotonic()
    finally:
        if writer:
            writer.close()


def main():
    parser = argparse.ArgumentParser(description="G3519 滚球控制串口联调工具")
    sub = parser.add_subparsers(dest="command", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--port", required=True, help="串口号，例如 COM7 或 /dev/ttyUSB0")
    common.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")

    p_send = sub.add_parser("send", parents=[common], help="发送模拟视觉位置")
    p_send.add_argument("--mode", choices=["fixed", "sine", "scenario"], default="scenario")
    p_send.add_argument("--x", type=float, default=0.0, help="fixed 模式下的位置 cm")
    p_send.add_argument("--freq", type=float, default=0.2, help="sine 模式频率 Hz")
    p_send.add_argument("--rate", type=float, default=30.0)
    p_send.set_defaults(func=run_send)

    p_decode = sub.add_parser("decode", parents=[common], help="解码遥测")
    p_decode.add_argument("--csv", default="", help="保存为 CSV 文件")
    p_decode.set_defaults(func=run_decode)

    p_sim = sub.add_parser("sim", parents=[common], help="半实物闭环仿真")
    p_sim.add_argument("--start-x", type=float, default=5.0)
    p_sim.add_argument("--travel", type=float, default=18.0)
    p_sim.add_argument("--rate", type=float, default=30.0)
    p_sim.add_argument("--csv", default="", help="保存遥测为 CSV 文件")
    p_sim.set_defaults(func=run_sim)

    args = parser.parse_args()
    ser = open_serial(args.port, args.baud)
    try:
        args.func(ser, args)
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
