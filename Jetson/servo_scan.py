# -*- coding: utf-8 -*-
"""G3519 舵机开环扫描。默认只打印计划，显式 --live 才打开指定串口。"""
import argparse
import csv
from decimal import Decimal, ROUND_CEILING
from itertools import chain
import math
import os
import select
import struct
import sys
import termios
import time
from collections import deque, namedtuple
from contextlib import closing

from position_feedback import LinkJournal, SerialFeedback, TelemetryParser, crc16


PERIOD = .050
TIMEOUT = .150
READY_WAIT = 5.
# 与固件 servo.h 的远程目标窗一致；现场还须显式确认本次扫描的较小窗口。
MIN_REMOTE_TICKS = 1333
MAX_REMOTE_TICKS = 1422
Target = namedtuple('Target', 'deg ticks')
CONTROL_FIELDS = LinkJournal.fields[12:]
CSV_FIELDS = ('event', 'nano_timestamp', 'command_seq', 'target_servo_deg',
              'target_pwm_ticks', 'output_pwm_ticks', 'board_timestamp', 'remote_state',
              'telemetry_seq', 'plateau_index', 'reason') + CONTROL_FIELDS


def degrees_to_ticks(degrees):
    """绝对舵机角映射到 800 kHz 脉宽计数，与 servo.c 相同，半计数向上取整。"""
    if not 0 <= degrees <= 180:
        raise ValueError('servo degrees outside 0..180')
    return int(400 + 1600 * degrees / 180. + .5)


def ticks_to_degrees(ticks):
    """返回脉宽的等效舵机角度；输出角度和实际摆杆倾角应分别记录。"""
    if not 400 <= ticks <= 2000 or int(ticks) != ticks:
        raise ValueError('expected integer servo pulse ticks in 400..2000')
    return (ticks - 400) * 180. / 1600


def _target(unit, value):
    if unit == 'deg':
        ticks = degrees_to_ticks(value)
        degrees = value
    elif unit == 'ticks':
        degrees = ticks_to_degrees(value)
        ticks = int(value)
    else:
        raise ValueError('unit must be deg or ticks')
    if not MIN_REMOTE_TICKS <= ticks <= MAX_REMOTE_TICKS:
        raise ValueError('target outside firmware remote window 1333..1422 ticks')
    return Target(degrees, ticks)


def make_plan(unit, low, high, step, hold_ms, direction='up'):
    """先检查两端与固件窗口，再返回平台迭代器。down 从高端沿相同台阶折返。"""
    if not all(math.isfinite(v) for v in (low, high, step)) or low > high or step <= 0:
        raise ValueError('expected finite min <= max and positive step')
    if hold_ms <= 0 or hold_ms % 50:
        raise ValueError('hold-ms must be a positive multiple of 50')
    if direction not in ('up', 'down'):
        raise ValueError('direction must be up or down')
    if unit == 'ticks' and int(step) != step:
        raise ValueError('expected integer tick step')
    _target(unit, low)
    _target(unit, high)

    # 按十进制计算台阶，避免浮点累加在折返点多生成一级。最后不足一步时仍含端点。
    low, high, step = (Decimal(str(v)) for v in (low, high, step))
    last = int(((high - low) / step).to_integral_value(rounding=ROUND_CEILING))
    if direction == 'up':
        indices = chain(range(last + 1), range(last - 1, -1, -1))
    else:
        indices = chain(range(last, -1, -1), range(1, last + 1))
    return (_target(unit, float(min(high, low + i * step))) for i in indices)


def command_packet(seq, ticks):
    """0x02 的序号位于帧头，两字节负载仅含绝对脉宽计数。"""
    if not MIN_REMOTE_TICKS <= ticks <= MAX_REMOTE_TICKS:
        raise ValueError('command outside firmware remote window')
    body = struct.pack('<BBBHH', 1, 2, 2, seq, ticks)
    return b'\xa5\x5a' + body + struct.pack('<H', crc16(body))


class ScanParser(object):
    """两类遥测共用一个帧边界，避免把有效负载内的同步字当作另一个包。"""
    def __init__(self):
        self.buffer = bytearray()
        self.control = TelemetryParser()

    def feed(self, data):
        return list(self.iter_rows(data))

    def iter_rows(self, data):
        # 流式交付完整帧，后续 CRC 错误发生时此前的有效遥测仍会入日志。
        buf = self.buffer
        buf.extend(data)
        while True:
            start = buf.find(b'\xa5\x5a')
            if start < 0:
                # 仅保留可能被拆分的同步字首字节。
                if buf.endswith(b'\xa5'):
                    del buf[:-1]
                else:
                    buf.clear()
                break
            del buf[:start]
            if len(buf) < 5:
                break
            version, kind, length = buf[2:5]
            if version != 1 or kind not in (0x81, 0x82):
                del buf[:1]
                continue
            if length != (32 if kind == 0x81 else 11):
                raise ValueError('invalid telemetry length')
            size = 9 + length
            if len(buf) < size:
                break
            packet = buf[:size]
            del buf[:size]
            if kind == 0x81:
                # 复用既有 CRC、二进制布局和 CSV 字段顺序，与位置监视工具保持一致。
                decoded = self.control.feed(packet)
                if not decoded:
                    raise ValueError('control telemetry CRC')
                msg = decoded[0]
                row = dict(zip(CONTROL_FIELDS, msg[2:]))
                flags = row['telemetry_flags']
                row.update(event='control', telemetry_seq=msg[0], board_timestamp=msg[1],
                           board_vision_valid=flags & 1, board_auto_enabled=(flags >> 1) & 1,
                           board_fault=(flags >> 2) & 1)
            else:
                if crc16(packet[2:-2]) != struct.unpack_from('<H', packet, size - 2)[0]:
                    raise ValueError('remote telemetry CRC')
                seq, ms, ack, target, output, state = struct.unpack_from('<HIHHHB', packet, 5)
                row = dict(event='remote', telemetry_seq=seq, board_timestamp=ms,
                           command_seq=ack, target_pwm_ticks=target, output_pwm_ticks=output,
                           remote_state=state)
                if 400 <= target <= 2000:
                    row['target_servo_deg'] = ticks_to_degrees(target)
            yield row


class RemoteSession(object):
    """只以 0x82 管理链路；按回显序号匹配已发命令，可跨过两次遥测间的命令。"""
    def __init__(self, now):
        self.deadline = now + READY_WAIT
        self.next_seq = None
        self.pending = deque()
        self.accepted = None
        self.last_send_time = None

    def receive(self, row, now):
        state = row['remote_state']
        ack = row['command_seq']
        if state == 1 and self.last_send_time is None:
            self.next_seq = (ack + 1) & 0xffff
        elif state == 0 and self.next_seq is None:
            # 本地切入远程模式前可等待，DISARMED 不延长首次 READY 的等待时间。
            return
        elif state == 2 and self.last_send_time is not None:
            pair = (ack, row['target_pwm_ticks'])
            if pair != self.accepted:
                while self.pending:
                    seq, target, _ = self.pending.popleft()
                    if (seq, target) == pair:
                        self.accepted = pair
                        break
                else:
                    raise ValueError('unmatched command echo')
        else:
            raise ValueError('unexpected remote state %d' % state)
        self.deadline = now + TIMEOUT

    def expires(self):
        deadline = self.deadline
        if self.last_send_time is not None:
            # 即使最近的 ACK 仍新鲜，主机发送间隔也不可越过固件看门狗。
            deadline = min(deadline, self.last_send_time + TIMEOUT)
        if self.pending:
            deadline = min(deadline, self.pending[0][2] + TIMEOUT)
        return deadline

    def check_time(self, now):
        if now >= self.expires():
            raise TimeoutError('READY or remote telemetry/command echo timeout')


def run_scan(plan, hold_ms, read, send, stream, clock=time.monotonic):
    """read(wait) 返回收到的字节，空块表示等待超时；断线由传输层抛异常。"""
    log = csv.DictWriter(stream, fieldnames=CSV_FIELDS, lineterminator="\n")
    log.writeheader()
    parser = ScanParser()
    session = RemoteSession(clock())
    targets = iter(plan)
    remaining = 0
    index = -1
    due = None
    finished = False
    try:
        if hold_ms <= 0 or hold_ms % 50:
            raise ValueError('hold-ms must be a positive multiple of 50')
        while True:
            # 用较早的截止时刻唤醒。主机被挂起后，即使串口已有缓存也不能续发。
            now = clock()
            session.check_time(now)
            wake = session.expires()
            if due is not None and not finished:
                wake = min(wake, due)
            data = read(max(0., wake - now))
            now = clock()
            session.check_time(now)
            for row in parser.iter_rows(data):
                log.writerow(dict(row, nano_timestamp=now))
                if row['event'] == 'remote':
                    session.receive(row, now)
                elif row['board_fault']:
                    raise ValueError('board fault')
            now = clock()
            session.check_time(now)
            if session.next_seq is None:
                continue
            if due is None or now >= due:
                if not remaining and not finished:
                    target = next(targets, None)
                    if target is None:
                        finished = True
                    else:
                        remaining = hold_ms // 50
                        index += 1
                if finished:
                    if session.last_send_time is None:
                        raise ValueError('empty scan plan')
                    if not session.pending:
                        break
                    continue
                seq = session.next_seq
                send(command_packet(seq, target.ticks))
                written = clock()
                # TX 只记整帧写入完成时刻，最后一个平台保持满时且收到最终回显才完成。
                log.writerow(dict(event='tx', nano_timestamp=written, command_seq=seq,
                                  target_servo_deg=target.deg, target_pwm_ticks=target.ticks,
                                  plateau_index=index))
                session.last_send_time = now
                session.pending.append((seq, target.ticks, now))
                session.next_seq = (seq + 1) & 0xffff
                remaining -= 1
                due = now + PERIOD
    except (OSError, ValueError, KeyboardInterrupt) as exc:
        log.writerow(dict(event='stop', nano_timestamp=clock(), reason=str(exc) or 'interrupted'))
        return False
    log.writerow(dict(event='stop', nano_timestamp=clock(), reason='complete'))
    return True


def _set_single(unit, value, hold_ms, read, send, stream, confirmed_bounds, clock):
    low, high = confirmed_bounds
    _target(unit, low)
    _target(unit, high)
    if not low <= value <= high:
        raise ValueError('setpoint outside confirmed bounds')
    plan = make_plan(unit, value, value, 1, hold_ms)
    return run_scan(plan, hold_ms, read, send, stream, clock=clock)


def set_servo_angle(degrees, hold_ms, read, send, stream, *, confirmed_bounds, clock=time.monotonic):
    """执行一个绝对舵机角平台，返回是否完成并获得末包回显。边界单位也是度。

    例：set_servo_angle(110.1, 500, read, send, csv_stream, confirmed_bounds=(109, 111))
    read(wait_seconds) 与 send(packet) 由调用方提供；可接模拟链路或已显式打开的串口。
    每次调用等待新的 READY。连续多级实验应一次调用 run_scan。
    """
    return _set_single('deg', degrees, hold_ms, read, send, stream, confirmed_bounds, clock)


def set_pwm_ticks(ticks, hold_ms, read, send, stream, *, confirmed_bounds, clock=time.monotonic):
    """执行一个整数脉宽平台，每计数 1.25 µs；边界单位也是 ticks。

    例：set_pwm_ticks(1378, 500, read, send, csv_stream, confirmed_bounds=(1370, 1380))
    与 set_servo_angle 共用 20 Hz、READY 与回显超时规则，并各自记录本次会话。
    """
    return _set_single('ticks', ticks, hold_ms, read, send, stream, confirmed_bounds, clock)


def read_serial(port, wait):
    """沿用 SerialFeedback 的 115200 8N1 独占串口；EOF 立即结束扫描。"""
    if not select.select([port.fd], [], [], wait)[0]:
        return b''
    data = os.read(port.fd, 4096)
    if not data:
        raise OSError('serial disconnected')
    return data


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--unit', choices=('deg', 'ticks'), default='deg')
    ap.add_argument('--min', type=float, dest='low', help='扫描下限，绝对舵机角或脉宽计数')
    ap.add_argument('--max', type=float, dest='high', help='扫描上限，与 --min 使用同一单位')
    ap.add_argument('--step', type=float, default=1, help='台阶间距，与边界使用同一单位')
    ap.add_argument('--hold-ms', type=int, default=500, help='每级保持时间，50 ms 的正整数倍')
    ap.add_argument('--direction', choices=('up', 'down'), default='up', help='初始行进方向')
    ap.add_argument('--live', action='store_true', help='实际运行扫描')
    ap.add_argument('--confirm-bounds', action='store_true', help='确认所给的上下限已现场核实')
    ap.add_argument('--device', help='明确指定串口设备路径')
    ap.add_argument('--log', help='新建事件 CSV 路径')
    args = ap.parse_args(argv)
    if args.live and (not args.device or not args.log or not args.confirm_bounds
                      or args.low is None or args.high is None):
        ap.error('--live requires --device, --log, --min, --max and --confirm-bounds')
    if not args.live and args.log:
        ap.error('--log is for live events; the dry plan is printed on stdout')
    defaults = (109, 111) if args.unit == 'deg' else (1377, 1379)
    low = defaults[0] if args.low is None else args.low
    high = defaults[1] if args.high is None else args.high
    try:
        plan = make_plan(args.unit, low, high, args.step, args.hold_ms, args.direction)
    except ValueError as exc:
        ap.error(str(exc))
    if not args.live:
        writer = csv.writer(sys.stdout)
        writer.writerow(('plateau_index', 'start_ms', 'hold_ms', 'target_servo_deg',
                         'target_pwm_ticks', 'commands'))
        for i, target in enumerate(plan):
            writer.writerow((i, i * args.hold_ms, args.hold_ms, target.deg,
                             target.ticks, args.hold_ms // 50))
        return 0
    try:
        with open(args.log, 'x', newline='', buffering=1) as stream:
            try:
                port = SerialFeedback(args.device)
            except (OSError, termios.error) as exc:
                log = csv.DictWriter(stream, fieldnames=CSV_FIELDS, lineterminator="\n")
                log.writeheader()
                log.writerow(dict(event='stop', nano_timestamp=time.monotonic(), reason=str(exc)))
                raise
            with closing(port):
                return 0 if run_scan(plan, args.hold_ms, lambda wait: read_serial(port, wait),
                                     port.send, stream) else 1
    except (OSError, termios.error) as exc:
        print(str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
