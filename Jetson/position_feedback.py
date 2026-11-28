# -*- coding: utf-8 -*-
"""Versioned, opt-in measurement telemetry. Does not command any actuator."""
import csv
import math
from numbers import Real
import os
import select
import struct
import threading
import termios
import time
import uuid


def crc16(data):
    """CRC-16/CCITT-FALSE: poly=0x1021, init=0xffff, xorout=0."""
    crc = 0xffff
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xffff
    return crc


class FeedbackEncoder(object):
    def __init__(self, session=None, max_age_ms=100):
        self.session = uuid.uuid4().int & 0xffffffff if session is None else session
        self.seq = 0
        self.last_capture = None
        self.max_age_ms = max_age_ms

    def assess(self, out, x, capture_time, now, pose_ok, enabled, source_time_verified,
               position_control=False):
        # 两种协议共用帧时间、单调顺序和图像有效性门控；时钟只在 Jetson 内比较。
        reason = 'ok'
        finite_time = (isinstance(capture_time, Real) and math.isfinite(capture_time)
                       and isinstance(now, Real) and math.isfinite(now) and 0 <= capture_time <= 2**53/1000
                       and 0 <= now <= 2**53/1000)
        age = (now-capture_time)*1000 if finite_time else -1
        conf = out.get('confidence',0.)
        if not finite_time or age < 0:
            reason = 'clock'
        elif self.last_capture is not None and capture_time <= self.last_capture:
            reason = 'duplicate'
        # 控制模式须先查完同帧测量与时间来源，才可判定是否仅因年龄跳过。
        elif not position_control and age > self.max_age_ms:
            reason = 'stale'
        elif not pose_ok:
            reason = 'pose'
        elif not out.get('valid') or out.get('predicted'):
            reason = 'vision'
        elif (not isinstance(x, Real) or not math.isfinite(x) or
              not isinstance(conf, Real) or not 0 <= conf <= 1):
            reason = 'calibration'
        elif not source_time_verified:
            reason = 'source_time'
        elif not enabled:
            reason = 'monitor'
        # 有限且向前的时刻沿用原去重规则，静默跳过也不能让同一曝光再次发布。
        if finite_time and age >= 0 and (self.last_capture is None or capture_time > self.last_capture):
            self.last_capture = capture_time
        stamp = int(capture_time*1000) & 0xffffffff if finite_time else 0
        return reason, stamp, age

    def encode(self, out, capture_time, now, pose_ok, enabled=False, source_time_verified=True):
        x = out.get('x_cm')
        conf = out.get('confidence', 0.)
        reason, stamp, age = self.assess(out, x, capture_time, now, pose_ok, enabled, source_time_verified)
        valid = reason == 'ok'
        payload = 'BV1,%08X,%u,%u,%d,%d,%s,%d,%s' % (
            self.session, self.seq, stamp, max(-1,min(2147483647,int(age))),
            int(valid), '%.4f' % x if valid else '',
            int(conf*1000) if math.isfinite(conf) and 0 <= conf <= 1 else 0, reason)
        self.seq = (self.seq+1) & 0xffffffff
        raw = payload.encode('ascii')
        return b'$'+raw+('*%04X\n' % crc16(raw)).encode('ascii'), valid, reason


class SerialFeedback(object):
    """Linux 115200 8N1, bounded writes; close on any transport failure.

    No reconnection or retries: receiver must time out and require re-arming.
    """
    def __init__(self, device):
        self.fd = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            import fcntl
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            attr = termios.tcgetattr(self.fd)
            attr[0] = 0
            attr[1] = 0
            attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attr[3] = 0
            attr[4] = attr[5] = termios.B115200
            attr[6][termios.VMIN] = 0
            attr[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attr)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except BaseException:
            self.close()
            raise

    def send(self, line):
        deadline = time.monotonic()+0.01
        remaining = line
        try:
            while remaining:
                wait = deadline-time.monotonic()
                if wait <= 0 or not select.select([], [self.fd], [], wait)[1]:
                    raise TimeoutError('UART write exceeded 10 ms')
                try:
                    n = os.write(self.fd, remaining)
                except BlockingIOError:
                    continue
                if n <= 0:
                    raise OSError('UART write failed')
                remaining = remaining[n:]
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.fd is not None:
            fd, self.fd = self.fd, None
            os.close(fd)


class G3519MonitorEncoder(FeedbackEncoder):
    """按 vision_link.h 编码 0x01；监测坐标可读，但 bit0 永远清零。"""
    control_capable = False
    remote_pwm = False

    def encode(self, out, capture_time, now, pose_ok, enabled=False, source_time_verified=False):
        x = out.get('rail_x_cm')
        calibrated = bool(out.get('calibration_valid'))
        eligible = calibrated and out.get('cm_valid') and out.get('pixel_valid')
        control = (enabled or self.remote_pwm) and self.control_capable
        reason, stamp, _ = self.assess(out, x if eligible else None, capture_time, now,
                                       pose_ok and out.get('reference_valid'), control, source_time_verified,
                                       position_control=control)
        # source_time 只表示传感器时间未经验证。新鲜的本帧厘米值可监测；
        # 缺测、预测、过期等情况清空置信度，使用协议浮点占位值 0。
        if control and reason == 'source_time':
            reason = 'unverified_capture_time'
        if control and reason == 'ok':
            # vision_link.c 和 ball_control.c 均接受 ±30 cm、线上整数置信度 >= 20。
            # 越界/低可信即使同时超时也发送无效包，使接收端立即感知该帧失效。
            if abs(x) > 30.0:
                reason = 'position_range'
            elif round(out['confidence'] * 100) < 20:
                reason = 'low_confidence'
            elif self.remote_pwm:
                # Time of exposure travels with this diagnostic position. Only the
                # Nano PWM loop can admit it; the G3519 AUTO bit stays clear.
                reason = 'remote_observation'
            elif now > capture_time + self.max_age_ms / 1000.0:
                # 比较同一时钟上的截止点，恰好 100 ms 仍有效。返回前不分配序号。
                return None, False, 'stale_only_drop'
        valid = reason == 'ok'
        present = valid or reason == 'remote_observation' or (not control and reason in ('monitor', 'source_time'))
        flags = int(valid) | (2 if calibrated else 0)
        payload = struct.pack('<IfBBH', stamp, x if present else 0.,
                              round(out['confidence']*100) if present else 0, flags, 0)
        body = struct.pack('<BBBH', 1, 1, len(payload), self.seq) + payload
        self.seq = (self.seq + 1) & 0xffff
        return b'\xa5\x5a' + body + struct.pack('<H', crc16(body)), valid, reason


class G3519PositionEncoder(G3519MonitorEncoder):
    """仅在调用者显式使能且新鲜度、几何和身份均通过时设置 bit0。"""
    control_capable = True


class TelemetryParser(object):
    """0x81 帧解码；支持分包、粘包、CRC 出错后在下一同步字处恢复。"""
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        messages = []
        # 遥测头有固定版本、类型和长度，先同步完整五字节头。
        header = b'\xa5\x5a\x01\x81\x20'
        while True:
            start = self.buffer.find(header)
            if start < 0:
                del self.buffer[:-4]
                break
            del self.buffer[:start]
            if len(self.buffer) < 41:
                break
            if crc16(self.buffer[2:39]) != struct.unpack_from('<H', self.buffer, 39)[0]:
                self.crc_errors += 1
                del self.buffer[:1]
                continue
            messages.append(struct.unpack_from('<HI5fHBBf', self.buffer, 5))
            del self.buffer[:41]
        return messages


class LinkJournal(object):
    """以本地单调时钟记录 TX/RX 事件。接收线程独立于视觉帧，不重复旧遥测。"""
    fields = ('event', 'local_monotonic_ms', 'frame', 'position_seq', 'capture_ms32',
              'rail_x_cm', 'confidence_pct', 'position_flags', 'reason', 'timestamp_verified',
              'telemetry_seq', 'g3519_ms', 'target_cm', 'board_ball_cm', 'ball_v_cm_s',
              'position_error_cm', 'beam_cmd_deg', 'servo_pwm_us', 'control_state',
              'telemetry_flags', 'beam_estimated_deg', 'board_vision_valid', 'board_auto_enabled',
              'board_fault')

    def __init__(self, serial, stream, split_rx=None):
        self.split_rx = split_rx
        self.serial = serial
        self.writer = csv.DictWriter(stream, fieldnames=self.fields)
        self.writer.writeheader()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.parser = TelemetryParser()
        self.rx_bytes = self.rx_frames = self.tx_frames = 0
        self.error = None
        self.thread = threading.Thread(target=self._receive, daemon=True)
        self.thread.start()

    def _row(self, values):
        with self.lock:
            self.writer.writerow(values)

    def _receive(self):
        try:
            # 在每次实际 read 返回后打接收时间戳；同一批字节内的帧共享该时间。
            fd = self.serial.fd
            while not self.stop.is_set():
                if not select.select([fd], [], [], .05)[0]:
                    continue
                data = os.read(fd, 4096)
                received = time.perf_counter()*1000.
                if not data:
                    raise OSError('USB serial disconnected')
                self.rx_bytes += len(data)
                if self.split_rx is not None:
                    data = self.split_rx(data, time.monotonic())
                for msg in self.parser.feed(data):
                    row = dict(zip(self.fields[10:], msg))
                    # 0x81 的帧头序号属于遥测发送端，负载未回显接收的位置帧序号。
                    flags = row['telemetry_flags']
                    row.update(board_vision_valid=flags & 1, board_auto_enabled=(flags >> 1) & 1,
                               board_fault=(flags >> 2) & 1)
                    row.update(event='rx', local_monotonic_ms=received)
                    self._row(row)
                    self.rx_frames += 1
        except (OSError, ValueError) as exc:
            self.error = exc

    def dropped(self, frame, capture_time, reason):
        # 只有已核验 SOE 的仅超时控制样本走到这里。无线上包，因此序号和标志留空。
        self._row(dict(event='drop', local_monotonic_ms=time.perf_counter()*1000.,
                       frame=frame, capture_ms32=int(capture_time*1000) & 0xffffffff,
                       reason=reason, timestamp_verified=1))
        if self.error:
            raise self.error

    def sent(self, packet, frame, reason, source_verified):
        # 主线程仅在整帧写完后记 TX；成功 write 不代表 G3519 已解码或应答。
        seq, stamp, x, confidence, flags = struct.unpack_from('<HIfBB', packet, 5)
        self._row(dict(event='tx', local_monotonic_ms=time.perf_counter()*1000.,
                       frame=frame, position_seq=seq, capture_ms32=stamp,
                       rail_x_cm=x if reason in ('monitor','source_time','ok') else '',
                       confidence_pct=confidence, position_flags=flags, reason=reason,
                       timestamp_verified=int(source_verified)))
        self.tx_frames += 1
        if self.error:
            raise self.error

    def close(self):
        self.stop.set()
        self.thread.join()
        print('G3519 TX=%d RX_bytes=%d telemetry=%d CRC_errors=%d' %
              (self.tx_frames, self.rx_bytes, self.rx_frames, self.parser.crc_errors))
        if self.error:
            raise self.error


def listen():
    """单独测试 USB 回传。只打开串口并收遥测，无需启动摄像头或发送位置帧。"""
    import argparse
    from contextlib import closing
    ap = argparse.ArgumentParser(description='仅监听 G3519 0x81 帧，115200 8N1')
    ap.add_argument('device')
    ap.add_argument('output', help='新建 CSV 文件')
    ap.add_argument('--seconds', type=float, default=5.)
    args = ap.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0:
        ap.error('--seconds must be finite and positive')
    with open(args.output, 'x', newline='', buffering=1) as stream, closing(SerialFeedback(args.device)) as port:
        journal = LinkJournal(port, stream)
        try:
            time.sleep(args.seconds)
        finally:
            journal.close()
    return 0 if journal.rx_frames else 2


if __name__ == '__main__':
    raise SystemExit(listen())
