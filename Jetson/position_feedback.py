# -*- coding: utf-8 -*-
"""Versioned, opt-in measurement telemetry. Does not command any actuator."""
import math
import os
import select
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

    def encode(self, out, capture_time, now, pose_ok, enabled=False, source_time_verified=True):
        reason = 'ok'
        finite_time = (capture_time is not None and math.isfinite(capture_time)
                       and math.isfinite(now) and 0 <= capture_time <= 2**53/1000
                       and 0 <= now <= 2**53/1000)
        age = (now-capture_time)*1000 if finite_time else -1
        x = out.get('x_cm')
        conf = out.get('confidence',0.)
        if not finite_time or age < 0:
            reason = 'clock'
        elif self.last_capture is not None and capture_time <= self.last_capture:
            reason = 'duplicate'
        elif age > self.max_age_ms:
            reason = 'stale'
        elif not pose_ok:
            reason = 'pose'
        elif not out.get('valid') or out.get('predicted'):
            reason = 'vision'
        elif x is None or not math.isfinite(x) or not math.isfinite(conf) or not 0 <= conf <= 1:
            reason = 'calibration'
        elif not source_time_verified:
            reason = 'source_time'
        elif not enabled:
            reason = 'monitor'
        if finite_time and age >= 0 and (self.last_capture is None or capture_time > self.last_capture):
            self.last_capture = capture_time
        valid = reason == 'ok'
        stamp = int(capture_time*1000) & 0xffffffff if finite_time else 0
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
