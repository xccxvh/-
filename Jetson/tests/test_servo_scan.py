# -*- coding: utf-8 -*-
"""离线验证扫描计划、协议和停机门控。所有时间与串口读写均由替身提供。"""
import csv
import io
import struct
import termios
import unittest
from binascii import crc_hqx
from contextlib import redirect_stdout, redirect_stderr
from unittest.mock import patch

import servo_scan as scan


def frame(kind, seq, payload):
    # 独立 CRC 作为测试基准，避免收发代码共享同一个错误实现。
    body = struct.pack('<BBBH', 1, kind, len(payload), seq) + payload
    return b'\xa5\x5a' + body + struct.pack('<H', crc_hqx(body, 0xffff))


def status(seq, ms, ack, target, state):
    return frame(0x82, seq, struct.pack('<IHHHB', ms, ack, target, 1324, state))


def control(flags=0):
    return frame(0x81, 54, struct.pack('<I5fHBBf', 800, 1, 2, 3, 4, 5,
                                       1722, 1, flags, -1.25))


class FakeLink(object):
    """按绝对时间排队返回接收块，记录每次实际发送，不主动生成 ACK。"""
    def __init__(self, arrivals):
        self.now = 1.
        self.arrivals = list(arrivals)
        self.sent = []

    def clock(self):
        return self.now

    def read(self, wait):
        until = self.now + wait
        if self.arrivals and self.arrivals[0][0] <= until:
            stamp, data = self.arrivals.pop(0)
            self.now = max(self.now, stamp)
            if isinstance(data, Exception):
                raise data
            return data
        self.now = until
        return b''

    def send(self, packet):
        self.sent.append((self.now, packet))


class ScanTests(unittest.TestCase):
    def test_plan_degrees_ticks_and_reverse(self):
        plan = scan.make_plan('deg', 103, 104.1, .5, 100)
        self.assertEqual([p.deg for p in plan], [103, 103.5, 104, 104.1, 104, 103.5, 103])
        self.assertEqual([p.ticks for p in scan.make_plan('ticks', 1323, 1326, 2, 50)],
                         [1323, 1325, 1326, 1325, 1323])
        self.assertEqual(scan.degrees_to_ticks(104), 1324)
        self.assertEqual(scan.degrees_to_ticks(scan.ticks_to_degrees(1324)), 1324)
        for args in [('deg', 0, 180, 1, 100), ('ticks', 1200.5, 1324, 1, 100),
                     ('deg', 104, 103, 1, 100), ('deg', 104, 105, 0, 100),
                     ('deg', 104, 105, 1, 51), ('deg', float('nan'), 105, 1, 100)]:
            with self.assertRaises(ValueError):
                list(scan.make_plan(*args))

    def test_board_limits_before_iteration_and_reverse_first(self):
        for low, high in [(1279, 1324), (1324, 1369)]:
            with self.assertRaises(ValueError):
                scan.make_plan('ticks', low, high, 1, 50)
        for low, high in [(98.8, 104), (104, 109.2)]:
            with self.assertRaises(ValueError):
                scan.make_plan('deg', low, high, 1, 50)
        plan = scan.make_plan('ticks', 1280, 1368, 40, 50, direction='down')
        self.assertEqual([p.ticks for p in plan], [1368, 1360, 1320, 1280, 1320, 1360, 1368])

    def test_single_target_api_uses_same_ready_echo_and_bounds(self):
        # 用户显式传入自己确认的上下限，两个 API 均须先拿到 READY 才写串口。
        for setter, value, bounds in [(scan.set_servo_angle, 103.99, (103, 105)),
                                      (scan.set_pwm_ticks, 1324, (1323, 1325))]:
            link = FakeLink([(1.2, status(140, 10, 4, 1324, 1)),
                             (1.21, status(141, 20, 5, 1324, 2))])
            log = io.StringIO()
            self.assertTrue(setter(value, 50, link.read, link.send, log,
                                   confirmed_bounds=bounds, clock=link.clock))
            self.assertEqual(link.sent, [(1.2, frame(2, 5, struct.pack('<H', 1324)))])
            tx = next(r for r in csv.DictReader(io.StringIO(log.getvalue())) if r['event'] == 'tx')
            self.assertEqual(float(tx['target_servo_deg']),
                             value if setter is scan.set_servo_angle else scan.ticks_to_degrees(value))
            with self.assertRaises(ValueError):
                setter(value, 50, link.read, link.send, log,
                       confirmed_bounds=(bounds[1], bounds[1]), clock=link.clock)
            self.assertEqual(len(link.sent), 1)

    def test_invalid_plan_fails_before_open(self):
        with patch.object(scan, 'SerialFeedback') as port, patch('builtins.open') as open_log, \
                redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                scan.main(['--live', '--device', '/mock', '--log', 'unused', '--confirm-bounds',
                           '--min', '104', '--max', '120'])
            open_log.assert_not_called()
            port.assert_not_called()

    def test_mixed_split_and_crc(self):
        remote = status(92, 917, 65535, 1324, 1)
        data = b'noise' + control() + remote + control()
        for boundary in range(len(data) + 1):
            parser = scan.ScanParser()
            rows = parser.feed(data[:boundary]) + parser.feed(data[boundary:])
            self.assertEqual([r['event'] for r in rows], ['control', 'remote', 'control'])
            self.assertEqual(rows[0]['beam_estimated_deg'], -1.25)
            self.assertEqual(rows[1]['telemetry_seq'], 92)
            self.assertEqual(rows[1]['command_seq'], 65535)
        for packet in (remote, control()):
            with self.assertRaises(ValueError):
                scan.ScanParser().feed(packet[:-1] + bytes([packet[-1] ^ 1]))

    def exercise(self, arrivals, hold=100, success=False):
        link = FakeLink(arrivals)
        stream = io.StringIO()
        plan = scan.make_plan('deg', 104, 104, 1, hold)
        result = scan.run_scan(plan, hold, link.read, link.send, stream, clock=link.clock)
        self.assertEqual(result, success)
        self.rows = list(csv.DictReader(io.StringIO(stream.getvalue())))
        self.assertEqual(self.rows[-1]['event'], 'stop')
        return link.sent

    def test_handshake_wrap_timing_ack_and_log(self):
        sent = self.exercise([(1.02, control()), (1.20, status(750, 100, 65534, 0, 1)),
                              (1.22, status(751, 120, 65535, 1324, 2)),
                              (1.28, status(752, 180, 0, 1324, 2))], success=True)
        self.assertEqual(len(sent), 2)
        for (stamp, packet), expected_time, seq in zip(sent, [1.20, 1.25], [65535, 0]):
            self.assertAlmostEqual(stamp, expected_time)
            self.assertEqual(packet, frame(2, seq, struct.pack('<H', 1324)))
        tx = [r for r in self.rows if r['event'] == 'tx']
        self.assertEqual([float(r['target_servo_deg']) for r in tx], [104, 104])
        self.assertEqual(self.rows[0]['board_timestamp'], '800')
        self.assertEqual(self.rows[-2]['output_pwm_ticks'], '1324')
        self.assertGreaterEqual(float(self.rows[-1]['nano_timestamp']), 1.30)

    def test_loss_timeout_errors_and_stale_ack(self):
        ready = (1., status(0, 0, 0, 1324, 1))
        active = (1.02, status(1, 20, 1, 1324, 2))
        self.assertEqual(self.exercise([(1.01, control())]), [])
        for packet in [status(2, 21, 1, 1324, 0), status(2, 21, 1, 1324, 3),
                       status(2, 21, 5, 1324, 2), status(2, 21, 1, 1323, 2),
                       status(2, 21, 1, 1324, 1), control(4), OSError('disconnect')]:
            sent = self.exercise([ready, active, (1.03, packet)], hold=500)
            self.assertEqual(len(sent), 1)
        # 单收 0x81 或重复回显旧命令都不能续命。已发末包也须得到回显。
        self.assertLess(len(self.exercise([ready] + [(1.03+i*.04, control())
                               for i in range(8)], hold=500)), 4)
        self.assertLess(len(self.exercise([ready, active] + [(1.05+i*.04,
                            status(2+i, 50+40*i, 1, 1324, 2)) for i in range(8)],
                            hold=500)), 5)
        self.exercise([ready], hold=50)

    def test_host_stall_cannot_resume_after_command_deadline(self):
        # 已应答的命令也有板端看门狗。模拟 ACK 后主机停顿超过上次发送的 150 ms。
        link = FakeLink([(1., status(0, 0, 0, 1324, 1)), (1.04, status(1, 40, 1, 1324, 2))])
        def read(wait):
            if not link.arrivals and link.now < 1.15:
                link.now = 1.16
                return control()
            return link.read(wait)
        self.assertFalse(scan.run_scan(scan.make_plan('deg', 104, 104, 1, 500), 500,
                                       read, link.send, io.StringIO(), clock=link.clock))
        self.assertEqual(len(link.sent), 1)

    def test_corruption_logs_earlier_frames_and_cancels_tx(self):
        packet = status(18, 150, 3, 1324, 2)
        bad = packet[:-1] + bytes([packet[-1] ^ 1])
        self.assertEqual(self.exercise([(1., control() + status(4, 1, 2, 1324, 1) + bad)]), [])
        self.assertEqual([r['event'] for r in self.rows], ['control', 'remote', 'stop'])
        self.assertIn('CRC', self.rows[-1]['reason'])

    def test_open_error_is_journalled(self):
        # 模拟 tcsetattr 失败，验证既无命令也有退出行，且不会尝试重连。
        log = io.StringIO()
        with patch.object(scan, 'SerialFeedback', side_effect=termios.error('setup failed')) as port, \
                patch('builtins.open') as open_log, redirect_stderr(io.StringIO()):
            open_log.return_value.__enter__.return_value = log
            self.assertEqual(scan.main(['--live', '--confirm-bounds', '--device', '/mock',
                                        '--log', 'unused', '--min', '103', '--max', '105']), 1)
        port.assert_called_once_with('/mock')
        row, = csv.DictReader(io.StringIO(log.getvalue()))
        self.assertEqual((row['event'], row['reason']), ('stop', 'setup failed'))

    def test_cli_dry_default_and_live_gates(self):
        with patch.object(scan, 'SerialFeedback') as port, redirect_stdout(io.StringIO()), \
                redirect_stderr(io.StringIO()):
            self.assertEqual(scan.main([]), 0)
            for args in [['--live'], ['--live', '--device', '/mock', '--log', 'unused'],
                         ['--live', '--device', '/mock', '--log', 'unused', '--confirm-bounds']]:
                with self.assertRaises(SystemExit):
                    scan.main(args)
            port.assert_not_called()


if __name__ == '__main__':
    unittest.main()
