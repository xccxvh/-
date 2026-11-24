"""监测帧协议回归：CRC、控制有效位、时间新鲜度和连续字节流。"""
import struct
import unittest
from binascii import crc_hqx
from position_feedback import G3519MonitorEncoder, G3519PositionEncoder, TelemetryParser


class MonitorTests(unittest.TestCase):
    def test_monitor_position_and_safety_flags(self):
        encoder = G3519MonitorEncoder()
        out = dict(valid=True, pixel_valid=True, predicted=False, cm_valid=True, reference_valid=True,
                   calibration_valid=True, rail_x_cm=-1.25, confidence=.82)
        def wire(ts, now):
            frame, allowed, reason = encoder.encode(out, ts, now, True, True, False)
            self.assertFalse(allowed)
            self.assertEqual(frame[:5], bytes.fromhex('a55a01010c'))
            self.assertEqual(crc_hqx(frame[2:-2], 0xffff), int.from_bytes(frame[-2:], 'little'))
            stamp, x, confidence, flags, reserved = struct.unpack('<IfBBH', frame[7:-2])
            self.assertEqual(stamp, int(ts*1000))
            self.assertFalse(flags & 1)
            self.assertEqual(reserved, 0)
            return x, confidence, flags, reason
        self.assertEqual(wire(1.,1.01), (-1.25,82,2,'source_time'))
        self.assertEqual(wire(1.,1.02), (0.,0,2,'duplicate'))
        self.assertEqual(wire(2.,2.5), (0.,0,2,'stale'))
        out['predicted'] = True
        self.assertEqual(wire(3.,3.01)[:3], (0.,0,2))
        out.update(predicted=False, calibration_valid=False)
        self.assertEqual(wire(4.,4.01)[:3], (0.,0,0))

    def test_explicit_control_requires_same_frame_position_and_verified_time(self):
        # 新协议与原板逐字节兼容；逐一注入错误，防止预测或旧时间开启 bit0。
        base = dict(valid=True, pixel_valid=True, cm_valid=True, calibration_valid=True,
                    reference_valid=True, predicted=False, rail_x_cm=-.97, confidence=.83)
        for change, age, verified, enabled, expected in [
                ({}, .04, True, True, 3), ({}, .04, True, False, 2),
                ({}, .04, False, True, 2),  # 仅超时的无包/序号语义由 stale_drop 专测覆盖。
                ({'valid': False}, .03, True, True, 2),
                ({'predicted': True}, .03, True, True, 2),
                ({'pixel_valid': False}, .03, True, True, 2),
                ({'reference_valid': False}, .03, True, True, 2),
                ({'cm_valid': False}, .03, True, True, 2),
                ({'calibration_valid': False}, .03, True, True, 0)]:
            enc = G3519PositionEncoder()
            frame, valid, reason = enc.encode(dict(base, **change), 2., 2.+age, True, enabled, verified)
            stamp, cm, conf, flags, reserved = struct.unpack('<IfBBH', frame[7:-2])
            self.assertEqual((flags, valid, stamp, reserved), (expected, bool(expected & 1), 2000, 0))
            self.assertEqual(crc_hqx(frame[2:-2], 0xffff), int.from_bytes(frame[-2:], 'little'))
            if valid or reason == 'monitor':
                self.assertAlmostEqual(cm, -.97)
                self.assertEqual(conf, 83)
            else:
                self.assertEqual((cm, conf), (0., 0))
            again, valid, reason = enc.encode(base, 2., 2.05, True, True, True)
            self.assertEqual((reason, valid, int.from_bytes(again[5:7], 'little')), ('duplicate', False, 1))

    def test_split_frames_and_resynchronization(self):
        payload = struct.pack('<I5fHBBf', 99, 0., -.5, 1., .5, -2., 1480, 1, 0, -1.75)
        body = struct.pack('<BBBH',1,0x81,32,65535)+payload
        good = b'\xa5\x5a'+body+struct.pack('<H',crc_hqx(body,0xffff))
        parser = TelemetryParser()
        self.assertEqual(parser.feed(b'ASCII'+good[:-1]+b'\xff'+good[:1]), [])
        result = parser.feed(good[1:11])+parser.feed(good[11:]+good)
        self.assertEqual(len(result),2)
        self.assertEqual(parser.crc_errors,1)
        self.assertEqual(result[0], (65535,99,0.,-.5,1.,.5,-2.,1480,1,0,-1.75))

    def test_pty_bidirectional_journal(self):
        # 使用内核伪串口替身验证收发方向、异步 RX 和 CSV。这里没有真实开发板。
        import csv
        import io
        import os
        import select
        import time
        from contextlib import ExitStack
        from position_feedback import SerialFeedback, LinkJournal
        with ExitStack() as cleanup:
            master, slave = os.openpty()
            cleanup.callback(os.close, master)
            cleanup.callback(os.close, slave)
            port = SerialFeedback(os.ttyname(slave))
            cleanup.callback(port.close)
            stream = io.StringIO()
            journal = LinkJournal(port, stream)
            cleanup.callback(journal.close)
            packet, _, reason = G3519MonitorEncoder().encode({}, 1., 1.01, False)
            port.send(packet)
            journal.sent(packet, 7, reason, False)
            self.assertTrue(select.select([master], [], [], 1.)[0])
            self.assertEqual(os.read(master, len(packet)), packet)
            payload = struct.pack('<I5fHBBf', 1234, 0, 0, 0, 0, .5, 1510, 1, 0, .4)
            body = struct.pack('<BBBH', 1, 0x81, 32, 42) + payload
            os.write(master,b'\xa5\x5a'+body+struct.pack('<H',crc_hqx(body,0xffff)))
            deadline = time.monotonic()+1.
            while not journal.rx_frames and time.monotonic()<deadline:
                time.sleep(.005)
        tx, rx = list(csv.DictReader(io.StringIO(stream.getvalue())))
        self.assertEqual((tx['event'],tx['frame'],tx['rail_x_cm']),('tx','7',''))
        self.assertEqual((rx['event'],rx['telemetry_seq'],rx['g3519_ms'],rx['servo_pwm_us']),
                         ('rx','42','1234','1510'))
        self.assertGreater(float(rx['local_monotonic_ms']),0)
