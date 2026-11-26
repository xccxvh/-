import csv
import io
import os
import pty
import select
import struct
import threading
import time
import unittest

from experiment_port import ExperimentPort
from position_feedback import SerialFeedback, LinkJournal, crc16


def frame(kind, seq, payload):
    body = struct.pack('<BBBH', 1, kind, len(payload), seq) + payload
    return b'\xa5\x5a' + body + struct.pack('<H', crc16(body))


class FakeCodec:
    """Test-only ASCII tokens deliberately unrelated to eventual board command IDs."""
    @staticmethod
    def encode(name, args):
        return name.encode('ascii')

    @staticmethod
    def ack(seq, payload):
        return seq, payload[1:].decode('ascii'), payload[:1] == b'Y', 'test_reply'


class PortTest(unittest.TestCase):
    def test_shared_pty_and_distinct_acks(self):
        master, slave = pty.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        uart = SerialFeedback(os.ttyname(slave))
        self.addCleanup(uart.close)
        port = ExperimentPort(uart, FakeCodec())
        port.next_command = 0
        journal = LinkJournal(port, io.StringIO(), split_rx=port.split_rx)
        self.addCleanup(journal.close)
        failures = []

        def request():
            try:
                port.execute('STOP')
            except Exception as exc:
                failures.append(str(exc))

        worker = threading.Thread(target=request)
        worker.start()
        self.assertTrue(select.select([master], [], [], .4)[0])
        self.assertEqual(os.read(master, 100), frame(3, 0, b'STOP'))
        payload = struct.pack('<I5fHBBf', 123, 0, -2.66, 0, 2.66, .2, 1725, 2, 3, .15)
        remote = struct.pack('<HIHHBB', 24, 128, 1378, 1379, 1, 0)
        bad = bytearray(frame(131, 0, b'YSTOP'))
        bad[-1] ^= 1
        data = frame(131, 3, b'YSTOP') + frame(131, 0, b'YSTART_AUTO') + bad
        data += frame(129, 4, payload) + frame(130, 5, remote)
        os.write(master, data[:3])
        os.write(master, data[3:])
        status = port.next_status(time.monotonic()+.4)
        self.assertEqual((status['control_state'],status['board_vision_valid'],status['g3519_ms']), (2,1,123))
        self.assertEqual(port.next_debug(time.monotonic()+.4)['output_pwm_ticks'],1379)
        self.assertTrue(worker.is_alive())
        os.write(master,frame(131,0,b'NSTOP'))
        worker.join(.5)
        self.assertFalse(worker.is_alive())
        self.assertIn('STOP refused: test_reply',failures)
        self.assertEqual(port.crc_errors,1)
