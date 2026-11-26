import csv
import io
import unittest
from collections import namedtuple

from host_experiment import BoardController, run_auto, run_remote
from servo_scan import command_packet
from analyze_servo_response import analyze


class FakeBoard:
    """Symbolic acknowledged actions and independent 10 Hz simulated board telemetry."""
    def __init__(self):
        self.now = 0.
        self.calls = []
        self.auto = False
        self.vision = True
        self.reject = None
        self.drop_at = None
        self.enable_on_start = True
        self.wire = []
        self.enabled = False
        self.ticks = 1378
        self.ack = 0
        self.next_report = .1
        self.target = 0
        self.host = BoardController(self.execute, self.send, (1378,1379))

    def execute(self, name, *args):
        self.calls.append(name)
        if self.reject == name:
            raise RuntimeError('rejected: '+name)
        if name == 'SET_TARGET':
            self.target = args[0]
        if name == 'START_AUTO':
            self.auto = self.enable_on_start
        if name == 'START_REMOTE':
            self.enabled = True
        if name == 'STOP':
            self.auto = self.enabled = False
            self.ticks = 1378

    def send(self, packet):
        self.wire.append(packet)
        self.ack = int.from_bytes(packet[5:7],'little')
        self.ticks = int.from_bytes(packet[7:9],'little')

    def tick(self, end):
        if self.next_report > end:
            self.now = end
            raise TimeoutError('no telemetry before deadline')
        self.now = self.next_report
        self.next_report += .1

    def debug(self,end):
        self.tick(end)
        return dict(received=self.now, command_seq=self.ack, board_timestamp=int(self.now*1000),
                    target_pwm_ticks=self.ticks, output_pwm_ticks=self.ticks,
                    remote_enabled=int(self.enabled), result=0 if self.enabled else 1)

    def status(self,end):
        self.tick(end)
        if self.drop_at and self.now >= self.drop_at:
            self.auto=False
        return dict(received=self.now, g3519_ms=int(self.now*1000),
                    control_state=2 if self.auto else 0, board_auto_enabled=int(self.auto),
                    board_fault=0,board_vision_valid=int(self.vision),
                    target_cm=self.target,board_ball_cm=-2.66)

    def run(self,log):
        run_auto(self.host,self.status,log,0,.6,.3,.4,lambda:self.now)


class Experiments(unittest.TestCase):
    def setUp(self):
        self.b=FakeBoard()
        self.log=io.StringIO()

    def test_confirmed_auto_and_stop(self):
        self.b.run(self.log)
        self.assertEqual(self.b.calls,['SET_MODE','SET_TARGET','SET_PD','START_AUTO','STOP'])
        self.assertIn('9.34',self.log.getvalue())
        self.assertIn('stop_confirmed',self.log.getvalue())

    def test_no_vision(self):
        self.b.vision=False
        with self.assertRaises(TimeoutError):
            self.b.run(self.log)
        self.assertNotIn('START_AUTO',self.b.calls)
        self.assertLess(self.b.now,2.)

    def test_rejected_start(self):
        self.b.reject='START_AUTO'
        with self.assertRaisesRegex(RuntimeError,'rejected'):
            self.b.run(self.log)
        self.assertEqual(self.b.calls.count('START_AUTO'),1)
        self.assertEqual(self.b.calls[-1],'STOP')

    def test_ack_without_auto(self):
        self.b.enable_on_start=False
        with self.assertRaises(TimeoutError):
            self.b.run(self.log)
        self.assertNotIn('auto_confirmed',self.log.getvalue())

    def test_exit_without_retry(self):
        self.b.drop_at=.4
        with self.assertRaisesRegex(RuntimeError,'AUTO exited'):
            self.b.run(self.log)
        self.assertEqual(self.b.calls.count('START_AUTO'),1)
        self.assertIn('abort',self.log.getvalue())

    def test_remote_step(self):
        target=namedtuple('Target','ticks')
        run_remote(self.b.host,self.b.debug,self.log,[target(i) for i in (1378,1379,1378)],
                   500,1378,lambda:self.b.now)
        self.assertEqual(self.b.calls,['SET_MODE','START_REMOTE','STOP'])
        self.assertIn('stop_confirmed',self.log.getvalue())
        self.assertTrue(any(p[7:9]==b'\x63\x05' for p in self.b.wire))

    def test_confirmed_window_and_encoding(self):
        self.b.host.set_servo_angle(110.)
        self.assertEqual(self.b.wire,[command_packet(0,1378)])
        with self.assertRaises(ValueError):
            self.b.host.set_pwm_ticks(1380)
        self.assertEqual(len(self.b.wire),1)

    def test_measured_acceleration(self):
        rows=[dict(event='tx',plateau_index=0,target_pwm_ticks=1378,nano_timestamp=0,cm_valid='0')]
        for i in range(11):
            t=i/20.
            rows.append(dict(event='remote',remote_enabled='1',output_pwm_ticks=1378,
                             cm_valid='1',vision_read_ms=t*1000,nano_timestamp=t,
                             ball_x_cm=2*t*t+.1*t))
        self.assertEqual(analyze(rows,.1,self.log),[(0,1378,0,1)])
        self.assertAlmostEqual(float(list(csv.reader(io.StringIO(self.log.getvalue())))[1][5]),4.)
