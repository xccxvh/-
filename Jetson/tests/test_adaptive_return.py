"""Deterministic controller regressions; no mechanical performance claims."""
import struct
import unittest
from host_experiment import BoardController
from servo_scan import make_plan


class AdaptiveReturnTests(unittest.TestCase):
    def test_new_firmware_window_propagates_through_encoding(self):
        packets = []
        board = BoardController(None, packets.append, (1250,1450))
        for target in make_plan('ticks', 1400, 1410, 10, 500, limits=(1250,1450)):
            board.set_pwm_ticks(target.ticks)
        self.assertEqual([struct.unpack_from('<H', p, 7)[0] for p in packets], [1400,1410,1400])
        with self.assertRaises(ValueError):
            board.set_pwm_ticks(1451)

class VisionGapTests(unittest.TestCase):
    def trial(self, gap_end):
        import csv
        import io
        import queue
        from types import SimpleNamespace
        from unittest.mock import patch
        from adaptive_return import run

        class Fixture:
            def __init__(self):
                self.t = 10.
                self.tx = []
                self.seq = 0
                self.enabled = False
                self.confirmed_ticks = (1180,1470)
                self.debug = queue.Queue()
                self.next_rx = self.t
                self.ticks = 1320
                self.stopped = False

            @property
            def measurement(self):
                n = int(self.t/.03)
                t = n*.03
                invalid = self.enabled and .52 <= self.t-self.start < gap_end
                return n,t,None if invalid else -2.,'vision' if invalid else 'ok',t

            def sleep(self, sec):
                self.t += sec
                if self.enabled and self.t >= self.next_rx:
                    self.debug.put(self.rx())
                    self.next_rx=self.t+.09

            def rx(self):
                return dict(received=self.t, remote_enabled=self.enabled,
                            result=0, target_pwm_ticks=self.ticks, output_pwm_ticks=self.ticks,
                            command_seq=(self.seq-1)&65535, board_timestamp=int(1000*self.t))

            def next_debug(self, end):
                self.t+=.003
                return self.rx()

            def set_mode(self, mode):
                assert mode == 'REMOTE'

            def start_remote(self):
                self.enabled=True
                self.start=self.t

            def stop(self):
                self.enabled=False
                self.stopped=True
                self.ticks=1320

            def set_pwm_ticks(self,ticks):
                self.ticks=ticks
                self.tx.append((self.t,ticks))
                self.seq+=1
                return self.seq-1

        f=Fixture()
        csv_log=io.StringIO()
        fault=None
        with patch('adaptive_return.time',SimpleNamespace(monotonic=lambda:f.t,sleep=f.sleep)):
            try:
                run(f,f,csv_log,1.,SimpleNamespace(is_set=lambda:False),
                    dict(neutral_ticks=1320,slew_ticks_per_s=180,firmware='fixture'))
            except TimeoutError as exc:
                fault=str(exc)
        self.assertTrue(f.stopped)
        events=list(csv.DictReader(io.StringIO(csv_log.getvalue())))
        self.assertEqual(events[-1]['event'],'stop_confirmed')
        self.assertTrue(any(r['phase']=='pause' for r in events))
        # 短时缺测保留实际已发送的指令；仅视觉事件可以包含实测厘米数。
        tx = [r for r in events if r['event'] == 'tx']
        self.assertGreater(len(tx), 30)
        self.assertTrue(all(r['x_cm'] == '' for r in tx))
        froze = [int(r['target_ticks']) for r in tx if .55 <= float(r['nano_s'])-f.start < .60]
        self.assertTrue(froze and len(set(froze)) == 1 and froze[0] != 1320)
        return fault

    def test_brief_dropout_resumes_on_fresh_measurements(self):
        self.assertIsNone(self.trial(.60))

    def test_sustained_dropout_stops(self):
        self.assertIn('220 ms',self.trial(.90))
