"""Deterministic controller regressions; no mechanical performance claims."""
import struct
import unittest
from adaptive_return import Motion, ReturnLaw
from host_experiment import BoardController
from servo_scan import make_plan


def law(bounds=(1170, 1470), neutral=1320, probe=None):
    return ReturnLaw(bounds, neutral, 180., polarity=1, probe=probe)


class AdaptiveReturnTests(unittest.TestCase):
    def test_new_firmware_window_propagates_through_encoding(self):
        packets = []
        board = BoardController(None, packets.append, (1250,1450))
        for target in make_plan('ticks', 1400, 1410, 10, 500, limits=(1250,1450)):
            board.set_pwm_ticks(target.ticks)
        self.assertEqual([struct.unpack_from('<H', p, 7)[0] for p in packets], [1400,1410,1400])
        with self.assertRaises(ValueError):
            board.set_pwm_ticks(1451)

    def test_speed_fit_allows_moving_start_but_rejects_jump_and_gap(self):
        m = Motion()
        for i in range(6):
            v = m.add(i*.032, .5+i*.032*3.)
        self.assertAlmostEqual(v, 3.)
        self.assertIsNone(m.add(.192, 2.5))
        self.assertIsNone(m.add(.45, 2.6))

    def test_distance_dependent_push_and_async_output(self):
        targets = []
        for x in (-.4,-3.):
            c = law()
            c.step(0., x, 0.)
            target,_,_ = c.step(.4, x, 0.)
            self.assertEqual(c.step(.9, x, 0.)[0],target)
            targets.append(target)
        self.assertGreater(targets[1],targets[0])
        self.assertGreater(targets[1],1323)

    def test_brakes_before_actuator_reaches_previous_goal(self):
        c = law()
        c.step(0., -3., 0.)
        c.step(.4, -3., 0.)
        target,phase,_ = c.step(.45, -.35, 4.)
        self.assertEqual(phase,'brake')
        self.assertLess(target,1320-12)
        self.assertEqual(c.step(.8,.05,0.)[:2],(1320,'hold'))

    def test_deadline_scales_with_travel(self):
        c = law()
        c.written(0.,1420,1320)
        c.check_actuator(.6)
        with self.assertRaises(TimeoutError):
            c.check_actuator(1.1)
        c.feedback(1.1,1420,1420)
        c.check_actuator(2.)

    def test_saturated_stall_and_end_guard(self):
        for x in (1.,-1.):
            c=law((1310,1330))
            with self.assertRaisesRegex(ValueError,'without useful motion'):
                for i in range(100):
                    target,_,_=c.step(i*.05,x,0.)
                    c.feedback(i*.05,target,target)
        with self.assertRaisesRegex(ValueError,'rail end'):
            law().step(0.,5.8,0.)

    def test_probe_rejects_constant_velocity(self):
        c=law(probe=-1)
        c.step(0.,0.,1.)
        c.feedback(.3,1310,1310)
        self.assertNotEqual(c.step(.4,.4,1.)[1],'probe_motion')
        self.assertEqual(c.step(.5,.25,-.3)[:2],(1320,'probe_motion'))


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
        for r in events:
            if r['event']=='tx' and .56 <= float(r['nano_s'])-f.start < gap_end:
                self.assertEqual(int(r['target_ticks']),1320)
        return fault

    def test_brief_dropout_resumes_on_fresh_measurements(self):
        self.assertIsNone(self.trial(.60))

    def test_sustained_dropout_stops(self):
        self.assertIn('220 ms',self.trial(.90))
