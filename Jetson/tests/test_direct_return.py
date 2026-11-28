"""直接 PWM 离板回归。所有位置都以零点为原点，时间为曝光时间。"""
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from direct_return import DirectReturn


class DirectTests(unittest.TestCase):
    def setUp(self):
        self.law = DirectReturn((1236, 1413), 1324, -1)

    def test_soft_start_both_directions_and_echo_dwell(self):
        for x in (-2, 2):
            law = DirectReturn((1236, 1413), 1324, -1)
            first = law.step(x, 0, 0, x, 1324, 1324)
            goal = first['target_ticks']
            self.assertEqual(first['phase'], 'breakaway')
            self.assertEqual(goal, 1324 + (10 if x > 0 else -10))
            for i in range(1, 18):
                self.assertEqual(law.step(x, 0, i*.05, x, 1324, 1324)['target_ticks'], goal)
            for i in range(18, 28):
                row = law.step(x, 0, i*.05, x, goal, goal)
            self.assertGreater(abs(row['target_ticks']-1324), 10)
            law.reset_observation()
            after = law.step(x, 0, 2., x, goal, goal)
            self.assertEqual(after['target_ticks'], row['target_ticks'])

    def test_real_displacement_and_applied_output_required_for_disk_learning(self):
        with tempfile.TemporaryDirectory() as temp:
            p = Path(temp)/'experience.json'
            law = DirectReturn((1236,1413),1324,-1,p)
            for i in range(6):
                law.step(-2+i*.06, .7, .05*i, -2, 1314, None)
            self.assertFalse(p.exists())
            law.reset_observation()
            law.step(-2, 0, 1, -2, 1314, 1314)
            for i in range(1, 6):
                row = law.step(-2+i*.05, .7, 1+.05*i, -2+i*.05, 1314, 1314)
            self.assertEqual(row['phase'], 'move')
            self.assertFalse(p.exists())
            law.save()
            history=json.loads(p.read_text())
            self.assertIn('1314', json.dumps(history))
            again=DirectReturn((1236,1413),1324,-1,p)
            self.assertEqual(again.experience, law.experience)
            other = DirectReturn((1200,1400),1324,-1,p)
            self.assertEqual(other.experience['regions'], {})
            self.assertNotEqual(other.path, p)
            self.assertEqual(json.loads(p.read_text()), history)

    def test_brake_keeps_direction_until_stopped_then_saves_observed_distance(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'experience.json'
            law=DirectReturn((1236,1413),1324,-1,p)
            x=-.28
            row=law.step(x,1.1,0,x,1314,1314)
            self.assertEqual(row['phase'],'brake')
            self.assertGreater(row['target_ticks'],1324)
            for i,(x,v) in enumerate([(-.22,1.),(-.16,.85),(-.12,.65),(-.095,.4),
                                       (-.08,.20),(-.078,.04),(-.077,.02),(-.076,0)],1):
                row=law.step(x,v,.06*i,x,1334,1334)
            self.assertEqual(row['phase'],'hold')
            self.assertFalse(p.exists())
            law.save()
            self.assertTrue(p.exists())
            self.assertIn('stop_distance_cm',p.read_text())

    def test_band_hold_and_gap(self):
        for i in range(102):
            row=self.law.step(0,0,i*.1,0,1324,1324)
        self.assertTrue(row['hold_complete'])
        self.law.reset_observation()
        row=self.law.step(0,0,10.3,0,1324,1324)
        self.assertEqual(row['hold_s'],0)
        self.assertFalse(row['hold_complete'])
        self.assertNotEqual(self.law.step(0,0,10.4,.5,1324,1324)['phase'],'hold')

    def test_endpoint_requires_fresh_applied_pwm_and_no_motion(self):
        x=-2
        for i in range(400):
            self.law.step(x,0,i*.05,x,1236,None)
        law=DirectReturn((1308,1340),1324,-1)
        tick=1324
        with self.assertRaisesRegex(ValueError,'endpoint'):
            for i in range(120):
                tick=law.step(x,0,i*.05,x,tick,tick)['target_ticks']
        self.assertEqual(tick,1308)

    def test_move_inherits_high_breakaway_then_increases_on_slow_travel(self):
        law = self.law
        pwm = 1324
        for i in range(40):
            pwm = law.step(-3,0,i*.05,-3,pwm,pwm)['target_ticks']
        applied = pwm
        self.assertLess(applied, 1300)
        for i in range(1,6):
            x = -3+i*.045
            row = law.step(x,.65,2+i*.05,x,pwm,pwm)
            pwm = row['target_ticks']
            if row['phase'] == 'move':
                break
        self.assertEqual(row['phase'],'move')
        self.assertEqual(pwm,applied)
        for j in range(1,16):
            slow = x+j*.01
            row = law.step(slow,.2,2+(i+j)*.05,slow,pwm,pwm)
            pwm = row['target_ticks']
            self.assertEqual(row['phase'],'move')
        self.assertLess(pwm,applied)

    def test_far_fast_motion_is_not_forced_to_brake(self):
        row = self.law.step(-4.,2.1,0,-4.,1302,1302)
        self.assertEqual(row['phase'], 'move')

    def test_hold_pwm_pd_and_full_bias_range(self):
        law = self.law
        law.balance = 1400.
        law.target = 1400
        row = law.step(.2,.15,0,.2,1400,1400)
        self.assertGreater(row['target_ticks'],1400)
        pwm = row['target_ticks']
        for i in range(1,41):
            row=law.step(.2,0,i*.05,.2,pwm,pwm)
            pwm=row['target_ticks']
        self.assertGreater(row['balance_ticks'],1400)
        other = DirectReturn((1236,1413),1324,-1)
        self.assertLess(other.step(-.2,-.15,0,-.2,1324,1324)['target_ticks'],1324)

    def test_start_travel_on_two_echoed_levels_is_not_attributed_to_last_level(self):
        law = self.law
        law.step(-2.,0.,0.,-2.,1314,1314)
        law.step(-1.93,.2,.08,-1.93,1314,1314)
        row = law.step(-1.86,.7,.16,-1.86,1306,1306)
        self.assertEqual(row['phase'],'move')
        self.assertEqual(row['target_ticks'],1306)
        self.assertEqual(law.experience['regions'],{})

    def test_move_uses_applied_effort_and_requires_platform_feedback(self):
        def travel(applied, speed, echo):
            law = DirectReturn((1236,1413),1324,-1)
            row = law.step(-4,.8,0,-4,applied,applied)
            for i in range(1,10):
                x = -4+i*.04*speed
                row = law.step(x,speed,.04*i,x,applied,echo)
            return row['target_ticks']
        self.assertEqual(travel(1290,.8,1290),1290)
        self.assertEqual(travel(1290,.15,None),1290)
        self.assertGreater(1290-travel(1290,.15,1290),1314-travel(1314,.15,1314))

    def test_duplicate_stamp_cannot_learn(self):
        self.law.step(-1,0,1.,-1,1324,1324)
        with self.assertRaises(ValueError):
            self.law.step(-1,0,1.,-.8,1314,1314)


if __name__ == '__main__':
    unittest.main()
