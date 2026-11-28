import copy
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
from velocity_return import AlphaBeta, VelocityPlan


class EstimatorTests(unittest.TestCase):
    def test_rejected_samples_leave_estimate_untouched(self):
        est = AlphaBeta()
        self.assertTrue(est.update(0, 1))
        self.assertTrue(est.update(.04, 1.08))
        state = (est.x, est.v, est.stamp, est.count)
        for t, x in ((.04, 1), (.03, 0), (.08, 5), (.08, float('nan'))):
            self.assertFalse(est.update(t, x))
            self.assertEqual((est.x, est.v, est.stamp, est.count), state)

    def test_confirmed_fast_motion_has_no_velocity_clip(self):
        est = AlphaBeta()
        for i in range(24):
            est.update(i * .02, -4.0 + i * .42)
        self.assertAlmostEqual(est.v, 21., delta=.2)

    def test_constant_velocity_and_bounded_prediction(self):
        est = AlphaBeta()
        for i in range(30):
            self.assertTrue(est.update(i * .04, 1 + i * .08))
        self.assertEqual(est.count, 30)
        self.assertAlmostEqual(est.v, 2, delta=.05)
        self.assertAlmostEqual(est.predict(est.stamp + .05)[0], est.x + .1, delta=.01)
        self.assertAlmostEqual(est.predict(100)[0] - est.x, .60, delta=.01)
        self.assertTrue(est.update(100, -2))
        self.assertEqual((est.x, est.v, est.count), (-2, 0, 1))


class PlanTests(unittest.TestCase):
    def plan(self, **kw):
        return VelocityPlan((1333, 1422), 1378, **kw)

    def test_direction_gain_and_fractional_output(self):
        model = dict(gain_pos=1., gain_neg=2., delay_s=0., brake_accel=6.)
        pos = self.plan(model=model).step(-1, 0, .04, 1378)
        neg = self.plan(model=model).step(1, 0, .04, 1378)
        self.assertAlmostEqual(1378 - pos['target_ticks'], pos['a_cmd'])
        self.assertAlmostEqual(neg['target_ticks'] - 1378, -neg['a_cmd'] / 2)
        self.assertGreater(pos['v_ref'], 0)
        self.assertLess(neg['v_ref'], 0)
        other = self.plan(model=model, polarity=1).step(-1, 0, .04, 1378)
        self.assertAlmostEqual(other['target_ticks'] - 1378, pos['a_cmd'])
        close = self.plan(model=model).step(-1.001, 0, .04, 1378)
        self.assertGreater(pos['target_ticks'] - close['target_ticks'], 0)
        self.assertLess(pos['target_ticks'] - close['target_ticks'], .1)

    def test_stopping_distance_accounts_for_delay(self):
        def sample(delay):
            return self.plan(model=dict(brake_accel=4, delay_s=delay)).step(1, -2, .04, 1378)
        a, b = sample(0), sample(.35)
        self.assertLess(abs(b['v_ref']), abs(a['v_ref']))
        self.assertGreater(b['a_cmd'], 0)
        self.assertEqual(b['phase'], 'brake')

    def test_preview_is_read_only_and_can_brake(self):
        plan = self.plan()
        plan.step(1, 0, .04, 1378)
        state = copy.deepcopy(vars(plan))
        for _ in range(20):
            self.assertLessEqual(plan.step(4, 0, 0, 1380)['target_ticks'], 1380)
            self.assertGreater(plan.step(.2, -3, 0, 1380)['a_cmd'], 0)
            self.assertLess(plan.step(.2, -3, 0, 1380)['target_ticks'], 1378)
            self.assertEqual(plan.step(-4, 0, 0, 1380)['target_ticks'], 1378)
        self.assertEqual(vars(plan), state)

    def test_preview_does_not_reapply_trim_from_neutral(self):
        plan = self.plan()
        applied = 1378
        for _ in range(50):
            row = plan.step(.12, 0, .05, applied)
            applied = round(row['target_ticks'])
        self.assertGreater(row['balance_ticks'], 1378)
        self.assertEqual(plan.step(2, 0, 0, 1378)['target_ticks'], 1378)

    def test_outward_drift_does_not_authorize_new_return_drive_on_preview(self):
        plan = self.plan()
        plan.step(2, 0, .05, 1378)
        self.assertEqual(plan.step(2, .05, 0, 1378)['target_ticks'], 1378)

    def test_hold_integrates_only_with_readback_and_small_velocity(self):
        plan = self.plan()
        for _ in range(20):
            self.assertEqual(plan.step(.12, .9, .05, 1378)['balance_ticks'], 1378)
            self.assertEqual(plan.step(.12, 0, .05, 1422)['balance_ticks'], 1378)
        applied = 1378
        for _ in range(20):
            result = plan.step(.12, 0, .05, applied)
            applied = round(result['target_ticks'])
        self.assertGreater(result['balance_ticks'], 1378)
        balance = result['balance_ticks']
        self.assertEqual(plan.step(.12, 0, 50, 1378)['balance_ticks'], balance)

    def test_breakaway_memory_needs_displacement_and_is_directional(self):
        plan = self.plan()
        applied = 1378
        for _ in range(20):
            rest = plan.step(2, 0, .05, applied)
            applied = round(rest['target_ticks'])
        self.assertGreater(rest['start_bias'], 0)
        for x in (1.96, 1.89, 1.8, 1.7):
            plan.step(x, -.65, .05, applied)
        # 经实际位移确认后，同向再次起动可以复用偏置。
        plan.step(0, 0, .05, 1378)
        self.assertGreater(plan.step(2, 0, .05, 1378)['start_bias'], 0)
        self.assertEqual(plan.step(-2, 0, .05, 1378)['start_bias'], 0)

    def test_gap_preserves_drive_and_excludes_unobserved_displacement(self):
        plan = self.plan()
        applied = 1378
        for _ in range(18):
            row = plan.step(2, 0, .05, applied)
            applied = round(row['target_ticks'])
        bias = row['start_bias']
        self.assertGreater(bias, 0)
        self.assertEqual(plan.step(1.7, 0, .20, applied)['start_bias'], bias)
        plan.step(1.7, -.7, .04, applied)
        plan.step(0, 0, .05, 1378)
        self.assertEqual(plan.step(2, 0, .05, 1378)['start_bias'], 0)

    def test_one_speed_spike_and_brake_echo_preserve_start_effort(self):
        plan = self.plan()
        applied = 1378
        for _ in range(18):
            row = plan.step(2, 0, .05, applied)
            applied = round(row['target_ticks'])
        bias = row['start_bias']
        self.assertGreater(bias, 0)
        spike = plan.step(2, -5, .04, applied)
        self.assertEqual(spike['phase'], 'brake')
        recovered = plan.step(2, 0, .04, round(spike['target_ticks']))
        self.assertEqual(recovered['start_bias'], bias)

    def test_extrapolated_displacement_cannot_confirm_breakaway(self):
        plan = self.plan()
        applied = 1378
        for i in range(30):
            x = 2 - max(0, i - 22) * .03
            v = -.65 if i > 22 else 0
            row = plan.step(x, v, .05, applied, observed_x=2.)
            applied = round(row['target_ticks'])
        plan.step(0, 0, .05, 1378)
        self.assertEqual(plan.step(2, 0, .05, 1378)['start_bias'], 0)

    def test_hold_trim_pauses_while_braking(self):
        plan = self.plan()
        self.assertEqual(plan.step(.04, -.31, .05, 1378)['balance_ticks'], 1378)

    def test_no_memory_from_velocity_or_from_missing_interval(self):
        plan = self.plan()
        for _ in range(8):
            plan.step(2, -.7, .05, 1390)
        plan.step(1.5, -.7, 20., 1390)
        plan.step(0, 0, .05, 1378)
        self.assertEqual(plan.step(2, 0, .05, 1378)['start_bias'], 0)

    def test_saturation_requires_applied_limit_and_observed_nonmotion(self):
        plan = VelocityPlan((1376, 1380), 1378)
        for _ in range(50):
            self.assertEqual(plan.step(4, 0, .05, 1378)['target_ticks'], 1380)
        for i in range(25):
            plan.step(4 - i * .03, -.6, .05, 1380)
        for _ in range(10):
            plan.step(3, 0, 2, 1380)
        with self.assertRaisesRegex(ValueError, 'stroke limit'):
            for _ in range(50):
                plan.step(3, 0, .05, 1380)


if __name__ == '__main__':
    unittest.main()
