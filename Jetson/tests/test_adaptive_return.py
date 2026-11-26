import unittest

from adaptive_return import ReturnLaw, NEUTRAL


class AdaptiveReturnTests(unittest.TestCase):
    def test_guard(self):
        with self.assertRaisesRegex(ValueError, 'rail end'):
            ReturnLaw((1280, 1368)).step(0, 5.9, NEUTRAL)

    def test_waits_for_real_output(self):
        law = ReturnLaw((1280, 1368))
        for n in range(75):
            target, _, _, _ = law.step(n*.04, -1., NEUTRAL)
        self.assertEqual(target, NEUTRAL+3)

    def test_stall_at_either_confirmed_limit_aborts(self):
        for x in (-1., 1.):
            law = ReturnLaw((1320, 1328))
            applied = NEUTRAL
            with self.assertRaisesRegex(ValueError, 'without rolling'):
                for n in range(140):
                    target, _, _, _ = law.step(n*.04, x, applied)
                    self.assertLessEqual(abs(target-applied), 3)
                    self.assertTrue(1320 <= target <= 1328)
                    applied = target

    def test_stationary_hold_and_moving_brake(self):
        law = ReturnLaw((1280, 1368))
        for n in range(20):
            target, _, phase, _ = law.step(n*.04, .14, NEUTRAL)
            self.assertEqual((target, phase), (NEUTRAL, 'hold'))
        law = ReturnLaw((1280, 1368))
        applied = NEUTRAL
        for n in range(11):
            applied, speed, phase, _ = law.step(n*.04, -1.05+n*.09, applied)
        self.assertGreater(speed, 1.)
        self.assertEqual(phase, 'brake')
        self.assertLess(applied, NEUTRAL)
