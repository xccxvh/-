import importlib.util
import unittest

class RouteTest(unittest.TestCase):
    def test_targets_follow_observed_positions(self):
        self.assertIsNotNone(importlib.util.find_spec('timed_route'), 'timed route unavailable')
        from timed_route import TimedRoute
        route=TimedRoute(1236,1324,1413)
        self.assertEqual(route.goal,5.)
        self.assertLess(route.step(0.,0.,0.,0.)['target_ticks'],1324)
        route.step(4.8,2.,.1,3.8)
        self.assertEqual(route.goal,5.)
        self.assertGreater(route.step(4.6,2.,.2,4.3)['target_ticks'],1324)
        self.assertEqual(route.goal,-5.)
        route.step(-4.7,0.,2.,-4.7)
        route.interrupt_hold()
        self.assertEqual(route.step(-4.7,0.,3.,-4.7)['hold_s'],0.)

    def test_applied_endpoint_with_no_motion_stops(self):
        from timed_route import TimedRoute
        route = TimedRoute(1236,1324,1413)
        self.assertTrue(hasattr(route,'check_endpoint'), 'reuse the no-motion protection at a confirmed endpoint')
        route.check_endpoint(1236,1236,1236,0.,0.,0.)
        with self.assertRaisesRegex(ValueError,'endpoint'):
            route.check_endpoint(1236,1236,1236,.4,.01,.01)
