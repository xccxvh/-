# -*- coding: utf-8 -*-
"""边界重获需要出向和入向的同帧实测。空帧、冷启动及冲突身份不可加速。"""
import unittest

from ball_detector import BallCandidate
from ball_tracker import BallTracker


class EdgeReturnTests(unittest.TestCase):
    def setUp(self):
        self.tracker = BallTracker(dict(identity_check=True, acquire_frames=5), 1280, 720)
        self.bounds = (250., 1050.)
        self.right = False
        self.t = 0.

    def observe(self, distance=None, count=1, confidence=.90, appearance=75., y=560., source=None):
        # 镜像同一段运动到左右边界，距离始终按向内为正。空帧同样推进采集时钟。
        self.t += .033
        candidates = []
        if distance is not None:
            x = self.bounds[1]-distance if self.right else self.bounds[0]+distance
            candidates = [BallCandidate(x+i, 50, 50, confidence, 'topview_circle',
                          dict(identity_valid=True, in_search_roi=True, appearance=appearance,
                               coordinate_version='topview-circle-v1', measurement_y_px=y, proposal_source=source))
                          for i in range(count)]
        return self.tracker.update(candidates, self.t, search_bounds=self.bounds)

    def depart(self, confirmed=True):
        if confirmed:
            for distance in [125, 115, 105, 95, 85]:
                out = self.observe(distance)
            self.assertTrue(out['valid'])
        for distance in [60, 40, 12]:
            self.observe(distance)
        for _ in range(12):
            self.assertFalse(self.observe()['valid'])

    def test_both_edges_recover_only_on_third_strong_inward_circle(self):
        for right in (False, True):
            self.setUp()
            self.right = right
            self.depart()
            self.assertFalse(self.observe(16)['valid'])
            self.assertFalse(self.observe()['valid'])
            self.assertFalse(self.observe(55)['valid'])
            out = self.observe(67)
            self.assertEqual((out['valid'], out['predicted'], out['reason']),
                             (True, False, 'edge_return'))
            self.assertEqual(out['x_px'], 983 if right else 317)

    def test_cold_or_expired_or_reset_cannot_accelerate(self):
        for case in ('cold', 'expired', 'reset', 'clock', 'reference', 'shift_on_empty'):
            self.setUp()
            self.depart(confirmed=case != 'cold')
            if case == 'expired':
                for _ in range(52):
                    self.observe()
            elif case == 'reset':
                self.tracker.reset()
            elif case == 'clock':
                self.tracker.update([], self.t-.01, search_bounds=self.bounds)
            elif case == 'shift_on_empty':
                self.bounds = (200., 1000.)
                self.observe()
                self.bounds = (250., 1050.)
            elif case == 'reference':
                self.t += .033
                self.tracker.update([], self.t, search_bounds=None)
            for distance in (16, 42, 55, 66):
                self.assertFalse(self.observe(distance)['valid'], case)
            self.assertTrue(self.observe(72)['valid'])

    def test_ambiguous_weak_wrong_identity_stationary_or_reverse_return(self):
        for case in ('ambiguous', 'weak', 'contrast', 'height', 'stationary', 'reverse',
                     'opposite', 'missing_twice', 'moving_reference'):
            self.setUp()
            self.depart()
            self.observe(16)
            if case == 'opposite':
                self.right = True
            if case == 'moving_reference':
                self.bounds = (180., 980.)
            if case == 'missing_twice':
                self.observe()
                self.observe()
            self.observe(16 if case == 'stationary' else 10 if case == 'reverse' else 45,
                         count=2 if case == 'ambiguous' else 1,
                         confidence=.80 if case == 'weak' else .90,
                         appearance=110 if case == 'contrast' else 75.,
                         y=610 if case == 'height' else 560.)
            self.assertFalse(self.observe(60)['valid'], case)

    def test_only_verified_inboard_motion_can_preserve_prior_hough_anchor(self):
        # 两个补搜圆均不能建立出向证据。其后正常关联丢失，须靠旧强锚点及新圆缘穿界。
        for source in ('motion_blur', None):
            self.setUp()
            for d in [295, 280, 265, 250, 235]:
                self.observe(d)
            for d in [220, 205]:
                self.observe(d, confidence=.825, source=source)
            for _ in range(5):
                self.observe()
            for d in [60, 40, 12]:
                self.observe(d)
            for _ in range(12):
                self.observe()
            self.observe(16)
            self.observe(42)
            self.assertEqual(self.observe(55)['valid'], source == 'motion_blur')

    def test_blur_even_strong_cannot_confirm_or_preserve_armed_exit(self):
        for position in [16, 140]:
            self.setUp()
            self.depart()
            self.observe(position, source='motion_blur')
            for d in [20, 40, 60]:
                self.assertFalse(self.observe(d)['valid'])
