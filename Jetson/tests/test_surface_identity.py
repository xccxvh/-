# -*- coding: utf-8 -*-
"""只有实测球面的局部纹理重合，才允许解释圆心拟合跳动造成的亮度变化。"""
import unittest

import numpy as np
from ball_detector import BallCandidate
from ball_tracker import BallTracker


def measured(x, patch, brightness):
    return BallCandidate(x, 48, 48, .93, 'topview_circle',
                         dict(coordinate_version='topview-circle-v1', measurement_y_px=60.,
                              in_search_roi=True, identity_valid=True, appearance=brightness,
                              surface_patch=patch))


class SurfaceIdentityTests(unittest.TestCase):
    def setUp(self):
        self.texture = np.random.RandomState(322).randint(0, 256, (70, 100)).astype(np.uint8)
        self.anchor = self.texture[:55, :55].copy()
        self.shift = self.texture[:55, 8:63].copy()

    def acquire(self):
        t = BallTracker(dict(identity_check=True, acquire_frames=2), 1280, 720)
        for ts in (0., .033):
            out = t.update([measured(200, self.anchor, 55)], ts)
        self.assertTrue(out['valid'])
        return t

    def test_fit_center_can_shift_over_same_measured_surface(self):
        t = self.acquire()
        out = t.update([measured(208, self.shift, 115)], .067)
        self.assertTrue(out['valid'])
        self.assertFalse(out['predicted'])
        self.assertEqual(out['x_px'], 208)

    def test_different_target_moving_texture_and_ambiguous_frame_remain_invalid(self):
        for patches in ([measured(207, self.anchor[::-1], 115)],
                        [measured(208, self.anchor, 115)],
                        [measured(208, self.shift, 115), measured(207, self.shift, 115)]):
            t = self.acquire()
            self.assertFalse(t.update(patches, .067)['valid'])
        t = self.acquire()
        self.assertFalse(t.update([], .067)['valid'])
        self.assertFalse(t.update([measured(208, self.shift, 115)], .10)['valid'])
