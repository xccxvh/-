import unittest
from types import SimpleNamespace

import cv2
import numpy as np

from ball_detector import BallCandidate
from ball_tracker import BallTracker
from measurement_contract import calibrated_method
from local_vision import topview_balls


class TopviewBallTests(unittest.TestCase):
    def setUp(self):
        self.gray = np.full((140, 320), 195, np.uint8)
        self.rail = SimpleNamespace(edge_profile=np.full(320, 100.),
                                    edge_support=np.ones(320, dtype=bool))

    def detect(self, bounds=(60., 260.)):
        return topview_balls(self.gray, self.rail, bounds, BallCandidate, [])[0]

    def test_round_metal_ball_beside_dark_tool(self):
        cv2.rectangle(self.gray, (45, 56), (144, 93), 40, -1)
        self.assertFalse(self.detect())
        cv2.circle(self.gray, (165, 74), 23, 75, -1)
        cv2.circle(self.gray, (158, 68), 7, 230, -1)
        balls = self.detect()
        self.assertEqual(len(balls), 1)
        self.assertAlmostEqual(balls[0].x, 165, delta=3)
        self.assertEqual(balls[0].extra['coordinate_version'], 'topview-circle-v1')
        self.assertFalse(self.detect((60., 154.)))

    def test_circle_center_cannot_claim_cap_identity_or_old_cm(self):
        tracker = BallTracker({'identity_check': True}, 320, 140)
        candidate = BallCandidate(165, 46, 46, .9, 'topview_circle',
                                  dict(coordinate_version='relative-cap-v1', measurement_y_px=74.,
                                       in_search_roi=True, identity_valid=True, appearance=80.))
        self.assertFalse(tracker._identity_ok(candidate))
        self.assertFalse(calibrated_method(candidate.method))
        candidate.extra['coordinate_version'] = 'topview-circle-v1'
        self.assertTrue(tracker._identity_ok(candidate))
