import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ball_detector import BallCandidate
from ball_tracker import BallTracker
from candidate_fusion import associate, merge


def observed(x, y, source):
    return BallCandidate(x, 48, 48, .9, 'topview_circle', dict(
        measurement_y_px=y, coordinate_version='topview-circle-v1',
        in_search_roi=True, identity_valid=True, appearance=50, proposal_source=source))


class CandidateFusionTests(unittest.TestCase):
    def test_two_response_lobes_do_not_vote_twice(self):
        contour = observed(400, 530, 'contour')
        same = observed(404, 532, 'motion')
        other = observed(460, 530, 'motion')
        self.assertEqual(merge([contour], [same, other]), [contour, other])

    def test_prediction_selects_only_a_measured_unambiguous_center(self):
        tracker = BallTracker({}, 1280, 720)
        left, right = observed(400, 530, 'contour'), observed(425, 530, 'motion')
        options = [left, right]
        self.assertEqual(associate(options, tracker, 1.), options)
        tracker.x, tracker.v, tracker.last_ts = 404., 400., 1.
        self.assertEqual(associate(options, tracker, 1.05), [right])
        self.assertEqual(associate(options, tracker, 1.2), options)
        self.assertEqual(associate([], tracker, 1.05), [])
        tracker.x, tracker.v = 412.5, 0.
        self.assertEqual(associate(options, tracker, 1.05), options)
