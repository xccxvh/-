import unittest
import cv2
import numpy as np
from ball_detector import BallDetector, BallCandidate
from ball_tracker import BallTracker
from rail_detector import RailLine


def cand(x, confidence=.85):
    return BallCandidate(x, 42, 13, confidence, 'geom')


class GeometryBoundaryTests(unittest.TestCase):
    def candidates(self, clipped):
        im = np.zeros((720,1280,3), np.uint8)
        im[330:345] = 220
        if clipped:
            # A bright distant object continues above the search band. It
            # resembles a 50x36 cap only because the ROI cuts it off.
            im[240:330,500:550] = 230
        else:
            cv2.ellipse(im, (525,330), (24,13), 0,180,360,(230,230,230),-1)
        d=BallDetector({'max_height_ratio':.06},1280,720)
        return d.detect(im,cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),RailLine(0,330,True))
    def test_clipped_background_is_not_ball(self):
        self.assertEqual(self.candidates(True), [])
    def test_visible_ball_keeps_geometric_center(self):
        cs=self.candidates(False)
        self.assertEqual(len(cs),1)
        self.assertAlmostEqual(cs[0].x,525,delta=1)


class TimedTrackingTests(unittest.TestCase):
    def tracker(self, **kw):
        cfg={'acquire_frames':1,'velocity_alpha':1.0}
        cfg.update(kw)
        return BallTracker(cfg,1280,720)
    def test_nearby_ball_beats_brighter_distractor(self):
        t=self.tracker();t.update([cand(500)],0)
        out=t.update([cand(505,.75),cand(550,.98)],.033)
        self.assertEqual(out['x_px'],505)
    def test_motion_uses_elapsed_time_after_dropped_frames(self):
        t=self.tracker();t.update([cand(400)],0);t.update([cand(420)],.02)
        out=t.update([cand(520,.8),cand(440,.98)],.12)
        self.assertEqual(out['x_px'],520)
    def test_nonmonotonic_measurement_invalid(self):
        t=self.tracker();t.update([cand(400)],1)
        self.assertFalse(t.update([cand(400)],1)['valid'])
    def test_nonfinite_time_invalid(self):
        t=self.tracker()
        self.assertFalse(t.update([cand(400)],float('nan'))['valid'])
    def test_long_gap_requires_new_acquisition(self):
        t=self.tracker(acquire_frames=3)
        for ts in [0,.03,.06]:t.update([cand(400)],ts)
        self.assertFalse(t.update([cand(400)],1)['valid'])
    def test_ambiguous_initial_candidates_do_not_acquire(self):
        t=self.tracker()
        self.assertFalse(t.update([cand(400,.9),cand(600,.89)],0)['valid'])
    def test_ambiguous_near_prediction_invalid(self):
        t=self.tracker();t.update([cand(500)],0)
        self.assertFalse(t.update([cand(494,.9),cand(506,.91)],.033)['valid'])

class AcquisitionVelocityTests(unittest.TestCase):
    def test_moving_acquisition_keeps_velocity(self):
        t=BallTracker({'acquire_frames':5,'velocity_alpha':.5},1280,720)
        for i,x in enumerate([400,420,440,460,480]):
            t.update([cand(x)],i*.02)
        out=t.update([cand(500,.85),cand(480,.75)],.10)
        self.assertEqual(out['x_px'],500)
    def test_filter_is_consistent_when_frames_are_dropped(self):
        trackers=[]
        for times in [[.10,.12,.14,.16],[.16]]:
            t=BallTracker({'acquire_frames':5,'velocity_alpha':.5},1280,720)
            for ts in [0,.02,.04,.06,.08]:t.update([cand(400)],ts)
            for ts in times:t.update([cand(400+1000*(ts-.08))],ts)
            trackers.append(t)
        self.assertAlmostEqual(trackers[0].v,trackers[1].v,places=6)
        results=[t.update([cand(520),cand(500)],.20) for t in trackers]
        self.assertEqual(results[0]['valid'],results[1]['valid'])
        self.assertAlmostEqual(results[0]['x_px'],results[1]['x_px'])
        # An ambiguous prediction may be rejected; it must never become a
        # valid measurement of the distractor merely because frames dropped.
        for out in results:
            if out['valid']:
                self.assertEqual(out['x_px'],520)
