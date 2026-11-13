import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from calibration import Calibration
from rail_detector import RailDetector, RailLine
from ball_tracker import BallTracker

class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = str(Path(self.tmp.name) / 'calibration.json')
        self.c = Calibration(self.path)
    def tearDown(self):
        self.tmp.cleanup()
    def points(self, xs=(100,500,900)):
        for cm, px in zip((-5,0,5), xs):
            self.c.add_point(cm, px, 1280,720)
    def test_two_points_not_ready(self):
        self.c.add_point(-5,100,1280,720)
        self.c.add_point(0,500,1280,720)
        self.assertFalse(self.c.fit()[0])
    def test_nonfinite_point_rejected(self):
        with self.assertRaises(ValueError):
            self.c.add_point(0,float('nan'),1280,720)
    def test_duplicate_pixel_rejected(self):
        self.points((100,100,900))
        self.assertFalse(self.c.fit()[0])
    def test_nonmonotonic_rejected(self):
        self.points((100,900,500))
        self.assertFalse(self.c.fit()[0])
    def test_linear_outside_range_invalid(self):
        self.points();self.c.fit()
        self.assertIsNone(self.c.px_to_cm(901,1280,720))
        self.assertTrue(self.c.extrapolated(901))
    def test_piecewise_reverse_axis(self):
        self.points((900,600,100));self.c.fit()
        self.assertAlmostEqual(self.c.px_to_cm(600,1280,720),0)
        self.assertIsNone(self.c.px_to_cm(99,1280,720))
    def test_roundtrip_and_resolution(self):
        self.points();self.c.fit()
        loaded=Calibration(self.path)
        self.assertTrue(loaded.ready)
        self.assertAlmostEqual(loaded.px_to_cm(500,1280,720),0)
        self.assertIsNone(loaded.px_to_cm(500,640,480))
    def test_corrupt_model_fails_closed(self):
        Path(self.path).write_text(json.dumps({'points':[{'x_cm':v,'x_px':p} for v,p in zip((-5,0,5),(100,500,900))], 'resolution':[1280,720], 'model':{'type':'linear','a':float('nan'),'b':0}}))
        self.assertFalse(Calibration(self.path).ready)

class TrackingTests(unittest.TestCase):
    def test_blank_frame_invalidates_old_rail(self):
        r=RailDetector({},1280,720)
        r.last=RailLine(0,330,True,.9)
        self.assertFalse(r.detect(np.zeros((720,1280),np.uint8),force=True).valid)
    def test_weak_candidates_expire(self):
        t=BallTracker({'acquire_frames':1},1280,720)
        t.update([SimpleNamespace(x=500,confidence=.9,method='geom')])
        for _ in range(t.uncertain_frames+1):
            out=t.update([SimpleNamespace(x=510,confidence=.2,method='color')])
        self.assertEqual(out['state'],'LOST')
        self.assertIsNone(t.x)
    def test_nonfinite_candidate_invalid(self):
        t=BallTracker({'acquire_frames':1},1280,720)
        out=t.update([SimpleNamespace(x=float('nan'),confidence=.9,method='geom')])
        self.assertFalse(out['valid'])

if __name__=='__main__': unittest.main()
