import unittest
from unittest.mock import patch
import numpy as np
from ball_detector import BallDetector, BallCandidate
from rail_detector import RailLine

class FusionTests(unittest.TestCase):
    def fuse(self, colors, geometry):
        d=BallDetector({},1280,720)
        frame=np.zeros((720,1280,3),np.uint8)
        with patch.object(d,'_color_candidates',return_value=colors),patch.object(d,'_geom_candidates',return_value=geometry):
            return d.detect(frame,frame[:,:,0],RailLine(0,330,True))
    def candidate(self,x,method,width=40,confidence=.85):
        return BallCandidate(x,width,13,confidence,method)
    def test_near_color_does_not_move_geometric_coordinate(self):
        cs=self.fuse([self.candidate(728,'color')],[self.candidate(720,'geom')])
        self.assertEqual(cs[0].method,'hybrid')
        self.assertEqual(cs[0].x,720)
    def test_offset_color_not_fused_or_promoted(self):
        cs=self.fuse([self.candidate(746.57,'color',90,.95)],[self.candidate(720,'geom')])
        self.assertEqual(cs[0].x,720)
        self.assertEqual(cs[0].method,'geom')
        self.assertLess(next(c.confidence for c in cs if c.method=='color'),.55)
    def test_real_motion_is_not_suppressed(self):
        out=[]
        for x in (300,320,350,390):
            out.append(self.fuse([self.candidate(x+8,'color')],[self.candidate(x,'geom')])[0].x)
        self.assertEqual(out,[300,320,350,390])
    def test_nearest_geometry_not_first_geometry(self):
        cs=self.fuse([self.candidate(718,'color')],[self.candidate(700,'geom'),self.candidate(720,'geom')])
        self.assertEqual(next(c.x for c in cs if c.method=='hybrid'),720)
    def test_color_alone_stays_weak(self):
        cs=self.fuse([self.candidate(720,'color')],[])
        self.assertEqual(cs[0].method,'color');self.assertLess(cs[0].confidence,.55)
    def test_each_geometry_used_once(self):
        cs=self.fuse([self.candidate(718,'color'),self.candidate(722,'color')],[self.candidate(720,'geom')])
        self.assertEqual(sum(c.method=='hybrid' for c in cs),1)

if __name__=='__main__':unittest.main()

class CalibrationRevisionTests(unittest.TestCase):
    def test_old_coordinate_calibration_cannot_be_reused_or_refitted(self):
        from calibration import Calibration
        import tempfile,json
        from pathlib import Path
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'c.json';c=Calibration(str(path))
            for cm,x in zip((-5,0,5),(100,500,900)):c.add_point(cm,x,1280,720)
            c.fit();data=json.loads(path.read_text());data.pop('measurement_version',None)
            path.write_text(json.dumps(data));old=Calibration(str(path))
            self.assertFalse(old.ready)
            self.assertFalse(old.fit()[0])
            with self.assertRaises(ValueError):old.add_point(0,500,1280,720)
