import unittest
from unittest.mock import patch
import cv2
import numpy as np
from ball_detector import BallDetector
from rail_detector import RailLine

class ColorPrefilterTests(unittest.TestCase):
    def test_small_glints_bypass_per_component_shape_work(self):
        im=np.zeros((720,1280,3),np.uint8)
        for x in range(170,1100,8):
            im[310:314,x:x+4]=(255,0,0)
        # One legitimate cap-sized component among many small glints.
        im[310:325,500:540]=(255,0,0)
        d=BallDetector({},1280,720);rail=RailLine(0,330,True)
        y0,y1=d._roi(rail);band=d._band(rail,y0,y1)
        with patch.object(d,'_shape_score',wraps=d._shape_score) as score:
            cs=d._color_candidates(im,band,rail,y0,y1)
        self.assertEqual(len(cs),1)
        self.assertAlmostEqual(cs[0].x,519.5)
        self.assertEqual(score.call_count,1)
