import unittest,json
from pathlib import Path
import cv2
from ball_detector import BallDetector
from rail_detector import RailLine

class LowCapTests(unittest.TestCase):
    def test_quantized_low_cap_remains_detectable(self):
        root=Path(__file__).parent/'fixtures'
        for idx,p in json.loads((root/'low_cap_poses.json').read_text()).items():
            with self.subTest(frame=idx):
                im=cv2.imread(str(root/('low_cap_%s.png'%idx)))
                cs=BallDetector({},1280,720).detect(im,cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),RailLine(p['rail_slope'],p['rail_intercept'],True))
                self.assertTrue(any(300<c.x<335 and 'recovered' in c.method for c in cs))

    def test_dim_right_endpoint_has_uncalibrated_candidate(self):
        root=Path(__file__).parent/'fixtures'
        for idx,p in json.loads((root/'dim_cap_poses.json').read_text()).items():
            with self.subTest(frame=idx):
                im=cv2.imread(str(root/('dim_cap_%s.png'%idx)))
                cs=BallDetector({},1280,720).detect(im,cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),RailLine(p['rail_slope'],p['rail_intercept'],True))
                self.assertTrue(any(880<c.x<920 and 'recovered' in c.method for c in cs))

    def test_flat_patch_above_secondary_threshold_not_promoted(self):
        import numpy as np
        im=np.zeros((720,1280,3),np.uint8);im[330:345]=180
        im[315:330,501:550]=90
        cs=BallDetector({},1280,720).detect(im,cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),RailLine(0,330,True))
        self.assertEqual(cs,[])

    def test_dim_rail_does_not_skip_recovery(self):
        import numpy as np
        im=np.zeros((720,1280,3),np.uint8);im[330:345]=90
        cv2.ellipse(im,(525,330),(24,15),0,180,360,(95,95,95),-1)
        cs=BallDetector({},1280,720).detect(im,cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),RailLine(0,330,True))
        self.assertTrue(any(abs(c.x-525)<2 and 'recovered' in c.method for c in cs))
