import unittest
import numpy as np
import cv2
from rail_detector import RailDetector, RailLine
from ball_detector import BallDetector
from local_vision import PhysicalSearch

class LocalVisionTests(unittest.TestCase):
    def test_roi_requires_identity_and_does_not_extrapolate(self):
        cfg={'enabled':True,'target_min_cm':-5,'target_max_cm':5,'recovery_margin_cm':2,
             'anchors':[[-7,100],[-5,200],[0,500],[5,800],[7,900]]}
        roi=PhysicalSearch(cfg)
        rail=RailLine(0,200,True)
        self.assertIsNone(roi.bounds(rail))
        rail.identity_valid=True;rail.reference_inverse=np.eye(2);rail.reference_translation=np.zeros(2)
        rail.reference_profile=np.array([[0,200],[1000,200]])
        self.assertEqual(roi.bounds(rail),(100.,900.))
        cfg['recovery_margin_cm']=3
        with self.assertRaises(ValueError):PhysicalSearch(cfg)
    def test_flat_background_no_ball_and_shallow_cap(self):
        rail=RailLine(0,200,True);rail.identity_valid=True
        rail.reference_inverse=np.eye(2);rail.reference_translation=np.zeros(2)
        rail.reference_profile=np.array([[0,200],[640,200]])
        rail.edge_profile=np.full(640,200.);rail.edge_support=np.ones(640,dtype=bool)
        cfg={'mode':'relative_cap','relative_cap':{'min_width_px':12,'max_width_px':65}}
        d=BallDetector(cfg,640,360)
        im=np.zeros((360,640,3),np.uint8);im[:]=(190,110,40);im[200:220]=210
        self.assertEqual(d.detect(im,cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),rail),[])
        cv2.ellipse(im,(320,201),(21,9),0,180,360,(160,165,170),-1)
        cs=d.detect(im,cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),rail)
        self.assertTrue(any(abs(c.x-320)<3 for c in cs))
        self.assertTrue(all(c.method=='relative_cap' for c in cs))
    def test_unconfirmed_rail_cannot_detect(self):
        d=BallDetector({'mode':'relative_cap'},640,360)
        f=np.zeros((360,640,3),np.uint8)
        self.assertEqual(d.detect(f,f[:,:,0],RailLine(0,200,True)),[])

    def test_current_reference_tracks_upper_not_lower_and_invalidates(self):
        import json
        from pathlib import Path
        root=Path(__file__).resolve().parents[1]
        cfg=json.loads((root/'config_local_roi_20261116.json').read_text())
        gray=cv2.imread(str(root/'assets/rolling35_reference.png'),0)
        detector=RailDetector(cfg['rail'],1280,720)
        rail=detector.detect(gray,True)
        self.assertTrue(rail.geometry_valid and rail.identity_valid and rail.valid)
        self.assertLess(abs(rail.edge_profile[640]-232),6)
        self.assertLess(rail.y_at(640),250)  # 下缘约288，不能当作钢珠露出的上缘。
        lost=detector.detect(np.zeros_like(gray),True)
        self.assertFalse(lost.valid)
        self.assertIsNone(detector.last)
        self.assertFalse(lost.identity_valid)

    def test_flat_reflection_and_detached_glint_are_rejected(self):
        rail=RailLine(0,200,True);rail.identity_valid=True;rail.edge_profile=np.full(640,200.);rail.edge_support=np.ones(640,dtype=bool)
        d=BallDetector({'mode':'relative_cap'},640,360)
        for kind in ('flat','detached','clipped'):
            im=np.full((360,640,3),(190,110,40),dtype=np.uint8);im[200:220]=210
            if kind=='flat':im[190:200,300:341]=190
            elif kind=='detached':cv2.ellipse(im,(320,185),(20,7),0,180,360,(180,180,180),-1)
            else:im[155:201,300:341]=190
            with self.subTest(kind=kind):
                self.assertEqual(d.detect(im,cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),rail),[])

    def test_search_follows_reference_shift_and_rotation(self):
        cfg={'enabled':True,'anchors':[[-7,100],[-5,200],[0,500],[5,800],[7,900]]}
        rail=RailLine(0,200,True);rail.identity_valid=True
        a=.03;matrix=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
        rail.reference_inverse=np.linalg.inv(matrix);rail.reference_translation=np.array([12.,8.])
        rail.reference_profile=np.array([[0,200],[1000,200]])
        bounds=PhysicalSearch(cfg).bounds(rail)
        expected=np.array([[100,200],[900,200]]).dot(matrix.T)+rail.reference_translation
        np.testing.assert_allclose(bounds,expected[:,0])

    def test_ball_crossing_search_boundary_not_clamped(self):
        cfg={'enabled':True,'target_min_cm':-5,'target_max_cm':5,'recovery_margin_cm':2,
             'anchors':[[-7,200],[-5,230],[0,320],[5,410],[7,440]]}
        rail=RailLine(0,200,True);rail.identity_valid=True;rail.edge_profile=np.full(640,200.);rail.edge_support=np.ones(640,dtype=bool)
        rail.reference_inverse=np.eye(2);rail.reference_translation=np.zeros(2)
        rail.reference_profile=np.array([[0,200],[640,200]])
        d=BallDetector({'mode':'relative_cap','ball_search':cfg},640,360)
        for x,expected in [(320,True),(415,True),(430,False),(500,False)]:
            im=np.full((360,640,3),(190,110,40),dtype=np.uint8);im[200:220]=210
            cv2.ellipse(im,(x,201),(21,9),0,180,360,(170,170,170),-1)
            cs=d.detect(im,cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),rail)
            self.assertEqual(bool(cs),expected)
            if expected:self.assertLess(abs(cs[0].x-x),3)

    def test_ruler_identity_without_upper_geometry_is_invalid(self):
        import json
        from pathlib import Path
        root=Path(__file__).resolve().parents[1]
        cfg=json.loads((root/'config_local_roi_20261116.json').read_text())
        gray=cv2.imread(str(root/'assets/rolling35_reference.png'),0)
        # 保留下面的数字纹理，却抹去配置上缘；不能追到更下方的新强边。
        gray[200:244,:]=140
        rail=RailDetector(cfg['rail'],1280,720).detect(gray,True)
        self.assertFalse(rail.valid)
        self.assertFalse(rail.geometry_valid)

    def test_old_calibration_rejects_new_coordinate_method(self):
        from measurement_contract import calibrated_method
        self.assertFalse(calibrated_method('relative_cap'))

    def test_bad_roi_parameters_rejected(self):
        base={'enabled':True,'anchors':[[-7,100],[-5,200],[0,500],[5,800],[7,900]]}
        for patch in [{'target_min_cm':float('nan')},{'recovery_margin_cm':float('inf')},
                      {'target_min_cm':5,'target_max_cm':-5,'recovery_margin_cm':7}]:
            with self.subTest(patch=patch):
                with self.assertRaises(ValueError):PhysicalSearch(dict(base,**patch))

    def test_legacy_roi_masks_outside_before_candidate_extraction(self):
        from unittest.mock import patch
        cfg={'enabled':True,'anchors':[[-7,200],[-5,230],[0,320],[5,410],[7,440]]}
        rail=RailLine(0,200,True);rail.identity_valid=True
        rail.reference_inverse=np.eye(2);rail.reference_translation=np.zeros(2)
        rail.reference_profile=np.array([[0,200],[640,200]])
        d=BallDetector({'ball_search':cfg},640,360);frame=np.zeros((360,640,3),np.uint8)
        def inspect_mask(gray,band,*args,**kwargs):
            self.assertFalse(band[:,:200].any());self.assertFalse(band[:,441:].any())
            return []
        with patch.object(d,'_geom_candidates',side_effect=inspect_mask):
            d.detect(frame,frame[:,:,0],rail)

    def test_detached_cap_over_locally_erased_rail_rejected(self):
        import json
        from pathlib import Path
        root=Path(__file__).resolve().parents[1]
        cfg=json.loads((root/'config_local_roi_20261116.json').read_text())
        frame=cv2.imread(str(root/'assets/rolling35_reference.png'))
        frame[175:244,460:585]=(190,110,40)
        cv2.ellipse(frame,(500,228),(21,9),0,180,360,(170,170,170),-1)
        rail=RailDetector(cfg['rail'],1280,720).detect(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),True)
        self.assertTrue(rail.identity_valid)
        bcfg=dict(cfg['ball']);bcfg['ball_search']=cfg['ball_search']
        cs=BallDetector(bcfg,1280,720).detect(frame,cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),rail)
        self.assertFalse(any(abs(c.x-500)<30 for c in cs))

    def test_relative_segmentation_is_not_blue_only(self):
        rail=RailLine(0,200,True);rail.identity_valid=True
        rail.edge_profile=np.full(640,200.);rail.edge_support=np.ones(640,dtype=bool)
        for color in [(25,25,25),(50,160,70),(80,120,190)]:
            im=np.full((360,640,3),color,dtype=np.uint8);im[200:220]=210
            cv2.ellipse(im,(320,201),(21,9),0,180,360,(170,170,170),-1)
            cs=BallDetector({'mode':'relative_cap'},640,360).detect(im,cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),rail)
            with self.subTest(background=color):self.assertTrue(any(abs(c.x-320)<3 for c in cs))
