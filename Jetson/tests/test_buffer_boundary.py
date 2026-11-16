# -*- coding: utf-8 -*-
"""搜索限制约束中心位置；额外图像证据不能变成范围外有效输出。"""
import unittest
import cv2
import numpy as np
from local_vision import relative_caps
from rail_detector import RailLine
from ball_detector import BallCandidate


class BufferBoundaryTests(unittest.TestCase):
    def caps(self, x, clipped=False):
        rail=RailLine(0,200,True);rail.identity_valid=True
        rail.edge_profile=np.full(640,200.);rail.edge_support=np.ones(640,dtype=bool)
        im=np.full((360,640,3),(190,110,40),np.uint8);im[200:220]=210
        cv2.ellipse(im,(x,201),(21,9),0,180,360,(170,170,170),-1)
        if clipped:im[:200,:x]=(190,110,40)
        return relative_caps(im,rail,{},(200,440),BallCandidate)[0]

    def test_complete_observed_cap_center_inside_both_boundaries(self):
        for x in [201,210,230,410,430,439]:
            with self.subTest(x=x):
                c=self.caps(x);self.assertEqual(len(c),1)
                self.assertLess(abs(c[0].x-x),3)

    def test_center_outside_never_clamped_or_valid(self):
        for x in [170,190,197,443,450,470]:
            with self.subTest(x=x):self.assertEqual(self.caps(x),[])

    def test_partial_contour_is_not_completed_by_prediction(self):
        self.assertEqual(self.caps(210,True),[])

    def test_symmetric_reentry_keeps_five_frame_confirmation(self):
        from ball_tracker import BallTracker
        for side in [-1,1]:
            tracker=BallTracker({'identity_check':True,'acquire_frames':5},640,360)
            sequence=[320]*6+([170] if side<0 else [470])*12
            sequence+=([201,210,220,230,240,260] if side<0 else [439,430,420,410,400,380])
            outputs=[]
            for i,x in enumerate(sequence):
                result=tracker.update(self.caps(x),i/30.)
                outputs.append(result)
                if result['predicted']:self.assertFalse(result['valid'])
                if x<200 or x>440:self.assertFalse(result['valid'])
            # 长时间越界必须重新确认，不能用旧身份或速度直接输出。
            self.assertFalse(any(o['valid'] for o in outputs[18:22]))
            self.assertTrue(outputs[22]['valid'])
            self.assertLess(abs(outputs[22]['x_px']-sequence[22]),3)
            # 核心左230/右410像素在返回第4帧到达，第5帧恢复，即33.3ms。
            self.assertLessEqual((22-21)*1000./30,150)

    def test_roi_refuses_geometry_invalid_even_if_identity_stale(self):
        from local_vision import PhysicalSearch
        search=PhysicalSearch({'enabled':True,'anchors':[[-7,200],[0,320],[7,440]]})
        rail=RailLine(0,200,False);rail.identity_valid=True
        self.assertIsNone(search.bounds(rail))

    def test_halo_needs_observed_edge_not_only_template_geometry(self):
        import json
        from pathlib import Path
        from rail_detector import RailDetector
        from ball_detector import BallDetector
        root=Path(__file__).resolve().parents[1]
        cfg=json.loads((root/'config_local_roi_20261116.json').read_text())
        # 两端分别保留参考数字纹理、抹除球冠两侧上缘；不能用外推线冒充观测。
        for x,y in [(239,225),(1028,240)]:
            im=cv2.imread(str(root/'assets/rolling35_reference.png'))
            im[170:255,x-80:x+80]=(190,110,40)
            cv2.ellipse(im,(x,y),(21,9),0,180,360,(170,170,170),-1)
            gray=cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
            rail=RailDetector(cfg['rail'],1280,720).detect(gray,True)
            self.assertTrue(rail.valid)
            detector=BallDetector(dict(cfg['ball'],ball_search=cfg['ball_search']),1280,720)
            cs=detector.detect(im,gray,rail)
            self.assertFalse(any(abs(c.x-x)<35 for c in cs))

    def test_partial_outside_cap_cannot_shift_center_inside_with_real_support(self):
        from local_vision import UpperEdgeReference
        from ball_tracker import BallTracker
        class Matcher(object):
            transform=(np.eye(2),np.zeros(2))
            def update(self,gray):return {'valid':True,'reason':'ok'}
        for center,cut,axes in [(197,181,(32,23)),(443,459,(32,23)),
                                (197,171,(32,7)),(190,164,(32,7)),
                                (443,469,(32,7)),(450,476,(32,7))]:
            ref=UpperEdgeReference.__new__(UpperEdgeReference)
            ref.matcher=Matcher();ref.profile=np.array([[200.,200.],[320.,200.],[440.,200.]])
            ref.max_adjust=5;ref.support_margin=80
            im=np.full((360,640,3),(190,110,40),np.uint8);im[200:220]=210
            cv2.ellipse(im,(center,201),axes,0,180,360,(170,170,170),-1)
            if center<200:im[:200,:cut]=(190,110,40)
            else:im[:200,cut+1:]=(190,110,40)
            evidence=ref.locate(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY))
            rail=RailLine(evidence['slope'],evidence['intercept'],True);rail.identity_valid=True
            for k,v in evidence.items():setattr(rail,k,v)
            candidates,_=relative_caps(im,rail,{},(200,440),BallCandidate)
            self.assertEqual(candidates,[],center)
            tracker=BallTracker({'identity_check':True,'acquire_frames':5},640,360)
            for i in range(6):self.assertFalse(tracker.update(candidates,i/30.)['valid'])
