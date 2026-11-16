# -*- coding: utf-8 -*-
"""轨道二维坐标合同：变换补偿、陈旧姿态、空间混用与无效厘米保护。"""
import copy
import json
from pathlib import Path
import unittest
import numpy as np
from ball_detector import BallCandidate
from rail_detector import RailLine
from rail_coordinates import RailCoordinates, reference_image_context

class GeometryStub(object):
    mode='raw';image_space='raw_image'
    def __init__(self,mode='raw'):
        self.mode=mode;self.image_space='undistorted_image' if mode=='image' else 'raw_image'
        self.context=dict(mode=mode,resolution=[1280,720],camera_id='test-camera',focus_id='fixed-test',intrinsics_sha256=None if mode=='raw' else 'a'*64)
    def correct_points(self,p):return np.asarray(p,dtype=float)

class RailCoordinatesTests(unittest.TestCase):
    def config(self,g):
        root=Path(__file__).resolve().parents[1]
        cfg=json.loads((root/'config_local_roi_20261116.json').read_text())
        cfg['rail']['upper_reference']['image_context']=reference_image_context(g)
        cfg['rail_coordinates']={'enabled':True,'left_px':[300.,230.],'origin_px':[600.,230.],'right_px':[900.,230.]}
        return cfg
    def rail(self,matrix,shift,seq=1):
        r=RailLine(0,230,True,validity_policy='reference_identity');r.identity_valid=True
        r.reference_inverse=np.linalg.inv(matrix);r.reference_translation=np.asarray(shift);r.reference_sequence=seq
        pts=np.array([[350.,250.],[650.,260.],[850.,250.]])
        r.registration=dict(valid=True,reference_points=pts.tolist(),current_points=(pts.dot(matrix.T)+shift).tolist(),image_rotation_deg=0.,scale=1.,residual_px=0.)
        return r
    def sample(self,point):
        c=BallCandidate(point[0],30,8,.9,'relative_cap',dict(coordinate_version='relative-cap-v1',measurement_y_px=point[1],in_search_roi=True,identity_valid=True))
        out=dict(valid=True,predicted=False,x_px=point[0],method='relative_cap')
        return out,[c]
    def test_xy_mapping_translation_rotation_scale(self):
        for mode in ['raw','points','image']:
            g=GeometryStub(mode);cfg=self.config(g);rc=RailCoordinates(cfg,g)
            for i,a in enumerate([0.,.035,-.035]):
                m=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])*1.03;shift=np.array([14.,-12.]);pt=m.dot([720.,217.])+shift
                o,cs=self.sample(pt);v=rc.process(o,cs,self.rail(m,shift,i+1),i)
                self.assertTrue(v['pixel_valid'] and v['reference_valid']);self.assertAlmostEqual(v['rail_s_px'],120.,places=6)
                self.assertFalse(v['cm_valid']);self.assertIsNone(v['rail_x_cm']);self.assertFalse(v['feedback_valid'])
    def test_stale_reference_and_prediction_never_valid(self):
        g=GeometryStub();rc=RailCoordinates(self.config(g),g);r=self.rail(np.eye(2),[0.,0.]);o,cs=self.sample([700.,220.])
        self.assertTrue(rc.process(o,cs,r,0)['reference_valid'])
        self.assertFalse(rc.process(o,cs,r,1)['reference_valid'])
        o['predicted']=True;v=rc.process(o,cs,self.rail(np.eye(2),[0.,0.],2),2)
        self.assertFalse(v['pixel_valid'] or v['cm_valid'])
    def test_invalid_geometry_and_missing_y_close_measurement(self):
        g=GeometryStub();rc=RailCoordinates(self.config(g),g);o,cs=self.sample([700.,220.]);r=self.rail(np.eye(2),[0.,0.]);r.geometry_valid=False
        self.assertFalse(rc.process(o,cs,r,0)['reference_valid'])
        del cs[0].extra['measurement_y_px']
        self.assertFalse(rc.process(o,cs,self.rail(np.eye(2),[0.,0.],2),1)['cm_valid'])
    def test_ambiguous_candidates_are_not_cm_samples(self):
        g=GeometryStub();rc=RailCoordinates(self.config(g),g);o,cs=self.sample([700.,220.]);cs+=self.sample([800.,220.])[1]
        v=rc.process(o,cs,self.rail(np.eye(2),[0.,0.]),0);self.assertIsNone(v['rail_s_px']);self.assertFalse(v['cm_valid'])
    def test_wrong_space_or_focus_rejected(self):
        g=GeometryStub();cfg=self.config(g)
        for key,value in [('image_space','undistorted_image'),('focus_id','moved-focus')]:
            c=copy.deepcopy(cfg);c['rail']['upper_reference']['image_context'][key]=value
            with self.assertRaises(ValueError):RailCoordinates(c,g)
    def test_point_correction_applies_to_registration_and_measurement(self):
        class ScaledPoints(GeometryStub):
            def correct_points(self,p):return np.asarray(p,dtype=float)*np.array([1.1,.9])
        g=ScaledPoints('points');rc=RailCoordinates(self.config(g),g)
        o,cs=self.sample([720.,217.]);v=rc.process(o,cs,self.rail(np.eye(2),[0.,0.]),0)
        self.assertAlmostEqual(v['rail_s_px'],132.,places=6)

    def test_calibrated_range_rejection_has_explicit_status(self):
        class RangeLimited(object):
            ready=True;validated=True;status='validated'
            def position(self,s):return None
        g=GeometryStub();rc=RailCoordinates(self.config(g),g);rc.calibration=RangeLimited()
        o,cs=self.sample([700.,220.]);v=rc.process(o,cs,self.rail(np.eye(2),[0.,0.]),0)
        self.assertTrue(v['calibration_valid']);self.assertFalse(v['cm_valid'])
        self.assertEqual(v['rail_coordinate_status'],'outside_calibrated_range')
