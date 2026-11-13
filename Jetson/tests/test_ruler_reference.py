import unittest
import cv2
import numpy as np
from ruler_reference import RulerReference

class RulerReferenceTests(unittest.TestCase):
    def setUp(self):
        rng=np.random.RandomState(4)
        self.ref=rng.randint(0,256,(240,640)).astype(np.uint8)
        self.boxes=[(90,100,40,24),(290,100,40,24),(490,100,40,24)]
        self.tr=RulerReference(self.ref,self.boxes,search_x=30,search_y=25,min_score=.7)
    def test_translation_preserves_reference_coordinate(self):
        moved=cv2.warpAffine(self.ref,np.float32([[1,0,14],[0,1,-8]]),(640,240))
        self.assertTrue(self.tr.update(moved)['valid'])
        x,y=self.tr.to_reference(314,112)
        self.assertAlmostEqual(x,300,places=4);self.assertAlmostEqual(y,120,places=4)
    def test_small_rotation_and_scale(self):
        mat=cv2.getRotationMatrix2D((320,120),1,.995)
        moved=cv2.warpAffine(self.ref,mat,(640,240))
        self.assertTrue(self.tr.update(moved)['valid'])
        pt=mat.dot(np.array([300,120,1]));x,y=self.tr.to_reference(*pt)
        self.assertAlmostEqual(x,300,delta=1);self.assertAlmostEqual(y,120,delta=1)
    def test_occlusion_invalidates_previous_transform(self):
        self.assertTrue(self.tr.update(self.ref)['valid'])
        self.assertFalse(self.tr.update(np.zeros_like(self.ref))['valid'])
        self.assertIsNone(self.tr.to_reference(300,120))
    def test_one_patch_moving_independently_rejected(self):
        moved=self.ref.copy();x,y,w,h=self.boxes[1]
        moved[y:y+h,x:x+w]=0;moved[y+10:y+10+h,x:x+w]=self.ref[y:y+h,x:x+w]
        self.assertFalse(self.tr.update(moved)['valid'])
    def test_resolution_change_rejected(self):
        self.assertFalse(self.tr.update(self.ref[:200])['valid'])
    def test_uniform_reference_rejected(self):
        with self.assertRaises(ValueError):RulerReference(np.zeros_like(self.ref),self.boxes)
    def test_nonfinite_point_rejected(self):
        self.tr.update(self.ref)
        self.assertIsNone(self.tr.to_reference(float('nan'),120))

    def test_repetitive_texture_is_ambiguous(self):
        rng=np.random.RandomState(8)
        tile=rng.randint(0,256,(240,20)).astype(np.uint8)
        image=np.tile(tile,(1,32))
        tracker=RulerReference(image,self.boxes,search_x=30,search_y=25)
        self.assertFalse(tracker.update(image)['valid'])

    def test_short_period_texture_is_ambiguous(self):
        rng=np.random.RandomState(9)
        image=np.zeros((240,640),np.uint8)
        boxes=[(90,100,80,24),(290,100,80,24),(490,100,80,24)]
        for x,y,w,h in boxes:
            tile=rng.randint(0,256,(24,8)).astype(np.uint8)
            image[y:y+h,x:x+88]=np.tile(tile,(1,11))
        tracker=RulerReference(image,boxes)
        self.assertFalse(tracker.update(image)['valid'])

    def test_empty_input_video_reports_no_measurements(self):
        from unittest.mock import Mock,patch
        from evaluate_ruler_reference import evaluate
        ref=Mock();ref.read.return_value=(True,cv2.cvtColor(self.ref,cv2.COLOR_GRAY2BGR))
        empty=Mock();empty.width=640;empty.height=240;empty.read.return_value=(False,None)
        with patch('evaluate_ruler_reference.VideoFile',side_effect=[ref,empty]):
            result=evaluate({'rail':{},'ball':{},'tracking':{}},'ref','empty',self.boxes)
        self.assertEqual(result['frames'],0)
        self.assertIsNone(result['registration_ms']['mean'])

    def test_geometry_preserves_actual_feature_y(self):
        from ball_detector import BallDetector
        from rail_detector import RailLine
        im=np.zeros((720,1280,3),np.uint8);im[330:345]=180
        cv2.ellipse(im,(525,330),(24,15),0,180,360,(220,220,220),-1)
        cs=BallDetector({},1280,720).detect(im,cv2.cvtColor(im,cv2.COLOR_BGR2GRAY),RailLine(0,330,True))
        self.assertEqual(len(cs),1)
        self.assertTrue(314<cs[0].extra.get('measurement_y_px',330)<325)

    def test_rotation_maps_the_actual_ball_feature(self):
        from ball_detector import BallCandidate
        from evaluate_ruler_reference import map_measurement
        mat=cv2.getRotationMatrix2D((320,120),3,1)
        # Perfect registration isolates coordinate projection from matching.
        self.tr.transform=(mat[:,:2].T,mat[:,2])
        feature=mat.dot(np.array([300,100,1]))
        candidate=BallCandidate(feature[0],30,12,.8,'geom',{'measurement_y_px':float(feature[1])})
        out={'valid':True,'predicted':False,'x_px':float(feature[0]),'method':'geom'}
        point=map_measurement(self.tr,out,[candidate])
        self.assertAlmostEqual(point[0],300,places=6)
        self.assertAlmostEqual(point[1],100,places=6)
