"""快搜必须测量本帧，并在小窗外的竞争、遮挡和跳变下执行完整身份判别。"""
import json
from pathlib import Path
import unittest

import cv2
import numpy as np

from camera_geometry import CameraGeometry
from static_fast_reference import make_rail_detector
from static_fast_geometry import StaticFastCoordinates
from static_reference_geometry import StaticReferenceCoordinates


JET = Path(__file__).resolve().parents[1]


class FastReferenceTests(unittest.TestCase):
    def setUp(self):
        self.old = json.loads((JET/'config_static_control_ruler_c.json').read_text())
        self.cfg = json.loads((JET/'config_static_fast_c.json').read_text())
        self.rc = self.cfg['rail']['upper_reference']
        self.gray = cv2.imread(str(JET/self.rc['image']), 0)
        self.rd = make_rail_detector(self.cfg['rail'], 1280, 720)

    def test_local_measurement_jump_and_failed_history(self):
        self.assertTrue(self.rd.detect(self.gray).valid)
        def moved(dx):
            return cv2.warpAffine(self.gray, np.float32([[1,0,dx],[0,1,0]]), (1280,720))
        r = self.rd.detect(moved(3))
        self.assertIn('local', r.registration['registration_mode'])
        np.testing.assert_allclose(r.reference_translation, [3,0], atol=0.01)
        r = self.rd.detect(moved(38))
        self.assertTrue(r.valid)
        self.assertNotIn('local_track', r.registration['registration_mode'])
        np.testing.assert_allclose(r.reference_translation, [38,0], atol=0.01)
        self.assertFalse(self.rd.detect(np.zeros_like(self.gray)).valid)
        self.assertTrue(self.rd.detect(self.gray).valid)

    def test_duplicate_outside_local_window_and_missing_upper_edge(self):
        for x,y,w,h in self.rc['boxes']:
            self.assertTrue(self.rd.detect(self.gray).valid)
            dup = self.gray.copy()
            dup[y-62:y-62+h,x:x+w] = self.gray[y:y+h,x:x+w]
            self.assertFalse(self.rd.detect(dup).valid)
        self.assertTrue(self.rd.detect(self.gray).valid)
        flat = np.full_like(self.gray,128)
        for x,y,w,h in self.rc['boxes']+self.rc['fallback_boxes']:
            flat[y:y+h,x:x+w]=self.gray[y:y+h,x:x+w]
        self.assertFalse(self.rd.detect(flat).valid)

    def test_geometry_binding_is_exclusive(self):
        geo = CameraGeometry(self.cfg['camera_geometry'],1280,720,JET)
        old = StaticReferenceCoordinates(self.old,geo,JET)
        fast = StaticFastCoordinates(self.cfg,geo,JET)
        self.assertTrue(old.calibration.validated)
        self.assertTrue(fast.calibration.validated)
        self.assertNotEqual(old.binding_id,fast.binding_id)
        self.cfg['rail_coordinates']['calibration_file']=self.old['rail_coordinates']['calibration_file']
        self.assertFalse(StaticFastCoordinates(self.cfg,geo,JET).calibration.ready)
        with self.assertRaises(ValueError):
            make_rail_detector(self.old['rail'],1280,720)
