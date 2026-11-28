"""Observed-pose offline fallback must not publish a partial, stale or ambiguous ruler."""
import json
from pathlib import Path
import unittest

import cv2
import numpy as np

from camera_geometry import CameraGeometry
from deep_pose_reference import DEEP, DeepPoseReference
from observed_pose_reference import ANCHOR, ObservedPoseReference, bind_offline_coordinates
from static_fast_geometry import StaticFastCoordinates
from static_fast_reference import FastUpperEdgeReference

JET = Path(__file__).resolve().parents[1]


class ObservedPoseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = json.loads((JET/'config_static_fast_c.json').read_text())
        cls.ref = cls.cfg['rail']['upper_reference']
        cls.deep = cv2.imread(str(ANCHOR.with_name('13489.448575.jpg')), 0)

    def test_full_fresh_three_patch_pose(self):
        candidate = ObservedPoseReference(self.ref)
        self.assertIsNone(candidate.base.locate(self.deep))
        observation = candidate.locate(self.deep)
        self.assertEqual(observation['registration']['reference_set'],'observed_pose')
        self.assertGreaterEqual(observation['registration']['min_margin'], .05)
        self.assertGreaterEqual(observation['registration']['min_score'], .85)
        self.assertLessEqual(observation['registration']['residual_px'], 2.5)
        self.assertIsNone(candidate.locate(np.zeros_like(self.deep)))
        self.assertIsNotNone(candidate.locate(self.deep))

    def test_missing_or_repeated_patch_on_recovered_frame(self):
        for i in range(3):
            for copy in (True,False):
                with self.subTest(i=i, copy=copy):
                    matcher = ObservedPoseReference(self.ref)
                    points = matcher.locate(self.deep)['registration']['current_points']
                    x,y,w,h = self.ref['boxes'][i]
                    x,y = np.rint(np.array(points[i])-[w/2,h/2]).astype(int)
                    altered = self.deep.copy()
                    if copy:
                        altered[y-3:y+h+3,x+112-3:x+w+112+3] = self.deep[y-3:y+h+3,x-3:x+w+3]
                    else:
                        altered[y-3:y+h+3,x-3:x+w+3] = 128
                    self.assertIsNone(matcher.locate(altered))

    def test_base_pose_exact_and_edge_required(self):
        src = cv2.imread(str(JET/self.ref['image']), 0)
        observed = ObservedPoseReference(self.ref)
        ref = FastUpperEdgeReference(self.ref).locate(src)
        same = observed.locate(src)
        np.testing.assert_array_equal(same['reference_translation'],ref['reference_translation'])
        np.testing.assert_array_equal(same['reference_inverse'],ref['reference_inverse'])
        new = observed.locate(self.deep)
        flat = np.full_like(self.deep,128)
        for (_,_,w,h),p in zip(self.ref['boxes'],new['registration']['current_points']):
            x,y = np.rint(np.asarray(p)-[w/2,h/2]).astype(int)
            flat[y:y+h,x:x+w] = self.deep[y:y+h,x:x+w]
        self.assertIsNone(observed.locate(flat))

    def test_bound_model_must_be_separate(self):
        geo = CameraGeometry(self.cfg['camera_geometry'],1280,720,JET)
        old = StaticFastCoordinates(self.cfg,geo,JET)
        with self.assertRaises(ValueError):
            bind_offline_coordinates(old)
        cfg = dict(self.cfg,rail_coordinates=dict(self.cfg['rail_coordinates'],calibration_file=None))
        candidate = StaticFastCoordinates(cfg,geo,JET)
        bind_offline_coordinates(candidate)
        self.assertNotEqual(candidate.binding_id,old.binding_id)
        self.assertEqual(candidate.binding['observed_pose_reference']['base_binding_id'],old.binding_id)

    def test_deep_pose_six_current_frame_checks(self):
        gray = cv2.imread(str(DEEP.with_name('13487.300585.jpg')),0)
        reference = DeepPoseReference(self.ref)
        self.assertEqual(reference.locate(gray)['registration']['reference_set'],'deep_pose')
        for bank in (reference.primary,reference.check):
            points = bank.update(gray)['current_points']
            for (_,_,w,h,_),p in zip(bank.patches,points):
                x,y = np.rint(np.asarray(p)-[w/2,h/2]).astype(int)
                for duplicate in (False,True):
                    altered=gray.copy()
                    if duplicate:
                        altered[y-2:y+h+2,x+110:x+w+114]=gray[y-2:y+h+2,x-2:x+w+2]
                    else:
                        altered[y-2:y+h+2,x-2:x+w+2]=128
                    # A valid old path may use different, unaffected print. The deep
                    # branch must not return a pose with a missing or ambiguous winner.
                    expected = FastUpperEdgeReference(self.ref).locate(altered)
                    actual = DeepPoseReference(self.ref).locate(altered)
                    self.assertEqual(actual is None,expected is None)
                    if actual is not None:
                        np.testing.assert_array_equal(actual['reference_translation'],expected['reference_translation'])
