"""数字复核与旧参考共用搜索域、上缘和竞争峰门限；测试无需外部原片或设备。"""
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from local_vision import UpperEdgeReference
from rail_detector import RailDetector
from static_control_reference import DigitUpperEdgeReference, make_rail_detector


class DigitReferenceTests(unittest.TestCase):
    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        path = Path(temp.name) / 'ref.png'
        self.ref = np.full((220, 500), 55, np.uint8)
        self.ref[100:] = 145
        primary = [[x, 135, 50, 20] for x in (70, 220, 365)]
        other = [[x, 165, 50, 20] for x in (70, 220, 365)]
        self.digits = [[x+10, 140, 18, 8] for x,y,w,h in primary]
        rng = np.random.RandomState(416)
        for x,y,w,h in primary + other:
            self.ref[y:y+h, x:x+w] = rng.randint(0, 256, (h,w), dtype=np.uint8)
        self.assertTrue(cv2.imwrite(str(path), self.ref))
        self.cfg = dict(enabled=True, image=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                        boxes=primary, fallback_boxes=other, upper_edge_points=[[40,100],[250,100],[460,100]],
                        search_x=70, search_y=35, rotation_recheck=True, adaptive_search=True)
        self.opt = dict(self.cfg, digit_boxes=self.digits)
        # 仅保留真实数字和可见的纸边；数字在本帧整体右移 4、上移 4 像素。
        self.frame = np.full_like(self.ref, 55)
        self.frame[96:] = 145
        for x,y,w,h in self.digits:
            self.frame[y-4:y-4+h, x+4:x+4+w] = self.ref[y:y+h, x:x+w]

    def test_factory_uses_empty_base_and_requires_exact_opt_in(self):
        rail_cfg = dict(upper_reference=self.opt, refresh_frames=7)
        with patch('static_control_reference.RailDetector', wraps=RailDetector) as base:
            rd = make_rail_detector(rail_cfg, 500, 220)
        base.assert_called_once_with(dict(rail_cfg, upper_reference={}), 500, 220)
        self.assertIsInstance(rd.upper_reference, DigitUpperEdgeReference)
        self.assertEqual(rail_cfg['upper_reference'], self.opt)
        self.assertTrue(rd.detect(self.frame).valid)
        for config in ({}, self.cfg, dict(self.opt, enabled=1), dict(self.opt, enabled=False),
                       dict(self.opt, digit_boxes=[])):
            with self.subTest(config=config), self.assertRaises(ValueError):
                make_rail_detector(dict(upper_reference=config), 500, 220)

    def test_digit_windows_must_be_three_complete_subboxes(self):
        for boxes in (None, self.digits[:2], [[65,140,18,8]] + self.digits[1:],
                      [[80,140,50,8]] + self.digits[1:], [[80.2,140,18,8]] + self.digits[1:]):
            with self.subTest(boxes=boxes), self.assertRaises(ValueError):
                DigitUpperEdgeReference(dict(self.cfg, digit_boxes=boxes))

    def test_old_profiles_keep_ordinary_results(self):
        plain = UpperEdgeReference(self.cfg)
        wrapped = DigitUpperEdgeReference(self.cfg)
        for g in (self.ref, self.frame, self.ref, None):
            a, b = plain.locate(g), wrapped.locate(g)
            self.assertEqual(plain.reason, wrapped.reason)
            if a is None:
                self.assertIsNone(b)
            else:
                self.assertEqual(a['registration'], b['registration'])
                np.testing.assert_array_equal(a['edge_profile'], b['edge_profile'])

    def test_current_digit_measurement_then_immediate_return_to_old_path(self):
        rd = make_rail_detector(dict(upper_reference=self.opt), 500, 220)
        first = rd.detect(self.frame)
        self.assertTrue(first.valid)
        self.assertEqual(first.registration['reference_set'], 'digit')
        ref = np.asarray(first.registration['reference_points'])
        np.testing.assert_allclose(np.asarray(first.registration['current_points']), ref + [4,-4])
        for boxes, expected in [(self.cfg['fallback_boxes'], 'primary'),
                                (self.cfg['boxes'], 'fallback')]:
            # 擦去一组原大框，另一组够用时应直接采用原结果。
            frame = self.ref.copy()
            for x,y,w,h in boxes:
                frame[y:y+h, x:x+w] = 145
            rail = rd.detect(frame)
            self.assertTrue(rail.valid)
            self.assertEqual(rail.registration['reference_set'], expected)

    def test_old_strong_competitor_veto_in_either_set(self):
        for boxes in (self.cfg['boxes'], self.cfg['fallback_boxes']):
            frame = self.frame.copy()
            x,y,w,h = boxes[1]
            tile = self.ref[y:y+h, x:x+w].copy()
            if boxes == self.cfg['boxes']:
                tile[5:13,10:28] = 145  # 只制造旧模板并列；数字自身仍为唯一峰。
            for dx in (-60, 60):
                frame[y:y+h, x+dx:x+dx+w] = tile
            digits_only = UpperEdgeReference(dict(self.cfg, boxes=self.digits,
                                                  fallback_boxes=self.digits))
            self.assertIsNotNone(digits_only.locate(frame))
            edge = DigitUpperEdgeReference(self.opt)
            self.assertIsNone(edge.locate(frame))
            self.assertEqual(edge.reason, 'reference_competing_patch')

    def test_digit_occlusion_duplicates_and_missing_edge_reject_after_success(self):
        edge = DigitUpperEdgeReference(self.opt)
        for j, (x,y,w,h) in enumerate(self.digits):
            for mode in ('erase','duplicate'):
                with self.subTest(j=j, mode=mode):
                    self.assertIsNotNone(edge.locate(self.frame))
                    frame = self.frame.copy()
                    x0,y0 = x+4,y-4
                    if mode == 'erase':
                        frame[y0:y0+h, x0:x0+w] = 145
                    else:
                        frame[y0:y0+h, x0+40:x0+40+w] = self.ref[y:y+h,x:x+w]
                    self.assertIsNone(edge.locate(frame))
        no_edge = self.frame.copy()
        no_edge[:115] = 145
        self.assertIsNone(edge.locate(no_edge))
        self.assertEqual(edge.reason, 'upper_edge_geometry')
        self.assertIsNone(edge.locate(None))
        self.assertIsNone(edge.locate(self.frame[:, :-1]))
