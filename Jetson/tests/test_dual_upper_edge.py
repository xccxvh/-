"""验证两套当帧配准的切换、竞争峰和真实上缘门控。"""
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import cv2
import numpy as np
from local_vision import UpperEdgeReference

JET = Path(__file__).resolve().parents[1]


class DualUpperEdgeTests(unittest.TestCase):
    def setUp(self):
        cfg = json.loads((JET/'config_motion_dual_points_20261122.json').read_text())
        self.cfg = cfg['rail']['upper_reference']
        self.image = cv2.imread(str(JET/self.cfg['image']), 0)
        self.edge = UpperEdgeReference(self.cfg)

    def test_weak_primary_rechecks_full_alternate_and_keeps_winner(self):
        # 两个中间裁框互有非重叠区域。擦去数字主框的两翼，备用框仍有原始完整纹理。
        damaged = self.image.copy()
        x,y,w,h = self.cfg['boxes'][1]
        fx,_,fw,_ = self.cfg['fallback_boxes'][1]
        damaged[y:y+h,x:fx] = 0
        damaged[y:y+h,fx+fw:x+w] = 0
        other = self.edge.other_matcher
        previous = other.update(self.image)
        self.assertTrue(previous['valid'])
        original = other.update
        def check_cold(frame):
            self.assertIsNone(other.transform)
            self.assertFalse(other._last_rectified)
            return original(frame)
        with patch.object(other,'update',side_effect=check_cold):
            result = self.edge.locate(damaged)
        self.assertEqual(result['registration']['reference_set'],'fallback')
        with patch.object(self.edge.other_matcher,'update',side_effect=AssertionError):
            next_result = self.edge.locate(self.image)
        self.assertEqual(next_result['registration']['reference_set'],'fallback')
        self.assertTrue(next_result['registration']['registration_mode'].startswith('fallback/'))

    def test_duplicate_primary_patch_blocks_otherwise_valid_alternate(self):
        # 保留整把真尺，并在搜索域内额外复制中间数字块，检验高分并列峰不能借备用框放行。
        dup = self.image.copy()
        x,y,w,h = self.cfg['boxes'][1]
        dup[y-65:y-65+h,x:x+w] = self.image[y:y+h,x:x+w]
        self.assertTrue(self.edge.other_matcher.update(dup)['valid'])
        self.assertIsNone(self.edge.locate(dup))
        self.assertEqual(self.edge.reason,'reference_competing_patch')
        self.assertIsNone(self.edge.matcher.transform)
        self.assertIsNone(self.edge.other_matcher.transform)

    def test_geometry_failure_keeps_identity_fallback_closed(self):
        # 复制刻纹确保原套模板可确认，但擦去上缘。备用身份不得使几何失败变为成功。
        ref = self.cfg
        points = np.array(ref['upper_edge_points'])
        dy = np.arange(self.image.shape[0])[:,None] - np.interp(np.arange(self.image.shape[1]),*points.T)
        flat = self.image.copy()
        flat[abs(dy) <= 10] = 128
        for x,y,w,h in ref['boxes'] + ref['fallback_boxes']:
            flat[y:y+h,x:x+w] = self.image[y:y+h,x:x+w]
        with patch.object(self.edge.other_matcher,'update',side_effect=AssertionError):
            self.assertIsNone(self.edge.locate(flat))
        self.assertEqual(self.edge.reason,'upper_edge_geometry')
