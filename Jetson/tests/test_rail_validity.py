# -*- coding: utf-8 -*-
"""区分轨道几何与身份：参考路径严格合取，旧路径显式保持几何策略。"""
import json
from pathlib import Path
import unittest

import cv2
import numpy as np

from ball_detector import BallDetector
from rail_detector import RailDetector, RailLine


class RailValidityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).resolve().parents[1]
        cls.cfg = json.loads((root / 'config_local_roi_20261116.json').read_text())
        cls.gray = cv2.imread(str(root / 'assets/rolling35_reference.png'), 0)
        if cls.gray is None:
            raise RuntimeError('reference image missing')

    def test_reference_validity_is_live_conjunction(self):
        line = RailDetector(self.cfg['rail'], 1280, 720).detect(self.gray, True)
        self.assertEqual(getattr(line, 'validity_policy', None), 'reference_identity')
        self.assertTrue(line.valid)
        # 各字段变化后仍必须遵守合取，避免保存一个过期的 valid 快照。
        for geometry, identity in [(False, True), (True, False),
                                   (False, False), (True, True)]:
            line.geometry_valid, line.identity_valid = geometry, identity
            self.assertEqual(line.valid, geometry and identity)

    def test_erased_upper_edge_clears_identity_preserves_reason_and_recovers(self):
        detector = RailDetector(self.cfg['rail'], 1280, 720)
        self.assertTrue(detector.detect(self.gray, True).valid)
        erased = self.gray.copy()
        erased[200:244, :] = 140
        line = detector.detect(erased, True)
        self.assertEqual(line.reason, 'upper_edge_geometry')
        self.assertFalse(line.geometry_valid)
        self.assertFalse(line.identity_valid)
        self.assertFalse(line.valid)
        self.assertIsNone(detector.last)
        recovered = detector.detect(self.gray, True)
        self.assertTrue(recovered.geometry_valid and recovered.identity_valid and recovered.valid)
        self.assertEqual(recovered.reason, 'confirmed_upper_edge')

    def test_reference_loss_preserves_specific_reason(self):
        detector = RailDetector(self.cfg['rail'], 1280, 720)
        detector.detect(self.gray, True)
        line = detector.detect(np.zeros_like(self.gray), True)
        self.assertEqual(line.reason, detector.upper_reference.reason)
        self.assertTrue(line.reason.startswith('reference_'))
        self.assertFalse(line.geometry_valid or line.identity_valid or line.valid)
        self.assertEqual(getattr(line, 'validity_policy', None), 'reference_identity')
        self.assertIsNone(detector.last)

    def test_invalid_reference_rail_stops_both_ball_paths(self):
        # 真实球顶图像先能产生旧候选；再用无效参考轨道证明检测入口关闭。
        frame = np.zeros((720, 1280, 3), np.uint8)
        frame[330:345] = 220
        cv2.ellipse(frame, (525, 330), (24, 13), 0, 180, 360, (230, 230, 230), -1)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.assertTrue(BallDetector({}, 1280, 720).detect(frame, gray, RailLine(0, 330, True)))
        line = RailDetector(self.cfg['rail'], 1280, 720).detect(self.gray, True)
        line.geometry_valid = False
        for config in ({}, {'mode': 'relative_cap'}):
            detector = BallDetector(config, 1280, 720)
            self.assertEqual(detector.detect(frame, gray, line), [])
            self.assertEqual(detector.last_reason, 'rail_invalid')

    def test_legacy_geometry_does_not_claim_identity(self):
        # 确定性水平边缘，覆盖首次拟合、缓存刷新以及显式失败后清空缓存。
        gray = np.zeros((720, 1280), np.uint8)
        gray[330:345] = 220
        detector = RailDetector({'expected_y_ratio': 330. / 720}, 1280, 720)
        line = detector.detect(gray, True)
        self.assertTrue(line.valid and line.geometry_valid)
        self.assertFalse(line.identity_valid)
        self.assertEqual(getattr(line, 'validity_policy', None), 'legacy_geometry')
        self.assertEqual(line.reason, 'legacy_geometry')
        self.assertIs(detector.detect(np.zeros_like(gray)), line)
        failed = detector.detect(np.zeros_like(gray), True)
        self.assertFalse(failed.valid or failed.geometry_valid or failed.identity_valid)
        self.assertIsNone(detector.last)

    def test_legacy_constructor_remains_usable_without_identity(self):
        line = RailLine(0, 330, True, .8)
        self.assertTrue(line.valid)
        self.assertFalse(line.identity_valid)
        self.assertEqual(getattr(line, 'validity_policy', None), 'legacy_geometry')
        self.assertEqual(line.confidence, .8)


if __name__ == '__main__':
    unittest.main()
