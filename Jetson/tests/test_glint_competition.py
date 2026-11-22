# -*- coding: utf-8 -*-
"""复现黑色固定工具端部与真球同时通过基础圆检的歧义。"""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

from ball_detector import BallDetector, BallCandidate


class GlintCompetitionTests(unittest.TestCase):
    def test_keep_uncertainty_unless_only_one_has_internal_reflections(self):
        # 模拟轨唇形成贯通采样窗的亮边，钢珠内部另有两个分离的点状反光。
        gray = np.full((100, 190), 60, np.uint8)
        cv2.line(gray, (0, 57), (189, 57), 150, 3)
        balls = [BallCandidate(x, 50, 50, .9, 'topview_circle',
                               dict(measurement_y_px=45.)) for x in (50, 135)]
        detector = BallDetector(dict(mode='topview_circle'), 190, 100)
        rail = SimpleNamespace(valid=True, identity_valid=True)
        with patch('local_vision.topview_balls', return_value=(balls, 'candidates')):
            def xs():
                return [c.x for c in detector.detect(None, gray, rail)]
            self.assertEqual(xs(), [50, 135])
            cv2.circle(gray, (45, 40), 3, 255, -1)
            cv2.circle(gray, (55, 44), 3, 255, -1)
            self.assertEqual(xs(), [50])
            cv2.circle(gray, (130, 40), 3, 255, -1)
            self.assertEqual(xs(), [50, 135])
