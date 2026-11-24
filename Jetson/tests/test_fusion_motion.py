# -*- coding: utf-8 -*-
"""用成对图像验证运动提案、纸尺配准和当帧测量契约。"""
import unittest
from types import SimpleNamespace

import cv2
import numpy as np

from ball_detector import BallCandidate
from ball_tracker import BallTracker
from fusion_motion import MotionCandidates


class FusionMotionTests(unittest.TestCase):
    def setUp(self):
        self.detector = MotionCandidates()
        self.bounds = (45., 300.)
        self.rail = SimpleNamespace(valid=True, identity_valid=True,
                                    reference_inverse=np.eye(2),
                                    reference_translation=np.zeros(2),
                                    edge_profile=np.full(340, 112.),
                                    edge_support=np.ones(340, bool))

    def frame(self, x=None, glint=True):
        # 保留可独立移动的球、固定轨唇和有纹理的背景。
        image = np.full((150, 340), 190, np.uint8)
        image[::7] = 176
        image[113:117] = 65
        if x is not None:
            cv2.circle(image, (x, 87), 24, 65, -1)
            if glint:
                cv2.circle(image, (x-3, 84), 7, 243, -1)
        return image

    def update(self, image, ts):
        return self.detector.update(image, self.rail, self.bounds, ts)

    def test_measure_current_ball_and_exclude_disappearance(self):
        self.assertEqual(self.update(self.frame(100), 0.), [])
        result = self.update(self.frame(155), .033)
        self.assertEqual(len(result), 1)
        ball = result[0]
        self.assertIsInstance(ball, BallCandidate)
        self.assertEqual(ball.method, 'topview_circle')
        self.assertEqual(ball.extra['proposal_source'], 'motion')
        self.assertEqual(ball.extra['coordinate_version'], 'topview-circle-v1')
        self.assertIs(ball.extra['identity_valid'], True)
        self.assertIs(ball.extra['in_search_roi'], True)
        self.assertGreaterEqual(ball.extra['appearance'], 35)
        self.assertAlmostEqual(ball.width, 48., delta=4.)
        self.assertEqual(ball.height, ball.width)
        self.assertAlmostEqual(ball.x, 155., delta=2.)
        self.assertAlmostEqual(ball.extra['measurement_y_px'], 87., delta=2.)
        tracker = BallTracker(dict(identity_check=True, acquire_frames=1), 340, 150)
        tracked = tracker.update(result, .033)
        self.assertTrue(tracked['valid'])
        self.assertFalse(tracked['predicted'])
        self.assertEqual(tracked['x_px'], ball.x)
        self.assertEqual(self.update(self.frame(), .066), [])

    def test_ruler_similarity_and_roi_origin(self):
        first = self.frame(150)
        self.update(first, 0.)
        # 让纸尺平移、转动、缩放，搜索框也随之移动；静止球不应触发运动提案。
        affine = cv2.getRotationMatrix2D((150., 87.), 4., 1.04)
        affine[:, 2] += (20., -6.)
        matrix = affine[:, :2]
        self.rail.reference_inverse = np.linalg.inv(matrix)
        self.rail.reference_translation = affine[:, 2]
        line = np.array([[0., 112.], [339., 112.]]).dot(matrix.T) + affine[:, 2]
        self.rail.edge_profile = np.interp(np.arange(340), line[:, 0], line[:, 1])
        self.bounds = (72., 318.)
        def warped(frame):
            return cv2.warpAffine(frame, affine, (340, 150))
        self.assertEqual(self.update(warped(first), .033), [])
        result = self.update(warped(self.frame(190)), .066)
        self.assertEqual(len(result), 1)
        expected = matrix.dot([190., 87.]) + affine[:, 2]
        np.testing.assert_allclose([result[0].x, result[0].extra['measurement_y_px']],
                                   expected, atol=2.)

    def test_missing_evidence_clears_history(self):
        self.update(self.frame(100), 0.)
        self.rail.valid = False
        self.assertEqual(self.update(self.frame(115), .033), [])
        self.rail.valid = True
        self.assertEqual(self.update(self.frame(145), .066), [])
        self.assertEqual(self.update(self.frame(190), .066), [])
        self.assertEqual(self.update(self.frame(145), 1.), [])
        self.assertEqual(self.update(self.frame(180, glint=False), 1.033), [])
        self.assertEqual(self.update(self.frame(322), 1.066), [])

    def test_invalid_transform_and_stale_frame_are_not_anchors(self):
        # 失效姿态和倒序采集帧都不能作为下一次图像配准的依据。
        self.update(self.frame(100), .1)
        self.rail.reference_inverse[:] = 0
        self.assertEqual(self.update(self.frame(140), .133), [])
        self.rail.reference_inverse = np.eye(2)
        self.assertEqual(self.update(self.frame(180), .166), [])
        self.assertEqual(self.update(self.frame(125), .14), [])
        self.assertEqual(self.update(self.frame(205), .199), [])
