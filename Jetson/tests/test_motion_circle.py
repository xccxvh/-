# -*- coding: utf-8 -*-
"""离线构造沿轨运动的模糊球；高光和轨沿均来自图像，直接运行 OpenCV。"""
import unittest
from types import SimpleNamespace

import cv2
import numpy as np

from ball_detector import BallCandidate
from local_vision import topview_balls


class MotionCircleTests(unittest.TestCase):
    def setUp(self):
        self.gray = np.full((140, 320), 192, np.uint8)
        self.rail = SimpleNamespace(edge_profile=np.full(320, 104.),
                                    edge_support=np.ones(320, dtype=bool))

    def draw_balls(self, centers):
        # 横向盒式积分模拟曝光期间移动 19 px，随后画入静止的轨沿。
        for x in centers:
            cv2.circle(self.gray, (x, 76), 19, 80, -1)
            cv2.circle(self.gray, (x - 5, 71), 7, 250, -1)
        self.gray = cv2.blur(self.gray, (19, 1))
        cv2.line(self.gray, (0, 104), (319, 104), 80, 3)

    def detect(self, bounds=(60., 260.)):
        return topview_balls(self.gray, self.rail, bounds, BallCandidate, [])[0]

    def test_one_blurred_ball_returns_one_observed_center(self):
        self.draw_balls([155])
        balls = self.detect()
        self.assertEqual(len(balls), 1)
        self.assertAlmostEqual(balls[0].x, 155, delta=5)
        self.assertAlmostEqual(balls[0].extra['measurement_y_px'], 76, delta=4)

    def test_distinct_blurred_objects_remain_distinct(self):
        # 跟踪器需要知道图中有多个球状物，不能只交付得分最高的一个。
        self.draw_balls([111, 199])
        xs = sorted(ball.x for ball in self.detect())
        self.assertEqual(len(xs), 2)
        np.testing.assert_allclose(xs, [111, 199], atol=5)

    def test_outside_roi_or_missing_edge_support(self):
        self.draw_balls([155])
        self.assertFalse(self.detect((60., 140.)))
        self.rail.edge_support[:] = False
        self.assertFalse(self.detect())


if __name__ == '__main__':
    unittest.main()
