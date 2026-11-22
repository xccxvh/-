# -*- coding: utf-8 -*-
"""离线构造沿轨运动的模糊球；高光和轨沿均来自图像，直接运行 OpenCV。"""
import unittest
from types import SimpleNamespace

import cv2
import numpy as np

from ball_detector import BallCandidate, BallDetector
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

    def motion_frame(self, x, blur):
        # 同一钢珠沿轨道移动，最后一次曝光加长到普通两轮 Hough 均漏检。
        g = np.full_like(self.gray, 192)
        cv2.circle(g, (x, 78), 22, 75, -1)
        for y in (68, 79):
            cv2.ellipse(g, (x, y), (8, 3), 0, 0, 360, 255, -1)
        g = cv2.blur(g, (blur, 1))
        cv2.line(g, (0, 104), (319, 104), 80, 3)
        return g

    def motion_detector(self, enabled):
        detector = BallDetector({'mode': 'topview_circle', 'motion_blur_recovery': enabled}, 320, 140)
        self.rail.valid = self.rail.identity_valid = True
        detector.search = SimpleNamespace(enabled=True, bounds=lambda rail: (60., 260.))
        return detector

    def test_motion_only_opt_in_fits_current_circle(self):
        # 用真 Hough 建立授权。在预测 x=146 的窄窗内，像素实际移动到 148；
        # 返回值必须来自当前帧的圆周拟合。缺省关闭的分支在同一帧应当无候选。
        for enabled in (False, True):
            detector = self.motion_detector(enabled)
            for i, x in enumerate((110, 122, 134)):
                self.assertEqual(len(detector.detect(None, self.motion_frame(x, 25), self.rail,
                                                     timestamp=i/30.)), 1)
            found = detector.detect(None, self.motion_frame(148, 31), self.rail, timestamp=.1)
            self.assertEqual(len(found), int(enabled))
            if enabled:
                self.assertAlmostEqual(found[0].x, 148, delta=1)
                self.assertEqual(found[0].extra['coordinate_version'], 'topview-circle-v1')
                self.assertEqual(found[0].extra['proposal_source'], 'motion_blur')
                self.assertEqual(detector.last_diagnostics[-1]['proposal_source'], 'motion_blur')

    def test_motion_authorization_expires_and_requires_direction(self):
        # 普通圆未形成同向运动、时钟断档、失效轨沿及一次补搜后的冷启动均不能授权。
        for centers, timestamp in [((134, 134, 134), .1), ((134, 122, 134), .1),
                                   ((110, 122, 134), .20), ((110, 122, 134), None)]:
            detector = self.motion_detector(True)
            for i, x in enumerate(centers):
                detector.detect(None, self.motion_frame(x, 25), self.rail, timestamp=i/30.)
            self.assertFalse(detector.detect(None, self.motion_frame(146, 31), self.rail,
                                             timestamp=timestamp))
        detector = self.motion_detector(True)
        for i, x in enumerate((110, 122, 134)):
            detector.detect(None, self.motion_frame(x, 25), self.rail, timestamp=i/30.)
        self.rail.valid = False
        self.assertFalse(detector.detect(None, self.motion_frame(146, 31), self.rail, timestamp=.1))
        self.rail.valid = True
        self.assertFalse(detector.detect(None, self.motion_frame(158, 31), self.rail, timestamp=.13))

    def test_second_measurement_has_a_fixed_hough_deadline(self):
        # 前两次补搜的图像都独立通过，第二次不能刷新普通圆时钟。停止、反向、
        # 超过 100 ms 及第三张连续模糊图均不能延续补搜。
        for x, timestamp, accepted in ((158, 4/30., True), (146, 4/30., False),
                                       (139, 4/30., False), (174, .17, False)):
            detector = self.motion_detector(True)
            for i, center in enumerate((110, 122, 134)):
                detector.detect(None, self.motion_frame(center, 25), self.rail, timestamp=i/30.)
            self.assertTrue(detector.detect(None, self.motion_frame(146, 31), self.rail, timestamp=.1))
            second = detector.detect(None, self.motion_frame(x, 31), self.rail, timestamp=timestamp)
            self.assertEqual(bool(second), accepted)
            self.assertFalse(detector.detect(None, self.motion_frame(170, 31), self.rail,
                                             timestamp=timestamp+1/30.))

    def test_parallel_tool_glints_without_curved_rim_are_rejected(self):
        # 双高光也会出现在直边工具上。用不同尺寸与转角构造硬负例，
        # 覆盖工具角点及斜边恰好落在预期圆周的情形，按正常检测入口验收。
        for angle in (0, 8, 18, 35, 45):
            for width in (32, 45, 55, 70, 114):
                for height in (30, 38, 45):
                    with self.subTest(angle=angle, width=width, height=height):
                        self.gray.fill(192)
                        box = cv2.boxPoints(((155, 77), (width, height), angle))
                        cv2.fillConvexPoly(self.gray, np.rint(box).astype(np.int32), 75)
                        for y in (68, 79):
                            cv2.ellipse(self.gray, (155, y), (17, 3),
                                        0, 0, 360, 243, -1)
                        self.assertFalse(self.detect())

    def test_outside_roi_or_missing_edge_support(self):
        self.draw_balls([155])
        self.assertFalse(self.detect((60., 140.)))
        self.rail.edge_support[:] = False
        self.assertFalse(self.detect())


if __name__ == '__main__':
    unittest.main()
