"""Pixel-only controls for the independent contour experiment."""
import unittest
from types import SimpleNamespace

import cv2
import numpy as np

from fusion_contour import contour_candidates


class ContourTests(unittest.TestCase):
    def setUp(self):
        self.rail = SimpleNamespace(valid=True, identity_valid=True,
                                    edge_profile=np.full(320, 99.),
                                    edge_support=np.ones(320, bool))
        self.bounds = (44., 277.)

    def scene(self, centers, shape="ball", blur=1, glints=2):
        image = np.full((160, 320), 177, np.uint8)
        image[99:] = 225
        for x in centers:
            if shape == "ball":
                cv2.circle(image, (x, 72), 24, 72, -1)
            else:
                cv2.rectangle(image, (x-24, 48), (x+24, 96), 72, -1)
            for y in (62, 74)[:glints]:
                cv2.ellipse(image, (x, y), (11, 3), 0, 0, 360, 245, -1)
        return cv2.blur(image, (blur, 1))

    def test_observed_center_and_tracker_fields(self):
        for smear in (1, 13):
            gray = self.scene([115], blur=smear)
            hits = contour_candidates(gray, self.rail, self.bounds)
            self.assertEqual(len(hits), 1)
            ball = hits[0]
            self.assertAlmostEqual(ball.x, 115, delta=3)
            self.assertAlmostEqual(ball.extra['measurement_y_px'], 72, delta=3)
            self.assertAlmostEqual(ball.width, 48, delta=5)
            self.assertEqual(ball.width, ball.height)
            self.assertEqual(ball.method, 'topview_circle')
            self.assertEqual(ball.extra['coordinate_version'], 'topview-circle-v1')
            self.assertEqual(ball.extra['proposal_source'], 'contour')
            self.assertIs(ball.extra['identity_valid'], True)
            self.assertIs(ball.extra['in_search_roi'], True)
            half = int(.47 * ball.width / 2)
            x, y = round(ball.x), round(ball.extra['measurement_y_px'])
            smooth = cv2.GaussianBlur(gray, (5, 5), 1.2)
            core = smooth[y-half:y+half+1, x-half:x+half+1]
            self.assertAlmostEqual(ball.extra['appearance'],
                                   np.percentile(core, 95) - np.median(core))

    def test_keep_separate_objects(self):
        hits = contour_candidates(self.scene([90, 217]), self.rail, self.bounds)
        self.assertEqual(sorted(round(b.x) for b in hits), [90, 217])

    def test_invalid_reference_or_missing_side_support(self):
        image = self.scene([120])
        self.rail.edge_support[72:100] = False
        self.assertFalse(contour_candidates(image, self.rail, self.bounds))
        self.rail.edge_support[:] = True
        self.rail.identity_valid = False
        self.assertFalse(contour_candidates(image, self.rail, self.bounds))

    def test_ball_outside_with_visible_glints(self):
        self.assertFalse(contour_candidates(self.scene([38]), self.rail, self.bounds))

    def test_rectangular_tool_with_two_glints(self):
        for blur in (1, 13):
            self.assertFalse(contour_candidates(self.scene([120], "tool", blur),
                                                self.rail, self.bounds))

    def test_rotated_glossy_tool_shapes_under_horizontal_blur(self):
        for angle in (-20, -10, 0, 10, 20, 32):
            for size in ((32, 52), (48, 48), (52, 39), (64, 42), (75, 21)):
                for blur in (1, 9, 17, 25):
                    for body in (65, 115):
                        gray = np.full((160, 320), 185, np.uint8)
                        gray[99:] = 225
                        box = cv2.boxPoints(((160, 71), size, angle))
                        cv2.fillConvexPoly(gray, box.round().astype(np.int32), body)
                        for y in (61, 73):
                            cv2.ellipse(gray, (160, y), (12, 3), angle, 0, 360, 245, -1)
                        hits = contour_candidates(cv2.blur(gray, (blur, 1)),
                                                  self.rail, self.bounds)
                        self.assertFalse(hits, (angle, size, blur, body))

    def test_single_glint_and_lip(self):
        for image in (self.scene([120], glints=1), self.scene([])):
            self.assertFalse(contour_candidates(image, self.rail, self.bounds))


if __name__ == '__main__':
    unittest.main()
