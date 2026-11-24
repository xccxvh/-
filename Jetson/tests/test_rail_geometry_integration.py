# -*- coding: utf-8 -*-
"""真实轨道坐标/厘米标定集成；全部输入为临时合成软件证据，不代表实测。

使用真实CameraGeometry(raw)、RailCoordinates、RailCalibration和模型重验证。
只合成当前帧参考变换与候选，不替换position()或厘米有效性门控。
"""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from ball_detector import BallCandidate
from camera_geometry import CameraGeometry
from rail_calibration import RailCalibration, binding_id, build_model
from rail_coordinates import RailCoordinates, reference_image_context
from rail_detector import RailLine


class RailGeometryIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.geometry = CameraGeometry(dict(mode='raw', camera_id='合成相机', focus_id='固定测试焦点'), 1280, 720)
        # 临时合成参考图仅用于真实绑定哈希；不使用实拍图、不运行硬件检测。
        image = np.zeros((720, 1280), dtype=np.uint8)
        image[230:270, 250:950] = 180
        ok, encoded = cv2.imencode('.png', image)
        self.assertTrue(ok)
        reference = self.root/'SYNTHETIC_REFERENCE.png'
        reference.write_bytes(encoded.tobytes())
        self.config = dict(rail=dict(upper_reference=dict(enabled=True, image=reference.name,
                                sha256=hashlib.sha256(reference.read_bytes()).hexdigest(),
                                image_context=reference_image_context(self.geometry))),
                           rail_coordinates=dict(enabled=True, left_px=[300., 230.],
                                                 origin_px=[600., 230.], right_px=[900., 230.]))
        uncalibrated = RailCoordinates(self.config, self.geometry, base_dir=self.root)
        self.binding = uncalibrated.binding
        self.assertEqual(binding_id(self.binding), uncalibrated.binding_id)
        # 合成真值s=20*cm，拟合[-5,0,+5]，独立验证[-3,+3]；不复用帧。
        fit = [self.session(cm, 100*i) for i, cm in enumerate((-5, 0, 5))]
        verify = [self.session(cm, 400+100*i) for i, cm in enumerate((-3, 3))]
        model = build_model(self.binding, fit, verify)
        self.assertTrue(model['validated'])
        path = self.root/'SYNTHETIC_VALIDATED_MODEL.json'
        path.write_text(json.dumps(model, ensure_ascii=False), encoding='utf-8')
        self.config['rail_coordinates']['calibration_file'] = path.name
        # 必须经过父模块真实构造/绑定/文件加载路径，不能直接注入标定替身。
        self.coordinates = RailCoordinates(self.config, self.geometry, base_dir=self.root)
        self.assertIsInstance(self.coordinates.calibration, RailCalibration)
        self.assertTrue(self.coordinates.calibration.ready)
        self.assertTrue(self.coordinates.calibration.validated)
        self.assertEqual(self.coordinates.binding, self.binding)

    def session(self, cm, start):
        """20个合成样本及显式已知位置确认；临时模型不能用于实物运行。"""
        bid = binding_id(self.binding)
        return dict(schema='rail-sampling-session-v1', binding=copy.deepcopy(self.binding),
                    rail_binding_id=bid, known_position=True, cm=cm, start_frame=start,
                    end_frame=start+19, prediction_contract='column',
                    source=dict(video_path='SYNTHETIC_NOT_REAL.avi', csv_path='SYNTHETIC_NOT_REAL.csv',
                                binding_path='SYNTHETIC_NOT_REAL.binding.json',
                                video_sha256=hashlib.sha256(b'SYNTHETIC VIDEO').hexdigest(),
                                csv_sha256=hashlib.sha256(b'SYNTHETIC CSV').hexdigest(),
                                binding_sha256=hashlib.sha256(b'SYNTHETIC SIDECAR').hexdigest()),
                    rows=[dict(frame=start+i, rail_s_px=20.*cm, pixel_valid=True, reference_valid=True,
                               calibration_valid=False, cm_valid=False, predicted=False, rail_binding_id=bid)
                          for i in range(20)])

    def inputs(self, seq, s=40., angle=0., shift=(0., 0.), scale=1.):
        """同一参考点按已知二维相似变换生成当前测量，含离轴y避免退化为只测x。"""
        matrix = scale*np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
        translation = np.asarray(shift, dtype=float)
        point = matrix.dot(np.array([600.+s, 217.]))+translation
        axis = np.array([[300., 230.], [900., 230.]]).dot(matrix.T)+translation
        slope = float((axis[1, 1]-axis[0, 1])/(axis[1, 0]-axis[0, 0]))
        rail = RailLine(slope, float(axis[0, 1]-slope*axis[0, 0]), True, 1., validity_policy='reference_identity')
        rail.identity_valid = True
        rail.reference_inverse = np.linalg.inv(matrix)
        rail.reference_translation = translation
        rail.reference_sequence = seq
        points = np.array([[350., 250.], [650., 260.], [850., 250.]])
        rail.registration = dict(valid=True, reference_points=points.tolist(),
                                 current_points=(points.dot(matrix.T)+translation).tolist(),
                                 image_rotation_deg=float(np.degrees(angle)), scale=scale, residual_px=0.)
        candidate = BallCandidate(float(point[0]), 30., 8., .9, 'relative_cap',
                                  dict(coordinate_version='relative-cap-v1', measurement_y_px=float(point[1]),
                                       in_search_roi=True, identity_valid=True))
        out = dict(valid=True, predicted=False, x_px=candidate.x, method=candidate.method)
        return out, [candidate], rail

    def assert_valid_cm(self, value, expected):
        self.assertTrue(value['pixel_valid'] and value['reference_valid'] and value['calibration_valid'])
        self.assertTrue(value['cm_valid'], value['rail_coordinate_status'])
        self.assertAlmostEqual(value['rail_x_cm'], expected, places=7)
        self.assertFalse(value['feedback_valid'])

    def seed_success(self):
        out, candidates, rail = self.inputs(1)
        value = self.coordinates.process(out, candidates, rail, 0)
        self.assert_valid_cm(value, 2.)
        return value

    def assert_closed_after_success(self, out, candidates, rail, frame=1):
        value = self.coordinates.process(out, candidates, rail, frame)
        self.assertTrue(value['calibration_valid'])
        self.assertFalse(value['cm_valid'])
        self.assertIsNone(value['rail_x_cm'])
        self.assertIsNone(value['rail_s_px'])
        self.assertFalse(value['feedback_valid'])
        return value

    def test_same_reference_point_has_same_cm_after_translation_and_rotation(self):
        seq = 0
        for s, expected in ((40., 2.), (-60., -3.)):
            baseline = None
            for angle, shift, scale in ((0., (0., 0.), 1.), (0., (14., -12.), 1.),
                                        (.035, (14., -12.), 1.), (-.035, (-8., 11.), 1.03)):
                with self.subTest(s=s, angle=angle, shift=shift, scale=scale):
                    seq += 1
                    out, candidates, rail = self.inputs(seq, s, angle, shift, scale)
                    value = self.coordinates.process(out, candidates, rail, seq-1)
                    self.assert_valid_cm(value, expected)
                    self.assertAlmostEqual(value['rail_s_px'], s, places=7)
                    np.testing.assert_allclose([value['reference_x_px'], value['reference_y_px']],
                                               [600.+s, 217.], rtol=0, atol=1e-7)
                    if baseline is None:
                        baseline = value['rail_x_cm']
                    self.assertAlmostEqual(value['rail_x_cm'], baseline, places=7)

    def test_invalid_reference_clears_previous_cm(self):
        self.seed_success()
        for i, flag in enumerate(('geometry_valid', 'identity_valid'), 2):
            with self.subTest(flag=flag):
                out, candidates, rail = self.inputs(i)
                setattr(rail, flag, False)
                value = self.assert_closed_after_success(out, candidates, rail, i-1)
                self.assertFalse(value['reference_valid'])
        out, candidates, rail = self.inputs(4)
        self.assert_valid_cm(self.coordinates.process(out, candidates, rail, 3), 2.)

    def test_prediction_clears_previous_cm_with_real_validated_model(self):
        self.seed_success()
        out, candidates, rail = self.inputs(2)
        out['predicted'] = True  # 即使valid被错误保留为True，也不能输出厘米。
        value = self.assert_closed_after_success(out, candidates, rail)
        self.assertFalse(value['pixel_valid'])

    def test_missing_candidate_clears_previous_cm(self):
        self.seed_success()
        out, _, rail = self.inputs(2)
        self.assert_closed_after_success(out, [], rail)

    def test_stale_reference_cannot_reuse_previous_cm(self):
        self.seed_success()
        out, candidates, rail = self.inputs(1)
        value = self.assert_closed_after_success(out, candidates, rail)
        self.assertFalse(value['reference_valid'])

    def test_real_model_refuses_buffer_outside_calibrated_interval(self):
        self.seed_success()
        out, candidates, rail = self.inputs(2, s=120.)  # 合成6cm不在仅[-5,+5]拟合区间。
        value = self.coordinates.process(out, candidates, rail, 1)
        self.assertTrue(value['calibration_valid'] and value['reference_valid'])
        self.assertAlmostEqual(value['rail_s_px'], 120.)
        self.assertFalse(value['cm_valid'])
        self.assertIsNone(value['rail_x_cm'])
        self.assertEqual(value['rail_coordinate_status'], 'outside_calibrated_range')


if __name__ == '__main__':
    unittest.main()
