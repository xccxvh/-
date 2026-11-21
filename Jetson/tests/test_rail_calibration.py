# -*- coding: utf-8 -*-
"""只用合成证据验证新厘米标定；这些数据不得作为实物标定发布。"""
import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


def binding():
    return dict(schema='rail-binding-v1', measurement_version='relative-cap-v1',
                rail_coordinate_version='rail-reference-v1', geometry={'width': 1280, 'height': 720},
                reference_image_sha256='a'*64, reference_config_sha256='b'*64,
                algorithm_sha256='c'*64)


def load_module(name):
    path = Path(__file__).resolve().parents[1] / (name+'.py')
    if not path.exists():
        raise AssertionError('missing new module '+name)
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def session(m, cm, start, count=20, shift=0., reverse=False, total=None):
    """生成明确标注为合成的自包含证据；每个区间在同一合成视频内互不重叠。"""
    b = binding()
    rows = [dict(frame=start+i, rail_s_px=500+(-20 if reverse else 20)*cm+shift,
                 pixel_valid=True, reference_valid=True, predicted=False,
                 calibration_valid=False, cm_valid=False, rail_binding_id=m.binding_id(b))
            for i in range(count)]
    return dict(schema='rail-sampling-session-v1', binding=b, rail_binding_id=m.binding_id(b),
                known_position=True, cm=cm, start_frame=start,
                end_frame=start+(total or count)-1, prediction_contract='column', rows=rows,
                source=dict(video_path='SYNTHETIC_NOT_REAL.avi', csv_path='SYNTHETIC_NOT_REAL.csv',
                            binding_path='SYNTHETIC_NOT_REAL.json', video_sha256='d'*64,
                            csv_sha256='e'*64, binding_sha256='f'*64))


class RailCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.m = load_module('rail_calibration')

    def build(self, method='linear', reverse=False, checks=True):
        fit = [session(self.m, cm, i*100, reverse=reverse) for i, cm in enumerate((-5, 0, 5))]
        verify = [session(self.m, cm, 400+i*100, reverse=reverse) for i, cm in enumerate((-3, 3))] if checks else []
        return self.m.build_model(binding(), fit, verify, method=method)

    def loaded(self, data, current_binding=None):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'model.json'
            path.write_text(json.dumps(data), encoding='utf-8')
            return self.m.RailCalibration(path, binding() if current_binding is None else current_binding)

    def test_outer_positions_allow_independent_core_endpoints(self):
        # 同一合同下，实测外侧刻度可替换旧端点；短覆盖或缺零点仍必须拒绝。
        fit = [session(self.m, cm, i*100) for i, cm in enumerate((-6, 0, 6))]
        verify = [session(self.m, cm, 400+i*100) for i, cm in enumerate((-5, -3, 3, 5))]
        model = self.loaded(self.m.build_model(binding(), fit, verify))
        self.assertTrue(model.validated)
        self.assertAlmostEqual(model.position(600), 5.)
        self.assertIsNone(model.position(621))
        for positions in [(-4, 0, 6), (-6, 0, 4), (-6, 1, 6)]:
            with self.assertRaises(ValueError):
                self.m.build_model(binding(), [session(self.m, cm, i*100) for i, cm in enumerate(positions)])

    def test_binding_id_matches_parent_json_including_unicode_geometry(self):
        import hashlib
        b = binding(); b['geometry']['说明'] = '纸尺坐标'
        # 与父模块一致，中文保留UTF-8原文，不使用ASCII转义。
        expected = hashlib.sha256(json.dumps(b, sort_keys=True, separators=(',', ':'), ensure_ascii=False, allow_nan=False).encode('utf-8')).hexdigest()
        self.assertEqual(self.m.binding_id(b), expected)
        self.assertIn('纸尺坐标', self.m.canonical(b))

    def test_unverified_fit_cannot_output_centimeters(self):
        c = self.loaded(self.build(checks=False))
        self.assertTrue(c.ready)
        self.assertFalse(c.validated)
        self.assertIsNone(c.position(500))

    def test_fit_requires_twenty_valid_samples_at_every_position(self):
        # 只数通过采样合同的有效行，不能用总行数或其他位置样本补足。
        for method in ('linear', 'piecewise'):
            for index in range(3):
                for count in (1, 19):
                    with self.subTest(method=method, index=index, count=count):
                        fit = [session(self.m, cm, i*100) for i, cm in enumerate((-5, 0, 5))]
                        for row in fit[index]['rows'][count:]:
                            row['pixel_valid'] = False
                        with self.assertRaisesRegex(ValueError, '20'):
                            self.m.build_model(binding(), fit, method=method)

    def test_loading_also_rejects_consistent_model_with_under_twenty_fit_samples(self):
        # 模拟旧版可生成的19样本文件，点统计也同步调整，不能只靠统计不一致拒绝。
        data = self.build()
        data['fit_sessions'][0]['rows'][-1]['pixel_valid'] = False
        data['points'][0]['samples'] = 19
        data['points'][0]['coverage'] = .95
        c = self.loaded(data)
        self.assertFalse(c.ready)
        self.assertFalse(c.validated)
        self.assertIsNone(c.position(500))

    def test_fit_reports_low_coverage_without_using_verification_coverage_gate(self):
        # 此处20/149为合成边界测试，不是现场数据；失败行保留在原始证据中。
        fit = [session(self.m, cm, i*200, count=149) for i, cm in enumerate((-5, 0, 5))]
        for sample in fit:
            for row in sample['rows'][20:]:
                row['pixel_valid'] = False
        for method in ('linear', 'piecewise'):
            with self.subTest(method=method):
                data = self.m.build_model(binding(), fit, method=method)
                self.assertTrue(data['ready'])
                self.assertFalse(data['validated'])
                for point in data['points']:
                    self.assertEqual(point['samples'], 20)
                    self.assertEqual(point['total_frames'], 149)
                    self.assertAlmostEqual(point['coverage'], 20./149.)
                self.assertEqual(len(data['fit_sessions'][2]['rows']), 149)
                # 同样低coverage用于独立验证时仍必须失败，不能借fit门槛绕过验收。
                verify = [session(self.m, -3, 800, total=149), session(self.m, 3, 1000)]
                rejected = self.m.build_model(binding(), fit, verify, method=method)
                self.assertFalse(rejected['validated'])
                self.assertFalse(rejected['validation']['metrics'][0]['passed'])
                self.assertEqual(rejected['validation']['thresholds']['min_coverage'], .95)
                self.assertEqual(rejected['validation']['thresholds']['min_samples'], 20)
                self.assertEqual(rejected['validation']['thresholds']['max_abs_cm'], .3)

    def test_fit_minimum_aggregates_nonoverlapping_sessions_of_same_position(self):
        fit = [session(self.m, -5, 0, count=10), session(self.m, -5, 30, count=10),
               session(self.m, 0, 100), session(self.m, 5, 200)]
        data = self.m.build_model(binding(), fit)
        self.assertEqual(data['points'][0]['samples'], 20)
        self.assertTrue(data['ready'])
        self.assertFalse(data['validated'])

    def test_linear_and_explicit_piecewise_allow_reverse_but_never_extrapolate(self):
        for method in ('linear', 'piecewise'):
            for reverse in (False, True):
                with self.subTest(method=method, reverse=reverse):
                    c = self.loaded(self.build(method, reverse))
                    self.assertTrue(c.ready and c.validated, c.status)
                    self.assertAlmostEqual(c.position(500), 0.)
                    self.assertAlmostEqual(c.position(400), 5. if reverse else -5.)
                    self.assertAlmostEqual(c.position(600), -5. if reverse else 5.)
                    for x in (399.999, 600.001, float('nan'), float('inf'), True, '500', None):
                        self.assertIsNone(c.position(x))

    def test_binding_exactness_and_live_mutation_fail_closed(self):
        b = binding(); c = self.loaded(self.build(), b)
        self.assertTrue(c.validated)
        b['geometry']['width'] = 640
        self.assertFalse(c.ready)
        self.assertFalse(c.validated)
        self.assertIsNone(c.position(500))
        for key, value in [('measurement_version', 'geom'), ('algorithm_sha256', '1'*64),
                           ('geometry', {'width':1280.0,'height':720})]:
            b = binding(); b[key] = value
            self.assertFalse(self.loaded(self.build(), b).ready)

    def test_loading_recomputes_model_points_and_independent_results(self):
        data = self.build()
        for kind in ('model', 'points', 'metrics', 'validated', 'rows'):
            bad = copy.deepcopy(data)
            if kind == 'model': bad['model']['a'] *= 2
            elif kind == 'points': bad['points'][0]['s_px'] += 1
            elif kind == 'metrics': bad['validation']['metrics'][0]['max_abs_cm'] = .29
            elif kind == 'validated': bad['validated'] = False
            else: bad['validation']['sessions'][0]['rows'][0]['rail_s_px'] += 100
            c = self.loaded(bad)
            self.assertFalse(c.validated, kind)
            self.assertIsNone(c.position(500), kind)

    def test_validation_quality_all_thresholds_and_no_hidden_out_of_range(self):
        fit = [session(self.m, cm, i*100) for i, cm in enumerate((-5, 0, 5))]
        for check in [session(self.m, -3, 400, count=19),
                      session(self.m, -3, 400, total=22),
                      session(self.m, -3, 400, shift=6.2),
                      session(self.m, -3, 400, shift=-100)]:
            data = self.m.build_model(binding(), fit, [check, session(self.m, 3, 500)])
            c = self.loaded(data)
            self.assertTrue(c.ready)
            self.assertFalse(c.validated)
            self.assertIsNone(c.position(500))
        # 20/21满足95%且误差0.3恰好在允许界限；独立于训练残差。
        data = self.m.build_model(binding(), fit, [session(self.m, -3, 400, total=21, shift=6), session(self.m, 3, 500)])
        self.assertTrue(self.loaded(data).validated)

    def test_independence_rejects_fit_positions_or_same_video_overlap(self):
        fit = [session(self.m, cm, i*100) for i, cm in enumerate((-5, 0, 5))]
        for check in [session(self.m, 0, 400), session(self.m, -3, 19), session(self.m, -3, 110)]:
            with self.assertRaises(ValueError):
                self.m.build_model(binding(), fit, [check, session(self.m, 3, 500)])
        with self.assertRaises(ValueError):
            self.m.build_model(binding(), fit, [session(self.m, -3, 400), session(self.m, -3, 500)])
        with self.assertRaises(ValueError):
            self.m.build_model(binding(), fit[:2], [])

    def test_piecewise_is_explicit_and_fit_residual_is_not_validation(self):
        fit = [session(self.m, -5, 0), session(self.m, 0, 100, shift=-50),
               session(self.m, 5, 200)]
        checks = [session(self.m, -3, 400, shift=-20), session(self.m, 3, 500, shift=-20)]
        linear = self.m.build_model(binding(), fit, checks)
        self.assertEqual(linear['model']['type'], 'linear')
        self.assertFalse(self.loaded(linear).validated)
        piecewise = self.m.build_model(binding(), fit, checks, method='piecewise')
        self.assertTrue(self.loaded(piecewise).validated)

    def test_evidence_cannot_be_reused_by_renaming_video(self):
        fit = [session(self.m, cm, i*100) for i, cm in enumerate((-5, 0, 5))]
        reused = session(self.m, -3, 10)
        reused['source']['video_path'] = 'renamed-video.avi'
        reused['source']['csv_path'] = 'different-export.csv'
        reused['source']['csv_sha256'] = '1'*64
        with self.assertRaises(ValueError):
            self.m.build_model(binding(), fit, [reused, session(self.m, 3, 500)])
        tampered = self.build()
        tampered['validation']['thresholds']['max_abs_cm'] = 30
        self.assertFalse(self.loaded(tampered).validated)

    def test_forged_ready_and_unknown_method_do_not_load(self):
        for data in [{'ready':True,'validated':True}, dict(self.build(), method='auto')]:
            self.assertFalse(self.loaded(data).ready)
        c = self.m.RailCalibration('/nonexistent/calibration.json', binding())
        self.assertFalse(c.ready)
        self.assertIsNone(c.position(500))


if __name__ == '__main__':
    unittest.main()
