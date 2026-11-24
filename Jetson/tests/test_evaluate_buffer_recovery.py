# -*- coding: utf-8 -*-
"""单版本回放契约：分段统计、旧接口兼容和完整解码，使用独立临时目录。"""
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np


class BufferReplayTests(unittest.TestCase):
    def module(self):
        path = Path(__file__).resolve().parents[1] / 'evaluate_buffer_recovery.py'
        self.assertTrue(path.exists(), 'single-version replay tool is missing')
        spec = importlib.util.spec_from_file_location('buffer_replay_test', str(path))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_labels_sparse_dense_and_no_labels(self):
        m = self.module()
        label = dict(frame=2, status='visible', x_px=20, tolerance_px=6)
        self.assertEqual(m.normalize_labels({'labels': [label]}, 4)[2], label)
        self.assertEqual(m.normalize_labels([label], 4)[2], label)
        self.assertEqual(m.normalize_labels(None, 4), {})
        for bad in [[label, label], [dict(label, frame=4)],
                    [dict(label, x_px=float('nan'))], [dict(label, tolerance_px=-1)]]:
            with self.assertRaises(ValueError):
                m.normalize_labels(bad, 4)

    def test_segments_do_not_bridge_unlabeled_unknown_or_region_changes(self):
        m = self.module()
        rows = [self.row(0, 'core', 'miss'), self.row(1, 'core', 'miss'),
                self.row(2, 'unlabeled', 'unlabeled'), self.row(3, 'core', 'miss'),
                self.row(4, 'buffer', 'miss'), self.row(5, 'core', 'false_valid')]
        s = m.summarize(rows, 10)
        self.assertEqual(s['regions']['core']['assessments'], {'miss': 3, 'false_valid': 1})
        self.assertEqual([r['frames'] for r in s['regions']['core']['miss_runs']], [2, 1])
        self.assertEqual(s['labeled_frames'], 5)

    def test_reentry_requires_contiguous_observed_outside_buffer_core(self):
        m = self.module()
        rows = [self.row(0, 'outside', 'outside_rejected'),
                self.row(1, 'buffer', 'miss'), self.row(2, 'core', 'miss'),
                self.row(3, 'core', 'correct'), self.row(4, 'unlabeled', 'unlabeled'),
                self.row(5, 'buffer', 'miss'), self.row(6, 'core', 'correct'),
                self.row(7, 'outside', 'outside_rejected'),
                self.row(8, 'core', 'correct')]
        events = m.summarize(rows, 10)['reentry_events']
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]['entry_frame'], 1)
        self.assertEqual(events[0]['core_frame'], 2)
        self.assertEqual(events[0]['first_correct_frame'], 3)
        self.assertEqual(events[0]['delay_frames'], 2)
        self.assertEqual(events[0]['delay_s'], .2)

    def test_reentry_censoring_and_buffer_success(self):
        m = self.module()
        rows = [self.row(0, 'outside', 'outside_rejected'), self.row(1, 'buffer', 'correct'),
                self.row(2, 'core', 'miss'), self.row(3, 'outside', 'outside_rejected'),
                self.row(4, 'buffer', 'miss'), self.row(5, 'core', 'miss'),
                self.row(7, 'core', 'correct')]
        events = m.summarize(rows, 10)['reentry_events']
        self.assertEqual(len(events), 2)
        self.assertEqual(events[0]['delay_frames'], 0)
        self.assertTrue(events[1]['censored'])
        self.assertIsNone(events[1]['delay_frames'])

    def test_core_delay_does_not_reuse_earlier_buffer_success(self):
        m = self.module()
        rows = [self.row(0, 'outside', 'outside_rejected'),
                self.row(1, 'buffer', 'correct'), self.row(2, 'core', 'miss'),
                self.row(3, 'core', 'correct')]
        event = m.summarize(rows, 10)['reentry_events'][0]
        self.assertEqual(event['delay_frames'], 0)
        self.assertEqual(event['delay_from_core_frames'], 1)
        self.assertEqual(event['core_first_correct_frame'], 3)
        self.assertEqual(event['delay_from_core_s'], .1)
        early = m.summarize(rows[:3], 10)['reentry_events'][0]
        self.assertIsNone(early['delay_from_core_frames'])
        self.assertTrue(early['core_censored'])

    @staticmethod
    def row(frame, region, assessment):
        return dict(frame=frame, evaluation_region=region, assessment=assessment,
                    valid=int(assessment in ('correct', 'false_valid')), algo_ms=1.)

    def test_bounds_and_reference_failure_keep_pixel_assessment(self):
        m = self.module()
        rail = type('Rail', (), dict(valid=True, identity_valid=True,
                    reference_profile=np.array([[0, 100], [100, 100]]),
                    reference_inverse=np.eye(2), reference_translation=np.array([10, 0])))()
        cfg = dict(enabled=True, target_min_cm=-5, target_max_cm=5, recovery_margin_cm=2,
                   anchors=[[-7, 0], [-5, 20], [0, 50], [5, 80], [7, 100]])
        core, roi = m.physical_bounds(cfg, rail)
        np.testing.assert_allclose(core, [30, 90])
        np.testing.assert_allclose(roi, [10, 110])
        label = dict(status='visible', x_px=50, tolerance_px=6)
        region, assessment, error = m.assess(label, dict(valid=True, x_px=52), None, None)
        self.assertEqual((region, assessment, error), ('reference_unknown', 'correct', 2))

    def test_real_loop_legacy_interfaces_and_no_config_mutation(self):
        m = self.module()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            video = root / 'input.avi'; video.write_bytes(b'fixture')
            cfg = {'rail': {}, 'ball': {}, 'tracking': {}}
            config = root / 'config.json'; config.write_text(json.dumps(cfg))
            # 旧检测器没有diagnostics/ROI字段；工具只能读取接口，不能改变跟踪配置。
            class RailDetector:
                def __init__(self, cfg, w, h): pass
                def detect(self, gray, force=False):
                    return type('Rail', (), dict(valid=False, slope=0., intercept=100.))()
            class BallDetector:
                def __init__(self, cfg, w, h): pass
                def detect(self, frame, gray, rail): return []
            class Tracker:
                def __init__(self, cfg, w, h): pass
                def update(self, candidates, ts):
                    return dict(x_px=None, valid=False, confidence=0., state='LOST',
                                predicted=False, method=None)
            class Capture:
                def __init__(self, path): self.i = 0
                def isOpened(self): return True
                def read(self):
                    self.i += 1
                    return (True, np.zeros((720, 1280, 3), np.uint8)) if self.i <= 3 else (False, None)
                def release(self): pass
            args = ['--video', str(video), '--config', str(config), '--output', str(root/'run'),
                    '--expected-frames', '3', '--fps', '10']
            with patch.object(m, 'RailDetector', RailDetector), patch.object(m, 'BallDetector', BallDetector), \
                 patch.object(m, 'BallTracker', Tracker), patch.object(m.cv2, 'VideoCapture', Capture):
                result = m.main(args)
                self.assertEqual(result['frames'], 3)
                self.assertEqual(result['labeled_frames'], 0)
                self.assertEqual(json.loads(config.read_text()), cfg)
                with (root/'run'/'frames.csv').open() as stream:
                    rows = list(csv.DictReader(stream))
                self.assertEqual([r['frame'] for r in rows], ['0', '1', '2'])
                self.assertEqual(rows[2]['video_time_s'], '0.2')
                self.assertEqual(json.loads(rows[0]['candidate_diagnostics_json']), [])
                self.assertEqual(rows[0]['acquire_count'], '')
                args[args.index('3')] = '4'; args[args.index(str(root/'run'))] = str(root/'short')
                with self.assertRaises(ValueError): m.main(args)
                self.assertFalse((root/'short'/'summary.json').exists())

    def test_dense_visual_ticks_override_algorithm_roi_and_keep_uncertainty(self):
        m = self.module()
        label = dict(frame=0, status='visible', x_px=230, tolerance_px=6,
                     ruler5_search_left_x_px=229, ruler7_core_left_x_px=319,
                     anchor_tolerance_px=3, center_region_nominal='left_buffer',
                     center_region_uncertain=True)
        m.normalize_labels({'labels': [label]}, 1)
        # 故意传入与视觉标注矛盾的算法core，仍必须使用人工tick。
        out = dict(valid=True, x_px=230)
        self.assertEqual(m.assess(label, out, (100, 400), (50, 500))[0], 'buffer')
        self.assertEqual(m.assess(label, out, (100, 400), (50, 500), True)[0], 'boundary_unknown')
        outside = dict(label, x_px=210, center_region_nominal='outside_search', center_region_uncertain=False)
        m.normalize_labels([outside], 1)
        self.assertEqual(m.assess(outside, out, None, None)[0], 'outside')

    def test_real_dense_schema_and_declared_windows(self):
        m = self.module()
        # 固定契约覆盖实际文件使用的outside_search枚举，测试不依赖桌面文件存在。
        rows = [dict(self.row(0, 'outside', 'outside_rejected'), label_window=1),
                dict(self.row(1, 'buffer', 'miss'), label_window=2),
                dict(self.row(2, 'core', 'correct'), label_window=2)]
        self.assertEqual(m.summarize(rows, 10)['reentry_events'], [])

    def test_valid_output_contract_raises_instead_of_hiding_bad_outputs(self):
        m = self.module()
        candidate = type('Candidate', (), {'x': 100.})()
        detector = type('Detector', (), {'last_roi': (90., 110.)})()
        m.validate_output(dict(valid=True, predicted=False, x_px=100.), [candidate], detector)
        for out, cs in [(dict(valid=True, predicted=True, x_px=100.), [candidate]),
                        (dict(valid=True, x_px=101.), [candidate]),
                        (dict(valid=True, x_px=100.), [])]:
            with self.assertRaises(ValueError): m.validate_output(out, cs, detector)
        detector.last_roi = (101., 110.)
        with self.assertRaises(ValueError):
            m.validate_output(dict(valid=True, x_px=100.), [candidate], detector)
        m.validate_output(dict(valid=False, predicted=True, x_px=100.), [], detector)

    def test_encoder_count_mismatch_rejected(self):
        m = self.module()
        with patch.object(m.subprocess, 'check_output', return_value=b'{"streams":[{"nb_read_frames":"2"}]}'):
            with self.assertRaises(ValueError): m.verify_encoded('unused.mp4', 3)


if __name__ == '__main__':
    unittest.main()
