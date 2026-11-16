# -*- coding: utf-8 -*-
"""基准工具只在合成输入上测试；显示和编码故障用边界替身，不访问硬件。"""
import csv
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np

import benchmark_live as bench


class SyntheticCamera(object):
    width, height, fps_set = 1280, 720, 30.

    def __init__(self, cfg):
        self.counter = bench.CaptureCounter()
        self.last_ts = None
        self.released = False
        self.fourcc = bench.cv2.VideoWriter_fourcc(*'MJPG')
        self.cap = SimpleNamespace(get=lambda prop: {
            bench.cv2.CAP_PROP_FRAME_WIDTH: self.width,
            bench.cv2.CAP_PROP_FRAME_HEIGHT: self.height,
            bench.cv2.CAP_PROP_FOURCC: self.fourcc}.get(prop, 0))

    def read(self):
        time.sleep(.005)
        now = time.perf_counter()
        # 每次处理读取前成功采到两帧，模拟最新帧队列覆盖；采集数不能等于处理数。
        self.counter.record(now - .001)
        self.counter.record(now)
        self.last_ts = now
        return True, np.zeros((self.height, self.width, 3), np.uint8)

    def release(self):
        self.released = True


class SyntheticWriter(object):
    def __init__(self, *args, **kwargs):
        self.frames = self.written = self.dropped = 0
        self.error = None
        self.proc = None
        self.submissions = []
        self.released = False

    def write(self, frame, timestamp=None):
        self.submissions.append((frame.copy(), timestamp))
        # 用明确的交替拒绝验证工具正确记录队列丢帧，而非把提交数当成功数。
        if len(self.submissions) % 2 == 0:
            self.dropped += 1
            return False
        self.frames += 1
        return True

    def release(self):
        self.written = self.frames
        self.released = True


class BenchmarkTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.config = self.root / 'config.json'
        self.config.write_text(json.dumps(dict(camera={}, rail={}, ball={}, tracking={})))
        self.source = SyntheticCamera({})

    def run_mode(self, mode, **kwargs):
        with patch.object(bench, 'CountingCamera', return_value=self.source):
            return bench.run_benchmark(mode, .035, self.config, self.root / mode, **kwargs)

    def rows(self, mode):
        with (self.root / mode / 'frames.csv').open() as stream:
            return list(csv.DictReader(stream))

    def test_counter_counts_only_active_successful_reads(self):
        counter = bench.CaptureCounter()
        counter.record(1.)
        counter.start(2.)
        class Capture(object):
            def __init__(self): self.results = iter([(True, 'a'), (False, None), (True, 'b')])
            def read(self): return next(self.results)
        cap = bench.CountingCapture(Capture(), counter)
        with patch.object(bench.time, 'perf_counter', side_effect=[2.1, 2.3]):
            cap.read(); cap.read(); cap.read()
        stats = counter.stop(3.)
        counter.record(4.)
        self.assertEqual(stats['frames'], 2)
        self.assertAlmostEqual(stats['fps'], 2.)
        self.assertAlmostEqual(stats['interval_fps'], 5.)

    def test_a_measures_capture_separately_without_overlay_or_writer(self):
        with patch.object(bench, 'draw_overlay') as overlay, patch.object(bench, 'AsyncVideoWriter') as writer:
            result = self.run_mode('A')
        self.assertEqual(result['status'], 'complete')
        self.assertEqual(result['capture']['frames'], result['processed_frames'] * 2)
        self.assertAlmostEqual(result['capture']['fps'], result['processing_fps'] * 2)
        self.assertTrue(self.source.released)
        overlay.assert_not_called(); writer.assert_not_called()
        rows = self.rows('A')
        self.assertTrue(rows)
        for row in rows:
            self.assertEqual((row['cm_valid'], row['x_cm'], row['uart_enabled']), ('0', '', '0'))
            self.assertTrue(row['rail_reason'] and row['detector_reason'] and row['tracker_reason'])
            self.assertIn('roi_min_px', row)
            self.assertGreaterEqual(float(row['capture_return_to_output_ms']), float(row['algo_ms']))
        self.assertIsNotNone(result['algo_ms']['p95'])
        self.assertGreater(result['process_rss_peak_mib'], 0)

    def test_c_records_annotated_frames_counts_drops_and_capture_timestamps(self):
        writer = SyntheticWriter()
        with patch.object(bench, 'AsyncVideoWriter', return_value=writer), patch.object(bench.cv2, 'imshow') as show:
            result = self.run_mode('C')
        show.assert_not_called()
        self.assertTrue(writer.released)
        self.assertEqual(result['recording']['frames'], writer.frames)
        self.assertEqual(result['recording']['dropped'], writer.dropped)
        self.assertEqual(writer.frames + writer.dropped, result['processed_frames'])
        self.assertTrue(all(frame.any() and ts > 0 for frame, ts in writer.submissions))
        self.assertEqual([float(r['capture_return_s']) for r in self.rows('C')],
                         [ts for frame, ts in writer.submissions])
        self.assertTrue(result['recording']['complete'])

    def test_b_headless_fails_before_camera_open(self):
        with patch.dict(bench.os.environ, {}, clear=True), patch.object(bench, 'CountingCamera') as camera:
            with self.assertRaises(RuntimeError):
                bench.run_benchmark('B', .01, self.config, self.root / 'B')
            camera.assert_not_called()
        result = json.loads((self.root / 'B' / 'summary.json').read_text())
        self.assertEqual(result['status'], 'error')
        self.assertFalse(result['display_verified'])

    def test_b_unusable_display_probe_fails(self):
        import subprocess
        with patch.dict(bench.os.environ, {'DISPLAY': ':invalid'}), patch.object(
                bench.subprocess, 'run', return_value=subprocess.CompletedProcess([], 1, '', 'display failed')):
            with self.assertRaises(RuntimeError):
                bench.check_display()

    def test_b_mocked_gui_exercises_actual_display_calls(self):
        with patch.object(bench, 'check_display'), patch.object(bench.cv2, 'namedWindow'), \
                patch.object(bench.cv2, 'imshow') as show, patch.object(bench.cv2, 'waitKey', return_value=-1), \
                patch.object(bench.cv2, 'getWindowProperty', return_value=1), \
                patch.object(bench.cv2, 'destroyWindow') as destroy:
            result = self.run_mode('B')
        self.assertEqual(show.call_count, result['processed_frames'])
        self.assertTrue(result['display_verified'])
        destroy.assert_called_once()

    def test_capture_failure_is_error_and_resources_close(self):
        self.source.read = lambda: (False, None)
        with self.assertRaises(RuntimeError): self.run_mode('A')
        self.assertTrue(self.source.released)
        self.assertEqual(json.loads((self.root / 'A' / 'summary.json').read_text())['status'], 'error')

    def test_writer_close_failure_cannot_report_success(self):
        writer = SyntheticWriter()
        def fail(): raise RuntimeError('encoder failed')
        writer.release = fail
        with patch.object(bench, 'AsyncVideoWriter', return_value=writer):
            with self.assertRaises(RuntimeError): self.run_mode('C')
        result = json.loads((self.root / 'C' / 'summary.json').read_text())
        self.assertEqual(result['status'], 'error')
        self.assertFalse(result['recording']['complete'])
        self.assertTrue(self.source.released)

    def test_counting_camera_preserves_parent_latest_frame_queue(self):
        # 真正运行父Camera的线程和队列；仅替换设备read，不复制采集模块。
        gate, ready = threading.Event(), threading.Event()
        class Capture(object):
            count = 0
            released = False
            def isOpened(self): return True
            def set(self, *args): return True
            def get(self, prop):
                return {bench.cv2.CAP_PROP_FRAME_WIDTH: 320,
                        bench.cv2.CAP_PROP_FRAME_HEIGHT: 180,
                        bench.cv2.CAP_PROP_FPS: 30}.get(prop, 0)
            def read(self):
                gate.wait(1)
                if self.count == 12:
                    ready.set()
                    return False, None
                self.count += 1
                return True, np.full((180, 320, 3), self.count, np.uint8)
            def release(self): self.released = True
        cap = Capture()
        with patch.object(bench.cv2, 'VideoCapture', return_value=cap):
            camera = bench.CountingCamera({})
            try:
                camera.counter.start(time.perf_counter())
                gate.set()
                self.assertTrue(ready.wait(2))
                ok, frame = camera.read()
                self.assertTrue(ok)
                self.assertEqual(int(frame[0, 0, 0]), 12)
                self.assertEqual(camera.counter.stop()['frames'], 12)
                self.assertIsNotNone(camera.last_ts)
                self.assertFalse(camera.timestamp_verified)
            finally:
                gate.set()
                camera.release()
        self.assertTrue(cap.released)

    def test_async_encoder_failure_during_submission_is_not_queue_drop(self):
        writer = SyntheticWriter()
        def fail(frame, timestamp=None):
            writer.error = RuntimeError('broken pipe')
            return False
        writer.write = fail
        with patch.object(bench, 'AsyncVideoWriter', return_value=writer):
            with self.assertRaisesRegex(RuntimeError, 'broken pipe'):
                self.run_mode('C')
        result = json.loads((self.root / 'C' / 'summary.json').read_text())
        self.assertEqual(result['status'], 'error')
        self.assertTrue(self.source.released)

    def test_output_latency_includes_overlay_work(self):
        real_overlay = bench.draw_overlay
        def slow_overlay(*args, **kwargs):
            time.sleep(.01)
            return real_overlay(*args, **kwargs)
        with patch.object(bench, 'AsyncVideoWriter', return_value=SyntheticWriter()), \
                patch.object(bench, 'draw_overlay', side_effect=slow_overlay):
            self.run_mode('C')
        for row in self.rows('C'):
            self.assertGreaterEqual(float(row['capture_return_to_output_ms']) - float(row['algo_ms']), 9.)

    def test_negotiated_fallback_is_error_before_processing(self):
        for change, value in [('width', 640), ('height', 480),
                              ('fourcc', bench.cv2.VideoWriter_fourcc(*'YUYV'))]:
            source = SyntheticCamera({})
            setattr(source, change, value)
            destination = self.root / change
            with patch.object(bench, 'CountingCamera', return_value=source):
                with self.assertRaises(RuntimeError):
                    bench.run_benchmark('A', .01, self.config, destination)
            result = json.loads((destination / 'summary.json').read_text())
            self.assertFalse(result['camera_format']['verified'])
            self.assertEqual(result['processed_frames'], 0)
            self.assertTrue(source.released)

    def test_provenance_and_recording_format_are_saved(self):
        with patch.object(bench, 'AsyncVideoWriter', return_value=SyntheticWriter()):
            result = self.run_mode('C')
        self.assertEqual(result['camera_format']['actual'],
                         dict(width=1280, height=720, fourcc='MJPG'))
        self.assertTrue(result['camera_format']['verified'])
        self.assertEqual([result['recording'][key] for key in ('width', 'height', 'container_fps')],
                         [1280, 720, 30.])
        import hashlib
        for name in ['rail_detector.py', 'ball_detector.py', 'ball_tracker.py',
                     'local_vision.py', 'camera.py', 'video_writer.py']:
            path = Path(bench.__file__).resolve().parent / name
            self.assertEqual(result['source_sha256'][name], hashlib.sha256(path.read_bytes()).hexdigest())

    def test_cli_defaults_to_identity_safe_config(self):
        with patch.object(bench, 'run_benchmark', return_value={'status': 'complete'}) as run:
            self.assertEqual(bench.main(['--mode', 'A', '--output', str(self.root / 'cli')]), 0)
        self.assertEqual(Path(run.call_args[0][2]).name, 'config_local_roi_20261116.json')

    def test_valid_intervals_exclude_boundary_invalid_runs(self):
        # 相邻有效观测之间允许夹无效帧；首尾无效帧只能计入边界段，不能伪造间隔。
        self.assertTrue(hasattr(bench, 'measurement_summary'))
        stats = bench.measurement_summary([1., 1.1, 1.2, 1.4, 1.5],
                                          [False, True, False, True, False])
        self.assertEqual(stats['valid_frames'], 2)
        self.assertEqual(stats['leading_invalid_run_frames'], 1)
        self.assertEqual(stats['trailing_invalid_run_frames'], 1)
        self.assertEqual(stats['interval_ms']['count'], 1)
        self.assertAlmostEqual(stats['interval_ms']['max'], 300.)
        self.assertAlmostEqual(stats['interval_ms']['mean'], 300.)
        for flags in ([False] * 3, [False, True, False]):
            result = bench.measurement_summary([1., 2., 3.], flags)
            self.assertIsNone(result['interval_ms'])
        self.assertEqual(bench.measurement_summary([1., 2.], [False, False])[
            'leading_invalid_run_frames'], 2)
        self.assertIsNone(bench.measurement_summary([], [])['interval_ms'])

    def test_raw_disabled_keeps_pixels_and_reports_no_centimeters(self):
        # 无几何配置必须走真实raw默认类；坐标禁用不能把像素有效数清零。
        from camera_geometry import CameraGeometry
        track = dict(x_px=321.25, valid=True, predicted=False, state='TRACKING',
                     confidence=.9, method='relative_cap', reason='tracked')
        tracker = SimpleNamespace(update=lambda *args: dict(track))
        with patch.object(bench, 'CameraGeometry', wraps=CameraGeometry, create=True) as geo, \
                patch.object(bench, 'BallTracker', return_value=tracker):
            result = self.run_mode('A')
        geo.assert_called_once_with({}, 1280, 720)
        self.assertEqual(result['pixel_valid_frames'], result['processed_frames'])
        self.assertEqual(result['reference_valid_frames'], 0)
        self.assertEqual(result['cm_valid_frames'], 0)
        self.assertIsNone(result['valid_measurements']['cm']['interval_ms'])
        self.assertFalse(result['timestamp_verified'])
        self.assertFalse(self.source.__dict__.get('timestamp_verified', False))
        for row in self.rows('A'):
            self.assertEqual(float(row['x_px']), 321.25)
            self.assertEqual((row['pixel_valid'], row['cm_valid'], row['x_cm']), ('1', '0', ''))
            self.assertEqual(row['camera_geometry_mode'], 'raw')
            self.assertEqual(row['image_space'], 'raw_image')
            self.assertEqual(row['timestamp_verified'], '0')

    def test_geometry_coordinate_order_fields_binding_and_validity_counts(self):
        # 只替换设备/算法边界；通过帧内容和调用序列证明几何预处理未被跳过。
        events = []
        geometry = SimpleNamespace(mode='image', image_space='undistorted_image',
                                   context={'mode': 'image'})
        def prepare(frame):
            events.append('geometry')
            time.sleep(.002)
            return np.full_like(frame, 7)
        geometry.prepare_frame = prepare
        rail = SimpleNamespace(valid=True, geometry_valid=True, identity_valid=True, reason='ok')
        def detect_rail(gray, force=False):
            self.assertTrue(np.all(gray == 7)); self.assertTrue(force)
            events.append('reference'); time.sleep(.002)
            return rail
        candidates = [object()]
        def detect_ball(frame, gray, r):
            self.assertTrue(np.all(frame == 7)); self.assertIs(r, rail)
            events.append('ball'); return candidates
        track = dict(x_px=333.25, valid=True, predicted=False, state='TRACKING',
                     confidence=.9, method='relative_cap', reason='tracked')
        def update(cs, timestamp):
            self.assertIs(cs, candidates); events.append('tracker'); return dict(track)
        rc = SimpleNamespace(enabled=True, binding={'schema': 'test-binding'}, binding_id='test-id')
        def process(out, cs, r, frame_id):
            events.append('coordinate'); time.sleep(.002)
            self.assertEqual(frame_id, events.count('coordinate')-1)
            self.assertIs(cs, candidates); self.assertIs(r, rail)
            pixel = frame_id in (1, 2, 3)
            ref = frame_id in (0, 1, 3)
            cm = frame_id == 3
            return dict(pixel_valid=pixel, reference_valid=ref, calibration_valid=True,
                        cm_valid=cm, feedback_valid=False, measurement_x_px=333.25 if pixel else None,
                        measurement_y_px=201.5 if pixel else None, reference_x_px=320. if ref else None,
                        reference_y_px=200. if ref else None, rail_s_px=12.5 if pixel and ref else None,
                        rail_x_cm=1.25 if cm else None, rail_angle_deg=2., reference_rotation_deg=1.,
                        reference_scale=1.01, reference_residual_px=.2, origin_x_px=300., origin_y_px=200.,
                        rail_binding_id='test-id', measurement_image_space='undistorted_image',
                        rail_coordinate_version='rail-reference-v1', rail_coordinate_status='ok' if cm else 'unavailable')
        rc.process = process
        with patch.object(bench, 'CountingCamera', return_value=self.source), \
                patch.object(bench, 'CameraGeometry', return_value=geometry, create=True), \
                patch.object(bench, 'RailCoordinates', return_value=rc, create=True), \
                patch.object(bench, 'RailDetector', return_value=SimpleNamespace(detect=detect_rail)), \
                patch.object(bench, 'BallDetector', return_value=SimpleNamespace(
                    detect=detect_ball, last_reason='candidates', last_roi=(200., 440.))), \
                patch.object(bench, 'BallTracker', return_value=SimpleNamespace(update=update)), \
                patch.object(bench, 'draw_overlay', side_effect=lambda frame, *a, **kw: frame), \
                patch.object(bench, 'check_display'), patch.object(bench.cv2, 'namedWindow'), \
                patch.object(bench.cv2, 'imshow'), patch.object(bench.cv2, 'destroyWindow'), \
                patch.object(bench.cv2, 'getWindowProperty', return_value=1), \
                patch.object(bench.cv2, 'waitKey', side_effect=[-1]*5+[ord('q')]):
            result = bench.run_benchmark('B', 5., self.config, self.root/'B')
        self.assertEqual(events, ['geometry', 'reference', 'ball', 'tracker', 'coordinate']*6)
        self.assertEqual((result['pixel_valid_frames'], result['reference_valid_frames'],
                          result['cm_valid_frames']), (3, 3, 1))
        self.assertFalse(result['uart_enabled']); self.assertFalse(result['timestamp_verified'])
        self.assertEqual(result['valid_measurements']['pixel']['leading_invalid_run_frames'], 1)
        self.assertEqual(result['valid_measurements']['pixel']['trailing_invalid_run_frames'], 2)
        self.assertEqual(result['valid_measurements']['pixel']['interval_ms']['count'], 2)
        self.assertIsNone(result['valid_measurements']['cm']['interval_ms'])
        for key in ('geometry_ms', 'reference_match_ms', 'coordinate_ms'):
            self.assertGreaterEqual(result[key]['mean'], 1.)
            self.assertIsNotNone(result[key]['max'])
        rows = self.rows('B')
        self.assertEqual(rows[3]['rail_x_cm'], '1.25')
        self.assertEqual(rows[3]['x_cm'], '1.25')
        self.assertEqual(rows[3]['measurement_y_px'], '201.5')
        self.assertEqual(rows[3]['reference_x_px'], '320.0')
        for row in rows:
            self.assertEqual(row['x_px'], '333.25')
            self.assertEqual(row['uart_enabled'], '0')
            self.assertEqual(row['rail_binding_id'], 'test-id')
            self.assertEqual(row['timestamp_verified'], '0')
        binding = json.loads((self.root/'B'/'frames.binding.json').read_text())
        self.assertEqual(binding, {'binding': rc.binding, 'rail_binding_id': rc.binding_id})
        for name in ('camera_geometry.py', 'rail_coordinates.py', 'rail_calibration.py'):
            self.assertIn(name, result['source_sha256'])

    def test_invalid_duration_and_existing_output_are_rejected(self):
        for duration in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                bench.run_benchmark('A', duration, self.config, self.root / 'invalid')
        with self.assertRaises(FileExistsError):
            bench.run_benchmark('A', 1, self.config, self.root)


if __name__ == '__main__':
    unittest.main()
