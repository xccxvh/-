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

    def test_invalid_duration_and_existing_output_are_rejected(self):
        for duration in (0, -1, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                bench.run_benchmark('A', duration, self.config, self.root / 'invalid')
        with self.assertRaises(FileExistsError):
            bench.run_benchmark('A', 1, self.config, self.root)


if __name__ == '__main__':
    unittest.main()
