# -*- coding: utf-8 -*-
"""用可控的 V4L2 输出和 OpenCV 设备验证单槽队列与时钟契约。"""
import queue
import subprocess
import unittest
from unittest.mock import MagicMock, patch

import cv2

from camera import Camera


SOE = ''.join('\tTimestamp: 98.%03ds (Monotonic, Start-of-Exposure)\n' % n
              for n in (801, 834, 867, 900))


class CameraTimestampTests(unittest.TestCase):
    def make_camera(self, cfg=None, output=SOE, error=None, width=1280):
        self.cap = MagicMock()
        self.position = 0.0
        self.clock = 99.0
        props = {cv2.CAP_PROP_FRAME_WIDTH: width, cv2.CAP_PROP_FRAME_HEIGHT: 720,
                 cv2.CAP_PROP_FOURCC: cv2.VideoWriter_fourcc(*'MJPG'),
                 cv2.CAP_PROP_FPS: 30}
        self.cap.get.side_effect = lambda key: (
            self.position if key == cv2.CAP_PROP_POS_MSEC else props[key])
        self.order = []
        self.probe_args = []

        def run(args, **kwargs):
            self.order.append('probe')
            self.probe_args = args
            self.assertGreater(kwargs['timeout'], 0)
            if error:
                raise error
            return subprocess.CompletedProcess(args, 0, output)

        def open_camera(*args):
            self.order.append('open')
            return self.cap

        # 控制生产循环一次处理的帧数，仍走实际的覆盖、入队和消费逻辑。
        with patch('subprocess.run', side_effect=run), \
                patch('camera.cv2.VideoCapture', side_effect=open_camera), \
                patch('camera.threading.Thread.start'), \
                patch('camera.time.perf_counter', return_value=self.clock):
            return Camera(dict(device=3, **(cfg if cfg is not None else
                                           dict(verify_v4l2_time=True, buffer_size=2))))

    def produce(self, cam, *frames):
        pending = iter(frames)
        cam._stop.clear()

        def read():
            frame, self.position, self.clock = next(pending)
            if frame == frames[-1][0]:
                cam._stop.set()
            return True, frame

        self.cap.read.side_effect = read
        with patch('camera.time.perf_counter', side_effect=lambda: self.clock):
            cam._capture_loop()

    def test_manual_exposure_applies_to_configured_camera_before_stream(self):
        # 在设备独占探查与正式打开前恢复实测曝光和增益，避免重插后自动曝光漂移。
        events = []
        with patch('camera.subprocess.check_call', side_effect=lambda cmd: events.append(cmd)):
            self.make_camera(dict(verify_v4l2_time=True,
                                  manual_exposure=dict(time_100us=300, gain=0)))
        self.assertEqual(events, [
            ['v4l2-ctl', '--device', '/dev/video3', '--set-ctrl=exposure_auto=1'],
            ['v4l2-ctl', '--device', '/dev/video3', '--set-ctrl=exposure_absolute=300,gain=0']])
        with patch('camera.subprocess.check_call') as set_controls:
            self.make_camera()
            set_controls.assert_not_called()

    def test_fresh_consumer_skips_already_queued_frame(self):
        # 处理耗时长于采集周期时，丢掉等待过的图像，不借用其曝光时间。
        cam = self.make_camera()
        from unittest.mock import Mock
        cam._q = Mock()
        cam._q.get.side_effect = [('old', 100.02, 99.99), ('new', 100.06, 100.03)]
        with patch('camera.time.perf_counter', return_value=100.04):
            self.assertEqual(cam.read(fresh=True), (True, 'new'))
        self.assertEqual((cam.last_ts, cam.last_capture_ts, cam.timestamp_verified),
                         (100.06, 100.03, True))

    def test_ready_single_slot_uses_soe_budget_and_preserves_clock_failure(self):
        # 读完时刻相同，只有经过验证且曝光年龄已耗尽预算的旧帧需要让位。
        from unittest.mock import Mock
        for soe, wanted in [(99.990, 'queued'), (99.960, 'next'), (None, 'queued')]:
            cam = self.make_camera()
            cam._q = Mock()
            cam._q.get.side_effect = [('queued', 100.030, soe), ('next', 100.045, 100.001)]
            with patch('camera.time.perf_counter', return_value=100.040):
                self.assertEqual(cam.read(fresh=True, max_start_age_ms=60), (True, wanted))
            self.assertEqual(cam.timestamp_verified, soe is not None)
            self.assertEqual(cam.last_capture_ts, 100.001 if wanted == 'next' else soe)

    def test_exclusive_probe_then_publish_aged_frame(self):
        cam = self.make_camera()
        self.assertEqual(self.order, ['probe', 'open'])
        self.assertIn('/dev/video3', self.probe_args)
        self.assertIn('--set-fmt-video=width=1280,height=720,pixelformat=MJPG', self.probe_args)
        self.assertIn('--stream-mmap=2', self.probe_args)
        self.assertIn('--stream-count=4', self.probe_args)
        self.cap.set.assert_any_call(cv2.CAP_PROP_BUFFERSIZE, 2)
        self.produce(cam, ('old', 99801., 100.))
        self.assertFalse(cam.timestamp_verified)
        self.assertEqual(cam.read(), (True, 'old'))
        self.assertTrue(cam.timestamp_verified)
        self.assertAlmostEqual(cam.last_ts - cam.last_capture_ts, .199)

    def test_replaced_frame_advances_watermark_and_keeps_metadata_paired(self):
        cam = self.make_camera()
        self.produce(cam, ('first', 99600., 100.))
        cam.read()
        self.assertTrue(cam.timestamp_verified)
        self.produce(cam, ('dropped', 99700., 100.1), ('new', 99650., 100.2))
        self.assertEqual(cam.last_capture_ts, 99.6)
        self.assertEqual(cam.read(), (True, 'new'))
        self.assertEqual(cam.last_ts, 100.2)
        self.assertIsNone(cam.last_capture_ts)
        self.assertFalse(cam.timestamp_verified)

    def test_bad_samples_and_timeout_clear_verified_state(self):
        cam = self.make_camera()
        self.produce(cam, ('valid', 99800., 100.))
        cam.read()
        self.assertTrue(cam.timestamp_verified)
        for pos in (99800., 99790., 0., -1., float('nan'), float('inf'), 100001.):
            with self.subTest(pos=pos):
                self.produce(cam, ('bad', pos, 100.))
                self.assertEqual(cam.read(), (True, 'bad'))
                self.assertFalse(cam.timestamp_verified)
                self.assertIsNone(cam.last_capture_ts)
        self.produce(cam, ('good', 99900., 100.))
        cam.read()
        self.assertTrue(cam.timestamp_verified)
        with patch.object(cam._q, 'get', side_effect=queue.Empty):
            self.assertEqual(cam.read(), (False, None))
        self.assertFalse(cam.timestamp_verified)
        self.assertIsNone(cam.last_capture_ts)
        self.assertEqual(cam.last_ts, 100.)

    def test_unproven_source_or_mode_still_returns_images(self):
        for options in (dict(output=''), dict(output=SOE.splitlines()[0]),
                        dict(output=SOE.replace('Monotonic', 'Unknown', 1)),
                        dict(output=SOE.replace('Start-of-Exposure', 'End-of-Frame', 1)),
                        dict(error=FileNotFoundError()),
                        dict(error=subprocess.TimeoutExpired('v4l2-ctl', 3)),
                        dict(width=640), dict(cfg={})):
            with self.subTest(options=options):
                cam = self.make_camera(**options)
                if 'cfg' in options:
                    self.assertEqual(self.order, ['open'])
                self.produce(cam, ('image', 99800., 100.))
                self.assertEqual(cam.read(), (True, 'image'))
                self.assertEqual(cam.last_ts, 100.)
                self.assertFalse(cam.timestamp_verified)
                self.assertIsNone(cam.last_capture_ts)
