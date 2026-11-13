"""Local subprocess/ffmpeg tests only; no camera or board access."""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

import numpy as np
from video_writer import AsyncVideoWriter


class WriterTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path = os.path.join(self.tmp.name, 'out.mp4')
        self.frame = np.zeros((240, 320, 3), dtype=np.uint8)
        self.popen = subprocess.Popen

    def child(self, script, **kwargs):
        def launch(cmd, **options):
            self.command = cmd
            return self.popen([sys.executable, '-c', script], **options)
        with mock.patch('video_writer.subprocess.Popen', side_effect=launch):
            writer = AsyncVideoWriter(self.path, 320, 240, 30, **kwargs)
        self.addCleanup(self.cleanup_writer, writer)
        return writer

    @staticmethod
    def cleanup_writer(writer):
        if writer.proc.poll() is None:
            writer.proc.kill()
        try:
            writer.release()
        except RuntimeError:
            pass

    def records(self):
        with open(self.path + '.timestamps.jsonl') as stream:
            return [json.loads(line) for line in stream]

    def test_encoder_and_filter_threads_are_limited(self):
        w = self.child('import sys; sys.stdin.buffer.read()')
        cmd = self.command
        self.assertIn('-filter_threads', cmd)
        self.assertEqual(cmd[cmd.index('-filter_threads') + 1], '1')
        self.assertEqual(cmd[cmd.index('-filter_complex_threads') + 1], '1')
        output = cmd[cmd.index('-i') + 2:]
        self.assertEqual(output[output.index('-threads') + 1], '1')
        w.release()

    def test_full_queue_never_waits_for_encoder_and_logs_drops(self):
        w = self.child('import time; time.sleep(60)', queue_size=1)
        done = threading.Event()
        results = []
        def produce():
            for ts in [10.0, 10.1, 10.9, 11.0, 11.1]:
                results.append(w.write(self.frame, timestamp=ts))
            done.set()
        t = threading.Thread(target=produce, daemon=True)
        t.start()
        self.assertTrue(done.wait(0.5), 'full queue blocked producer')
        self.assertIn(False, results)
        self.assertIn(True, results)
        self.cleanup_writer(w)
        rows = [r for r in self.records() if r['event'] == 'frame']
        self.assertEqual([r['timestamp'] for r in rows], [10.0, 10.1, 10.9, 11.0, 11.1])
        self.assertEqual([r['accepted'] for r in rows], results)
        self.assertEqual([r['frame_index'] for r in rows if r['accepted']], list(range(sum(results))))

    def test_stalled_pipe_times_out_and_release_reaps_child(self):
        w = self.child('import time; time.sleep(60)', write_timeout=.15, close_timeout=.4)
        self.assertTrue(w.write(self.frame))
        w._thread.join(1)
        self.assertIsNotNone(w.error)
        start = time.monotonic()
        with self.assertRaises(RuntimeError):
            w.release()
        self.assertLess(time.monotonic() - start, 1.5)
        self.assertIsNotNone(w.proc.poll())
        self.assertFalse(w._thread.is_alive())
        with self.assertRaises(RuntimeError):
            w.release()

    def test_close_while_queue_full_is_bounded(self):
        w = self.child('import time; time.sleep(60)', queue_size=1,
                       write_timeout=60, close_timeout=.2)
        for i in range(8):
            w.write(self.frame)
        start = time.monotonic()
        with self.assertRaises(RuntimeError):
            w.release()
        self.assertLess(time.monotonic() - start, 1.5)
        self.assertFalse(w._thread.is_alive())
        self.assertIsNotNone(w.proc.poll())

    def test_nonzero_exit_reports_stderr_and_summary(self):
        w = self.child("import sys; sys.stdin.buffer.read(); sys.stderr.write('encoder exploded'); sys.exit(7)")
        w.write(self.frame, timestamp=3.25)
        with self.assertRaisesRegex(RuntimeError, 'encoder exploded'):
            w.release()
        summary = self.records()[-1]
        self.assertFalse(summary['complete'])
        self.assertEqual(summary['returncode'], 7)

    def test_early_exit_is_visible_to_write_and_release(self):
        w = self.child("import sys; sys.stderr.write('early failure'); sys.exit(9)")
        w.proc.wait(timeout=2)
        self.assertFalse(w.write(self.frame, timestamp=7))
        self.assertIsNotNone(w.error)
        with self.assertRaisesRegex(RuntimeError, 'early failure'):
            w.release()
        row = self.records()[0]
        self.assertEqual(row['timestamp'], 7)
        self.assertEqual(row['reason'], 'encoder_error')
        self.assertFalse(row['accepted'])
        self.assertIsNone(row['frame_index'])

    def test_process_that_ignores_eof_is_killed(self):
        w = self.child('import sys, time; sys.stdin.buffer.read(); time.sleep(60)',
                       close_timeout=.2)
        start = time.monotonic()
        with self.assertRaises(RuntimeError):
            w.release()
        self.assertLess(time.monotonic() - start, 1.5)
        self.assertIsNotNone(w.proc.poll())
        self.assertFalse(w._thread.is_alive())

    def test_sidecar_failure_is_not_silent(self):
        w = self.child('import sys; sys.stdin.buffer.read()')
        w._sidecar.close()
        with self.assertRaisesRegex(RuntimeError, 'sidecar'):
            w.write(self.frame, timestamp=1)
        self.assertIsNotNone(w.error)
        with self.assertRaises(RuntimeError):
            w.release()
        self.assertIsNotNone(w.proc.poll())

    def test_spawn_failure_closes_sidecar(self):
        real_open = open
        opened = []
        def capture_open(*args, **kwargs):
            stream = real_open(*args, **kwargs)
            opened.append(stream)
            return stream
        with mock.patch('video_writer.open', side_effect=capture_open), \
                mock.patch('video_writer.subprocess.Popen', side_effect=OSError('missing ffmpeg')):
            with self.assertRaises(OSError):
                AsyncVideoWriter(self.path, 320, 240, 30)
        self.assertTrue(opened[0].closed)

    def test_default_timestamp_and_closed_write(self):
        w = self.child('import sys; sys.stdin.buffer.read()')
        before = time.monotonic()
        self.assertTrue(w.write(self.frame))
        after = time.monotonic()
        w.release()
        w.release()
        row = self.records()[0]
        self.assertLessEqual(before, row['timestamp'])
        self.assertLessEqual(row['timestamp'], after)
        self.assertEqual(row['timestamp_source'], 'monotonic_submit')
        with self.assertRaises(RuntimeError):
            w.write(self.frame)

    def test_bad_frames_and_timestamps_do_not_poison_stream(self):
        w = self.child('import sys; sys.stdin.buffer.read()')
        for frame in [self.frame[:2], self.frame.astype(np.float32)]:
            with self.assertRaises(ValueError):
                w.write(frame)
        with self.assertRaises(ValueError):
            w.write(self.frame, timestamp=float('nan'))
        self.assertTrue(w.write(self.frame, timestamp=0))
        w.release()
        self.assertEqual(w.frames, 1)

    @unittest.skipUnless(shutil.which('ffmpeg') and shutil.which('ffprobe'), 'ffmpeg required')
    def test_real_ffmpeg_drains_and_preserves_frame_order(self):
        w = AsyncVideoWriter(self.path, 320, 240, 25, queue_size=8)
        self.addCleanup(self.cleanup_writer, w)
        for value, ts in [(0, 10), (80, 10.04), (160, 11.5)]:
            self.frame[:] = value
            self.assertTrue(w.write(self.frame, timestamp=ts))
        w.release()
        info = json.loads(subprocess.check_output([
            'ffprobe', '-v', 'error', '-show_streams', '-of', 'json', self.path]))
        stream = info['streams'][0]
        self.assertEqual(stream['codec_name'], 'h264')
        self.assertEqual(stream['pix_fmt'], 'yuv420p')
        self.assertEqual(int(stream['nb_frames']), 3)
        data = subprocess.check_output(['ffmpeg', '-v', 'error', '-i', self.path,
                                       '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-'])
        frames = np.frombuffer(data, dtype=np.uint8).reshape(3, 240, 320, 3)
        np.testing.assert_allclose(frames.mean(axis=(1, 2, 3)), [0, 80, 160], atol=4)
        rows = self.records()
        self.assertEqual([r['timestamp'] for r in rows[:-1]], [10, 10.04, 11.5])
        self.assertTrue(rows[-1]['complete'])
        self.assertEqual(rows[-1]['written'], 3)


if __name__ == '__main__':
    unittest.main()
