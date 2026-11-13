# -*- coding: utf-8 -*-
"""Bounded asynchronous BGR -> H.264 recording (Linux, Python 3.6+).

write(frame, timestamp=None) returns whether the frame was queued; a full
queue drops the NEW frame immediately. Encoding/filter threads are limited.
frames counts accepted frames, written counts frames delivered to ffmpeg
(not confirmed encoded frames). error exposes asynchronous failures;
release() drains with a deadline, reaps ffmpeg and raises on any failure.

<path>.timestamps.jsonl records every valid submission, including drops.
Caller timestamps are seconds in the caller's clock; omitted timestamps use
monotonic submission time, NOT camera capture time. frame_index maps accepted
frames to CFR video indexes only if the final summary says complete=true.
No resampling is performed: index / fps is NOT elapsed real time.
Sidecar writes are buffered local file I/O; they are not hard real-time and
are flushed on release (a crash may lose buffered metadata).
"""
import fcntl
import json
import math
import os
import queue
import select
import subprocess
import tempfile
import threading
import time

import numpy as np


class AsyncVideoWriter(object):
    def __init__(self, path, width, height, fps,
                 bitrate="4M", queue_size=4, write_timeout=2.0,
                 close_timeout=5.0):
        self.path = os.fspath(path)
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.write_timeout = float(write_timeout)
        self.close_timeout = float(close_timeout)
        if (self.width <= 0 or self.height <= 0 or
                self.width % 2 or self.height % 2):
            raise ValueError('yuv420p requires positive even dimensions')
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.fps, self.write_timeout, self.close_timeout)):
            raise ValueError('fps and timeouts must be finite and positive')
        self.error = None
        self.frames = 0
        self.written = 0
        self.dropped = 0
        self._submitted = 0
        self._closed = False
        self._released = False
        self._lock = threading.Lock()
        self._release_lock = threading.Lock()
        self._stop = threading.Event()
        self._abort = threading.Event()
        self._q = queue.Queue(maxsize=max(1, int(queue_size)))
        self.sidecar_path = self.path + '.timestamps.jsonl'
        self._sidecar = open(self.sidecar_path, 'w', encoding='utf-8')
        self._stderr = None
        self.proc = None
        cmd = [
            'ffmpeg', '-y', '-loglevel', 'error',
            '-filter_threads', '1', '-filter_complex_threads', '1',
            '-f', 'rawvideo', '-pix_fmt', 'bgr24',
            '-s', '%dx%d' % (self.width, self.height),
            '-r', '%.9g' % self.fps, '-threads', '1', '-i', '-', '-an',
            '-c:v', 'libx264', '-preset', 'ultrafast',
            '-tune', 'zerolatency', '-b:v', bitrate,
            '-threads', '1', '-pix_fmt', 'yuv420p', self.path,
        ]
        try:
            # A file avoids filling a stderr pipe and deadlocking the encoder.
            self._stderr = tempfile.TemporaryFile()
            self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                         stderr=self._stderr, bufsize=0)
            fd = self.proc.stdin.fileno()
            flags = fcntl.fcntl(fd, fcntl.F_GETFL)
            fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
            self._thread = threading.Thread(target=self._encode_loop, daemon=True)
            self._thread.start()
        except Exception:
            if self.proc is not None:
                self.proc.kill()
                self.proc.wait(timeout=1)
                self.proc.stdin.close()
            if self._stderr is not None:
                self._stderr.close()
            self._sidecar.close()
            raise

    def _fail(self, exc):
        if self.error is None:
            self.error = exc

    def _encode_loop(self):
        try:
            while not self._abort.is_set():
                try:
                    data = self._q.get(timeout=.05)
                except queue.Empty:
                    if self._stop.is_set():
                        break
                    if self.proc.poll() is not None:
                        raise RuntimeError('ffmpeg exited before release')
                    continue
                pending = memoryview(data)
                deadline = time.monotonic() + self.write_timeout
                fd = self.proc.stdin.fileno()
                while pending:
                    if self._abort.is_set():
                        raise RuntimeError('encoding cancelled during close')
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise RuntimeError('ffmpeg frame write timed out')
                    if not select.select([], [fd], [], min(.05, remaining))[1]:
                        continue
                    try:
                        count = os.write(fd, pending)
                    except (BlockingIOError, InterruptedError):
                        continue
                    if count <= 0:
                        raise RuntimeError('ffmpeg pipe made no progress')
                    pending = pending[count:]
                self.written += 1
        except Exception as exc:
            self._fail(exc)
        finally:
            # Only this worker owns stdin after construction.
            try:
                self.proc.stdin.close()
            except Exception as exc:
                self._fail(exc)

    def _record(self, row):
        try:
            self._sidecar.write(json.dumps(row, allow_nan=False) + '\n')
        except Exception as exc:
            self._fail(exc)
            raise RuntimeError('video timestamp sidecar failed: %s' % exc)

    def write(self, frame, timestamp=None):
        """Queue a copied uint8 HxWx3 BGR frame, or return False on overload/error.

        Invalid input raises ValueError; calls after release raise RuntimeError.
        Pass a capture timestamp explicitly to obtain capture-time metadata.
        """
        source = 'monotonic_submit' if timestamp is None else 'caller'
        ts = time.monotonic() if timestamp is None else float(timestamp)
        if not math.isfinite(ts):
            raise ValueError('timestamp must be finite seconds')
        arr = np.asarray(frame)
        if arr.dtype != np.uint8 or arr.shape != (self.height, self.width, 3):
            raise ValueError('frame must be uint8 BGR with configured dimensions')
        with self._lock:
            if self._closed:
                raise RuntimeError('video writer is closed')
            accepted = False
            index = None
            reason = 'encoder_error' if self.error is not None else 'queue_full'
            if self.error is None and self.proc.poll() is not None:
                self._fail(RuntimeError('ffmpeg exited before release'))
                reason = 'encoder_error'
            if self.error is None and not self._q.full():
                # Avoid copying a full image when it will simply be dropped.
                try:
                    self._q.put_nowait(arr.tobytes(order='C'))
                    accepted = True
                    index = self.frames
                    self.frames += 1
                    reason = 'accepted'
                except queue.Full:
                    pass
            if not accepted:
                self.dropped += 1
            self._record({'event': 'frame', 'submission_index': self._submitted,
                          'timestamp': ts, 'timestamp_source': source,
                          'accepted': accepted, 'reason': reason,
                          'frame_index': index})
            self._submitted += 1
            return accepted

    def release(self):
        """Drain within close_timeout, then kill/reap if needed (1s cleanup grace).

        Repeat calls are safe; a failed release keeps raising the same error.
        A successful summary is required before trusting frame_index mappings.
        """
        with self._release_lock:
            if not self._released:
                with self._lock:
                    self._closed = True
                    self._stop.set()
                deadline = time.monotonic() + self.close_timeout
                self._thread.join(max(0, deadline - time.monotonic()))
                if self._thread.is_alive():
                    self._fail(RuntimeError('video close timed out'))
                if self.error is not None:
                    self._abort.set()
                    if self.proc.poll() is None:
                        self.proc.kill()
                try:
                    self.proc.wait(timeout=max(0, deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    self._fail(RuntimeError('ffmpeg close timed out'))
                    self._abort.set()
                    self.proc.kill()
                # Bound forced cleanup independently of the drain budget.
                cleanup_deadline = time.monotonic() + 1.0
                try:
                    self.proc.wait(timeout=max(0, cleanup_deadline - time.monotonic()))
                except subprocess.TimeoutExpired as exc:
                    self._fail(exc)
                self._thread.join(max(0, cleanup_deadline - time.monotonic()))
                self._stderr.seek(0, os.SEEK_END)
                size = self._stderr.tell()
                self._stderr.seek(max(0, size - 8192))
                detail = self._stderr.read().decode('utf-8', errors='replace').strip()
                self._stderr.close()
                rc = self.proc.returncode
                if rc != 0:
                    self._fail(RuntimeError('ffmpeg exit status %s' % rc))
                if self.error is not None and detail:
                    self.error = RuntimeError('%s; ffmpeg: %s' % (self.error, detail))
                try:
                    self._record({'event': 'summary', 'accepted': self.frames,
                                  'dropped': self.dropped, 'written': self.written,
                                  'fps': self.fps, 'timeline': 'constant_fps',
                                  'returncode': rc, 'complete': self.error is None,
                                  'error': str(self.error) if self.error else None})
                    self._sidecar.flush()
                except Exception as exc:
                    self._fail(exc)
                finally:
                    try:
                        self._sidecar.close()
                    except Exception as exc:
                        self._fail(exc)
                    self._released = True
            if self.error is not None:
                raise RuntimeError('video encoding failed: %s' % self.error)
