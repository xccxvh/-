# -*- coding: utf-8 -*-
"""摄像头与视频源封装。

所有采集都通过 OpenCV + V4L2,不做任何桌面截图。
"""
import json
import os
import queue
import threading
import time

import cv2


class Camera(object):
    """USB 摄像头封装(OpenCV + V4L2)。

    采集在独立线程中进行, 队列只保留最新一帧。处理慢于采集时会自动
    跳过旧帧, 避免画面/坐标滞后累积。
    """

    def __init__(self, cfg):
        self.cfg = cfg
        dev = cfg.get("device", 0)
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError("无法打开摄像头: %s" % dev)

        fourcc = cfg.get("fourcc", "MJPG")
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg.get("width", 1280)))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg.get("height", 720)))
        self.cap.set(cv2.CAP_PROP_FPS, int(cfg.get("fps", 30)))
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE,
                         int(cfg.get("buffer_size", 4)))
        except Exception:
            pass

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps_set = float(self.cap.get(cv2.CAP_PROP_FPS))

        self._q = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        # OpenCV does not expose the sensor/V4L2 capture timestamp here.
        self.timestamp_verified = False
        self.last_ts = None
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def _capture_loop(self):
        while not self._stop.is_set():
            ok, frame = self.cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            ts = time.perf_counter()
            try:
                while True:
                    self._q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._q.put_nowait((frame, ts))
            except queue.Full:
                pass

    def read(self):
        try:
            frame, ts = self._q.get(timeout=3.0)
        except queue.Empty:
            return False, None
        self.last_ts = ts
        return True, frame

    def release(self):
        self._stop.set()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self.cap.release()


class VideoFile(object):
    """离线视频源,接口与 Camera 保持一致。"""

    def __init__(self, path):
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise RuntimeError("无法打开视频: %s" % path)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps_set = float(self.cap.get(cv2.CAP_PROP_FPS))
        self.header_fps = self.fps_set
        # 摄像头 MJPG 封装时基可能不准(例如实际 19 fps 写成 60 fps),
        # capture_raw.py 会写同名 .json, 记录实测平均帧率。
        sidecar = os.path.splitext(path)[0] + ".json"
        if os.path.exists(sidecar):
            try:
                with open(sidecar, "r") as f:
                    data = json.load(f)
                actual = float(data.get("actual_fps", 0))
                if actual > 1.0:
                    self.fps_set = actual
            except Exception:
                pass
        # OpenCV does not expose the sensor/V4L2 capture timestamp here.
        self.timestamp_verified = False
        self.last_ts = None

    def read(self):
        ok, frame = self.cap.read()
        if ok:
            self.last_ts = time.perf_counter()
        return ok, frame

    def release(self):
        self.cap.release()
