# -*- coding: utf-8 -*-
"""摄像头与视频源封装。

所有采集都通过 OpenCV + V4L2,不做任何桌面截图。
"""
import json
import os
import queue
import re
import subprocess
import threading
import time

import cv2


def _probe_monotonic_soe(dev):
    """在 OpenCV 占用设备前独占采四帧，逐条确认驱动标注的时钟与曝光起点。"""
    device = "/dev/video%d" % dev if isinstance(dev, int) else dev
    command = ["v4l2-ctl", "--device", device,
               "--set-fmt-video=width=1280,height=720,pixelformat=MJPG",
               "--stream-mmap=2", "--stream-count=4", "--verbose"]
    try:
        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                universal_newlines=True, env=dict(os.environ, LC_ALL="C"),
                                timeout=4)
    except (OSError, subprocess.TimeoutExpired):
        return False
    if result.returncode:
        return False
    records = [line.strip() for line in result.stdout.splitlines()
               if line.lstrip().startswith("Timestamp:")]
    if len(records) != 4:
        return False
    previous = 0.0
    for line in records:
        match = re.fullmatch(r"Timestamp:\s*([0-9]+(?:\.[0-9]+)?)s\s*"
                             r"\(Monotonic,\s*Start-of-Exposure\)", line)
        if not match:
            return False
        current = float(match.group(1))
        if not previous < current:
            return False
        previous = current
    return True


class Camera(object):
    """USB 摄像头封装(OpenCV + V4L2)。

    采集在独立线程中进行, 队列只保留最新一帧。处理慢于采集时会自动
    跳过旧帧, 避免画面/坐标滞后累积。
    """

    def __init__(self, cfg):
        self.cfg = cfg
        dev = cfg.get("device", 0)
        # 该机位在自动曝光下纸尺纹理可能变弱。先切手动，再设置曝光和增益，
        # 使时钟探查和正式采集从相同设置开始；历史配置沿用设备状态。
        # V4L2 曝光单位是 100 微秒。设置失败应在开始反馈前报告。
        exposure = cfg.get("manual_exposure")
        if exposure is not None:
            device = "/dev/video%d" % dev if isinstance(dev, int) else dev
            ctl = ["v4l2-ctl", "--device", device]
            subprocess.check_call(ctl + ["--set-ctrl=exposure_auto=1"])
            subprocess.check_call(ctl + ["--set-ctrl=exposure_absolute=%d,gain=%d" %
                                         (exposure["time_100us"], exposure["gain"])])

        # 仅正式采集模式可请求探查；run 返回后探查进程已释放设备。
        width, height = int(cfg.get("width", 1280)), int(cfg.get("height", 720))
        fourcc = cfg.get("fourcc", "MJPG")
        self._soe_source = (cfg.get("verify_v4l2_time") is True and
                            (width, height, fourcc) == (1280, 720, "MJPG") and
                            _probe_monotonic_soe(dev))
        # 新视频流的帧不会早于打开设备。这个下界也排除了从零开始的媒体时间。
        self._previous_capture_ts = time.perf_counter() if self._soe_source else None
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError("无法打开摄像头: %s" % dev)

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, int(cfg.get("fps", 30)))
        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE,
                         int(cfg.get("buffer_size", 4)))
        except Exception:
            pass

        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps_set = float(self.cap.get(cv2.CAP_PROP_FPS))

        if self._soe_source:
            self._soe_source = ((self.width, self.height) == (width, height) and
                                self.cap.get(cv2.CAP_PROP_FOURCC) == cv2.VideoWriter_fourcc(*fourcc))

        self._q = queue.Queue(maxsize=1)
        self._stop = threading.Event()
        # 公开字段描述最近一次消费的帧，初始化时尚无曝光时间。
        self.timestamp_verified = False
        self.last_capture_ts = None
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
            capture_ts = None
            if self._soe_source:
                # POS_MSEC 属于刚读出的缓冲区。保留每个有效样本的上界记录，
                # 即使单槽随后覆盖了它，也能识别下一帧回退、重复或异常值。
                try:
                    sample = self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
                    if self._previous_capture_ts < sample <= ts:
                        capture_ts = self._previous_capture_ts = sample
                except (cv2.error, TypeError, ValueError):
                    pass
            try:
                while True:
                    self._q.get_nowait()
            except queue.Empty:
                pass
            try:
                self._q.put_nowait((frame, ts, capture_ts))
            except queue.Full:
                pass

    def read(self, fresh=False):
        # 静态控制的参考匹配慢于单帧周期。只开始处理在本轮就绪后读完的图像，
        # 避免为队列里已经等了一帧的图像花费剩余的 100 ms 年龄预算。
        ready = time.perf_counter() if fresh else None
        try:
            frame, ts, capture_ts = self._q.get(timeout=3.0)
            if fresh and ts < ready:
                frame, ts, capture_ts = self._q.get(timeout=3.0)
        except queue.Empty:
            self.last_capture_ts = None
            self.timestamp_verified = False
            return False, None
        self.last_capture_ts = capture_ts
        self.timestamp_verified = capture_ts is not None
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
        # 公开字段描述最近一次消费的帧，初始化时尚无曝光时间。
        self.timestamp_verified = False
        self.last_capture_ts = None
        self.last_ts = None

    def read(self):
        ok, frame = self.cap.read()
        if ok:
            self.last_ts = time.perf_counter()
        return ok, frame

    def release(self):
        self.cap.release()
