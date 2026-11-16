#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""实时像素基准：A纯处理；B真实显示；C无显示、全尺寸标注异步录像。

例：python3 benchmark_live.py --mode C --duration 60 --config config_local_roi_20261116.json
    --output /tmp/benchmark_C
输出目录必须不存在，包含 frames.csv、summary.json；C另含 annotated.mp4 及时间旁文件。
不加载标定、不开放UART；cm_valid恒假。所有 perf_counter 时间以秒为单位，耗时以ms为单位。
capture_return 是 OpenCV read 返回时刻，不是传感器曝光时刻；output 是显示调用/
录像入队返回，不代表屏幕呈现或编码完成。FPS统计使用实测运行窗口，清理耗时另列。
"""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

from camera import Camera
from rail_detector import RailDetector
from ball_detector import BallDetector
from ball_tracker import BallTracker
from video_writer import AsyncVideoWriter
from visualization import draw_overlay


class CaptureCounter(object):
    """只统计测量窗口内成功采集；不依赖处理线程看到的帧数。"""
    def __init__(self):
        self.lock = threading.Lock()
        self.active = False
        self.count = 0
        self.first = self.last = None
        self.begin = None

    def start(self, begin):
        with self.lock:
            self.begin = begin
            self.count = 0
            self.first = self.last = None
            self.active = True

    def record(self, timestamp):
        with self.lock:
            if self.active and timestamp >= self.begin:
                self.count += 1
                if self.first is None:
                    self.first = timestamp
                self.last = timestamp

    def stop(self, end=None):
        with self.lock:
            if end is None:
                end = time.perf_counter()
            self.active = False
            elapsed = max(0., end - self.begin) if self.begin is not None else 0.
            span = self.last - self.first if self.count > 1 else 0.
            return dict(frames=self.count, elapsed_s=elapsed,
                        fps=self.count / elapsed if elapsed else 0.,
                        interval_fps=(self.count - 1) / span if span > 0 else None,
                        first_return_s=self.first, last_return_s=self.last)


class CountingCapture(object):
    """包装一次read，不缓存/复制帧，也不修改原Camera队列或时间戳。"""
    def __init__(self, capture, counter):
        self.capture, self.counter = capture, counter

    def read(self):
        ok, frame = self.capture.read()
        if ok:
            self.counter.record(time.perf_counter())
        return ok, frame

    def __getattr__(self, name):
        return getattr(self.capture, name)


class CountingCamera(Camera):
    def __init__(self, cfg):
        self.counter = CaptureCounter()
        super(CountingCamera, self).__init__(cfg)

    def _capture_loop(self):
        # 父构造器已经完成cap配置；线程内先装计数代理，再原样运行父采集循环。
        self.cap = CountingCapture(self.cap, self.counter)
        super(CountingCamera, self)._capture_loop()


def check_display():
    """先在子进程实测HighGUI，隔离Qt无显示时可能直接abort的行为。"""
    if (not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')) or
            os.environ.get('QT_QPA_PLATFORM', '').lower() in ('offscreen', 'minimal')):
        raise RuntimeError('mode B requires an actual display; headless/offscreen is not allowed')
    probe = '''import cv2, numpy as np
cv2.namedWindow('benchmark-display-probe', cv2.WINDOW_NORMAL)
cv2.imshow('benchmark-display-probe', np.zeros((64,128,3), np.uint8))
cv2.waitKey(100)
visible = cv2.getWindowProperty('benchmark-display-probe', cv2.WND_PROP_VISIBLE)
cv2.destroyAllWindows()
raise SystemExit(0 if visible >= 1 else 2)
'''
    try:
        result = subprocess.run([sys.executable, '-B', '-c', probe],
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                universal_newlines=True, timeout=10)
    except subprocess.TimeoutExpired:
        raise RuntimeError('mode B display probe timed out')
    if result.returncode:
        raise RuntimeError('mode B display unavailable: ' + result.stderr[-1500:])


def rss_mib(pid=None):
    # Linux /proc resident pages是当前RSS；不将ru_maxrss误写成当前内存。
    try:
        path = '/proc/{}/statm'.format(pid if pid is not None else 'self')
        with open(path) as stream:
            pages = int(stream.read().split()[1])
        return pages * os.sysconf('SC_PAGE_SIZE') / (1024. * 1024.)
    except (OSError, ValueError, IndexError):
        return None


def distribution(values):
    return dict(mean=float(np.mean(values)) if values else None,
                p95=float(np.percentile(values, 95)) if values else None)


def children_cpu_s():
    usage = resource.getrusage(resource.RUSAGE_CHILDREN)
    return usage.ru_utime + usage.ru_stime


FIELDS = ['frame', 'capture_return_s', 'result_ready_s', 'output_return_s',
          'queue_wait_ms', 'algo_ms', 'capture_return_to_result_ms',
          'capture_return_to_output_ms', 'draw_ms', 'display_ms', 'record_submit_ms',
          'x_px', 'valid', 'state', 'predicted', 'confidence', 'method',
          'rail_valid', 'rail_geometry_valid', 'rail_identity_valid', 'rail_reason',
          'roi_min_px', 'roi_max_px', 'candidate_count', 'detector_reason', 'tracker_reason',
          'cm_valid', 'x_cm', 'cm_status', 'uart_enabled', 'record_accepted',
          'record_frames', 'record_dropped', 'process_cpu_pct', 'process_rss_mib', 'encoder_rss_mib']


def run_benchmark(mode, duration, config, output):
    if mode not in ('A', 'B', 'C') or not math.isfinite(duration) or duration <= 0:
        raise ValueError('mode must be A/B/C; duration must be finite positive seconds')
    config = Path(config).resolve()
    config_bytes = config.read_bytes()
    cfg = json.loads(config_bytes.decode('utf-8'))
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    source = writer = csv_file = None
    window = False
    display_verified = False
    recording_complete = False
    errors = []
    status = 'complete'
    count = 0
    begin = None
    capture_stats = dict(frames=0, elapsed_s=0., fps=0., interval_fps=None)
    algo_values, latency_values, rss_values, encoder_rss_values = [], [], [], []
    loop_cpu = 0.
    setup_start = time.perf_counter()
    total_cpu_start = time.process_time()
    child_start = children_cpu_s()
    loop_end = setup_start
    fps_header = None
    camera_format = dict(expected=dict(width=1280, height=720, fourcc='MJPG'),
                         configured={k: cfg['camera'].get(k, default) for k, default in
                                     [('width', 1280), ('height', 720), ('fourcc', 'MJPG')]},
                         actual=None, verified=False)
    record_format = dict(width=None, height=None, container_fps=None)
    # 配置SHA之外保留算法/采集/录像源码SHA，避免同配置不同实现混淆性能。
    source_root = Path(__file__).resolve().parent
    source_names = ['benchmark_live.py', 'rail_detector.py', 'ball_detector.py',
                    'ball_tracker.py', 'local_vision.py', 'camera.py', 'video_writer.py',
                    'visualization.py', 'ruler_reference.py', 'measurement_contract.py']
    source_hashes = {name: hashlib.sha256((source_root / name).read_bytes()).hexdigest()
                     for name in source_names}
    try:
        if mode == 'B':
            check_display()
            # 显示探测进程不计入编码器CPU。
            child_start = children_cpu_s()
            cv2.namedWindow('Ball benchmark', cv2.WINDOW_NORMAL)
            window = True
        source = CountingCamera(cfg['camera'])
        width, height = source.width, source.height
        # 必须检查设备实际协商结果，不能把低分辨率/YUYV回退算成目标MJPG基准。
        # 仅查询属性，不设置曝光、增益或其他相机控制。
        actual_fourcc = int(source.cap.get(cv2.CAP_PROP_FOURCC))
        camera_format['actual'] = dict(
            width=int(source.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            height=int(source.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            fourcc=''.join(chr((actual_fourcc >> (8 * i)) & 255) for i in range(4)))
        camera_format['verified'] = (camera_format['actual'] == camera_format['expected'] and
                                     (width, height) == (1280, 720))
        if not camera_format['verified']:
            raise RuntimeError('camera format mismatch: expected 1280x720 MJPG, negotiated {}'.format(
                camera_format['actual']))
        fps_header = source.fps_set if math.isfinite(source.fps_set) else None
        rail_detector = RailDetector(cfg['rail'], width, height)
        ball_cfg = dict(cfg['ball'])
        if 'ball_search' in cfg:
            ball_cfg['ball_search'] = cfg['ball_search']
        ball_detector = BallDetector(ball_cfg, width, height)
        tracker = BallTracker(cfg['tracking'], width, height)
        if mode == 'C':
            # 全尺寸、逐处理帧提交，保留原writer的有界队列/丢新帧行为。
            # CFR播放帧率仅是容器参数，不冒充实际采集fps。
            fps_out = fps_header if fps_header is not None and fps_header > 0 else 30.
            writer = AsyncVideoWriter(str(output / 'annotated.mp4'), width, height, fps_out)
            record_format = dict(width=width, height=height, container_fps=fps_out)
        csv_file = (output / 'frames.csv').open('w', newline='', encoding='utf-8')
        csv_writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
        csv_writer.writeheader()
        begin = time.perf_counter()
        cpu_begin = time.process_time()
        last_cpu, last_sample = cpu_begin, begin
        source.counter.start(begin)
        while time.perf_counter() - begin < duration:
            ok, frame = source.read()
            if not ok:
                raise RuntimeError('camera read failed or timed out')
            read_time = time.perf_counter()
            if frame.shape != (height, width, 3) or frame.dtype != np.uint8:
                raise RuntimeError('decoded camera frame does not match negotiated BGR dimensions')
            capture_time = source.last_ts
            if capture_time is None or not math.isfinite(capture_time) or capture_time > read_time:
                raise RuntimeError('invalid Camera.last_ts')
            if capture_time < begin:
                continue  # 跳过测量窗口开始前已经排队的帧，不改原队列。
            start_algo = time.perf_counter()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # 无标定/UART；遵守配置刷新策略。参考检测器会自行逐帧确认身份。
            rail = rail_detector.detect(gray)
            candidates = ball_detector.detect(frame, gray, rail)
            track = tracker.update(candidates, capture_time)
            track.update(x_cm=None, cm_valid=False, cm_status='benchmark_disabled')
            result_time = time.perf_counter()
            algo_ms = (result_time - start_algo) * 1000.
            draw_ms = display_ms = submit_ms = 0.
            accepted = None
            stop_requested = False
            if mode in ('B', 'C'):
                draw_start = time.perf_counter()
                vis = draw_overlay(frame, rail, candidates, track,
                                   count / max(result_time - begin, 1e-9), algo_ms,
                                   x_cm=None, calibrated=False)
                draw_ms = (time.perf_counter() - draw_start) * 1000.
                io_start = time.perf_counter()
                if mode == 'B':
                    cv2.imshow('Ball benchmark', vis)
                    key = cv2.waitKey(1) & 0xff
                    if cv2.getWindowProperty('Ball benchmark', cv2.WND_PROP_VISIBLE) < 1:
                        raise RuntimeError('mode B display window is unavailable or closed')
                    display_verified = True
                    stop_requested = key == ord('q')
                    display_ms = (time.perf_counter() - io_start) * 1000.
                else:
                    accepted = writer.write(vis, timestamp=capture_time)
                    submit_ms = (time.perf_counter() - io_start) * 1000.
                    if writer.error is not None:
                        raise RuntimeError('encoder failed: {}'.format(writer.error))
            output_time = time.perf_counter()
            latency_ms = (output_time - capture_time) * 1000.
            # 资源采样与CSV写入不计入本帧output延迟，但其成本包含在处理FPS内。
            sample_time, cpu_now = time.perf_counter(), time.process_time()
            cpu_pct = 100. * (cpu_now - last_cpu) / max(sample_time - last_sample, 1e-9)
            last_cpu, last_sample = cpu_now, sample_time
            memory = rss_mib()
            process = getattr(writer, 'proc', None)
            encoder_memory = rss_mib(process.pid) if process is not None else None
            if memory is not None:
                rss_values.append(memory)
            if encoder_memory is not None:
                encoder_rss_values.append(encoder_memory)
            roi = getattr(ball_detector, 'last_roi', None)
            csv_writer.writerow(dict(
                frame=count, capture_return_s=capture_time, result_ready_s=result_time,
                output_return_s=output_time, queue_wait_ms=(read_time-capture_time)*1000.,
                algo_ms=algo_ms, capture_return_to_result_ms=(result_time-capture_time)*1000.,
                capture_return_to_output_ms=latency_ms, draw_ms=draw_ms,
                display_ms=display_ms, record_submit_ms=submit_ms,
                x_px=track['x_px'], valid=int(track['valid']), state=track['state'],
                predicted=int(track['predicted']), confidence=track['confidence'], method=track['method'],
                rail_valid=int(rail.valid), rail_geometry_valid=int(rail.geometry_valid),
                rail_identity_valid=int(rail.identity_valid), rail_reason=rail.reason,
                roi_min_px=roi[0] if roi is not None else None,
                roi_max_px=roi[1] if roi is not None else None, candidate_count=len(candidates),
                detector_reason=ball_detector.last_reason, tracker_reason=track.get('reason', ''),
                cm_valid=0, x_cm=None, cm_status='benchmark_disabled', uart_enabled=0,
                record_accepted=int(accepted) if accepted is not None else None,
                record_frames=writer.frames if writer is not None else 0,
                record_dropped=writer.dropped if writer is not None else 0,
                process_cpu_pct=cpu_pct, process_rss_mib=memory, encoder_rss_mib=encoder_memory))
            count += 1
            algo_values.append(algo_ms)
            latency_values.append(latency_ms)
            if stop_requested:
                status = 'interrupted'
                break
        if count == 0:
            raise RuntimeError('no processed frames')
    except KeyboardInterrupt:
        status = 'interrupted'
    except Exception as exc:
        status = 'error'
        errors.append('{}: {}'.format(type(exc).__name__, exc))
    finally:
        if source is not None:
            capture_stats = source.counter.stop()
        loop_end = time.perf_counter()
        if begin is not None:
            loop_cpu = time.process_time() - cpu_begin
        # 即使编码器收尾失败，也必须释放摄像头/窗口，且不能输出成功汇总。
        for label, obj in [('camera', source), ('writer', writer), ('csv', csv_file)]:
            if obj is None:
                continue
            try:
                obj.close() if label == 'csv' else obj.release()
                if label == 'writer':
                    recording_complete = True
            except Exception as exc:
                errors.append('{} cleanup: {}'.format(label, exc))
                status = 'error'
        if window:
            try:
                cv2.destroyWindow('Ball benchmark')
            except Exception as exc:
                errors.append('display cleanup: {}'.format(exc))
                status = 'error'
    source_hashes_after = {name: hashlib.sha256((source_root / name).read_bytes()).hexdigest()
                           for name in source_names}
    if source_hashes_after != source_hashes:
        status = 'error'
        errors.append('source files changed during benchmark')
    end = time.perf_counter()
    elapsed = capture_stats['elapsed_s']
    child_cpu = max(0., children_cpu_s() - child_start)
    summary = dict(
        status=status, errors=errors, mode=mode, requested_duration_s=duration,
        elapsed_s=elapsed, cleanup_s=end-loop_end,
        duration_overrun_s=max(0., elapsed-duration),
        config=str(config), config_sha256=hashlib.sha256(config_bytes).hexdigest(),
        capture=capture_stats, camera_reported_fps=fps_header, camera_format=camera_format,
        source_sha256=source_hashes, source_sha256_after=source_hashes_after,
        sources_unchanged=source_hashes == source_hashes_after,
        processed_frames=count, processing_fps=count/elapsed if elapsed else 0.,
        algo_ms=distribution(algo_values), capture_return_to_output_ms=distribution(latency_values),
        process_cpu_pct=100.*loop_cpu/elapsed if elapsed else None,
        encoder_cpu_s=child_cpu if mode == 'C' else None,
        total_cpu_including_setup_cleanup_pct=100.*(time.process_time()-total_cpu_start+child_cpu)/max(end-setup_start,1e-9),
        process_rss_peak_mib=max(rss_values) if rss_values else None,
        encoder_rss_peak_mib=max(encoder_rss_values) if encoder_rss_values else None,
        display_verified=display_verified, uart_enabled=False, cm_valid=False,
        recording=dict(record_format, enabled=mode == 'C', complete=recording_complete if writer is not None else False,
                       frames=writer.frames if writer is not None else 0,
                       written=writer.written if writer is not None else 0,
                       dropped=writer.dropped if writer is not None else 0),
        runtime=dict(python=sys.version, opencv=cv2.__version__, numpy=np.__version__),
        semantics=dict(capture_fps='successful cap.read count / measured loop elapsed seconds, including overwritten frames',
                       processing_fps='completed CSV rows / same elapsed seconds, excludes setup/cleanup',
                       latency='Camera.last_ts (post-read, not exposure) to output call return; excludes CSV/resource sample; not display presentation or encoding completion',
                       algo='grayscale + rail + candidates + tracker; milliseconds',
                       cpu='100% = one logical core; process includes all its threads, excludes ffmpeg; encoder_cpu_s includes final drain after child reap',
                       rss='sampled resident memory peak in MiB (1024^2 bytes), parent and ffmpeg separate; not allocation total',
                       recording='frames accepted; written delivered to ffmpeg, not individually confirmed encoded; trust complete only after successful release; CFR is not actual capture FPS',
                       duration='checked between frames; blocking Camera.read or processing can overrun; actual elapsed and overrun reported'))
    (output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    if status == 'error':
        raise RuntimeError('; '.join(errors))
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', required=True, choices=['A', 'B', 'C'])
    parser.add_argument('--duration', type=float, default=60., help='measurement duration in seconds')
    parser.add_argument('--config', default=str(Path(__file__).resolve().parent / 'config_local_roi_20261116.json'))
    parser.add_argument('--output', required=True, help='new output directory (must not exist)')
    args = parser.parse_args(argv)
    try:
        result = run_benchmark(args.mode, args.duration, args.config, args.output)
    except (OSError, ValueError, RuntimeError) as exc:
        print('benchmark failed: {}'.format(exc), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 130 if result['status'] == 'interrupted' else 0


if __name__ == '__main__':
    sys.exit(main())
