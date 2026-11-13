# -*- coding: utf-8 -*-
"""钢珠识别主程序。

用法示例:
  # 实时显示(经 SSH X11 转发或本机显示器)
  OPENBLAS_CORETYPE=ARMV8 python3 main.py --display

  # 离线跑一段视频并输出标注视频 + CSV
  OPENBLAS_CORETYPE=ARMV8 python3 main.py --video recordings/xxx.avi \
      --record --csv

统一输出字段:
  timestamp, x_px, x_cm, valid, confidence, state
"""
import argparse
import csv
import json
import os
import time
import signal
import math
from contextlib import ExitStack

import cv2

from camera import Camera, VideoFile
from rail_detector import RailDetector
from ball_detector import BallDetector
from ball_tracker import BallTracker
from calibration import Calibration
from reference_preview import ReferencePreview
from measurement_contract import calibrated_method
from video_writer import AsyncVideoWriter
from recording_schedule import RecordingSchedule
from visualization import draw_overlay
from position_feedback import FeedbackEncoder, SerialFeedback

BASE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(BASE, "config.json")


def resolve(path):
    if path is None:
        return None
    if os.path.isabs(path):
        return path
    return os.path.join(BASE, path)


def load_config(path):
    with open(path, "r") as f:
        return json.load(f)


def _pct(values, q):
    """简单分位数, 避免为几行统计额外引入依赖。"""
    if not values:
        return 0.0
    s = sorted(values)
    k = int(round((len(s) - 1) * q))
    return s[max(0, min(k, len(s) - 1))]


def main():
    ap = argparse.ArgumentParser(description="钢珠识别系统")
    ap.add_argument("--config", default=DEFAULT_CONFIG)
    ap.add_argument("--video", default=None, help="离线视频路径; 不填则用摄像头")
    ap.add_argument("--display", action="store_true", help="实时显示画面")
    ap.add_argument("--record", action="store_true", help="保存带标注的视频")
    ap.add_argument("--csv", action="store_true", help="保存 CSV")
    ap.add_argument("--record-fps", type=float, default=None,
                    help="标注录像最高帧率；实时默认15，离线默认源帧率")
    ap.add_argument("--record-scale", type=float, default=None,
                    help="标注录像缩放；实时默认0.5，离线默认1；检测分辨率不变")
    ap.add_argument("--duration", type=float, default=0.0, help="运行秒数, 0=不限")
    ap.add_argument("--fps", type=float, default=0.0,
                    help="覆盖视频帧率(离线文件头帧率不准时使用)")
    ap.add_argument("--label", default="run", help="输出文件名标签")
    ap.add_argument("--calibration", default=None, help="标定文件路径")
    ap.add_argument("--reference-preview", default=None, help="实验性纸尺补偿厘米预览清单；禁止UART")
    ap.add_argument("--serial", default=None, help="显式启用 UART 监测，115200 8N1")
    ap.add_argument("--uart-enable-valid", action="store_true",
                    help="现场验证和接收端协议确认后才允许有效测量；默认只发送无效监测帧")
    args = ap.parse_args()
    if args.record_fps is not None and (not math.isfinite(args.record_fps) or args.record_fps <= 0):
        ap.error("--record-fps must be finite and positive")
    if args.record_scale is not None and (not math.isfinite(args.record_scale) or not 0 < args.record_scale <= 1):
        ap.error("--record-scale must be in (0, 1]")
    if args.reference_preview and args.serial:
        ap.error("reference preview cannot open UART")
    if args.video and args.serial:
        ap.error("offline replay cannot send UART measurements")
    if args.uart_enable_valid and not args.serial:
        ap.error("--uart-enable-valid requires --serial")
    def stop_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop_signal)
    signal.signal(signal.SIGHUP, stop_signal)

    cfg = load_config(resolve(args.config))
    recordings_dir = resolve(cfg.get("output", {}).get("recordings_dir", "recordings"))
    logs_dir = resolve(cfg.get("output", {}).get("logs_dir", "logs"))
    os.makedirs(recordings_dir, exist_ok=True)
    os.makedirs(logs_dir, exist_ok=True)

    calib_path = resolve(args.calibration or cfg.get("calibration", {}).get("file"))
    calib = Calibration(calib_path) if calib_path else None
    preview = ReferencePreview(resolve(args.reference_preview), cfg) if args.reference_preview else None

    with ExitStack() as resources:
        if args.video:
            source = VideoFile(resolve(args.video))
        else:
            source = Camera(cfg["camera"])
        if args.fps and args.fps > 0:
            source.fps_set = float(args.fps)
        resources.callback(source.release)
        W, H = source.width, source.height

        rail_det = RailDetector(cfg["rail"], W, H)
        ball_det = BallDetector(cfg["ball"], W, H)
        tracker = BallTracker(cfg["tracking"], W, H)

        # 标定只在"已拟合且采集分辨率与当前一致"时才可用。相机移动、变焦
        # 或改分辨率都会让旧标定失效, 此时 x_cm 必须输出空值。
        calib_ok = bool(calib is not None and calib.ready and
                        calib.resolution_ok(W, H))
        if calib is None or not calib.ready:
            calib_status = "not ready"
        elif not calib.resolution_ok(W, H):
            calib_status = "not ready (calibrated at %dx%d, current %dx%d)" % (
                calib.resolution[0], calib.resolution[1], W, H)
        else:
            calib_status = "ready (%dx%d)" % (W, H)

        stamp = time.strftime("%Y%m%d_%H%M%S")
        video_path = None
        writer = None
        record_schedule = None
        record_size = None
        if args.record:
            video_path = os.path.join(recordings_dir,
                                      "annotated_%s_%s.mp4" % (args.label, stamp))
            source_fps = source.fps_set if source.fps_set and source.fps_set > 1 else 30.0
            fps_out = min(source_fps, args.record_fps if args.record_fps is not None
                          else (source_fps if args.video else 15.0))
            scale = args.record_scale if args.record_scale is not None else (1.0 if args.video else .5)
            record_size = (max(2, int(W*scale)//2*2), max(2, int(H*scale)//2*2))
            record_schedule = RecordingSchedule(fps_out)
            writer = AsyncVideoWriter(video_path, record_size[0], record_size[1], fps_out)
            resources.callback(writer.release)

        csv_path = None
        csv_file = None
        csv_writer = None
        if args.csv:
            csv_path = os.path.join(logs_dir, "track_%s_%s.csv" % (args.label, stamp))
            csv_file = open(csv_path, "w", newline="")
            resources.callback(csv_file.close)
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow([
                "timestamp", "frame", "video_time_s", "x_px", "x_cm",
                "valid", "confidence", "state", "predicted", "method",
                "fps", "algo_ms", "wait_ms", "pipe_ms",
                "draw_ms", "write_ms", "frame_ms",
                "rail_slope", "rail_intercept", "rail_valid", "pose_ok",
                "capture_monotonic_ms", "feedback_valid", "feedback_reason",
                "cm_valid", "cm_status", "reference_x_px", "reference_ms",
            ])

        print("source: %s  %dx%d  fps=%.1f" % (
            args.video if args.video else cfg["camera"].get("device"),
            W, H, source.fps_set))
        print("calibration: %s" % calib_status)
        if preview is not None:
            print("reference cm PREVIEW: experimental, UART disabled")

        capture_origin = None
        frame_idx = 0
        fps_ema = 0.0
        t_start = time.perf_counter()
        t_prev = t_start
        state_counts = {"DETECTED": 0, "UNCERTAIN": 0, "LOST": 0}
        wait_ms_list = []
        algo_ms_list = []
        pipe_ms_list = []
        draw_ms_list = []
        write_ms_list = []
        frame_ms_list = []

        if args.display:
            resources.callback(cv2.destroyAllWindows)
        serial = None
        encoder = FeedbackEncoder()
        try:
            if args.serial:
                serial = SerialFeedback(args.serial)
            while True:
                ret, frame = source.read()
                if not ret:
                    break
                t_read = time.perf_counter()
                # Camera.last_ts 是采集线程拿到该帧的时间; 两者之差是帧在
                # 队列里等待处理的时间。VideoFile 没有实际排队, 差值约为 0。
                t_cap = getattr(source, "last_ts", None)
                wait_ms = ((t_read - t_cap) * 1000.0
                           if t_cap is not None else 0.0)
                t0 = time.perf_counter()
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                rail = rail_det.detect(gray, force=bool(args.serial or calib_ok or preview is not None))
                candidates = ball_det.detect(frame, gray, rail)
                # Replay speed must not change motion prediction. Live read
                # time is monotonic but still not a verified sensor timestamp.
                track_ts = (frame_idx / (source.fps_set if source.fps_set > 0 else 30.0)
                            if args.video else t_cap)
                if capture_origin is None:
                    capture_origin = track_ts
                out = tracker.update(candidates, track_ts)
                out["timestamp"] = time.time()

                x_cm = None
                pose_ok = bool(calib_ok and calib.pose_ok(rail))
                if out["valid"] and pose_ok and calibrated_method(out.get("method")):
                    x_cm = calib.px_to_cm(out["x_px"], W, H)
                cm_status = ('ok' if x_cm is not None else
                             'vision' if not out['valid'] or out.get('predicted') else
                             'pose_or_calibration' if not pose_ok else 'coordinate_unavailable')
                reference_ms = 0.0
                if preview is not None:
                    t_ref = time.perf_counter()
                    if not out['valid'] or out.get('predicted'):
                        preview.invalidate('vision')
                    elif not rail.valid:
                        preview.invalidate('rail')
                    else:
                        preview.update(gray)
                    reference_ms = (time.perf_counter() - t_ref) * 1000.0
                    pose_ok = bool(preview.valid and rail.valid)
                    x_cm = preview.position(out, candidates) if pose_ok else None
                    cm_status = preview.status
                    out['reference_preview'] = True
                out['cm_status'] = cm_status
                out["x_cm"] = x_cm
                state_counts[out["state"]] = state_counts.get(out["state"], 0) + 1

                t_out = time.perf_counter()
                line, feedback_valid, feedback_reason = encoder.encode(
                    out, t_cap, t_out, pose_ok, args.uart_enable_valid,
                    source_time_verified=getattr(source, "timestamp_verified", False))
                if preview is not None:
                    feedback_valid, feedback_reason = False, 'reference_preview'
                if serial is not None:
                    serial.send(line)
                algo_ms = (t_out - t0) * 1000.0
                # 采集时刻到检测/跟踪结果产生时刻。不含曝光时间和显示延迟,
                # 曝光时间需根据相机实际设置另行计入。
                pipe_ms = ((t_out - t_cap) * 1000.0
                           if t_cap is not None else None)
                wait_ms_list.append(wait_ms)
                algo_ms_list.append(algo_ms)
                if pipe_ms is not None:
                    pipe_ms_list.append(pipe_ms)
                now = t_out
                dt = now - t_prev
                t_prev = now
                if dt > 1e-6:
                    inst = 1.0 / dt
                    fps_ema = inst if fps_ema <= 0 else 0.9 * fps_ema + 0.1 * inst

                record_due = writer is not None and record_schedule.due(track_ts)
                need_vis = args.display or record_due
                vis = None
                draw_ms = 0.0
                write_ms = 0.0
                if need_vis:
                    t_vis0 = time.perf_counter()
                    vis = draw_overlay(frame, rail, candidates, out, fps_ema,
                                       algo_ms, x_cm,
                                       calibrated=(x_cm is not None) if preview is not None else pose_ok and calibrated_method(out.get("method")),
                                       output_size=record_size if record_due and not args.display else None)
                    draw_ms = (time.perf_counter() - t_vis0) * 1000.0
                    if record_due:
                        t_w0 = time.perf_counter()
                        record_frame = (vis if record_size == (vis.shape[1], vis.shape[0]) else
                                        cv2.resize(vis, record_size, interpolation=cv2.INTER_AREA))
                        writer.write(record_frame, timestamp=track_ts)
                        write_ms = (time.perf_counter() - t_w0) * 1000.0
                    if args.display:
                        cv2.imshow("Ball Vision", vis)
                        if (cv2.waitKey(1) & 0xFF) == ord("q"):
                            break
                frame_ms = (time.perf_counter() - t_read) * 1000.0
                draw_ms_list.append(draw_ms)
                write_ms_list.append(write_ms)
                frame_ms_list.append(frame_ms)

                if csv_writer is not None:
                    vt = track_ts if args.video else track_ts - capture_origin
                    csv_writer.writerow([
                        "%.6f" % out["timestamp"], frame_idx, "%.4f" % vt,
                        "" if out["x_px"] is None else "%.2f" % out["x_px"],
                        "" if x_cm is None else "%.4f" % x_cm,
                        int(out["valid"]), "%.4f" % out["confidence"],
                        out["state"], int(out["predicted"]), out["method"] or "",
                        "%.2f" % fps_ema, "%.2f" % algo_ms,
                        "%.2f" % wait_ms,
                        "" if pipe_ms is None else "%.2f" % pipe_ms,
                        "%.2f" % draw_ms, "%.2f" % write_ms,
                        "%.2f" % frame_ms,
                        "%.5f" % rail.slope, "%.2f" % rail.intercept,
                        int(rail.valid), int(pose_ok),
                        "" if t_cap is None else "%.3f" % (t_cap*1000),
                        int(feedback_valid), feedback_reason,
                        int(x_cm is not None), cm_status,
                        "" if preview is None or preview.reference_x_px is None else "%.4f" % preview.reference_x_px,
                        "%.3f" % reference_ms,
                    ])

                frame_idx += 1
                if frame_idx % 30 == 0:
                    print("frame=%d fps=%.1f x=%s state=%s conf=%.2f algo=%.1fms" % (
                        frame_idx, fps_ema,
                        "-" if out["x_px"] is None else "%.1f" % out["x_px"],
                        out["state"], out["confidence"], algo_ms))

                if args.duration > 0 and (time.perf_counter() - t_start) >= args.duration:
                    break
        except KeyboardInterrupt:
            print("\ninterrupted")
        finally:
            if serial is not None and serial.fd is not None:
                try:
                    stop_time = time.perf_counter()
                    line, _, _ = encoder.encode({'valid':False, 'confidence':0.},
                                                 stop_time, stop_time, False, False)
                    serial.send(line)
                except Exception as exc:
                    print("final UART invalid frame not delivered: %s" % exc)
                finally:
                    serial.close()
        elapsed = time.perf_counter() - t_start
        print("\n--- summary ---")
        print("frames: %d, elapsed: %.1fs, avg fps: %.1f" % (
            frame_idx, elapsed, frame_idx / elapsed if elapsed > 0 else 0.0))
        print("states: %s" % state_counts)
        if algo_ms_list:
            print("algo_ms: mean=%.2f p95=%.2f max=%.2f" % (
                sum(algo_ms_list) / len(algo_ms_list),
                _pct(algo_ms_list, 0.95), max(algo_ms_list)))
        if wait_ms_list:
            print("queue_wait_ms: mean=%.2f p95=%.2f max=%.2f" % (
                sum(wait_ms_list) / len(wait_ms_list),
                _pct(wait_ms_list, 0.95), max(wait_ms_list)))
        if pipe_ms_list:
            print("capture_to_result_ms: mean=%.2f p95=%.2f max=%.2f (n=%d)" % (
                sum(pipe_ms_list) / len(pipe_ms_list),
                _pct(pipe_ms_list, 0.95), max(pipe_ms_list),
                len(pipe_ms_list)))
        for label, values in (("draw_ms", draw_ms_list),
                              ("write_ms", write_ms_list),
                              ("frame_ms", frame_ms_list)):
            if values:
                print("%s: mean=%.2f p95=%.2f max=%.2f" % (
                    label, sum(values) / len(values),
                    _pct(values, 0.95), max(values)))
        if video_path:
            print("annotated video: %s" % video_path)
            print("recording timestamps: %s (CFR video is display-only; use timestamps for timing)" % writer.sidecar_path)
            print("recording queued=%d dropped=%d" % (writer.frames, writer.dropped))
        if csv_path:
            print("csv: %s" % csv_path)


if __name__ == "__main__":
    main()
