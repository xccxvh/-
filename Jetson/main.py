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
from camera_geometry import CameraGeometry
from rail_coordinates import RailCoordinates
from rail_detector import RailDetector
from ball_detector import BallDetector
from ball_tracker import BallTracker
from calibration import Calibration
from reference_preview import ReferencePreview
from measurement_contract import calibrated_method
from video_writer import AsyncVideoWriter
from recording_schedule import RecordingSchedule
from visualization import draw_overlay
from position_feedback import FeedbackEncoder, SerialFeedback, G3519PositionEncoder, LinkJournal

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


def main(argv=None, experiment=None, codec=None):
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
    args = ap.parse_args(argv)
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

    use_motion = cfg.get('static_control_motion', False)
    binary_link = bool(cfg.get('rail_coordinates', {}).get('enabled') and args.serial)
    if binary_link and args.uart_enable_valid and not use_motion:
        ap.error('binary control requires the calibrated static-control C configuration')
    if use_motion:
        from fusion_motion import MotionCandidates
        from static_control_detection import supplement
        from static_control_geometry import StaticControlCoordinates
        motion = MotionCandidates()
        cv2.setNumThreads(2 if cfg['rail'].get('upper_reference', {}).get('fast_search') else 1)
    with ExitStack() as resources:
        if args.video:
            source = VideoFile(resolve(args.video))
        else:
            camera_cfg = cfg["camera"]
            if binary_link and use_motion:
                camera_cfg = dict(camera_cfg, verify_v4l2_time=True, buffer_size=2)
            source = Camera(camera_cfg)
        if args.fps and args.fps > 0:
            source.fps_set = float(args.fps)
        resources.callback(source.release)
        W, H = source.width, source.height
        geometry = CameraGeometry(cfg.get('camera_geometry', {}), W, H, BASE)
        coordinates_type = StaticControlCoordinates if use_motion else RailCoordinates
        detector_type = RailDetector
        ref_config = cfg['rail'].get('upper_reference', {})
        if use_motion and ref_config.get('fast_search') is True:
            from static_fast_reference import make_rail_detector
            from static_fast_geometry import StaticFastCoordinates
            detector_type, coordinates_type = make_rail_detector, StaticFastCoordinates
        elif use_motion and 'digit_boxes' in ref_config:
            # 纸尺数字复核和专属标定成对加载，同一帧仅建立一套轨道姿态。
            from static_control_reference import make_rail_detector
            from static_reference_geometry import StaticReferenceCoordinates
            detector_type, coordinates_type = make_rail_detector, StaticReferenceCoordinates
        rail_coordinates = coordinates_type(cfg, geometry, BASE)
        if rail_coordinates.enabled and preview is not None:
            ap.error('rail coordinates and legacy preview cannot be combined')
        if binary_link:
            args.csv = True
        if preview is not None and geometry.mode != 'raw':
            ap.error('legacy preview cannot use corrected image/point coordinates')

        rail_det = detector_type(cfg["rail"], W, H)
        ball_cfg = dict(cfg["ball"])
        if "ball_search" in cfg:
            ball_cfg["ball_search"] = cfg["ball_search"]
        ball_det = BallDetector(ball_cfg, W, H)
        tracker = BallTracker(cfg["tracking"], W, H)

        # 标定只在"已拟合且采集分辨率与当前一致"时才可用。相机移动、变焦
        # 或改分辨率都会让旧标定失效, 此时 x_cm 必须输出空值。
        calib_ok = bool(calib is not None and calib.ready and
                        calib.resolution_ok(W, H) and geometry.mode == 'raw' and not rail_coordinates.enabled)
        if rail_coordinates.enabled:
            calib_status = 'rail model: ' + (rail_coordinates.calibration.status if rail_coordinates.calibration is not None else 'not available')
        elif geometry.mode != 'raw':
            calib_status = 'legacy calibration disabled in corrected coordinates'
        elif calib is None or not calib.ready:
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
                "rail_geometry_valid", "rail_identity_valid", "rail_reason",
                "roi_min_px", "roi_max_px", "candidate_count", "detector_reason", "tracker_reason",
                "rail_validity_policy", "candidate_diagnostics",
                "pixel_valid", "reference_valid", "calibration_valid", "measurement_x_px", "measurement_y_px",
                "reference_y_px", "rail_s_px", "rail_x_cm", "rail_angle_deg", "reference_rotation_deg",
                "reference_scale", "reference_residual_px", "rail_coordinate_status", "rail_binding_id",
                "image_space", "camera_geometry_mode", "geometry_ms", "reference_match_ms", "coordinate_ms",
                "uart_frame_time_ms", "timestamp_verified", "uart_age_ms", "measurement_source",

            ])

        if args.csv and rail_coordinates.enabled:
            # 同一CSV绑定当前几何/算法；另建sidecar，不覆盖任何历史标定。
            binding_path = os.path.splitext(csv_path)[0] + '.binding.json'
            with open(binding_path, 'x') as stream:
                json.dump(dict(binding=rail_coordinates.binding, rail_binding_id=rail_coordinates.binding_id),
                          stream, ensure_ascii=False, indent=2, allow_nan=False)

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
        captured_at_start = getattr(source, 'frames_captured', 0)
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
        journal = None
        task = None
        encoder = G3519PositionEncoder() if binary_link else FeedbackEncoder()
        try:
            if args.serial:
                serial = SerialFeedback(args.serial)
                resources.callback(serial.close)
                if binary_link:
                    # 独立事件表保留全部遥测。两端毫秒时钟不相减，不将旧 RX 冒充当前帧。
                    link_path = os.path.join(logs_dir, "link_%s_%s.csv" % (args.label, stamp))
                    stream = resources.enter_context(open(link_path, "x", newline="", buffering=1))
                    if experiment is None:
                        journal = LinkJournal(serial, stream)
                    else:
                        from concurrent.futures import ThreadPoolExecutor
                        from threading import Event
                        from experiment_port import ExperimentPort
                        serial = ExperimentPort(serial, codec)
                        journal = LinkJournal(serial, stream, split_rx=serial.split_rx)
                    resources.callback(journal.close)
                    print("G3519 UART", "control" if args.uart_enable_valid else "monitor", link_path)
            if experiment is not None:
                path = os.path.join(logs_dir, "experiment_%s_%s.csv" % (args.label, stamp))
                events = resources.enter_context(open(path, 'x', newline='', buffering=1))
                pool = resources.enter_context(ThreadPoolExecutor(max_workers=1))
                cancelled = Event()
                task = pool.submit(experiment, serial, events, cancelled)
            while True:
                if task is not None and task.done():
                    task.result()
                    break
                if use_motion and binary_link:
                    # 近期处理耗时仅用于选择当前单槽或下一帧。发送前仍按本帧 SOE
                    # 实测年龄执行 100 ms 门限；冷启动时等待下一张新图。
                    budget_ms = (encoder.max_age_ms - 2.0 - max(algo_ms_list[-4:])
                                 if algo_ms_list else None)
                    ret, frame = source.read(fresh=True, max_start_age_ms=budget_ms)
                else:
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
                frame = geometry.prepare_frame(frame)
                geometry_ms = (time.perf_counter() - t0) * 1000.0
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                t_reference = time.perf_counter()
                rail = rail_det.detect(gray, force=bool(args.serial or calib_ok or preview is not None or rail_coordinates.enabled))
                reference_match_ms = (time.perf_counter() - t_reference) * 1000.0
                # Replay speed must not change motion prediction. Live read
                # time is monotonic but still not a verified sensor timestamp.
                track_ts = (frame_idx / (source.fps_set if source.fps_set > 0 else 30.0)
                            if args.video else t_cap)
                # 模糊提案与跟踪器共用该帧采集时刻；文件回放速度不会改变运动门控。
                if cfg['ball'].get('motion_blur_recovery'):
                    candidates = ball_det.detect(frame, gray, rail, timestamp=track_ts)
                else:
                    candidates = ball_det.detect(frame, gray, rail)
                if capture_origin is None:
                    capture_origin = track_ts
                # 本帧的物理搜索边界供出界重获核验；参考失效时清除边界证据。
                bounds = ball_det.last_roi if cfg.get('ball_search', {}).get('enabled') else None
                tracking_bounds = bounds
                if use_motion:
                    candidates, tracking_bounds = supplement(motion, gray, rail, bounds, track_ts, candidates)
                out = tracker.update(candidates, track_ts, tracking_bounds)
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
                t_coordinate = time.perf_counter()
                coordinate = rail_coordinates.process(out, candidates, rail, frame_idx)
                coordinate_ms = (time.perf_counter() - t_coordinate) * 1000.0
                if rail_coordinates.enabled:
                    pose_ok = coordinate['reference_valid']
                    x_cm = coordinate['rail_x_cm']
                    cm_status = coordinate['rail_coordinate_status']
                    reference_ms = reference_match_ms
                    out.update(coordinate)
                out['cm_status'] = cm_status
                out["x_cm"] = x_cm
                state_counts[out["state"]] = state_counts.get(out["state"], 0) + 1

                t_out = time.perf_counter()
                # 运动估计沿用读帧时钟，控制帧年龄使用本张图像经核验的曝光时钟。
                verified = getattr(source, "timestamp_verified", False)
                frame_time = source.last_capture_ts if binary_link and verified else t_cap
                line, feedback_valid, feedback_reason = encoder.encode(
                    out, frame_time, t_out, pose_ok, args.uart_enable_valid,
                    source_time_verified=verified)
                if preview is not None:
                    feedback_valid, feedback_reason = False, 'reference_preview'
                if rail_coordinates.enabled and not binary_link:
                    feedback_valid, feedback_reason = False, 'rail_measurement_only'
                if line is None:
                    # 仅 G3519 有效控制模式可能跳过纯超时样本；保留本帧 track 和独立 drop 事件。
                    journal.dropped(frame_idx, frame_time, feedback_reason)
                elif serial is not None:
                    serial.send(line)
                    if journal is not None:
                        journal.sent(line, frame_idx, feedback_reason, verified)
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
                                       calibrated=(x_cm is not None) if (preview is not None or rail_coordinates.enabled) else pose_ok and calibrated_method(out.get("method")),
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
                        coordinate['reference_x_px'] if rail_coordinates.enabled else ("" if preview is None or preview.reference_x_px is None else "%.4f" % preview.reference_x_px),
                        "%.3f" % reference_ms,
                        int(getattr(rail, 'geometry_valid', rail.valid)),
                        int(getattr(rail, 'identity_valid', False)), getattr(rail, 'reason', ''),
                        '' if getattr(ball_det, 'last_roi', None) is None else '%.2f' % ball_det.last_roi[0],
                        '' if getattr(ball_det, 'last_roi', None) is None else '%.2f' % ball_det.last_roi[1],
                        len(candidates), ball_det.last_reason, out.get('reason', ''),
                        getattr(rail, 'validity_policy', 'legacy_geometry'),
                        json.dumps(getattr(ball_det, 'last_diagnostics', []), separators=(',', ':')),
                        int(coordinate['pixel_valid'] if rail_coordinates.enabled else out['valid'] and not out['predicted']),
                        # 旧路径保留原算法，同时让追加有效性列与既有厘米输出一致。
                        int(coordinate['reference_valid'] if rail_coordinates.enabled else (preview.valid if preview is not None else rail.valid)),
                        int(coordinate['calibration_valid'] if rail_coordinates.enabled else (x_cm is not None or calib_ok)),
                        coordinate['measurement_x_px'], coordinate['measurement_y_px'], coordinate['reference_y_px'],
                        coordinate['rail_s_px'], coordinate['rail_x_cm'], coordinate['rail_angle_deg'],
                        coordinate['reference_rotation_deg'], coordinate['reference_scale'], coordinate['reference_residual_px'],
                        coordinate['rail_coordinate_status'], coordinate['rail_binding_id'],
                        geometry.image_space, geometry.mode, geometry_ms, reference_match_ms, coordinate_ms,
                        frame_time*1000 if binary_link and frame_time is not None else None, int(verified),
                        (t_out-frame_time)*1000 if binary_link and frame_time is not None else None,
                        (candidates[0].extra.get('proposal_source', out.get('method'))
                         if use_motion and coordinate['cm_valid'] else None),
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
            try:
                if task is not None:
                    cancelled.set()
                    # Keep RX and port alive until the worker has attempted STOP.
                    task.result()
            finally:
                if serial is not None and serial.fd is not None:
                    try:
                        stop_time = time.perf_counter()
                        line, _, reason = encoder.encode({'valid':False, 'confidence':0.},
                                                     stop_time, stop_time, False, False)
                        serial.send(line)
                        if journal is not None:
                            journal.sent(line, '', reason, False)
                    except Exception as exc:
                        print("final UART invalid frame not delivered: %s" % exc)
                    # ExitStack 先停接收线程并关闭 CSV，最后释放串口。
        elapsed = time.perf_counter() - t_start
        print("\n--- summary ---")
        print("frames: %d, elapsed: %.1fs, avg fps: %.1f" % (
            frame_idx, elapsed, frame_idx / elapsed if elapsed > 0 else 0.0))
        print("states: %s" % state_counts)
        if hasattr(source, 'frames_captured'):
            # 同一段运行时间内采集/处理分开计数；两者之差还包含单槽覆盖。
            captured = source.frames_captured - captured_at_start
            print("capture: %d frames, %.1f fps, wait-for-newer: %d" %
                  (captured, captured / elapsed, source.skipped_for_age))
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
