# -*- coding: utf-8 -*-
"""单版本顺序回放，兼容Python3.6；复制本文件到另一源码目录即可比较。

只实例化本目录导轨/检测/跟踪模块，不覆盖参数、不混入另一版本算法。
--labels可省略；人工视觉估计（含独立agent）不是human ground truth。
输出frames.csv和summary.json；--annotate额外同步输出全部帧annotated.mp4。
"""
import argparse
import copy
import csv
import hashlib
import json
import math
import platform
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from rail_detector import RailDetector
from ball_detector import BallDetector
from ball_tracker import BallTracker

# 历史CSV字段顺序保持不变；新增诊断只追加，不改变算法返回对象。
HISTORICAL = ('timestamp frame video_time_s x_px x_cm valid confidence state predicted method fps '
              'algo_ms wait_ms pipe_ms draw_ms write_ms frame_ms rail_slope rail_intercept '
              'rail_valid pose_ok capture_monotonic_ms feedback_valid feedback_reason cm_valid '
              'cm_status reference_x_px reference_ms rail_geometry_valid rail_identity_valid '
              'rail_reason roi_min_px roi_max_px candidate_count detector_reason tracker_reason '
              'evaluation_region manual_x_px assessment error_px coordinate_version').split()
EXTRA = ('candidate_diagnostics_json candidates_json acquire_count acquire_count_before acquire_required '
         'core_min_px core_max_px search_min_px search_max_px region_source '
         'evaluation_region_tolerance assessment_tolerance label_window '
         'manual_search_left_px manual_core_left_px label_json').split()
DISCLAIMER = '人工视觉估计/独立agent视觉审阅，非human independent ground truth；不代表全视频准确率。'


def finite(value):
    """拒绝布尔/字符串伪装的数值，避免坏标签静默进入分母。"""
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def normalize_labels(document, expected):
    """旧53点与连续标签均使用labels数组；无标签时不推断任何真值。"""
    if document is None:
        return {}
    items = document.get('labels') if isinstance(document, dict) else document
    if not isinstance(items, list):
        raise ValueError('labels must be a list or a document containing labels')
    truth = {}
    for item in items:
        frame = item.get('frame')
        if type(frame) is not int or not 0 <= frame < expected or frame in truth:
            raise ValueError('duplicate or outside label frame')
        if item.get('status') == 'visible':
            if not finite(item.get('x_px')) or not 0 <= item['x_px'] < 1280:
                raise ValueError('visible label requires finite in-frame x_px')
        tolerance = item.get('tolerance_px', 6)
        if not finite(tolerance) or tolerance < 0:
            raise ValueError('invalid label tolerance')
        # 新标签只审阅左边界；分区声明保留右边界远离当前目标的审阅上下文。
        if 'ruler5_search_left_x_px' in item or 'ruler7_core_left_x_px' in item:
            a, b = item.get('ruler5_search_left_x_px'), item.get('ruler7_core_left_x_px')
            if not finite(a) or not finite(b) or not 0 <= a < b < 1280:
                raise ValueError('invalid visual tick boundaries')
            if item.get('center_region_nominal') not in ('core', 'left_buffer', 'outside', 'outside_left', 'outside_search'):
                raise ValueError('visual left-boundary audit requires explicit center region')
            t = item.get('anchor_tolerance_px', 3)
            if not finite(t) or t < 0:
                raise ValueError('invalid anchor tolerance')
        truth[frame] = item
    return truth


def physical_bounds(cfg, rail):
    """仅为日志投影搜索/核心范围；不调用或修改检测器内部ROI状态。"""
    if not cfg.get('enabled', False) or not getattr(rail, 'valid', False) or not getattr(rail, 'identity_valid', False):
        return None, None
    try:
        anchors = np.asarray(cfg['anchors'], dtype=float)
        low, high = float(cfg['target_min_cm']), float(cfg['target_max_cm'])
        margin = float(cfg.get('recovery_margin_cm', 2))
        cm = np.array([low, high, low-margin, high+margin])
        if (anchors.ndim != 2 or anchors.shape[1] != 2 or len(anchors) < 2 or
                not np.isfinite(anchors).all() or not np.isfinite(cm).all() or
                low >= high or margin < 0 or not np.all(np.diff(anchors[:, 0]) > 0) or
                not np.all(np.diff(anchors[:, 1]) > 0) or cm.min() < anchors[0, 0] or cm.max() > anchors[-1, 0]):
            raise ValueError('invalid physical ROI anchors')
        x = np.interp(cm, anchors[:, 0], anchors[:, 1])
        profile = np.asarray(rail.reference_profile)
        if x.min() < profile[0, 0] or x.max() > profile[-1, 0]:
            return None, None
        y = np.interp(x, profile[:, 0], profile[:, 1])
        pts = np.column_stack((x, y)).dot(np.linalg.inv(rail.reference_inverse).T) + rail.reference_translation
        if not np.isfinite(pts).all():
            return None, None
        return tuple(float(v) for v in pts[:2, 0]), tuple(float(v) for v in pts[2:, 0])
    except (AttributeError, np.linalg.LinAlgError):
        return None, None


def region_for(label, core, roi, tolerant=False):
    """人工逐帧tick优先；旧标签缺人工边界时显式标记算法投影来源。"""
    if label is None:
        return 'unlabeled', 'none'
    if label.get('status') != 'visible' or label.get('x_px') is None:
        return 'visibility_unknown', 'visual_label'
    x = label['x_px']
    if 'ruler5_search_left_x_px' in label:
        a, b = label['ruler5_search_left_x_px'], label['ruler7_core_left_x_px']
        nominal = 'outside' if x < a else 'buffer' if x < b else 'core'
        tolerance = label.get('tolerance_px', 6) + label.get('anchor_tolerance_px', 3)
        uncertain = bool(label.get('center_region_uncertain', False)) or min(abs(x-a), abs(x-b)) <= tolerance
        return ('boundary_unknown' if tolerant and uncertain else nominal), 'visual_ticks_left_audit'
    if core is None or roi is None:
        return 'reference_unknown', 'algorithm_projection_unavailable'
    nominal = 'core' if core[0] <= x <= core[1] else 'buffer' if roi[0] <= x <= roi[1] else 'outside'
    uncertain = min(abs(x-v) for v in core+roi) <= label.get('tolerance_px', 6)
    return ('boundary_unknown' if tolerant and uncertain else nominal), 'algorithm_projection_not_independent'


def validate_output(out, candidates, detector):
    """有效测量必须有当前候选且位于本次实际ROI；违反合同立即停止回放。"""
    if not out.get('valid'):
        return
    x = out.get('x_px')
    if out.get('predicted', False) or not finite(x) or not any(c.x == x for c in candidates):
        raise ValueError('valid output must be a finite current candidate, never a prediction')
    roi = getattr(detector, 'last_roi', None)
    if roi is not None and (not np.isfinite(roi).all() or not roi[0] <= x <= roi[1]):
        raise ValueError('valid output outside detector ROI')


def assess(label, out, core, roi, tolerant=False):
    """即使参考/边界未知，仍报告像素误差；未知帧不偷偷归为core。"""
    region, _ = region_for(label, core, roi, tolerant)
    if label is None:
        return region, 'unlabeled', None
    if label.get('status') != 'visible' or label.get('x_px') is None:
        return region, 'visibility_unknown', None
    valid = bool(out.get('valid')) and not out.get('predicted', False)
    error = abs(out['x_px']-label['x_px']) if valid else None
    if region == 'outside':
        return region, 'outside_false_valid' if valid else 'outside_rejected', error
    return region, ('correct' if valid and error <= label.get('tolerance_px', 6)
                    else 'false_valid' if valid else 'miss'), error


def summarize(rows, fps, region_key='evaluation_region', assessment_key='assessment'):
    """连续帧和同标签窗口内统计；miss仅指无有效输出，不与false_valid混用。"""
    names = ('core', 'buffer', 'outside', 'boundary_unknown', 'reference_unknown', 'visibility_unknown')
    regions = {name: {'labeled_frames': 0, 'assessments': {}, 'miss_runs': []} for name in names}
    windows, events = [], []
    previous = None
    run = None
    active = None
    window = None

    def finish_event():
        # 必须亲眼覆盖outside->buffer->core，跳过未知或只见半次入口不报告延迟。
        if active is not None and active['core_frame'] is not None:
            first = active['first_correct_frame']
            event = dict(active)
            event['censored'] = first is None
            event['delay_frames'] = None if first is None else first-active['entry_frame']
            event['delay_s'] = None if first is None else event['delay_frames']/fps
            core_first = active['core_first_correct_frame']
            event['core_censored'] = core_first is None
            event['delay_from_core_frames'] = None if core_first is None else core_first-active['core_frame']
            event['delay_from_core_s'] = None if core_first is None else event['delay_from_core_frames']/fps
            events.append(event)

    for row in rows:
        region, assessment, frame = row[region_key], row[assessment_key], row['frame']
        labeled = region != 'unlabeled'
        adjacent = (previous is not None and frame == previous['frame']+1 and
                    row.get('label_window') == previous.get('label_window'))
        if labeled and adjacent and window is not None:
            window['end_frame'] = frame
            window['frames'] += 1
        elif labeled:
            window = dict(start_frame=frame, end_frame=frame, frames=1)
            windows.append(window)
        else:
            window = None
        if region in regions:
            data = regions[region]
            data['labeled_frames'] += 1
            data['assessments'][assessment] = data['assessments'].get(assessment, 0)+1
            if assessment == 'miss':
                if run is not None and adjacent and previous[region_key] == region:
                    run['end_frame'] = frame; run['frames'] += 1
                else:
                    run = dict(start_frame=frame, end_frame=frame, frames=1)
                    data['miss_runs'].append(run)
            else:
                run = None
        else:
            run = None
        known = region in ('core', 'buffer', 'outside')
        if not adjacent or not known or region == 'outside':
            finish_event(); active = None
        if (adjacent and region == 'buffer' and previous[region_key] == 'outside'):
            active = dict(outside_frame=previous['frame'], entry_frame=frame, core_frame=None,
                          first_correct_frame=None, core_first_correct_frame=None, last_observed_frame=frame)
        if active is not None:
            active['last_observed_frame'] = frame
            if region == 'core' and active['core_frame'] is None:
                active['core_frame'] = frame
            if region == 'core' and assessment == 'correct' and active['core_first_correct_frame'] is None:
                active['core_first_correct_frame'] = frame
            if assessment == 'correct' and active['first_correct_frame'] is None:
                active['first_correct_frame'] = frame
        previous = row
    finish_event()
    for data in regions.values():
        data['longest_miss_frames'] = max([r['frames'] for r in data['miss_runs']] or [0])
    return dict(regions=regions, labeled_frames=sum(v['labeled_frames'] for v in regions.values()),
                continuous_label_windows=windows, reentry_events=events)


def jsonable(value):
    """候选诊断含NumPy标量/数组；非有限值写null，保证JSON可跨平台读取。"""
    if isinstance(value, np.ndarray):
        return jsonable(value.tolist())
    if isinstance(value, np.generic):
        return jsonable(value.item())
    if isinstance(value, dict):
        return {str(k): jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [jsonable(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def dumps(value):
    return json.dumps(jsonable(value), ensure_ascii=False, sort_keys=True, allow_nan=False)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def source_manifest():
    """保存实际加载的本地模块；结束重查，运行期间源码变更则不发表成功summary。"""
    root = Path(__file__).resolve().parent
    files = {Path(__file__).resolve()}
    for module in list(sys.modules.values()):
        path = getattr(module, '__file__', None)
        if path and Path(path).suffix == '.py' and Path(path).resolve().parent == root:
            files.add(Path(path).resolve())
    return {str(p): sha256(p) for p in sorted(files)}


def verify_encoded(path, expected):
    """实际逐帧计数，不能相信容器宣称的nb_frames。"""
    result = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
                         '-count_frames', '-show_entries', 'stream=nb_read_frames', '-of', 'json', str(path)]))
    count = int(result['streams'][0]['nb_read_frames'])
    if count != expected:
        raise ValueError('encoded video frame loss: %d != %d' % (count, expected))
    return count


def annotate(frame, rail, candidates, row):
    """仅绘制副本，避免旧版overlay依赖新字段或修改算法输入。"""
    vis = frame.copy()
    # 只显示本帧确认的真实曲线，不把失效回退线画成已观测上缘。
    edge = getattr(rail, 'edge_profile', None)
    if rail.valid and edge is not None:
        edge = np.asarray(edge)
        if edge.shape == (frame.shape[1],) and np.isfinite(edge).all():
            points = np.column_stack((np.arange(len(edge)), np.rint(edge))).astype(np.int32)
            cv2.polylines(vis, [points.reshape(-1, 1, 2)], False, (0, 255, 0), 1)
    for key, color in [('roi_min_px', (0, 255, 255)), ('roi_max_px', (0, 255, 255)),
                       ('core_min_px', (255, 200, 0)), ('core_max_px', (255, 200, 0))]:
        x = row[key]
        if x != '' and x is not None and math.isfinite(x):
            cv2.line(vis, (int(round(x)), 150), (int(round(x)), 300), color, 1)
    for c in candidates:
        y = getattr(c, 'extra', {}).get('measurement_y_px', rail.slope*c.x+rail.intercept)
        cv2.circle(vis, (int(round(c.x)), int(round(y))), 5, (0, 200, 255), 1)
        if row['valid'] and c.x == row['x_px']:
            cv2.circle(vis, (int(round(c.x)), int(round(y))), 13, (0, 255, 0), 2)
    text = 'frame %d | valid %d | %s | %s | acq %s' % (row['frame'], row['valid'], row['state'], row['assessment'], row['acquire_count'])
    cv2.putText(vis, text, (12, 35), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 255, 0), 1)
    return vis


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--labels', help='省略即无标签回放；不自动加载其他标签')
    parser.add_argument('--output', required=True)
    parser.add_argument('--fps', type=float, default=29.8)
    parser.add_argument('--expected-frames', type=int, default=1043)
    parser.add_argument('--annotate', action='store_true')
    args = parser.parse_args(argv)
    if not math.isfinite(args.fps) or args.fps <= 0 or args.expected_frames < 1:
        raise ValueError('positive finite FPS and frame count required')
    video = Path(args.video).resolve()
    if not video.is_file():
        raise ValueError('video must be a local regular file, not a camera or URL')
    config = Path(args.config).resolve()
    cfg = json.loads(config.read_text(encoding='utf-8'))
    document = json.loads(Path(args.labels).read_text(encoding='utf-8')) if args.labels else None
    truth = normalize_labels(document, args.expected_frames)
    # 首次读取之后保留精确配置快照，复制工具到baseline时无需新版本模块。
    manifest = source_manifest()
    input_hashes = dict(video=sha256(video), config=sha256(config), labels=sha256(args.labels) if args.labels else None)
    output = Path(args.output).resolve(); output.mkdir(parents=True, exist_ok=False)
    (output/'config_used.json').write_text(dumps(cfg), encoding='utf-8')
    if document is not None:
        (output/'labels_used.json').write_text(dumps(document), encoding='utf-8')
    rd = RailDetector(copy.deepcopy(cfg['rail']), 1280, 720)
    bc = copy.deepcopy(cfg['ball'])
    if 'ball_search' in cfg:
        bc['ball_search'] = copy.deepcopy(cfg['ball_search'])
    bd = BallDetector(bc, 1280, 720)
    bt = BallTracker(copy.deepcopy(cfg['tracking']), 1280, 720)
    # BallDetector初始化可能延迟导入local_vision，必须纳入源码指纹。
    manifest.update(source_manifest())
    cap = cv2.VideoCapture(str(video)); encoder = None; rows = []; encoded = None
    try:
        if not cap.isOpened():
            raise ValueError('cannot open video')
        if args.annotate:
            encoder = subprocess.Popen(['ffmpeg', '-y', '-v', 'error', '-f', 'rawvideo', '-pix_fmt', 'bgr24',
                '-s', '1280x720', '-r', str(args.fps), '-i', '-', '-an', '-c:v', 'libx264',
                '-preset', 'ultrafast', '-crf', '21', '-threads', '1', '-pix_fmt', 'yuv420p',
                str(output/'annotated.mp4')], stdin=subprocess.PIPE)
        with (output/'frames.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.DictWriter(stream, fieldnames=HISTORICAL+EXTRA); writer.writeheader()
            while True:
                ok, frame = cap.read()
                if not ok:
                    break
                idx = len(rows)
                if idx >= args.expected_frames or frame.shape != (720, 1280, 3):
                    raise ValueError('unexpected frame count or resolution')
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                start = time.perf_counter()
                rail = rd.detect(gray, force=True)
                candidates = bd.detect(frame, gray, rail)
                before = getattr(bt, 'acq', '')
                out = bt.update(candidates, idx/args.fps)
                validate_output(out, candidates, bd)
                ms = (time.perf_counter()-start)*1000
                core, roi = physical_bounds(cfg.get('ball_search', {}), rail)
                label = truth.get(idx)
                region, assessment, error = assess(label, out, core, roi)
                tolerant_region, tolerant_assessment, _ = assess(label, out, core, roi, True)
                last_roi = getattr(bd, 'last_roi', None)
                candidate_data = [dict(x_px=c.x, width_px=c.width, height_px=c.height,
                                      confidence=c.confidence, method=c.method, extra=getattr(c, 'extra', {})) for c in candidates]
                selected = [c for c in candidates if c.x == out.get('x_px') and c.method == out.get('method')]
                coordinate = getattr(selected[0], 'extra', {}).get('coordinate_version', 'legacy') if len(selected) == 1 else ''
                row = {key: '' for key in HISTORICAL+EXTRA}
                row.update(frame=idx, video_time_s=idx/args.fps, x_px=out.get('x_px'), valid=int(out['valid']),
                    confidence=out['confidence'], state=out['state'], predicted=int(out.get('predicted', False)),
                    method=out.get('method'), fps=args.fps, algo_ms=ms, rail_slope=rail.slope, rail_intercept=rail.intercept,
                    rail_valid=int(rail.valid), pose_ok=0, feedback_valid=0, feedback_reason='offline', cm_valid=0,
                    cm_status='offline_uncalibrated', rail_geometry_valid=int(getattr(rail, 'geometry_valid', rail.valid)),
                    rail_identity_valid=int(getattr(rail, 'identity_valid', False)), rail_reason=getattr(rail, 'reason', ''),
                    roi_min_px=last_roi[0] if last_roi is not None else '', roi_max_px=last_roi[1] if last_roi is not None else '',
                    candidate_count=len(candidates), detector_reason=getattr(bd, 'last_reason', ''), tracker_reason=out.get('reason', ''),
                    evaluation_region=region, manual_x_px=label.get('x_px') if label else '', assessment=assessment, error_px=error,
                    coordinate_version=coordinate, candidate_diagnostics_json=dumps(getattr(bd, 'last_diagnostics', [])),
                    candidates_json=dumps(candidate_data), acquire_count=getattr(bt, 'acq', ''), acquire_count_before=before,
                    acquire_required=getattr(bt, 'acquire_frames', ''), core_min_px=core[0] if core else '', core_max_px=core[1] if core else '',
                    search_min_px=roi[0] if roi else '', search_max_px=roi[1] if roi else '', region_source=region_for(label, core, roi)[1],
                    evaluation_region_tolerance=tolerant_region, assessment_tolerance=tolerant_assessment,
                    label_window=label.get('sample_frame', '') if label else '',
                    manual_search_left_px=label.get('ruler5_search_left_x_px', '') if label else '',
                    manual_core_left_px=label.get('ruler7_core_left_x_px', '') if label else '', label_json=dumps(label))
                writer.writerow(row); rows.append(row)
                if encoder is not None:
                    encoder.stdin.write(annotate(frame, rail, candidates, row).tobytes())
    finally:
        cap.release()
        if encoder is not None:
            try:
                encoder.stdin.close()
            finally:
                try:
                    returncode = encoder.wait(timeout=30)
                except subprocess.TimeoutExpired:
                    encoder.kill(); encoder.wait()
                    raise RuntimeError('encoder shutdown timed out')
    if encoder is not None and returncode:
        raise RuntimeError('encoder failed')
    if len(rows) != args.expected_frames:
        raise ValueError('incomplete decode: %d != %d' % (len(rows), args.expected_frames))
    if args.annotate:
        encoded = verify_encoded(output/'annotated.mp4', len(rows))
    if any(sha256(path) != digest for path, digest in manifest.items()):
        raise RuntimeError('source changed during replay; discard mixed provenance run')
    if sha256(config) != input_hashes['config'] or (args.labels and sha256(args.labels) != input_hashes['labels']):
        raise RuntimeError('config or labels changed during replay')
    result = summarize(rows, args.fps)
    result.update(frames=len(rows), media_fps=args.fps, encoded_frames=encoded,
                  unlabeled_frames=len(rows)-len(truth), disclaimer=DISCLAIMER,
                  labels_scope=document.get('audit_type', 'visual_estimates') if isinstance(document, dict) else 'none' if document is None else 'visual_estimates',
                  tolerance_sensitivity=summarize(rows, args.fps, 'evaluation_region_tolerance', 'assessment_tolerance'),
                  source_manifest=manifest, input_sha256=input_hashes,
                  source_video=str(video), config_path=str(config), labels_path=str(Path(args.labels).resolve()) if args.labels else None,
                  label_windows=document.get('windows', []) if isinstance(document, dict) else [],
                  reported_valid_frames=sum(r['valid'] for r in rows),
                  algo_mean_ms=float(np.mean([r['algo_ms'] for r in rows])),
                  runtime=dict(python=platform.python_version(), opencv=cv2.__version__, numpy=np.__version__, machine=platform.machine(), host=platform.node()),
                  replay_policy='single imported version; force rail each frame; media frame/fps time; config unchanged',
                  reentry_definition='contiguous labeled outside->buffer->core; delay from first buffer frame to first correct in buffer/core; censored if none before gap/exit/end',
                  tolerance_note='dense: center tolerance + visual tick tolerance; boundary_unknown retains pixel assessment; sparse: projected boundaries are NOT independent')
    (output/'summary.json').write_text(json.dumps(jsonable(result), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print('frames=%d labeled=%d valid=%d output=%s' % (len(rows), len(truth), result['reported_valid_frames'], output))
    return result


if __name__ == '__main__':
    main()
