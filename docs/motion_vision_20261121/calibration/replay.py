"""从主机原片重做轨道配准与 raw/points 评估。全程只使用本地文件。"""
import argparse
import csv
import json
import math
import os
import sys
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[3]
JET = Path(os.environ.get('MOTION_REPLAY_JETSON', REPO / 'Jetson')).resolve()
STAGE = REPO / 'docs/board_stage1_20261121'
HOST = Path.home() / '桌面/jetson_nano/7'
sys.path[:0] = [str(JET), str(STAGE)]
from camera_geometry import CameraGeometry
from rail_coordinates import RailCoordinates
from rail_detector import RailDetector
from ball_detector import BallDetector
from ball_tracker import BallTracker
from calibrate_rail import collect, sha256
from rail_calibration import build_model, map_position, RailCalibration, FIT_MIN_SAMPLES
from static_analysis import metrics
# 旧评估模块导入时会添加仓库源码目录；延迟导入的检测器也须来自同一快照。
sys.path.insert(0, str(JET))


def read_csv(path):
    with path.open(encoding='utf-8-sig', newline='') as stream:
        return list(csv.DictReader(stream))


def write_rows(path, rows):
    with path.open('x', newline='') as stream:
        writer = csv.DictWriter(stream, rows[0], lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def json_out(path, data):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(data, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')


def clip_clock(path, count, nominal_30hz=False):
    # 采集器有两种时标，统一为秒。MKV 可使用容器时间戳。
    # 裸 MJPEG 的解码器默认帧率可能与实采不符，不能用它补造采集时间。
    side = path.parent / 'frames.csv'
    if not side.exists():
        if nominal_30hz:
            return [i / 30.0 for i in range(count)], 'nominal_30Hz_static_only'
        if path.suffix != '.mkv':
            raise ValueError(f'{path}: 缺少帧时钟')
        return None, 'container_pts'
    rows = read_csv(side)
    if [int(r['frame']) for r in rows] != list(range(count)):
        raise ValueError(f'{side}: 帧号或总数不符')
    key = 'read_s' if 'read_s' in rows[0] else 'nano_read_monotonic_ms'
    factor = 1 if key == 'read_s' else .001
    return [float(r[key]) * factor for r in rows], key


def peak_offsets(gray, ref, settings):
    # 中帧诊断：保留横向搜索范围，把纵向搜索扩至全图。仅记录相关峰，
    # 不用这些候选点修正图像或计算厘米，实际采样仍须经过原匹配器。
    result = []
    for x, y, w, h in settings['boxes']:
        left = max(0, x - settings['search_x'])
        right = min(gray.shape[1], x + w + settings['search_x'])
        response = cv2.matchTemplate(gray[:, left:right], ref[y:y+h, x:x+w],
                                     cv2.TM_CCOEFF_NORMED)
        _, score, _, (dx, yy) = cv2.minMaxLoc(response)
        result.append(dict(dx=left+dx-x, dy=yy-y, score=score))
    return result


def replay(clip, cfg, points_geometry, dest):
    video = clip['video']
    count = clip['count']
    times, source = clip_clock(video, count, clip.get('nominal_30hz', False))
    w, h = (cfg['camera'][key] for key in ('width', 'height'))
    detector = RailDetector(cfg['rail'], w, h)
    balls = BallDetector(dict(cfg['ball'], ball_search=cfg['ball_search']), w, h)
    tracker = BallTracker(cfg['tracking'], w, h)
    # 两种模式共享当帧的原始图像检测。points 在矫正参考对应点后
    # 重新估计相似变换，并独立矫正球点。每次实物重摆都重置跟踪器。
    coordinates = {}
    for mode in ('raw', 'points'):
        geo_cfg = cfg['camera_geometry'] if mode == 'raw' else points_geometry
        geo = CameraGeometry(geo_cfg, w, h, JET)
        if geo.context['mode'] != mode:
            raise ValueError(f'{mode}: 相机几何配置错误')
        coordinates[mode] = RailCoordinates(cfg, geo, JET)
    settings = cfg['rail']['upper_reference']
    ref = cv2.imread(settings['image'], cv2.IMREAD_GRAYSCALE)
    # 现场冻结入口从 MJPEG 直拷 JPEG 负载，用 imdecode 转 BGR；
    # VideoCapture 的颜色解码可能改变 Hough 候选，旧片对照保留原解码器。
    field_decode = clip.get('field') or (
        video.suffix == '.mjpg' and source in ('read_s', 'nominal_30Hz_static_only')
        and (JET/'motion_observe.py').is_file())
    cap = None
    stream = None
    if field_decode:
        from motion_observe import packets
        stream = packets(['ffmpeg','-nostdin','-hide_banner','-loglevel','error','-f','mjpeg',
                          '-i',str(video),'-c:v','copy','-f','mpjpeg','pipe:1'])
    else:
        cap = cv2.VideoCapture(str(video))
    logs = {mode: [] for mode in coordinates}
    prev = -math.inf
    review = []
    try:
        for i in range(count):
            if stream is not None:
                jpeg = next(stream, None)
                img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR) if jpeg else None
            else:
                _, img = cap.read()
            if img is None or img.shape[:2] != (h, w):
                raise ValueError(f'{video}: 无法解码预期尺寸的第 {i} 帧')
            ts = times[i] if times is not None else cap.get(cv2.CAP_PROP_POS_MSEC) / 1000
            if not math.isfinite(ts) or ts <= prev:
                raise ValueError(f'{video}: 时间戳无效，帧 {i}')
            prev = ts
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            rail = detector.detect(gray, force=True)
            candidates = balls.detect(img, gray, rail)
            out = tracker.update(candidates, ts)
            if i == count // 2:
                peaks = peak_offsets(gray, ref, settings)
            if clip.get('field') and i in (0, count//2, count-1):
                # 保留可读尺数字、球和挡具，便于独立核对标签与中途移动。
                top = max(0, min(box[1] for box in settings['boxes']) - 100)
                strip = img[top:].copy()
                cv2.putText(strip, f"{clip['run']}  frame {i}", (8, 22),
                            cv2.FONT_HERSHEY_SIMPLEX, .6, (255,255,255), 1, cv2.LINE_AA)
                review.append(strip)
            for mode, coord in coordinates.items():
                position = coord.process(out, candidates, rail, i)
                logs[mode].append(dict(frame=i, time_s=ts, predicted=out['predicted'],
                                       rail_reason=rail.reason, ball_reason=balls.last_reason,
                                       **position))
        extra_frame = next(stream, None) is not None if stream is not None else cap.read()[0]
        if extra_frame:
            raise ValueError(f'{video}: 原片比清单更长')
    finally:
        if stream is not None:
            stream.close()
        else:
            cap.release()
    summaries, sessions = [], {}
    folder = dest / clip['run']
    folder.mkdir()
    if review and not cv2.imwrite(str(folder/'first_middle_last.jpg'), np.vstack(review)):
        raise OSError('无法保存真值核对图')
    for mode, data in logs.items():
        coord = coordinates[mode]
        csv_path = folder / (mode+'.csv')
        side = folder / (mode+'.binding.json')
        write_rows(csv_path, data)
        json_out(side, coord.binding)
        # 临时绑定只描述本次回放。仅带真值的整段交给项目原采样器。
        if clip['role'] in ('fit', 'verify', 'repeat_zero'):
            args = SimpleNamespace(video=video, csv=csv_path, cm=clip['cm'],
                                   known_position=True, start_frame=0, end_frame=count-1)
            sessions[mode] = collect(args, coord.binding, side)
        summaries.append(dict(run=clip['run'], role=clip['role'], cm=clip['cm'], mode=mode,
                              reported_cm=clip.get('reported_cm', clip['cm']),
                              note=clip.get('note', ''),
                              frames=count, reference=sum(r['reference_valid'] for r in data),
                              samples=sum(r['rail_s_px'] is not None for r in data),
                              clock=source, decoder='imdecode' if field_decode else 'VideoCapture',
                              video_sha256=clip['sha256'],
                              replay_binding=coord.binding_id,
                              rail_reasons=json.dumps(Counter(r['rail_reason'] for r in data))))
    return summaries, sessions, peaks


def evaluate(mode, sessions, dest, export_model):
    # 验收按整次摆放计，保证两轮都达标；缺测和模型域外值保留在分母。
    fit = [s for role, _, s in sessions if role == 'fit']
    verify = [s for role, _, s in sessions if role == 'verify']
    counts = Counter()
    for s in fit:
        counts[s['cm']] += s['sampling_summary']['samples']
    if any(counts[cm] < FIT_MIN_SAMPLES for cm in (-6, 0, 6)):
        return dict(mode=mode, status='insufficient_fit_samples', fit_samples=counts,
                    independent_mae_cm=None, independent_max_abs_cm=None)
    repeats = Counter(s['cm'] for s in verify)
    # 同刻度的独立重摆零点作为复测，单独统计，不进入异刻度验证器。
    candidate = build_model(fit[0]['binding'], fit, verify if len(repeats) >= 2 else [], method='linear')
    reports, zero_checks, errors = [], [], []
    seen = Counter()
    for role, run, s in sessions:
        if role == 'fit':
            continue
        values = [map_position(candidate['model'], r['rail_s_px'])
                  if r['pixel_valid'] and r['reference_valid'] and not r['predicted'] else None
                  for r in s['rows']]
        report = dict(run=run, cm=s['cm'], **metrics(s['cm'], values))
        measured = [v for v in values if v is not None]
        report['bias_cm'] = float(np.mean(measured) - s['cm']) if measured else None
        if role == 'repeat_zero':
            zero_checks.append(report)
            continue
        seen[s['cm']] += 1
        report['round'] = seen[s['cm']]
        reports.append(report)
        errors.extend(abs(v-s['cm']) for v in values if v is not None)
    round_summary = []
    for number in sorted(set(p['round'] for p in reports)):
        members = [p for p in reports if p['round'] == number]
        n = sum(p['valid'] for p in members)
        total = sum(p['frames'] for p in members)
        round_summary.append(dict(round=number, frames=total, valid=n, coverage=n/total,
                                  mae_cm=sum(p['mae_cm']*p['valid'] for p in members if p['valid'])/n if n else None,
                                  max_abs_cm=max((p['max_abs_cm'] for p in members if p['valid']), default=None),
                                  complete={p['cm'] for p in members} == {-5,-3,3,5},
                                  all_observed_passed=all(p['passed'] for p in members)))
    round_passed = (candidate['validated'] and all(repeats[cm] >= 1 for cm in (-5,-3,3,5))
                    and all(r['passed'] for r in reports + zero_checks))
    accepted = round_passed and all(repeats[cm] >= 2 for cm in (-5,-3,3,5))
    filename = None
    if export_model:
        # 双轮失败时只序列化审计信息，不能由运行时加载为厘米模型。
        if accepted:
            filename = mode+'_model.json'
            document = candidate
        else:
            filename = mode+'_audit.json'
            document = dict(schema='offline-static-cm-audit-v1', deployable=False,
                            binding_id=candidate['rail_binding_id'], replay_binding=candidate['binding'],
                            coefficients=candidate['model'], fit_points=candidate['points'],
                            verification=reports, zero_checks=zero_checks)
        json_out(dest/filename, document)
        loader = RailCalibration(str(dest/filename), fit[0]['binding'])
        if loader.ready != bool(accepted) or loader.validated != bool(accepted):
            raise ValueError(f'交付文件的加载状态异常: {loader.status}')
    return dict(mode=mode, status='evaluated_offline' if reports else 'fit_only',
                fit_samples=counts, fit_points=candidate['points'], zero_checks=zero_checks,
                rounds=round_summary,
                model_file=filename, loader_validated=candidate['validated'],
                measured_round_passed=bool(round_passed), heldout_repetitions=repeats,
                candidate_linear=candidate['model'], placements=reports,
                independent_mae_cm=float(np.mean(errors)) if errors else None,
                independent_max_abs_cm=max(errors, default=None),
                passed=bool(accepted))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dest', type=Path, help='7/motion_calibration 下的新目录')
    parser.add_argument('--videos', type=Path, default=HOST/'board_stage1_20261121')
    parser.add_argument('--index', type=Path, default=STAGE/'capture_index.csv')
    parser.add_argument('--placements', type=Path, default=STAGE/'placements.csv')
    parser.add_argument('--config', type=Path, help='现场默认冻结配置；历史对照默认 moving_check 配置')
    parser.add_argument('--points-config', type=Path, default=STAGE/'config_board_points.json')
    parser.add_argument('--motion', nargs='*', type=Path,
                        default=[HOST/'moving_check'/k/'raw.mjpg' for k in ('baseline', 'live_pixel')])
    parser.add_argument('--field', nargs='*', metavar='RUN', help='当前机位全套实物摆放与错放记录；可指定已到片子集')
    parser.add_argument('--export-model', action='store_true', help='输出离板审计；仅双轮逐段通过时导出正式模型')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    if args.config is None:
        args.config = (JET/'config_motion_topview_20261121.json' if args.field is not None
                       else HOST/'moving_check/config_moving_pixel.json')
    cfg = json.loads(args.config.read_text())
    ref = cfg['rail']['upper_reference']
    ref['image'] = str((args.config.resolve().parent / ref['image']).resolve())
    cfg['rail_coordinates']['calibration_file'] = None
    geo = json.loads(args.points_config.read_text())['camera_geometry']
    # 只借用已验证内参；相机或焦点标识不一致时 points 不可用。
    if any(geo[k] != cfg['camera_geometry'][k] for k in ('camera_id', 'focus_id')):
        raise ValueError('points 内参与当前相机或焦点不一致')
    clips = []
    if args.field is not None:
        # 仅使用现场明确的独立摆放名和整段真值。不会把原片拆成第二次重摆。
        labels = {'fit_m6': ('fit', -6), 'fit_zero': ('fit', 0), 'fit_p6': ('fit', 6),
                  'verify_m5': ('verify', -5), 'verify_p3': ('reported_position_conflict', None),
                  'verify_p3_retry': ('verify', 3),
                  'verify_m3': ('verify', -3), 'verify_p5': ('verify', 5),
                  'verify_zero': ('repeat_zero', 0), 'verify_m5_02': ('verify', -5),
                  'verify_p3_02': ('reported_position_conflict', None),
                  'verify_p3_02_retry': ('verify', 3), 'verify_m3_02': ('verify', -3),
                  'verify_p5_02': ('verify', 5)}
        for run in args.field or labels:
            role, cm = labels[run]
            video = HOST/'motion_20261121/runs'/run/'raw.mjpg'
            clips.append(dict(run=run, role=role, cm=cm, count=150, video=video, field=True,
                              nominal_30hz=(run == 'fit_m6'), sha256=sha256(video)))
            if role == 'reported_position_conflict':
                seen_mark = 18 if run == 'verify_p3' else 11
                clips[-1].update(reported_cm=3, note=f'标称纸尺15，现场核对首中末约{seen_mark}，排除')
    else:
        index = {r['run']: r for r in read_csv(args.index)}
        for p in read_csv(args.placements):
            run = p['run']
            row = index[run]
            if p['role'] not in ('fit', 'verify', 'repeat_zero'):
                raise ValueError(f'{run}: 未知摆放用途')
            if (row['role'] != p['role'] or float(row['known_cm']) != float(p['cm'])
                    or (p['role'] == 'repeat_zero' and float(p['cm']) != 0)):
                raise ValueError(f'{run}: 用途或真值不一致')
            clips.append(dict(run=run, role=p['role'], cm=float(p['cm']), count=int(row['frames']),
                              video=args.videos/run/row['file'], sha256=row['sha256'],
                              nominal_30hz=(run == 'fit_m6' and p['role'] == 'fit' and float(p['cm']) == -6)))
        for video in args.motion:
            clips.append(dict(run=video.parent.name, role='unlabelled', cm=None, video=video,
                              count=len(read_csv(video.parent/'frames.csv')), sha256=sha256(video)))
    for key in ('run', 'sha256'):
        if len(set(c[key] for c in clips)) != len(clips):
            raise ValueError(f'重复的摆放或原片: {key}')
    dest = args.dest.resolve()
    if (HOST/'motion_calibration').resolve() not in dest.parents:
        raise ValueError('输出须为 7/motion_calibration 的子目录')
    for c in clips:
        # 现场纠正的错放片即使被改名，也绝不进入采样或独立验收。
        if (c['sha256'] in {'0e5d3d450a9c55d7474d1d911c4ec8a1958f2e4e9892ea1b4d470b372d44a0c1',
                            'b24c2332eb02876c72b24227c34d491ee3d35a9b29d366017b0e8eaa4c28a078'}
                and c['role'] in ('fit', 'verify', 'repeat_zero')):
            raise ValueError('现场确认的 +3 错放原片仅可作诊断')
        if (Path(c['run']).name != c['run'] or c['run'] in ('', '.', '..')
                or c['count'] <= 0 or sha256(c['video']) != c['sha256']):
            raise ValueError(f"清单或原片校验失败: {c['run']}")
    dest.mkdir(parents=True, exist_ok=False)
    summaries, offsets = [], []
    sessions = {'raw': [], 'points': []}
    inputs = [args.config, args.points_config, Path(ref['image']), JET/geo['intrinsics_file']]
    if args.field is None:
        inputs += [args.index, args.placements]
    if (JET/'motion_observe.py').is_file():
        inputs.append(JET/'motion_observe.py')
    inputs += [c['video'].parent/'frames.csv' for c in clips if (c['video'].parent/'frames.csv').exists()]
    input_hashes = {str(p.resolve()): sha256(p) for p in inputs}
    for clip in clips:
        rows, pair, peaks = replay(clip, cfg, geo, dest)
        summaries.extend(rows)
        for mode, session in pair.items():
            sessions[mode].append((clip['role'], clip['run'], session))
        for i, peak in enumerate(peaks):
            offsets.append(dict(run=clip['run'], frame=clip['count']//2, patch=i+1, **peak))
        print(clip['run'], [(r['mode'], r['reference'], r['samples']) for r in rows], flush=True)
    for mode in sessions:
        if len({r['replay_binding'] for r in summaries if r['mode'] == mode}) != 1:
            raise ValueError('本次回放期间配置或算法发生变化，请重新运行')
    if any(sha256(path) != digest for path, digest in input_hashes.items()):
        raise ValueError('配置、参考、内参或帧时钟在回放期间发生变化')
    json_out(dest/'inputs.json', dict(sources=str(JET), opencv=cv2.__version__,
                                     numpy=np.__version__, files=input_hashes))
    write_rows(dest/'placements.csv', summaries)
    write_rows(dest/'middle_peaks.csv', offsets)
    results = [evaluate(mode, rows, dest, args.export_model) for mode, rows in sessions.items()]
    json_out(dest/'evaluation.json', results)
    print([(r['mode'], r['status']) for r in results])


if __name__ == '__main__':
    main()
