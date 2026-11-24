"""离线候选实验。复用正式参考与跟踪；只输出当前帧的原图像素。"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

from motion_observe import (BallDetector, BallTracker, CameraGeometry, RailCoordinates,
                            RailDetector, cv2, decoded, latest_decoded, live, replay)


def merge(contours, motion):
    """同一球体的运动响应与轮廓不重复投票，轮廓提供中心。独立物体保留歧义。"""
    proposals = list(contours)
    for candidate in motion:
        if not any(math.hypot(c.x-candidate.x,
                              c.extra['measurement_y_px']-candidate.extra['measurement_y_px'])
                   <= .25*min(c.width, candidate.width) for c in contours):
            proposals.append(candidate)
    return proposals


def associate(proposals, tracker, timestamp):
    """E 只在最近有实测且多个实物候选竞争时使用原跟踪器的速度门和歧义检查。"""
    if len(proposals) < 2 or tracker.x is None or tracker.last_ts is None:
        return proposals
    dt = timestamp-tracker.last_ts
    if not 0 < dt <= min(.1, tracker.max_gap):
        return proposals
    eligible = [c for c in proposals if c.confidence >= tracker.conf_thr and tracker._identity_ok(c)]
    if not eligible:
        return proposals
    chosen = tracker._nearest(eligible, tracker.x+tracker.v*dt, tracker._gate(dt))
    return [chosen] if chosen is not None else proposals


def offline_frames(directory, schedule):
    for i, ts, jpeg in replay(directory):
        if schedule is not None and i not in schedule:
            yield i, ts, None, None, None
            continue
        if schedule is not None and abs(ts-schedule[i]) > 1e-6:
            raise ValueError('processed frame and capture timestamps differ')
        yield next(decoded(((i, ts, jpeg),)))


FIELDS = ('frame timestamp processed mode pixel_valid x_px y_px confidence candidates '
          'tracker_reason source ref_ms hough_ms extra_ms total_ms rail_valid '
          'decode_ms tracker_ms coords_ms hough_count contour_count motion_count '
          'contour_xy motion_xy detector_reason read_to_output_ms').split()


def centers(candidates):
    return '|'.join('{:.2f}/{:.2f}'.format(c.x, c.extra['measurement_y_px']) for c in candidates)


def run(config, directory, output, modes, all_frames=False, live_seconds=None):
    cfg = json.loads(config.read_text())
    ref = cfg['rail']['upper_reference']
    ref['image'] = str((config.parent / ref['image']).resolve())
    geo = CameraGeometry(cfg['camera_geometry'], 1280, 720, config.parent)
    rail_detector = RailDetector(cfg['rail'], 1280, 720)
    ball = BallDetector(dict(cfg['ball'], ball_search=cfg['ball_search']), 1280, 720)
    trackers = {m: BallTracker(cfg['tracking'], 1280, 720) for m in modes}
    # 正式厘米标定仅用于 A 的逐帧基线复现。实验候选始终与厘米换算隔离。
    coords = {}
    for m in modes:
        settings = cfg if m == 'A' else dict(cfg, rail_coordinates=dict(
            cfg['rail_coordinates'], calibration_file=None))
        coords[m] = RailCoordinates(settings, geo, config.parent)
    need_contour = bool(set(modes) & set('BDE'))
    need_motion = bool(set(modes) & set('CDE'))
    if need_contour:
        from fusion_contour import contour_candidates
    if need_motion:
        from fusion_motion import MotionCandidates
        motion_model = MotionCandidates()
    schedule = None
    if not all_frames and live_seconds is None:
        with (directory/'measurements.csv').open() as f:
            schedule = {int(r['frame']): float(r['timestamp']) for r in csv.DictReader(f)}
    cv2.setNumThreads(1)
    clock = time.perf_counter
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open('x') as f:
        writer = csv.DictWriter(f, FIELDS, lineterminator='\n')
        writer.writeheader()
        stream = (offline_frames(directory, schedule) if live_seconds is None else
                  latest_decoded(live(directory, live_seconds)))
        for i, ts, frame, gray, decode_s in stream:
            rows = [dict(frame=i, timestamp=ts, mode=m, processed=0, pixel_valid=0) for m in modes]
            if decode_s is None:
                writer.writerows(rows)
                continue
            if gray is None:
                for row in rows:
                    row.update(processed=1, tracker_reason='decode_failed')
                writer.writerows(rows)
                continue
            t0 = clock()
            rail = rail_detector.detect(gray, force=True)
            t1 = clock()
            original = ball.detect(frame, gray, rail, timestamp=ts)
            bounds = ball.last_roi
            t2 = clock()
            contours = []
            if need_contour and not original and bounds is not None and rail.valid:
                contours = contour_candidates(gray, rail, bounds)
            t3 = clock()
            movement = []
            if need_motion:
                movement = motion_model.update(gray, rail, bounds, ts, propose=not original)
            t4 = clock()
            for row in rows:
                m = row['mode']
                start = clock()
                if original or m == 'A':
                    proposals = original
                elif m in 'DE':
                    proposals = merge(contours, movement)
                    if m == 'E':
                        proposals = associate(proposals, trackers[m], ts)
                else:
                    proposals = contours if m == 'B' else movement
                # 实验补测不充当正式 Hough 的边界身份锚点。
                out = trackers[m].update(proposals, ts, bounds if original or not proposals else None)
                tc = clock()
                if m == 'A':
                    pos = coords[m].process(out, original, rail, i)
                    valid = pos['rail_s_px'] is not None
                else:
                    pos = coords[m].process({}, [], rail, i)
                    valid = bool(pos['reference_valid'] and out['valid'] and not out['predicted']
                                 and len(proposals) == 1 and proposals[0].x == out['x_px'])
                end = clock()
                extra = (t3-t2 if m in 'BDE' else 0.) + (t4-t3 if m in 'CDE' else 0.)
                row.update(processed=1, pixel_valid=int(valid), rail_valid=int(pos['reference_valid']),
                           tracker_reason=out['reason'], detector_reason=ball.last_reason,
                           candidates=len(proposals), hough_count=len(original),
                           contour_count=len(contours) if m in 'BDE' else None,
                           motion_count=len(movement) if m in 'CDE' else None,
                           contour_xy=centers(contours) if m in 'BDE' else None,
                           motion_xy=centers(movement) if m in 'CDE' else None,
                           read_to_output_ms=(time.monotonic()-ts)*1000 if live_seconds is not None else None,
                           confidence=out['confidence'] if valid else 0.,
                           ref_ms=(t1-t0)*1000, hough_ms=(t2-t1)*1000, extra_ms=extra*1000,
                           tracker_ms=(tc-start)*1000, coords_ms=(end-tc)*1000,
                           decode_ms=decode_s*1000, total_ms=(decode_s+t2-t0+extra+end-start)*1000)
                if valid:
                    c = proposals[0]
                    row.update(x_px=c.x, y_px=c.extra['measurement_y_px'],
                               source=c.extra.get('proposal_source', 'hough'))
            writer.writerows(rows)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('directory', type=Path, help='采集目录，含 raw.mjpg、frames.csv、measurements.csv')
    ap.add_argument('output', type=Path)
    ap.add_argument('--config', type=Path, default=Path(__file__).with_name('config_motion_cm_20261122.json'))
    ap.add_argument('--modes', default='ABCD')
    ap.add_argument('--all-frames', action='store_true', help='处理所有采集帧，独立于现场处理时序')
    ap.add_argument('--live-seconds', type=float, help='单一模式实测，directory 为新的完整录像目录')
    args = ap.parse_args()
    if not args.modes or set(args.modes)-set('ABCDE') or len(set(args.modes)) != len(args.modes):
        ap.error('--modes 只能包含不重复的 A、B、C、D、E')
    if args.live_seconds is not None:
        if len(args.modes) != 1 or args.all_frames or args.live_seconds <= 0:
            ap.error('实时采集需单一模式及正时长')
        args.directory.mkdir(parents=True, exist_ok=False)
    run(args.config.resolve(), args.directory, args.output, args.modes, args.all_frames, args.live_seconds)
