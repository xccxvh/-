"""回放五段正式原片，每帧检测一次，分送两个独立的轨道几何上下文。"""
import argparse
import csv
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
JETSON = HERE.parents[1] / 'Jetson'
sys.path.insert(0, str(JETSON))
from ball_detector import BallDetector
from ball_tracker import BallTracker
from calibrate_rail import collect, sha256
from camera_geometry import CameraGeometry
from rail_calibration import build_model, map_position
from rail_coordinates import RailCoordinates
from rail_detector import RailDetector


def save_csv(path, rows):
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def replay(video, config, points_cfg, old):
    # 每段从冷启动开始，时间来自录像帧号。输入只能是已验哈希的本地录像。
    raw = RailCoordinates(config, CameraGeometry(config['camera_geometry'], 1280, 720))
    points_geo = CameraGeometry(points_cfg, 1280, 720, JETSON)
    points = RailCoordinates(dict(config, rail_coordinates=dict(config['rail_coordinates'],
                             calibration_file=None)), points_geo)
    assert raw.calibration.validated and raw.binding == old['binding'] and raw.binding_id != points.binding_id
    rd = RailDetector(config['rail'], 1280, 720)
    bd = BallDetector(dict(config['ball'], ball_search=config['ball_search']), 1280, 720)
    tracker = BallTracker(config['tracking'], 1280, 720)
    rows, samples = [], []
    cap = cv2.VideoCapture(str(video))
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            i = len(rows)
            assert frame.shape[:2] == (720, 1280)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            rail = rd.detect(gray, force=True)
            candidates = bd.detect(frame, gray, rail)
            out = tracker.update(candidates, i/30.)
            a, b = [coords.process(out, candidates, rail, i) for coords in (raw, points)]
            xy = None
            if a['measurement_x_px'] is not None:
                xy = points_geo.correct_points([[a['measurement_x_px'], a['measurement_y_px']]])[0]
            rows.append(dict(clip=video.stem, frame=i, raw_x=a['measurement_x_px'], raw_y=a['measurement_y_px'],
                             corrected_x=None if xy is None else xy[0], corrected_y=None if xy is None else xy[1],
                             raw_ref_x=a['reference_x_px'], raw_ref_y=a['reference_y_px'],
                             points_ref_x=b['reference_x_px'], points_ref_y=b['reference_y_px'],
                             raw_s=a['rail_s_px'], points_s=b['rail_s_px'], raw_cm=a['rail_x_cm'],
                             raw_status=a['rail_coordinate_status'], points_pre_fit_status=b['rail_coordinate_status']))
            samples.append(dict(frame=i, **{k:b[k] for k in ('pixel_valid','reference_valid','cm_valid',
                                'calibration_valid','rail_binding_id','rail_s_px')}))
    finally:
        cap.release()
    return rows, samples, points


def run(folder, dest):
    cv2.setNumThreads(1)
    dest.mkdir(parents=True, exist_ok=True)
    def read(path):
        return json.loads(path.read_text())
    cfg = read(JETSON/'config_topview_circle_20261120.json')
    pg = read(JETSON/'config_geometry_topview_undistorted_20261120.json')['camera_geometry']
    pg.update(mode='points')
    old = read(JETSON/cfg['rail_coordinates']['calibration_file'])
    rows, fit, check = [], [], []
    for index, session in enumerate(old['fit_sessions'] + old['validation']['sessions']):
        video = folder / Path(session['source']['video_path']).name
        assert video.is_file() and sha256(video) == session['source']['video_sha256']
        run_rows, samples, axis = replay(video, cfg, pg, old)
        assert len(samples) == 150 and session['start_frame']==0 and session['end_frame']==149
        binding = dest/'points_binding.json'
        binding.write_text(json.dumps(axis.binding, indent=2)+'\n')
        src = dest/(video.stem+'_points.csv')
        save_csv(src, samples)
        args = SimpleNamespace(csv=src, video=video, cm=session['cm'], known_position=True, start_frame=0, end_frame=149)
        evidence = collect(args, axis.binding, binding)
        (fit if index < len(old['fit_sessions']) else check).append(evidence)
        for r in run_rows:
            r['known_cm'] = session['cm']
        rows.extend(run_rows)
    new = build_model(axis.binding, fit, check)
    (dest/'points_model.json').write_text(json.dumps(new, separators=(',',':'))+'\n')
    for r in rows:
        s = r['points_s']
        r['points_cm'] = map_position(new['model'], s) if new['validated'] and s is not None else None
    save_csv(dest/'same_frames.csv', rows)
    # 分母含所有启动及丢失帧；带符号误差用来区分帧间抖动和固定摆放偏移。
    metrics = []
    for clip in dict.fromkeys(r['clip'] for r in rows):
        group = [r for r in rows if r['clip']==clip]
        truth = group[0]['known_cm']
        paired = [r for r in group if r['raw_s'] is not None and r['points_s'] is not None]
        for key in ('raw','points'):
            cm = np.array([r[key+'_cm'] for r in group if r[key+'_cm'] is not None])
            ss = np.array([r[key+'_s'] for r in group if r[key+'_s'] is not None])
            model = (old if key=='raw' else new)['model']
            lo, hi = model['domain_s_px']
            metrics.append(dict(clip=clip,mode=key,truth_cm=truth,frames=len(group),s_count=len(ss),cm_count=len(cm),
                outside=int(np.sum((ss<lo)|(ss>hi))),s_median=np.median(ss),s_min=min(ss),s_max=max(ss),
                bias_cm=np.mean(cm-truth),mae_cm=np.mean(abs(cm-truth)),max_abs_cm=max(abs(cm-truth)),std_cm=np.std(cm),
                median_ball_shift_px=np.median([np.hypot(r['corrected_x']-r['raw_x'],r['corrected_y']-r['raw_y']) for r in paired]),
                median_ref_shift_px=np.median([np.hypot(r['points_ref_x']-r['raw_ref_x'],r['points_ref_y']-r['raw_ref_y']) for r in paired]),
                median_delta_s_px=np.median([r['points_s']-r['raw_s'] for r in paired])))
    save_csv(dest/'vision_summary.csv',metrics)
    # 原模型来自历史 Nano 拟合。另列同机重拟合 raw 对照，区分主机重拟合与几何变化。
    medians = [np.median([r['raw_s'] for r in rows if r['known_cm']==cm and r['raw_s'] is not None])
               for cm in (-5, 0, 5)]
    a, b = np.polyfit(medians, [-5, 0, 5], 1)
    held = [r for r in rows if abs(r['known_cm'])==3 and r['raw_cm'] is not None and r['points_cm'] is not None]
    controls = []
    for name, values in [('original_A',[r['raw_cm'] for r in held]),
                         ('same_host_raw_refit',[a*r['raw_s']+b for r in held]),
                         ('points_B',[r['points_cm'] for r in held])]:
        assert all(medians[0] <= r['raw_s'] <= medians[-1] for r in held)
        error = np.array(values)-[r['known_cm'] for r in held]
        controls.append(dict(model=name,paired=len(held),mae_cm=np.mean(abs(error)),max_abs_cm=max(abs(error))))
    save_csv(dest/'holdout.csv',controls)
    print('points validated:',new['validated'],new['model'])
    print('axis delta xy (px):',axis.axis_points-axis.points)
    print('replayed', len(rows), 'frames')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video_folder',type=Path)
    parser.add_argument('--out',type=Path,default=HERE/'vision')
    args=parser.parse_args()
    run(args.video_folder,args.out)
