# -*- coding: utf-8 -*-
"""Calibrate/validate only an explicitly confirmed known-position video interval.
Frame numbers are zero-based, inclusive; no automatic selection of 'stable' balls.
"""
import argparse
import hashlib
import json
import os
import numpy as np
import cv2
from camera import VideoFile
from rail_detector import RailDetector
from ball_detector import BallDetector
from ball_tracker import BallTracker
from calibration import Calibration
from measurement_contract import calibrated_method

BASE = os.path.dirname(os.path.abspath(__file__))


def resolve(path):
    return path if os.path.isabs(path) else os.path.join(BASE,path)


def error_report(values, truth, selected_frames):
    if not values or not np.isfinite(truth) or selected_frames < len(values):
        raise ValueError('没有有效验证样本或无效真实坐标')
    a = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(a)):
        raise ValueError('验证坐标包含非有限值')
    errors = a-truth
    maximum = float(np.max(np.abs(errors)))
    coverage = len(a)/float(selected_frames)
    return {'samples':len(a), 'selected_frames':selected_frames,
            'valid_fraction':coverage, 'truth_cm':truth,
            'mean_error_cm':float(np.mean(errors)), 'mae_cm':float(np.mean(np.abs(errors))),
            'max_abs_error_cm':maximum, 'jitter_std_cm':float(np.std(a)),
            'jitter_peak_to_peak_cm':float(np.ptp(a)),
            'passed':bool(len(a)>=20 and coverage>=.95 and maximum<=.3)}


def collect_window(cfg, path, start, end):
    if start < 0 or end < start:
        raise ValueError('帧区间必须为 0 <= start <= end（两端均包含）')
    source = VideoFile(path)
    try:
        w,h = source.width,source.height
        rail_det=RailDetector(cfg['rail'],w,h)
        ball_det=BallDetector(cfg['ball'],w,h)
        tracker=BallTracker(cfg['tracking'],w,h)
        samples=[]
        for i in range(end+1):
            ok,frame=source.read()
            if not ok:
                raise ValueError('视频结束早于指定 end-frame，未写入标定')
            gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            # Every frame during calibration; do not reuse a stale pose.
            rail=rail_det.detect(gray,force=True)
            out=tracker.update(ball_det.detect(frame,gray,rail), i/max(source.fps_set,1))
            if (i >= start and out['valid'] and not out['predicted'] and
                    rail.valid and calibrated_method(out.get('method'))):
                samples.append({'frame':i,'x_px':out['x_px'],
                                'rail_slope':rail.raw_slope,'rail_intercept':rail.raw_intercept})
    finally:
        source.release()
    return samples,w,h


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--config',default=os.path.join(BASE,'config.json'))
    ap.add_argument('--calibration',default=None)
    sub=ap.add_subparsers(dest='cmd')
    for name in ('add','verify'):
        p=sub.add_parser(name)
        p.add_argument('--cm',type=float,required=True)
        p.add_argument('--video',required=True)
        p.add_argument('--start-frame',type=int,required=True)
        p.add_argument('--end-frame',type=int,required=True)
        p.add_argument('--known-position',action='store_true',help='确认整个所选区间钢珠实际位于指定刻度且定位工具不干扰检测')
        p.add_argument('--max-spread',type=float,default=5.,help='add: 全部有效样本像素峰峰值上限')
        p.add_argument('--report',default=None,help='verify: 新建 JSON 报告文件（不覆盖）')
    sub.add_parser('fit')
    sub.add_parser('show')
    args=ap.parse_args()
    if args.cmd is None:
        ap.print_help();return
    with open(resolve(args.config)) as f:
        cfg=json.load(f)
    c=Calibration(resolve(args.calibration or cfg['calibration']['file']))
    if args.cmd=='show':
        print(json.dumps({'ready':c.ready,'points':c.points,'model':c.model,'stats':c.stats},indent=2,ensure_ascii=False));return
    if args.cmd=='fit':
        ok,result=c.fit()
        if not ok: ap.error(result)
        print(json.dumps(result,indent=2));return
    if not args.known_position:
        ap.error('必须检查原视频并用 --known-position 确认所选画面真实位置；不能使用滚动后的未知位置')
    if not np.isfinite(args.cm) or not np.isfinite(args.max_spread) or args.max_spread < 0:
        ap.error('无效坐标或抖动阈值')
    path=resolve(args.video)
    samples,w,h=collect_window(cfg,path,args.start_frame,args.end_frame)
    if not samples: ap.error('所选区间没有可靠检测，未写入')
    digest=hashlib.sha256()
    with open(path,'rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''): digest.update(chunk)
    evidence={'video':os.path.abspath(path),'sha256':digest.hexdigest(),
              'start_frame':args.start_frame,'end_frame':args.end_frame,
              'known_position_confirmed':True,'samples':samples,
              'rail_slope':float(np.median([s['rail_slope'] for s in samples])),
              'rail_intercept':float(np.median([s['rail_intercept'] for s in samples]))}
    selected=args.end_frame-args.start_frame+1
    if args.cmd=='add':
        xs=[s['x_px'] for s in samples]
        if np.ptp(xs)>args.max_spread: ap.error('所选区间位置变化过大，未写入')
        # Refuse inconsistent rail poses even within the selected interval.
        for s in samples:
            if any(abs((s['rail_slope']-evidence['rail_slope'])*x+s['rail_intercept']-evidence['rail_intercept'])>3 for x in (0,w-1)):
                ap.error('所选区间轨道姿态变化，未写入')
        c.add_point(args.cm,float(np.median(xs)),w,h,evidence=evidence)
        print('已记录 %d/%d 帧；请完成 -5/0/+5 后显式运行 fit。' % (len(samples),selected))
        return
    if not c.ready: ap.error('标定未就绪')
    # Verification must use a physical position not used for fitting.
    if any(abs(p['x_cm']-args.cm)<1e-6 for p in c.points): ap.error('请选择非拟合刻度进行独立验证，如 -4/-2/+2/+4 cm')
    values=[]
    from rail_detector import RailLine
    for s in samples:
        rail=RailLine(s['rail_slope'],s['rail_intercept'],True)
        x=c.px_to_cm(s['x_px'],w,h)
        if x is not None and c.pose_ok(rail): values.append(x)
    report=error_report(values,args.cm,selected)
    report['evidence']=evidence
    with open(c.path,'rb') as f:
        report['calibration_sha256']=hashlib.sha256(f.read()).hexdigest()
    print(json.dumps(report,indent=2,ensure_ascii=False))
    if args.report:
        with open(resolve(args.report),'x') as f: json.dump(report,f,indent=2,ensure_ascii=False,allow_nan=False)


if __name__=='__main__':
    main()
