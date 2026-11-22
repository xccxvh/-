"""同一帧核对球心、纸尺 12 cm 印线与像平面姿态，按时间窗统计观测幅度。"""
import argparse
from pathlib import Path

import cv2
import numpy as np

from refit import old
from motion_observe import packets


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('replay', type=Path)
    parser.add_argument('output',type=Path)
    args = parser.parse_args()
    args.output.mkdir()
    models = old.json.loads((args.replay/'evaluation.json').read_text())
    model = next(m['candidate_linear'] for m in models if m['mode']=='points')
    rows = old.read_csv(args.replay/'fixed_O_tilt.csv')
    values = [old.map_position(model,float(r['rail_s_px'])) if r['rail_s_px'] else None for r in rows]
    valid = np.array([v for v in values if v is not None])
    lo = min((v,i) for i,v in enumerate(values) if v is not None)[1]
    hi = max((v,i) for i,v in enumerate(values) if v is not None)[1]
    windows = []
    for i in range(0,len(rows),60):
        sample = [v for v in values[i:i+60] if v is not None]
        data = [r for r in rows[i:i+60] if r['rail_s_px']]
        rec = dict(first=i,last=min(i+59,len(rows)-1),valid=len(sample),
                   median_cm=np.median(sample) if sample else None)
        for name in ('rail_angle_deg','reference_rotation_deg','origin_x_px','origin_y_px'):
            rec[name] = np.median([float(r[name]) for r in data]) if data else None
        windows.append(rec)
    old.write_rows(args.output/'windows.csv',windows)
    medians = [w['median_cm'] for w in windows if w['valid']]
    old.json_out(args.output/'excursion.json',dict(frames=len(rows),valid=len(valid),
                 signed_mean=float(valid.mean()),std=float(valid.std()),peak_to_peak=float(np.ptp(valid)),
                 percentiles=dict(zip(('min','p05','median','p95','max'),np.percentile(valid,[0,5,50,95,100]))),
                 min_frame=lo,max_frame=hi,window_median_span=float(np.ptp(medians))))
    panels, table = [], []
    for run,indices in [('fit_zero',{75}),('verify_zero',{75}),('fixed_O_tilt',{75,lo,hi})]:
        lookup = old.read_csv(args.replay/(run+'.csv'))
        video = old.HOST/'motion_20261121/runs'/run/'raw.mjpg'
        stream = packets(['ffmpeg','-nostdin','-loglevel','error','-f','mjpeg','-i',str(video),
                          '-c:v','copy','-f','mpjpeg','pipe:1'])
        for i,jpeg in enumerate(stream):
            if i not in indices:
                continue
            frame=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
            gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            r=lookup[i]
            # 人工在相同尺度裁剪中确定 12 cm 长刻线；窄横窗避开相邻毫米刻线，
            # 取六行暗像素所在列的中位数。二维横距与三维球心真值分别记录。
            x0=round(float(r['origin_x_px']))
            tick=float(np.median(gray[654:660,x0-4:x0+5].argmin(axis=1)+x0-4))
            x,y=[float(r[k]) for k in ('measurement_x_px','measurement_y_px')]
            table.append(dict(run=run,frame=i,x_px=x,y_px=y,tick_x_px=tick,dx_px=x-tick,
                              audit_cm=old.map_position(model,float(r['rail_s_px']))))
            crop=frame[575:682,550:725].copy()
            cv2.drawMarker(crop,(round(x)-550,round(y)-575),(0,255,255),cv2.MARKER_CROSS,8,1)
            cv2.line(crop,(round(tick)-550,654-575),(round(tick)-550,659-575),(255,255,0),1)
            panel=cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST)
            panel=cv2.copyMakeBorder(panel,30,0,0,0,cv2.BORDER_CONSTANT)
            cv2.putText(panel,f'{run}  f{i}',(6,20),0,.55,(255,255,255),1,cv2.LINE_AA)
            panels.append(panel)
    old.write_rows(args.output/'image_checks.csv',table)
    assert cv2.imwrite(str(args.output/'zero_tick.jpg'),np.hstack(panels))


if __name__ == '__main__':
    main()
