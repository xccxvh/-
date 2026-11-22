"""诊断负五厘米重摆的逐列上缘支持，以及同帧圆候选的位置。"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from refit import old
from motion_observe import packets


def column_evidence(gray, points, radius):
    # 仅统计已配准尺段；与运行代码同样保留正梯度、阈值和搜索带宽。
    cols = np.arange(int(np.ceil(points[0, 0])), int(np.floor(points[-1, 0]))+1)
    expected = np.interp(cols, points[:, 0], points[:, 1])
    rows = np.floor(expected).astype(int) + np.arange(-radius, radius+2)[:, None]
    y0, y1 = rows.min()-2, rows.max()+3
    grad = cv2.Sobel(gray[y0:y1], cv2.CV_32F, 0, 1, ksize=3)
    values = np.where(abs(rows-expected)<=radius, grad[rows-y0,cols], -np.inf)
    strength = values.max(axis=0)
    good = strength > 35
    return cols, good, rows[values.argmax(axis=0), np.arange(len(cols))]-expected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--run', default='verify_m5_02')
    args = parser.parse_args()
    cfg = json.loads((old.JET/'config_motion_left_print_20261121.json').read_text())
    folder = old.HOST/'motion_20261121/runs'/args.run
    ref = cfg['rail']['upper_reference']
    ref['image'] = str(old.JET/ref['image'])
    detector = old.RailDetector(cfg['rail'],1280,720)
    balls = old.BallDetector(dict(cfg['ball'],ball_search=cfg['ball_search']),1280,720)
    tracker = old.BallTracker(cfg['tracking'],1280,720)
    cv2.setNumThreads(1)
    clock, _ = old.clip_clock(folder/'raw.mjpg',150)
    records = []
    stream = packets(['ffmpeg','-nostdin','-loglevel','error','-f','mjpeg','-i',
                      str(folder/'raw.mjpg'),'-c:v','copy','-f','mpjpeg','pipe:1'])
    for i,jpeg in enumerate(stream):
        frame = cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        rail = detector.detect(gray,force=True)
        candidates = balls.detect(frame,gray,rail)
        track = tracker.update(candidates,clock[i])
        rec = dict(frame=i,rail_reason=rail.reason,tracker_reason=track['reason'],
                   track_valid=track['valid'],candidate_xy=json.dumps(
                       [[round(c.x,2),round(c.extra['measurement_y_px'],2)] for c in candidates]))
        edge = detector.upper_reference
        transform = edge.matcher.transform
        if transform is not None:
            inv,t = transform
            points = edge.profile @ np.linalg.inv(inv).T + t
            # 原始各颜色通道单独取证，记录整段与分区。颜色诊断不会生成轨道或球坐标。
            for channel,img in [('gray',gray),('blue',frame[:,:,0]),('green',frame[:,:,1]),('red',frame[:,:,2])]:
                cols,good,offset = column_evidence(img,points,5)
                rec[channel+'_columns'] = int(good.sum())
                rec[channel+'_sectors'] = json.dumps([round(float(np.mean(a)),3) for a in np.array_split(good,6)])
                rec[channel+'_median_offset_px'] = round(float(np.median(offset[good])),3) if good.any() else None
            rec['domain'] = len(cols)
            if rail.reason in ('confirmed_upper_edge','upper_edge_geometry'):
                assert (rec['gray_columns']>=.55*len(cols)) == rail.valid
        records.append(rec)
    assert len(records)==len(clock)
    old.write_rows(args.output, records)


if __name__ == '__main__':
    main()
