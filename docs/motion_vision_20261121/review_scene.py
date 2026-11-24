"""按固定时间抽帧，将 Nano 当时输出的圆心和纸尺纹理并排核对；抽样独立于检测状态。"""
import csv
import sys
from pathlib import Path
import cv2
import numpy as np

directory, target = map(Path,sys.argv[1:])
with (directory/'measurements.csv').open() as f:
    measurements = {int(r['frame']):r for r in csv.DictReader(f)}
with (directory/'frames.csv').open() as f:
    captures = list(csv.DictReader(f))
# 每两秒取一帧原片，无效、跳帧与有效结果同样入图。
times = np.array([float(r['read_s']) for r in captures])
choose = set(np.searchsorted(times, times[0]+np.arange(1,30,2)))
cap = cv2.VideoCapture(str(directory/'raw.mjpg'))
tiles = []
for i in range(len(captures)):
    ok, image = cap.read()
    if not ok:
        raise ValueError('raw frame count mismatch')
    if i not in choose:
        continue
    row = measurements.get(i,{})
    status = 'skipped'
    if row:
        status = row['coordinate_status']
        if status == 'pixel_invalid':
            status = row['tracker_reason'] or row['detector_reason']
    if row.get('x_px'):
        x,y = round(float(row['x_px'])),round(float(row['y_px']))
        cv2.drawMarker(image,(x,y),(0,255,0),cv2.MARKER_CROSS,12,1)
        status = 'measured {:.1f}px'.format(float(row['x_px']))
    tile = cv2.resize(image[465:715],(550,108))
    cv2.putText(tile,'#{:03d}  {}'.format(i,status),(6,18),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,245,255),1)
    tiles.append(tile)
cap.release()
if len(tiles)%2:
    tiles.append(np.zeros_like(tiles[0]))
sheet = np.vstack([np.hstack(tiles[i:i+2]) for i in range(0,len(tiles),2)])
if not cv2.imwrite(str(target),sheet):
    raise IOError('review image write failed')
