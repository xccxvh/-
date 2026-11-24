"""生成等距与困难帧的无标记复读图、全帧球心缩略图和带轨道上下文的代表帧。"""
import csv
import sys
from pathlib import Path

import cv2 as cv
import numpy as np
from label_raw import HERE, RUNS, jpeg_frames


def write(path, image):
    if not cv.imwrite(str(path), image, [cv.IMWRITE_JPEG_QUALITY, 90]):
        raise IOError(path)


def choose(rows):
    # 固定网格与图像本身的最弱模板分数、最近边界、最快位移组成复核样本。
    fixed = [11, 123, 235, 347, 459, 571, 683, 795]
    x = np.array([float(r["center_x_px"]) for r in rows])
    add = [np.argmin([float(r["template_ncc"]) for r in rows]),
           np.argmin([abs(float(r["signed_margin_px"])) for r in rows]),
           np.argmax(abs(np.diff(x))) + 1, 894]
    return sorted(set(fixed + list(map(int, add))))


def build(root, out, run):
    with (out/(run+".csv")).open() as stream:
        rows = list(csv.DictReader(stream))
    selected = choose(rows)
    # 额外记录可复查的端挡、进出界和高速样例；包含停车段曾发现误锁的手部入镜帧。
    extra = {RUNS[0]: [0, 30, 117, 158, 170, 195, 196, 203],
             RUNS[1]: [0, 19, 30, 35, 37, 60, 460, 894],
             RUNS[2]: [0, 30, 60, 170, 280, 460, 640, 894]}[run]
    grid = np.zeros((28*42, 32*42, 3), np.uint8)
    panels, examples, records = [], [], []
    for i, jpg in enumerate(jpeg_frames(root/run/"raw.mjpg")):
        row = rows[i]
        x, y = [round(float(row[k])) for k in ("center_x_px", "center_y_px")]
        im = cv.imdecode(np.frombuffer(jpg, np.uint8), cv.IMREAD_COLOR)
        crop = im[y-32:y+32, x-32:x+32]
        cell = cv.resize(crop, (42, 42), interpolation=cv.INTER_AREA)
        # 四个边缘短线指向预测中心，保留球体内部像素便于直接目视。
        for a,b in [((21,0),(21,6)),((21,35),(21,41)),((0,21),(6,21)),((35,21),(41,21))]:
            cv.line(cell,a,b,(20,230,70),1)
        gy, gx = divmod(i, 32)
        grid[gy*42:(gy+1)*42,gx*42:(gx+1)*42] = cell
        if i in selected:
            # 仅用粗定位选择 112 像素窗口，中心和 ROI 不入图。复读者按原像素坐标格估中心。
            ox = max(0, (x//32)*32-40)
            oy = (y//32)*32-40
            tile = im[oy:oy+112, ox:ox+112].copy()
            for j in range(10, 111, 10):
                col = (100, 255, 70) if j%50 else (0, 255, 255)
                cv.line(tile, (j, 0), (j, 7), col)
                cv.line(tile, (0, j), (7, j), col)
            tile = cv.copyMakeBorder(tile,0,16,0,0,cv.BORDER_CONSTANT)
            cv.putText(tile, f"#{i}", (4,124),0,.4,(240,240,240),1)
            panels.append(tile)
            records.append(dict(run=run, frame=i, crop_x=ox, crop_y=oy))
        if i in extra:
            left, right = [round(float(row[k])) for k in ("roi_left_px","roi_right_px")]
            tile = im[450:680].copy()
            for bound in (left, right):
                cv.line(tile,(bound,0),(bound,229),(255,200,10),2)
            cv.drawMarker(tile,(x,y-450),(0,255,0),cv.MARKER_CROSS,12,1)
            tile = cv.resize(tile,(640,115),interpolation=cv.INTER_AREA)
            cv.putText(tile,f"#{i}  {row['label']}  ({x}, {y})",(8,17),0,.48,(0,255,255),1,cv.LINE_AA)
            examples.append(tile)
    while len(panels)%4:
        panels.append(np.zeros_like(panels[0]))
    write(out/(run+"_reread.jpg"), np.vstack([np.hstack(panels[i:i+4]) for i in range(0,len(panels),4)]))
    write(out/(run+"_all.jpg"), grid)
    write(out/(run+"_examples.jpg"), np.vstack([np.hstack(examples[i:i+2]) for i in range(0,len(examples),2)]))
    return records


if __name__ == "__main__":
    root = Path(sys.argv[1])
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else HERE
    records = [r for run in RUNS for r in build(root, out, run)]
    with (out/"reread_windows.csv").open("w") as stream:
        writer = csv.DictWriter(stream, fieldnames=records[0])
        writer.writeheader()
        writer.writerows(records)
