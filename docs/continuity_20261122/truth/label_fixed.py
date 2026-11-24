"""固定 O 段用可见球与挡具接触处定位。原帧 178 缺 JPEG 尾标，按 JFIF 包头保持时钟对齐。"""
import argparse
import csv
import json
import sys
from pathlib import Path

import cv2 as cv
import numpy as np
from label_raw import HERE, JETSON, decode, pose, ruler_features


def fixed(root, out, run, seed):
    cv.setNumThreads(1)
    sx, sy = seed
    out.mkdir(parents=True, exist_ok=True)
    cfg = json.loads((JETSON/"config_motion_topview_20261121.json").read_text())
    patches, bounds = ruler_features(cfg)
    with (root/run/"frames.csv").open() as stream:
        times = list(csv.DictReader(stream))
    marker = b"\xff\xd8\xff\xe0"
    head, *packets = (root/run/"raw.mjpg").read_bytes().split(marker)
    assert not head and len(packets) == len(times) and times
    tiles, rows = [], []
    for i, packet in enumerate(packets):
        # 缺失尾标以帧元数据单列记录；原图球与尺纹均可读，不借用邻帧像素。
        assert packet[2:7] == b"JFIF\x00" and int(times[i]["frame"]) == i
        partial = b"\xff\xd9" not in packet
        gray = decode(marker + packet + (b"\xff\xd9" if partial else b""))
        if i == 0:
            template = gray[sy-24:sy+25, sx-24:sx+25].copy()
            ruler = gray[sy+32:sy+69, sx-85:sx+86].copy()
        scores = cv.matchTemplate(gray[549:675,515:735], template, cv.TM_CCOEFF_NORMED)
        _, peak, _, (u,v) = cv.minMaxLoc(scores)
        x, y = u+515+24, v+549+24
        left,right,n,rms = pose(gray,patches,bounds)
        margin = min(x-left,right-x)
        assert peak>.80 and margin>200
        # 当帧匹配紧邻球的数字和刻度，用球与局部纸尺的相对位移检查是否稳定。
        local = gray[sy+17:sy+84, sx-100:sx+101]
        _, paper_ncc, _, (rx, ry) = cv.minMaxLoc(cv.matchTemplate(local,ruler,cv.TM_CCOEFF_NORMED))
        rows.append(dict(frame=i,read_s=times[i]["read_s"],center_x_px=x,center_y_px=y,
                         template_ncc=round(peak,3),roi_left_px=round(left,2),roi_right_px=round(right,2),
                         ruler_inliers=n,ruler_rms_px=round(rms,2),signed_margin_px=round(margin,2),
                         border_band_px=10,label="inside",source_missing_eoi=int(partial),
                         ball_minus_ruler_dx=x-sx-(rx-15),ball_minus_ruler_dy=y-sy-(ry-15),
                         local_paper_ncc=round(paper_ncc,3)))
        if i in (0,100,177,178,179,300,500,700,837,len(times)-1):
            tile=cv.cvtColor(gray[558:676,526:716],cv.COLOR_GRAY2BGR)
            cv.drawMarker(tile,(x-526,y-558),(0,255,0),cv.MARKER_CROSS,10,1)
            cv.putText(tile,str(i),(4,17),0,.45,(0,255,255),1)
            tiles.append(tile)
    with (out/(run+".csv")).open("w") as stream:
        writer=csv.DictWriter(stream,fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    assert cv.imwrite(str(out/(run+"_examples.jpg")),np.vstack([np.hstack(tiles[:5]),np.hstack(tiles[5:])]))


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs",type=Path)
    parser.add_argument("--out",type=Path,default=HERE)
    parser.add_argument("--scene",default="fixed_O_tilt")
    parser.add_argument("--seed",type=int,nargs=2,default=(626,619))
    args=parser.parse_args()
    fixed(args.runs,args.out,args.scene,args.seed)
